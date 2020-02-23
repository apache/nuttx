/**************************************************************************************
 * drivers/lcd/st7565.c
 *
 * Definitions for the ST7565 128x64 Dot Matrix LCD Driver with C
 *
 *   Copyright (C) 2014 Pierre-noel Bouteville. All rights reserved.
 *   Author: Pierre-noel Boutevlle <pnb990@gmail.com>
 *
 * Based on drivers/lcd/st7567.c
 *   Copyright (C) 2013 Zilogic Systems. All rights reserved.
 *   Author: Manikandan <code@zilogic.com>
 *
 * Based on drivers/lcd/ug-9664hswag01.c
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Reference: "Product Specification, OEL Display Module, ST7567", Univision
 *            Technology Inc., SAS1-6020-B, January 3, 2008.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 **************************************************************************************/

/**************************************************************************************
 * Included Files
 **************************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/spi/spi.h>
#include <nuttx/lcd/lcd.h>
#include <nuttx/lcd/st7565.h>

#include "st7565.h"

/**************************************************************************************
 * Pre-processor Definitions
 **************************************************************************************/

/* Configuration **********************************************************************/

/* ST7565 Configuration Settings:
 *
 * CONFIG_ST7565_NINTERFACES - Specifies the number of physical
 *   ST7565 devices that will be supported.  NOTE:  At present, this
 *   must be undefined or defined to be 1.
 * CONFIG_LCD_ST7565DEBUG - Enable detailed ST7565 debst7565 output
 *   (CONFIG_DEBUG_FEATURES and CONFIG_VERBOSE must also be enabled).
 *
 * Required LCD driver settings:
 * CONFIG_LCD_ST7565 - Enable ST7565 support
 * CONFIG_LCD_MAXCONTRAST should be 255, but any value >0 and <=255 will be accepted.
 *
 */

/* Verify that all configuration requirements have been met */

/* CONFIG_ST7565_NINTERFACES determines the number of physical interfaces
 * that will be supported.
 */

#ifndef CONFIG_ST7565_NINTERFACES
#  define CONFIG_ST7565_NINTERFACES 1
#endif

#if CONFIG_ST7565_NINTERFACES != 1
#  warning "Only a single ST7565 interface is supported"
#  undef CONFIG_ST7565_NINTERFACES
#  define CONFIG_ST7565_NINTERFACES 1
#endif

/* Verbose debst7565 must also be enabled to use the extra OLED debst7565 */

#ifndef CONFIG_DEBUG_FEATURES
#  undef CONFIG_DEBUG_INFO
#  undef CONFIG_DEBUG_GRAPHICS
#endif

#ifndef CONFIG_DEBUG_INFO
#  undef CONFIG_LCD_ST7565DEBUG
#endif

/* Check contrast selection */

#ifndef CONFIG_LCD_MAXCONTRAST
#  define CONFIG_LCD_MAXCONTRAST 255
#endif

#if CONFIG_LCD_MAXCONTRAST <= 0 || CONFIG_LCD_MAXCONTRAST > 255
#  error "CONFIG_LCD_MAXCONTRAST exceeds supported maximum"
#endif

#if CONFIG_LCD_MAXCONTRAST < 255
#  warning "Optimal setting of CONFIG_LCD_MAXCONTRAST is 255"
#endif

/* Color Properties *******************************************************************/

/* The ST7565 display controller can handle a resolution of 128x64.
 */

/* Display Resolution */

#ifdef CONFIG_ST7565_XRES
#  define ST7565_XRES         CONFIG_ST7565_XRES
#else
#  define ST7565_XRES         128
#endif

#ifdef CONFIG_ST7565_YRES
#  define ST7565_YRES         CONFIG_ST7565_YRES
#else
#  define ST7565_YRES         64
#endif

/* Color depth and format */

#define ST7565_BPP          1
#define ST7565_COLORFMT     FB_FMT_Y1

/* Bytes per logical row andactual device row */

#define ST7565_XSTRIDE      (ST7565_XRES >> 3)  /* Pixels arrange "horizontally
                                                 * for user" */
#define ST7565_YSTRIDE      (ST7565_YRES >> 3)  /* But actual device
                                                 * arrangement is "vertical" */

/* The size of the shadow frame buffer */

#define ST7565_FBSIZE       (ST7565_XRES * ST7565_YSTRIDE)

/* Bit helpers */

#define LS_BIT          (1 << 0)
#define MS_BIT          (1 << 7)

/**************************************************************************************
 * Private Type Definition
 **************************************************************************************/

/* This structure describes the state of this driver */

struct st7565_dev_s
{
  /* Publicly visible device structure */

  struct lcd_dev_s dev;

  /* Private LCD-specific information follows */

  FAR struct st7565_lcd_s *lcd;
  uint8_t contrast;
  uint8_t power_level;

  /* TODO implement READ mode, possible in 8080bus interface */

  /* The ST7565 does not support reading from the display memory in SPI mode.
   * Since there is 1 BPP and access is byte-by-byte, it is necessary to keep
   * a shadow copy of the framebuffer memory. */

  uint8_t fb[ST7565_FBSIZE];
};

/**************************************************************************************
 * Private Function Prototypes
 **************************************************************************************/

/* Drivers helpers */

static inline void st7565_reset(FAR struct st7565_dev_s *priv, bool on);

static inline void st7565_select(FAR struct st7565_dev_s *priv);
static inline void st7565_deselect(FAR struct st7565_dev_s *priv);

static inline void st7565_cmddata(FAR struct st7565_dev_s *priv, bool cmd);
static inline int st7565_send_one_data(FAR struct st7565_dev_s *priv,
                                       uint8_t data);
static inline int st7565_send_data_buf(FAR struct st7565_dev_s *priv,
                                       FAR const uint8_t * buf, int size);
static inline int st7565_backlight(FAR struct st7565_dev_s *priv, int level);

/* LCD Data Transfer Methods */

static int st7565_putrun(fb_coord_t row, fb_coord_t col,
                         FAR const uint8_t * buffer, size_t npixels);
static int st7565_getrun(fb_coord_t row, fb_coord_t col, FAR uint8_t * buffer,
                         size_t npixels);

/* LCD Configuration */

static int st7565_getvideoinfo(FAR struct lcd_dev_s *dev,
                               FAR struct fb_videoinfo_s *vinfo);
static int st7565_getplaneinfo(FAR struct lcd_dev_s *dev, unsigned int planeno,
                               FAR struct lcd_planeinfo_s *pinfo);

/* LCD RGB Mapping */

#ifdef CONFIG_FB_CMAP
#  error "RGB color mapping not supported by this driver"
#endif

/* Cursor Controls */

#ifdef CONFIG_FB_HWCURSOR
#  error "Cursor control not supported by this driver"
#endif

/* LCD Specific Controls */

static int st7565_getpower(struct lcd_dev_s *dev);
static int st7565_setpower(struct lcd_dev_s *dev, int power);
static int st7565_getcontrast(struct lcd_dev_s *dev);
static int st7565_setcontrast(struct lcd_dev_s *dev, unsigned int contrast);

/* Initialization */

static inline void up_clear(FAR struct st7565_dev_s *priv);

/**************************************************************************************
 * Private Data
 **************************************************************************************/

/* This is working memory allocated by the LCD driver for each LCD device
 * and for each color plane.  This memory will hold one raster line of data.
 * The size of the allocated run buffer must therefore be at least
 * (bpp * xres / 8).  Actual alignment of the buffer must conform to the
 * bitwidth of the underlying pixel type.
 *
 * If there are multiple planes, they may share the same working buffer
 * because different planes will not be operate on concurrently.  However,
 * if there are multiple LCD devices, they must each have unique run buffers.
 */

static uint8_t g_runbuffer[ST7565_XSTRIDE + 1];

/* This structure describes the overall LCD video controller */

static const struct fb_videoinfo_s g_videoinfo =
{
  .fmt     = ST7565_COLORFMT,         /* Color format: RGB16-565: RRRR RGGG GGGB BBBB */
  .xres    = ST7565_XRES,             /* Horizontal resolution in pixel columns */
  .yres    = ST7565_YRES,             /* Vertical resolution in pixel rows */
  .nplanes = 1,                       /* Number of color planes supported */
};

/* This is the standard, NuttX Plane information object */

static const struct lcd_planeinfo_s g_planeinfo =
{
  .putrun  = st7565_putrun,           /* Put a run into LCD memory */
  .getrun  = st7565_getrun,           /* Get a run from LCD memory */
  .buffer  = (uint8_t *) g_runbuffer, /* Run scratch buffer */
  .bpp     = ST7565_BPP,              /* Bits-per-pixel */
};

/* This is the standard, NuttX LCD driver object */

static struct st7565_dev_s g_st7565dev =
{
  .dev =
  {
    /* LCD Configuration */

    .getvideoinfo = st7565_getvideoinfo,
    .getplaneinfo = st7565_getplaneinfo,

    /* LCD RGB Mapping -- Not supported */
    /* Cursor Controls -- Not supported */

    /* LCD Specific Controls */

    .getpower     = st7565_getpower,
    .setpower     = st7565_setpower,
    .getcontrast  = st7565_getcontrast,
    .setcontrast  = st7565_setcontrast,
  },
};

/**************************************************************************************
 * Private Functions
 **************************************************************************************/

/**************************************************************************************
 * Name: st7565_reset
 *
 * Description:
 *   Enable/Disable reset pin of LCD the device.
 *
 *   priv - A reference to the driver specific structure
 *
 **************************************************************************************/

static inline void st7565_reset(FAR struct st7565_dev_s *priv, bool on)
{
  if (priv->lcd->reset != NULL)
    {
      priv->lcd->reset(priv->lcd, on);
    }
}

/**************************************************************************************
 * Name: st7565_select
 *
 * Description:
 *   Select the device (as necessary) before performing any operations.
 *
 *   priv - A reference to the driver specific structure
 *
 **************************************************************************************/

static inline void st7565_select(FAR struct st7565_dev_s *priv)
{
  priv->lcd->select(priv->lcd);
}

/**************************************************************************************
 * Name:  st7565_deselect
 *
 * Description:
 *   Deselect the device (as necessary).
 *
 *   priv - A reference to the driver specific structure
 *
 **************************************************************************************/

static inline void st7565_deselect(FAR struct st7565_dev_s *priv)
{
  priv->lcd->deselect(priv->lcd);
}

/**************************************************************************************
 * Name:  st7565_cmddata
 *
 * Description:
 *   Select command (A0 = 0) or data (A0 = 1) mode.
 *
 *   priv - A reference to the driver specific structure.
 *   cmd  - If true command mode will be selected.
 *
 **************************************************************************************/

static inline void st7565_cmddata(FAR struct st7565_dev_s *priv, bool cmd)
{
  priv->lcd->cmddata(priv->lcd, cmd);
}

/**************************************************************************************
 * Name:  st7565_send_one_data
 *
 * Description:
 *   Send one data to the LCD driver (A0 = 1).
 *
 *   priv - A reference to the driver specific structure.
 *   data - Byte to send as data to LCD driver.
 *
 **************************************************************************************/

static inline int st7565_send_one_data(FAR struct st7565_dev_s *priv,
                                       uint8_t data)
{
  return priv->lcd->senddata(priv->lcd, &data, 1);
}

/**************************************************************************************
 * Name:  st7565_send_data_buf
 *
 * Description:
 *   Send a data buffer to the LCD driver (A0 = 1).
 *
 *   priv   - A reference to the driver specific structure.
 *   buf    - Buffer sent as data to LCD driver.
 *   size   - Size of buffer in bytes.
 *
 **************************************************************************************/

static inline int st7565_send_data_buf(FAR struct st7565_dev_s *priv,
                                       FAR const uint8_t * buf, int size)
{
  return priv->lcd->senddata(priv->lcd, buf, size);
}

/**************************************************************************************
 * Name:  st7565_backlight
 *
 * Description:
 *   Change backlight level of display.
 *
 *   priv  - A reference to the driver specific structure.
 *   level - Set backlight pwm from 0 CONFIG_LCD_MAXPOWER-1.
 *
 **************************************************************************************/

static inline int st7565_backlight(FAR struct st7565_dev_s *priv, int level)
{
  return priv->lcd->backlight(priv->lcd, level);
}

/**************************************************************************************
 * Name:  st7565_putrun
 *
 * Description:
 *   This method can be used to write a partial raster line to the LCD:
 *
 *   row     - Starting row to write to (range: 0 <= row < yres)
 *   col     - Starting column to write to (range: 0 <= col <= xres-npixels)
 *   buffer  - The buffer containing the run to be written to the LCD
 *   npixels - The number of pixels to write to the LCD
 *             (range: 0 < npixels <= xres-col)
 *
 **************************************************************************************/

static int st7565_putrun(fb_coord_t row, fb_coord_t col,
                         FAR const uint8_t * buffer, size_t npixels)
{
  /* Because of this line of code, we will only be able to support a single
   * ST7565 device.
   */

  FAR struct st7565_dev_s *priv = &g_st7565dev;
  FAR uint8_t *fbptr;
  FAR uint8_t *ptr;
  uint8_t fbmask;
  uint8_t page;
  uint8_t usrmask;
  uint8_t i;
  int pixlen;

  ginfo("row: %d col: %d npixels: %d\n", row, col, npixels);
  DEBUGASSERT(buffer);

  /* Clip the run to the display */

  pixlen = npixels;
  if ((unsigned int)col + (unsigned int)pixlen > (unsigned int)ST7565_XRES)
    {
      pixlen = (int)ST7565_XRES - (int)col;
    }

  /* Verify that some portion of the run remains on the display */

  if (pixlen <= 0 || row > ST7565_YRES)
    {
      return OK;
    }

  /* Get the page number.  The range of 64 lines is divided up into eight pages
   * of 8 lines each.
   */

  page = row >> 3;

  /* Update the shadow frame buffer memory. First determine the pixel position
   * in the frame buffer memory.  Pixels are organized like this:
   *
   *  --------+---+---+---+---+-...-+-----+
   *  Segment | 0 | 1 | 2 | 3 | ... | 131 |
   *  --------+---+---+---+---+-...-+-----+
   *  Bit 0   |   | X |   |   |     |     |
   *  Bit 1   |   | X |   |   |     |     |
   *  Bit 2   |   | X |   |   |     |     |
   *  Bit 3   |   | X |   |   |     |     |
   *  Bit 4   |   | X |   |   |     |     |
   *  Bit 5   |   | X |   |   |     |     |
   *  Bit 6   |   | X |   |   |     |     |
   *  Bit 7   |   | X |   |   |     |     |
   *  --------+---+---+---+---+-...-+-----+
   *
   * So, in order to draw a white, horizontal line, at row 45. we would have
   * to modify all of the bytes in* page 45/8 = 5.  We would have to set bit
   * 45%8 = 5 in every byte in the page.
   */

  fbmask = 1 << (row & 7);
  fbptr = &priv->fb[page * ST7565_XRES + col];
  ptr = fbptr;
#ifdef CONFIG_LCD_PACKEDMSFIRST
  usrmask = MS_BIT;
#else
  usrmask = LS_BIT;
#endif

  for (i = 0; i < pixlen; i++)
    {
      /* Set or clear the corresponding bit */

      if ((*buffer & usrmask) != 0)
        {
          *ptr++ |= fbmask;
        }
      else
        {
          *ptr++ &= ~fbmask;
        }

      /* Inc/Decrement to the next source pixel */

#ifdef CONFIG_LCD_PACKEDMSFIRST
      if (usrmask == LS_BIT)
        {
          buffer++;
          usrmask = MS_BIT;
        }
      else
        {
          usrmask >>= 1;
        }
#else
      if (usrmask == MS_BIT)
        {
          buffer++;
          usrmask = LS_BIT;
        }
      else
        {
          usrmask <<= 1;
        }
#endif
    }

  /* Select and lock the device */

  st7565_select(priv);

  /* Select command transfer */

  st7565_cmddata(priv, true);

  /* Set the starting position for the run */

  st7565_send_one_data(priv, ST7565_SETPAGESTART + page);
  st7565_send_one_data(priv, ST7565_SETCOLL + (col & 0x0f));
  st7565_send_one_data(priv, ST7565_SETCOLH + (col >> 4));

  /* Select data transfer */

  st7565_cmddata(priv, false);

  /* Then transfer all of the data */

  st7565_send_data_buf(priv, fbptr, pixlen);

  /* Unlock and de-select the device */

  st7565_deselect(priv);

  return OK;
}

/**************************************************************************************
 * Name:  st7565_getrun
 *
 * TODO implement read function that possible in 8080bus
 *
 * Description:
 *   This method can be used to read a partial raster line from the LCD:
 *
 *  row     - Starting row to read from (range: 0 <= row < yres)
 *  col     - Starting column to read read (range: 0 <= col <= xres-npixels)
 *  buffer  - The buffer in which to return the run read from the LCD
 *  npixels - The number of pixels to read from the LCD
 *            (range: 0 < npixels <= xres-col)
 *
 **************************************************************************************/

static int st7565_getrun(fb_coord_t row, fb_coord_t col, FAR uint8_t * buffer,
                         size_t npixels)
{
  /* Because of this line of code, we will only be able to support a single
   * ST7565 device.
   */

  FAR struct st7565_dev_s *priv = &g_st7565dev;
  FAR uint8_t *fbptr;
  uint8_t page;
  uint8_t fbmask;
  uint8_t usrmask;
  uint8_t i;
  int pixlen;

  ginfo("row: %d col: %d npixels: %d\n", row, col, npixels);
  DEBUGASSERT(buffer);

  /* Clip the run to the display */

  pixlen = npixels;
  if ((unsigned int)col + (unsigned int)pixlen > (unsigned int)ST7565_XRES)
    {
      pixlen = (int)ST7565_XRES - (int)col;
    }

  /* Verify that some portion of the run is actually the display */

  if (pixlen <= 0 || row > ST7565_YRES)
    {
      return -EINVAL;
    }

  /* Then transfer the display data from the shadow frame buffer memory */
  /* Get the page number.  The range of 64 lines is divided up into eight pages
   * of 8 lines each.
   */

  page = row >> 3;

  /* Update the shadow frame buffer memory. First determine the pixel position
   * in the frame buffer memory.  Pixels are organized like this:
   *
   *  --------+---+---+---+---+-...-+-----+
   *  Segment | 0 | 1 | 2 | 3 | ... | 131 |
   *  --------+---+---+---+---+-...-+-----+
   *  Bit 0   |   | X |   |   |     |     |
   *  Bit 1   |   | X |   |   |     |     |
   *  Bit 2   |   | X |   |   |     |     |
   *  Bit 3   |   | X |   |   |     |     |
   *  Bit 4   |   | X |   |   |     |     |
   *  Bit 5   |   | X |   |   |     |     |
   *  Bit 6   |   | X |   |   |     |     |
   *  Bit 7   |   | X |   |   |     |     |
   *  --------+---+---+---+---+-...-+-----+
   *
   * So, in order to draw a white, horizontal line, at row 45. we would have
   * to modify all of the bytes in page 45/8 = 5.  We would have to set bit
   * 45%8 = 5 in every byte in the page.
   */

  fbmask = 1 << (row & 7);
  fbptr = &priv->fb[page * ST7565_XRES + col];
#ifdef CONFIG_LCD_PACKEDMSFIRST
  usrmask = MS_BIT;
#else
  usrmask = LS_BIT;
#endif

  *buffer = 0;
  for (i = 0; i < pixlen; i++)
    {
      /* Set or clear the corresponding bit */

      uint8_t byte = *fbptr++;
      if ((byte & fbmask) != 0)
        {
          *buffer |= usrmask;
        }

      /* Inc/Decrement to the next destination pixel. Hmmmm. It looks like this
       * logic could write past the end of the user buffer.  Revisit this!
       */

#ifdef CONFIG_LCD_PACKEDMSFIRST
      if (usrmask == LS_BIT)
        {
          buffer++;
          *buffer = 0;
          usrmask = MS_BIT;
        }
      else
        {
          usrmask >>= 1;
        }
#else
      if (usrmask == MS_BIT)
        {
          buffer++;
          *buffer = 0;
          usrmask = LS_BIT;
        }
      else
        {
          usrmask <<= 1;
        }
#endif
    }

  return OK;
}

/**************************************************************************************
 * Name:  st7565_getvideoinfo
 *
 * Description:
 *   Get information about the LCD video controller configuration.
 *
 **************************************************************************************/

static int st7565_getvideoinfo(FAR struct lcd_dev_s *dev,
                               FAR struct fb_videoinfo_s *vinfo)
{
  DEBUGASSERT(dev && vinfo);
  ginfo("fmt: %d xres: %d yres: %d nplanes: %d\n",
        g_videoinfo.fmt, g_videoinfo.xres, g_videoinfo.yres,
        g_videoinfo.nplanes);
  memcpy(vinfo, &g_videoinfo, sizeof(struct fb_videoinfo_s));
  return OK;
}

/**************************************************************************************
 * Name:  st7565_getplaneinfo
 *
 * Description:
 *   Get information about the configuration of each LCD color plane.
 *
 **************************************************************************************/

static int st7565_getplaneinfo(FAR struct lcd_dev_s *dev, unsigned int planeno,
                               FAR struct lcd_planeinfo_s *pinfo)
{
  DEBUGASSERT(dev && pinfo && planeno == 0);
  ginfo("planeno: %d bpp: %d\n", planeno, g_planeinfo.bpp);
  memcpy(pinfo, &g_planeinfo, sizeof(struct lcd_planeinfo_s));
  return OK;
}

/**************************************************************************************
 * Name:  st7565_getpower
 *
 * Description:
 *   Get the LCD panel power status (0: full off - CONFIG_LCD_MAXPOWER: full on). On
 *   backlit LCDs, this setting may correspond to the backlight setting.
 *
 **************************************************************************************/

static int st7565_getpower(struct lcd_dev_s *dev)
{
  struct st7565_dev_s *priv = (struct st7565_dev_s *)dev;
  DEBUGASSERT(priv);
  ginfo("powered: %s\n", priv->power_level);
  return priv->power_level;
}

/**************************************************************************************
 * Name:  st7565_setpower
 *
 * Description:
 *   Enable/disable LCD panel power (0: full off - CONFIG_LCD_MAXPOWER: full on). On
 *   backlight LCDs, this setting may correspond to the backlight setting.
 *
 **************************************************************************************/

static int st7565_setpower(struct lcd_dev_s *dev, int power)
{
  struct st7565_dev_s *priv = (struct st7565_dev_s *)dev;

  DEBUGASSERT(priv && (unsigned)power <= CONFIG_LCD_MAXPOWER);
  ginfo("power: %s powered: %s\n", power, priv->power_level);

  /* Select and lock the device */

  st7565_select(priv);

  st7565_cmddata(priv, true);

  if (power <= 0)
    {
      /* Turn the display off */

      st7565_send_one_data(priv, ST7565_DISPOFF);
      st7565_backlight(priv, 0);
      priv->power_level = 0;
    }
  else
    {
      st7565_send_one_data(priv, ST7565_DISPON);
      st7565_send_one_data(priv, ST7565_DISPRAM);

      /* Don't use value 1 of backlight to allow low power mode */

      if (power == 1)
        {
          st7565_backlight(priv, 0);
        }
      else
        {
          st7565_backlight(priv, power);
        }

      priv->power_level = 1;
    }

  st7565_cmddata(priv, false);
  st7565_deselect(priv);

  return OK;
}

/**************************************************************************************
 * Name:  st7565_getcontrast
 *
 * Description:
 *   Get the current contrast setting (0-CONFIG_LCD_MAXCONTRAST).
 *
 **************************************************************************************/

static int st7565_getcontrast(struct lcd_dev_s *dev)
{
  struct st7565_dev_s *priv = (struct st7565_dev_s *)dev;
  DEBUGASSERT(priv);
  return (int)priv->contrast;
}

/**************************************************************************************
 * Name:  st7565_setcontrast
 *
 * Description:
 *   Set LCD panel contrast (0-CONFIG_LCD_MAXCONTRAST).
 *
 **************************************************************************************/

static int st7565_setcontrast(struct lcd_dev_s *dev, unsigned int contrast)
{
  struct st7565_dev_s *priv = (struct st7565_dev_s *)dev;

  ginfo("contrast: %d\n", contrast);
  DEBUGASSERT(priv);

  if (contrast > 255)
    {
      return -EINVAL;
    }

  /* Select and lock the device */

  st7565_select(priv);

  /* Select command transfer */

  st7565_cmddata(priv, true);

  /* Set the contrast */

  st7565_send_one_data(priv, ST7565_SETEVMODE);
  st7565_send_one_data(priv, ST7565_SETEVREG(contrast));
  priv->contrast = contrast;

  /* Deselect command transfer */

  st7565_cmddata(priv, false);

  /* Unlock and de-select the device */

  st7565_deselect(priv);
  return OK;
}

/**************************************************************************************
 * Name:  up_clear
 *
 * Description:
 *   Clear the display.
 *
 **************************************************************************************/

static inline void up_clear(FAR struct st7565_dev_s *priv)
{
  int page;
  int i;

  /* Clear the framebuffer */

  memset(priv->fb, 0x00, ST7565_FBSIZE);

  /* Select and lock the device */

  st7565_select(priv);

  /* Go throw all 8 pages */

  for (page = 0, i = 0; i < 8; i++)
    {
      /* Select command transfer */

      st7565_cmddata(priv, true);

      /* Set the starting position for the run */

      st7565_send_one_data(priv, ST7565_SETPAGESTART + i);
      st7565_send_one_data(priv, ST7565_SETCOLL);
      st7565_send_one_data(priv, ST7565_SETCOLH);

      /* Select data transfer */

      st7565_cmddata(priv, false);

      /* Then transfer all 96 columns of data */

      st7565_send_data_buf(priv, &priv->fb[page * ST7565_XRES],
                           ST7565_XRES);
    }

  /* Unlock and de-select the device */

  st7565_deselect(priv);
}

/**************************************************************************************
 * Public Functions
 **************************************************************************************/

/**************************************************************************************
 * Name:  st7565_initialize
 *
 * Description:
 *   Initialize the ST7565 video hardware.  The initial state of the
 *   OLED is fully initialized, display memory cleared, and the OLED ready to
 *   use, but with the power setting at 0 (full off == sleep mode).
 *
 * Input Parameters:
 *
 *   lcd - A reference to the lcd low level driver instance.
 *   devno - A value in the range of 0 throw CONFIG_ST7565_NINTERFACES-1.
 *     This allows support for multiple OLED devices.
 *
 * Returned Value:
 *
 *   On success, this function returns a reference to the LCD object
 *   for the specified
 *   OLED.  NULL is returned on any failure.
 *
 **************************************************************************************/

FAR struct lcd_dev_s *st7565_initialize(FAR struct st7565_lcd_s *lcd,
                                        unsigned int devno)
{
  /* Configure and enable LCD */

  FAR struct st7565_dev_s *priv = &g_st7565dev;

  ginfo("Initializing\n");
  DEBUGASSERT(lcd && devno == 0);

  /* Save the reference to the SPI device */

  priv->lcd = lcd;

  /* Select and lock the device */

  st7565_select(priv);

  /* Reset device */

  st7565_reset(priv, true);

  /* it seems too long but written in NHD‐C12864KGZ DISPLAY
   * INITIALIZATION...
   */

  up_mdelay(150);

  st7565_reset(priv, false);

  /* it seems too long but written in NHD‐C12864KGZ DISPLAY
   * INITIALIZATION...
   */

  up_mdelay(150);

  /* Make sure that LCD backlight is off */

  st7565_backlight(priv, 0);

  /* Select command transfer */

  st7565_cmddata(priv, true);

  /* Reset by command in case of st7565_reset not implemented */

  st7565_send_one_data(priv, ST7565_EXIT_SOFTRST);

  /* Follow NHD-C12864KGZ DISPLAY INITIALIZATION...  */

#if defined(CONFIG_NHD_C12864KGZ)

  st7565_send_one_data(priv, ST7565_BIAS_1_9);

  st7565_send_one_data(priv, ST7565_REG_RES_5_5);
  st7565_send_one_data(priv, ST7565_SETEVMODE);
  st7565_send_one_data(priv, ST7565_SETEVREG(15));
  st7565_send_one_data(priv, ST7565_POWERCTRL_INT);
  st7565_send_one_data(priv, ST7565_SETSTARTLINE);

#elif defined(CONFIG_ERC_12864_3)

  st7565_send_one_data(priv, ST7565_ADCNORMAL);
  st7565_send_one_data(priv, ST7565_SETCOMREVERSE);
  st7565_send_one_data(priv, ST7565_BIAS_1_9);
  st7565_send_one_data(priv, ST7565_POWERCTRL_INT);
  st7565_send_one_data(priv, ST7565_REG_RES_5_5);
  st7565_send_one_data(priv, ST7565_SETEVMODE);
  st7565_send_one_data(priv, ST7565_SETEVREG(0x24));
  st7565_send_one_data(priv, ST7565_SETSTARTLINE);

#elif defined(CONFIG_AQM_1248A)

  st7565_send_one_data(priv, ST7565_DISPOFF);
  st7565_send_one_data(priv, ST7565_ADCNORMAL);
  st7565_send_one_data(priv, ST7565_SETCOMREVERSE);
  st7565_send_one_data(priv, ST7565_BIAS_1_7);

  st7565_send_one_data(priv, ST7565_POWERCTRL_B);
  up_mdelay(2);
  st7565_send_one_data(priv, ST7565_POWERCTRL_BR);
  up_mdelay(2);
  st7565_send_one_data(priv, ST7565_POWERCTRL_INT);

  st7565_send_one_data(priv, ST7565_REG_RES_4_5);
  st7565_send_one_data(priv, ST7565_SETEVMODE);
  st7565_send_one_data(priv, ST7565_SETEVREG(0x1c));
  st7565_send_one_data(priv, ST7565_DISPRAM);
  st7565_send_one_data(priv, ST7565_SETSTARTLINE);
  st7565_send_one_data(priv, ST7565_DISPNORMAL);
  st7565_send_one_data(priv, ST7565_DISPON);

#else
#  error "No initialization sequence selected"
#endif

#if 0
  st7565_send_one_data(priv, ST7565_DISPON);
  st7565_send_one_data(priv, ST7565_SETCOMREVERSE);
  st7565_send_one_data(priv, ST7565_REG_RES_RR1);
  st7565_send_one_data(priv, ST7565_SETEV);
  st7565_send_one_data(priv, 0x32);
  st7565_send_one_data(priv, ST7565_POWERCTRL);
  st7565_send_one_data(priv, ST7565_SETSTARTLINE);
  st7565_send_one_data(priv, ST7565_SETPAGESTART);
  st7565_send_one_data(priv, ST7565_SETCOLH);
  st7565_send_one_data(priv, ST7565_SETCOLL);
  st7565_send_one_data(priv, ST7565_DISPON);
  st7565_send_one_data(priv, ST7565_DISPRAM);
#endif

#ifdef CONFIG_ST7565_MIRROR_X
  st7565_send_one_data(priv, ST7565_ADCINVERSE);
#else
  st7565_send_one_data(priv, ST7565_ADCNORMAL);
#endif

#ifdef CONFIG_ST7565_MIRROR_Y
  st7565_send_one_data(priv, ST7565_SETCOMREVERSE);
#else
  st7565_send_one_data(priv, ST7565_SETCOMNORMAL);
#endif

#ifdef CONFIG_ST7565_INVERSE_VIDEO
  st7565_send_one_data(priv, ST7565_DISPINVERSE);
#else
  st7565_send_one_data(priv, ST7565_DISPNORMAL);
#endif

  /* Let go of the SPI lock and de-select the device */

  st7565_deselect(priv);

  up_mdelay(10);

  /* Clear the framebuffer */

  up_clear(priv);

  return &priv->dev;
}
