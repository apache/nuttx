/**************************************************************************************
 * drivers/lcd/st7567.c
 *
 * Driver for the TM12864J1CCWGWA Display with the ST7567 LCD
 * controller.
 *
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
#include <nuttx/spi.h>
#include <nuttx/lcd/lcd.h>
#include <nuttx/lcd/st7567.h>

#include "st7567.h"

/**************************************************************************************
 * Pre-processor Definitions
 **************************************************************************************/

/* Configuration **********************************************************************/
/* ST7567 Configuration Settings:
 *
 * CONFIG_ST7567_SPIMODE - Controls the SPI mode
 * CONFIG_ST7567_FREQUENCY - Define to use a different bus frequency
 * CONFIG_ST7567_NINTERFACES - Specifies the number of physical
 *   ST7567 devices that will be supported.  NOTE:  At present, this
 *   must be undefined or defined to be 1.
 * CONFIG_ST7567_POWER
 *   If the hardware supports a controllable OLED a power supply, this
 *   configuration shold be defined.  (See st7567_power() below).
 * CONFIG_LCD_ST7567DEBUG - Enable detailed ST7567 debst7567 output
 *   (CONFIG_DEBUG and CONFIG_VERBOSE must also be enabled).
 *
 * Required LCD driver settings:
 * CONFIG_LCD_ST7567 - Enable ST7567 support
 * CONFIG_LCD_MAXCONTRAST should be 255, but any value >0 and <=255 will be accepted.
 * CONFIG_LCD_MAXPOWER should be 2:  0=off, 1=dim, 2=normal
 *
 * Required SPI driver settings:
 * CONFIG_SPI_CMDDATA - Include support for cmd/data selection.
 */

/* Verify that all configuration requirements have been met */

/* The ST7567 spec says that is supports SPI mode 0,0 only.  However, somtimes
 * you need to tinker with these things.
 */

#ifndef CONFIG_ST7567_SPIMODE
#  define CONFIG_ST7567_SPIMODE SPIDEV_MODE0
#endif

/* SPI frequency */

#ifndef CONFIG_ST7567_FREQUENCY
#  define CONFIG_ST7567_FREQUENCY 3500000
#endif

/* CONFIG_ST7567_NINTERFACES determines the number of physical interfaces
 * that will be supported.
 */

#ifndef CONFIG_ST7567_NINTERFACES
#  define CONFIG_ST7567_NINTERFACES 1
#endif

#if CONFIG_ST7567_NINTERFACES != 1
#  warning "Only a single ST7567 interface is supported"
#  undef CONFIG_ST7567_NINTERFACES
#  define CONFIG_ST7567_NINTERFACES 1
#endif

/* Verbose debst7567 must also be enabled to use the extra OLED debst7567 */

#ifndef CONFIG_DEBUG
#  undef CONFIG_DEBUG_VERBOSE
#  undef CONFIG_DEBUG_GRAPHICS
#endif

#ifndef CONFIG_DEBUG_VERBOSE
#  undef CONFIG_LCD_ST7567DEBUG
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

/* Check power setting */

#if !defined(CONFIG_LCD_MAXPOWER)
#  define CONFIG_LCD_MAXPOWER 1
#endif

#if CONFIG_LCD_MAXPOWER != 1
#  warning "CONFIG_LCD_MAXPOWER should be 1"
#  undef CONFIG_LCD_MAXPOWER
#  define CONFIG_LCD_MAXPOWER 1
#endif

/* The OLED requires CMD/DATA SPI support */

#ifndef CONFIG_SPI_CMDDATA
#  error "CONFIG_SPI_CMDDATA must be defined in your NuttX configuration"
#endif

/* Color is 1bpp monochrome with leftmost column contained in bits 0  */

#ifdef CONFIG_NX_DISABLE_1BPP
#  warning "1 bit-per-pixel support needed"
#endif

/* Color Properties *******************************************************************/
/* The ST7567 display controller can handle a resolution of 128x64.
 */
/* Display Resolution */

#ifdef CONFIG_ST7567_XRES
#define ST7567_XRES         CONFIG_ST7567_XRES
#else
#define ST7567_XRES         128
#endif

#ifdef CONFIG_ST7567_YRES
#define ST7567_YRES         CONFIG_ST7567_YRES
#else
#define ST7567_YRES         64
#endif

/* Color depth and format */

#define ST7567_BPP          1
#define ST7567_COLORFMT     FB_FMT_Y1

/* Bytes per logical row andactual device row */

#define ST7567_XSTRIDE      (ST7567_XRES >> 3) /* Pixels arrange "horizontally for user" */
#define ST7567_YSTRIDE      (ST7567_YRES >> 3) /* But actual device arrangement is "vertical" */

/* The size of the shadow frame buffer */

#define ST7567_FBSIZE       (ST7567_XRES * ST7567_YSTRIDE)

/* Bit helpers */

#define LS_BIT          (1 << 0)
#define MS_BIT          (1 << 7)

/* Debst7567 ******************************************************************************/

#ifdef CONFIG_LCD_ST7567DEBUG
# define st7567dbg(format, arg...)  vdbg(format, ##arg)
#else
# define st7567dbg(x...)
#endif

/**************************************************************************************
 * Private Type Definition
 **************************************************************************************/

/* This structure describes the state of this driver */

struct st7567_dev_s
{
  /* Publically visible device structure */

  struct lcd_dev_s dev;

  /* Private LCD-specific information follows */

  FAR struct spi_dev_s *spi;
  uint8_t contrast;
  uint8_t powered;

 /* The ST7567 does not support reading from the display memory in SPI mode.
  * Since there is 1 BPP and access is byte-by-byte, it is necessary to keep
  * a shadow copy of the framebuffer memory.
  */

  uint8_t fb[ST7567_FBSIZE];
};

/**************************************************************************************
 * Private Function Protototypes
 **************************************************************************************/

/* SPI helpers */

#ifdef CONFIG_SPI_OWNBUS
static inline void st7567_select(FAR struct spi_dev_s *spi);
static inline void st7567_deselect(FAR struct spi_dev_s *spi);
#else
static void st7567_select(FAR struct spi_dev_s *spi);
static void st7567_deselect(FAR struct spi_dev_s *spi);
#endif

/* LCD Data Transfer Methods */

static int st7567_putrun(fb_coord_t row, fb_coord_t col, FAR const uint8_t *buffer,
                     size_t npixels);
static int st7567_getrun(fb_coord_t row, fb_coord_t col, FAR uint8_t *buffer,
                     size_t npixels);

/* LCD Configuration */

static int st7567_getvideoinfo(FAR struct lcd_dev_s *dev,
                           FAR struct fb_videoinfo_s *vinfo);
static int st7567_getplaneinfo(FAR struct lcd_dev_s *dev, unsigned int planeno,
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

static int st7567_getpower(struct lcd_dev_s *dev);
static int st7567_setpower(struct lcd_dev_s *dev, int power);
static int st7567_getcontrast(struct lcd_dev_s *dev);
static int st7567_setcontrast(struct lcd_dev_s *dev, unsigned int contrast);

/* Initialization */

static inline void up_clear(FAR struct st7567_dev_s  *priv);

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

static uint8_t g_runbuffer[ST7567_XSTRIDE+1];

/* This structure describes the overall LCD video controller */

static const struct fb_videoinfo_s g_videoinfo =
{
  .fmt     = ST7567_COLORFMT,    /* Color format: RGB16-565: RRRR RGGG GGGB BBBB */
  .xres    = ST7567_XRES,        /* Horizontal resolution in pixel columns */
  .yres    = ST7567_YRES,        /* Vertical resolution in pixel rows */
  .nplanes = 1,              /* Number of color planes supported */
};

/* This is the standard, NuttX Plane information object */

static const struct lcd_planeinfo_s g_planeinfo =
{
  .putrun = st7567_putrun,             /* Put a run into LCD memory */
  .getrun = st7567_getrun,             /* Get a run from LCD memory */
  .buffer = (uint8_t*)g_runbuffer, /* Run scratch buffer */
  .bpp    = ST7567_BPP,                /* Bits-per-pixel */
};

/* This is the standard, NuttX LCD driver object */

static struct st7567_dev_s g_st7567dev =
{
  .dev =
  {
    /* LCD Configuration */

    .getvideoinfo = st7567_getvideoinfo,
    .getplaneinfo = st7567_getplaneinfo,

    /* LCD RGB Mapping -- Not supported */
    /* Cursor Controls -- Not supported */

    /* LCD Specific Controls */

    .getpower     = st7567_getpower,
    .setpower     = st7567_setpower,
    .getcontrast  = st7567_getcontrast,
    .setcontrast  = st7567_setcontrast,
  },
};

/**************************************************************************************
 * Private Functions
 **************************************************************************************/

/**************************************************************************************
 * Function: st7567_select
 *
 * Description:
 *   Select the SPI, locking and  re-configuring if necessary
 *
 * Parameters:
 *   spi  - Reference to the SPI driver structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 **************************************************************************************/

#ifdef CONFIG_SPI_OWNBUS
static inline void st7567_select(FAR struct spi_dev_s *spi)
{
  /* We own the SPI bus, so just select the chip */

  SPI_SELECT(spi, SPIDEV_DISPLAY, true);
}
#else
static void st7567_select(FAR struct spi_dev_s *spi)
{
  /* Select ST7567 chip (locking the SPI bus in case there are multiple
   * devices competing for the SPI bus
   */

  SPI_LOCK(spi, true);
  SPI_SELECT(spi, SPIDEV_DISPLAY, true);

  /* Now make sure that the SPI bus is configured for the ST7567 (it
   * might have gotten configured for a different device while unlocked)
   */

  SPI_SETMODE(spi, CONFIG_ST7567_SPIMODE);
  SPI_SETBITS(spi, 8);
#ifdef CONFIG_ST7567_FREQUENCY
  SPI_SETFREQUENCY(spi, CONFIG_ST7567_FREQUENCY);
#endif
}
#endif

/**************************************************************************************
 * Function: st7567_deselect
 *
 * Description:
 *   De-select the SPI
 *
 * Parameters:
 *   spi  - Reference to the SPI driver structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 **************************************************************************************/

#ifdef CONFIG_SPI_OWNBUS
static inline void st7567_deselect(FAR struct spi_dev_s *spi)
{
  /* We own the SPI bus, so just de-select the chip */

  SPI_SELECT(spi, SPIDEV_DISPLAY, false);
}
#else
static void st7567_deselect(FAR struct spi_dev_s *spi)
{
  /* De-select ST7567 chip and relinquish the SPI bus. */

  SPI_SELECT(spi, SPIDEV_DISPLAY, false);
  SPI_LOCK(spi, false);
}
#endif

/**************************************************************************************
 * Name:  st7567_putrun
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

static int st7567_putrun(fb_coord_t row, fb_coord_t col, FAR const uint8_t *buffer,
                       size_t npixels)
{
  /* Because of this line of code, we will only be able to support a single ST7567 device */

  FAR struct st7567_dev_s *priv = &g_st7567dev;
  FAR uint8_t *fbptr;
  FAR uint8_t *ptr;
  uint8_t devcol;
  uint8_t fbmask;
  uint8_t page;
  uint8_t usrmask;
  uint8_t i;
  int     pixlen;

  gvdbg("row: %d col: %d npixels: %d\n", row, col, npixels);
  DEBUGASSERT(buffer);

  /* Clip the run to the display */

  pixlen = npixels;
  if ((unsigned int)col + (unsigned int)pixlen > (unsigned int)ST7567_XRES)
    {
      pixlen = (int)ST7567_XRES - (int)col;
    }

  /* Verify that some portion of the run remains on the display */

  if (pixlen <= 0 || row > ST7567_YRES)
    {
      return OK;
    }

  /* Get the page number.  The range of 64 lines is divided up into eight
   * pages of 8 lines each.
   */

  page = row >> 3;

  /* Update the shadow frame buffer memory. First determine the pixel
   * position in the frame buffer memory.  Pixels are organized like
   * this:
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
   * So, in order to draw a white, horizontal line, at row 45. we
   * would have to modify all of the bytes in page 45/8 = 5.  We
   * would have to set bit 45%8 = 5 in every byte in the page.
   */

  fbmask  = 1 << (row & 7);
  fbptr   = &priv->fb[page * ST7567_XRES + col];
  ptr     = fbptr;
#ifdef CONFIG_NX_PACKEDMSFIRST
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

#ifdef CONFIG_NX_PACKEDMSFIRST
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

  st7567_select(priv->spi);

  /* Select command transfer */

  SPI_CMDDATA(priv->spi, SPIDEV_DISPLAY, true);

  /* Set the starting position for the run */

  (void)SPI_SEND(priv->spi, ST7567_SETPAGESTART+page);         /* Set the page start */
  (void)SPI_SEND(priv->spi, ST7567_SETCOLL + (col & 0x0f));    /* Set the low column */
  (void)SPI_SEND(priv->spi, ST7567_SETCOLH + (col >> 4));      /* Set the high column */

  /* Select data transfer */

  SPI_CMDDATA(priv->spi, SPIDEV_DISPLAY, false);

  /* Then transfer all of the data */

  (void)SPI_SNDBLOCK(priv->spi, fbptr, pixlen);

  /* Unlock and de-select the device */

  st7567_deselect(priv->spi);
  return OK;
}

/**************************************************************************************
 * Name:  st7567_getrun
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

static int st7567_getrun(fb_coord_t row, fb_coord_t col, FAR uint8_t *buffer,
                     size_t npixels)
{
  /* Because of this line of code, we will only be able to support a single ST7567 device */

  FAR struct st7567_dev_s *priv = &g_st7567dev;
  FAR uint8_t *fbptr;
  uint8_t page;
  uint8_t fbmask;
  uint8_t usrmask;
  uint8_t i;
  int     pixlen;

  gvdbg("row: %d col: %d npixels: %d\n", row, col, npixels);
  DEBUGASSERT(buffer);

  /* Clip the run to the display */

  pixlen = npixels;
  if ((unsigned int)col + (unsigned int)pixlen > (unsigned int)ST7567_XRES)
    {
      pixlen = (int)ST7567_XRES - (int)col;
    }

  /* Verify that some portion of the run is actually the display */

  if (pixlen <= 0 || row > ST7567_YRES)
    {
      return -EINVAL;
    }

  /* Then transfer the display data from the shadow frame buffer memory */
  /* Get the page number.  The range of 64 lines is divided up into eight
   * pages of 8 lines each.
   */

  page = row >> 3;

  /* Update the shadow frame buffer memory. First determine the pixel
   * position in the frame buffer memory.  Pixels are organized like
   * this:
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
   * So, in order to draw a white, horizontal line, at row 45. we
   * would have to modify all of the bytes in page 45/8 = 5.  We
   * would have to set bit 45%8 = 5 in every byte in the page.
   */

  fbmask  = 1 << (row & 7);
  fbptr   = &priv->fb[page * ST7567_XRES + col];
#ifdef CONFIG_NX_PACKEDMSFIRST
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

      /* Inc/Decrement to the next destination pixel. Hmmmm. It looks like
       * this logic could write past the end of the user buffer.  Revisit
       * this!
       */

#ifdef CONFIG_NX_PACKEDMSFIRST
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
 * Name:  st7567_getvideoinfo
 *
 * Description:
 *   Get information about the LCD video controller configuration.
 *
 **************************************************************************************/

static int st7567_getvideoinfo(FAR struct lcd_dev_s *dev,
                              FAR struct fb_videoinfo_s *vinfo)
{
  DEBUGASSERT(dev && vinfo);
  gvdbg("fmt: %d xres: %d yres: %d nplanes: %d\n",
         g_videoinfo.fmt, g_videoinfo.xres, g_videoinfo.yres, g_videoinfo.nplanes);
  memcpy(vinfo, &g_videoinfo, sizeof(struct fb_videoinfo_s));
  return OK;
}

/**************************************************************************************
 * Name:  st7567_getplaneinfo
 *
 * Description:
 *   Get information about the configuration of each LCD color plane.
 *
 **************************************************************************************/

static int st7567_getplaneinfo(FAR struct lcd_dev_s *dev, unsigned int planeno,
                              FAR struct lcd_planeinfo_s *pinfo)
{
  DEBUGASSERT(dev && pinfo && planeno == 0);
  gvdbg("planeno: %d bpp: %d\n", planeno, g_planeinfo.bpp);
  memcpy(pinfo, &g_planeinfo, sizeof(struct lcd_planeinfo_s));
  return OK;
}

/**************************************************************************************
 * Name:  st7567_getpower
 *
 * Description:
 *   Get the LCD panel power status (0: full off - CONFIG_LCD_MAXPOWER: full on). On
 *   backlit LCDs, this setting may correspond to the backlight setting.
 *
 **************************************************************************************/

static int st7567_getpower(struct lcd_dev_s *dev)
{
  struct st7567_dev_s *priv = (struct st7567_dev_s *)dev;
  DEBUGASSERT(priv);
  gvdbg("powered: %s\n", st7567_powerstring(priv->powered));
  return priv->powered;
}

/**************************************************************************************
 * Name:  st7567_setpower
 *
 * Description:
 *   Enable/disable LCD panel power (0: full off - CONFIG_LCD_MAXPOWER: full on). On
 *   backlit LCDs, this setting may correspond to the backlight setting.
 *
 **************************************************************************************/

static int st7567_setpower(struct lcd_dev_s *dev, int power)
{
  struct st7567_dev_s *priv = (struct st7567_dev_s *)dev;

  DEBUGASSERT(priv && (unsigned)power <= CONFIG_LCD_MAXPOWER);
  gvdbg("power: %s powered: %s\n",
        st7567_powerstring(power), st7567_powerstring(priv->powered));

  /* Select and lock the device */

  st7567_select(priv->spi);
  if (power <= ST7567_POWER_OFF)
    {
      /* Turn the display off */

      (void)SPI_SEND(priv->spi, ST7567_DISPOFF);       /* Display off */
      priv->powered = ST7567_POWER_OFF;
    }
  else
    {
      (void)SPI_SEND(priv->spi, ST7567_DISPON);        /* Display on, normal mode */
      power = ST7567_POWER_ON;

      (void)SPI_SEND(priv->spi, ST7567_DISPRAM);       /* Resume to RAM content display */
      priv->powered = power;
    }

  st7567_deselect(priv->spi);

  return OK;
}

/**************************************************************************************
 * Name:  st7567_getcontrast
 *
 * Description:
 *   Get the current contrast setting (0-CONFIG_LCD_MAXCONTRAST).
 *
 **************************************************************************************/

static int st7567_getcontrast(struct lcd_dev_s *dev)
{
  struct st7567_dev_s *priv = (struct st7567_dev_s *)dev;
  DEBUGASSERT(priv);
  return (int)priv->contrast;
}

/**************************************************************************************
 * Name:  st7567_setcontrast
 *
 * Description:
 *   Set LCD panel contrast (0-CONFIG_LCD_MAXCONTRAST).
 *
 **************************************************************************************/

static int st7567_setcontrast(struct lcd_dev_s *dev, unsigned int contrast)
{
  struct st7567_dev_s *priv = (struct st7567_dev_s *)dev;

  gvdbg("contrast: %d\n", contrast);
  DEBUGASSERT(priv);

  if (contrast > 255)
    {
      return -EINVAL;
    }

  /* Select and lock the device */

  st7567_select(priv->spi);

  /* Select command transfer */

  SPI_CMDDATA(priv->spi, SPIDEV_DISPLAY, true);

  /* Set the contrast */

  (void)SPI_SEND(priv->spi, ST7567_SETEV);         /* Set contrast control register */
  (void)SPI_SEND(priv->spi, contrast);             /* Data 1: Set 1 of 256 contrast steps */
  priv->contrast = contrast;

  /* Unlock and de-select the device */

  st7567_deselect(priv->spi);
  return OK;
}

/**************************************************************************************
 * Name:  up_clear
 *
 * Description:
 *   Clear the display.
 *
 **************************************************************************************/

static inline void up_clear(FAR struct st7567_dev_s  *priv)
{
  FAR struct spi_dev_s *spi  = priv->spi;
  int page;
  int i;

  /* Clear the framebuffer */

  memset(priv->fb, ST7567_Y1_BLACK, ST7567_FBSIZE);

  /* Select and lock the device */

  st7567_select(priv->spi);

  /* Go throst7567h all 8 pages */

  for (page = 0, i = 0; i < 8; i++)
    {
      /* Select command transfer */

      SPI_CMDDATA(spi, SPIDEV_DISPLAY, true);

      /* Set the starting position for the run */

      (void)SPI_SEND(priv->spi, ST7567_SETPAGESTART+i);
      (void)SPI_SEND(priv->spi, ST7567_SETCOLL);
      (void)SPI_SEND(priv->spi, ST7567_SETCOLH);

      /* Select data transfer */

      SPI_CMDDATA(spi, SPIDEV_DISPLAY, false);

       /* Then transfer all 96 columns of data */

       (void)SPI_SNDBLOCK(priv->spi, &priv->fb[page * ST7567_XRES], ST7567_XRES);
    }

  /* Unlock and de-select the device */

  st7567_deselect(spi);
}

/**************************************************************************************
 * Public Functions
 **************************************************************************************/

/**************************************************************************************
 * Name:  st7567_initialize
 *
 * Description:
 *   Initialize the ST7567 video hardware.  The initial state of the
 *   OLED is fully initialized, display memory cleared, and the OLED ready to
 *   use, but with the power setting at 0 (full off == sleep mode).
 *
 * Input Parameters:
 *
 *   spi - A reference to the SPI driver instance.
 *   devno - A value in the range of 0 throst7567h CONFIG_ST7567_NINTERFACES-1.
 *     This allows support for multiple OLED devices.
 *
 * Returned Value:
 *
 *   On success, this function returns a reference to the LCD object for the specified
 *   OLED.  NULL is returned on any failure.
 *
 **************************************************************************************/

FAR struct lcd_dev_s *st7567_initialize(FAR struct spi_dev_s *spi, unsigned int devno)
{
  /* Configure and enable LCD */

  FAR struct st7567_dev_s  *priv = &g_st7567dev;

  gvdbg("Initializing\n");
  DEBUGASSERT(spi && devno == 0);

  /* Save the reference to the SPI device */

  priv->spi = spi;

  /* Select and lock the device */

  st7567_select(spi);

  /* Make sure that the OLED off */

  st7567_power(0, false);

  /* Select command transfer */

  SPI_CMDDATA(spi, SPIDEV_DISPLAY, true);

  /* Set the starting position for the run */

  (void)SPI_SEND(spi, ST7567_EXIT_SOFTRST);
  (void)SPI_SEND(spi, ST7567_BIAS_1_7);
  (void)SPI_SEND(spi, ST7567_DISPON);
  (void)SPI_SEND(spi, SSD1305_MAPCOL0);
  (void)SPI_SEND(spi, ST7567_SETCOMREVERSE);
  (void)SPI_SEND(spi, ST7567_REG_RES_RR1);
  (void)SPI_SEND(spi, ST7567_SETEV);
  (void)SPI_SEND(spi, 0x32);
  (void)SPI_SEND(spi, ST7567_POWERCTRL);
  (void)SPI_SEND(spi, ST7567_SETSTARTLINE);
  (void)SPI_SEND(spi, ST7567_SETPAGESTART);
  (void)SPI_SEND(spi, ST7567_SETCOLH);
  (void)SPI_SEND(spi, ST7567_SETCOLL);
  (void)SPI_SEND(spi, ST7567_DISPON);
  (void)SPI_SEND(spi, ST7567_DISPRAM);

  /* Let go of the SPI lock and de-select the device */

  st7567_deselect(spi);

  /* Clear the framebuffer */

  up_mdelay(100);
  up_clear(priv);
  return &priv->dev;
}

