/**************************************************************************************
 * drivers/lcd/ssd1306_base.c
 * Driver for Univision UG-2864HSWEG01 OLED display or UG-2832HSWEG04 both with the
 * Univision SSD1306 controller in SPI mode and Densitron DD-12864WO-4A with SSD1309
 * in SPI mode.
 *
 *   Copyright (C) 2012-2013, 2015, 2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * References:
 *   1. Product Specification (Preliminary), Part Name: OEL Display Module, Part ID:
 *      UG-2864HSWEG01, Doc No: SAS1-9046-B, Univision Technology Inc.
 *   2. Product Specification, Part Name: OEL Display Module, Part ID: UG-2832HSWEG04,
 *      Doc No.: SAS1-B020-B, Univision Technology Inc.
 *   3. SSD1306, 128 X 64 Dot Matrix OLED/PLED, Preliminary Segment/Common Driver with
 *      Controller,  Solomon Systech
 *   4. SSD1309, 128 x 64 Dot Matrix OLED/PLED Segment/Common Driver with Controller,
 *      Solomon Systech
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
 * Device memory organization:
 *
 *          +----------------------------+
 *          |           Column           |
 *  --------+----+---+---+---+-...-+-----+
 *  Page    | 0  | 1 | 2 | 3 | ... | 127 |
 *  --------+----+---+---+---+-...-+-----+
 *  Page 0  | D0 | X |   |   |     |     |
 *          | D1 | X |   |   |     |     |
 *          | D2 | X |   |   |     |     |
 *          | D3 | X |   |   |     |     |
 *          | D4 | X |   |   |     |     |
 *          | D5 | X |   |   |     |     |
 *          | D6 | X |   |   |     |     |
 *          | D7 | X |   |   |     |     |
 *  --------+----+---+---+---+-...-+-----+
 *  Page 1  |    |   |   |   |     |     |
 *  --------+----+---+---+---+-...-+-----+
 *  Page 2  |    |   |   |   |     |     |
 *  --------+----+---+---+---+-...-+-----+
 *  Page 3  |    |   |   |   |     |     |
 *  --------+----+---+---+---+-...-+-----+
 *  Page 4  |    |   |   |   |     |     |
 *  --------+----+---+---+---+-...-+-----+
 *  Page 5  |    |   |   |   |     |     |
 *  --------+----+---+---+---+-...-+-----+
 *  Page 6  |    |   |   |   |     |     |
 *  --------+----+---+---+---+-...-+-----+
 *  Page 7  |    |   |   |   |     |     |
 *  --------+----+---+---+---+-...-+-----+
 *
 *  -----------------------------------+---------------------------------------
 *  Landscape Display:                 | Reverse Landscape Display:
 *  --------+-----------------------+  |  --------+---------------------------+
 *          |       Column          |  |          |         Column            |
 *  --------+---+---+---+-...-+-----+  |  --------+-----+-----+-----+-...-+---+
 *  Page 0  | 0 | 1 | 2 |     | 127 |  |  Page 7  | 127 | 126 | 125 |     | 0 |
 *  --------+---+---+---+-...-+-----+  |  --------+-----+-----+-----+-...-+---+
 *  Page 1  | V                     |  |  Page 6  |                         ^ |
 *  --------+---+---+---+-...-+-----+  |  --------+-----+-----+-----+-...-+---+
 *  Page 2  | V                     |  |  Page 5  |                         ^ |
 *  --------+---+---+---+-...-+-----+  |  --------+-----+-----+-----+-...-+---+
 *  Page 3  | V                     |  |  Page 4  |                         ^ |
 *  --------+---+---+---+-...-+-----+  |  --------+-----+-----+-----+-...-+---+
 *  Page 4  | V                     |  |  Page 3  |                         ^ |
 *  --------+---+---+---+-...-+-----+  |  --------+-----+-----+-----+-...-+---+
 *  Page 5  | V                     |  |  Page 2  |                         ^ |
 *  --------+---+---+---+-...-+-----+  |  --------+-----+-----+-----+-...-+---+
 *  Page 6  | V                     |  |  Page 1  |                         ^ |
 *  --------+---+---+---+-...-+-----+  |  --------+-----+-----+-----+-...-+---+
 *  Page 7  | V                     |  |  Page 0  |                         ^ |
 *  --------+---+---+---+-...-+-----+  |  --------+-----+-----+-----+-...-+---+
 *  -----------------------------------+---------------------------------------
 *
 *  -----------------------------------+---------------------------------------
 *  Portrait Display:                  | Reverse Portrait Display:
 *  -----------+---------------------+ |  -----------+---------------------+
 *             |         Page        | |             |       Page          |
 *  -----------+---+---+---+-...-+---+ |  -----------+---+---+---+-...-+---+
 *  Column 0   | 0 | 1 | 2 |     | 7 | |  Column 127 | 7 | 6 | 5 |     | 0 |
 *  -----------+---+---+---+-...-+---+ |  -----------+---+---+---+-...-+---+
 *  Column 1   | >   >   >    >    > | |  Column 126 |                     |
 *  -----------+---+---+---+-...-+---+ |  -----------+---+---+---+-...-+---+
 *  Column 2   |                     | |  Column 125 |                     |
 *  -----------+---+---+---+-...-+---+ |  -----------+---+---+---+-...-+---+
 *  ...        |                     | |  ...        |                     |
 *  -----------+---+---+---+-...-+---+ |  -----------+---+---+---+-...-+---+
 *  Column 127 |                     | |  Column 0   | <   <   <    <    < |
 *  -----------+---+---+---+-...-+---+ |  -----------+---+---+---+-...-+---+
 *  -----------------------------------+----------------------------------------
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
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/spi/spi.h>
#include <nuttx/lcd/lcd.h>
#include <nuttx/lcd/ssd1306.h>

#include <arch/irq.h>

#include "ssd1306.h"

#ifdef CONFIG_LCD_SSD1306

/**************************************************************************************
 * Private Function Prototypes
 **************************************************************************************/

/* LCD Data Transfer Methods */

static int ssd1306_putrun(fb_coord_t row, fb_coord_t col,
                          FAR const uint8_t *buffer, size_t npixels);
static int ssd1306_getrun(fb_coord_t row, fb_coord_t col, FAR uint8_t *buffer,
                          size_t npixels);

/* LCD Configuration */

static int ssd1306_getvideoinfo(FAR struct lcd_dev_s *dev,
                                FAR struct fb_videoinfo_s *vinfo);
static int ssd1306_getplaneinfo(FAR struct lcd_dev_s *dev, unsigned int planeno,
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

static int  ssd1306_getpower(struct lcd_dev_s *dev);
static int  ssd1306_setpower(struct lcd_dev_s *dev, int power);
static int  ssd1306_getcontrast(struct lcd_dev_s *dev);
static int  ssd1306_setcontrast(struct lcd_dev_s *dev, unsigned int contrast);

static int  ssd1306_do_disponoff(struct ssd1306_dev_s *priv, bool on);
static int  ssd1306_configuredisplay(struct ssd1306_dev_s *priv);
static int  ssd1306_redrawfb(struct ssd1306_dev_s *priv);

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

static uint8_t g_runbuffer[SSD1306_DEV_ROWSIZE];

/* This structure describes the overall LCD video controller */

static const struct fb_videoinfo_s g_videoinfo =
{
  .fmt     = SSD1306_DEV_COLORFMT,  /* Color format: B&W */
  .xres    = SSD1306_DEV_XRES,      /* Horizontal resolution in pixel columns */
  .yres    = SSD1306_DEV_YRES,      /* Vertical resolution in pixel rows */
  .nplanes = 1,                     /* Number of color planes supported */
};

/* This is the standard, NuttX Plane information object */

static const struct lcd_planeinfo_s g_planeinfo =
{
  .putrun = ssd1306_putrun,             /* Put a run into LCD memory */
  .getrun = ssd1306_getrun,             /* Get a run from LCD memory */
  .buffer = (FAR uint8_t *)g_runbuffer, /* Run scratch buffer */
  .bpp    = SSD1306_DEV_BPP,            /* Bits-per-pixel */
};

/* This is the outside visible interface for the OLED driver */

static const struct lcd_dev_s g_oleddev_dev =
{
  /* LCD Configuration */

  .getvideoinfo = ssd1306_getvideoinfo,
  .getplaneinfo = ssd1306_getplaneinfo,

  /* LCD RGB Mapping -- Not supported */
  /* Cursor Controls -- Not supported */

  /* LCD Specific Controls */

  .getpower     = ssd1306_getpower,
  .setpower     = ssd1306_setpower,
  .getcontrast  = ssd1306_getcontrast,
  .setcontrast  = ssd1306_setcontrast,
};

/* This is the OLED driver instance (only a single device is supported for now) */

static struct ssd1306_dev_s g_oleddev;

/**************************************************************************************
 * Private Functions
 **************************************************************************************/

/**************************************************************************************
 * Name:  ssd1306_putrun
 *
 * Description:
 *   This method can be used to write a partial raster line to the LCD.
 *
 * Input Parameters:
 *   row     - Starting row to write to (range: 0 <= row < yres)
 *   col     - Starting column to write to (range: 0 <= col <= xres-npixels)
 *   buffer  - The buffer containing the run to be written to the LCD
 *   npixels - The number of pixels to write to the LCD
 *             (range: 0 < npixels <= xres-col)
 *
 **************************************************************************************/

#if defined(CONFIG_LCD_LANDSCAPE) || defined(CONFIG_LCD_RLANDSCAPE)
static int ssd1306_putrun(fb_coord_t row, fb_coord_t col, FAR const uint8_t *buffer,
                          size_t npixels)
{
  /* Because of this line of code, we will only be able to support a single UG device */

  FAR struct ssd1306_dev_s *priv = (FAR struct ssd1306_dev_s *)&g_oleddev;
  FAR uint8_t *fbptr;
  FAR uint8_t *ptr;
  uint8_t devcol;
  uint8_t fbmask;
  uint8_t page;
  uint8_t usrmask;
  int pixlen;
  uint8_t i;
  int ret;

  lcdinfo("row: %d col: %d npixels: %d\n", row, col, npixels);
  DEBUGASSERT(buffer);

  /* Clip the run to the display */

  pixlen = npixels;
  if ((unsigned int)col + (unsigned int)pixlen > (unsigned int)SSD1306_DEV_XRES)
    {
      pixlen = (int)SSD1306_DEV_XRES - (int)col;
    }

  /* Verify that some portion of the run remains on the display */

  if (pixlen <= 0 || row > SSD1306_DEV_YRES)
    {
      return OK;
    }

  /* Perform coordinate conversion for reverse landscape mode.
   * If the rows are reversed then rows are are a mirror reflection of
   * top to bottom.
   */

#ifdef SSD1306_DEV_REVERSEY
  row = (SSD1306_DEV_YRES-1) - row;
#endif

  /* If the column is switched then the start of the run is the mirror of
   * the end of the run.
   *
   *            col+pixlen-1
   *     col    |
   *  0  |      |                    XRES
   *  .  S>>>>>>E                    .
   *  .                    E<<<<<<S  .
   *                       |      |
   *                       |      `-(XRES-1)-col
   *                       ` (XRES-1)-col-(pixlen-1)
   */

#ifdef SSD1306_DEV_REVERSEX
  col  = (SSD1306_DEV_XRES-1) - col;
  col -= (pixlen - 1);
#endif

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
   *     D0   |   | X |   |   |     |     |
   *     D1   |   | X |   |   |     |     |
   *     D2   |   | X |   |   |     |     |
   *     D3   |   | X |   |   |     |     |
   *     D4   |   | X |   |   |     |     |
   *     D5   |   | X |   |   |     |     |
   *     D6   |   | X |   |   |     |     |
   *     D7   |   | X |   |   |     |     |
   *  --------+---+---+---+---+-...-+-----+
   *
   * So, in order to draw a white, horizontal line, at row 45. we
   * would have to modify all of the bytes in page 45/8 = 5.  We
   * would have to set bit 45%8 = 5 in every byte in the page.
   */

  fbmask  = 1 << (row & 7);
  fbptr   = &priv->fb[page * SSD1306_DEV_XRES + col];
#ifdef SSD1306_DEV_REVERSEX
  ptr     = fbptr + (pixlen - 1);
#else
  ptr     = fbptr;
#endif

#ifdef CONFIG_LCD_PACKEDMSFIRST
  usrmask = MS_BIT;
#else
  usrmask = LS_BIT;
#endif

  for (i = 0; i < pixlen; i++)
    {
      /* Set or clear the corresponding bit */

#ifdef SSD1306_DEV_REVERSEX
      if ((*buffer & usrmask) != 0)
        {
          *ptr-- |= fbmask;
        }
      else
        {
          *ptr-- &= ~fbmask;
        }
#else
      if ((*buffer & usrmask) != 0)
        {
          *ptr++ |= fbmask;
        }
      else
        {
          *ptr++ &= ~fbmask;
        }
#endif

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

  /* Offset the column position to account for smaller horizontal
   * display range.
   */

  devcol = col + SSD1306_DEV_XOFFSET;

  /* Lock and select device */

  ssd1306_select(priv, true);

  /* Select command transfer */

  ssd1306_cmddata(priv, true);

  /* Set the starting position for the run */
  /* Set the column address to the XOFFSET value */

  ret = ssd1306_sendbyte(priv, SSD1306_SETCOLL(devcol & 0x0f));
  if (ret < 0)
    {
      return ret;
    }

  ret = ssd1306_sendbyte(priv, SSD1306_SETCOLH(devcol >> 4));
  if (ret < 0)
    {
      return ret;
    }

  /* Set the page address */

  ret = ssd1306_sendbyte(priv, SSD1306_PAGEADDR(page));
  if (ret < 0)
    {
      return ret;
    }

  /* Select data transfer */

  ssd1306_cmddata(priv, false);

  /* Then transfer all of the data */

  ret = ssd1306_sendblk(priv, fbptr, pixlen);
  if (ret < 0)
    {
      return ret;
    }

  /* De-select and unlock the device */

  ssd1306_select(priv, false);
  return OK;
}
#else
#  error "Configuration not implemented"
#endif

/**************************************************************************************
 * Name:  ssd1306_getrun
 *
 * Description:
 *   This method can be used to read a partial raster line from the LCD.
 *
 * Description:
 *   This method can be used to write a partial raster line to the LCD.
 *
 *  row     - Starting row to read from (range: 0 <= row < yres)
 *  col     - Starting column to read read (range: 0 <= col <= xres-npixels)
 *  buffer  - The buffer in which to return the run read from the LCD
 *  npixels - The number of pixels to read from the LCD
 *            (range: 0 < npixels <= xres-col)
 *
 **************************************************************************************/

#if defined(CONFIG_LCD_LANDSCAPE) || defined(CONFIG_LCD_RLANDSCAPE)
static int ssd1306_getrun(fb_coord_t row, fb_coord_t col, FAR uint8_t *buffer,
                      size_t npixels)
{
  /* Because of this line of code, we will only be able to support a single UG device */

  FAR struct ssd1306_dev_s *priv = &g_oleddev;
  FAR uint8_t *fbptr;
  uint8_t page;
  uint8_t fbmask;
  uint8_t usrmask;
  int pixlen;
  uint8_t i;

  lcdinfo("row: %d col: %d npixels: %d\n", row, col, npixels);
  DEBUGASSERT(buffer);

  /* Clip the run to the display */

  pixlen = npixels;
  if ((unsigned int)col + (unsigned int)pixlen > (unsigned int)SSD1306_DEV_XRES)
    {
      pixlen = (int)SSD1306_DEV_XRES - (int)col;
    }

  /* Verify that some portion of the run is actually the display */

  if (pixlen <= 0 || row > SSD1306_DEV_YRES)
    {
      return -EINVAL;
    }

  /* Perform coordinate conversion for reverse landscape mode.
   * If the rows are reversed then rows are are a mirror reflection of
   * top to bottom.
   */

#ifdef SSD1306_DEV_REVERSEY
  row = (SSD1306_DEV_YRES-1) - row;
#endif

  /* If the column is switched then the start of the run is the mirror of
   * the end of the run.
   *
   *            col+pixlen-1
   *     col    |
   *  0  |      |                    XRES
   *  .  S>>>>>>E                    .
   *  .                    E<<<<<<S  .
   *                       |      |
   *                       |      `-(XRES-1)-col
   *                       ` (XRES-1)-col-(pixlen-1)
   */

#ifdef SSD1306_DEV_REVERSEX
  col  = (SSD1306_DEV_XRES-1) - col;
#endif

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
   *     D0   |   | X |   |   |     |     |
   *     D1   |   | X |   |   |     |     |
   *     D2   |   | X |   |   |     |     |
   *     D3   |   | X |   |   |     |     |
   *     D4   |   | X |   |   |     |     |
   *     D5   |   | X |   |   |     |     |
   *     D6   |   | X |   |   |     |     |
   *     D7   |   | X |   |   |     |     |
   *  --------+---+---+---+---+-...-+-----+
   *
   * So, in order to draw a white, horizontal line, at row 45. we
   * would have to modify all of the bytes in page 45/8 = 5.  We
   * would have to set bit 45%8 = 5 in every byte in the page.
   */

  fbmask  = 1 << (row & 7);
  fbptr   = &priv->fb[page * SSD1306_DEV_XRES + col];

#ifdef CONFIG_LCD_PACKEDMSFIRST
  usrmask = MS_BIT;
#else
  usrmask = LS_BIT;
#endif

  for (i = 0; i < pixlen; i++)
    {
      /* Set or clear the corresponding bit */

#ifdef SSD1306_DEV_REVERSEX
      uint8_t byte = *fbptr--;
#else
      uint8_t byte = *fbptr++;
#endif
      if ((byte & fbmask) != 0)
        {
          *buffer |= usrmask;
        }
      else
        {
          *buffer &= ~usrmask;
        }

      /* Inc/Decrement to the next destination pixel. */

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

  return OK;
}
#else
#  error "Configuration not implemented"
#endif

/**************************************************************************************
 * Name:  ssd1306_getvideoinfo
 *
 * Description:
 *   Get information about the LCD video controller configuration.
 *
 **************************************************************************************/

static int ssd1306_getvideoinfo(FAR struct lcd_dev_s *dev,
                                FAR struct fb_videoinfo_s *vinfo)
{
  DEBUGASSERT(dev && vinfo);
  lcdinfo("fmt: %d xres: %d yres: %d nplanes: %d\n",
          g_videoinfo.fmt, g_videoinfo.xres, g_videoinfo.yres, g_videoinfo.nplanes);
  memcpy(vinfo, &g_videoinfo, sizeof(struct fb_videoinfo_s));
  return OK;
}

/**************************************************************************************
 * Name:  ssd1306_getplaneinfo
 *
 * Description:
 *   Get information about the configuration of each LCD color plane.
 *
 **************************************************************************************/

static int ssd1306_getplaneinfo(FAR struct lcd_dev_s *dev, unsigned int planeno,
                                FAR struct lcd_planeinfo_s *pinfo)
{
  DEBUGASSERT(pinfo && planeno == 0);
  lcdinfo("planeno: %d bpp: %d\n", planeno, g_planeinfo.bpp);
  memcpy(pinfo, &g_planeinfo, sizeof(struct lcd_planeinfo_s));
  return OK;
}

/**************************************************************************************
 * Name:  ssd1306_getpower
 *
 * Description:
 *   Get the LCD panel power status (0: full off - CONFIG_LCD_MAXPOWER: full on. On
 *   backlit LCDs, this setting may correspond to the backlight setting.
 *
 **************************************************************************************/

static int ssd1306_getpower(FAR struct lcd_dev_s *dev)
{
  FAR struct ssd1306_dev_s *priv = (FAR struct ssd1306_dev_s *)dev;
  DEBUGASSERT(priv);

  lcdinfo("power: %s\n", priv->on ? "ON" : "OFF");
  return priv->on ? CONFIG_LCD_MAXPOWER : 0;
}

 /**************************************************************************************
 * Name:  ssd1306_do_disponoff
 *
 * Description:
 *   Enable/disable LCD panel power
 *
 **************************************************************************************/

static int ssd1306_do_disponoff(struct ssd1306_dev_s *priv, bool on)
{
  int ret;

  /* Lock and select device */

  ssd1306_select(priv, true);

  /* Select command transfer */

  ssd1306_cmddata(priv, true);

  /* Turn the display on/off */

  ret = ssd1306_sendbyte(priv, (on ? SSD1306_DISPON : SSD1306_DISPOFF));

  /* De-select and unlock the device */

  ssd1306_cmddata(priv, false);
  ssd1306_select(priv, false);

  return ret;
}

/**************************************************************************************
 * Name:  ssd1306_setpower
 *
 * Description:
 *   Enable/disable LCD panel power (0: full off - CONFIG_LCD_MAXPOWER: full on). On
 *   backlit LCDs, this setting may correspond to the backlight setting.
 *
 **************************************************************************************/

static int ssd1306_setpower(FAR struct lcd_dev_s *dev, int power)
{
  struct ssd1306_dev_s *priv = (struct ssd1306_dev_s *)dev;
  int ret;

  lcdinfo("power: %d [%d]\n", power, priv->on ? CONFIG_LCD_MAXPOWER : 0);
  DEBUGASSERT(priv && (unsigned)power <= CONFIG_LCD_MAXPOWER);

  if (power <= 0)
    {
      /* Turn the display off */

      ret = ssd1306_do_disponoff(priv, false);
      if (ret < 0)
        {
          return ret;
        }

      priv->on = false;

      /* Try turn off power completely */

      if (priv->board_priv && priv->board_priv->set_vcc)
        {
          /* Do power off. */

          if (priv->board_priv->set_vcc(false))
            {
              /* Display is completely powered off, not configured anymore. */

              priv->is_conf = false;
            }
        }
    }
  else
    {
      if (priv->board_priv && priv->board_priv->set_vcc)
        {
          /* Do power on. */

          (void)priv->board_priv->set_vcc(true);
        }

      if (!priv->is_conf)
        {
          /* Configure display and turn the display on */

          ret = ssd1306_configuredisplay(priv);
          if (ret < 0)
            {
              return ret;
            }

          /* Draw the framebuffer */

          ret = ssd1306_redrawfb(priv);
        }
      else
        {
          /* Turn the display on */

          ret = ssd1306_do_disponoff(priv, true);
        }

      if (ret < 0)
        {
          return ret;
        }

      priv->on = true;
    }

  return OK;
}

/**************************************************************************************
 * Name:  ssd1306_getcontrast
 *
 * Description:
 *   Get the current contrast setting (0-CONFIG_LCD_MAXCONTRAST).
 *
 **************************************************************************************/

static int ssd1306_getcontrast(struct lcd_dev_s *dev)
{
  struct ssd1306_dev_s *priv = (struct ssd1306_dev_s *)dev;
  DEBUGASSERT(priv);

  lcdinfo("contrast: %d\n", priv->contrast);
  return priv->contrast;
}

/**************************************************************************************
 * Name:  ssd1306_setcontrast
 *
 * Description:
 *   Set LCD panel contrast (0-CONFIG_LCD_MAXCONTRAST).
 *
 **************************************************************************************/

static int ssd1306_setcontrast(struct lcd_dev_s *dev, unsigned int contrast)
{
  struct ssd1306_dev_s *priv = (struct ssd1306_dev_s *)dev;
  unsigned int scaled;
  int ret;

  lcdinfo("contrast: %d\n", contrast);
  DEBUGASSERT(priv);

  /* Verify the contrast value */

#ifdef CONFIG_DEBUG_FEATURES
  if (contrast > CONFIG_LCD_MAXCONTRAST)
    {
      return -EINVAL;
    }
#endif

  /* Scale contrast:  newcontrast = 255 * contrast / CONFIG_LCD_MAXCONTRAST
   * Where contrast is in the range {1,255}
   */

#if CONFIG_LCD_MAXCONTRAST != 255
  scaled = ((contrast << 8) - 1) / CONFIG_LCD_MAXCONTRAST;
#else
  scaled = contrast;
#endif

  /* Lock and select device */

  ssd1306_select(priv, true);

  /* Select command transfer */

  ssd1306_cmddata(priv, true);

  /* Set the contrast */

  ret = ssd1306_sendbyte(priv, SSD1306_CONTRAST_MODE);    /* Set contrast control register */
  if (ret < 0)
    {
      return ret;
    }

  ret = ssd1306_sendbyte(priv, SSD1306_CONTRAST(scaled)); /* Data 1: Set 1 of 256 contrast steps */
  if (ret < 0)
    {
      return ret;
    }

  priv->contrast = contrast;

  /* De-select and unlock the device */

  ssd1306_select(priv, false);
  return OK;
}

/**************************************************************************************
 * Name:  ssd1306_configuredisplay
 *
 * Description:
 *   Setup LCD display.
 *
 **************************************************************************************/

static int ssd1306_configuredisplay(struct ssd1306_dev_s *priv)
{
  int ret;

  /* Lock and select device */

  ssd1306_select(priv, true);

  /* Select command transfer */

  ssd1306_cmddata(priv, true);

  /* Configure OLED SPI or I/O, must be delayed 1-10ms */

  up_mdelay(5);

  /* Configure the device */

#ifdef IS_SSD1309

  ret = ssd1306_sendbyte(priv, SSD1309_PROTOFF);         /* Unlock driver IC */
  if (ret < 0)
    {
      return ret;
    }

  ret = ssd1306_sendbyte(priv, SSD1306_DISPOFF);         /* Display off 0xae */
  if (ret < 0)
    {
      return ret;
    }

  ret = ssd1306_sendbyte(priv, SSD1309_SETMEMORY);       /* Set page addressing mode: 0x0, 0x01 or 0x02 */
  if (ret < 0)
    {
      return ret;
    }

  ret = ssd1306_sendbyte(priv, SSD1309_MEMADDR(0x02));
  if (ret < 0)
    {
      return ret;
    }

  ret = ssd1306_sendbyte(priv, SSD1306_SETCOLL(0));      /* Set lower column address 0x00 */
  if (ret < 0)
    {
      return ret;
    }

  ret = ssd1306_sendbyte(priv, SSD1306_SETCOLH(0));      /* Set higher column address 0x10 */
  if (ret < 0)
    {
      return ret;
    }

  ret = ssd1306_sendbyte(priv, SSD1306_STARTLINE(0));    /* Set display start line 0x40 */
  if (ret < 0)
    {
      return ret;
    }

  ret = ssd1306_sendbyte(priv, SSD1306_PAGEADDR(0));     /* Set page address (Can ignore) */
  if (ret < 0)
    {
      return ret;
    }

  ret = ssd1306_sendbyte(priv, SSD1306_CONTRAST_MODE);   /* Contrast control 0x81 */
  if (ret < 0)
    {
      return ret;
    }

  ret = ssd1306_sendbyte(priv ,SSD1306_CONTRAST(SSD1309_DEV_CONTRAST));  /* Default contrast 0xff */
  if (ret < 0)
    {
      return ret;
    }

  ret = ssd1306_sendbyte(priv, SSD1306_REMAPPLEFT);      /* Set segment remap left 95 to 0 | 0xa1 */
  if (ret < 0)
    {
      return ret;
    }

  ret = ssd1306_sendbyte(priv, SSD1306_EDISPOFF);        /* Normal display off 0xa4 (Can ignore) */
  if (ret < 0)
    {
      return ret;
    }

  ret = ssd1306_sendbyte(priv, SSD1306_NORMAL);          /* Normal (un-reversed) display mode 0xa6 */
  if (ret < 0)
    {
      return ret;
    }

  ret = ssd1306_sendbyte(priv, SSD1306_MRATIO_MODE);     /* Multiplex ratio 0xa8 */
  if (ret < 0)
    {
      return ret;
    }

  ret = ssd1306_sendbyte(priv, SSD1306_MRATIO(SSD1306_DEV_DUTY));  /* Duty = 1/64 or 1/32 */
  if (ret < 0)
    {
      return ret;
    }

  ret = ssd1306_sendbyte(priv, SSD1306_SCANFROMCOM0);    /* Com scan direction: Scan from COM[0] to COM[n-1] */
  if (ret < 0)
    {
      return ret;
    }

  ret = ssd1306_sendbyte(priv, SSD1306_DISPOFFS_MODE);   /* Set display offset 0xd3 */
  if (ret < 0)
    {
      return ret;
    }

  ret = ssd1306_sendbyte(priv, SSD1306_DISPOFFS(0));
  if (ret < 0)
    {
      return ret;
    }

  ret = ssd1306_sendbyte(priv, SSD1306_CLKDIV_SET);      /* Set clock divider 0xd5 */
  if (ret < 0)
    {
      return ret;
    }

  ret = ssd1306_sendbyte(priv, SSD1306_CLKDIV(7,0));     /* 0x70 */
  if (ret < 0)
    {
      return ret;
    }

  ret = ssd1306_sendbyte(priv, SSD1306_CHRGPER_SET);     /* Set pre-charge period 0xd9 */
  if (ret < 0)
    {
      return ret;
    }

  ret = ssd1306_sendbyte(priv, SSD1306_CHRGPER(0x0f,0x0a)); /* 0xfa: Fh cycles for discharge and Ah cycles for pre-charge */
  if (ret < 0)
    {
      return ret;
    }

  ret = ssd1306_sendbyte(priv, SSD1306_CMNPAD_CONFIG);   /* Set common pads / set com pins hardware configuration 0xda */
  if (ret < 0)
    {
      return ret;
    }

  ret = ssd1306_sendbyte(priv, SSD1306_CMNPAD(SSD1306_DEV_CMNPAD)); /* 0x12 or 0x02 */
  if (ret < 0)
    {
      return ret;
    }

  ret = ssd1306_sendbyte(priv, SSD1306_VCOM_SET);        /* set vcomh 0xdb */
  if (ret < 0)
    {
      return ret;
    }

  ret = ssd1306_sendbyte(priv, SSD1306_VCOM(0x3C));
  if (ret < 0)
    {
      return ret;
    }

#else

  ret = ssd1306_sendbyte(priv, SSD1306_DISPOFF);          /* Display off 0xae */
  if (ret < 0)
    {
      return ret;
    }

  ret = ssd1306_sendbyte(priv, SSD1306_SETCOLL(0));       /* Set lower column address 0x00 */
  if (ret < 0)
    {
      return ret;
    }

  ret = ssd1306_sendbyte(priv, SSD1306_SETCOLH(0));       /* Set higher column address 0x10 */
  if (ret < 0)
    {
      return ret;
    }

  ret = ssd1306_sendbyte(priv, SSD1306_STARTLINE(0));     /* Set display start line 0x40 */
  if (ret < 0)
    {
      return ret;
    }

#if 0
  ret = ssd1306_sendbyte(priv, SSD1306_PAGEADDR(0));    /* Set page address  (Can ignore) */
  if (ret < 0)
    {
      return ret;
    }
#endif

  ret = ssd1306_sendbyte(priv, SSD1306_CONTRAST_MODE);    /* Contrast control 0x81 */
  if (ret < 0)
    {
      return ret;
    }

  ret = ssd1306_sendbyte(priv, SSD1306_CONTRAST(SSD1306_DEV_CONTRAST));  /* Default contrast 0xCF */
  if (ret < 0)
    {
      return ret;
    }

  ret = ssd1306_sendbyte(priv, SSD1306_REMAPPLEFT);       /* Set segment remap left 95 to 0 | 0xa1 */
  if (ret < 0)
    {
      return ret;
    }

#if 0
  ret = ssd1306_sendbyte(priv, SSD1306_EDISPOFF);       /* Normal display off 0xa4 (Can ignore) */
  if (ret < 0)
    {
      return ret;
    }
#endif

  ret = ssd1306_sendbyte(priv, SSD1306_NORMAL);           /* Normal (un-reversed) display mode 0xa6 */
  if (ret < 0)
    {
      return ret;
    }

  ret = ssd1306_sendbyte(priv, SSD1306_MRATIO_MODE);      /* Multiplex ratio 0xa8 */
  if (ret < 0)
    {
      return ret;
    }

  ret = ssd1306_sendbyte(priv, SSD1306_MRATIO(SSD1306_DEV_DUTY));  /* Duty = 1/64 or 1/32 */
  if (ret < 0)
    {
      return ret;
    }

#if 0
  ret = ssd1306_sendbyte(priv, SSD1306_SCANTOCOM0);     /* Com scan direction: Scan from COM[n-1] to COM[0] (Can ignore) */
  if (ret < 0)
    {
      return ret;
    }
#endif

  ret = ssd1306_sendbyte(priv, SSD1306_DISPOFFS_MODE);    /* Set display offset 0xd3 */
  if (ret < 0)
    {
      return ret;
    }

  ret = ssd1306_sendbyte(priv, SSD1306_DISPOFFS(0));
  if (ret < 0)
    {
      return ret;
    }

  ret = ssd1306_sendbyte(priv, SSD1306_CLKDIV_SET);       /* Set clock divider 0xd5 */
  if (ret < 0)
    {
      return ret;
    }

  ret = ssd1306_sendbyte(priv, SSD1306_CLKDIV(8, 0));     /* 0x80 */
  if (ret < 0)
    {
      return ret;
    }

  ret = ssd1306_sendbyte(priv, SSD1306_CHRGPER_SET);      /* Set pre-charge period 0xd9 */
  if (ret < 0)
    {
      return ret;
    }

  ret = ssd1306_sendbyte(priv, SSD1306_CHRGPER(0x0f, 1)); /* 0xf1 or 0x22 Enhanced mode */
  if (ret < 0)
    {
      return ret;
    }

  ret = ssd1306_sendbyte(priv, SSD1306_CMNPAD_CONFIG);    /* Set common pads / set com pins hardware configuration 0xda */
  if (ret < 0)
    {
      return ret;
    }

  ret = ssd1306_sendbyte(priv, SSD1306_CMNPAD(SSD1306_DEV_CMNPAD)); /* 0x12 or 0x02 */
  if (ret < 0)
    {
      return ret;
    }

  ret = ssd1306_sendbyte(priv, SSD1306_VCOM_SET);         /* set vcomh 0xdb */
  if (ret < 0)
    {
      return ret;
    }

  ret = ssd1306_sendbyte(priv, SSD1306_VCOM(0x40));
  if (ret < 0)
    {
      return ret;
    }

  ret = ssd1306_sendbyte(priv, SSD1306_CHRPUMP_SET);      /* Set Charge Pump enable/disable 0x8d ssd1306 */
  if (ret < 0)
    {
      return ret;
    }

  ret = ssd1306_sendbyte(priv, SSD1306_CHRPUMP_ON);       /* 0x14 close 0x10 */
  if (ret < 0)
    {
      return ret;
    }

#if 0
  ret = ssd1306_sendbyte(priv, SSD1306_DCDC_MODE);      /* DC/DC control mode: on (SSD1306 Not supported) */
  if (ret < 0)
    {
      return ret;
    }

  ret = ssd1306_sendbyte(priv, SSD1306_DCDC_ON);
  if (ret < 0)
    {
      return ret;
    }
#endif
#endif

  ret = ssd1306_sendbyte(priv, SSD1306_DISPON);           /* Display ON 0xaf */
  if (ret < 0)
    {
      return ret;
    }

  /* De-select and unlock the device */

  ssd1306_select(priv, false);

  up_mdelay(100);

  priv->is_conf = true;
  return OK;
}

/**************************************************************************************
 * Name:  ssd1306_redrawfb
 *
 * Description:
 *   Redraw full framebuffer to display
 *
 * Input Parameters:
 *   priv   - Reference to private driver structure
 *
 * Assumptions:
 *   Caller has selected the OLED section.
 *
 **************************************************************************************/

static int ssd1306_redrawfb(struct ssd1306_dev_s *priv)
{
  unsigned int page;
  int ret;

  /* Lock and select device */

  ssd1306_select(priv, true);

  /* Visit each page */

  for (page = 0; page < SSD1306_DEV_PAGES; page++)
    {
      /* Select command transfer */

      ssd1306_cmddata(priv, true);

      /* Set the column address to the XOFFSET value */

      ret = ssd1306_sendbyte(priv, SSD1306_SETCOLL(SSD1306_DEV_XOFFSET));
      if (ret < 0)
        {
          return ret;
        }

      ret = ssd1306_sendbyte(priv, SSD1306_SETCOLH(0));
      if (ret < 0)
        {
          return ret;
        }

      /* Set the page address */

      ret = ssd1306_sendbyte(priv, SSD1306_PAGEADDR(page));
      if (ret < 0)
        {
          return ret;
        }

      /* Select data transfer */

      ssd1306_cmddata(priv, false);

      /* Transfer one page of the selected color */

      ret = ssd1306_sendblk(priv, &priv->fb[page * SSD1306_DEV_XRES],
                            SSD1306_DEV_XRES);
      if (ret < 0)
        {
          return ret;
        }
    }

  /* De-select and unlock the device */

  ssd1306_select(priv, false);
  return OK;
}

/**************************************************************************************
 * Public Functions
 **************************************************************************************/

/**************************************************************************************
 * Name:  ssd1306_initialize
 *
 * Description:
 *   Initialize the UG-2864HSWEG01 video hardware.  The initial state of the
 *   OLED is fully initialized, display memory cleared, and the OLED ready
 *   to use, but with the power setting at 0 (full off == sleep mode).
 *
 * Input Parameters:
 *
 *   spi - A reference to the SPI driver instance.
 *   devno - A value in the range of 0 through CONFIG_SSD1306_NINTERFACES-1.
 *     This allows support for multiple OLED devices.
 *
 * Returned Value:
 *
 *   On success, this function returns a reference to the LCD object for
 *   the specified OLED.  NULL is returned on any failure.
 *
 **************************************************************************************/

#ifdef CONFIG_LCD_SSD1306_SPI
FAR struct lcd_dev_s *ssd1306_initialize(FAR struct spi_dev_s *dev,
                                         FAR const struct ssd1306_priv_s *board_priv,
                                         unsigned int devno)
#else
FAR struct lcd_dev_s *ssd1306_initialize(FAR struct i2c_master_s *dev,
                                         FAR const struct ssd1306_priv_s *board_priv,
                                         unsigned int devno)
#endif
{
  FAR struct ssd1306_dev_s *priv = &g_oleddev;

  priv->dev = g_oleddev_dev;

  DEBUGASSERT(dev && devno == 0);

  priv->on = false;
  priv->is_conf = false;

  /* Register board specific functions */

  priv->board_priv = board_priv;

#ifdef CONFIG_LCD_SSD1306_SPI
  priv->spi = dev;

  /* Configure the SPI */

  ssd1306_configspi(priv->spi);

#else
  /* Remember the I2C configuration */

  priv->i2c  = dev;
  priv->addr = CONFIG_SSD1306_I2CADDR;
#endif

  /* Initialize the framebuffer */

  memset(priv->fb, SSD1306_Y1_BLACK & 1 ? 0xff : 0x00, SSD1306_DEV_FBSIZE);

  /* Power on and configure display */

  ssd1306_setpower(&priv->dev, true);

  return &priv->dev;
}

/**************************************************************************************
 * Name:  ssd1306_fill
 *
 * Description:
 *   This non-standard method can be used to clear the entire display by writing one
 *   color to the display.  This is much faster than writing a series of runs.
 *
 * Input Parameters:
 *   priv   - Reference to private driver structure
 *
 * Assumptions:
 *   Caller has selected the OLED section.
 *
 **************************************************************************************/

int ssd1306_fill(FAR struct lcd_dev_s *dev, uint8_t color)
{
  FAR struct ssd1306_dev_s *priv = (struct ssd1306_dev_s *)dev;

  /* Make an 8-bit version of the selected color */

  if (color & 1)
    {
      color = 0xff;
    }
  else
    {
      color = 0;
    }

  /* Initialize the framebuffer */

  memset(priv->fb, color, SSD1306_DEV_FBSIZE);

  /* Draw the framebuffer */

  return ssd1306_redrawfb(priv);
}

#endif /* CONFIG_LCD_SSD1306 */
