/****************************************************************************
 * drivers/lcd/lpm013m091a.c
 *
 * Driver for LPM013M091A LCD based on ili9341.c.
 *
 *   Copyright (C) 2014 Marco Krahl. All rights reserved.
 *   Author: Marco Krahl <ocram.lhark@gmail.com>
 *
 *   Copyright 2018 Sony Semiconductor Solutions Corporation
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/lcd/lcd.h>
#include <nuttx/lcd/lpm013m091a.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Display resolution */

#define LPM013M091A_XRES        320
#define LPM013M091A_YRES        300

/* TODO: Stride should be configurable by LCD orientation */

#define LPM013M091A_STRIDE      LPM013M091A_XRES

/* Dolor depth and format */

#define LPM013M091A_BPP           16
#define LPM013M091A_COLORFMT      FB_FMT_RGB16_565

/****************************************************************************
 * Private types
 ****************************************************************************/

struct lpm013m091a_dev_s
{
  /* Publicly visible device structure */

  struct lcd_dev_s dev;

  /* Private lcd-specific information follows */

  struct lpm013m091a_lcd_s *lcd;

  uint8_t power;                  /* Current power setting */
};

/****************************************************************************
 * Private Function Protototypes
 ****************************************************************************/

static void lpm013m091a_selectarea(FAR struct lpm013m091a_lcd_s *lcd,
                                   uint16_t x0, int16_t y0,
                                   uint16_t x1, int16_t y1);
static int lpm013m091a_hwinitialize(FAR struct lpm013m091a_dev_s *dev);

/* lcd data transfer methods */

static int lpm013m091a_putrun(fb_coord_t row, fb_coord_t col,
                              FAR const uint8_t *buffer, size_t npixels);
#ifndef CONFIG_LCD_NOGETRUN
static int lpm013m091a_getrun(fb_coord_t row, fb_coord_t col,
                              FAR uint8_t *buffer,
                              size_t npixels);
#endif

/* lcd configuration */

static int lpm013m091a_getvideoinfo(FAR struct lcd_dev_s *dev,
                                    FAR struct fb_videoinfo_s *vinfo);
static int lpm013m091a_getplaneinfo(FAR struct lcd_dev_s *dev,
                                    unsigned int planeno,
                                    FAR struct lcd_planeinfo_s *pinfo);

/* lcd specific controls */

static int lpm013m091a_getpower(FAR struct lcd_dev_s *dev);
static int lpm013m091a_setpower(FAR struct lcd_dev_s *dev, int power);
static int lpm013m091a_getcontrast(FAR struct lcd_dev_s *dev);
static int lpm013m091a_setcontrast(FAR struct lcd_dev_s *dev,
                                   unsigned int contrast);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint16_t g_runbuffer[LPM013M091A_STRIDE];

/* This structure describes the overall lcd video controller */

static const struct fb_videoinfo_s g_videoinfo =
{
  .fmt = LPM013M091A_COLORFMT,            /* Color format: rgb16-565: rrrr rggg gggb bbbb */
  .xres = LPM013M091A_XRES,               /* Horizontal resolution in pixel columns */
  .yres = LPM013M091A_YRES,               /* Vertical resolution in pixel rows */
  .nplanes = 1,                           /* Number of color planes supported */
};

/* This is the standard, nuttx plane information object */

static const struct lcd_planeinfo_s g_planeinfo =
{
  .putrun = lpm013m091a_putrun,           /* Put a run into lcd memory */
#ifndef CONFIG_LCD_NOGETRUN
  .getrun = lpm013m091a_getrun,           /* Get a run from lcd memory */
#endif
  .buffer = (uint8_t *) g_runbuffer,      /* Run scratch buffer */
  .bpp = LPM013M091A_BPP,                 /* Bits-per-pixel */
};

static struct lpm013m091a_dev_s g_lpm013m091a_dev =
{
  .dev =
    {
      /* lcd configuration */

      .getvideoinfo = lpm013m091a_getvideoinfo,
      .getplaneinfo = lpm013m091a_getplaneinfo,

      /* lcd specific controls */

      .getpower = lpm013m091a_getpower,
      .setpower = lpm013m091a_setpower,
      .getcontrast = lpm013m091a_getcontrast,
      .setcontrast = lpm013m091a_setcontrast,
    },
  .lcd = 0,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  lpm013m091a_selectarea
 *
 * Description:
 *   Select the active area for displaying pixel
 *
 * Parameter:
 *   lcd       - Reference to private driver structure
 *   x0        - Start x position
 *   y0        - Start y position
 *   x1        - End x position
 *   y1        - End y position
 *
 ****************************************************************************/

static void lpm013m091a_selectarea(FAR struct lpm013m091a_lcd_s *lcd,
                                   uint16_t x0, int16_t y0,
                                   uint16_t x1, int16_t y1)
{
  lcd->sendcmd(lcd, LPM013M091A_CASET);
  lcd->sendparam(lcd, x0 >> 8);
  lcd->sendparam(lcd, x0 & 0xff);
  lcd->sendparam(lcd, x1 >> 8);
  lcd->sendparam(lcd, x1 & 0xff);

  lcd->sendcmd(lcd, LPM013M091A_PASET);
  lcd->sendparam(lcd, y0 >> 8);
  lcd->sendparam(lcd, y0 & 0xff);
  lcd->sendparam(lcd, y1 >> 8);
  lcd->sendparam(lcd, y1 & 0xff);
}

/****************************************************************************
 * Name:  lpm013m091a_hwinitialize
 *
 * Description:
 *   Initialize and configure the LPM013M091A LCD driver hardware.
 *
 * Parameter:
 *   dev - A reference to the driver specific structure
 *
 * Returned Value:
 *
 *   On success - OK
 *   On error - EINVAL
 *
 ****************************************************************************/

static int lpm013m091a_hwinitialize(FAR struct lpm013m091a_dev_s *dev)
{
  FAR struct lpm013m091a_lcd_s *lcd = dev->lcd;

  /* Soft reset */

  lcd->sendcmd(lcd, LPM013M091A_SWRESET);
  up_mdelay(10);

  /* Analog mode */

  lcd->sendcmd(lcd, 0xb3);
  lcd->sendparam(lcd, 0x02);

  /* Set Display Mode */

  lcd->sendcmd(lcd, 0xbb);
  lcd->sendparam(lcd, 0x10);

  /* SPI GRAM access enable */

  lcd->sendcmd(lcd, 0xf3);
  lcd->sendparam(lcd, 0x02);

  /* Bright Level Max */

  lcd->sendcmd(lcd, 0x51);
  lcd->sendparam(lcd, 0xff);

  /* Backlight ON */

  lcd->sendcmd(lcd, 0x53);
  lcd->sendparam(lcd, 0x24);

  /* Frame rate 60Hz */

  lcd->sendcmd(lcd, 0xff);
  lcd->sendparam(lcd, 0x24);
  lcd->sendcmd(lcd, 0xd8);
  lcd->sendparam(lcd, 0x41);
  lcd->sendcmd(lcd, 0xd9);
  lcd->sendparam(lcd, 0x1e);

  lcd->sendcmd(lcd, 0xff);
  lcd->sendparam(lcd, 0x10);

  /* Set the color format (18bit:0x06, 16bit:0x05) */

  lcd->sendcmd(lcd, LPM013M091A_PIXFMT);
  lcd->sendparam(lcd, 0x05);

  /* Sleep out */

  lcd->sendcmd(lcd, LPM013M091A_SLPOUT);
  up_mdelay(10);

  /* Display on */

  lcd->sendcmd(lcd, LPM013M091A_DISPON);
  up_mdelay(120);

  return OK;
}

/****************************************************************************
 * Name:  lpm013m091a_putrun
 *
 * Description:
 *   Write a partial raster line to the LCD.
 *
 * Parameters:
 *   devno   - Number of lcd device
 *   row     - Starting row to write to (range: 0 <= row < yres)
 *   col     - Starting column to write to (range: 0 <= col <= xres-npixels)
 *   buffer  - The buffer containing the run to be written to the LCD
 *   npixels - The number of pixels to write to the
 *             (range: 0 < npixels <= xres-col)
 *
 * Returned Value:
 *
 *   On success - OK
 *   On error   - -EINVAL
 *
 ****************************************************************************/

static int lpm013m091a_putrun(fb_coord_t row, fb_coord_t col,
                              FAR const uint8_t *buffer, size_t npixels)
{
  FAR struct lpm013m091a_dev_s *dev = (FAR struct lpm013m091a_dev_s *)
                                       &g_lpm013m091a_dev;
  FAR struct lpm013m091a_lcd_s *lcd = dev->lcd;
  FAR const uint16_t *src = (FAR const uint16_t *)buffer;

  DEBUGASSERT(buffer && ((uintptr_t)buffer & 1) == 0);

  /* Check if position outside of area */

  if (col + npixels > LPM013M091A_XRES || row > LPM013M091A_YRES)
    {
      return -EINVAL;
    }

  /* Select lcd driver */

  lcd->select(lcd);

  /* Select column and area similar to the partial raster line */

  lpm013m091a_selectarea(lcd, col, row, col + npixels - 1, row);

  /* Send memory write cmd */

  lcd->sendcmd(lcd, LPM013M091A_RAMWR);

  /* Send pixel to gram */

  lcd->sendgram(lcd, src, npixels);

  /* Deselect the lcd driver */

  lcd->deselect(lcd);

  return OK;
}

/****************************************************************************
 * Name:  lpm013m091a_getrun
 *
 * Description:
 *   Read a partial raster line from the LCD.
 *
 * Parameter:
 *   devno   - Number of the lcd device
 *   row     - Starting row to read from (range: 0 <= row < yres)
 *   col     - Starting column to read read (range: 0 <= col <= xres-npixels)
 *   buffer  - The buffer in which to return the run read from the LCD
 *   npixels - The number of pixels to read from the LCD
 *            (range: 0 < npixels <= xres-col)
 *
 * Returned Value:
 *
 *   On success - OK
 *   On error   - -EINVAL
 *
 ****************************************************************************/

#ifndef CONFIG_LCD_NOGETRUN
int lpm013m091a_getrun(fb_coord_t row, fb_coord_t col, FAR uint8_t * buffer,
                       size_t npixels)
{
  lcderr("getrun is not supported for now.\n");
  return -ENOSYS;
}
#endif

/****************************************************************************
 * Name:  lpm013m091a_getvideoinfo
 *
 * Description:
 *   Get information about the LCD video controller configuration.
 *
 * Parameter:
 *   dev - A reference to the driver specific structure
 *   vinfo - A reference to the videoinfo structure
 *
 * Returned Value:
 *
 *  On success - OK
 *  On error   - -EINVAL
 *
 ****************************************************************************/

static int lpm013m091a_getvideoinfo(FAR struct lcd_dev_s *dev,
                                    FAR struct fb_videoinfo_s *vinfo)
{
  if (dev && vinfo)
    {
      memcpy(vinfo, &g_videoinfo, sizeof(struct fb_videoinfo_s));

      lcdinfo("fmt: %d xres: %d yres: %d nplanes: %d\n",
              vinfo->fmt, vinfo->xres, vinfo->yres, vinfo->nplanes);

      return OK;
    }

  return -EINVAL;
}

/****************************************************************************
 * Name:  lpm013m091a_getplaneinfo
 *
 * Description:
 *   Get information about the configuration of each LCD color plane.
 *
 * Parameter:
 *   dev     - A reference to the driver specific structure
 *   planeno - The plane number
 *   pinfo   - A reference to the planeinfo structure
 *
 * Returned Value:
 *
 *  On success - OK
 *  On error   - -EINVAL
 *
 ****************************************************************************/

static int lpm013m091a_getplaneinfo(FAR struct lcd_dev_s *dev,
                                    unsigned int planeno,
                                    FAR struct lcd_planeinfo_s *pinfo)
{
  if (dev && pinfo && planeno == 0)
    {
      memcpy(pinfo, &g_planeinfo, sizeof(struct lcd_planeinfo_s));

      lcdinfo("planeno: %d bpp: %d\n", planeno, pinfo->bpp);

      return OK;
    }

  return -EINVAL;
}

/****************************************************************************
 * Name:  lpm013m091a_getpower
 *
 * Description:
 *   Get the LCD panel power status
 *   0: full off - CONFIG_LCD_MAXPOWER: full on.
 *   On backlit LCDs, this setting may correspond to the backlight setting.
 *
 * Parameter:
 *   dev     - A reference to the driver specific structure
 *
 * Returned Value:
 *
 *  On success - OK
 *  On error   - -EINVAL
 *
 ****************************************************************************/

static int lpm013m091a_getpower(FAR struct lcd_dev_s *dev)
{
  FAR struct lpm013m091a_dev_s *priv = (FAR struct lpm013m091a_dev_s *)dev;

  lcdinfo("%d\n", priv->power);
  return priv->power;
}

/****************************************************************************
 * Name:  lpm013m091a_setpower
 *
 * Description:
 *   Enable/disable LCD panel power
 *  (0: full off - CONFIG_LCD_MAXPOWER: full on).
 *   On backlight LCDs, this setting may correspond to the backlight setting.
 *
 * Parameter:
 *   dev   - A reference to the driver specific structure
 *   power - Value of the power
 *
 * Returned Value:
 *
 *  On success - OK
 *  On error   - -EINVAL
 *
 ****************************************************************************/

static int lpm013m091a_setpower(FAR struct lcd_dev_s *dev, int power)
{
  FAR struct lpm013m091a_dev_s *priv = (FAR struct lpm013m091a_dev_s *)dev;
  FAR struct lpm013m091a_lcd_s *lcd = priv->lcd;

  if (!dev)
    {
      return -EINVAL;
    }

  lcdinfo("%d\n", power);

  lcd->select(lcd);

  if (power > 0)
    {
      lcd->backlight(lcd, power);

      lcd->sendcmd(lcd, LPM013M091A_DISPON);
      up_mdelay(120);
    }
  else
    {
      lcd->sendcmd(lcd, LPM013M091A_DISPOFF);
    }

  lcd->deselect(lcd);

  priv->power = power;

  return OK;
}

/****************************************************************************
 * Name:  ili9340_getcontrast
 *
 * Description:
 *   Get the current contrast setting (0-CONFIG_LCD_MAXCONTRAST).
 *
 * Parameter:
 *   dev   - A reference to the lcd driver structure
 *
 * Returned Value:
 *
 *  On success - current contrast value
 *  On error   - -ENOSYS, not supported by the ili9340.
 *
 ****************************************************************************/

static int lpm013m091a_getcontrast(FAR struct lcd_dev_s *dev)
{
  lcdinfo("Not implemented\n");
  return -ENOSYS;
}

/****************************************************************************
 * Name:  ili9340_setcontrast
 *
 * Description:
 *   Set LCD panel contrast (0-CONFIG_LCD_MAXCONTRAST).
 *
 * Parameter:
 *   dev   - A reference to the lcd driver structure
 *
 * Returned Value:
 *
 *  On success - OK
 *  On error   - -ENOSYS, not supported by the ili9340.
 *
 ****************************************************************************/

static int lpm013m091a_setcontrast(FAR struct lcd_dev_s *dev,
                                   unsigned int contrast)
{
  lcdinfo("Not implemented\n");
  return -ENOSYS;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Initialize LCD
 ****************************************************************************/

FAR struct lcd_dev_s *
  lpm013m091a_initialize(FAR struct lpm013m091a_lcd_s *lcd, int devno)
{
  FAR struct lpm013m091a_dev_s *priv = &g_lpm013m091a_dev;

  if (lcd && devno == 0)
    {
      if (!priv->lcd)
        {
          FAR struct lcd_dev_s *dev = &priv->dev;
          int   ret;

          /* Initialize internal structure */

          dev->getvideoinfo = lpm013m091a_getvideoinfo;
          dev->getplaneinfo = lpm013m091a_getplaneinfo;
          dev->getpower     = lpm013m091a_getpower;
          dev->setpower     = lpm013m091a_setpower;
          dev->getcontrast  = lpm013m091a_getcontrast;
          dev->setcontrast  = lpm013m091a_setcontrast;
          priv->lcd         = lcd;

          /* Initialize the LCD driver */

          ret = lpm013m091a_hwinitialize(priv);

          if (ret == OK)
            {
              return &priv->dev;
            }

          errno = EINVAL;
        }
    }

  return NULL;
}
