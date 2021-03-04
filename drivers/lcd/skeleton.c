/****************************************************************************
 * drivers/lcd/skeleton.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
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
#include <nuttx/spi/spi.h>
#include <nuttx/lcd/lcd.h>

#include "up_arch.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Verify that all configuration requirements have been met */

/* Debug ********************************************************************/

/* Define the following to enable register-level debug output */

#undef CONFIG_LCD_SKELDEBUG

/* Verbose debug must also be enabled */

#ifndef CONFIG_DEBUG_FEATURES
#  undef CONFIG_DEBUG_INFO
#  undef CONFIG_DEBUG_GRAPHICS
#endif

#ifndef CONFIG_DEBUG_INFO
#  undef CONFIG_LCD_SKELDEBUG
#endif

/* Color Properties *********************************************************/

/* Display Resolution */

#define SKEL_XRES         320
#define SKEL_YRES         240

/* Color depth and format */

#define SKEL_BPP          16
#define SKEL_COLORFMT     FB_FMT_RGB16_565

/* Debug ********************************************************************/

#ifdef CONFIG_LCD_SKELDEBUG
# define skelerr(format, ...)  _err(format, ##__VA_ARGS__)
# define skelwarn(format, ...) _warn(format, ##__VA_ARGS__)
# define skelinfo(format, ...) _info(format, ##__VA_ARGS__)
#else
# define skelerr(x...)
# define skelwarn(x...)
# define skelinfo(x...)
#endif

/****************************************************************************
 * Private Type Definition
 ****************************************************************************/

/* This structure describes the state of this driver */

struct skel_dev_s
{
  /* Publicly visible device structure */

  struct lcd_dev_s dev;

  /* Private LCD-specific information follows */
};

/****************************************************************************
 * Private Function Protototypes
 ****************************************************************************/

/* LCD Data Transfer Methods */

static int skel_putrun(fb_coord_t row, fb_coord_t col,
                       FAR const uint8_t *buffer,
                       size_t npixels);
static int skel_getrun(fb_coord_t row, fb_coord_t col,
                       FAR uint8_t *buffer,
                       size_t npixels);

/* LCD Configuration */

static int skel_getvideoinfo(FAR struct lcd_dev_s *dev,
                             FAR struct fb_videoinfo_s *vinfo);
static int skel_getplaneinfo(FAR struct lcd_dev_s *dev,
                             unsigned int planeno,
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

static int skel_getpower(struct lcd_dev_s *dev);
static int skel_setpower(struct lcd_dev_s *dev, int power);
static int skel_getcontrast(struct lcd_dev_s *dev);
static int skel_setcontrast(struct lcd_dev_s *dev,
                            unsigned int contrast);

/****************************************************************************
 * Private Data
 ****************************************************************************/

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

static uint16_t g_runbuffer[SKEL_XRES];

/* This structure describes the overall LCD video controller */

static const struct fb_videoinfo_s g_videoinfo =
{
  .fmt     = SKEL_COLORFMT,    /* Color format: RGB16-565: RRRR RGGG GGGB BBBB */
  .xres    = SKEL_XRES,        /* Horizontal resolution in pixel columns */
  .yres    = SKEL_YRES,        /* Vertical resolution in pixel rows */
  .nplanes = 1,                /* Number of color planes supported */
};

/* This is the standard, NuttX Plane information object */

static const struct lcd_planeinfo_s g_planeinfo =
{
  .putrun = skel_putrun,                /* Put a run into LCD memory */
  .getrun = skel_getrun,                /* Get a run from LCD memory */
  .buffer = (FAR uint8_t *)g_runbuffer, /* Run scratch buffer */
  .bpp    = SKEL_BPP,                   /* Bits-per-pixel */
};

/* This is the standard, NuttX LCD driver object */

static struct skel_dev_s g_lcddev =
{
  .dev =
  {
    /* LCD Configuration */

    .getvideoinfo = skel_getvideoinfo,
    .getplaneinfo = skel_getplaneinfo,

    /* LCD RGB Mapping -- Not supported */

    /* Cursor Controls -- Not supported */

    /* LCD Specific Controls */

    .getpower     = skel_getpower,
    .setpower     = skel_setpower,
    .getcontrast  = skel_getcontrast,
    .setcontrast  = skel_setcontrast,
  },
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  skel_putrun
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
 ****************************************************************************/

static int skel_putrun(fb_coord_t row, fb_coord_t col,
                       FAR const uint8_t *buffer,
                       size_t npixels)
{
  /* Buffer must be provided and aligned to a 16-bit address boundary */

  ginfo("row: %d col: %d npixels: %d\n", row, col, npixels);
  DEBUGASSERT(buffer && ((uintptr_t)buffer & 1) == 0);

  /* Set up to write the run. */

  /* Write the run to GRAM. */
#warning "Missing logic"
  return OK;
}

/****************************************************************************
 * Name:  skel_getrun
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
 ****************************************************************************/

static int skel_getrun(fb_coord_t row, fb_coord_t col, FAR uint8_t *buffer,
                       size_t npixels)
{
  /* Buffer must be provided and aligned to a 16-bit address boundary */

  ginfo("row: %d col: %d npixels: %d\n", row, col, npixels);
  DEBUGASSERT(buffer && ((uintptr_t)buffer & 1) == 0);

#warning "Missing logic"
  return -ENOSYS;
}

/****************************************************************************
 * Name:  skel_getvideoinfo
 *
 * Description:
 *   Get information about the LCD video controller configuration.
 *
 ****************************************************************************/

static int skel_getvideoinfo(FAR struct lcd_dev_s *dev,
                              FAR struct fb_videoinfo_s *vinfo)
{
  DEBUGASSERT(dev && vinfo);
  ginfo("fmt: %d xres: %d yres: %d nplanes: %d\n",
         g_videoinfo.fmt, g_videoinfo.xres,
         g_videoinfo.yres, g_videoinfo.nplanes);
  memcpy(vinfo, &g_videoinfo, sizeof(struct fb_videoinfo_s));
  return OK;
}

/****************************************************************************
 * Name:  skel_getplaneinfo
 *
 * Description:
 *   Get information about the configuration of each LCD color plane.
 *
 ****************************************************************************/

static int skel_getplaneinfo(FAR struct lcd_dev_s *dev,
                             unsigned int planeno,
                             FAR struct lcd_planeinfo_s *pinfo)
{
  DEBUGASSERT(dev && pinfo && planeno == 0);
  ginfo("planeno: %d bpp: %d\n", planeno, g_planeinfo.bpp);
  memcpy(pinfo, &g_planeinfo, sizeof(struct lcd_planeinfo_s));
  return OK;
}

/****************************************************************************
 * Name:  skel_getpower
 *
 * Description:
 *   Get the LCD panel power status
 *  (0: full off - CONFIG_LCD_MAXPOWER: full on).
 *   On backlit LCDs, this setting may correspond to the backlight setting.
 *
 ****************************************************************************/

static int skel_getpower(struct lcd_dev_s *dev)
{
  struct skel_dev_s *priv = (struct skel_dev_s *)dev;
  ginfo("power: %d\n", 0);
#warning "Missing logic"
  return 0;
}

/****************************************************************************
 * Name:  skel_setpower
 *
 * Description:
 *   Enable/disable LCD panel power
 *  (0: full off - CONFIG_LCD_MAXPOWER: full on).
 *   On backlit LCDs, this setting may correspond to the backlight setting.
 *
 ****************************************************************************/

static int skel_setpower(struct lcd_dev_s *dev, int power)
{
  struct skel_dev_s *priv = (struct skel_dev_s *)dev;

  ginfo("power: %d\n", power);
  DEBUGASSERT(power <= CONFIG_LCD_MAXPOWER);

  /* Set new power level */
#warning "Missing logic"

  return OK;
}

/****************************************************************************
 * Name:  skel_getcontrast
 *
 * Description:
 *   Get the current contrast setting (0-CONFIG_LCD_MAXCONTRAST).
 *
 ****************************************************************************/

static int skel_getcontrast(struct lcd_dev_s *dev)
{
  ginfo("Not implemented\n");
#warning "Missing logic"
  return -ENOSYS;
}

/****************************************************************************
 * Name:  skel_setcontrast
 *
 * Description:
 *   Set LCD panel contrast (0-CONFIG_LCD_MAXCONTRAST).
 *
 ****************************************************************************/

static int skel_setcontrast(struct lcd_dev_s *dev, unsigned int contrast)
{
  ginfo("contrast: %d\n", contrast);
#warning "Missing logic"
  return -ENOSYS;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  up_oledinitialize
 *
 * Description:
 *   Initialize the LCD video hardware.
 *   The initial state of the LCD is fully initialized, display memory
 *   cleared, and the LCD ready to use, but with the power  setting at 0
 *  (full off).
 *
 ****************************************************************************/

FAR struct lcd_dev_s *up_oledinitialize(FAR struct spi_dev_s *spi)
{
  ginfo("Initializing\n");

  /* Configure GPIO pins */
#warning "Missing logic"

  /* Enable clocking */
#warning "Missing logic"

  /* Configure and enable LCD */
#warning "Missing logic"

  return &g_lcddev.dev;
}
