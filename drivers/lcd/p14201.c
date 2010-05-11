/**************************************************************************************
 * drivers/lcd/p14201.c
 * Driver for RiT P14201 series display (wih sd1329 IC controller)
 *
 *   Copyright (C) 2010 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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
#include <nuttx/lcd.h>

#include <arch/irq.h>

#include "sd1329.h"

/**************************************************************************************
 * Pre-processor Definitions
 **************************************************************************************/

/* Configuration **********************************************************************/
/* Verify that all configuration requirements have been met */

/* Define the following to enable register-level debug output */

#undef CONFIG_LCD_RITDEBUG

/* Verbose debug must also be enabled */

#ifndef CONFIG_DEBUG
#  undef CONFIG_DEBUG_VERBOSE
#  undef CONFIG_DEBUG_GRAPHICS
#endif

#ifndef CONFIG_DEBUG_VERBOSE
#  undef CONFIG_LCD_RITDEBUG
#endif

/* Color Properties *******************************************************************/

/* Display Resolution */

#define RIT_XRES         128
#define RIT_YRES         96

/* Color depth and format */

#define RIT_BPP          4
#define RIT_COLORFMT     FB_FMT_Y4

/* Debug ******************************************************************************/

#ifdef CONFIG_LCD_RITDEBUG
# define ritdbg(format, arg...)  vdbg(format, ##arg)
#else
# define ritdbg(x...)
#endif

/**************************************************************************************
 * Private Type Definition
 **************************************************************************************/

/* This structure describes the state of this driver */

struct rit_dev_s
{
  /* Publically visible device structure */

  struct lcd_dev_s dev;

  /* Private LCD-specific information follows */
};

/**************************************************************************************
 * Private Function Protototypes
 **************************************************************************************/

/* LCD Data Transfer Methods */

static int rit_putrun(fb_coord_t row, fb_coord_t col, FAR const uint8_t *buffer,
             size_t npixels);
static int rit_getrun(fb_coord_t row, fb_coord_t col, FAR uint8_t *buffer,
             size_t npixels);

/* LCD Configuration */

static int rit_getvideoinfo(FAR struct lcd_dev_s *dev,
             FAR struct fb_videoinfo_s *vinfo);
static int rit_getplaneinfo(FAR struct lcd_dev_s *dev, unsigned int planeno,
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

static int rit_getpower(struct lcd_dev_s *dev);
static int rit_setpower(struct lcd_dev_s *dev, int power);
static int rit_getcontrast(struct lcd_dev_s *dev);
static int rit_setcontrast(struct lcd_dev_s *dev, unsigned int contrast);

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

static uint8_t g_runbuffer[RIT_XRES/2];

/* This structure describes the overall LCD video controller */

static const struct fb_videoinfo_s g_videoinfo =
{
  .fmt     = RIT_COLORFMT,        /* Color format: RGB16-565: RRRR RGGG GGGB BBBB */
  .xres    = RIT_XRES,            /* Horizontal resolution in pixel columns */
  .yres    = RIT_YRES,            /* Vertical resolution in pixel rows */
  .nplanes = 1,                   /* Number of color planes supported */
};

/* This is the standard, NuttX Plane information object */

static const struct lcd_planeinfo_s g_planeinfo = 
{
  .putrun = rit_putrun,            /* Put a run into LCD memory */
  .getrun = rit_getrun,            /* Get a run from LCD memory */
  .buffer = (uint8_t*)g_runbuffer, /* Run scratch buffer */
  .bpp    = RIT_BPP,               /* Bits-per-pixel */
};

/* This is the standard, NuttX LCD driver object */

static struct rit_dev_s g_lcddev_s = 
{
  .dev =
  {
    /* LCD Configuration */
 
    .getvideoinfo = rit_getvideoinfo,
    .getplaneinfo = rit_getplaneinfo,

    /* LCD RGB Mapping -- Not supported */
    /* Cursor Controls -- Not supported */

    /* LCD Specific Controls */

    .getpower     = rit_getpower,
    .setpower     = rit_setpower,
    .getcontrast  = rit_getcontrast,
    .setcontrast  = rit_setcontrast,
  },
};

/**************************************************************************************
 * Private Functions
 **************************************************************************************/

/**************************************************************************************
 * Name:  rit_putrun
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

static int rit_putrun(fb_coord_t row, fb_coord_t col, FAR const uint8_t *buffer,
                       size_t npixels)
{
  /* Buffer must be provided and aligned to a 16-bit address boundary */

  gvdbg("row: %d col: %d npixels: %d\n", row, col, npixels);
  DEBUGASSERT(buffer && ((uintptr_t)buffer & 1) == 0);

  /* Set up to write the run. */

  /* Write the run to GRAM. */

  return OK;
}

/**************************************************************************************
 * Name:  rit_getrun
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

static int rit_getrun(fb_coord_t row, fb_coord_t col, FAR uint8_t *buffer,
                       size_t npixels)
{
  /* Buffer must be provided and aligned to a 16-bit address boundary */

  gvdbg("row: %d col: %d npixels: %d\n", row, col, npixels);
  DEBUGASSERT(buffer && ((uintptr_t)buffer & 1) == 0);

  /* Set up to read the run */

  /* Read the run from GRAM. */

  return OK;
}

/**************************************************************************************
 * Name:  rit_getvideoinfo
 *
 * Description:
 *   Get information about the LCD video controller configuration.
 *
 **************************************************************************************/

static int rit_getvideoinfo(FAR struct lcd_dev_s *dev,
                              FAR struct fb_videoinfo_s *vinfo)
{
  DEBUGASSERT(dev && vinfo);
  gvdbg("fmt: %d xres: %d yres: %d nplanes: %d\n",
         g_videoinfo.fmt, g_videoinfo.xres, g_videoinfo.yres, g_videoinfo.nplanes);
  memcpy(vinfo, &g_videoinfo, sizeof(struct fb_videoinfo_s));
  return OK;
}

/**************************************************************************************
 * Name:  rit_getplaneinfo
 *
 * Description:
 *   Get information about the configuration of each LCD color plane.
 *
 **************************************************************************************/

static int rit_getplaneinfo(FAR struct lcd_dev_s *dev, unsigned int planeno,
                              FAR struct lcd_planeinfo_s *pinfo)
{
  DEBUGASSERT(dev && pinfo && planeno == 0);
  gvdbg("planeno: %d bpp: %d\n", planeno, g_planeinfo.bpp);
  memcpy(pinfo, &g_planeinfo, sizeof(struct lcd_planeinfo_s));
  return OK;
}

/**************************************************************************************
 * Name:  rit_getpower
 *
 * Description:
 *   Get the LCD panel power status (0: full off - CONFIG_LCD_MAXPOWER: full on. On
 *   backlit LCDs, this setting may correspond to the backlight setting.
 *
 **************************************************************************************/

static int rit_getpower(struct lcd_dev_s *dev)
{
  struct rit_dev_s *priv = (struct rit_dev_s *)dev;
  gvdbg("power: %d\n", 0);
  return 0;
}

/**************************************************************************************
 * Name:  rit_setpower
 *
 * Description:
 *   Enable/disable LCD panel power (0: full off - CONFIG_LCD_MAXPOWER: full on). On
 *   backlit LCDs, this setting may correspond to the backlight setting.
 *
 **************************************************************************************/

static int rit_setpower(struct lcd_dev_s *dev, int power)
{
  struct rit_dev_s *priv = (struct rit_dev_s *)dev;

  gvdbg("power: %d\n", power);
  DEBUGASSERT(power <= CONFIG_LCD_MAXPOWER);

  /* Set new power level */

  return OK;
}

/**************************************************************************************
 * Name:  rit_getcontrast
 *
 * Description:
 *   Get the current contrast setting (0-CONFIG_LCD_MAXCONTRAST).
 *
 **************************************************************************************/

static int rit_getcontrast(struct lcd_dev_s *dev)
{
  gvdbg("Not implemented\n");
  return -ENOSYS;
}

/**************************************************************************************
 * Name:  rit_getcontrast
 *
 * Description:
 *   Set LCD panel contrast (0-CONFIG_LCD_MAXCONTRAST).
 *
 **************************************************************************************/

static int rit_setcontrast(struct lcd_dev_s *dev, unsigned int contrast)
{
  gvdbg("contrast: %d\n", contrast);
  return -ENOSYS;
}

/**************************************************************************************
 * Public Functions
 **************************************************************************************/

/**************************************************************************************
 * Name:  up_lcdinitialize
 *
 * Description:
 *   Initialize the LCD video hardware.  The initial state of the LCD is fully
 *   initialized, display memory cleared, and the LCD ready to use, but with the power
 *   setting at 0 (full off).
 *
 **************************************************************************************/

int up_lcdinitialize(void)
{
  gvdbg("Initializing\n");

  /* Configure GPIO pins */

  /* Enable clocking */

  /* Configure and enable LCD */
 
  return OK;
}

/**************************************************************************************
 * Name:  up_lcdgetdev
 *
 * Description:
 *   Return a a reference to the LCD object for the specified LCD.  This allows
 *   support for multiple LCD devices.
 *
 **************************************************************************************/

FAR struct lcd_dev_s *up_lcdgetdev(int lcddev)
{
  gvdbg("lcddev: %d\n", lcddev);
  return lcddev == 0 ? &g_lcddev_s.dev : NULL;
}

/**************************************************************************************
 * Name:  up_lcduninitialize
 *
 * Description:
 *   Unitialize the framebuffer support.
 *
 **************************************************************************************/

void up_lcduninitialize(void)
{
  /* Turn the LCD off */

  /* Disable clocking  */
}


