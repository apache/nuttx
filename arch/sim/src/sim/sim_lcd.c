/****************************************************************************
 * arch/sim/src/sim/sim_lcd.c
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
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/lcd/lcd.h>
#include "sim_internal.h"

#if defined(CONFIG_SIM_X11FB)
#include <nuttx/wqueue.h>
#endif
/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Check contrast selection */

#if !defined(CONFIG_LCD_MAXCONTRAST)
#  define CONFIG_LCD_MAXCONTRAST 100
#endif

/* Check power setting */

#if !defined(CONFIG_LCD_MAXPOWER)
#  define CONFIG_LCD_MAXPOWER 100
#endif

/* Simulated LCD geometry and color format */

#define FB_STRIDE ((CONFIG_SIM_FBBPP * CONFIG_SIM_FBWIDTH + 7) >> 3)

#undef FB_FMT
#if CONFIG_SIM_FBBPP == 1
#  define FB_FMT FB_FMT_Y1
#elif CONFIG_SIM_FBBPP == 4
#  define FB_FMT FB_FMT_RGB4
#elif CONFIG_SIM_FBBPP == 8
#  define FB_FMT FB_FMT_RGB8
#elif CONFIG_SIM_FBBPP == 16
#  define FB_FMT FB_FMT_RGB16_565
#elif CONFIG_SIM_FBBPP == 24
#  define FB_FMT FB_FMT_RGB24
#elif CONFIG_SIM_FBBPP == 32
#  define FB_FMT FB_FMT_RGB32
#else
#  error "Unsupported BPP"
#endif

/****************************************************************************
 * Private Type Definition
 ****************************************************************************/

/* This structure describes the state of this driver */

struct sim_dev_s
{
  /* Publicly visible device structure */

  struct lcd_dev_s dev;

  /* Private LCD-specific information follows */

  uint8_t power;        /* Current power setting */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* LCD Data Transfer Methods */

static int sim_putrun(struct lcd_dev_s *dev, fb_coord_t row, fb_coord_t col,
                      const uint8_t *buffer, size_t npixels);
static int sim_putarea(struct lcd_dev_s *dev, fb_coord_t row_start,
                       fb_coord_t row_end, fb_coord_t col_start,
                       fb_coord_t col_end, const uint8_t *buffer);
static int sim_getrun(struct lcd_dev_s *dev, fb_coord_t row, fb_coord_t col,
                      uint8_t *buffer, size_t npixels);

/* LCD Configuration */

static int sim_getvideoinfo(struct lcd_dev_s *dev,
                            struct fb_videoinfo_s *vinfo);
static int sim_getplaneinfo(struct lcd_dev_s *dev, unsigned int planeno,
                            struct lcd_planeinfo_s *pinfo);

/* LCD RGB Mapping */

#ifdef CONFIG_FB_CMAP
#  error "RGB color mapping not supported by this driver"
#endif

/* Cursor Controls */

#ifdef CONFIG_FB_HWCURSOR
#  error "Cursor control not supported by this driver"
#endif

/* LCD Specific Controls */

static int sim_getpower(struct lcd_dev_s *dev);
static int sim_setpower(struct lcd_dev_s *dev, int power);
static int sim_getcontrast(struct lcd_dev_s *dev);
static int sim_setcontrast(struct lcd_dev_s *dev, unsigned int contrast);

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifndef CONFIG_SIM_X11FB
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

static uint8_t g_runbuffer[FB_STRIDE];
#else
static size_t g_fblen;
static unsigned short g_stride;
#endif

/* This structure describes the overall LCD video controller */

static const struct fb_videoinfo_s g_videoinfo =
{
  .fmt     = FB_FMT,                     /* Color format: RGB16-565: RRRR RGGG GGGB BBBB */
  .xres    = CONFIG_SIM_FBWIDTH,         /* Horizontal resolution in pixel columns */
  .yres    = CONFIG_SIM_FBHEIGHT,        /* Vertical resolution in pixel rows */
  .nplanes = 1,                          /* Number of color planes supported */
};

/* This is the standard, NuttX Plane information object */

static struct lcd_planeinfo_s g_planeinfo =
{
  .putrun  = sim_putrun,                 /* Put a run into LCD memory */
  .putarea = sim_putarea,                /* Put a rectangular area to LCD */
  .getrun  = sim_getrun,                 /* Get a run from LCD memory */
#ifndef CONFIG_SIM_X11FB
  .buffer  = (uint8_t *)g_runbuffer, /* Run scratch buffer */
#endif
  .bpp     = CONFIG_SIM_FBBPP,           /* Bits-per-pixel */
};

/* This is the standard, NuttX LCD driver object */

static struct sim_dev_s g_lcddev =
{
  .dev =
  {
    /* LCD Configuration */

    .getvideoinfo = sim_getvideoinfo,
    .getplaneinfo = sim_getplaneinfo,

    /* LCD RGB Mapping -- Not supported */

    /* Cursor Controls -- Not supported */

    /* LCD Specific Controls */

    .getpower     = sim_getpower,
    .setpower     = sim_setpower,
    .getcontrast  = sim_getcontrast,
    .setcontrast  = sim_setcontrast,
  },
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  sim_putrun
 *
 * Description:
 *   This method can be used to write a partial raster line to the LCD:
 *
 *   dev     - The LCD device
 *   row     - Starting row to write to (range: 0 <= row < yres)
 *   col     - Starting column to write to (range: 0 <= col <= xres-npixels)
 *   buffer  - The buffer containing the run to be written to the LCD
 *   npixels - The number of pixels to write to the LCD
 *             (range: 0 < npixels <= xres-col)
 *
 ****************************************************************************/

static int sim_putrun(struct lcd_dev_s *dev, fb_coord_t row, fb_coord_t col,
                      const uint8_t *buffer, size_t npixels)
{
  lcdinfo("row: %d col: %d npixels: %zu\n", row, col, npixels);

#ifdef CONFIG_SIM_X11FB
  memcpy(&g_planeinfo.buffer[row * g_stride + col * (g_planeinfo.bpp / 8)],
         buffer, npixels * g_planeinfo.bpp / 8);
#endif

  return OK;
}

/****************************************************************************
 * Name:  sim_putarea
 *
 * Description:
 *   This method can be used to write a partial raster line to the LCD:
 *
 *   dev       - The LCD device
 *   row_start - Starting row to write to (range: 0 <= row < yres)
 *   row_end   - Ending row to write to (range: row_start <= row < yres)
 *   col_start - Starting column to write to (range: 0 <= col <= xres)
 *   col_end   - Ending column to write to
 *               (range: col_start <= col_end < xres)
 *   buffer    - The buffer containing the area to be written to the LCD
 *
 ****************************************************************************/

static int sim_putarea(struct lcd_dev_s *dev, fb_coord_t row_start,
                       fb_coord_t row_end, fb_coord_t col_start,
                       fb_coord_t col_end, const uint8_t *buffer)
{
  fb_coord_t row;
  size_t rows;
  size_t cols;
  size_t row_size;

  lcdinfo("row_start: %d row_end: %d col_start: %d col_end: %d\n",
          row_start, row_end, col_start, col_end);

  cols = col_end - col_start + 1;
  rows = row_end - row_start + 1;
  row_size = cols * (g_planeinfo.bpp >> 3);

#ifdef CONFIG_SIM_X11FB
  if (col_start == 0 && col_end == (g_videoinfo.xres - 1) &&
      g_stride == row_size)
    {
      /* simpler case, we can just memcpy() the whole buffer */

      memcpy(&g_planeinfo.buffer[row_start * g_stride], buffer,
             rows * row_size);
    }
  else
    {
      /* We have to go row by row */

      for (row = row_start; row <= row_end; row++)
        {
          memcpy(&g_planeinfo.buffer[row * g_stride + col_start *
                                     (g_planeinfo.bpp >> 3)],
                 &buffer[(row - row_start) * row_size],
                 cols * (g_planeinfo.bpp >> 3));
        }
    }
#endif

  return OK;
}

/****************************************************************************
 * Name:  sim_getrun
 *
 * Description:
 *   This method can be used to read a partial raster line from the LCD:
 *
 *  dev     - The LCD device
 *  row     - Starting row to read from (range: 0 <= row < yres)
 *  col     - Starting column to read read (range: 0 <= col <= xres-npixels)
 *  buffer  - The buffer in which to return the run read from the LCD
 *  npixels - The number of pixels to read from the LCD
 *            (range: 0 < npixels <= xres-col)
 *
 ****************************************************************************/

static int sim_getrun(struct lcd_dev_s *dev, fb_coord_t row, fb_coord_t col,
                      uint8_t *buffer, size_t npixels)
{
  lcdinfo("row: %d col: %d npixels: %zu\n", row, col, npixels);
  return -ENOSYS;
}

/****************************************************************************
 * Name:  sim_getvideoinfo
 *
 * Description:
 *   Get information about the LCD video controller configuration.
 *
 ****************************************************************************/

static int sim_getvideoinfo(struct lcd_dev_s *dev,
                            struct fb_videoinfo_s *vinfo)
{
  DEBUGASSERT(dev && vinfo);
  ginfo("fmt: %d xres: %d yres: %d nplanes: %d\n",
         g_videoinfo.fmt, g_videoinfo.xres, g_videoinfo.yres,
         g_videoinfo.nplanes);

  memcpy(vinfo, &g_videoinfo, sizeof(struct fb_videoinfo_s));
  return OK;
}

/****************************************************************************
 * Name:  sim_getplaneinfo
 *
 * Description:
 *   Get information about the configuration of each LCD color plane.
 *
 ****************************************************************************/

static int sim_getplaneinfo(struct lcd_dev_s *dev, unsigned int planeno,
                              struct lcd_planeinfo_s *pinfo)
{
  DEBUGASSERT(dev && pinfo && planeno == 0);
  ginfo("planeno: %d bpp: %d\n", planeno, g_planeinfo.bpp);
  memcpy(pinfo, &g_planeinfo, sizeof(struct lcd_planeinfo_s));
  pinfo->dev = dev;
  return OK;
}

/****************************************************************************
 * Name:  sim_getpower
 *
 * Description:
 *   Get the LCD panel power status (0: full off - CONFIG_LCD_MAXPOWER:
 *   full on). On backlit LCDs, this setting may correspond to the backlight
 *   setting.
 *
 ****************************************************************************/

static int sim_getpower(struct lcd_dev_s *dev)
{
  ginfo("power: %d\n", 0);
  return g_lcddev.power;
}

/****************************************************************************
 * Name:  sim_setpower
 *
 * Description:
 *   Enable/disable LCD panel power (0: full off - CONFIG_LCD_MAXPOWER:
 *   full on). On backlit LCDs, this setting may correspond to the backlight
 *   setting.
 *
 ****************************************************************************/

static int sim_setpower(struct lcd_dev_s *dev, int power)
{
  ginfo("power: %d\n", power);
  DEBUGASSERT(power <= CONFIG_LCD_MAXPOWER);

  /* Set new power level */

  g_lcddev.power = power;
  return OK;
}

/****************************************************************************
 * Name:  sim_getcontrast
 *
 * Description:
 *   Get the current contrast setting (0-CONFIG_LCD_MAXCONTRAST).
 *
 ****************************************************************************/

static int sim_getcontrast(struct lcd_dev_s *dev)
{
  ginfo("Not implemented\n");
  return -ENOSYS;
}

/****************************************************************************
 * Name:  sim_setcontrast
 *
 * Description:
 *   Set LCD panel contrast (0-CONFIG_LCD_MAXCONTRAST).
 *
 ****************************************************************************/

static int sim_setcontrast(struct lcd_dev_s *dev, unsigned int contrast)
{
  ginfo("contrast: %d\n", contrast);
  return -ENOSYS;
}

/****************************************************************************
 * Name: sim_updatework
 ****************************************************************************/

void sim_x11loop(void)
{
#ifdef CONFIG_SIM_X11FB
  if (g_planeinfo.buffer != NULL)
    {
      static clock_t last;
      clock_t now = clock_systime_ticks();

      if (now - last >= MSEC2TICK(16))
        {
          sim_x11update();
          last = now;
        }
    }
#endif
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  board_lcd_initialize
 *
 * Description:
 *   Initialize the LCD video hardware.  The initial state of the LCD is
 *   fully initialized, display memory cleared, and the LCD ready to use,
 *   but with the power setting at 0 (full off).
 *
 ****************************************************************************/

int board_lcd_initialize(void)
{
  int ret = OK;

  ginfo("Initializing\n");

#ifdef CONFIG_SIM_X11FB
  ret = sim_x11initialize(CONFIG_SIM_FBWIDTH, CONFIG_SIM_FBHEIGHT,
                          (void**)&g_planeinfo.buffer, &g_fblen,
                          &g_planeinfo.bpp, &g_stride);
#endif

  return ret;
}

/****************************************************************************
 * Name:  board_lcd_getdev
 *
 * Description:
 *   Return a a reference to the LCD object for the specified LCD.  This
 *   allows support for multiple LCD devices.
 *
 ****************************************************************************/

struct lcd_dev_s *board_lcd_getdev(int lcddev)
{
  DEBUGASSERT(lcddev == 0);
  return &g_lcddev.dev;
}

/****************************************************************************
 * Name:  board_lcd_uninitialize
 *
 * Description:
 *   Uninitialize the LCD support
 *
 ****************************************************************************/

void board_lcd_uninitialize(void)
{
}
