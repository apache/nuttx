/****************************************************************************
 * arch/arm/src/am335x/am335x_lcdc.h
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * The LCD driver derives from the LPC54xx LCD driver but also includes
 * information from the FreeBSD AM335x LCD driver which was released under
 * a two-clause BSD license:
 *
 *   Copyright 2013 Oleksandr Tymoshenko <gonzo@freebsd.org>
 *   All rights reserved.
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

/* The LPC54 LCD driver uses the common framebuffer interfaces declared in
 * include/nuttx/video/fb.h.
 */

#ifndef __ARCH_ARM_SRC_AM335X_AM335X_LCDC_H
#define __ARCH_ARM_SRC_AM335X_AM335X_LCDC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>

#include <nuttx/nx/nxglib.h>

#include "hardware/am335x_lcd.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Background color */

#ifndef CONFIG_AM335X_LCDC_BACKCOLOR
#  warning "Assuming background color == 16"
#  define CONFIG_AM335X_LCDC_BACKCOLOR 0  /* Initial background color */
#endif

/* Default characteristics
 * (may be overridden via struct am335x_panel_info_s
 */

/* Bits per pixel / color format */

#undef AM335X_COLOR_FMT
#if defined(CONFIG_AM335X_LCDC_BPP1)
#  define AM335X_BPP                    1
#  define AM335X_COLOR_FMT              FB_FMT_Y1
#elif defined(CONFIG_AM335X_LCDC_BPP2)
#  define AM335X_BPP                    2
#  define AM335X_COLOR_FMT              FB_FMT_Y2
#elif defined(CONFIG_AM335X_LCDC_BPP4)
#  define AM335X_BPP                    4
#  define AM335X_COLOR_FMT              FB_FMT_Y4
#elif defined(CONFIG_AM335X_LCDC_BPP8)
#  define AM335X_BPP                    8
#  define AM335X_COLOR_FMT              FB_FMT_Y8
#elif defined(CONFIG_AM335X_LCDC_BPP12_444)
#  define AM335X_BPP       1            12
#  define AM335X_COLOR_FMT              FB_FMT_RGB12_444
#elif defined(CONFIG_AM335X_LCDC_BPP16_565)
#  define AM335X_BPP                    16
#  define AM335X_COLOR_FMT              FB_FMT_RGB16_565
#elif defined(CONFIG_AM335X_LCDC_BPP24)
#  define AM335X_BPP                    24 RGB */
#  define AM335X_COLOR_FMT              FB_FMT_RGB24
#else
#  warning "Assuming 16 BPP 5:6:5"
#  define AM335X_BPP                    16
#  define CONFIG_AM335X_LCDC_BPP16_565   1
#  define AM335X_COLOR_FMT              FB_FMT_RGB16_565
#endif

#ifndef CONFIG_AM335X_LCDC_ACBIAS
#  warning "Assuming AC bias == 255"
#  define CONFIG_AM335X_LCDC_ACBIAS 255
#endif

#ifndef CONFIG_AM335X_LCDC_ACBIAS_PINT
#  warning "Assuming AC bias per interrupt == 0"
#  define CONFIG_AM335X_LCDC_ACBIAS_PINT 0
#endif

#if defined(CONFIG_AM335X_LCDC_DMA_BURST1)
#  define AM335X_LCD_DMA_BURSTSZ 1
#elif defined(CONFIG_AM335X_LCDC_DMA_BURST2)
#  define AM335X_LCD_DMA_BURSTSZ 2
#elif defined(CONFIG_AM335X_LCDC_DMA_BURST4)
#  define AM335X_LCD_DMA_BURSTSZ 4
#elif defined(CONFIG_AM335X_LCDC_DMA_BURST8)
#  define AM335X_LCD_DMA_BURSTSZ 8
#elif defined(CONFIG_AM335X_LCDC_DMA_BURST16)
#  define AM335X_LCD_DMA_BURSTSZ 16
#else
#  warning "Assuming DMA burst size == 16"
#  define CONFIG_AM335X_LCDC_DMA_BURST16 1
#  define AM335X_LCD_DMA_BURSTSZ 16
#endif

#ifndef CONFIG_AM335X_LCDC_FDD
#  warning "Assuming FDD == 128"
#  define CONFIG_AM335X_LCDC_FDD 128
#endif

#if (CONFIG_AM335X_LCDC_FB_SIZE & 0x000fffff) != 0
#  warning "Framebuffer size must be a multiple of 1Mb"
#endif

#define AM335X_LCDC_FB_SIZE \
    ((CONFIG_AM335X_LCDC_FB_SIZE + 0x000fffff) & ~0x000fffff)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Describes the LCD panel configuration */

struct am335x_panel_info_s
{
  bool     hsync_active;    /* HSync active */
  bool     vsync_active;    /* Invert VSync */
  bool     sync_edge;       /* HSYNC/VSYNC rise or fall */
  bool     sync_ctrl;       /* Hsync/Vsync pixel clock control on/off */
  bool     pixelclk_active; /* Invert pixel clock */

  uint32_t width;           /* Display width (pixels) */
  uint32_t height;          /* Display height (lines) */
  uint32_t hfp;             /* Horizontal front porch (pixels) */
  uint32_t hbp;             /* Horizontal back porch (pixels) */
  uint32_t hsw;             /* HSync width */
  uint32_t vfp;             /* Vertical front porch (lines) */
  uint32_t vbp;             /* Vertical back porch (lines) */
  uint32_t vsw;             /* VSync width */
  uint32_t pixclk;          /* Pixel clock */

  uint32_t acbias;          /* AC bias pin frequency */
  uint32_t acbias_pint;     /* AC bias pins transitions per interrupt */
  uint32_t dma_burstsz;     /* DMA burst size */
  uint32_t bpp;             /* Bits per pixel */
  uint32_t fdd;             /* Palette loading delay */
};

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: am335x_lcd_initialize
 *
 * Description:
 *   Initialize the AM335x for use the display described by the provided
 *   instance of struct am335x_panel_info_s.
 *
 *   This function must be called by board specific logic to initialize the
 *   LCD.  Normally the calling sequence is as follows:
 *
 *   1a. Graphics application starts and initializes the NX server via
 *       boardctl(BOARDIOC_NX_START).  This calls the graphics
 *       initialization function nxmu_start() which, in turn, will call
 *       up_fbinitialize().  Or,
 *   1b. The framebuffer character driver is initialized and calls
 *       up_fbinitialize().
 *   2.  The function up_fbinitialize() must reside in board specific logic
 *       under boards/.  It must create the instance of struct
 *       am335x_panel_info_s and call this function with that instance.
 *
 *   For a directly connected LCD, either (1) the struct am335x_panel_info_s
 *   may be initialized with constant data or (2) the desired video mode can
 *   obtained via lookup from edid_mode_lookup() and the struct
 *   am335x_panel_info_s can be created with am335x_lcd_videomode().
 *
 *   If there is access to Extended Display Identification Data (EDID), then
 *   the board-specific logic may read the EDID data and use
 *   am335x_lcd_edid() to use the EDID data to initialize the struct
 *   am335x_panel_info_s instance.
 *
 * Input Parameters:
 *   panel  - Provides information about the connect LCD panel.
 *
 * Returned value:
 *   Zero (OK) is returned on success; a negated errno value is returned in
 *   the the case of a failure.
 *
 ****************************************************************************/

int am335x_lcd_initialize(const struct am335x_panel_info_s *panel);

/****************************************************************************
 * Name:  am335x_lcdclear
 *
 * Description:
 *   This is a non-standard LCD interface just for the AM335x.  Clearing
 *   the display in the normal way by writing a sequences of runs that
 *   covers the entire display can be slow.  Here the display is cleared by
 *   simply setting all VRAM memory to the specified color.
 *
 ****************************************************************************/

void am335x_lcdclear(nxgl_mxpixel_t color);

/****************************************************************************
 * Name: am335x_lcd_videomode
 *
 * Description:
 *   If the video mod is known, then the board-specific logic may read the
 *   use this function to convert the video mode data to an instance of
 *   struct am335x_panel_info_s which then may be used to initialize the
 *   the LCD/
 *
 * Input Parameters:
 *    videomode - A reference to the desired video mode.
 *    panel    - A user provided location to receive the panel data.
 *
 * Returned value:
 *   None.  Always succeeds.
 *
 ****************************************************************************/

struct videomode_s; /* Forward reference */

void am335x_lcd_videomode(const struct videomode_s *videomode,
                          struct am335x_panel_info_s *panel);

/****************************************************************************
 * Name: am335x_lcd_edid
 *
 * Description:
 *   If there is access to Extended Display Identification Data (EDID),
 *   then the board-specific logic may read the EDID data and use this
 *   function to initialize an instance of struct am335x_panel_info_s.
 *
 *   The returned video mode may optionally be returned to configure HDMI.
 *
 * Input Parameters:
 *   edid     - A reference to the raw EDID data.
 *   len      - The length of the EDID data in bytes
 *   panel    - A user provided location to receive the panel data.
 *   selected - A user provided location to receive the selected video mode.
 *
 * Returned value:
 *   None.  Always succeeds.  The logic will fallback to VGA mode if no
 *   EDID data is provided or if there is no valid video mode in the EDID
 *   data.
 *
 ****************************************************************************/

void am335x_lcd_edid(const uint8_t *edid, size_t edid_len,
                     struct am335x_panel_info_s *panel,
                     const struct videomode_s **selected);

/****************************************************************************
 * Name: am335x_backlight
 *
 * Description:
 *   If CONFIG_AM335X_LCDC_BACKLIGHT is defined, then the board-specific
 *   logic must provide this interface to turn the backlight on and off.
 *
 *   REVISIT:  Current assumes a discrete ON/OFF control.  Needs additional
 *   support for backlight level control.
 *
 ****************************************************************************/

#ifdef CONFIG_AM335X_LCDC_BACKLIGHT
void am335x_backlight(bool blon);
#endif

#endif /* __ARCH_ARM_SRC_AM335X_AM335X_LCDC_H */
