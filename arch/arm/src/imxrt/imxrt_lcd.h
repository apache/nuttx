/****************************************************************************
 * arch/arm/src/imxrt/imxrt_lcd.h
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

/* The I.MX RT LCD driver uses the common framebuffer interfaces declared in
 * include/nuttx/video/fb.h.
 */

#ifndef __ARCH_ARM_SRC_IMXRT_IMXRT_LCD_H
#define __ARCH_ARM_SRC_IMXRT_IMXRT_LCD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>

#include <nuttx/nx/nxglib.h>

#include "hardware/imxrt_lcd.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration */

#ifndef IMXRT_LCD_VIDEO_PLL_FREQ
#  define IMXRT_LCD_VIDEO_PLL_FREQ 92000000
#endif

/* Base address of the video RAM frame buffer */

#ifndef CONFIG_IMXRT_LCD_VRAMBASE
#  define CONFIG_IMXRT_LCD_VRAMBASE (0x20240000)
#endif

/* LCD refresh rate */

#ifndef CONFIG_IMXRT_LCD_REFRESH_FREQ
#  define CONFIG_IMXRT_LCD_REFRESH_FREQ (60) /* Hz */
#endif

#undef IMXRT_BPP
#undef IMXRT_COLOR_FMT

/* Bits per pixel / color format */

#if defined (CONFIG_IMXRT_LCD_INPUT_BPP8_LUT)
#  define IMXRT_BPP 8
#  define IMXRT_COLOR_FMT FB_FMT_RGB8
#elif defined (CONFIG_IMXRT_LCD_INPUT_BPP8)
#  define IMXRT_BPP 8
#  define IMXRT_COLOR_FMT FB_FMT_RGB8_332
#elif defined (CONFIG_IMXRT_LCD_INPUT_BPP15)
#  define IMXRT_BPP 16
#  define IMXRT_COLOR_FMT FB_FMT_RGB16_555
#elif defined (CONFIG_IMXRT_LCD_INPUT_BPP16)
#  define IMXRT_BPP 16
#  define IMXRT_COLOR_FMT FB_FMT_RGB16_565
#elif defined (CONFIG_IMXRT_LCD_INPUT_BPP24)
#  define IMXRT_BPP 24
#  define IMXRT_COLOR_FMT FB_FMT_RGB24
#elif defined (CONFIG_IMXRT_LCD_INPUT_BPP32)
#  define IMXRT_BPP 32
#  define IMXRT_COLOR_FMT FB_FMT_RGB32
#else
#  define CONFIG_IMXRT_LCD_INPUT_BPP16
#  define IMXRT_BPP 16
#  define IMXRT_COLOR_FMT FB_FMT_RGB16_565
#endif

#if defined (CONFIG_IMXRT_LCD_OUTPUT_8)
#elif defined (CONFIG_IMXRT_LCD_OUTPUT_16)
#elif defined (CONFIG_IMXRT_LCD_OUTPUT_18)
#elif defined (CONFIG_IMXRT_LCD_OUTPUT_24)
#else
#  define CONFIG_IMXRT_LCD_OUTPUT_24
#endif

#ifndef CONFIG_IMXRT_LCD_BACKCOLOR
#  define CONFIG_IMXRT_LCD_BACKCOLOR 0x0
#endif

#ifndef CONFIG_IMXRT_LCD_HWIDTH
#  define CONFIG_IMXRT_LCD_HWIDTH 480
#endif

#ifndef CONFIG_IMXRT_LCD_HPULSE
#  define CONFIG_IMXRT_LCD_HPULSE 41
#endif

#ifndef CONFIG_IMXRT_LCD_HFRONTPORCH
#  define CONFIG_IMXRT_LCD_HFRONTPORCH 4
#endif

#ifndef CONFIG_IMXRT_LCD_HBACKPORCH
#  define CONFIG_IMXRT_LCD_HBACKPORCH 8
#endif

#ifndef CONFIG_IMXRT_LCD_VHEIGHT
#  define CONFIG_IMXRT_LCD_VHEIGHT 272
#endif

#ifndef CONFIG_IMXRT_LCD_VPULSE
#  define CONFIG_IMXRT_LCD_VPULSE 10
#endif

#ifndef CONFIG_IMXRT_LCD_VFRONTPORCH
#  define CONFIG_IMXRT_LCD_VFRONTPORCH 4
#endif

#ifndef CONFIG_IMXRT_LCD_VBACKPORCH
#  define CONFIG_IMXRT_LCD_VBACKPORCH 2
#endif

#ifdef CONFIG_IMXRT_VSYNC_ACTIVE_HIGH
#  define VSYNC_POL LCDIF_VDCTRL0_VSYNC_POL_MASK
#else
#  define VSYNC_POL 0
#endif

#ifdef CONFIG_IMXRT_HSYNC_ACTIVE_HIGH
#  define HSYNC_POL LCDIF_VDCTRL0_HSYNC_POL_MASK
#else
#  define HSYNC_POL 0
#endif

#ifdef CONFIG_IMXRT_DATAEN_ACTIVE_HIGH
#  define DATAEN_POL LCDIF_VDCTRL0_ENABLE_POL_MASK
#else
#  define DATAEN_POL 0
#endif

#ifdef CONFIG_IMXRT_DATA_RISING_EDGE
#  define DATA_EDGE LCDIF_VDCTRL0_DOTCLK_POL_MASK
#else
#  define DATA_EDGE 0
#endif

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: IMXRT_lcdclear
 *
 * Description:
 *   This is a non-standard LCD interface just for the I.MX RT.  Clearing
 *   the display in the normal way by writing a sequences of runs that
 *   covers the entire display can be slow.  Here the display is cleared by
 *   simply setting all VRAM memory to the specified color.
 *
 ****************************************************************************/

void imxrt_lcdclear(nxgl_mxpixel_t color);

/****************************************************************************
 * Name: imxrt_lcd_initialize
 *
 * Description:
 *   Initialize the LCD.  Setup backlight (initially off)
 *
 ****************************************************************************/

void imxrt_lcd_initialize(void);

/****************************************************************************
 * Name: IMXRT_backlight
 *
 * Description:
 *   If CONFIG_IMXRT_LCD_BACKLIGHT is defined, then the board-specific logic
 *   must provide this interface to turn the backlight on and off.
 *
 ****************************************************************************/

#ifdef CONFIG_IMXRT_LCD_BACKLIGHT
void imxrt_backlight(bool blon);
#endif

#endif /* __ARCH_ARM_SRC_IMXRT_IMXRT_LCD_H */
