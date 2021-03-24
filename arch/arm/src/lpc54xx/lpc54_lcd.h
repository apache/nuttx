/****************************************************************************
 * arch/arm/src/lpc54xx/lpc54_lcd.h
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

/* The LPC54 LCD driver uses the common framebuffer interfaces declared in
 * include/nuttx/video/fb.h.
 */

#ifndef __ARCH_ARM_SRC_LPC54XX_LPC54_LCD_H
#define __ARCH_ARM_SRC_LPC54XX_LPC54_LCD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>

#include <nuttx/nx/nxglib.h>

#include "hardware/lpc54_lcd.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration */

/* Base address of the video RAM frame buffer */

#ifndef CONFIG_LPC54_LCD_VRAMBASE
#  define CONFIG_LPC54_LCD_VRAMBASE ((uint32_t)LPC54_EXTDRAM_CS0 + 0x00010000)
#endif

/* LCD refresh rate */

#ifndef CONFIG_LPC54_LCD_REFRESH_FREQ
#  define CONFIG_LPC54_LCD_REFRESH_FREQ (50) /* Hz */
#endif

/* Bits per pixel / color format */

#undef LPC54_COLOR_FMT
#if defined(CONFIG_LPC54_LCD_BPP1)
#  define LPC54_BPP                    1
#  define LPC54_COLOR_FMT              FB_FMT_Y1
#elif defined(CONFIG_LPC54_LCD_BPP2)
#  define LPC54_BPP                    2
#  define LPC54_COLOR_FMT              FB_FMT_Y2
#elif defined(CONFIG_LPC54_LCD_BPP4)
#  define LPC54_BPP                    4
#  define LPC54_COLOR_FMT              FB_FMT_Y4
#elif defined(CONFIG_LPC54_LCD_BPP8)
#  define LPC54_BPP                    8
#  define LPC54_COLOR_FMT              FB_FMT_Y8
#elif defined(CONFIG_LPC54_LCD_BPP12_444)
#  define LPC54_BPP       1            12
#  define LPC54_COLOR_FMT              FB_FMT_RGB12_444
#elif defined(CONFIG_LPC54_LCD_BPP16)
#  define LPC54_BPP                    16
#  define LPC54_COLOR_FMT              FB_FMT_Y16
#elif defined(CONFIG_LPC54_LCD_BPP16_565)
#  define LPC54_BPP                    16
#  define LPC54_COLOR_FMT              FB_FMT_RGB16_565
#elif defined(CONFIG_LPC54_LCD_BPP24)
#  define LPC54_BPP                    32  /* Only 24 of 32 bits used for RGB */
#  define LPC54_COLOR_FMT              FB_FMT_RGB24
#  ifndef CONFIG_LPC54_LCD_TFTPANEL
#    error "24 BPP is only available for a TFT panel"
#  endif
#else
#  ifndef CONFIG_LPC54_LCD_TFTPANEL
#    warning "Assuming 24 BPP"
#    define LPC54_BPP                  24
#    define CONFIG_LPC54_LCD_BPP24     1
#    define LPC54_COLOR_FMT            FB_FMT_RGB24
#  else
#    warning "Assuming 16 BPP 5:6:5"
#    define LPC54_BPP                  16
#    define CONFIG_LPC54_LCD_BPP16_565 1
#    define LPC54_COLOR_FMT            FB_FMT_RGB16_565
#  endif
#endif

/* Background color */

#ifndef CONFIG_LPC54_LCD_BACKCOLOR
#  define CONFIG_LPC54_LCD_BACKCOLOR 0  /* Initial background color */
#endif

/* Horizontal video characteristics */

#ifndef CONFIG_LPC54_LCD_HWIDTH
#  define CONFIG_LPC54_LCD_HWIDTH 480 /* Width in pixels */
#endif

#ifndef CONFIG_LPC54_LCD_HPULSE
#  define CONFIG_LPC54_LCD_HPULSE 2
#endif

#ifndef CONFIG_LPC54_LCD_HFRONTPORCH
#  define CONFIG_LPC54_LCD_HFRONTPORCH 5
#endif

#ifndef CONFIG_LPC54_LCD_HBACKPORCH
#  define CONFIG_LPC54_LCD_HBACKPORCH 40
#endif

/* Vertical video characteristics */

#ifndef CONFIG_LPC54_LCD_VHEIGHT
#  define CONFIG_LPC54_LCD_VHEIGHT 272 /* Height in rows */
#endif

#ifndef CONFIG_LPC54_LCD_VPULSE
#  define CONFIG_LPC54_LCD_VPULSE 2
#endif

#ifndef CONFIG_LPC54_LCD_VFRONTPORCH
#  define CONFIG_LPC54_LCD_VFRONTPORCH 8
#endif

#ifndef CONFIG_LPC54_LCD_VBACKPORCH
#  define CONFIG_LPC54_LCD_VBACKPORCH 8
#endif

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: lpc54_lcdclear
 *
 * Description:
 *   This is a non-standard LCD interface just for the LPC54xx.  Clearing
 *   the display in the normal way by writing a sequences of runs that
 *   covers the entire display can be slow.  Here the display is cleared by
 *   simply setting all VRAM memory to the specified color.
 *
 ****************************************************************************/

void lpc54_lcdclear(nxgl_mxpixel_t color);

/****************************************************************************
 * Name: lpc54_backlight
 *
 * Description:
 *   If CONFIG_LPC54_LCD_BACKLIGHT is defined, then the board-specific logic
 *   must provide this interface to turn the backlight on and off.
 *
 ****************************************************************************/

#ifdef CONFIG_LPC54_LCD_BACKLIGHT
void lpc54_backlight(bool blon);
#endif

#endif /* __ARCH_ARM_SRC_LPC54XX_LPC54_LCD_H */
