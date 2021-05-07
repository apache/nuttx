/****************************************************************************
 * arch/arm/src/lpc17xx_40xx/lpc17_40_lcd.h
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

#ifndef __ARCH_ARM_SRC_LPC17XX_40XX_LPC17_40_LCD_H
#define __ARCH_ARM_SRC_LPC17XX_40XX_LPC17_40_LCD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>

#include <nuttx/nx/nxglib.h>

#include "hardware/lpc17_40_lcd.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration */

/* Base address of the video RAM frame buffer */

#ifndef CONFIG_LPC17_40_LCD_VRAMBASE
#  define CONFIG_LPC17_40_LCD_VRAMBASE ((uint32_t)LPC17_40_EXTDRAM_CS0 + 0x00010000)
#endif

/* LCD refresh rate */

#ifndef CONFIG_LPC17_40_LCD_REFRESH_FREQ
#  define CONFIG_LPC17_40_LCD_REFRESH_FREQ (50) /* Hz */
#endif

/* Bits per pixel / color format */

#undef LPC17_40_COLOR_FMT
#if defined(CONFIG_LPC17_40_LCD_BPP1)
#  define LPC17_40_BPP                    1
#  define LPC17_40_COLOR_FMT              FB_FMT_Y1
#elif defined(CONFIG_LPC17_40_LCD_BPP2)
#  define LPC17_40_BPP                    2
#  define LPC17_40_COLOR_FMT              FB_FMT_Y2
#elif defined(CONFIG_LPC17_40_LCD_BPP4)
#  define LPC17_40_BPP                    4
#  define LPC17_40_COLOR_FMT              FB_FMT_Y4
#elif defined(CONFIG_LPC17_40_LCD_BPP8)
#  define LPC17_40_BPP                    8
#  define LPC17_40_COLOR_FMT              FB_FMT_Y8
#elif defined(CONFIG_LPC17_40_LCD_BPP16)
#  define LPC17_40_BPP                    16
#  define LPC17_40_COLOR_FMT              FB_FMT_Y16
#elif defined(CONFIG_LPC17_40_LCD_BPP24)
#  define LPC17_40_BPP                    32  /* Only 24 of 32 bits used for RGB */
#  define LPC17_40_COLOR_FMT              FB_FMT_RGB24
#  ifndef CONFIG_LPC17_40_LCD_TFTPANEL
#    error "24 BPP is only available for a TFT panel"
#  endif
#elif defined(CONFIG_LPC17_40_LCD_BPP16_565)
#  define LPC17_40_BPP                    16
#  define LPC17_40_COLOR_FMT              FB_FMT_RGB16_565
#elif defined(CONFIG_LPC17_40_LCD_BPP12_444)
#  define LPC17_40_BPP       1             2
#  define LPC17_40_COLOR_FMT              FB_FMT_RGB12_444
#else
#  ifndef CONFIG_LPC17_40_LCD_TFTPANEL
#    warning "Assuming 24 BPP"
#    define LPC17_40_BPP                  24
#    define CONFIG_LPC17_40_LCD_BPP24     1
#    define LPC17_40_COLOR_FMT            FB_FMT_RGB24
#  else
#    warning "Assuming 16 BPP 5:6:5"
#    define LPC17_40_BPP                  16
#    define CONFIG_LPC17_40_LCD_BPP16_565 1
#    define LPC17_40_COLOR_FMT            FB_FMT_RGB16_565
#  endif
#endif

/* Background color */

#ifndef CONFIG_LPC17_40_LCD_BACKCOLOR
#  define CONFIG_LPC17_40_LCD_BACKCOLOR 0  /* Initial background color */
#endif

/* Horizontal video characteristics */

#ifndef CONFIG_LPC17_40_LCD_HWIDTH
#  define CONFIG_LPC17_40_LCD_HWIDTH 480 /* Width in pixels */
#endif

#ifndef CONFIG_LPC17_40_LCD_HPULSE
#  define CONFIG_LPC17_40_LCD_HPULSE 2
#endif

#ifndef CONFIG_LPC17_40_LCD_HFRONTPORCH
#  define CONFIG_LPC17_40_LCD_HFRONTPORCH 5
#endif

#ifndef CONFIG_LPC17_40_LCD_HBACKPORCH
#  define CONFIG_LPC17_40_LCD_HBACKPORCH 40
#endif

/* Vertical video characteristics */

#ifndef CONFIG_LPC17_40_LCD_VHEIGHT
#  define CONFIG_LPC17_40_LCD_VHEIGHT 272 /* Height in rows */
#endif

#ifndef CONFIG_LPC17_40_LCD_VPULSE
#  define CONFIG_LPC17_40_LCD_VPULSE 2
#endif

#ifndef CONFIG_LPC17_40_LCD_VFRONTPORCH
#  define CONFIG_LPC17_40_LCD_VFRONTPORCH 8
#endif

#ifndef CONFIG_LPC17_40_LCD_VBACKPORCH
#  define CONFIG_LPC17_40_LCD_VBACKPORCH 8
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

/* The LPC17 LCD driver uses the common framebuffer interfaces declared in
 * include/nuttx/video/fb.h.
 */

/****************************************************************************
 * Name: lpc17_40_lcdclear
 *
 * Description:
 *   This is a non-standard LCD interface just for the LPC17xx/LPC40xx.
 *   Clearing the display in the normal way by writing a sequences of runs
 *   that covers the entire display can be slow.
 *    Here the display is cleared by simply setting all VRAM memory to
 *   the specified color.
 *
 ****************************************************************************/

void lpc17_40_lcdclear(nxgl_mxpixel_t color);

/****************************************************************************
 * Name: lpc17_40_backlight
 *
 * Description:
 *   If CONFIG_LPC17_40_LCD_BACKLIGHT is defined, then the board-specific
 *   logic must provide this interface to turn the backlight on and off.
 *
 ****************************************************************************/

#ifdef CONFIG_LPC17_40_LCD_BACKLIGHT
void lpc17_40_backlight(bool blon);
#endif

#endif /* __ARCH_ARM_SRC_LPC17XX_40XX_LPC17_40_LCD_H */
