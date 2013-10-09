/****************************************************************************
 * arch/arm/src/sama5/sam_lcd.c
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * References:
 *   SAMA5D3 Series Data Sheet
 *   Atmel NoOS sample code.
 *
 * The Atmel sample code has a BSD compatibile license that requires this
 * copyright notice:
 *
 *   Copyright (c) 2012, Atmel Corporation
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
 * 3. Neither the names NuttX nor Atmel nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
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

#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/fb.h>
#include <nuttx/kmalloc.h>

#include <arch/board/board.h>

#include "up_arch.h"
#include "cache.h"
#include "chip/sam_lcdc.h"
#include "chip/sam_pinmap.h"
#include "sam_pio.h"
#include "sam_periphclks.h"
#include "sam_memories.h"
#include "sam_lcd.h"

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/

#ifndef CONFIG_SAMA5_LCDC_DEFBACKLIGHT
#  define CONFIG_SAMA5_LCDC_DEFBACKLIGHT 0xf0
#endif
#define SAMA5_LCDC_BACKLIGHT_OFF 0x00

/* Color formats */

#if defined(CONFIG_SAMA5_LCDC_BASE_RGB444)
#  define SAMA5_LCDC_BASE_BPP       16  /* 12BPP but must be 16-bit aligned */
#  define SAMA5_LCDC_BASE_COLOR_FMT FB_FMT_RGB12_444
#elif defined(CONFIG_SAMA5_LCDC_BASE_ARGB4444)
#  define SAMA5_LCDC_BASE_BPP       16
#  define SAMA5_LCDC_BASE_COLOR_FMT ???
#elif defined(CONFIG_SAMA5_LCDC_BASE_RGBA4444)
#  define SAMA5_LCDC_BASE_BPP       16
#  define SAMA5_LCDC_BASE_COLOR_FMT ???
#elif defined(CONFIG_SAMA5_LCDC_BASE_RGB565)
#  define SAMA5_LCDC_BASE_BPP       16
#  define SAMA5_LCDC_BASE_COLOR_FMT FB_FMT_RGB16_565
#elif defined(CONFIG_SAMA5_LCDC_BASE_TRGB1555)
#  define SAMA5_LCDC_BASE_BPP       16
#  define SAMA5_LCDC_BASE_COLOR_FMT FB_FMT_RGBT16
#elif defined(CONFIG_SAMA5_LCDC_BASE_RGB666)
#  define SAMA5_LCDC_BASE_BPP       32  /* 18BPP but must be 32-bit aligned */
#  define SAMA5_LCDC_BASE_COLOR_FMT RGB666
#elif defined(CONFIG_SAMA5_LCDC_BASE_RGB666P)
#  define SAMA5_LCDC_BASE_BPP       24  /* 18BPP but must be byte aligned */
#  define SAMA5_LCDC_BASE_COLOR_FMT ???
#elif defined(CONFIG_SAMA5_LCDC_BASE_TRGB1666)
#  define SAMA5_LCDC_BASE_BPP       32  /* 19BPP but must be 32-bit aligned */
#  define SAMA5_LCDC_BASE_COLOR_FMT ???
#elif defined(CONFIG_SAMA5_LCDC_BASE_TRGBP)
#  define SAMA5_LCDC_BASE_BPP       24  /* 19BPP but must be byte aligned */
#  define SAMA5_LCDC_BASE_COLOR_FMT ???
#elif defined(CONFIG_SAMA5_LCDC_BASE_RGB888)
#  define SAMA5_LCDC_BASE_BPP       24
#  define SAMA5_LCDC_BASE_COLOR_FMT FB_FMT_RGB12_444
#elif defined(CONFIG_SAMA5_LCDC_BASE_RGB888P)
#  define SAMA5_LCDC_BASE_BPP       24
#  define SAMA5_LCDC_BASE_COLOR_FMT FB_FMT_RGB12_444
#elif defined(CONFIG_SAMA5_LCDC_BASE_TRGB1888)
#  define SAMA5_LCDC_BASE_BPP       32  /* 25BPP but must be byte aligned */
#  define SAMA5_LCDC_BASE_COLOR_FMT FB_FMT_RGB12_444
#elif defined(CONFIG_SAMA5_LCDC_BASE_ARGB8888)
#  define SAMA5_LCDC_BASE_BPP       32
#  define SAMA5_LCDC_BASE_COLOR_FMT ???
#elif defined(CONFIG_SAMA5_LCDC_BASE_RGBA8888)
#  define SAMA5_LCDC_BASE_BPP       32
#  define SAMA5_LCDC_BASE_COLOR_FMT FB_FMT_RGBA32
#endif

#if defined(CONFIG_SAMA5_LCDC_OVR1_RGB444)
#  define SAMA5_LCDC_OVR1_BPP       16  /* 12BPP but must be 16-bit aligned */
#  define SAMA5_LCDC_OVR1_COLOR_FMT FB_FMT_RGB12_444
#elif defined(CONFIG_SAMA5_LCDC_OVR1_ARGB4444)
#  define SAMA5_LCDC_OVR1_BPP       16
#  define SAMA5_LCDC_OVR1_COLOR_FMT ???
#elif defined(CONFIG_SAMA5_LCDC_OVR1_RGBA4444)
#  define SAMA5_LCDC_OVR1_BPP       16
#  define SAMA5_LCDC_OVR1_COLOR_FMT ???
#elif defined(CONFIG_SAMA5_LCDC_OVR1_RGB565)
#  define SAMA5_LCDC_OVR1_BPP       16
#  define SAMA5_LCDC_OVR1_COLOR_FMT FB_FMT_RGB16_565
#elif defined(CONFIG_SAMA5_LCDC_OVR1_TRGB1555)
#  define SAMA5_LCDC_OVR1_BPP       16
#  define SAMA5_LCDC_OVR1_COLOR_FMT FB_FMT_RGBT16
#elif defined(CONFIG_SAMA5_LCDC_OVR1_RGB666)
#  define SAMA5_LCDC_OVR1_BPP       32  /* 18BPP but must be 32-bit aligned */
#  define SAMA5_LCDC_OVR1_COLOR_FMT RGB666
#elif defined(CONFIG_SAMA5_LCDC_OVR1_RGB666P)
#  define SAMA5_LCDC_OVR1_BPP       24  /* 18BPP but must be byte aligned */
#  define SAMA5_LCDC_OVR1_COLOR_FMT ???
#elif defined(CONFIG_SAMA5_LCDC_OVR1_TRGB1666)
#  define SAMA5_LCDC_OVR1_BPP       32  /* 19BPP but must be 32-bit aligned */
#  define SAMA5_LCDC_OVR1_COLOR_FMT ???
#elif defined(CONFIG_SAMA5_LCDC_OVR1_TRGBP)
#  define SAMA5_LCDC_OVR1_BPP       24  /* 19BPP but must be byte aligned */
#  define SAMA5_LCDC_OVR1_COLOR_FMT ???
#elif defined(CONFIG_SAMA5_LCDC_OVR1_RGB888)
#  define SAMA5_LCDC_OVR1_BPP       24
#  define SAMA5_LCDC_OVR1_COLOR_FMT FB_FMT_RGB12_444
#elif defined(CONFIG_SAMA5_LCDC_OVR1_RGB888P)
#  define SAMA5_LCDC_OVR1_BPP       24
#  define SAMA5_LCDC_OVR1_COLOR_FMT FB_FMT_RGB12_444
#elif defined(CONFIG_SAMA5_LCDC_OVR1_TRGB1888)
#  define SAMA5_LCDC_OVR1_BPP       32  /* 25BPP but must be byte aligned */
#  define SAMA5_LCDC_OVR1_COLOR_FMT FB_FMT_RGB12_444
#elif defined(CONFIG_SAMA5_LCDC_OVR1_ARGB8888)
#  define SAMA5_LCDC_OVR1_BPP       32
#  define SAMA5_LCDC_OVR1_COLOR_FMT ???
#elif defined(CONFIG_SAMA5_LCDC_OVR1_RGBA8888)
#  define SAMA5_LCDC_OVR1_BPP       32
#  define SAMA5_LCDC_OVR1_COLOR_FMT FB_FMT_RGBA32
#endif

#if defined(CONFIG_SAMA5_LCDC_OVR2_RGB444)
#  define SAMA5_LCDC_OVR2_BPP       16  /* 12BPP but must be 16-bit aligned */
#  define SAMA5_LCDC_OVR2_COLOR_FMT FB_FMT_RGB12_444
#elif defined(CONFIG_SAMA5_LCDC_OVR2_ARGB4444)
#  define SAMA5_LCDC_OVR2_BPP       16
#  define SAMA5_LCDC_OVR2_COLOR_FMT ???
#elif defined(CONFIG_SAMA5_LCDC_OVR2_RGBA4444)
#  define SAMA5_LCDC_OVR2_BPP       16
#  define SAMA5_LCDC_OVR2_COLOR_FMT ???
#elif defined(CONFIG_SAMA5_LCDC_OVR2_RGB565)
#  define SAMA5_LCDC_OVR2_BPP       16
#  define SAMA5_LCDC_OVR2_COLOR_FMT FB_FMT_RGB16_565
#elif defined(CONFIG_SAMA5_LCDC_OVR2_TRGB1555)
#  define SAMA5_LCDC_OVR2_BPP       16
#  define SAMA5_LCDC_OVR2_COLOR_FMT FB_FMT_RGBT16
#elif defined(CONFIG_SAMA5_LCDC_OVR2_RGB666)
#  define SAMA5_LCDC_OVR2_BPP       32  /* 18BPP but must be 32-bit aligned */
#  define SAMA5_LCDC_OVR2_COLOR_FMT RGB666
#elif defined(CONFIG_SAMA5_LCDC_OVR2_RGB666P)
#  define SAMA5_LCDC_OVR2_BPP       24  /* 18BPP but must be byte aligned */
#  define SAMA5_LCDC_OVR2_COLOR_FMT ???
#elif defined(CONFIG_SAMA5_LCDC_OVR2_TRGB1666)
#  define SAMA5_LCDC_OVR2_BPP       32  /* 19BPP but must be 32-bit aligned */
#  define SAMA5_LCDC_OVR2_COLOR_FMT ???
#elif defined(CONFIG_SAMA5_LCDC_OVR2_TRGBP)
#  define SAMA5_LCDC_OVR2_BPP       24  /* 19BPP but must be byte aligned */
#  define SAMA5_LCDC_OVR2_COLOR_FMT ???
#elif defined(CONFIG_SAMA5_LCDC_OVR2_RGB888)
#  define SAMA5_LCDC_OVR2_BPP       24
#  define SAMA5_LCDC_OVR2_COLOR_FMT FB_FMT_RGB12_444
#elif defined(CONFIG_SAMA5_LCDC_OVR2_RGB888P)
#  define SAMA5_LCDC_OVR2_BPP       24
#  define SAMA5_LCDC_OVR2_COLOR_FMT FB_FMT_RGB12_444
#elif defined(CONFIG_SAMA5_LCDC_OVR2_TRGB1888)
#  define SAMA5_LCDC_OVR2_BPP       32  /* 25BPP but must be byte aligned */
#  define SAMA5_LCDC_OVR2_COLOR_FMT FB_FMT_RGB12_444
#elif defined(CONFIG_SAMA5_LCDC_OVR2_ARGB8888)
#  define SAMA5_LCDC_OVR2_BPP       32
#  define SAMA5_LCDC_OVR2_COLOR_FMT ???
#elif defined(CONFIG_SAMA5_LCDC_OVR2_RGBA8888)
#  define SAMA5_LCDC_OVR2_BPP       32
#  define SAMA5_LCDC_OVR2_COLOR_FMT FB_FMT_RGBA32
#endif

#if defined(CONFIG_SAMA5_LCDC_HEO_RGB444)
#  define SAMA5_LCDC_HEO_BPP       16  /* 12BPP but must be 16-bit aligned */
#  define SAMA5_LCDC_HEO_COLOR_FMT FB_FMT_RGB12_444
#elif defined(CONFIG_SAMA5_LCDC_HEO_ARGB4444)
#  define SAMA5_LCDC_HEO_BPP       16
#  define SAMA5_LCDC_HEO_COLOR_FMT ???
#elif defined(CONFIG_SAMA5_LCDC_HEO_RGBA4444)
#  define SAMA5_LCDC_HEO_BPP       16
#  define SAMA5_LCDC_HEO_COLOR_FMT ???
#elif defined(CONFIG_SAMA5_LCDC_HEO_RGB565)
#  define SAMA5_LCDC_HEO_BPP       16
#  define SAMA5_LCDC_HEO_COLOR_FMT FB_FMT_RGB16_565
#elif defined(CONFIG_SAMA5_LCDC_HEO_TRGB1555)
#  define SAMA5_LCDC_HEO_BPP       16
#  define SAMA5_LCDC_HEO_COLOR_FMT FB_FMT_RGBT16
#elif defined(CONFIG_SAMA5_LCDC_HEO_RGB666)
#  define SAMA5_LCDC_HEO_BPP       32  /* 18BPP but must be 32-bit aligned */
#  define SAMA5_LCDC_HEO_COLOR_FMT RGB666
#elif defined(CONFIG_SAMA5_LCDC_HEO_RGB666P)
#  define SAMA5_LCDC_HEO_BPP       24  /* 18BPP but must be byte aligned */
#  define SAMA5_LCDC_HEO_COLOR_FMT ???
#elif defined(CONFIG_SAMA5_LCDC_HEO_TRGB1666)
#  define SAMA5_LCDC_HEO_BPP       32  /* 19BPP but must be 32-bit aligned */
#  define SAMA5_LCDC_HEO_COLOR_FMT ???
#elif defined(CONFIG_SAMA5_LCDC_HEO_TRGBP)
#  define SAMA5_LCDC_HEO_BPP       24  /* 19BPP but must be byte aligned */
#  define SAMA5_LCDC_HEO_COLOR_FMT ???
#elif defined(CONFIG_SAMA5_LCDC_HEO_RGB888)
#  define SAMA5_LCDC_HEO_BPP       24
#  define SAMA5_LCDC_HEO_COLOR_FMT FB_FMT_RGB12_444
#elif defined(CONFIG_SAMA5_LCDC_HEO_RGB888P)
#  define SAMA5_LCDC_HEO_BPP       24
#  define SAMA5_LCDC_HEO_COLOR_FMT FB_FMT_RGB12_444
#elif defined(CONFIG_SAMA5_LCDC_HEO_TRGB1888)
#  define SAMA5_LCDC_HEO_BPP       32  /* 25BPP but must be byte aligned */
#  define SAMA5_LCDC_HEO_COLOR_FMT FB_FMT_RGB12_444
#elif defined(CONFIG_SAMA5_LCDC_HEO_ARGB8888)
#  define SAMA5_LCDC_HEO_BPP       32
#  define SAMA5_LCDC_HEO_COLOR_FMT ???
#elif defined(CONFIG_SAMA5_LCDC_HEO_RGBA8888)
#  define SAMA5_LCDC_HEO_BPP       32
#  define SAMA5_LCDC_HEO_COLOR_FMT FB_FMT_RGBA32
#endif

#if defined(CONFIG_SAMA5_LCDC_HCR_RGB444)
#  define SAMA5_LCDC_HCR_BPP       16  /* 12BPP but must be 16-bit aligned */
#  define SAMA5_LCDC_HCR_COLOR_FMT FB_FMT_RGB12_444
#elif defined(CONFIG_SAMA5_LCDC_HCR_ARGB4444)
#  define SAMA5_LCDC_HCR_BPP       16
#  define SAMA5_LCDC_HCR_COLOR_FMT ???
#elif defined(CONFIG_SAMA5_LCDC_HCR_RGBA4444)
#  define SAMA5_LCDC_HCR_BPP       16
#  define SAMA5_LCDC_HCR_COLOR_FMT ???
#elif defined(CONFIG_SAMA5_LCDC_HCR_RGB565)
#  define SAMA5_LCDC_HCR_BPP       16
#  define SAMA5_LCDC_HCR_COLOR_FMT FB_FMT_RGB16_565
#elif defined(CONFIG_SAMA5_LCDC_HCR_TRGB1555)
#  define SAMA5_LCDC_HCR_BPP       16
#  define SAMA5_LCDC_HCR_COLOR_FMT FB_FMT_RGBT16
#elif defined(CONFIG_SAMA5_LCDC_HCR_RGB666)
#  define SAMA5_LCDC_HCR_BPP       32  /* 18BPP but must be 32-bit aligned */
#  define SAMA5_LCDC_HCR_COLOR_FMT RGB666
#elif defined(CONFIG_SAMA5_LCDC_HCR_RGB666P)
#  define SAMA5_LCDC_HCR_BPP       24  /* 18BPP but must be byte aligned */
#  define SAMA5_LCDC_HCR_COLOR_FMT ???
#elif defined(CONFIG_SAMA5_LCDC_HCR_TRGB1666)
#  define SAMA5_LCDC_HCR_BPP       32  /* 19BPP but must be 32-bit aligned */
#  define SAMA5_LCDC_HCR_COLOR_FMT ???
#elif defined(CONFIG_SAMA5_LCDC_HCR_TRGBP)
#  define SAMA5_LCDC_HCR_BPP       24  /* 19BPP but must be byte aligned */
#  define SAMA5_LCDC_HCR_COLOR_FMT ???
#elif defined(CONFIG_SAMA5_LCDC_HCR_RGB888)
#  define SAMA5_LCDC_HCR_BPP       24
#  define SAMA5_LCDC_HCR_COLOR_FMT FB_FMT_RGB12_444
#elif defined(CONFIG_SAMA5_LCDC_HCR_RGB888P)
#  define SAMA5_LCDC_HCR_BPP       24
#  define SAMA5_LCDC_HCR_COLOR_FMT FB_FMT_RGB12_444
#elif defined(CONFIG_SAMA5_LCDC_HCR_TRGB1888)
#  define SAMA5_LCDC_HCR_BPP       32  /* 25BPP but must be byte aligned */
#  define SAMA5_LCDC_HCR_COLOR_FMT FB_FMT_RGB12_444
#elif defined(CONFIG_SAMA5_LCDC_HCR_ARGB8888)
#  define SAMA5_LCDC_HCR_BPP       32
#  define SAMA5_LCDC_HCR_COLOR_FMT ???
#elif defined(CONFIG_SAMA5_LCDC_HCR_RGBA8888)
#  define SAMA5_LCDC_HCR_BPP       32
#  define SAMA5_LCDC_HCR_COLOR_FMT FB_FMT_RGBA32
#endif

/* Framebuffer sizes in bytes */

#ifndef BOARD_LCD_WIDTH
#  error BOARD_LCD_WIDTH must be defined in the board.h header file
#endif

#ifndef BOARD_LCD_HEIGHT
#  error BOARD_LCD_HEIGHT must be defined in the board.h header file
#endif

#if SAMA5_LCDC_BASE_BPP == 16
#  define SAMA5_BASE_STRIDE ((BOARD_LCD_WIDTH * 16 + 7) / 8)
#elif SAMA5_LCDC_BASE_BPP == 24
#  define SAMA5_BASE_STRIDE ((BOARD_LCD_WIDTH * 24 + 7) / 8)
#elif SAMA5_LCDC_BASE_BPP == 32
#  define SAMA5_BASE_STRIDE ((BOARD_LCD_WIDTH * 32 + 7) / 8)
#endif

#define SAMA5_BASE_FBSIZE (SAMA5_BASE_STRIDE * BOARD_LCD_HEIGHT)

#ifndef CONFIG_SAMA5_LCDC_OVR1_MAXWIDTH
#  define CONFIG_SAMA5_LCDC_OVR1_MAXWIDTH BOARD_LCD_WIDTH
#endif

#if CONFIG_SAMA5_LCDC_OVR1_MAXWIDTH > BOARD_LCD_WIDTH
#  error Width of overlay 1 exceeds the width of the display
#endif

#ifndef CONFIG_SAMA5_LCDC_OVR1_MAXHEIGHT
#  define CONFIG_SAMA5_LCDC_OVR1_MAXHEIGHT BOARD_LCD_HEIGHT
#endif

#if CONFIG_SAMA5_LCDC_OVR1_MAXHEIGHT > BOARD_LCD_HEIGHT
#  error Height of overlay 1 exceeds the height of the display
#endif

#if SAMA5_LCDC_OVR1_BPP == 16
#  define SAMA5_OVR1_STRIDE ((CONFIG_SAMA5_LCDC_OVR1_MAXWIDTH * 16 + 7) / 8)
#elif SAMA5_LCDC_OVR1_BPP == 24
#  define SAMA5_OVR1_STRIDE ((CONFIG_SAMA5_LCDC_OVR1_MAXWIDTH * 24 + 7) / 8)
#elif SAMA5_LCDC_OVR1_BPP == 32
#  define SAMA5_OVR1_STRIDE ((CONFIG_SAMA5_LCDC_OVR1_MAXWIDTH * 32 + 7) / 8)
#endif

#define SAMA5_OVR1_FBSIZE (SAMA5_OVR1_STRIDE * CONFIG_SAMA5_LCDC_OVR1_MAXHEIGHT)

#ifndef CONFIG_SAMA5_LCDC_OVR2_MAXWIDTH
#  define CONFIG_SAMA5_LCDC_OVR2_MAXWIDTH BOARD_LCD_WIDTH
#endif

#if CONFIG_SAMA5_LCDC_OVR2_MAXWIDTH > BOARD_LCD_WIDTH
#  error Width of overlay 2 exceeds the width of the display
#endif

#ifndef CONFIG_SAMA5_LCDC_OVR2_MAXHEIGHT
#  define CONFIG_SAMA5_LCDC_OVR2_MAXHEIGHT BOARD_LCD_HEIGHT
#endif

#if CONFIG_SAMA5_LCDC_OVR2_MAXHEIGHT > BOARD_LCD_HEIGHT
#  error Height of overlay 2 exceeds the height of the display
#endif

#if SAMA5_LCDC_OVR2_BPP == 16
#  define SAMA5_OVR2_STRIDE ((CONFIG_SAMA5_LCDC_OVR2_MAXWIDTH * 16 + 7) / 8)
#elif SAMA5_LCDC_OVR2_BPP == 24
#  define SAMA5_OVR2_STRIDE ((CONFIG_SAMA5_LCDC_OVR2_MAXWIDTH * 24 + 7) / 8)
#elif SAMA5_LCDC_OVR2_BPP == 32
#  define SAMA5_OVR2_STRIDE ((CONFIG_SAMA5_LCDC_OVR2_MAXWIDTH * 32 + 7) / 8)
#endif

#define SAMA5_OVR2_FBSIZE (SAMA5_OVR2_STRIDE * CONFIG_SAMA5_LCDC_OVR2_MAXHEIGHT)

#ifndef CONFIG_SAMA5_LCDC_HEO_MAXWIDTH
#  define CONFIG_SAMA5_LCDC_HEO_MAXWIDTH BOARD_LCD_WIDTH
#endif

#if CONFIG_SAMA5_LCDC_HEO_MAXWIDTH > BOARD_LCD_WIDTH
#  error Width of HEO exceeds the width of the display
#endif

#ifndef CONFIG_SAMA5_LCDC_HEO_MAXHEIGHT
#  define CONFIG_SAMA5_LCDC_HEO_MAXHEIGHT BOARD_LCD_HEIGHT
#endif

#if CONFIG_SAMA5_LCDC_HEO_MAXHEIGHT > BOARD_LCD_HEIGHT
#  error Height of HEO exceeds the height of the display
#endif

#if SAMA5_LCDC_HEO_BPP == 16
#  define SAMA5_HEO_STRIDE ((CONFIG_SAMA5_LCDC_HEO_MAXWIDTH * 16 + 7) / 8)
#elif SAMA5_LCDC_HEO_BPP == 24
#  define SAMA5_HEO_STRIDE ((CONFIG_SAMA5_LCDC_HEO_MAXWIDTH * 24 + 7) / 8)
#elif SAMA5_LCDC_HEO_BPP == 32
#  define SAMA5_HEO_STRIDE ((CONFIG_SAMA5_LCDC_HEO_MAXWIDTH * 32 + 7) / 8)
#endif

#define SAMA5_HEO_FBSIZE (SAMA5_HEO_STRIDE * CONFIG_SAMA5_LCDC_HEO_MAXHEIGHT)

#ifndef CONFIG_SAMA5_LCDC_HCR_MAXWIDTH
#  define CONFIG_SAMA5_LCDC_HCR_MAXWIDTH BOARD_LCD_WIDTH
#endif

#if CONFIG_SAMA5_LCDC_HCR_MAXWIDTH > BOARD_LCD_WIDTH
#  error Width of the hardware cursor exceeds the width of the display
#endif

#ifndef CONFIG_SAMA5_LCDC_HCR_MAXHEIGHT
#  define CONFIG_SAMA5_LCDC_HCR_MAXHEIGHT BOARD_LCD_HEIGHT
#endif

#if CONFIG_SAMA5_LCDC_HCR_MAXHEIGHT > BOARD_LCD_HEIGHT
#  error Height of the hardware cursor exceeds the height of the display
#endif

#if SAMA5_LCDC_HCR_BPP == 16
#  define SAMA5_HCR_STRIDE ((CONFIG_SAMA5_LCDC_HCR_MAXWIDTH * 16 + 7) / 8)
#elif SAMA5_LCDC_HCR_BPP == 24
#  define SAMA5_HCR_STRIDE ((CONFIG_SAMA5_LCDC_HCR_MAXWIDTH * 24 + 7) / 8)
#elif SAMA5_LCDC_HCR_BPP == 32
#  define SAMA5_HCR_STRIDE ((CONFIG_SAMA5_LCDC_HCR_MAXWIDTH * 32 + 7) / 8)
#endif

#define SAMA5_HCR_FBSIZE (SAMA5_HCR_STRIDE * CONFIG_SAMA5_LCDC_HCR_MAXHEIGHT)

/* Are size, position, and pixel stride support needed? */

#undef SAMA5_HAVE_POSITION  /* The base layer has none of these */
#undef SAMA5_HAVE_SIZE
#undef SAMA5_HAVE_PSTRIDE

#if defined(CONFIG_SAMA5_LCDC_OVR1) || defined(CONFIG_SAMA5_LCDC_OVR2) || \
    defined(CONFIG_SAMA5_LCDC_HEO)
#  define SAMA5_HAVE_POSITION 1
#  define SAMA5_HAVE_SIZE     1
#  define SAMA5_HAVE_PSTRIDE  1
#elif defined(CONFIG_SAMA5_LCDC_HCR)
#  define SAMA5_HAVE_POSITION 1
#  define SAMA5_HAVE_SIZE     1
#endif

/* Where do we get framebuffer memory */

#if defined(CONFIG_SAMA5_LCDC_FBPREALLOCATED)
#  undef CONFIG_SAMA5_LCDC_FBALLOCATED
#  undef CONFIG_SAMA5_LCDC_FBFIXED
#elif defined(CONFIG_SAMA5_LCDC_FBALLOCATED)
#  undef CONFIG_SAMA5_LCDC_FBALLOCATED
#elif defined(CONFIG_SAMA5_LCDC_FBALLOCATED)
#  define CONFIG_SAMA5_LCDC_FBALLOCATED 1
#endif

#if defined(CONFIG_SAMA5_LCDC_FBFIXED) && !defined(CONFIG_SAMA5_LCDC_FBFIXED_BASE)
#  error CONFIG_SAMA5_LCDC_FBFIXED_BASE must be defined
#endif

/* Debug */

#ifndef CONFIG_DEBUG
#  undef CONFIG_SAMA5_LCDC_REGDEBUG
#endif

/* LCDC flags */

#define LCDC_FLAG_BOTTOMUP  (1 << 0)  /* Rend bottom-up */
#define LCDC_FLAG_RIGHTLEFT (1 << 1)  /* Rend right-to-left */

/****************************************************************************
 * Private Types
 ****************************************************************************/
/* This enumeration names each layer supported by the hardware */

enum sam_layer_e
{
  LCDC_LAYER_BASE = 0,     /* LCD base layer, display fixed size image */
  LCDC_LAYER_OVR1,         /* LCD Overlay 1 */
  LCDC_LAYER_OVR2,         /* LCD Overlay 2 */
  LCDC_LAYER_HEO,          /* LCD HighEndOverlay, support resize */
  LCDC_LAYER_HCR           /* LCD Cursor, max size 128x128 */
};
#define LCDC_NLAYERS 5

/* Possible rotations supported by all layers */

enum sam_rotation_e
{
  LCDC_ROT_0 = 0,          /* LCD base layer, display fixed size image */
  LCDC_ROT_90,             /* LCD Overlay 1 */
  LCDC_ROT_180,            /* LCD Overlay 2 */
  LCDC_ROT_270             /* LCD HighEndOverlay, support resize */
};

/* LCDC General Layer information */

struct sam_layer_s
{
  /* Descriptors and buffering */

  struct sam_dscr_s *dscr; /* DMA descriptor(s) */
  uint8_t *framebuffer;    /* DMA framebuffer memory */
  uint8_t lid;             /* Layer ID (see enum sam_layer_e) */

  /* Orientation information */

  uint8_t rotation;        /* See enum_rotation_e */
  uint8_t flags;           /* See LDCD_FLAG_* definitions */

  /* Color information */

  uint8_t bpp;             /* Bits per pixel */
#ifdef CONFIG_FB_CMAP
  uint8_t offset;          /* Offset to first value entry in CLUT */
  uint8_t nclut;           /* Number of colors in the CLUT */
#endif
};

/* This describes how the base, ovr1, ovr1, and hcr layers area allocated */

struct sam_layer_alloc_s
{
  /* These is the DMA descriptors as seen by the hardware.  This descriptor
   * must be the first element of the structure and each structure instance
   * must conform to the alignment requirements of the DMA descriptor.
   */

  struct sam_dscr_s  dscr;
  struct sam_layer_s layer;
};

/* LCDC HEO Layer information */

struct sam_heolayer_alloc_s
{
  /* These are the HEO DMA descriptors as seen by the hardware.  This array
   * must be the first element of the structure and each structure instance
   * must conform to the alignment requirements of the DMA descriptor.
   */

  struct sam_dscr_s dscr[3];
  struct sam_layer_s layer;
};

/* This structure provides the overall state of the LCDC */

#if defined(CONFIG_FB_HWCURSOR) || defined(CONFIG_SAMA5_LCDC_REGDEBUG)
struct sam_lcdc_s
{
#ifdef CONFIG_FB_HWCURSOR
  struct fb_cursorpos_s cpos;     /* Current cursor position */
#ifdef CONFIG_FB_HWCURSORSIZE
  struct fb_cursorsize_s csize;   /* Current cursor size */
#endif
#endif

  /* Debug stuff */

#ifdef CONFIG_SAMA5_LCDC_REGDEBUG
   bool wrlast;                   /* True: Last access was a write */
   uintptr_t addrlast;            /* Last address accessed */
   uint32_t vallast;              /* Last value read or written */
   int ntimes;                    /* Number of consecutive accesses */
#endif
};
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
/* Register operations ******************************************************/

#if defined(CONFIG_SAMA5_LCDC_REGDEBUG) && defined(CONFIG_DEBUG)
static bool sam_checkreg(bool wr, uint32_t regval, uintptr_t address);
static uint32_t sam_getreg(uintptr_t addr);
static void sam_putreg(uintptr_t addr, uint32_t val);
#else
#  define sam_getreg(addr)      getreg32(addr)
#  define sam_putreg(addr,val)  putreg32(val,addr)
#endif

/* Frame buffer interface ***************************************************/
/* Get information about the video controller configuration and the
 * configuration of each color plane.
 */

static int sam_base_getvideoinfo(struct fb_vtable_s *vtable,
              struct fb_videoinfo_s *vinfo);
static int sam_base_getplaneinfo(struct fb_vtable_s *vtable,
              int planeno, struct fb_planeinfo_s *pinfo);

/* The following is provided only if the video hardware supports RGB color
 * mapping
 */

#ifdef CONFIG_FB_CMAP
static int sam_base_getcmap(struct fb_vtable_s *vtable,
              struct fb_cmap_s *cmap);
static int sam_base_putcmap(struct fb_vtable_s *vtable,
              const struct fb_cmap_s *cmap);
#endif

/* The following is provided only if the video hardware supports a hardware
 * cursor
 */

#ifdef CONFIG_FB_HWCURSOR
static int sam_hcr_getcursor(struct fb_vtable_s *vtable,
              struct fb_cursorattrib_s *attrib);
static int sam_hcr_setcursor(struct fb_vtable_s *vtable,
              struct fb_setcursor_s *setttings);
#endif

/* Initialization ***********************************************************/

static void sam_dmasetup(int lid, struct sam_dscr_s *dscr, uint8_t *buffer);
#if 0 /* #if defined(SAMA5_HAVE_POSITION) && defined(SAMA5_HAVE_SIZE) -- not used */
static void sam_setposition(int lid, uint32_t x, uint32_t y)
#endif
#ifdef CONFIG_FB_CMAP
static int sam_setclut(struct sam_layer_s *layer,
              const struct fb_cmap_s *cmap);
static int sam_getclut(struct sam_layer_s *layer,
              struct fb_cmap_s *cmap);
#endif

static void sam_pio_config(void);
static void sam_backlight(uint32_t level);
static void sam_base_disable(void);
static void sam_ovr1_disable(void);
static void sam_ovr2_disable(void);
static void sam_heo_disable(void);
static void sam_hcr_disable(void);
static void sam_lcd_disable(void);
static void sam_layer_orientation(void);
static void sam_layer_color(void);
static void sam_lcd_enable(void);
static int  sam_fb_allocate(void);
#ifdef CONFIG_SAMA5_LCDC_HEO
static uint32_t sam_scalefactor(uint32_t wnew, uint32_t oldw);
#endif
static void sam_show_layer(struct sam_layer_s *layer,
              uint32_t dispx, uint32_t dispy, uint32_t dispw, uint32_t disph,
              uint32_t imgw, uint32_t imgh);
static void sam_show_base(void);
#ifdef CONFIG_FB_HWCURSOR
static void sam_show_hcr(void);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This structure describes the simulated video controller */

static const struct fb_videoinfo_s g_base_videoinfo =
{
  .fmt      = SAMA5_LCDC_BASE_COLOR_FMT,
  .xres     = BOARD_LCD_WIDTH,
  .yres     = BOARD_LCD_HEIGHT,
  .nplanes  = 1,
};

/* This structure provides the overall state of the LCDC */

#if defined(CONFIG_FB_HWCURSOR) || defined(CONFIG_SAMA5_LCDC_REGDEBUG)
static struct sam_lcdc_s g_lcdc;
#endif

/* This structure provides the base layer interface */

static const struct fb_vtable_s g_base_vtable =
{
  .getvideoinfo  = sam_base_getvideoinfo,
  .getplaneinfo  = sam_base_getplaneinfo,
#ifdef CONFIG_FB_CMAP
  .getcmap       = sam_base_getcmap,
  .putcmap       = sam_base_putcmap,
#endif
#ifdef CONFIG_FB_HWCURSOR
  .getcursor     = sam_hcr_getcursor,
  .setcursor     = sam_hcr_setcursor,
#endif
};

/* Preallocated LCDC layer structures */

/* Base layer */

static struct sam_layer_alloc_s g_base __attribute__((aligned(64)));

/* Overlay 1/2 Layers */

static struct sam_layer_alloc_s g_ovr1 __attribute__((aligned(64)));
static struct sam_layer_alloc_s g_ovr2 __attribute__((aligned(64)));

/* High End Overlay (HEO) Layer */

static struct sam_heolayer_alloc_s g_heo __attribute__((aligned(64)));

/* Hardware cursor (HRC) Layer */

static struct sam_layer_alloc_s g_hcr __attribute__((aligned(64)));

/* PIO pin configurations */

static pio_pinset_t g_lcdcpins[] =
{
  PIO_LCD_DAT0,  PIO_LCD_DAT2,  PIO_LCD_DAT1,  PIO_LCD_DAT3,
  PIO_LCD_DAT4,  PIO_LCD_DAT5,  PIO_LCD_DAT6,  PIO_LCD_DAT7,

  PIO_LCD_DAT8,  PIO_LCD_DAT9,  PIO_LCD_DAT10, PIO_LCD_DAT11,
  PIO_LCD_DAT12, PIO_LCD_DAT13, PIO_LCD_DAT14, PIO_LCD_DAT15,

#if SAMA5_LCDC_BASE_BPP > 16
  PIO_LCD_DAT16, PIO_LCD_DAT17, PIO_LCD_DAT18, PIO_LCD_DAT19,
  PIO_LCD_DAT20, PIO_LCD_DAT21, PIO_LCD_DAT22, PIO_LCD_DAT23,
#endif

  PIO_LCD_PWM,   PIO_LCD_DISP,  PIO_LCD_VSYNC, PIO_LCD_HSYNC,
  PIO_LCD_PCK,   PIO_LCD_DEN
};
#define SAMA5_LCDC_NPINCONFIGS (sizeof(g_lcdcpins) / sizeof(pio_pinset_t))

/* Register lookup tables permit common logic to deal with different
 * layers.
 */

static const uintptr_t g_layerenable[LCDC_NLAYERS] =
{
  SAM_LCDC_BASECHER, SAM_LCDC_OVR1CHER, SAM_LCDC_OVR2CHER, SAM_LCDC_HEOCHER,
  SAM_LCDC_HCRCHER
};

static const uintptr_t g_layerdisable[LCDC_NLAYERS] =
{
  SAM_LCDC_BASECHDR, SAM_LCDC_OVR1CHDR, SAM_LCDC_OVR2CHDR, SAM_LCDC_HEOCHDR,
  SAM_LCDC_HCRCHDR
};

static const uintptr_t g_layerstatus[LCDC_NLAYERS] =
{
  SAM_LCDC_BASECHSR, SAM_LCDC_OVR1CHSR, SAM_LCDC_OVR2CHSR, SAM_LCDC_HEOCHSR,
  SAM_LCDC_HCRCHSR
};

static const uintptr_t g_layerblend[LCDC_NLAYERS] =
{
  SAM_LCDC_BASECFG4, SAM_LCDC_OVR1CFG9, SAM_LCDC_OVR2CFG9, SAM_LCDC_HEOCFG12,
  SAM_LCDC_HCRCFG9
};

static const uintptr_t g_layerhead[LCDC_NLAYERS] =
{
  SAM_LCDC_BASEHEAD, SAM_LCDC_OVR1HEAD, SAM_LCDC_OVR2HEAD, SAM_LCDC_HEOHEAD,
  SAM_LCDC_HCRHEAD
};

static const uintptr_t g_layeraddr[LCDC_NLAYERS] =
{
  SAM_LCDC_BASEADDR, SAM_LCDC_OVR1ADDR, SAM_LCDC_OVR2ADDR, SAM_LCDC_HEOADDR,
  SAM_LCDC_HCRADDR
};

static const uintptr_t g_layerctrl[LCDC_NLAYERS] =
{
  SAM_LCDC_BASECTRL, SAM_LCDC_OVR1CTRL, SAM_LCDC_OVR2CTRL, SAM_LCDC_HEOCTRL,
  SAM_LCDC_HCRCTRL
};

static const uintptr_t g_layernext[LCDC_NLAYERS] =
{
  SAM_LCDC_BASENEXT, SAM_LCDC_OVR1NEXT, SAM_LCDC_OVR2NEXT, SAM_LCDC_HEONEXT,
  SAM_LCDC_HCRNEXT
};

static const uintptr_t g_layercfg[LCDC_NLAYERS] =
{
  SAM_LCDC_BASECFG0, SAM_LCDC_OVR1CFG0, SAM_LCDC_OVR2CFG0, SAM_LCDC_HEOCFG0,
  SAM_LCDC_HCRCFG0
};

static const uintptr_t g_layercolor[LCDC_NLAYERS] =
{
  SAM_LCDC_BASECFG1, SAM_LCDC_OVR1CFG1, SAM_LCDC_OVR2CFG1, SAM_LCDC_HEOCFG1,
  SAM_LCDC_HCRCFG1
};

#ifdef SAMA5_HAVE_POSITION
static const uintptr_t g_layerpos[LCDC_NLAYERS] =
{
  0,                 SAM_LCDC_OVR1CFG2, SAM_LCDC_OVR2CFG2, SAM_LCDC_HEOCFG2,
  SAM_LCDC_HCRCFG2
};
#endif

#ifdef SAMA5_HAVE_SIZE
static const uintptr_t g_layersize[LCDC_NLAYERS] =
{
  0,                 SAM_LCDC_OVR1CFG3, SAM_LCDC_OVR2CFG3, SAM_LCDC_HEOCFG3,
  SAM_LCDC_HCRCFG3
};
#endif

static const uintptr_t g_layerstride[LCDC_NLAYERS] =
{
  SAM_LCDC_BASECFG2, SAM_LCDC_OVR1CFG4, SAM_LCDC_OVR2CFG4, SAM_LCDC_HEOCFG5,
  SAM_LCDC_HCRCFG4
};

#ifdef SAMA5_HAVE_PSTRIDE
static const uintptr_t g_layerpstride[LCDC_NLAYERS] =
{
  0,                 SAM_LCDC_OVR1CFG5, SAM_LCDC_OVR2CFG5, SAM_LCDC_HEOCFG6,
  0
};
#endif

#ifdef CONFIG_FB_CMAP
static const uintptr_t g_layerclut[LCDC_NLAYERS] =
{
  SAM_LCDC_BASECLUT, SAM_LCDC_OVR1CLUT, SAM_LCDC_OVR2CLUT, SAM_LCDC_HEOCLUT,
  SAM_LCDC_HCRCLUT
};
#endif

/* Framebuffer memory */

#if defined(CONFIG_SAMA5_LCDC_FBPREALLOCATED)
static const uint8_t g_basefb[SAMA5_BASE_FBSIZE];

#  ifdef CONFIG_SAMA5_LCDC_OVR1
static const uint8_t g_ovr1fb[SAMA5_OVR1_FBSIZE];
#  endif

#  ifdef CONFIG_SAMA5_LCDC_OVR2
static const uint8_t g_ovr2fb[SAMA5_OVR2_FBSIZE];
#  endif

#  ifdef CONFIG_SAMA5_LCDC_HEO
static const uint8_t g_heofb[SAMA5_HEO_FBSIZE];
#  endif

#  ifdef CONFIG_FB_HWCURSOR
static const uint8_t g_hcrfb[SAMA5_HCR_FBSIZE];
#  endif

#elif defined(CONFIG_SAMA5_LCDC_FBFIXED)

#  define SAMA5_LCDC_BUFFER_BASE   CONFIG_SAMA5_LCDC_FBFIXED_BASE
#  define SAMA5_LCDC_ENDBUF_BASE   (CONFIG_SAMA5_LCDC_FBFIXED_BASE + SAMA5_BASE_FBSIZE)

#  ifdef CONFIG_SAMA5_LCDC_OVR1
#    define SAMA5_LCDC_BUFFER_OVR1 SAMA5_LCDC_ENDBUF_BASE
#    define SAMA5_LCDC_ENDBUF_OVR1 (SAMA5_LCDC_ENDBUF_BASE + SAMA5_OVR1_FBSIZE)
#  else
#    define SAMA5_LCDC_ENDBUF_OVR1 SAMA5_LCDC_ENDBUF_BASE
#  endif

#  ifdef CONFIG_SAMA5_LCDC_OVR2
#    define SAMA5_LCDC_BUFFER_OVR2 SAMA5_LCDC_ENDBUF_OVR1
#    define SAMA5_LCDC_ENDBUF_OVR2 (SAMA5_LCDC_ENDBUF_OVR1 + SAMA5_OVR2_FBSIZE)
#  else
#    define SAMA5_LCDC_ENDBUF_OVR2 SAMA5_LCDC_ENDBUF_OVR1
#  endif

#  ifdef CONFIG_SAMA5_LCDC_HEO
#    define SAMA5_LCDC_BUFFER_HEO  SAMA5_LCDC_ENDBUF_OVR2
#    define SAMA5_LCDC_ENDBUF_HEO  (SAMA5_LCDC_ENDBUF_OVR2 + SAMA5_HEO_FBSIZE)
#  else
#    define SAMA5_LCDC_ENDBUF_HEO  SAMA5_LCDC_ENDBUF_OVR2
#  endif

#  ifdef CONFIG_FB_HWCURSOR
#    define SAMA5_LCDC_BUFFER_HCR  SAMA5_LCDC_ENDBUF_HEO
#    define SAMA5_LCDC_ENDBUF_HCR  (SAMA5_LCDC_ENDBUF_HEO + SAMA5_HCR_FBSIZE)
#  else
#    define SAMA5_LCDC_ENDBUF_HCR  SAMA5_LCDC_ENDBUF_HEO
#  endif

#  ifdef CONFIG_SAMA5_LCDC_FBFIXED_SIZE
#    if SAMA5_LCDC_ENDBUF_HCR > \
        (CONFIG_SAMA5_LCDC_FBFIXED_BASE + CONFIG_SAMA5_LCDC_FBFIXED_SIZE)
#      error Fixed memory allocation not large enough
#    endif
#  endif
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_checkreg
 *
 * Description:
 *   Check if the current register access is a duplicate of the preceding.
 *
 * Input Parameters:
 *   regval  - The value to be written
 *   address - The address of the register to write to
 *
 * Returned Value:
 *   true:  This is the first register access of this type.
 *   flase: This is the same as the preceding register access.
 *
 ****************************************************************************/

#ifdef CONFIG_SAMA5_LCDC_REGDEBUG
static bool sam_checkreg(bool wr, uint32_t regval, uintptr_t address)
{
  if (wr      == g_lcdc.wrlast &&   /* Same kind of access? */
      regval  == g_lcdc.vallast &&  /* Same value? */
      address == g_lcdc.addrlast)   /* Same address? */
    {
      /* Yes, then just keep a count of the number of times we did this. */

      g_lcdc.ntimes++;
      return false;
    }
  else
    {
      /* Did we do the previous operation more than once? */

      if (g_lcdc.ntimes > 0)
        {
          /* Yes... show how many times we did it */

          lldbg("...[Repeats %d times]...\n", g_lcdc.ntimes);
        }

      /* Save information about the new access */

      g_lcdc.wrlast   = wr;
      g_lcdc.vallast  = regval;
      g_lcdc.addrlast = address;
      g_lcdc.ntimes   = 0;
    }

  /* Return true if this is the first time that we have done this operation */

  return true;
}
#endif

/****************************************************************************
 * Name: sam_getreg
 *
 * Description:
 *  Read any 32-bit register using an absolute
 *
 ****************************************************************************/

#ifdef CONFIG_SAMA5_LCDC_REGDEBUG
static uint32_t sam_getreg(uintptr_t address)
{
  uint32_t regval = getreg32(address);

  if (sam_checkreg(false, regval, address))
    {
      lldbg("%08x->%08x\n", address, regval);
    }

  return regval;
}
#endif

/****************************************************************************
 * Name: sam_putreg
 *
 * Description:
 *  Write to any 32-bit register using an absolute address
 *
 ****************************************************************************/

#ifdef CONFIG_SAMA5_LCDC_REGDEBUG
static void sam_putreg(uintptr_t address, uint32_t regval)
{
  if (sam_checkreg(true, regval, address))
    {
      lldbg("%08x<-%08x\n", address, regval);
    }

  putreg32(regval, address);
}
#endif

/****************************************************************************
 * Name: sam_base_getvideoinfo
 ****************************************************************************/

static int sam_base_getvideoinfo(struct fb_vtable_s *vtable,
                                 struct fb_videoinfo_s *vinfo)
{
  gvdbg("vtable=%p vinfo=%p\n", vtable, vinfo);
  if (vtable && vinfo)
    {
      memcpy(vinfo, &g_base_videoinfo, sizeof(struct fb_videoinfo_s));
      return OK;
    }

  gdbg("ERROR: Returning EINVAL\n");
  return -EINVAL;
}

/****************************************************************************
 * Name: sam_base_getplaneinfo
 ****************************************************************************/

static int sam_base_getplaneinfo(struct fb_vtable_s *vtable, int planeno,
                                 struct fb_planeinfo_s *pinfo)
{
  gvdbg("vtable=%p planeno=%d pinfo=%p\n", vtable, planeno, pinfo);
  if (vtable && planeno == 0 && pinfo)
    {
      pinfo->fbmem  = (void *)g_base.layer.framebuffer;
      pinfo->fblen  = SAMA5_BASE_FBSIZE;
      pinfo->stride = SAMA5_BASE_STRIDE,
      pinfo->bpp    = g_base.layer.bpp;
      return OK;
    }

  gdbg("Returning EINVAL\n");
  return -EINVAL;
}

/****************************************************************************
 * Name: sam_base_getcmap
 ****************************************************************************/

#ifdef CONFIG_FB_CMAP
static int sam_base_getcmap(struct fb_vtable_s *vtable,
                            struct fb_cmap_s *cmap)
{
  return sam_getclut(&g_base.layer, cmap);
}
#endif

/****************************************************************************
 * Name: sam_base_putcmap
 ****************************************************************************/

#ifdef CONFIG_FB_CMAP
static int sam_base_putcmap(struct fb_vtable_s *vtable,
                            const struct fb_cmap_s *cmap)
{
  return sam_setclut(&g_base.layer, cmap);
}
#endif

/****************************************************************************
 * Name: sam_hcr_getcursor
 ****************************************************************************/

#ifdef CONFIG_FB_HWCURSOR
static int sam_hcr_getcursor(struct fb_vtable_s *vtable,
                             struct fb_cursorattrib_s *attrib)
{
  gvdbg("vtable=%p attrib=%p\n", vtable, attrib);
  if (vtable && attrib)
    {
#ifdef CONFIG_FB_HWCURSORIMAGE
      attrib->fmt = SAMA5_HCR_COLOR_FMT;
#endif

      gvdbg("pos: (x=%d, y=%d)\n", g_lcdc.cpos.x, g_lcdc.cpos.y);
      attrib->pos = g_lcdc.cpos;

#ifdef CONFIG_FB_HWCURSORSIZE
      attrib->mxsize.h = CONFIG_SAMA5_LCDC_HCR_HEIGHT;
      attrib->mxsize.w = CONFIG_SAMA5_LCDC_HCR_WIDTH;

      gvdbg("size: (h=%d, w=%d)\n", g_lcdc.csize.h, g_lcdc.csize.w);
      attrib->size = g_lcdc.csize;
#endif
      return OK;
    }

  gdbg("Returning EINVAL\n");
  return -EINVAL;
}
#endif

/****************************************************************************
 * Name: sam_hcr_setcursor
 ****************************************************************************/

#ifdef CONFIG_FB_HWCURSOR
static int sam_hcr_setcursor(struct fb_vtable_s *vtable,
                             struct fb_setcursor_s *setttings)
{
  gvdbg("vtable=%p setttings=%p\n", vtable, setttings);
  if (vtable && setttings)
    {
      gvdbg("flags: %02x\n", settings->flags);
      if ((flags & FB_CUR_SETPOSITION) != 0)
        {
          g_lcdc.cpos = settings->pos;
          gvdbg("pos: (h:%d, w:%d)\n", g_lcdc.cpos.x, g_lcdc.cpos.y);
        }
#ifdef CONFIG_FB_HWCURSORSIZE
      if ((flags & FB_CUR_SETSIZE) != 0)
        {
          g_lcdc.csize = settings->size;
          gvdbg("size: (h:%d, w:%d)\n", g_lcdc.csize.h, g_lcdc.csize.w);
        }
#endif
#ifdef CONFIG_FB_HWCURSORIMAGE
      if ((flags & FB_CUR_SETIMAGE) != 0)
        {
          gvdbg("image: (h:%d, w:%d) @ %p\n",
                settings->img.height, settings->img.width,
                settings->img.image);
        }
#endif
      return OK;
    }

  gdbg("Returning EINVAL\n");
  return -EINVAL;
}
#endif

/****************************************************************************
 * Name: sam_dmasetup
 *
 * Description:
 *   Configure the channel DMA
 *
 ****************************************************************************/

static void sam_dmasetup(int lid, struct sam_dscr_s *dscr, uint8_t *buffer)
{
  uintptr_t physaddr;

  /* 2. Write the channel descriptor (DSCR) structure in the system memory by
   *    writing DSCR.CHXADDR Frame base address, DSCR.CHXCTRL channel control
   *    and DSCR.CHXNEXT next descriptor location.
   * 3. If more than one descriptor is expected, the DFETCH field of
   *    DSCR.CHXCTRL is set to one to enable the descriptor fetch operation.
   * 4. Write the DSCR.CHXNEXT register with the address location of the
   *    descriptor structure and set DFETCH field of the DSCR.CHXCTRL register
   *    to one.
   */

  /* Modify descriptor */

  dscr->addr = (uint32_t)buffer;
  dscr->ctrl = LCDC_BASECTRL_DFETCH;
  dscr->next = (uint32_t)dscr;

  /* Flush the modified descriptor to RAM */

  cp15_clean_dcache((uintptr_t)dscr,
                    (uintptr_t)dscr + sizeof(struct sam_dscr_s));

  /* Modify registers */

  physaddr = sam_physramaddr((uint32_t)buffer);
  sam_putreg(g_layerhead[lid], physaddr);

  sam_putreg(g_layerctrl[lid], LCDC_BASECTRL_DFETCH);

  physaddr = sam_physramaddr((uint32_t)dscr);
  sam_putreg(g_layernext[lid], physaddr);
}

/****************************************************************************
 * Name: sam_setposition
 *
 * Description:
 *   Set the new position of a move-able layer (any layer except the base
 *   layer).
 *
 ****************************************************************************/

#if 0 /* #if defined(SAMA5_HAVE_POSITION) && defined(SAMA5_HAVE_SIZE) -- not used */
static void sam_setposition(int lid, uint32_t x, uint32_t y)
{
  uintptr_t regaddr;
  uintptr_t regval;
  uint32_t h;
  uint32_t w;

  regaddr = g_layersize[lid];
  if (regaddr)
    {
      /* Get the layer size */

      regval = sam_getreg(regaddr);
      w = (regval & LCDC_OVR1CFG3_XSIZE_MASK) >> LCDC_OVR1CFG3_XSIZE_SHIFT;
      h = (regval & LCDC_OVR1CFG3_YSIZE_MASK) >> LCDC_OVR1CFG3_YSIZE_SHIFT;

      /* Clip the position so that the window lies on the physical display */

      if (x + w >= BOARD_LCD_WIDTH)
        {
          x = BOARD_LCD_WIDTH - w;
        }

      if (y + h >= BOARD_LCD_HEIGHT)
        {
          y = BOARD_LCD_HEIGHT - h;
        }

      /* Set the new position of the layer */

      regaddr = g_layerpos[lid];
      if (regaddr)
        {
          sam_putreg(regaddr, LCDC_OVR1CFG2_XPOS(x) | LCDC_OVR1CFG2_YPOS(y));

          /* If the channel is enabled, then update the layer */

          regaddr = g_layerstatus[lid];
          if ((sam_getreg(regaddr) & LCDC_OVR1CHSR_CH) != 0)
            {
              regaddr = g_layerenable[lid];
              sam_putreg(regaddr, LCDC_OVR1CHER_UPDATE);
            }
        }
    }
}
#endif

/****************************************************************************
 * Name: sam_setclut
 *
 * Description:
 *   Set a range of CLUT values for any layer
 *
 ****************************************************************************/

#ifdef CONFIG_FB_CMAP
static int sam_setclut(struct sam_layer_s *layer,
                       const struct fb_cmap_s *cmap)
{
  uintptr_t regaddr;
  uint32_t offset;
  uint32_t rgb;
  unsigned int len;
  unsigned int end;
  int i;

  gvdbg("layer=%d cmap=%p first=%d len=%d\n",
        layer->lid, cmap, cmap->first, cmap->len);

  DEBUGASSERT(layer && cmap);

  /* Get and verify the range of CLUT entries to modify */

  offset = (uint32_t)cmap->first;
  len    = (unsigned int)cmap->len;

  if (offset >= SAM_LCDC_NCLUT)
    {
      gdbg("ERROR: CLUT offset is out of range: %d\n", offset);
      return -EINVAL;
    }

  if (offset + len > SAM_LCDC_NCLUT)
    {
      len = SAM_LCDC_NCLUT - offset;
    }

  /* Update the valid range of CLUT entries */

  if (offset < layer->offset)
    {
      layer->offset = offset;
    }

  end = offset + len;
  if (end > (layer->offset + layer->nclut))
    {
      layer->nclut = end - layer->offset;offset
    }

  /* Get the offset address to the first CLUT entry to modify */

  regaddr = g_layerclut[layer->lid] + offset << 2;

  /* Then set the number of CLUT entries beginning at this offset */

  for (i = 0; i < len; i++)
    {
      /* Pack the RGB (+transparency?) values as required */

      rgb = (uint32_t)cmap->red[i]   << LCDC_BASECLUT_RCLUT_SHIFT |
            (uint32_t)cmap->green[i] << LCDC_BASECLUT_GCLUT_SHIFT |
            (uint32_t)cmap->blue[i]  << LCDC_BASECLUT_BCLUT_SHIFT;

#ifdef CONFIG_FB_TRANSPARENCY
      if (camp->transp)
        {
          rgb |= (uint32_t)cmap->transp[i] << LCDC_OVR1CLUT_ACLUT_SHIFT;
        }
#endif

      /* And write to the CLUT register */

      sam_putreg(regaddr, clut[i]);
      regaddr += sizeof(uint32_t);
    }

  return OK;
}
#endif

/****************************************************************************
 * Name: sam_getclut
 *
 * Description:
 *   Get a range of CLUT values for any layer
 *
 ****************************************************************************/

#ifdef CONFIG_FB_CMAP
static int sam_getclut(struct sam_layer_s *layer,
                       struct fb_cmap_s *cmap)
{
  uintptr_t regaddr;
  uintptr_t regval;
  int i;

  gvdbg("layer=%d cmap=%p first=%d len=%d\n",
        layer->lid, cmap, layer->offset, layer->nclut);

  DEBUGASSERT(layer && cmap);

  /* Return the range of CLUT entries to modify */

  cmap->first = layer->offset;
  cmap->len   = layer->nclut;

  /* Get the offset address to the first CLUT entry to modify */

  regaddr = g_layerclut[layer->lid] + (uint32_t)cmap->first << 2;

  /* Then set the number of CLUT entries beginning at this offset */

  for (i = 0; i < (int)cmap->len; i++)
    {
      /* Read the CLUT entry */

      regval = getreg(regaddr);
      regaddr += sizeof(uint32_t);

      /* Unpack and return the RGB (+transparency?) values as required */

      cmap->red[i] = (uint8_t)
        (regval & LCDC_BASECLUT_RCLUT_MASK) << LCDC_BASECLUT_RCLUT_SHIFT;
      cmap->green[i] = (uint8_t)
        (regval & LCDC_BASECLUT_GCLUT_MASK) << LCDC_BASECLUT_GCLUT_SHIFT;
      cmap->blue[i] = (uint8_t)
        (regval & LCDC_BASECLUT_GCLUT_MASK) << LCDC_BASECLUT_BCLUT_SHIFT;

#ifdef CONFIG_FB_TRANSPARENCY
      cmap->transp[i] = (uint8_t)
        (regval & LCDC_OVR1CLUT_ACLUT_MASK) << LCDC_OVR1CLUT_ACLUT_SHIFT;
#endif
    }

  return OK;
}
#endif

/****************************************************************************
 * Name: sam_pio_config
 *
 * Description:
 *   Configure PIO pins for use with the LCDC
 *
 ****************************************************************************/

static void sam_pio_config(void)
{
  int i;

  gvdbg("Configuring pins\n");

  /* Configure each pin */

  for (i = 0; i < SAMA5_LCDC_NPINCONFIGS; i++)
    {
      sam_configpio(g_lcdcpins[i]);
    }
}

/****************************************************************************
 * Name: sam_backlight
 *
 * Description:
 *   Set the backlight level
 *
 ****************************************************************************/

static void sam_backlight(uint32_t level)
{
  uint32_t regval;

  /* Are we turning the backlight off? */

  if (level == SAMA5_LCDC_BACKLIGHT_OFF)
    {
      /* Disable the backlight */

      sam_putreg(SAM_LCDC_LCDDIS, LCDC_LCDDIS_PWM);

      /* And wait for the PWM to be disabled */

      while ((sam_getreg(SAM_LCDC_LCDSR) & LCDC_LCDSR_PWM) != 0);
    }
#ifdef CONFIG_SAMA5_LCDC_BACKLIGHT
  else
    {
      /* Set the backight level */

      regval = sam_getreg(SAM_LCDC_LCDCFG6);
      regval &= ~LCDC_LCDCFG6_PWMCVAL_MASK;
      regval |=  LCDC_LCDCFG6_PWMCVAL(level);
      sam_putreg(SAM_LCDC_LCDCFG6, regval);

      /* Enable the backlight */

      sam_putreg(SAM_LCDC_LCDEN, LCDC_LCDEN_PWM);
    }
#endif
}

/****************************************************************************
 * Name: sam_base_disable
 *
 * Description:
 *   Disable the base layer
 *
 ****************************************************************************/

static void sam_base_disable(void)
{
  uintptr_t physaddr;
  uintptr_t dscr;
  uint32_t regval;

  /* 1. Clear the DFETCH bit in the DSCR.CHXCTRL field of the DSCR structure
   *    will disable the channel at the end of the frame.
   */

  g_base.dscr.ctrl &= ~LCDC_BASECTRL_DFETCH;

  regval = sam_getreg(SAM_LCDC_BASECTRL);
  regval &= ~LCDC_BASECTRL_DFETCH;
  sam_putreg(SAM_LCDC_BASECTRL, regval);

  /* 2. Set the DSCR.CHXNEXT field of the DSCR structure will disable the
   *    channel at the end of the frame.
   */

  dscr             = (uintptr_t)&g_base.dscr;
  physaddr         = sam_physramaddr(dscr);
  g_base.dscr.next = physaddr;

  sam_putreg(SAM_LCDC_BASENEXT, physaddr);

  /* Flush the modified DMA descriptor to RAM */

  cp15_clean_dcache(dscr, dscr + sizeof(struct sam_dscr_s));

  /* 3. Writing one to the CHDIS field of the CHXCHDR register will disable
   *    the channel at the end of the frame.
   */

  sam_putreg(SAM_LCDC_BASECHDR, LCDC_BASECHDR_CH);

  /* 4. Writing one to the CHRST field of the CHXCHDR register will disable
   *    the channel immediately. This may occur in the middle of the image.
   */

  /* 5. Poll CHSR field in the CHXCHSR register until the channel is
   *    successfully disabled.
   */

  while ((sam_getreg(SAM_LCDC_BASECHSR) & LCDC_BASECHSR_CH) != 0);
}

/****************************************************************************
 * Name: sam_ovr1_disable
 *
 * Description:
 *   Disable the overlay 1 layer
 *
 ****************************************************************************/

static void sam_ovr1_disable(void)
{
  uintptr_t physaddr;
  uintptr_t dscr;
  uint32_t regval;

  /* 1. Clear the DFETCH bit in the DSCR.CHXCTRL field of the DSCR structure
   *    will disable the channel at the end of the frame.
   */

  g_ovr1.dscr.ctrl &= ~LCDC_OVR1CTRL_DFETCH;

  regval = sam_getreg(SAM_LCDC_OVR1CTRL);
  regval &= ~LCDC_OVR1CTRL_DFETCH;
  sam_putreg(SAM_LCDC_OVR1CTRL, regval);

  /* 2. Set the DSCR.CHXNEXT field of the DSCR structure will disable the
   *    channel at the end of the frame.
   */

  dscr             = (uintptr_t)&g_ovr1.dscr;
  physaddr         = sam_physramaddr(dscr);
  g_ovr1.dscr.next = physaddr;

  sam_putreg(SAM_LCDC_OVR1NEXT, physaddr);

  /* Flush the modified DMA descriptor to RAM */

  cp15_clean_dcache(dscr, dscr + sizeof(struct sam_dscr_s));

  /* 3. Writing one to the CHDIS field of the CHXCHDR register will disable
   *    the channel at the end of the frame.
   */

  sam_putreg(SAM_LCDC_OVR1CHDR, LCDC_OVR1CHDR_CH);

  /* 4. Writing one to the CHRST field of the CHXCHDR register will disable
   *    the channel immediately. This may occur in the middle of the image.
   */

  /* 5. Poll CHSR field in the CHXCHSR register until the channel is
   *    successfully disabled.
   */

  while ((sam_getreg(SAM_LCDC_OVR1CHSR) & LCDC_OVR1CHSR_CH) != 0);
}

/****************************************************************************
 * Name: sam_ovr2_disable
 *
 * Description:
 *   Disable the overlay 2 layer
 *
 ****************************************************************************/

static void sam_ovr2_disable(void)
{
  uintptr_t physaddr;
  uintptr_t dscr;
  uint32_t regval;

  /* 1. Clear the DFETCH bit in the DSCR.CHXCTRL field of the DSCR structure
   *    will disable the channel at the end of the frame.
   */

  g_ovr2.dscr.ctrl &= ~LCDC_OVR2CTRL_DFETCH;

  regval = sam_getreg(SAM_LCDC_OVR2CTRL);
  regval &= ~LCDC_OVR2CTRL_DFETCH;
  sam_putreg(SAM_LCDC_OVR2CTRL, regval);

  /* 2. Set the DSCR.CHXNEXT field of the DSCR structure will disable the
   *    channel at the end of the frame.
   */

  dscr             = (uintptr_t)&g_ovr1.dscr;
  physaddr         = sam_physramaddr(dscr);
  g_ovr2.dscr.next = physaddr;

  sam_putreg(SAM_LCDC_OVR2NEXT, physaddr);

  /* Flush the modified DMA descriptor to RAM */

  cp15_clean_dcache(dscr, dscr + sizeof(struct sam_dscr_s));

  /* 3. Writing one to the CHDIS field of the CHXCHDR register will disable
   *    the channel at the end of the frame.
   */

  sam_putreg(SAM_LCDC_OVR2CHDR, LCDC_OVR2CHDR_CH);

  /* 4. Writing one to the CHRST field of the CHXCHDR register will disable
   *    the channel immediately. This may occur in the middle of the image.
   */

  /* 5. Poll CHSR field in the CHXCHSR register until the channel is
   *    successfully disabled.
   */

  while ((sam_getreg(SAM_LCDC_OVR2CHSR) & LCDC_OVR2CHSR_CH) != 0);
}

/****************************************************************************
 * Name: sam_heo_disable
 *
 * Description:
 *   Disable the High End Overlay (HEO) layer
 *
 ****************************************************************************/

static void sam_heo_disable(void)
{
  uintptr_t physaddr;
  uint32_t regval;

  /* 1. Clear the DFETCH bit in the DSCR.CHXCTRL field of the DSCR structure
   *    will disable the channel at the end of the frame.
   */

  g_heo.dscr[0].ctrl &= ~LCDC_HEOCTRL_DFETCH;
  g_heo.dscr[1].ctrl &= ~LCDC_HEOUCTRL_DFETCH;
  g_heo.dscr[2].ctrl &= ~LCDC_HEOVCTRL_DFETCH;

  regval = sam_getreg(SAM_LCDC_HEOCTRL);
  regval &= ~LCDC_HEOCTRL_DFETCH;
  sam_putreg(SAM_LCDC_HEOCTRL, regval);

  regval = sam_getreg(SAM_LCDC_HEOUCTRL);
  regval &= ~LCDC_HEOUCTRL_DFETCH;
  sam_putreg(SAM_LCDC_HEOUCTRL, regval);

  regval = sam_getreg(SAM_LCDC_HEOVCTRL);
  regval &= ~LCDC_HEOVCTRL_DFETCH;
  sam_putreg(SAM_LCDC_HEOVCTRL, regval);

  /* 2. Set the DSCR.CHXNEXT field of the DSCR structure will disable the
   *    channel at the end of the frame.
   */

  physaddr = sam_physramaddr((uintptr_t)&g_heo.dscr[0]);
  g_heo.dscr[0].next = physaddr;
  sam_putreg(SAM_LCDC_HEONEXT, physaddr);

  physaddr = sam_physramaddr((uintptr_t)&g_heo.dscr[1]);
  g_heo.dscr[1].next = physaddr;
  sam_putreg(SAM_LCDC_HEOUNEXT, physaddr);

  physaddr = sam_physramaddr((uintptr_t)&g_heo.dscr[2]);
  g_heo.dscr[2].next = physaddr;
  sam_putreg(SAM_LCDC_HEOVNEXT, physaddr);

  /* Flush the modified DMA descriptors to RAM */

  cp15_clean_dcache((uintptr_t)g_heo.dscr,
                    (uintptr_t)g_heo.dscr + sizeof(struct sam_dscr_s));

  /* 3. Writing one to the CHDIS field of the CHXCHDR register will disable
   *    the channel at the end of the frame.
   */

  sam_putreg(SAM_LCDC_HEOCHDR, LCDC_HEOCHDR_CH);

  /* 4. Writing one to the CHRST field of the CHXCHDR register will disable
   *    the channel immediately. This may occur in the middle of the image.
   */

  /* 5. Poll CHSR field in the CHXCHSR register until the channel is
   *    successfully disabled.
   */

  while ((sam_getreg(SAM_LCDC_HEOCHSR) & LCDC_HEOCHSR_CH) != 0);
}

/****************************************************************************
 * Name: sam_hcr_disable
 *
 * Description:
 *   Disable the Hardware Cursor Channel (HCR) layer
 *
 ****************************************************************************/

static void sam_hcr_disable(void)
{
  uintptr_t physaddr;
  uintptr_t dscr;
  uint32_t regval;

  /* 1. Clear the DFETCH bit in the DSCR.CHXCTRL field of the DSCR structure
   *    will disable the channel at the end of the frame.
   */

  g_hcr.dscr.ctrl &= ~LCDC_HCRCTRL_DFETCH;

  regval = sam_getreg(SAM_LCDC_HCRCTRL);
  regval &= ~LCDC_HCRCTRL_DFETCH;
  sam_putreg(SAM_LCDC_HCRCTRL, regval);

  /* 2. Set the DSCR.CHXNEXT field of the DSCR structure will disable the
   *    channel at the end of the frame.
   */

  dscr            = (uintptr_t)&g_hcr.dscr;
  physaddr        = sam_physramaddr(dscr);
  g_hcr.dscr.next = physaddr;

  sam_putreg(SAM_LCDC_HCRNEXT, physaddr);

  /* Flush the modified DMA descriptor to RAM */

  cp15_clean_dcache(dscr, dscr + sizeof(struct sam_dscr_s));

  /* 3. Writing one to the CHDIS field of the CHXCHDR register will disable
   *    the channel at the end of the frame.
   */

  sam_putreg(SAM_LCDC_HCRCHDR, LCDC_HCRCHDR_CH);

  /* 4. Writing one to the CHRST field of the CHXCHDR register will disable
   *    the channel immediately. This may occur in the middle of the image.
   */

  /* 5. Poll CHSR field in the CHXCHSR register until the channel is
   *    successfully disabled.
   */

  while ((sam_getreg(SAM_LCDC_HCRCHSR) & LCDC_HCRCHSR_CH) != 0);
}

/****************************************************************************
 * Name: sam_lcd_disable
 *
 * Description:
 *   Disable the LCD peripheral
 *
 ****************************************************************************/

static void sam_lcd_disable(void)
{
  /* Disable layers */

  sam_base_disable();
  sam_ovr1_disable();
  sam_ovr2_disable();
  sam_heo_disable();
  sam_hcr_disable();

  /* Disable DMA path */

  sam_putreg(SAM_LCDC_BASECFG4, 0);

  /* Turn off the back light */

  sam_backlight(SAMA5_LCDC_BACKLIGHT_OFF);

  /* Timing Engine Power Down Software Operation */

  /* 1. Disable the DISP signal writing DISPDIS field of the LCDC_LCDDIS
   *    register.
   */

  sam_putreg(SAM_LCDC_LCDDIS, LCDC_LCDDIS_DISP);

  /* 2. Poll DISPSTS field of the LCDC_LCDSR register to verify that the DISP
   *    is no longer activated.
   */

  while ((sam_getreg(SAM_LCDC_LCDSR) & LCDC_LCDSR_DISP) != 0);

  /* 3. Disable the hsync and vsync signals by writing one to SYNCDIS field of
   *    the LCDC_LCDDIS register.
   */

  sam_putreg(SAM_LCDC_LCDDIS, LCDC_LCDDIS_SYNC);

  /* 4. Poll LCDSTS field of the LCDC_LCDSR register to check that the
   *    synchronization is off.
   */

  while ((sam_getreg(SAM_LCDC_LCDSR) & LCDC_LCDSR_LCD) != 0);

  /* 5. Disable the Pixel clock by writing one in the CLKDIS field of the
   *    LCDC_LCDDIS register.
   */

  sam_putreg(SAM_LCDC_LCDDIS, LCDC_LCDDIS_CLK);

  /* 6. Poll CLKSTS field of the LCDC_CLKSR register to check that Pixel Clock
   *    is disabled.
   */

  while ((sam_getreg(SAM_LCDC_LCDSR) & LCDC_LCDSR_CLK) != 0);

  /* Disable peripheral clock */

  sam_lcdc_disableclk();

  /* Disable the LCD clock */

  sam_putreg(SAM_PMC_SCDR, PMC_LCDCK);
}

/****************************************************************************
 * Name: sam_layer_orientation
 *
 * Description:
 *   Configure LCDC layer orientation
 *
 ****************************************************************************/

static void sam_layer_orientation(void)
{
  /* Base channel orientation */

  g_base.layer.flags = 0;

#if defined(CONFIG_SAMA5_LCDC_BASE_ROT90)
  g_base.layer.rotation = LCDC_ROT_90;
#elif defined(CONFIG_SAMA5_LCDC_BASE_ROT180)
  g_base.layer.rotation = LCDC_ROT_180;
#elif defined(CONFIG_SAMA5_LCDC_BASE_ROT270)
  g_base.layer.rotation = LCDC_ROT_270;
#else
  g_base.layer.rotation = LCDC_ROT_0;
#endif

#ifdef CONFIG_SAMA5_LCDC_OVR1
  /* Overlay 1 orientation */

  g_ovr1.layer.flags = 0;
#ifdef CONFIG_SAMA5_LCDC_OVR1_BOTTOMUP
  g_ovr1.layer.flags |= LCDC_FLAG_BOTTOMUP;
#endif
#ifdef CONFIG_SAMA5_LCDC_OVR1_RIGHTLEFT
  g_ovr1.layer.flags |= LCDC_FLAG_RIGHTLEFT;
#endif

#if defined(CONFIG_SAMA5_LCDC_OVR1_ROT90)
  g_ovr1.layer.rotation = LCDC_ROT_90;
#elif defined(CONFIG_SAMA5_LCDC_OVR1_ROT180)
  g_ovr1.layer.rotation = LCDC_ROT_180;
#elif defined(CONFIG_SAMA5_LCDC_OVR1_ROT270)
  g_ovr1.layer.rotation = LCDC_ROT_270;
#else
  g_ovr1.layer.rotation = LCDC_ROT_0;
#endif
#endif

#ifdef CONFIG_SAMA5_LCDC_OVR2
  /* Overlay 2 orientation */

  g_ovr2.layer.flags = 0;
#ifdef CONFIG_SAMA5_LCDC_OVR2_BOTTOMUP
  g_ovr1.layer.flags |= LCDC_FLAG_BOTTOMUP;
#endif
#ifdef CONFIG_SAMA5_LCDC_OVR2_RIGHTLEFT
  g_ovr1.layer.flags |= LCDC_FLAG_RIGHTLEFT;
#endif

#if defined(CONFIG_SAMA5_LCDC_OVR2_ROT90)
  g_ovr2.layer.rotation = LCDC_ROT_90;
#elif defined(CONFIG_SAMA5_LCDC_OVR2_ROT180)
  g_ovr2.layer.rotation = LCDC_ROT_180;
#elif defined(CONFIG_SAMA5_LCDC_OVR2_ROT270)
  g_ovr2.layer.rotation = LCDC_ROT_270;
#else
  g_ovr2.layer.rotation = LCDC_ROT_0;
#endif
#endif

#ifdef CONFIG_SAMA5_LCDC_HEO
  /* High End Overlay orientation */

  g_heo.layer.flags = 0;
#ifdef CONFIG_SAMA5_LCDC_HEO_BOTTOMUP
  g_ovr1.layer.flags |= LCDC_FLAG_BOTTOMUP;
#endif
#ifdef CONFIG_SAMA5_LCDC_HEO_RIGHTLEFT
  g_ovr1.layer.flags |= LCDC_FLAG_RIGHTLEFT;
#endif

#if defined(CONFIG_SAMA5_LCDC_HEO_ROT90)
  g_heo.layer.rotation = LCDC_ROT_90;
#elif defined(CONFIG_SAMA5_LCDC_HEO_ROT180)
  g_heo.layer.rotation = LCDC_ROT_180;
#elif defined(CONFIG_SAMA5_LCDC_HEO_ROT270)
  g_heo.layer.rotation = LCDC_ROT_270;
#else
  g_heo.layer.rotation = LCDC_ROT_0;
#endif
#endif

#ifdef CONFIG_FB_HWCURSOR
  /* Hardware Cursor orientation */

  g_hcr.layer.flags = 0;

#if defined(CONFIG_SAMA5_LCDC_HCR_ROT90)
  g_hcr.layer.rotation = LCDC_ROT_90;
#elif defined(CONFIG_SAMA5_LCDC_HCR_ROT180)
  g_hcr.layer.rotation = LCDC_ROT_180;
#elif defined(CONFIG_SAMA5_LCDC_HCR_ROT270)
  g_hcr.layer.rotation = LCDC_ROT_270;
#else
  g_hcr.layer.rotation = LCDC_ROT_0;
#endif
#endif
}

/****************************************************************************
 * Name: sam_layer_color
 *
 * Description:
 *   Configure LCDC layer color mode
 *
 ****************************************************************************/

static void sam_layer_color(void)
{
  /* Mark the CLUTs as empty */

#ifdef CONFIG_FB_CMAP
  g_base.layer.offset = SAM_LCDC_NCLUT - 1;
  g_ovr1.layer.offset = SAM_LCDC_NCLUT - 1;
  g_ovr2.layer.offset = SAM_LCDC_NCLUT - 1;
  g_heo.layer.offset  = SAM_LCDC_NCLUT - 1;
  g_hcr.layer.offset  = SAM_LCDC_NCLUT - 1;
#endif

  /* Base channel color configuration */

#ifdef CONFIG_SAMA5_LCDC_BASE_RGB888P
  g_base.layer.bpp = 24;

  sam_putreg(SAM_LCDC_BASECFG0,
             LCDC_BASECFG0_DLBO | LCDC_BASECFG0_BLEN_INCR16);
  sam_putreg(SAM_LCDC_BASECFG1,
             LCDC_BASECFG1_24BPP_RGB888P);
#else
# error Support for this resolution is not yet supported
#endif

#ifdef CONFIG_SAMA5_LCDC_OVR1
#  ifdef CONFIG_SAMA5_LCDC_OVR1_RGB888P
  /* Overlay 1 color configuration, GA 0xff */

  g_ovr1.layer.bpp = 24;

  sam_putreg(SAM_LCDC_OVR1CFG0,
             LCDC_OVR1CFG0_DLBO | LCDC_OVR1CFG0_BLEN_INCR16 |
             LCDC_OVR1CFG0_ROTDIS);
  sam_putreg(SAM_LCDC_OVR1CFG1,
             LCDC_OVR1CFG1_24BPP_RGB888P);
  sam_putreg(SAM_LCDC_OVR1CFG9,
             LCDC_OVR1CFG9_GA(0xff) | LCDC_OVR1CFG9_GAEN);
#  else
#   error Support for this resolution is not yet supported
#  endif
#endif

#ifdef CONFIG_SAMA5_LCDC_OVR2
#  ifdef CONFIG_SAMA5_LCDC_OVR2_RGB888P
  /* Overlay 2 color configuration, GA 0xff */

  g_ovr2.layer.bpp = 24;

  sam_putreg(SAM_LCDC_OVR2CFG0,
             LCDC_OVR2CFG0_DLBO | LCDC_OVR2CFG0_BLEN_INCR16 |
             LCDC_OVR2CFG0_ROTDIS;
  sam_putreg(SAM_LCDC_OVR2CFG1,
             LCDC_OVR2CFG1_24BPP_RGB888P);
  sam_putreg(SAM_LCDC_OVR2CFG9,
             LCDC_OVR2CFG9_GA(0xff) | LCDC_OVR2CFG9_GAEN;
#  else
#   error Support for this resolution is not yet supported
#  endif
#endif

#ifdef CONFIG_SAMA5_LCDC_HEO
#  ifdef CONFIG_SAMA5_LCDC_HEO_RGB888P
  /* High End Overlay color configuration, GA 0xff */

  g_heo.layer.bpp = 24;

  sam_putreg(SAM_LCDC_HEOCFG0,
             LCDC_HEOCFG0_DLBO | LCDC_HEOCFG0_BLEN_INCR16 |
             LCDC_HEOCFG0_ROTDIS);
  sam_putreg(SAM_LCDC_HEOCFG1,
             LCDC_HEOCFG1_24BPP_RGB888P);
  sam_putreg(SAM_LCDC_HEOCFG12,
             LCDC_HEOCFG12_GA(0xff) | LCDC_HEOCFG12_GAEN);
#  else
#   error Support for this resolution is not yet supported
#  endif
#endif

#ifdef CONFIG_FB_HWCURSOR
#  ifdef CONFIG_SAMA5_LCDC_HEO_RGB888P
  /* Hardware Cursor color configuration, GA 0xff, Key #000000 */

  g_hcr.layer.bpp = 24;

  sam_putreg(SAM_LCDC_HCRCFG0,
             LCDC_HCRCFG0_DLBO | LCDC_HCRCFG0_BLEN_INCR16);
  sam_putreg(SAM_LCDC_HCRCFG1,
             LCDC_HCRCFG1_24BPP_RGB888P;
  sam_putreg(SAM_LCDC_HCRCFG7,
             0x000000);
  sam_putreg(SAM_LCDC_HCRCFG8,
             0xffffff);
  sam_putreg(SAM_LCDC_HCRCFG9,
             LCDC_HCRCFG9_GAEN(0xff) | LCDC_HCRCFG9_GAEN);
#  else
#   error Support for this resolution is not yet supported
#  endif
#endif
}

/****************************************************************************
 * Name: sam_lcd_enable
 *
 * Description:
 *   Enable the LCD for normal use
 *
 ****************************************************************************/

static void sam_lcd_enable(void)
{
  uint32_t regval;
  uint32_t div;

  /* Enable the LCD peripheral clock */

  sam_lcdc_enableclk();

  /* Enable the LCD clock */

  sam_putreg(SAM_PMC_SCER, PMC_LCDCK);

  /* 1. Configure LCD timing parameters, signal polarity and clock period. */

  div = ((2 * BOARD_MCK_FREQUENCY) / BOARD_LCD_PIXELCLOCK) - 2;
  regval = LCDC_LCDCFG0_CLKPOL | LCDC_LCDCFG0_CLKSEL |
           LCDC_LCDCFG0_CLKPWMSEL | LCDC_LCDCFG0_CGDISBASE |
           LCDC_LCDCFG0_CGDISOVR1 | LCDC_LCDCFG0_CGDISOVR2 |
           LCDC_LCDCFG0_CGDISHEO | LCDC_LCDCFG0_CGDISHCR |
           LCDC_LCDCFG0_CLKDIV(div);
  sam_putreg(SAM_LCDC_LCDCFG0, regval);

  regval = LCDC_LCDCFG1_HSPW(BOARD_LCD_TIMING_HPW - 1) |
           LCDC_LCDCFG1_VSPW(BOARD_LCD_TIMING_VPW - 1);
  sam_putreg(SAM_LCDC_LCDCFG1, regval);

  regval = LCDC_LCDCFG2_VFPW(BOARD_LCD_TIMING_VFP - 1) |
           LCDC_LCDCFG2_VBPW(BOARD_LCD_TIMING_VBP);
  sam_putreg(SAM_LCDC_LCDCFG2, regval);

  regval = LCDC_LCDCFG3_HFPW(BOARD_LCD_TIMING_HFP - 1) |
           LCDC_LCDCFG3_HBPW(BOARD_LCD_TIMING_HBP - 1);
  sam_putreg(SAM_LCDC_LCDCFG3, regval);

  regval = LCDC_LCDCFG4_PPL(BOARD_LCD_WIDTH - 1) |
           LCDC_LCDCFG4_RPF(BOARD_LCD_HEIGHT - 1);
  sam_putreg(SAM_LCDC_LCDCFG4, regval);

  regval = LCDC_LCDCFG5_HSPOL | LCDC_LCDCFG5_VSPOL |
           LCDC_LCDCFG5_VSPDLYS | LCDC_LCDCFG5_DISPDLY |
           LCDC_LCDCFG5_MODE_24BPP | LCDC_LCDCFG5_GUARDTIME(30);
  sam_putreg(SAM_LCDC_LCDCFG5, regval);

  regval = LCDC_LCDCFG6_PWMPS_DIV64 | LCDC_LCDCFG6_PWMPOL |
           LCDC_LCDCFG6_PWMCVAL(CONFIG_SAMA5_LCDC_DEFBACKLIGHT);
  sam_putreg(SAM_LCDC_LCDCFG6, regval);

  /* 2. Enable the Pixel Clock by writing one to the CLKEN field of the
   *    LCDC_LCDEN register.
   */

  sam_putreg(SAM_LCDC_LCDEN, LCDC_LCDEN_CLK);

  /* 3. Poll CLKSTS field of the LCDC_LCDSR register to check that the clock
   * is running.
   */

  while ((sam_getreg(SAM_LCDC_LCDSR) & LCDC_LCDSR_CLK) == 0);

  /* 4. Enable Horizontal and Vertical Synchronization by writing one to the
   *    SYNCEN field of the LCDC_LCDEN register.
   */

  sam_putreg(SAM_LCDC_LCDEN, LCDC_LCDEN_SYNC);

  /* 5. Poll LCDSTS field of the LCDC_LCDSR register to check that the
   *    synchronization is up.
   */

  while ((sam_getreg(SAM_LCDC_LCDSR) & LCDC_LCDSR_LCD) == 0);

  /* 6. Enable the display power signal writing one to the DISPEN field of the
   *    LCDC_LCDEN register.
   */

  sam_putreg(SAM_LCDC_LCDEN, LCDC_LCDEN_DISP);

  /* 7. Poll DISPSTS field of the LCDC_LCDSR register to check that the power
   *    signal is activated.
   */

  while ((sam_getreg(SAM_LCDC_LCDSR) & LCDC_LCDSR_DISP) == 0);
}

/****************************************************************************
 * Name: sam_fb_allocate
 *
 * Description:
 *   Allocate framebuffer memory
 *
 ****************************************************************************/

static int sam_fb_allocate(void)
{
#if defined(CONFIG_SAMA5_LCDC_FBPREALLOCATED)
  /* Used pre-allocated buffers in .bss */

  g_base.layer.framebuffer = g_basefb;

#ifdef CONFIG_SAMA5_LCDC_OVR1
  g_ovr1.layer.framebuffer = g_ovr1fb;
#endif

#ifdef CONFIG_SAMA5_LCDC_OVR2
  g_ovr2.layer.framebuffer = g_ovr2fb;
#endif

#ifdef CONFIG_SAMA5_LCDC_HEO
  g_heo.layer.framebuffer  = g_heofb;
#endif

#ifdef CONFIG_FB_HWCURSOR
  g_hcr.layer.framebuffer  = g_hcrfb;
#endif

  return OK;

#elif defined(CONFIG_SAMA5_LCDC_FBFIXED)
  /* Use buffers in external memory at an offset from a fixed address */

  g_base.layer.framebuffer = (uint8_t *)SAMA5_LCDC_BUFFER_BASE;

#ifdef CONFIG_SAMA5_LCDC_OVR1
  g_ovr1.layer.framebuffer = (uint8_t *)SAMA5_LCDC_BUFFER_OVR1;
#endif

#ifdef CONFIG_SAMA5_LCDC_OVR2
  g_ovr2.layer.framebuffer = (uint8_t *)SAMA5_LCDC_BUFFER_OVR2;
#endif

#ifdef CONFIG_SAMA5_LCDC_HEO
  g_heo.layer.framebuffer  = (uint8_t *)SAMA5_LCDC_BUFFER_HEO;
#endif

#ifdef CONFIG_FB_HWCURSOR
  g_hcr.layer.framebuffer  = (uint8_t *)SAMA5_LCDC_BUFFER_HCR;
#endif

  return OK;

#else
  /* Allocate frame buffers from the heap */

  g_base.layer.framebuffer = (uint8_t *)kmalloc(SAMA5_BASE_FBSIZE);
  if (!g_base.layer.framebuffer)
    {
      goto errout;
    }

#ifdef CONFIG_SAMA5_LCDC_OVR1
  g_ovr1.layer.framebuffer = (uint8_t *)kmalloc(SAMA5_OVR1_FBSIZE);
  if (!g_ovr1.layer.framebuffer)
    {
      goto errout_with_base;
    }
#endif

#ifdef CONFIG_SAMA5_LCDC_OVR2
  g_ovr2.layer.framebuffer = (uint8_t *)kmalloc(SAMA5_OVR2_FBSIZE);
  if (!g_ovr2.layer.framebuffer)
    {
      goto errout_with_ovr1;
    }
#endif

#ifdef CONFIG_SAMA5_LCDC_HEO
  g_heo.buffer = (uint8_t *)kmalloc(SAMA5_HEO_FBSIZE);
  if (!g_heo.layer.framebuffer)
    {
      goto errout_with_ovr2;
    }
#endif

#ifdef CONFIG_SAMA5_LCDC_HCR
  g_hcr.layer.framebuffer = (uint8_t *)kmalloc(SAMA5_HCR_FBSIZE);
  if (!g_hcr.layer.framebuffer)
    {
      goto errout_with_heo;
    }
#endif

  return OK;

#ifdef CONFIG_SAMA5_LCDC_HCR
errout_with_heo:
#endif

#ifdef CONFIG_SAMA5_LCDC_HEO

  kfree(g_heo.layer.framebuffer);
  g_heo.layer.framebuffer = NULL;

errout_with_ovr2:
#endif

#ifdef CONFIG_SAMA5_LCDC_OVR2
  kfree(g_ovr2.layer.framebuffer);
  g_ovr2.layer.framebuffer = NULL;

errout_with_ovr1:
#endif

#ifdef CONFIG_SAMA5_LCDC_OVR1

  kfree(g_ovr1.layer.framebuffer);
  g_ovr1.layer.framebuffer = NULL;

errout_with_base:
#endif
  kfree(g_base.layer.framebuffer);
  g_base.layer.framebuffer = NULL;

errout:
  return -ENOMEM;
#endif
}

/****************************************************************************
 * Name: sam_scalefactor
 *
 * Description:
 *   Calculate HEO scale factor
 *
 * Input Parameters:
 *   oldw - the old image width
 *   neww - The new image width
 *
 ****************************************************************************/

#ifdef CONFIG_SAMA5_LCDC_HEO
static uint32_t sam_scalefactor(uint32_t oldw, uint32_t neww)
{
  return 2048 * (neww + 1) / (oldw + 1);
}
#endif

/****************************************************************************
 * Name: sam_show_layer
 *
 * Description:
 *   Show the give layer with the specified orientation and (perhaps) scaling.
 *
 ****************************************************************************/

static void sam_show_layer(struct sam_layer_s *layer,
                           uint32_t dispx, uint32_t dispy,
                           uint32_t dispw, uint32_t disph,
                           uint32_t imgw, uint32_t imgh)
{
  struct sam_dscr_s *dscr;
  uint8_t *buffer;
  uintptr_t cfgaddr;
  uintptr_t regaddr;
  uintptr_t physaddr;
  uint32_t padding = 0;
  uint32_t regval;
  uint32_t bytesprow;
  uint32_t bytespp;
  uint32_t bprow;
  bool bottomup;
  bool rightleft;
  int lid;

  DEBUGASSERT(layer && layer->dscr);

  /* Windows position & size check */

  if (dispx + dispw > BOARD_LCD_WIDTH)
    {
      dispw = BOARD_LCD_WIDTH - dispx;
    }

  if (dispy + disph > BOARD_LCD_HEIGHT)
    {
      disph = BOARD_LCD_HEIGHT - dispy;
    }

  if (dispw <= 0)
    {
      dispw = 1;
    }

  if (disph <= 0)
    {
      disph = 1;
    }

  if (imgw <= 0)
    {
      imgw = 1;
    }

  if (imgh <= 0)
    {
      imgh = 1;
    }

  /* Set display buffer and mode setup*/

  bytespp   = (uint32_t)layer->bpp >> 3;
  bprow     = imgw * (uint32_t)layer->bpp;
  bytesprow = bprow >> 3;

  if ((bprow & 7) != 0)
    {
      bytesprow ++;
    }

  padding = 0;
  if ((bytesprow & 3) != 0)
    {
      padding = 4 - (bytesprow & 0x3);
    }

  /* Bottom-up and Right-to-left mode setup */

  bottomup  = (layer->flags & LCDC_FLAG_BOTTOMUP) != 0;
  rightleft = (layer->flags & LCDC_FLAG_RIGHTLEFT) != 0;

  /* No X mirror supported layer, no Right->Left scan */

#ifdef SAMA5_HAVE_PSTRIDE
  regaddr = g_layerpstride[lid];
  if (regaddr)
    {
      rightleft = false;
    }
#endif

  dscr    = layer->dscr;
  lid     = layer->lid;
  buffer  = layer->framebuffer;

  cfgaddr = g_layercfg[lid];

  /* Normal direction: Left,Top -> Right,Down */

  if ((!rightleft && !bottomup && layer->rotation == LCDC_ROT_0  ) ||
      ( rightleft &&  bottomup && layer->rotation == LCDC_ROT_180))
    {
      /* No rotation optimization */

      regval  = sam_getreg(cfgaddr);
      regval |= LCDC_HEOCFG0_ROTDIS;
      sam_putreg(cfgaddr, regval);

      /* X0 ++ */

#ifdef SAMA5_HAVE_PSTRIDE
      regaddr = g_layerpstride[lid];
      if (regaddr)
        {
          sam_putreg(regaddr, 0);
        }
#endif

      /* Y0 ++ */

      regaddr = g_layerstride[lid];
      sam_putreg(regaddr, padding);

      /* Pointer to Left,Top (x0,y0) */
    }

  /* X mirror: Right,Top -> Left,Down */

  else if (( rightleft && !bottomup && layer->rotation == LCDC_ROT_0  ) ||
           (!rightleft &&  bottomup && layer->rotation == LCDC_ROT_180))
    {
      /* No rotation optimization */

      regval  = sam_getreg(cfgaddr);
      regval |= LCDC_HEOCFG0_ROTDIS;
      sam_putreg(cfgaddr, regval);

      /* X1 -- */

#ifdef SAMA5_HAVE_PSTRIDE
      regaddr = g_layerpstride[lid];
      if (regaddr)
        {
          sam_putreg(regaddr, 0 - 2*bytespp);
        }
#endif

      /* Y0 ++ */

      regaddr = g_layerstride[lid];
      sam_putreg(regaddr, 2*bytesprow + padding - 2*bytespp);

      /* Pointer to Right,Top (x1,y0) */

      buffer = (uint8_t *)
        ((uint32_t)layer->framebuffer + bytespp*(imgw - 1));
    }

  /* Y mirror: Left,Down -> Right,Top */

  else if ((!rightleft &&  bottomup && layer->rotation == LCDC_ROT_0  ) ||
           ( rightleft && !bottomup && layer->rotation == LCDC_ROT_180))
    {
      /* No rotation optimization */

      regval  = sam_getreg(cfgaddr);
      regval |= LCDC_HEOCFG0_ROTDIS;
      sam_putreg(cfgaddr, regval);

      /* X0 ++ */

#ifdef SAMA5_HAVE_PSTRIDE
      regaddr = g_layerpstride[lid];
      if (regaddr)
        {
          sam_putreg(regaddr, 0);
        }
#endif

      /* Y1 -- */

      regaddr = g_layerstride[lid];
      sam_putreg(regaddr, 0 - (2*bytesprow + padding));

      /* Pointer to Left,Down (x0,y1) */

      buffer = (uint8_t *)
        ((uintptr_t)layer->framebuffer + (bytesprow+padding)*(imgh-1));
    }

  /* X,Y mirror: Right,Top -> Left,Down */

  else if (( rightleft &&  bottomup && layer->rotation == LCDC_ROT_0  ) ||
           (!rightleft && !bottomup && layer->rotation == LCDC_ROT_180))
    {
      /* No rotation optimization */

      regval  = sam_getreg(cfgaddr);
      regval |= LCDC_HEOCFG0_ROTDIS;
      sam_putreg(cfgaddr, regval);

      /* X1 -- */

#ifdef SAMA5_HAVE_PSTRIDE
      regaddr = g_layerpstride[lid];
      if (regaddr)
        {
          sam_putreg(regaddr, 0 - 2*bytespp;
        }
#endif

      /* Y1 -- */

      regaddr = g_layerstride[lid];
      sam_putreg(regaddr, 0 - (2*bytespp + padding));

      /* Pointer to Left,Down (x1,y1) */

      buffer = (uint8_t *)
        ((uint32_t)layer->framebuffer +
         (bytesprow + padding)*(imgh - 1) +
         bytespp*(imgw -1 ));
    }

  /* Rotate  90: Down,Left -> Top,Right (with w,h swap) */

  else if ((!rightleft && !bottomup && layer->rotation == LCDC_ROT_90 ) ||
           ( rightleft &&  bottomup && layer->rotation == LCDC_ROT_270))
    {
      /* No rotation optimization */

      regval  = sam_getreg(cfgaddr);
      regval |= LCDC_HEOCFG0_ROTDIS;
      sam_putreg(cfgaddr, regval);

      /* Y -- as pixels in row */

#ifdef SAMA5_HAVE_PSTRIDE
      regaddr = g_layerpstride[lid];
      if (regaddr)
        {
          sam_putreg(regaddr, 0 - (bytespp + bytesprow + padding));
        }
#endif

      /* X ++ as rows */

      regaddr = g_layerstride[lid];
      sam_putreg(regaddr, (bytesprow + padding)*(imgh - 1));

      /* Pointer to Bottom,Left */

      buffer = (uint8_t *)
        ((uint32_t)layer->framebuffer +
         (bytesprow + padding)*(imgh - 1));
    }

  /* Rotate 270: Top,Right -> Down,Left (with w,h swap) */

  else if ((!rightleft && !bottomup && layer->rotation == LCDC_ROT_270) ||
           ( rightleft &&  bottomup && layer->rotation == LCDC_ROT_90 ))
    {
      /* No rotation optimization */

      regval  = sam_getreg(cfgaddr);
      regval |= LCDC_HEOCFG0_ROTDIS;
      sam_putreg(cfgaddr, regval);

      /* Y ++ as pixels in row */

#ifdef SAMA5_HAVE_PSTRIDE
      regaddr = g_layerpstride[lid];
      if (regaddr)
        {
          sam_putreg(regaddr, bytesprow + padding - bytespp);
        }
#endif

      /* X -- as rows */

      regaddr = g_layerstride[lid];
      sam_putreg(regaddr, 0 - 2*bytespp - (bytesprow + padding)*(imgh - 1));

      /* Pointer to top right */

      buffer = (uint8_t *)
        ((uintptr_t)layer->framebuffer + bytespp*(imgw - 1));
    }

  /* Mirror X then Rotate 90: Down,Right -> Top,Left */

  else if (( rightleft && !bottomup && layer->rotation == LCDC_ROT_90 ) ||
           (!rightleft &&  bottomup && layer->rotation == LCDC_ROT_270))
    {
      /* No rotation optimization */

      regval  = sam_getreg(cfgaddr);
      regval |= LCDC_HEOCFG0_ROTDIS;
      sam_putreg(cfgaddr, regval);

      /* Y -- as pixels in row */

#ifdef SAMA5_HAVE_PSTRIDE
      regaddr = g_layerpstride[lid];
      if (regaddr)
        {
          sam_putreg(regaddr, 0 - (bytespp + bytesprow + padding));
        }
#endif

      /* X -- as rows */

      regaddr = g_layerstride[lid];
      sam_putreg(regaddr, 0 - 2*bytespp + (bytesprow + padding)*(imgh - 1));

      /* Pointer to down right (x1,y1) */

      buffer = (uint8_t *)
        ((uintptr_t)layer->framebuffer +
         (bytesprow+padding)*(imgh - 1) +
         (bytespp)*(imgw - 1));
    }

  /* Mirror Y then Rotate 90: Top,Left -> Down,Right */

  else if ((!rightleft &&  bottomup && layer->rotation ==  90)
          ||(rightleft && !bottomup && layer->rotation == LCDC_ROT_270))
    {
      /* No rotation optimization */

      regval  = sam_getreg(cfgaddr);
      regval |= LCDC_HEOCFG0_ROTDIS;
      sam_putreg(cfgaddr, regval);

      /* Y ++ as pixels in row */

#ifdef SAMA5_HAVE_PSTRIDE
      regaddr = g_layerpstride[lid];
      if (regaddr)
        {
          sam_putreg(regaddr, bytesprow + padding - bytespp);
        }
#endif

      /* X ++ as rows */

      regaddr = g_layerstride[lid];
      sam_putreg(regaddr, 0 - (bytesprow + padding)*(imgh - 1));

      /* Pointer to top left (x0,y0) */
    }

  /* DMA is running, just add new descriptor to queue */

  regaddr = g_layerblend[lid];
  if ((sam_getreg(regaddr) & LCDC_HEOCFG12_DMA) != 0)
    {
      dscr->addr = (uint32_t)buffer;
      dscr->ctrl = LCDC_HEOCTRL_DFETCH;
      dscr->next = (uint32_t)dscr;

      physaddr = sam_physramaddr((uintptr_t)dscr);
      regaddr = g_layerhead[lid];
      sam_putreg(regaddr, physaddr);

      regaddr = g_layerenable[lid];
      sam_putreg(regaddr, LCDC_HEOCHER_A2Q);
    }
  else
    {
      /* Configure the DMA */

      sam_dmasetup(lid, dscr, buffer);
    }

  cp15_flush_dcache((uint32_t)dscr, ((uint32_t)dscr) + sizeof(dscr));

  /* Set layer position and size */

#ifdef SAMA5_HAVE_POSITION
  regaddr = g_layerpos[lid];
  if (regaddr)
    {
      sam_putreg(regaddr,
                 LCDC_HEOCFG2_XPOS(dispx) | LCDC_HEOCFG2_YPOS(dispy));
    }
#endif

#ifdef SAMA5_HAVE_SIZE
  regaddr = g_layersize[lid];
  if (regaddr)
    {
      sam_putreg(regaddr,
                 LCDC_HEOCFG3_XSIZE(dispw - 1) | LCDC_HEOCFG3_YSIZE(disph - 1);
    }
#endif

#ifdef CONFIG_SAMA5_LCDC_HEO
  /* Scaling setup */

  if (lid == LCDC_HEO)
    {
      uint32_t srcw;
      uint32_t srch;

      /* Image size only used in scaling */
      /* Scaling target */

      if (layer->rotation == LCDC_ROT_90 || layer->rotation == LCDC_ROT_270)
        {
          srcw = imgh;
          srch = imgw;
        }
      else
        {
          srcw = imgw;
          srch = imgh;
        }

      sam_putreg(SAM_LCDC_HEOCFG4,
                 LCDC_HEOCFG4_XMEM_SIZE(srcw - 1) |
                 LCDC_HEOCFG4_YMEM_SIZE(srch - 1));

      /* Scaled */

      if (dispw != srcw || disph != srch)
        {
          uint16_t xfactor;
          uint16_t yfactor;

          xfactor = sam_scalefactor(dispw, srcw);
          yfactor = sam_scalefactor(disph, srch);

          sam_putreg(LCDC_HEOCFG13,
                     LCDC_HEOCFG13_YFACTOR(yfactor) |
                     LCDC_HEOCFG13_XFACTOR(xfactor) |
                     LCDC_HEOCFG13_SCALEN;
        }

      /* Disable scaling */

      else
        {
          sam_putreg(LCDC_HEOCFG13, 0);
        }
    }
#endif

  /* Enable DMA */

  if (buffer)
    {
      regaddr = g_layerblend[lid];
      regval = sam_getreg(regaddr);
      regval |= LCDC_HEOCFG12_DMA | LCDC_HEOCFG12_OVR;
      sam_putreg(regaddr, regval);
    }

  /* Enable and Update */
  /* 5. Enable the relevant channel by writing one to the CHEN field of the
   *    CHXCHER register.
   */

  regaddr = g_layerenable[lid];
  sam_putreg(regaddr, LCDC_HEOCHER_UPDATE | LCDC_HEOCHER_CH);

  /* 6. An interrupt may be raised if unmasked when the descriptor has been
   *    loaded.
   */
}

/****************************************************************************
 * Name: sam_show_base
 *
 * Description:
 *   Show the base layer
 *
 ****************************************************************************/

static void sam_show_base(void)
{
  sam_show_layer(&g_base.layer, 0, 0,
      BOARD_LCD_WIDTH, BOARD_LCD_HEIGHT, BOARD_LCD_WIDTH, BOARD_LCD_HEIGHT);
}

/****************************************************************************
 * Name: sam_show_hcr
 *
 * Description:
 *   Show the hardware cursor layer
 *
 ****************************************************************************/

#ifdef CONFIG_FB_HWCURSOR
static void sam_show_hcr(uint32_t x, uint32_t y)
{
  /* Enable default transparent keying */

  sam_putreg(SAM_LCDC_HCRCFG7, 0x00000000);
  sam_putreg(SAM_LCDC_HCRCFG8, 0xffffffff);

  regval  = sam_getreg(SAM_LCDC_HCRCFG9);
  regval |= LCDC_HCRCFG9_CRKEY);
  sam_putreg(SAM_LCDC_HCRCFG9, regval);

  /* And show the hardware cursor layer */

  sam_show_layer(&g_hcr.layer,
    (BOARD_LCD_WIDTH - CONFIG_SAMA5_LCDC_HCR_MAXWIDTH) / 2,
    (BOARD_LCD_HEIGHT - CONFIG_SAMA5_LCDC_HCR_MAXHEIGHT) / 2,
    CONFIG_SAMA5_LCDC_HCR_MAXWIDTH, CONFIG_SAMA5_LCDC_HCR_MAXHEIGHT,
    CONFIG_SAMA5_LCDC_HCR_MAXWIDTH, CONFIG_SAMA5_LCDC_HCR_MAXHEIGHT);
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_fbinitialize
 *
 * Description:
 *   Initialize the framebuffer video hardware
 *
 ****************************************************************************/

int up_fbinitialize(void)
{
  int ret;

  gvdbg("Entry\n");

  /* Disable the LCD */

  sam_lcd_disable();

  /* Reset layer information */

  memset(&g_base, 0, sizeof(struct sam_layer_alloc_s));
  g_base.layer.dscr = &g_base.dscr;
  g_base.layer.lid  = LCDC_LAYER_BASE;

  memset(&g_ovr1, 0, sizeof(struct sam_layer_alloc_s));
  g_ovr1.layer.dscr = &g_ovr1.dscr;
  g_ovr1.layer.lid  = LCDC_LAYER_OVR1;

  memset(&g_ovr2, 0, sizeof(struct sam_layer_alloc_s));
  g_ovr2.layer.dscr = &g_ovr2.dscr;
  g_ovr2.layer.lid  = LCDC_LAYER_OVR2;

  memset(&g_heo,  0, sizeof(struct sam_heolayer_alloc_s));
  g_heo.layer.dscr  =  g_heo.dscr; /* DSCR is an array */
  g_heo.layer.lid   = LCDC_LAYER_HEO; /* DSCR is an array */

  memset(&g_hcr,  0, sizeof(struct sam_layer_alloc_s));
  g_hcr.layer.dscr  = &g_hcr.dscr;
  g_hcr.layer.lid   = LCDC_LAYER_HCR;

  /* Allocate framebuffer memory */

  ret = sam_fb_allocate();
  if (ret < 0)
    {
      gdbg("ERROR: Failed to allocate framebuffer memory\n");
      return ret;
    }

  /* Configure PIO pins */

  sam_pio_config();

  gvdbg("Configuring the LCD controller\n");

  /* Enable the LCD peripheral clock */

  sam_lcdc_enableclk();

  /* Enable the LCD clock */

  sam_putreg(SAM_PMC_SCER, PMC_LCDCK);

  /* Disable LCD interrupts */

  sam_putreg(SAM_LCDC_LCDIDR, LCDC_LCDINT_ALL);

  /* Configure layer orientation */

  sam_layer_orientation();

  /* Configure layer colors */

  sam_layer_color();

  /* Clear the display memory */

  sam_lcdclear(CONFIG_SAMA5_LCDC_BACKCOLOR);

  /* And turn the LCD on */

  gvdbg("Enabling the display\n");
  sam_lcd_enable();

  /* Display base layer */

  sam_show_base();

#if defined(CONFIG_SAMA5_LCDC_OVR1) && defined(CONFIG_SAMA5_LCDC_HEO)
  /* Overlay 1 is above the HEO layer */

  regval = sam_getreg(SAM_LCDC_HEOCFG12);
  regval |= LCDC_HEOCFG12_VIDPRI;
  sam_putreg(SAM_LCDC_HEOCFG12, regval);

  sam_putreg(SAM_LCDC_HEOCHER, LCDC_HEOCHER_UPDATE);
#endif

#ifdef CONFIG_FB_HWCURSOR
  /* Show cursor layer */

  sam_show_hcr();
#endif

  /* Enable the backlight.
   *
   * REVISIT:  Backlight level could be dynamically adjustable
   */

  sam_backlight(CONFIG_SAMA5_LCDC_DEFBACKLIGHT);

  return OK;
}

/****************************************************************************
 * Name: sam_fbgetvplane
 *
 * Description:
 *   Return a a reference to the framebuffer object for the specified video
 *   plane.
 *
 * Input parameters:
 *   None
 *
 * Returned value:
 *   Reference to the framebuffer object (NULL on failure)
 *
 ***************************************************************************/

struct fb_vtable_s *up_fbgetvplane(int vplane)
{
  gvdbg("vplane: %d\n", vplane);
  if (vplane == 0)
    {
      return (struct fb_vtable_s *)&g_base_vtable;
    }
  else
    {
      return NULL;
    }
}

/****************************************************************************
 * Name: fb_uninitialize
 *
 * Description:
 *   Unitialize the framebuffer support
 *
 ****************************************************************************/

void fb_uninitialize(void)
{
  /* Disable the LCD controller */

  sam_lcd_disable();
}

/************************************************************************************
 * Name:  sam_lcdclear
 *
 * Description:
 *   This is a non-standard LCD interface just for the SAMA5.  Clearing the display
 *   in the normal way by writing a sequences of runs that covers the entire display
 *   can be slow.  Here the display is cleared by simply setting all video memory to
 *   the specified color.
 *
 ************************************************************************************/

void sam_lcdclear(nxgl_mxpixel_t color)
{
#if SAMA5_LCDC_BASE_BPP == 16
  uint16_t *dest = (uint16_t*)g_base.layer.framebuffer;
  int i;

  gvdbg("Clearing display: BPP=16 color=%04x framebuffer=%08x size=%d\n",
        color, g_base.layer.framebuffer, SAMA5_BASE_FBSIZE);

  for (i = 0; i < SAMA5_BASE_FBSIZE; i += sizeof(uint16_t))
    {
      *dest++ = (uint16_t)color;
    }
#elif SAMA5_LCDC_BASE_BPP == 24
  uint8_t *dest = (uint8_t*)g_base.layer.framebuffer;
  uint8_t r;
  uint8_t g;
  uint8_t b;
  int i;

  gvdbg("Clearing display: BPP=24 color=%06x framebuffer=%08x size=%d\n",
        color, g_base.layer.framebuffer, SAMA5_BASE_FBSIZE);

  b =  color        & 0xff;
  g = (color >> 8)  & 0xff;
  r = (color >> 16) & 0xff;

  for (i = 0; i < SAMA5_BASE_FBSIZE; i += 3*sizeof(uint8_t))
    {
      *dest++ = b;
      *dest++ = r;
      *dest++ = g;
    }
#elif SAMA5_LCDC_BASE_BPP == 32
  uint32_t *dest = (uint32_t*)g_base.layer.framebuffer;
  int i;

  gvdbg("Clearing display: BPP=32 color=%08x framebuffer=%08x size=%d\n",
        color, g_base.layer.framebuffer, SAMA5_BASE_FBSIZE);

  for (i = 0; i < SAMA5_BASE_FBSIZE; i += sizeof(uint32_t))
    {
      *dest++ = (uint32_t)color;
    }
#endif
}
