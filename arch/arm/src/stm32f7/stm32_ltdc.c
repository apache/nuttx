/****************************************************************************
 * arch/arm/src/stm32f7/stm32_ltdc.c
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

/* References:
 *   STM32F429 Technical Reference Manual and Data Sheet
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/video/fb.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "hardware/stm32_ltdc.h"
#include "hardware/stm32_dma2d.h"
#include "stm32_rcc.h"
#include "stm32_gpio.h"
#include "stm32_ltdc.h"
#include "stm32_dma2d.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register definition ******************************************************/

#ifndef BOARD_LTDC_WIDTH
#  error BOARD_LTDC_WIDTH must be defined in the board.h header file
#endif

#ifndef BOARD_LTDC_HEIGHT
#  error BOARD_LTDC_HEIGHT must be defined in the board.h header file
#endif

#define STM32_LTDC_HEIGHT           BOARD_LTDC_HEIGHT
#define STM32_LTDC_WIDTH            BOARD_LTDC_WIDTH

/* Configure LTDC register */

/* LTDC_LxWHPCR register */

#define STM32_LTDC_LXWHPCR_WHSTPOS  (BOARD_LTDC_HSYNC + BOARD_LTDC_HBP - 1)
#define STM32_LTDC_LxWHPCR_WHSPPOS  (BOARD_LTDC_HSYNC + BOARD_LTDC_HBP + \
                                    STM32_LTDC_WIDTH - 1)

/* LTDC_LxWVPCR register */

#define STM32_LTDC_LXWVPCR_WVSTPOS  (BOARD_LTDC_VSYNC + BOARD_LTDC_VBP - 1)
#define STM32_LTDC_LxWVPCR_WVSPPOS  (BOARD_LTDC_VSYNC + BOARD_LTDC_VBP + \
                                    STM32_LTDC_HEIGHT - 1)

/* LTDC_SSCR register */

#define STM32_LTDC_SSCR_VSH         LTDC_SSCR_VSH(BOARD_LTDC_VSYNC - 1)
#define STM32_LTDC_SSCR_HSW         LTDC_SSCR_HSW(BOARD_LTDC_HSYNC - 1)

/* LTDC_BPCR register */

#define STM32_LTDC_BPCR_AVBP        LTDC_BPCR_AVBP(STM32_LTDC_LXWVPCR_WVSTPOS)
#define STM32_LTDC_BPCR_AHBP        LTDC_BPCR_AHBP(STM32_LTDC_LXWHPCR_WHSTPOS)

/* LTDC_AWCR register */

#define STM32_LTDC_AWCR_AAH         LTDC_AWCR_AAH(STM32_LTDC_LxWVPCR_WVSPPOS)
#define STM32_LTDC_AWCR_AAW         LTDC_AWCR_AAW(STM32_LTDC_LxWHPCR_WHSPPOS)

/* LTDC_TWCR register */

#define STM32_LTDC_TWCR_TOTALH      LTDC_TWCR_TOTALH(BOARD_LTDC_VSYNC + \
                                    BOARD_LTDC_VBP + \
                                    STM32_LTDC_HEIGHT + BOARD_LTDC_VFP - 1)
#define STM32_LTDC_TWCR_TOTALW      LTDC_TWCR_TOTALW(BOARD_LTDC_HSYNC + \
                                    BOARD_LTDC_HBP + \
                                    STM32_LTDC_WIDTH + BOARD_LTDC_HFP - 1)

/* Global GCR register */

/* Synchronisation and Polarity */

#define STM32_LTDC_GCR_PCPOL        BOARD_LTDC_GCR_PCPOL
#define STM32_LTDC_GCR_DEPOL        BOARD_LTDC_GCR_DEPOL
#define STM32_LTDC_GCR_VSPOL        BOARD_LTDC_GCR_VSPOL
#define STM32_LTDC_GCR_HSPOL        BOARD_LTDC_GCR_HSPOL

/* Dither */

#define STM32_LTDC_GCR_DEN          BOARD_LTDC_GCR_DEN
#define STM32_LTDC_GCR_DBW          LTDC_GCR_GBW(BOARD_LTDC_GCR_DBW)
#define STM32_LTDC_GCR_DGW          LTDC_GCR_DGW(BOARD_LTDC_GCR_DGW)
#define STN32_LTDC_GCR_DRW          LTDC_GCR_DBW(BOARD_LTDC_GCR_DRW)

/* LIPCR register */

#define STM32_LTDC_LIPCR_LIPOS      LTDC_LIPCR_LIPOS(STM32_LTDC_TWCR_TOTALW)

/* Configuration ************************************************************/

#ifndef CONFIG_STM32F7_LTDC_DEFBACKLIGHT
#  define CONFIG_STM32F7_LTDC_DEFBACKLIGHT 0xf0
#endif
#define STM32_LTDC_BACKLIGHT_OFF 0x00

/* Color/video formats */

/* Layer 1 format */

#if defined(CONFIG_STM32F7_LTDC_L1_L8)
#  define STM32_LTDC_L1_BPP         8
#  define STM32_LTDC_L1_COLOR_FMT   FB_FMT_RGB8
#  define STM32_LTDC_L1PFCR_PF      LTDC_LXPFCR_PF(LTDC_PF_L8)
#  define STM32_LTDC_L1_DMA2D_PF    DMA2D_PF_L8
#  define STM32_LTDC_L1CMAP
#elif defined(CONFIG_STM32F7_LTDC_L1_RGB565)
#  define STM32_LTDC_L1_BPP         16
#  define STM32_LTDC_L1_COLOR_FMT   FB_FMT_RGB16_565
#  define STM32_LTDC_L1PFCR_PF      LTDC_LXPFCR_PF(LTDC_PF_RGB565)
#  define STM32_LTDC_L1_DMA2D_PF    DMA2D_PF_RGB565
#elif defined(CONFIG_STM32F7_LTDC_L1_RGB888)
#  define STM32_LTDC_L1_BPP         24
#  define STM32_LTDC_L1_COLOR_FMT   FB_FMT_RGB24
#  define STM32_LTDC_L1PFCR_PF      LTDC_LXPFCR_PF(LTDC_PF_RGB888)
#  define STM32_LTDC_L1_DMA2D_PF    DMA2D_PF_RGB888
#elif defined(CONFIG_STM32F7_LTDC_L1_ARGB8888)
#  define STM32_LTDC_L1_BPP         32
#  define STM32_LTDC_L1_COLOR_FMT   FB_FMT_RGB32
#  define STM32_LTDC_L1PFCR_PF      LTDC_LXPFCR_PF(LTDC_PF_ARGB8888)
#  define STM32_LTDC_L1_DMA2D_PF    DMA2D_PF_ARGB8888
#else
#  error "LTDC pixel format not supported"
#endif

/* Layer 2 format */

#ifdef CONFIG_STM32F7_LTDC_L2
#  if defined(CONFIG_STM32F7_LTDC_L2_L8)
#   define STM32_LTDC_L2_BPP         8
#   define STM32_LTDC_L2_COLOR_FMT   FB_FMT_RGB8
#   define STM32_LTDC_L2PFCR_PF      LTDC_LXPFCR_PF(LTDC_PF_L8)
#   define STM32_LTDC_L2_DMA2D_PF    DMA2D_PF_L8
#   define STM32_LTDC_L2CMAP
#  elif defined(CONFIG_STM32F7_LTDC_L2_RGB565)
#   define STM32_LTDC_L2_BPP         16
#   define STM32_LTDC_L2_COLOR_FMT   FB_FMT_RGB16_565
#   define STM32_LTDC_L2PFCR_PF      LTDC_LXPFCR_PF(LTDC_PF_RGB565)
#   define STM32_LTDC_L2_DMA2D_PF    DMA2D_PF_RGB565
#  elif defined(CONFIG_STM32F7_LTDC_L2_RGB888)
#   define STM32_LTDC_L2_BPP         24
#   define STM32_LTDC_L2_COLOR_FMT   FB_FMT_RGB24
#   define STM32_LTDC_L2PFCR_PF      LTDC_LXPFCR_PF(LTDC_PF_RGB888)
#   define STM32_LTDC_L2_DMA2D_PF    DMA2D_PF_RGB888
#  elif defined(CONFIG_STM32F7_LTDC_L2_ARGB8888)
#   define STM32_LTDC_L2_BPP         32
#   define STM32_LTDC_L2_COLOR_FMT   FB_FMT_RGB32
#   define STM32_LTDC_L2PFCR_PF      LTDC_LXPFCR_PF(LTDC_PF_ARGB8888)
#   define STM32_LTDC_L2_DMA2D_PF    DMA2D_PF_ARGB8888
#  else
#   error "LTDC pixel format not supported"
#  endif
#endif /* CONFIG_STM32F7_LTDC_L2 */

/* Framebuffer sizes in bytes */

#if STM32_LTDC_L1_BPP == 8
#  define STM32_LTDC_L1_STRIDE      (STM32_LTDC_WIDTH)
#elif STM32_LTDC_L1_BPP == 16
#  define STM32_LTDC_L1_STRIDE      ((STM32_LTDC_WIDTH * 16 + 7) / 8)
#elif STM32_LTDC_L1_BPP == 24
#  define STM32_LTDC_L1_STRIDE      ((STM32_LTDC_WIDTH * 24 + 7) / 8)
#elif STM32_LTDC_L1_BPP == 32
#  define STM32_LTDC_L1_STRIDE      ((STM32_LTDC_WIDTH * 32 + 7) / 8)
#else
#  error Undefined or unrecognized base resolution
#endif

/* LTDC only supports 8 bit per pixel overal */

#define STM32_LTDC_LX_BYPP(n)       ((n) / 8)

#define STM32_LTDC_L1_FBSIZE        (STM32_LTDC_L1_STRIDE * STM32_LTDC_HEIGHT)

#ifdef CONFIG_STM32F7_LTDC_L2
#  ifndef CONFIG_STM32F7_LTDC_L2_WIDTH
#    define CONFIG_STM32F7_LTDC_L2_WIDTH STM32_LTDC_WIDTH
#  endif

#  if CONFIG_STM32F7_LTDC_L2_WIDTH > STM32_LTDC_WIDTH
#    error Width of Layer 2 exceeds the width of the display
#  endif

#  ifndef CONFIG_STM32F7_LTDC_L2_HEIGHT
#    define CONFIG_STM32F7_LTDC_L2_HEIGHT STM32_LTDC_HEIGHT
#  endif

#  if CONFIG_STM32F7_LTDC_L2_HEIGHT > STM32_LTDC_HEIGHT
#    error Height of Layer 2 exceeds the height of the display
#  endif

#  if STM32_LTDC_L2_BPP == 8
#    define STM32_LTDC_L2_STRIDE    (CONFIG_STM32F7_LTDC_L2_WIDTH)
#  elif STM32_LTDC_L2_BPP == 16
#    define STM32_LTDC_L2_STRIDE    ((CONFIG_STM32F7_LTDC_L2_WIDTH * 16 + 7) / 8)
#  elif STM32_LTDC_L2_BPP == 24
#    define STM32_LTDC_L2_STRIDE    ((CONFIG_STM32F7_LTDC_L2_WIDTH * 24 + 7) / 8)
#  elif STM32_LTDC_L2_BPP == 32
#    define STM32_LTDC_L2_STRIDE    ((CONFIG_STM32F7_LTDC_L2_WIDTH * 32 + 7) / 8)
#  else
#    error Undefined or unrecognized base resolution
#  endif

#  define STM32_LTDC_L2_FBSIZE      (STM32_LTDC_L2_STRIDE * \
                                     CONFIG_STM32F7_LTDC_L2_HEIGHT)

#else
#  define STM32_LTDC_L2_FBSIZE (0)
#  define STM32_LTDC_L2_BPP 0
#endif

/* Total memory used for framebuffers */

#define STM32_LTDC_TOTAL_FBSIZE     (STM32_LTDC_L1_FBSIZE + \
                                     STM32_LTDC_L2_FBSIZE)

/* Debug option */

#ifdef CONFIG_STM32F7_LTDC_REGDEBUG
#  define regerr       lcderr
#  define reginfo      lcdinfo
#else
#  define regerr(x...)
#  define reginfo(x...)
#endif

/* Preallocated LTDC framebuffers */

/* Position the framebuffer memory in the center of the memory set aside.
 * We will use any skirts before or after the framebuffer memory as a guard
 * against wild framebuffer writes.
 */

#define STM32_LTDC_BUFFER_SIZE      CONFIG_STM32F7_LTDC_FB_SIZE
#define STM32_LTDC_BUFFER_FREE      (STM32_LTDC_BUFFER_SIZE - \
                                    STM32_LTDC_TOTAL_FBSIZE)
#define STM32_LTDC_BUFFER_START     (CONFIG_STM32F7_LTDC_FB_BASE + \
                                    STM32_LTDC_BUFFER_FREE/2)

#if STM32_LTDC_BUFFER_FREE < 0
#  error "STM32_LTDC_BUFFER_SIZE not large enough for frame buffers"
#endif

/* Layer frame buffer */

#define STM32_LTDC_BUFFER_L1        STM32_LTDC_BUFFER_START
#define STM32_LTDC_ENDBUF_L1        (STM32_LTDC_BUFFER_L1 + \
                                     STM32_LTDC_L1_FBSIZE)

#ifdef CONFIG_STM32F7_LTDC_L2
#  define STM32_LTDC_BUFFER_L2      STM32_LTDC_ENDBUF_L1
#  define STM32_LTDC_ENDBUF_L2      (STM32_LTDC_BUFFER_L2 + \
                                     STM32_LTDC_L2_FBSIZE)
#else
#  define STM32_LTDC_ENDBUF_L2      STM32_LTDC_ENDBUF_L1
#endif

/* LTDC layer */

#ifdef CONFIG_STM32F7_LTDC_L2
#  define LTDC_NLAYERS 2
#else
#  define LTDC_NLAYERS 1
#endif

/* DMA2D layer */

#ifdef CONFIG_STM32F7_DMA2D
#  define DMA2D_NLAYERS             CONFIG_STM32F7_DMA2D_NLAYERS
#  if DMA2D_NLAYERS < 1
#    error "DMA2D must at least support 1 overlay"
#  endif

#define STM32_DMA2D_WIDTH           CONFIG_STM32F7_DMA2D_LAYER_PPLINE

#  if defined(CONFIG_STM32F7_DMA2D_L8)
#    define STM32_DMA2D_STRIDE      (STM32_DMA2D_WIDTH)
#    define STM32_DMA2D_BPP         8
#    define STM32_DMA2D_COLOR_FMT   DMA2D_PF_L8
#  elif defined(CONFIG_STM32F7_DMA2D_RGB565)
#    define STM32_DMA2D_STRIDE      ((STM32_DMA2D_WIDTH * 16 + 7) / 8)
#    define STM32_DMA2D_BPP         16
#    define STM32_DMA2D_COLOR_FMT   DMA2D_PF_RGB565
#  elif defined(CONFIG_STM32F7_DMA2D_RGB888)
#    define STM32_DMA2D_STRIDE      ((STM32_DMA2D_WIDTH * 24 + 7) / 8)
#    define STM32_DMA2D_BPP         24
#    define STM32_DMA2D_COLOR_FMT   DMA2D_PF_RGB888
#  elif defined(CONFIG_STM32F7_DMA2D_ARGB8888)
#    define STM32_DMA2D_STRIDE      ((STM32_DMA2D_WIDTH * 32 + 7) / 8)
#    define STM32_DMA2D_BPP         32
#    define STM32_DMA2D_COLOR_FMT   DMA2D_PF_ARGB8888
#  else
#    error "DMA2D pixel format not supported"
#  endif

#  ifdef CONFIG_STM32F7_DMA2D_LAYER_SHARED
#    define STM32_DMA2D_FBSIZE      CONFIG_STM32F7_DMA2D_FB_SIZE
#    define STM32_DMA2D_LAYER_SIZE  0
#  else
#    define STM32_DMA2D_FBSIZE      CONFIG_STM32F7_DMA2D_FB_SIZE / DMA2D_NLAYERS
#    define STM32_DMA2D_LAYER_SIZE  STM32_DMA2D_FBSIZE
#    if STM32_DMA2D_FBSIZE * DMA2D_NLAYERS > CONFIG_STM32F7_DMA2D_FB_SIZE
#      error "DMA2D framebuffer size to small for configured number of overlays"
#    endif
#  endif /* CONFIG_STM32F7_DMA2D_LAYER_SHARED */

#  define STM32_DMA2D_HEIGHT         STM32_DMA2D_FBSIZE / STM32_DMA2D_STRIDE

#  define STM32_DMA2D_BUFFER_START   CONFIG_STM32F7_DMA2D_FB_BASE
#else
#  define DMA2D_NLAYERS              0
#endif /* CONFIG_STM32F7_DMA2D */

#define LTDC_NOVERLAYS              LTDC_NLAYERS + DMA2D_NLAYERS

/* Dithering */

#ifndef CONFIG_STM32F7_LTDC_DITHER_RED
#  define STM32_LTDC_DITHER_RED     0
#else
#  define STM32_LTDC_DITHER_RED     CONFIG_STM32F7_LTDC_DITHER_RED
#endif
#ifndef CONFIG_STM32F7_LTDC_DITHER_GREEN
#  define STM32_LTDC_DITHER_GREEN   0
#else
#  define STM32_LTDC_DITHER_GREEN   CONFIG_STM32F7_LTDC_DITHER_GREEN
#endif
#ifndef CONFIG_STM32F7_LTDC_DITHER_BLUE
#  define STM32_LTDC_DITHER_BLUE    0
#else
#  define STM32_LTDC_DITHER_BLUE    CONFIG_STM32F7_LTDC_DITHER_BLUE
#endif

/* Background color */

#ifndef CONFIG_STM32F7_LTDC_BACKCOLOR
#  define STM32_LTDC_BACKCOLOR      0
#else
#  define STM32_LTDC_BACKCOLOR      CONFIG_STM32F7_LTDC_BACKCOLOR
#endif

/* Layer default color */

#ifdef CONFIG_STM32F7_LTDC_L1_COLOR
#  define STM32_LTDC_L1_COLOR        CONFIG_STM32F7_LTDC_L1_COLOR
#else
#  define STM32_LTDC_L1_COLOR        0x000000
#endif

#ifdef CONFIG_STM32F7_LTDC_L2
#  ifdef CONFIG_STM32F7_LTDC_L2_COLOR
#    define STM32_LTDC_L2_COLOR        CONFIG_STM32F7_LTDC_L2_COLOR
#  else
#    define STM32_LTDC_L2_COLOR        0x000000
#  endif
#endif

/* Internal operation flags */

#define LTDC_LAYER_SETAREA          (1 << 0) /* Change visible area */
#define LTDC_LAYER_SETALPHAVALUE    (1 << 1) /* Change constant alpha value */
#define LTDC_LAYER_SETBLENDMODE     (1 << 2) /* Change blendmode */
#define LTDC_LAYER_SETCOLORKEY      (1 << 3) /* Change color key */
#define LTDC_LAYER_ENABLECOLORKEY   (1 << 4) /* Enable colorkey */
#define LTDC_LAYER_SETCOLOR         (1 << 5) /* Change default color */
#define LTDC_LAYER_SETENABLE        (1 << 6) /* Change enabled state */
#define LTDC_LAYER_ENABLE           (1 << 7) /* Enable the layer */

/* Layer initializing state */

#define LTDC_LAYER_INIT             LTDC_LAYER_SETAREA | \
                                    LTDC_LAYER_SETALPHAVALUE | \
                                    LTDC_LAYER_SETBLENDMODE | \
                                    LTDC_LAYER_SETCOLORKEY | \
                                    LTDC_LAYER_SETCOLOR | \
                                    LTDC_LAYER_SETENABLE | \
                                    LTDC_LAYER_ENABLE

/* Blendfactor reset values for flip operation */

#define STM32_LTDC_BF1_RESET        6
#define STM32_LTDC_BF2_RESET        7

/* Check pixel format support by DMA2D driver */

#ifdef CONFIG_STM32F7_DMA2D
#  if defined(CONFIG_STM32F7_LTDC_L1_L8) || \
      defined(CONFIG_STM32F7_LTDC_L2_L8)
#    if !defined(CONFIG_STM32F7_DMA2D_L8)
#      error "DMA2D must support FB_FMT_RGB8 pixel format"
#    endif
#  endif
#  if defined(CONFIG_STM32F7_LTDC_L1_RGB565) || \
      defined(CONFIG_STM32F7_LTDC_L2_RGB565)
#    if !defined(CONFIG_STM32F7_DMA2D_RGB565)
#      error "DMA2D must support FB_FMT_RGB16_565 pixel format"
#    endif
#  endif
#  if defined(CONFIG_STM32F7_LTDC_L1_RGB888) || \
      defined(CONFIG_STM32F7_LTDC_L2_RGB888)
#    if !defined(CONFIG_STM32F7_DMA2D_RGB888)
#      error "DMA2D must support FB_FMT_RGB24 pixel format"
#    endif
#  endif
#  if defined(CONFIG_STM32F7_LTDC_L1_ARGB8888) || \
      defined(CONFIG_STM32F7_LTDC_L2_ARGB8888)
#    if !defined(CONFIG_STM32F7_DMA2D_ARGB8888)
#      error "DMA2D must support FB_FMT_RGB32 pixel format"
#    endif
#  endif
#endif

/* Calculate the size of the layers clut table */

#ifdef CONFIG_STM32F7_FB_CMAP
#  if defined(CONFIG_STM32F7_DMA2D) && !defined(CONFIG_STM32F7_DMA2D_L8)
#    error "DMA2D must also support L8 CLUT pixel format if supported by LTDC"
#  endif
#  ifdef STM32_LTDC_L1CMAP
#    ifdef CONFIG_STM32F7_FB_TRANSPARENCY
#      define STM32_LAYER_CLUT_SIZE STM32_LTDC_NCLUT * sizeof(uint32_t)
#    else
#      define STM32_LAYER_CLUT_SIZE STM32_LTDC_NCLUT * 3 * sizeof(uint8_t)
#    endif
#  endif
#  ifdef STM32_LTDC_L2CMAP
#    undef  STM32_LAYER_CLUT_SIZE
#    ifdef CONFIG_STM32F7_FB_TRANSPARENCY
#      define STM32_LAYER_CLUT_SIZE STM32_LTDC_NCLUT * sizeof(uint32_t) * 2
#    else
#      define STM32_LAYER_CLUT_SIZE STM32_LTDC_NCLUT * 3 * sizeof(uint8_t) * 2
#    endif
#  endif
#endif

#ifndef CONFIG_STM32F7_FB_CMAP
#  if defined(STM32_LTDC_L1CMAP) || defined(STM32_LTDC_L2CMAP)
#    undef STM32_LTDC_L1CMAP
#    undef STM32_LTDC_L2CMAP
#    error "Enable cmap to support the configured layer format!"
#  endif
#endif

/* Layer clut rgb value positioning */

#define LTDC_L1CLUT_REDOFFSET       0
#define LTDC_L1CLUT_GREENOFFSET     256
#define LTDC_L1CLUT_BLUEOFFSET      512
#define LTDC_L2CLUT_REDOFFSET       768
#define LTDC_L2CLUT_GREENOFFSET     1024
#define LTDC_L2CLUT_BLUEOFFSET      1280

/* Layer argb clut register position */

#define LTDC_CLUT_ADD(n)            ((uint32_t)(n) << 24)
#define LTDC_CLUT_ALPHA(n)          LTDC_CLUT_ADD(n)
#define LTDC_CLUT_RED(n)            ((uint32_t)(n) << 16)
#define LTDC_CLUT_GREEN(n)          ((uint32_t)(n) << 8)
#define LTDC_CLUT_BLUE(n)           ((uint32_t)(n) << 0)
#define LTDC_CLUT_RGB888_MASK       0xffffff

/* Layer argb cmap conversion */

#define LTDC_CMAP_ALPHA(n)          ((uint32_t)(n) >> 24)
#define LTDC_CMAP_RED(n)            ((uint32_t)(n) >> 16)
#define LTDC_CMAP_GREEN(n)          ((uint32_t)(n) >> 8)
#define LTDC_CMAP_BLUE(n)           ((uint32_t)(n) >> 0)

/* Hardware acceleration support */

/* Acceleration support for LTDC overlays */

#ifdef CONFIG_STM32F7_LTDC_L1_CHROMAKEYEN
#  define STM32_LTDC_L1_CHROMAEN    true
#  define STM32_LTDC_L1_CHROMAKEY   CONFIG_STM32F7_LTDC_L1_CHROMAKEY
#  define LTDC_LTDC_ACCL_L1         FB_ACCL_TRANSP | FB_ACCL_CHROMA
#else
#  define STM32_LTDC_L1_CHROMAEN    false
#  define STM32_LTDC_L1_CHROMAKEY   0
#  define LTDC_LTDC_ACCL_L1         FB_ACCL_TRANSP
#endif

#ifdef CONFIG_STM32F7_LTDC_L2_CHROMAKEYEN
#  define STM32_LTDC_L2_CHROMAEN    true
#  define STM32_LTDC_L2_CHROMAKEY   CONFIG_STM32F7_LTDC_L2_CHROMAKEY
#  define LTDC_LTDC_ACCL_L2         FB_ACCL_TRANSP | FB_ACCL_CHROMA
#else
#  define STM32_LTDC_L2_CHROMAEN    false
#  define STM32_LTDC_L2_CHROMAKEY   0
#  define LTDC_LTDC_ACCL_L2         FB_ACCL_TRANSP
#endif

#ifdef CONFIG_STM32F7_DMA2D
#  ifdef CONFIG_FB_OVERLAY_BLIT
#    ifdef CONFIG_STM32F7_FB_CMAP
#      define LTDC_BLIT_ACCL        FB_ACCL_BLIT
#    else
#      define LTDC_BLIT_ACCL        FB_ACCL_BLIT | FB_ACCL_BLEND
#    endif /* CONFIG_STM32F7_FB_CMAP */
#  else
#    define LTDC_BLIT_ACCL          0
#  endif /* CONFIG_FB_OVERLAY_BLIT */

#  ifdef CONFIG_STM32F7_FB_CMAP
#    define LTDC_DMA2D_ACCL         LTDC_BLIT_ACCL
#  else
#    define LTDC_DMA2D_ACCL         FB_ACCL_COLOR | LTDC_BLIT_ACCL
#  endif /* CONFIG_STM32F7_FB_CMAP */
#else
#  define LTDC_DMA2D_ACCL           0
#endif /* CONFIG_STM32F7_DMA2D */

#define LTDC_L1_ACCL                LTDC_LTDC_ACCL_L1 | LTDC_DMA2D_ACCL
#ifdef CONFIG_STM32F7_LTDC_L2
#  define LTDC_L2_ACCL              LTDC_LTDC_ACCL_L2 | LTDC_DMA2D_ACCL
#endif

/* Acceleration support for DMA2D overlays */

#ifdef CONFIG_STM32F7_FB_CMAP
#  ifdef CONFIG_FB_OVERLAY_BLIT
#    define DMA2D_ACCL              FB_ACCL_BLIT | FB_ACCL_AREA
#  else
#    define DMA2D_ACCL              FB_ACCL_AREA
#  endif
#else
#  ifdef CONFIG_FB_OVERLAY_BLIT
#    define DMA2D_ACCL              FB_ACCL_AREA  | \
                                    FB_ACCL_TRANSP | \
                                    FB_ACCL_COLOR | \
                                    FB_ACCL_BLIT | \
                                    FB_ACCL_BLEND
#  else
#    define DMA2D_ACCL              FB_ACCL_AREA  | \
                                    FB_ACCL_TRANSP | \
                                    FB_ACCL_COLOR
#  endif
#endif

/* Helper */

#define MIN(x,y)                    ((x) < (y) ? (x) : (y))

/* Color normalization */

#if defined(CONFIG_STM32F7_LTDC_L1_RGB565)
#  define RGB888_R(x)               (((((x) >> 11) & 0x1f) * 527 + 23) >> 6)
#  define RGB888_G(x)               (((((x) >> 5) & 0x3f) * 259 + 33) >> 6)
#  define RGB888_B(x)               ((((x) & 0x1f) * 527 + 23) >> 6)
#  define ARGB8888(x)               ((RGB888_R(x) << 16) | \
                                     (RGB888_G(x) << 8)  | \
                                      RGB888_B(x))
#else
#  define ARGB8888(x)               (x)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This enumeration names each layer supported by the hardware */

enum stm32_layer_e
{
  LTDC_LAYER_L1 = 0,       /* LCD Layer 1 */
  LTDC_LAYER_L2,           /* LCD Layer 2 */
};

/* LTDC General layer information */

struct stm32_ltdc_s
{
  int layerno;                                /* layer number */

#ifdef CONFIG_FB_OVERLAY
  struct   fb_overlayinfo_s oinfo;            /* Overlay info */
#endif

#ifdef CONFIG_STM32F7_DMA2D
  struct stm32_dma2d_overlay_s dma2dinfo;     /* Overlay info for DMA2D */
#endif

  mutex_t *lock;                                /* Layer exclusive access */
};

/* This structure provides the overall state of the LTDC layer */

struct stm32_ltdcdev_s
{
  /* Framebuffer interface */

  struct fb_vtable_s vtable;

  /* Framebuffer video information */

  struct fb_videoinfo_s vinfo;

  /* Framebuffer plane information */

  struct fb_planeinfo_s pinfo;

  /* Cmap information */

#ifdef CONFIG_STM32F7_FB_CMAP
  struct fb_cmap_s cmap;
#endif

  /* Layer information */

  struct stm32_ltdc_s layer[LTDC_NOVERLAYS];

#ifdef CONFIG_STM32F7_DMA2D
  /* Interface to the dma2d controller */

  struct dma2d_layer_s *dma2d;
#endif
};

/* Interrupt handling */

struct stm32_interrupt_s
{
  int   irq;        /* irq number */
  int error;        /* Interrupt error */
  sem_t *sem;       /* Semaphore for waiting for irq */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Overal LTDC helper */

static void stm32_ltdc_enable(bool enable);
static void stm32_ltdc_gpioconfig(void);
static void stm32_ltdc_periphconfig(void);
static void stm32_ltdc_bgcolor(uint32_t rgb);
static void stm32_ltdc_dither(bool enable, uint8_t red,
                              uint8_t green, uint8_t blue);
static int stm32_ltdcirq(int irq, void *context, void *arg);
static int stm32_ltdc_waitforirq(void);
static int stm32_ltdc_reload(uint8_t value, bool waitvblank);

/* Helper for layer register configuration */

static void stm32_ltdc_lpixelformat(struct stm32_ltdc_s *layer);
static void stm32_ltdc_lframebuffer(struct stm32_ltdc_s *layer);
static void stm32_ltdc_lenable(struct stm32_ltdc_s *layer, bool enable);
static void stm32_ltdc_ldefaultcolor(struct stm32_ltdc_s *layer,
                                     uint32_t rgb);
static void stm32_ltdc_ltransp(struct stm32_ltdc_s *layer,
                               uint8_t transp,
                               uint32_t mode);
static void stm32_ltdc_lchromakey(struct stm32_ltdc_s *layer,
                                  uint32_t chromakey);
static void stm32_ltdc_lchromakeyenable(struct stm32_ltdc_s *layer,
                                        bool enable);
static void stm32_ltdc_linit(uint8_t lid);

#ifdef CONFIG_STM32F7_DMA2D
static void stm32_ltdc_dma2dlinit(void);

#  ifdef CONFIG_FB_OVERLAY_BLIT
static bool stm32_ltdc_lvalidate(const struct stm32_ltdc_s *layer,
                                 const struct fb_area_s *area);
#  endif
#endif

#ifdef CONFIG_STM32F7_FB_CMAP
static void stm32_ltdc_lputclut(struct stm32_ltdc_s *layer,
                                const struct fb_cmap_s *cmap);
static void stm32_ltdc_lgetclut(struct stm32_ltdc_s *layer,
                                struct fb_cmap_s *cmap);
static void stm32_ltdc_lclutenable(struct stm32_ltdc_s *layer,
                                   bool enable);
#endif

static void stm32_ltdc_lclear(uint8_t overlayno);

/* Framebuffer interface */

static int stm32_getvideoinfo(struct fb_vtable_s *vtable,
                              struct fb_videoinfo_s *vinfo);
static int stm32_getplaneinfo(struct fb_vtable_s *vtable,
                              int planeno,
                              struct fb_planeinfo_s *pinfo);

/* The following is provided only if the video hardware supports RGB color
 * mapping
 */

#ifdef CONFIG_STM32F7_FB_CMAP
static int stm32_getcmap(struct fb_vtable_s *vtable,
                         struct fb_cmap_s *cmap);
static int stm32_putcmap(struct fb_vtable_s *vtable,
                         const struct fb_cmap_s *cmap);
#endif

/* The following is provided only if the video hardware signals vertical
 * synchronisation
 */

#ifdef CONFIG_FB_SYNC
static int stm32_waitforvsync(struct fb_vtable_s *vtable);
#endif

/* The following is provided only if the video hardware supports overlays */

#ifdef CONFIG_FB_OVERLAY
static int stm32_getoverlayinfo(struct fb_vtable_s *vtable,
                                int overlayno,
                                struct fb_overlayinfo_s *oinfo);
static int stm32_settransp(struct fb_vtable_s *vtable,
                           const struct fb_overlayinfo_s *oinfo);
static int stm32_setchromakey(struct fb_vtable_s *vtable,
                              const struct fb_overlayinfo_s *oinfo);
static int stm32_setcolor(struct fb_vtable_s *vtable,
                          const struct fb_overlayinfo_s *oinfo);
static int stm32_setblank(struct fb_vtable_s *vtable,
                          const struct fb_overlayinfo_s *oinfo);
static int stm32_setarea(struct fb_vtable_s *vtable,
                         const struct fb_overlayinfo_s *oinfo);

/* The following is provided only if the video hardware supports blit and
 * blend operation
 */

#  ifdef CONFIG_FB_OVERLAY_BLIT
static int stm32_blit(struct fb_vtable_s *vtable,
                      const struct fb_overlayblit_s *blit);
static int stm32_blend(struct fb_vtable_s *vtable,
                       const struct fb_overlayblend_s *blend);
#  endif /* CONFIG_FB_OVERLAY_BLIT */
#endif /* CONFIG_FB_OVERLAY */

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* PIO pin configurations */

static const uint32_t g_ltdcpins[] =
{
  GPIO_LTDC_R4, GPIO_LTDC_R5, GPIO_LTDC_R6, GPIO_LTDC_R7,
  GPIO_LTDC_G4, GPIO_LTDC_G5, GPIO_LTDC_G6, GPIO_LTDC_G7,
  GPIO_LTDC_B4, GPIO_LTDC_B5, GPIO_LTDC_B6, GPIO_LTDC_B7,
#if BOARD_LTDC_OUTPUT_BPP > 12
  GPIO_LTDC_R3, GPIO_LTDC_G2, GPIO_LTDC_G3, GPIO_LTDC_B3,
#  if BOARD_LTDC_OUTPUT_BPP > 16
  GPIO_LTDC_R2, GPIO_LTDC_B2,
#    if BOARD_LTDC_OUTPUT_BPP > 18
  GPIO_LTDC_R0, GPIO_LTDC_R1, GPIO_LTDC_G0, GPIO_LTDC_G1,
  GPIO_LTDC_B0, GPIO_LTDC_B1,
#    endif
#  endif
#endif
  GPIO_LTDC_VSYNC, GPIO_LTDC_HSYNC, GPIO_LTDC_DE, GPIO_LTDC_CLK
};

#define STM32_LTDC_NPINCONFIGS (sizeof(g_ltdcpins) / sizeof(uint32_t))

#ifdef CONFIG_STM32F7_FB_CMAP
/* The layers clut table entries */

static uint8_t g_redclut[STM32_LTDC_NCLUT];
static uint8_t g_greenclut[STM32_LTDC_NCLUT];
static uint8_t g_blueclut[STM32_LTDC_NCLUT];
#  ifdef CONFIG_STM32F7_FB_TRANSPARENCY
static uint8_t g_transpclut[STM32_LTDC_NCLUT];
#  endif
#endif /* CONFIG_STM32F7_FB_CMAP */

/* The LTDC mutex that enforces mutually exclusive access */

static mutex_t g_lock = NXMUTEX_INITIALIZER;

/* The semaphore for interrupt handling */

static sem_t g_semirq = SEM_INITIALIZER(0);

/* This structure provides irq handling */

static struct stm32_interrupt_s g_interrupt =
{
  .irq     = STM32_IRQ_LTDCINT,
  .error   = OK,
  .sem     = &g_semirq
};

/* This structure provides the internal interface */

static struct stm32_ltdcdev_s g_vtable =
{
  .vtable =
    {
      .getvideoinfo    = stm32_getvideoinfo,
      .getplaneinfo    = stm32_getplaneinfo
#ifdef CONFIG_FB_SYNC
      ,
      .waitforvsync    = stm32_waitforvsync
#endif

#ifdef CONFIG_STM32F7_FB_CMAP
      ,
      .getcmap         = stm32_getcmap,
      .putcmap         = stm32_putcmap
#endif

#ifdef CONFIG_FB_OVERLAY
      ,
      .getoverlayinfo  = stm32_getoverlayinfo,
      .settransp       = stm32_settransp,
      .setchromakey    = stm32_setchromakey,
      .setcolor        = stm32_setcolor,
      .setblank        = stm32_setblank,
      .setarea         = stm32_setarea
#  ifdef CONFIG_FB_OVERLAY_BLIT
      ,
      .blit            = stm32_blit,
      .blend           = stm32_blend
#  endif
#endif /* CONFIG_FB_OVERLAY */
  },
#ifdef CONFIG_STM32F7_LTDC_L2
  .pinfo =
    {
      .fbmem           = (uint8_t *)STM32_LTDC_BUFFER_L2,
      .fblen           = STM32_LTDC_L2_FBSIZE,
      .stride          = STM32_LTDC_L2_STRIDE,
      .display         = 0,
      .bpp             = STM32_LTDC_L2_BPP
    },
  .vinfo =
    {
      .fmt             = STM32_LTDC_L2_COLOR_FMT,
      .xres            = STM32_LTDC_WIDTH,
      .yres            = STM32_LTDC_HEIGHT,
      .nplanes         = 1,
#  ifdef CONFIG_FB_OVERLAY
      .noverlays       = LTDC_NOVERLAYS
#  endif
    }
#else
  .pinfo =
    {
      .fbmem           = (uint8_t *)STM32_LTDC_BUFFER_L1,
      .fblen           = STM32_LTDC_L1_FBSIZE,
      .stride          = STM32_LTDC_L1_STRIDE,
      .display         = 0,
      .bpp             = STM32_LTDC_L1_BPP
    },
  .vinfo =
    {
      .fmt             = STM32_LTDC_L1_COLOR_FMT,
      .xres            = STM32_LTDC_WIDTH,
      .yres            = STM32_LTDC_HEIGHT,
      .nplanes         = 1,
#  ifdef CONFIG_FB_OVERLAY
      .noverlays       = LTDC_NOVERLAYS
#  endif
    }
#endif /* CONFIG_STM32F7_LTDC_L2 */
  ,
#ifdef CONFIG_STM32F7_FB_CMAP
  .cmap =
    {
      .first           = 0,
      .len             = STM32_LTDC_NCLUT,
      .red             = g_redclut,
      .green           = g_greenclut,
      .blue            = g_blueclut,
#  ifdef CONFIG_STM32F7_FB_TRANSPARENCY
      .transp          = g_transpclut
#  endif
    }
  ,
#endif
  .layer[LTDC_LAYER_L1] =
    {
      .layerno = LTDC_LAYER_L1,
#ifdef CONFIG_FB_OVERLAY
      .oinfo =
        {
          .fbmem            = (uint8_t *)STM32_LTDC_BUFFER_L1,
          .fblen            = STM32_LTDC_L1_FBSIZE,
          .stride           = STM32_LTDC_L1_STRIDE,
          .overlay          = LTDC_LAYER_L1,
          .bpp              = STM32_LTDC_L1_BPP,
          .blank            = 0,
          .chromakey        = 0,
          .color            = 0,
          .transp =
            {
              .transp       = 0xff,
              .transp_mode  = FB_CONST_ALPHA
            },
          .sarea =
            {
              .x            = 0,
              .y            = 0,
              .w            = STM32_LTDC_WIDTH,
              .h            = STM32_LTDC_HEIGHT
            },
          .accl             = LTDC_L1_ACCL
        },
#endif

#ifdef CONFIG_STM32F7_DMA2D
      .dma2dinfo =
        {
            .fmt            = STM32_LTDC_L1_DMA2D_PF,
            .transp_mode    = STM32_DMA2D_PFCCR_AM_NONE,
            .xres           = STM32_LTDC_WIDTH,
            .yres           = STM32_LTDC_HEIGHT,
            .oinfo          = &g_vtable.layer[LTDC_LAYER_L1].oinfo
        },
#endif
      .lock = &g_lock
    }
#ifdef CONFIG_STM32F7_LTDC_L2
  ,
  .layer[LTDC_LAYER_L2] =
    {
      .layerno = LTDC_LAYER_L2,
#ifdef CONFIG_FB_OVERLAY
      .oinfo =
        {
          .overlay          = LTDC_LAYER_L2,
          .fbmem            = (uint8_t *)STM32_LTDC_BUFFER_L2,
          .fblen            = STM32_LTDC_L2_FBSIZE,
          .stride           = STM32_LTDC_L2_STRIDE,
          .bpp              = STM32_LTDC_L2_BPP,
          .blank            = 0,
          .chromakey        = 0,
          .color            = 0,
          .transp =
            {
              .transp       = 0xff,
              .transp_mode  = FB_CONST_ALPHA
            },
          .sarea =
            {
              .x            = 0,
              .y            = 0,
              .w            = STM32_LTDC_WIDTH,
              .h            = STM32_LTDC_HEIGHT
            },
          .accl             = LTDC_L2_ACCL
        },
#endif

#ifdef CONFIG_STM32F7_DMA2D
      .dma2dinfo =
        {
            .fmt            = STM32_LTDC_L2_DMA2D_PF,
            .transp_mode    = STM32_DMA2D_PFCCR_AM_NONE,
            .xres           = STM32_LTDC_WIDTH,
            .yres           = STM32_LTDC_HEIGHT,
            .oinfo          = &g_vtable.layer[LTDC_LAYER_L2].oinfo
        },
#endif
      .lock = &g_lock
    }
#endif
};

/* Configuration lookup tables */

/* LTDC width */

static const uint32_t stm32_width_layer_t[LTDC_NLAYERS] =
{
  STM32_LTDC_WIDTH
#ifdef CONFIG_STM32F7_LTDC_L2
  , STM32_LTDC_WIDTH
#endif
};

/* LTDC height */

static const uint32_t stm32_height_layer_t[LTDC_NLAYERS] =
{
  STM32_LTDC_HEIGHT
#ifdef CONFIG_STM32F7_LTDC_L2
  , STM32_LTDC_HEIGHT
#endif
};

/* LTDC stride */

static const uint32_t stm32_stride_layer_t[LTDC_NLAYERS] =
{
  STM32_LTDC_L1_STRIDE
#ifdef CONFIG_STM32F7_LTDC_L2
  , STM32_LTDC_L2_STRIDE
#endif
};

/* LTDC bpp */

static const uint32_t stm32_bpp_layer_t[LTDC_NLAYERS] =
{
  STM32_LTDC_L1_BPP
#ifdef CONFIG_STM32F7_LTDC_L2
  , STM32_LTDC_L2_BPP
#endif
};

/* LTDC framebuffer len */

static const uint32_t stm32_fblen_layer_t[LTDC_NLAYERS] =
{
  STM32_LTDC_L1_FBSIZE
#ifdef CONFIG_STM32F7_LTDC_L2
  , STM32_LTDC_L2_FBSIZE
#endif
};

/* LTDC framebuffer */

static const uint32_t stm32_fbmem_layer_t[LTDC_NLAYERS] =
{
  STM32_LTDC_BUFFER_L1
#ifdef CONFIG_STM32F7_LTDC_L2
  , STM32_LTDC_BUFFER_L2
#endif
};

/* LTDC default color lookup table */

static const uint32_t stm32_defaultcolor_layer_t[LTDC_NLAYERS] =
{
  STM32_LTDC_L1_COLOR
#ifdef CONFIG_STM32F7_LTDC_L2
  , STM32_LTDC_L2_COLOR
#endif
};

/* LTDC default chromakey */

static const uint32_t stm32_chromakey_layer_t[LTDC_NLAYERS] =
{
  STM32_LTDC_L1_CHROMAKEY
#ifdef CONFIG_STM32F7_LTDC_L2
  , STM32_LTDC_L2_CHROMAKEY
#endif
};

/* LTDC chromakey enabled state */

static const bool stm32_chromakeyen_layer_t[LTDC_NLAYERS] =
{
  STM32_LTDC_L1_CHROMAEN
#ifdef CONFIG_STM32F7_LTDC_L2
  , STM32_LTDC_L2_CHROMAEN
#endif
};

/* LTDC pixel format lookup table */

static const uint32_t stm32_fmt_layer_t[LTDC_NLAYERS] =
{
  STM32_LTDC_L1PFCR_PF
#ifdef CONFIG_STM32F7_LTDC_L2
  , STM32_LTDC_L2PFCR_PF
#endif
};

/* Register lookup tables */

/* LTDC_LxCR */

static const uintptr_t stm32_cr_layer_t[LTDC_NLAYERS] =
{
  STM32_LTDC_L1CR
#ifdef CONFIG_STM32F7_LTDC_L2
  , STM32_LTDC_L2CR
#endif
};

/* LTDC_LxWHPCR */

static const uintptr_t stm32_whpcr_layer_t[LTDC_NLAYERS] =
{
  STM32_LTDC_L1WHPCR
#ifdef CONFIG_STM32F7_LTDC_L2
  , STM32_LTDC_L2WHPCR
#endif
};

/* LTDC_LxWVPCR */

static const uintptr_t stm32_wvpcr_layer_t[LTDC_NLAYERS] =
{
  STM32_LTDC_L1WVPCR
#ifdef CONFIG_STM32F7_LTDC_L2
  , STM32_LTDC_L2WVPCR
#endif
};

/* LTDC_LxPFCR */

static const uintptr_t stm32_pfcr_layer_t[LTDC_NLAYERS] =
{
  STM32_LTDC_L1PFCR
#ifdef CONFIG_STM32F7_LTDC_L2
  , STM32_LTDC_L2PFCR
#endif
};

/* LTDC_LxDCCR */

static const uintptr_t stm32_dccr_layer_t[LTDC_NLAYERS] =
{
  STM32_LTDC_L1DCCR
#ifdef CONFIG_STM32F7_LTDC_L2
  , STM32_LTDC_L2DCCR
#endif
};

/* LTDC_LxCKCR */

static const uintptr_t stm32_ckcr_layer_t[LTDC_NLAYERS] =
{
  STM32_LTDC_L1CKCR
#ifdef CONFIG_STM32F7_LTDC_L2
  , STM32_LTDC_L2CKCR
#endif
};

/* LTDC_LxCACR */

static const uintptr_t stm32_cacr_layer_t[LTDC_NLAYERS] =
{
  STM32_LTDC_L1CACR
#ifdef CONFIG_STM32F7_LTDC_L2
  , STM32_LTDC_L2CACR
#endif
};

/* LTDC_LxBFCR */

static const uintptr_t stm32_bfcr_layer_t[LTDC_NLAYERS] =
{
  STM32_LTDC_L1BFCR
#ifdef CONFIG_STM32F7_LTDC_L2
  , STM32_LTDC_L2BFCR
#endif
};

/* LTDC_LxCFBAR */

static const uintptr_t stm32_cfbar_layer_t[LTDC_NLAYERS] =
{
  STM32_LTDC_L1CFBAR
#ifdef CONFIG_STM32F7_LTDC_L2
  , STM32_LTDC_L2CFBAR
#endif
};

/* LTDC_LxCFBLR */

static const uintptr_t stm32_cfblr_layer_t[LTDC_NLAYERS] =
{
  STM32_LTDC_L1CFBLR
#ifdef CONFIG_STM32F7_LTDC_L2
  , STM32_LTDC_L2CFBLR
#endif
};

/* LTDC_LxCFBLNR */

static const uintptr_t stm32_cfblnr_layer_t[LTDC_NLAYERS] =
{
  STM32_LTDC_L1CFBLNR
#ifdef CONFIG_STM32F7_LTDC_L2
  , STM32_LTDC_L2CFBLNR
#endif
};

/* LTDC_LxCLUTWR */

#ifdef CONFIG_STM32F7_FB_CMAP
static const uintptr_t stm32_clutwr_layer_t[LTDC_NLAYERS] =
{
  STM32_LTDC_L1CLUTWR
#  ifdef CONFIG_STM32F7_LTDC_L2
  , STM32_LTDC_L2CLUTWR
#  endif
};
#endif /* CONFIG_STM32F7_FB_CMAP */

/* The initialized state of the driver */

static bool g_initialized;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_ltdc_gpioconfig
 *
 * Description:
 *   Configure GPIO pins for use with the LTDC
 *
 ****************************************************************************/

static void stm32_ltdc_gpioconfig(void)
{
  int i;

  lcdinfo("Configuring pins\n");

  /* Configure each pin */

  for (i = 0; i < STM32_LTDC_NPINCONFIGS; i++)
    {
      reginfo("set gpio%d = %08x\n", i, g_ltdcpins[i]);
      stm32_configgpio(g_ltdcpins[i]);
    }
}

/****************************************************************************
 * Name: stm32_ltdc_periphconfig
 *
 * Description:
 *   Configures the synchronous timings
 *   Configures the synchronous signals and clock polarity
 *
 ****************************************************************************/

static void stm32_ltdc_periphconfig(void)
{
  uint32_t regval;

  /* Configure GPIO's */

  stm32_ltdc_gpioconfig();

  /* Configure APB2 LTDC clock external */

  reginfo("configured RCC_APB2ENR=%08x\n", getreg32(STM32_RCC_APB2ENR));

  /* Configure the SAI PLL external to provide the LCD_CLK */

  reginfo("configured RCC_PLLSAI=%08x\n", getreg32(STM32_RCC_PLLSAICFGR));

  /* Configure dedicated clock external.
   * Division factor for LCD_CLK in DCKCFGR1
   */

  reginfo("configured RCC_DCKCFGR1=%08x\n", getreg32(STM32_RCC_DCKCFGR1));

  /* Configure LTDC_SSCR */

  regval = (STM32_LTDC_SSCR_VSH | STM32_LTDC_SSCR_HSW);
  reginfo("set LTDC_SSCR=%08x\n", regval);
  putreg32(regval, STM32_LTDC_SSCR);
  reginfo("configured LTDC_SSCR=%08x\n", getreg32(STM32_LTDC_SSCR));

  /* Configure LTDC_BPCR */

  regval = (STM32_LTDC_BPCR_AVBP | STM32_LTDC_BPCR_AHBP);
  reginfo("set LTDC_BPCR=%08x\n", regval);
  putreg32(regval, STM32_LTDC_BPCR);
  reginfo("configured LTDC_BPCR=%08x\n", getreg32(STM32_LTDC_BPCR));

  /* Configure LTDC_AWCR */

  regval = (STM32_LTDC_AWCR_AAH | STM32_LTDC_AWCR_AAW);
  reginfo("set LTDC_AWCR=%08x\n", regval);
  putreg32(regval, STM32_LTDC_AWCR);
  reginfo("configured LTDC_AWCR=%08x\n", getreg32(STM32_LTDC_AWCR));

  /* Configure LTDC_TWCR */

  regval = (STM32_LTDC_TWCR_TOTALH | STM32_LTDC_TWCR_TOTALW);
  reginfo("set LTDC_TWCR=%08x\n", regval);
  putreg32(regval, STM32_LTDC_TWCR);
  reginfo("configured LTDC_TWCR=%08x\n", getreg32(STM32_LTDC_TWCR));

  /* Configure LTDC_GCR */

  regval  = getreg32(STM32_LTDC_GCR);
  regval &= ~(LTDC_GCR_PCPOL | LTDC_GCR_DEPOL | LTDC_GCR_VSPOL |
              LTDC_GCR_HSPOL);
  regval |= (STM32_LTDC_GCR_PCPOL | STM32_LTDC_GCR_DEPOL |
             STM32_LTDC_GCR_VSPOL | STM32_LTDC_GCR_HSPOL);

  reginfo("set LTDC_GCR=%08x\n", regval);
  putreg32(regval, STM32_LTDC_GCR);
  reginfo("configured LTDC_GCR=%08x\n", getreg32(STM32_LTDC_GCR));
}

/****************************************************************************
 * Name: stm32_ltdc_ldefaultcolor
 *
 * Description:
 *   Configures layer default color.
 *
 * Input Parameters:
 *   layer - Reference to the layer control structure
 *   rgb - RGB888 background color
 *
 ****************************************************************************/

static void stm32_ltdc_ldefaultcolor(struct stm32_ltdc_s *layer,
                                     uint32_t rgb)
{
  DEBUGASSERT(layer->layerno < LTDC_NLAYERS);
  reginfo("set LTDC_L%dDCCR=%08x\n", layer->layerno + 1, rgb);

  putreg32(rgb, stm32_dccr_layer_t[layer->layerno]);

  /* Reload shadow register */

  stm32_ltdc_reload(LTDC_SRCR_IMR, false);

  reginfo("configured LTDC_L%dDCCR=%08x\n", layer->layerno + 1,
          getreg32(STM32_LTDC_BCCR));
}

/****************************************************************************
 * Name: stm32_ltdc_bgcolor
 *
 * Description:
 *   Configures background color of the LCD controller.
 *
 * Input Parameters:
 *   rgb - RGB888 background color
 *
 ****************************************************************************/

static void stm32_ltdc_bgcolor(uint32_t rgb)
{
  reginfo("set LTDC_BCCR=%08x\n", rgb);
  putreg32(rgb, STM32_LTDC_BCCR);
  reginfo("configured LTDC_BCCR=%08x\n", getreg32(STM32_LTDC_BCCR));
}

/****************************************************************************
 * Name: stm32_ltdc_dither
 *
 * Description:
 *   Configures dither settings of the LCD controller.
 *
 * Input Parameters:
 *   enable - Enable dithering
 *   red    - Red dither width
 *   green  - Green dither width
 *   blue   - Blue dither width
 *
 ****************************************************************************/

static void stm32_ltdc_dither(bool enable, uint8_t red,
                              uint8_t green, uint8_t blue)
{
  uint32_t regval;

  regval = getreg32(STM32_LTDC_GCR);

  if (enable == true)
    {
      regval |= LTDC_GCR_DEN;
    }
  else
    {
      regval &= ~LTDC_GCR_DEN;
    }

  regval &= ~(LTDC_GCR_DBW_MASK | LTDC_GCR_DGW_MASK | LTDC_GCR_DRW_MASK);
  regval |= (LTDC_GCR_DRW(red) | LTDC_GCR_DGW(green) | LTDC_GCR_DBW(blue));

  reginfo("set LTDC_GCR=%08x\n", regval);
  putreg32(regval, STM32_LTDC_GCR);
  reginfo("configured LTDC_GCR=%08x\n", getreg32(STM32_LTDC_GCR));
}

/****************************************************************************
 * Name: stm32_ltdc_linepos
 *
 * Description:
 *   Configures line position register
 *
 ****************************************************************************/

static void stm32_ltdc_linepos(void)
{
  /* Configure LTDC_LIPCR */

  reginfo("set LTDC_LIPCR=%08x\n", STM32_LTDC_LIPCR_LIPOS);
  putreg32(STM32_LTDC_LIPCR_LIPOS, STM32_LTDC_LIPCR);
  reginfo("configured LTDC_LIPCR=%08x\n", getreg32(STM32_LTDC_LIPCR));
}

/****************************************************************************
 * Name: stm32_ltdc_irqctrl
 *
 * Description:
 *   Control  interrupts generated by the ltdc controller
 *
 * Input Parameters:
 *   setirqs  - set interrupt mask
 *   clrirqs  - clear interrupt mask
 *
 ****************************************************************************/

static void stm32_ltdc_irqctrl(uint32_t setirqs, uint32_t clrirqs)
{
  uint32_t regval;

  regval = getreg32(STM32_LTDC_IER);
  regval &= ~clrirqs;
  regval |= setirqs;
  reginfo("set LTDC_IER=%08x\n", regval);
  putreg32(regval, STM32_LTDC_IER);
  reginfo("configured LTDC_IER=%08x\n", getreg32(STM32_LTDC_IER));
}

/****************************************************************************
 * Name: stm32_ltdcirq
 *
 * Description:
 *   LTDC interrupt handler
 *
 ****************************************************************************/

static int stm32_ltdcirq(int irq, void *context, void *arg)
{
  int ret;
  struct stm32_interrupt_s *priv = &g_interrupt;
  uint32_t regval = getreg32(STM32_LTDC_ISR);

  reginfo("irq = %d, regval = %08x\n", irq, regval);

  if (regval & LTDC_ISR_RRIF)
    {
      /* Register reload interrupt */

      /* Clear the interrupt status register */

      reginfo("Register reloaded\n");
      putreg32(LTDC_ICR_CRRIF, STM32_LTDC_ICR);
      priv->error = OK;
    }
  else if (regval & LTDC_IER_LIE)
    {
      /* Line interrupt */

      /* Clear the interrupt status register */

      reginfo("Line interrupt\n");
      putreg32(LTDC_ICR_CLIF, STM32_LTDC_ICR);
      priv->error = OK;
    }
  else if (regval & LTDC_IER_TERRIE)
    {
      /* Transfer error interrupt */

      /* Clear the interrupt status register */

      reginfo("Error transfer\n");
      putreg32(LTDC_ICR_CTERRIF, STM32_LTDC_ICR);
      priv->error = -ECANCELED;
    }
  else if (regval & LTDC_IER_FUIE)
    {
      /* Fifo underrun error interrupt */

      /* Clear the interrupt status register */

      reginfo("Error fifo underrun\n");
      putreg32(LTDC_ICR_CFUIF, STM32_LTDC_ICR);
      priv->error = -ECANCELED;
    }
  else
    {
      DEBUGASSERT("Unknown interrupt");
    }

  /* Unlock the semaphore if locked */

  ret = nxsem_post(priv->sem);

  if (ret < 0)
    {
      lcderr("ERROR: nxsem_post() failed\n");
    }

  return OK;
}

/****************************************************************************
 * Name: stm32_ltdc_waitforirq
 *
 * Description:
 *   Helper waits until the ltdc irq occurs. In the current design That means
 *   that a register reload was been completed.
 *   Note! The caller must use this function within a critical section.
 *
 * Returned Value:
 *   OK - On success otherwise ERROR
 *
 ****************************************************************************/

static int stm32_ltdc_waitforirq(void)
{
  int ret = OK;
  struct stm32_interrupt_s *priv = &g_interrupt;

  ret = nxsem_wait(priv->sem);

  if (ret < 0)
    {
      lcderr("ERROR: nxsem_wait() failed\n");
    }

  ret = priv->error;

  return ret;
}

/****************************************************************************
 * Name: stm32_ltdc_reload
 *
 * Description:
 *   Reload the layer shadow register and make layer changes visible.
 *   Note! The caller must ensure that a previous register reloading has been
 *   completed.
 *
 * Input Parameters:
 *   value      - Reload flag (e.g. upon vertical blank or immediately)
 *   waitvblank - Wait until register reload is finished
 *
 ****************************************************************************/

static int stm32_ltdc_reload(uint8_t value, bool waitvblank)
{
  int ret = OK;

  /* Reloads the shadow register.
   * Note! This will not trigger an register reload interrupt if
   * immediately reload is set.
   */

  reginfo("set LTDC_SRCR=%08x\n", value);
  putreg32(value, STM32_LTDC_SRCR);
  reginfo("configured LTDC_SRCR=%08x\n", getreg32(STM32_LTDC_SRCR));

  if (value == LTDC_SRCR_VBR && waitvblank)
    {
      /* Wait upon vertical blanking period */

      ret = stm32_ltdc_waitforirq();
    }
  else
    {
      /* Wait until register reload hase been done */

      while (getreg32(STM32_LTDC_SRCR) & value);
    }

  return ret;
}

/****************************************************************************
 * Name: stm32_ltdc_irqconfig
 *
 * Description:
 *   Configure interrupts
 *
 ****************************************************************************/

static void stm32_ltdc_irqconfig(void)
{
  /* Attach LTDC interrupt vector */

  irq_attach(g_interrupt.irq, stm32_ltdcirq, NULL);

  /* Enable the IRQ at the NVIC */

  up_enable_irq(g_interrupt.irq);

  /* Enable interrupts expect line interrupt */

  stm32_ltdc_irqctrl(LTDC_IER_RRIE |
                     LTDC_IER_TERRIE |
                     LTDC_IER_FUIE,
                     LTDC_IER_LIE);

  /* Configure line interrupt */

  stm32_ltdc_linepos();
}

/****************************************************************************
 * Name: stm32_ltdc_globalconfig
 *
 * Description:
 *   Configure background color
 *   Configure dithering
 *
 ****************************************************************************/

static void stm32_ltdc_globalconfig(void)
{
  /* Configure dither */

  stm32_ltdc_dither(
#ifdef CONFIG_STM32F7_LTDC_DITHER
                    true,
#else
                    false,
#endif
                    STM32_LTDC_DITHER_RED,
                    STM32_LTDC_DITHER_GREEN,
                    STM32_LTDC_DITHER_BLUE);

  /* Configure background color */

  stm32_ltdc_bgcolor(STM32_LTDC_BACKCOLOR);
}

/****************************************************************************
 * Name: stm32_ltdc_enable
 *
 * Description:
 *   Disable the LCD peripheral
 *
 * Input Parameters:
 *   enable - Enable or disable
 *
 ****************************************************************************/

static void stm32_ltdc_enable(bool enable)
{
  uint32_t    regval;

  regval = getreg32(STM32_LTDC_GCR);
  reginfo("get LTDC_GCR=%08x\n", regval);

  if (enable == true)
    {
      regval |= LTDC_GCR_LTDCEN;
    }
  else
    {
      regval &= ~LTDC_GCR_LTDCEN;
    }

  reginfo("set LTDC_GCR=%08x\n", regval);
  putreg32(regval, STM32_LTDC_GCR);
  reginfo("configured LTDC_GCR=%08x\n", getreg32(STM32_LTDC_GCR));
}

/****************************************************************************
 * Name: stm32_ltdc_lpixelformat
 *
 * Description:
 *   Set the layer pixel format.
 *   Note! This changes have no effect until the shadow register reload has
 *   been done.
 *
 * Input Parameters:
 *   Reference to the layer control structure
 *
 ****************************************************************************/

static void stm32_ltdc_lpixelformat(struct stm32_ltdc_s *layer)
{
  uint8_t overlay = layer->layerno;
  DEBUGASSERT(layer->layerno < LTDC_NLAYERS);

  /* Configure PFCR register */

  reginfo("set LTDC_L%dPFCR=%08x\n", overlay + 1,
          stm32_fmt_layer_t[overlay]);
  putreg32(stm32_fmt_layer_t[overlay], stm32_pfcr_layer_t[overlay]);

  /* Reload shadow register */

  stm32_ltdc_reload(LTDC_SRCR_IMR, false);
}

/****************************************************************************
 * Name: stm32_ltdc_lframebuffer
 *
 * Description:
 *   Configure layer framebuffer of the entire window.
 *   Note! This changes have no effect until the shadow register reload has
 *   been done.
 *
 * Input Parameters:
 *   Reference to the layer control structure
 *
 ****************************************************************************/

static void stm32_ltdc_lframebuffer(struct stm32_ltdc_s *layer)
{
  uint32_t cfblr;
  uint32_t rxpos;
  uint32_t rypos;
  uint32_t whpcr;
  uint32_t wvpcr;
  uint8_t  layerno = layer->layerno;

  DEBUGASSERT(layer->layerno < LTDC_NLAYERS);
  reginfo("xpos = %d, ypos = %d, xres = %d, yres = %d\n", 0, 0,
          stm32_width_layer_t[layerno], stm32_height_layer_t[layerno]);

  /* Calculate register position */

  rxpos = STM32_LTDC_LXWHPCR_WHSTPOS + 1;
  rypos = STM32_LTDC_LXWVPCR_WVSTPOS + 1;

  /* Accumulate horizontal position */

  whpcr =  LTDC_LXWHPCR_WHSTPOS(rxpos);
  whpcr |= LTDC_LXWHPCR_WHSPPOS(rxpos + stm32_width_layer_t[layerno] - 1);

  /* Accumulate vertical position */

  wvpcr =  LTDC_LXWVPCR_WVSTPOS(rypos);
  wvpcr |= LTDC_LXWVPCR_WVSPPOS(rypos + stm32_height_layer_t[layerno] - 1);

  /* Configure LxWHPCR / LxWVPCR register */

  reginfo("set LTDC_L%dWHPCR=%08x\n", layerno + 1, whpcr);
  putreg32(whpcr, stm32_whpcr_layer_t[layerno]);
  reginfo("set LTDC_L%dWVPCR=%08x\n", layerno + 1, wvpcr);
  putreg32(wvpcr, stm32_wvpcr_layer_t[layerno]);

  /* Configure LxCFBAR register */

  reginfo("set LTDC_L%dCFBAR=%08x\n", layerno + 1,
          stm32_fbmem_layer_t[layerno]);
  putreg32(stm32_fbmem_layer_t[layerno], stm32_cfbar_layer_t[layerno]);

  /* Configure LxCFBLR register */

  /* Calculate line length */

  cfblr = LTDC_LXCFBLR_CFBP(stm32_stride_layer_t[layerno]) |
          LTDC_LXCFBLR_CFBLL(stm32_width_layer_t[layerno] *
          STM32_LTDC_LX_BYPP(stm32_bpp_layer_t[layerno]) + 3);

  reginfo("set LTDC_L%dCFBLR=%08x\n", layerno + 1, cfblr);
  putreg32(cfblr, stm32_cfblr_layer_t[layerno]);

  /* Configure LxCFBLNR register */

  reginfo("set LTDC_L%dCFBLNR=%08x\n", layerno + 1,
          stm32_height_layer_t[layerno]);
  putreg32(stm32_height_layer_t[layerno], stm32_cfblnr_layer_t[layerno]);

  /* Reload shadow register */

  stm32_ltdc_reload(LTDC_SRCR_IMR, false);
}

/****************************************************************************
 * Name: stm32_ltdc_lenable
 *
 * Description:
 *   Enable or disable layer.
 *   Note! This changes have no effect until the shadow register reload has
 *   been done.
 *
 * Input Parameters:
 *   layer  - Reference to the layer control structure
 *   enable - Enable or disable layer
 *
 ****************************************************************************/

static void stm32_ltdc_lenable(struct stm32_ltdc_s *layer, bool enable)
{
  uint32_t   regval;
  DEBUGASSERT(layer->layerno < LTDC_NLAYERS);

  regval = getreg32(stm32_cr_layer_t[layer->layerno]);

  if (enable == true)
    {
      regval |= LTDC_LXCR_LEN;
    }
  else
    {
      regval &= ~LTDC_LXCR_LEN;
    }

  /* Enable/Disable layer */

  reginfo("set LTDC_L%dCR=%08x\n", layer->layerno + 1, regval);
  putreg32(regval, stm32_cr_layer_t[layer->layerno]);

  /* Reload shadow register */

  stm32_ltdc_reload(LTDC_SRCR_IMR, false);
}

/****************************************************************************
 * Name: stm32_ltdc_ltransp
 *
 * Description:
 *   Change layer transparency.
 *   Note! This changes have no effect until the shadow register reload has
 *   been done.
 *
 * Input Parameters:
 *   layer  - Reference to the layer control structure
 *   transp - Transparency
 *   mode   - Transparency mode
 *
 ****************************************************************************/

static void stm32_ltdc_ltransp(struct stm32_ltdc_s *layer,
                               uint8_t transp, uint32_t mode)
{
  uint32_t bf1;
  uint32_t bf2;

  DEBUGASSERT(layer->layerno < LTDC_NLAYERS);

#ifdef CONFIG_FB_OVERLAY
  if (mode == FB_CONST_ALPHA)
    {
      bf1 = LTDC_BF1_CONST_ALPHA;
      bf2 = LTDC_BF2_CONST_ALPHA;
    }
  else
    {
      bf1 = LTDC_BF1_PIXEL_ALPHA;
      bf2 = LTDC_BF2_PIXEL_ALPHA;
    }
#else
  bf1 = LTDC_BF1_CONST_ALPHA;
  bf2 = LTDC_BF2_CONST_ALPHA;
#endif

  reginfo("set LTDC_L%dBFCR=%08x\n", layer->layerno + 1,
          (LTDC_LXBFCR_BF1(bf1) | LTDC_LXBFCR_BF2(bf2)));

  /* Set blendmode */

  putreg32((LTDC_LXBFCR_BF1(bf1) | LTDC_LXBFCR_BF2(bf2)),
            stm32_bfcr_layer_t[layer->layerno]);

  /* Set alpha */

  reginfo("set LTDC_L%dCACR=%02x\n", layer->layerno + 1, transp);
  putreg32(transp, stm32_cacr_layer_t[layer->layerno]);

  /* Reload shadow register */

  stm32_ltdc_reload(LTDC_SRCR_IMR, false);
}

/****************************************************************************
 * Name: stm32_ltdc_lchromakey
 *
 * Description:
 *   Change layer chromakey.
 *   Note! This changes have no effect until the shadow register reload has
 *   been done.
 *
 * Input Parameters:
 *   layer  - Reference to the layer control structure
 *   chroma - chromakey
 *
 ****************************************************************************/

static void stm32_ltdc_lchromakey(struct stm32_ltdc_s *layer,
                                  uint32_t chroma)
{
  uint32_t rgb;
  DEBUGASSERT(layer->layerno < LTDC_NLAYERS);

  reginfo("%08x\n", getreg32(stm32_cr_layer_t[layer->layerno]));

  /* Set chromakey */

#ifdef CONFIG_STM32F7_FB_CMAP
  uint8_t r = g_vtable.cmap.red[chroma];
  uint8_t g = g_vtable.cmap.green[chroma];
  uint8_t b = g_vtable.cmap.blue[chroma];
  rgb = ((r << 16) | (g << 8) | b);
#else
  rgb = ARGB8888(chroma);
#endif

  reginfo("set LTDC_L%dCKCR=%08x\n", layer->layerno + 1, rgb);
  putreg32(rgb, stm32_ckcr_layer_t[layer->layerno]);

  /* Reload shadow register */

  stm32_ltdc_reload(LTDC_SRCR_IMR, false);
}

/****************************************************************************
 * Name: stm32_ltdc_lchromakeyenable
 *
 * Description:
 *   Enable or disable layer chromakey support.
 *   Note! This changes have no effect until the shadow register reload has
 *   been done.
 *
 * Input Parameters:
 *   layer  - Reference to the layer control structure
 *   enable - Enable or disable chromakey
 *
 ****************************************************************************/

static void stm32_ltdc_lchromakeyenable(struct stm32_ltdc_s *layer,
                                        bool enable)
{
  uint32_t   regval;
  DEBUGASSERT(layer->layerno < LTDC_NLAYERS);

  regval = getreg32(stm32_cr_layer_t[layer->layerno]);

  /* Enable/Disable colorkey */

  if (enable == true)
    {
      regval |= LTDC_LXCR_COLKEN;
    }
  else
    {
      regval &= ~LTDC_LXCR_COLKEN;
    }

  reginfo("set LTDC_L%dCR=%08x\n", layer->layerno + 1, regval);
  putreg32(regval, stm32_cr_layer_t[layer->layerno]);

  /* Reload shadow register */

  stm32_ltdc_reload(LTDC_SRCR_IMR, false);
}

/****************************************************************************
 * Name: stm32_ltdc_lclutenable
 *
 * Description:
 *   Disable or enable the layer clut support
 *
 * Input Parameters:
 *   layer  - Reference to the layer control structure
 *   enable - Enable or disable
 *
 ****************************************************************************/

#ifdef CONFIG_STM32F7_FB_CMAP
static void stm32_ltdc_lclutenable(struct stm32_ltdc_s *layer,
                                   bool enable)
{
  uint32_t    regval;

  regval = getreg32(stm32_cr_layer_t[layer->oinfo.overlay]);
  reginfo("get LTDC_L%dCR=%08x\n", layer->oinfo.overlay + 1, regval);

  /* Disable the clut support during update the color table */

  if (enable == true)
    {
      regval |= LTDC_LXCR_CLUTEN;
    }
  else
    {
      regval &= ~LTDC_LXCR_CLUTEN;
    }

  reginfo("set LTDC_L%dCR=%08x\n", layer->oinfo.overlay, regval);
  putreg32(regval, stm32_cr_layer_t[layer->oinfo.overlay]);

  /* Reload shadow register */

  stm32_ltdc_reload(LTDC_SRCR_IMR, false);
}

/****************************************************************************
 * Name: stm32_ltdc_lputclut
 *
 * Description:
 *   Update the clut layer register during blank period.
 *   Note! The clut register is no shadow register.
 *
 * Input Parameters:
 *   layer  - Reference to the layer control structure
 *   cmap   - Color map
 *
 ****************************************************************************/

static void stm32_ltdc_lputclut(struct stm32_ltdc_s *layer,
                                const struct fb_cmap_s *cmap)
{
  int            n;
  irqstate_t flags;

  /* Disable clut during register update */

  stm32_ltdc_lclutenable(layer, false);

  /* Update the clut registers. Ensure operation is atomic or in interrupt
   * protected context.
   */

  flags = enter_critical_section();

  for (n = cmap->first; n < cmap->len && n < STM32_LTDC_NCLUT; n++)
    {
      uint32_t regval;

      regval = (uint32_t)LTDC_CLUT_ADD(n) |
               (uint32_t)LTDC_CLUT_RED(cmap->red[n]) |
               (uint32_t)LTDC_CLUT_GREEN(cmap->green[n]) |
               (uint32_t)LTDC_CLUT_BLUE(cmap->blue[n]);

      reginfo("set LTDC_L%dCLUTWR = %08x, first = %d, len = %d\n",
              layer->oinfo.overlay + 1, regval, cmap->first, cmap->len);
      putreg32(regval, stm32_clutwr_layer_t[layer->oinfo.overlay]);
    }

  leave_critical_section(flags);

  /* Enable clut after register update */

  stm32_ltdc_lclutenable(layer, true);

  /* Reload shadow control register */

  stm32_ltdc_reload(LTDC_SRCR_IMR, false);
}

/****************************************************************************
 * Name: stm32_ltdc_lgetclut
 *
 * Description:
 *   Copy the layers color lookup table.
 *
 * Input Parameters:
 *   layer  - Reference to the layer control structure
 *   cmap   - Color map
 *
 ****************************************************************************/

static void stm32_ltdc_lgetclut(struct stm32_ltdc_s *layer,
                                struct fb_cmap_s *cmap)
{
  int n;
  struct fb_cmap_s *priv_cmap = &g_vtable.cmap;

  /* Copy from internal cmap */

  for (n = cmap->first; n < cmap->len && n < STM32_LTDC_NCLUT; n++)
    {
#  ifdef CONFIG_STM32F7_FB_TRANSPARENCY
      cmap->transp[n] = priv_cmap->transp[n];
#  endif
      cmap->red[n]    = priv_cmap->red[n];
      cmap->green[n]  = priv_cmap->green[n];
      cmap->blue[n]   = priv_cmap->blue[n];

      reginfo("color = %d, transp=%02x, red=%02x, green=%02x, blue=%02x\n",
              n,
#  ifdef CONFIG_STM32F7_FB_TRANSPARENCY
              cmap->transp[n],
#  endif
              cmap->red[n],
              cmap->green[n],
              cmap->blue[n]);
    }
}
#endif /* CONFIG_STM32F7_FB_CMAP */

/****************************************************************************
 * Name: stm32_ltdc_lclear
 *
 * Description:
 *   Clear the whole layer
 *
 * Input Parameters:
 *   overlayno - Number overlay
 *
 ****************************************************************************/

static void stm32_ltdc_lclear(uint8_t overlayno)
{
  memset((uint8_t *)stm32_fbmem_layer_t[overlayno], 0,
        stm32_fblen_layer_t[overlayno]);
}

/****************************************************************************
 * Name: stm32_ltdc_lvalidate
 *
 * Description:
 *   Validates if the given area is within the overlay framebuffer memory
 *   region
 *
 * Input Parameters:
 *   layer  - Reference to the layer control structure
 *   area   - Reference to the overlay area
 *
 ****************************************************************************/

#if defined(CONFIG_STM32F7_DMA2D) && defined(CONFIG_FB_OVERLAY_BLIT)
static bool stm32_ltdc_lvalidate(const struct stm32_ltdc_s *layer,
                                 const struct fb_area_s *area)
{
  uint32_t offset;

  offset = (area->y + area->h - 1) * layer->oinfo.stride +
           (area->x + area->w) * layer->oinfo.bpp / 8;

  return (offset <= layer->oinfo.fblen && area->w > 0 && area->h > 0);
}
#endif /* defined(CONFIG_STM32F7_DMA2D) && defined(CONFIG_FB_OVERLAY_BLIT) */

/****************************************************************************
 * Name: stm32_ltdc_linit
 *
 * Description:
 *   Initialize layer to their default states.
 *
 *   Initialize:
 *   - layer framebuffer
 *   - layer pixelformat
 *   - layer defaultcolor
 *   - layer chromakey
 *   - layer transparency
 *   - layer clut
 *
 * Input Parameters:
 *   layer          - Reference to the layer control structure
 *
 ****************************************************************************/

static void stm32_ltdc_linit(uint8_t overlay)
{
  DEBUGASSERT(overlay < LTDC_NLAYERS);

  struct stm32_ltdcdev_s *dev = &g_vtable;
  struct stm32_ltdc_s *layer = &dev->layer[overlay];

  /* Disable layer */

  stm32_ltdc_lenable(layer, false);

  /* Clear the layer framebuffer */

  stm32_ltdc_lclear(overlay);

  /* Set layers framebuffer */

  stm32_ltdc_lframebuffer(layer);

  /* Set layers pixel input format */

  stm32_ltdc_lpixelformat(layer);

  /* Configure layer default color */

  stm32_ltdc_ldefaultcolor(layer, stm32_defaultcolor_layer_t[overlay]);

  /* Layers default transparency */

  stm32_ltdc_ltransp(layer, 0xff, 0);

  /* Layers chromakey */

  stm32_ltdc_lchromakey(layer, stm32_chromakey_layer_t[overlay]);

  /* Enable chromakey */

  stm32_ltdc_lchromakeyenable(layer, stm32_chromakeyen_layer_t[overlay]);

#ifdef CONFIG_STM32F7_FB_CMAP
  /* Disable clut by default */

  if (dev->vinfo.fmt == FB_FMT_RGB8)
    {
      /* Initialize LTDC clut register */

      stm32_ltdc_lputclut(layer, &g_vtable.cmap);

      /* Configure the clut register */

      stm32_ltdc_lclutenable(layer, true);
    }
#endif

  /* Finally enable the layer */

  stm32_ltdc_lenable(layer, true);
}

/****************************************************************************
 * Name: stm32_ltdc_dma2dlinit
 *
 * Description:
 *   Initialize dma2d layer to their default states.
 *
 *   Initialize:
 *   - layer framebuffer
 *   - layer pixelformat
 *   - layer size
 *   - layer color
 *   - layer chromakey
 *   - layer transparency
 *   - layer clut
 *
 * Input Parameters:
 *   layer - Reference to the layer control structure
 *
 ****************************************************************************/

#ifdef CONFIG_STM32F7_DMA2D
static void stm32_ltdc_dma2dlinit(void)
{
  int n;
  struct stm32_ltdcdev_s *dev = &g_vtable;

  for (n = 0; n < DMA2D_NLAYERS; n++)
    {
      uint32_t overlay = n + LTDC_NLAYERS;
      struct stm32_ltdc_s *layer = &dev->layer[overlay];
      uint8_t * fbmem = (uint8_t *)STM32_DMA2D_BUFFER_START;

      layer->layerno                  = overlay;
      layer->oinfo.fbmem              = fbmem + STM32_DMA2D_LAYER_SIZE * n;
      layer->oinfo.fblen              = STM32_DMA2D_FBSIZE;
      layer->oinfo.stride             = STM32_DMA2D_STRIDE;
      layer->oinfo.overlay            = overlay;
      layer->oinfo.bpp                = STM32_DMA2D_BPP;
      layer->oinfo.blank              = 0;
      layer->oinfo.chromakey          = 0;
      layer->oinfo.color              = 0;
      layer->oinfo.transp.transp      = 0xff;
      layer->oinfo.transp.transp_mode = 0;
      layer->oinfo.sarea.x            = 0;
      layer->oinfo.sarea.y            = 0;
      layer->oinfo.sarea.w            = STM32_DMA2D_WIDTH;
      layer->oinfo.sarea.h            = STM32_DMA2D_HEIGHT;
      layer->oinfo.accl               = DMA2D_ACCL;
      layer->lock                     = &g_lock;
      layer->dma2dinfo.fmt            = STM32_DMA2D_COLOR_FMT;
      layer->dma2dinfo.transp_mode    = STM32_DMA2D_PFCCR_AM_NONE;
      layer->dma2dinfo.xres           = layer->oinfo.sarea.w;
      layer->dma2dinfo.yres           = layer->oinfo.sarea.h;
      layer->dma2dinfo.oinfo          = &layer->oinfo;
    }
}
#endif /* CONFIG_STM32F7_DMA2D */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_getvideoinfo
 *
 * Description:
 *   Entrypoint ioctl FBIOGET_VIDEOINFO
 *   Get the videoinfo for the framebuffer
 *
 * Input Parameters:
 *   vtable - The framebuffer driver object
 *   vinfo  - the videoinfo object
 *
 * Returned Value:
 *   On success - OK
 *   On error   - -EINVAL
 *
 ****************************************************************************/

static int stm32_getvideoinfo(struct fb_vtable_s *vtable,
                              struct fb_videoinfo_s *vinfo)
{
  struct stm32_ltdcdev_s *priv = (struct stm32_ltdcdev_s *)vtable;

  lcdinfo("vtable=%p vinfo=%p\n", vtable, vinfo);
  DEBUGASSERT(vtable != NULL && priv == &g_vtable && vinfo != NULL);

  memcpy(vinfo, &priv->vinfo, sizeof(struct fb_videoinfo_s));
  return OK;
}

/****************************************************************************
 * Name: stm32_getplaneinfo
 *
 * Description:
 *   Entrypoint ioctl FBIOGET_PLANEINFO
 *   Get the planeinfo for the framebuffer
 *
 * Input Parameters:
 *   vtable - The framebuffer driver object
 *   pinfo  - the planeinfo object
 *
 * Returned Value:
 *   On success - OK
 *   On error   - -EINVAL
 *
 ****************************************************************************/

static int stm32_getplaneinfo(struct fb_vtable_s *vtable, int planeno,
                              struct fb_planeinfo_s *pinfo)
{
  struct stm32_ltdcdev_s *priv = (struct stm32_ltdcdev_s *)vtable;

  DEBUGASSERT(vtable != NULL && priv == &g_vtable);
  lcdinfo("vtable=%p planeno=%d pinfo=%p\n", vtable, planeno, pinfo);

  if (planeno == 0)
    {
      memcpy(pinfo, &priv->pinfo, sizeof(struct fb_planeinfo_s));
      return OK;
    }

  lcderr("ERROR: Returning EINVAL\n");
  return -EINVAL;
}

/****************************************************************************
 * Name: stm32_getcmap
 *
 * Description:
 *   Entrypoint ioctl FBIOGET_CMAP
 *   Get a range of CLUT values for the LCD
 *
 * Input Parameters:
 *   vtable - The framebuffer driver object
 *   cmap   - the color table
 *
 * Returned Value:
 *   On success - OK
 *   On error   - -EINVAL
 *
 ****************************************************************************/

#ifdef CONFIG_STM32F7_FB_CMAP
static int stm32_getcmap(struct fb_vtable_s *vtable,
                         struct fb_cmap_s *cmap)
{
  int ret;
  struct stm32_ltdcdev_s *priv = (struct stm32_ltdcdev_s *)vtable;

  DEBUGASSERT(vtable != NULL && priv == &g_vtable && cmap != NULL);
  lcdinfo("vtable=%p cmap=%p\n", vtable, cmap);

  if (priv->vinfo.fmt != FB_FMT_RGB8)
    {
      lcderr("ERROR: CLUT is not supported for the pixel format: %d\n",
              priv->vinfo.fmt);
      ret = -EINVAL;
    }
  else if (cmap->first >= STM32_LTDC_NCLUT)
    {
      lcderr("ERROR: only %d color table entries supported\n",
              STM32_LTDC_NCLUT);
      ret = -EINVAL;
    }
  else
    {
      /* Currently, there is no api to set color map for each overlay
       * separately. LTDC layers can have different color maps. Get the cmap
       * from the main overlay.
       */

      struct stm32_ltdc_s *layer;
#  ifdef CONFIG_STM32F7_LTDC_L2
      layer = &priv->layer[LTDC_LAYER_L2];
#  else
      layer = &priv->layer[LTDC_LAYER_L1];
#  endif
      nxmutex_lock(layer->lock);
      stm32_ltdc_lgetclut(layer, cmap);
      nxmutex_unlock(layer->lock);

      ret = OK;
    }

  return ret;
}

/****************************************************************************
 * Name: stm32_putcmap
 *
 * Description:
 *   Entrypoint ioctl FBIOPUT_CMAP
 *   Set a range of the CLUT values for the LCD
 *
 * Input Parameters:
 *   vtable - The framebuffer driver object
 *   cmap   - the color table
 *
 * Returned Value:
 *   On success - OK
 *   On error   - -EINVAL
 *
 ****************************************************************************/

static int stm32_putcmap(struct fb_vtable_s *vtable,
                         const struct fb_cmap_s *cmap)
{
  int ret;
  struct stm32_ltdcdev_s *priv = (struct stm32_ltdcdev_s *)vtable;

  DEBUGASSERT(vtable != NULL && priv == &g_vtable && cmap != NULL);
  lcdinfo("vtable=%p cmap=%p\n", vtable, cmap);

  if (priv->vinfo.fmt != FB_FMT_RGB8)
    {
      lcderr("ERROR: CLUT is not supported for the pixel format: %d\n",
              priv->vinfo.fmt);
      ret = -EINVAL;
    }
  else if (cmap->first >= STM32_LTDC_NCLUT)
    {
      lcderr("ERROR: only %d color table entries supported\n",
              STM32_LTDC_NCLUT);
      ret = -EINVAL;
    }
  else
    {
      /* Currently, there is no api to set color map for each overlay
       * separately. LTDC layers can have different color maps, but is shared
       * for now.
       */

      int n;
      struct fb_cmap_s *priv_cmap = &g_vtable.cmap;

      /* First copy to internal cmap */

      for (n = cmap->first; n < cmap->len && n < STM32_LTDC_NCLUT; n++)
        {
          priv_cmap->red[n] = cmap->red[n];
          priv_cmap->green[n] = cmap->green[n];
          priv_cmap->blue[n] = cmap->blue[n];
#  ifdef CONFIG_STM32F7_FB_TRANSPARENCY
          /* Not supported by LTDC */

          priv_cmap->transp[n] = cmap->transp[n];
#  endif
        }

      priv_cmap->first = cmap->first;
      priv_cmap->len = cmap->len;

      /* Update the layer clut register */

      nxmutex_lock(&g_lock);

      for (n = 0; n < LTDC_NLAYERS; n++)
        {
          struct stm32_ltdc_s *layer = &priv->layer[n];
          stm32_ltdc_lputclut(layer, priv_cmap);
        }

#  ifdef CONFIG_STM32F7_DMA2D
      /* Update dma2d cmap */

      priv->dma2d->setclut(cmap);
#  endif
      nxmutex_unlock(&g_lock);

      ret = OK;
    }

  return ret;
}
#endif /* CONFIG_STM32F7_FB_CMAP */

/****************************************************************************
 * Name: stm32_ioctl_waitforvsync
 * Description:
 *   Entrypoint ioctl FBIO_WAITFORSYNC
 ****************************************************************************/

#ifdef CONFIG_FB_SYNC
static int stm32_waitforvsync(struct fb_vtable_s *vtable)
{
  int ret;

  DEBUGASSERT(vtable != NULL && vtable == &g_vtable.vtable);

  /* Wait upon vertical synchronization. */

  ret = stm32_ltdc_reload(LTDC_SRCR_VBR, true);

  return ret;
}
#endif /* CONFIG_FB_SYNC */

/****************************************************************************
 * Name: stm32_getoverlayinfo
 * Description:
 *   Entrypoint ioctl FBIOGET_OVERLAYINFO
 ****************************************************************************/

#ifdef CONFIG_FB_OVERLAY
static int stm32_getoverlayinfo(struct fb_vtable_s *vtable,
                                int overlayno,
                                struct fb_overlayinfo_s *oinfo)
{
  struct stm32_ltdcdev_s *priv = (struct stm32_ltdcdev_s *)vtable;

  lcdinfo("vtable=%p overlay=%d oinfo=%p\n", vtable, overlayno, oinfo);
  DEBUGASSERT(vtable != NULL && priv == &g_vtable);

  if (overlayno < LTDC_NOVERLAYS)
    {
      struct stm32_ltdc_s *layer = &priv->layer[overlayno];
      memcpy(oinfo, &layer->oinfo, sizeof(struct fb_overlayinfo_s));
      return OK;
    }

  lcderr("ERROR: Returning EINVAL\n");
  return -EINVAL;
}

/****************************************************************************
 * Name: stm32_settransp
 * Description:
 *   Entrypoint ioctl FBIOSET_TRANSP
 ****************************************************************************/

static int stm32_settransp(struct fb_vtable_s *vtable,
                           const struct fb_overlayinfo_s *oinfo)
{
  struct stm32_ltdcdev_s *priv = (struct stm32_ltdcdev_s *)vtable;

  DEBUGASSERT(vtable != NULL && priv == &g_vtable);
  lcdinfo("vtable=%p, overlay=%d, transp=%02x, transp_mode=%02x\n", vtable,
          oinfo->overlay, oinfo->transp.transp, oinfo->transp.transp_mode);

  if (oinfo->transp.transp_mode > 1)
    {
      lcderr("ERROR: Returning ENOSYS, transparency mode not supported\n");
      return -ENOSYS;
    }

  if (oinfo->overlay < LTDC_NOVERLAYS)
    {
      struct stm32_ltdc_s *layer = &priv->layer[oinfo->overlay];

      nxmutex_lock(layer->lock);
      layer->oinfo.transp.transp      = oinfo->transp.transp;
      layer->oinfo.transp.transp_mode = oinfo->transp.transp_mode;

#  ifdef CONFIG_STM32F7_DMA2D
      if (layer->oinfo.transp.transp_mode == 0)
        {
          layer->dma2dinfo.transp_mode = STM32_DMA2D_PFCCR_AM_CONST;
        }
      else if (layer->oinfo.transp.transp_mode == 1)
        {
          layer->dma2dinfo.transp_mode = STM32_DMA2D_PFCCR_AM_PIXEL;
        }

      if (oinfo->overlay < LTDC_NLAYERS)
#  endif
        {
          /* Set LTDC blendmode and alpha value */

          stm32_ltdc_ltransp(layer, layer->oinfo.transp.transp,
                             layer->oinfo.transp.transp_mode);
        }

      nxmutex_unlock(layer->lock);
      return OK;
    }

  lcderr("ERROR: Returning EINVAL\n");
  return -EINVAL;
}

/****************************************************************************
 * Name: stm32_setchromakey
 * Description:
 *   Entrypoint ioctl FBIOSET_CHROMAKEY
 ****************************************************************************/

static int stm32_setchromakey(struct fb_vtable_s *vtable,
                              const struct fb_overlayinfo_s *oinfo)
{
  struct stm32_ltdcdev_s *priv = (struct stm32_ltdcdev_s *)vtable;

  DEBUGASSERT(vtable != NULL && priv == &g_vtable && oinfo != NULL);
  lcdinfo("vtable=%p, overlay=%d, chromakey=%08x\n", vtable,
          oinfo->overlay, oinfo->chromakey);

  if (oinfo->overlay < LTDC_NLAYERS)
    {
      int ret;
      struct stm32_ltdc_s *layer = &priv->layer[oinfo->overlay];

#  ifndef CONFIG_STM32F7_LTDC_L1_CHROMAKEY
      if (oinfo->overlay == LTDC_LAYER_L1)
        {
          return -ENOSYS;
        }
#  endif

#  ifndef CONFIG_STM32F7_LTDC_L2_CHROMAKEY
      if (oinfo->overlay == LTDC_LAYER_L2)
        {
          return -ENOSYS;
        }
#  endif

      nxmutex_lock(layer->lock);
#  ifdef CONFIG_STM32F7_FB_CMAP
      if (oinfo->chromakey >= g_vtable.cmap.len)
        {
          lcderr("ERROR: Clut index %d is out of range\n", oinfo->chromakey);
          ret = -EINVAL;
        }
      else
#  endif
        {
          layer->oinfo.chromakey = oinfo->chromakey;

          /* Set chromakey */

          stm32_ltdc_lchromakey(layer, layer->oinfo.chromakey);
          ret = OK;
        }

      nxmutex_unlock(layer->lock);
      return ret;
    }
#  ifdef CONFIG_STM32F7_DMA2D
  else if (oinfo->overlay < LTDC_NOVERLAYS)
    {
      /* Chromakey not supported by DMA2D */

      return -ENOSYS;
    }
#  endif

  lcderr("ERROR: Returning EINVAL\n");
  return -EINVAL;
}

/****************************************************************************
 * Name: stm32_setcolor
 * Description:
 *   Entrypoint ioctl FBIOSET_COLOR
 ****************************************************************************/

static int stm32_setcolor(struct fb_vtable_s *vtable,
                          const struct fb_overlayinfo_s *oinfo)
{
  DEBUGASSERT(vtable != NULL && vtable == &g_vtable.vtable && oinfo != NULL);
  lcdinfo("vtable=%p, overlay=%d, color=%08x\n", vtable, oinfo->color);

  if (oinfo->overlay < LTDC_NOVERLAYS)
    {
#  ifdef CONFIG_STM32F7_DMA2D

      /* Set color within the active overlay is not supported by LTDC. So use
       * DMA2D controller instead when configured.
       */

      int ret;
      struct stm32_ltdcdev_s *priv =
        (struct stm32_ltdcdev_s *)vtable;
      struct stm32_ltdc_s *layer = &priv->layer[oinfo->overlay];
      struct fb_overlayinfo_s *poverlay = layer->dma2dinfo.oinfo;

      DEBUGASSERT(&layer->oinfo == poverlay);

      nxmutex_lock(layer->lock);
      poverlay->color = oinfo->color;
      ret = priv->dma2d->fillcolor(&layer->dma2dinfo, &poverlay->sarea,
                                   poverlay->color);
      nxmutex_unlock(layer->lock);

      return ret;
#  else
      /* Coloring not supported by LTDC */

      return -ENOSYS;
#  endif
    }

  lcderr("ERROR: Returning EINVAL\n");
  return -EINVAL;
}

/****************************************************************************
 * Name: stm32_setblank
 * Description:
 *   Entrypoint ioctl FBIOSET_BLANK
 ****************************************************************************/

static int stm32_setblank(struct fb_vtable_s *vtable,
                          const struct fb_overlayinfo_s *oinfo)
{
  struct stm32_ltdcdev_s *priv = (struct stm32_ltdcdev_s *)vtable;

  DEBUGASSERT(vtable != NULL && priv == &g_vtable && oinfo != NULL);
  lcdinfo("vtable=%p, overlay=%d, blank=%02x\n", vtable, oinfo->blank);

  if (oinfo->overlay < LTDC_NLAYERS)
    {
      struct stm32_ltdc_s *layer = &priv->layer[oinfo->overlay];

      nxmutex_lock(layer->lock);
      layer->oinfo.blank = oinfo->blank;

      /* Enable or disable layer */

      stm32_ltdc_lenable(layer, (layer->oinfo.blank == 0));
      nxmutex_unlock(layer->lock);

      return OK;
    }
#  ifdef CONFIG_STM32F7_DMA2D
  else if (oinfo->overlay < LTDC_NOVERLAYS)
    {
      /* DMA2D overlays are non visible */

      return OK;
    }
#  endif

  lcderr("ERROR: Returning EINVAL\n");
  return -EINVAL;
}

/****************************************************************************
 * Name: stm32_setarea
 * Description:
 *   Entrypoint ioctl FBIOSET_AREA
 ****************************************************************************/

static int stm32_setarea(struct fb_vtable_s *vtable,
                         const struct fb_overlayinfo_s *oinfo)
{
  DEBUGASSERT(vtable != NULL && vtable == &g_vtable.vtable && oinfo != NULL);
  lcdinfo("vtable=%p, overlay=%d, x=%d, y=%d, w=%d, h=%d\n", vtable,
          oinfo->overlay, oinfo->sarea.x, oinfo->sarea.y, oinfo->sarea.w,
          oinfo->sarea.h);

  if (oinfo->overlay < LTDC_NLAYERS)
    {
      /* LTDC area is defined by the overlay size (display resolution) only */

      return -ENOSYS;
    }

#  ifdef CONFIG_STM32F7_DMA2D
  if (oinfo->overlay < LTDC_NOVERLAYS)
    {
      struct stm32_ltdcdev_s *priv =
        (struct stm32_ltdcdev_s *)vtable;
      struct stm32_ltdc_s *layer =
        &priv->layer[oinfo->overlay];

      nxmutex_lock(layer->lock);
      memcpy(&layer->oinfo.sarea, &oinfo->sarea, sizeof(struct fb_area_s));
      nxmutex_unlock(layer->lock);

      return OK;
    }
#  endif

  lcderr("ERROR: Returning EINVAL\n");
  return -EINVAL;
}

/****************************************************************************
 * Name: stm32_blit
 * Description:
 *   Entrypoint ioctl FBIOSET_BLIT
 ****************************************************************************/

#  ifdef CONFIG_FB_OVERLAY_BLIT
static int stm32_blit(struct fb_vtable_s *vtable,
                      const struct fb_overlayblit_s *blit)
{
  DEBUGASSERT(vtable != NULL && vtable == &g_vtable.vtable && blit != NULL);
  lcdinfo("vtable = %p, blit = %p\n", vtable, blit);

  if (blit->dest.overlay < LTDC_NOVERLAYS &&
      blit->src.overlay < LTDC_NOVERLAYS)
    {
#    ifdef CONFIG_STM32F7_DMA2D
      int ret;
      struct fb_area_s sarea;
      const struct fb_area_s *darea = &blit->dest.area;
      struct stm32_ltdcdev_s *priv =
        (struct stm32_ltdcdev_s *)vtable;
      struct stm32_ltdc_s *dlayer =
        &priv->layer[blit->dest.overlay];
      struct stm32_ltdc_s *slayer =
        &priv->layer[blit->src.overlay];

      DEBUGASSERT(&dlayer->oinfo == dlayer->dma2dinfo.oinfo &&
                  &slayer->oinfo == slayer->dma2dinfo.oinfo);

      /* DMA2D doesn't support image scale, so set to the smallest area */

      memcpy(&sarea, &blit->src.area, sizeof(struct fb_area_s));

      /* Check if area is within the entire overlay */

      if (!stm32_ltdc_lvalidate(dlayer, darea) ||
          !stm32_ltdc_lvalidate(slayer, &sarea))
        {
          return -EINVAL;
        }

      sarea.w = MIN(darea->w, sarea.w);
      sarea.h = MIN(darea->h, sarea.h);

      nxmutex_lock(dlayer->lock);
      ret = priv->dma2d->blit(&dlayer->dma2dinfo, darea->x, darea->y,
                              &slayer->dma2dinfo, &sarea);
      nxmutex_unlock(dlayer->lock);

      return ret;
#    else
      /* LTDC doesn't support blit transfer */

      return -ENOSYS;
#    endif
    }

  lcderr("ERROR: Returning EINVAL\n");
  return -EINVAL;
}

/****************************************************************************
 * Name: stm32_blend
 * Description:
 *   Entrypoint ioctl FBIOSET_BLEND
 ****************************************************************************/

static int stm32_blend(struct fb_vtable_s *vtable,
                       const struct fb_overlayblend_s *blend)
{
  DEBUGASSERT(vtable != NULL && vtable == &g_vtable.vtable && blend != NULL);
  lcdinfo("vtable = %p, blend = %p\n", vtable, blend);

  if (blend->dest.overlay < LTDC_NOVERLAYS &&
      blend->foreground.overlay < LTDC_NOVERLAYS &&
      blend->background.overlay < LTDC_NOVERLAYS)
    {
#    ifdef CONFIG_STM32F7_DMA2D
      int ret;
      struct fb_area_s barea;
      const struct fb_area_s *darea = &blend->dest.area;
      const struct fb_area_s *farea = &blend->foreground.area;
      struct stm32_ltdcdev_s *priv =
        (struct stm32_ltdcdev_s *)vtable;
      struct stm32_ltdc_s *dlayer = &priv->layer[blend->dest.overlay];
      struct stm32_ltdc_s *flayer =
        &priv->layer[blend->foreground.overlay];
      struct stm32_ltdc_s *blayer =
        &priv->layer[blend->background.overlay];

      DEBUGASSERT(&dlayer->oinfo == dlayer->dma2dinfo.oinfo &&
                  &flayer->oinfo == flayer->dma2dinfo.oinfo &&
                  &blayer->oinfo == blayer->dma2dinfo.oinfo);

      /* DMA2D doesn't support image scale, so set to the smallest area */

      memcpy(&barea, &blend->background.area, sizeof(struct fb_area_s));

      /* Check if area is within the entire overlay */

      if (!stm32_ltdc_lvalidate(dlayer, darea) ||
          !stm32_ltdc_lvalidate(flayer, farea) ||
          !stm32_ltdc_lvalidate(blayer, &barea))
        {
          lcderr("ERROR: Returning EINVAL\n");
          return -EINVAL;
        }

      barea.w = MIN(darea->w, barea.w);
      barea.h = MIN(darea->h, barea.h);
      barea.w = MIN(farea->w, barea.w);
      barea.h = MIN(farea->h, barea.h);

      nxmutex_lock(dlayer->lock);
      ret = priv->dma2d->blend(&dlayer->dma2dinfo, darea->x, darea->y,
                               &flayer->dma2dinfo, farea->x, farea->y,
                               &blayer->dma2dinfo, &barea);
      nxmutex_unlock(dlayer->lock);

      return ret;
#    else
      /* LTDC doesn't support blend transfer */

      return -ENOSYS;
#    endif
    }

  lcderr("ERROR: Returning EINVAL\n");
  return -EINVAL;
}
#  endif /* CONFIG_FB_OVERLAY_BLIT */
#endif /* CONFIG_FB_OVERLAY */

/****************************************************************************
 * Name: stm32_ltdcreset
 *
 * Description:
 *   Reset LTDC via APB2RSTR
 *
 ****************************************************************************/

void stm32_ltdcreset(void)
{
  uint32_t regval = getreg32(STM32_RCC_APB2RSTR);
  putreg32(regval | RCC_APB2RSTR_LTDCRST, STM32_RCC_APB2RSTR);
  putreg32(regval & ~RCC_APB2RSTR_LTDCRST, STM32_RCC_APB2RSTR);
}

/****************************************************************************
 * Name: stm32_ltdcinitialize
 *
 * Description:
 *   Initialize the ltdc controller
 *
 * Returned Value:
 *   OK
 *
 ****************************************************************************/

int stm32_ltdcinitialize(void)
{
  int ret = OK;

  lcdinfo("Initialize LTDC driver\n");

  if (g_initialized == true)
    {
      return ret;
    }

  /* Disable the LCD */

  stm32_ltdc_enable(false);

  lcdinfo("Configuring the LCD controller\n");

  /* Configure LCD periphery */

  lcdinfo("Configure lcd periphery\n");
  stm32_ltdc_periphconfig();

  /* Configure interrupts */

  lcdinfo("Configure interrupts\n");
  stm32_ltdc_irqconfig();

  /* Configure global ltdc register */

  lcdinfo("Configure global register\n");
  stm32_ltdc_globalconfig();

#ifdef CONFIG_STM32F7_DMA2D
  /* Initialize the dma2d controller */

  ret = stm32_dma2dinitialize();

  if (ret != OK)
    {
      return ret;
    }

  /* Bind the dma2d interface */

  g_vtable.dma2d = stm32_dma2ddev();
  DEBUGASSERT(g_vtable.dma2d != NULL);
#endif

#ifdef CONFIG_STM32F7_FB_CMAP
  /* Cleanup clut */

  memset(&g_redclut, 0, STM32_LTDC_NCLUT);
  memset(&g_blueclut, 0, STM32_LTDC_NCLUT);
  memset(&g_greenclut, 0, STM32_LTDC_NCLUT);
#  ifdef CONFIG_STM32F7_FB_TRANSPARENCY
  memset(&g_transpclut, 0, STM32_LTDC_NCLUT);
#  endif
#endif /* CONFIG_STM32F7_FB_CMAP */

  /* Initialize ltdc layer */

  lcdinfo("Initialize ltdc layer\n");
  stm32_ltdc_linit(LTDC_LAYER_L1);
#ifdef CONFIG_STM32F7_LTDC_L2
  stm32_ltdc_linit(LTDC_LAYER_L2);
#endif

#ifdef CONFIG_STM32F7_DMA2D
  stm32_ltdc_dma2dlinit();
#endif
  /* Enable the backlight */

#ifdef CONFIG_STM32F7_LCD_BACKLIGHT
  stm32_backlight(true);
#endif

  /* Reload shadow register */

  lcdinfo("Reload shadow register\n");
  stm32_ltdc_reload(LTDC_SRCR_IMR, false);

  /* Turn the LCD on */

  lcdinfo("Enabling the display\n");
  stm32_ltdc_enable(true);

  /* Set initialized state */

  g_initialized = true;
  return ret;
}

/****************************************************************************
 * Name: stm32_ltdcgetvplane
 *
 * Description:
 *   Return a a reference to the framebuffer object for the specified video
 *   plane.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Reference to the framebuffer object (NULL on failure)
 *
 ****************************************************************************/

struct fb_vtable_s *stm32_ltdcgetvplane(int vplane)
{
  lcdinfo("vplane: %d\n", vplane);

  if (vplane == 0)
    {
      return &g_vtable.vtable;
    }

  return NULL;
}

/****************************************************************************
 * Name: stm32_ltdcuninitialize
 *
 * Description:
 *   Uninitialize the framebuffer driver.  Bad things will happen if you
 *   call this without first calling fb_initialize()!
 *
 ****************************************************************************/

void stm32_ltdcuninitialize(void)
{
  /* Disable all ltdc interrupts */

  stm32_ltdc_irqctrl(0, LTDC_IER_RRIE | LTDC_IER_TERRIE |
                        LTDC_IER_FUIE | LTDC_IER_LIE);

  up_disable_irq(g_interrupt.irq);
  irq_detach(g_interrupt.irq);

  /* Disable the LCD controller */

  stm32_ltdc_enable(false);

  /* Set initialized state */

  g_initialized = false;
}

/****************************************************************************
 * Name: stm32_lcd_backlight
 *
 * Description:
 *   Provide this interface to turn the backlight on and off.
 *
 * Input Parameters:
 *   blon - Enable or disable the lcd backlight
 *
 ****************************************************************************/

#ifdef CONFIG_STM32F7_LCD_BACKLIGHT
void stm32_backlight(bool blon)
{
  /* Set default backlight level CONFIG_STM32F7_LTDC_DEFBACKLIGHT */

  lcderr("ERROR: Not supported\n");
}
#endif
