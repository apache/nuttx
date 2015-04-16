/****************************************************************************
 * arch/arm/src/stm32/stm32_ltdc.c
 *
 *   Copyright (C) 2013-2015 Ken Pettit. All rights reserved.
 *   Authors: Ken Pettit <pettitd@gmail.com>
 *            Marco Krahl <ocram.lhark@gmail.com>
 *
 * References:
 *   STM32F429 Technical Reference Manual and Data Sheet
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

#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/video/fb.h>
#include <nuttx/kmalloc.h>

#include <arch/chip/ltdc.h>
#include <arch/chip/dma2d.h>
#include <arch/board/board.h>

#include "up_arch.h"
#include "up_internal.h"
#include "stm32.h"
#include "chip/stm32_ltdc.h"
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

#define STM32_LTDC_LxWHPCR_WHSTPOS  (BOARD_LTDC_HSYNC + BOARD_LTDC_HBP - 1)
#define STM32_LTDC_LxWHPCR_WHSPPOS  (BOARD_LTDC_HSYNC + BOARD_LTDC_HBP + \
                                     STM32_LTDC_WIDTH - 1)

/* LTDC_LxWVPCR register */

#define STM32_LTDC_LxWVPCR_WVSTPOS  (BOARD_LTDC_VSYNC + BOARD_LTDC_VBP - 1)
#define STM32_LTDC_LxWVPCR_WVSPPOS  (BOARD_LTDC_VSYNC + BOARD_LTDC_VBP + \
                                     STM32_LTDC_HEIGHT - 1)

/* LTDC_SSCR register */

#define STM32_LTDC_SSCR_VSH          LTDC_SSCR_VSH(BOARD_LTDC_VSYNC - 1)
#define STM32_LTDC_SSCR_HSW          LTDC_SSCR_HSW(BOARD_LTDC_HSYNC - 1)

/* LTDC_BPCR register */

#define STM32_LTDC_BPCR_AVBP         LTDC_BPCR_AVBP(STM32_LTDC_LxWVPCR_WVSTPOS)
#define STM32_LTDC_BPCR_AHBP         LTDC_BPCR_AHBP(STM32_LTDC_LxWHPCR_WHSTPOS)

/* LTDC_AWCR register */

#define STM32_LTDC_AWCR_AAH          LTDC_AWCR_AAH(STM32_LTDC_LxWVPCR_WVSPPOS)
#define STM32_LTDC_AWCR_AAW          LTDC_AWCR_AAW(STM32_LTDC_LxWHPCR_WHSPPOS)

/* LTDC_TWCR register */

#define STM32_LTDC_TWCR_TOTALH       LTDC_TWCR_TOTALH(BOARD_LTDC_VSYNC + \
                                     BOARD_LTDC_VBP + \
                                     STM32_LTDC_HEIGHT + BOARD_LTDC_VFP - 1)
#define STM32_LTDC_TWCR_TOTALW       LTDC_TWCR_TOTALW(BOARD_LTDC_HSYNC + \
                                     BOARD_LTDC_HBP + \
                                     STM32_LTDC_WIDTH + BOARD_LTDC_HFP - 1)

/* Global GCR register */

/* Synchronisation and Polarity */

#define STM32_LTDC_GCR_PCPOL         BOARD_LTDC_GCR_PCPOL
#define STM32_LTDC_GCR_DEPOL         BOARD_LTDC_GCR_DEPOL
#define STM32_LTDC_GCR_VSPOL         BOARD_LTDC_GCR_VSPOL
#define STM32_LTDC_GCR_HSPOL         BOARD_LTDC_GCR_HSPOL

/* Dither */

#define STM32_LTDC_GCR_DEN           BOARD_LTDC_GCR_DEN
#define STM32_LTDC_GCR_DBW           LTDC_GCR_GBW(BOARD_LTDC_GCR_DBW)
#define STM32_LTDC_GCR_DGW           LTDC_GCR_DGW(BOARD_LTDC_GCR_DGW)
#define STN32_LTDC_GCR_DRW           LTDC_GCR_DBW(BOARD_LTDC_GCR_DRW)

/* LIPCR register */

#define STM32_LTDC_LIPCR_LIPOS       LTDC_LIPCR_LIPOS(STM32_LTDC_TWCR_TOTALW)

/* Configuration ************************************************************/

#ifndef CONFIG_STM32_LTDC_DEFBACKLIGHT
#  define CONFIG_STM32_LTDC_DEFBACKLIGHT 0xf0
#endif
#define STM32_LTDC_BACKLIGHT_OFF 0x00

/* Color/video formats */

/* Layer 1 format */

#if defined(CONFIG_STM32_LTDC_L1_L8)
#  define STM32_LTDC_L1_BPP         8
#  define STM32_LTDC_L1_COLOR_FMT   FB_FMT_RGB8
#  define STM32_LTDC_L1PFCR_PF      LTDC_LxPFCR_PF(LTDC_PF_L8)
#  define STM32_LTDC_L1CMAP
#elif defined(CONFIG_STM32_LTDC_L1_AL44)
#  define STM32_LTDC_L1_BPP         8
#  define STM32_LTDC_L1_COLOR_FMT   ???
#  define STM32_LTDC_L1PFCR_PF      LTDC_LxPFCR_PF(LTDC_PF_AL44)
#elif defined(CONFIG_STM32_LTDC_L1_AL88)
#  define STM32_LTDC_L1_BPP         16
#  define STM32_LTDC_L1_COLOR_FMT   ???
#  define STM32_LTDC_L1PFCR_PF      LTDC_LxPFCR_PF(LTDC_PF_AL88)
#elif defined(CONFIG_STM32_LTDC_L1_ARGB4444)
#  define STM32_LTDC_L1_BPP         16
#  define STM32_LTDC_L1_COLOR_FMT   ???
#  define STM32_LTDC_L1PFCR_PF      LTDC_LxPFCR_PF(LTDC_PF_ARGB4444)
#elif defined(CONFIG_STM32_LTDC_L1_RGB565)
#  define STM32_LTDC_L1_BPP         16
#  define STM32_LTDC_L1_COLOR_FMT   FB_FMT_RGB16_565
#  define STM32_LTDC_L1PFCR_PF      LTDC_LxPFCR_PF(LTDC_PF_RGB565)
#elif defined(CONFIG_STM32_LTDC_L1_ARGB1555)
#  define STM32_LTDC_L1_BPP         16
#  define STM32_LTDC_L1_COLOR_FMT   ???
#  define STM32_LTDC_L1PFCR_PF      LTDC_LxPFCR_PF(LTDC_PF_ARGB1555)
#elif defined(CONFIG_STM32_LTDC_L1_RGB888)
#  define STM32_LTDC_L1_BPP         24
#  define STM32_LTDC_L1_COLOR_FMT   FB_FMT_RGB24
#  define STM32_LTDC_L1PFCR_PF      LTDC_LxPFCR_PF(LTDC_PF_RGB888)
#elif defined(CONFIG_STM32_LTDC_L1_ARGB8888)
#  define STM32_LTDC_L1_BPP         32
#  define STM32_LTDC_L1_COLOR_FMT   ???
#  define STM32_LTDC_L1PFCR_PF      LTDC_LxPFCR_PF(LTDC_PF_ARGB8888)
#endif

/* Layer 2 format */

#if defined(CONFIG_STM32_LTDC_L2_L8)
#  define STM32_LTDC_L2_BPP         8
#  define STM32_LTDC_L2_COLOR_FMT   FB_FMT_RGB8
#  define STM32_LTDC_L2PFCR_PF      LTDC_LxPFCR_PF(LTDC_PF_L8)
#  define STM32_LTDC_L2CMAP
#elif defined(CONFIG_STM32_LTDC_L2_AL44)
#  define STM32_LTDC_L2_BPP         8
#  define STM32_LTDC_L2_COLOR_FMT   ???
#  define STM32_LTDC_L2PFCR_PF      LTDC_LxPFCR_PF(LTDC_PF_AL44)
#elif defined(CONFIG_STM32_LTDC_L2_AL88)
#  define STM32_LTDC_L2_BPP         16
#  define STM32_LTDC_L2_COLOR_FMT   ???
#  define STM32_LTDC_L2PFCR_PF      LTDC_LxPFCR_PF(LTDC_PF_AL88)
#elif defined(CONFIG_STM32_LTDC_L2_ARGB4444)
#  define STM32_LTDC_L2_BPP         16
#  define STM32_LTDC_L2_COLOR_FMT   ???
#  define STM32_LTDC_L2PFCR_PF      LTDC_LxPFCR_PF(LTDC_PF_ARGB4444)
#elif defined(CONFIG_STM32_LTDC_L2_RGB565)
#  define STM32_LTDC_L2_BPP         16
#  define STM32_LTDC_L2_COLOR_FMT   FB_FMT_RGB16_565
#  define STM32_LTDC_L2PFCR_PF      LTDC_LxPFCR_PF(LTDC_PF_RGB565)
#elif defined(CONFIG_STM32_LTDC_L2_ARGB1555)
#  define STM32_LTDC_L2_BPP         16
#  define STM32_LTDC_L2_COLOR_FMT   ???
#  define STM32_LTDC_L2PFCR_PF      LTDC_LxPFCR_PF(LTDC_PF_ARGB1555)
#elif defined(CONFIG_STM32_LTDC_L2_RGB888)
#  define STM32_LTDC_L2_BPP         24
#  define STM32_LTDC_L2_COLOR_FMT   FB_FMT_RGB24
#  define STM32_LTDC_L2PFCR_PF      LTDC_LxPFCR_PF(LTDC_PF_RGB888)
#elif defined(CONFIG_STM32_LTDC_L2_ARGB8888)
#  define STM32_LTDC_L2_BPP         32
#  define STM32_LTDC_L2_COLOR_FMT   ???
#  define STM32_LTDC_L2PFCR_PF      LTDC_LxPFCR_PF(LTDC_PF_ARGB8888)
#endif

/* Framebuffer sizes in bytes */

#if STM32_LTDC_L1_BPP == 8
#  define STM32_L1_STRIDE (STM32_LTDC_WIDTH)
#elif STM32_LTDC_L1_BPP == 16
#  define STM32_L1_STRIDE ((STM32_LTDC_WIDTH * 16 + 7) / 8)
#elif STM32_LTDC_L1_BPP == 24
#  define STM32_L1_STRIDE ((STM32_LTDC_WIDTH * 24 + 7) / 8)
#elif STM32_LTDC_L1_BPP == 32
#  define STM32_L1_STRIDE ((STM32_LTDC_WIDTH * 32 + 7) / 8)
#else
#  error Undefined or unrecognized base resolution
#endif

/* LTDC only supports 8 bit per pixel overal */

#define STM32_LTDC_Lx_BYPP(n)        ((n) / 8)

#define STM32_L1_FBSIZE (STM32_L1_STRIDE * STM32_LTDC_HEIGHT)

#ifdef CONFIG_STM32_LTDC_L2
#  ifndef CONFIG_STM32_LTDC_L2_WIDTH
#    define CONFIG_STM32_LTDC_L2_WIDTH STM32_LTDC_WIDTH
#  endif

#  if CONFIG_STM32_LTDC_L2_WIDTH > STM32_LTDC_WIDTH
#    error Width of Layer 2 exceeds the width of the display
#  endif

#  ifndef CONFIG_STM32_LTDC_L2_HEIGHT
#    define CONFIG_STM32_LTDC_L2_HEIGHT STM32_LTDC_HEIGHT
#  endif

#  if CONFIG_STM32_LTDC_L2_HEIGHT > STM32_LTDC_HEIGHT
#    error Height of Layer 2 exceeds the height of the display
#  endif

#  if STM32_LTDC_L2_BPP == 8
#    define STM32_L2_STRIDE (CONFIG_STM32_LTDC_L2_WIDTH)
#  elif STM32_LTDC_L2_BPP == 16
#    define STM32_L2_STRIDE ((CONFIG_STM32_LTDC_L2_WIDTH * 16 + 7) / 8)
#  elif STM32_LTDC_L2_BPP == 24
#    define STM32_L2_STRIDE ((CONFIG_STM32_LTDC_L2_WIDTH * 24 + 7) / 8)
#  elif STM32_LTDC_L2_BPP == 32
#    define STM32_L2_STRIDE ((CONFIG_STM32_LTDC_L2_WIDTH * 32 + 7) / 8)
#  else
#    error Undefined or unrecognized base resolution
#  endif

#  define STM32_L2_FBSIZE (STM32_L2_STRIDE * CONFIG_STM32_LTDC_L2_HEIGHT)

#else
#  define STM32_L2_FBSIZE (0)
#endif

/* Total memory used for framebuffers */

#define STM32_TOTAL_FBSIZE (STM32_L1_FBSIZE + STM32_L2_FBSIZE)

/* Debug option */

#ifdef CONFIG_STM32_LTDC_REGDEBUG
#  define regdbg       dbg
#  define regvdbg      vdbg
#else
#  define regdbg(x...)
#  define regvdbg(x...)
#endif

/* Preallocated LTDC framebuffers */

/* Position the framebuffer memory in the center of the memory set aside.  We
 * will use any skirts before or after the framebuffer memory as a guard against
 * wild framebuffer writes.
 */

#define STM32_LTDC_BUFFER_SIZE  CONFIG_STM32_LTDC_FB_SIZE
#define STM32_LTDC_BUFFER_FREE  (STM32_LTDC_BUFFER_SIZE - STM32_TOTAL_FBSIZE)
#define STM32_LTDC_BUFFER_START (CONFIG_STM32_LTDC_FB_BASE + \
                                STM32_LTDC_BUFFER_FREE/2)

#if STM32_LTDC_BUFFER_FREE < 0
#  error "STM32_LTDC_BUFFER_SIZE not large enough for frame buffers"
#endif

/* Layer frame buffer */

#define STM32_LTDC_BUFFER_L1     STM32_LTDC_BUFFER_START
#define STM32_LTDC_ENDBUF_L1     (STM32_LTDC_BUFFER_L1 + STM32_L1_FBSIZE)

#ifdef CONFIG_STM32_LTDC_L2
#  define STM32_LTDC_BUFFER_L2   STM32_LTDC_ENDBUF_L1
#  define STM32_LTDC_ENDBUF_L2   (STM32_LTDC_BUFFER_L2 + STM32_L2_FBSIZE)
#else
#  define STM32_LTDC_ENDBUF_L2   STM32_LTDC_ENDBUF_L1
#endif

/* Layer helpers */

#ifdef CONFIG_STM32_LTDC_L2
#  define LTDC_NLAYERS 2
#else
#  define LTDC_NLAYERS 1
#endif

#define LAYER(i)     g_ltdc.layer[i]
#define LAYER_L1     g_ltdc.layer[LTDC_LAYER_L1]
#define LAYER_L2     g_ltdc.layer[LTDC_LAYER_L2]

/* Dithering */

#ifndef CONFIG_STM32_LTDC_DITHER_RED
# define STM32_LTDC_DITHER_RED      0
#else
# define STM32_LTDC_DITHER_RED      CONFIG_STM32_LTDC_DITHER_RED
#endif
#ifndef CONFIG_STM32_LTDC_DITHER_GREEN
# define STM32_LTDC_DITHER_GREEN    0
#else
# define STM32_LTDC_DITHER_GREEN    CONFIG_STM32_LTDC_DITHER_GREEN
#endif
#ifndef CONFIG_STM32_LTDC_DITHER_BLUE
# define STM32_LTDC_DITHER_BLUE     0
#else
# define STM32_LTDC_DITHER_BLUE     CONFIG_STM32_LTDC_DITHER_BLUE
#endif

/* Background color */

#ifndef CONFIG_STM32_LTDC_BACKCOLOR
# define STM32_LTDC_BACKCOLOR       0
#else
# define STM32_LTDC_BACKCOLOR       CONFIG_STM32_LTDC_BACKCOLOR
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

#ifdef CONFIG_STM32_DMA2D
# if defined(CONFIG_STM32_LTDC_L1_L8) || \
        defined(CONFIG_STM32_LTDC_L2_L8)
#  if !defined(CONFIG_STM32_DMA2D_L8)
#   error "DMA2D must support FB_FMT_RGB8 pixel format"
#  endif
# endif
# if defined(CONFIG_STM32_LTDC_L1_RGB565) || \
        defined(CONFIG_STM32_LTDC_L2_RGB565)
#  if !defined(CONFIG_STM32_DMA2D_RGB565)
#   error "DMA2D must support FB_FMT_RGB16_565 pixel format"
#  endif
# endif
# if defined(CONFIG_STM32_LTDC_L1_RGB888) || \
        defined(CONFIG_STM32_LTDC_L2_RGB888)
#  if !defined(CONFIG_STM32_DMA2D_RGB888)
#   error "DMA2D must support FB_FMT_RGB24 pixel format"
#  endif
# endif
#endif

/* Calculate the size of the layers clut table */

#ifdef CONFIG_FB_CMAP
# if defined(CONFIG_STM32_DMA2D) && !defined(CONFIG_STM32_DMA2D_L8)
#  error "DMA2D must also support L8 CLUT pixel format if supported by LTDC"
# endif
# ifdef STM32_LTDC_L1CMAP
#  ifdef CONFIG_FB_TRANSPARENCY
#   define STM32_LAYER_CLUT_SIZE     STM32_LTDC_NCLUT * sizeof(uint32_t)
#  else
#   define STM32_LAYER_CLUT_SIZE     STM32_LTDC_NCLUT * 3 * sizeof(uint8_t)
#  endif
# endif
# ifdef STM32_LTDC_L2CMAP
#  undef  STM32_LAYER_CLUT_SIZE
#  ifdef CONFIG_FB_TRANSPARENCY
#   define STM32_LAYER_CLUT_SIZE     STM32_LTDC_NCLUT * sizeof(uint32_t) * 2
#  else
#   define STM32_LAYER_CLUT_SIZE     STM32_LTDC_NCLUT * 3 * sizeof(uint8_t) * 2
#  endif
# endif
#endif

#ifndef CONFIG_FB_CMAP
# if defined(STM32_LTDC_L1CMAP) || defined(STM32_LTDC_L2CMAP)
#  undef STM32_LTDC_L1CMAP
#  undef STM32_LTDC_L2CMAP
#  error "Enable cmap to support the configured layer format!"
# endif
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

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This enumeration names each layer supported by the hardware */

enum stm32_layer_e
{
  LTDC_LAYER_L1 = 0,       /* LCD Layer 1*/
  LTDC_LAYER_L2,           /* LCD Layer 2 */
};

/* LTDC General layer information */

struct stm32_layer_s
{
#ifdef CONFIG_STM32_LTDC_INTERFACE
  /* LTDC interface */

  struct ltdc_layer_s ltdc;     /* Layer control structure */
#endif

  struct stm32_ltdc_s state; /* Layer state structure */

  /* Blending */

  uint8_t  opac;            /* Opacity value for blending */
  uint8_t  bf1;             /* Blend factor 1 */
  uint8_t  bf2;             /* Blend factor 2 */

  /* Operation */

  uint8_t operation;        /* Operation flags */
#ifdef CONFIG_STM32_DMA2D
  FAR struct dma2d_layer_s *dma2d; /* dma2d interface */
#endif
};

/* This structure provides the state of each LTDC layer */

struct stm32_state_s
{
  /* Layer state */

  struct stm32_ltdc_s state[LTDC_NLAYERS];
};

/* This structure provides the overall state of the LTDC layer */

struct stm32_ltdcdev_s
{
  /* Layer information */

  struct stm32_layer_s layer[LTDC_NLAYERS];
};

/* Layer cmap table description */

#ifdef STM32_LAYER_CLUT_SIZE
enum stm32_clut_e
{
  LTDC_L1CLUT_OFFSET = 0,
  LTDC_L2CLUT_OFFSET = STM32_LTDC_NCLUT * sizeof(uint32_t)
};
#endif

/* Interrupt handling */

struct stm32_interrupt_s
{
  bool  wait;       /* Informs that the task is waiting for the irq */
  bool  handled;    /* Informs that an irq was handled */
  int   irq;        /* irq number */
  sem_t *sem;       /* Semaphore for waiting for irq */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Global register operation */

static void stm32_lcd_enable(bool enable);
static void stm32_ltdc_gpioconfig(void);
static void stm32_ltdc_periphconfig(void);
static void stm32_ltdc_bgcolor(uint32_t rgb);
static void stm32_ltdc_dither(bool enable, uint8_t red,
                              uint8_t green, uint8_t blue);
static int stm32_ltdcirq(int irq, void *context);
static int stm32_ltdc_waitforirq(void);
static int stm32_ltdc_reload(uint8_t value, bool waitvblank);

/* Layer and layer register operation */

static inline void stm32_ltdc_lsetopac(FAR struct stm32_layer_s *layer);
static inline void stm32_ltdc_lunsetopac(FAR struct stm32_layer_s *layer);
static inline uint8_t stm32_ltdc_lgetopac(FAR struct stm32_layer_s *layer);
static inline bool stm32_ltdc_lvalidate(FAR const struct stm32_layer_s *layer);
#ifdef CONFIG_STM32_LTDC_INTERFACE
static int stm32_ltdc_lvalidatearea(FAR struct stm32_layer_s *layer,
                                    fb_coord_t xpos, fb_coord_t ypos,
                                    fb_coord_t xres, fb_coord_t yres,
                                    fb_coord_t srcxpos, fb_coord_t srcypos);
#endif
static void stm32_ltdc_lupdate(FAR struct stm32_layer_s *layer);

static void stm32_ltdc_lpixelformat(FAR struct stm32_layer_s *layer);
static inline void stm32_ltdc_lframebuffer(FAR struct stm32_layer_s *layer);
static void stm32_ltdc_larea(FAR struct stm32_layer_s *layer);
static void stm32_ltdc_lcolor(FAR struct stm32_layer_s *layer, uint32_t argb);
static void stm32_ltdc_lcolorkey(FAR struct stm32_layer_s *layer);
static void stm32_ltdc_lalpha(FAR struct stm32_layer_s *layer);
static void stm32_ltdc_lblendmode(FAR struct stm32_layer_s *layer,
                                  uint8_t bf1, uint8_t bf2);

#ifdef STM32_LAYER_CLUT_SIZE
static void stm32_ltdc_lclut(FAR struct stm32_layer_s *layer,
                             FAR const struct fb_cmap_s *cmap);
static void stm32_ltdc_lclutenable(FAR struct stm32_layer_s* layer,
                                    bool enable);
#endif
static void stm32_ltdc_linit(int lid);
static void stm32_ltdc_lenable(FAR struct stm32_layer_s *layer);
static void stm32_ltdc_lclear(FAR struct stm32_layer_s *layer,
                              nxgl_mxpixel_t color);

/* Generic frame buffer interface */

static int stm32_getvideoinfo(FAR struct fb_vtable_s *vtable,
                              struct fb_videoinfo_s *vinfo);
static int stm32_getplaneinfo(FAR struct fb_vtable_s *vtable,
                              int planeno, struct fb_planeinfo_s *pinfo);

/* The following is provided only if the video hardware supports RGB color
 * mapping
 */

#ifdef STM32_LAYER_CLUT_SIZE
static int stm32_getcmap(struct fb_vtable_s *vtable,
                         struct fb_cmap_s *cmap);
static int stm32_putcmap(struct fb_vtable_s *vtable,
                         const struct fb_cmap_s *cmap);
#endif

/* ltdc interface */

#ifdef STM32_LAYER_CLUT_SIZE
static int stm32_setclut(struct ltdc_layer_s *layer,
              const struct fb_cmap_s *cmap);
static int stm32_getclut(struct ltdc_layer_s *layer,
              struct fb_cmap_s *cmap);
#endif
static int stm32_lgetvideoinfo(struct ltdc_layer_s *layer,
                                 struct fb_videoinfo_s *vinfo);
static int stm32_lgetplaneinfo(struct ltdc_layer_s *layer, int planeno,
                                 struct fb_planeinfo_s *pinfo);

#ifdef CONFIG_STM32_LTDC_INTERFACE
static int stm32_getlid(FAR struct ltdc_layer_s *layer,
                        int *lid, uint32_t flag);
static int stm32_setcolor(FAR struct ltdc_layer_s *layer, uint32_t argb);
static int stm32_getcolor(FAR struct ltdc_layer_s *layer, uint32_t *argb);
static int stm32_setcolorkey(FAR struct ltdc_layer_s *layer, uint32_t argb);
static int stm32_getcolorkey(FAR struct ltdc_layer_s *layer, uint32_t *argb);
static int stm32_setalpha(FAR struct ltdc_layer_s *layer, uint8_t alpha);
static int stm32_getalpha(FAR struct ltdc_layer_s *layer, uint8_t *alpha);
static int stm32_setblendmode(FAR struct ltdc_layer_s *layer, uint32_t mode);
static int stm32_getblendmode(FAR struct ltdc_layer_s *layer, uint32_t *mode);
static int stm32_setarea(FAR struct ltdc_layer_s *layer,
                         FAR const struct ltdc_area_s *area,
                         fb_coord_t srcxpos, fb_coord_t srcypos);
static int stm32_getarea(FAR struct ltdc_layer_s *layer,
                         FAR struct ltdc_area_s *area,
                         fb_coord_t *srcxpos, fb_coord_t *srcypos);
static int stm32_update(FAR struct ltdc_layer_s *layer, uint32_t mode);

#ifdef CONFIG_STM32_DMA2D
static int stm32_blit(FAR struct ltdc_layer_s *dest,
                      fb_coord_t destxpos, fb_coord_t destypos,
                      FAR const struct dma2d_layer_s *src,
                      FAR const struct ltdc_area_s *srcarea);
static int stm32_blend(FAR struct ltdc_layer_s *dest,
                        fb_coord_t destxpos, fb_coord_t destypos,
                        FAR const struct dma2d_layer_s *fore,
                        fb_coord_t forexpos, fb_coord_t foreypos,
                        FAR const struct dma2d_layer_s *back,
                        FAR const struct ltdc_area_s *backarea);
static int stm32_fillarea(FAR struct ltdc_layer_s *layer,
                            FAR const struct ltdc_area_s *area,
                            uint32_t color);
#endif
#endif /* CONFIG_STM32_LTDC_INTERFACE */

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
# if BOARD_LTDC_OUTPUT_BPP > 16
  GPIO_LTDC_R2, GPIO_LTDC_B2,
#  if BOARD_LTDC_OUTPUT_BPP > 18
  GPIO_LTDC_R0, GPIO_LTDC_R1, GPIO_LTDC_G0, GPIO_LTDC_G1,
  GPIO_LTDC_B0, GPIO_LTDC_B1,
#  endif
# endif
#endif
  GPIO_LTDC_VSYNC, GPIO_LTDC_HSYNC, GPIO_LTDC_DE, GPIO_LTDC_CLK
};

#define STM32_LTDC_NPINCONFIGS (sizeof(g_ltdcpins) / sizeof(uint32_t))

/* This structure provides the base layer interface */

static const struct fb_vtable_s g_vtable =
{
  .getvideoinfo  = stm32_getvideoinfo,
  .getplaneinfo  = stm32_getplaneinfo
#ifdef STM32_LAYER_CLUT_SIZE
 ,.getcmap       = stm32_getcmap,
  .putcmap       = stm32_putcmap
#endif
};

/* The LTDC semaphore that enforces mutually exclusive access */

static sem_t g_lock;

/* The semaphore for interrupt handling */

static sem_t g_semirq;

/* This structure provides irq handling */

static struct stm32_interrupt_s g_interrupt =
{
  .wait    = false,
  .handled = true,
  .irq     = STM32_IRQ_LTDCINT,
  .sem     = &g_semirq
};

/* The layer active state */

static uint8_t g_lactive;

#ifdef STM32_LAYER_CLUT_SIZE
/* The layers clut table entries */

static uint32_t g_clut[STM32_LAYER_CLUT_SIZE];
#endif

/* The initialized state of the overall LTDC layers */

static struct stm32_ltdcdev_s g_ltdc =
{
  .layer[LTDC_LAYER_L1] =
    {
      .state =
        {
          .lid   = LTDC_LAYER_L1,
          .pinfo =
            {
              .fbmem    = (uint8_t *)STM32_LTDC_BUFFER_L1,
              .fblen    = STM32_L1_FBSIZE,
              .stride   = STM32_L1_STRIDE,
              .bpp      = STM32_LTDC_L1_BPP
            },
          .vinfo =
            {
              .fmt      = STM32_LTDC_L1_COLOR_FMT,
              .xres     = STM32_LTDC_WIDTH,
              .yres     = STM32_LTDC_HEIGHT,
              .nplanes  = 1
            }
#ifdef STM32_LTDC_L1CMAP
         ,.clut         = &g_clut[LTDC_L1CLUT_OFFSET]
#endif
        }
    }
#ifdef CONFIG_STM32_LTDC_L2
 ,.layer[LTDC_LAYER_L2] =
    {
      .state =
        {
          .lid   = LTDC_LAYER_L2,
          .pinfo =
            {
              .fbmem    = (uint8_t *)STM32_LTDC_BUFFER_L2,
              .fblen    = STM32_L2_FBSIZE,
              .stride   = STM32_L2_STRIDE,
              .bpp      = STM32_LTDC_L2_BPP
            },
          .vinfo =
            {
              .fmt      = STM32_LTDC_L2_COLOR_FMT,
              .xres     = STM32_LTDC_WIDTH,
              .yres     = STM32_LTDC_HEIGHT,
              .nplanes  = 1
            }
#ifdef STM32_LTDC_L2CMAP
         ,.clut         = &g_clut[LTDC_L2CLUT_OFFSET]
#endif
        }
    }
#endif
};

/* Pixel format lookup table */

static const uint32_t stm32_fmt_layer_t[LTDC_NLAYERS] =
{
  STM32_LTDC_L1PFCR_PF
#ifdef CONFIG_STM32_LTDC_L2
 ,STM32_LTDC_L2PFCR_PF
#endif
};

/* Register lookup tables */

/* LTDC_LxCR */

static const uintptr_t stm32_cr_layer_t[LTDC_NLAYERS] =
{
  STM32_LTDC_L1CR
#ifdef CONFIG_STM32_LTDC_L2
  , STM32_LTDC_L2CR
#endif
};

/* LTDC_LxWHPCR */

static const uintptr_t stm32_whpcr_layer_t[LTDC_NLAYERS] =
{
  STM32_LTDC_L1WHPCR
#ifdef CONFIG_STM32_LTDC_L2
  , STM32_LTDC_L2WHPCR
#endif
};

/* LTDC_LxWVPCR */

static const uintptr_t stm32_wvpcr_layer_t[LTDC_NLAYERS] =
{
  STM32_LTDC_L1WVPCR
#ifdef CONFIG_STM32_LTDC_L2
  , STM32_LTDC_L2WVPCR
#endif
};

/* LTDC_LxPFCR */

static const uintptr_t stm32_pfcr_layer_t[LTDC_NLAYERS] =
{
  STM32_LTDC_L1PFCR
#ifdef CONFIG_STM32_LTDC_L2
  , STM32_LTDC_L2PFCR
#endif
};

/* LTDC_LxDCCR */

static const uintptr_t stm32_dccr_layer_t[LTDC_NLAYERS] =
{
  STM32_LTDC_L1DCCR
#ifdef CONFIG_STM32_LTDC_L2
  , STM32_LTDC_L2DCCR
#endif
};

/* LTDC_LxCKCR */

static const uintptr_t stm32_ckcr_layer_t[LTDC_NLAYERS] =
{
  STM32_LTDC_L1CKCR
#ifdef CONFIG_STM32_LTDC_L2
  , STM32_LTDC_L2CKCR
#endif
};

/* LTDC_LxCACR */

static const uintptr_t stm32_cacr_layer_t[LTDC_NLAYERS] =
{
  STM32_LTDC_L1CACR
#ifdef CONFIG_STM32_LTDC_L2
  , STM32_LTDC_L2CACR
#endif
};

/* LTDC_LxBFCR */

static const uintptr_t stm32_bfcr_layer_t[LTDC_NLAYERS] =
{
  STM32_LTDC_L1BFCR
#ifdef CONFIG_STM32_LTDC_L2
  , STM32_LTDC_L2BFCR
#endif
};

/* LTDC_LxCFBAR */

static const uintptr_t stm32_cfbar_layer_t[LTDC_NLAYERS] =
{
  STM32_LTDC_L1CFBAR
#ifdef CONFIG_STM32_LTDC_L2
  , STM32_LTDC_L2CFBAR
#endif
};

/* LTDC_LxCFBLR */

static const uintptr_t stm32_cfblr_layer_t[LTDC_NLAYERS] =
{
  STM32_LTDC_L1CFBLR
#ifdef CONFIG_STM32_LTDC_L2
  , STM32_LTDC_L2CFBLR
#endif
};

/* LTDC_LxCFBLNR */

static const uintptr_t stm32_cfblnr_layer_t[LTDC_NLAYERS] =
{
  STM32_LTDC_L1CFBLNR
#ifdef CONFIG_STM32_LTDC_L2
  , STM32_LTDC_L2CFBLNR
#endif
};

/* LTDC_LxCLUTWR */

#ifdef STM32_LAYER_CLUT_SIZE
static const uintptr_t stm32_clutwr_layer_t[LTDC_NLAYERS] =
{
  STM32_LTDC_L1CLUTWR
# ifdef CONFIG_STM32_LTDC_L2
  , STM32_LTDC_L2CLUTWR
# endif
};
#endif

/* The initialized state of the driver */

static bool g_initialized;

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Configure global register
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

  gvdbg("Configuring pins\n");

  /* Configure each pin */

  for (i = 0; i < STM32_LTDC_NPINCONFIGS; i++)
    {
      regvdbg("set gpio%d = %08x\n", i, g_ltdcpins[i]);
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

  regvdbg("configured RCC_APB2ENR=%08x\n", getreg32(STM32_RCC_APB2ENR));

  /* Configure the SAI PLL external to provide the LCD_CLK */

  regvdbg("configured RCC_PLLSAI=%08x\n", getreg32(STM32_RCC_PLLSAICFGR));

  /* Configure dedicated clock external */

  regvdbg("configured RCC_DCKCFGR=%08x\n", getreg32(STM32_RCC_DCKCFGR));

  /* Configure LTDC_SSCR */

  regval = (STM32_LTDC_SSCR_VSH | STM32_LTDC_SSCR_HSW);
  regvdbg("set LTDC_SSCR=%08x\n", regval);
  putreg32(regval, STM32_LTDC_SSCR);
  regvdbg("configured LTDC_SSCR=%08x\n", getreg32(STM32_LTDC_SSCR));

  /* Configure LTDC_BPCR */

  regval = (STM32_LTDC_BPCR_AVBP | STM32_LTDC_BPCR_AHBP);
  regvdbg("set LTDC_BPCR=%08x\n", regval);
  putreg32(regval, STM32_LTDC_BPCR);
  regvdbg("configured LTDC_BPCR=%08x\n", getreg32(STM32_LTDC_BPCR));

  /* Configure LTDC_AWCR */

  regval = (STM32_LTDC_AWCR_AAH | STM32_LTDC_AWCR_AAW);
  regvdbg("set LTDC_AWCR=%08x\n", regval);
  putreg32(regval, STM32_LTDC_AWCR);
  regvdbg("configured LTDC_AWCR=%08x\n", getreg32(STM32_LTDC_AWCR));

  /* Configure LTDC_TWCR */

  regval = (STM32_LTDC_TWCR_TOTALH | STM32_LTDC_TWCR_TOTALW);
  regvdbg("set LTDC_TWCR=%08x\n", regval);
  putreg32(regval, STM32_LTDC_TWCR);
  regvdbg("configured LTDC_TWCR=%08x\n", getreg32(STM32_LTDC_TWCR));

  /* Configure LTDC_GCR */

  regval = (STM32_LTDC_GCR_PCPOL | STM32_LTDC_GCR_DEPOL
           | STM32_LTDC_GCR_VSPOL | STM32_LTDC_GCR_HSPOL);
  regvdbg("set LTDC_GCR=%08x\n", regval);
  putreg32(regval, STM32_LTDC_GCR);
  regvdbg("configured LTDC_GCR=%08x\n", getreg32(STM32_LTDC_GCR));
}

/****************************************************************************
 * Name: stm32_ltdc_bgcolor
 *
 * Description:
 *   Configures background color of the LCD controller.
 *
 * Parameter:
 *   rgb - RGB888 background color
 *
 ****************************************************************************/

static void stm32_ltdc_bgcolor(uint32_t rgb)
{
  regvdbg("set LTDC_BCCR=%08x\n", rgb);
  putreg32(rgb, STM32_LTDC_BCCR);
  regvdbg("configured LTDC_BCCR=%08x\n", getreg32(STM32_LTDC_BCCR));
}

/****************************************************************************
 * Name: stm32_ltdc_dither
 *
 * Description:
 *   Configures dither settings of the LCD controller.
 *
 * Parameter:
 *   enable - Enable dithering
 *   red    - Red dither width
 *   green  - Green dither width
 *   blue   - Blue dither width
 *
 ****************************************************************************/

static void stm32_ltdc_dither(bool enable,
                              uint8_t red,
                              uint8_t green,
                              uint8_t blue)
{
  uint32_t    regval;

  regval = getreg32(STM32_LTDC_GCR);

  if (enable == true)
    {
      regval |= LTDC_GCR_DEN;
    }
  else
    {
      regval &= ~LTDC_GCR_DEN;
    }

  regval &= ~(!LTDC_GCR_DEN | LTDC_GCR_DRW(0) |
                LTDC_GCR_DGW(0) | LTDC_GCR_DBW(0));
  regval |= (LTDC_GCR_DRW(red) | LTDC_GCR_DGW(green) | LTDC_GCR_DBW(blue));

  regvdbg("set LTDC_GCR=%08x\n", regval);
  putreg32(regval, STM32_LTDC_GCR);
  regvdbg("configured LTDC_GCR=%08x\n", getreg32(STM32_LTDC_GCR));
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

  regvdbg("set LTDC_LIPCR=%08x\n", STM32_LTDC_LIPCR_LIPOS);
  putreg32(STM32_LTDC_LIPCR_LIPOS, STM32_LTDC_LIPCR);
  regvdbg("configured LTDC_LIPCR=%08x\n", getreg32(STM32_LTDC_LIPCR));
}

/****************************************************************************
 * Name: stm32_ltdc_irqctrl
 *
 * Description:
 *   Control  interrupts generated by the ltdc controller
 *
 * Parameter:
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
  regvdbg("set LTDC_IER=%08x\n", regval);
  putreg32(regval, STM32_LTDC_IER);
  regvdbg("configured LTDC_IER=%08x\n", getreg32(STM32_LTDC_IER));
}

/****************************************************************************
 * Name: stm32_ltdcirq
 *
 * Description:
 *   LTDC interrupt handler
 *
 ****************************************************************************/

static int stm32_ltdcirq(int irq, void *context)
{
  FAR struct stm32_interrupt_s *priv = &g_interrupt;

  uint32_t regval = getreg32(STM32_LTDC_ISR);

  regvdbg("irq = %d, regval = %08x\n", irq, regval);

  if (regval & LTDC_ISR_RRIF)
    {
      /* Register reload interrupt */

      /* Clear the interrupt status register */

      putreg32(LTDC_ICR_CRRIF, STM32_LTDC_ICR);

      /* Update the handled flag */

      priv->handled = true;

      /* Unlock the semaphore if locked */

      if (priv->wait)
        {
          int ret = sem_post(priv->sem);

          if (ret != OK)
            {
              dbg("sem_post() failed\n");
              return ret;
            }
        }
    }

  return OK;
}

/****************************************************************************
 * Name: stm32_ltdc_waitforirq
 *
 * Description:
 *   Helper waits until the ltdc irq occurs. In the current design That means
 *   that a register reload was been completed.
 *   Note! The caller must use this function within irqsave state.
 *
 * Return:
 *   OK - On success otherwise ERROR
 *
 ****************************************************************************/

static int stm32_ltdc_waitforirq(void)
{
  int ret = OK;
  FAR struct stm32_interrupt_s *priv = &g_interrupt;

  irqstate_t flags;

  flags = irqsave();

  /* Only waits if last enabled interrupt is currently not handled */

  if (!priv->handled)
    {
      /* Inform the irq handler the task is able to wait for the irq */

      priv->wait = true;

      ret = sem_wait(priv->sem);

      /* irq or an error occurs, reset the wait flag */

      priv->wait = false;

      if (ret != OK)
        {
          dbg("sem_wait() failed\n");
        }
    }

  irqrestore(flags);
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
 * Parameter:
 *   value      - Reload flag (e.g. upon vertical blank or immediately)
 *   waitvblank - Wait until register reload is finished
 *
 ****************************************************************************/

static int stm32_ltdc_reload(uint8_t value, bool waitvblank)
{
  int ret = OK;
  FAR struct stm32_interrupt_s *priv = &g_interrupt;

  if (value == LTDC_SRCR_VBR)
    {
      irqstate_t flags;

      /* Prepare shadow register reload for later detection by the task.
       * At this point the last register reload must be completed. This is done
       * in stm32_update before the next operation is triggered and manipulates
       * the shadow register. This handling is only neccessary in the case of
       * the application causes shadow register reload.
       */

      flags = irqsave();

      ASSERT(priv->handled == true);

      /* Reset the handled flag */

      priv->handled = false;
      irqrestore(flags);
    }

  /* Reloads the shadow register.
   * Note! This will not trigger an register reload interrupt if
   * immediately reload is set.
   */

  regvdbg("set LTDC_SRCR=%08x\n", value);
  putreg32(value, STM32_LTDC_SRCR);
  regvdbg("configured LTDC_SRCR=%08x\n", getreg32(STM32_LTDC_SRCR));

  if (waitvblank & (value == LTDC_SRCR_VBR))
    {
      /* Wait upon vertical blanking period */

      ret = stm32_ltdc_waitforirq();
    }

  /* Otherwise check if reload is completed before the next operation */

  return ret;
}

/****************************************************************************
 * Name: stm32_global_configure
 *
 * Description:
 *   Configure background color
 *   Configure interrupts
 *   Configure dithering
 *
 ****************************************************************************/

static void stm32_global_configure(void)
{
  /* Initialize the LTDC semaphore that enforces mutually exclusive access */

  sem_init(&g_lock, 0, 1);

  /* Initialize the semaphore for interrupt handling */

  sem_init(g_interrupt.sem, 0, 0);

  /* Attach LTDC interrupt vector */

  (void)irq_attach(g_interrupt.irq, stm32_ltdcirq);

  /* Enable the IRQ at the NVIC */

  up_enable_irq(g_interrupt.irq);

  /* Enable register reload interrupt only */

  stm32_ltdc_irqctrl(LTDC_IER_RRIE,LTDC_IER_TERRIE|LTDC_IER_FUIE|LTDC_IER_LIE);

  /* Configure line interrupt */

  stm32_ltdc_linepos();

  /* Set the default active layer */

#ifndef CONFIG_STM32_LTDC_L2
  g_lactive = LTDC_LAYER_L1;
#else
  g_lactive = LTDC_LAYER_L2;
#endif

#ifdef STM32_LAYER_CLUT_SIZE
  /* cleanup clut */

  memset(g_clut, 0, sizeof(g_clut));
#endif

  /* Configure dither */

  stm32_ltdc_dither(
#ifdef CONFIG_STM32_LTDC_DITHER
                    true,
#else
                    false,
#endif
                    STM32_LTDC_DITHER_RED,
                    STM32_LTDC_DITHER_GREEN,
                    STM32_LTDC_DITHER_BLUE);

  /* Configure background color of the controller */

  stm32_ltdc_bgcolor(STM32_LTDC_BACKCOLOR);
}

/****************************************************************************
 * Name: stm32_lcd_enable
 *
 * Description:
 *   Disable the LCD peripheral
 *
 * Parameter:
 *   enable - Enable or disable
 *
 ****************************************************************************/

static void stm32_lcd_enable(bool enable)
{
  uint32_t    regval;

  regval = getreg32(STM32_LTDC_GCR);
  regvdbg("get LTDC_GCR=%08x\n", regval);

  if (enable == true)
    {
      regval |= LTDC_GCR_LTDCEN;
    }
  else
    {
      regval &= ~LTDC_GCR_LTDCEN;
    }

  regvdbg("set LTDC_GCR=%08x\n", regval);
  putreg32(regval, STM32_LTDC_GCR);
  regvdbg("configured LTDC_GCR=%08x\n", getreg32(STM32_LTDC_GCR));
}

/****************************************************************************
 * Configure layer register
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_ltdc_lclutenable
 *
 * Description:
 *   Disable or enable the layer clut support
 *
 * Parameter:
 *   layer  - Reference to the layer control structure
 *   enable - Enable or disable
 *
 ****************************************************************************/

#ifdef STM32_LAYER_CLUT_SIZE
static void stm32_ltdc_lclutenable(FAR struct stm32_layer_s* layer, bool enable)
{
  uint32_t    regval;

  regval = getreg32(stm32_cr_layer_t[layer->state.lid]);
  regvdbg("get LTDC_L%dCR=%08x\n", layer->state.lid + 1, regval);

  /* Disable the clut support during update the color table */

  if (enable == true)
    {
      regval |= LTDC_LxCR_CLUTEN;
    }
  else
    {
      regval &= ~LTDC_LxCR_CLUTEN;
    }

  regvdbg("set LTDC_L%dCR=%08x\n", layer->state.lid + 1, regval);
  putreg32(regval, stm32_cr_layer_t[layer->state.lid]);
}
#endif

/****************************************************************************
 * Name: stm32_ltdc_lsetopac
 *
 * Description:
 *   Helper to set the layer to opac
 *
 * Parameter:
 *   layer - Reference to the layer control structure
 *
 ****************************************************************************/

static inline void stm32_ltdc_lsetopac(FAR struct stm32_layer_s *layer)
{
  layer->opac = 0xff;
}

/****************************************************************************
 * Name: stm32_ltdc_lunsetopac
 *
 * Description:
 *   Helper to set the layer opacity to the alpha value
 *
 * Parameter:
 *   layer - Reference to the layer control structure
 *
 ****************************************************************************/

static inline void stm32_ltdc_lunsetopac(FAR struct stm32_layer_s *layer)
{
  layer->opac = 0;
}

/****************************************************************************
 * Name: stm32_ltdc_lgetopac
 *
 * Description:
 *   Helper to get the configured layer opacity
 *
 * Parameter:
 *   layer - Reference to the layer control structure
 *
 ****************************************************************************/

static inline uint8_t stm32_ltdc_lgetopac(FAR struct stm32_layer_s *layer)
{
  return layer->opac | layer->state.alpha;
}

/****************************************************************************
 * Name: stm32_ltdc_lvalidate
 *
 * Description:
 *   Helper to check if the layer is an valid ltdc layer
 *
 * Parameter:
 *   layer - Reference to the layer control structure
 *
 * Return:
 *   true  - layer valid
 *   false - layer invalid
 *
 ****************************************************************************/

static inline bool stm32_ltdc_lvalidate(FAR const struct stm32_layer_s *layer)
{
#ifdef CONFIG_STM32_LTDC_L2
  return layer == &LAYER_L1 || layer == &LAYER_L2;
#else
  return layer == &LAYER_L1;
#endif
}

/****************************************************************************
 * Name: stm32_ltdc_lvalidatearea
 *
 * Description:
 *   Check if layer coordinates out of valid area.
 *
 * Parameter:
 *   layer   - Reference to the layer control structure
 *   xpos    - top left x position of the active area
 *   ypos    - top left y position of the active area
 *   xres    - width of the active area
 *   yres    - height of teh active area
 *   srcxpos - Top left x position from where data visible in the active area
 *   srcypos - Top left y position from where data visible in the active area
 *
 * Return:
 *   On success - OK
 *   On error   - -EINVAL
 *
 ****************************************************************************/

#ifdef CONFIG_STM32_LTDC_INTERFACE
static int stm32_ltdc_lvalidatearea(FAR struct stm32_layer_s *layer,
                                    fb_coord_t xpos, fb_coord_t ypos,
                                    fb_coord_t xres, fb_coord_t yres,
                                    fb_coord_t srcxpos, fb_coord_t srcypos)
{
  FAR const struct fb_videoinfo_s *vinfo = &layer->state.vinfo;

  if ((xpos > vinfo->xres - 1) ||
      (ypos > vinfo->yres -1) ||
      (xres > vinfo->xres - xpos) ||
      (yres > vinfo->yres - ypos) ||
      (srcxpos > xpos + xres - 1) ||
      (srcypos > ypos + yres - 1))

    {
      gdbg("layer coordinates out of valid area: xpos = %d > %d, \
           ypos = %d > %d, width = %d > %d, height = %d > %d, \
           srcxpos = %d > %d, srcypos = %d > %d",
           xpos, vinfo->xres - 1,
           ypos, vinfo->yres - 1,
           xres, vinfo->xres - xpos,
           yres, vinfo->yres - ypos,
           srcxpos, xpos + xres - 1,
           srcypos, ypos + yres - 1);

      gdbg("Returning EINVAL\n");
      return -EINVAL;
    }

  return OK;
}
#endif

/****************************************************************************
 * Name: stm32_ltdc_lupdate
 *
 * Description:
 *   Updates shadow register content depending on the layer operation flag.
 *   This made changes for the given layer visible after the next shadow
 *   register reload.
 *
 * Parameter:
 *   layer - Reference to the layer control structure
 *
 ****************************************************************************/

static void stm32_ltdc_lupdate(FAR struct stm32_layer_s *layer)
{
  if (layer->operation & LTDC_LAYER_SETAREA)
    {
      /* Updates the layer horizontal and vertical position register */

      stm32_ltdc_larea(layer);
    }

  if (layer->operation & LTDC_LAYER_SETALPHAVALUE)
    {
      /* Updates the constant alpha register */

      stm32_ltdc_lalpha(layer);
    }

  if (layer->operation & LTDC_LAYER_SETBLENDMODE)
    {
      /* Update blendfactor 1 and 2 register */

      stm32_ltdc_lblendmode(layer, layer->bf1, layer->bf2);
    }

  if (layer->operation & LTDC_LAYER_SETCOLORKEY)
    {
      /* Update layer colorkey register */

      stm32_ltdc_lcolorkey(layer);
    }

  if (layer->operation & LTDC_LAYER_SETCOLOR)
    {
      /* Update layer color register */

      stm32_ltdc_lcolor(layer, layer->state.color);
    }

  if (layer->operation & LTDC_LAYER_SETENABLE)
    {
      /* Enable the layer */

      stm32_ltdc_lenable(layer);
    }
}

/****************************************************************************
 * Name: stm32_ltdc_larea
 *
 * Description:
 *   Change the active area of the layer
 *
 * Parameter:
 *   layer - Reference to the layer control structure
 *
 ****************************************************************************/

static void stm32_ltdc_larea(struct stm32_layer_s *layer)
{
  uint32_t    lxpos;
  uint32_t    lypos;
  uint32_t    whpcr;
  uint32_t    wvpcr;
  FAR struct stm32_ltdc_s *priv = &layer->state;
  FAR struct ltdc_area_s *area = &priv->area;

  regvdbg("xpos = %d, ypos = %d, xres = %d, yres = %d\n",
        area->xpos, area->ypos, area->xres, area->yres);

  lxpos = area->xpos + (STM32_LTDC_LxWHPCR_WHSTPOS + 1);
  lypos = area->ypos + (STM32_LTDC_LxWVPCR_WVSTPOS + 1);

  /* Accumulate horizontal position */

  whpcr =  LTDC_LxWHPCR_WHSTPOS(lxpos);
  whpcr |= LTDC_LxWHPCR_WHSPPOS(lxpos + area->xres - 1);

  /* Accumulate vertical position */

  wvpcr =  LTDC_LxWVPCR_WVSTPOS(lypos);
  wvpcr |= LTDC_LxWVPCR_WVSPPOS(lypos + area->yres - 1);

  /* Configure LxWHPCR / LxWVPCR register */

  regvdbg("set LTDC_L%dWHPCR=%08x\n", priv->lid + 1, whpcr);
  putreg32(whpcr, stm32_whpcr_layer_t[priv->lid]);
  regvdbg("set LTDC_L%dWVPCR=%08x\n", priv->lid + 1, wvpcr);
  putreg32(wvpcr, stm32_wvpcr_layer_t[priv->lid]);

  /* Configure framebuffer */

  stm32_ltdc_lframebuffer(layer);

  /* Clear area operation flag */

  layer->operation &= ~LTDC_LAYER_SETAREA;
}

/****************************************************************************
 * Name: stm32_ltdc_lpixelformat
 *
 * Description:
 *   Set the layer pixel format.
 *   Note! This changes have no effect until the shadow register reload has
 *   been done.
 *
 * Parameter:
 *   Reference to the layer control structure
 *
 ****************************************************************************/

static void stm32_ltdc_lpixelformat(FAR struct stm32_layer_s *layer)
{
  /* Configure PFCR register */

  regvdbg("set LTDC_L%dPFCR=%08x\n", layer->state.lid + 1,
            stm32_fmt_layer_t[layer->state.lid]);
  putreg32(stm32_fmt_layer_t[layer->state.lid],
            stm32_pfcr_layer_t[layer->state.lid]);
}

/****************************************************************************
 * Name: stm32_ltdc_framebuffer
 *
 * Description:
 *   Change layer framebuffer offset.
 *   Note! This changes have no effect until the shadow register reload has
 *   been done.
 *
 * Parameter:
 *   Reference to the layer control structure
 *
 ****************************************************************************/

static inline void stm32_ltdc_lframebuffer(FAR struct stm32_layer_s *layer)
{
  uint32_t offset;
  uint32_t cfblr;
  FAR struct stm32_ltdc_s *priv = &layer->state;
  FAR struct ltdc_area_s *area = &priv->area;
  FAR const struct fb_planeinfo_s *pinfo = &priv->pinfo;

  /* Configure LxCFBAR register */

  /* Calculate offset position in the framebuffer */

  offset = priv->xpos * STM32_LTDC_Lx_BYPP(pinfo->bpp) +
            pinfo->stride * priv->ypos;

  regvdbg("set LTDC_L%dCFBAR=%08x\n", priv->lid + 1, pinfo->fbmem + offset);
  putreg32((uint32_t)pinfo->fbmem + offset, stm32_cfbar_layer_t[priv->lid]);

  /* Configure LxCFBLR register */

  /* Calculate line length */

  cfblr = LTDC_LxCFBLR_CFBP(pinfo->stride) |
          LTDC_LxCFBLR_CFBLL(area->xres * STM32_LTDC_Lx_BYPP(pinfo->bpp) + 3);

  regvdbg("set LTDC_L%dCFBLR=%08x\n", priv->lid + 1, cfblr);
  putreg32(cfblr, stm32_cfblr_layer_t[priv->lid]);

  /* Configure LxCFBLNR register */

  regvdbg("set LTDC_L%dCFBLNR=%08x\n", priv->lid + 1, area->yres);
  putreg32(area->yres, stm32_cfblnr_layer_t[priv->lid]);
}

/****************************************************************************
 * Name: stm32_ltdc_lalpha
 *
 * Description:
 *   Change the layer alpha value and clear the alpha operation flag.
 *   Note! This changes have no effect until the shadow register reload has
 *   been done.
 *
 * Parameter:
 *   layer - Reference to the layer control structure
 *
 ****************************************************************************/

static void stm32_ltdc_lalpha(FAR struct stm32_layer_s *layer)
{
  uint8_t opac = stm32_ltdc_lgetopac(layer);
  regvdbg("set LTDC_L%dCACR=%02x\n", layer->state.lid + 1, opac);
  putreg32(opac, stm32_cacr_layer_t[layer->state.lid]);

  /* Clear the constant alpha operation flag */

  layer->operation &= ~LTDC_LAYER_SETALPHAVALUE;
}

/****************************************************************************
 * Name: stm32_ltdc_blendfactor
 *
 * Description:
 *   Change layer blend factors used for blend operation and clear the
 *   blendmode operation flag.
 *   Note! This changes have no effect until the shadow register reload has
 *   been done.
 *
 * Parameter:
 *   layer - Reference to the laxer control structure
 *   bf1   - Value of blend factor 1
 *   bf2   - Value of blend factor 2
 *
 ****************************************************************************/

static void stm32_ltdc_lblendmode(FAR struct stm32_layer_s *layer,
                                    uint8_t bf1, uint8_t bf2)
{
  regvdbg("set LTDC_L%dBFCR=%08x\n", layer->state.lid + 1,
          (LTDC_LxBFCR_BF1(bf1) | LTDC_LxBFCR_BF2(bf2)));
  putreg32((LTDC_LxBFCR_BF1(bf1) | LTDC_LxBFCR_BF2(bf2)),
           stm32_bfcr_layer_t[layer->state.lid]);

  /* Clear the blendmode operation flag */

  layer->operation &= ~LTDC_LAYER_SETBLENDMODE;
}

/****************************************************************************
 * Name: stm32_ltdc_lcolor
 *
 * Description:
 *   Change layer default color and clear the color operation flag.
 *   Note! This changes have no effect until the shadow register reload has
 *   been done.
 *
 * Parameter:
 *   layer - Reference to the layer control structure
 *
 ****************************************************************************/

static void stm32_ltdc_lcolor(FAR struct stm32_layer_s *layer, uint32_t argb)
{
  regvdbg("set LTDC_L%dDCCR=%08x\n", layer->state.lid + 1, argb);
  putreg32(argb, stm32_dccr_layer_t[layer->state.lid]);

  /* Clear the color operation flag */

  layer->operation &= ~LTDC_LAYER_SETCOLOR;
}

/****************************************************************************
 * Name: stm32_ltdc_lcolorkey
 *
 * Description:
 *   Change layer colorkey and clear the colorkey operation flag.
 *   Note! This changes have no effect until the shadow register reload has
 *   been done.
 *
 * Parameter:
 *   layer - Reference to the layer control structure
 *
 ****************************************************************************/

static void stm32_ltdc_lcolorkey(FAR struct stm32_layer_s *layer)
{
  uint32_t   regval;

  regval = getreg32(stm32_cr_layer_t[layer->state.lid]);

  if (layer->operation & LTDC_LAYER_ENABLECOLORKEY)
    {
      /* Set colorkey */

      regvdbg("set LTDC_L%dCKCR=%08x\n",
            layer->state.lid + 1, layer->state.colorkey);
      putreg32(layer->state.colorkey, stm32_ckcr_layer_t[layer->state.lid]);

      /* Enable colorkey */

      regval |= LTDC_LxCR_COLKEN;
    }
  else
    {
      /* Disable colorkey */

      regval &= ~LTDC_LxCR_COLKEN;
    }

  regvdbg("set LTDC_L%dCR=%08x\n", layer->state.lid + 1, regval);
  putreg32(regval, stm32_cr_layer_t[layer->state.lid]);

  /* Clear the colorkey operation flag */

  layer->operation &= ~LTDC_LAYER_SETCOLORKEY;
}

/****************************************************************************
 * Name: stm32_ltdc_lclut
 *
 * Description:
 *   Update the clut layer register during blank period.
 *   Note! The clut register are no shadow register.
 *
 * Parameter:
 *   layer  - Reference to the layer control structure
 *
 ****************************************************************************/

#ifdef STM32_LAYER_CLUT_SIZE
static void stm32_ltdc_lclut(FAR struct stm32_layer_s *layer,
                             FAR const struct fb_cmap_s *cmap)
{
  int            n;
  uint32_t  regval;
  uint32_t   *clut;
  irqstate_t flags;

  /* Disable clut during register update */

  stm32_ltdc_lclutenable(layer, false);

  /* Set the clut memory address */

  clut = layer->state.clut;

  /* Reload shadow control register.
   * This never changed any layer setting as long the layer register not up to
   * date. This is what stm32_update does.
   */

  stm32_ltdc_reload(LTDC_SRCR_IMR, false);

  flags = irqsave();

  /* Update the clut registers */

  for (n = cmap->first; n < cmap->len && n < STM32_LTDC_NCLUT; n++)
    {
      /* Update the layer clut entry */
#ifndef CONFIG_FB_TRANSPARENCY
       uint8_t  *clut888 = (uint8_t*)clut;
       uint16_t offset   = 3 * n;

       clut888[offset]     = cmap->blue[n];
       clut888[offset + 1] = cmap->green[n];
       clut888[offset + 2] = cmap->red[n];

       regval = (uint32_t)LTDC_CLUT_BLUE(clut888[offset]) |
                (uint32_t)LTDC_CLUT_GREEN(clut888[offset + 1]) |
                (uint32_t)LTDC_CLUT_RED(clut888[offset + 2]) |
                (uint32_t)LTDC_CLUT_ADD(n);
#else
      clut[n] = (uint32_t)LTDC_CLUT_ALPHA(cmap->transp[n]) |
                (uint32_t)LTDC_CLUT_RED(cmap->red[n]) |
                (uint32_t)LTDC_CLUT_GREEN(cmap->green[n]) |
                (uint32_t)LTDC_CLUT_BLUE(cmap->blue[n]);
      regval  = (uint32_t)LTDC_CLUT_ADD(n) | (clut[n] & LTDC_CLUT_RGB888_MASK);
#endif


      regvdbg("set LTDC_L%dCLUTWR = %08x, cmap->first = %d, cmap->len = %d\n",
             layer->state.lid + 1, regval, cmap->first, cmap->len);
      putreg32(regval, stm32_clutwr_layer_t[layer->state.lid]);
    }

  irqrestore(flags);

  /* Enable clut */

  stm32_ltdc_lclutenable(layer, true);

  /* Reload shadow control register */

  stm32_ltdc_reload(LTDC_SRCR_IMR, false);
}
#endif

/****************************************************************************
 * Name: stm32_ltdc_lenable
 *
 * Description:
 *   Disable or enable specific layer.
 *   Note! This changes have no effect until the shadow register reload has
 *   been done.
 *
 * Parameter:
 *   layer - Reference to the layer control structure
 *
 ****************************************************************************/

static void stm32_ltdc_lenable(FAR struct stm32_layer_s *layer)
{
  uint32_t   regval;

  /* Enable or disable layer */

  regval = getreg32(stm32_cr_layer_t[layer->state.lid]);

  if (layer->operation & LTDC_LAYER_ENABLE)
    {
      regval |= LTDC_LxCR_LEN;
    }
  else
    {
      regval &= ~LTDC_LxCR_LEN;
    }

  regvdbg("set LTDC_L%dCR=%08x\n", layer->state.lid + 1, regval);
  putreg32(regval, stm32_cr_layer_t[layer->state.lid]);

  /* Clear the enable operation flag */

  layer->operation &= ~LTDC_LAYER_SETENABLE;
}

/****************************************************************************
 * Name stm32_ltdc_lclear
 *
 * Description:
 *   Clear the whole layer
 *
 * Parameter:
 *   layer - Reference to the layer control structure
 *   color - The color to clear
 *
 * Return:
 *   OK      - On success
 *   -EINVAL - If one of the parameter invalid
 *
 ****************************************************************************/

static void stm32_ltdc_lclear(FAR struct stm32_layer_s *layer,
                                nxgl_mxpixel_t color)
{
  FAR struct stm32_ltdc_s *priv = &layer->state;

#if STM32_LTDC_L1_BPP == 8 || STM32_LTDC_L2_BPP == 8
  if (priv->pinfo.bpp == 8)
    {
      uint8_t *dest = (uint8_t*)priv->pinfo.fbmem;
      int i;

      gvdbg("Clearing display: BPP=%d color=%04x framebuffer=%08x size=%d\n",
            priv->pinfo.bpp, color, dest, priv->pinfo.fblen);

      for (i = 0; i < priv->pinfo.fblen; i += sizeof(uint8_t))
        {
          *dest++ = (uint8_t)color;
        }

      return;
    }
#endif

#if STM32_LTDC_L1_BPP == 16 || STM32_LTDC_L2_BPP == 16
  if (priv->pinfo.bpp == 16)
    {
      uint16_t *dest = (uint16_t*)priv->pinfo.fbmem;
      int i;

      gvdbg("Clearing display: BPP=%d color=%04x framebuffer=%08x size=%d\n",
            priv->pinfo.bpp, color, dest, priv->pinfo.fblen);

      for (i = 0; i < priv->pinfo.fblen; i += sizeof(uint16_t))
        {
          *dest++ = (uint16_t)color;
        }

      return;
    }
#endif

#if STM32_LTDC_L1_BPP == 24 || STM32_LTDC_L2_BPP == 24
  if (priv->pinfo.bpp == 24)
    {
      uint8_t *dest = (uint8_t*)priv->pinfo.fbmem;
      uint8_t r;
      uint8_t g;
      uint8_t b;
      int i;

      gvdbg("Clearing display: BPP=%d color=%04x framebuffer=%08x size=%d\n",
            priv->pinfo.bpp, color, dest, priv->pinfo.fblen);

      r = (uint8_t) color;
      g = (uint8_t) (color >> 8);
      b = (uint8_t) (color >> 16);

      for (i = 0; i < priv->pinfo.fblen; i += 3*sizeof(uint8_t))
        {
          *dest++ = r;
          *dest++ = g;
          *dest++ = b;
        }

      return;
    }
#endif

#if STM32_LTDC_L1_BPP == 32 || STM32_LTDC_L2_BPP == 32
  if (priv->pinfo.bpp == 32)
    {
      uint32_t *dest = (uint32_t*)priv->pinfo.fbmem;
      int i;

      gvdbg("Clearing display: BPP=%d color=%04x framebuffer=%08x size=%d\n",
            priv->pinfo.bpp, color, dest, priv->pinfo.fblen);

      for (i = 0; i < priv->pinfo.fblen; i += sizeof(uint32_t))
        {
          *dest++ = (uint32_t)color;
        }
    }
#endif
}

/****************************************************************************
 * Name: stm32_ltdc_linit
 *
 * Description:
 *   Initialize layer to their default states.
 *
 *   Initialize:
 *   - Reset layer
 *   - layer fram
 *   - Reset layerebuffers
 *   - layer position
 *   - layer pixelformat
 *   - layer color
 *   - layer colorkey
 *   - layer alpha
 *   - layer blendmode
 *   - layer dma2d interface binding
 *
 * Parameter
 *   layer - Reference to the layer control structure
 *
 ****************************************************************************/

static void stm32_ltdc_linit(int lid)
{
  /* Reset layer to their default state */

  FAR struct stm32_layer_s *layer = &LAYER(lid);
  FAR struct stm32_ltdc_s *state = &layer->state;
#ifdef CONFIG_STM32_LTDC_INTERFACE
  FAR struct ltdc_layer_s *ltdc = &layer->ltdc;

  /* Initialize the ltdc interface */

  ltdc->getlid       = stm32_getlid;
  ltdc->getvideoinfo = stm32_lgetvideoinfo;
  ltdc->getplaneinfo = stm32_lgetplaneinfo;
# ifdef STM32_LAYER_CLUT_SIZE
  ltdc->setclut      = stm32_setclut;
  ltdc->getclut      = stm32_getclut;
 #endif
  ltdc->setcolor     = stm32_setcolor;
  ltdc->getcolor     = stm32_getcolor;
  ltdc->setcolorkey  = stm32_setcolorkey;
  ltdc->getcolorkey  = stm32_getcolorkey;
  ltdc->setalpha     = stm32_setalpha;
  ltdc->getalpha     = stm32_getalpha;
  ltdc->setblendmode = stm32_setblendmode;
  ltdc->getblendmode = stm32_getblendmode;
  ltdc->setarea      = stm32_setarea;
  ltdc->getarea      = stm32_getarea;
  ltdc->update       = stm32_update;
 #ifdef CONFIG_STM32_DMA2D
  ltdc->blit         = stm32_blit;
  ltdc->blend        = stm32_blend;
  ltdc->fillarea     = stm32_fillarea;
 #endif
#endif

  /* Initialize the layer state */

  state->area.xpos   = 0;
  state->area.ypos   = 0;
  state->area.xres   = STM32_LTDC_WIDTH;
  state->area.yres   = STM32_LTDC_HEIGHT;
  state->xpos        = 0;
  state->ypos        = 0;
  state->color       = 0;
  state->colorkey    = 0;
  state->alpha       = 0xff;
  state->blendmode   = LTDC_BLEND_NONE;
  state->lock        = &g_lock;

  /* Initialize driver internals */

  layer->opac        = 0xff;
  layer->bf1         = LTDC_BF1_CONST_ALPHA;
  layer->bf2         = LTDC_BF2_CONST_ALPHA;
  layer->operation   = LTDC_LAYER_INIT;

  /* Clear the layer framebuffer */

  stm32_ltdc_lclear(layer, 0);

  /* Set Pixel input format */

  stm32_ltdc_lpixelformat(layer);

  /* Set position, color, colorkey, blendmode, alpha */

  stm32_ltdc_lupdate(layer);

#ifdef STM32_LAYER_CLUT_SIZE
  /* Disable clut by default */

  if (layer->state.vinfo.fmt == FB_FMT_RGB8)
    {
      stm32_ltdc_lclutenable(layer, false);
    }
#endif

#ifdef CONFIG_STM32_DMA2D
  /* Bind the dma2d interface */

  layer->dma2d = stm32_dma2dinitltdc(state);
  DEBUGASSERT(layer->dma2d);
#endif
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_getvideoinfo
 *
 * Description:
 *   Get the videoinfo for the framebuffer
 *
 * Parameter:
 *   vtable - The framebuffer driver object
 *   vinfo  - the videoinfo object
 *
 * Return:
 *   On success - OK
 *   On error   - -EINVAL
 *
 ****************************************************************************/

static int stm32_getvideoinfo(struct fb_vtable_s *vtable,
                                 struct fb_videoinfo_s *vinfo)
{
  gvdbg("vtable=%p vinfo=%p\n", vtable, vinfo);
  if (vtable)
    {
      FAR struct ltdc_layer_s *ltdc;
#ifdef CONFIG_STM32_LTDC_L2
      ltdc = (FAR struct ltdc_layer_s*)&LAYER_L2;
#else
      ltdc = (FAR struct ltdc_layer_s*)&LAYER_L1;
#endif
      return stm32_lgetvideoinfo(ltdc, vinfo);
    }

  gdbg("ERROR: Returning EINVAL\n");
  return -EINVAL;
}

/****************************************************************************
 * Name: stm32_getplaneinfo
 *
 * Description:
 *   Get the planeinfo for the framebuffer
 *
 * Parameter:
 *   vtable - The framebuffer driver object
 *   pinfo  - the planeinfo object
 *
 * Return:
 *   On success - OK
 *   On error   - -EINVAL
 *
 ****************************************************************************/

static int stm32_getplaneinfo(struct fb_vtable_s *vtable, int planeno,
                                 struct fb_planeinfo_s *pinfo)
{
  gvdbg("vtable=%p planeno=%d pinfo=%p\n", vtable, planeno, pinfo);
  if (vtable)
    {
      FAR struct ltdc_layer_s *ltdc;
#ifdef CONFIG_STM32_LTDC_L2
      ltdc = (FAR struct ltdc_layer_s*)&LAYER_L2;
#else
      ltdc = (FAR struct ltdc_layer_s*)&LAYER_L1;
#endif
      return stm32_lgetplaneinfo(ltdc, planeno, pinfo);
    }

  gdbg("Returning EINVAL\n");
  return -EINVAL;
}

/****************************************************************************
 * Name: stm32_getcmap
 *
 * Description:
 *   Get a range of CLUT values for the LCD
 *
 * Parameter:
 *   vtable - The framebuffer driver object
 *   cmap   - the color table
 *
 * Return:
 *   On success - OK
 *   On error   - -EINVAL
 *
 ****************************************************************************/

#ifdef STM32_LAYER_CLUT_SIZE
static int stm32_getcmap(struct fb_vtable_s *vtable,
                            struct fb_cmap_s *cmap)
{
#ifdef CONFIG_STM32_LTDC_L2
  return stm32_getclut((FAR struct ltdc_layer_s*)&LAYER_L2, cmap);
#else
  return stm32_getclut((FAR struct ltdc_layer_s*)&LAYER_L1, cmap);
#endif
}

/****************************************************************************
 * Name: stm32_putcmap
 *
 * Description:
 *   Set a range of the CLUT values for the LCD
 *
 * Parameter:
 *   vtable - The framebuffer driver object
 *   cmap   - the color table
 *
 * Return:
 *   On success - OK
 *   On error   - -EINVAL
 *
 ****************************************************************************/

static int stm32_putcmap(struct fb_vtable_s *vtable,
                            const struct fb_cmap_s *cmap)
{
#ifdef CONFIG_STM32_LTDC_L2
  return stm32_setclut((FAR struct ltdc_layer_s*)&LAYER_L2, cmap);
#else
  return stm32_setclut((FAR struct ltdc_layer_s*)&LAYER_L1, cmap);
#endif
}
#endif /* STM32_LAYER_CLUT_SIZE */

/****************************************************************************
 * Name: stm32_lgetvideoinfo
 *
 * Description:
 *   Get video information about the layer
 *
 * Parameter:
 *   layer  - Reference to the layer control structure
 *   vinfo  - Reference to the video info structure
 *
 * Return:
 *   On success - OK
 *   On error   - -EINVAL
 *
 ****************************************************************************/

static int stm32_lgetvideoinfo(struct ltdc_layer_s *layer,
                                 struct fb_videoinfo_s *vinfo)
{
  gvdbg("layer=%p vinfo=%p\n", layer, vinfo);
  FAR struct stm32_layer_s *priv = (FAR struct stm32_layer_s *)layer;

  if (stm32_ltdc_lvalidate(priv))
    {
      memcpy(vinfo, &priv->state.vinfo, sizeof(struct fb_videoinfo_s));

      return OK;
    }

  gdbg("Returning EINVAL\n");
  return -EINVAL;
}

/****************************************************************************
 * Name: stm32_lgetplaneinfo
 *
 * Description:
 *   Get plane information about the layer
 *
 * Parameter:
 *   layer   - Reference to the layer control structure
 *   planeno - Number of the plane
 *   pinfo   - Reference to the plane info structure
 *
 * Return:
 *   On success - OK
 *   On error   - -EINVAL
 *
 ****************************************************************************/

static int stm32_lgetplaneinfo(struct ltdc_layer_s *layer, int planeno,
                                 struct fb_planeinfo_s *pinfo)
{
  gvdbg("layer=%p planeno=%d pinfo=%p\n", layer, planeno, pinfo);
  FAR struct stm32_layer_s *priv = (FAR struct stm32_layer_s *)layer;

  if (stm32_ltdc_lvalidate(priv) && planeno == 0)
    {
      memcpy(pinfo, &priv->state.pinfo, sizeof(struct fb_planeinfo_s));
      return OK;
    }

  gdbg("Returning EINVAL\n");
  return -EINVAL;
}

/****************************************************************************
 * Name: stm32_setclut
 *
 * Description:
 *   Configure layer clut (color lookup table).
 *   Non clut is defined during initializing.
 *   Clut is active during next vertical blank period. Do not need an update.
 *
 * Parameter:
 *   layer  - Reference to the layer structure
 *   cmap   - color lookup table with up the 256 entries
 *
 * Return:
 *   On success - OK
 *   On error   - -EINVAL
 *
 ****************************************************************************/

#ifdef STM32_LAYER_CLUT_SIZE
static int stm32_setclut(struct ltdc_layer_s *layer,
                        const struct fb_cmap_s *cmap)
{
  int   ret;
  FAR struct stm32_layer_s *priv = (FAR struct stm32_layer_s *)layer;
  gvdbg("layer=%p cmap=%p\n", layer, cmap);

  if (stm32_ltdc_lvalidate(priv) && cmap)
    {
      sem_wait(priv->state.lock);

      if (priv->state.vinfo.fmt != FB_FMT_RGB8)
        {
          gdbg("Error: CLUT is not supported for the pixel format: %d\n",
                    priv->state.vinfo.fmt);
          ret = -EINVAL;
        }
      else if (cmap->first >= STM32_LTDC_NCLUT)
        {
          gdbg("Error: only %d color table entries supported\n",
                    STM32_LTDC_NCLUT);
          ret = -EINVAL;
        }
      else
        {
          /* Update layer clut and clut register */

          stm32_ltdc_lclut(priv, cmap);

          ret = OK;
        }

      sem_post(priv->state.lock);

      return ret;
    }

  gdbg("Returning EINVAL\n");
  return -EINVAL;
}

/****************************************************************************
 * Name: stm32_getclut
 *
 * Description:
 *   Get configured layer clut (color lookup table).
 *
 * Parameter:
 *   layer - Reference to the layer structure
 *   cmap  - Reference to valid color lookup table accept up the 256 color
 *           entries
 *
 * Return:
 *   On success - OK
 *   On error   - -EINVAL
 *
 ****************************************************************************/

static int stm32_getclut(struct ltdc_layer_s *layer,
                         struct fb_cmap_s *cmap)
{
  int   ret;
  FAR struct stm32_layer_s *priv = (FAR struct stm32_layer_s *)layer;
  gvdbg("layer=%p cmap=%p\n", layer, cmap);

  if (priv == &LAYER_L1 || priv == &LAYER_L2)
    {
      sem_wait(priv->state.lock);
#ifdef CONFIG_STM32_DMA2D
      /*
       * Note! We share the same color lookup table with the dma2d driver and
       * the getclut implementation works in the same way.
       * To prevent redundant code we simply call the getclut function of the
       * dma2d interface.
       */

      ret = priv->dma2d->getclut(priv->dma2d, cmap);
#else
      if (priv->state.vinfo.fmt != FB_FMT_RGB8)
        {
          gdbg("Error: CLUT is not supported for the pixel format: %d\n",
                    priv->state.vinfo.fmt);
          ret = -EINVAL;
        }
      else if (cmap->first >= STM32_LTDC_NCLUT)
        {
          gdbg("Error: only %d color table entries supported\n",
                    STM32_LTDC_NCLUT);
          ret = -EINVAL;
        }
      else
        {
          /* Copy from the layer clut */

          uint32_t *clut;
          int      n;

          clut = priv->state.clut;

          for (n = cmap->first; n < cmap->len && n < STM32_LTDC_NCLUT; n++)
            {
# ifndef CONFIG_FB_TRANSPARENCY
              uint8_t  *clut888 = (uint8_t*)clut;
              uint16_t offset   = 3 * n;

              cmap->blue[n]   = clut888[offset];
              cmap->green[n]  = clut888[offset + 1];
              cmap->red[n]    = clut888[offset + 2];

              regvdbg("n=%d, red=%02x, green=%02x, blue=%02x\n", n,
                      clut888[offset], clut888[offset + 1],
                      clut888[offset + 2]);
# else
              cmap->transp[n] = (uint8_t)LTDC_CMAP_ALPHA(clut[n]);
              cmap->red[n]    = (uint8_t)LTDC_CMAP_RED(clut[n]);
              cmap->green[n]  = (uint8_t)LTDC_CMAP_GREEN(clut[n]);
              cmap->blue[n]   = (uint8_t)LTDC_CMAP_BLUE(clut[n]);

              regvdbg("n=%d, alpha=%02x, red=%02x, green=%02x, blue=%02x\n", n,
                      DMA2D_CMAP_ALPHA(clut[n]), DMA2D_CMAP_RED(clut[n]),
                      DMA2D_CMAP_GREEN(clut[n]), DMA2D_CMAP_BLUE(clut[n]));
# endif
            }

          ret = OK;
        }
#endif
      sem_post(priv->state.lock);

      return ret;
    }

  gdbg("Returning EINVAL\n");
  return -EINVAL;
}
#endif /* STM32_LAYER_CLUT_SIZE */

#ifdef CONFIG_STM32_LTDC_INTERFACE
/****************************************************************************
 * Name: getlid
 *
 * Description:
 *   Get a specific layer identifier.
 *
 * Parameter:
 *   layer - Reference to the layer structure
 *   lid   - Reference to store the layer id
 *   flag  - Operation flag describe the layer identifier
 *           e.g. get the current active or inactive layer.
 *           See LTDC_LAYER_* for possible values
 *
 * Return:
 *   OK - On success
 *   Null if invalid flag
 *
 ****************************************************************************/

static int stm32_getlid(FAR struct ltdc_layer_s *layer, int *lid, uint32_t flag)
{
  FAR struct stm32_layer_s *priv = (FAR struct stm32_layer_s *)layer;

  gvdbg("flag = %08x\n", flag);

  if (stm32_ltdc_lvalidate(priv))
    {
      int   ret = OK;

      sem_wait(priv->state.lock);

      switch (flag)
        {
          case LTDC_LAYER_OWN:
            *lid = priv->state.lid;
            break;
#ifdef CONFIG_STM32_LTDC_L2
          case LTDC_LAYER_ACTIVE:
            *lid = g_lactive;
            break;
          case LTDC_LAYER_INACTIVE:
            *lid = !g_lactive;
            break;
          case LTDC_LAYER_TOP:
            *lid = LTDC_LAYER_L2;
            break;
          case LTDC_LAYER_BOTTOM:
            *lid = LTDC_LAYER_L1;
            break;
#else
          case LTDC_LAYER_ACTIVE:
          case LTDC_LAYER_INACTIVE:
          case LTDC_LAYER_TOP:
          case LTDC_LAYER_BOTTOM:
            *lid = LTDC_LAYER_L1;
            break;
#endif
#ifdef CONFIG_STM32_DMA2D
          case LTDC_LAYER_DMA2D:
            ret = priv->dma2d->getlid(priv->dma2d, lid);
            break;
#endif
          default:
            ret = EINVAL;
            gdbg("Returning EINVAL\n");
            break;
        }

      sem_post(priv->state.lock);

      return ret;
    }

  gdbg("Returning EINVAL\n");
  return -EINVAL;
}

/****************************************************************************
 * Name: stm32_setcolor
 *
 * Description:
 *    Configure layer default color value for the non active layer area.
 *    Default value during initializing: 0x00000000
 *    Color is active after next update.
 *
 * Parameter:
 *   layer - Reference to the layer structure
 *   argb  - ARGB8888 color value
 *
 * Return:
 *   On success - OK
 *   On error - -EINVAL
 *
 ****************************************************************************/

static int stm32_setcolor(FAR struct ltdc_layer_s *layer, uint32_t argb)
{
  FAR struct stm32_layer_s *priv = (FAR struct stm32_layer_s *)layer;
  gvdbg("layer = %p, argb = %08x\n", layer, argb);

  if (stm32_ltdc_lvalidate(priv))
    {
      sem_wait(priv->state.lock);
      priv->state.color = argb;
      priv->operation |= LTDC_LAYER_SETCOLOR;
      sem_post(priv->state.lock);

      return OK;
    }

  gdbg("Returning EINVAL\n");
  return -EINVAL;
}

/****************************************************************************
 * Name: stm32_getcolor
 *
 * Description:
 *   Get configured layer color for the non active layer area.
 *
 * Parameter:
 *   layer - Reference to the layer structure
 *   argb  - Reference to store the ARGB8888 color value
 *
 * Return:
 *   On success - OK
 *   On error   - -EINVAL
 *
*******************************************************************************/

static int stm32_getcolor(FAR struct ltdc_layer_s *layer, uint32_t *argb)
{
  FAR struct stm32_layer_s *priv = (FAR struct stm32_layer_s *)layer;
  gvdbg("layer = %p, argb = %p\n", layer, argb);

  if (stm32_ltdc_lvalidate(priv))
    {
      sem_wait(priv->state.lock);
      *argb = priv->state.color;
      sem_post(priv->state.lock);

      return OK;
    }

  gdbg("Returning EINVAL\n");
  return -EINVAL;
}

/****************************************************************************
 * Name: stm32_setcolorkey
 *
 * Description:
 *    Configure layer default color key (chromakey) value for transparency.
 *    Layer default value during initializing: 0x00000000
 *    Colorkey is active after next update.
 *
 * Parameter:
 *   layer  - Reference to the layer structure
 *   rgb   - RGB888 color value
 *
 * Return:
 *   On success - OK
 *   On error   - -EINVAL
 *
 ****************************************************************************/

static int stm32_setcolorkey(FAR struct ltdc_layer_s *layer, uint32_t rgb)
{
  FAR struct stm32_layer_s *priv = (FAR struct stm32_layer_s *)layer;
  gvdbg("layer = %p, argb = %08x\n", layer, rgb);

  if (stm32_ltdc_lvalidate(priv))
    {
      sem_wait(priv->state.lock);
      priv->state.colorkey = rgb;
      priv->operation |= LTDC_LAYER_SETCOLORKEY;
      sem_post(priv->state.lock);

      return OK;
    }

  gdbg("Returning EINVAL\n");
  return -EINVAL;
}

/****************************************************************************
 * Name: stm32_getcolorkey
 *
 * Description:
 *   Get the configured layer color key (chromakey) for transparency.
 *
 * Parameter:
 *   layer  - Reference to the layer structure
 *   rgb    - Reference to store the RGB888 color key
 *
 * Return:
 *   On success - OK
 *   On error   - -EINVAL
 *
 ****************************************************************************/

static int stm32_getcolorkey(FAR struct ltdc_layer_s *layer, uint32_t *rgb)
{
  FAR struct stm32_layer_s *priv = (FAR struct stm32_layer_s *)layer;
  gvdbg("layer = %p, argb = %p\n", layer, rgb);

  if (stm32_ltdc_lvalidate(priv))
    {
      sem_wait(priv->state.lock);
      *rgb = priv->state.colorkey;
      sem_post(priv->state.lock);

      return OK;
    }

  gdbg("Returning EINVAL\n");
  return -EINVAL;
}

/****************************************************************************
 * Name: setalpha
 *
 * Description:
 *   Configure layer alpha value factor into blend operation.
 *   During the layer blend operation the source alpha value is multiplied
 *   with this alpha value. If the source color format doesn't support alpha
 *   channel (e.g. non ARGB8888) this alpha value will be used as constant
 *   alpha value for blend operation.
 *   Default alpha value during initializing: 0xff
 *   Alpha is active after next update.
 *
 * Parameter:
 *   layer - Reference to the layer structure
 *   alpha - Alpha value
 *
 * Return:
 *   On success - OK
 *   On error - -EINVAL
 *
 ****************************************************************************/

static int stm32_setalpha(FAR struct ltdc_layer_s *layer, uint8_t alpha)
{
  FAR struct stm32_layer_s *priv = (FAR struct stm32_layer_s *)layer;
  gvdbg("layer = %p, alpha = %02x\n", layer, alpha);

  if (stm32_ltdc_lvalidate(priv))
    {
      sem_wait(priv->state.lock);
      priv->state.alpha = alpha;
      priv->operation  |= LTDC_LAYER_SETALPHAVALUE;
      sem_post(priv->state.lock);

      return OK;
    }

  gdbg("Returning EINVAL\n");
  return -EINVAL;
}

/****************************************************************************
 * Name: stm32_getalpha
 *
 * Description:
 *   Get configured layer alpha value factor for blend operation.
 *
 * Parameter:
 *   layer - Reference to the layer structure
 *   alpha - Reference to store the alpha value
 *
 * Return:
 *   On success - OK
 *   On error - -EINVAL
 *
 ****************************************************************************/

static int stm32_getalpha(FAR struct ltdc_layer_s *layer, uint8_t *alpha)
{
  FAR struct stm32_layer_s *priv = (FAR struct stm32_layer_s *)layer;
  gvdbg("layer = %p, alpha = %p\n", layer, alpha);

  if (stm32_ltdc_lvalidate(priv))
    {
      sem_wait(priv->state.lock);
      *alpha = priv->state.alpha;
      sem_post(priv->state.lock);

      return OK;
    }

  gdbg("Returning EINVAL\n");
  return -EINVAL;
}

/****************************************************************************
 * Name: setblendmode
 *
 * Description:
 *   Configure blend mode of the layer.
 *   Default mode during initializing: LTDC_BLEND_NONE
 *   Blendmode is active after next update.
 *
 * Parameter:
 *   layer - Reference to the layer structure
 *   mode  - Blend mode (see LTDC_BLEND_*)
 *
 * Return:
 *   On success - OK
 *   On error - -EINVAL
 *
 * Procedure information:
 *   LTDC_BLEND_NONE:
 *     Informs the driver to disable all blend operation for the given layer.
 *     That means the layer is opaque. Note this has no effect on the
 *     colorkey settings.
 *
 *   LTDC_BLEND_ALPHA:
 *     Informs the driver to enable alpha blending for the given layer.
 *
 *   LTDC_BLEND_COLORKEY:
 *     Informs the driver to enable colorkeying for the given layer.
 *
 *   LTDC_BLEND_SRCPIXELALPHA:
 *     Informs the driver to use the pixel alpha value of the layer instead
 *     the constant alpha value. This is only useful for ARGB8888
 *     color format.
 *
 *   LTDC_BLEND_DESTPIXELALPHA:
 *     Informs the driver to use the pixel alpha value of the subjacent layer
 *     instead the constant alpha value. This is only useful for ARGB8888
 *     color format.
 *
 ****************************************************************************/

static int stm32_setblendmode(FAR struct ltdc_layer_s *layer, uint32_t mode)
{
  FAR struct stm32_layer_s *priv = (FAR struct stm32_layer_s *)layer;
  uint32_t   blendmode = mode;
  gvdbg("layer = %p, mode = %08x\n", layer, mode);

  if (stm32_ltdc_lvalidate(priv))
    {
      int         ret = OK;

      sem_wait(priv->state.lock);

      /* Disable colorkeying by default */

      priv->operation &=~ LTDC_LAYER_ENABLECOLORKEY;

      if (blendmode & (LTDC_BLEND_ALPHA|LTDC_BLEND_PIXELALPHA|
                        LTDC_BLEND_ALPHAINV|LTDC_BLEND_PIXELALPHAINV))
        {
          /* Enable any alpha blending */

          stm32_ltdc_lunsetopac(priv);
        }
      else
        {
          /* Disable any alpha blending */

          stm32_ltdc_lsetopac(priv);
        }

      if (blendmode & LTDC_BLEND_ALPHA || blendmode == LTDC_BLEND_NONE)
        {
          /* alpha blending introduce LTDC_BLEND_ALPHAINV */

          priv->bf1         = LTDC_BF1_CONST_ALPHA;
          priv->bf2         = LTDC_BF2_CONST_ALPHA;
          blendmode        &= ~LTDC_BLEND_ALPHA;
        }

      if (blendmode & LTDC_BLEND_PIXELALPHA)
        {
          /* pixel alpha blending introduce LTDC_BLEND_PIXELALPHAINV */

          priv->bf1         = LTDC_BF1_PIXEL_ALPHA;
          priv->bf2         = LTDC_BF2_PIXEL_ALPHA;
          blendmode        &= ~LTDC_BLEND_PIXELALPHA;
        }

      if (blendmode & LTDC_BLEND_ALPHAINV)
        {
          /* alpha blending of source input */

          priv->bf2         = LTDC_BF2_CONST_ALPHA;
          blendmode        &= ~LTDC_BLEND_ALPHAINV;
        }

      if (blendmode & LTDC_BLEND_PIXELALPHAINV)
        {
          /* pixel alpha blending of source input */

          priv->bf2         = LTDC_BF2_PIXEL_ALPHA;
          blendmode        &= ~LTDC_BLEND_PIXELALPHAINV;
        }

      if (mode & LTDC_BLEND_COLORKEY)
        {
          /* Enable colorkeying */

          priv->operation |= LTDC_LAYER_ENABLECOLORKEY;
          blendmode       &= ~LTDC_BLEND_COLORKEY;
        }
      if (blendmode)
        {
          gdbg("Unknown blendmode %02x\n", blendmode);
          ret = -EINVAL;
        }

      if (ret == OK)
        {
          priv->state.blendmode = mode;
          priv->operation      |= (LTDC_LAYER_SETBLENDMODE|
                                   LTDC_LAYER_SETALPHAVALUE|
                                   LTDC_LAYER_SETCOLORKEY);
        }

      sem_post(priv->state.lock);
      return ret;
    }

  gdbg("Returning EINVAL\n");
  return -EINVAL;
}

/****************************************************************************
 * Name: stm32_getblendmode
 *
 * Description:
 *   Get configured blend mode of the layer.
 *
 * Parameter:
 *   layer - Reference to the layer structure
 *   mode  - Reference to store the blend mode
 *
 * Return:
 *   On success - OK
 *   On error - -EINVAL
 ****************************************************************************/

static int stm32_getblendmode(FAR struct ltdc_layer_s *layer, uint32_t *mode)
{
  FAR struct stm32_layer_s *priv = (FAR struct stm32_layer_s *)layer;
  gvdbg("layer = %p, mode = %p\n", layer, mode);

  if (stm32_ltdc_lvalidate(priv))
    {
      sem_wait(priv->state.lock);
      *mode = priv->state.blendmode;
      sem_post(priv->state.lock);

      return OK;
    }

  gdbg("Returning EINVAL\n");
  return -EINVAL;
}

/****************************************************************************
 * Name: stm32_setarea
 *
 * Description:
 *   Configure visible layer area and the reference position of the first
 *   pixel of the whole layer which is the first visible top left pixel in
 *   the active area.
 *   Area is active after next update.
 *
 * Parameter:
 *   layer   - Reference to the layer control structure
 *   area    - Reference to the valid area structure for the new active area
 *   srcxpos - x position of the visible pixel of the whole layer
 *   srcypos - y position of the visible pixel of the whole layer
 *
 * Return:
 *   On success - OK
 *   On error - -EINVAL
 *
 * Procedure Information:
 *   If the srcxpos and srcypos unequal the the xpos and ypos of the area
 *   structure this acts like moving the visible area to another position on
 *   the screen during the next update operation.
 *
 ****************************************************************************/

static int stm32_setarea(FAR struct ltdc_layer_s *layer,
                        FAR const struct ltdc_area_s *area,
                        fb_coord_t srcxpos,
                        fb_coord_t srcypos)
{
  FAR struct stm32_layer_s *priv = (FAR struct stm32_layer_s *)layer;
  gvdbg("layer = %p, area = %p, srcxpos = %d, srcypos = %d\n",
            layer, area, srcxpos, srcypos);

  if (stm32_ltdc_lvalidate(priv))
    {
      int ret;

      sem_wait(priv->state.lock);

      ret = stm32_ltdc_lvalidatearea(priv, area->xpos, area->ypos, area->xres,
                                    area->yres, srcxpos, srcypos);

      if (ret == OK)
        {
          priv->state.xpos       = srcxpos;
          priv->state.ypos       = srcypos;
          priv->state.area.xpos  = area->xpos;
          priv->state.area.ypos  = area->ypos;
          priv->state.area.xres  = area->xres;
          priv->state.area.yres  = area->yres;
          priv->operation       |= LTDC_LAYER_SETAREA;
        }

      sem_post(priv->state.lock);

      return ret;
    }

  gdbg("Returning EINVAL\n");
  return -EINVAL;
}

/****************************************************************************
 * Name: stm32_getarea
 *
 * Description:
 *    Get configured visible layer area.
 *
 * Parameter:
 *   layer   - Reference to the layer control structure
 *   area    - Reference to the area structure to store the active area
 *   srcxpos - Reference to store the referenced x position of the whole layer
 *   srcypos - Reference to store the reterenced y position of the whole layer
 *
 * Return:
 *   On success - OK
 *   On error - -EINVAL
 *
 ****************************************************************************/

static int stm32_getarea(FAR struct ltdc_layer_s *layer,
                        FAR struct ltdc_area_s *area,
                        fb_coord_t *srcxpos, fb_coord_t *srcypos)
{
  FAR struct stm32_layer_s *priv = (FAR struct stm32_layer_s *)layer;
  gvdbg("layer = %p, area = %p, srcxpos = %p, srcypos = %p\n",
            layer, area, srcxpos, srcypos);

  if (stm32_ltdc_lvalidate(priv))
    {
      sem_wait(priv->state.lock);
      *srcxpos = priv->state.xpos;
      *srcypos = priv->state.ypos;
      memcpy(area, &priv->state.area, sizeof(struct ltdc_area_s));
      sem_post(priv->state.lock);

      return OK;
    }

  gdbg("Returning EINVAL\n");
  return -EINVAL;
}

/****************************************************************************
 * Name: stm32_update
 *
 * Description:
 *   Update current layer settings and make changes visible.
 *
 * Parameter:
 *   layer   - Reference to the layer structure
 *   mode    - operation mode
 *
 * Return:
 *    OK        - On success
 *   -EINVAL    - If one of the parameter invalid
 *   -ECANCELED - Operation cancelled, something goes wrong
 *
 * Procedure information:
 *   LTDC_UPDATE_SIM:
 *     Informs the driver to update both layers simultaneously. Otherwise update
 *     the given layer only.
 *
 *   LTDC_UPDATE_FLIP:
 *     Informs the driver to perform a flip operation.
 *     This only effects the ltdc layer 1 and 2 and can be useful for double
 *     buffering. Each flip operation changed the active layer to the inactive
 *     and vice versa. In the context of the ltdc that means, the inactive layer
 *     is complete disabled. So the subjacent layer is the background layer
 *     (background color). To reactivate both layer and their current settings
 *     perform an update without LTDC_UPDATE_FLIP flag.
 *
 *   LTDC_UPDATE_ACTIVATE:
 *     Informs the driver that the given layer should be the active layer when
 *     the operation is complete.
 *
 *   LTDC_SYNC_VBLANK:
 *     Informs the driver to update the layer upon vertical blank. Otherwise
 *     immediately.
 *
 ****************************************************************************/

static int stm32_update(FAR struct ltdc_layer_s *layer, uint32_t mode)
{
  FAR struct stm32_layer_s *priv = (FAR struct stm32_layer_s *)layer;
#ifdef CONFIG_STM32_LTDC_L2
  FAR struct stm32_layer_s *active = &LAYER(g_lactive);
  FAR struct stm32_layer_s *inactive = &LAYER(!g_lactive);
#endif

  gvdbg("layer = %p, mode = %08x\n", layer, mode);

  if (stm32_ltdc_lvalidate(priv))
    {
      /* Reload immediately by default */

      bool    waitvblank = false;
      uint8_t reload = LTDC_SRCR_IMR;

      sem_wait(priv->state.lock);

      if (mode & LTDC_SYNC_VBLANK)
        {
          reload = LTDC_SRCR_VBR;
        }

      if (mode & LTDC_SYNC_WAIT)
        {
          waitvblank = true;
        }

      /* Ensures that last register reload operation has been completed */

      if (stm32_ltdc_waitforirq() != OK)
        {
          gdbg("Returning ECANCELED\n");
          return -ECANCELED;
        }

      /* Update the given layer */

      stm32_ltdc_lupdate(priv);

#ifdef CONFIG_STM32_LTDC_L2
      /* The following operation only useful if layer 2 is supported.
       * Otherwise ignore it.
       */

      if (mode & LTDC_UPDATE_SIM)
        {
          /* Also update the flip layer */

          stm32_ltdc_lupdate(&LAYER(!priv->state.lid));
        }

      if (mode & LTDC_UPDATE_ACTIVATE)
        {
          /* Set the given layer to the next active layer */

          g_lactive = priv->state.lid;

          /* Also change the current active layer for flip operation */

          active = &LAYER(!g_lactive);
        }

      if (mode & LTDC_UPDATE_FLIP)
        {
          /* Reset if manipulated by ACTIVATE flag */

          inactive = &LAYER(!active->state.lid);

          /* Set blendfactor for current active layer to there reset value */

          stm32_ltdc_lblendmode(active, STM32_LTDC_BF1_RESET,
                                        STM32_LTDC_BF2_RESET);

          /* Set blendfactor for current inactive layer */

          stm32_ltdc_lblendmode(inactive, inactive->bf1, inactive->bf2);

          /* Disable the active layer */

          active->operation &= ~LTDC_LAYER_ENABLE;

          stm32_ltdc_lenable(active);

          /* Enable the inactive layer */

          inactive->operation |= LTDC_LAYER_ENABLE;

          stm32_ltdc_lenable(inactive);

          /* Ensure that both layer active and the manipulated layer
           * settings restored during the next update (non flip) operation
           */

          active->operation |= (LTDC_LAYER_SETBLENDMODE|
                                LTDC_LAYER_ENABLE|
                                LTDC_LAYER_SETCOLOR|
                                LTDC_LAYER_SETENABLE);

          /* Change layer activity */

          g_lactive = inactive->state.lid;
        }
#endif

      /* Make the changes visible */

      stm32_ltdc_reload(reload, waitvblank);

      sem_post(priv->state.lock);

      return OK;
    }

  gdbg("Returning EINVAL\n");
  return -EINVAL;
}

#ifdef CONFIG_STM32_DMA2D
/****************************************************************************
 * Name: stm32_blit
 *
 * Description:
 *   Copy selected area from a source layer to selected position of the
 *   destination layer.
 *
 * Parameter:
 *   dest     - Reference to the destination layer
 *   destxpos - Selected x position of the destination layer
 *   destypos - Selected y position of the destination layer
 *   src      - Reference to the source layer
 *   srcarea  - Reference to the selected area of the source layer
 *
 * Return:
 *    OK      - On success
 *   -EINVAL  - If one of the parameter invalid or if the size of the selected
 *              source area outside the visible area of the destination layer.
 *              (The visible area usually represents the display size)
 *
 ****************************************************************************/

static int stm32_blit(FAR struct ltdc_layer_s *dest,
                      fb_coord_t destxpos, fb_coord_t destypos,
                      FAR const struct dma2d_layer_s *src,
                      FAR const struct ltdc_area_s *srcarea)
{
  FAR struct stm32_layer_s *priv = (FAR struct stm32_layer_s *)dest;

  gvdbg("dest = %p, destxpos = %d, destypos = %d, src = %p, srcarea = %p\n",
        dest, destxpos, destypos, src, srcarea);

  if (stm32_ltdc_lvalidate(priv))
    {
      int   ret;

      sem_wait(priv->state.lock);
      priv->dma2d->blit(priv->dma2d, destxpos, destypos, src, srcarea);
      sem_post(priv->state.lock);

      return ret;
    }

  gdbg("Returning EINVAL\n");
  return -EINVAL;
}

/****************************************************************************
 * Name: stm32_blend
 *
 * Description:
 *   Blends the selected area from a foreground layer with selected position
 *   of the background layer. Copy the result to the destination layer. Note!
 *   The content of the foreground and background layer is not changed.
 *
 * Parameter:
 *   dest     - Reference to the destination layer
 *   destxpos - Selected x position of the destination layer
 *   destypos - Selected y position of the destination layer
 *   fore     - Reference to the foreground layer
 *   forexpos - Selected x position of the foreground layer
 *   foreypos - Selected y position of the foreground layer
 *   back     - Reference to the background layer
 *   backarea - Reference to the selected area of the background layer
 *
 * Return:
 *    OK      - On success
 *   -EINVAL  - If one of the parameter invalid or if the size of the selected
 *              source area outside the visible area of the destination layer.
 *              (The visible area usually represents the display size)
 *
 ****************************************************************************/

static int stm32_blend(FAR struct ltdc_layer_s *dest,
                        fb_coord_t destxpos, fb_coord_t destypos,
                        FAR const struct dma2d_layer_s *fore,
                        fb_coord_t forexpos, fb_coord_t foreypos,
                        FAR const struct dma2d_layer_s *back,
                        FAR const struct ltdc_area_s *backarea)
{
  FAR struct stm32_layer_s *priv = (FAR struct stm32_layer_s *)dest;

  gvdbg("dest=%p, destxpos=%d, destypos=%d, "
        "fore=%p, forexpos=%d foreypos=%d, "
        "back=%p, backarea=%p\n",
        dest, destxpos, destypos, fore, forexpos, foreypos, back, backarea);

  if (stm32_ltdc_lvalidate(priv))
    {
      int   ret;

      sem_wait(priv->state.lock);
      priv->dma2d->blend(priv->dma2d, destxpos, destypos,
                        fore, forexpos, foreypos, back, backarea);
      sem_post(priv->state.lock);

      return ret;
    }

  gdbg("Returning EINVAL\n");
  return -EINVAL;
}

/****************************************************************************
 * Name: fillarea
 *
 * Description:
 *   Fill the selected area of the whole layer with a specific color.
 *
 * Parameter:
 *   layer    - Reference to the layer structure
 *   area     - Reference to the valid area structure select the area
 *   color    - Color to fill the selected area. Color must be formatted
 *              according to the layer pixel format.
 *
 * Return:
 *    OK      - On success
 *   -EINVAL  - If one of the parameter invalid or if the size of the selected
 *              area outside the visible area of the layer.
 *
 ****************************************************************************/

static int stm32_fillarea(FAR struct ltdc_layer_s *layer,
                            FAR const struct ltdc_area_s *area,
                            uint32_t color)
{
  FAR struct stm32_layer_s *priv = (FAR struct stm32_layer_s *)layer;
  gvdbg("layer = %p, area = %p, color = %08x\n", layer, area, color);

  if (stm32_ltdc_lvalidate(priv))
    {
      int   ret;

      sem_wait(priv->state.lock);
      priv->dma2d->fillarea(priv->dma2d, area, color);
      sem_post(priv->state.lock);

      return ret;
    }

  gdbg("Returning EINVAL\n");
  return -EINVAL;
}
#endif

/****************************************************************************
 * Name: stm32_ltdcgetlayer
 *
 * Description:
 *   This is a non-standard framebuffer interface.
 *   Get the specific layer object by the layer id to enable layer hardware
 *   support.
 *
 * Parameter:
 *   lid - Layer identifier
 *
 * Return:
 *   Reference to the layer control structure on success or Null if lid
 *   is invalid.
 *
 ****************************************************************************/

FAR struct ltdc_layer_s *stm32_ltdcgetlayer(int lid)
{
  gvdbg("lid: %d\n", lid);
  if (lid == LTDC_LAYER_L1 || lid == LTDC_LAYER_L2)
    {
      return (FAR struct ltdc_layer_s *) &LAYER(lid);
    }

  gdbg("EINVAL\n");
  errno = EINVAL;
  return NULL;
}
#endif /* CONFIG_STM32_LTDC_INTERFACE */

/****************************************************************************
 * Name: stm32_ltdcinitialize
 *
 * Description:
 *   Initialize the ltdc controller
 *
 * Return:
 *   OK
 *
 ****************************************************************************/

int stm32_ltdcinitialize(void)
{
#ifdef CONFIG_STM32_DMA2D
  int   ret;
#endif

  dbg("Initialize LTDC driver\n");

  if (g_initialized == true)
    {
      return OK;
    }

  /* Disable the LCD */

  stm32_lcd_enable(false);

  gvdbg("Configuring the LCD controller\n");

  /* Configure LCD periphery */

  gvdbg("Configure lcd periphery\n");
  stm32_ltdc_periphconfig();

  /* Configure global ltdc register */

  gvdbg("Configure global register\n");
  stm32_global_configure();

#ifdef CONFIG_STM32_DMA2D
  /* Initialize the dma2d controller */

  ret = up_dma2dinitialize();

  if (ret != OK)
    {
      return ret;
    }
#endif

  /* Initialize ltdc layer */

  gvdbg("Initialize ltdc layer\n");
  stm32_ltdc_linit(LTDC_LAYER_L1);
#ifdef CONFIG_STM32_LTDC_L2
  stm32_ltdc_linit(LTDC_LAYER_L2);
#endif

  /* Display layer 1 and 2 */

  stm32_ltdc_lenable(&LAYER_L1);
#ifdef CONFIG_STM32_LTDC_L2
  stm32_ltdc_lenable(&LAYER_L2);
#endif

  /* Enable the backlight */

#ifdef CONFIG_STM32_LCD_BACKLIGHT
  stm32_backlight(true);
#endif

  /* Reload shadow register */

  gvdbg("Reload shadow register\n");
  stm32_ltdc_reload(LTDC_SRCR_IMR, false);

  /* Turn the LCD on */

  gvdbg("Enabling the display\n");
  stm32_lcd_enable(true);

  /* Set initialized state */

  g_initialized = true;
  return OK;
}

/****************************************************************************
 * Name: stm32_ltdcgetvplane
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
 ****************************************************************************/

struct fb_vtable_s *stm32_ltdcgetvplane(int vplane)
{
  gvdbg("vplane: %d\n", vplane);

  if (vplane == 0)
    {
      return (struct fb_vtable_s *)&g_vtable;
    }

  return NULL;
}

/****************************************************************************
 * Name: fb_uninitialize
 *
 * Description:
 *   Uninitialize the framebuffer driver.  Bad things will happen if you
 *   call this without first calling fb_initialize()!
 *
 ****************************************************************************/

void stm32_ltdcuninitialize(void)
{
  /* Disable all ltdc interrupts */

  stm32_ltdc_irqctrl(0, LTDC_IER_RRIE|LTDC_IER_TERRIE|
                        LTDC_IER_FUIE|LTDC_IER_LIE);

  up_disable_irq(g_interrupt.irq);
  irq_detach(g_interrupt.irq);

  /* Disable the LCD controller */

  stm32_lcd_enable(false);

  /* Set initialized state */

  g_initialized = false;
}

/****************************************************************************
 * Name:  stm32_lcdclear
 *
 * Description:
 *   This is a non-standard LCD interface just for the STM32 LTDC. Clearing the
 *   display in the normal way by writing a sequences of runs that covers the
 *   entire display can be slow.  Here the display is cleared by simply setting
 *   all video memory to the specified color.
 *
 * Parameter:
 *   color - The color the clear the whole framebuffer
 *
 ****************************************************************************/

void stm32_lcdclear(nxgl_mxpixel_t color)
{
#ifdef CONFIG_STM32_LTDC_L2
  stm32_ltdc_lclear(&LAYER(LTDC_LAYER_L2), color);
#endif
  stm32_ltdc_lclear(&LAYER(LTDC_LAYER_L1), color);
}

/****************************************************************************
 * Name: stm32_lcd_backlight
 *
 * Description:
 *   Provide this interface to turn the backlight on and off.
 *
 * Parameter:
 *   blon - Enable or disable the lcd backlight
 *
 ****************************************************************************/

#ifdef CONFIG_STM32_LCD_BACKLIGHT
void stm32_backlight(bool blon)
{
  /* Set default backlight level CONFIG_STM32_LTDC_DEFBACKLIGHT */

  gdbg("Not supported\n");
}
#endif
