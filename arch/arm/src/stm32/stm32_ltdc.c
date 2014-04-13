/****************************************************************************
 * arch/arm/src/stm32/stm32_ltdc.c
 *
 *   Copyright (C) 2013 Ken Pettit. All rights reserved.
 *   Author: Ken Pettit <pettitd@gmail.com>
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

#include <nuttx/video/fb.h>
#include <nuttx/kmalloc.h>

#include <arch/board/board.h>

#include "up_arch.h"
#include "up_internal.h"
#include "stm32.h"
#include "stm32_ltdc.h"

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/
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
#elif defined(CONFIG_STM32_LTDC_L1_AL44)
#  define STM32_LTDC_L1_BPP         8
#  define STM32_LTDC_L1_COLOR_FMT   ???
#elif defined(CONFIG_STM32_LTDC_L1_AL88)
#  define STM32_LTDC_L1_BPP         16
#  define STM32_LTDC_L1_COLOR_FMT   ???
#elif defined(CONFIG_STM32_LTDC_L1_ARGB4444)
#  define STM32_LTDC_L1_BPP         16
#  define STM32_LTDC_L1_COLOR_FMT   ???
#elif defined(CONFIG_STM32_LTDC_L1_RGB565)
#  define STM32_LTDC_L1_BPP         16
#  define STM32_LTDC_L1_COLOR_FMT   FB_FMT_RGB16_565
#elif defined(CONFIG_STM32_LTDC_L1_ARGB1555)
#  define STM32_LTDC_L1_BPP         16
#  define STM32_LTDC_L1_COLOR_FMT   ???
#elif defined(CONFIG_STM32_LTDC_L1_RGB888)
#  define STM32_LTDC_L1_BPP         24
#  define STM32_LTDC_L1_COLOR_FMT   FB_FMT_RGB24
#elif defined(CONFIG_STM32_LTDC_L1_ARGB8888)
#  define STM32_LTDC_L1_BPP         32
#  define STM32_LTDC_L1_COLOR_FMT   ???
#endif

/* Layer 2 format */

#if defined(CONFIG_STM32_LTDC_L2_L8)
#  define STM32_LTDC_L2_BPP         8
#  define STM32_LTDC_L2_COLOR_FMT   FB_FMT_RGB8
#elif defined(CONFIG_STM32_LTDC_L2_AL44)
#  define STM32_LTDC_L2_BPP         8
#  define STM32_LTDC_L2_COLOR_FMT   ???
#elif defined(CONFIG_STM32_LTDC_L2_AL88)
#  define STM32_LTDC_L2_BPP         16
#  define STM32_LTDC_L2_COLOR_FMT   ???
#elif defined(CONFIG_STM32_LTDC_L2_ARGB4444)
#  define STM32_LTDC_L2_BPP         16
#  define STM32_LTDC_L2_COLOR_FMT   ???
#elif defined(CONFIG_STM32_LTDC_L2_RGB565)
#  define STM32_LTDC_L2_BPP         16
#  define STM32_LTDC_L2_COLOR_FMT   FB_FMT_RGB16_565
#elif defined(CONFIG_STM32_LTDC_L2_ARGB1555)
#  define STM32_LTDC_L2_BPP         16
#  define STM32_LTDC_L2_COLOR_FMT   ???
#elif defined(CONFIG_STM32_LTDC_L2_RGB888)
#  define STM32_LTDC_L2_BPP         24
#  define STM32_LTDC_L2_COLOR_FMT   FB_FMT_RGB24
#elif defined(CONFIG_STM32_LTDC_L2_ARGB8888)
#  define STM32_LTDC_L2_BPP         32
#  define STM32_LTDC_L2_COLOR_FMT   ???
#endif

/* Framebuffer sizes in bytes */

#ifndef BOARD_LTDC_WIDTH
#  error BOARD_LTDC_WIDTH must be defined in the board.h header file
#endif

#ifndef BOARD_LTDC_HEIGHT
#  error BOARD_LTDC_HEIGHT must be defined in the board.h header file
#endif

#if STM32_LTDC_L1_BPP == 8
#  define STM32_L1_STRIDE (BOARD_LTDC_WIDTH)
#elif STM32_LTDC_L1_BPP == 16
#  define STM32_L1_STRIDE ((BOARD_LTDC_WIDTH * 16 + 7) / 8)
#elif STM32_LTDC_L1_BPP == 24
#  define STM32_L1_STRIDE ((BOARD_LTDC_WIDTH * 24 + 7) / 8)
#elif STM32_LTDC_L1_BPP == 32
#  define STM32_L1_STRIDE ((BOARD_LTDC_WIDTH * 32 + 7) / 8)
#else
#  error Undefined or unrecognized base resolution
#endif

#define STM32_L1_FBSIZE (STM32_L1_STRIDE * BOARD_LTDC_HEIGHT)

#ifdef CONFIG_STM32_LTDC_L2
#  ifndef CONFIG_STM32_LTDC_L2_WIDTH
#    define CONFIG_STM32_LTDC_L2_WIDTH BOARD_LTDC_WIDTH
#  endif

#  if CONFIG_STM32_LTDC_L2_WIDTH > BOARD_LTDC_WIDTH
#    error Width of Layer 2 exceeds the width of the display
#  endif

#  ifndef CONFIG_STM32_LTDC_L2_HEIGHT
#    define CONFIG_STM32_LTDC_L2_HEIGHT BOARD_LTDC_HEIGHT
#  endif

#  if CONFIG_STM32_LTDC_L2_HEIGHT > BOARD_LTDC_HEIGHT
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

/* Debug */

#ifndef CONFIG_DEBUG
#  undef CONFIG_STM32_LTDC_REGDEBUG
#endif

/* Preallocated LTDC framebuffers */
/* Maybe we need some meta-data for each layer? */

#define SIZEOF_STM32_LAYER_META_S     32
#define STM32_LTDC_LAYER_META_SIZE    (2*SIZEOF_STM32_LAYER_META_S)
#define STM32_LTDC_LAYER_META_END     (CONFIG_STM32_LTDC_FB_BASE+STM32_LTDC_LAYER_META_SIZE)

/* Position the framebuffer memory in the center of the memory set aside.  We
 * will use any skirts before or after the framebuffer memory as a guard against
 * wild framebuffer writes.
 */

#define STM32_LTDC_BUFFER_SIZE  (CONFIG_STM32_LTDC_FB_SIZE-STM32_LTDC_LAYER_META_SIZE)
#define STM32_LTDC_BUFFER_FREE  (STM32_LTDC_BUFFER_SIZE-STM32_TOTAL_FBSIZE)
#define STM32_LTDC_BUFFER_START (STM32_LTDC_LAYER_META_END + STM32_LTDC_BUFFER_FREE/2)

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

#define LTDC_NLAYERS 2

#define LAYER(i)     g_ltdc.layer[i]
#define LAYER_L1     g_ltdc.layer[LTDC_LAYER_L1]
#define LAYER_L2     g_ltdc.layer[LTDC_LAYER_L2]

/****************************************************************************
 * Private Types
 ****************************************************************************/
/* This enumeration names each layer supported by the hardware */

enum stm32_layer_e
{
  LTDC_LAYER_L1 = 0,       /* LCD Layer 1*/
  LTDC_LAYER_L2,           /* LCD Layer 2 */
};

/* LTDC General Layer information */

struct stm32_layer_s
{
  /* Descriptors and buffering */

  uint8_t *framebuffer;    /* DMA framebuffer memory */
  uint8_t lid;             /* Layer ID (see enum stm32_layer_e) */

  /* Window position information */

  uint16_t xpos;           /* Window x position */
  uint16_t width;          /* Window width */
  uint16_t ypos;           /* Window y position */
  uint16_t height;         /* Window width */

  /* Color information */

  uint8_t bpp;             /* Bits per pixel */
};

/* This structure provides the overall state of the LTDC */

struct stm32_ltdc_s
{
  /* Layer information */

  struct stm32_layer_s layer[LTDC_NLAYERS];

  /* Debug stuff */

#ifdef CONFIG_STM32_LtDC_REGDEBUG
   bool wrlast;                   /* True: Last access was a write */
   uintptr_t addrlast;            /* Last address accessed */
   uint32_t vallast;              /* Last value read or written */
   int ntimes;                    /* Number of consecutive accesses */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
/* Register operations ******************************************************/

#if defined(CONFIG_STM32_LTDC_REGDEBUG) && defined(CONFIG_DEBUG)
static bool stm32_checkreg(bool wr, uint32_t regval, uintptr_t address);
static uint32_t stm32_getreg(uintptr_t addr);
static void stm32_putreg(uintptr_t addr, uint32_t val);
#else
#  define stm32_getreg(addr)      getreg32(addr)
#  define stm32_putreg(addr,val)  putreg32(val,addr)
#endif
static void stm32_wait_lcdstatus(uint32_t mask, uint32_t value);

/* Frame buffer interface ***************************************************/
/* Get information about the video controller configuration and the
 * configuration of each color plane.
 */

static int stm32_getvideoinfo(struct fb_vtable_s *vtable,
              struct fb_videoinfo_s *vinfo);
static int stm32_getplaneinfo(struct fb_vtable_s *vtable,
              int planeno, struct fb_planeinfo_s *pinfo);

/* The following is provided only if the video hardware supports RGB color
 * mapping
 */

#ifdef CONFIG_FB_CMAP
static int stm32_getcmap(struct fb_vtable_s *vtable,
              struct fb_cmap_s *cmap);
static int stm32_putcmap(struct fb_vtable_s *vtable,
              const struct fb_cmap_s *cmap);
#endif

/* Initialization ***********************************************************/

#ifdef CONFIG_FB_CMAP
static int stm32_setclut(struct stm32_layer_s *layer,
              const struct fb_cmap_s *cmap);
static int stm32_getclut(struct stm32_layer_s *layer,
              struct fb_cmap_s *cmap);
#endif

static void stm32_backlight(uint32_t level);
static void stm32_l1_disable(void);
static void stm32_l2_disable(void);
static void stm32_lcd_disable(void);
static void stm32_layer_color(void);
static void stm32_lcd_enable(void);
static void stm32_layer_configure(void);
static void stm32_show_layer(struct stm32_layer_s *layer,
              uint32_t dispx, uint32_t dispy, uint32_t dispw, uint32_t disph);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This structure describes the video controller */

static const struct fb_videoinfo_s g_videoinfo =
{
  .fmt      = STM32_LTDC_L1_COLOR_FMT,
  .xres     = BOARD_LTDC_WIDTH,
  .yres     = BOARD_LTDC_HEIGHT,
  .nplanes  = 1,
};

/* This structure provides the overall state of the LTDC */

static struct stm32_ltdc_s g_ltdc;

/* This structure provides the base layer interface */

static const struct fb_vtable_s g_vtable =
{
  .getvideoinfo  = stm32_getvideoinfo,
  .getplaneinfo  = stm32_getplaneinfo,
#ifdef CONFIG_FB_CMAP
  .getcmap       = stm32_getcmap,
  .putcmap       = stm32_putcmap,
#endif
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_checkreg
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

#ifdef CONFIG_STM32_LTDC_REGDEBUG
static bool stm32_checkreg(bool wr, uint32_t regval, uintptr_t address)
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
 * Name: stm32_getreg
 *
 * Description:
 *  Read any 32-bit register using an absolute
 *
 ****************************************************************************/

#ifdef CONFIG_STM32_LTDC_REGDEBUG
static uint32_t stm32_getreg(uintptr_t address)
{
  uint32_t regval = getreg32(address);

  if (stm_checkreg(false, regval, address))
    {
      lldbg("%08x->%08x\n", address, regval);
    }

  return regval;
}
#endif

/****************************************************************************
 * Name: stm32_putreg
 *
 * Description:
 *  Write to any 32-bit register using an absolute address
 *
 ****************************************************************************/

#ifdef CONFIG_STM32_LTDC_REGDEBUG
static void stm32_putreg(uintptr_t address, uint32_t regval)
{
  if (stm_checkreg(true, regval, address))
    {
      lldbg("%08x<-%08x\n", address, regval);
    }

  putreg32(regval, address);
}
#endif

/****************************************************************************
 * Name: stm32_wait_lcdstatus
 *
 * Description:
 *   Wait for the masked set of bits in the LTDC status register to take a
 *   specific value.
 *
 ****************************************************************************/

static void stm32_wait_lcdstatus(uint32_t mask, uint32_t value)
{
}

/****************************************************************************
 * Name: stm32_getvideoinfo
 ****************************************************************************/

static int stm32_getvideoinfo(struct fb_vtable_s *vtable,
                                 struct fb_videoinfo_s *vinfo)
{
  gvdbg("vtable=%p vinfo=%p\n", vtable, vinfo);
  if (vtable && vinfo)
    {
      memcpy(vinfo, &g_videoinfo, sizeof(struct fb_videoinfo_s));
      return OK;
    }

  gdbg("ERROR: Returning EINVAL\n");
  return -EINVAL;
}

/****************************************************************************
 * Name: stm32_getplaneinfo
 ****************************************************************************/

static int stm32_getplaneinfo(struct fb_vtable_s *vtable, int planeno,
                                 struct fb_planeinfo_s *pinfo)
{
  gvdbg("vtable=%p planeno=%d pinfo=%p\n", vtable, planeno, pinfo);
  if (vtable && planeno == 0 && pinfo)
    {
      pinfo->fbmem  = (void *)LAYER_L1.framebuffer;
      pinfo->fblen  = STM32_L1_FBSIZE;
      pinfo->stride = STM32_L1_STRIDE,
      pinfo->bpp    = LAYER_L1.bpp;
      return OK;
    }

  gdbg("Returning EINVAL\n");
  return -EINVAL;
}

/****************************************************************************
 * Name: stm32_base_getcmap
 ****************************************************************************/

#ifdef CONFIG_FB_CMAP
static int stm32_getcmap(struct fb_vtable_s *vtable,
                            struct fb_cmap_s *cmap)
{
  return stm_getclut(&LAYER_L1, cmap);
}
#endif

/****************************************************************************
 * Name: stm_base_putcmap
 ****************************************************************************/

#ifdef CONFIG_FB_CMAP
static int stm32_putcmap(struct fb_vtable_s *vtable,
                            const struct fb_cmap_s *cmap)
{
  return stm32_setclut(&LAYER_L1, cmap);
}
#endif

/****************************************************************************
 * Name: stm32_setposition
 *
 * Description:
 *   Set the new position of a move-able layer (any layer except the base
 *   layer).
 *
 ****************************************************************************/

static void stm32_setposition(int lid, uint32_t x, uint32_t y)
{
}

/****************************************************************************
 * Name: stm32_setclut
 *
 * Description:
 *   Set a range of CLUT values for any layer
 *
 ****************************************************************************/

#ifdef CONFIG_FB_CMAP
static int stm32_setclut(struct stm32_layer_s *layer,
                       const struct fb_cmap_s *cmap)
{

  return OK;
}
#endif

/****************************************************************************
 * Name: stm32_getclut
 *
 * Description:
 *   Get a range of CLUT values for any layer
 *
 ****************************************************************************/

#ifdef CONFIG_FB_CMAP
static int stm32_getclut(struct stm_layer_s *layer,
                       struct fb_cmap_s *cmap)
{

  return OK;
}
#endif

/****************************************************************************
 * Name: stm32_backlight
 *
 * Description:
 *   Set the backlight level
 *
 ****************************************************************************/

static void stm32_backlight(uint32_t level)
{
}

/****************************************************************************
 * Name: stm32_l1_disable
 *
 * Description:
 *   Disable Layer 1
 *
 ****************************************************************************/

static void stm32_l1_disable(void)
{
}

/****************************************************************************
 * Name: stm32_l2_disable
 *
 * Description:
 *   Disable Layer 2
 *
 ****************************************************************************/

static void stm32_l2_disable(void)
{
}

/****************************************************************************
 * Name: stm32_lcd_disable
 *
 * Description:
 *   Disable the LCD peripheral
 *
 ****************************************************************************/

static void stm32_lcd_disable(void)
{
  /* Disable layers */

  stm32_l1_disable();
  stm32_l2_disable();

}

/****************************************************************************
 * Name: stm32_layer_position
 *
 * Description:
 *   Configure LTDC layer position
 *
 ****************************************************************************/

static void stm32_layer_position(void)
{
}

/****************************************************************************
 * Name: stm32_layer_color
 *
 * Description:
 *   Configure LTDC layer color mode
 *
 ****************************************************************************/

static void stm32_layer_color(void)
{
}

/****************************************************************************
 * Name: stm32_lcd_enable
 *
 * Description:
 *   Enable the LCD for normal use
 *
 ****************************************************************************/

static void stm32_lcd_enable(void)
{
}

/****************************************************************************
 * Name: stm32_layer_configure
 *
 * Description:
 *   Configure layer layer structures and framebuffers
 *
 ****************************************************************************/

static void stm32_layer_configure(void)
{
  /* Common layer initialization */

  memset(&LAYER_L1, 0, sizeof(struct stm32_layer_s));
  LAYER_L1.lid           = LTDC_LAYER_L1;
  LAYER_L1.framebuffer   = (uint8_t *)STM32_LTDC_BUFFER_L1;

  memset(&LAYER_L2, 0, sizeof(struct stm32_layer_s));
  LAYER_L2.lid           = LTDC_LAYER_L2;
#ifdef CONFIG_STM32_LTDC_L2
  LAYER_L2.framebuffer   = (uint8_t *)STM32_LTDC_BUFFER_L2;
#endif
}

/****************************************************************************
 * Name: stm32_ltdc_periphconfig
 *
 * Description:
 *   Configures the LTDC peripheral clock and SAI PLL clocks to drive the
 *   LCD.
 *
 ****************************************************************************/

static void stm32_config_lcd_clock(void)
{
  uint32_t    regval;

  /* Configure the SAI PLL to provide the LCD_CLK */

}

/****************************************************************************
 * Name: stm32_show_layer
 *
 * Description:
 *   Show the given layer with the specified window
 *
 ****************************************************************************/

static void stm32_show_layer(struct stm32_layer_s *layer,
                           uint32_t dispx, uint32_t dispy,
                           uint32_t dispw, uint32_t disph)
{
}

/****************************************************************************
 * Name: stm32_show_l1
 *
 * Description:
 *   Show Layer 1 (the "base" layer)
 *
 ****************************************************************************/

static void stm32_show_l1(void)
{
  stm32_show_layer(&LAYER_L1, 0, 0, BOARD_LTDC_WIDTH, BOARD_LTDC_HEIGHT);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_fbinitialize
 *
 * Description:
 *   Initialize the framebuffer video hardware
 *
 ****************************************************************************/

int up_fbinitialize(void)
{
  gvdbg("Entry\n");

  /* Configure layer layer structures, DMA descriptor memory, and
   * framebuffers
   */

  stm32_layer_configure();

  /* Disable the LCD */

  stm32_lcd_disable();

  gvdbg("Configuring the LCD controller\n");

  /* Configure and Enable the LCD clock */

  /* Disable LCD interrupts */

  /* Configure layer positions */

  stm32_layer_position();

  /* Configure layer colors */

  stm32_layer_color();

  /* Clear the display memory */

  stm32_lcdclear(CONFIG_STM32_LTDC_BACKCOLOR);

  /* And turn the LCD on */

  gvdbg("Enabling the display\n");
  stm32_lcd_enable();

  /* Display layer 1 */

  stm32_show_l1();

  /* Enable the backlight.
   *
   * REVISIT:  Backlight level could be dynamically adjustable
   */

  stm32_backlight(CONFIG_STM32_LTDC_DEFBACKLIGHT);

  return OK;
}

/****************************************************************************
 * Name: stm32_fbgetvplane
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

void fb_uninitialize(void)
{
  /* Disable the LCD controller */

  stm32_lcd_disable();
}

/************************************************************************************
 * Name:  stm32_lcdclear
 *
 * Description:
 *   This is a non-standard LCD interface just for the STM32.  Clearing the display
 *   in the normal way by writing a sequences of runs that covers the entire display
 *   can be slow.  Here the display is cleared by simply setting all video memory to
 *   the specified color.
 *
 ************************************************************************************/

void stm32_lcdclear(nxgl_mxpixel_t color)
{
}
