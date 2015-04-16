/******************************************************************************
 * arch/arm/src/stm32/stm32_dma2d.c
 *
 *   Copyright (C) 2014-2015 Marco Krahl. All rights reserved.
 *   Author: Marco Krahl <ocram.lhark@gmail.com>
 *
 * References:
 *   STM32F429 Technical Reference Manual
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
 ******************************************************************************/

/******************************************************************************
 * Included Files
 ******************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <debug.h>
#include <semaphore.h>

#include <nuttx/irq.h>
#include <nuttx/video/fb.h>
#include <nuttx/kmalloc.h>

#include <arch/chip/dma2d.h>
#include <arch/board/board.h>

#include "up_arch.h"
#include "up_internal.h"
#include "stm32.h"
#include "chip/stm32_ltdc.h"
#include "chip/stm32_dma2d.h"
#include "chip/stm32_ccm.h"
#include "stm32_dma2d.h"

/******************************************************************************
 * Pre-processor Definitions
 ******************************************************************************/

/* output, foreground and background layer */

#define DMA2D_NLAYERS                   3

/* DMA2D PFC value definitions */

#define DMA2D_PF_ARGB8888               0
#define DMA2D_PF_RGB888                 1
#define DMA2D_PF_RGB565                 2
#define DMA2D_PF_ARGB1555               3
#define DMA2D_PF_ARGB14444              4
#define DMA2D_PF_L8                     5
#define DMA2D_PF_AL44                   6
#define DMA2D_PF_AL88                   7
#define DMA2D_PF_L4                     8
#define DMA2D_PF_A8                     9
#define DMA2D_PF_A4                     10

/* DMA2D blender control */

#define STM32_DMA2D_CR_MODE_BLIT        DMA2D_CR_MODE(0)
#define STM32_DMA2D_CR_MODE_BLITPFC     DMA2D_CR_MODE(1)
#define STM32_DMA2D_CR_MODE_BLEND       DMA2D_CR_MODE(2)
#define STM32_DMA2D_CR_MODE_COLOR       DMA2D_CR_MODE(3)
#define STM32_DMA2D_CR_MODE_CLEAR       STM32_DMA2D_CR_MODE_COLOR

/* DMA2D PFC alpha mode */

#define STM32_DMA2D_PFCCR_AM_NONE       0
#define STM32_DMA2D_PFCCR_AM_CONST      1
#define STM32_DMA2D_PFCCR_AM_PIXEL      10

/* Only 8 bit per pixel overal supported */

#define DMA2D_PF_BYPP(n)                ((n) / 8)

#define DMA2D_CLUT_SIZE                 STM32_LTDC_NCLUT - 1

/* Layer argb cmap conversion */

#define DMA2D_CLUT_ALPHA(n)          ((uint32_t)(n) << 24)
#define DMA2D_CLUT_RED(n)            ((uint32_t)(n) << 16)
#define DMA2D_CLUT_GREEN(n)          ((uint32_t)(n) << 8)
#define DMA2D_CLUT_BLUE(n)           ((uint32_t)(n) << 0)

#define DMA2D_CMAP_ALPHA(n)          ((uint32_t)(n) >> 24)
#define DMA2D_CMAP_RED(n)            ((uint32_t)(n) >> 16)
#define DMA2D_CMAP_GREEN(n)          ((uint32_t)(n) >> 8)
#define DMA2D_CMAP_BLUE(n)           ((uint32_t)(n) >> 0)

/* Define shadow layer for ltdc interface */

#ifdef CONFIG_STM32_LTDC_INTERFACE
# ifdef CONFIG_STM32_LTDC_L2
#  define DMA2D_SHADOW_LAYER    2
#  define DMA2D_SHADOW_LAYER_L1 0
#  define DMA2D_SHADOW_LAYER_L2 1
# else
#  define DMA2D_SHADOW_LAYER    1
#  define DMA2D_SHADOW_LAYER_L1 0
# endif
# define DMA2D_LAYER_NSIZE      CONFIG_STM32_DMA2D_NLAYERS + DMA2D_SHADOW_LAYER
#else
# define DMA2D_LAYER_NSIZE      CONFIG_STM32_DMA2D_NLAYERS
# define DMA2D_SHADOW_LAYER     0
#endif

/* Debug option */

#ifdef CONFIG_STM32_DMA2D_REGDEBUG
#  define regdbg       dbg
#  define regvdbg      vdbg
#else
#  define regdbg(x...)
#  define regvdbg(x...)
#endif

/* check clut support */

#ifdef CONFIG_STM32_DMA2D_L8
# ifndef CONFIG_FB_CMAP
#  error "Enable cmap to support the configured layer formats!"
# endif
#endif

/* check ccm heap allocation */

#ifndef CONFIG_STM32_CCMEXCLUDE
# error "Enable CONFIG_STM32_CCMEXCLUDE from the heap allocation"
#endif

/******************************************************************************
 * Private Types
 ******************************************************************************/

/* DMA2D General layer information */

struct stm32_dma2d_s
{
  struct dma2d_layer_s dma2d;   /* public dma2d interface */

  /* Fixed settings */

  int lid;                      /* Layer identifier */
  struct fb_videoinfo_s vinfo;  /* Layer videoinfo */
  struct fb_planeinfo_s pinfo;  /* Layer planeinfo */

  /* Blending */

  uint32_t blendmode;           /* the interface blendmode */
  uint8_t  alpha;               /* the alpha value */

  /* Coloring */

#ifdef CONFIG_STM32_DMA2D_L8
  uint32_t *clut;               /* Color lookup table */
#endif

  /* Operation */
  uint8_t  fmt;                 /* the controller pixel format */
  sem_t    *lock;               /* Ensure mutually exclusive access */
};

#ifdef CONFIG_STM32_LTDC_INTERFACE

/* This structures provides the DMA2D layer for each LTDC layer */

struct stm32_ltdc_dma2d_s
{
  struct stm32_dma2d_s dma2ddev;
#ifdef CONFIG_STM32_DMA2D_L8
  FAR struct ltdc_layer_s *ltdc;
#endif
};

struct stm32_ltdc_layer_s
{
  /* Layer state */

  struct stm32_ltdc_dma2d_s layer[DMA2D_SHADOW_LAYER];
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

/* This enumeration foreground and background layer supported by the dma2d
 * controller
 */

enum stm32_layer_e
{
  DMA2D_LAYER_LFORE = 0,       /* Foreground Layer */
  DMA2D_LAYER_LBACK,           /* Background Layer */
  DMA2D_LAYER_LOUT,            /* Output Layer */
};

/* DMA2D memory address register */

static const uintptr_t stm32_mar_layer_t[DMA2D_NLAYERS] =
{
  STM32_DMA2D_FGMAR,
  STM32_DMA2D_BGMAR,
  STM32_DMA2D_OMAR
};

/* DMA2D offset register */

static const uintptr_t stm32_or_layer_t[DMA2D_NLAYERS] =
{
  STM32_DMA2D_FGOR,
  STM32_DMA2D_BGOR,
  STM32_DMA2D_OOR
};

/* DMA2D pfc control register */

static const uintptr_t stm32_pfccr_layer_t[DMA2D_NLAYERS] =
{
  STM32_DMA2D_FGPFCCR,
  STM32_DMA2D_BGPFCCR,
  STM32_DMA2D_OPFCCR
};

/* DMA2D color register */

static const uintptr_t stm32_color_layer_t[DMA2D_NLAYERS] =
{
  STM32_DMA2D_FGCOLR,
  STM32_DMA2D_BGCOLR,
  STM32_DMA2D_OCOLR
};

/* DMA2D clut memory address register */

static const uintptr_t stm32_cmar_layer_t[DMA2D_NLAYERS - 1] =
{
  STM32_DMA2D_FGCMAR,
  STM32_DMA2D_BGCMAR
};

/******************************************************************************
 * Private Function Prototypes
 ******************************************************************************/

/* Private functions */

static int stm32_dma2d_pixelformat(uint8_t fmt, uint8_t *fmtmap);
static int stm32_dma2d_bpp(uint8_t fmt, uint8_t *bpp);
static void stm32_dma2d_control(uint32_t setbits, uint32_t clrbits);
static int stm32_dma2dirq(int irq, void *context);
static int stm32_dma2d_waitforirq(void);
static int stm32_dma2d_start(void);
#ifdef CONFIG_STM32_DMA2D_L8
static int stm32_dma2d_loadclut(uintptr_t reg);
#endif
static uint32_t stm32_dma2d_memaddress(FAR const struct stm32_dma2d_s *layer,
                                        fb_coord_t xpos, fb_coord_t ypos);
static fb_coord_t stm32_dma2d_lineoffset(FAR const struct stm32_dma2d_s *layer,
                                          FAR const struct ltdc_area_s *area);

static int stm32_dma2d_lfreelid(void);
static FAR struct stm32_dma2d_s * stm32_dma2d_lalloc(void);
static void stm32_dma2d_lfree(FAR struct stm32_dma2d_s *layer);
static void stm32_dma2d_llayerscleanup(void);
static bool stm32_dma2d_lvalidate(FAR const struct stm32_dma2d_s *layer);
static bool stm32_dma2d_lvalidatesize(FAR const struct stm32_dma2d_s *layer,
                                      fb_coord_t xpos, fb_coord_t ypos,
                                      FAR const struct ltdc_area_s *area);
static void stm32_dma2d_linit(FAR struct stm32_dma2d_s *layer,
                                int lid, uint8_t fmt);

static void stm32_dma2d_lfifo(FAR const struct stm32_dma2d_s *layer, int lid,
                              fb_coord_t xpos, fb_coord_t ypos,
                              FAR const struct ltdc_area_s *area);
static void stm32_dma2d_lcolor(FAR const struct stm32_dma2d_s *layer,
                               int lid, uint32_t color);
static void stm32_dma2d_llnr(FAR struct stm32_dma2d_s *layer,
                                FAR const struct ltdc_area_s *area);
static int stm32_dma2d_loutpfc(FAR const struct stm32_dma2d_s *layer);
static void stm32_dma2d_lpfc(FAR const struct stm32_dma2d_s *layer,
                              int lid, uint32_t blendmode);
/* Public functions */

static int stm32_dma2dgetvideoinfo(FAR struct dma2d_layer_s *layer,
                                  FAR struct fb_videoinfo_s *vinfo);
static int stm32_dma2dgetplaneinfo(FAR struct dma2d_layer_s *layer, int planeno,
                                  FAR struct fb_planeinfo_s *pinfo);
static int stm32_dma2dgetlid(FAR struct dma2d_layer_s *layer, int *lid);
#ifdef CONFIG_STM32_DMA2D_L8
static int stm32_dma2dsetclut(FAR struct dma2d_layer_s *layer,
                            const FAR struct fb_cmap_s *cmap);
static int stm32_dma2dgetclut(FAR struct dma2d_layer_s *layer,
                            FAR struct fb_cmap_s *cmap);
#endif
static int stm32_dma2dsetalpha(FAR struct dma2d_layer_s *layer, uint8_t alpha);
static int stm32_dma2dgetalpha(FAR struct dma2d_layer_s *layer, uint8_t *alpha);
static int stm32_dma2dsetblendmode(FAR struct dma2d_layer_s *layer,
                                    uint32_t mode);
static int stm32_dma2dgetblendmode(FAR struct dma2d_layer_s *layer,
                                    uint32_t *mode);
static int stm32_dma2dblit(FAR struct dma2d_layer_s *dest,
                            fb_coord_t destxpos, fb_coord_t destypos,
                            FAR const struct dma2d_layer_s *src,
                            FAR const struct ltdc_area_s *srcarea);
static int stm32_dma2dblend(FAR struct dma2d_layer_s *dest,
                            fb_coord_t destxpos, fb_coord_t destypos,
                            FAR const struct dma2d_layer_s *fore,
                            fb_coord_t forexpos, fb_coord_t foreypos,
                            FAR const struct dma2d_layer_s *back,
                            FAR const struct ltdc_area_s *backarea);
static int stm32_dma2dfillarea(FAR struct dma2d_layer_s *layer,
                            FAR const struct ltdc_area_s *area, uint32_t color);

/******************************************************************************
 * Private Data
 ******************************************************************************/

/* Remember the layer references for alloc/deallocation */

static struct stm32_dma2d_s *g_layers[DMA2D_LAYER_NSIZE];

/* The DMA2D semaphore that enforces mutually exclusive access */

static sem_t g_lock;

#ifdef CONFIG_STM32_LTDC_INTERFACE
/* This structure provides the DMA2D layer for each LTDC layer */

static struct stm32_ltdc_layer_s g_ltdc_layer;
#endif

/* The initalized state of the driver */

static bool g_initialized;

/* Semaphore for interrupt handling */

static sem_t g_semirq;

/* This structure provides irq handling */

static struct stm32_interrupt_s g_interrupt =
{
  .wait    = false,
  .handled = true,
  .irq     = STM32_IRQ_DMA2D,
  .sem     = &g_semirq
};

/******************************************************************************
 * Public Data
 ******************************************************************************/

/******************************************************************************
 * Private Functions
 ******************************************************************************/

/******************************************************************************
 * Name: stm32_dma2d_control
 *
 * Description:
 *   Change the DMA2D control register
 *
 * Parameter:
 *   setbits - The bits to set
 *   clrbits - The bits to clear
 *
 ****************************************************************************/

static void stm32_dma2d_control(uint32_t setbits, uint32_t clrbits)
{
  uint32_t   cr;

  gvdbg("setbits=%08x, clrbits=%08x\n", setbits, clrbits);

  cr = getreg32(STM32_DMA2D_CR);
  cr &= ~clrbits;
  cr |= setbits;
  putreg32(cr, STM32_DMA2D_CR);
}

/****************************************************************************
 * Name: stm32_dma2dirq
 *
 * Description:
 *   DMA2D interrupt handler
 *
 ****************************************************************************/

static int stm32_dma2dirq(int irq, void *context)
{
  uint32_t regval = getreg32(STM32_DMA2D_ISR);
  FAR struct stm32_interrupt_s *priv = &g_interrupt;

  regvdbg("irq = %d, regval = %08x\n", irq, regval);

  if (regval & DMA2D_ISR_TCIF)
    {
      /* Transfer complete interrupt */

      /* Clear the interrupt status register */

      putreg32(DMA2D_IFCR_CTCIF, STM32_DMA2D_IFCR);
    }
#ifdef CONFIG_STM32_DMA2D_L8
  else if (regval & DMA2D_ISR_CTCIF)
    {
      /* CLUT transfer complete interrupt */

      /* Clear the interrupt status register */

      putreg32(DMA2D_IFCR_CCTCIF, STM32_DMA2D_IFCR);
    }
#endif
  else
    {
      /* Unknown irq, should not occur */

      return OK;
    }

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

  return OK;
}

/******************************************************************************
 * Name: stm32_dma2d_waitforirq
 *
 * Description:
 *   Helper waits until the dma2d irq occurs. That means that an ongoing clut
 *   loading or dma transfer was completed.
 *   Note! The caller must use this function within irqsave state.
 *
 * Return:
 *   On success OK otherwise ERROR
 *
 ****************************************************************************/

static int stm32_dma2d_waitforirq(void)
{
  FAR struct stm32_interrupt_s *priv = &g_interrupt;

  /* Only waits if last enabled interrupt is currently not handled */

  if (!priv->handled)
    {
      int ret;

      /* Inform the irq handler the task is able to wait for the irq */

      priv->wait = true;

      ret = sem_wait(priv->sem);

      /* irq or an error occurs, reset the wait flag */

      priv->wait = false;

      if (ret != OK)
        {
          dbg("sem_wait() failed\n");
          return ret;
        }
    }

  return OK;
}


#ifdef CONFIG_STM32_DMA2D_L8
/******************************************************************************
 * Name: stm32_dma2d_loadclut
 *
 * Description:
 *   Starts clut loading but doesn't wait until loading is complete!
 *
 * Parameter:
 *   pfcreg - PFC control Register
 *
 * Return:
 *   On success - OK
 *   On error - -EINVAL
 *
 ****************************************************************************/

static int stm32_dma2d_loadclut(uintptr_t pfcreg)
{
  int        ret;
  uint32_t   regval;
  irqstate_t flags;

  flags = irqsave();

  ret = stm32_dma2d_waitforirq();
  if (ret == OK)
    {
      FAR struct stm32_interrupt_s *priv = &g_interrupt;

      /* Reset the handled flag */

      priv->handled = false;

      /* Start clut loading */

      regval  = getreg32(pfcreg);
      regval |= DMA2D_xGPFCCR_START;
      regvdbg("set regval=%08x\n", regval);
      putreg32(regval, pfcreg);
      regvdbg("configured regval=%08x\n", getreg32(pfcreg));
    }

  irqrestore(flags);
  return OK;
}
#endif

/******************************************************************************
 * Name: stm32_dma2d_start
 *
 * Description:
 *   Starts the dma transfer and waits until completed.
 *
 * Parameter:
 *   reg       - Register to set the start
 *   startflag - The related flag to start the dma transfer
 *   irqflag   - The interrupt enable flag in the DMA2D_CR register
 *
 ****************************************************************************/

static int stm32_dma2d_start(void)
{
  int        ret;
  irqstate_t flags;

  flags = irqsave();

  ret = stm32_dma2d_waitforirq();
  if (ret == OK)
    {
      FAR struct stm32_interrupt_s *priv = &g_interrupt;

      /* Reset the handled flag */

      priv->handled = false;

      /* Start clut loading */

      stm32_dma2d_control(DMA2D_CR_START, 0);

      /* wait until transfer is complete */

      ret = stm32_dma2d_waitforirq();
    }

  irqrestore(flags);
  return ret;
}

/******************************************************************************
 * Name: stm32_dma2d_memaddress
 *
 * Description:
 *   Helper to calculate the layer memory address
 *
 * Parameter:
 *   layer - Reference to the common layer state structure
 *
 * Return:
 *   memory address
 *
 *****************************************************************************/

static uint32_t stm32_dma2d_memaddress(FAR const struct stm32_dma2d_s *layer,
                                        fb_coord_t xpos, fb_coord_t ypos)
{
  FAR const struct fb_planeinfo_s *pinfo = &layer->pinfo;
  uint32_t offset;

  offset = xpos * DMA2D_PF_BYPP(layer->pinfo.bpp) + layer->pinfo.stride * ypos;

  gvdbg("%p\n", ((uint32_t) pinfo->fbmem) + offset);
  return ((uint32_t) pinfo->fbmem) + offset;
}

/******************************************************************************
 * Name: stm32_dma2d_lineoffset
 *
 * Description:
 *   Helper to calculate the layer line offset
 *
 * Parameter:
 *   layer - Reference to the common layer state structure
 *
 * Return:
 *   line offset
 *
 *****************************************************************************/

static fb_coord_t stm32_dma2d_lineoffset(FAR const struct stm32_dma2d_s *layer,
                                          FAR const struct ltdc_area_s *area)
{
  /* offset at the end of each line in the context to the area layer */

  gvdbg("%d\n", layer->vinfo.xres - area->xres);
  return layer->vinfo.xres - area->xres;
}

/******************************************************************************
 * Name: stm32_dma2d_pixelformat
 *
 * Description:
 *   Helper to map to dma2d controller pixel format
 *
 * Parameter:
 *   layer - Reference to the common layer state structure
 *   fmt   - Reference to the location to store the pixel format
 *
 * Return:
 *   On success - OK
 *   On error   - -EINVAL
 *
 *****************************************************************************/

static int stm32_dma2d_pixelformat(uint8_t fmt, uint8_t *fmtmap)
{
  gvdbg("fmt=%d, fmtmap=%p\n", fmt, fmtmap);

  /* Map to the controller known format
   *
   * Not supported by NuttX:
   * ARGB8888
   * ARGB1555
   * ARGB4444
   * AL44
   * AL88
   * L8 (non output layer only)
   * L4
   * A8
   * A4
   */

  switch(fmt)
    {
#ifdef CONFIG_STM32_DMA2D_RGB565
      case FB_FMT_RGB16_565:
        *fmtmap = DMA2D_PF_RGB565;
        break;
#endif
#ifdef CONFIG_STM32_DMA2D_RGB888
      case FB_FMT_RGB24:
        *fmtmap = DMA2D_PF_RGB888;
        break;
#endif
#ifdef CONFIG_STM32_DMA2D_L8
      case FB_FMT_RGB8:
        *fmtmap = DMA2D_PF_L8;
        break;
#endif
      default:
        gdbg("ERROR: Returning EINVAL\n");
        return -EINVAL;
    }

  return OK;
}

/******************************************************************************
 * Name: stm32_dma2d_bpp
 *
 * Description:
 *   Helper to get the bits per pixel
 *
 * Parameter:
 *   layer - Reference to the common layer state structure
 *   bpp   - Reference to the location to store the pixel format
 *
 * Return:
 *   On success - OK
 *   On error   - -EINVAL
 *
 *****************************************************************************/

static int stm32_dma2d_bpp(uint8_t fmt, uint8_t *bpp)
{
  gvdbg("fmt=%d, bpp=%p\n", fmt, bpp);

  switch(fmt)
    {
#ifdef CONFIG_STM32_DMA2D_RGB565
      case FB_FMT_RGB16_565:
        *bpp = 16;
        break;
#endif
#ifdef CONFIG_STM32_DMA2D_RGB888
      case FB_FMT_RGB24:
        *bpp = 24;
        break;
#endif
#ifdef CONFIG_STM32_DMA2D_L8
      case FB_FMT_RGB8:
        *bpp = 8;
        break;
#endif
      default:
        gdbg("ERROR: Returning EINVAL\n");
        return -EINVAL;
    }

  return OK;
}

/******************************************************************************
 * Name: stm32_dma2d_lfreelid
 *
 * Description:
 *   Get a free layer id
 *
 * Return:
 *   The number of the free layer
 *   -1 if no free layer is available
 *
 ******************************************************************************/

static int stm32_dma2d_lfreelid(void)
{
  int n;

  for (n = DMA2D_SHADOW_LAYER; n < DMA2D_LAYER_NSIZE; n++)
    {
      if (g_layers[n] == NULL)
        {
          return n;
        }
    }

  return -1;
}

/******************************************************************************
 * Name: stm32_dma2d_lalloc
 *
 * Description:
 *   Allocate a new layer structure
 *
 * Return:
 *   A new allocated layer structure or NULL on error.
 *
 ******************************************************************************/

static FAR struct stm32_dma2d_s * stm32_dma2d_lalloc(void)
{
  FAR struct stm32_dma2d_s *layer;

#ifdef HAVE_CCM_HEAP
  /* First try to allocate from the ccm heap */

  layer = ccm_malloc(sizeof(struct stm32_dma2d_s));

  if (!layer)
    {
      /* Use default allocator */

      layer = kmm_malloc(sizeof(struct stm32_dma2d_s));
    }
#else
  layer = kmm_malloc(sizeof(struct stm32_dma2d_s));
#endif

  return layer;
}

/******************************************************************************
 * Name: stm32_dma2d_lfree
 *
 * Description:
 *   Deallocate the dynamic allocated layer structure
 *
 * Input Parameters:
 *   A previous allocated layer structure
 *
 ******************************************************************************/

static void stm32_dma2d_lfree(FAR struct stm32_dma2d_s *layer)
{
  if (layer)
    {
#ifdef HAVE_CCM_HEAP
      if (((uint32_t)layer & 0xf0000000) == 0x10000000)
        {
          ccm_free(layer);
        }
      else
        {
          kmm_free(layer);
        }
#else
      kmm_free(layer);
#endif
    }
}

/******************************************************************************
 * Name: stm32_dma2d_llayerscleanup
 *
 * Description:
 *   Cleanup all allocated layers
 *
 ******************************************************************************/

static void stm32_dma2d_llayerscleanup(void)
{
  int n;

  /* Do not uninitialize the ltdc related dma2d layer */

  for (n = DMA2D_SHADOW_LAYER; n < DMA2D_LAYER_NSIZE; n++)
    {
      FAR struct stm32_dma2d_s *priv = g_layers[n];
      if (priv)
        {
          kmm_free(priv->pinfo.fbmem);
          stm32_dma2d_lfree(priv);
          g_layers[n] = NULL;
        }
    }
}

/******************************************************************************
 * Name: stm32_dma2d_lvalidate
 *
 * Description:
 *   Helper to validate if the layer is valid
 *
 * Return:
 *   true if validates otherwise false
 *
 ******************************************************************************/

static inline bool stm32_dma2d_lvalidate(FAR const struct stm32_dma2d_s *layer)
{
  return layer && layer->lid < DMA2D_LAYER_NSIZE;
}

/****************************************************************************
 * Name: stm32_dma2d_lvalidatesize
 *
 * Description:
 *   Helper to check if area is outside the whole layer.
 *
 * Parameter:
 *   layer   - Reference to the layer control structure
 *   xpos    - The x position inside the whole layer
 *   ypos    - The y position inside the whole layer
 *   area    - the area inside the whole layer
 *
 * Return:
 *   true if area is inside the whole layer otherwise false
 *
 ****************************************************************************/

static bool stm32_dma2d_lvalidatesize(FAR const struct stm32_dma2d_s *layer,
                                         fb_coord_t xpos, fb_coord_t ypos,
                                         FAR const struct ltdc_area_s *area)
{
  return stm32_dma2d_lvalidate(layer) &&
            ((layer->vinfo.xres - xpos) * (layer->vinfo.yres - ypos) >=
                area->xres * area->yres);
}

/******************************************************************************
 * Name: stm32_dma2d_linit
 *
 * Description:
 *   Initialize the internal layer structure
 *
 * Parameter:
 *
 *
 ******************************************************************************/

static void stm32_dma2d_linit(FAR struct stm32_dma2d_s *layer,
                                int lid, uint8_t fmt)
{
  FAR struct dma2d_layer_s *priv = &layer->dma2d;

  gvdbg("layer=%p, lid=%d, fmt=%02x\n", layer, lid, fmt);

  /* initialize the layer interface */

  priv->getvideoinfo = stm32_dma2dgetvideoinfo;
  priv->getplaneinfo = stm32_dma2dgetplaneinfo;
  priv->getlid       = stm32_dma2dgetlid;
#ifdef CONFIG_STM32_DMA2D_L8
  priv->setclut      = stm32_dma2dsetclut;
  priv->getclut      = stm32_dma2dgetclut;
#endif
  priv->setalpha     = stm32_dma2dsetalpha;
  priv->getalpha     = stm32_dma2dgetalpha;
  priv->setblendmode = stm32_dma2dsetblendmode;
  priv->getblendmode = stm32_dma2dgetblendmode;
  priv->blit         = stm32_dma2dblit;
  priv->blend        = stm32_dma2dblend;
  priv->fillarea     = stm32_dma2dfillarea;

  /* Initialize the layer structure */

  layer->lid          = lid;
#ifdef CONFIG_STM32_DMA2D_L8
  layer->clut         = 0;
#endif
  layer->blendmode    = DMA2D_BLEND_NONE;
  layer->alpha        = 255;
  layer->fmt          = fmt;
  layer->lock         = &g_lock;
}

/******************************************************************************
 * Name: stm32_dma2d_lfifo
 *
 * Description:
 *   Set the fifo for the foreground, background and output layer
 *   Configures the memory address register
 *   Configures the line offset register
 *
 * Parameter:
 *   layer - Reference to the common layer state structure
 *
 ****************************************************************************/

static void stm32_dma2d_lfifo(FAR const struct stm32_dma2d_s *layer, int lid,
                              fb_coord_t xpos, fb_coord_t ypos,
                              FAR const struct ltdc_area_s *area)
{
  gvdbg("layer=%p, lid=%d, xpos=%d, ypos=%d, area=%p\n",
            layer, lid, xpos, ypos, area);

  putreg32(stm32_dma2d_memaddress(layer, xpos, ypos), stm32_mar_layer_t[lid]);
  putreg32(stm32_dma2d_lineoffset(layer, area), stm32_or_layer_t[lid]);
}

/******************************************************************************
 * Name: stm32_dma2d_lcolor
 *
 * Description:
 *   Set the color for the layer
 *
 * Parameter:
 *   layer - Reference to the common layer state structure
 *
 ****************************************************************************/

static void stm32_dma2d_lcolor(FAR const struct stm32_dma2d_s *layer,
                               int lid, uint32_t color)
{
  gvdbg("layer=%p, lid=%d, color=%08x\n", layer, lid, color);
  putreg32(color, stm32_color_layer_t[lid]);
}

/******************************************************************************
 * Name: stm32_dma2d_llnr
 *
 * Description:
 *   Set the number of line register
 *
 * Parameter:
 *   layer   - Reference to the common layer state structure
 *   area    - Reference to the area to copy
 *
 ****************************************************************************/

static void stm32_dma2d_llnr(FAR struct stm32_dma2d_s *layer,
                                FAR const struct ltdc_area_s *area)
{
  uint32_t nlrreg;

  gvdbg("pixel per line: %d, number of lines: %d\n", area->xres, area->yres);

  nlrreg = getreg32(STM32_DMA2D_NLR);
  nlrreg = (DMA2D_NLR_PL(area->xres)|DMA2D_NLR_NL(area->yres));
  putreg32(nlrreg, STM32_DMA2D_NLR);
}

/******************************************************************************
 * Name: stm32_dma2d_loutpfc
 *
 * Description:
 *   Set the output PFC control register
 *
 * Parameter:
 *   layer - Reference to the common layer state structure
 *
 ****************************************************************************/

static int stm32_dma2d_loutpfc(FAR const struct stm32_dma2d_s *layer)
{
  gvdbg("layer=%p\n", layer);

  /* CLUT format isn't supported by the dma2d controller */

  if (layer->fmt == DMA2D_PF_L8)
    {
      /* Destination layer doesn't support CLUT output */

      gdbg("ERROR: Returning ENOSYS, "
            "output to layer with CLUT format not supported.\n");
      return -ENOSYS;
    }

  /* Set the mapped pixel format of source layer */

  putreg32(DMA2D_OPFCCR_CM(layer->fmt), STM32_DMA2D_OPFCCR);

  return OK;
}

/******************************************************************************
 * Name: stm32_dma2d_lpfc
 *
 * Description:
 *   Configure foreground and background layer PFC control register
 *
 * Parameter:
 *   layer - Reference to the common layer state structure
 *
 ****************************************************************************/

static void stm32_dma2d_lpfc(FAR const struct stm32_dma2d_s *layer,
                              int lid, uint32_t blendmode)
{
  uint32_t   pfccrreg;

  gvdbg("layer=%p, lid=%d, blendmode=%08x\n", layer, lid, blendmode);

  /* Set color format */

  pfccrreg = DMA2D_xGPFCCR_CM(layer->fmt);

#ifdef CONFIG_STM32_DMA2D_L8
  if (layer->fmt == DMA2D_PF_L8)
    {
      /* Load CLUT automatically */

      pfccrreg |= DMA2D_xGPFCCR_START;

      /* Set the CLUT color mode */

#ifndef CONFIG_FB_TRANSPARENCY
      pfccrreg |= DMA2D_xGPFCCR_CCM;
#endif

      /* Set CLUT size */

      pfccrreg |= DMA2D_xGPFCCR_CS(DMA2D_CLUT_SIZE);

      /* Set the CLUT memory address */

      putreg32((uint32_t) layer->clut, stm32_cmar_layer_t[lid]);

      /* Start async clut loading */

      stm32_dma2d_loadclut(stm32_pfccr_layer_t[lid]);
    }
#endif

  if (blendmode & DMA2D_BLEND_NONE)
    {
      /* No blend operation */

      pfccrreg |= DMA2D_xGPFCCR_AM(STM32_DMA2D_PFCCR_AM_NONE);
    }
  else
    {
      /* Set alpha value */

      pfccrreg |= DMA2D_xGPFCCR_ALPHA(layer->alpha);

      /* Set alpha mode */

      if (layer->blendmode & DMA2D_BLEND_ALPHA)
        {
          /* Blend with constant alpha */

          pfccrreg |= DMA2D_xGPFCCR_AM(STM32_DMA2D_PFCCR_AM_CONST);
        }
      else if (layer->blendmode & DMA2D_BLEND_PIXELALPHA)
        {
          /* Blend with pixel alpha value */

          pfccrreg |= DMA2D_xGPFCCR_AM(STM32_DMA2D_PFCCR_AM_PIXEL);
        }
    }

  putreg32(pfccrreg, stm32_pfccr_layer_t[lid]);
}

/******************************************************************************
 * Public Functions
 ******************************************************************************/

/******************************************************************************
 * Name: stm32_dma2dgetvideoinfo
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
 ******************************************************************************/

static int stm32_dma2dgetvideoinfo(FAR struct dma2d_layer_s *layer,
                            FAR struct fb_videoinfo_s *vinfo)
{
  FAR struct stm32_dma2d_s *priv = (FAR struct stm32_dma2d_s *)layer;

  gvdbg("layer=%p, vinfo=%p\n", layer, vinfo);

  if (stm32_dma2d_lvalidate(priv) && vinfo)
    {
      sem_wait(priv->lock);
      memcpy(vinfo, &priv->vinfo, sizeof(struct fb_videoinfo_s));
      sem_post(priv->lock);

      return OK;
    }

  gdbg("ERROR: Returning EINVAL\n");
  return -ENOSYS;
}

/******************************************************************************
 * Name: stm32_dma2dgetplaneinfo
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
 ******************************************************************************/

static int stm32_dma2dgetplaneinfo(FAR struct dma2d_layer_s *layer, int planeno,
                              FAR struct fb_planeinfo_s *pinfo)
{
  FAR struct stm32_dma2d_s *priv = (FAR struct stm32_dma2d_s *)layer;

  gvdbg("layer=%p, planeno=%d, pinfo=%p\n", layer, planeno, pinfo);

  if (stm32_dma2d_lvalidate(priv) && pinfo && planeno == 0)
    {
      sem_wait(priv->lock);
      memcpy(pinfo, &priv->pinfo, sizeof(struct fb_planeinfo_s));
      sem_post(priv->lock);

      return OK;
    }

  gdbg("ERROR: Returning EINVAL\n");
  return -EINVAL;
}

/******************************************************************************
 * Name: stm32_dma2dgetlid
 *
 * Description:
 *   Get a specific layer identifier.
 *
 * Parameter:
 *   layer - Reference to the layer structure
 *   lid   - Reference to store the layer id
 *
 * Return:
 *   On success - OK
 *   On error   - -EINVAL
 *
 ******************************************************************************/

static int stm32_dma2dgetlid(FAR struct dma2d_layer_s *layer, int *lid)
{
  FAR struct stm32_dma2d_s *priv = (FAR struct stm32_dma2d_s *)layer;

  gvdbg("layer=%p, lid=%p\n", layer, lid);

  if (stm32_dma2d_lvalidate(priv) && lid)
    {
      sem_wait(priv->lock);
      *lid = priv->lid;
      sem_post(priv->lock);
      return OK;
    }

  gdbg("ERROR: Returning EINVAL\n");
  return -EINVAL;
}

#ifdef CONFIG_STM32_DMA2D_L8
/******************************************************************************
 * Name: stm32_dma2dsetclut
 *
 * Description:
 *   Configure layer clut (color lookup table).
 *   Non clut is defined during initializing.
 *
 * Parameter:
 *   layer  - Reference to the layer structure
 *   cmap   - color lookup table with up the 256 entries
 *
 * Return:
 *   On success - OK
 *   On error   - -EINVAL
 *
 ******************************************************************************/

static int stm32_dma2dsetclut(FAR struct dma2d_layer_s *layer,
                            const FAR struct fb_cmap_s *cmap)
{
  int   ret;
  FAR struct stm32_dma2d_s *priv = (FAR struct stm32_dma2d_s *)layer;

  gvdbg("layer=%p, cmap=%p\n", layer, cmap);

  if (stm32_dma2d_lvalidate(priv) && cmap)
    {
      sem_wait(priv->lock);

#ifdef CONFIG_STM32_LTDC_INTERFACE
      if (priv->lid < DMA2D_SHADOW_LAYER)
        {
          /* Update the shared color lookup table.
           *
           * Background:
           *
           *  We share the same memory region of the clut table with the LTDC
           *  driver. (see stm32_dma2dinitltdc). This is important because any
           *  changes to the framebuffer and color lookup table by the ltdc
           *  related dma2d layer should also effects to the ltdc visibility,
           *  except operation settings, alpha and blendmode.
           *
           *  But we can not only update the clut memory region. The LTDC driver
           *  also must update they own LTDC clut register to make the changes
           *  visible. Using the LTDC interface to update the clut table will
           *  also update the clut table of the related dma2d layer.
           */

          FAR struct ltdc_layer_s *ltdc =
              g_ltdc_layer.layer[DMA2D_SHADOW_LAYER_L1].ltdc;

          ret = ltdc->setclut(ltdc, cmap);

          sem_post(priv->lock);

          return ret;
      }
#endif

      if (priv->fmt != DMA2D_PF_L8)
        {
          gdbg("Error: CLUT is not supported for the pixel format: %d\n",
                    priv->vinfo.fmt);
          ret = -EINVAL;
        }
      else if (cmap->first >= STM32_DMA2D_NCLUT)
        {
          gdbg("Error: only %d color table entries supported\n",
                    STM32_DMA2D_NCLUT);
          ret = -EINVAL;
        }
      else
        {
          uint32_t *clut;
          int      n;

          clut = priv->clut;

          for (n = cmap->first; n < cmap->len && n < STM32_DMA2D_NCLUT; n++)
            {
              /* Update the layer clut entry */

#ifndef CONFIG_FB_TRANSPARENCY
              uint8_t  *clut888 = (uint8_t*)clut;
              uint16_t offset   = 3 * n;

              clut888[offset]     = cmap->blue[n];
              clut888[offset + 1] = cmap->green[n];
              clut888[offset + 2] = cmap->red[n];

              regvdbg("n=%d, red=%02x, green=%02x, blue=%02x\n", n,
                        clut888[offset], clut888[offset + 1],
                        clut888[offset + 2]);
#else
              clut[n] = (uint32_t)DMA2D_CLUT_RED(cmap->transp[n]) |
                        (uint32_t)DMA2D_CLUT_GREEN(cmap->red[n]) |
                        (uint32_t)DMA2D_CLUT_GREEN(cmap->green[n]) |
                        (uint32_t)DMA2D_CLUT_BLUE(cmap->blue[n]);

              regvdbg("n=%d, alpha=%02x, red=%02x, green=%02x, blue=%02x\n", n,
                        DMA2D_CLUT_ALPHA(cmap->alpha[n]),
                        DMA2D_CLUT_RED(cmap->red[n]),
                        DMA2D_CLUT_GREEN(cmap->green[n]),
                        DMA2D_CLUT_BLUE(cmap->blue[n]));
#endif
            }


          ret = OK;
        }

      sem_post(priv->lock);
      return ret;
    }

  gdbg("ERROR: Returning EINVAL\n");
  return -EINVAL;
}

/******************************************************************************
 * Name: stm32_dma2dgetclut
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
 ******************************************************************************/

static int stm32_dma2dgetclut(FAR struct dma2d_layer_s *layer,
                            FAR struct fb_cmap_s *cmap)
{
  int ret;
  FAR struct stm32_dma2d_s *priv = (FAR struct stm32_dma2d_s *)layer;

  gvdbg("layer=%p, cmap=%p\n", layer, cmap);

  if (stm32_dma2d_lvalidate(priv) && cmap)
    {
      sem_wait(priv->lock);

      if (priv->fmt != DMA2D_PF_L8)
        {
          gdbg("Error: CLUT is not supported for the pixel format: %d\n",
                    priv->vinfo.fmt);
          ret = -EINVAL;
        }
      else if (cmap->first >= STM32_DMA2D_NCLUT)
        {
          gdbg("Error: only %d color table entries supported\n",
                    STM32_DMA2D_NCLUT);
          ret = -EINVAL;
        }
      else
        {
          /* Copy from the layer clut */

          uint32_t *clut;
          int      n;

          clut = priv->clut;

          for (n = cmap->first; n < cmap->len && n < STM32_DMA2D_NCLUT; n++)
            {
#ifndef CONFIG_FB_TRANSPARENCY
              uint8_t  *clut888 = (uint8_t*)clut;
              uint16_t offset   = 3 * n;

              cmap->blue[n]   = clut888[offset];
              cmap->green[n]  = clut888[offset + 1];
              cmap->red[n]    = clut888[offset + 2];

              regvdbg("n=%d, red=%02x, green=%02x, blue=%02x\n", n,
                        clut888[offset], clut888[offset + 1],
                        clut888[offset + 2]);
#else
              cmap->transp[n] = (uint8_t)DMA2D_CMAP_ALPHA(clut[n]);
              cmap->red[n]    = (uint8_t)DMA2D_CMAP_RED(clut[n]);
              cmap->green[n]  = (uint8_t)DMA2D_CMAP_GREEN(clut[n]);
              cmap->blue[n]   = (uint8_t)DMA2D_CMAP_BLUE(clut[n]);

              regvdbg("n=%d, alpha=%02x, red=%02x, green=%02x, blue=%02x\n", n,
                        DMA2D_CMAP_ALPHA(clut[n]), DMA2D_CMAP_RED(clut[n]),
                        DMA2D_CMAP_GREEN(clut[n]), DMA2D_CMAP_BLUE(clut[n]));
#endif
            }

          ret = OK;
        }

      sem_post(priv->lock);

      return ret;
    }

  gdbg("ERROR: Returning EINVAL\n");
  return -EINVAL;
}
#endif

/******************************************************************************
 * Name: stm32_dma2dsetalpha
 *
 * Description:
 *   Configure layer alpha value factor into blend operation.
 *   During the layer blend operation the source alpha value is multiplied
 *   with this alpha value. If the source color format doesn't support alpha
 *   channel (e.g. non ARGB8888) this alpha value will be used as constant
 *   alpha value for blend operation.
 *   Default value during initializing: 0xff
 *
 * Parameter:
 *   layer - Reference to the layer structure
 *   alpha - Alpha value
 *
 * Return:
 *   On success - OK
 *   On error - -EINVAL
 *
 ******************************************************************************/

static int stm32_dma2dsetalpha(FAR struct dma2d_layer_s *layer, uint8_t alpha)
{
  FAR struct stm32_dma2d_s *priv = (FAR struct stm32_dma2d_s *)layer;

  gvdbg("layer=%p, alpha=%02x\n", layer, alpha);

  if (stm32_dma2d_lvalidate(priv))
    {
      sem_wait(priv->lock);
      priv->alpha = alpha;
      sem_post(priv->lock);

      return OK;
    }

  gdbg("ERROR: Returning EINVAL\n");
  return -EINVAL;
}

/******************************************************************************
 * Name: stm32_dma2dgetalpha
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
 *****************************************************************************/

static int stm32_dma2dgetalpha(FAR struct dma2d_layer_s *layer, uint8_t *alpha)
{
  FAR struct stm32_dma2d_s *priv = (FAR struct stm32_dma2d_s *)layer;

  gvdbg("layer=%p, alpha=%p\n", layer, alpha);

  if (stm32_dma2d_lvalidate(priv))
    {
      sem_wait(priv->lock);
      *alpha = priv->alpha;
      sem_post(priv->lock);

      return OK;
    }

  gdbg("ERROR: Returning EINVAL\n");
  return -EINVAL;
}

/******************************************************************************
 * Name: stm32_dma2dsetblendmode
 *
 * Description:
 *   Configure blend mode of the layer.
 *   Default mode during initializing: DMA2D_BLEND_NONE
 *   Blendmode is active after next update.
 *
 * Parameter:
 *   layer - Reference to the layer structure
 *   mode  - Blend mode (see DMA2D_BLEND_*)
 *
 * Return:
 *   On success - OK
 *   On error - -EINVAL
 *
 * Procedure information:
 *   DMA2D_BLEND_NONE:
 *     Informs the driver to disable all blend operation for the given layer.
 *     That means the layer is opaque.
 *
 *   DMA2D_BLEND_ALPHA:
 *     Informs the driver to enable alpha blending for the given layer.
 *
 *   DMA2D_BLEND_PIXELALPHA:
 *     Informs the driver to use the pixel alpha value of the layer instead
 *     the constant alpha value. This is only useful for ARGB8888
 *     color format.
 *
 ******************************************************************************/

static int stm32_dma2dsetblendmode(FAR struct dma2d_layer_s *layer,
                                    uint32_t mode)
{
  FAR struct stm32_dma2d_s *priv = (FAR struct stm32_dma2d_s *)layer;

  gvdbg("layer=%p, mode=%08x\n", layer, mode);

  if (stm32_dma2d_lvalidate(priv))
    {
      sem_wait(priv->lock);
      priv->blendmode = mode;
      sem_post(priv->lock);

      return OK;
    }

  gdbg("ERROR: Returning EINVAL\n");
  return -EINVAL;
}

/******************************************************************************
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
 *
 *****************************************************************************/

static int stm32_dma2dgetblendmode(FAR struct dma2d_layer_s *layer,
                                    uint32_t *mode)
{
  FAR struct stm32_dma2d_s *priv = (FAR struct stm32_dma2d_s *)layer;

  gvdbg("layer=%p, mode=%p\n", layer, mode);

  if (stm32_dma2d_lvalidate(priv) && mode)
    {
      sem_wait(priv->lock);
      *mode = priv->blendmode;
      sem_post(priv->lock);

      return OK;
    }

  gdbg("ERROR: Returning EINVAL\n");
  return -EINVAL;
}

/******************************************************************************
 * Name: stm32_dma2dblit
 *
 * Description:
 *   Copy selected area from a source layer to selected position of the
 *   destination layer.
 *
 * Parameter:
 *   dest     - Valid reference to the destination layer
 *   destxpos - Valid selected x position of the destination layer
 *   destypos - Valid selected y position of the destination layer
 *   src      - Valid reference to the source layer
 *   srcarea  - Valid reference to the selected area of the source layer
 *
 * Return:
 *    OK        - On success
 *   -EINVAL    - If one of the parameter invalid or if the size of the selected
 *                source area outside the visible area of the destination layer.
 *                (The visible area usually represents the display size)
 *   -ECANCELED - Operation cancelled, something goes wrong.
 *
 ****************************************************************************/

static int stm32_dma2dblit(FAR struct dma2d_layer_s *dest,
                            fb_coord_t destxpos, fb_coord_t destypos,
                            FAR const struct dma2d_layer_s *src,
                            FAR const struct ltdc_area_s *srcarea)
{
  uint32_t   mode;
  int        ret;
  FAR struct stm32_dma2d_s * destlayer = (FAR struct stm32_dma2d_s *)dest;
  FAR struct stm32_dma2d_s * srclayer = (FAR struct stm32_dma2d_s *)src;

  gvdbg("dest=%p, destxpos=%d, destypos=%d, src=%p, srcarea=%p\n",
            dest, destxpos, destypos, src, srcarea);

  if (stm32_dma2d_lvalidatesize(destlayer, destxpos, destypos, srcarea) &&
        stm32_dma2d_lvalidatesize(srclayer, srcarea->xpos,
                                    srcarea->ypos, srcarea))
    {
      sem_wait(destlayer->lock);

      /* Set output pfc */

      ret = stm32_dma2d_loutpfc(destlayer);

      if (ret == OK)
        {
          /* Set foreground pfc */

          stm32_dma2d_lpfc(srclayer, DMA2D_LAYER_LFORE, DMA2D_BLEND_NONE);

          /* Set foreground fifo */

          stm32_dma2d_lfifo(srclayer, DMA2D_LAYER_LFORE,
                            srcarea->xpos, srcarea->ypos, srcarea);

          /* Set output fifo */

          stm32_dma2d_lfifo(destlayer, DMA2D_LAYER_LOUT,
                            destxpos, destypos, srcarea);

          /* Set number of lines and pixel per line */

          stm32_dma2d_llnr(destlayer, srcarea);

          /* Set dma2d mode for blit operation */

          if (destlayer->fmt == srclayer->fmt)
            {
              /* Blit without pfc */

              mode = STM32_DMA2D_CR_MODE_BLIT;
            }
          else
            {
              /* Blit with pfc */

              mode = STM32_DMA2D_CR_MODE_BLITPFC;
            }

          stm32_dma2d_control(mode, STM32_DMA2D_CR_MODE_CLEAR);

          /* Start DMA2D and wait until completed */

          ret = stm32_dma2d_start();

          if (ret != OK)
            {
              ret = -ECANCELED;
              gdbg("ERROR: Returning ECANCELED\n");
            }
        }

      sem_post(destlayer->lock);
    }
  else
    {
      ret = -EINVAL;
      gdbg("ERROR: Returning EINVAL\n");
    }

  return ret;
}

/****************************************************************************
 * Name: stm32_dma2dblend
 *
 * Description:
 *   Blends the selected area from a background layer with selected position
 *   of the foreground layer. Copies the result to the selected position of
 *   the destination layer. Note! The content of the foreground and background
 *   layer keeps unchanged as long destination layer is unequal to the
 *   foreground and background layer.
 *
 * Parameter:
 *   dest     - Reference to the destination layer
 *   fore     - Reference to the foreground layer
 *   forexpos - Selected x target position of the foreground layer
 *   foreypos - Selected y target position of the foreground layer
 *   back     - Reference to the background layer
 *   backarea - Reference to the selected area of the background layer
 *
 * Return:
 *    OK        - On success
 *   -EINVAL    - If one of the parameter invalid or if the size of the selected
 *                source area outside the visible area of the destination layer.
 *                (The visible area usually represents the display size)
 *   -ECANCELED - Operation cancelled, something goes wrong.
 *
 ****************************************************************************/

static int stm32_dma2dblend(FAR struct dma2d_layer_s *dest,
                            fb_coord_t destxpos, fb_coord_t destypos,
                            FAR const struct dma2d_layer_s *fore,
                            fb_coord_t forexpos, fb_coord_t foreypos,
                            FAR const struct dma2d_layer_s *back,
                            FAR const struct ltdc_area_s *backarea)
{
  int    ret;
  FAR struct stm32_dma2d_s * destlayer = (FAR struct stm32_dma2d_s *)dest;
  FAR struct stm32_dma2d_s * forelayer = (FAR struct stm32_dma2d_s *)fore;
  FAR struct stm32_dma2d_s * backlayer = (FAR struct stm32_dma2d_s *)back;

  gvdbg("dest=%p, destxpos=%d, destypos=%d, "
        "fore=%p, forexpos=%d, foreypos=%d, "
        "back=%p, backarea=%p\n",
        dest, destxpos, destypos, fore, forexpos, foreypos, back, backarea);

  if (stm32_dma2d_lvalidatesize(destlayer, destxpos, destypos, backarea) &&
        stm32_dma2d_lvalidatesize(forelayer, forexpos, foreypos, backarea) &&
            stm32_dma2d_lvalidatesize(backlayer, backarea->xpos,
                                        backarea->ypos, backarea))
    {

      sem_wait(destlayer->lock);

      /* Set output pfc */

      ret = stm32_dma2d_loutpfc(destlayer);

      if (ret == OK)
        {
          /* Set background pfc */

          stm32_dma2d_lpfc(backlayer, DMA2D_LAYER_LBACK, backlayer->blendmode);

          /* Set foreground pfc */

          stm32_dma2d_lpfc(forelayer, DMA2D_LAYER_LFORE, forelayer->blendmode);

          /* Set background fifo */

          stm32_dma2d_lfifo(backlayer, DMA2D_LAYER_LBACK,
                                backarea->xpos, backarea->ypos, backarea);

          /* Set foreground fifo */

          stm32_dma2d_lfifo(forelayer, DMA2D_LAYER_LFORE,
                                forexpos, foreypos, backarea);

          /* Set output fifo */

          stm32_dma2d_lfifo(destlayer, DMA2D_LAYER_LOUT,
                                destxpos, destypos, backarea);

          /* Set number of lines and pixel per line */

          stm32_dma2d_llnr(destlayer, backarea);

          /* Set watermark */

          /* Enable DMA2D blender */

          stm32_dma2d_control(STM32_DMA2D_CR_MODE_BLEND,
                                STM32_DMA2D_CR_MODE_CLEAR);

          /* Start DMA2D and wait until completed */

          ret = stm32_dma2d_start();

          if (ret != OK)
            {
              ret = -ECANCELED;
              gdbg("ERROR: Returning ECANCELED\n");
            }
        }

      sem_post(destlayer->lock);
    }
  else
    {
      ret = -EINVAL;
      gdbg("ERROR: Returning EINVAL\n");
    }

  return ret;
}

/****************************************************************************
 * Name: stm32_dma2dfillarea
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
 *    OK        - On success
 *   -EINVAL    - If one of the parameter invalid or if the size of the selected
 *                area outside the visible area of the layer.
 *   -ECANCELED - Operation cancelled, something goes wrong.
 *
 ****************************************************************************/

static int stm32_dma2dfillarea(FAR struct dma2d_layer_s *layer,
                               FAR const struct ltdc_area_s *area,
                               uint32_t color)
{
  int ret;
  FAR struct stm32_dma2d_s *priv = (FAR struct stm32_dma2d_s *)layer;

  gvdbg("layer=%p, area=%p, color=%08x\n", layer, area, color);

  if (stm32_dma2d_lvalidatesize(priv, area->xpos, area->ypos, area))
    {

      sem_wait(priv->lock);

      /* Set output pfc */

      ret = stm32_dma2d_loutpfc(priv);

      if (ret == OK)
        {
          /* Set output fifo */

          stm32_dma2d_lfifo(priv, DMA2D_LAYER_LOUT,
                                area->xpos, area->ypos, area);

          /* Set the output color register */

          stm32_dma2d_lcolor(priv, DMA2D_LAYER_LOUT, color);

          /* Set number of lines and pixel per line */

          stm32_dma2d_llnr(priv, area);

          /* Set register to memory transfer */

          stm32_dma2d_control(STM32_DMA2D_CR_MODE_COLOR,
                                STM32_DMA2D_CR_MODE_CLEAR);

          /* Start DMA2D and wait until completed */

          ret = stm32_dma2d_start();

          if (ret != OK)
            {
              ret = -ECANCELED;
              gdbg("ERROR: Returning ECANCELED\n");
            }
        }

      sem_post(priv->lock);
    }
  else
    {
      ret = -EINVAL;
      gdbg("ERROR: Returning EINVAL\n");
    }

  return ret;
}

/*******************************************************************************
 * Name: up_dma2dgetlayer
 *
 * Description:
 *   Get a dma2d layer structure by the layer identifier
 *
 * Parameter:
 *   lid - Layer identifier
 *
 * Return:
 *   Reference to the dma2d layer control structure on success or Null if no
 *   related exist.
 *
 ******************************************************************************/

FAR struct dma2d_layer_s * up_dma2dgetlayer(int lid)
{
  if (lid < DMA2D_LAYER_NSIZE)
    {
      FAR struct stm32_dma2d_s *priv;
      sem_wait(&g_lock);
      priv = g_layers[lid];
      sem_post(&g_lock);

      return &priv->dma2d;
    }

  gdbg("ERROR: EINVAL, Unknown layer identifier\n");
  errno = EINVAL;
  return NULL;
}

/******************************************************************************
 * Name: up_dma2dcreatelayer
 *
 * Description:
 *   Create a new dma2d layer object to interact with the dma2d controller
 *
 * Parameter:
 *   width  - Layer width
 *   height - Layer height
 *   fmt    - Pixel format of the layer
 *
 * Return:
 *   On success - A valid dma2d layer reference
 *   On error   - NULL and errno is set to
 *                -EINVAL if one of the parameter is invalid
 *                -ENOMEM if no memory available or exceeds
 *                 CONFIG_STM32_DMA2D_NLAYERS
 *
 ******************************************************************************/

FAR struct dma2d_layer_s *up_dma2dcreatelayer(fb_coord_t width,
                                              fb_coord_t height,
                                              uint8_t fmt)
{
  int        ret;
  int        lid;
  uint8_t    fmtmap;
  uint8_t    bpp = 0;
  FAR struct stm32_dma2d_s *layer = NULL;

  gvdbg("width=%d, height=%d, fmt=%02x \n", width, height, fmt);

  /* Validate if pixel format supported */

  ret = stm32_dma2d_pixelformat(fmt, &fmtmap);

  if (ret != OK)
    {
      errno = ret;
      return NULL;
    }

  ret = stm32_dma2d_bpp(fmt, &bpp);

  sem_wait(&g_lock);

  /* Get a free layer identifier */

  lid = stm32_dma2d_lfreelid();

  if (lid >= 0)
    {
      layer = stm32_dma2d_lalloc();

      if (layer)
        {
          uint32_t   fblen;
          void       *fbmem;
          fb_coord_t stride;

          /* Stride calculation for the supported formats */

          stride = width * bpp / 8;

          /* Calculate buffer size */

          fblen = stride * height;

          /* Allocate 32-bit aligned memory for the layer buffer. As reported in
           * mm_memalign 8-byte alignment is guaranteed by normal malloc calls.
           * We have also ensure memory is allocated from the SRAM1/2/3 block.
           * The CCM block is only accessible through the D-BUS but not by
           * the AHB-BUS. Ensure that CONFIG_STM32_CCMEXCLUDE is set!
           */

          fbmem = kmm_zalloc(fblen);

          if (fbmem)
            {
              FAR struct fb_videoinfo_s *vinfo = &layer->vinfo;
              FAR struct fb_planeinfo_s *pinfo = &layer->pinfo;

              /* Initialize dma2d structure */

              stm32_dma2d_linit(layer, lid, fmtmap);

              /* Initialize the videoinfo structure */

              vinfo->fmt     = fmt;
              vinfo->xres    = width;
              vinfo->yres    = height;
              vinfo->nplanes = 1;

              /* Initialize the planeinfo structure */

              pinfo->fbmem  = fbmem;
              pinfo->fblen  = fblen;
              pinfo->stride = stride;
              pinfo->bpp    = bpp;

              /* Bind the layer to the identifier */

              g_layers[lid] = layer;
            }
          else
            {
              /* free the layer struture */

              kmm_free(layer);
              gdbg("ERROR: ENOMEM, Unable to allocate layer buffer\n");
              errno = ENOMEM;
            }
        }
      else
        {
          gdbg("ERROR: ENOMEM, unable to allocate layer structure\n");
          errno = ENOMEM;
        }
    }
  else
    {
      gdbg("ERROR: EINVAL, no free layer available\n");
      errno = EINVAL;
    }

  sem_post(&g_lock);
  return (FAR struct dma2d_layer_s *)layer;
}

/******************************************************************************
 * Name: up_dma2dremovelayer
 *
 * Description:
 *  Remove and deallocate the dma2d layer
 *
 * Parameter:
 *   layer  - Reference to the layer to remove
 *
 * Return:
 *   On success - OK
 *   On error   - -EINVAL
 *
 *****************************************************************************/

int up_dma2dremovelayer(FAR struct dma2d_layer_s *layer)
{
  int ret = -EINVAL;
  FAR struct stm32_dma2d_s *priv = (FAR struct stm32_dma2d_s *)layer;

  /* Check if the layer is valid and unlike a ltdc related layer */

  if (stm32_dma2d_lvalidate(priv) && priv->lid >= DMA2D_SHADOW_LAYER)
    {
      sem_wait(priv->lock);

      /* Check also if the layer id is valid to the layer reference */

      if (priv == g_layers[priv->lid])
        {
          int lid = priv->lid;

          kmm_free(priv->pinfo.fbmem);
          stm32_dma2d_lfree(priv);

          g_layers[lid] = NULL;
          ret = OK;
        }

      sem_post(priv->lock);
    }

  return ret;
}

/******************************************************************************
 * Name: up_dma2dinitialize
 *
 * Description:
 *   Initialize the dma2d controller
 *
 * Return:
 *   OK - On success
 *   An error if initializing failed.
 *
 ******************************************************************************/

int up_dma2dinitialize(void)
{
  dbg("Initialize DMA2D driver\n");

  if (g_initialized == false)
    {
      /* Abort current dma2d data transfer */

      up_dma2duninitialize();

     /* Enable dma2d is done in rcc_enableahb1, see
      * arch/arm/src/stm32/stm32f40xxx_rcc.c
      */

      /* Initialize the DMA2D semaphore that enforces mutually exclusive access
       * to the driver
       */

      sem_init(&g_lock, 0, 1);

      /* Initialize the semaphore for interrupt handling */

      sem_init(g_interrupt.sem, 0, 0);

#ifdef CONFIG_STM32_DMA2D_L8
      /* Enable dma2d transfer and clut loading interrupts only */

      stm32_dma2d_control(DMA2D_CR_TCIE|DMA2D_CR_CTCIE, DMA2D_CR_TEIE|
                            DMA2D_CR_TWIE|DMA2D_CR_CAEIE||DMA2D_CR_CEIE);
#else
      /* Enable dma transfer interrupt only */

      stm32_dma2d_control(DMA2D_CR_TCIE, DMA2D_CR_TEIE|DMA2D_CR_TWIE|
                            DMA2D_CR_CAEIE|DMA2D_CR_CTCIE|DMA2D_CR_CEIE);
#endif

      /* Attach DMA2D interrupt vector */

      (void)irq_attach(g_interrupt.irq, stm32_dma2dirq);

      /* Enable the IRQ at the NVIC */

      up_enable_irq(g_interrupt.irq);

      /* Initialize the dma2d layer for ltdc binding */

#ifdef DMA2D_SHADOW_LAYER_L1
      g_layers[DMA2D_SHADOW_LAYER_L1] =
        &g_ltdc_layer.layer[DMA2D_SHADOW_LAYER_L1].dma2ddev;
#endif
#ifdef DMA2D_SHADOW_LAYER_L2
      g_layers[DMA2D_SHADOW_LAYER_L2] =
        &g_ltdc_layer.layer[DMA2D_SHADOW_LAYER_L2].dma2ddev;
#endif
      /* Set initialized state */

      g_initialized = true;
    }

  return OK;
}

/******************************************************************************
 * Name: up_dma2duninitialize
 *
 * Description:
 *   Uninitialize the dma2d controller
 *
 ******************************************************************************/

void up_dma2duninitialize(void)
{
  /* Disable DMA2D interrupts */

  up_disable_irq(g_interrupt.irq);
  irq_detach(g_interrupt.irq);

  /* Cleanup all layers */

  stm32_dma2d_llayerscleanup();

  /* Abort current dma2d transfer */

  stm32_dma2d_control(DMA2D_CR_ABORT, 0);

  /* Set initialized state */

  g_initialized = false;
}

#ifdef CONFIG_STM32_LTDC_INTERFACE
/******************************************************************************
 * Name: stm32_dma2dinitltdc
 *
 * Description:
 *   Get a reference to the dma2d layer coupled with the ltdc layer.
 *   It not intends to use this by user space applications.
 *   It resolves the following requirements:
 *   1. Share the color lookup table
 *   2. Share the planeinfo information
 *   3. Share the videoinfo information
 *
 * Parameter:
 *   layer  - a valid reference to the low level ltdc layer structure
 *   clut   - a pointer to a valid memory region to hold 256 clut colors
 *
 * Return:
 *   On success - A valid dma2d layer reference
 *   On error   - NULL and errno is set to
 *                -EINVAL if one of the parameter is invalid
 *
 ******************************************************************************/

FAR struct dma2d_layer_s * stm32_dma2dinitltdc(FAR struct stm32_ltdc_s *layer)
{
  int        ret;
  uint8_t    fmt = 0;
  FAR struct stm32_ltdc_dma2d_s *priv;

  gvdbg("layer=%p\n", layer);
  DEBUGASSERT(layer && layer->lid >= 0 && layer->lid < DMA2D_SHADOW_LAYER);

  ret = stm32_dma2d_pixelformat(layer->vinfo.fmt, &fmt);

  if (ret != OK)
    {
      dbg("Returning -EINVAL, unsupported pixel format: %d\n",
            layer->vinfo.fmt);
      errno = -EINVAL;
      return NULL;
    }

  priv = &g_ltdc_layer.layer[layer->lid];

  stm32_dma2d_linit(&priv->dma2ddev, layer->lid, fmt);

  memcpy(&priv->dma2ddev.vinfo, &layer->vinfo, sizeof(struct fb_videoinfo_s));
  memcpy(&priv->dma2ddev.pinfo, &layer->pinfo, sizeof(struct fb_planeinfo_s));

#ifdef CONFIG_STM32_DMA2D_L8
  /* Verifies that the ltdc layer has a clut. This ensures that DMA2D driver can
   * support clut format but the LTDC driver does not and vice versa.
   */

  if (layer->vinfo.fmt == FB_FMT_RGB8)
    {
      priv->dma2ddev.clut = layer->clut;
      priv->ltdc = stm32_ltdcgetlayer(layer->lid);
      DEBUGASSERT(priv->ltdc != NULL);
    }
#endif

  return &priv->dma2ddev.dma2d;
}
#endif /* CONFIG_STM32_LTDC_INTERFACE */
