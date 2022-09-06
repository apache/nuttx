/****************************************************************************
 * arch/arm/src/stm32/stm32_dma2d.c
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
 *   STM32F429 Technical Reference Manual
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <inttypes.h>
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
#include "stm32.h"
#include "hardware/stm32_ltdc.h"
#include "hardware/stm32_dma2d.h"
#include "stm32_ccm.h"
#include "stm32_dma2d.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* DMA2D supported operation layer (output, foreground, background) */

#define DMA2D_NLAYERS                       3

/* DMA2D blender control */

#define STM32_DMA2D_CR_MODE_BLIT            DMA2D_CR_MODE(0)
#define STM32_DMA2D_CR_MODE_BLITPFC         DMA2D_CR_MODE(1)
#define STM32_DMA2D_CR_MODE_BLEND           DMA2D_CR_MODE(2)
#define STM32_DMA2D_CR_MODE_COLOR           DMA2D_CR_MODE(3)
#define STM32_DMA2D_CR_MODE_CLEAR           STM32_DMA2D_CR_MODE_BLITPFC | \
                                            STM32_DMA2D_CR_MODE_BLEND   | \
                                            STM32_DMA2D_CR_MODE_COLOR

/* Only 8 bit per pixel overal supported */

#define DMA2D_PF_BYPP(n)                    ((n) / 8)

/* CC clut size */

#define DMA2D_CLUT_SIZE                     STM32_DMA2D_NCLUT - 1

/* Layer argb cmap conversion */

#define DMA2D_CLUT_ALPHA(n)                 ((uint32_t)(n) << 24)
#define DMA2D_CLUT_RED(n)                   ((uint32_t)(n) << 16)
#define DMA2D_CLUT_GREEN(n)                 ((uint32_t)(n) << 8)
#define DMA2D_CLUT_BLUE(n)                  ((uint32_t)(n) << 0)

#define DMA2D_CMAP_ALPHA(n)                 ((uint32_t)(n) >> 24)
#define DMA2D_CMAP_RED(n)                   ((uint32_t)(n) >> 16)
#define DMA2D_CMAP_GREEN(n)                 ((uint32_t)(n) >> 8)
#define DMA2D_CMAP_BLUE(n)                  ((uint32_t)(n) >> 0)

/* Debug option */

#ifdef CONFIG_STM32_DMA2D_REGDEBUG
#  define regerr                            lcderr
#  define reginfo                           lcdinfo
#else
#  define regerr(x...)
#  define reginfo(x...)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* DMA2D General layer information */

struct stm32_dma2d_s
{
  struct dma2d_layer_s dma2d;  /* Public dma2d interface */

#ifdef CONFIG_STM32_FB_CMAP
  uint32_t *clut;              /* Color lookup table */
#endif

  mutex_t  *lock;              /* Ensure mutually exclusive access */
};

/* Interrupt handling */

struct stm32_interrupt_s
{
  int    irq;       /* irq number */
  int  error;       /* Interrupt error */
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

#ifdef CONFIG_STM32_FB_CMAP
static const uintptr_t stm32_cmar_layer_t[DMA2D_NLAYERS - 1] =
{
  STM32_DMA2D_FGCMAR,
  STM32_DMA2D_BGCMAR
};
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Private functions */

static void stm32_dma2d_control(uint32_t setbits, uint32_t clrbits);
static int stm32_dma2dirq(int irq, void *context, void *arg);
static int stm32_dma2d_waitforirq(void);
static int stm32_dma2d_start(void);
#ifdef CONFIG_STM32_FB_CMAP
static int stm32_dma2d_loadclut(uintptr_t reg);
#endif
static uint32_t stm32_dma2d_memaddress(
                                   struct stm32_dma2d_overlay_s *oinfo,
                                   uint32_t xpos, uint32_t ypos);
static uint32_t stm32_dma2d_lineoffset(
                                   struct stm32_dma2d_overlay_s *oinfo,
                                   const struct fb_area_s *area);
static void stm32_dma2d_lfifo(struct stm32_dma2d_overlay_s *oinfo,
                              int lid,
                              uint32_t xpos, uint32_t ypos,
                              const struct fb_area_s *area);
static void stm32_dma2d_lcolor(int lid, uint32_t argb);
static void stm32_dma2d_llnr(const struct fb_area_s *area);
static int stm32_dma2d_loutpfc(uint8_t fmt);
static void stm32_dma2d_lpfc(int lid, uint32_t blendmode, uint8_t alpha,
                             uint8_t fmt);

/* Public Functions */

#ifdef CONFIG_STM32_FB_CMAP
static int stm32_dma2d_setclut(const struct fb_cmap_s *cmap);
#endif
static int stm32_dma2d_fillcolor(struct stm32_dma2d_overlay_s *oinfo,
                                 const struct fb_area_s *area,
                                 uint32_t argb);
static int stm32_dma2d_blit(struct stm32_dma2d_overlay_s *doverlay,
                            uint32_t destxpos, uint32_t destypos,
                            struct stm32_dma2d_overlay_s *soverlay,
                            const struct fb_area_s *sarea);
static int stm32_dma2d_blend(struct stm32_dma2d_overlay_s *doverlay,
                             uint32_t destxpos, uint32_t destypos,
                             struct stm32_dma2d_overlay_s *foverlay,
                             uint32_t forexpos, uint32_t foreypos,
                             struct stm32_dma2d_overlay_s *boverlay,
                             const struct fb_area_s *barea);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The initialized state of the driver */

static bool g_initialized;

/* Allocate clut */

#ifdef CONFIG_STM32_FB_CMAP
static uint32_t g_clut[STM32_DMA2D_NCLUT *
#  ifdef CONFIG_STM32_FB_TRANSPARENCY
                      4
#  else
                      3
#  endif
                      / 4];
#endif /* CONFIG_STM32_FB_CMAP */

/* The DMA2D mutex that enforces mutually exclusive access */

static mutex_t g_lock;

/* Semaphore for interrupt handling */

static sem_t g_semirq;

/* This structure provides irq handling */

static struct stm32_interrupt_s g_interrupt =
{
  .irq     = STM32_IRQ_DMA2D,
  .error   = OK,
  .sem     = &g_semirq
};

static struct stm32_dma2d_s g_dma2ddev =
{
  .dma2d =
  {
#ifdef CONFIG_STM32_FB_CMAP
    .setclut   = stm32_dma2d_setclut,
#endif
    .fillcolor = stm32_dma2d_fillcolor,
    .blit      = stm32_dma2d_blit,
    .blend     = stm32_dma2d_blend
  },
#ifdef CONFIG_STM32_FB_CMAP
  .clut = g_clut,
#endif
  .lock = &g_lock
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_dma2d_control
 *
 * Description:
 *   Change the DMA2D control register
 *
 * Input Parameters:
 *   setbits - The bits to set
 *   clrbits - The bits to clear
 *
 ****************************************************************************/

static void stm32_dma2d_control(uint32_t setbits, uint32_t clrbits)
{
  uint32_t   cr;

  lcdinfo("setbits=%08" PRIx32 ", clrbits=%08" PRIx32 "\n",
          setbits, clrbits);

  cr = getreg32(STM32_DMA2D_CR);
  cr &= ~clrbits;
  cr |= setbits;

  lcdinfo("cr=%08" PRIx32 "\n", cr);
  putreg32(cr, STM32_DMA2D_CR);
}

/****************************************************************************
 * Name: stm32_dma2dirq
 *
 * Description:
 *   DMA2D interrupt handler
 *
 ****************************************************************************/

static int stm32_dma2dirq(int irq, void *context, void *arg)
{
  int ret;
  uint32_t regval = getreg32(STM32_DMA2D_ISR);
  struct stm32_interrupt_s *priv = &g_interrupt;

  reginfo("irq = %d, regval = %08x\n", irq, regval);

  if (regval & DMA2D_ISR_TCIF)
    {
      /* Transfer complete interrupt */

      /* Clear the interrupt status register */

      reginfo("DMA transfer complete\n");
      putreg32(DMA2D_IFCR_CTCIF, STM32_DMA2D_IFCR);
      priv->error = OK;
    }
#ifdef CONFIG_STM32_DMA2D_L8
  else if (regval & DMA2D_ISR_CTCIF)
    {
      /* CLUT transfer complete interrupt */

      /* Clear the interrupt status register */

      reginfo("CLUT transfer complete\n");
      putreg32(DMA2D_IFCR_CCTCIF, STM32_DMA2D_IFCR);
      priv->error = OK;
    }
#endif
  else if (regval & DMA2D_ISR_TWIF)
    {
      /* Watermark transfer complete interrupt */

      /* Clear the interrupt status register */

      reginfo("Watermark transfer complete\n");
      putreg32(DMA2D_IFCR_CTWIF, STM32_DMA2D_IFCR);
      priv->error = OK;
    }
  else if (regval & DMA2D_ISR_TEIF)
    {
      /* Transfer error interrupt */

      /* Clear the interrupt status register */

      reginfo("ERROR: transfer\n");
      putreg32(DMA2D_IFCR_CTEIF, STM32_DMA2D_IFCR);
      priv->error = -ECANCELED;
    }
  else if (regval & DMA2D_ISR_CAEIF)
    {
      /* CLUT access error interrupt */

      /* Clear the interrupt status register */

      reginfo("ERROR: clut access\n");
      putreg32(DMA2D_IFCR_CAECIF, STM32_DMA2D_IFCR);
      priv->error = -ECANCELED;
    }
  else if (regval & DMA2D_ISR_CEIF)
    {
      /* Configuration error interrupt */

      /* Clear the interrupt status register */

      reginfo("ERROR: configuration\n");
      putreg32(DMA2D_IFCR_CCEIF, STM32_DMA2D_IFCR);
      priv->error = -ECANCELED;
    }
  else
    {
      /* Unknown irq, should not occur */

      DEBUGASSERT("Unknown interrupt error\n");
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
 * Name: stm32_dma2d_waitforirq
 *
 * Description:
 *   Helper waits until the dma2d irq occurs. That means that an ongoing clut
 *   loading or dma transfer was completed.
 *   Note! The caller must use this function within a critical section.
 *
 * Returned Value:
 *   On success OK otherwise ERROR
 *
 ****************************************************************************/

static int stm32_dma2d_waitforirq(void)
{
  int ret;
  struct stm32_interrupt_s *priv = &g_interrupt;

  ret = nxsem_wait(priv->sem);

  if (ret < 0)
    {
      lcderr("ERROR: nxsem_wait() failed\n");
      return ret;
    }

  ret = priv->error;

  return ret;
}

/****************************************************************************
 * Name: stm32_dma2d_loadclut
 *
 * Description:
 *   Starts clut loading but doesn't wait until loading is complete!
 *
 * Input Parameters:
 *   pfcreg - PFC control Register
 *
 * Returned Value:
 *   On success - OK
 *   On error - -EINVAL
 *
 ****************************************************************************/

#ifdef CONFIG_STM32_DMA2D_L8
static int stm32_dma2d_loadclut(uintptr_t pfcreg)
{
  int        ret;
  uint32_t   regval;

  /* Start clut loading */

  regval  = getreg32(pfcreg);
  regval |= DMA2D_XGPFCCR_START;
  reginfo("set regval=%08x\n", regval);
  putreg32(regval, pfcreg);
  reginfo("configured regval=%08x\n", getreg32(pfcreg));

  /* Wait until clut is finished */

  ret = stm32_dma2d_waitforirq();

  return ret;
}
#endif

/****************************************************************************
 * Name: stm32_dma2d_start
 *
 * Description:
 *   Starts the dma transfer and waits until completed.
 *
 * Input Parameters:
 *   reg       - Register to set the start
 *   startflag - The related flag to start the dma transfer
 *   irqflag   - The interrupt enable flag in the DMA2D_CR register
 *
 ****************************************************************************/

static int stm32_dma2d_start(void)
{
  int        ret;

  /* Start dma transfer */

  stm32_dma2d_control(DMA2D_CR_START, 0);

  /* wait until transfer is complete */

  ret = stm32_dma2d_waitforirq();

  return ret;
}

/****************************************************************************
 * Name: stm32_dma2d_memaddress
 *
 * Description:
 *   Helper to calculate the layer memory address
 *
 * Input Parameters:
 *   oinfo - Reference to overlay information
 *   xpos  - x-Offset
 *   ypos  - y-Offset
 *
 * Returned Value:
 *   memory address
 *
 ****************************************************************************/

static uint32_t stm32_dma2d_memaddress(
                                   struct stm32_dma2d_overlay_s *oinfo,
                                   uint32_t xpos, uint32_t ypos)
{
  uint32_t offset;
  struct fb_overlayinfo_s *poverlay = oinfo->oinfo;

  offset = xpos * DMA2D_PF_BYPP(poverlay->bpp) + poverlay->stride * ypos;

  lcdinfo("%" PRIx32 ", offset=%" PRId32 "\n",
          ((uint32_t) poverlay->fbmem) + offset, offset);
  return ((uint32_t) poverlay->fbmem) + offset;
}

/****************************************************************************
 * Name: stm32_dma2d_lineoffset
 *
 * Description:
 *   Helper to calculate the layer line offset
 *
 * Input Parameters:
 *   oinfo - Reference to overlay information
 *
 * Returned Value:
 *   line offset
 *
 ****************************************************************************/

static uint32_t stm32_dma2d_lineoffset(
                                   struct stm32_dma2d_overlay_s *oinfo,
                                   const struct fb_area_s *area)
{
  uint32_t loffset;

  /* offset at the end of each line in the context to the area layer */

  loffset = oinfo->xres - area->w;

  lcdinfo("%" PRId32 "\n", loffset);
  return loffset;
}

/****************************************************************************
 * Name: stm32_dma2d_lfifo
 *
 * Description:
 *   Set the fifo for the foreground, background and output layer
 *   Configures the memory address register
 *   Configures the line offset register
 *
 * Input Parameters:
 *   layer - Reference to the common layer state structure
 *
 ****************************************************************************/

static void stm32_dma2d_lfifo(struct stm32_dma2d_overlay_s *oinfo,
                              int lid, uint32_t xpos, uint32_t ypos,
                              const struct fb_area_s *area)
{
  lcdinfo("oinfo=%p, lid=%d, xpos=%" PRId32 ", ypos=%" PRId32 ", area=%p\n",
           oinfo, lid, xpos, ypos, area);

  putreg32(stm32_dma2d_memaddress(oinfo, xpos, ypos),
           stm32_mar_layer_t[lid]);
  putreg32(stm32_dma2d_lineoffset(oinfo, area), stm32_or_layer_t[lid]);
}

/****************************************************************************
 * Name: stm32_dma2d_lcolor
 *
 * Description:
 *   Set the color for the layer
 *
 * Input Parameters:
 *   lid  - Layer type (output, foreground, background)
 *   argb - argb8888 color
 *
 ****************************************************************************/

static void stm32_dma2d_lcolor(int lid, uint32_t argb)
{
  lcdinfo("lid=%d, argb=%08" PRIx32 "\n", lid, argb);
  putreg32(argb, stm32_color_layer_t[lid]);
}

/****************************************************************************
 * Name: stm32_dma2d_llnr
 *
 * Description:
 *   Set the number of line register
 *
 * Input Parameters:
 *   area - Reference to area information
 *
 ****************************************************************************/

static void stm32_dma2d_llnr(const struct fb_area_s *area)
{
  uint32_t nlrreg;

  lcdinfo("pixel per line: %d, number of lines: %d\n", area->w, area->h);

  nlrreg = getreg32(STM32_DMA2D_NLR);
  nlrreg = (DMA2D_NLR_PL(area->w) | DMA2D_NLR_NL(area->h));
  putreg32(nlrreg, STM32_DMA2D_NLR);
}

/****************************************************************************
 * Name: stm32_dma2d_loutpfc
 *
 * Description:
 *   Set the output PFC control register
 *
 * Input Parameters:
 *   fmt - DMA2D pixel format
 *
 ****************************************************************************/

static int stm32_dma2d_loutpfc(uint8_t fmt)
{
  lcdinfo("pixel format: %d\n", fmt);

  /* Set the mapped pixel format of the destination layer */

  putreg32(DMA2D_OPFCCR_CM(fmt), STM32_DMA2D_OPFCCR);

  return OK;
}

/****************************************************************************
 * Name: stm32_dma2d_lpfc
 *
 * Description:
 *   Configure foreground and background layer PFC control register
 *
 * Input Parameters:
 *   lid       - Layer id (output, foreground, background)
 *   blendmode - Layer blendmode (dma2d register values)
 *   alpha     - Transparency
 *
 ****************************************************************************/

static void stm32_dma2d_lpfc(int lid, uint32_t blendmode, uint8_t alpha,
                             uint8_t fmt)
{
  uint32_t   pfccrreg;

  lcdinfo("lid=%d, blendmode=%08" PRIx32 ", alpha=%02x, fmt=%d\n",
          lid, blendmode, alpha, fmt);

  /* Set color format */

  pfccrreg = DMA2D_XGPFCCR_CM(fmt);

#ifdef CONFIG_STM32_FB_CMAP
  if (fmt == DMA2D_PF_L8)
    {
      struct stm32_dma2d_s * layer = &g_dma2ddev;

      /* Load CLUT automatically */

      pfccrreg |= DMA2D_XGPFCCR_START;

      /* Set the CLUT color mode */

#  ifndef CONFIG_STM32_FB_TRANSPARENCY
      pfccrreg |= DMA2D_XGPFCCR_CCM;
#  endif

      /* Set CLUT size */

      pfccrreg |= DMA2D_XGPFCCR_CS(DMA2D_CLUT_SIZE);

      /* Set the CLUT memory address */

      putreg32((uint32_t) layer->clut, stm32_cmar_layer_t[lid]);

      /* Start async clut loading */

      stm32_dma2d_loadclut(stm32_pfccr_layer_t[lid]);
    }
#endif /* CONFIG_STM32_FB_CMAP */

  /* Set alpha blend mode */

  pfccrreg |= DMA2D_XGPFCCR_AM(blendmode);

  if (blendmode == STM32_DMA2D_PFCCR_AM_CONST ||
        blendmode == STM32_DMA2D_PFCCR_AM_PIXEL)
    {
      /* Set alpha value */

      pfccrreg |= DMA2D_XGPFCCR_ALPHA(alpha);
    }

  putreg32(pfccrreg, stm32_pfccr_layer_t[lid]);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_dma2d_setclut
 *
 * Description:
 *   Configure layer clut (color lookup table).
 *
 * Input Parameters:
 *   cmap   - Color lookup table with up the 256 entries
 *
 * Returned Value:
 *   On success - OK
 *   On error   - -EINVAL
 *
 ****************************************************************************/

#ifdef CONFIG_STM32_FB_CMAP
static int stm32_dma2d_setclut(const struct fb_cmap_s *cmap)
{
  int n;
  struct stm32_dma2d_s * priv = &g_dma2ddev;

  lcdinfo("cmap=%p\n", cmap);

  nxmutex_lock(priv->lock);

  for (n = cmap->first; n < cmap->len - 1 && n < STM32_DMA2D_NCLUT; n++)
    {
      /* Update the layer clut entry, will be automatically loaded before
       * blit operation becomes active
       */

#  ifndef CONFIG_STM32_FB_TRANSPARENCY
      uint8_t *clut   = (uint8_t *)g_dma2ddev.clut;
      uint16_t offset = 3 * n;

      clut[offset]     = cmap->blue[n];
      clut[offset + 1] = cmap->green[n];
      clut[offset + 2] = cmap->red[n];

      reginfo("n=%d, red=%02x, green=%02x, blue=%02x\n", n, clut[offset],
              clut[offset + 1], clut[offset + 2]);
#  else
      uint32_t *clut  = g_dma2ddev.clut;

      clut[n] = (uint32_t)DMA2D_CLUT_ALPHA(cmap->transp[n]) |
                (uint32_t)DMA2D_CLUT_RED(cmap->red[n]) |
                (uint32_t)DMA2D_CLUT_GREEN(cmap->green[n]) |
                (uint32_t)DMA2D_CLUT_BLUE(cmap->blue[n]);

      reginfo("n=%d, alpha=%02x, red=%02x, green=%02x, blue=%02x\n", n,
                DMA2D_CLUT_ALPHA(cmap->transp[n]),
                DMA2D_CLUT_RED(cmap->red[n]),
                DMA2D_CLUT_GREEN(cmap->green[n]),
                DMA2D_CLUT_BLUE(cmap->blue[n]));
#  endif
    }

  nxmutex_unlock(priv->lock);
  return OK;
}
#endif /* CONFIG_STM32_FB_CMAP */

/****************************************************************************
 * Name: stm32_dma2d_fillcolor
 *
 * Description:
 *   Fill the selected area of the whole overlay with a specific color.
 *   The caller must ensure that the area is within the entire overlay.
 *
 * Input Parameters:
 *   oinfo - Overlay to fill
 *   area  - Reference to the valid area structure select the area
 *   argb  - Color to fill the selected area. Color must be argb8888
 *           formatted.
 *
 * Returned Value:
 *    OK        - On success
 *   -EINVAL    - If one of the parameter invalid or if the size of the
 *                selected area outside the visible area of the layer.
 *   -ECANCELED - Operation cancelled, something goes wrong.
 *
 ****************************************************************************/

static int stm32_dma2d_fillcolor(struct stm32_dma2d_overlay_s *oinfo,
                                 const struct fb_area_s *area,
                                 uint32_t argb)
{
  int ret;
  struct stm32_dma2d_s * priv = &g_dma2ddev;
  DEBUGASSERT(oinfo != NULL && oinfo->oinfo != NULL && area != NULL);

  lcdinfo("oinfo=%p, argb=%08" PRIx32 "\n", oinfo, argb);

#ifdef CONFIG_STM32_FB_CMAP
  if (oinfo->fmt == DMA2D_PF_L8)
    {
      /* CLUT output not supported */

      lcderr("ERROR: Returning ENOSYS, "
             "output to layer with CLUT format not supported.\n");
      return -ENOSYS;
    }
#endif

  nxmutex_lock(priv->lock);

  /* Set output pfc */

  stm32_dma2d_loutpfc(oinfo->fmt);

  /* Set output fifo */

  stm32_dma2d_lfifo(oinfo, DMA2D_LAYER_LOUT, area->x, area->y, area);

  /* Set the output color register */

  stm32_dma2d_lcolor(DMA2D_LAYER_LOUT, argb);

  /* Set number of lines and pixel per line */

  stm32_dma2d_llnr(area);

  /* Set register to memory transfer */

  stm32_dma2d_control(STM32_DMA2D_CR_MODE_COLOR, STM32_DMA2D_CR_MODE_CLEAR);

  /* Start DMA2D and wait until completed */

  ret = stm32_dma2d_start();

  if (ret != OK)
    {
      ret = -ECANCELED;
      lcderr("ERROR: Returning ECANCELED\n");
    }

  nxmutex_unlock(priv->lock);
  return ret;
}

/****************************************************************************
 * Name: stm32_dma2d_blit
 *
 * Description:
 *   Copy memory from a source overlay (defined by sarea) to destination
 *   overlay position (defined by destxpos and destypos).
 *
 * Input Parameters:
 *   doverlay - Valid reference to the destination overlay
 *   destxpos - Valid selected x position of the destination overlay
 *   destypos - Valid selected y position of the destination overlay
 *   soverlay - Valid reference to the source overlay
 *   sarea    - Valid reference to the selected area of the source overlay
 *
 * Returned Value:
 *    OK        - On success
 *   -EINVAL    - If one of the parameter invalid or if the size of the
 *                selected source area outside the visible area of the
 *                destination layer.
 *                (The visible area usually represents the display size)
 *   -ECANCELED - Operation cancelled, something goes wrong.
 *
 ****************************************************************************/

static int stm32_dma2d_blit(struct stm32_dma2d_overlay_s *doverlay,
                            uint32_t destxpos, uint32_t destypos,
                            struct stm32_dma2d_overlay_s *soverlay,
                            const struct fb_area_s *sarea)
{
  int        ret;
  uint32_t  mode;
  struct stm32_dma2d_s * priv = &g_dma2ddev;

  lcdinfo("doverlay=%p, destxpos=%" PRId32 ", destypos=%" PRId32
          ", soverlay=%p, sarea=%p\n",
          doverlay, destxpos, destypos, soverlay, sarea);

  nxmutex_lock(priv->lock);

  /* Set output pfc */

  stm32_dma2d_loutpfc(doverlay->fmt);

  /* Set foreground pfc */

  stm32_dma2d_lpfc(DMA2D_LAYER_LFORE, STM32_DMA2D_PFCCR_AM_NONE, 0,
                   soverlay->fmt);

  /* Set foreground fifo */

  stm32_dma2d_lfifo(soverlay, DMA2D_LAYER_LFORE, sarea->x, sarea->y, sarea);

  /* Set output fifo */

  stm32_dma2d_lfifo(doverlay, DMA2D_LAYER_LOUT, destxpos, destypos, sarea);

  /* Set number of lines and pixel per line */

  stm32_dma2d_llnr(sarea);

  /* Set dma2d mode for blit operation */

  if (doverlay->fmt == soverlay->fmt)
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
      lcderr("ERROR: Returning ECANCELED\n");
    }

  nxmutex_unlock(priv->lock);
  return ret;
}

/****************************************************************************
 * Name: stm32_dma2d_blend
 *
 * Description:
 *   Blends the selected area from a background layer with selected position
 *   of the foreground layer. Copies the result to the selected position of
 *   the destination layer. Note! The content of the foreground and
 *   background layer keeps unchanged as long destination layer is unequal to
 *   the foreground and background layer.
 *
 * Input Parameters:
 *   doverlay - Destination overlay
 *   destxpos - x-Offset destination overlay
 *   destypos - y-Offset destination overlay
 *   foverlay - Foreground overlay
 *   forexpos - x-Offset foreground overlay
 *   foreypos - y-Offset foreground overlay
 *   boverlay - Background overlay
 *   barea    - x-Offset, y-Offset, x-resolution and y-resolution of
 *              background overlay
 *
 * Returned Value:
 *    OK        - On success
 *   -EINVAL    - If one of the parameter invalid or if the size of the
 *                selected source area outside the visible area of the
 *                destination layer.
 *                (The visible area usually represents the display size)
 *   -ECANCELED - Operation cancelled, something goes wrong.
 *
 ****************************************************************************/

static int stm32_dma2d_blend(struct stm32_dma2d_overlay_s *doverlay,
                             uint32_t destxpos, uint32_t destypos,
                             struct stm32_dma2d_overlay_s *foverlay,
                             uint32_t forexpos, uint32_t foreypos,
                             struct stm32_dma2d_overlay_s *boverlay,
                             const struct fb_area_s *barea)
{
  int    ret;
  struct stm32_dma2d_s * priv = &g_dma2ddev;

  lcdinfo("doverlay=%p, destxpos=%" PRId32 ", destypos=%" PRId32 ", "
          "foverlay=%p, forexpos=%" PRId32 ", foreypos=%" PRId32 ", "
          "boverlay=%p, barea=%p, barea.x=%d, barea.y=%d, barea.w=%d, "
          "barea.h=%d\n", doverlay, destxpos, destypos, foverlay, forexpos,
          foreypos, boverlay, barea, barea->x, barea->y, barea->w, barea->h);

#ifdef CONFIG_STM32_FB_CMAP
  if (doverlay->fmt == DMA2D_PF_L8)
    {
      /* CLUT output not supported */

      lcderr("ERROR: Returning ENOSYS, "
             "output to layer with CLUT format not supported.\n");
      return -ENOSYS;
    }
#endif

  nxmutex_lock(priv->lock);

  /* Set output pfc */

  stm32_dma2d_loutpfc(doverlay->fmt);

  /* Set background pfc */

  stm32_dma2d_lpfc(DMA2D_LAYER_LBACK, boverlay->transp_mode,
                   boverlay->oinfo->transp.transp, boverlay->fmt);

  /* Set foreground pfc */

  stm32_dma2d_lpfc(DMA2D_LAYER_LFORE, foverlay->transp_mode,
                   foverlay->oinfo->transp.transp, foverlay->fmt);

  /* Set background fifo */

  stm32_dma2d_lfifo(boverlay, DMA2D_LAYER_LBACK, barea->x, barea->y, barea);

  /* Set foreground fifo */

  stm32_dma2d_lfifo(foverlay, DMA2D_LAYER_LFORE, forexpos, foreypos, barea);

  /* Set output fifo */

  stm32_dma2d_lfifo(doverlay, DMA2D_LAYER_LOUT, destxpos, destypos, barea);

  /* Set number of lines and pixel per line */

  stm32_dma2d_llnr(barea);

  /* Set watermark */

  /* Enable DMA2D blender */

  stm32_dma2d_control(STM32_DMA2D_CR_MODE_BLEND, STM32_DMA2D_CR_MODE_CLEAR);

  /* Start DMA2D and wait until completed */

  ret = stm32_dma2d_start();

  if (ret != OK)
    {
      ret = -ECANCELED;
      lcderr("ERROR: Returning ECANCELED\n");
    }

  nxmutex_unlock(priv->lock);
  return ret;
}

/****************************************************************************
 * Name: stm32_dma2dinitialize
 *
 * Description:
 *   Initialize the dma2d controller
 *
 * Returned Value:
 *   OK - On success
 *   An error if initializing failed.
 *
 ****************************************************************************/

int stm32_dma2dinitialize(void)
{
  lcdinfo("Initialize DMA2D driver\n");

  if (g_initialized == false)
    {
      /* Abort current dma2d data transfer */

      stm32_dma2duninitialize();

      /* Enable dma2d is done in rcc_enableahb1, see
       * arch/arm/src/stm32/stm32f40xxx_rcc.c
       */

      /* Initialize the DMA2D mutex that enforces mutually exclusive
       * access to the driver
       */

      nxmutex_init(&g_lock);

      /* Initialize the semaphore for interrupt handling.  This waitsem
       * semaphore is used for signaling and, hence, should not have
       * priority inheritance enabled.
       */

      nxsem_init(g_interrupt.sem, 0, 0);
      nxsem_set_protocol(g_interrupt.sem, SEM_PRIO_NONE);

#ifdef CONFIG_STM32_FB_CMAP
      /* Enable dma2d transfer and clut loading interrupts only */

      stm32_dma2d_control(DMA2D_CR_TCIE | DMA2D_CR_CTCIE, DMA2D_CR_TEIE |
                          DMA2D_CR_TWIE | DMA2D_CR_CAEIE | DMA2D_CR_CEIE);
#else
      /* Enable dma transfer interrupt only */

      stm32_dma2d_control(DMA2D_CR_TCIE, DMA2D_CR_TEIE | DMA2D_CR_TWIE |
                          DMA2D_CR_CAEIE | DMA2D_CR_CTCIE | DMA2D_CR_CEIE);
#endif

      stm32_dma2d_control(DMA2D_CR_TCIE | DMA2D_CR_CTCIE | DMA2D_CR_TEIE |
                          DMA2D_CR_CAEIE | DMA2D_CR_CTCIE | DMA2D_CR_CEIE,
                          0);

      /* Attach DMA2D interrupt vector */

      irq_attach(g_interrupt.irq, stm32_dma2dirq, NULL);

      /* Enable the IRQ at the NVIC */

      up_enable_irq(g_interrupt.irq);

      g_initialized = true;
    }

  return OK;
}

/****************************************************************************
 * Name: stm32_dma2duninitialize
 *
 * Description:
 *   Uninitialize the dma2d controller
 *
 ****************************************************************************/

void stm32_dma2duninitialize(void)
{
  /* Disable DMA2D interrupts */

  up_disable_irq(g_interrupt.irq);
  irq_detach(g_interrupt.irq);

  /* Abort current dma2d transfer */

  stm32_dma2d_control(DMA2D_CR_ABORT, 0);

  /* Set initialized state */

  g_initialized = false;
}

/****************************************************************************
 * Name: stm32_dma2ddev
 *
 * Description:
 *   Get a reference to the dma2d controller.
 *
 * Returned Value:
 *   On success - A valid dma2d layer reference
 *   On error   - NULL
 *
 ****************************************************************************/

struct dma2d_layer_s *stm32_dma2ddev(void)
{
  return &g_dma2ddev.dma2d;
}
