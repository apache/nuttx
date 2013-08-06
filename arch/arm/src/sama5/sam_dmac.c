/****************************************************************************
 * arch/arm/src/sama5/sam3u_dmac.c
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
#include <stdbool.h>
#include <string.h>
#include <semaphore.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <arch/irq.h>

#include "up_arch.h"
#include "up_internal.h"
#include "os_internal.h"

#include "chip.h"
#include "sam_dmac.h"
#include "sam_periphclks.h"
#include "chip/sam_pmc.h"
#include "chip/sam_dmac.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* All of the currently supported SAMA5 chips support two DMA controllers
 * of 8 DMA Channels each.
 */

#if SAM_NDMAC < 1
#  undef CONFIG_SAMA5_DMAC1
#  undef CONFIG_SAMA5_DMAC0
#elif SAM_NDMAC < 2
#  undef CONFIG_SAMA5_DMAC1
#endif

/* Condition out the whole file unless DMA is selected in the configuration */

#if defined(CONFIG_SAMA5_DMAC0) || defined(CONFIG_SAMA5_DMAC1)

/* If SAMA5 DMA support is enabled, then OS DMA support should also be
 * enabled
 */

#ifndef CONFIG_ARCH_DMA
#  warning "SAMA5 DMA enabled but CONFIG_ARCH_DMA disabled"
#endif

/* Check the number of link list descriptors to allocate */

#ifndef CONFIG_SAMA5_NLLDESC
#  define CONFIG_SAMA5_NLLDESC SAM_NDMACHAN
#endif

#if CONFIG_SAMA5_NLLDESC < SAM_NDMACHAN
#  warning "At least SAM_NDMACHAN descriptors must be allocated"

#  undef CONFIG_SAMA5_NLLDESC
#  define CONFIG_SAMA5_NLLDESC SAM_NDMACHAN
#endif

/* Register values **********************************************************/

#define DMAC_CH_CTRLB_BOTHDSCR \
  (DMAC_CH_CTRLB_SRCDSCR | DMAC_CH_CTRLB_DSTDSCR)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure descibes one DMA channel */

struct sam_dmach_s
{
#if defined(CONFIG_SAMA5_DMAC0) && defined(CONFIG_SAMA5_DMAC1)
  uint8_t dmac;                   /* DMA controller number (0-1) */
#endif
  uint8_t chan;                   /* DMA channel number (0-6) */
  bool inuse;                     /* TRUE: The DMA channel is in use */
  uint32_t flags;                 /* DMA channel flags */
  uint32_t base;                  /* DMA register channel base address */
  uint32_t cfg;                   /* Pre-calculated CFG register for transfer */
  dma_callback_t callback;        /* Callback invoked when the DMA completes */
  void *arg;                      /* Argument passed to callback function */
  struct dma_linklist_s *llhead;  /* DMA link list head */
  struct dma_linklist_s *lltail;  /* DMA link list head */
};

/* This structure describes the stae of one DMA controller */

struct sam_dmac_s
{
  /* These semaphores protect the DMA channel and descriptor tables */

  sem_t chsem;                       /* Protects channel table */
  sem_t dsem;                        /* Protects descriptior table */
  uint32_t base;                     /* DMA register channel base address */

  /* This array describes the available link list descriptors */

  struct dma_linklist_s *desc;

  /* This array describes each DMA channel */

  struct sam_dmach_s *dmach;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* CTRLA field lookups */

static const uint32_t g_srcwidth[4] =
{
  DMAC_CH_CTRLA_SRCWIDTH_BYTE,
  DMAC_CH_CTRLA_SRCWIDTH_HWORD,
  DMAC_CH_CTRLA_SRCWIDTH_WORD,
  DMAC_CH_CTRLA_SRCWIDTH_DWORD
};

static const uint32_t g_destwidth[4] =
{
  DMAC_CH_CTRLA_DSTWIDTH_BYTE,
  DMAC_CH_CTRLA_DSTWIDTH_HWORD,
  DMAC_CH_CTRLA_DSTWIDTH_WORD,
  DMAC_CH_CTRLA_DSTWIDTH_DWORD
};

static const uint32_t g_fifocfg[3] =
{
  DMAC_CH_CFG_FIFOCFG_ALAP,
  DMAC_CH_CFG_FIFOCFG_HALF,
  DMAC_CH_CFG_FIFOCFG_ASAP
};

#ifdef CONFIG_SAMA5_DMAC0

/* This array describes the available link list descriptors */

struct dma_linklist_s g_desc0[CONFIG_SAMA5_NLLDESC];

/* This array describes the state of each DMAC0 channel 0 */

static struct sam_dmach_s g_dmach0[SAM_NDMACHAN] =
{
#if SAM_NDMACHAN > 0
  {
#if defined(CONFIG_SAMA5_DMAC0) && defined(CONFIG_SAMA5_DMAC1)
    .dmac     = 0,
#endif
    .chan     = 0,
    .base     = SAM_DMAC0_CH0_BASE,
  },
#endif
#if SAM_NDMACHAN > 1
  {
#if defined(CONFIG_SAMA5_DMAC0) && defined(CONFIG_SAMA5_DMAC1)
    .dmac     = 0,
#endif
    .chan     = 1,
    .base     = SAM_DMAC0_CH1_BASE,
  },
#endif
#if SAM_NDMACHAN > 2
  {
#if defined(CONFIG_SAMA5_DMAC0) && defined(CONFIG_SAMA5_DMAC1)
    .dmac     = 0,
#endif
    .chan     = 2,
    .base     = SAM_DMAC0_CH2_BASE,
  },
#endif
#if SAM_NDMACHAN > 3
  {
#if defined(CONFIG_SAMA5_DMAC0) && defined(CONFIG_SAMA5_DMAC1)
    .dmac     = 0,
#endif
    .chan     = 3,
    .base     = SAM_DMAC0_CH3_BASE,
  },
#endif
#if SAM_NDMACHAN > 4
  {
#if defined(CONFIG_SAMA5_DMAC0) && defined(CONFIG_SAMA5_DMAC1)
    .dmac     = 0,
#endif
    .chan     = 4,
    .base     = SAM_DMAC0_CH4_BASE,
  },
#endif
#if SAM_NDMACHAN > 5
  {
#if defined(CONFIG_SAMA5_DMAC0) && defined(CONFIG_SAMA5_DMAC1)
    .dmac     = 0,
#endif
    .chan     = 5,
    .base     = SAM_DMAC0_CH5_BASE,
  },
#endif
#if SAM_NDMACHAN > 6
  {
#if defined(CONFIG_SAMA5_DMAC0) && defined(CONFIG_SAMA5_DMAC1)
    .dmac     = 0,
#endif
    .chan     = 6,
    .base     = SAM_DMAC0_CH6_BASE,
  },
#endif
#if SAM_NDMACHAN > 7
  {
#if defined(CONFIG_SAMA5_DMAC0) && defined(CONFIG_SAMA5_DMAC1)
    .dmac     = 0,
#endif
    .chan     = 7,
    .base     = SAM_DMAC0_CH7_BASE,
  }
#endif
};

/* This describes the overall state of DMA controller 0 */

static struct sam_dmac_s g_dmac0 =
{
  /* DMAC 0 base address */

  .base       = SAM_DMAC0_VBASE,

  /* This array describes the available link list descriptors */

  .desc       = g_desc0,

  /* This array describes each DMA channel */

  .dmach      = g_dmach0,
};

#endif /* CONFIG_SAMA5_DMAC0 */

/* This array describes the state of DMA controller 1 */

#ifdef CONFIG_SAMA5_DMAC1
/* This array describes the available link list descriptors */

struct dma_linklist_s g_desc1[CONFIG_SAMA5_NLLDESC];

static struct sam_dmach_s g_dmach1[SAM_NDMACHAN] =
{
#if SAM_NDMACHAN > 0
  {
#if defined(CONFIG_SAMA5_DMAC0) && defined(CONFIG_SAMA5_DMAC1)
    .dmac     = 1,
#endif
    .chan     = 0,
    .base     = SAM_DMAC1_CH0_BASE,
  },
#endif
#if SAM_NDMACHAN > 1
  {
#if defined(CONFIG_SAMA5_DMAC0) && defined(CONFIG_SAMA5_DMAC1)
    .dmac     = 1,
#endif
    .chan     = 1,
    .base     = SAM_DMAC1_CH1_BASE,
  },
#endif
#if SAM_NDMACHAN > 2
  {
#if defined(CONFIG_SAMA5_DMAC0) && defined(CONFIG_SAMA5_DMAC1)
    .dmac     = 1,
#endif
    .chan     = 2,
    .base     = SAM_DMAC1_CH2_BASE,
  },
#endif
#if SAM_NDMACHAN > 3
  {
#if defined(CONFIG_SAMA5_DMAC0) && defined(CONFIG_SAMA5_DMAC1)
    .dmac     = 1,
#endif
    .chan     = 3,
    .base     = SAM_DMAC1_CH3_BASE,
  },
#endif
#if SAM_NDMACHAN > 4
  {
#if defined(CONFIG_SAMA5_DMAC0) && defined(CONFIG_SAMA5_DMAC1)
    .dmac     = 1,
#endif
    .chan     = 4,
    .base     = SAM_DMAC1_CH4_BASE,
  },
#endif
#if SAM_NDMACHAN > 5
  {
#if defined(CONFIG_SAMA5_DMAC0) && defined(CONFIG_SAMA5_DMAC1)
    .dmac     = 1,
#endif
    .chan     = 5,
    .base     = SAM_DMAC1_CH5_BASE,
  },
#endif
#if SAM_NDMACHAN > 6
  {
#if defined(CONFIG_SAMA5_DMAC0) && defined(CONFIG_SAMA5_DMAC1)
    .dmac     = 1,
#endif
    .chan     = 6,
    .base     = SAM_DMAC1_CH6_BASE,
  },
#endif
#if SAM_NDMACHAN > 7
  {
#if defined(CONFIG_SAMA5_DMAC0) && defined(CONFIG_SAMA5_DMAC1)
    .dmac     = 1,
#endif
    .chan     = 7,
    .base     = SAM_DMAC1_CH7_BASE,
  }
#endif
};

/* This describes the overall state of DMA controller 1 */

static struct sam_dmac_s g_dmac1 =
{
  /* DMAC 0 base address */

  .base       = SAM_DMAC1_VBASE,

  /* This array describes the available link list descriptors */

  .desc       = g_desc1,

  /* This array describes each DMA channel */

  .dmach      = g_dmach1,
};

#endif /* CONFIG_SAMA5_DMAC1 */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_takechsem() and sam_givechsem()
 *
 * Description:
 *   Used to get exclusive access to the DMA channel table
 *
 ****************************************************************************/

static void sam_takechsem(struct sam_dmac_s *dmac)
{
  /* Take the semaphore (perhaps waiting) */

  while (sem_wait(&dmac->chsem) != 0)
    {
      /* The only case that an error should occur here is if the wait was
       * awakened by a signal.
       */

      ASSERT(errno == EINTR);
    }
}

static inline void sam_givechsem(struct sam_dmac_s *dmac)
{
  (void)sem_post(&dmac->chsem);
}

/****************************************************************************
 * Name: sam_takedsem() and sam_givedsem()
 *
 * Description:
 *   Used to wait for availability of descriptors in the descriptor table.
 *
 ****************************************************************************/

static void sam_takedsem(struct sam_dmac_s *dmac)
{
  /* Take the semaphore (perhaps waiting) */

  while (sem_wait(&dmac->dsem) != 0)
    {
      /* The only case that an error should occur here is if the wait was
       * awakened by a signal.
       */

      ASSERT(errno == EINTR);
    }
}

static inline void sam_givedsem(struct sam_dmac_s *dmac)
{
  (void)sem_post(&dmac->dsem);
}

/****************************************************************************
 * Name: sam_getdmac
 *
 * Description:
 *  Read a global DMAC register
 *
 ****************************************************************************/

static inline uint32_t sam_getdmac(struct sam_dmac_s *dmac,
                                   unsigned int offset)
{
  return getreg32(dmac->base + offset);
}

/****************************************************************************
 * Name: sam_putdmac
 *
 * Description:
 *  Write a value to a global DMAC register
 *
 ****************************************************************************/

static inline void sam_putdmac(struct sam_dmac_s *dmac, uint32_t value,
                               unsigned int offset)
{
  putreg32(value, dmac->base + offset);
}

/****************************************************************************
 * Name: sam_getdmach
 *
 * Description:
 *  Read a DMAC channel register
 *
 ****************************************************************************/

static inline uint32_t sam_getdmach(struct sam_dmach_s *dmach,
                                    unsigned int offset)
{
  return getreg32(dmach->base + offset);
}

/****************************************************************************
 * Name: sam_putdmach
 *
 * Description:
 *  Write a value to a DMAC channel register
 *
 ****************************************************************************/

static inline void sam_putdmach(struct sam_dmach_s *dmach, uint32_t value,
                                unsigned int offset)
{
  putreg32(value, dmach->base + offset);
}

/****************************************************************************
 * Name: sam_controller
 *
 * Description:
 *    Given a DMA channel instance, return a pointer to the parent DMA
 *    controller instance.
 *
 ****************************************************************************/

static inline struct sam_dmac_s *sam_controller(struct sam_dmach_s *dmach)
{
#if defined(CONFIG_SAMA5_DMAC0) && defined(CONFIG_SAMA5_DMAC1)
  return dmach->dmac ? &g_dmac1 : &g_dmac0;
#elif defined(CONFIG_SAMA5_DMAC0)
  return &g_dmac0;
#else
  return &g_dmac1;
#endif
}

/****************************************************************************
 * Name: sam_fifocfg
 *
 * Description:
 *  Decode the FIFO config from the flags
 *
 ****************************************************************************/

static inline uint32_t sam_fifocfg(struct sam_dmach_s *dmach)
{
  unsigned int ndx;

  ndx = (dmach->flags & DMACH_FLAG_FIFOCFG_MASK) >> DMACH_FLAG_FIFOCFG_SHIFT;
  DEBUGASSERT(ndx < 3);
  return g_fifocfg[ndx];
}

/****************************************************************************
 * Name: sam_txcfg
 *
 * Description:
 *  Decode the the flags to get the correct CFG register bit settings for
 *  a transmit (memory to peripheral) transfer.
 *
 ****************************************************************************/

static inline uint32_t sam_txcfg(struct sam_dmach_s *dmach)
{
  uint32_t regval;

  /* Set transfer (memory to peripheral) DMA channel configuration register */

  regval  = (((dmach->flags & DMACH_FLAG_MEMPID_MASK) >> DMACH_FLAG_MEMPID_SHIFT)
    << DMAC_CH_CFG_SRCPER_SHIFT);
  regval |=   (dmach->flags & DMACH_FLAG_MEMH2SEL) != 0 ? DMAC_CH_CFG_SRCH2SEL : 0;
  regval |= (((dmach->flags & DMACH_FLAG_PERIPHPID_MASK) >> DMACH_FLAG_PERIPHPID_SHIFT)
    << DMAC_CH_CFG_DSTPER_SHIFT);
  regval |=   (dmach->flags & DMACH_FLAG_PERIPHH2SEL) != 0 ? DMAC_CH_CFG_DSTH2SEL : 0;
  regval |= sam_fifocfg(dmach);
  return regval;
}

/****************************************************************************
 * Name: sam_rxcfg
 *
 * Description:
 *  Decode the the flags to get the correct CFG register bit settings for
 *  a receive (peripheral to memory) transfer.
 *
 ****************************************************************************/

static inline uint32_t sam_rxcfg(struct sam_dmach_s *dmach)
{
  uint32_t regval;

  /* Set received (peripheral to memory) DMA channel config */

  regval  = (((dmach->flags & DMACH_FLAG_PERIPHPID_MASK) >> DMACH_FLAG_PERIPHPID_SHIFT)
    << DMAC_CH_CFG_SRCPER_SHIFT);
  regval |=   (dmach->flags & DMACH_FLAG_PERIPHH2SEL) != 0 ? DMAC_CH_CFG_SRCH2SEL : 0;
  regval |= (((dmach->flags & DMACH_FLAG_MEMPID_MASK) >> DMACH_FLAG_MEMPID_SHIFT)
    << DMAC_CH_CFG_DSTPER_SHIFT);
  regval |=   (dmach->flags & DMACH_FLAG_MEMH2SEL) != 0 ? DMAC_CH_CFG_DSTH2SEL : 0;
  regval |= sam_fifocfg(dmach);
  return regval;
}

/****************************************************************************
 * Name: sam_txctrlabits
 *
 * Description:
 *  Decode the the flags to get the correct CTRLA register bit settings for
 *  a transmit (memory to peripheral) transfer.  These are only the "fixed"
 *  CTRLA values and  need to be updated with the actual transfer size before
 *  being written to CTRLA sam_txctrla).
 *
 ****************************************************************************/

static inline uint32_t sam_txctrlabits(struct sam_dmach_s *dmach)
{
  uint32_t regval;
  unsigned int ndx;

  DEBUGASSERT(dmach);

  /* Since this is a transmit, the source is described by the memory selections.
   * Set the source width (memory width).
   */

  ndx = (dmach->flags & DMACH_FLAG_MEMWIDTH_MASK) >> DMACH_FLAG_MEMWIDTH_SHIFT;
  DEBUGASSERT(ndx < 4);
  regval = g_srcwidth[ndx];

  /* Set the source chuck size (memory chunk size) */

  if ((dmach->flags & DMACH_FLAG_MEMCHUNKSIZE) == DMACH_FLAG_MEMCHUNKSIZE_4)
    {
      regval |= DMAC_CH_CTRLA_SCSIZE_4;
    }
#if 0 /* DMAC_CH_CTRLA_SCSIZE_1 is zero */
  else
    {
      regval |= DMAC_CH_CTRLA_SCSIZE_1;
    }
#endif

  /* Since this is a transmit, the destination is described by the peripheral selections.
   * Set the destination width (peripheral width).
   */

  ndx = (dmach->flags & DMACH_FLAG_PERIPHWIDTH_MASK) >> DMACH_FLAG_PERIPHWIDTH_SHIFT;
  DEBUGASSERT(ndx < 4);
  regval |= g_destwidth[ndx];

  /* Set the destination chuck size (peripheral chunk size) */

  if ((dmach->flags & DMACH_FLAG_PERIPHCHUNKSIZE) == DMACH_FLAG_PERIPHCHUNKSIZE_4)
    {
      regval |= DMAC_CH_CTRLA_DCSIZE_4;
    }
#if 0 /* DMAC_CH_CTRLA_DCSIZE_1 is zero */
  else
    {
      regval |= DMAC_CH_CTRLA_DCSIZE_1;
    }
#endif

  return regval;
}

/****************************************************************************
 * Name: sam_txctrla
 *
 * Description:
 *  Or in the variable CTRLA bits
 *
 ****************************************************************************/

static inline uint32_t sam_txctrla(struct sam_dmach_s *dmach,
                                   uint32_t dmasize, uint32_t txctrlabits)
{
  /* Set the buffer transfer size field.  This is the number of transfers to
   * be performed, that is, the number of source width transfers to perform.
   */

  /* Adjust the the source transfer size for the source chunk size (memory
   * chunk size)
   */

  if ((dmach->flags & DMACH_FLAG_MEMCHUNKSIZE) == DMACH_FLAG_MEMCHUNKSIZE_4)
    {
      dmasize >>= 2;
    }

  DEBUGASSERT(dmasize <= DMAC_CH_CTRLA_BTSIZE_MAX);
  return (txctrlabits & ~DMAC_CH_CTRLA_BTSIZE_MASK) |
         (dmasize << DMAC_CH_CTRLA_BTSIZE_SHIFT);
}

/****************************************************************************
 * Name: sam_rxctrlabits
 *
 * Description:
 *  Decode the the flags to get the correct CTRLA register bit settings for
 *  a read (peripheral to memory) transfer. These are only the "fixed" CTRLA
 *  values and need to be updated with the actual transfer size before being
 *  written to CTRLA sam_rxctrla).
 *
 ****************************************************************************/

static inline uint32_t sam_rxctrlabits(struct sam_dmach_s *dmach)
{
  uint32_t regval;
  unsigned int ndx;

  DEBUGASSERT(dmach);

  /* Since this is a receive, the source is described by the peripheral
   * selections. Set the source width (peripheral width).
   */

  ndx = (dmach->flags & DMACH_FLAG_PERIPHWIDTH_MASK)
    >> DMACH_FLAG_PERIPHWIDTH_SHIFT;

  DEBUGASSERT(ndx < 4);
  regval = g_srcwidth[ndx];

  /* Set the source chuck size (peripheral chunk size) */

  if ((dmach->flags & DMACH_FLAG_PERIPHCHUNKSIZE) ==
       DMACH_FLAG_PERIPHCHUNKSIZE_4)
    {
      regval |= DMAC_CH_CTRLA_SCSIZE_4;
    }
#if 0 /* DMAC_CH_CTRLA_SCSIZE_1 is zero */
  else
    {
      regval |= DMAC_CH_CTRLA_SCSIZE_1;
    }
#endif

  /* Since this is a receive, the destination is described by the memory
   * selections. Set the destination width (memory width).
   */

  ndx = (dmach->flags & DMACH_FLAG_MEMWIDTH_MASK)
    >> DMACH_FLAG_MEMWIDTH_SHIFT;

  DEBUGASSERT(ndx < 4);
  regval |= g_destwidth[ndx];

  /* Set the destination chuck size (memory chunk size) */

  if ((dmach->flags & DMACH_FLAG_MEMCHUNKSIZE) == DMACH_FLAG_MEMCHUNKSIZE_4)
    {
      regval |= DMAC_CH_CTRLA_DCSIZE_4;
    }
#if 0 /* DMAC_CH_CTRLA_DCSIZE_1 is zero */
  else
    {
      regval |= DMAC_CH_CTRLA_DCSIZE_1;
    }
#endif

  return regval;
}

/****************************************************************************
 * Name: sam_rxctrla
 *
 * Description:
 *  'OR' in the variable CTRLA bits
 *
 ****************************************************************************/

static inline uint32_t sam_rxctrla(struct sam_dmach_s *dmach,
                                   uint32_t dmasize, uint32_t txctrlabits)
{
  /* Set the buffer transfer size field.  This is the number of transfers to
   * be performed, that is, the number of source width transfers to perform.
   */

  /* Adjust the the source transfer size for the source chunk size (peripheral
   * chunk size)
   */

  if ((dmach->flags & DMACH_FLAG_PERIPHCHUNKSIZE) ==
      DMACH_FLAG_PERIPHCHUNKSIZE_4)
    {
      dmasize >>= 2;
    }

  DEBUGASSERT(dmasize <= DMAC_CH_CTRLA_BTSIZE_MAX);
  return (txctrlabits & ~DMAC_CH_CTRLA_BTSIZE_MASK) |
         (dmasize << DMAC_CH_CTRLA_BTSIZE_SHIFT);
}

/****************************************************************************
 * Name: sam_txctrlb
 *
 * Description:
 *  Decode the the flags to get the correct CTRLB register bit settings for
 *  a transmit (memory to peripheral) transfer.
 *
 ****************************************************************************/

static inline uint32_t sam_txctrlb(struct sam_dmach_s *dmach)
{
  uint32_t regval;

  /* Assume that we will not be using the link list and disable the source
   * and destination descriptors.  The default will be single transfer mode.
   */

  regval = DMAC_CH_CTRLB_BOTHDSCR;

  /* Select flow control (even if the channel doesn't support it).  The
   * naming convention from TX is memory to peripheral, but that is really
   * be determined by bits in the DMA flags.
   */

  /* Is the memory source really a peripheral? */

  if ((dmach->flags & DMACH_FLAG_MEMISPERIPH) != 0)
    {
      /* Yes.. is the peripheral destination also a peripheral? */

      if ((dmach->flags & DMACH_FLAG_PERIPHISPERIPH) != 0)
        {
          /* Yes.. Use peripheral-to-peripheral flow control */

          regval |= DMAC_CH_CTRLB_FC_P2P;
        }
      else
        {
          /* No.. Use peripheral-to-memory flow control */

          regval |= DMAC_CH_CTRLB_FC_P2M;
        }
    }
  else
    {
      /* No, the source is memory.  Is the peripheral destination a
       * peripheral
       */

      if ((dmach->flags & DMACH_FLAG_PERIPHISPERIPH) != 0)
        {
          /* Yes.. Use memory-to-peripheral flow control */

          regval |= DMAC_CH_CTRLB_FC_M2P;
        }
      else
        {
          /* No.. Use memory-to-memory flow control */

          regval |= DMAC_CH_CTRLB_FC_M2M;
        }
    }

  /* Select source address incrementing */

  if ((dmach->flags & DMACH_FLAG_MEMINCREMENT) == 0)
    {
      regval |= DMAC_CH_CTRLB_SRCINCR_FIXED;
    }

  /* Select destination address incrementing */

  if ((dmach->flags & DMACH_FLAG_PERIPHINCREMENT) == 0)
    {
      regval |= DMAC_CH_CTRLB_DSTINCR_FIXED;
    }

  return regval;
}

/****************************************************************************
 * Name: sam_rxctrlb
 *
 * Description:
 *  Decode the the flags to get the correct CTRLB register bit settings for
 *  a receive (peripheral to memory) transfer.
 *
 ****************************************************************************/

static inline uint32_t sam_rxctrlb(struct sam_dmach_s *dmach)
{
  uint32_t regval;

  /* Assume that we will not be using the link list and disable the source
   * and destination descriptors.  The default will be single transfer mode.
   */

  regval = DMAC_CH_CTRLB_BOTHDSCR;

  /* Select flow control (even if the channel doesn't support it).  The
   * naming convention from RX is peripheral to memory, but that is really
   * be determined by bits in the DMA flags.
   */

  /* Is the peripheral source really a peripheral? */

  if ((dmach->flags & DMACH_FLAG_PERIPHISPERIPH) != 0)
    {
      /* Yes.. is the memory destination also a peripheral? */

      if ((dmach->flags & DMACH_FLAG_MEMISPERIPH) != 0)
        {
          /* Yes.. Use peripheral-to-peripheral flow control */

          regval |= DMAC_CH_CTRLB_FC_P2P;
        }
      else
        {
          /* No.. Use peripheral-to-memory flow control */

          regval |= DMAC_CH_CTRLB_FC_P2M;
        }
    }
  else
    {
      /* No, the peripheral source is memory.  Is the memory destination
       * a peripheral
       */

      if ((dmach->flags & DMACH_FLAG_MEMISPERIPH) != 0)
        {
          /* Yes.. Use memory-to-peripheral flow control */

          regval |= DMAC_CH_CTRLB_FC_M2P;
        }
      else
        {
          /* No.. Use memory-to-memory flow control */

          regval |= DMAC_CH_CTRLB_FC_M2M;
        }
    }

  /* Select source address incrementing */

  if ((dmach->flags & DMACH_FLAG_PERIPHINCREMENT) == 0)
    {
      regval |= DMAC_CH_CTRLB_SRCINCR_FIXED;
    }

  /* Select address incrementing */

  if ((dmach->flags & DMACH_FLAG_MEMINCREMENT) == 0)
    {
      regval |= DMAC_CH_CTRLB_DSTINCR_FIXED;
    }

  return regval;
}

/****************************************************************************
 * Name: sam_allocdesc
 *
 * Description:
 *  Allocate and add one descriptor to the DMA channel's link list.
 *
 *  NOTE: link list entries are freed by the DMA interrupt handler.
 *  However, since the setting/clearing of the 'in use' indication is
 *  atomic, no special actions need be performed.  It would be a good thing
 *  to add logic to handle the case where all of the entries are exhausted
 *  and we could wait for some to be freed by the interrupt handler.
 *
 ****************************************************************************/

static struct dma_linklist_s *
sam_allocdesc(struct sam_dmach_s *dmach, struct dma_linklist_s *prev,
              uint32_t saddr, uint32_t daddr, uint32_t ctrla, uint32_t ctrlb)
{
  struct sam_dmac_s *dmac = sam_controller(dmach);
  struct dma_linklist_s *desc = NULL;
  int i;

  /* Sanity check -- saddr == 0 is the indication that the link is unused.
   * Obviously setting it to zero would break that usage.
   */

#ifdef CONFIG_DEBUG
  if (saddr != 0)
#endif
    {
      /* Table a descriptor table semaphore count.  When we get one, then there
       * is at least one free descriptor in the table and it is ours.
       */

      sam_takedsem(dmac);

      /* Examine each link list entry to find an available one -- i.e., one
       * with saddr == 0.  That saddr field is set to zero by the DMA transfer
       * complete interrupt handler.  The following should be safe because
       * that is an atomic operation.
       */

      sam_takechsem(dmac);
      for (i = 0; i < CONFIG_SAMA5_NLLDESC; i++)
        {
          if (dmac->desc[i].saddr == 0)
            {
              /* We have it.  Initialize the new link list entry */

              desc        = &dmac->desc[i];
              desc->saddr = saddr;  /* Source address */
              desc->daddr = daddr;  /* Destination address */
              desc->ctrla = ctrla;  /* Control A value */
              desc->ctrlb = ctrlb;  /* Control B value */
              desc->dscr  = 0;      /* Next descriptor address */

              /* And then hook it at the tail of the link list */

              if (!prev)
                {
                  /* There is no previous link.  This is the new head of
                   * the list
                   */

                  DEBUGASSERT(dmach->llhead == NULL && dmach->lltail == NULL);
                  dmach->llhead = desc;
                }
              else
                {
                  DEBUGASSERT(dmach->llhead != NULL && dmach->lltail == prev);

                  /* When the second link is added to the list, that is the
                   * cue that we are going to do the link list transfer.
                   *
                   * Enable the source and destination descriptor in the link
                   * list entry just before this one.  We assume that both
                   * source and destination buffers are non-continuous, but
                   * this should work even if that is not the case.
                   */

                  prev->ctrlb &= ~DMAC_CH_CTRLB_BOTHDSCR;

                  /* Link the previous tail to the new tail */

                  prev->dscr = (uint32_t)desc;
                }

              /* In any event, this is the new tail of the list.  The source
               * and destination descriptors must be disabled for the last entry
               * in the link list. */

              desc->ctrlb  |= DMAC_CH_CTRLB_BOTHDSCR;
              dmach->lltail = desc;
              break;
            }
        }

      /* Because we hold a count from the counting semaphore, the above
       * search loop should always be successful.
       */

      sam_givechsem(dmac);
      DEBUGASSERT(desc != NULL);
    }

  return desc;
}

/****************************************************************************
 * Name: sam_freelinklist
 *
 * Description:
 *  Free all descriptors in the DMA channel's link list.
 *
 *  NOTE: Called from the DMA interrupt handler.
 *
 ****************************************************************************/

static void sam_freelinklist(struct sam_dmach_s *dmach)
{
  struct sam_dmac_s *dmac = sam_controller(dmach);
  struct dma_linklist_s *desc;
  struct dma_linklist_s *dscr;

  /* Get the head of the link list and detach the link list from the DMA
   * channel
   */

  desc             = dmach->llhead;
  dmach->llhead    = NULL;
  dmach->lltail    = NULL;

  /* Reset each descriptor in the link list (thereby freeing them) */

  while (desc != NULL)
    {
      dscr = (struct dma_linklist_s *)desc->dscr;
      DEBUGASSERT(desc->saddr != 0);
      memset(desc, 0, sizeof(struct dma_linklist_s));
      sam_givedsem(dmac);
      desc = dscr;
    }
}

/****************************************************************************
 * Name: sam_txbuffer
 *
 * Description:
 *   Configure DMA for transmit of one buffer (memory to peripheral).  This
 *   function may be called multiple times to handle large and/or dis-
 *   continuous transfers.
 *
 ****************************************************************************/

static int sam_txbuffer(struct sam_dmach_s *dmach, uint32_t paddr,
                        uint32_t maddr, size_t nbytes)
{
  uint32_t regval;
  uint32_t ctrla;
  uint32_t ctrlb;

  /* If we are appending a buffer to a linklist, then re-use the CTRLA/B
   * values.  Otherwise, create them from the properties of the transfer.
   */

  if (dmach->llhead)
    {
      regval = dmach->llhead->ctrla;
      ctrlb  = dmach->llhead->ctrlb;
    }
  else
    {
      regval = sam_txctrlabits(dmach);
      ctrlb  = sam_txctrlb(dmach);
    }

  ctrla = sam_txctrla(dmach, regval, nbytes);

  /* Add the new link list entry */

  if (!sam_allocdesc(dmach, dmach->lltail, maddr, paddr, ctrla, ctrlb))
    {
      return -ENOMEM;
    }

  /* Pre-calculate the transmit CFG register setting (it won't be used until
   * the DMA is started).
   */

  dmach->cfg = sam_txcfg(dmach);
  return OK;
}

/****************************************************************************
 * Name: sam_rxbuffer
 *
 * Description:
 *   Configure DMA for receipt of one buffer (peripheral to memory).  This
 *   function may be called multiple times to handle large and/or dis-
 *   continuous transfers.
 *
 ****************************************************************************/

static int sam_rxbuffer(struct sam_dmach_s *dmach, uint32_t paddr,
                        uint32_t maddr, size_t nbytes)
{
  uint32_t regval;
  uint32_t ctrla;
  uint32_t ctrlb;

  /* If we are appending a buffer to a linklist, then re-use the CTRLA/B
   * values.  Otherwise, create them from the properties of the transfer.
   */

  if (dmach->llhead)
    {
      regval = dmach->llhead->ctrla;
      ctrlb  = dmach->llhead->ctrlb;
    }
  else
    {
      regval = sam_rxctrlabits(dmach);
      ctrlb  = sam_rxctrlb(dmach);
    }

   ctrla  = sam_rxctrla(dmach, regval, nbytes);

  /* Add the new link list entry */

  if (!sam_allocdesc(dmach, dmach->lltail, paddr, maddr, ctrla, ctrlb))
    {
      return -ENOMEM;
    }

  /* Pre-calculate the receive CFG register setting (it won't be used until
   * the DMA is started).
   */

  dmach->cfg = sam_rxcfg(dmach);
  return OK;
}

/****************************************************************************
 * Name: sam_single
 *
 * Description:
 *   Start a single buffer DMA.
 *
 ****************************************************************************/

static inline int sam_single(struct sam_dmach_s *dmach)
{
  struct sam_dmac_s *dmac = sam_controller(dmach);
  struct dma_linklist_s *llhead = dmach->llhead;

  /* Clear any pending interrupts from any previous DMAC transfer by reading
   * the interrupt status register.
   */

  (void)sam_getdmac(dmac, SAM_DMAC_EBCISR_OFFSET);

  /* Write the starting source address in the SADDR register */

  DEBUGASSERT(llhead != NULL && llhead->saddr != 0);
  sam_putdmach(dmach, llhead->saddr, SAM_DMAC_CH_SADDR_OFFSET);

  /* Write the starting destination address in the DADDR register */

  sam_putdmach(dmach, llhead->daddr, SAM_DMAC_CH_DADDR_OFFSET);

  /* Set up the CTRLA register */

  sam_putdmach(dmach, llhead->ctrla, SAM_DMAC_CH_CTRLA_OFFSET);

  /* Set up the CTRLB register */

  sam_putdmach(dmach, llhead->ctrlb, SAM_DMAC_CH_CTRLA_OFFSET);

  /* Both the DST and SRC DSCR bits should be '1' in CTRLB */

  DEBUGASSERT((llhead->ctrlb & DMAC_CH_CTRLB_BOTHDSCR) ==
              DMAC_CH_CTRLB_BOTHDSCR);

  /* Set up the CFG register */

  sam_putdmach(dmach, dmach->cfg, SAM_DMAC_CH_CFG_OFFSET);

  /* Enable the channel by writing a ‘1’ to the CHER enable bit */

  sam_putdmac(dmac, DMAC_CHER_ENA(dmach->chan), SAM_DMAC_CHER_OFFSET);

  /* The DMA has been started. Once the transfer completes, hardware sets
   * the interrupts and disables the channel.  We will received buffer
   * complete and* transfer complete interrupts.
   *
   * Enable error, buffer complete and transfer complete interrupts.
   * (Since there is only a single buffer, we don't need the buffer
   * complete interrupt).
   */

  sam_putdmac(dmac, DMAC_EBC_CBTCINTS(dmach->chan), SAM_DMAC_EBCIER_OFFSET);
  return OK;
}

/****************************************************************************
 * Name: sam_multiple
 *
 * Description:
 *   Start a multiple buffer DMA.
 *
 ****************************************************************************/

static inline int sam_multiple(struct sam_dmach_s *dmach)
{
  struct sam_dmac_s *dmac = sam_controller(dmach);
  struct dma_linklist_s *llhead = dmach->llhead;

  DEBUGASSERT(llhead != NULL && llhead->saddr != 0);

  /* Check the first and last CTRLB values */

  DEBUGASSERT((llhead->ctrlb & DMAC_CH_CTRLB_BOTHDSCR) == 0);
  DEBUGASSERT((dmach->lltail->ctrlb & DMAC_CH_CTRLB_BOTHDSCR) ==
              DMAC_CH_CTRLB_BOTHDSCR);

  /* Clear any pending interrupts from any previous DMAC transfer by reading
   * the status register
   */

  (void)sam_getdmac(dmac, SAM_DMAC_EBCISR_OFFSET);

  /* Set up the initial CTRLB register (to enable descriptors) */

  sam_putdmach(dmach, llhead->ctrlb, SAM_DMAC_CH_CTRLA_OFFSET);

  /* Set up the CTRLB register */

  sam_putdmach(dmach, llhead->ctrlb, SAM_DMAC_CH_CTRLA_OFFSET);

  /* Write the channel configuration information into the CFG register */

  sam_putdmach(dmach, dmach->cfg, SAM_DMAC_CH_CFG_OFFSET);

  /* Program the DSCR register with the pointer to the firstlink list entry. */

  sam_putdmach(dmach, (uint32_t)llhead, SAM_DMAC_CH_DSCR_OFFSET);

  /* Finally, enable the channel by writing a ‘1’ to the CHER enable */

  sam_putdmac(dmac, DMAC_CHER_ENA(dmach->chan), SAM_DMAC_CHER_OFFSET);

  /* As each buffer of data is transferred, the CTRLA register is written
   * back into the link list entry.  The CTRLA contains updated BTSIZE and
   * DONE bits.  Additionally, the CTRLA DONE bit is asserted when the
   * buffer transfer has completed.
   *
   * The DMAC transfer continues until the CTRLB register disables the
   * descriptor (DSCR bits) registers at the final buffer tranfer.
   *
   * Enable error, buffer complete and transfer complete interrupts.  We
   * don't really need the buffer complete interrupts, but we will take them
   * just to handle stall conditions.
   */

  sam_putdmac(dmac, DMAC_EBC_CHANINTS(dmach->chan), SAM_DMAC_EBCIER_OFFSET);
  return OK;
}

/****************************************************************************
 * Name: sam_dmaterminate
 *
 * Description:
 *   Terminate the DMA transfer and disable the DMA channel
 *
 ****************************************************************************/

static void sam_dmaterminate(struct sam_dmach_s *dmach, int result)
{
  struct sam_dmac_s *dmac = sam_controller(dmach);

  /* Disable all channel interrupts */

  sam_putdmac(dmac, DMAC_EBC_CHANINTS(dmach->chan), SAM_DMAC_EBCIDR_OFFSET);

  /* Disable the channel by writing one to the write-only channel disable
   * register.
   */

  sam_putdmac(dmac, DMAC_CHDR_DIS(dmach->chan), SAM_DMAC_CHDR_OFFSET);

  /* Free the linklist */

  sam_freelinklist(dmach);

  /* Perform the DMA complete callback */

  if (dmach->callback)
    {
      dmach->callback((DMA_HANDLE)dmach, dmach->arg, result);
    }

  dmach->callback = NULL;
  dmach->arg      = NULL;
}

/****************************************************************************
 * Name: sam_dmac_interrupt
 *
 * Description:
 *  DMA interrupt handler
 *
 ****************************************************************************/

static int sam_dmac_interrupt(struct sam_dmac_s *dmac)
{
  struct sam_dmach_s *dmach;
  unsigned int chndx;
  uint32_t regval;

  /* Get the DMAC status register value.  Ignore all masked interrupt
   * status bits.
   */

  regval = sam_getdmac(dmac, SAM_DMAC_EBCISR_OFFSET) &
           sam_getdmac(dmac, SAM_DMAC_EBCIMR_OFFSET);

  /* Check if the any transfer has completed */

  if (regval & DMAC_EBC_BTC_MASK)
    {
      /* Yes.. Check each bit  to see which channel has interrupted */

      for (chndx = 0; chndx < SAM_NDMACHAN; chndx++)
        {
          /* Are any interrupts pending for this channel? */

          if ((regval & DMAC_EBC_CHANINTS(chndx)) != 0)
            {
              dmach = &dmac->dmach[chndx];

              /* Yes.. Did an error occur? */

              if ((regval & DMAC_EBC_ERR(chndx)) != 0)
                {
                   /* Yes... Terminate the transfer with an error? */

                  sam_dmaterminate(dmach, -EIO);
                }

              /* Is the transfer complete? */

              else if ((regval & DMAC_EBC_CBTC(chndx)) != 0)
               {
                  /* Yes.. Terminate the transfer with success */

                  sam_dmaterminate(dmach, OK);
                }

              /* Otherwise, this must be a Bufffer Transfer Complete (BTC)
               * interrupt as part of a multiple buffer transfer.
               */

              else /* f ((regval & DMAC_EBC_BTC(chndx)) != 0) */
                {
                  /* Write the KEEPON field to clear the STALL states */

                  sam_putdmac(dmac, DMAC_CHER_KEEP(dmach->chan),
                              SAM_DMAC_CHER_OFFSET);
                }
            }
        }
    }

  return OK;
}

/****************************************************************************
 * Name: sam_dmac0_interrupt and sam_dmac1_interrupt
 *
 * Description:
 *  DMA interrupt handler
 *
 ****************************************************************************/

#ifdef CONFIG_SAMA5_DMAC0
static int sam_dmac0_interrupt(int irq, void *context)
{
  return sam_dmac_interrupt(&g_dmac0);
}
#endif

#ifdef CONFIG_SAMA5_DMAC1
static int sam_dmac1_interrupt(int irq, void *context)
{
  return sam_dmac_interrupt(&g_dmac1);
}
#endif

/****************************************************************************
 * Name: sam_dmainitialize
 *
 * Description:
 *   Initialize the DMA subsystem
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void sam_dmainitialize(struct sam_dmac_s *dmac)
{
  /* Disable all DMA interrupts */

  sam_putdmac(dmac, DMAC_EBC_ALLINTS, SAM_DMAC_EBCIDR_OFFSET);

  /* Disable all DMA channels */

  sam_putdmac(dmac, DMAC_CHDR_DIS_ALL, SAM_DMAC_CHDR_OFFSET);

  /* Enable the DMA controller */

  sam_putdmac(dmac,DMAC_EN_ENABLE, SAM_DMAC_EN_OFFSET);

  /* Initialize semaphores */

  sem_init(&dmac->chsem, 0, 1);
  sem_init(&dmac->dsem, 0, SAM_NDMACHAN);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_dmainitialize
 *
 * Description:
 *   Initialize the DMA subsystem
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void weak_function up_dmainitialize(void)
{
#ifdef CONFIG_SAMA5_DMAC0
  dmallvdbg("Iinitialize DMAC0\n");

  /* Enable peripheral clock */

  sam_dmac0_enableclk();

  /* Attach DMA interrupt vector */

  (void)irq_attach(SAM_IRQ_DMAC0, sam_dmac0_interrupt);

  /* Initialize the controller */

  sam_dmainitialize(&g_dmac0);

  /* Enable the IRQ at the AIC (still disabled at the DMA controller) */

  up_enable_irq(SAM_IRQ_DMAC0);
#endif

#ifdef CONFIG_SAMA5_DMAC1
  dmallvdbg("Iinitialize DMAC1\n");

  /* Enable peripheral clock */

  sam_dmac1_enableclk();

  /* Attach DMA interrupt vector */

  (void)irq_attach(SAM_IRQ_DMAC1, sam_dmac1_interrupt);

  /* Initialize the controller */

  sam_dmainitialize(&g_dmac1);

  /* Enable the IRQ at the AIC (still disabled at the DMA controller) */

  up_enable_irq(SAM_IRQ_DMAC1);
#endif
}

/****************************************************************************
 * Name: sam_dmachannel
 *
 * Description:
 *   Allocate a DMA channel.  This function sets aside a DMA channel with
 *   the required FIFO size and flow control capabilities (determined by
 *   dma_flags) then  gives the caller exclusive access to the DMA channel.
 *
 *   The naming convention in all of the DMA interfaces is that one side is
 *   the 'peripheral' and the other is 'memory'.  Howerver, the interface
 *   could still be used if, for example, both sides were memory although
 *   the naming would be awkward.
 *
 * Returned Value:
 *   If a DMA channel if the required FIFO size is available, this function
 *   returns a non-NULL, void* DMA channel handle.  NULL is returned on any
 *   failure.
 *
 ****************************************************************************/

DMA_HANDLE sam_dmachannel(uint8_t dmacno, uint32_t chflags)
{
  struct sam_dmac_s *dmac;
  struct sam_dmach_s *dmach;
  unsigned int chndx;

  /* Pick the DMA controller */

#ifdef CONFIG_SAMA5_DMAC0
  if (dmacno == 0)
    {
      dmac = &g_dmac0;
    }
  else
#endif

#ifdef CONFIG_SAMA5_DMAC0
  if (dmacno == 1)
    {
      dmac = &g_dmac1;
    }
  else
#endif

    {
      dmadbg("Bad DMAC number: %d\n", dmacno);
      DEBUGPANIC();
      return (DMA_HANDLE)NULL;
    }

  /* Search for an available DMA channel with at least the requested FIFO
   * size.
   */

  dmach = NULL;
  sam_takechsem(dmac);
  for (chndx = 0; chndx < SAM_NDMACHAN; chndx++)
    {
      struct sam_dmach_s *candidate = &dmac->dmach[chndx];
      if (!candidate->inuse)
        {
          dmach         = candidate;
          dmach->inuse  = true;

          /* Read the status register to clear any pending interrupts on the
           * channel
           */

          (void)sam_getdmac(dmac, SAM_DMAC_EBCISR_OFFSET);

          /* Disable the channel by writing one to the write-only channel
           * disable register
           */

          sam_putdmac(dmac,DMAC_CHDR_DIS(chndx), SAM_DMAC_CHDR_OFFSET);

          /* See the DMA channel flags. */

          dmach->flags = chflags;
          break;
        }
    }

  sam_givechsem(dmac);

  dmavdbg("dmacno: %d chflags: %08x returning dmach: %p\n",
          (int)dmacno, (int)chflags, dmach);
  return (DMA_HANDLE)dmach;
}

/****************************************************************************
 * Name: sam_dmafree
 *
 * Description:
 *   Release a DMA channel.  NOTE:  The 'handle' used in this argument must
 *   NEVER be used again until sam_dmachannel() is called again to re-gain
 *   a valid handle.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void sam_dmafree(DMA_HANDLE handle)
{
  struct sam_dmach_s *dmach = (struct sam_dmach_s *)handle;

  dmavdbg("dmach: %p\n", dmach);
  DEBUGASSERT((dmach != NULL) && (dmach->inuse));

  /* Mark the channel no longer in use.  Clearing the inuse flag is an atomic
   * operation and so should be safe.
   */

  dmach->flags = 0;
  dmach->inuse = false;                   /* No longer in use */
}

/****************************************************************************
 * Name: sam_dmatxsetup
 *
 * Description:
 *   Configure DMA for transmit of one buffer (memory to peripheral).  This
 *   function may be called multiple times to handle large and/or dis-
 *   continuous transfers.  Calls to sam_dmatxsetup() and sam_dmarxsetup()
 *   must not be intermixed on the same transfer, however.
 *
 ****************************************************************************/

int sam_dmatxsetup(DMA_HANDLE handle, uint32_t paddr, uint32_t maddr, size_t nbytes)
{
  struct sam_dmach_s *dmach = (struct sam_dmach_s *)handle;
  int ret = OK;

  dmavdbg("dmach: %p paddr: %08x maddr: %08x nbytes: %d\n",
          dmach, (int)paddr, (int)maddr, (int)nbytes);
  DEBUGASSERT(dmach);
  dmavdbg("llhead: %p lltail: %p\n", dmach->llhead, dmach->lltail);

  /* If this is a large transfer, break it up into smaller buffers */

  while (nbytes > DMAC_CH_CTRLA_BTSIZE_MAX)
    {
      /* Set up the maximum size transfer */

      ret = sam_txbuffer(dmach, paddr, maddr, DMAC_CH_CTRLA_BTSIZE_MAX);
      if (ret == OK);
        {
          /* Decrement the number of bytes left to transfer */

          nbytes -= DMAC_CH_CTRLA_BTSIZE_MAX;

          /* Increment the memory & peripheral address (if it is appropriate to
           * do do).
           */

          if ((dmach->flags & DMACH_FLAG_PERIPHINCREMENT) != 0)
            {
              paddr += DMAC_CH_CTRLA_BTSIZE_MAX;
            }

          if ((dmach->flags & DMACH_FLAG_MEMINCREMENT) != 0)
            {
              maddr += DMAC_CH_CTRLA_BTSIZE_MAX;
            }
        }
    }

  /* Then set up the final buffer transfer */

  if (ret == OK && nbytes > 0)
    {
      ret = sam_txbuffer(dmach, paddr, maddr, nbytes);
    }

  return ret;
}

/****************************************************************************
 * Name: sam_dmarxsetup
 *
 * Description:
 *   Configure DMA for receipt of one buffer (peripheral to memory).  This
 *   function may be called multiple times to handle large and/or dis-
 *   continuous transfers.  Calls to sam_dmatxsetup() and sam_dmarxsetup()
 *   must not be intermixed on the same transfer, however.
 *
 ****************************************************************************/

int sam_dmarxsetup(DMA_HANDLE handle, uint32_t paddr, uint32_t maddr,
                   size_t nbytes)
{
  struct sam_dmach_s *dmach = (struct sam_dmach_s *)handle;
  int ret = OK;

  dmavdbg("dmach: %p paddr: %08x maddr: %08x nbytes: %d\n",
          dmach, (int)paddr, (int)maddr, (int)nbytes);
  DEBUGASSERT(dmach);
  dmavdbg("llhead: %p lltail: %p\n", dmach->llhead, dmach->lltail);

  /* If this is a large transfer, break it up into smaller buffers */

  while (nbytes > DMAC_CH_CTRLA_BTSIZE_MAX)
    {
      /* Set up the maximum size transfer */

      ret = sam_rxbuffer(dmach, paddr, maddr, DMAC_CH_CTRLA_BTSIZE_MAX);
      if (ret == OK);
        {
          /* Decrement the number of bytes left to transfer */

          nbytes -= DMAC_CH_CTRLA_BTSIZE_MAX;

          /* Increment the memory & peripheral address (if it is appropriate to
           * do do).
           */

          if ((dmach->flags & DMACH_FLAG_PERIPHINCREMENT) != 0)
            {
              paddr += DMAC_CH_CTRLA_BTSIZE_MAX;
            }

          if ((dmach->flags & DMACH_FLAG_MEMINCREMENT) != 0)
            {
              maddr += DMAC_CH_CTRLA_BTSIZE_MAX;
            }
        }
    }

  /* Then set up the final buffer transfer */

  if (ret == OK && nbytes > 0)
    {
      ret = sam_rxbuffer(dmach, paddr, maddr, nbytes);
    }

  return ret;
}

/****************************************************************************
 * Name: sam_dmastart
 *
 * Description:
 *   Start the DMA transfer
 *
 ****************************************************************************/

int sam_dmastart(DMA_HANDLE handle, dma_callback_t callback, void *arg)
{
  struct sam_dmach_s *dmach = (struct sam_dmach_s *)handle;
  int ret = -EINVAL;

  dmavdbg("dmach: %p callback: %p arg: %p\n", dmach, callback, arg);
  DEBUGASSERT(dmach != NULL);

  /* Verify that the DMA has been setup (i.e., at least one entry in the
   * link list).
   */

  if (dmach->llhead)
    {
      /* Save the callback info.  This will be invoked whent the DMA commpletes */

      dmach->callback = callback;
      dmach->arg      = arg;

      /* Is this a single block transfer?  Or a multiple block tranfer? */

      if (dmach->llhead == dmach->lltail)
        {
          ret = sam_single(dmach);
        }
      else
        {
          ret = sam_multiple(dmach);
        }
    }

  return ret;
}

/****************************************************************************
 * Name: sam_dmastop
 *
 * Description:
 *   Cancel the DMA.  After sam_dmastop() is called, the DMA channel is
 *   reset and sam_dmarx/txsetup() must be called before sam_dmastart() can be
 *   called again
 *
 ****************************************************************************/

void sam_dmastop(DMA_HANDLE handle)
{
  struct sam_dmach_s *dmach = (struct sam_dmach_s *)handle;
  irqstate_t flags;

  dmavdbg("dmach: %p\n", dmach);
  DEBUGASSERT(dmach != NULL);

  flags = irqsave();
  sam_dmaterminate(dmach, -EINTR);
  irqrestore(flags);
}

/****************************************************************************
 * Name: sam_dmasample
 *
 * Description:
 *   Sample DMA register contents
 *
 * Assumptions:
 *   - DMA handle allocated by sam_dmachannel()
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_DMA
void sam_dmasample(DMA_HANDLE handle, struct sam_dmaregs_s *regs)
{
  struct sam_dmach_s *dmach = (struct sam_dmach_s *)handle;
  struct sam_dmac_s *dmac = sam_controller(dmach);
  irqstate_t flags;

  /* Sample global registers.  NOTE: reading EBCISR clears interrupts, but
   * that should be okay IF interrupts are enabled when this function is
   * called.  But there is a race condition where this instrumentation could
   * cause lost interrupts.
   */

  flags        = irqsave();
  regs->gcfg   = sam_getdmac(dmac, SAM_DMAC_GCFG_OFFSET);
  regs->en     = sam_getdmac(dmac, SAM_DMAC_EN_OFFSET);
  regs->sreq   = sam_getdmac(dmac, SAM_DMAC_SREQ_OFFSET);
  regs->creq   = sam_getdmac(dmac, SAM_DMAC_CREQ_OFFSET);
  regs->last   = sam_getdmac(dmac, SAM_DMAC_LAST_OFFSET);
  regs->ebcimr = sam_getdmac(dmac, SAM_DMAC_EBCIMR_OFFSET);
  regs->ebcisr = sam_getdmac(dmac, SAM_DMAC_EBCISR_OFFSET);
  regs->chsr   = sam_getdmac(dmac, SAM_DMAC_CHSR_OFFSET);
  regs->wpmr   = sam_getdmac(dmac, SAM_DMAC_WPMR_OFFSET);
  regs->wpsr   = sam_getdmac(dmac, SAM_DMAC_WPSR_OFFSET);

  /* Sample channel registers */

  regs->saddr  = sam_getdmach(dmach, SAM_DMAC_CH_SADDR_OFFSET);
  regs->daddr  = sam_getdmach(dmach, SAM_DMAC_CH_DADDR_OFFSET);
  regs->dscr   = sam_getdmach(dmach, SAM_DMAC_CH_DSCR_OFFSET);
  regs->ctrla  = sam_getdmach(dmach, SAM_DMAC_CH_CTRLA_OFFSET);
  regs->ctrlb  = sam_getdmach(dmach, SAM_DMAC_CH_CTRLB_OFFSET);
  regs->cfg    = sam_getdmach(dmach, SAM_DMAC_CH_CFG_OFFSET);
  regs->spip   = sam_getdmach(dmach, SAM_DMAC_CH_SPIP_OFFSET);
  regs->dpip   = sam_getdmach(dmach, SAM_DMAC_CH_DPIP_OFFSET);
  irqrestore(flags);
}
#endif /* CONFIG_DEBUG_DMA */

/****************************************************************************
 * Name: sam_dmadump
 *
 * Description:
 *   Dump previously sampled DMA register contents
 *
 * Assumptions:
 *   - DMA handle allocated by sam_dmachannel()
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_DMA
void sam_dmadump(DMA_HANDLE handle, const struct sam_dmaregs_s *regs,
                 const char *msg)
{
  struct sam_dmach_s *dmach = (struct sam_dmach_s *)handle;
  struct sam_dmac_s *dmac = sam_controller(dmach);

  dmadbg("%s\n", msg);
  dmadbg("  DMA Global Registers:\n");
  dmadbg("      GCFG[%08x]: %08x\n", dmac->base + SAM_DMAC_GCFG_OFFSET, regs->gcfg);
  dmadbg("        EN[%08x]: %08x\n", dmac->base + SAM_DMAC_EN_OFFSET, regs->en);
  dmadbg("      SREQ[%08x]: %08x\n", dmac->base + SAM_DMAC_SREQ_OFFSET, regs->sreq);
  dmadbg("      CREQ[%08x]: %08x\n", dmac->base + SAM_DMAC_CREQ_OFFSET, regs->creq);
  dmadbg("      LAST[%08x]: %08x\n", dmac->base + SAM_DMAC_LAST_OFFSET, regs->last);
  dmadbg("    EBCIMR[%08x]: %08x\n", dmac->base + SAM_DMAC_EBCIMR_OFFSET, regs->ebcimr);
  dmadbg("    EBCISR[%08x]: %08x\n", dmac->base + SAM_DMAC_EBCISR_OFFSET, regs->ebcisr);
  dmadbg("      CHSR[%08x]: %08x\n", dmac->base + SAM_DMAC_CHSR_OFFSET, regs->chsr);
  dmadbg("      WPMR[%08x]: %08x\n", dmac->base + SAM_DMAC_WPMR_OFFSET, regs->wpmr);
  dmadbg("      WPSR[%08x]: %08x\n", dmac->base + SAM_DMAC_WPSR_OFFSET, regs->wpsr);
  dmadbg("  DMA Channel Registers:\n");
  dmadbg("     SADDR[%08x]: %08x\n", dmach->base + SAM_DMAC_CH_SADDR_OFFSET, regs->saddr);
  dmadbg("     DADDR[%08x]: %08x\n", dmach->base + SAM_DMAC_CH_DADDR_OFFSET, regs->daddr);
  dmadbg("      DSCR[%08x]: %08x\n", dmach->base + SAM_DMAC_CH_DSCR_OFFSET, regs->dscr);
  dmadbg("     CTRLA[%08x]: %08x\n", dmach->base + SAM_DMAC_CH_CTRLA_OFFSET, regs->ctrla);
  dmadbg("     CTRLB[%08x]: %08x\n", dmach->base + SAM_DMAC_CH_CTRLB_OFFSET, regs->ctrlb);
  dmadbg("       CFG[%08x]: %08x\n", dmach->base + SAM_DMAC_CH_CFG_OFFSET, regs->cfg);
  dmadbg("      SPIP[%08x]: %08x\n", dmach->base + SAM_DMAC_CH_SPIP_OFFSET, regs->spip);
  dmadbg("      DPIP[%08x]: %08x\n", dmach->base + SAM_DMAC_CH_DPIP_OFFSET, regs->dpip);
}
#endif /* CONFIG_DEBUG_DMA */
#endif /* CONFIG_SAMA5_DMAC0 || CONFIG_SAMA5_DMAC1 */
