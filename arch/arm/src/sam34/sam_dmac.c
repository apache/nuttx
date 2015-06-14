/****************************************************************************
 * arch/arm/src/sam34/sam_dmac.c
 *
 *   Copyright (C) 2010, 2013-2014 Gregory Nutt. All rights reserved.
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
#include "sched/sched.h"
#include "chip.h"

#include "sam_dmac.h"
#include "sam_periphclks.h"
#include "chip/sam_pmc.h"
#include "chip/sam_dmac.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Condition out the whole file unless DMA is selected in the configuration */

#ifdef CONFIG_SAM34_DMAC0

/* If SAM3/4 support is enabled, then OS DMA support should also be enabled */

#ifndef CONFIG_ARCH_DMA
#  warning "SAM3/4 DMA enabled but CONFIG_ARCH_DMA disabled"
#endif

/* Check the number of link list descriptors to allocate */

#ifndef CONFIG_SAM34_NLLDESC
#  define CONFIG_SAM34_NLLDESC SAM34_NDMACHAN
#endif

#if CONFIG_SAM34_NLLDESC < SAM34_NDMACHAN
#  warning "At least SAM34_NDMACHAN descriptors must be allocated"

#  undef CONFIG_SAM34_NLLDESC
#  define CONFIG_SAM34_NLLDESC SAM34_NDMACHAN
#endif

/* Register values **********************************************************/

#define DMACHAN_CTRLB_BOTHDSCR \
  (DMACHAN_CTRLB_SRCDSCR | DMACHAN_CTRLB_DSTDSCR)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure describes one DMA channel */

struct sam_dma_s
{
  uint8_t                chan;       /* DMA channel number (0-6) */
  bool                   inuse;      /* TRUE: The DMA channel is in use */
  uint32_t               flags;      /* DMA channel flags */
  uint32_t               base;       /* DMA register channel base address */
  uint32_t               cfg;        /* Pre-calculated CFG register for transfer */
  dma_callback_t         callback;   /* Callback invoked when the DMA completes */
  void                  *arg;        /* Argument passed to callback function */
  struct dma_linklist_s *llhead;     /* DMA link list head */
  struct dma_linklist_s *lltail;     /* DMA link list head */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* These semaphores protect the DMA channel and descriptor tables */

static sem_t g_chsem;
static sem_t g_dsem;

/* CTRLA field lookups */

static const uint32_t g_srcwidth[3] =
{
  DMACHAN_CTRLA_SRCWIDTH_BYTE,
  DMACHAN_CTRLA_SRCWIDTH_HWORD,
  DMACHAN_CTRLA_SRCWIDTH_WORD
};

static const uint32_t g_destwidth[3] =
{
  DMACHAN_CTRLA_DSTWIDTH_BYTE,
  DMACHAN_CTRLA_DSTWIDTH_HWORD,
  DMACHAN_CTRLA_DSTWIDTH_WORD
};

static const uint32_t g_fifocfg[3] =
{
  DMACHAN_CFG_FIFOCFG_LARGEST,
  DMACHAN_CFG_FIFOCFG_HALF,
  DMACHAN_CFG_FIFOCFG_SINGLE
};

/* This array describes the available link list descriptors */

static struct dma_linklist_s g_linklist[CONFIG_SAM34_NLLDESC];

/* This array describes the state of each DMA */

static struct sam_dma_s g_dma[SAM34_NDMACHAN] =
{
#if defined(CONFIG_ARCH_CHIP_SAM3U)
  /* The SAM3U has four DMA channels.  The FIFOs for channels 0-2 are
   * 8 bytes in size; channel 3 is 32 bytes.
   */

#if SAM34_NDMACHAN != 4
#  error "Logic here assumes SAM34_NDMACHAN is 4"
#endif

  {
    .chan     = 0,
    .flags    = DMACH_FLAG_FIFO_8BYTES,
    .base     = SAM_DMACHAN0_BASE,
  },
  {
    .chan     = 1,
    .flags    = DMACH_FLAG_FIFO_8BYTES,
    .base     = SAM_DMACHAN1_BASE,
  },
  {
    .chan     = 2,
    .flags    = DMACH_FLAG_FIFO_8BYTES,
    .base     = SAM_DMACHAN2_BASE,
  },
  {
    .chan     = 3,
    .flags    = DMACH_FLAG_FIFO_32BYTES,
    .base     = SAM_DMACHAN3_BASE,
  }

#elif defined(CONFIG_ARCH_CHIP_SAM3X) || defined(CONFIG_ARCH_CHIP_SAM3A)
  /* The SAM3A/X have six DMA channels.  The FIFOs for channels 0-2 are
   * 8 bytes in size; channel 3 is 32 bytes.
   */

#if SAM34_NDMACHAN != 6
#  error "Logic here assumes SAM34_NDMACHAN is 6"
#endif

  {
    .chan     = 0,
    .flags    = DMACH_FLAG_FIFO_8BYTES,
    .base     = SAM_DMACHAN0_BASE,
  },
  {
    .chan     = 1,
    .flags    = DMACH_FLAG_FIFO_8BYTES,
    .base     = SAM_DMACHAN1_BASE,
  },
  {
    .chan     = 2,
    .flags    = DMACH_FLAG_FIFO_8BYTES,
    .base     = SAM_DMACHAN2_BASE,
  },
  {
    .chan     = 3,
    .flags    = DMACH_FLAG_FIFO_32BYTES,
    .base     = SAM_DMACHAN3_BASE,
  }
  {
    .chan     = 4,
    .flags    = DMACH_FLAG_FIFO_8BYTES,
    .base     = SAM_DMACHAN4_BASE,
  }
  {
    .chan     = 5,
    .flags    = DMACH_FLAG_FIFO_32BYTES,
    .base     = SAM_DMACHAN5_BASE,
  }

#elif defined(CONFIG_ARCH_CHIP_SAM4E)
  /* The SAM4E16E, SAM4E8E, SAM4E16C, and SAM4E8C have four DMA channels.
   *
   * REVISIT:  I have not yet found any documentation for the per-channel
   * FIFO depth.  Here I am assuming that the FIFO characteristics are
   * the same as for the SAM3U.
   */

#if SAM34_NDMACHAN != 4
#  error "Logic here assumes SAM34_NDMACHAN is 4"
#endif

  {
    .chan     = 0,
    .flags    = DMACH_FLAG_FIFO_8BYTES,
    .base     = SAM_DMACHAN0_BASE,
  },
  {
    .chan     = 1,
    .flags    = DMACH_FLAG_FIFO_8BYTES,
    .base     = SAM_DMACHAN1_BASE,
  },
  {
    .chan     = 2,
    .flags    = DMACH_FLAG_FIFO_8BYTES,
    .base     = SAM_DMACHAN2_BASE,
  },
  {
    .chan     = 3,
    .flags    = DMACH_FLAG_FIFO_32BYTES,
    .base     = SAM_DMACHAN3_BASE,
  }

#else
#  error "Nothing is known about the DMA channels for this device"
#endif
};

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

static void sam_takechsem(void)
{
  /* Take the semaphore (perhaps waiting) */

  while (sem_wait(&g_chsem) != 0)
    {
      /* The only case that an error should occur here is if the wait was
       * awakened by a signal.
       */

      ASSERT(errno == EINTR);
    }
}

static inline void sam_givechsem(void)
{
  (void)sem_post(&g_chsem);
}

/****************************************************************************
 * Name: sam_takedsem() and sam_givedsem()
 *
 * Description:
 *   Used to wait for availability of descriptors in the descriptor table.
 *
 ****************************************************************************/

static void sam_takedsem(void)
{
  /* Take the semaphore (perhaps waiting) */

  while (sem_wait(&g_dsem) != 0)
    {
      /* The only case that an error should occur here is if the wait was
       * awakened by a signal.
       */

      ASSERT(errno == EINTR);
    }
}

static inline void sam_givedsem(void)
{
  (void)sem_post(&g_dsem);
}

/****************************************************************************
 * Name: sam_fifosize
 *
 * Description:
 *  Decode the FIFO size from the flags
 *
 ****************************************************************************/

static unsigned int sam_fifosize(uint8_t chflags)
{
  chflags &= DMACH_FLAG_FIFOSIZE_MASK;
  if (chflags == DMACH_FLAG_FIFO_8BYTES)
    {
      return 8;
    }
  else /* if (chflags == DMACH_FLAG_FIFO_32BYTES) */
    {
      return 32;
    }
}

/****************************************************************************
 * Name: sam_fifocfg
 *
 * Description:
 *  Decode the FIFO config from the flags
 *
 ****************************************************************************/

static inline uint32_t sam_fifocfg(struct sam_dma_s *dmach)
{
  unsigned int ndx = (dmach->flags & DMACH_FLAG_FIFOCFG_MASK) >> DMACH_FLAG_FIFOCFG_SHIFT;
  DEBUGASSERT(ndx < 3);
  return g_fifocfg[ndx];
}

/****************************************************************************
 * Name: sam_txcfg
 *
 * Description:
 *  Decode the flags to get the correct CFG register bit settings for
 *  a transmit (memory to peripheral) transfer.
 *
 ****************************************************************************/

static inline uint32_t sam_txcfg(struct sam_dma_s *dmach)
{
  uint32_t regval;

  /* Set transfer (memory to peripheral) DMA channel configuration register */

  regval   = DMACHAN_CFG_SOD;
  regval  |= (((dmach->flags & DMACH_FLAG_MEMPID_MASK) >> DMACH_FLAG_MEMPID_SHIFT) << DMACHAN_CFG_SRCPER_SHIFT);
  regval  |=   (dmach->flags & DMACH_FLAG_MEMH2SEL) != 0 ? DMACHAN_CFG_SRCH2SEL : 0;
  regval  |= (((dmach->flags & DMACH_FLAG_PERIPHPID_MASK) >> DMACH_FLAG_PERIPHPID_SHIFT) << DMACHAN_CFG_DSTPER_SHIFT);
  regval  |=   (dmach->flags & DMACH_FLAG_PERIPHH2SEL) != 0 ? DMACHAN_CFG_DSTH2SEL : 0;
  regval  |= sam_fifocfg(dmach);
  return regval;
}

/****************************************************************************
 * Name: sam_rxcfg
 *
 * Description:
 *  Decode the flags to get the correct CFG register bit settings for
 *  a receive (peripheral to memory) transfer.
 *
 ****************************************************************************/

static inline uint32_t sam_rxcfg(struct sam_dma_s *dmach)
{
  uint32_t regval;

  /* Set received (peripheral to memory) DMA channel config */

  regval   = DMACHAN_CFG_SOD;
  regval  |= (((dmach->flags & DMACH_FLAG_PERIPHPID_MASK) >> DMACH_FLAG_PERIPHPID_SHIFT) << DMACHAN_CFG_SRCPER_SHIFT);
  regval  |=   (dmach->flags & DMACH_FLAG_PERIPHH2SEL) != 0 ? DMACHAN_CFG_SRCH2SEL : 0;
  regval  |= (((dmach->flags & DMACH_FLAG_MEMPID_MASK) >> DMACH_FLAG_MEMPID_SHIFT) << DMACHAN_CFG_DSTPER_SHIFT);
  regval  |=   (dmach->flags & DMACH_FLAG_MEMH2SEL) != 0 ? DMACHAN_CFG_DSTH2SEL : 0;
  regval  |= sam_fifocfg(dmach);
  return regval;
}

/****************************************************************************
 * Name: sam_txctrlabits
 *
 * Description:
 *  Decode the flags to get the correct CTRLA register bit settings for
 *  a transmit (memory to peripheral) transfer.  These are only the "fixed"
 *  CTRLA values and  need to be updated with the actual transfer size before
 *  being written to CTRLA sam_txctrla).
 *
 ****************************************************************************/

static inline uint32_t
sam_txctrlabits(struct sam_dma_s *dmach)
{
  uint32_t regval;
  unsigned int ndx;

  DEBUGASSERT(dmach);

  /* Since this is a transmit, the source is described by the memory selections.
   * Set the source width (memory width).
   */

  ndx = (dmach->flags & DMACH_FLAG_MEMWIDTH_MASK) >> DMACH_FLAG_MEMWIDTH_SHIFT;
  DEBUGASSERT(ndx < 3);
  regval = g_srcwidth[ndx];

#if defined(CONFIG_ARCH_CHIP_SAM3U) ||  defined(CONFIG_ARCH_CHIP_SAM3X) || \
    defined(CONFIG_ARCH_CHIP_SAM3A)
  /* Set the source chunk size (memory chunk size) */

  if ((dmach->flags & DMACH_FLAG_MEMCHUNKSIZE) == DMACH_FLAG_MEMCHUNKSIZE_4)
    {
      regval |= DMACHAN_CTRLA_SCSIZE_4;
    }
#if 0 /* DMACHAN_CTRLA_SCSIZE_1 is zero */
  else
    {
      regval |= DMACHAN_CTRLA_SCSIZE_1;
    }
#endif
#endif

  /* Since this is a transmit, the destination is described by the peripheral selections.
   * Set the destination width (peripheral width).
   */

  ndx = (dmach->flags & DMACH_FLAG_PERIPHWIDTH_MASK) >> DMACH_FLAG_PERIPHWIDTH_SHIFT;
  DEBUGASSERT(ndx < 3);
  regval |= g_destwidth[ndx];

#if defined(CONFIG_ARCH_CHIP_SAM3U) ||  defined(CONFIG_ARCH_CHIP_SAM3X) || \
    defined(CONFIG_ARCH_CHIP_SAM3A)
  /* Set the destination chunk size (peripheral chunk size) */

  if ((dmach->flags & DMACH_FLAG_PERIPHCHUNKSIZE) == DMACH_FLAG_PERIPHCHUNKSIZE_4)
    {
      regval |= DMACHAN_CTRLA_DCSIZE_4;
    }
#if 0 /* DMACHAN_CTRLA_DCSIZE_1 is zero */
  else
    {
      regval |= DMACHAN_CTRLA_DCSIZE_1;
    }
#endif
#endif

  return regval;
}

/****************************************************************************
 * Name: sam_maxtxtransfer
 *
 * Description:
 *  Maximum number of bytes that can be sent in on transfer
 *
 ****************************************************************************/

static size_t sam_maxtxtransfer(struct sam_dma_s *dmach)
{
  unsigned int srcwidth;
  size_t maxtransfer;

  /* Get the maximum transfer size in bytes.  BTSIZE is "the number of
   * transfers to be performed, that is, for writes it refers to the number
   * of source width transfers to perform when DMAC is flow controller. For
   * Reads, BTSIZE refers to the number of transfers completed on the Source
   * Interface. ..."
   */

  srcwidth = (dmach->flags & DMACH_FLAG_MEMWIDTH_MASK)
    >> DMACH_FLAG_MEMWIDTH_SHIFT;

  switch (srcwidth)
    {
      default:
      case 0: /* 8 bits, 1 byte */
        maxtransfer = DMACHAN_CTRLA_BTSIZE_MAX;
        break;

      case 1: /* 16 bits, 2 bytes */
        maxtransfer = 2 * DMACHAN_CTRLA_BTSIZE_MAX;
        break;

      case 2: /* 32 bits 4 bytes */
        maxtransfer = 4 * DMACHAN_CTRLA_BTSIZE_MAX;
        break;
    }

  return maxtransfer;
}

/****************************************************************************
 * Name: sam_txctrla
 *
 * Description:
 *  Or in the variable CTRLA bits
 *
 ****************************************************************************/

static inline uint32_t sam_txctrla(struct sam_dma_s *dmach,
                                   uint32_t ctrla, uint32_t dmasize)
{
  unsigned int srcwidth;

  /* Set the buffer transfer size field.  This is the number of transfers to
   * be performed, that is, the number of source width transfers to perform.
   */

  srcwidth = (dmach->flags & DMACH_FLAG_MEMWIDTH_MASK)
    >> DMACH_FLAG_MEMWIDTH_SHIFT;

  switch (srcwidth)
    {
      default:
      case 0: /* 8 bits, 1 byte */
        break;

      case 1: /* 16 bits, 2 bytes */
        dmasize = (dmasize + 1) >> 1;
        break;

      case 2: /* 32 bits, 4 bytes */
        dmasize = (dmasize + 3) >> 2;
        break;
    }

  DEBUGASSERT(dmasize <= DMACHAN_CTRLA_BTSIZE_MAX);
  return (ctrla & ~DMACHAN_CTRLA_BTSIZE_MASK) |
         (dmasize << DMACHAN_CTRLA_BTSIZE_SHIFT);
}

/****************************************************************************
 * Name: sam_rxctrlabits
 *
 * Description:
 *  Decode the flags to get the correct CTRLA register bit settings for
 *  a read (peripheral to memory) transfer. These are only the "fixed" CTRLA
 *  values and need to be updated with the actual transfer size before being
 *  written to CTRLA sam_rxctrla).
 *
 ****************************************************************************/

static inline uint32_t sam_rxctrlabits(struct sam_dma_s *dmach)
{
  uint32_t     regval;
  unsigned int ndx;

  DEBUGASSERT(dmach);

  /* Since this is a receive, the source is described by the peripheral
   * selections. Set the source width (peripheral width).
   */

  ndx = (dmach->flags & DMACH_FLAG_PERIPHWIDTH_MASK) >> DMACH_FLAG_PERIPHWIDTH_SHIFT;
  DEBUGASSERT(ndx < 3);
  regval = g_srcwidth[ndx];

#if defined(CONFIG_ARCH_CHIP_SAM3U) ||  defined(CONFIG_ARCH_CHIP_SAM3X) || \
    defined(CONFIG_ARCH_CHIP_SAM3A)
  /* Set the source chunk size (peripheral chunk size) */

  if ((dmach->flags & DMACH_FLAG_PERIPHCHUNKSIZE) == DMACH_FLAG_PERIPHCHUNKSIZE_4)
    {
      regval |= DMACHAN_CTRLA_SCSIZE_4;
    }
#if 0 /* DMACHAN_CTRLA_SCSIZE_1 is zero */
  else
    {
      regval |= DMACHAN_CTRLA_SCSIZE_1;
    }
#endif
#endif

  /* Since this is a receive, the destination is described by the memory selections.
   * Set the destination width (memory width).
   */

  ndx = (dmach->flags & DMACH_FLAG_MEMWIDTH_MASK) >> DMACH_FLAG_MEMWIDTH_SHIFT;
  DEBUGASSERT(ndx < 3);
  regval |= g_destwidth[ndx];

#if defined(CONFIG_ARCH_CHIP_SAM3U) ||  defined(CONFIG_ARCH_CHIP_SAM3X) || \
    defined(CONFIG_ARCH_CHIP_SAM3A)
  /* Set the destination chunk size (memory chunk size) */

  if ((dmach->flags & DMACH_FLAG_MEMCHUNKSIZE) == DMACH_FLAG_MEMCHUNKSIZE_4)
    {
      regval |= DMACHAN_CTRLA_DCSIZE_4;
    }
#if 0 /* DMACHAN_CTRLA_DCSIZE_1 is zero */
  else
    {
      regval |= DMACHAN_CTRLA_DCSIZE_1;
    }
#endif
#endif

  return regval;
}

/****************************************************************************
 * Name: sam_maxrxtransfer
 *
 * Description:
 *  Maximum number of bytes that can be sent in on transfer
 *
 ****************************************************************************/

static size_t sam_maxrxtransfer(struct sam_dma_s *dmach)
{
  unsigned int srcwidth;
  size_t maxtransfer;

  /* Get the maximum transfer size in bytes.  BTSIZE is "the number of
   * transfers to be performed, that is, for writes it refers to the number
   * of source width transfers to perform when DMAC is flow controller. For
   * Reads, BTSIZE refers to the number of transfers completed on the Source
   * Interface. ..."
   */

  srcwidth = (dmach->flags & DMACH_FLAG_PERIPHWIDTH_MASK)
    >> DMACH_FLAG_PERIPHWIDTH_SHIFT;

  switch (srcwidth)
    {
      default:
      case 0: /* 8 bits, 1 byte */
        maxtransfer = DMACHAN_CTRLA_BTSIZE_MAX;
        break;

      case 1: /* 16 bits, 2 bytes */
        maxtransfer = 2 * DMACHAN_CTRLA_BTSIZE_MAX;
        break;

      case 2: /* 32 bits, 4 bytes */
        maxtransfer = 4 * DMACHAN_CTRLA_BTSIZE_MAX;
        break;
    }

  return maxtransfer;
}

/****************************************************************************
 * Name: sam_rxctrla
 *
 * Description:
 *  'OR' in the variable CTRLA bits
 *
 ****************************************************************************/

static inline uint32_t sam_rxctrla(struct sam_dma_s *dmach,
                                   uint32_t ctrla, uint32_t dmasize)
{
  unsigned int srcwidth;

  /* Set the buffer transfer size field.  This is the number of transfers to
   * be performed, that is, the number of source width transfers to perform.
   */

  srcwidth = (dmach->flags & DMACH_FLAG_PERIPHWIDTH_MASK)
    >> DMACH_FLAG_PERIPHWIDTH_SHIFT;

  switch (srcwidth)
    {
      default:
      case 0: /* 8 bits, 1 byte */
        break;

      case 1: /* 16 bits, 2 bytes */
        dmasize = (dmasize + 1) >> 1;
        break;

      case 2: /* 32 bits, 4 bytes */
        dmasize = (dmasize + 3) >> 2;
        break;
    }

  DEBUGASSERT(dmasize <= DMACHAN_CTRLA_BTSIZE_MAX);
  return (ctrla & ~DMACHAN_CTRLA_BTSIZE_MASK) |
         (dmasize << DMACHAN_CTRLA_BTSIZE_SHIFT);
}

/****************************************************************************
 * Name: sam_txctrlb
 *
 * Description:
 *  Decode the flags to get the correct CTRLB register bit settings for
 *  a transmit (memory to peripheral) transfer.
 *
 ****************************************************************************/

static inline uint32_t sam_txctrlb(struct sam_dma_s *dmach)
{
  uint32_t regval;

  /* Assume that we will not be using the link list and disable the source
   * and destination descriptors.  The default will be single transfer mode.
   */

  regval = DMACHAN_CTRLB_BOTHDSCR;

  /* Select flow control (even if the channel doesn't support it).  The
   * naming convention from TX is memory to peripheral, but that is really be
   * determined by bits in the DMA flags.
   */

  /* Is the memory source really a peripheral? */

  if ((dmach->flags & DMACH_FLAG_MEMISPERIPH) != 0)
    {
      /* Yes.. is the peripheral destination also a peripheral? */

      if ((dmach->flags & DMACH_FLAG_PERIPHISPERIPH) != 0)
        {
          /* Yes.. Use peripheral-to-peripheral flow control */

          regval |= DMACHAN_CTRLB_FC_P2P;
        }
      else
        {
          /* No.. Use peripheral-to-memory flow control */

          regval |= DMACHAN_CTRLB_FC_P2M;
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

          regval |= DMACHAN_CTRLB_FC_M2P;
        }
      else
        {
          /* No.. Use memory-to-memory flow control */

          regval |= DMACHAN_CTRLB_FC_M2M;
        }
    }

  /* Select source address incrementing */

  if ((dmach->flags & DMACH_FLAG_MEMINCREMENT) == 0)
    {
      regval |= DMACHAN_CTRLB_SRCINCR_FIXED;
    }

  /* Select destination address incrementing */

  if ((dmach->flags & DMACH_FLAG_PERIPHINCREMENT) == 0)
    {
      regval |= DMACHAN_CTRLB_DSTINCR_FIXED;
    }
  return regval;
}

/****************************************************************************
 * Name: sam_rxctrlb
 *
 * Description:
 *  Decode the flags to get the correct CTRLB register bit settings for
 *  a receive (peripheral to memory) transfer.
 *
 ****************************************************************************/

static inline uint32_t sam_rxctrlb(struct sam_dma_s *dmach)
{
  uint32_t regval;

  /* Assume that we will not be using the link list and disable the source and
   * destination descriptors.  The default will be single transfer mode.
   */

  regval = DMACHAN_CTRLB_BOTHDSCR;

  /* Select flow control (even if the channel doesn't support it).  The
   * naming convention from RX is peripheral to memory, but that is really be
   * determined by bits in the DMA flags.
   */

  /* Is the peripheral source really a peripheral? */

  if ((dmach->flags & DMACH_FLAG_PERIPHISPERIPH) != 0)
    {
      /* Yes.. is the memory destination also a peripheral? */

      if ((dmach->flags & DMACH_FLAG_MEMISPERIPH) != 0)
        {
          /* Yes.. Use peripheral-to-peripheral flow control */

          regval |= DMACHAN_CTRLB_FC_P2P;
        }
      else
        {
          /* No.. Use peripheral-to-memory flow control */

          regval |= DMACHAN_CTRLB_FC_P2M;
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

          regval |= DMACHAN_CTRLB_FC_M2P;
        }
      else
        {
          /* No.. Use memory-to-memory flow control */

          regval |= DMACHAN_CTRLB_FC_M2M;
        }
    }

  /* Select source address incrementing */

  if ((dmach->flags & DMACH_FLAG_PERIPHINCREMENT) == 0)
    {
      regval |= DMACHAN_CTRLB_SRCINCR_FIXED;
    }

  /* Select address incrementing */

  if ((dmach->flags & DMACH_FLAG_MEMINCREMENT) == 0)
    {
      regval |= DMACHAN_CTRLB_DSTINCR_FIXED;
    }

  return regval;
}

/****************************************************************************
 * Name: sam_allocdesc
 *
 * Description:
 *  Allocate and add one descriptor to the DMA channel's link list.
 *
 *  NOTE: link list entries are freed by the DMA interrupt handler.  However,
 *  since the setting/clearing of the 'in use' indication is atomic, no
 *  special actions need be performed.  It would be a good thing to add logic
 *  to handle the case where all of the entries are exhausted and we could
 *  wait for some to be freed by the interrupt handler.
 *
 ****************************************************************************/

static struct dma_linklist_s *
sam_allocdesc(struct sam_dma_s *dmach, struct dma_linklist_s *prev,
              uint32_t src, uint32_t dest, uint32_t ctrla, uint32_t ctrlb)
{
  struct dma_linklist_s *desc = NULL;
  int i;

  /* Sanity check -- src == 0 is the indication that the link is unused.
   * Obviously setting it to zero would break that usage.
   */

#ifdef CONFIG_DEBUG
  if (src != 0)
#endif
    {
      /* Table a descriptor table semaphore count.  When we get one, then there
       * is at least one free descriptor in the table and it is ours.
       */

      sam_takedsem();

      /* Examine each link list entry to find an available one -- i.e., one
       * with src == 0.  That src field is set to zero by the DMA transfer
       * complete interrupt handler.  The following should be safe because
       * that is an atomic operation.
       */

      sam_takechsem();
      for (i = 0; i < CONFIG_SAM34_NLLDESC; i++)
        {
          if (g_linklist[i].src == 0)
            {
              /* We have it.  Initialize the new link list entry */

              desc        = &g_linklist[i];
              desc->src   = src;    /* Source address */
              desc->dest  = dest;   /* Destination address */
              desc->ctrla = ctrla;  /* Control A value */
              desc->ctrlb = ctrlb;  /* Control B value */
              desc->next  = 0;      /* Next descriptor address */

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

                  prev->ctrlb &= ~DMACHAN_CTRLB_BOTHDSCR;

                  /* Link the previous tail to the new tail */

                  prev->next = (uint32_t)desc;
                }

              /* In any event, this is the new tail of the list.  The source
               * and destination descriptors must be disabled for the last entry
               * in the link list. */

              desc->ctrlb  |= DMACHAN_CTRLB_BOTHDSCR;
              dmach->lltail = desc;
              break;
            }
        }

      /* Because we hold a count from the counting semaphore, the above
       * search loop should always be successful.
       */

      sam_givechsem();
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

static void sam_freelinklist(struct sam_dma_s *dmach)
{
  struct dma_linklist_s *desc;
  struct dma_linklist_s *next;

  /* Get the head of the link list and detach the link list from the DMA
   * channel
   */

  desc             = dmach->llhead;
  dmach->llhead    = NULL;
  dmach->lltail    = NULL;

  /* Reset each descriptor in the link list (thereby freeing them) */

  while (desc != NULL)
    {
      next = (struct dma_linklist_s *)desc->next;
      DEBUGASSERT(desc->src != 0);
      memset(desc, 0, sizeof(struct dma_linklist_s));
      sam_givedsem();
      desc = next;
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

static int sam_txbuffer(struct sam_dma_s *dmach, uint32_t paddr,
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

static int sam_rxbuffer(struct sam_dma_s *dmach, uint32_t paddr,
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

   ctrla = sam_rxctrla(dmach, regval, nbytes);

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

static inline int sam_single(struct sam_dma_s *dmach)
{
  struct dma_linklist_s *llhead = dmach->llhead;

  /* Clear any pending interrupts from any previous DMAC transfer by reading
   * the interrupt status register.
   *
   * REVISIT: If DMAC interrupts are disabled at the NVIKC, then reading the
   * EBCISR register could cause a loss of interrupts!
   */

  (void)getreg32(SAM_DMAC_EBCISR);

  /* Write the starting source address in the SADDR register */

  DEBUGASSERT(llhead != NULL && llhead->src != 0);
  putreg32(llhead->src, dmach->base + SAM_DMACHAN_SADDR_OFFSET);

  /* Write the starting destination address in the DADDR register */

  putreg32(llhead->dest, dmach->base + SAM_DMACHAN_DADDR_OFFSET);

  /* Clear the next descriptor address register */

  putreg32(0, dmach->base + SAM_DMACHAN_DSCR_OFFSET);

  /* Set up the CTRLA register */

  putreg32(llhead->ctrla, dmach->base + SAM_DMACHAN_CTRLA_OFFSET);

  /* Set up the CTRLB register */

  putreg32(llhead->ctrlb, dmach->base + SAM_DMACHAN_CTRLB_OFFSET);

  /* Both the DST and SRC DSCR bits should be '1' in CTRLB */

  DEBUGASSERT((llhead->ctrlb & DMACHAN_CTRLB_BOTHDSCR) == DMACHAN_CTRLB_BOTHDSCR);

  /* Set up the CFG register */

  putreg32(dmach->cfg, dmach->base + SAM_DMACHAN_CFG_OFFSET);

  /* Enable the channel by writing a ‘1’ to the CHER enable bit */

  putreg32(DMAC_CHER_ENA(dmach->chan), SAM_DMAC_CHER);

  /* The DMA has been started. Once the transfer completes, hardware sets the
   * interrupts and disables the channel.  We will received buffer complete and
   * transfer complete interrupts.
   *
   * Enable error, buffer complete and transfer complete interrupts.
   * (Since there is only a single buffer, we don't need the buffer complete
   * interrupt).
   */

  putreg32(DMAC_EBC_CBTCINTS(dmach->chan), SAM_DMAC_EBCIER);
  return OK;
}

/****************************************************************************
 * Name: sam_multiple
 *
 * Description:
 *   Start a multiple buffer DMA.
 *
 ****************************************************************************/

static inline int sam_multiple(struct sam_dma_s *dmach)
{
  struct dma_linklist_s *llhead = dmach->llhead;

  DEBUGASSERT(llhead != NULL && llhead->src != 0);

  /* Check the first and last CTRLB values */

  DEBUGASSERT((llhead->ctrlb & DMACHAN_CTRLB_BOTHDSCR) == 0);
  DEBUGASSERT((dmach->lltail->ctrlb & DMACHAN_CTRLB_BOTHDSCR) == DMACHAN_CTRLB_BOTHDSCR);

  /* Clear any pending interrupts from any previous DMAC transfer by reading the
   * status register
   *
   * REVISIT: If DMAC interrupts are disabled at the NVIKC, then reading the
   * EBCISR register could cause a loss of interrupts!
   */

  (void)getreg32(SAM_DMAC_EBCISR);

  /* Set up the initial CTRLA register */

  putreg32(llhead->ctrla, dmach->base + SAM_DMACHAN_CTRLA_OFFSET);

  /* Set up the CTRLB register (will enable descriptors) */

  putreg32(llhead->ctrlb, dmach->base + SAM_DMACHAN_CTRLB_OFFSET);

  /* Write the channel configuration information into the CFG register */

  putreg32(dmach->cfg, dmach->base + SAM_DMACHAN_CFG_OFFSET);

  /* Program the DSCR register with the pointer to the firstlink list entry. */

  putreg32((uint32_t)llhead, dmach->base + SAM_DMACHAN_DSCR_OFFSET);

  /* Finally, enable the channel by writing a ‘1’ to the CHER enable */

  putreg32(DMAC_CHER_ENA(dmach->chan), SAM_DMAC_CHER);

  /* As each buffer of data is transferred, the CTRLA register is written back
   * into the link list entry.  The CTRLA contains updated BTSIZE and DONE bits.
   * Additionally, the CTRLA DONE bit is asserted when the buffer transfer has completed.
   *
   * The DMAC transfer continues until the CTRLB register disables the descriptor
   * (DSCR bits) registers at the final buffer tranfer.
   *
   * Enable error, buffer complete and transfer complete interrupts.  We
   * don't really need the buffer complete interrupts, but we will take them
   * just to handle stall conditions.
   */

  putreg32(DMAC_EBC_CHANINTS(dmach->chan), SAM_DMAC_EBCIER);
  return OK;
}

/****************************************************************************
 * Name: sam_dmaterminate
 *
 * Description:
 *   Terminate the DMA transfer and disable the DMA channel
 *
 ****************************************************************************/

static void sam_dmaterminate(struct sam_dma_s *dmach, int result)
{
  /* Disable all channel interrupts */

  putreg32(DMAC_EBC_CHANINTS(dmach->chan), SAM_DMAC_EBCIDR);

  /* Disable the channel by writing one to the write-only channel disable register */

  putreg32(DMAC_CHDR_DIS(dmach->chan), SAM_DMAC_CHDR);

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
 * Name: sam_dmainterrupt
 *
 * Description:
 *  DMA interrupt handler
 *
 ****************************************************************************/

static int sam_dmainterrupt(int irq, void *context)
{
  struct sam_dma_s *dmach;
  unsigned int chndx;
  uint32_t regval;

  /* Get the DMAC status register value.  Ignore all masked interrupt
   * status bits.
   */

  regval = getreg32(SAM_DMAC_EBCISR) & getreg32(SAM_DMAC_EBCIMR);

  /* Check if the any transfer has completed or any errors have occurred */

  if ((regval & DMAC_EBC_ALLINTS) != 0)
    {
      /* Yes.. Check each bit  to see which channel has interrupted */

      for (chndx = 0; chndx < SAM34_NDMACHAN; chndx++)
        {
          /* Are any interrupts pending for this channel? */

          if ((regval & DMAC_EBC_CHANINTS(chndx)) != 0)
            {
              dmach = &g_dma[chndx];

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

              /* Otherwise, this must be a Buffer Transfer Complete (BTC)
               * interrupt as part of a multiple buffer transfer.
               */

              else /* if ((regval & DMAC_EBC_BTC(chndx)) != 0) */
                {
                  /* Write the KEEPON field to clear the STALL states */

                  putreg32(DMAC_CHER_KEEP(dmach->chan), SAM_DMAC_CHER);
                }
            }
        }
    }

  return OK;
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
  dmallvdbg("Initialize DMAC0\n");

  /* Enable peripheral clock */

  sam_dmac_enableclk();

  /* Disable all DMA interrupts */

  putreg32(DMAC_EBC_ALLINTS, SAM_DMAC_EBCIDR);

  /* Disable all DMA channels */

  putreg32(DMAC_CHDR_DIS_ALL, SAM_DMAC_CHDR);

  /* Attach DMA interrupt vector */

  (void)irq_attach(SAM_IRQ_DMAC, sam_dmainterrupt);

  /* Enable the IRQ at the NVIC (still disabled at the DMA controller) */

  up_enable_irq(SAM_IRQ_DMAC);

  /* Enable the DMA controller */

  putreg32(DMAC_EN_ENABLE, SAM_DMAC_EN);

  /* Initialize semaphores */

  sem_init(&g_chsem, 0, 1);
  sem_init(&g_dsem, 0, CONFIG_SAM34_NLLDESC);
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

DMA_HANDLE sam_dmachannel(uint32_t chflags)
{
  struct sam_dma_s *dmach;
  unsigned int fifosize;
  unsigned int chndx;

  /* Get the search parameters */

  fifosize = sam_fifosize(chflags);

  /* Search for an available DMA channel with at least the requested FIFO
   * size.
   */

  dmach = NULL;
  sam_takechsem();

  for (chndx = 0; chndx < SAM34_NDMACHAN; chndx++)
    {
      struct sam_dma_s *candidate = &g_dma[chndx];
      if (!candidate->inuse &&
          (sam_fifosize(candidate->flags) >= fifosize))
        {
          dmach         = candidate;
          dmach->inuse  = true;

          /* Read the status register to clear any pending interrupts on the
           * channel
           *
           * REVISIT: If DMAC interrupts are disabled at the NVIKC, then
           * reading the EBCISR register could cause a loss of interrupts!
           */

          (void)getreg32(SAM_DMAC_EBCISR);

          /* Disable the channel by writing one to the write-only channel
           * disable register
           */

          putreg32(DMAC_CHDR_DIS(chndx), SAM_DMAC_CHDR);

          /* Set the DMA channel flags, retaining the fifo size setting
           * which is an inherent properties of the FIFO and cannot be
           * changed.
           */

          dmach->flags &= DMACH_FLAG_FIFOSIZE_MASK;
          dmach->flags |= (chflags & ~DMACH_FLAG_FIFOSIZE_MASK);
          break;
        }
    }

  sam_givechsem();

  dmavdbg("chflags: %08x returning dmach: %p\n",  (int)chflags, dmach);
  return (DMA_HANDLE)dmach;
}

/************************************************************************************
 * Name: sam_dmaconfig
 *
 * Description:
 *   There are two channel usage models:  (1) The channel is allocated and configured
 *   in one step.  This is the typical case where a DMA channel performs a constant
 *   role.  The alternative is (2) where the DMA channel is reconfigured on the fly.
 *   In this case, the chflags provided to sam_dmachannel are not used and
 *   sam_dmaconfig() is called before each DMA to configure the DMA channel
 *   appropriately.
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

void sam_dmaconfig(DMA_HANDLE handle, uint32_t chflags)
{
  struct sam_dma_s *dmach = (struct sam_dma_s *)handle;

  /* Set the new DMA channel flags. */

  dmavdbg("chflags: %08x\n",  (int)chflags);
  dmach->flags = chflags;
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
  struct sam_dma_s *dmach = (struct sam_dma_s *)handle;

  dmavdbg("dmach: %p\n", dmach);
  DEBUGASSERT((dmach != NULL) && (dmach->inuse));

  /* Mark the channel no longer in use.  Clearing the inuse flag is an atomic
   * operation and so should be safe.
   */

  dmach->flags &= DMACH_FLAG_FIFOSIZE_MASK;
  dmach->inuse  = false;                   /* No longer in use */
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
  struct sam_dma_s *dmach = (struct sam_dma_s *)handle;
  ssize_t remaining = (ssize_t)nbytes;
  size_t maxtransfer;
  int ret = OK;

  dmavdbg("dmach: %p paddr: %08x maddr: %08x nbytes: %d\n",
          dmach, (int)paddr, (int)maddr, (int)nbytes);
  DEBUGASSERT(dmach);
  dmavdbg("llhead: %p lltail: %p\n", dmach->llhead, dmach->lltail);

  /* The maximum transfer size in bytes depends upon the maximum number of
   * transfers and the number of bytes per transfer.
   */

  maxtransfer = sam_maxtxtransfer(dmach);

  /* If this is a large transfer, break it up into smaller buffers */

  while (remaining > maxtransfer)
    {
      /* Set up the maximum size transfer */

      ret = sam_txbuffer(dmach, paddr, maddr, maxtransfer);
      if (ret == OK);
        {
          /* Decrement the number of bytes left to transfer */

          remaining -= maxtransfer;

          /* Increment the memory & peripheral address (if it is appropriate to
           * do do).
           */

          if ((dmach->flags & DMACH_FLAG_PERIPHINCREMENT) != 0)
            {
              paddr += maxtransfer;
            }

          if ((dmach->flags & DMACH_FLAG_MEMINCREMENT) != 0)
            {
              maddr += maxtransfer;
            }
        }
    }

  /* Then set up the final buffer transfer */

  if (ret == OK && remaining > 0)
    {
      ret = sam_txbuffer(dmach, paddr, maddr, remaining);
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

int sam_dmarxsetup(DMA_HANDLE handle, uint32_t paddr, uint32_t maddr, size_t nbytes)
{
  struct sam_dma_s *dmach = (struct sam_dma_s *)handle;
  ssize_t remaining = (ssize_t)nbytes;
  size_t maxtransfer;
  int ret = OK;

  dmavdbg("dmach: %p paddr: %08x maddr: %08x nbytes: %d\n",
          dmach, (int)paddr, (int)maddr, (int)nbytes);
  DEBUGASSERT(dmach);
  dmavdbg("llhead: %p lltail: %p\n", dmach->llhead, dmach->lltail);

  /* The maximum transfer size in bytes depends upon the maximum number of
   * transfers and the number of bytes per transfer.
   */

  maxtransfer = sam_maxrxtransfer(dmach);

  /* If this is a large transfer, break it up into smaller buffers */

  while (remaining > maxtransfer)
    {
      /* Set up the maximum size transfer */

      ret = sam_rxbuffer(dmach, paddr, maddr, maxtransfer);
      if (ret == OK);
        {
          /* Decrement the number of bytes left to transfer */

          remaining -= maxtransfer;

          /* Increment the memory & peripheral address (if it is appropriate to
           * do do).
           */

          if ((dmach->flags & DMACH_FLAG_PERIPHINCREMENT) != 0)
            {
              paddr += maxtransfer;
            }

          if ((dmach->flags & DMACH_FLAG_MEMINCREMENT) != 0)
            {
              maddr += maxtransfer;
            }
        }
    }

  /* Then set up the final buffer transfer */

  if (ret == OK && remaining > 0)
    {
      ret = sam_rxbuffer(dmach, paddr, maddr, remaining);
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
  struct sam_dma_s *dmach = (struct sam_dma_s *)handle;
  int ret = -EINVAL;

  dmavdbg("dmach: %p callback: %p arg: %p\n", dmach, callback, arg);
  DEBUGASSERT(dmach != NULL);

  /* Verify that the DMA has been setup (i.e., at least one entry in the
   * link list).
   */

  if (dmach->llhead)
    {
      /* Save the callback info.  This will be invoked when the DMA completes */

      dmach->callback = callback;
      dmach->arg      = arg;

      /* Is this a single block transfer?  Or a multiple block transfer? */

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
  struct sam_dma_s *dmach = (struct sam_dma_s *)handle;
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
  struct sam_dma_s *dmach = (struct sam_dma_s *)handle;
  irqstate_t flags;

  /* Sample global registers.  NOTE: reading EBCISR clears interrupts, but
   * that should be okay IF interrupts are enabled when this function is
   * called.  But there is a race condition where this instrumentation could
   * cause lost interrupts.
   */

  flags        = irqsave();
  regs->gcfg   = getreg32(SAM_DMAC_GCFG);
  regs->en     = getreg32(SAM_DMAC_EN);
  regs->sreq   = getreg32(SAM_DMAC_SREQ);
  regs->creq   = getreg32(SAM_DMAC_CREQ);
  regs->last   = getreg32(SAM_DMAC_LAST);
  regs->ebcimr = getreg32(SAM_DMAC_EBCIMR);
  regs->chsr   = getreg32(SAM_DMAC_CHSR);

  /* Sample channel registers */

  regs->saddr  = getreg32(dmach->base + SAM_DMACHAN_SADDR_OFFSET);
  regs->daddr  = getreg32(dmach->base + SAM_DMACHAN_DADDR_OFFSET);
  regs->dscr   = getreg32(dmach->base + SAM_DMACHAN_DSCR_OFFSET);
  regs->ctrla  = getreg32(dmach->base + SAM_DMACHAN_CTRLA_OFFSET);
  regs->ctrlb  = getreg32(dmach->base + SAM_DMACHAN_CTRLB_OFFSET);
  regs->cfg    = getreg32(dmach->base + SAM_DMACHAN_CFG_OFFSET);
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
  struct sam_dma_s *dmach = (struct sam_dma_s *)handle;

  dmadbg("%s\n", msg);
  dmadbg("  DMA Global Registers:\n");
  dmadbg("      GCFG[%08x]: %08x\n", SAM_DMAC_GCFG, regs->gcfg);
  dmadbg("        EN[%08x]: %08x\n", SAM_DMAC_EN, regs->en);
  dmadbg("      SREQ[%08x]: %08x\n", SAM_DMAC_SREQ, regs->sreq);
  dmadbg("      CREQ[%08x]: %08x\n", SAM_DMAC_CREQ, regs->creq);
  dmadbg("      LAST[%08x]: %08x\n", SAM_DMAC_LAST, regs->last);
  dmadbg("    EBCIMR[%08x]: %08x\n", SAM_DMAC_EBCIMR, regs->ebcimr);
  dmadbg("      CHSR[%08x]: %08x\n", SAM_DMAC_CHSR, regs->chsr);
  dmadbg("  DMA Channel Registers:\n");
  dmadbg("     SADDR[%08x]: %08x\n", dmach->base + SAM_DMACHAN_SADDR_OFFSET, regs->saddr);
  dmadbg("     DADDR[%08x]: %08x\n", dmach->base + SAM_DMACHAN_DADDR_OFFSET, regs->daddr);
  dmadbg("      DSCR[%08x]: %08x\n", dmach->base + SAM_DMACHAN_DSCR_OFFSET, regs->dscr);
  dmadbg("     CTRLA[%08x]: %08x\n", dmach->base + SAM_DMACHAN_CTRLA_OFFSET, regs->ctrla);
  dmadbg("     CTRLB[%08x]: %08x\n", dmach->base + SAM_DMACHAN_CTRLB_OFFSET, regs->ctrlb);
  dmadbg("       CFG[%08x]: %08x\n", dmach->base + SAM_DMACHAN_CFG_OFFSET, regs->cfg);
}
#endif /* CONFIG_DEBUG_DMA */
#endif /* CONFIG_SAM34_DMAC0 */
