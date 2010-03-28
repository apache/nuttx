/****************************************************************************
 * arch/arm/src/sam3u-ek/sam3u_dmac.c
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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
#include "sam3u_dmac.h"
#include "sam3u_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure descibes one DMA channel */

struct sam3u_dma_s
{
  uint8_t           chan;       /* DMA channel number (0-6) */
  bool              inuse;      /* TRUE: The DMA channel is in use */
  uint16_t          flags;      /* DMA channel flags */
  uint32_t          base;       /* DMA register channel base address */
  dma_callback_t    callback;   /* Callback invoked when the DMA completes */
  void             *arg;        /* Argument passed to callback function */
  uint16_t          bufsize;    /* Transfer buffer size in bytes */
  volatile uint16_t remaining;  /* Total number of bytes remaining to be transferred */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This semaphore protects the DMA channel table */

static sem_t g_dmasem;

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

/* This array describes the state of each DMA */

static struct sam3u_dma_s g_dma[CONFIG_SAM3U_NDMACHAN] =
{
#ifdef CONFIG_ARCH_CHIP_AT91SAM3U4E
  /* the AT91SAM3U4E has four DMA channels.  The FIFOs for channels 0-2 are
   * 8 bytes in size; channel 3 is 32 bytes.
   */

#if CONFIG_SAM3U_NDMACHAN != 4
#  error "Logic here assumes CONFIG_SAM3U_NDMACHAN is 4"
#endif

  {
    .chan     = 0,
    .flags    = DMACH_FLAG_FIFO_8BYTES;
    .base     = SAM3U_DMACHAN0_BASE,
  },
  {
    .chan     = 1,
    .flags    = DMACH_FLAG_FIFO_8BYTES;
    .base     = SAM3U_DMACHAN1_BASE,
  },
  {
    .chan     = 2,
    .flags    = DMACH_FLAG_FIFO_8BYTES;
    .base     = SAM3U_DMACHAN2_BASE,
  },
  {
    .chan     = 3,
    .flags    = (DMACH_FLAG_FIFO_32BYTES | DMACH_FLAG_FLOWCONTROL),
    .base     = SAM3U_DMACHAN3_BASE,
  }
#else
#  error "Nothing is known about the DMA channels for this device"
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/************************************************************************************
 * Name: sam3u_dmatake() and sam3u_dmagive()
 *
 * Description:
 *   Used to get exclusive access to a DMA channel.
 *
 ************************************************************************************/

static void sam3u_dmatake(void)
{
  /* Take the semaphore (perhaps waiting) */

  while (sem_wait(&g_dmasem) != 0)
    {
      /* The only case that an error should occur here is if the wait was awakened
       * by a signal.
       */

      ASSERT(errno == EINTR);
    }
}

static inline void sam3u_dmagive(void)
{
  (void)sem_post(&g_dmasem);
}

/************************************************************************************
 * Name: sam3u_fifosize
 *
 * Description:
 *  Decode the FIFO size from the flags
 *
 ************************************************************************************/

static unsigned int sam3u_fifosize(uint8_t dmach_flags)
{
  dmach_flags &= DMACH_FLAG_FIFOSIZE_MASK;
  if (dmach_flags == DMACH_FLAG_FIFO_8BYTES)
    {
      return 8;
    }
  else /* if (dmach_flags == DMACH_FLAG_FIFO_32BYTES) */
    {
      return 32;
    }
}

/************************************************************************************
 * Name: sam3u_flowcontrol
 *
 * Description:
 *  Decode the FIFO flow control from the flags
 *
 ************************************************************************************/

static inline boolean sam3u_flowcontrol(uint8_t dmach_flags)
{
  return ((dmach_flags & DMACH_FLAG_FLOWCONTROL) != 0);
}

/************************************************************************************
 * Name: sam3u_settxctrla
 *
 * Description:
 *  Decode the the flags to get the correct CTRLA register bit settings for a transmit
 *  (memory to peripheral) transfer.
 *
 ************************************************************************************/

static inline void
sam3u_settxctrla(struct sam3u_dma_s *dmach, uint32_t dmasize, uint32_t otherbits)
{
  uint32_t regval;
  unsigned int ndx;

  DEBUGASSERT(dmach && dmasize <= DMACHAN_CTRLA_BTSIZE_MAX);
  regval = (dmasize << DMACHAN_CTRLA_BTSIZE_SHIFT) | otherbits;

  /* Since this is a transmit, the source is described by the memeory selections */

  ndx = (dmach->flags & DMACH_FLAG_MEMWIDTH_MASK) >> DMACH_FLAG_MEMWIDTH_SHIFT;
  DEBUGASSERT(ndx < 3);
  regval |= g_srcwidth[ndx];
  return regval;

  /* Since this is a transmit, the destination is described by the peripheral selections */

  ndx = (dmach->flags & DMACH_FLAG_PERIPHWIDTH_MASK) >> DMACH_FLAG_PERIPHWIDTH_SHIFT;
  DEBUGASSERT(ndx < 3);
  regval |= g_destwidth[ndx];
  return regval;
}

/************************************************************************************
 * Name: sam3u_setrxctrla
 *
 * Description:
 *  Decode the the flags to get the correct CTRLA register bit settings for a read
 *  (peripheral to memory) transfer.
 *
 ************************************************************************************/

static inline void
sam3u_setrxctrla(struct sam3u_dma_s *dmach, uint32_t dmasize, uint32_t otherbits)
{
  uint32_t     regval;
  unsigned int ndx;

  DEBUGASSERT(dmach && dmasize <= DMACHAN_CTRLA_BTSIZE_MAX);
  regval = (dmasize << DMACHAN_CTRLA_BTSIZE_SHIFT) | otherbits;

  /* Since this is a receive, the source is described by the peripheral selections */

  ndx = (dmach->flags & DMACH_FLAG_PERIPHWIDTH_MASK) >> DMACH_FLAG_PERIPHWIDTH_SHIFT;
  DEBUGASSERT(ndx < 3);
  regval |= g_srcwidth[ndx];

  /* Since this is a receive, the destination is described by the memory selections */

  ndx = (dmach->flags & DMACH_FLAG_MEMWIDTH_MASK) >> DMACH_FLAG_MEMWIDTH_SHIFT;
  DEBUGASSERT(ndx < 3);
  regval |= g_destwidth[ndx];
  return regval;
}

/************************************************************************************
 * Name: sam3u_srcctrlb
 *
 * Description:
 *  Set source related CTRLB fields
 *
 ************************************************************************************/

static void sam3u_srcctrlb(struct sam3u_dma_s *dmach, bool lli, bool autoincr)
{
  uint32_t regval;

  /* Fetch CTRLB and clear the configurable bits */

  regval = getreg32(dmach->base + SAM3U_DMACHAN_CTRLB_OFFSET);
  regval &= ~ (DMACHAN_CTRLB_SRCDSCR | DMACHAN_CTRLB_SRCINCR_MASK | 1<<31);

  /* Disable the source descriptor if we are not using the LLI transfer mode */

  if (lli)
    {
      regval |= DMACHAN_CTRLB_SRCDSCR;
    }

  /* Select address incrementing */

  regval |= autoincr ? DMACHAN_CTRLB_SRCINCR_INCR ? DMACHAN_CTRLB_SRCINCR_FIXED;

  /* Save the updated CTRLB value */

  putreg32(regval, dmach->base + SAM3U_DMACHAN_CTRLB_OFFSET)
}

/************************************************************************************
 * Name: sam3u_destctrlb
 *
 * Description:
 *  Set destination related CTRLB fields
 *
 ************************************************************************************/

static void sam3u_destctrlb(struct sam3u_dma_s *dmach, bool lli, bool autoincr)
{
  uint32_t regval;

  /* Fetch CTRLB and clear the configurable bits */

  regval = getreg32(dmach->base + SAM3U_DMACHAN_CTRLB_OFFSET);
  regval &= ~ (DMACHAN_CTRLB_DSTDSCR | DMACHAN_CTRLB_DSTINCR_MASK);

  /* Disable the source descriptor if we are not using the LLI transfer mode */

  if (lli)
    {
      regval |= DMACHAN_CTRLB_DSTDSCR;
    }

  /* Select address incrementing */

  regval |= autoincr ? DMACHAN_CTRLB_DESTINCR_INCR ? DMACHAN_CTRLB_DESTINCR_FIXED;
        
  /* Save the updated CTRLB value */

  putreg32(regval, dmach->base + SAM3U_DMACHAN_CTRLB_OFFSET)
}

/************************************************************************************
 * Name: sam3u_flowcontrol
 *
 * Description:
 *  Select flow control
 *
 ************************************************************************************/

static inline void sam3u_flowcontrol(struct sam3u_dma_s *dmach, uint32_t setting)
{
    uint32_t regval;

    regval = getreg32(dmach->base + SAM3U_DMACHAN_CTRLB_OFFSET);
    regval &= ~(DMACHAN_CTRLB_FC_MASK);
    regval |= setting;
    putreg(regval, dmach->base + SAM3U_DMACHAN_CTRLB_OFFSET);   
}

/************************************************************************************
 * Name: sam3u_dmainterrupt
 *
 * Description:
 *  DMA interrupt handler
 *
 ************************************************************************************/

static int sam3u_dmainterrupt(int irq, void *context)
{
# warning "Missing logic"
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam3u_dmainitialize
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
  /* Enable peripheral clock */

  putreg32((1 << SAM3U_PID_DMAC), SAM3U_PMC_PCER);

  /* Disable all DMA interrupts */

  putreg32(DMAC_DBC_ERR_ALLINTS, SAM3U_DMAC_EBCIDR);

  /* Disable all DMA channels */

  putreg32(DMAC_CHDR_DIS_ALL, SAM3U_DMAC_CHDR);

  /* Attach DMA interrupt vector */

  (void)irq_attach(SAM3U_IRQ_DMAC, sam3u_dmainterrupt);

  /* Enable the IRQ at the NVIC (still disabled at the DMA controller) */

  up_enable_irq(SAM3U_IRQ_DMAC);

  /* Enable the DMA controller */

  putreg32(DMAC_EN_ENABLE, SAM3U_DMAC_EN); 
}

/****************************************************************************
 * Name: sam3u_dmachannel
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

DMA_HANDLE sam3u_dmachannel(uint16_t dmach_flags)
{
  struct sam3u_dma_s *dmach;
  unsigned int chndx;
  uint32_t regval;

  /* Get the search parameters */

  bool flowcontrol = sam3u_flowcontrol(dmach_flags);
  unsigned int fifosize = sam3u_fifosize(dmach_flags);

  /* Search for an available DMA channel with at least the requested FIFO
   * size.
   */

  dmach = NULL;
  sam3u_dmatake();
  for (chndx = 0; chndx < CONFIG_SAM3U_NDMACHAN; chndx++)
    {
      struct sam3u_dma_s *candidate = &g_dma[chndx];
      if (!candidate->inuse &&
          (sam3u_fifosize(candidate->flags) >= fifosize) &&
          (!flowcontrol || sam3u_flowcontrol(dmach_flags)))
        {
          dmach         = candidate;
          dmach->inuse  = true;

          /* Read the status register to clear any pending interrupts on the channel */

          (void)getreg32(SAM3U_DMAC_EBCISR);

          /* Disable the channel by writing one to the write-only channel disable register */
 
          putreg32(DMAC_CHDR_DIS(chndx), SAM3U_DMAC_CHDR);

          /* See the DMA channel flags, retaining the fifo size and flow control
           * settings which are inherent properties of the FIFO and cannot be changed.
           */

          dmach->flags &= (DMACH_FLAG_FLOWCONTROL | DMACH_FLAG_FIFOSIZE_MASK);
          dmach->flags |= (dma_flags & ~((DMACH_FLAG_FLOWCONTROL | DMACH_FLAG_FIFOSIZE_MASK)));

          /* Initialize the transfer state */

          dmach->remaining = 0;
          break;
        }
    }
  sam3u_dmagive();
  return (DMA_HANDLE)dmach;
}

/****************************************************************************
 * Name: sam3u_dmafree
 *
 * Description:
 *   Release a DMA channel.  NOTE:  The 'handle' used in this argument must
 *   NEVER be used again until sam3u_dmachannel() is called again to re-gain
 *   a valid handle.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void sam3u_dmafree(DMA_HANDLE handle)
{
  struct sam3u_dma_s *dmach = (struct sam3u_dma_s *)handle;

  /* Mark the channel no longer in use.  Clearing the inuse flag is an atomic
   * operation and so should be safe.
   */

  DEBUGASSERT((dmach != NULL) && (dmach->inuse));
  dmach->flags &= (DMACH_FLAG_FLOWCONTROL | DMACH_FLAG_FIFOSIZE_MASK);
  dmach->inuse  = false;                   /* No longer in use */
}

/****************************************************************************
 * Name: sam3u_dmatxsetup
 *
 * Description:
 *   Configure DMA for transmit (memory to peripheral) before using
 *
 ****************************************************************************/

void sam3u_dmarxsetup(DMA_HANDLE handle, uint32_t paddr, uint32_t maddr, size_t nbytes)
{
  struct sam3u_dma_s *dmach = (struct sam3u_dma_s *)handle;
  uint32_t regval;
# warning "Missing logic"
}

/****************************************************************************
 * Name: sam3u_dmarxsetup
 *
 * Description:
 *   Configure DMA for receuve (peripheral to memory) before using
 *
 ****************************************************************************/

void sam3u_dmarxsetup(DMA_HANDLE handle, uint32_t paddr, uint32_t maddr, size_t nbytes)
{
  struct sam3u_dma_s *dmach = (struct sam3u_dma_s *)handle;
  uint32_t regval;
# warning "Missing logic"
}

/****************************************************************************
 * Name: sam3u_dmastart
 *
 * Description:
 *   Start the DMA transfer
 *
 * Assumptions:
 *   - DMA handle allocated by sam3u_dmachannel()
 *   - No DMA in progress
 *
 ****************************************************************************/

void sam3u_dmastart(DMA_HANDLE handle, dma_callback_t callback, void *arg, bool half)
{
  struct sam3u_dma_s *dmach = (struct sam3u_dma_s *)handle;

  DEBUGASSERT(handle != NULL);

  /* Save the callback info.  This will be invoked whent the DMA commpletes */

  dmach->callback = callback;
  dmach->arg      = arg;
# warning "Missing logic"
}

/****************************************************************************
 * Name: sam3u_dmastop
 *
 * Description:
 *   Cancel the DMA.  After sam3u_dmastop() is called, the DMA channel is
 *   reset and sam3u_dmasetup() must be called before sam3u_dmastart() can be
 *   called again
 *
 * Assumptions:
 *   - DMA handle allocated by sam3u_dmachannel()
 *
 ****************************************************************************/

void sam3u_dmastop(DMA_HANDLE handle)
{
  struct sam3u_dma_s *dmach = (struct sam3u_dma_s *)handle;

  /* Disable the channel by writing one to the write-only channel disable register */
 
  DEBUGASSERT(dmach != NULL);
  putreg32(DMAC_CHDR_DIS(dmach->chan), SAM3U_DMAC_CHDR);
}

/****************************************************************************
 * Name: sam3u_dmasample
 *
 * Description:
 *   Sample DMA register contents
 *
 * Assumptions:
 *   - DMA handle allocated by sam3u_dmachannel()
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_DMA
void sam3u_dmasample(DMA_HANDLE handle, struct sam3u_dmaregs_s *regs)
{
  struct sam3u_dma_s *dmach = (struct sam3u_dma_s *)handle;
  irqstate_t flags;

  /* Sample global registers.  NOTE: reading EBCISR clears interrupts, but
   * that should be okay IF interrupts are enabled when this function is
   * called.  But there is a race condition where this instrumentation could
   * cause lost interrupts.
   */

  flags        = irqsave();
  regs->gcfg   = getreg32(SAM3U_DMAC_GCFG);
  regs->en     = getreg32(SAM3U_DMAC_EN);
  regs->sreq   = getreg32(SAM3U_DMAC_SREQ);
  regs->creq   = getreg32(SAM3U_DMAC_CREQ);
  regs->last   = getreg32(SAM3U_DMAC_LAST);
  regs->ebcimr = getreg32(SAM3U_DMAC_EBCIMR);
  regs->ebcisr = getreg32(SAM3U_DMAC_EBCISR);
  regs->chsr   = getreg32(SAM3U_DMAC_CHSR);

  /* Sample channel registers */

  regs->saddr  = getreg32(dmach->base + SAM3U_DMACHAN_SADDR_OFFSET);
  regs->daddr  = getreg32(dmach->base + SAM3U_DMACHAN_DADDR_OFFSET);
  regs->dscr   = getreg32(dmach->base + SAM3U_DMACHAN_DSCR_OFFSET);
  regs->ctrla  = getreg32(dmach->base + SAM3U_DMACHAN_CTRLA_OFFSET);
  regs->ctrlb  = getreg32(dmach->base + SAM3U_DMACHAN_CTRLB_OFFSET);
  regs->cfg    = getreg32(dmach->base + SAM3U_DMACHAN_CFG_OFFSET);
  irqrestore(flags);
}
#endif

/****************************************************************************
 * Name: sam3u_dmadump
 *
 * Description:
 *   Dump previously sampled DMA register contents
 *
 * Assumptions:
 *   - DMA handle allocated by sam3u_dmachannel()
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_DMA
void sam3u_dmadump(DMA_HANDLE handle, const struct sam3u_dmaregs_s *regs,
                   const char *msg)
{
  struct sam3u_dma_s *dmach = (struct sam3u_dma_s *)handle;

  dmadbg("%s\n", msg);
  dmadbg("  DMA Global Registers:\n");
  dmadbg("      GCFG[%08x]: %08x\n", SAM3U_DMAC_GCFG, regs->gcfg);
  dmadbg("        EN[%08x]: %08x\n", SAM3U_DMAC_EN, regs->en);
  dmadbg("      SREQ[%08x]: %08x\n", SAM3U_DMAC_SREQ, regs->sreq);
  dmadbg("      CREQ[%08x]: %08x\n", SAM3U_DMAC_CREQ, regs->creq);
  dmadbg("      LAST[%08x]: %08x\n", SAM3U_DMAC_LAST, regs->last);
  dmadbg("    EBCIMR[%08x]: %08x\n", SAM3U_DMAC_EBCIMR, regs->ebcimr);
  dmadbg("    EBCISR[%08x]: %08x\n", SAM3U_DMAC_EBCISR, regs->ebcisr);
  dmadbg("      CHSR[%08x]: %08x\n", SAM3U_DMAC_CHSR, regs->chsr);
  dmadbg("  DMA Channel Registers:\n");
  dmadbg("     SADDR[%08x]: %08x\n", dmach->base + SAM3U_DMACHAN_SADDR_OFFSET, regs->saddr);
  dmadbg("     DADDR[%08x]: %08x\n", dmach->base + SAM3U_DMACHAN_DADDR_OFFSET, regs->daddr);
  dmadbg("      DSCR[%08x]: %08x\n", dmach->base + SAM3U_DMACHAN_DSCR_OFFSET, regs->dscr);
  dmadbg("     CTRLA[%08x]: %08x\n", dmach->base + SAM3U_DMACHAN_CTRLA_OFFSET, regs->ctrla);
  dmadbg("     CTRLB[%08x]: %08x\n", dmach->base + SAM3U_DMACHAN_CTRLB_OFFSET, regs->ctrlb);
  dmadbg("       CFG[%08x]: %08x\n", dmach->base + SAM3U_DMACHAN_CFG_OFFSET, regs->cfg);
}
#endif

