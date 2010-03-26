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
  uint8_t        chan;     /* DMA channel number (0-6) */
//  uint8_t        irq;      /* DMA channel IRQ number */
  sem_t          sem;      /* Used to wait for DMA channel to become available */
  uint32_t       base;     /* DMA register channel base address */
  dma_callback_t callback; /* Callback invoked when the DMA completes */
  void          *arg;      /* Argument passed to callback function */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This array describes the state of each DMA */

#if CONFIG_SAM3U_NDMACHAN != 4
#  error "Logic here assumes CONFIG_SAM3U_NDMACHAN is 4"
#endif

static struct sam3u_dma_s g_dma[CONFIG_SAM3U_NDMACHAN] =
{
  {
    .chan     = 0,
//    .irq      = SAM3U_IRQ_DMA1CH1,
    .base     = SAM3U_DMACHAN0_BASE,
  },
  {
    .chan     = 1,
//    .irq      = SAM3U_IRQ_DMA1CH2,
    .base     = SAM3U_DMACHAN1_BASE,
  },
  {
    .chan     = 2,
//    .irq      = SAM3U_IRQ_DMA1CH3,
    .base     = SAM3U_DMACHAN2_BASE,
  },
  {
    .chan     = 3,
//    .irq      = SAM3U_IRQ_DMA1CH4,
    .base     = SAM3U_DMACHAN3_BASE,
  }
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

static void sam3u_dmatake(FAR struct sam3u_dma_s *dmach)
{
  /* Take the semaphore (perhaps waiting) */

  while (sem_wait(&dmach->sem) != 0)
    {
      /* The only case that an error should occur here is if the wait was awakened
       * by a signal.
       */

      ASSERT(errno == EINTR);
    }
}

static inline void sam3u_dmagive(FAR struct sam3u_dma_s *dmach)
{
  (void)sem_post(&dmach->sem);
}

/************************************************************************************
 * Name: sam3u_dmachandisable
 *
 * Description:
 *  Disable the DMA channel
 *
 ************************************************************************************/

static void sam3u_dmachandisable(struct sam3u_dma_s *dmach)
{
  /* Disable all interrupts at the DMA controller */

  /* Disable the DMA channel */

  /* Clear pending channel interrupts */
# warning "Missing logic"
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
  struct sam3u_dma_s *dmach;
  int chndx;

  /* Initialize each DMA channel */

  for (chndx = 0; chndx < CONFIG_SAM3U_NDMACHAN; chndx++)
    {
      dmach = &g_dma[chndx];
      sem_init(&dmach->sem, 0, 1);

      /* Attach DMA interrupt vectors */

//      (void)irq_attach(dmach->irq, sam3u_dmainterrupt);

      /* Disable the DMA channel */

      sam3u_dmachandisable(dmach);

      /* Enable the IRQ at the NVIC (still disabled at the DMA controller) */

//      up_enable_irq(dmach->irq);
    }
}

/****************************************************************************
 * Name: sam3u_dmachannel
 *
 * Description:
 *   Allocate a DMA channel.  This function gives the caller mutually
 *   exclusive access to the DMA channel specified by the 'chndx' argument.
 *   DMA channels are shared on the SAM3U:  Devices sharing the same DMA
 *   channel cannot do DMA concurrently!  See the DMACHAN_* definitions in
 *   sam3u_dma.h.
 *
 *   If the DMA channel is not available, then sam3u_dmachannel() will wait
 *   until the holder of the channel relinquishes the channel by calling
 *   sam3u_dmafree().  WARNING: If you have two devices sharing a DMA
 *   channel and the code never releases the channel, the sam3u_dmachannel
 *   call for the other will hang forever in this function!  Don't let your
 *   design do that!
 *
 *   Hmm.. I suppose this interface could be extended to make a non-blocking
 *   version.  Feel free to do that if that is what you need.
 *
 * Returned Value:
 *   Provided that 'chndx' is valid, this function ALWAYS returns a non-NULL,
 *   void* DMA channel handle.  (If 'chndx' is invalid, the function will
 *   assert if debug is enabled or do something ignorant otherwise).
 *
 * Assumptions:
 *   - The caller does not hold he DMA channel.
 *   - The caller can wait for the DMA channel to be freed if it is no
 *     available.
 *
 ****************************************************************************/

DMA_HANDLE sam3u_dmachannel(int chndx)
{
  struct sam3u_dma_s *dmach = &g_dma[chndx];

  DEBUGASSERT(chndx < CONFIG_SAM3U_NDMACHAN);

  /* Get exclusive access to the DMA channel -- OR wait until the channel
   * is available if it is currently being used by another driver
   */

  sam3u_dmatake(dmach);

  /* The caller now has exclusive use of the DMA channel */

  return (DMA_HANDLE)dmach;
}

/****************************************************************************
 * Name: sam3u_dmafree
 *
 * Description:
 *   Release a DMA channel.  If another thread is waiting for this DMA channel
 *   in a call to sam3u_dmachannel, then this function will re-assign the
 *   DMA channel to that thread and wake it up.  NOTE:  The 'handle' used
 *   in this argument must NEVER be used again until sam3u_dmachannel() is
 *   called again to re-gain access to the channel.
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   - The caller holds the DMA channel.
 *   - There is no DMA in progress
 *
 ****************************************************************************/

void sam3u_dmafree(DMA_HANDLE handle)
{
  struct sam3u_dma_s *dmach = (struct sam3u_dma_s *)handle;

  DEBUGASSERT(handle != NULL);

  /* Release the channel */

  sam3u_dmagive(dmach);
}

/****************************************************************************
 * Name: sam3u_dmasetup
 *
 * Description:
 *   Configure DMA before using
 *
 ****************************************************************************/

void sam3u_dmasetup(DMA_HANDLE handle, uint32_t paddr, uint32_t maddr, size_t ntransfers, uint32_t ccr)
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
  sam3u_dmachandisable(dmach);
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

  /* Sample global registers */

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

