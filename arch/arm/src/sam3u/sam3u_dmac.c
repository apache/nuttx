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
  uint8_t           fifosize;   /* Size of DMA FIFO in btyes */
  bool              inuse;      /* TRUE: The DMA channel is in use */
  uint32_t          base;       /* DMA register channel base address */
  dma_callback_t    callback;   /* Callback invoked when the DMA completes */
  void             *arg;        /* Argument passed to callback function */
  volatile uint16_t xfrsize;    /* Total transfer size */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This semaphore protects the DMA channel table */

static sem_t g_dmasem;

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
    .fifosize = 8,
    .base     = SAM3U_DMACHAN0_BASE,
  },
  {
    .chan     = 1,
    .fifosize = 8,
    .base     = SAM3U_DMACHAN1_BASE,
  },
  {
    .chan     = 2,
    .fifosize = 8,
    .base     = SAM3U_DMACHAN2_BASE,
  },
  {
    .chan     = 3,
    .fifosize = 32,
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
 *   Allocate a DMA channel.  This function gives the caller mutually
 *   sets aside a DMA channel with the required FIFO size and gives the
 *   caller exclusive access to the DMA channelt.
 *
 * Returned Value:
 *   If a DMA channel if the required FIFO size is available, this function
 *   returns a non-NULL, void* DMA channel handle.  NULL is returned on any
 *   failure.
 *
 ****************************************************************************/

DMA_HANDLE sam3u_dmachannel(unsigned int fifosize)
{
  struct sam3u_dma_s *dmach;
  unsigned int chndx;
  uint32_t regval;

  /* Search for an available DMA channel with at least the requested FIFO
   * size.
   */

  dmach = NULL;
  sam3u_dmatake();
  for (chndx = 0; chndx < CONFIG_SAM3U_NDMACHAN; chndx++)
    {
      struct sam3u_dma_s *candidate = &g_dma[chndx];
      if (!candidate->inuse && candidate->fifosize >= fifosize)
        {
          dmach        = candidate;
          dmach->inuse = true;
          break;
        }
    }
  sam3u_dmagive();

  /* Did we get one? */

  if (dmach)
    {
      /* Read the status register to clear any pending interrupts on the channel */

      (void)getreg32(SAM3U_DMAC_EBCISR);

      /* Disable the channel by writing one to the write-only channel disable register */
 
      putreg32(DMAC_CHDR_DIS(chndx), SAM3U_DMAC_CHDR);

      /* Initilize the transfer size */

      dmach->xfrsize = 0;
    }

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

  /* Mark the channel no longer in use.  This is an atomic operation and so
   * should be safe.
   */

  DEBUGASSERT(dmach != NULL);
  dmach->inuse = true;
}

/****************************************************************************
 * Name: sam3u_dmatxsetup
 *
 * Description:
 *   Configure DMA for transmit (memory to peripheral) before using
 *
 ****************************************************************************/

void sam3u_dmarxsetup(DMA_HANDLE handle, uint32_t paddr, uint32_t maddr, size_t ntransfers, uint32_t ccr)
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

void sam3u_dmarxsetup(DMA_HANDLE handle, uint32_t paddr, uint32_t maddr, size_t ntransfers, uint32_t ccr)
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

