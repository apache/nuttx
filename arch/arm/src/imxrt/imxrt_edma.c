/****************************************************************************
 * arch/arm/src/imxrt/sam3u_edma.c
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
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

#include "up_arch.h"
#include "cache.h"
#include "up_internal.h"
#include "sched/sched.h"

#include "chip.h"
#include "chip/imxrt_edma.h"
#include "chip/imxrt_dmamux.h"
#include "imxrt_edma.h"

#ifdef CONFIG_IMXRT_EDMA

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure describes one DMA channel */

struct imxrt_dmach_s
{
  uint8_t chan;                   /* DMA channel number (0-IMXRT_EDMA_NCHANNELS) */
  bool inuse;                     /* true: The DMA channel is in use */
  bool active;                    /* true: DMA has been started */
  bool rx;                        /* true: Peripheral to memory transfer */
  uint32_t flags;                 /* DMA channel flags */
  dma_callback_t callback;        /* Callback invoked when the DMA completes */
  void *arg;                      /* Argument passed to callback function */
  uint32_t rxaddr;                /* RX memory address */
  size_t rxsize;                  /* Size of RX memory region */
};

/* This structure describes the state of the eDMA controller */

struct imxrt_edma_s
{
  /* These semaphores protect the DMA channel and descriptor tables */

  sem_t chsem;                    /* Protects channel table */
  sem_t dsem;                     /* Protects descriptor table */

  /* This array describes each DMA channel */

  struct imxrt_dmach_s dmach[IMXRT_EDMA_NCHANNELS];
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The state of the eDMA */

static struct imxrt_edma_s g_edma;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imxrt_takechsem() and imxrt_givechsem()
 *
 * Description:
 *   Used to get exclusive access to the DMA channel table
 *
 ****************************************************************************/

static void imxrt_takechsem(void)
{
  int ret;

  do
    {
      /* Take the semaphore (perhaps waiting) */

      ret = nxsem_wait(&g_edma.chsem);

      /* The only case that an error should occur here is if the wait was
       * awakened by a signal.
       */

      DEBUGASSERT(ret == OK || ret == -EINTR || ret == -ECANCELED);
    }
  while (ret == -EINTR);
}

static inline void imxrt_givechsem(void)
{
  (void)nxsem_post(&g_edma.chsem);
}

/****************************************************************************
 * Name: imxrt_takedsem() and imxrt_givedsem()
 *
 * Description:
 *   Used to wait for availability of descriptors in the descriptor table.
 *
 ****************************************************************************/

static void imxrt_takedsem(void)
{
  int ret;

  do
    {
      /* Take the semaphore (perhaps waiting) */

      ret = nxsem_wait(&g_edma.dsem);

      /* The only case that an error should occur here is if the wait was
       * awakened by a signal.
       */

      DEBUGASSERT(ret == OK || ret == -EINTR || ret == -ECANCELED);
    }
  while (ret == -EINTR);
}

static inline void imxrt_givedsem(void)
{
  (void)nxsem_post(&g_edma.dsem);
}

/****************************************************************************
 * Name: imxrt_getdmach
 *
 * Description:
 *  Read a eDMA channel register
 *
 ****************************************************************************/

static inline uint32_t imxrt_getdmach(struct imxrt_dmach_s *dmach,
                                      unsigned int offset)
{
  return getreg32(IMXRT_EDMA_TCD_BASE(dmach->chan) + offset);
}

/****************************************************************************
 * Name: imxrt_putdmach
 *
 * Description:
 *  Write a value to a eDMA channel register
 *
 ****************************************************************************/

static inline void imxrt_putdmach(struct imxrt_dmach_s *dmach, uint32_t value,
                                  unsigned int offset)
{
  putreg32(value, IMXRT_EDMA_TCD_BASE(dmach->chan) + offset);
}

/****************************************************************************
 * Name: imxrt_dmaterminate
 *
 * Description:
 *   Terminate the DMA transfer and disable the DMA channel
 *
 ****************************************************************************/

static void imxrt_dmaterminate(struct imxrt_dmach_s *dmach, int result)
{
  struct imxrt_edma_s *dmac = imxrt_controller(dmach);

  /* Disable all channel interrupts */

  /* Disable the channel. */

  /* If this was an RX DMA (peripheral-to-memory), then invalidate the cache
   * to force reloads from memory.
   */

  if (dmach->rx)
    {
      arch_invalidate_dcache(dmach->rxaddr, dmach->rxaddr + dmach->rxsize);
    }

  /* Perform the DMA complete callback */

  if (dmach->callback)
    {
      dmach->callback((DMA_HANDLE)dmach, dmach->arg, result);
    }

  dmach->callback = NULL;
  dmach->arg      = NULL;
  dmach->active   = false;
}

/****************************************************************************
 * Name: imxrt_dmach_interrupt
 *
 * Description:
 *  DMA interrupt handler
 *
 ****************************************************************************/

static void imxrt_dmach_interrupt(struct imxrt_dmach_s *dmach)
{
#warning Missing logic
  /* Get the eDMA status register value.  Ignore all masked interrupt
   * status bits.
   */

  /* Check if the any transfer has completed or any errors have occurred. */

  imxrt_dmaterminate(dmach);
}

/****************************************************************************
 * Name: imxrt_edma_interrupt
 *
 * Description:
 *  DMA interrupt handler
 *
 ****************************************************************************/

static int imxrt_edma_interrupt(int irq, void *context, FAR void *arg)
{
  struct imxrt_dmach_s *dmach;
  unsigned int chan;

  /* 'arg' should the the DMA channel instance.
   *
   * NOTE that there are only 16 vectors for 32 DMA channels.  The 'arg' will
   * always be the lower-numbered DMA channel.  The other DMA channel will
   * have a channel number 16 greater that this one.
   */

  dmach = (struct imxrt_dmach_s *)arg;
  DEBUGASSERT(dmach != NULL);
  chan  = dmach->chan;
  DEBUGASSERT(chan < IMXRT_EDMA_NCHANNELS && dmach == &g_edma.dmach[chan]);

  /* Check for an interrupt on the lower numbered DMA channel */

  if (dmach->active)
    {
      imxrt_dmach_interrupt(dmach);
    }

  /* Check for an interrupt on the lower numbered DMA channel */

  chan += 16;
  DEBUGASSERT(chan < IMXRT_EDMA_NCHANNELS);
  dmach = &g_edma.dmach[chan];

  if (dmach->active)
    {
      imxrt_dmach_interrupt(dmach);
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
  int i;

  dmainfo("Initialize eDMA\n");

  /* Enable peripheral clock */

  /* Enable data structures */

  memset(&g_edma, 0, sizeof());
  for (i = 0; i < IMXRT_EDMA_NCHANNELS; i++)
    {
      g_edma.dmach[i].chan = i;
    }

  /* Initialize semaphores */

  nxsem_init(&g_edma.chsem, 0, 1);
  nxsem_init(&g_edma.dsem, 0, SAM_NDMACHAN);

  /* Attach DMA interrupt vectors.
   *
   * NOTE that there are only 16 vectors for 32 DMA channels.
   */

  (void)irq_attach(IMXRT_IRQ_EDMA0_16,  imxrt_edma_interrupt, &g_edma.dmach[0]);
  (void)irq_attach(IMXRT_IRQ_EDMA1_17,  imxrt_edma_interrupt, &g_edma.dmach[1]);
  (void)irq_attach(IMXRT_IRQ_EDMA2_18,  imxrt_edma_interrupt, &g_edma.dmach[2]);
  (void)irq_attach(IMXRT_IRQ_EDMA3_19,  imxrt_edma_interrupt, &g_edma.dmach[3]);
  (void)irq_attach(IMXRT_IRQ_EDMA4_20,  imxrt_edma_interrupt, &g_edma.dmach[4]);
  (void)irq_attach(IMXRT_IRQ_EDMA5_21,  imxrt_edma_interrupt, &g_edma.dmach[5]);
  (void)irq_attach(IMXRT_IRQ_EDMA6_22,  imxrt_edma_interrupt, &g_edma.dmach[6]);
  (void)irq_attach(IMXRT_IRQ_EDMA7_23,  imxrt_edma_interrupt, &g_edma.dmach[7]);
  (void)irq_attach(IMXRT_IRQ_EDMA8_24,  imxrt_edma_interrupt, &g_edma.dmach[8]);
  (void)irq_attach(IMXRT_IRQ_EDMA9_25,  imxrt_edma_interrupt, &g_edma.dmach[9]);
  (void)irq_attach(IMXRT_IRQ_EDMA10_26, imxrt_edma_interrupt, &g_edma.dmach[10]);
  (void)irq_attach(IMXRT_IRQ_EDMA11_27, imxrt_edma_interrupt, &g_edma.dmach[11]);
  (void)irq_attach(IMXRT_IRQ_EDMA12_28, imxrt_edma_interrupt, &g_edma.dmach[12]);
  (void)irq_attach(IMXRT_IRQ_EDMA13_29, imxrt_edma_interrupt, &g_edma.dmach[13]);
  (void)irq_attach(IMXRT_IRQ_EDMA14_30, imxrt_edma_interrupt, &g_edma.dmach[14]);
  (void)irq_attach(IMXRT_IRQ_EDMA15_31, imxrt_edma_interrupt, &g_edma.dmach[15]);

  /* Disable all DMA interrupts at the eDMA controller */

  /* Disable all DMA channels */

  /* Enable the DMA controller */

  /* Enable the IRQ at the NVIC (still disabled at the eDMA controller) */

  up_enable_irq(IMXRT_IRQ_EDMA0_16);
  up_enable_irq(IMXRT_IRQ_EDMA1_17);
  up_enable_irq(IMXRT_IRQ_EDMA2_18);
  up_enable_irq(IMXRT_IRQ_EDMA3_19);
  up_enable_irq(IMXRT_IRQ_EDMA4_20);
  up_enable_irq(IMXRT_IRQ_EDMA5_21);
  up_enable_irq(IMXRT_IRQ_EDMA6_22);
  up_enable_irq(IMXRT_IRQ_EDMA7_23);
  up_enable_irq(IMXRT_IRQ_EDMA8_24);
  up_enable_irq(IMXRT_IRQ_EDMA9_25);
  up_enable_irq(IMXRT_IRQ_EDMA10_26);
  up_enable_irq(IMXRT_IRQ_EDMA11_27);
  up_enable_irq(IMXRT_IRQ_EDMA12_28);
  up_enable_irq(IMXRT_IRQ_EDMA13_29);
  up_enable_irq(IMXRT_IRQ_EDMA14_30);
  up_enable_irq(IMXRT_IRQ_EDMA15_31);
}

/****************************************************************************
 * Name: imxrt_dmachannel
 *
 *   Allocate a DMA channel.  This function sets aside a DMA channel then
 *   gives the caller exclusive access to the DMA channel.
 *
 *   The naming convention in all of the DMA interfaces is that one side is
 *   the 'peripheral' and the other is 'memory'.  However, the interface
 *   could still be used if, for example, both sides were memory although
 *   the naming would be awkward.
 *
 * Returned Value:
 *   If a DMA channel is available, this function returns a non-NULL, void*
 *   DMA channel handle.  NULL is returned on any failure.
 *
 ****************************************************************************/

DMA_HANDLE imxrt_dmachannel(void)
{
  struct imxrt_dmach_s *dmach;
  unsigned int chndx;

  /* Search for an available DMA channel */

  dmach = NULL;
  imxrt_takechsem();
  for (chndx = 0; chndx < SAM_NDMACHAN; chndx++)
    {
      struct imxrt_dmach_s *candidate = &g_edma.dmach[chndx];
      if (!candidate->inuse)
        {
          dmach         = candidate;
          dmach->inuse  = true;
          dmach->active = false;

          /* Clear any pending interrupts on the channel */

          /* Disable the channel. */

          break;
        }
    }

  imxrt_givechsem();

  /* Show the result of the allocation */

  if (dmach != NULL)
    {
      dmainfo("CH%d: returning dmach: %p\n", dmach->chan, dmach);
    }
  else
    {
      dmaerr("ERROR: Failed allocate eDMA channel\n");
    }

  return (DMA_HANDLE)dmach;
}

/****************************************************************************
 * Name: imxrt_dmafree
 *
 * Description:
 *   Release a DMA channel.  NOTE:  The 'handle' used in this argument must
 *   NEVER be used again until imxrt_dmachannel() is called again to re-gain
 *   a valid handle.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void imxrt_dmafree(DMA_HANDLE handle)
{
  struct imxrt_dmach_s *dmach = (struct imxrt_dmach_s *)handle;

  dmainfo("dmach: %p\n", dmach);
  DEBUGASSERT(dmach != NULL && dmach->inuse && !dmach->active);

  /* Mark the channel no longer in use.  Clearing the inuse flag is an atomic
   * operation and so should be safe.
   */

  dmach->flags = 0;
  dmach->inuse = false;                   /* No longer in use */
  dmach->inuse = active;                  /* Better not be active */
}

/****************************************************************************
 * Name: imxrt_dmatxsetup
 *
 * Description:
 *   Configure DMA for transmit of one buffer (memory to peripheral).  This
 *   function may be called multiple times to handle large and/or dis-
 *   continuous transfers.  Calls to imxrt_dmatxsetup() and imxrt_dmarxsetup()
 *   must not be intermixed on the same transfer, however.
 *
 ****************************************************************************/

int imxrt_dmatxsetup(DMA_HANDLE handle, uint8_t pchan, uint32_t maddr,
                     size_t nbytes, uint32_t chflags)
{
  struct imxrt_dmach_s *dmach = (struct imxrt_dmach_s *)handle;
  int ret = OK;

  dmainfo("dmach: %p pchan: %u maddr: %08x nbytes: %d chflags %08x\n",
          dmach, (int)pchan, (int)maddr, (int)nbytes, (unsigned int)chflags);
  DEBUGASSERT(dmach != NULL);

#warning Missing logic

  /* Save an indication so that the DMA interrupt completion logic will know
   * that this was not an RX transfer.
   */

  dmach->rx     = false;
  dmach->active = true;

  /* Clean caches associated with the DMA memory */

  arch_clean_dcache(maddr, maddr + nbytes);
  return ret;
}

/****************************************************************************
 * Name: imxrt_dmarxsetup
 *
 * Description:
 *   Configure DMA for receipt of one buffer (peripheral to memory).  This
 *   function may be called multiple times to handle large and/or dis-
 *   continuous transfers.  Calls to imxrt_dmatxsetup() and imxrt_dmarxsetup()
 *   must not be intermixed on the same transfer, however.
 *
 ****************************************************************************/

int imxrt_dmarxsetup(DMA_HANDLE handle, uint8_t pchan, uint32_t maddr,
                     size_t nbytes, uint32_t chflags)
{
  struct imxrt_dmach_s *dmach = (struct imxrt_dmach_s *)handle;
  size_t maxtransfer;
  size_t remaining;
  int ret = OK;

  dmainfo("dmach: %p pchan: %u maddr: %08x nbytes: %d chflags %08x\n",
          dmach, (int)pchan, (int)maddr, (int)nbytes, (unsigned int)chflags);
  DEBUGASSERT(dmach);

#warning Missing logic

  /* Save an indication so that the DMA interrupt completion logic will know
   * that this was an RX transfer and will invalidate the cache.
   */

  dmach->rx     = true;
  dmach->rxaddr = maddr;
  dmach->rxsize = nbytes;
  dmach->active = true;

  /* Clean caches associated with the DMA memory */

  arch_clean_dcache(maddr, maddr + nbytes);
  return ret;
}

/****************************************************************************
 * Name: imxrt_dmastart
 *
 * Description:
 *   Start the DMA transfer
 *
 ****************************************************************************/

int imxrt_dmastart(DMA_HANDLE handle, dma_callback_t callback, void *arg)
{
  struct imxrt_dmach_s *dmach = (struct imxrt_dmach_s *)handle;
  int ret = -EINVAL;

  dmainfo("dmach: %p callback: %p arg: %p\n", dmach, callback, arg);
  DEBUGASSERT(dmach != NULL);

  /* Verify that the DMA has been setup (i.e., at least one entry in the
   * link list).
   */

  /* Save the callback info.  This will be invoked whent the DMA commpletes */

  dmach->callback = callback;
  dmach->arg      = arg;

#warning Missing logic

  return ret;
}

/****************************************************************************
 * Name: imxrt_dmastop
 *
 * Description:
 *   Cancel the DMA.  After imxrt_dmastop() is called, the DMA channel is
 *   reset and imxrt_dmarx/txsetup() must be called before imxrt_dmastart()
 *   can be called again
 *
 ****************************************************************************/

void imxrt_dmastop(DMA_HANDLE handle)
{
  struct imxrt_dmach_s *dmach = (struct imxrt_dmach_s *)handle;
  irqstate_t flags;

  dmainfo("dmach: %p\n", dmach);
  DEBUGASSERT(dmach != NULL);

  flags = spin_lock_irqsave();
  imxrt_dmaterminate(dmach, -EINTR);
  spin_unlock_irqrestore(flags);
}

/****************************************************************************
 * Name: imxrt_dmasample
 *
 * Description:
 *   Sample DMA register contents
 *
 * Assumptions:
 *   - DMA handle allocated by imxrt_dmachannel()
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_DMA
void imxrt_dmasample(DMA_HANDLE handle, struct imxrt_dmaregs_s *regs)
{
  struct imxrt_dmach_s *dmach = (struct imxrt_dmach_s *)handle;
  uintptr_t regaddr;
  unsigned int chan;
  irqstate_t flags;

  DEBUGASSERT(dmach != NULL && regs != NULL);
  chan           = dmach->chan;
  regs->chan     = chan;

  /* eDMA Global Registers */

  flags          = spin_lock_irqsave();

  regs->cr       = getreg32(IMXRT_EDMA_CR);   /* Control */
  regs->es       = getreg32(IMXRT_EDMA_ES);   /* Error Status */
  regs->erq      = getreg32(IMXRT_EDMA_ERQ);  /* Enable Request */
  regs->req      = getreg32(IMXRT_EDMA_INT);  /* Interrupt Request */
  regs->err      = getreg32(IMXRT_EDMA_ERR);  /* Error */
  regs->hrs      = getreg32(IMXRT_EDMA_HRS);  /* Hardware Request Status */
  regs->ears     = getreg32(IMXRT_EDMA_EARS); /* Enable Asynchronous Request in Stop */

  /* eDMA Channel registers */

  regaddr        = IMXRT_EDMA_DCHPRI(chan);
  regs->dchpri   = getreg8(regaddr);          /* Channel priority */

  /* eDMA TCD */

  base           = IMXRT_EDMA_TCD_BASE(chan);
  regs->saddr    = getreg32(base + IMXRT_EDMA_TCD_SADDR_OFFSET);
  regs->soff     = getreg16(base + IMXRT_EDMA_TCD_SOFF_OFFSET);
  regs->attr     = getreg16(base + IMXRT_EDMA_TCD_ATTR_OFFSET);
  regs->nbml     = getreg32(base + IMXRT_EDMA_TCD_NBYTES_ML_OFFSET);
  regs->slast    = getreg32(base + IMXRT_EDMA_TCD_SLAST_OFFSET);
  regs->daddr    = getreg32(base + IMXRT_EDMA_TCD_DADDR_OFFSET);
  regs->doff     = getreg16(base + IMXRT_EDMA_TCD_DOFF_OFFSET);
  regs->citer    = getreg16(base + IMXRT_EDMA_TCD_CITER_ELINK_OFFSET);
  regs->dlastsga = getreg32(base + IMXRT_EDMA_TCD_DLASTSGA_OFFSET);
  regs->csr      = getreg16(base + IMXRT_EDMA_TCD_CSR_OFFSET);
  regs->biter    = getreg16(base + IMXRT_EDMA_TCD_BITER_ELINK_OFFSET);

  /* DMAMUX registers */

  regaddr        = IMXRT_DMAMUX_CHCF(chan);
  regs->dmamux   = getreg32(regaddr);         /* Channel configuration */

  spin_unlock_irqrestore(flags);
}
#endif /* CONFIG_DEBUG_DMA */

/****************************************************************************
 * Name: imxrt_dmadump
 *
 * Description:
 *   Dump previously sampled DMA register contents
 *
 * Assumptions:
 *   - DMA handle allocated by imxrt_dmachannel()
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_DMA
void imxrt_dmadump(const struct imxrt_dmaregs_s *regs, const char *msg)
{
  unsigned int chan;

  DEBUGASSERT(regs != NULL && msg != NULL);

  chan = regs->chan;
  DEBUGASSERT(chan < IMXRT_EDMA_NCHANNELS);

  dmainfo("%s\n", msg);
  dmainfo("  eDMA Global Registers:\n");
  dmainfo("          CR: %08x\n", regs->cr);
  dmainfo("          ES: %08x\n", regs->es);
  dmainfo("         ERQ: %08x\n", regs->erq);
  dmainfo("         INT: %08x\n", regs->req);
  dmainfo("         ERR: %08x\n", regs->err);
  dmainfo("        EARS: %08x\n", regs->hrs);

  /* eDMA Channel registers */

  dmainfo("  eDMA Channel %u Registers:\n", chan);
  dmainfo("    DCHPRI: %02x\n", regs->dchpri);

  /* eDMA TCD */

  dmainfo("  eDMA Channel %u TCD Registers:\n", chan);
  dmainfo("       SADDR: %08x\n", regs->saddr);
  dmainfo("        SOFF: %04x\n", regs->soff);
  dmainfo("        ATTR: %04x\n", regs->attr);
  dmainfo("        NBML: %05x\n", regs->nbml);
  dmainfo("       SLAST: %05x\n", regs->slast);
  dmainfo("       DADDR: %05x\n", regs->daddr);
  dmainfo("        DOFF: %04x\n", regs->doff);
  dmainfo("       CITER: %04x\n", regs->citer);
  dmainfo("    DLASTSGA: %08x\n", regs->dlastsga);
  dmainfo("         CSR: %04x\n", regs->csr);
  dmainfo("       BITER: %04x\n", regs->biter);

  /* DMAMUX registers */

  dmainfo("  DMAMUX Channel %u Registers:\n", chan);
  dmainfo("      DMAMUX: %08x\n", regs->dmamux);
}
#endif /* CONFIG_DEBUG_DMA */
#endif /* CONFIG_IMXRT_EDMA */
