/****************************************************************************
 * arch/arm/src/samdl/sam_dmac.c
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
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

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Condition out the whole file unless DMA is selected in the configuration */

#ifdef CONFIG_SAMDL_DMAC

/* If SAMD/L support is enabled, then OS DMA support should also be enabled */

#ifndef CONFIG_ARCH_DMA
#  warning "SAM3/4 DMA enabled but CONFIG_ARCH_DMA disabled"
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure describes one DMA channel */

struct sam_dmach_s
{
  uint8_t         dc_chan;       /* DMA channel number (0-6) */
  bool            dc_inuse;      /* TRUE: The DMA channel is in use */
  uint32_t        dc_flags;      /* DMA channel flags */
  dma_callback_t  dc_callback;   /* Callback invoked when the DMA completes */
  void           *dc_arg;        /* Argument passed to callback function */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* These semaphores protect the DMA channel tables */

static sem_t g_chsem;

/* This array describes the state of each DMA channel */

static struct sam_dmach_s g_dmach[SAMDL_NDMACHAN];

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
 * Name: sam_dmaterminate
 *
 * Description:
 *   Terminate the DMA transfer and disable the DMA channel
 *
 ****************************************************************************/

static void sam_dmaterminate(struct sam_dmach_s *dmach, int result)
{
  /* Disable all channel interrupts */
#warning Missing logic

  /* Disable the channel */
#warning Missing logic

  /* Free the resources */
#warning Missing logic

  /* Perform the DMA complete callback */

  if (dmach->dc_callback)
    {
      dmach->dc_callback((DMA_HANDLE)dmach, dmach->dc_arg, result);
    }

  dmach->dc_callback = NULL;
  dmach->dc_arg      = NULL;
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
  struct sam_dmach_s *dmach;
  unsigned int chndx;

  /* Get the DMAC status register value.  Ignore all masked interrupt
   * status bits.
   */
#warning Missing logic

  /* Check if the any transfer has completed or any errors have occurred */
#warning Missing logic

    {
      /* Yes.. Check each bit  to see which channel has interrupted */
#warning Missing logic

      for (chndx = 0; chndx < SAMDL_NDMACHAN; chndx++)
        {
          dmach = &g_dmach[chndx];

          /* Are any interrupts pending for this channel? */
#warning Missing logic

            {
              /* Yes.. Did an error occur? */

                {
                   /* Yes... Terminate the transfer with an error? */

                  sam_dmaterminate(dmach, -EIO);
                }

              /* Is the transfer complete? */

               {
                  /* Yes.. Terminate the transfer with success */

                  sam_dmaterminate(dmach, OK);
                }
            }
        }
    }

  return OK;
}

/****************************************************************************
 * Name: sam_addrincr
 *
 * Description:
 *   Peripheral address increment for each beat.
 *
 ****************************************************************************/

#if 0 // Not used
static size_t sam_addrincr(struct sam_dmach_s *dmach)
{
  size_t beatsize;
  size_t stepsize;
  int shift

  /* How bit is one beat? {1,2,4} */

  shift    = (dmach->dc_flags & DMACH_FLAG_BEATSIZE_MASK) >> DMACH_FLAG_BEATSIZE_SHIFT;
  beatsize = (1 << shift);

  /* What is the address increment per beat? {1,4,6,...,128} */

  shift    = (dmach->dc_flags & DMACH_FLAG_STEPSIZE_MASK) >> DMACH_FLAG_STEPSIZE_SHIFT;
  stepsize = (1 << shift);

  return (beatsize * stepsize);
}
#endif

/****************************************************************************
 * Name: sam_maxtransfer
 *
 * Description:
 *   Maximum number of bytes that can be sent/received in one transfer
 *
 ****************************************************************************/

static size_t sam_maxtransfer(struct sam_dmach_s *dmach)
{
#warning Missing logic
  return 0;
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
#warning Missing logic
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
#warning Missing logic
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
  /* Clear any pending interrupts from any previous DMAC transfer.
   *
   * REVISIT: If DMAC interrupts are disabled at the NVIKC, then reading the
   * EBCISR register could cause a loss of interrupts!
   */
#warning Missing logic

  /* Set up the DMA */
#warning Missing logic

  /* Enable the channel */
#warning Missing logic

  /* Enable DMA interrupts */
#warning Missing logic

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
  /* Clear any pending interrupts from any previous DMAC transfer.
   *
   * REVISIT: If DMAC interrupts are disabled at the NVIKC, then reading the
   * EBCISR register could cause a loss of interrupts!
   */
#warning Missing logic

  /* Set up the initial DMA */
#warning Missing logic

  /* Enable the channel */
#warning Missing logic

  /* Enable DMA interrupts */
#warning Missing logic

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
  dmallvdbg("Initialize DMAC\n");
  int i;

  /* Initialize global semaphores */

  sem_init(&g_chsem, 0, 1);

  /* Initialized the DMA channel table */

  for (i = 0; i < SAMDL_NDMACHAN; i++)
    {
      g_dmach[i].dc_chan = i;
    }

  /* Enable peripheral clock */

  sam_dmac_enableperiph();

  /* Disable all DMA interrupts */
#warning Missing logic

  /* Disable all DMA channels */
#warning Missing logic

  /* Attach DMA interrupt vector */

  (void)irq_attach(SAM_IRQ_DMAC, sam_dmainterrupt);

  /* Enable the IRQ at the NVIC (still disabled at the DMA controller) */

  up_enable_irq(SAM_IRQ_DMAC);

  /* Enable the DMA controller */
#warning Missing logic
}

/****************************************************************************
 * Name: sam_dmachannel
 *
 * Description:
 *   Allocate a DMA channel.  This function sets aside a DMA channel and
 *   gives the caller exclusive access to the DMA channel.
 *
 *   The naming convention in all of the DMA interfaces is that one side is
 *   the 'peripheral' and the other is 'memory'.  However, the interface
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
  struct sam_dmach_s *dmach;
  unsigned int chndx;

  /* Search for an available DMA channel */

  dmach = NULL;
  sam_takechsem();

  for (chndx = 0; chndx < SAMDL_NDMACHAN; chndx++)
    {
      struct sam_dmach_s *candidate = &g_dmach[chndx];
      if (!candidate->dc_inuse)
        {
          dmach           = candidate;
          dmach->dc_inuse = true;

          /* Clear any pending interrupts.
           *
           * REVISIT: If DMAC interrupts are disabled at the NVIC, then
           * reading the EBCISR register could cause a loss of interrupts!
           */
#warning Missing logic

          /* Disable the channel */
#warning Missing logic

          /* Set the DMA channel flags */

          dmach->dc_flags = chflags;
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
 *   There are two channel usage models:  (1) The channel is allocated and
 *   configured in one step.  This is the typical case where a DMA channel performs
 *   a constant role.  The alternative is (2) where the DMA channel is reconfigured
 *   on the fly.  In this case, the chflags provided to sam_dmachannel are not used
 *   and sam_dmaconfig() is called before each DMA to configure the DMA channel
 *   appropriately.
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

void sam_dmaconfig(DMA_HANDLE handle, uint32_t chflags)
{
  struct sam_dmach_s *dmach = (struct sam_dmach_s *)handle;

  /* Set the new DMA channel flags. */

  dmavdbg("chflags: %08x\n",  (int)chflags);
  dmach->dc_flags = chflags;
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
  DEBUGASSERT((dmach != NULL) && (dmach->dc_inuse));

  /* Mark the channel no longer in use.  Clearing the inuse flag is an atomic
   * operation and so should be safe.
   */

  dmach->dc_flags = 0;
  dmach->dc_inuse = false;
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

int sam_dmatxsetup(DMA_HANDLE handle, uint32_t paddr, uint32_t maddr,
                   size_t nbytes)
{
  struct sam_dmach_s *dmach = (struct sam_dmach_s *)handle;
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

  maxtransfer = sam_maxtransfer(dmach);

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
           *
           * REVISIT: What if stepsize is not 1?
           */

          if ((dmach->dc_flags & DMACH_FLAG_PERIPHINCREMENT) != 0)
            {
              paddr += maxtransfer;
            }

          if ((dmach->dc_flags & DMACH_FLAG_MEMINCREMENT) != 0)
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
  struct sam_dmach_s *dmach = (struct sam_dmach_s *)handle;
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

  maxtransfer = sam_maxtransfer(dmach);

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
           *
           * REVISIT: What if stepsize is not 1?
           */

          if ((dmach->dc_flags & DMACH_FLAG_PERIPHINCREMENT) != 0)
            {
              paddr += maxtransfer;
            }

          if ((dmach->dc_flags & DMACH_FLAG_MEMINCREMENT) != 0)
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
  struct sam_dmach_s *dmach = (struct sam_dmach_s *)handle;
  int ret = -EINVAL;

  dmavdbg("dmach: %p callback: %p arg: %p\n", dmach, callback, arg);
  DEBUGASSERT(dmach != NULL);

  /* Verify that the DMA has been setup */
#warning Missing logic

    {
      /* Save the callback info.  This will be invoked when the DMA completes */

      dmach->dc_callback = callback;
      dmach->dc_arg      = arg;

      /* Is this a single block transfer?  Or a multiple block transfer? */
#warning Missing logic

      if (true)
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
  uintptr_t base;
  irqstate_t flags;

  /* Sample DMAC registers. */

  flags            = irqsave();
  regs->ctrl       = getreg16(SAM_DMAC_CTRL);       /* Control Register */
  regs->crcctrl    = getreg16(SAM_DMAC_CRCCTRL);    /* CRC Control Register */
  regs->crcdatain  = getreg32(SAM_DMAC_CRCDATAIN);  /* CRC Data Input Register */
  regs->crcchksum  = getreg32(SAM_DMAC_CRCCHKSUM);  /* CRC Checksum Register */
  regs->crcstatus  = getreg8(SAM_DMAC_CRCSTATUS);   /* CRC Status Register */
  regs->dbgctrl    = getreg8(SAM_DMAC_DBGCTRL);     /* Debug Control Register */
  regs->qosctrl    = getreg8(SAM_DMAC_QOSCTRL);     /* Quality of Service Control Register */
  regs->swtrigctrl = getreg32(SAM_DMAC_SWTRIGCTRL); /* Software Trigger Control Register */
  regs->prictrl0   = getreg32(SAM_DMAC_PRICTRL0);   /* Priority Control 0 Register */
  regs->intpend    = getreg16(SAM_DMAC_INTPEND);    /* Interrupt Pending Register */
  regs->intstatus  = getreg32(SAM_DMAC_INTSTATUS);  /* Interrupt Status Register */
  regs->busych     = getreg32(SAM_DMAC_BUSYCH);     /* Busy Channels Register */
  regs->pendch     = getreg32(SAM_DMAC_PENDCH);     /* Pending Channels Register */
  regs->active     = getreg32(SAM_DMAC_ACTIVE);     /* Active Channels and Levels Register */
  regs->baseaddr   = getreg32(SAM_DMAC_BASEADDR);   /* Descriptor Memory Section Base Address Register */
  regs->wrbaddr    = getreg32(SAM_DMAC_WRBADDR);    /* Write-Back Memory Section Base Address Register */
  regs->chid       = getreg8(SAM_DMAC_CHID);        /* Channel ID Register */
  regs->chctrla    = getreg8(SAM_DMAC_CHCTRLA);     /* Channel Control A Register */
  regs->chctrlb    = getreg32(SAM_DMAC_CHCTRLB);    /* Channel Control B Register */
  regs->chintflag  = getreg8(SAM_DMAC_CHINTFLAG);   /* Channel Interrupt Flag Status and Clear Register */
  regs->chstatus   = getreg8(SAM_DMAC_CHSTATUS);    /* Channel Status Register */

  /* LPSRAM Registers Relative to BASEADDR or WRBADDR */
#warning How do we get the base address

  regs->btctrl     = getreg16(base + SAM_LPSRAM_BTCTRL_OFFSET);
  regs->btcnt      = getreg16(base + SAM_LPSRAM_BTCNT_OFFSET);
  regs->srcaddr    = getreg32(base + SAM_LPSRAM_SRCADDR_OFFSET);
  regs->dstaddr    = getreg32(base + SAM_LPSRAM_DSTADDR_OFFSET);
  regs->descaddr   = getreg32(base + SAM_LPSRAM_DESCADDR_OFFSET);
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

  dmadbg("%s\n", msg);
  dmadbg("  DMAC Registers:\n");
  dmadbg("         CTRL: %04x      CRCCTRL: %04x       RCDATAIN: %08x  CRCCHKSUM: %08x\n",
         regs->ctrl, regs->crcctrl, regs->crcdatain, regs->crcchksum);
  dmadbg("    CRCSTATUS: %02x        DBGCTRL: %02x          QOSCTRL: %02x       SWTRIGCTRL: %08x\n",
         regs->crcstatus, regs->dbgctrl, regs->qosctrl, regs->swtrigctrl);
  dmadbg("     PRICTRL0: %08x  INTPEND: %04x     INSTSTATUS: %08x     BUSYCH: %08x\n",
         regs->prictrl0, regs->intpend, regs->intstatus, regs->busych);
  dmadbg("       PENDCH: %08x   ACTIVE: %08x   BASEADDR: %08x    WRBADDR: %08x\n",
         regs->pendch, regs->active, regs->baseaddr, regs->wrbaddr);
  dmadbg("     BASEADDR: %08x  WRBADDR: %08x       CHID: %08x    CHCTRLA: %08x\n",
         regs->baseaddr, regs->wrbaddr, regs->chid, regs->chctrla);
  dmadbg("         CHID: %02x       CHCRTRLA: %02x         CHCRTRLB: %08x   CHINFLAG: %02x\n",
         regs->chid, regs->chctrla, regs->chctrlb, regs->chintflag, 
  dmadbg("     CHSTATUS: %02x\n",
         regs->chstatus);

  /* LPSRAM Registers Relative to BASEADDR or WRBADDR */
#warning How do we get the base address

  dmadbg("  LPSRAM Values:\n");
  dmadbg("       BTCTRL: %04x        BTCNT: %04x        SRCADDR: %08x    DSTADDR: %08x\n",
         regs->btctrl, regs->btcnt, regs->srcaddr, regs->dstaddr);
  dmadbg("     DESCADDR: %08x\n", regs->descaddr);
}
#endif /* CONFIG_DEBUG_DMA */
#endif /* CONFIG_SAMDL_DMAC */
