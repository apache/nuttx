/****************************************************************************
 * arch/arm/src/lpc54xx/lpc54_dma.c
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
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

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>

#include "up_internal.h"
#include "up_arch.h"

#include "chip/lpc54_dma.h"
#include "lpc54_enableclk.h"
#include "lpc54_reset.h"
#include "lpc54_dma.h"

#ifdef CONFIG_LPC54_DMA

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure represents the state of one DMA channel */

struct lpc54_dmach_s
{
  uint8_t chn;             /* The DMA channel number */
  bool inuse;              /* True: The channel is in use */
  bool inprogress;         /* True: DMA is in progress on this channel */
  uint16_t nxfrs;          /* Number of bytes to transfers */
  dma_callback_t callback; /* DMA completion callback function */
  void *arg;               /* Argument to pass to the callback function */
};

/* This structure represents the state of the LPC54 DMA block */

struct lpc54_dma_s
{
  sem_t exclsem;           /* For exclusive access to the DMA channel list */

  /* This is the state of each DMA channel */

  struct lpc54_dmach_s dmach[LPC54_DMA_NCHANNELS];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The state of the LPC54 DMA block */

static struct lpc54_dma_s g_dma;

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* If the following value is zero, then there is no DMA in progress. This
 * value is needed in the IDLE loop to determine if the IDLE loop should
 * go into lower power power consumption modes.  According to the LPC54xx
 * User Manual: "The DMA controller can continue to work in Sleep mode, and
 * has access to the peripheral SRAMs and all peripheral registers. The
 * flash memory and the Main SRAM are not available in Sleep mode, they are
 * disabled in order to save power."
 */

volatile uint8_t g_dma_inprogress;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc54_dma_inprogress
 *
 * Description:
 *   Another DMA has started. Increment the g_dma_inprogress counter.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void lpc54_dma_inprogress(struct lpc54_dmach_s *dmach)
{
  irqstate_t flags;

  /* Increment the DMA in progress counter */

  flags = enter_critical_section();
  DEBUGASSERT(!dmach->inprogress && g_dma_inprogress < LPC54_DMA_NCHANNELS);
  g_dma_inprogress++;
  dmach->inprogress = true;
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: lpc54_dma_done
 *
 * Description:
 *   A DMA has completed. Decrement the g_dma_inprogress counter.
 *
 *   This function is called only from lpc54_dmastop which, in turn, will be
 *   called either by the user directly, by the user indirectly via
 *   lpc54_dmafree(), or from lpc54_dma_interrupt when the transfer completes.
 *
 *   NOTE: In the first two cases, we must be able to handle the case where
 *   there is no DMA in progress and gracefully ignore the call.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void lpc54_dma_done(struct lpc54_dmach_s *dmach)
{
  irqstate_t flags;

  /* Increment the DMA in progress counter */

  flags = enter_critical_section();
  if (dmach->inprogress)
    {
      DEBUGASSERT(g_dma_inprogress > 0);
      dmach->inprogress = false;
      g_dma_inprogress--;
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: lpc54_dma_dispatch
 *
 * Description:
 *   Dispatch a DMA interrupt.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void lpc54_dma_dispatch(int ch, int result)
{
  struct lpc54_dmach_s *dmach;

  /* Yes.. Is this channel assigned? Is there a callback function? */

  dmach = &g_dma.dmach[ch];
  if (dmach->inuse && dmach->callback != NULL)
    {
      /* Perform the callback */

      dmach->callback((DMA_HANDLE)dmach, dmach->arg, result);
    }

  /* Disable this channel, mask any further interrupts for this channel, and
   * clear any pending interrupts.
   */

  lpc54_dmastop((DMA_HANDLE)dmach);
}

/****************************************************************************
 * Name: lpc54_dma_interrupt
 *
 * Description:
 *   The common DMA interrupt handler.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static int lpc54_dma_interrupt(int irq, FAR void *context, FAR void *arg)
{
  uint32_t pending;
  uint32_t bitmask;
  int ch;

  /* Check for pending DMA channel error interrupts */

  pending = getreg32(LPC54_DMA_ERRINT0);
  putreg32(pending, LPC54_DMA_ERRINT0);

  for (ch = 0; pending != 0 && ch < LPC54_DMA_NCHANNELS; ch++)
    {
      /* Check if there is a pending error on this channel */

      bitmask = DMA_CHANNEL((uint32_t)ch);
      if ((pending & bitmask) != 0)
        {
          /* Dispatch the DMA channel error event */

          lpc54_dma_dispatch(ch, -EIO);
          pending &= ~bitmask;
        }
    }

  /* Check for pending DMA interrupt A events */

  pending = getreg32(LPC54_DMA_INTA0);
  putreg32(pending, LPC54_DMA_INTA0);

  for (ch = 0; pending != 0 && ch < LPC54_DMA_NCHANNELS; ch++)
    {
      /* Check if there is a pending interrupt A on this channel */

      bitmask = DMA_CHANNEL((uint32_t)ch);
      if ((pending & bitmask) != 0)
        {
          /* Dispatch DMA channel interrupt A event */

          lpc54_dma_dispatch(ch, OK);
          pending &= ~bitmask;
        }
    }

#if 0 /* interrupt B is not used */
  /* Check for pending DMA interrupt B events */

  pending = getreg32(LPC54_DMA_INTB0);
  putreg32(pending, LPC54_DMA_INTB0);

  for (ch = 0; pending != 0 && ch < LPC54_DMA_NCHANNELS; ch++)
    {
      /* Check if there is a pending interrupt A on this channel */

      bitmask = DMA_CHANNEL((uint32_t)ch);
      if ((pending & bitmask) != 0)
        {
          /* Dispatch DMA channel interrupt B event */

          lpc54_dma_dispatch(ch, OK);
          pending &= ~bitmask;
        }
    }
#endif

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_dmainitialize
 *
 * Description:
 *   Initialize the DMA subsystem.
 *
 * Returned Value:
 *   Zero on success; A negated errno value on failure.
 *
 ****************************************************************************/

void weak_function up_dmainitialize(void)
{
  uint32_t regval;
  int ret;
  int ch;

  /* Enable clocking to the DMA block */

  lpc54_dma_enableclk();

  /* Reset the DMA peripheral */

  lpc54_reset_dma();

  /* Reset all channel configurations */

  for (ch = 0; ch < LPC54_DMA_NCHANNELS; ch++)
    {
#warning Missing logic
    }

  /* Disable and clear all DMA interrupts */

  putreg32(DMA_ALL_CHANNELS, LPC54_DMA_INTENCLR0);
  putreg32(DMA_ALL_CHANNELS, LPC54_DMA_ERRINT0);
  putreg32(DMA_ALL_CHANNELS, LPC54_DMA_INTA0);
  putreg32(DMA_ALL_CHANNELS, LPC54_DMA_INTB0);

  /* Initialize the DMA state structure */

  nxsem_init(&g_dma.exclsem, 0, 1);

  for (ch = 0; ch < LPC54_DMA_NCHANNELS; ch++)
    {
      g_dma.dmach[ch].chn   = ch;     /* Channel number */
      g_dma.dmach[ch].inuse = false;  /* Channel is not in-use */
    }

  /* Attach and enable the DMA interrupt handler */

  ret = irq_attach(LPC54_IRQ_DMA, lpc54_dma_interrupt, NULL);
  if (ret == OK)
    {
      up_enable_irq(LPC54_IRQ_DMA);
    }

  /* Enable the DMA controller */

  putreg32(DMA_CTRL_ENABLE, LPC54_DMA_CTRL);
}

/****************************************************************************
 * Name: lpc54_dmachannel
 *
 * Description:
 *   Allocate a DMA channel.  This function sets aside a DMA channel and
 *   gives the caller exclusive access to the DMA channel.
 *
 * Returned Value:
 *   One success, this function returns a non-NULL, void* DMA channel
 *   handle.  NULL is returned on any failure.  This function can fail only
 *   if no DMA channel is available.
 *
 ****************************************************************************/

DMA_HANDLE lpc54_dmachannel(void)
{
  struct lpc54_dmach_s *dmach = NULL;
  int ret;
  int ch;

  /* Get exclusive access to the DMA state structure */

  do
    {
      ret = nxsem_wait(&g_dma.exclsem);
      DEBUGASSERT(ret == OK || ret == -EINTR);
    }
  while (ret < 0);

  /* Find an available DMA channel */

  for (ch = 0; ch < LPC54_DMA_NCHANNELS; ch++)
    {
      if (!g_dma.dmach[ch].inuse)
        {
          /* Found one! */

          dmach = &g_dma.dmach[ch];
          g_dma.dmach[ch].inuse = true;
          break;
        }
    }

  /* Return what we found (or not) */

  nxsem_post(&g_dma.exclsem);
  return (DMA_HANDLE)dmach;
}

/****************************************************************************
 * Name: lpc54_dmafree
 *
 * Description:
 *   Release a DMA channel.  NOTE:  The 'handle' used in this argument must
 *   NEVER be used again until lpc54_dmachannel() is called again to re-gain
 *   a valid handle.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void lpc54_dmafree(DMA_HANDLE handle)
{
  struct lpc54_dmach_s *dmach = (DMA_HANDLE)handle;

  DEBUGASSERT(dmach && dmach->inuse);

  /* Make sure that the DMA channel was properly stopped */

  lpc54_dmastop(handle);

  /* Mark the channel available.  This is an atomic operation and needs no
   * special protection.
   */

  dmach->inuse = false;
}

/****************************************************************************
 * Name: lpc54_dmasetup
 *
 * Description:
 *   Configure DMA for one transfer.
 *
 ****************************************************************************/

int lpc54_dmarxsetup(DMA_HANDLE handle, uint32_t control, uint32_t config,
                     uint32_t srcaddr, uint32_t destaddr, size_t nbytes)
{
  struct lpc54_dmach_s *dmach = (DMA_HANDLE)handle;
  uint32_t bitmask;
  uint32_t regval;
  uintptr_t base;

  DEBUGASSERT(dmach && dmach->inuse && nbytes < 4096);

  bitmask = DMA_CHANNEL((uint32_t)dmach->chn);
  base    = LPC54_DMA_CHAN_BASE((uint32_t)dmach->chn);

  /* Put the channel in a known state. */
#warning Missing logic

  /* Program the DMA channel */
#warning Missing logic

  return OK;
}

/****************************************************************************
 * Name: lpc54_dmastart
 *
 * Description:
 *   Start the DMA transfer
 *
 ****************************************************************************/

int lpc54_dmastart(DMA_HANDLE handle, dma_callback_t callback, void *arg)
{
  struct lpc54_dmach_s *dmach = (DMA_HANDLE)handle;
  uint32_t regval;
  uint32_t bitmask;
  uintptr_t base;

  DEBUGASSERT(dmach && dmach->inuse && callback);

  /* Save the callback information */

  dmach->callback = callback;
  dmach->arg      = arg;

  /* Increment the count of DMAs in-progress.  This count will be
   * decremented when lpc54_dmastop() is called, either by the user,
   * indirectly via lpc54_dmafree(), or from lpc54_dma_interrupt when the
   * transfer completes.
   */

  lpc54_dma_inprogress(dmach);

  /* Clear any pending DMA interrupts */

  bitmask = DMA_CHANNEL((uint32_t)dmach->chn);
  putreg32(bitmask, LPC54_DMA_ERRINT0);
  putreg32(bitmask, LPC54_DMA_INTA0);
  putreg32(bitmask, LPC54_DMA_INTB0);

  /* Enable terminal count interrupt. */
#warning Missing logic

  /* Enable the channel and unmask terminal count and error interrupts. */
#warning Missing logic

  return OK;
}

/****************************************************************************
 * Name: lpc54_dmastop
 *
 * Description:
 *   Cancel the DMA.  After lpc54_dmastop() is called, the DMA channel is
 *   reset and lpc54_dmasetup() must be called before lpc54_dmastart() can be
 *   called again
 *
 *   This function will be called either by the user directly, by the user
 *   indirectly via lpc54_dmafree(), or from lpc54_dma_interrupt when the
 *   transfer completes.
 *
 ****************************************************************************/

void lpc54_dmastop(DMA_HANDLE handle)
{
  struct lpc54_dmach_s *dmach = (DMA_HANDLE)handle;
  uintptr_t regaddr;
  uint32_t regval;
  uint32_t bitmask;

  DEBUGASSERT(dmach && dmach->inuse);

  /* Disable this channel and mask any further interrupts from the channel.
   * this channel.
   */
#warning Missing logic

  /* Clear any pending interrupts for this channel */

  bitmask = DMA_CHANNEL((uint32_t)dmach->chn);
  putreg32(bitmask, LPC54_DMA_ERRINT0);
  putreg32(bitmask, LPC54_DMA_INTA0);
  putreg32(bitmask, LPC54_DMA_INTB0);

  /* Decrement the count of DMAs in progress */

  lpc54_dma_done(dmach);
}

/****************************************************************************
 * Name: lpc54_dmasample
 *
 * Description:
 *   Sample DMA register contents
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_DMA
void lpc54_dmasample(DMA_HANDLE handle, struct lpc54_dmaregs_s *regs)
{
  struct lpc54_dmach_s *dmach = (DMA_HANDLE)handle;
  uintptr_t base;

  DEBUGASSERT(dmach);

  /* Sample the global DMA registers */

  regs->gbl.ctrl        = getreg32(LPC54_DMA_CTRL);
  regs->gbl.intstat     = getreg32(LPC54_DMA_INTSTAT);
  regs->gbl.srambase    = getreg32(LPC54_DMA_SRAMBASE);
  regs->gbl.enableset0  = getreg32(LPC54_DMA_ENABLESET0);
  regs->gbl.active0     = getreg32(LPC54_DMA_ACTIVE0);
  regs->gbl.busy0       = getreg32(LPC54_DMA_BUSY0);
  regs->gbl.errint0     = getreg32(LPC54_DMA_ERRINT0);
  regs->gbl.intenset0   = getreg32(LPC54_DMA_INTENSET0);
  regs->gbl.inta0       = getreg32(LPC54_DMA_INTA0);
  regs->gbl.intb0       = getreg32(LPC54_DMA_INTB0);

  /* Sample the DMA channel registers */

  base                  = LPC54_DMA_CHAN_BASE((uint32_t)dmach->chn);
  regs->ch.cfg          = getreg32(base + LPC54_DMA_CFG_OFFSET);
  regs->ch.ctlstat      = getreg32(base + LPC54_DMA_CTLSTAT_OFFSET);
  regs->ch.xfercfg      = getreg32(base + LPC54_DMA_XFERCFG_OFFSET);
}
#endif /* CONFIG_DEBUG_DMA */

/****************************************************************************
 * Name: lpc54_dmadump
 *
 * Description:
 *   Dump previously sampled DMA register contents
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_DMA
void lpc54_dmadump(DMA_HANDLE handle, const struct lpc54_dmaregs_s *regs, const char *msg)
{
  struct lpc54_dmach_s *dmach = (DMA_HANDLE)handle;
  uintptr_t base;

  DEBUGASSERT(dmach);

  /* Dump the sampled global DMA registers */

  dmainfo("Global DMA Registers: %s\n", msg);
  dmainfo("        CTRL[%08x]: %08lx\n",
          LPC54_DMA_CTRL, (unsigned long)regs->gbl.ctrl);
  dmainfo("     INTSTAT[%08x]: %08lx\n",
          LPC54_DMA_INTSTAT, (unsigned long)regs->gbl.intstat);
  dmainfo("    SRAMBASE[%08x]: %08lx\n",
          LPC54_DMA_SRAMBASE, (unsigned long)regs->gbl.srambase);
  dmainfo("  ENABLESET0[%08x]: %08lx\n",
          LPC54_DMA_ENABLESET0, (unsigned long)regs->gbl.enableset0);
  dmainfo("     ACTIVE0[%08x]: %08lx\n",
          LPC54_DMA_ACTIVE0, (unsigned long)regs->gbl.active0);
  dmainfo("       BUSY0[%08x]: %08lx\n",
          LPC54_DMA_BUSY0, (unsigned long)regs->gbl.busy0);
  dmainfo("     ERRINT0[%08x]: %08lx\n",
          LPC54_DMA_ERRINT0, (unsigned long)regs->gbl.errint0);
  dmainfo("   INTENSET0[%08x]: %08lx\n",
          LPC54_DMA_INTENSET0, (unsigned long)regs->gbl.intenset0);
  dmainfo("       INTA0[%08x]: %08lx\n",
          LPC54_DMA_INTA0, (unsigned long)regs->gbl.inta0);
  dmainfo("       INTB0[%08x]: %08lx\n",
          LPC54_DMA_INTB0, (unsigned long)regs->gbl.intb0);

  /* Dump the DMA channel registers */

  base = LPC54_DMA_CHAN_BASE((uint32_t)dmach->chn);

  dmainfo("Channel DMA Registers: %d\n", dmach->chn);

  dmainfo("         CFG[%08x]: %08lx\n",
          base + LPC54_DMA_CFG_OFFSET, (unsigned long)regs->ch.cfg);
  dmainfo("     CTLSTAT[%08x]: %08lx\n",
          base + LPC54_DMA_CTLSTAT_OFFSET, (unsigned long)regs->ch.ctlstat);
  dmainfo("     XFERCFG[%08x]: %08lx\n",
          base + LPC54_DMA_XFERCFG_OFFSET, (unsigned long)regs->ch.xfercfg);
}
#endif /* CONFIG_DEBUG_DMA */

#endif /* CONFIG_LPC54_DMA */
