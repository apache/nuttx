/****************************************************************************
 * arch/arm/src/lpc43xx/lpc43_gpdma.c
 *
 *   Copyright (C) 2016-2017 Gregory Nutt. All rights reserved.
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

#include "chip.h"

#include "lpc43_ccu.h"
#include "lpc43_creg.h"
#include "lpc43_gpdma.h"

#ifdef CONFIG_LPC43_GPDMA

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure represents the state of one DMA channel */

struct lpc43_dmach_s
{
  uint8_t chn;             /* The DMA channel number */
  bool inuse;              /* True: The channel is in use */
  bool inprogress;         /* True: DMA is in progress on this channel */
  uint16_t nxfrs;          /* Number of bytes to transfers */
  dma_callback_t callback; /* DMA completion callback function */
  void *arg;               /* Argument to pass to the callback function */
};

/* This structure represents the state of the LPC43 DMA block */

struct lpc43_gpdma_s
{
  sem_t exclsem;           /* For exclusive access to the DMA channel list */

  /* This is the state of each DMA channel */

  struct lpc43_dmach_s dmach[LPC43_NDMACH];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The state of the LPC43 DMA block */

static struct lpc43_gpdma_s g_gpdma;

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* If the following value is zero, then there is no DMA in progress. This
 * value is needed in the IDLE loop to determine if the IDLE loop should
 * go into lower power power consumption modes.  According to the LPC43xx
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
 * Name: lpc43_dmainprogress
 *
 * Description:
 *   Another DMA has started. Increment the g_dma_inprogress counter.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void lpc43_dmainprogress(struct lpc43_dmach_s *dmach)
{
  irqstate_t flags;

  /* Increment the DMA in progress counter */

  flags = enter_critical_section();
  DEBUGASSERT(!dmach->inprogress && g_dma_inprogress < LPC43_NDMACH);
  g_dma_inprogress++;
  dmach->inprogress = true;
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: lpc43_dmadone
 *
 * Description:
 *   A DMA has completed. Decrement the g_dma_inprogress counter.
 *
 *   This function is called only from lpc43_dmastop which, in turn, will be
 *   called either by the user directly, by the user indirectly via
 *   lpc43_dmafree(), or from gpdma_interrupt when the transfer completes.
 *
 *   NOTE: In the first two cases, we must be able to handle the case where
 *   there is no DMA in progress and gracefully ignore the call.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void lpc43_dmadone(struct lpc43_dmach_s *dmach)
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
 * Name: gpdma_interrupt
 *
 * Description:
 *   The common GPDMA interrupt handler.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static int gpdma_interrupt(int irq, FAR void *context, FAR void *arg)
{
  struct lpc43_dmach_s *dmach;
  uint32_t regval;
  uint32_t chbit;
  int result;
  int i;

  /* Check each DMA channel */

  for (i = 0; i < LPC43_NDMACH; i++)
    {
      chbit = GPDMA_CHANNEL((uint32_t)i);

      /* Is there an interrupt pending for this channel?  If the bit for
       * this channel is set, that indicates that a specific DMA channel
       * interrupt request is active. The request can be generated from
       * either the error or terminal count interrupt requests.
       */

      regval = getreg32(LPC43_GPDMA_INTSTAT);
      if ((regval & chbit) != 0)
        {
          /* Yes.. Is this channel assigned? Is there a callback function? */

          dmach = &g_gpdma.dmach[i];
          if (dmach->inuse && dmach->callback)
            {
              /* Yes.. did an error occur? */

              regval = getreg32(LPC43_GPDMA_INTERRSTAT);
              if ((regval & chbit) != 0)
                {
                  /* Yes.. report error status */

                  result = -EIO;
                }

              /* Then this must be a terminal transfer event */

              else
                {
                  /* Let's make sure it is the terminal transfer event. */

                  regval = getreg32(LPC43_GPDMA_INTTCSTAT);
                  if ((regval & chbit) != 0)
                    {
                      result = OK;
                    }

                  /* This should not happen */

                  else
                    {
                      result = -EINVAL;
                    }
                }

              /* Perform the callback */

              dmach->callback((DMA_HANDLE)dmach, dmach->arg, result);
            }

          /* Disable this channel, mask any further interrupts for
           * this channel, and clear any pending interrupts.
           */

          lpc43_dmastop((DMA_HANDLE)dmach);
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
 *   Initialize the GPDMA subsystem.  Called from up_initialize() early in the
 *   boot-up sequence.  Prototyped in up_internal.h.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void weak_function up_dmainitialize(void)
{
  uint32_t regval;
  int ret;
  int i;

  /* Enable clocking to the GPDMA block */

  regval  = getreg32(LPC43_CCU1_M4_DMA_CFG);
  regval |= CCU_CLK_CFG_RUN;
  putreg32(regval, LPC43_CCU1_M4_DMA_CFG);

  /* Reset all channel configurations */

  for (i = 0; i < LPC43_NDMACH; i++)
    {
      putreg32(0, LPC43_GPDMA_CONFIG_(i));
    }

  /* Clear all DMA interrupts */

  putreg32(DMACH_ALL, LPC43_GPDMA_INTTCCLEAR);
  putreg32(DMACH_ALL, LPC43_GPDMA_INTERRCLR);

  /* Initialize the DMA state structure */

  nxsem_init(&g_gpdma.exclsem, 0, 1);

  for (i = 0; i < LPC43_NDMACH; i++)
    {
      g_gpdma.dmach[i].chn   = i;      /* Channel number */
      g_gpdma.dmach[i].inuse = false;  /* Channel is not in-use */
    }

  /* Attach and enable the common interrupt handler */

  ret = irq_attach(LPC43M4_IRQ_DMA, gpdma_interrupt, NULL);
  if (ret == OK)
    {
      up_enable_irq(LPC43M4_IRQ_DMA);
    }

  /* Enable the DMA controller (for little endian operation) */

  putreg32(GPDMA_CONFIG_ENA, LPC43_GPDMA_GLOBAL_CONFIG);
}

/****************************************************************************
 * Name: lpc43_dmaconfigure
 *
 * Description:
 *   Configure a DMA request.  Each DMA request may have four different DMA
 *   request sources.  This associates one of the sources with a DMA request.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void lpc43_dmaconfigure(uint8_t dmarequest, uint8_t dmasrc)
{
  uint32_t regval;

  DEBUGASSERT(dmarequest < LPC43_NDMAREQ);

  /* Set or clear the DMASEL bit corresponding to the request number */

  regval = getreg32(LPC43_CREG_DMAMUX);

  switch (dmasrc)
    {
      case 0:
        regval &= ~(3 << dmarequest);
        break;

      case 1:
        regval &= ~(3 << dmarequest);
        regval |= (1 << dmarequest);
        break;

      case 2:
        regval &= ~(3 << dmarequest);
        regval |= (2 << dmarequest);
        break;

      case 3:
        regval |= (3 << dmarequest);
        break;
    }

  putreg32(regval, LPC43_CREG_DMAMUX);
}

/****************************************************************************
 * Name: lpc43_dmachannel
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

DMA_HANDLE lpc43_dmachannel(void)
{
  struct lpc43_dmach_s *dmach = NULL;
  int ret;
  int i;

  /* Get exclusive access to the GPDMA state structure */

  do
    {
      ret = nxsem_wait(&g_gpdma.exclsem);
      DEBUGASSERT(ret == OK || ret == -EINTR);
    }
  while (ret < 0);

  /* Find an available DMA channel */

  for (i = 0; i < LPC43_NDMACH; i++)
    {
      if (!g_gpdma.dmach[i].inuse)
        {
          /* Found one! */

          dmach = &g_gpdma.dmach[i];
          g_gpdma.dmach[i].inuse = true;
          break;
        }
    }

  /* Return what we found (or not) */

  nxsem_post(&g_gpdma.exclsem);
  return (DMA_HANDLE)dmach;
}

/****************************************************************************
 * Name: lpc43_dmafree
 *
 * Description:
 *   Release a DMA channel.  NOTE:  The 'handle' used in this argument must
 *   NEVER be used again until lpc43_dmachannel() is called again to re-gain
 *   a valid handle.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void lpc43_dmafree(DMA_HANDLE handle)
{
  struct lpc43_dmach_s *dmach = (DMA_HANDLE)handle;

  DEBUGASSERT(dmach && dmach->inuse);

  /* Make sure that the DMA channel was properly stopped */

  lpc43_dmastop(handle);

  /* Mark the channel available.  This is an atomic operation and needs no
   * special protection.
   */

  dmach->inuse = false;
}

/****************************************************************************
 * Name: lpc43_dmasetup
 *
 * Description:
 *   Configure DMA for one transfer.
 *
 ****************************************************************************/

int lpc43_dmasetup(DMA_HANDLE handle, uint32_t control, uint32_t config,
                   uint32_t srcaddr, uint32_t destaddr, size_t nbytes)
{
  struct lpc43_dmach_s *dmach = (DMA_HANDLE)handle;
  uint32_t chbit;
  uint32_t regval;
  uint32_t base;

  DEBUGASSERT(dmach && dmach->inuse && nbytes < 4096);

  chbit = GPDMA_CHANNEL((uint32_t)dmach->chn);
  base  = LPC43_GPDMA_CHANNEL((uint32_t)dmach->chn);

  /* Put the channel in a known state.  Zero disables everything */

  putreg32(0, base + LPC43_GPDMA_CONTROL_CHOFFSET);
  putreg32(0, base + LPC43_GPDMA_CONFIG_CHOFFSET);

  /* "Programming a DMA channel
   *
   * 1. "Choose a free DMA channel with the priority needed. DMA channel 0
   *     has the highest priority and DMA channel 7 the lowest priority.
   */

  regval = getreg32(LPC43_GPDMA_ENBLDCHNS);
  if ((regval & chbit) != 0)
    {
      /* There is an active DMA on this channel! */

      return -EBUSY;
    }

  /* 2. "Clear any pending interrupts on the channel to be used by writing
   *     to the DMACIntTCClear and DMACIntErrClear register. The previous
   *     channel operation might have left interrupt active.
   */

  putreg32(chbit, LPC43_GPDMA_INTTCCLEAR);
  putreg32(chbit, LPC43_GPDMA_INTERRCLR);

  /* 3. "Write the source address into the DMACCxSrcAddr register. */

  putreg32(srcaddr, base + LPC43_GPDMA_SRCADDR_CHOFFSET);

  /* 4. "Write the destination address into the DMACCxDestAddr register. */

  putreg32(destaddr, base + LPC43_GPDMA_DESTADDR_CHOFFSET);

  /* 5. "Write the address of the next LLI into the DMACCxLLI register. If
   *     the transfer comprises of a single packet of data then 0 must be
   *     written into this register.
   */

  putreg32(0, base + LPC43_GPDMA_LLI_CHOFFSET);

  /* 6. "Write the control information into the DMACCxControl register."
   *
   * The caller provides all CONTROL register fields except for the transfer
   * size which is passed as a separate parameter and for the terminal count
   * interrupt enable bit which is controlled by the driver.
   */

  regval  = control & ~(GPDMA_CONTROL_XFRSIZE_MASK | GPDMA_CONTROL_IE);
  regval |= ((uint32_t)nbytes << GPDMA_CONTROL_XFRSIZE_SHIFT);
  putreg32(regval, base + LPC43_GPDMA_CONTROL_CHOFFSET);

  /* Save the number of transfer to perform for lpc43_dmastart */

  dmach->nxfrs = (uint16_t)nbytes;

  /* 7. "Write the channel configuration information into the DMACCxConfig
   *     register. If the enable bit is set then the DMA channel is
   *     automatically enabled."
   *
   * Only the SRCPER, DSTPER, and FCNTRL fields of the CONFIG register
   * are provided by the caller.  Little endian is assumed.
   */

  regval = config & (GPDMA_CONFIG_SRCPER_MASK |
                     GPDMA_CONFIG_DESTPER_MASK |
                     GPDMA_CONFIG_FCNTRL_MASK);
  putreg32(regval, base + LPC43_GPDMA_CONFIG_CHOFFSET);

  return OK;
}

/****************************************************************************
 * Name: lpc43_dmastart
 *
 * Description:
 *   Start the DMA transfer
 *
 ****************************************************************************/

int lpc43_dmastart(DMA_HANDLE handle, dma_callback_t callback, void *arg)
{
  struct lpc43_dmach_s *dmach = (DMA_HANDLE)handle;
  uint32_t regval;
  uint32_t chbit;
  uint32_t base;

  DEBUGASSERT(dmach && dmach->inuse && callback);

  /* Save the callback information */

  dmach->callback = callback;
  dmach->arg      = arg;

  /* Increment the count of DMAs in-progress.  This count will be
   * decremented when lpc43_dmastop() is called, either by the user,
   * indirectly via lpc43_dmafree(), or from gpdma_interrupt when the
   * transfer completes.
   */

  lpc43_dmainprogress(dmach);

  /* Clear any pending DMA interrupts */

  chbit = GPDMA_CHANNEL((uint32_t)dmach->chn);
  putreg32(chbit, LPC43_GPDMA_INTTCCLEAR);
  putreg32(chbit, LPC43_GPDMA_INTERRCLR);

  /* Enable terminal count interrupt.  Note that we need to restore the
   * number transfers.  That is because the value has a different meaning
   * when it is read.
   */

  base    = LPC43_GPDMA_CHANNEL((uint32_t)dmach->chn);
  regval  = getreg32(base + LPC43_GPDMA_CONTROL_CHOFFSET);
  regval &= ~GPDMA_CONTROL_XFRSIZE_MASK;
  regval |= (GPDMA_CONTROL_IE | ((uint32_t)dmach->nxfrs << GPDMA_CONTROL_XFRSIZE_SHIFT));
  putreg32(regval, base + LPC43_GPDMA_CONTROL_CHOFFSET);

  /* Enable the channel and unmask terminal count and error interrupts.
   * According to the user manual, zero masks and one unmasks (hence,
   * these are really enables).
   */

  regval  = getreg32(base + LPC43_GPDMA_CONFIG_CHOFFSET);
  regval |= (GPDMA_CONFIG_ENA | GPDMA_CONFIG_IE | GPDMA_CONFIG_ITC);
  putreg32(regval, base + LPC43_GPDMA_CONFIG_CHOFFSET);

  return OK;
}

/****************************************************************************
 * Name: lpc43_dmastop
 *
 * Description:
 *   Cancel the DMA.  After lpc43_dmastop() is called, the DMA channel is
 *   reset and lpc43_dmasetup() must be called before lpc43_dmastart() can be
 *   called again
 *
 *   This function will be called either by the user directly, by the user
 *   indirectly via lpc43_dmafree(), or from gpdma_interrupt when the
 *   transfer completes.
 *
 ****************************************************************************/

void lpc43_dmastop(DMA_HANDLE handle)
{
  struct lpc43_dmach_s *dmach = (DMA_HANDLE)handle;
  uint32_t regaddr;
  uint32_t regval;
  uint32_t chbit;

  DEBUGASSERT(dmach && dmach->inuse);

  /* Disable this channel and mask any further interrupts from the channel.
   * this channel.  The channel is disabled by clearing the channel
   * enable bit. Any outstanding data in the FIFOs is lost.
   */

  regaddr = LPC43_GPDMA_CONFIG_((uint32_t)dmach->chn);
  regval  = getreg32(regaddr);
  regval &= ~(GPDMA_CONFIG_ENA | GPDMA_CONFIG_IE | GPDMA_CONFIG_ITC);
  putreg32(regval, regaddr);

  /* Clear any pending interrupts for this channel */

  chbit = GPDMA_CHANNEL((uint32_t)dmach->chn);
  putreg32(chbit, LPC43_GPDMA_INTTCCLEAR);
  putreg32(chbit, LPC43_GPDMA_INTERRCLR);

  /* Decrement the count of DMAs in progress */

  lpc43_dmadone(dmach);
}

/****************************************************************************
 * Name: lpc43_dmasample
 *
 * Description:
 *   Sample DMA register contents
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_DMA
void lpc43_dmasample(DMA_HANDLE handle, struct lpc43_dmaregs_s *regs)
{
  struct lpc43_dmach_s *dmach = (DMA_HANDLE)handle;
  uint32_t base;

  DEBUGASSERT(dmach);

  /* Sample the global DMA registers */

  regs->gbl.intst         = getreg32(LPC43_GPDMA_INTSTAT);
  regs->gbl.inttcst       = getreg32(LPC43_GPDMA_INTTCSTAT);
  regs->gbl.interrst      = getreg32(LPC43_GPDMA_INTERRSTAT);
  regs->gbl.rawinttcst    = getreg32(LPC43_GPDMA_RAWINTTCSTAT);
  regs->gbl.rawinterrst   = getreg32(LPC43_GPDMA_RAWINTERRSTAT);
  regs->gbl.enbldchns     = getreg32(LPC43_GPDMA_ENBLDCHNS);
  regs->gbl.softbreq      = getreg32(LPC43_GPDMA_SOFTBREQ);
  regs->gbl.softsreq      = getreg32(LPC43_GPDMA_SOFTSREQ);
  regs->gbl.softlbreq     = getreg32(LPC43_GPDMA_SOFTLBREQ);
  regs->gbl.softlsreq     = getreg32(LPC43_GPDMA_SOFTLSREQ);
  regs->gbl.config        = getreg32(LPC43_GPDMA_GLOBAL_CONFIG);
  regs->gbl.sync          = getreg32(LPC43_GPDMA_SYNC);

  /* Sample the DMA channel registers */

  base                  = LPC43_GPDMA_CHANNEL((uint32_t)dmach->chn);
  regs->ch.srcaddr      = getreg32(base + LPC43_GPDMA_SRCADDR_CHOFFSET);
  regs->ch.destaddr     = getreg32(base + LPC43_GPDMA_DESTADDR_CHOFFSET);
  regs->ch.lli          = getreg32(base + LPC43_GPDMA_LLI_CHOFFSET);
  regs->ch.control      = getreg32(base + LPC43_GPDMA_CONTROL_CHOFFSET);
  regs->ch.config       = getreg32(base + LPC43_GPDMA_CONFIG_CHOFFSET);
}
#endif /* CONFIG_DEBUG_DMA */

/****************************************************************************
 * Name: lpc43_dmadump
 *
 * Description:
 *   Dump previously sampled DMA register contents
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_DMA
void lpc43_dmadump(DMA_HANDLE handle, const struct lpc43_dmaregs_s *regs, const char *msg)
{
  struct lpc43_dmach_s *dmach = (DMA_HANDLE)handle;
  uint32_t base;

  DEBUGASSERT(dmach);

  /* Dump the sampled global DMA registers */

  dmainfo("Global GPDMA Registers: %s\n", msg);
  dmainfo("       INTST[%08x]: %08x\n",
          LPC43_GPDMA_INTSTAT, regs->gbl.intst);
  dmainfo("     INTTCSTAT[%08x]: %08x\n",
          LPC43_GPDMA_INTTCSTAT, regs->gbl.inttcst);
  dmainfo("    INTERRSTAT[%08x]: %08x\n",
          LPC43_GPDMA_INTERRSTAT, regs->gbl.interrst);
  dmainfo("  RAWINTTCSTAT[%08x]: %08x\n",
          LPC43_GPDMA_RAWINTTCSTAT, regs->gbl.rawinttcst);
  dmainfo(" RAWINTERRSTAT[%08x]: %08x\n",
          LPC43_GPDMA_RAWINTERRSTAT, regs->gbl.rawinterrst);
  dmainfo("   ENBLDCHNS[%08x]: %08x\n",
          LPC43_GPDMA_ENBLDCHNS, regs->gbl.enbldchns);
  dmainfo("    SOFTBREQ[%08x]: %08x\n",
          LPC43_GPDMA_SOFTBREQ, regs->gbl.softbreq);
  dmainfo("    SOFTSREQ[%08x]: %08x\n",
          LPC43_GPDMA_SOFTSREQ, regs->gbl.softsreq);
  dmainfo("   SOFTLBREQ[%08x]: %08x\n",
          LPC43_GPDMA_SOFTLBREQ, regs->gbl.softlbreq);
  dmainfo("   SOFTLSREQ[%08x]: %08x\n",
          LPC43_GPDMA_SOFTLSREQ, regs->gbl.softlsreq);
  dmainfo("      CONFIG[%08x]: %08x\n",
          LPC43_GPDMA_GLOBAL_CONFIG, regs->gbl.config);
  dmainfo("        SYNC[%08x]: %08x\n",
          LPC43_GPDMA_SYNC, regs->gbl.sync);

  /* Dump the DMA channel registers */

  base = LPC43_GPDMA_CHANNEL((uint32_t)dmach->chn);

  dmainfo("Channel GPDMA Registers: %d\n", dmach->chn);

  dmainfo("     SRCADDR[%08x]: %08x\n",
          base + LPC43_GPDMA_SRCADDR_CHOFFSET, regs->ch.srcaddr);
  dmainfo("    DESTADDR[%08x]: %08x\n",
          base + LPC43_GPDMA_DESTADDR_CHOFFSET, regs->ch.destaddr);
  dmainfo("         LLI[%08x]: %08x\n",
          base + LPC43_GPDMA_LLI_CHOFFSET, regs->ch.lli);
  dmainfo("     CONTROL[%08x]: %08x\n",
          base + LPC43_GPDMA_CONTROL_CHOFFSET, regs->ch.control);
  dmainfo("      CONFIG[%08x]: %08x\n",
          base + LPC43_GPDMA_CONFIG_CHOFFSET, regs->ch.config);
}
#endif /* CONFIG_DEBUG_DMA */

#endif /* CONFIG_LPC43_GPDMA */
