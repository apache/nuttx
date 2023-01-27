/****************************************************************************
 * arch/arm/src/lpc17xx_40xx/lpc17_40_gpdma.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/mutex.h>

#include "arm_internal.h"
#include "chip.h"

#include "hardware/lpc17_40_syscon.h"
#include "lpc17_40_gpdma.h"

#ifdef CONFIG_LPC17_40_GPDMA

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure represents the state of one DMA channel */

struct lpc17_40_dmach_s
{
  uint8_t chn;             /* The DMA channel number */
  bool inuse;              /* True: The channel is in use */
  bool inprogress;         /* True: DMA is in progress on this channel */
  uint16_t nxfrs;          /* Number of transfers */
  dma_callback_t callback; /* DMA completion callback function */
  void *arg;               /* Argument to pass to the callback function */
};

/* This structure represents the state of the LPC17 DMA block */

struct lpc17_40_gpdma_s
{
  mutex_t lock;            /* For exclusive access to the DMA channel list */

  /* This is the state of each DMA channel */

  struct lpc17_40_dmach_s dmach[LPC17_40_NDMACH];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The state of the LPC17 DMA block */

static struct lpc17_40_gpdma_s g_gpdma =
{
  .lock = NXMUTEX_INITIALIZER,
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* If the following value is zero, then there is no DMA in progress. This
 * value is needed in the IDLE loop to determine if the IDLE loop should
 * go into lower power power consumption modes.  According to the LPC17xx
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
 * Name: lpc17_40_dmainprogress
 *
 * Description:
 *   Another DMA has started. Increment the g_dma_inprogress counter.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void lpc17_40_dmainprogress(struct lpc17_40_dmach_s *dmach)
{
  irqstate_t flags;

  /* Increment the DMA in progress counter */

  flags = enter_critical_section();
  DEBUGASSERT(!dmach->inprogress && g_dma_inprogress < LPC17_40_NDMACH);
  g_dma_inprogress++;
  dmach->inprogress = true;
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: lpc17_40_dmadone
 *
 * Description:
 *   A DMA has completed. Decrement the g_dma_inprogress counter.
 *
 *   This function is called only from lpc17_40_dmastop which, in turn, will
 *   be called either by the user directly, by the user indirectly via
 *   lpc17_40_dmafree(), or from gpdma_interrupt when the transfer completes.
 *
 *   NOTE: In the first two cases, we must be able to handle the case where
 *   there is no DMA in progress and gracefully ignore the call.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void lpc17_40_dmadone(struct lpc17_40_dmach_s *dmach)
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

static int gpdma_interrupt(int irq, void *context, void *arg)
{
  struct lpc17_40_dmach_s *dmach;
  uint32_t regval;
  uint32_t chbit;
  int result;
  int i;

  /* Check each DMA channel */

  for (i = 0; i < LPC17_40_NDMACH; i++)
    {
      chbit = DMACH((uint32_t)i);

      /* Is there an interrupt pending for this channel?  If the bit for
       * this channel is set, that indicates that a specific DMA channel
       * interrupt request is active. The request can be generated from
       * either the error or terminal count interrupt requests.
       */

      regval = getreg32(LPC17_40_DMA_INTST);
      if ((regval & chbit) != 0)
        {
          /* Yes.. Is this channel assigned? Is there a callback function? */

          dmach = &g_gpdma.dmach[i];
          if (dmach->inuse && dmach->callback)
            {
              /* Yes.. did an error occur? */

              regval = getreg32(LPC17_40_DMA_INTERRST);
              if ((regval & chbit) != 0)
                {
                  /* Yes.. report error status */

                  result = -EIO;
                }

              /* Then this must be a terminal transfer event */

              else
                {
                  /* Let's make sure it is the terminal transfer event. */

                  regval = getreg32(LPC17_40_DMA_INTTCST);
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

          lpc17_40_dmastop((DMA_HANDLE)dmach);
        }
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_dma_initialize
 *
 * Description:
 *   Initialize the GPDMA subsystem.  Called from up_initialize() early in
 *   the boot-up sequence.  Prototyped in arm_internal.h.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void weak_function arm_dma_initialize(void)
{
  uint32_t regval;
  int ret;
  int i;

  /* Enable clocking to the GPDMA block */

  regval  = getreg32(LPC17_40_SYSCON_PCONP);
  regval |= SYSCON_PCONP_PCGPDMA;
  putreg32(regval, LPC17_40_SYSCON_PCONP);

  /* Reset all channel configurations */

  for (i = 0; i < LPC17_40_NDMACH; i++)
    {
      putreg32(0, LPC17_40_DMACH_CONFIG(i));
    }

  /* Clear all DMA interrupts */

  putreg32(DMACH_ALL, LPC17_40_DMA_INTTCCLR);
  putreg32(DMACH_ALL, LPC17_40_DMA_INTERRCLR);

  /* Initialize the DMA state structure */

  for (i = 0; i < LPC17_40_NDMACH; i++)
    {
      g_gpdma.dmach[i].chn   = i;      /* Channel number */
      g_gpdma.dmach[i].inuse = false;  /* Channel is not in-use */
    }

  /* Attach and enable the common interrupt handler */

  ret = irq_attach(LPC17_40_IRQ_GPDMA, gpdma_interrupt, NULL);
  if (ret == OK)
    {
      up_enable_irq(LPC17_40_IRQ_GPDMA);
    }

  /* Enable the DMA controller (for little endian operation) */

  putreg32(DMA_CONFIG_E, LPC17_40_DMA_CONFIG);
}

/****************************************************************************
 * Name: lpc17_40_dmaconfigure
 *
 * Description:
 *   Configure a DMA request.  Each DMA request may have two different DMA
 *   request sources.  This associates one of the sources with a DMA request.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void lpc17_40_dmaconfigure(uint8_t dmarequest, bool alternate)
{
  uint32_t regval;

  DEBUGASSERT(dmarequest < LPC17_40_NDMAREQ);

#ifdef LPC176x
  /* For the LPC176x family, only request numbers 8-15 have DMASEL bits */

  if (dmarequest < 8)
    {
      return;
    }

  dmarequest -= 8;
#endif

  /* Set or clear the DMASEL bit corresponding to the request number */

  regval = getreg32(LPC17_40_SYSCON_DMAREQSEL);

  if (alternate)
    {
      regval |= (1 << dmarequest);
    }
  else
    {
      regval &= ~(1 << dmarequest);
    }

  putreg32(regval, LPC17_40_SYSCON_DMAREQSEL);
}

/****************************************************************************
 * Name: lpc17_40_dmachannel
 *
 * Description:
 *   Allocate a DMA channel.  This function sets aside a DMA channel and
 *   gives the caller exclusive access to the DMA channel.
 *
 * Returned Value:
 *   On success, this function returns a non-NULL, void* DMA channel handle.
 *   NULL is returned on any failure.  This function can fail only if no DMA
 *   channel is available.
 *
 ****************************************************************************/

DMA_HANDLE lpc17_40_dmachannel(void)
{
  struct lpc17_40_dmach_s *dmach = NULL;
  int i;
  int ret;

  /* Get exclusive access to the GPDMA state structure */

  ret = nxmutex_lock(&g_gpdma.lock);
  if (ret < 0)
    {
      return NULL;
    }

  /* Find an available DMA channel */

  for (i = 0; i < LPC17_40_NDMACH; i++)
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

  nxmutex_unlock(&g_gpdma.lock);
  return (DMA_HANDLE)dmach;
}

/****************************************************************************
 * Name: lpc17_40_dmafree
 *
 * Description:
 *   Release a DMA channel.  NOTE:  The 'handle' used in this argument must
 *   NEVER be used again until lpc17_40_dmachannel() is called again to
 *   re-gain a valid handle.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void lpc17_40_dmafree(DMA_HANDLE handle)
{
  struct lpc17_40_dmach_s *dmach = (DMA_HANDLE)handle;

  DEBUGASSERT(dmach && dmach->inuse);

  /* Make sure that the DMA channel was properly stopped */

  lpc17_40_dmastop(handle);

  /* Mark the channel available.  This is an atomic operation and needs no
   * special protection.
   */

  dmach->inuse = false;
}

/****************************************************************************
 * Name: lpc17_40_dmasetup
 *
 * Description:
 *   Configure DMA for one transfer.
 *
 ****************************************************************************/

int lpc17_40_dmasetup(DMA_HANDLE handle, uint32_t control, uint32_t config,
                   uint32_t srcaddr, uint32_t destaddr, size_t nxfrs)
{
  struct lpc17_40_dmach_s *dmach = (DMA_HANDLE)handle;
  uint32_t chbit;
  uint32_t regval;
  uint32_t base;

  DEBUGASSERT(dmach && dmach->inuse && nxfrs < 4096);

  chbit = DMACH((uint32_t)dmach->chn);
  base  = LPC17_40_DMACH_BASE((uint32_t)dmach->chn);

  /* Put the channel in a known state.  Zero disables everything */

  putreg32(0, base + LPC17_40_DMACH_CONTROL_OFFSET);
  putreg32(0, base + LPC17_40_DMACH_CONFIG_OFFSET);

  /* "Programming a DMA channel
   *
   * 1. "Choose a free DMA channel with the priority needed. DMA channel 0
   *     has the highest priority and DMA channel 7 the lowest priority.
   */

  regval = getreg32(LPC17_40_DMA_ENBLDCHNS);
  if ((regval & chbit) != 0)
    {
      /* There is an active DMA on this channel! */

      return -EBUSY;
    }

  /* 2. "Clear any pending interrupts on the channel to be used by writing
   *     to the DMACIntTCClear and DMACIntErrClear register. The previous
   *     channel operation might have left interrupt active.
   */

  putreg32(chbit, LPC17_40_DMA_INTTCCLR);
  putreg32(chbit, LPC17_40_DMA_INTERRCLR);

  /* 3. "Write the source address into the DMACCxSrcAddr register. */

  putreg32(srcaddr, base + LPC17_40_DMACH_SRCADDR_OFFSET);

  /* 4. "Write the destination address into the DMACCxDestAddr register. */

  putreg32(destaddr, base + LPC17_40_DMACH_DESTADDR_OFFSET);

  /* 5. "Write the address of the next LLI into the DMACCxLLI register. If
   *     the transfer comprises of a single packet of data then 0 must be
   *     written into this register.
   */

  putreg32(0, base + LPC17_40_DMACH_LLI_OFFSET);

  /* 6. "Write the control information into the DMACCxControl register."
   *
   * The caller provides all CONTROL register fields except for the transfer
   * size which is passed as a separate parameter and for the terminal count
   * interrupt enable bit which is controlled by the driver.
   */

  regval  = control & ~(DMACH_CONTROL_XFRSIZE_MASK | DMACH_CONTROL_I);
  regval |= ((uint32_t)nxfrs << DMACH_CONTROL_XFRSIZE_SHIFT);
  putreg32(regval, base + LPC17_40_DMACH_CONTROL_OFFSET);

  /* Save the number of transfer to perform for lpc17_40_dmastart */

  dmach->nxfrs = (uint16_t)nxfrs;

  /* 7. "Write the channel configuration information into the DMACCxConfig
   *     register. If the enable bit is set then the DMA channel is
   *     automatically enabled."
   *
   * Only the SRCPER, DSTPER, and XFRTTYPE fields of the CONFIG register
   * are provided by the caller.  Little endian is assumed.
   */

  regval = config & (DMACH_CONFIG_SRCPER_MASK | DMACH_CONFIG_DSTPER_MASK |
                     DMACH_CONFIG_XFRTYPE_MASK);
  putreg32(regval, base + LPC17_40_DMACH_CONFIG_OFFSET);

  return OK;
}

/****************************************************************************
 * Name: lpc17_40_dmastart
 *
 * Description:
 *   Start the DMA transfer
 *
 ****************************************************************************/

int lpc17_40_dmastart(DMA_HANDLE handle, dma_callback_t callback, void *arg)
{
  struct lpc17_40_dmach_s *dmach = (DMA_HANDLE)handle;
  uint32_t regval;
  uint32_t chbit;
  uint32_t base;

  DEBUGASSERT(dmach && dmach->inuse && callback);

  /* Save the callback information */

  dmach->callback = callback;
  dmach->arg      = arg;

  /* Increment the count of DMAs in-progress.  This count will be
   * decremented when lpc17_40_dmastop() is called, either by the user,
   * indirectly via lpc17_40_dmafree(), or from gpdma_interrupt when the
   * transfer completes.
   */

  lpc17_40_dmainprogress(dmach);

  /* Clear any pending DMA interrupts */

  chbit = DMACH((uint32_t)dmach->chn);
  putreg32(chbit, LPC17_40_DMA_INTTCCLR);
  putreg32(chbit, LPC17_40_DMA_INTERRCLR);

  /* Enable terminal count interrupt.  Note that we need to restore the
   * number transfers.  That is because the value has a different meaning
   * when it is read.
   */

  base    = LPC17_40_DMACH_BASE((uint32_t)dmach->chn);
  regval  = getreg32(base + LPC17_40_DMACH_CONTROL_OFFSET);
  regval &= ~DMACH_CONTROL_XFRSIZE_MASK;
  regval |= (DMACH_CONTROL_I |
             ((uint32_t)dmach->nxfrs << DMACH_CONTROL_XFRSIZE_SHIFT));
  putreg32(regval, base + LPC17_40_DMACH_CONTROL_OFFSET);

  /* Enable the channel and unmask terminal count and error interrupts.
   * According to the user manual, zero masks and one unmasks (hence,
   * these are really enables).
   */

  regval  = getreg32(base + LPC17_40_DMACH_CONFIG_OFFSET);
  regval |= (DMACH_CONFIG_E | DMACH_CONFIG_IE | DMACH_CONFIG_ITC);
  putreg32(regval, base + LPC17_40_DMACH_CONFIG_OFFSET);

  return OK;
}

/****************************************************************************
 * Name: lpc17_40_dmastop
 *
 * Description:
 *   Cancel the DMA.  After lpc17_40_dmastop() is called, the DMA channel is
 *   reset and lpc17_40_dmasetup() must be called before lpc17_40_dmastart()
 *   can be called again
 *
 *   This function will be called either by the user directly, by the user
 *   indirectly via lpc17_40_dmafree(), or from gpdma_interrupt when the
 *   transfer completes.
 *
 ****************************************************************************/

void lpc17_40_dmastop(DMA_HANDLE handle)
{
  struct lpc17_40_dmach_s *dmach = (DMA_HANDLE)handle;
  uint32_t regaddr;
  uint32_t regval;
  uint32_t chbit;

  DEBUGASSERT(dmach && dmach->inuse);

  /* Disable this channel and mask any further interrupts from the channel.
   * this channel.  The channel is disabled by clearning the channel
   * enable bit. Any outstanding data in the FIFO's is lost.
   */

  regaddr = LPC17_40_DMACH_CONFIG((uint32_t)dmach->chn);
  regval  = getreg32(regaddr);
  regval &= ~(DMACH_CONFIG_E | DMACH_CONFIG_IE | DMACH_CONFIG_ITC);
  putreg32(regval, regaddr);

  /* Clear any pending interrupts for this channel */

  chbit = DMACH((uint32_t)dmach->chn);
  putreg32(chbit, LPC17_40_DMA_INTTCCLR);
  putreg32(chbit, LPC17_40_DMA_INTERRCLR);

  /* Decrement the count of DMAs in progress */

  lpc17_40_dmadone(dmach);
}

/****************************************************************************
 * Name: lpc17_40_dmasample
 *
 * Description:
 *   Sample DMA register contents
 *
 ****************************************************************************/

#ifdef CONFIG__DEBUG_DMA_INFO
void lpc17_40_dmasample(DMA_HANDLE handle, struct lpc17_40_dmaregs_s *regs)
{
  struct lpc17_40_dmach_s *dmach = (DMA_HANDLE)handle;
  uint32_t base;

  DEBUGASSERT(dmach);

  /* Sample the global DMA registers */

  regs->gbl.intst       = getreg32(LPC17_40_DMA_INTST);
  regs->gbl.inttcst     = getreg32(LPC17_40_DMA_INTTCST);
  regs->gbl.interrst    = getreg32(LPC17_40_DMA_INTERRST);
  regs->gbl.rawinttcst  = getreg32(LPC17_40_DMA_RAWINTTCST);
  regs->gbl.rawinterrst = getreg32(LPC17_40_DMA_RAWINTERRST);
  regs->gbl.enbldchns   = getreg32(LPC17_40_DMA_ENBLDCHNS);
  regs->gbl.softbreq    = getreg32(LPC17_40_DMA_SOFTBREQ);
  regs->gbl.softsreq    = getreg32(LPC17_40_DMA_SOFTSREQ);
  regs->gbl.softlbreq   = getreg32(LPC17_40_DMA_SOFTLBREQ);
  regs->gbl.softlsreq   = getreg32(LPC17_40_DMA_SOFTLSREQ);
  regs->gbl.config      = getreg32(LPC17_40_DMA_CONFIG);
  regs->gbl.sync        = getreg32(LPC17_40_DMA_SYNC);

  /* Sample the DMA channel registers */

  base                  = LPC17_40_DMACH_BASE((uint32_t)dmach->chn);
  regs->ch.srcaddr      = getreg32(base + LPC17_40_DMACH_SRCADDR_OFFSET);
  regs->ch.destaddr     = getreg32(base + LPC17_40_DMACH_DESTADDR_OFFSET);
  regs->ch.lli          = getreg32(base + LPC17_40_DMACH_LLI_OFFSET);
  regs->ch.control      = getreg32(base + LPC17_40_DMACH_CONTROL_OFFSET);
  regs->ch.config       = getreg32(base + LPC17_40_DMACH_CONFIG_OFFSET);
}
#endif /* CONFIG__DEBUG_DMA_INFO */

/****************************************************************************
 * Name: lpc17_40_dmadump
 *
 * Description:
 *   Dump previously sampled DMA register contents
 *
 ****************************************************************************/

#ifdef CONFIG__DEBUG_DMA_INFO
void lpc17_40_dmadump(DMA_HANDLE handle,
                      const struct lpc17_40_dmaregs_s *regs,
                      const char *msg)
{
  struct lpc17_40_dmach_s *dmach = (DMA_HANDLE)handle;
  uint32_t base;

  DEBUGASSERT(dmach);

  /* Dump the sampled global DMA registers */

  dmainfo("Global GPDMA Registers: %s\n", msg);
  dmainfo("       INTST[%08x]: %08x\n",
          LPC17_40_DMA_INTST, regs->gbl.intst);
  dmainfo("     INTTCST[%08x]: %08x\n",
          LPC17_40_DMA_INTTCST, regs->gbl.inttcst);
  dmainfo("    INTERRST[%08x]: %08x\n",
          LPC17_40_DMA_INTERRST, regs->gbl.interrst);
  dmainfo("  RAWINTTCST[%08x]: %08x\n",
          LPC17_40_DMA_RAWINTTCST, regs->gbl.rawinttcst);
  dmainfo(" RAWINTERRST[%08x]: %08x\n",
          LPC17_40_DMA_RAWINTERRST, regs->gbl.rawinterrst);
  dmainfo("   ENBLDCHNS[%08x]: %08x\n",
          LPC17_40_DMA_ENBLDCHNS, regs->gbl.enbldchns);
  dmainfo("    SOFTBREQ[%08x]: %08x\n",
          LPC17_40_DMA_SOFTBREQ, regs->gbl.softbreq);
  dmainfo("    SOFTSREQ[%08x]: %08x\n",
          LPC17_40_DMA_SOFTSREQ, regs->gbl.softsreq);
  dmainfo("   SOFTLBREQ[%08x]: %08x\n",
          LPC17_40_DMA_SOFTLBREQ, regs->gbl.softlbreq);
  dmainfo("   SOFTLSREQ[%08x]: %08x\n",
          LPC17_40_DMA_SOFTLSREQ, regs->gbl.softlsreq);
  dmainfo("      CONFIG[%08x]: %08x\n",
          LPC17_40_DMA_CONFIG, regs->gbl.config);
  dmainfo("        SYNC[%08x]: %08x\n",
          LPC17_40_DMA_SYNC, regs->gbl.sync);

  /* Dump the DMA channel registers */

  base = LPC17_40_DMACH_BASE((uint32_t)dmach->chn);

  dmainfo("Channel GPDMA Registers: %d\n", dmach->chn);

  dmainfo("     SRCADDR[%08x]: %08x\n",
          base + LPC17_40_DMACH_SRCADDR_OFFSET, regs->ch.srcaddr);
  dmainfo("    DESTADDR[%08x]: %08x\n",
          base + LPC17_40_DMACH_DESTADDR_OFFSET, regs->ch.destaddr);
  dmainfo("         LLI[%08x]: %08x\n",
          base + LPC17_40_DMACH_LLI_OFFSET, regs->ch.lli);
  dmainfo("     CONTROL[%08x]: %08x\n",
          base + LPC17_40_DMACH_CONTROL_OFFSET, regs->ch.control);
  dmainfo("      CONFIG[%08x]: %08x\n",
          base + LPC17_40_DMACH_CONFIG_OFFSET, regs->ch.config);
}
#endif /* CONFIG__DEBUG_DMA_INFO */

#endif /* CONFIG_LPC17_40_GPDMA */
