/****************************************************************************
 * arch/arm/src/max326xx/max32660/max32660_dma.c
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

#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/spinlock.h>

#include "arm_internal.h"
#include "hardware/max326_dma.h"
#include "max326_periphclks.h"
#include "max326_dma.h"

#ifdef CONFIG_MAX326XX_DMA

/****************************************************************************
 * Private Types
 ****************************************************************************/

#define MAX326_IRQ_DMA(n)     (MAX326_IRQ_DMA0 + (n))

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure describes state of a DMA channel
 * TODO:  Additional reload values for chains > 2 could be held here
 */

struct max326_dmach_s
{
  uint8_t chno;       /* Channel number, 0..(MAX326_DMA_NCHAN - 1) */
  bool inuse;         /* True: DMA channel is in-use */
  bool reload;        /* True: Buffer chained, reload needed */
  dma_callback_t cb;  /* DMA complete callback function */
  void *arg;          /* Argument provided with the DMA callback */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct max326_dmach_s g_max326_dmach[MAX326_DMA_NCHAN];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: max326_dmach_terminate
 *
 * Description:
 *   Terminate the DMA and perform the DMA complete callback.
 *
 ****************************************************************************/

static void max326_dma_terminate(struct max326_dmach_s *dmach, int result)
{
  DEBUGASSERT(dmach != NULL && dmach->cb != NULL);
  uintptr_t base;

  /* Disable channel interrupts at the NVIC */

  up_disable_irq(MAX326_IRQ_DMA(dmach->chno));

  /* Enable channel interrupts at the DMA controller */

  modifyreg32(MAX326_DMA_INTEN_OFFSET, 1 << dmach->chno, 0);

  /* Disable the channel. */

  base = MAX326_DMACH_BASE(dmach->chno);
  putreg32(0, base + MAX326_DMACH_CFG_OFFSET);

  /* Perform the callback */

  dmach->cb((DMA_HANDLE)dmach, dmach->arg, result);

  /* Make sure that no further callbacks are performed */

  dmach->cb  = NULL;
  dmach->arg = NULL;
}

/****************************************************************************
 * Name: max326_dmach_interrupt
 *
 * Description:
 *   DMA channel interrupt handler.
 *
 ****************************************************************************/

static int max326_dmach_interrupt(int irq, void *context, void *arg)
{
  struct max326_dmach_s *dmach = (struct max326_dmach_s *)arg;
  uintptr_t base;
  uint32_t stat;

  DEBUGASSERT(dmach != NULL && dmach->inuse);

  /* Get and clear the DMA status */

  base = MAX326_DMACH_BASE(dmach->chno);
  stat = getreg32(base + MAX326_DMACH_STAT_OFFSET);
  putreg32(stat, base + MAX326_DMACH_STAT_OFFSET);

  /* Verify that an interrupt is pending (how could it not be?) */

  DEBUGASSERT((stat & DMACH_STAT_IPEND) != 0);
  if ((stat & DMACH_STAT_IPEND) == 0)
    {
      dmaerr("Interrupt with nothing pending: %04x\n", stat);
      return OK;
    }

  DEBUGASSERT((stat & (DMACH_STAT_CTZST | DMACH_STAT_BUSERR |
                       DMACH_STAT_TOST)) != 0);

  /* Check for DMA errors. */

  if ((stat & (DMACH_STAT_BUSERR | DMACH_STAT_TOST)) != 0)
    {
      int result;

      /* Map the error to an errno value */

      if ((stat & DMACH_STAT_BUSERR) != 0)
        {
          /* A bus error occurred */

          result = -EIO;
        }
      else
        {
          /* Otherwise, a timeout occurred */

          result = -ETIMEDOUT;
        }

      max326_dma_terminate(dmach, result);
    }

  /* The DMA status register indicates that the CTZ and /or reload events
   * occurred.  In the case of a reload, the channel status bit (CHST) will
   * be set indicating that the DMA is now busy with the second DMA transfer
   * defined in the reload registers. If CHST = 0, then the initial and
   * second DMA transfers have completed. If there are additional buffers to
   * chain, the interrupt service routine initializes the SRCRLD, DSTRLD,
   * and CNTRLD registers and sets the RLDEN bit in the CNTRLD register. The
   * interrupt service routine does not write to the CFG, SRC, DST, and CNT
   * registers, just the reload registers.
   */

  if ((stat & DMACH_STAT_CHST) != 0)
    {
      /* We must be on the buffer of a chained DMA
       * TODO:  Add software logic to manage more than two DMA buffers in
       * the chain.
       */

      /* We don't really need to do anything; the RLDEN bit is automatically
       * disabled after the reload occurs.  But let's leave some bread
       * crumbs so that we will know we have been this way.
       */

      putreg32(0, base + MAX326_DMACH_SRCRLD_OFFSET);
      putreg32(0, base + MAX326_DMACH_DSTRLD_OFFSET);
      putreg32(0, base + MAX326_DMACH_CNTRLD_OFFSET);
    }
  else
    {
      /* Otherwise, the DMA completed successfully. */

      max326_dma_terminate(dmach, OK);
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
 *   Initialize the DMA subsystem
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void weak_function arm_dma_initialize(void)
{
  int i;

  /* Enable peripheral clocking to the DMA */

  max326_dma_enableclk();

  /* Initialize each DMA channel */

  for (i = 0; i < MAX326_DMA_NCHAN; i++)
    {
      struct max326_dmach_s *dmach = &g_max326_dmach[i];

      /* Initialize the state structure
       * (assuming that it is already zeroed)
       */

      dmach->chno = i;

      /* Attach the DMA channel interrupt handler */

      DEBUGVERIFY(irq_attach(MAX326_IRQ_DMA(i), max326_dmach_interrupt,
                             dmach));
    }
}

/****************************************************************************
 * Name: max326_dma_channel
 *
 * Description:
 *   Allocate a DMA channel.
 *
 * Returned Value:
 *   If a DMA channel is available, this function returns a non-NULL, void*
 *   DMA channel handle.  NULL is returned on any failure.
 *
 ****************************************************************************/

DMA_HANDLE max326_dma_channel(void)
{
  irqstate_t flags;
  int i;

  /* Since there are only 4 DMA channel, no point in any exotic channel
   * allocation.  Just check each channel until a free one is found (on not).
   */

  flags = spin_lock_irqsave(NULL);
  for (i = 0; i < 0; i++)
    {
      struct max326_dmach_s *dmach = &g_max326_dmach[i];

      /* Check if this channel is available */

      if (!dmach->inuse)
        {
          /* No.. allocate this channel */

          dmach->inuse = true;
          spin_unlock_irqrestore(NULL, flags);
          return (DMA_HANDLE)dmach;
        }
    }

  spin_unlock_irqrestore(NULL, flags);
  return NULL;
}

/****************************************************************************
 * Name: max326_dma_free
 *
 * Description:
 *   Release a DMA channel.  NOTE:  The 'handle' used in this argument must
 *   NEVER be used again until max326_dma_channel() is called again
 *   to re-gain a valid handle.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void max326_dma_free(DMA_HANDLE handle)
{
  struct max326_dmach_s *dmach = (struct max326_dmach_s *)handle;

  DEBUGASSERT(dmach != NULL && dmach->inuse);
  dmach->inuse = false;
}

/****************************************************************************
 * Name: max326_dma_setup
 *
 * Description:
 *   Configure DMA for one transfer.
 *
 *   NOTE that an image of the DMA CFG is a required input.  Only the
 *   following fields need be provided however.  The DMA logic will handle
 *   the rest.
 *
 *     PRI          DMA priority
 *     REQSEL       Request Select
 *     REQWAIT      Request Wait Enable
 *     TOSEL        Time-Out Select
 *     PSSEL        Pre-Scale Select
 *     SRCWD        Source Width
 *     SRCINC       Source Increment Enable
 *     DSTWD        Destination Width
 *     DSTINC       Destination Increment Enable
 *     BRST         Burst Size
 *
 ****************************************************************************/

int max326_dma_setup(DMA_HANDLE handle, uint32_t cfg, uint32_t saddr,
                     uint32_t daddr, size_t nbytes)
{
  struct max326_dmach_s *dmach = (struct max326_dmach_s *)handle;
  uintptr_t base;

  DEBUGASSERT(dmach != NULL && dmach->inuse);
  DEBUGASSERT(nbytes > 0 && nbytes < (1 << 24));

  /* Set up the DMA channel registers */

  base = MAX326_DMACH_BASE(dmach->chno);
  cfg &= (DMACH_CFG_PRI_MASK | DMACH_CFG_REQSEL_MASK | DMACH_CFG_REQWAIT |
          DMACH_CFG_TOSEL_MASK | DMACH_CFG_PSSEL_MASK |
          DMACH_CFG_SRCWD_MASK | DMACH_CFG_SRCINC | DMACH_CFG_DSTWD_MASK |
          DMACH_CFG_DSTINC | DMACH_CFG_BRST_MASK);
  putreg32(cfg, base + MAX326_DMACH_CFG_OFFSET);

  putreg32(saddr,  base + MAX326_DMACH_SRC_OFFSET);
  putreg32(daddr,  base + MAX326_DMACH_DST_OFFSET);
  putreg32(nbytes, base + MAX326_DMACH_CNT_OFFSET);

  /* No reload (yet) */

  putreg32(0, base + MAX326_DMACH_SRCRLD_OFFSET);
  putreg32(0, base + MAX326_DMACH_DSTRLD_OFFSET);
  putreg32(0, base + MAX326_DMACH_CNTRLD_OFFSET);

  /* Setup the DMA channel state */

  dmach->reload = false;
  dmach->cb     = NULL;
  dmach->arg    = NULL;
  return OK;
}

/****************************************************************************
 * Name: max326_dma_append
 *
 * Description:
 *   Append one buffer to the DMA chain.  max326_dma_setup() should have
 *   been called to set up the first buffer.  This function may be called
 *   to chain the DMA to a second buffer.  This is done by setting the
 *   source, destination, and count reload registers.
 *
 *   REVISIT:  Currently, the implementation is limited to a single
 *   appended buffer in the chain.
 *
 ****************************************************************************/

int max326_dma_append(DMA_HANDLE handle, uint32_t saddr, uint32_t daddr,
                      size_t nbytes)
{
  struct max326_dmach_s *dmach = (struct max326_dmach_s *)handle;
  uintptr_t base;

  DEBUGASSERT(dmach != NULL && dmach->inuse && !dmach->reload);

  /* Set up the DMA channel reload registers */

  base = MAX326_DMACH_BASE(dmach->chno);
  putreg32(saddr,  base + MAX326_DMACH_SRCRLD_OFFSET);
  putreg32(daddr,  base + MAX326_DMACH_DSTRLD_OFFSET);
  putreg32(nbytes, base + MAX326_DMACH_CNTRLD_OFFSET);

  /* Setup the DMA channel state */

  dmach->reload = true;
  return OK;
}

/****************************************************************************
 * Name: max326_dmastart
 *
 * Description:
 *   Start the DMA transfer
 *
 ****************************************************************************/

int max326_dmastart(DMA_HANDLE handle, dma_callback_t callback, void *arg)
{
  struct max326_dmach_s *dmach = (struct max326_dmach_s *)handle;
  uintptr_t base;
  uint32_t cfg;

  DEBUGASSERT(dmach != NULL && dmach->inuse);

  /* Enable interrupts and reload (if appropriate) */

  base = MAX326_DMACH_BASE(dmach->chno);
  cfg  = getreg32(base + MAX326_DMACH_CFG_OFFSET);
  cfg |= (DMACH_CFG_CHDIEN | DMACH_CFG_CTZIEN);

  if (dmach->reload)
    {
      cfg |= DMACH_CFG_RLDEN;
    }

  putreg32(cfg, base + MAX326_DMACH_CFG_OFFSET);

  /* Enable channel interrupts at the DMA controller */

  modifyreg32(MAX326_DMA_INTEN_OFFSET, 0, 1 << dmach->chno);

  /* Enable the channel and start the DMA */

  cfg |= DMACH_CFG_CHEN;
  putreg32(cfg, base + MAX326_DMACH_CFG_OFFSET);

  /* Enable channel interrupts at the NVIC */

  up_enable_irq(MAX326_IRQ_DMA(dmach->chno));
  return OK;
}

/****************************************************************************
 * Name: max326_dmastop
 *
 * Description:
 *   Cancel the DMA.  After max326_dmastop() is called, the DMA channel is
 *   reset and max326_dma_setup() must be called before max326_dmastart() can
 *   be called again
 *
 ****************************************************************************/

void max326_dmastop(DMA_HANDLE handle)
{
  struct max326_dmach_s *dmach = (struct max326_dmach_s *)handle;
  max326_dma_terminate(dmach, -ECANCELED);
}

/****************************************************************************
 * Name: max326_dmasample
 *
 * Description:
 *   Sample DMA register contents
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_DMA_INFO
void max326_dmasample(DMA_HANDLE handle, struct max326_dmaregs_s *regs)
{
  struct max326_dmach_s *dmach = (struct max326_dmach_s *)handle;
  uintptr_t base;
  uint32_t cfg;

  DEBUGASSERT(dmach != NULL);

  /* Global Registers */

  regs->inten  = getreg32(MAX326_DMA_INTEN);
  regs->intfl  = getreg32(MAX326_DMA_INTRL);

  /* Channel Registers */

  base         = MAX326_DMACH_BASE(dmach->chno);
  regs->cfg    = getreg32(base + MAX326_DMACH_CFG_OFFSET);
  regs->stat   = getreg32(base + MAX326_DMACH_STAT_OFFSET);
  regs->src    = getreg32(base + MAX326_DMACH_SRC_OFFSET);
  regs->dst    = getreg32(base + MAX326_DMACH_DST_OFFSET);
  regs->cnt    = getreg32(base + MAX326_DMACH_CNT_OFFSET);
  regs->srcrld = getreg32(base + MAX326_DMACH_SRCRLD_OFFSET);
  regs->dstrld = getreg32(base + MAX326_DMACH_DSTRLD_OFFSET);
  regs->cntrld = getreg32(base + MAX326_DMACH_CNTRLD_OFFSET);
}
#endif

/****************************************************************************
 * Name: max326_dmadump
 *
 * Description:
 *   Dump previously sampled DMA register contents
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_DMA_INFO
void max326_dmadump(DMA_HANDLE handle, const struct max326_dmaregs_s *regs,
                    const char *msg)
{
  struct max326_dmach_s *dmach = (struct max326_dmach_s *)handle;

  DEBUGASSERT(dmach != NULL);

  /* Global Registers */

  dmainfo("DMA Global Registers: %s\n", msg);
  dmainfo("   INTEN:  %04x\n", regs->inten);
  dmainfo("   INTFL:  %04x\n", regs->intfl);

  /* Channel Registers */

  dmainfo("DMA Channel %u Registers:\n", dmach->chno);
  dmainfo("     CFG:  %04x\n", regs->cfg);
  dmainfo("    STAT:  %04x\n", regs->stat);
  dmainfo("     SRC:  %04x\n", regs->src);
  dmainfo("     DST:  %04x\n", regs->dst);
  dmainfo("     CNT:  %04x\n", regs->cnt);
  dmainfo("  SRCRLD:  %04x\n", regs->srcrld);
  dmainfo("  DSTRLD:  %04x\n", regs->dstrld);
  dmainfo("  CNTRLD:  %04x\n", regs->cntrld);
}
#endif

#endif /* CONFIG_MAX326XX_DMA */
