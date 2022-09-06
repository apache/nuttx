/****************************************************************************
 * arch/arm/src/lpc54xx/lpc54_dma.c
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
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/mutex.h>

#include "arm_internal.h"
#include "hardware/lpc54_inputmux.h"
#include "hardware/lpc54_dma.h"
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
  bool inuse;              /* True: The channel is in use */
  dma_callback_t callback; /* DMA completion callback function */
  void *arg;               /* Argument to pass to the callback function */
};

/* This structure represents the state of the LPC54 DMA block */

struct lpc54_dma_s
{
  mutex_t lock;           /* For exclusive access to the DMA channel list */

  /* This is the state of each DMA channel */

  struct lpc54_dmach_s dmach[LPC54_DMA_NCHANNELS];
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The state of the LPC54 DMA block */

static struct lpc54_dma_s g_dma;

/* The SRAMBASE register must be configured with an address (preferably in
 * on-chip SRAM) where DMA descriptors will be stored.  Each DMA channel has
 * an entry for the channel descriptor in the SRAM table.
 */

static struct lpc54_dmachan_desc_s g_dma_desc[LPC54_DMA_NCHANNELS];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

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

      dmach->callback(ch, dmach->arg, result);
    }

  /* Disable this channel, mask any further interrupts for this channel, and
   * clear any pending interrupts.
   */

  lpc54_dmastop(ch);
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

static int lpc54_dma_interrupt(int irq, void *context, void *arg)
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
 * Name: arm_dma_initialize
 *
 * Description:
 *   Initialize the DMA subsystem.  Called from up_initialize() early in the
 *   boot-up sequence.  Prototyped in arm_internal.h.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void weak_function arm_dma_initialize(void)
{
  int ret;

  /* Enable clocking to the DMA block */

  lpc54_dma_enableclk();

  /* Reset the DMA peripheral */

  lpc54_reset_dma();

  /* Disable and clear all DMA interrupts */

  putreg32(DMA_ALL_CHANNELS, LPC54_DMA_INTENCLR0);
  putreg32(DMA_ALL_CHANNELS, LPC54_DMA_ERRINT0);
  putreg32(DMA_ALL_CHANNELS, LPC54_DMA_INTA0);
  putreg32(DMA_ALL_CHANNELS, LPC54_DMA_INTB0);

  /* Initialize the DMA state structure */

  nxmutex_init(&g_dma.lock);

  /* Set the SRAMBASE to the beginning a array of DMA descriptors, one for
   * each DMA channel.
   */

  putreg32((uint32_t)g_dma_desc, LPC54_DMA_SRAMBASE);

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
 * Name: lpc54_dma_setup
 *
 * Description:
 *   Configure DMA for one transfer.
 *
 * Input Parameters:
 *   ch      - DMA channel number
 *   cfg     - The content of the DMA channel configuration register.  See
 *             peripheral channel definitions in chip/lpc54_dma.h.  The
 *             caller must provide all fields:  PERIPHREQEN, TRIGPOL,
 *             TRIGTYPE, TRIGBURST, BURSTPOWER, SRCBURSTWRAP, DSTBURSTWRAP,
 *             and CHPRIORITY.
 *   xfrcfg  - The content of the DMA channel configuration register.  See
 *             peripheral channel definitions in chip/lpc54_dma.h.  The
 *             caller must provide all fields:  WIDTH, SRCINC, and DSTINC.\
 *             All of fields are managed by the DMA driver
 *   trigsrc - See input mux DMA trigger ITRIG_INMUX_* definitions in
 *             chip/lpc54_inputmux.h.
 *   srcaddr  - Source address of the DMA transfer
 *   dstaddr  - Destination address of the DMA transfer
 *   nbytes   - Number of bytes to transfer
 *
 ****************************************************************************/

int lpc54_dma_setup(int ch, uint32_t cfg, uint32_t xfrcfg, uint8_t trigsrc,
                    uintptr_t srcaddr, uintptr_t dstaddr, size_t nbytes)
{
  struct lpc54_dmach_s *dmach;
  uintptr_t base;
  uintptr_t regaddr;
  uint32_t nxfrs;
  uint32_t width;
  uint32_t incr;
  int ret;

  DEBUGASSERT((unsigned)ch < LPC54_DMA_NCHANNELS && nbytes < 4096);
  dmach = &g_dma.dmach[ch];

  /* Get exclusive access to the DMA data structures and interface */

  ret = nxmutex_lock(&g_dma.lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Make sure that the DMA channel is not in use */

  DEBUGASSERT(!dmach->inuse);
  if (dmach->inuse)
    {
      ret = -EBUSY;
      goto errout_with_excllock;
    }

  dmach->inuse = true;

  /* Make sure that the trigger is not active */

  base = LPC54_DMA_CHAN_BASE(ch);
  putreg32(0, base + LPC54_DMA_CFG_OFFSET);

  /* Number of transfers */

  switch (xfrcfg & DMA_XFERCFG_WIDTH_MASK)
   {
     default:
     case DMA_XFERCFG_WIDTH_8BIT:
       width = 1;
       nxfrs = nbytes;
       break;

     case DMA_XFERCFG_WIDTH_16BIT:
       width = 2;
       nxfrs = ((nbytes + 1) >> 1);
       break;

     case DMA_XFERCFG_WIDTH_32BIT:
       width = 4;
       nxfrs = ((nbytes + 3) >> 2);
       break;
   }

  /* Check if the number of transfers can be performed */

  if (nxfrs > LPC54_DMA_MAXXFRS)
    {
      return -E2BIG;
    }

  /* Set up the channel DMA descriptor */

  g_dma_desc[ch].reserved = 0;

  switch (cfg & DMA_XFERCFG_SRCINC_MASK)
    {
      default:
      case DMA_XFERCFG_SRCINC_NONE:
        incr = 0;
        break;

      case DMA_XFERCFG_SRCINC_1X:
        incr = width;
        break;

      case DMA_XFERCFG_SRCINC_2X:
        incr = width << 1;
        break;

      case DMA_XFERCFG_SRCINC_4X:
        incr = width << 2;
        incr = 0;
        break;
    }

  g_dma_desc[ch].srcend = (uint32_t)srcaddr + nxfrs * incr;

  switch (cfg & DMA_XFERCFG_DSTINC_MASK)
    {
      default:
      case DMA_XFERCFG_DSTINC_NONE:
        incr = 0;
        break;

      case DMA_XFERCFG_DSTINC_1X:
        incr = width;
        break;

      case DMA_XFERCFG_DSTINC_2X:
        incr = width << 1;
        break;

      case DMA_XFERCFG_DSTINC_4X:
        incr = width << 2;
        incr = 0;
        break;
    }

  g_dma_desc[ch].dstend = (uint32_t)dstaddr + nxfrs * incr;
  g_dma_desc[ch].link   = 0;

  /* Set the trigger source */

  regaddr = LPC54_MUX_DMA_ITRIG_INMUX(ch);
  putreg32(MUX_DMA_ITRIG_INMUX(trigsrc), regaddr);

  /* Set the channel configuration register.
   *
   *   PERIPHREQEN  - Provided by caller
   *   TRIGPOL      - Provided by caller
   *   TRIGTYPE     - Provided by caller
   *   TRIGBURST    - Provided by caller
   *   BURSTPOWER   - Provided by caller
   *   SRCBURSTWRAP - Provided by caller
   *   DSTBURSTWRAP - Provided by caller
   *   CHPRIORITY   - Provided by caller
   */

  putreg32(cfg, base + LPC54_DMA_CFG_OFFSET);

  /* Set the channel transfer configuration register
   *
   *   CFGVALID  - Current channel descriptor is valid.
   *   RELOAD    - No reload
   *   SWTRIG    - No software trigger
   *   CLRTRIG   - Trigger cleared when descriptor is exhausted
   *   SETINTA   - Use interrupt A
   *   SETINTB   - Don't use interrupt B
   *   WIDTH     - Provided by caller
   *   SRCINC    - Provided by caller
   *   DSTINC    - Provided by caller
   *   XFERCOUNT - Derived from with and nbytes
   */

  xfrcfg &= ~(DMA_XFERCFG_RELOAD | DMA_XFERCFG_SWTRIG |
              DMA_XFERCFG_SETINTB | DMA_XFERCFG_XFERCOUNT_MASK);
  xfrcfg |= (DMA_XFERCFG_CFGVALID | DMA_XFERCFG_CLRTRIG |
              DMA_XFERCFG_SETINTA);
  xfrcfg |= DMA_XFERCFG_XFERCOUNT(nxfrs);
  putreg32(xfrcfg, base + LPC54_DMA_XFERCFG_OFFSET);
  ret = OK;

errout_with_excllock:
  nxmutex_unlock(&g_dma.lock);
  return ret;
}

/****************************************************************************
 * Name: lpc54_dmastart
 *
 * Description:
 *   Start the DMA transfer
 *
 ****************************************************************************/

int lpc54_dmastart(int ch, dma_callback_t callback, void *arg)
{
  struct lpc54_dmach_s *dmach;
  uintptr_t regaddr;
  uint32_t bitmask;

  DEBUGASSERT((unsigned)ch < LPC54_DMA_NCHANNELS);
  dmach = &g_dma.dmach[ch];
  DEBUGASSERT(dmach->inuse && callback != NULL);

  /* Save the callback information */

  dmach->callback = callback;
  dmach->arg      = arg;

  /* Clear any pending DMA interrupts */

  bitmask = DMA_CHANNEL(ch);
  putreg32(bitmask, LPC54_DMA_ERRINT0);
  putreg32(bitmask, LPC54_DMA_INTA0);
  putreg32(bitmask, LPC54_DMA_INTB0);

  /* Enable the channel and enable interrupt A and error interrupts. */

  putreg32(bitmask, LPC54_DMA_ENABLESET0); /* Enable the channel */
  putreg32(bitmask, LPC54_DMA_INTENSET0);  /* Enable channel interrupts */

  /* Enable the trigger for this channel */

  regaddr = LPC54_DMA_CTLSTAT(ch);
  modifyreg32(regaddr, 0, DMA_CTLSTAT_TRIG);
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

void lpc54_dmastop(int ch)
{
  struct lpc54_dmach_s *dmach;
  uint32_t bitmask;

  DEBUGASSERT((unsigned)ch < LPC54_DMA_NCHANNELS);
  dmach = &g_dma.dmach[ch];
  DEBUGASSERT(dmach->inuse);

  /* Disable this channel and mask any further interrupts from the channel.
   * this channel.
   */

  bitmask = DMA_CHANNEL(ch);
  putreg32(bitmask, LPC54_DMA_INTENCLR0);  /* Disable channel interrupts */
  putreg32(bitmask, LPC54_DMA_ENABLECLR0); /* Disable the channel */

  /* Clear any pending interrupts for this channel */

  putreg32(bitmask, LPC54_DMA_ERRINT0);
  putreg32(bitmask, LPC54_DMA_INTA0);
  putreg32(bitmask, LPC54_DMA_INTB0);

  /* This channel is no longer in use */

  g_dma.dmach[ch].inuse = false;
}

/****************************************************************************
 * Name: lpc54_dmasample
 *
 * Description:
 *   Sample DMA register contents
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_DMA
void lpc54_dmasample(int ch, struct lpc54_dmaregs_s *regs)
{
  uintptr_t base;

  DEBUGASSERT((unsigned)ch <  LPC54_DMA_NCHANNELS);

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

  base                  = LPC54_DMA_CHAN_BASE(ch);
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
void lpc54_dmadump(int ch, const struct lpc54_dmaregs_s *regs,
                   const char *msg)
{
  uintptr_t base;

  DEBUGASSERT((unsigned)ch <  LPC54_DMA_NCHANNELS &&
               regs != NULL && msg != NULL);

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

  base = LPC54_DMA_CHAN_BASE(ch);

  dmainfo("Channel DMA Registers: %d\n", ch);

  dmainfo("         CFG[%08x]: %08lx\n",
          base + LPC54_DMA_CFG_OFFSET, (unsigned long)regs->ch.cfg);
  dmainfo("     CTLSTAT[%08x]: %08lx\n",
          base + LPC54_DMA_CTLSTAT_OFFSET, (unsigned long)regs->ch.ctlstat);
  dmainfo("     XFERCFG[%08x]: %08lx\n",
          base + LPC54_DMA_XFERCFG_OFFSET, (unsigned long)regs->ch.xfercfg);
}
#endif /* CONFIG_DEBUG_DMA */

#endif /* CONFIG_LPC54_DMA */
