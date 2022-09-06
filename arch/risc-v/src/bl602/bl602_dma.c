/****************************************************************************
 * arch/risc-v/src/bl602/bl602_dma.c
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
#include <nuttx/arch.h>

#include <stdint.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include "chip.h"
#include "riscv_internal.h"
#include "hardware/bl602_dma.h"
#include "bl602_dma.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure describes one DMA channel */

struct dma_channel_s
{
  uint8_t              chan;          /* DMA channel number (0-BL602_DMA_NCHANNELS) */
  bool                 inuse;         /* TRUE: The DMA channel is in use */
  bl602_dma_callback_t callback;      /* Callback invoked when the DMA completes */
  void                 *arg;          /* Argument passed to callback function */
};

/* This structure describes the state of the DMA controller */

struct dma_controller_s
{
  sem_t exclsem; /* Protects channel table */
  sem_t chansem; /* Count of free channels */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This is the overall state of the DMA controller */

static struct dma_controller_s g_dmac =
{
  .exclsem = NXSEM_INITIALIZER(1, PRIOINHERIT_FLAGS_ENABLE),
  .chansem = SEM_INITIALIZER(BL602_DMA_NCHANNELS),
};

/* This is the array of all DMA channels */

static struct dma_channel_s g_dmach[BL602_DMA_NCHANNELS];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bl602_dma_int_handler
 *
 * Description:
 *   DMA interrupt handler.
 *
 ****************************************************************************/

static int bl602_dma_int_handler(int irq, void *context, void *arg)
{
  /* We need to ack the IRQ or a mess is made */

  /* Itterate over each of the channels checking for and clearing:
   * DMA_INTTCSTATUS
   * DMA_INTERRORSTATUS
   */

  uint8_t ch;
  uint32_t tc_status;
  uint32_t err_status;

  tc_status = getreg32(BL602_DMA_INTTCSTATUS) & DMA_INTTCSTATUS_MASK;
  err_status = getreg32(BL602_DMA_INTERRORSTATUS) & DMA_INTERRORSTATUS_MASK;

  for (ch = 0; ch < BL602_DMA_NCHANNELS; ch++)
    {
      if (tc_status & (1 << ch))
        {
          dmainfo("CH %d TC Int fired\n", ch);
          putreg32((1 << ch), BL602_DMA_INTTCCLEAR);
          if (g_dmach[ch].callback != NULL)
            {
              g_dmach[ch].callback(
                ch,
                BL602_DMA_INT_EVT_TC,
                g_dmach[ch].arg);
            }
        }

      if (err_status & (1 << ch))
        {
          dmainfo("CH %d Error Int fired\n", ch);
          putreg32((1 << ch), BL602_DMA_INTERRCLR);
          if (g_dmach[ch].callback != NULL)
            {
              g_dmach[ch].callback(
                ch,
                BL602_DMA_INT_EVT_ERR,
                g_dmach[ch].arg);
            }
        }
    }

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bl602_dma_channel_request
 *
 * Description:
 *   Allocate a new DMA channel.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   0-3: DMA channel
 *   -1: Failed
 *
 ****************************************************************************/

int8_t bl602_dma_channel_request(bl602_dma_callback_t callback, void *arg)
{
  struct dma_channel_s *dmach;
  unsigned int ch;
  int ret;

  /* Take a count from the channel counting semaphore.  We may block
   * if there are no free channels.  When we get the count, then we can
   * be assured that a channel is available in the channel list and is
   * reserved for us.
   */

  ret = nxsem_wait_uninterruptible(&g_dmac.chansem);
  if (ret < 0)
    {
      return -1;
    }

  /* Get exclusive access to the DMA channel list */

  ret = nxsem_wait_uninterruptible(&g_dmac.exclsem);
  if (ret < 0)
    {
      nxsem_post(&g_dmac.chansem);
      return -1;
    }

  /* Search for an available DMA channel */

  for (ch = 0, dmach = NULL; ch < BL602_DMA_NCHANNELS; ch++)
    {
      struct dma_channel_s *candidate = &g_dmach[ch];
      if (!candidate->inuse)
        {
          dmainfo("DMA Channel %u assigned.\n", ch);
          dmach        = candidate;
          dmach->inuse = true;

          break;
        }
    }

  nxsem_post(&g_dmac.exclsem);

  /* Since we have reserved a DMA descriptor by taking a count from chansem,
   * it would be a serious logic failure if we could not find a free channel
   * for our use.
   */

  DEBUGASSERT(dmach);
  dmach->callback = callback;
  dmach->arg = arg;
  return dmach->chan;
}

/****************************************************************************
 * Name: bl602_dma_channel_release
 *
 * Description:
 *   Release a DMA channel.
 *
 * Input Parameters:
 *   channel: DMA channel.
 *
 * Returned Value:
 *   Zero (OK) is returned on success. Otherwise -1 (ERROR).
 *
 ****************************************************************************/

int bl602_dma_channel_release(uint8_t channel_id)
{
  /* Get exclusive access to the DMA channel list */

  if (nxsem_wait_uninterruptible(&g_dmac.exclsem) < 0)
    {
      return -1;
    }

  /* Verify if the channel is actually in use */

  if (g_dmach[channel_id].inuse)
    {
      /* This channel was infact in use, release it and increment the
       * count of free channels for use.
       */

      g_dmach[channel_id].inuse = false;
      nxsem_post(&g_dmac.chansem);
    }

  nxsem_post(&g_dmac.exclsem);
  return 0;
}

/****************************************************************************
 * Name: bl602_dma_channel_start
 *
 * Description:
 *   Start a DMA channel.
 *
 * Input Parameters:
 *   channel: DMA channel.
 *
 * Returned Value:
 *   Zero (OK) is returned on success. Otherwise -1 (ERROR).
 *
 ****************************************************************************/

int bl602_dma_channel_start(uint8_t channel_id)
{
  /* Unmask interrupts for:
   *  - DMA_INT_TCOMPLETED
   *  - DMA_INT_ERR
   * Note it is expected that the TC interupt to be enabled prior to this
   * function call if needed as it is nominally controlled via the LLI
   * mechanism.
   */

  modifyreg32(BL602_DMA_CH_N_REG(BL602_DMA_CONFIG_OFFSET, channel_id),
              DMA_C0CONFIG_ITC | DMA_C0CONFIG_IE,
              0);

  /* Enable channel */

  modifyreg32(BL602_DMA_CH_N_REG(BL602_DMA_CONFIG_OFFSET, channel_id),
              0,
              DMA_C0CONFIG_E);

  return 0;
}

/****************************************************************************
 * Name: bl602_dma_channel_stop
 *
 * Description:
 *   Stop a DMA channel.
 *
 * Input Parameters:
 *   channel: DMA channel.
 *
 * Returned Value:
 *   Zero (OK) is returned on success. Otherwise -1 (ERROR).
 *
 ****************************************************************************/

int bl602_dma_channel_stop(uint8_t channel_id)
{
  /* Disable channel */

  modifyreg32(BL602_DMA_CH_N_REG(BL602_DMA_CONFIG_OFFSET, channel_id),
              DMA_C0CONFIG_E,
              0);

  /* Mask interrupts for:
   *  - DMA_INT_TCOMPLETED
   *  - DMA_INT_ERR
   */

  modifyreg32(BL602_DMA_CH_N_REG(BL602_DMA_CONFIG_OFFSET, channel_id),
              0,
              DMA_C0CONFIG_ITC | DMA_C0CONFIG_IE);

  /* Clear interrupts for channel in:
   *  - DMA_INTTCCLEAR
   *  - DMA_INTERRORSTATUS
   */

  putreg32((1 << channel_id), BL602_DMA_INTTCCLEAR);
  putreg32((1 << channel_id), BL602_DMA_INTERRCLR);

  return 0;
}

/****************************************************************************
 * Name: riscv_dma_initialize
 *
 * Description:
 *   Intialize DMA controller.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void weak_function riscv_dma_initialize(void)
{
  uint8_t ch;

  #ifdef CONFIG_DEBUG_DMA_INFO
    struct bl602_dmaregs_s regs;
  #endif

  dmainfo("Initialize DMA\n");

  /* Note we may want to set EN bits in DMAEN as part of clk_cfg2.
   * At reset these bits are already set to enabled, and the documentation
   * is a little thin around this.  If we implement more low power
   * configuration we will want to be more clear about these bits.
   */

  /* Initialize the channel list  */

  for (ch = 0; ch < BL602_DMA_NCHANNELS; ch++)
    {
      g_dmach[ch].chan = ch;

      /* Disable the DMA channel */

      putreg32(0, BL602_DMA_CH_N_REG(BL602_DMA_CONFIG_OFFSET, ch));
    }

  /* Attach DMA tranfer complete interrupt handler */

  irq_attach(BL602_IRQ_DMA_ALL, bl602_dma_int_handler, NULL);
  up_enable_irq(BL602_IRQ_DMA_ALL);

  /* Enable SMDMA controller */

  modifyreg32(BL602_DMA_TOP_CONFIG, 0, DMA_TOP_CONFIG_E);

  /* Dump DMA register state */

  bl602_dmasample(&regs);
  bl602_dmadump(&regs, "Initialized DMA");
}

/****************************************************************************
 * Name: bl602_dmasample
 *
 * Description:
 *   Sample DMA register contents
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_DMA_INFO
void bl602_dmasample(struct bl602_dmaregs_s *regs)
{
  irqstate_t flags;

  /* Sample DMA registers. */

  flags              = enter_critical_section();

  regs->intstatus         = getreg32(BL602_DMA_INTSTATUS);
  regs->inttcstatus       = getreg32(BL602_DMA_INTTCSTATUS);
  regs->inttcclear        = getreg32(BL602_DMA_INTTCCLEAR);
  regs->interrorstatus    = getreg32(BL602_DMA_INTERRORSTATUS);
  regs->interrclr         = getreg32(BL602_DMA_INTERRCLR);
  regs->rawinttcstatus    = getreg32(BL602_DMA_RAWINTTCSTATUS);
  regs->rawinterrorstatus = getreg32(BL602_DMA_RAWINTERRORSTATUS);
  regs->enbldchns         = getreg32(BL602_DMA_ENBLDCHNS);
  regs->softbreq          = getreg32(BL602_DMA_SOFTBREQ);
  regs->softsreq          = getreg32(BL602_DMA_SOFTSREQ);
  regs->softlbreq         = getreg32(BL602_DMA_SOFTLBREQ);
  regs->softlsreq         = getreg32(BL602_DMA_SOFTLSREQ);
  regs->top_config        = getreg32(BL602_DMA_TOP_CONFIG);
  regs->sync              = getreg32(BL602_DMA_SYNC);
  regs->c0srcaddr         = getreg32(BL602_DMA_C0SRCADDR);
  regs->c0dstaddr         = getreg32(BL602_DMA_C0DSTADDR);
  regs->c0lli             = getreg32(BL602_DMA_C0LLI);
  regs->c0control         = getreg32(BL602_DMA_C0CONTROL);
  regs->c0config          = getreg32(BL602_DMA_C0CONFIG);
  regs->c1srcaddr         = getreg32(BL602_DMA_C1SRCADDR);
  regs->c1dstaddr         = getreg32(BL602_DMA_C1DSTADDR);
  regs->c1lli             = getreg32(BL602_DMA_C1LLI);
  regs->c1control         = getreg32(BL602_DMA_C1CONTROL);
  regs->c1config          = getreg32(BL602_DMA_C1CONFIG);
  regs->c2srcaddr         = getreg32(BL602_DMA_C2SRCADDR);
  regs->c2dstaddr         = getreg32(BL602_DMA_C2DSTADDR);
  regs->c2lli             = getreg32(BL602_DMA_C2LLI);
  regs->c2control         = getreg32(BL602_DMA_C2CONTROL);
  regs->c2config          = getreg32(BL602_DMA_C2CONFIG);
  regs->c3srcaddr         = getreg32(BL602_DMA_C3SRCADDR);
  regs->c3dstaddr         = getreg32(BL602_DMA_C3DSTADDR);
  regs->c3lli             = getreg32(BL602_DMA_C3LLI);
  regs->c3control         = getreg32(BL602_DMA_C3CONTROL);
  regs->c3config          = getreg32(BL602_DMA_C3CONFIG);

  leave_critical_section(flags);
}
#endif

/****************************************************************************
 * Name: bl602_dmadump
 *
 * Description:
 *   Dump previously sampled DMA register contents
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_DMA_INFO
void bl602_dmadump(const struct bl602_dmaregs_s *regs,
                   const char *msg)
{
  dmainfo("%s\n", msg);
  dmainfo("  DMA Registers:\n");
  dmainfo("   INTSTATUS: %08x\n", regs->intstatus);
  dmainfo("  INTTCSTATU: %08x\n", regs->inttcstatus);
  dmainfo("  INTTCCLEAR: %08x\n", regs->inttcclear);
  dmainfo("  INTERRORST: %08x\n", regs->interrorstatus);
  dmainfo("   INTERRCLR: %08x\n", regs->interrclr);
  dmainfo("  RAWINTTCST: %08x\n", regs->rawinttcstatus);
  dmainfo("  RAWINTERRO: %08x\n", regs->rawinterrorstatus);
  dmainfo("   ENBLDCHNS: %08x\n", regs->enbldchns);
  dmainfo("    SOFTBREQ: %08x\n", regs->softbreq);
  dmainfo("    SOFTSREQ: %08x\n", regs->softsreq);
  dmainfo("   SOFTLBREQ: %08x\n", regs->softlbreq);
  dmainfo("   SOFTLSREQ: %08x\n", regs->softlsreq);
  dmainfo("  TOP_CONFIG: %08x\n", regs->top_config);
  dmainfo("        SYNC: %08x\n", regs->sync);
  dmainfo("  === Channel 0 ===\n");
  dmainfo("   C0SRCADDR: %08x\n", regs->c0srcaddr);
  dmainfo("   C0DSTADDR: %08x\n", regs->c0dstaddr);
  dmainfo("       C0LLI: %08x\n", regs->c0lli);
  dmainfo("   C0CONTROL: %08x\n", regs->c0control);
  dmainfo("    C0CONFIG: %08x\n", regs->c0config);
  dmainfo("  === Channel 1 ===\n");
  dmainfo("   C1SRCADDR: %08x\n", regs->c1srcaddr);
  dmainfo("   C1DSTADDR: %08x\n", regs->c1dstaddr);
  dmainfo("       C1LLI: %08x\n", regs->c1lli);
  dmainfo("   C1CONTROL: %08x\n", regs->c1control);
  dmainfo("    C1CONFIG: %08x\n", regs->c1config);
  dmainfo("  === Channel 2 ===\n");
  dmainfo("   C2SRCADDR: %08x\n", regs->c2srcaddr);
  dmainfo("   C2DSTADDR: %08x\n", regs->c2dstaddr);
  dmainfo("       C2LLI: %08x\n", regs->c2lli);
  dmainfo("   C2CONTROL: %08x\n", regs->c2control);
  dmainfo("    C2CONFIG: %08x\n", regs->c2config);
  dmainfo("  === Channel 3 ===\n");
  dmainfo("   C3SRCADDR: %08x\n", regs->c3srcaddr);
  dmainfo("   C3DSTADDR: %08x\n", regs->c3dstaddr);
  dmainfo("       C3LLI: %08x\n", regs->c3lli);
  dmainfo("   C3CONTROL: %08x\n", regs->c3control);
  dmainfo("    C3CONFIG: %08x\n", regs->c3config);
}
#endif
