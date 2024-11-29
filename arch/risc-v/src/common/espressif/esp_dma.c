/****************************************************************************
 * arch/risc-v/src/common/espressif/esp_dma.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

#include <sys/param.h>
#include <sys/types.h>
#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/mutex.h>
#include <nuttx/nuttx.h>
#include <nuttx/kmalloc.h>
#include <arch/irq.h>

#include "riscv_internal.h"
#include "esp_dma.h"

#include "soc/gdma_periph.h"
#include "hal/gdma_hal.h"
#include "hal/gdma_types.h"
#include "hal/gdma_ll.h"
#include "periph_ctrl.h"
#include "hal/dma_types.h"

/****************************************************************************
 * Pre-processor Macros
 ****************************************************************************/

/* DMA channel number */

#define ESPRESSIF_DMA_CHAN_MAX  (SOC_GDMA_PAIRS_PER_GROUP)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static bool g_dma_chan_used[ESPRESSIF_DMA_CHAN_MAX];
static mutex_t g_dma_lock = NXMUTEX_INITIALIZER;
static gdma_hal_context_t ctx;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp_dma_request
 *
 * Description:
 *   Request DMA channel and config it with given parameters.
 *
 * Input Parameters:
 *   periph   - Peripheral for which the DMA channel request was made
 *   tx_prio  - Interrupt priority
 *   rx_prio  - Interrupt flags
 *   burst_en - Enable burst transmission
 *
 * Returned Value:
 *   DMA channel number (>=0) if success or -1 if fail.
 *
 ****************************************************************************/

int32_t esp_dma_request(enum esp_dma_periph_e periph,
                        uint32_t tx_prio,
                        uint32_t rx_prio,
                        bool burst_en)
{
  int chan;

  DEBUGASSERT((periph  <= (int)ESPRESSIF_DMA_PERIPH_PARLIO));

  DEBUGASSERT(tx_prio <= GDMA_LL_CHANNEL_MAX_PRIORITY);
  DEBUGASSERT(rx_prio <= GDMA_LL_CHANNEL_MAX_PRIORITY);

  dmainfo("periph=%" PRIu32 " tx_prio=%" PRIu32 " rx_prio=%" PRIu32 "\n",
          (uint32_t)periph, tx_prio, rx_prio);

  nxmutex_lock(&g_dma_lock);

  for (chan = 0; chan < SOC_GDMA_PAIRS_PER_GROUP; chan++)
    {
      if (!g_dma_chan_used[chan])
        {
          g_dma_chan_used[chan] = true;
          break;
        }
    }

  if (chan == SOC_GDMA_PAIRS_PER_GROUP)
    {
      dmaerr("No available GDMA channel for allocation\n");

      nxmutex_unlock(&g_dma_lock);
      return ERROR;
    }

  dmainfo("Allocated channel=%d\n", chan);

  gdma_ll_rx_connect_to_periph(ctx.dev, chan, periph, periph);
  gdma_ll_tx_connect_to_periph(ctx.dev, chan, periph, periph);

  if (burst_en)
    {
      /* Enable DMA TX/RX channels burst sending data */

      gdma_ll_tx_enable_data_burst(ctx.dev, chan, true);
      gdma_ll_rx_enable_data_burst(ctx.dev, chan, true);

      /* Enable DMA TX/RX channels burst reading descriptor link */

      gdma_ll_tx_enable_descriptor_burst(ctx.dev, chan, true);
      gdma_ll_rx_enable_descriptor_burst(ctx.dev, chan, true);
    }

  /* Set priority for DMA TX/RX channels */

  gdma_ll_tx_set_priority(ctx.dev, chan, tx_prio);
  gdma_ll_rx_set_priority(ctx.dev, chan, rx_prio);

  nxmutex_unlock(&g_dma_lock);
  return chan;
}

/****************************************************************************
 * Name: esp_dma_setup
 *
 * Description:
 *   Set up DMA descriptor with given parameters.
 *
 * Input Parameters:
 *   chan    - DMA channel
 *   tx      - true: TX mode; false: RX mode
 *   dmadesc - DMA descriptor pointer
 *   num     - DMA descriptor number
 *   pbuf    - Buffer pointer
 *   len     - Buffer length by byte
 *
 * Returned Value:
 *   Bind pbuf data bytes.
 *
 ****************************************************************************/

uint32_t esp_dma_setup(int chan, bool tx,
                       struct esp_dmadesc_s *dmadesc, uint32_t num,
                       uint8_t *pbuf, uint32_t len)
{
  int i;
  uint32_t regval;
  uint32_t bytes = len;
  uint8_t *pdata = pbuf;
  uint32_t data_len;
  uint32_t buf_len;
  dma_descriptor_t *dma_desc = (dma_descriptor_t *)dmadesc;

  DEBUGASSERT(chan >= 0);
  DEBUGASSERT(dmadesc != NULL);
  DEBUGASSERT(num > 0);
  DEBUGASSERT(pbuf != NULL);
  DEBUGASSERT(len > 0);

  for (i = 0; i < num; i++)
    {
      data_len = MIN(bytes, ESPRESSIF_DMA_BUFLEN_MAX);

      /* Buffer length must be rounded to next 32-bit boundary. */

      buf_len = ALIGN_UP(data_len, sizeof(uintptr_t));

      dma_desc[i].dw0.size = buf_len;
      dma_desc[i].dw0.length = data_len;
      dma_desc[i].dw0.owner = 1;
      dma_desc[i].buffer = pdata;

      dmadesc[i].next = &dmadesc[i + 1];

      bytes -= data_len;
      if (bytes == 0)
        {
          break;
        }

      pdata += data_len;
    }

  dma_desc[i].dw0.suc_eof = 1;
  dmadesc[i].next  = NULL;

  if (tx)
    {
      /* Reset DMA TX channel FSM and FIFO pointer */

      gdma_ll_tx_reset_channel(ctx.dev, chan);

      /* Set the descriptor link base address for TX channel */

      gdma_ll_tx_set_desc_addr(ctx.dev, chan, (uint32_t)dmadesc);
    }
  else
    {
      /* Reset DMA RX channel FSM and FIFO pointer */

      gdma_ll_rx_reset_channel(ctx.dev, chan);

      /* Set the descriptor link base address for RX channel */

      gdma_ll_rx_set_desc_addr(ctx.dev, chan, (uint32_t)dmadesc);
    }

  return len - bytes;
}

/****************************************************************************
 * Name: esp_dma_load
 *
 * Description:
 *   Load the address of the first DMA descriptor of an already bound
 *   inlink/outlink to the corresponding GDMA_<IN/OUT>LINK_ADDR_CHn register
 *
 * Input Parameters:
 *   dmadesc - Pointer of the previously bound inlink/outlink
 *   chan    - DMA channel of the receiver/transmitter
 *   tx      - true: TX mode (transmitter); false: RX mode (receiver)
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void esp_dma_load(struct esp_dmadesc_s *dmadesc, int chan, bool tx)
{
  uint32_t regval;

  DEBUGASSERT(chan >= 0);
  DEBUGASSERT(dmadesc != NULL);

  if (tx)
    {
      /* Reset DMA TX channel FSM and FIFO pointer */

      gdma_ll_rx_reset_channel(ctx.dev, chan);

      /* Set the descriptor link base address for TX channel */

      gdma_ll_tx_set_desc_addr(ctx.dev, chan, (uint32_t)dmadesc);
    }
  else
    {
      /* Reset DMA RX channel FSM and FIFO pointer */

      gdma_ll_rx_reset_channel(ctx.dev, chan);

      /* Set the descriptor link base address for RX channel */

      gdma_ll_rx_set_desc_addr(ctx.dev, chan, (uint32_t)dmadesc);
    }
}

/****************************************************************************
 * Name: esp_dma_enable
 *
 * Description:
 *   Enable DMA channel transmission.
 *
 * Input Parameters:
 *   chan - DMA channel
 *   tx   - true: TX mode; false: RX mode
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp_dma_enable(int chan, bool tx)
{
  if (tx)
    {
      gdma_ll_tx_start(ctx.dev, chan);
    }
  else
    {
      gdma_ll_rx_start(ctx.dev, chan);
    }
}

/****************************************************************************
 * Name: esp_dma_disable
 *
 * Description:
 *   Disable DMA channel transmission.
 *
 * Input Parameters:
 *   chan - DMA channel
 *   tx   - true: TX mode; false: RX mode
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp_dma_disable(int chan, bool tx)
{
  if (tx)
    {
      gdma_ll_tx_stop(ctx.dev, chan);
    }
  else
    {
      gdma_ll_rx_stop(ctx.dev, chan);
    }
}

/****************************************************************************
 * Name: esp_dma_wait_idle
 *
 * Description:
 *   Wait until transmission ends.
 *
 * Input Parameters:
 *   chan - DMA channel
 *   tx   - true: TX mode; false: RX mode
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp_dma_wait_idle(int chan, bool tx)
{
  if (tx)
    {
      while (gdma_ll_tx_is_fsm_idle(ctx.dev, chan) == 0);
    }
  else
    {
      while (gdma_ll_rx_is_fsm_idle(ctx.dev, chan) == 0);
    }
}

/****************************************************************************
 * Name: esp_dma_init
 *
 * Description:
 *   Initialize DMA driver.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp_dma_init(void)
{
  periph_module_enable(PERIPH_GDMA_MODULE);
  gdma_hal_init(&ctx, 0);
  gdma_ll_enable_clock(ctx.dev, true);
}

/****************************************************************************
 * Name: esp_dma_deinit
 *
 * Description:
 *   Deinitialize DMA driver.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp_dma_deinit(void)
{
  nxmutex_lock(&g_dma_lock);

  /* Disable DMA clock gating */

  gdma_ll_enable_clock(ctx.dev, false);

  /* Disable DMA module by gating the clock and asserting the reset
   * signal.
   */

  periph_module_disable(PERIPH_GDMA_MODULE);

  nxmutex_unlock(&g_dma_lock);
}
