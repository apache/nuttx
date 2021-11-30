/****************************************************************************
 * arch/risc-v/src/esp32c3/esp32c3_dma.c
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
#include <string.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <arch/irq.h>

#include "riscv_arch.h"

#include "esp32c3_dma.h"

#include "hardware/esp32c3_dma.h"
#include "hardware/esp32c3_soc.h"
#include "hardware/esp32c3_system.h"

/****************************************************************************
 * Pre-processor Macros
 ****************************************************************************/

#define REG_OFF  (DMA_OUT_CONF0_CH1_REG - DMA_OUT_CONF0_CH0_REG)

#define SET_REG(_r, _ch, _v)    putreg32((_v), (_r) + (_ch) * REG_OFF)
#define GET_REG(_r, _ch)        getreg32((_r) + (_ch) * REG_OFF)

#define SET_BITS(_r, _ch, _b)   modifyreg32((_r) + (_ch) * REG_OFF, 0, (_b))
#define CLR_BITS(_r, _ch, _b)   modifyreg32((_r) + (_ch) * REG_OFF, (_b), 0)

#ifndef MIN
#  define MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif

#ifndef ALIGN_UP
#  define ALIGN_UP(num, align) (((num) + ((align) - 1)) & ~((align) - 1))
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static bool  g_dma_chan_used[ESP32C3_DMA_CHAN_MAX];
static sem_t g_dma_exc_sem = SEM_INITIALIZER(1);

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32c3_dma_request
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

int32_t esp32c3_dma_request(enum esp32c3_dma_periph_e periph,
                            uint32_t tx_prio,
                            uint32_t rx_prio,
                            bool burst_en)
{
  int chan;

  DEBUGASSERT((periph  < ESP32C3_DMA_PERIPH_NUM) &&
              (periph != ESP32C3_DMA_PERIPH_RD0) &&
              (periph != ESP32C3_DMA_PERIPH_RD1));

  DEBUGASSERT(tx_prio <= ESP32C3_DMA_TX_PRIO_MAX);
  DEBUGASSERT(rx_prio <= ESP32C3_DMA_RX_PRIO_MAX);

  dmainfo("periph=%" PRIu32 " tx_prio=%" PRIu32 " rx_prio=%" PRIu32 "\n",
          (uint32_t)periph, tx_prio, rx_prio);

  nxsem_wait_uninterruptible(&g_dma_exc_sem);

  for (chan = 0; chan < ESP32C3_DMA_CHAN_MAX; chan++)
    {
      if (!g_dma_chan_used[chan])
        {
          g_dma_chan_used[chan] = true;
          break;
        }
    }

  if (chan == ESP32C3_DMA_CHAN_MAX)
    {
      dmaerr("No available GDMA channel for allocation\n");

      nxsem_post(&g_dma_exc_sem);

      return ERROR;
    }

  dmainfo("Allocated channel=%d\n", chan);

  if (periph == ESP32C3_DMA_PERIPH_MEM)
    {
      /* Enable DMA channel M2M mode */

      SET_BITS(DMA_IN_CONF0_CH0_REG, chan, DMA_MEM_TRANS_EN_CH0_M);

      /* Just setting a valid value to the register */

      SET_REG(DMA_OUT_PERI_SEL_CH0_REG, chan, 0);
      SET_REG(DMA_IN_PERI_SEL_CH0_REG, chan, 0);
    }
  else
    {
      /* Disable DMA channel M2M mode */

      CLR_BITS(DMA_IN_CONF0_CH0_REG, chan, DMA_MEM_TRANS_EN_CH0_M);

      /* Connect DMA TX/RX channels to a given peripheral */

      SET_REG(DMA_OUT_PERI_SEL_CH0_REG, chan, periph);
      SET_REG(DMA_IN_PERI_SEL_CH0_REG, chan, periph);
    }

  if (burst_en)
    {
      /* Enable DMA TX/RX channels burst sending data */

      SET_BITS(DMA_OUT_CONF0_CH0_REG, chan, DMA_OUT_DATA_BURST_EN_CH0_M);
      SET_BITS(DMA_IN_CONF0_CH0_REG, chan, DMA_IN_DATA_BURST_EN_CH0_M);

      /* Enable DMA TX/RX channels burst reading descriptor link */

      SET_BITS(DMA_OUT_CONF0_CH0_REG, chan, DMA_OUTDSCR_BURST_EN_CH0_M);
      SET_BITS(DMA_IN_CONF0_CH0_REG, chan, DMA_INDSCR_BURST_EN_CH0_M);
    }

  /* Set priority for DMA TX/RX channels */

  SET_REG(DMA_OUT_PRI_CH0_REG, chan, tx_prio);
  SET_REG(DMA_IN_PRI_CH0_REG, chan, rx_prio);

  nxsem_post(&g_dma_exc_sem);

  return chan;
}

/****************************************************************************
 * Name: esp32c3_dma_setup
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

uint32_t esp32c3_dma_setup(int chan, bool tx,
                           struct esp32c3_dmadesc_s *dmadesc, uint32_t num,
                           uint8_t *pbuf, uint32_t len)
{
  int i;
  uint32_t regval;
  uint32_t bytes = len;
  uint8_t *pdata = pbuf;
  uint32_t data_len;
  uint32_t buf_len;

  DEBUGASSERT(chan >= 0);
  DEBUGASSERT(dmadesc != NULL);
  DEBUGASSERT(num > 0);
  DEBUGASSERT(pbuf != NULL);
  DEBUGASSERT(len > 0);

  for (i = 0; i < num; i++)
    {
      data_len = MIN(bytes, ESP32C3_DMA_BUFLEN_MAX);

      /* Buffer length must be rounded to next 32-bit boundary. */

      buf_len = ALIGN_UP(data_len, sizeof(uintptr_t));

      dmadesc[i].ctrl = (data_len << ESP32C3_DMA_CTRL_DATALEN_S) |
                        (buf_len << ESP32C3_DMA_CTRL_BUFLEN_S) |
                        ESP32C3_DMA_CTRL_OWN;
      dmadesc[i].pbuf = pdata;
      dmadesc[i].next = &dmadesc[i + 1];

      bytes -= data_len;
      if (bytes == 0)
        {
          break;
        }

      pdata += data_len;
    }

  dmadesc[i].ctrl |= ESP32C3_DMA_CTRL_EOF;
  dmadesc[i].next  = NULL;

  if (tx)
    {
      /* Reset DMA TX channel FSM and FIFO pointer */

      SET_BITS(DMA_OUT_CONF0_CH0_REG, chan, DMA_OUT_RST_CH0_M);
      CLR_BITS(DMA_OUT_CONF0_CH0_REG, chan, DMA_OUT_RST_CH0_M);

      /* Set the descriptor link base address for TX channel */

      regval = (uint32_t)dmadesc & DMA_OUTLINK_ADDR_CH0;
      SET_BITS(DMA_OUT_LINK_CH0_REG, chan, regval);
    }
  else
    {
      /* Reset DMA RX channel FSM and FIFO pointer */

      SET_BITS(DMA_IN_CONF0_CH0_REG, chan, DMA_IN_RST_CH0_M);
      CLR_BITS(DMA_IN_CONF0_CH0_REG, chan, DMA_IN_RST_CH0_M);

      /* Set the descriptor link base address for RX channel */

      regval = (uint32_t)dmadesc & DMA_INLINK_ADDR_CH0;
      SET_BITS(DMA_IN_LINK_CH0_REG, chan, regval);
    }

  return len - bytes;
}

/****************************************************************************
 * Name: esp32c3_dma_enable
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

void esp32c3_dma_enable(int chan, bool tx)
{
  if (tx)
    {
      SET_BITS(DMA_OUT_LINK_CH0_REG, chan, DMA_OUTLINK_START_CH0_M);
    }
  else
    {
      SET_BITS(DMA_IN_LINK_CH0_REG, chan, DMA_INLINK_START_CH0_M);
    }
}

/****************************************************************************
 * Name: esp32c3_dma_disable
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

void esp32c3_dma_disable(int chan, bool tx)
{
  if (tx)
    {
      SET_BITS(DMA_OUT_LINK_CH0_REG, chan, DMA_OUTLINK_STOP_CH0_M);
    }
  else
    {
      SET_BITS(DMA_IN_LINK_CH0_REG, chan, DMA_INLINK_STOP_CH0_M);
    }
}

/****************************************************************************
 * Name: esp32c3_dma_wait_idle
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

void esp32c3_dma_wait_idle(int chan, bool tx)
{
  uint32_t regval;
  uint32_t regaddr;
  uint32_t waitbits;

  if (tx)
    {
      regaddr  = DMA_OUT_LINK_CH0_REG + chan * REG_OFF;
      waitbits = DMA_OUTLINK_PARK_CH0;
    }
  else
    {
      regaddr  = DMA_IN_LINK_CH0_REG + chan * REG_OFF;
      waitbits = DMA_INLINK_PARK_CH0;
    }

  do
    {
      regval = getreg32(regaddr);
    }
  while ((waitbits & regval) == 0);
}

/****************************************************************************
 * Name: esp32c3_dma_init
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

void esp32c3_dma_init(void)
{
  modifyreg32(SYSTEM_PERIP_CLK_EN1_REG, 0, SYSTEM_DMA_CLK_EN_M);
  modifyreg32(SYSTEM_PERIP_RST_EN1_REG, SYSTEM_DMA_RST_M, 0);

  modifyreg32(DMA_MISC_CONF_REG, 0, DMA_CLK_EN_M);
}

