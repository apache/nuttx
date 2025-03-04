/****************************************************************************
 * arch/xtensa/src/esp32s3/esp32s3_dma.c
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
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/nuttx.h>
#include <arch/irq.h>

#include "xtensa.h"
#include "esp32s3_dma.h"

#include "hardware/esp32s3_dma.h"
#include "hardware/esp32s3_soc.h"
#include "hardware/esp32s3_system.h"

/****************************************************************************
 * Pre-processor Macros
 ****************************************************************************/

#define DMA_INVALID_PERIPH_ID        (0x3F)
#define GDMA_CH_REG_ADDR(_r, _ch)    ((_r) + (_ch) * GDMA_REG_OFFSET)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static bool    g_dma_chan_used[ESP32S3_DMA_CHAN_MAX];
static mutex_t g_dma_lock = NXMUTEX_INITIALIZER;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32s3_dma_request
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

int32_t esp32s3_dma_request(enum esp32s3_dma_periph_e periph,
                            uint32_t tx_prio,
                            uint32_t rx_prio,
                            bool burst_en)
{
  int chan;

  DEBUGASSERT(periph  < ESP32S3_DMA_PERIPH_NUM);

  DEBUGASSERT(tx_prio <= ESP32S3_DMA_TX_PRIO_MAX);
  DEBUGASSERT(rx_prio <= ESP32S3_DMA_RX_PRIO_MAX);

  dmainfo("periph=%" PRIu32 " tx_prio=%" PRIu32 " rx_prio=%" PRIu32 "\n",
          (uint32_t)periph, tx_prio, rx_prio);

  nxmutex_lock(&g_dma_lock);

  for (chan = 0; chan < ESP32S3_DMA_CHAN_MAX; chan++)
    {
      if (!g_dma_chan_used[chan])
        {
          g_dma_chan_used[chan] = true;
          break;
        }
    }

  if (chan == ESP32S3_DMA_CHAN_MAX)
    {
      dmaerr("No available GDMA channel for allocation\n");

      nxmutex_unlock(&g_dma_lock);
      return ERROR;
    }

  dmainfo("Allocated channel=%d\n", chan);

  if (periph == ESP32S3_DMA_PERIPH_MEM)
    {
      /* Enable DMA channel M2M mode */

      SET_GDMA_CH_BITS(DMA_IN_CONF0_CH0_REG, chan, DMA_MEM_TRANS_EN_CH0_M);

      /* Just setting a valid value to the register */

      SET_GDMA_CH_REG(DMA_OUT_PERI_SEL_CH0_REG, chan, 0);
      SET_GDMA_CH_REG(DMA_IN_PERI_SEL_CH0_REG, chan, 0);
    }
  else
    {
      /* Disable DMA channel M2M mode */

      CLR_GDMA_CH_BITS(DMA_IN_CONF0_CH0_REG, chan, DMA_MEM_TRANS_EN_CH0_M);

      /* Connect DMA TX/RX channels to a given peripheral */

      SET_GDMA_CH_REG(DMA_OUT_PERI_SEL_CH0_REG, chan, periph);
      SET_GDMA_CH_REG(DMA_IN_PERI_SEL_CH0_REG, chan, periph);
    }

  if (burst_en)
    {
      /* Enable DMA TX/RX channels burst sending data */

      SET_GDMA_CH_BITS(DMA_OUT_CONF0_CH0_REG, chan,
                       DMA_OUT_DATA_BURST_EN_CH0_M);
      SET_GDMA_CH_BITS(DMA_IN_CONF0_CH0_REG, chan,
                       DMA_IN_DATA_BURST_EN_CH0_M);

      /* Enable DMA TX/RX channels burst reading descriptor link */

      SET_GDMA_CH_BITS(DMA_OUT_CONF0_CH0_REG, chan,
                       DMA_OUTDSCR_BURST_EN_CH0_M);
      SET_GDMA_CH_BITS(DMA_IN_CONF0_CH0_REG, chan,
                       DMA_INDSCR_BURST_EN_CH0_M);
    }

  /* Set priority for DMA TX/RX channels */

  SET_GDMA_CH_REG(DMA_OUT_PRI_CH0_REG, chan, tx_prio);
  SET_GDMA_CH_REG(DMA_IN_PRI_CH0_REG, chan, rx_prio);

  nxmutex_unlock(&g_dma_lock);
  return chan;
}

/****************************************************************************
 * Name: esp32s3_dma_release
 *
 * Description:
 *   Release DMA channel from peripheral.
 *
 * Input Parameters:
 *   chan - Peripheral for which the DMA channel request was made
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp32s3_dma_release(int chan)
{
  DEBUGASSERT(chan  < ESP32S3_DMA_CHAN_MAX);

  nxmutex_lock(&g_dma_lock);

  /* Disconnect DMA TX channel from peripheral */

  SET_GDMA_CH_REG(DMA_OUT_PERI_SEL_CH0_REG, chan, DMA_INVALID_PERIPH_ID);

  /* Disconnect DMA RX channel from peripheral */

  SET_GDMA_CH_REG(DMA_IN_PERI_SEL_CH0_REG, chan, DMA_INVALID_PERIPH_ID);
  CLR_GDMA_CH_BITS(DMA_IN_CONF0_CH0_REG, chan, DMA_MEM_TRANS_EN_CH0_M);

  /* Disable DMA TX/RX channels burst sending data */

  CLR_GDMA_CH_BITS(DMA_OUT_CONF0_CH0_REG, chan, DMA_OUT_DATA_BURST_EN_CH0_M);
  CLR_GDMA_CH_BITS(DMA_IN_CONF0_CH0_REG, chan, DMA_IN_DATA_BURST_EN_CH0_M);

  /* Disable DMA TX/RX channels burst reading descriptor link */

  CLR_GDMA_CH_BITS(DMA_OUT_CONF0_CH0_REG, chan, DMA_OUTDSCR_BURST_EN_CH0_M);
  CLR_GDMA_CH_BITS(DMA_IN_CONF0_CH0_REG, chan, DMA_INDSCR_BURST_EN_CH0_M);

  /* Reset the priority to 0 (lowest) */

  SET_GDMA_CH_REG(DMA_OUT_PRI_CH0_REG, chan, 0);
  SET_GDMA_CH_REG(DMA_IN_PRI_CH0_REG, chan, 0);

  g_dma_chan_used[chan] = false;

  nxmutex_unlock(&g_dma_lock);
}

/****************************************************************************
 * Name: esp32s3_dma_setup
 *
 * Description:
 *   Initialize the DMA inlink/outlink (linked list) and bind the target
 *   buffer to its DMA descriptors.
 *
 * Input Parameters:
 *   dmadesc - Pointer to the DMA descriptors
 *   num     - Number of DMA descriptors
 *   pbuf    - RX/TX buffer pointer
 *   len     - RX/TX buffer length
 *   tx      - true: TX mode (transmitter); false: RX mode (receiver)
 *   chan    - DMA channel of the receiver/transmitter
 *
 * Returned Value:
 *   Bound pbuf data bytes
 *
 ****************************************************************************/

uint32_t esp32s3_dma_setup(struct esp32s3_dmadesc_s *dmadesc, uint32_t num,
                           uint8_t *pbuf, uint32_t len, bool tx, int chan)
{
  int i;
  uint32_t regval;
  uint32_t bytes = len;
  uint8_t *pdata = pbuf;
  uint32_t data_len;
  uint32_t buf_len;
  int alignment = 4;
  int dma_size = ESP32S3_DMA_BUFFER_MAX_SIZE;
  bool buffer_in_psram = esp32s3_ptr_extram(pdata);
  int block_size_index = 0;
  uint32_t addr = GDMA_CH_REG_ADDR(DMA_IN_CONF0_CH0_REG, chan);
  bool burst_en = REG_GET_FIELD(addr, DMA_IN_DATA_BURST_EN_CH0);

  DEBUGASSERT(dmadesc != NULL);
  DEBUGASSERT(num > 0);
  DEBUGASSERT(pbuf != NULL);
  DEBUGASSERT(len > 0);
  DEBUGASSERT(chan >= 0 && chan  < ESP32S3_DMA_CHAN_MAX);

  if (!tx && buffer_in_psram)
    {
      addr = GDMA_CH_REG_ADDR(DMA_IN_CONF1_CH0_REG, chan);
      block_size_index = REG_GET_FIELD(addr, DMA_IN_EXT_MEM_BK_SIZE_CH0);
      switch (block_size_index)
        {
          case ESP32S3_DMA_EXT_MEMBLK_64B:
            alignment = 64;
            break;

          case ESP32S3_DMA_EXT_MEMBLK_32B:
            alignment = 32;
            break;

          case ESP32S3_DMA_EXT_MEMBLK_16B:
          default:
            alignment = 16;
            break;
        }

      dma_size = 0x1000 - alignment;
    }
  else if(!tx && burst_en)
    {
      dma_size = ESP32S3_DMA_BUFLEN_MAX_4B_ALIGNED;
    }

  for (i = 0; i < num; i++)
    {
      data_len = MIN(bytes, dma_size);
      if (!tx && (burst_en || buffer_in_psram))
        {
          /* Buffer length must be rounded to next alignment boundary. */

          buf_len = ALIGN_UP(data_len, alignment);
        }
      else
        {
          buf_len = data_len;
        }

      dmadesc[i].ctrl = ESP32S3_DMA_CTRL_OWN;

      /* Only set Data Length if it's a TX buffer. Otherwise, it will be
       * written automatically by hardware.
       */

      if (tx)
        {
          dmadesc[i].ctrl |= (data_len << ESP32S3_DMA_CTRL_DATALEN_S);
        }

      dmadesc[i].ctrl |= (buf_len << ESP32S3_DMA_CTRL_BUFLEN_S);
      dmadesc[i].pbuf = pdata;
      dmadesc[i].next = &dmadesc[i + 1];

      bytes -= data_len;
      if (bytes == 0)
        {
          break;
        }

      pdata += data_len;
    }

  /* suc_eof (defined by ESP32S3_DMA_CTRL_EOF) is set by software
   * only in transmit descriptor.
   */

  if (tx)
    {
      dmadesc[i].ctrl |= ESP32S3_DMA_CTRL_EOF;
    }

  dmadesc[i].next  = NULL;

  return len - bytes;
}

/****************************************************************************
 * Name: esp32s3_dma_load
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

void esp32s3_dma_load(struct esp32s3_dmadesc_s *dmadesc, int chan, bool tx)
{
  uint32_t regval;

  DEBUGASSERT(chan >= 0);
  DEBUGASSERT(dmadesc != NULL);

  if (tx)
    {
      /* Reset DMA TX channel FSM and FIFO pointer */

      SET_GDMA_CH_BITS(DMA_OUT_CONF0_CH0_REG, chan, DMA_OUT_RST_CH0_M);
      CLR_GDMA_CH_BITS(DMA_OUT_CONF0_CH0_REG, chan, DMA_OUT_RST_CH0_M);

      /* Set the descriptor link base address for TX channel */

      regval = (uint32_t)dmadesc & DMA_OUTLINK_ADDR_CH0;
      CLR_GDMA_CH_BITS(DMA_OUT_LINK_CH0_REG, chan, DMA_OUTLINK_ADDR_CH0);
      SET_GDMA_CH_BITS(DMA_OUT_LINK_CH0_REG, chan, regval);
    }
  else
    {
      /* Reset DMA RX channel FSM and FIFO pointer */

      SET_GDMA_CH_BITS(DMA_IN_CONF0_CH0_REG, chan, DMA_IN_RST_CH0_M);
      CLR_GDMA_CH_BITS(DMA_IN_CONF0_CH0_REG, chan, DMA_IN_RST_CH0_M);

      /* Set the descriptor link base address for RX channel */

      regval = (uint32_t)dmadesc & DMA_INLINK_ADDR_CH0;
      CLR_GDMA_CH_BITS(DMA_IN_LINK_CH0_REG, chan, DMA_INLINK_ADDR_CH0);
      SET_GDMA_CH_BITS(DMA_IN_LINK_CH0_REG, chan, regval);
    }
}

/****************************************************************************
 * Name: esp32s3_dma_enable
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

void esp32s3_dma_enable(int chan, bool tx)
{
  if (tx)
    {
      SET_GDMA_CH_BITS(DMA_OUT_LINK_CH0_REG, chan, DMA_OUTLINK_START_CH0_M);
    }
  else
    {
      SET_GDMA_CH_BITS(DMA_IN_LINK_CH0_REG, chan, DMA_INLINK_START_CH0_M);
    }
}

/****************************************************************************
 * Name: esp32s3_dma_disable
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

void esp32s3_dma_disable(int chan, bool tx)
{
  if (tx)
    {
      SET_GDMA_CH_BITS(DMA_OUT_LINK_CH0_REG, chan, DMA_OUTLINK_STOP_CH0_M);
    }
  else
    {
      SET_GDMA_CH_BITS(DMA_IN_LINK_CH0_REG, chan, DMA_INLINK_STOP_CH0_M);
    }
}

/****************************************************************************
 * Name: esp32s3_dma_wait_idle
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

void esp32s3_dma_wait_idle(int chan, bool tx)
{
  uint32_t regval;
  uint32_t regaddr;
  uint32_t waitbits;

  if (tx)
    {
      regaddr  = DMA_OUT_LINK_CH0_REG + chan * GDMA_REG_OFFSET;
      waitbits = DMA_OUTLINK_PARK_CH0;
    }
  else
    {
      regaddr  = DMA_IN_LINK_CH0_REG + chan * GDMA_REG_OFFSET;
      waitbits = DMA_INLINK_PARK_CH0;
    }

  do
    {
      regval = getreg32(regaddr);
    }
  while ((waitbits & regval) == 0);
}

/****************************************************************************
 * Name: esp32s3_dma_set_ext_memblk
 *
 * Description:
 *   Configure DMA external memory block size.
 *
 * Input Parameters:
 *   chan - DMA channel
 *   tx   - true: TX mode; false: RX mode
 *   type - block size type
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp32s3_dma_set_ext_memblk(int chan, bool tx,
                                enum esp32s3_dma_ext_memblk_e type)
{
  uint32_t val;

  if (tx)
    {
      val = ((uint32_t)type << DMA_OUT_EXT_MEM_BK_SIZE_CH0_S);

      CLR_GDMA_CH_BITS(DMA_OUT_CONF1_CH0_REG,
                       chan,
                       DMA_OUT_EXT_MEM_BK_SIZE_CH0_M);
      SET_GDMA_CH_BITS(DMA_OUT_CONF1_CH0_REG, chan, val);
    }
  else
    {
      val = ((uint32_t)type << DMA_IN_EXT_MEM_BK_SIZE_CH0_S);

      CLR_GDMA_CH_BITS(DMA_IN_CONF1_CH0_REG,
                       chan,
                       DMA_IN_EXT_MEM_BK_SIZE_CH0_M);
      SET_GDMA_CH_BITS(DMA_IN_CONF1_CH0_REG, chan, val);
    }
}

/****************************************************************************
 * Name: esp32s3_dma_init
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

void esp32s3_dma_init(void)
{
  modifyreg32(SYSTEM_PERIP_CLK_EN1_REG, 0, SYSTEM_DMA_CLK_EN_M);
  modifyreg32(SYSTEM_PERIP_RST_EN1_REG, SYSTEM_DMA_RST_M, 0);

  /* enable DMA clock gating */

  modifyreg32(DMA_MISC_CONF_REG, 0, DMA_CLK_EN_M);
}
