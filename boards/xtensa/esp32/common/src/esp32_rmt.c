/****************************************************************************
 * boards/xtensa/esp32/common/src/esp32_rmt.c
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

#include <errno.h>
#include <debug.h>
#include <stdio.h>

#include "xtensa.h"

#include <nuttx/kmalloc.h>
#include "esp32_rmt.h"

#ifdef CONFIG_ESP32_RMT

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define APB_PERIOD (12.5)

#define T0H ((uint16_t)(350 / APB_PERIOD))   // ns
#define T0L ((uint16_t)(900 / APB_PERIOD))   // ns
#define T1H ((uint16_t)(900 / APB_PERIOD))   // ns
#define T1L ((uint16_t)(350 / APB_PERIOD))   // ns
#define RES ((uint16_t)(60000 / APB_PERIOD)) // ns

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

static int rmt_open(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct rmt_dev_channel_s *dev_data = inode->i_private;

  FAR struct rmt_dev_s *parent_dev =
    (struct rmt_dev_s *)dev_data->parent_dev;
  int ret;
  irqstate_t flags;
  DEBUGASSERT(parent_dev);

  nxsem_wait(&dev_data->tx_sem);

  flags = spin_lock_irqsave(&parent_dev->lock);

  if (dev_data->open_count == 0)
    {
      int ch_idx = dev_data->ch_idx;

      uint32_t reg0_addr = RMT_CHNCONF0_REG(ch_idx);
      uint32_t reg1_addr = RMT_CHNCONF1_REG(ch_idx);
      uint32_t reg_val = 0x00;

      /* a single memory block with double buffering is enough */

      uint32_t mem_blocks = 1;
      dev_data->available_words = RMT_DATA_MEMORY_BLOCK_WORDS*mem_blocks;
      dev_data->reload_thresh = dev_data->available_words / 2;
      uint32_t start_addr_chn = RMT_DATA_BASE_ADDR +
                                RMT_DATA_MEMORY_BLOCK_WORDS * 4 * ch_idx;

      dev_data->start_address = start_addr_chn;

      reg_val = (mem_blocks) << 24;
      uint32_t clock_divider = 1;
      reg_val |= (clock_divider);
      putreg32(reg_val, reg0_addr);
      reg_val = 0;

      /* use APB clock */

      reg_val |= RMT_REF_ALWAYS_ON_CHN;

      /* memory block in transmission mode */

      reg_val &= ~RMT_MEM_OWNER_CHN;
      putreg32(reg_val, reg1_addr);

      /* set when the buffer swapping IRQ must be generated */

      uint32_t reload_addr = RMT_CHN_TX_LIM_REG(ch_idx);
      rmtinfo("Setting thr limit at %08X to %d",
        reload_addr, dev_data->reload_thresh);
      putreg32(dev_data->reload_thresh, reload_addr);

      /* allow direct access to RMT's memory */

      modifyreg32(RMT_APB_CONF_REG, 0, BIT(0));
    }
  else
    {
      rmtwarn("Be careful on opening this channel multiple times");
    }

  dev_data->open_count += 1;

  ret = OK;

  spin_unlock_irqrestore(&parent_dev->lock, flags);
  nxsem_post(&dev_data->tx_sem);

  return ret;
}

static int rmt_close(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct rmt_dev_channel_s *dev_data = inode->i_private;

  FAR struct rmt_dev_s *parent_dev =
    (struct rmt_dev_s *)dev_data->parent_dev;

  int ret;
  irqstate_t flags;
  DEBUGASSERT(parent_dev);
  nxsem_wait(&dev_data->tx_sem);
  flags = spin_lock_irqsave(&parent_dev->lock);

  dev_data->open_count -= 1;

  ret = OK;

  spin_unlock_irqrestore(&parent_dev->lock, flags);
  nxsem_post(&dev_data->tx_sem);
  return ret;
}

static ssize_t rmt_write(FAR struct file *filep,
                        FAR const char *data,
                        size_t len)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct rmt_dev_channel_s *dev_data = inode->i_private;

  FAR struct rmt_dev_s *parent_dev =
    (struct rmt_dev_s *)dev_data->parent_dev;

  irqstate_t flags;
  size_t len_in_words = len / 4;

  DEBUGASSERT(parent_dev);

  if (data == NULL || (len_in_words == 0) || (len % 4))
    {
      return -EINVAL;
    }

  flags = spin_lock_irqsave(&parent_dev->lock);

  /* set RMT's memory as writable */

  uint32_t reg1_addr = RMT_CHNCONF1_REG(dev_data->ch_idx);
  modifyreg32(reg1_addr, 0, RMT_MEM_RD_RST_CHN);
  modifyreg32(reg1_addr, RMT_MEM_RD_RST_CHN, 0);

  dev_data->src = (uint32_t *)data;
  dev_data->src_offset = 0;
  dev_data->words_to_send = len_in_words;

  /* enable IRQs for buffer refill and End-of-Transmition (EOT) */

  modifyreg32(
    RMT_INT_ENA_REG,
    0,
    RMT_CHN_TX_THR_EVENT_INT_ENA(dev_data->ch_idx) |
    RMT_CHN_TX_END_INT_ENA(dev_data->ch_idx));

  rmt_load_tx_buffer(dev_data);

  /* tell RMT to start the transmition */

  modifyreg32(reg1_addr, 0, RMT_TX_START_CHN(dev_data->ch_idx));

  spin_unlock_irqrestore(&parent_dev->lock, flags);

  /* wait for the transmition to finish */

  nxsem_wait(&dev_data->tx_sem);
  nxsem_post(&dev_data->tx_sem);

  return len;
}

/****************************************************************************
 * Name: board_rmt_initialize
 *
 * Description:
 *   Initialize and register the RMT driver
 *
 * Input Parameters:
 *   devno - The device number, used to build the device path as /dev/rmtN
 *   rmt_dev - Pointer to the RMT device that will be used
 *   nleds - number of LEDs
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static const struct file_operations g_rmt_channel_fops =
{
  rmt_open,    /* open  */
  rmt_close,   /* close */
  NULL,        /* read  */
  rmt_write,   /* write */
  NULL,        /* seek  */
  NULL,        /* ioctl */
};

int board_rmt_initialize(int channel, int output_pin)
{
  struct rmt_dev_s *rmt_dev = esp32_rmtinitialize();
  DEBUGASSERT(rmt_dev);

  char devpath[13];
  int ret;

  rmt_attach_pin_to_channel(rmt_dev, channel, output_pin);

  struct rmt_dev_channel_s *channel_data  = &(rmt_dev->channels[channel]);

  /* Register the WS2812 driver at the specified location. */

  snprintf(devpath, sizeof(devpath), "/dev/rmt%d", channel);

  /* Register the character driver */

  ret = register_driver(devpath, &g_rmt_channel_fops, 0666, channel_data);

  if (ret < 0)
    {
      rmterr("ERROR: board_rmt_initialize(%s) failed: %d\n",
             devpath, ret);
      return ret;
    }

  return OK;
}
#endif
