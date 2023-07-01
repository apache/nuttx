/****************************************************************************
 * arch/xtensa/src/esp32/esp32_rmt.c
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

#include <stdio.h>
#include <sys/types.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <arch/board/board.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/spinlock.h>

#include "xtensa.h"

#include "esp32_gpio.h"
#include "esp32_rmt.h"
#include "esp32_irq.h"
#include "esp32_clockconfig.h"

#include "hardware/esp32_dport.h"
#include "hardware/esp32_gpio_sigmap.h"

#ifdef CONFIG_ESP32_RMT

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* RMT methods */

static void rmt_reset(struct rmt_dev_s *dev);
static int rmt_setup(struct rmt_dev_s *dev);
IRAM_ATTR static int rmt_interrupt(int irq, void *context, void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct rmt_dev_s g_rmt_dev =
{
  .periph  = ESP32_PERIPH_RMT,
  .irq     = ESP32_IRQ_RMT,
  .cpu     = 0,
  .cpuint  = -ENOMEM,
  .lock    = 0
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rmt_reset
 *
 * Description:
 *   Reset the RMT device.  Called early to initialize the hardware. This
 *   function is called, before esp32_rmt_setup().
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" RMT driver state structure.
 *
 * Returned Value:
 *  None
 *
 ****************************************************************************/

static void rmt_reset(struct rmt_dev_s *dev)
{
  irqstate_t flags;

  flags = spin_lock_irqsave(&dev->lock);

  modifyreg32(DPORT_PERIP_RST_EN_REG, DPORT_RMT_RST, 1);
  modifyreg32(DPORT_PERIP_RST_EN_REG, DPORT_RMT_RST, 0);

  /* Clear any spurious IRQ Flag   */

  putreg32(0xffffffff, RMT_INT_CLR_REG);

  /* Enable memory wrap-around */

  modifyreg32(RMT_APB_CONF_REG, 0 , BIT(1));

  spin_unlock_irqrestore(&dev->lock, flags);
}

/****************************************************************************
 * Name: rmt_setup
 *
 * Description:
 *   Configure the RMT. This method is called the first time that the RMT
 *   device is opened.  This will occur when the port is first opened.
 *   This setup includes configuring and attaching RMT interrupts.
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" RMT driver state structure.
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure
 *
 ****************************************************************************/

static int rmt_setup(struct rmt_dev_s *dev)
{
  irqstate_t flags;
  int ret = OK;

  flags = spin_lock_irqsave(&dev->lock);

  if (dev->cpuint != -ENOMEM)
    {
      /* Disable the provided CPU Interrupt to configure it. */

      up_disable_irq(dev->cpuint);
    }

  dev->cpu = up_cpu_index();
  dev->cpuint = esp32_setup_irq(dev->cpu, dev->periph,
                                1, ESP32_CPUINT_LEVEL);
  if (dev->cpuint < 0)
    {
      /* Failed to allocate a CPU interrupt of this type. */

      ret = dev->cpuint;
      spin_unlock_irqrestore(&dev->lock, flags);

      return ret;
    }

  ret = irq_attach(dev->irq, rmt_interrupt, dev);

  if (ret != OK)
    {
      /* Failed to attach IRQ, so CPU interrupt must be freed. */

      esp32_teardown_irq(dev->cpu, dev->periph, dev->cpuint);
      dev->cpuint = -ENOMEM;
      spin_unlock_irqrestore(&dev->lock, flags);

      return ret;
    }

  /* Enable the CPU interrupt that is linked to the RMT device. */

  up_enable_irq(dev->irq);

  spin_unlock_irqrestore(&dev->lock, flags);

  return ret;
}

/****************************************************************************
 * Name: rmt_load_tx_buffer
 *
 * Description:
 *   Copies chunks of data from the buffer to the RMT device memory
 *   This function can also be called on the first transmition data chunk
 *
 * Input Parameters:
 *   channel - Pointer to the channel to be reloaded
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

IRAM_ATTR void rmt_load_tx_buffer(struct rmt_dev_channel_s *channel)
{
  uint32_t *src = channel->src;
  uint32_t dst_mem;
  uint32_t buffer_size;

  if (channel->src_offset == 0)
    {
      buffer_size = channel->available_words;
      dst_mem = channel->start_address;
      channel->next_buffer = 0;
    }
  else
    {
      buffer_size =  channel->reload_thresh;
      dst_mem = channel->start_address +
        4*channel->next_buffer*channel->reload_thresh;

      /* only swap buffers after the first call */

      if (channel->next_buffer == 0)
        {
          channel->next_buffer = 1;
        }
      else
        {
          channel->next_buffer = 0;
        }
    }

  while (channel->src_offset < channel->words_to_send && buffer_size > 0)
    {
      uint32_t word_to_send = *(src + channel->src_offset);
      putreg32(word_to_send, dst_mem);

      channel->src_offset++;
      dst_mem += 4;
      buffer_size--;
    }

  /* Adding 0x00 on RMT's buffer marks the EOT */

  if (channel->src_offset == channel->words_to_send && buffer_size > 0)
    {
      putreg32(0x00, dst_mem);
    }
}

/****************************************************************************
 * Name: rmt_interrupt
 *
 * Description:
 *   RMT TX interrupt handler
 *
 * Input Parameters:
 *   irq - The IRQ number of the interrupt.
 *   context - The register state save array at the time of the interrupt.
 *   arg - The pointer to driver structure.
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure
 *
 ****************************************************************************/

IRAM_ATTR static int rmt_interrupt(int irq, void *context, void *arg)
{
  struct rmt_dev_s *dev = (struct rmt_dev_s *)arg;
  uint32_t regval = getreg32(RMT_INT_ST_REG);

  uint8_t error_flag = 0;

  int flags = spin_lock_irqsave(&dev->lock);

  for (int ch_idx = 0; ch_idx < RMT_NUMBER_OF_CHANNELS; ch_idx++)
    {
      struct rmt_dev_channel_s *channel_data =
        (struct rmt_dev_channel_s *) &(dev->channels[ch_idx]);

      /* IRQs from channels with no pins, should be ignored */

      if (channel_data->output_pin < 0)
        {
          putreg32(RMT_CHN_TX_THR_EVENT_INT_CLR(ch_idx), RMT_INT_CLR_REG);
          putreg32(RMT_CHN_TX_END_INT_CLR(ch_idx), RMT_INT_CLR_REG);
          continue;
        }

      if (regval & RMT_CHN_TX_THR_EVENT_INT_ST(ch_idx))
        {
          putreg32(RMT_CHN_TX_THR_EVENT_INT_CLR(ch_idx), RMT_INT_CLR_REG);

          /* buffer refill */

          rmt_load_tx_buffer(channel_data);
        }
      else if (regval & RMT_CHN_TX_END_INT_ST(ch_idx))
        {
          /* end of transmition */

          modifyreg32(RMT_INT_ENA_REG,
            RMT_CHN_TX_END_INT_ENA(ch_idx) |
            RMT_CHN_TX_THR_EVENT_INT_ENA(ch_idx),
            0
          );

          putreg32(RMT_CHN_TX_END_INT_CLR(ch_idx), RMT_INT_CLR_REG);
          putreg32(RMT_CHN_TX_THR_EVENT_INT_CLR(ch_idx), RMT_INT_CLR_REG);

          /* release the lock so the write function can return */

          nxsem_post(&channel_data->tx_sem);
        }
    }

  if (error_flag)
    {
      /* clear any spurious IRQ flag */

      putreg32(0xffffffff, RMT_INT_CLR_REG);
    }

  spin_unlock_irqrestore(&dev->lock, flags);
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32_rmtinitialize
 *
 * Description:
 *   Initialize the selected RMT device
 *
 * Input Parameters:
 *
 * Returned Value:
 *   Valid RMT device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct rmt_dev_s *esp32_rmtinitialize(void)
{
  struct rmt_dev_s *rmtdev = &g_rmt_dev;
  irqstate_t flags;

  flags = spin_lock_irqsave(&rmtdev->lock);

  modifyreg32(DPORT_PERIP_CLK_EN_REG, 0, DPORT_RMT_CLK_EN);
  modifyreg32(DPORT_PERIP_RST_EN_REG, DPORT_RMT_RST, 0);

  spin_unlock_irqrestore(&rmtdev->lock, flags);

  rmt_reset(rmtdev);
  rmt_setup(rmtdev);

  rmtdev->channels = kmm_zalloc(
    sizeof(struct rmt_dev_channel_s)*
    RMT_NUMBER_OF_CHANNELS
  );

  if (!rmtdev->channels)
    {
      rmterr("Failed to allocate memory for RMT Channels");
      return NULL;
    }

  for (int ch_idx = 0; ch_idx < RMT_NUMBER_OF_CHANNELS; ch_idx++)
    {
      struct rmt_dev_channel_s *channel_data =
        (struct rmt_dev_channel_s *) &(rmtdev->channels[ch_idx]);

      channel_data->open_count  = 0;
      channel_data->ch_idx      = ch_idx;
      channel_data->output_pin  = -1;

      channel_data->available_words  = 64;
      uint32_t start_addr_chn = RMT_DATA_BASE_ADDR +
                                RMT_DATA_MEMORY_BLOCK_WORDS * 4 * ch_idx;

      channel_data->start_address  = start_addr_chn;
      channel_data->reload_thresh  = channel_data->available_words / 2;
      channel_data->parent_dev     = rmtdev;
    }

  return rmtdev;
}

/****************************************************************************
 * Name: rmt_attach_pin_to_channel
 *
 * Description:
 *   Binds a gpio pin to a RMT channel
 *
 * Input Parameters:
 *   rmtdev     - pointer the rmt device, needed for the locks
 *   output_pin - the pin used for output
 *   channel    - the RMT's channel that will be used
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure
 *
 ****************************************************************************/

int rmt_attach_pin_to_channel(struct rmt_dev_s *rmtdev, int ch_idx, int pin)
{
  irqstate_t flags;

  if (ch_idx >= RMT_NUMBER_OF_CHANNELS || pin < 0)
    {
      return -EINVAL;
    }

  flags = spin_lock_irqsave(&rmtdev->lock);

  struct rmt_dev_channel_s *channel_data =
    (struct rmt_dev_channel_s *) &(rmtdev->channels[ch_idx]);

  channel_data->output_pin = pin;
  nxsem_init(&channel_data->tx_sem, 0, 1);

  /* Configure RMT GPIO pin */

  esp32_gpio_matrix_out(pin, RMT_SIG_OUT0_IDX + ch_idx, 0, 0);
  esp32_configgpio(pin, OUTPUT_FUNCTION_1);

  spin_unlock_irqrestore(&rmtdev->lock, flags);

  return OK;
}

#endif
