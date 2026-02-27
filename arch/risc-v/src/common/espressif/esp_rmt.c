/****************************************************************************
 * arch/risc-v/src/common/espressif/esp_rmt.c
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
#include <nuttx/rmt/rmt.h>
#include <nuttx/spinlock.h>
#include <nuttx/circbuf.h>

#include "esp_gpio.h"
#include "esp_irq.h"

#include "esp_err.h"
#include "driver/rmt_types.h"
#include "driver/rmt_rx.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_encoder.h"
#include "driver/rmt_common.h"
#include "esp_private/rmt.h"
#include "rmt_private.h"

#include "esp_rmt.h"

#ifdef CONFIG_ESP_RMT

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RMT_RX_CHANNEL_ENCODING_START \
  (SOC_RMT_CHANNELS_PER_GROUP-SOC_RMT_TX_CANDIDATES_PER_GROUP)
#define RMT_TX_CHANNEL_ENCODING_END   (SOC_RMT_TX_CANDIDATES_PER_GROUP-1)

#define RMT_IS_RX_CHANNEL(channel)  \
  ((channel) >= RMT_RX_CHANNEL_ENCODING_START)
#define RMT_IS_TX_CHANNEL(channel)  \
  ((channel) <= RMT_TX_CHANNEL_ENCODING_END)
#define RMT_DECODE_RX_CHANNEL(encode_chan)  \
  ((encode_chan - RMT_RX_CHANNEL_ENCODING_START))
#define RMT_ENCODE_RX_CHANNEL(decode_chan)  \
  ((decode_chan + RMT_RX_CHANNEL_ENCODING_START))

#if SOC_PERIPH_CLK_CTRL_SHARED
#define RMT_CLOCK_SRC_ATOMIC() PERIPH_RCC_ATOMIC()
#else
#define RMT_CLOCK_SRC_ATOMIC()
#endif

#if !SOC_RCC_IS_INDEPENDENT
#define RMT_RCC_ATOMIC() PERIPH_RCC_ATOMIC()
#else
#define RMT_RCC_ATOMIC()
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct rmt_dev_lowerhalf_s
{
  /* The following block is part of the upper-half device struct */

  const struct rmt_ops_s *ops;
  struct circbuf_s       *circbuf;
  sem_t                  *recvsem;
  int                     minor;

  /* The following is private to the ESP32 RMT driver */

  rmt_channel_handle_t handle;
  rmt_encoder_handle_t encoder;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static ssize_t esp_rmt_read(struct rmt_dev_s *dev, char *buffer,
                            size_t buflen);
static ssize_t esp_rmt_write(struct rmt_dev_s *dev,
                             const char *buffer,
                             size_t buflen);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct rmt_ops_s g_rmtops =
{
  .read = esp_rmt_read,
  .write = esp_rmt_write,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp_rmt_read
 *
 * Description:
 *   This function reads data from the RMT device.
 *   It starts the RMT module in receiving mode for a specific channel and
 *   checks for any errors. If an error occurs during the start of the RMT
 *   module, it returns the error code. Please note that this function
 *   starts the receiver, but the actual data is read from the ring buffer
 *   by the upper half driver.
 *
 * Input Parameters:
 *   dev     - Pointer to the RMT device structure.
 *   buffer  - Pointer to the buffer where the read data should be stored.
 *   buflen  - The maximum amount of data to be read.
 *
 * Returned Value:
 *   Returns the number of bytes read from the RMT device; a negated errno
 *   value is returned on any failure.
 *
 ****************************************************************************/

static ssize_t esp_rmt_read(struct rmt_dev_s *dev, char *buffer,
                            size_t buflen)
{
  esp_err_t esp_ret;
  struct rmt_dev_lowerhalf_s *priv = (struct rmt_dev_lowerhalf_s *)dev;
  rmt_receive_config_t receive_config =
    {
      .signal_range_min_ns = 1250,
      .signal_range_max_ns = 3000000,
    };

  DEBUGASSERT((buflen % 4) == 0);

  if ((buflen / 4) > (CONFIG_RMT_DEFAULT_RX_BUFFER_SIZE / 4))
    {
      rmtwarn("WARN: RMT RX buffer (%d bytes) is smaller than requested "
              "read bytes (%d bytes). A partial read will take place!\n",
              CONFIG_RMT_DEFAULT_RX_BUFFER_SIZE,
              buflen);
    }

#ifndef SOC_RMT_SUPPORT_RX_PINGPONG
  if ((buflen / 4) > RMT_MEM_ITEM_NUM)
    {
      rmtwarn("WARN: RMT RX channel is able to receive up to "
              "%d RMT items (%d bytes)!",
              RMT_MEM_ITEM_NUM, RMT_MEM_ITEM_NUM * 4);
    }
#endif

  esp_ret = rmt_receive(priv->handle, buffer,
                        buflen, &receive_config);
  if (esp_ret != ESP_OK)
    {
      rmterr("Failed to Receive RMT data");
      return -ERROR;
    }

  return (ssize_t)0;
}

/****************************************************************************
 * Name: esp_rmt_write
 *
 * Description:
 *   This function writes data to the RMT memory for a specific channel. It
 *   asserts that the length of the data is a multiple of 4, then calls the
 *   rmt_write_items function to write the items to the RMT memory.
 *
 * Input Parameters:
 *   dev     - Pointer to the RMT device structure.
 *   buffer  - Pointer to the data to be written to the RMT memory.
 *   buflen  - The length of the data to be written.
 *
 * Returned Value:
 *   Returns the number of items written to the RMT memory.
 *
 ****************************************************************************/

static ssize_t esp_rmt_write(struct rmt_dev_s *dev, const char *buffer,
                             size_t buflen)
{
  struct rmt_dev_lowerhalf_s *priv = (struct rmt_dev_lowerhalf_s *)dev;
  rmt_transmit_config_t tx_config =
    {
      .loop_count = 0,
    };

  DEBUGASSERT((buflen % 4) == 0);

  rmt_transmit(priv->handle, priv->encoder, buffer, buflen, &tx_config);
  rmt_tx_wait_all_done(priv->handle, -1);

  return (ssize_t)buflen;
}

static size_t encoder_callback(const void *data, size_t data_size,
                               size_t symbols_written, size_t symbols_free,
                               rmt_symbol_word_t *symbols, bool *done,
                               void *arg)
{
  size_t data_pos = symbols_written;
  rmt_symbol_word_t data_bytes =
    (rmt_symbol_word_t)((uint32_t *)data)[data_pos];

  /* Encode a single symbol */

  symbols[0] = data_bytes;

  if (((symbols_written + 1) * 4) == data_size)
    {
      *done = true;
    }

  return 1;
}

static bool rmt_rx_done_callback(rmt_channel_handle_t channel,
                                 const rmt_rx_done_event_data_t *edata,
                                 void *user_data)
{
  struct rmt_dev_lowerhalf_s *priv = (struct rmt_dev_lowerhalf_s *)user_data;

  int bytes = circbuf_write(priv->circbuf,
                            edata->received_symbols,
                            edata->num_symbols * 4);

  nxsem_post(priv->recvsem);
  if (bytes < (edata->num_symbols * 4))
    {
      rmterr("RMT RX BUFFER FULL");
    }

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp_rmt_tx_init
 *
 * Description:
 *   Initialize the selected RMT device in TX mode
 *
 * Input Parameters:
 *   tx_pin  - The pin used for the TX channel
 *
 * Returned Value:
 *   Valid RMT device structure reference on success; NULL, otherwise.
 *
 ****************************************************************************/

struct rmt_dev_s *esp_rmt_tx_init(int tx_pin)
{
  int channel_id;
  esp_err_t ret;
  struct rmt_dev_lowerhalf_s *priv;
  rmt_channel_handle_t tx_chan = NULL;
  rmt_encoder_handle_t simple_encoder = NULL;
  rmt_tx_channel_config_t tx_chan_config =
    {
      .clk_src = RMT_CLK_SRC_DEFAULT, /* select source clock */
      .gpio_num = tx_pin,
      .mem_block_symbols = 64,        /* increase the block size can make the LED less flickering */
      .resolution_hz = 10000000,
      .trans_queue_depth = 4,         /* set the number of transactions that can be pending in the background */
    };

  const rmt_simple_encoder_config_t simple_encoder_cfg =
    {
      .callback = encoder_callback
    };

  ret = rmt_new_tx_channel(&tx_chan_config, &tx_chan);
  if (ret != ESP_OK)
    {
      rmterr("Failed to Initialize RMT TX channel");
      return NULL;
    }

  ret = rmt_new_simple_encoder(&simple_encoder_cfg, &simple_encoder);
  if (ret != ESP_OK)
    {
      rmterr("Failed to Initialize RMT Simple Encoder");
      return NULL;
    }

  ret = rmt_enable(tx_chan);
  if (ret != ESP_OK)
    {
      rmterr("Failed to Enable RMT TX channel");
      return NULL;
    }

  priv = kmm_zalloc(sizeof(struct rmt_dev_lowerhalf_s));
  if (priv)
    {
      priv->ops = &g_rmtops;
      priv->handle = tx_chan;
      priv->encoder = simple_encoder;
    }
  else
    {
      rmterr("ERROR: memory allocation failed\n");
      return NULL;
    }

  ret = rmt_get_channel_id(tx_chan, &channel_id);
  if (ret != ESP_OK)
    {
      rmterr("Failed to Get RMT RX channel ID");
      return NULL;
    }

  priv->minor = channel_id + RMT_TX_CHANNEL_OFFSET_IN_GROUP;

  return (struct rmt_dev_s *)priv;
}

/****************************************************************************
 * Name: esp_rmt_rx_init
 *
 * Description:
 *   Initialize the selected RMT device in RX mode
 *
 * Input Parameters:
 *   rx_pin  - The pin used for the RX channel
 *
 * Returned Value:
 *   Valid RMT device structure reference on success; NULL, otherwise.
 *
 ****************************************************************************/

struct rmt_dev_s *esp_rmt_rx_init(int rx_pin)
{
  int channel_id;
  esp_err_t esp_ret;
  int ret;
  struct rmt_dev_lowerhalf_s *priv;
  rmt_channel_handle_t rx_chan = NULL;
  rmt_rx_channel_config_t rx_channel_cfg =
    {
      .clk_src = RMT_CLK_SRC_DEFAULT,
      .resolution_hz = 10000000,
      .mem_block_symbols = SOC_RMT_MEM_WORDS_PER_CHANNEL, /* amount of RMT symbols that the channel can store at a time */
      .gpio_num = rx_pin,
    };

  rmt_rx_event_callbacks_t cbs =
    {
      .on_recv_done = rmt_rx_done_callback,
    };

  esp_ret = rmt_new_rx_channel(&rx_channel_cfg, &rx_chan);
  if (esp_ret != ESP_OK)
    {
      rmterr("Failed to Initialize RMT RX channel");
      return NULL;
    }

  priv = kmm_zalloc(sizeof(struct rmt_dev_lowerhalf_s));
  if (priv)
    {
      priv->ops = &g_rmtops;
      priv->handle = rx_chan;
    }
  else
    {
      rmterr("ERROR: memory allocation failed\n");
      return NULL;
    }

  esp_ret = rmt_get_channel_id(rx_chan, &channel_id);
  if (esp_ret != ESP_OK)
    {
      rmterr("Failed to Get RMT RX channel ID");
      return NULL;
    }

  priv->minor = channel_id + RMT_RX_CHANNEL_OFFSET_IN_GROUP;

  priv->recvsem = kmm_zalloc(sizeof(sem_t));
  if (priv->recvsem == NULL)
    {
      rmterr("ERROR: memory allocation failed\n");
      return NULL;
    }

  nxsem_init(priv->recvsem, 0, 0);

  priv->circbuf = kmm_zalloc(sizeof(struct circbuf_s));
  if (priv->circbuf == NULL)
    {
      rmterr("ERROR: memory allocation failed\n");
      return NULL;
    }

  ret = circbuf_init(priv->circbuf, NULL,
                     CONFIG_RMT_DEFAULT_RX_BUFFER_SIZE);
  if (ret != OK)
    {
      rmterr("Failed to Initialize RMT RX buffer");
      return NULL;
    }

  esp_ret = rmt_rx_register_event_callbacks(rx_chan, &cbs, priv);
  if (esp_ret != ESP_OK)
    {
      rmterr("Failed to Register RMT RX event callbacks");
      return NULL;
    }

  esp_ret = rmt_enable(rx_chan);
  if (esp_ret != ESP_OK)
    {
      rmterr("Failed to Enable RMT RX channel");
      return NULL;
    }

  return (struct rmt_dev_s *)priv;
}

#endif
