/****************************************************************************
 * arch/xtensa/src/esp32/esp32_rmt.h
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

#ifndef __ARCH_XTENSA_SRC_ESP32_ESP32_RMT_H
#define __ARCH_XTENSA_SRC_ESP32_ESP32_RMT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <semaphore.h>
#include <nuttx/spinlock.h>
#include "hardware/esp32_rmt.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct rmt_dev_channel_s
{
  /* Parameters for each RMT channel */

  int       open_count;      /* Unused */
  int       ch_idx;          /* RMT channel number (0-7) */
  int       output_pin;      /* GPIO pin number attached to the RMT */
  int       next_buffer;     /* Tracking buffer (0 or 1) used next reload */
  sem_t     tx_sem;          /* Semaphore structure */

  uint32_t  *src;            /* Data to be copied to the internal buffer */
  uint32_t  src_offset;      /* Offset pointer to the src */
  size_t    words_to_send;   /* Number of 32-bit words to be sent */
  uint32_t  available_words; /* Counter of available words in the src */
  uint32_t  start_address;   /* Current RMT register buffer address */
  uint32_t  reload_thresh;   /* Threshold for reloading the internal buffer */
  void      *parent_dev;     /* Pointer to the parent RMT device structure */
};

struct rmt_dev_s
{
  /* Device configuration */

  uint8_t periph;     /* Peripheral ID */
  uint8_t irq;        /* IRQ associated with this RMT */
  uint8_t cpu;        /* CPU ID */
  int cpuint;         /* CPU interrupt assigned to this RMT */
  spinlock_t lock;

  struct rmt_dev_channel_s *channels;
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#if defined(CONFIG_ESP32_RMT)

/****************************************************************************
 * Name: rmt_load_tx_buffer
 *
 * Description:
 *   Copies chunks of data from the buffer to the RMT device memory
 *   This function can also be called on the first transmission data chunk
 *
 * Input Parameters:
 *   channel - Pointer to the channel to be reloaded
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

IRAM_ATTR void rmt_load_tx_buffer(struct rmt_dev_channel_s *channel);

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

struct rmt_dev_s *esp32_rmtinitialize(void);

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

int rmt_attach_pin_to_channel(struct rmt_dev_s *rmtdev, int ch_idx, int pin);

/****************************************************************************
 * Name: board_rmt_initialize
 *
 * Description:
 *   Initialize RMT driver and register the channel/pin pair at /dev/rtm0
 *
 * Input Parameters:
 *  output_pin          - the output pin to assing to the channel
 *  channel             - the channel that will be initialized
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned
 *   to indicate the nature of any failure.
 *
 ****************************************************************************/

int board_rmt_initialize(int output_pin, int channel);

#endif

#ifdef __cplusplus
}
#endif
#endif /* __ASSEMBLY__ */

#endif /* __ARCH_XTENSA_SRC_ESP32_ESP32_RMT_H */
