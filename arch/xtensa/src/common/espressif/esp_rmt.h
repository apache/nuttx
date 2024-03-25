/****************************************************************************
 * arch/xtensa/src/common/espressif/esp_rmt.h
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

#ifndef __ARCH_XTENSA_SRC_COMMON_ESPRESSIF_ESP_RMT_H
#define __ARCH_XTENSA_SRC_COMMON_ESPRESSIF_ESP_RMT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <semaphore.h>
#include <nuttx/spinlock.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RMT_MEM_ITEM_NUM SOC_RMT_MEM_WORDS_PER_CHANNEL

#define RMT_DEFAULT_CLK_DIV 1

/* Channel can work during APB clock scaling */

#define RMT_CHANNEL_FLAGS_AWARE_DFS (1 << 0)

/* Invert RMT signal */

#define RMT_CHANNEL_FLAGS_INVERT_SIG (1 << 1)

/****************************************************************************
 * Public Types
 ****************************************************************************/

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

#if defined(CONFIG_ESP_RMT)

/****************************************************************************
 * Name: esp_rmt_tx_init
 *
 * Description:
 *   Initialize the selected RMT device in TX mode
 *
 * Input Parameters:
 *   ch   - The RMT's channel that will be used
 *   pin  - The pin used for the TX channel
 *
 * Returned Value:
 *   Valid RMT device structure reference on success; NULL, otherwise.
 *
 ****************************************************************************/

struct rmt_dev_s *esp_rmt_tx_init(int ch, int pin);

/****************************************************************************
 * Name: esp_rmt_rx_init
 *
 * Description:
 *   Initialize the selected RMT device in RC mode
 *
 * Input Parameters:
 *   ch   - The RMT's channel that will be used
 *   pin  - The pin used for the RX channel
 *
 * Returned Value:
 *   Valid RMT device structure reference on success; NULL, otherwise.
 *
 ****************************************************************************/

struct rmt_dev_s *esp_rmt_rx_init(int ch, int pin);

#endif

#ifdef __cplusplus
}
#endif
#endif /* __ASSEMBLY__ */

#endif /* __ARCH_XTENSA_SRC_COMMON_ESPRESSIF_ESP_RMT_H */
