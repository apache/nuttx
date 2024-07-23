/****************************************************************************
 * boards/risc-v/esp32c3/common/include/esp_board_rmt.h
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

#ifndef __BOARDS_RISC_V_ESP32C3_COMMON_INCLUDE_ESP_BOARD_RMT_H
#define __BOARDS_RISC_V_ESP32C3_COMMON_INCLUDE_ESP_BOARD_RMT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_ESP_RMT

/****************************************************************************
 * Name: board_rmt_rxinitialize
 *
 * Description:
 *   Initialize the RMT peripheral and register an RX device.
 *
 * Input Parameters:
 *   ch  - The RMT's channel that will be used
 *   pin - The pin used for the RX channel
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int board_rmt_rxinitialize(int ch, int pin);

/****************************************************************************
 * Name: board_rmt_txinitialize
 *
 * Description:
 *   Initialize the RMT peripheral and register an TX device.
 *
 * Input Parameters:
 *   ch  - The RMT's channel that will be used
 *   pin - The pin used for the TX channel
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int board_rmt_txinitialize(int ch, int pin);

#endif /* CONFIG_ESP_RMT */

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_RISC_V_ESP32C3_COMMON_INCLUDE_ESP_BOARD_RMT_H */
