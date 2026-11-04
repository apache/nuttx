/****************************************************************************
 * wireless/ble_ll/ble_ll_priv.h
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
 * Derived from Apache Mynewt NimBLE (nimble/controller/src/ble_ll_priv.h).
 *
 ****************************************************************************/

#ifndef __WIRELESS_BLE_LL_BLE_LL_PRIV_H
#define __WIRELESS_BLE_LL_BLE_LL_PRIV_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "ble_ll_os.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* if there is any radio related activity enabled
 * (scanning, advertising, connection etc)
 */

#define BLE_LL_BUSY_EXCLUDE_CONNECTIONS 0x01

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern int8_t g_ble_ll_tx_power;

extern int8_t g_ble_ll_tx_power_compensation;

extern int8_t g_ble_ll_rx_power_compensation;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int ble_ll_tx_power_round(int tx_power);

void ble_ll_tx_power_set(int tx_power);

int ble_ll_is_busy(unsigned int flags);

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ble_ll_rx_gain
 *
 ****************************************************************************/

static inline int ble_ll_rx_gain(void)
{
  int gain = g_ble_ll_rx_power_compensation;

  return gain;
}

#endif /* __WIRELESS_BLE_LL_BLE_LL_PRIV_H */
