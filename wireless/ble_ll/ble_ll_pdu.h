/****************************************************************************
 * wireless/ble_ll/ble_ll_pdu.h
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
 * Derived from Apache Mynewt NimBLE
 * (nimble/controller/include/controller/ble_ll_pdu.h).
 *
 ****************************************************************************/

#ifndef __WIRELESS_BLE_LL_BLE_LL_PDU_H
#define __WIRELESS_BLE_LL_BLE_LL_PDU_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "ble_ll_os.h"
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Header mask for keystream generation */

#define BLE_LL_PDU_HEADERMASK_DATA (0xe3)
#define BLE_LL_PDU_HEADERMASK_BIS  (0xc3)
#define BLE_LL_PDU_HEADERMASK_CIS  (0xa3)
#define BLE_LL_PDU_PREAMBLE_1M_LEN (1)
#define BLE_LL_PDU_PREAMBLE_2M_LEN (2)
#define BLE_LL_PDU_AA_LEN          (4)
#define BLE_LL_PDU_HEADER_LEN      (2)
#define BLE_LL_PDU_CRC_LEN         (3)

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

uint32_t ble_ll_pdu_syncword_us(uint8_t phy_mode);

uint32_t ble_ll_pdu_us(uint8_t payload_len, uint8_t phy_mode);

#endif /* __WIRELESS_BLE_LL_BLE_LL_PDU_H */
