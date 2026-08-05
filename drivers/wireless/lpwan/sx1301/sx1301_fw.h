/****************************************************************************
 * drivers/wireless/lpwan/sx1301/sx1301_fw.h
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

#ifndef __DRIVERS_WIRELESS_LPWAN_SX1301_SX1301_FW_H
#define __DRIVERS_WIRELESS_LPWAN_SX1301_SX1301_FW_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Both internal MCUs have an 8 KiB program memory */

#define SX1301_MCU_FW_SIZE 8192

/* Firmware versions, as read back from the MCU RAM at address 0x20 */

#define SX1301_FW_VERSION_AGC 4
#define SX1301_FW_VERSION_ARB 1
#define SX1301_FW_VERSION_CAL 2

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern const uint8_t g_sx1301_agc_fw[SX1301_MCU_FW_SIZE];
extern const uint8_t g_sx1301_arb_fw[SX1301_MCU_FW_SIZE];
extern const uint8_t g_sx1301_cal_fw[SX1301_MCU_FW_SIZE];

#endif /* __DRIVERS_WIRELESS_LPWAN_SX1301_SX1301_FW_H */
