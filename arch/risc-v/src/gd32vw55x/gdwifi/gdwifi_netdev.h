/****************************************************************************
 * arch/risc-v/src/gd32vw55x/gdwifi/gdwifi_netdev.h
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

#ifndef __ARCH_RISCV_SRC_GD32VW55X_GDWIFI_GDWIFI_NETDEV_H
#define __ARCH_RISCV_SRC_GD32VW55X_GDWIFI_GDWIFI_NETDEV_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include "lwip_import.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* ABI of the buffer that the MAC firmware (libwifi.a) manipulates directly.
 *
 * THIS LAYOUT MUST NOT CHANGE: it is identical to the lwIP "struct pbuf"
 * the prebuilt libraries were compiled against
 * (macsw/import/lwip_import.h:77).  The MAC reads and writes these fields.
 * 16 bytes, 4-byte alignment.
 */

/* The struct pbuf / pbuf_custom / net_buf_* definitions come from the SDK
 * header itself (macsw/import/lwip_import.h) -- it is the binary contract
 * with the MAC firmware.  Do not redefine them here: FIXED 16-byte layout.
 *
 *   struct pbuf { next; payload; tot_len; len; type_internal; flags;
 *                 ref; if_idx; }   <- the MAC reads and writes these fields
 *   NET_AL_TX_HEADROOM = 348 B before the payload in every TX buffer
 */

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: gdwifi_netdev_register
 *
 * Description:
 *   Register the wlan0 interface with the NuttX network stack.
 *
 * Input Parameters:
 *   mac - MAC address (6 bytes) read from the efuse by the MAC firmware
 *
 ****************************************************************************/

int gdwifi_netdev_register(const uint8_t *mac);

/* Access to the wlan0 net_driver_s (used by the wifi_manager compat) */

struct net_driver_s;
struct net_driver_s *gdwifi_netdev_get(void);

#endif /* __ARCH_RISCV_SRC_GD32VW55X_GDWIFI_GDWIFI_NETDEV_H */
