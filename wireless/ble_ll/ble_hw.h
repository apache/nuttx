/****************************************************************************
 * wireless/ble_ll/ble_hw.h
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
 * (nimble/controller/include/controller/ble_hw.h).
 *
 ****************************************************************************/

#ifndef __WIRELESS_BLE_LL_BLE_HW_H
#define __WIRELESS_BLE_LL_BLE_HW_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <nuttx/wireless/bluetooth/ble_ll.h>

#include "ble.h"
#include "ble_phy.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_BLE_LL_HW_WHITELIST
#  define BLE_USES_HW_WHITELIST 1
#else
#  define BLE_USES_HW_WHITELIST 0
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Random number generation */

typedef CODE void (*ble_rng_isr_cb_t)(uint8_t rnum);

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* AES-128 ECB block encryption */

int ble_hw_encrypt_block(FAR struct ble_encryption_block *ecb);

/* Random number generator */

int ble_hw_rng_init(ble_rng_isr_cb_t cb, int bias);
int ble_hw_rng_start(void);
int ble_hw_rng_stop(void);

/* Device addresses: return 0 if present, -1 otherwise */

int ble_hw_get_public_addr(FAR ble_addr_t *addr);
int ble_hw_get_static_addr(FAR ble_addr_t *addr);

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

#ifdef CONFIG_BLE_LL_HW_WHITELIST
static inline uint8_t ble_hw_whitelist_size(void)
{
  return BLE_PHY_OP(whitelist_size);
}

static inline void ble_hw_whitelist_clear(void)
{
  BLE_PHY_OP(whitelist_clear);
}

static inline void ble_hw_whitelist_rmv(FAR const uint8_t *addr,
                                        uint8_t addr_type)
{
  BLE_PHY_OP(whitelist_rmv, addr, addr_type);
}

static inline int ble_hw_whitelist_add(FAR const uint8_t *addr,
                                       uint8_t addr_type)
{
  return BLE_PHY_OP(whitelist_add, addr, addr_type);
}

static inline void ble_hw_whitelist_enable(void)
{
  BLE_PHY_OP(whitelist_enable);
}

static inline void ble_hw_whitelist_disable(void)
{
  BLE_PHY_OP(whitelist_disable);
}

static inline int ble_hw_whitelist_match(void)
{
  return BLE_PHY_OP(whitelist_match);
}

#endif

static inline void ble_hw_resolv_list_clear(void)
{
  BLE_PHY_OP(resolv_list_clear);
}

static inline int ble_hw_resolv_list_add(FAR const uint8_t *irk)
{
  return BLE_PHY_OP(resolv_list_add, irk);
}

static inline void ble_hw_resolv_list_rmv(int index)
{
  BLE_PHY_OP(resolv_list_rmv, index);
}

static inline uint8_t ble_hw_resolv_list_size(void)
{
  return BLE_PHY_OP(resolv_list_size);
}

static inline int ble_hw_resolv_list_match(void)
{
  return BLE_PHY_OP(resolv_list_match);
}

#endif /* __WIRELESS_BLE_LL_BLE_HW_H */
