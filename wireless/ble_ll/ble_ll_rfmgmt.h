/****************************************************************************
 * wireless/ble_ll/ble_ll_rfmgmt.h
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
 * (nimble/controller/include/controller/ble_ll_rfmgmt.h).
 *
 ****************************************************************************/

#ifndef __WIRELESS_BLE_LL_BLE_LL_RFMGMT_H
#define __WIRELESS_BLE_LL_BLE_LL_RFMGMT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "ble_ll_os.h"
#include "ble_ll_tmr.h"

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

void ble_ll_rfmgmt_init(void);
#if CONFIG_BLE_LL_RFMGMT_ENABLE_TIME > 0
void ble_ll_rfmgmt_reset(void);

/* Notify rfmgmt that scan window has changed (only called from ble_ll_scan)
 */

void ble_ll_rfmgmt_scan_changed(bool enabled, uint32_t next_window);

/* Notify rfmgmt that 1st scheduled item has changed (only called from
 * ble_ll_sched)
 */

void ble_ll_rfmgmt_sched_changed(struct ble_ll_sched_item *first);

/* Notify rfmgmt that RF is no longer needed by current event */

void ble_ll_rfmgmt_release(void);

/* Enables RF immediately and returns tick at which RF will be fully enabled
 */

uint32_t ble_ll_rfmgmt_enable_now(void);

/* Returns true only if RF is currently fully enabled (i.e. not off or
 * enabling)
 */

bool ble_ll_rfmgmt_is_enabled(void);

#endif

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

#if !(CONFIG_BLE_LL_RFMGMT_ENABLE_TIME > 0)
/****************************************************************************
 * Name: ble_ll_rfmgmt_reset
 *
 ****************************************************************************/

static inline void ble_ll_rfmgmt_reset(void)
{
}

/****************************************************************************
 * Name: ble_ll_rfmgmt_scan_changed
 *
 ****************************************************************************/

static inline void ble_ll_rfmgmt_scan_changed(bool e, uint32_t n)
{
}

/****************************************************************************
 * Name: ble_ll_rfmgmt_sched_changed
 *
 ****************************************************************************/

static inline void ble_ll_rfmgmt_sched_changed(struct ble_ll_sched_item *f)
{
}

/****************************************************************************
 * Name: ble_ll_rfmgmt_release
 *
 ****************************************************************************/

static inline void ble_ll_rfmgmt_release(void)
{
}

/****************************************************************************
 * Name: ble_ll_rfmgmt_enable_now
 *
 ****************************************************************************/

static inline uint32_t ble_ll_rfmgmt_enable_now(void)
{
  return ble_ll_tmr_get();
}

/****************************************************************************
 * Name: ble_ll_rfmgmt_is_enabled
 *
 ****************************************************************************/

static inline bool ble_ll_rfmgmt_is_enabled(void)
{
  return true;
}

#endif

#endif /* __WIRELESS_BLE_LL_BLE_LL_RFMGMT_H */
