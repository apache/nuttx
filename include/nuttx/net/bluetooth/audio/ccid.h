/****************************************************************************
 * include/nuttx/net/bluetooth/audio/ccid.h
 *
 * Copyright (c) ByteDance Ltd. and/or its affiliates.
 *
 * Author: chao an <anchao.archer@bytedance.com>
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

/**
 * @file
 * @brief Header for Bluetooth Audio Content Control Identifier.
 *
 * Copyright (c) 2020 Bose Corporation Copyright (c) 2021-2024 Nordic
 * Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __INCLUDE_NUTTX_NET_BLUETOOTH_AUDIO_CCID_H
#define __INCLUDE_NUTTX_NET_BLUETOOTH_AUDIO_CCID_H

/**
 * @brief Bluetooth Content Control Identifier (CCID)
 * @defgroup bt_ccid Bluetooth Content Control Identifier
 *
 * @since 3.7
 * @version 0.8.0
 *
 * @ingroup bluetooth
 * @{
 *
 * The Content Control Identifier (CCID) API manages CCIDs for @ref
 * BT_UUID_CCID characteristics.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

#include <nuttx/net/bluetooth/gatt.h>

#ifdef __cplusplus
extern "C"
{
#endif

/** Minimum CCID value */
#define BT_CCID_MIN 0U
/** Maximum CCID value */
#define BT_CCID_MAX 255U

/**
 * @brief Allocates a CCID value.
 *
 * This should always be called right before registering a GATT service that
 * contains a @ref BT_UUID_CCID characteristic. Allocating a CCID without
 * registering the characteristic may (in very rare cases) result in
 * duplicated CCIDs on the device.
 *
 * Requires that @kconfig{CONFIG_BT_CONN} is enabled.
 *
 * @retval ccid 8-bit unsigned CCID value on success
 * @retval -ENOMEM No more CCIDs can be allocated
 */

int bt_ccid_alloc_value(void);

/**
 * @brief Get the GATT attribute of a CCID value
 *
 * Searches the current GATT database for a CCID characteristic that has the
 * supplied CCID value.
 *
 * Requires that @kconfig{CONFIG_BT_CONN} is enabled.
 *
 * @param ccid The CCID to search for
 *
 * @retval NULL None was found
 * @retval attr Pointer to a GATT attribute
 */

const struct bt_gatt_attr *bt_ccid_find_attr(uint8_t ccid);

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_NET_BLUETOOTH_AUDIO_CCID_H */
