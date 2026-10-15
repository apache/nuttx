/****************************************************************************
 * include/nuttx/net/bluetooth/mesh/proxy.h
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

/** @file
 *  @brief Proxy APIs.
 */

/* Copyright (c) 2017 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef __INCLUDE_NUTTX_NET_BLUETOOTH_MESH_PROXY_H
#define __INCLUDE_NUTTX_NET_BLUETOOTH_MESH_PROXY_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

#include <nuttx/net/bluetooth/compat/kernel.h>
#include <nuttx/net/bluetooth/compat/sys/iterable_sections.h>

/**
 * @brief Proxy
 * @defgroup bt_mesh_proxy Proxy
 * @ingroup bt_mesh
 * @{
 */

#ifdef __cplusplus
extern "C"
{
#endif

/** Callbacks for the Proxy feature.
 *
 *  Should be instantiated with @ref BT_MESH_PROXY_CB_DEFINE.
 */

struct bt_mesh_proxy_cb
{
  /** @brief Started sending Node Identity beacons on the given subnet.
   *
   *  @param net_idx Network index the Node Identity beacons are running on.
   */

  void (*identity_enabled)(uint16_t net_idx);
  /** @brief Stopped sending Node Identity beacons on the given subnet.
   *
   *  @param net_idx Network index the Node Identity beacons were running on.
   */

  void (*identity_disabled)(uint16_t net_idx);
};

/**
 *  @brief Register a callback structure for Proxy events.
 *
 *  Registers a structure with callback functions that gets called on various
 *  Proxy events.
 *
 *  @param _name Name of callback structure.
 */
#define BT_MESH_PROXY_CB_DEFINE(_name)                                                             \
static const STRUCT_SECTION_ITERABLE(bt_mesh_proxy_cb, _CONCAT(bt_mesh_proxy_cb_, _name))

/** @brief Enable advertising with Node Identity.
 *
 *  This API requires that GATT Proxy support has been enabled. Once called
 *  each subnet will start advertising using Node Identity for the next 60
 *  seconds.
 *
 *  @return 0 on success, or (negative) error code on failure.
 */

int bt_mesh_proxy_identity_enable(void);

/** @brief Enable advertising with Private Node Identity.
 *
 *  This API requires that GATT Proxy support has been enabled. Once called
 *  each subnet will start advertising using Private Node Identity for the
 *  next 60 seconds.
 *
 *  @return 0 on success, or (negative) error code on failure.
 */

int bt_mesh_proxy_private_identity_enable(void);

/** @brief Allow Proxy Client to auto connect to a network.
 *
 *  This API allows a proxy client to auto-connect a given network.
 *
 *  @param net_idx Network Key Index
 *
 *  @return 0 on success, or (negative) error code on failure.
 */

int bt_mesh_proxy_connect(uint16_t net_idx);

/** @brief Disallow Proxy Client to auto connect to a network.
 *
 *  This API disallows a proxy client to connect a given network.
 *
 *  @param net_idx Network Key Index
 *
 *  @return 0 on success, or (negative) error code on failure.
 */

int bt_mesh_proxy_disconnect(uint16_t net_idx);

/** @brief Schedule advertising of Solicitation PDUs.
 *
 *  Once called, the device will schedule advertising Solicitation PDUs for
 *  the amount of time defined by @c adv_int * (@c
 *     CONFIG_BT_MESH_SOL_ADV_XMIT + 1), where @c adv_int is 20ms for
 *     Bluetooth v5.0 or higher, or 100ms otherwise.
 *
 *  If the number of advertised Solicitation PDUs reached 0xFFFFFF, the
 *  advertisements will no longer be started until the node is reprovisioned.
 *
 *  @param net_idx  Network Key Index
 *
 *  @return 0 on success, or (negative) error code on failure.
 */

int bt_mesh_proxy_solicit(uint16_t net_idx);

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif /* __INCLUDE_NUTTX_NET_BLUETOOTH_MESH_PROXY_H */
