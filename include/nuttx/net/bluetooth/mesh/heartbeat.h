/****************************************************************************
 * include/nuttx/net/bluetooth/mesh/heartbeat.h
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
 *  @brief Heartbeat APIs.
 */

/* Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef __INCLUDE_NUTTX_NET_BLUETOOTH_MESH_HEARTBEAT_H
#define __INCLUDE_NUTTX_NET_BLUETOOTH_MESH_HEARTBEAT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

#include <nuttx/net/bluetooth/compat/sys/iterable_sections.h>
#include <nuttx/list.h>

/**
 * @brief Heartbeat
 * @defgroup bt_mesh_heartbeat Heartbeat
 * @ingroup bt_mesh
 * @{
 */

#ifdef __cplusplus
extern "C"
{
#endif

/** Heartbeat Publication parameters */

struct bt_mesh_hb_pub
{
  /** Destination address. */

  uint16_t dst;

  /** Remaining publish count. */

  uint16_t count;

  /** Time To Live value. */

  uint8_t ttl;
  /** Bitmap of features that trigger a Heartbeat publication if they change.
   * Legal values are @ref BT_MESH_FEAT_RELAY,
   * @ref BT_MESH_FEAT_PROXY, @ref BT_MESH_FEAT_FRIEND and
   * @ref BT_MESH_FEAT_LOW_POWER.
   */

  uint16_t feat;

  /** Network index used for publishing. */

  uint16_t net_idx;

  /** Publication period in seconds. */

  uint32_t period;
};

/** Heartbeat Subscription parameters. */

struct bt_mesh_hb_sub
{
  /** Subscription period in seconds. */

  uint32_t period;

  /** Remaining subscription time in seconds. */

  uint32_t remaining;

  /** Source address to receive Heartbeats from. */

  uint16_t src;

  /** Destination address to received Heartbeats on. */

  uint16_t dst;

  /** The number of received Heartbeat messages so far. */

  uint16_t count;
/* Minimum hops in received messages, ie the shortest registered path from
 * the publishing node to the subscribing node. A Heartbeat received from
 * an immediate neighbor has hop count = 1.
 */

  uint8_t min_hops;
/* Maximum hops in received messages, ie the longest registered path from
 * the publishing node to the subscribing node. A Heartbeat received from
 * an immediate neighbor has hop count = 1.
 */

  uint8_t max_hops;
};

/** Heartbeat callback structure */

struct bt_mesh_hb_cb
{
  /** @brief Receive callback for heartbeats.
   *
   *  Gets called on every received Heartbeat that matches the current
   *  Heartbeat subscription parameters.
   *
   *  @param sub  Current Heartbeat subscription parameters.
   *  @param hops The number of hops the Heartbeat was received with.
   *  @param feat The feature set of the publishing node. The value is a
   *              bitmap of @ref BT_MESH_FEAT_RELAY,
   *              @ref BT_MESH_FEAT_PROXY,
   *              @ref BT_MESH_FEAT_FRIEND and
   *              @ref BT_MESH_FEAT_LOW_POWER.
   */

  void (*recv)(const struct bt_mesh_hb_sub *sub,
        uint8_t hops,
        uint16_t feat);

  /** @brief Subscription end callback for heartbeats.
   *
   *  Gets called when the subscription period ends, providing a summary of
   *  the received heartbeat messages.
   *
   *  @param sub Current Heartbeat subscription parameters.
   */

  void (*sub_end)(const struct bt_mesh_hb_sub *sub);

  /** @brief Publication sent callback for heartbeats.
   *
   *  Gets called when the heartbeat is successfully published.
   *
   *  @param pub Current Heartbeat publication parameters.
   */

  void (*pub_sent)(const struct bt_mesh_hb_pub *pub);
};

/**
 *  @brief Register a callback structure for Heartbeat events.
 *
 *  Registers a callback structure that will be called whenever Heartbeat
 *  events occur
 *
 *  @param _name Name of callback structure.
 */
#define BT_MESH_HB_CB_DEFINE(_name) static const STRUCT_SECTION_ITERABLE(bt_mesh_hb_cb, _name)

/** @brief Get the current Heartbeat publication parameters.
 *
 *  @param get Heartbeat publication parameters return buffer.
 */

void bt_mesh_hb_pub_get(struct bt_mesh_hb_pub *get);

/** @brief Get the current Heartbeat subscription parameters.
 *
 *  @param get Heartbeat subscription parameters return buffer.
 */

void bt_mesh_hb_sub_get(struct bt_mesh_hb_sub *get);

#ifdef __cplusplus
}
#endif
/**
 * @}
 */

#endif /* __INCLUDE_NUTTX_NET_BLUETOOTH_MESH_HEARTBEAT_H */
