/****************************************************************************
 * include/nuttx/net/bluetooth/mesh/brg_cfg.h
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

/* Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Header file for Bluetooth Mesh Bridge Configuration common
 * definitions.
 * @ingroup bt_mesh_brg_cfg
 */

#ifndef __INCLUDE_NUTTX_NET_BLUETOOTH_MESH_BRG_CFG_H
#define __INCLUDE_NUTTX_NET_BLUETOOTH_MESH_BRG_CFG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @defgroup bt_mesh_brg_cfg Bridge Configuration common header
 * @ingroup bt_mesh
 * @{
 */

/** Subnet Bridge states */

enum bt_mesh_brg_cfg_state
{
  /** Subnet bridge functionality is disabled. */

  BT_MESH_BRG_CFG_DISABLED,

  /** Subnet bridge state functionality is enabled. */

  BT_MESH_BRG_CFG_ENABLED,
};

/* Bridging from Addr1 to Addr2. */
#define BT_MESH_BRG_CFG_DIR_ONEWAY 1
/* Bidirectional bridging between Addr1 and Addr2. */
#define BT_MESH_BRG_CFG_DIR_TWOWAY 2

/** Bridging Table state entry corresponding to a entry in the Bridging
 *  Table.
 */

struct bt_mesh_brg_cfg_table_entry
{
  /** Allowed directions for the bridged traffic (or bridged traffic not
   *  allowed)
   */

  uint8_t directions;

  /** NetKey Index of the first subnet */

  uint16_t net_idx1;

  /** NetKey Index of the second subnet */

  uint16_t net_idx2;

  /** Address of the node in the first subnet */

  uint16_t addr1;

  /** Address of the node in the second subnet */

  uint16_t addr2;
};

/** Bridging Table Status response */

struct bt_mesh_brg_cfg_table_status
{
  /** Status Code of the requesting message */

  uint8_t status;

  /** Requested Bridging Table entry */

  struct bt_mesh_brg_cfg_table_entry entry;
};

/** Used to filter set of pairs of NetKey Indexes from the Bridging Table */

struct bt_mesh_brg_cfg_filter_netkey
{
  uint16_t filter : 2;     /* Filter applied to the set of pairs of NetKey Indexes */
  uint16_t prohibited : 2; /* Prohibited */
  uint16_t net_idx : 12;   /* NetKey Index used for filtering or ignored */
};

/** Bridged Subnets List response */

struct bt_mesh_brg_cfg_subnets_list
{
  /** Filter applied NetKey Indexes, and NetKey Index used for filtering. */

  struct bt_mesh_brg_cfg_filter_netkey net_idx_filter;

  /** Start offset in units of bridges */

  uint8_t start_idx;

  /** Pointer to allocated buffer for storing filtered of NetKey Indexes */

  struct net_buf_simple *list;
};

/** Bridging Table List response */

struct bt_mesh_brg_cfg_table_list
{
  /** Status Code of the requesting message */

  uint8_t status;

  /** NetKey Index of the first subnet */

  uint16_t net_idx1;

  /** NetKey Index of the second subnet */

  uint16_t net_idx2;

  /** Start offset in units of bridging table state entries */

  uint16_t start_idx;

  /** Pointer to allocated buffer for storing list of bridged addresses and
   *  directions
   */

  struct net_buf_simple *list;
};

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_NET_BLUETOOTH_MESH_BRG_CFG_H */
