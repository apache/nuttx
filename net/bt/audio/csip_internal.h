/****************************************************************************
 * net/bt/audio/csip_internal.h
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
 * @brief Internal APIs for Bluetooth CSIP
 *
 * Copyright (c) 2021-2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

#include <nuttx/net/bluetooth/audio/csip.h>
#include <nuttx/net/bluetooth/conn.h>
#include <nuttx/net/bluetooth/gatt.h>

#define BT_CSIP_SIRK_TYPE_ENCRYPTED 0x00U
#define BT_CSIP_SIRK_TYPE_PLAIN 0x01U

#define BT_CSIP_RELEASE_VALUE 0x01U
#define BT_CSIP_LOCK_VALUE 0x02U

struct bt_csip_sirk
{
  uint8_t type;

  /* SIRK stored in little endian byte order. */

  uint8_t value[BT_CSIP_SIRK_SIZE];
} __packed;

struct bt_csip_set_coordinator_svc_inst *
bt_csip_set_coordinator_lookup_instance_by_index(const struct bt_conn *conn,
                                                 uint8_t idx);

struct bt_csip_set_coordinator_svc_inst
{
  uint8_t set_lock;

  uint16_t start_handle;
  uint16_t end_handle;
  uint16_t sirk_handle;
  uint16_t set_size_handle;
  uint16_t set_lock_handle;
  uint16_t rank_handle;

  uint8_t idx;
  struct bt_gatt_subscribe_params sirk_sub_params;
  struct bt_gatt_discover_params sirk_sub_disc_params;
  struct bt_gatt_subscribe_params size_sub_params;
  struct bt_gatt_discover_params size_sub_disc_params;
  struct bt_gatt_subscribe_params lock_sub_params;
  struct bt_gatt_discover_params lock_sub_disc_params;

  struct bt_conn *conn;
  struct bt_csip_set_coordinator_set_info *set_info;
};

struct bt_csip_set_coordinator_csis_inst *
bt_csip_set_coordinator_csis_inst_by_handle(struct bt_conn *conn,
                                            uint16_t start_handle);
