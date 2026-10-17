/****************************************************************************
 * net/bt/mesh/proxy_msg.h
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

/*  Bluetooth Mesh */

/* Copyright (c) 2017 Intel Corporation
 * Copyright (c) 2021 Lingao Meng
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_SUBSYS_BLUETOOTH_MESH_PROXY_MSG_H_
#define ZEPHYR_SUBSYS_BLUETOOTH_MESH_PROXY_MSG_H_

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/net/bluetooth/gatt.h>

#define PDU_TYPE(data) (data[0] & BIT_MASK(6))
#define CFG_FILTER_SET 0x00
#define CFG_FILTER_ADD 0x01
#define CFG_FILTER_REMOVE 0x02
#define CFG_FILTER_STATUS 0x03

#define BT_MESH_PROXY_NET_PDU 0x00
#define BT_MESH_PROXY_BEACON 0x01
#define BT_MESH_PROXY_CONFIG 0x02
#define BT_MESH_PROXY_PROV 0x03

#define PDU_HDR(sar, type) (sar << 6 | (type & BIT_MASK(6)))

struct bt_mesh_proxy_role;

typedef int (*proxy_send_cb_t)(struct bt_conn *conn, const void *data,
                               uint16_t len, bt_gatt_complete_func_t end,
                               void *user_data);

typedef void (*proxy_recv_cb_t)(struct bt_mesh_proxy_role *role);

struct bt_mesh_proxy_role
{
  struct bt_conn *conn;
  uint8_t msg_type;

  struct k_fifo pending;
  struct k_work work;

  struct
  {
    proxy_send_cb_t send;
    proxy_recv_cb_t recv;
  } cb;

  struct k_work_delayable sar_timer;
  struct net_buf_simple buf;
};

ssize_t bt_mesh_proxy_msg_recv(struct bt_conn *conn,
                               const void *buf, uint16_t len);
int bt_mesh_proxy_msg_send(
  struct bt_conn *conn, uint8_t type, struct net_buf_simple *msg,
  bt_gatt_complete_func_t end, void *user_data);
int bt_mesh_proxy_relay_send(struct bt_conn *conn, struct bt_mesh_adv *adv);
struct bt_mesh_proxy_role *bt_mesh_proxy_role_setup(
  struct bt_conn *conn, proxy_send_cb_t send, proxy_recv_cb_t recv);
void bt_mesh_proxy_role_cleanup(struct bt_mesh_proxy_role *role);

bool bt_mesh_proxy_has_avail_conn(void);

#endif /* ZEPHYR_SUBSYS_BLUETOOTH_MESH_PROXY_MSG_H_ */
