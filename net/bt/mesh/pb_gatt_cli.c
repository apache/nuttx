/****************************************************************************
 * net/bt/mesh/pb_gatt_cli.c
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

/* Copyright (c) 2021 Xiaomi Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/net/bluetooth/compat/kernel.h>
#include <nuttx/net/bluetooth/compat/sys/byteorder.h>

#include <nuttx/mm/net_buf.h>
#include <nuttx/net/bluetooth/bluetooth.h>
#include <nuttx/net/bluetooth/hci.h>
#include <nuttx/net/bluetooth/uuid.h>
#include <nuttx/net/bluetooth/conn.h>
#include <nuttx/net/bluetooth/gatt.h>
#include <nuttx/net/bluetooth/mesh.h>

#include "mesh.h"
#include "net.h"
#include "rpl.h"
#include "transport.h"
#include "prov.h"
#include "pb_gatt.h"
#include "beacon.h"
#include "foundation.h"
#include "access.h"
#include "proxy.h"
#include "gatt_cli.h"
#include "proxy_msg.h"

#define LOG_LEVEL CONFIG_BT_MESH_PROV_LOG_LEVEL
#include <nuttx/net/bluetooth/compat/logging/log.h>
LOG_MODULE_REGISTER(bt_mesh_pb_gatt_client);

static struct
{
  bool target_set;
  uint8_t target_uuid[16];
  struct bt_mesh_proxy_role *srv;
} server;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void pb_gatt_msg_recv(struct bt_mesh_proxy_role *role)
{
  switch (role->msg_type)
    {
      case BT_MESH_PROXY_PROV:
        LOG_DBG("Mesh Provisioning PDU");
        bt_mesh_pb_gatt_recv(role->conn, &role->buf);
        break;
      default:
        LOG_WRN("Unhandled Message Type 0x%02x", role->msg_type);
        break;
    }
}

static void pb_gatt_connected(struct bt_conn *conn, void *user_data)
{
  server.srv = bt_mesh_proxy_role_setup(conn, bt_mesh_gatt_send,
          pb_gatt_msg_recv);

  server.target_set = false;

  bt_mesh_pb_gatt_cli_start(conn);
}

static void pb_gatt_link_open(struct bt_conn *conn)
{
  bt_mesh_pb_gatt_cli_open(conn);
}

static void pb_gatt_disconnected(struct bt_conn *conn)
{
  bt_mesh_pb_gatt_close(conn);

  bt_mesh_proxy_role_cleanup(server.srv);

  server.srv = NULL;
}

static const struct bt_mesh_gatt_cli pbgatt =
{
        .srv_uuid = BT_UUID_INIT_16(BT_UUID_MESH_PROV_VAL),
        .data_in_uuid = BT_UUID_INIT_16(BT_UUID_MESH_PROV_DATA_IN_VAL),
        .data_out_uuid = BT_UUID_INIT_16(BT_UUID_MESH_PROV_DATA_OUT_VAL),
        .data_out_cccd_uuid = BT_UUID_INIT_16(BT_UUID_GATT_CCC_VAL),

        .connected = pb_gatt_connected,
        .link_open = pb_gatt_link_open,
          .disconnected = pb_gatt_disconnected
};

int bt_mesh_pb_gatt_cli_setup(const uint8_t uuid[16])
{
  if (server.srv)
    {
      return -EBUSY;
    }

  memcpy(server.target_uuid, uuid, 16);
  server.target_set = true;
  return 0;
}

void bt_mesh_pb_gatt_cli_adv_recv(const struct bt_le_scan_recv_info *info,
                                  struct net_buf_simple *buf)
{
  uint8_t *uuid;
  bt_mesh_prov_oob_info_t oob_info;

  if (server.srv)
    {
      return;
    }

  if (buf->len != 18)
    {
      return;
    }

  uuid = net_buf_simple_pull_mem(buf, 16);

  if (server.target_set && !memcmp(server.target_uuid, uuid, 16))
    {
      (void)bt_mesh_gatt_cli_connect(info->addr, &pbgatt, NULL);
      return;
    }

  if (!bt_mesh_prov->unprovisioned_beacon_gatt)
    {
      return;
    }

  oob_info = (bt_mesh_prov_oob_info_t)net_buf_simple_pull_le16(buf);

  bt_mesh_prov->unprovisioned_beacon_gatt(uuid, oob_info);
}
