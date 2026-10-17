/****************************************************************************
 * net/bt/services/ots/ots_l2cap_internal.h
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

/* Copyright (c) 2020 - 2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef BT_GATT_OTS_L2CAP_H_
#define BT_GATT_OTS_L2CAP_H_

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <sys/types.h>
#include <nuttx/list.h>
#include <sys/types.h>

#include <nuttx/net/bluetooth/l2cap.h>

struct bt_gatt_ots_l2cap_tx
{
  uint8_t *data;
  uint32_t len;
  uint32_t len_sent;
};

struct bt_gatt_ots_l2cap
{
  struct list_node node;
  struct bt_l2cap_le_chan ot_chan;
  struct bt_gatt_ots_l2cap_tx tx;
  void (*tx_done)(struct bt_gatt_ots_l2cap *l2cap_ctx, struct bt_conn *conn);
ssize_t (*rx_done)(struct bt_gatt_ots_l2cap *l2cap_ctx,
                   struct bt_conn *conn,
                   struct net_buf *buf);
  void (*closed)(struct bt_gatt_ots_l2cap *l2cap_ctx, struct bt_conn *conn);
};

bool bt_gatt_ots_l2cap_is_open(struct bt_gatt_ots_l2cap *l2cap_ctx,
                               struct bt_conn *conn);

int bt_gatt_ots_l2cap_send(struct bt_gatt_ots_l2cap *l2cap_ctx,
                           uint8_t *data, uint32_t len);

int bt_gatt_ots_l2cap_register(struct bt_gatt_ots_l2cap *l2cap_ctx);

int bt_gatt_ots_l2cap_unregister(struct bt_gatt_ots_l2cap *l2cap_ctx);

/** @brief Connect OTS L2CAP channel
 *
 *  This function is for the OTS client to make an L2CAP connection to the
 *  OTS server. One of the available registered L2CAP contexts will be used
 *  for the connection.
 *
 * @param[in]  conn       Connection pointer
 * @param[out] l2cap_ctx  The context that was connected
 *
 * @return     0 in case of success or negative value in case of error
 */

int bt_gatt_ots_l2cap_connect(struct bt_conn *conn,
                              struct bt_gatt_ots_l2cap **l2cap_ctx);

int bt_gatt_ots_l2cap_disconnect(struct bt_gatt_ots_l2cap *l2cap_ctx);

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif /* BT_GATT_OTS_L2CAP_H_ */
