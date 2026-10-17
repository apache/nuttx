/****************************************************************************
 * net/bt/mesh/gatt_cli.h
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

struct bt_mesh_gatt_cli
{
  struct bt_uuid_16 srv_uuid;
  struct bt_uuid_16 data_in_uuid;
  struct bt_uuid_16 data_out_uuid;
  struct bt_uuid_16 data_out_cccd_uuid;

  void (*connected)(struct bt_conn *conn, void *user_data);
  void (*link_open)(struct bt_conn *conn);
  void (*disconnected)(struct bt_conn *conn);
};

int bt_mesh_gatt_cli_connect(const bt_addr_le_t *addr,
                             const struct bt_mesh_gatt_cli *gatt,
                             void *user_data);

int bt_mesh_gatt_send(struct bt_conn *conn, const void *data, uint16_t len,
                      bt_gatt_complete_func_t end, void *user_data);

void bt_mesh_gatt_client_init(void);

void bt_mesh_gatt_client_deinit(void);
