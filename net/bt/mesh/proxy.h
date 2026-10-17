/****************************************************************************
 * net/bt/mesh/proxy.h
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

/* Copyright (c) 2017 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#if defined(CONFIG_BT_MESH_DEBUG_USE_ID_ADDR)
#define ADV_OPT_USE_IDENTITY BT_LE_ADV_OPT_USE_IDENTITY
#else
#define ADV_OPT_USE_IDENTITY 0
#endif

#define BT_MESH_ID_TYPE_NET 0x00
#define BT_MESH_ID_TYPE_NODE 0x01
#define BT_MESH_ID_TYPE_PRIV_NET 0x02
#define BT_MESH_ID_TYPE_PRIV_NODE 0x03

int bt_mesh_proxy_gatt_enable(void);
int bt_mesh_proxy_gatt_disable(void);
void bt_mesh_proxy_gatt_disconnect(void);

void bt_mesh_proxy_beacon_send(struct bt_mesh_subnet *sub);

int bt_mesh_proxy_adv_start(void);

void bt_mesh_proxy_identity_start(struct bt_mesh_subnet *sub, bool private);
void bt_mesh_proxy_identity_stop(struct bt_mesh_subnet *sub);

bool bt_mesh_proxy_relay(struct bt_mesh_adv *adv, uint16_t dst);
void bt_mesh_proxy_addr_add(struct net_buf_simple *buf, uint16_t addr);
uint8_t bt_mesh_proxy_srv_connected_cnt(void);
