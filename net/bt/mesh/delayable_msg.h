/****************************************************************************
 * net/bt/mesh/delayable_msg.h
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

/* Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_BLUETOOTH_MESH_DELAYABLE_MSG_H__
#define ZEPHYR_INCLUDE_BLUETOOTH_MESH_DELAYABLE_MSG_H__

int bt_mesh_delayable_msg_manage(struct bt_mesh_msg_ctx *ctx,
                                 struct net_buf_simple *buf,
                                 uint16_t src_addr,
                                 const struct bt_mesh_send_cb *cb,
                                 void *cb_data);
void bt_mesh_delayable_msg_init(void);
void bt_mesh_delayable_msg_stop(void);

#endif /* ZEPHYR_INCLUDE_BLUETOOTH_MESH_DELAYABLE_MSG_H__ */
