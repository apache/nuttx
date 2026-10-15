/****************************************************************************
 * net/bt/mesh/op_agg.h
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

/* Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

struct op_agg_ctx
{
  /** Context is initialized. */

  bool initialized;

  /** NetKey Index of the subnet to send the message on. */

  uint16_t net_idx;

  /** AppKey Index to encrypt the message with. */

  uint16_t app_idx;

  /** Remote element address. */

  uint16_t addr;

  /** Aggregated message buffer. */

  struct net_buf_simple *sdu;
};

int bt_mesh_op_agg_encode_msg(struct net_buf_simple *msg,
                              struct net_buf_simple *buf);
int bt_mesh_op_agg_decode_msg(struct net_buf_simple *msg,
                              struct net_buf_simple *buf);
int bt_mesh_op_agg_cli_send(const struct bt_mesh_model *model,
                            struct net_buf_simple *msg);
int bt_mesh_op_agg_cli_accept(struct bt_mesh_msg_ctx *ctx,
                              struct net_buf_simple *buf);
int bt_mesh_op_agg_srv_send(const struct bt_mesh_model *model,
                            struct net_buf_simple *msg);
int bt_mesh_op_agg_srv_accept(struct bt_mesh_msg_ctx *ctx,
                              struct net_buf_simple *buf);
bool bt_mesh_op_agg_is_op_agg_msg(struct net_buf_simple *buf);
