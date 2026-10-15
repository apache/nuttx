/****************************************************************************
 * net/bt/mesh/rpr.h
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

/* Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define RPR_OP_EXTENDED_SCAN_REPORT BT_MESH_MODEL_OP_2(0x80, 0x57)
#define RPR_OP_EXTENDED_SCAN_START BT_MESH_MODEL_OP_2(0x80, 0x56)
#define RPR_OP_LINK_CLOSE BT_MESH_MODEL_OP_2(0x80, 0x5A)
#define RPR_OP_LINK_GET BT_MESH_MODEL_OP_2(0x80, 0x58)
#define RPR_OP_LINK_OPEN BT_MESH_MODEL_OP_2(0x80, 0x59)
#define RPR_OP_LINK_REPORT BT_MESH_MODEL_OP_2(0x80, 0x5C)
#define RPR_OP_LINK_STATUS BT_MESH_MODEL_OP_2(0x80, 0x5B)
#define RPR_OP_PDU_OUTBOUND_REPORT BT_MESH_MODEL_OP_2(0x80, 0x5E)
#define RPR_OP_PDU_REPORT BT_MESH_MODEL_OP_2(0x80, 0x5F)
#define RPR_OP_PDU_SEND BT_MESH_MODEL_OP_2(0x80, 0x5D)
#define RPR_OP_SCAN_CAPS_GET BT_MESH_MODEL_OP_2(0x80, 0x4F)
#define RPR_OP_SCAN_CAPS_STATUS BT_MESH_MODEL_OP_2(0x80, 0x50)
#define RPR_OP_SCAN_GET BT_MESH_MODEL_OP_2(0x80, 0x51)
#define RPR_OP_SCAN_REPORT BT_MESH_MODEL_OP_2(0x80, 0x55)
#define RPR_OP_SCAN_START BT_MESH_MODEL_OP_2(0x80, 0x52)
#define RPR_OP_SCAN_STATUS BT_MESH_MODEL_OP_2(0x80, 0x54)
#define RPR_OP_SCAN_STOP BT_MESH_MODEL_OP_2(0x80, 0x53)

#define RPR_NODE(ctx) {.addr = (ctx)->addr, .net_idx = (ctx)->net_idx, .ttl = BT_MESH_TTL_DEFAULT}

static inline bool rpr_node_equal(const struct bt_mesh_rpr_node *a,
                                  const struct bt_mesh_rpr_node *b)
{
  return (a->addr == b->addr) && (a->net_idx == b->net_idx);
}

enum bt_mesh_rpr_node_refresh bt_mesh_node_refresh_get(void);
