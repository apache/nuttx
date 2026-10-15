/****************************************************************************
 * net/bt/mesh/lpn.h
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

int bt_mesh_lpn_friend_update(struct bt_mesh_net_rx *rx,
                              struct net_buf_simple *buf);
int bt_mesh_lpn_friend_offer(struct bt_mesh_net_rx *rx,
                             struct net_buf_simple *buf);
int bt_mesh_lpn_friend_clear_cfm(struct bt_mesh_net_rx *rx,
                                 struct net_buf_simple *buf);
int bt_mesh_lpn_friend_sub_cfm(struct bt_mesh_net_rx *rx,
                               struct net_buf_simple *buf);

static inline bool bt_mesh_lpn_established(void)
{
#if defined(CONFIG_BT_MESH_LOW_POWER)
  return bt_mesh.lpn.established;
#else
  return false;
#endif
}

static inline bool bt_mesh_lpn_match(uint16_t addr)
{
#if defined(CONFIG_BT_MESH_LOW_POWER)
  if (bt_mesh_lpn_established())
    {
      return (addr == bt_mesh.lpn.frnd);
    }
#endif
  return false;
}

static inline bool bt_mesh_lpn_waiting_update(void)
{
#if defined(CONFIG_BT_MESH_LOW_POWER)
  return (bt_mesh.lpn.state == BT_MESH_LPN_WAIT_UPDATE);
#else
  return false;
#endif
}

static inline bool bt_mesh_lpn_clearing(void)
{
#if defined(CONFIG_BT_MESH_LOW_POWER)
  return (bt_mesh.lpn.state == BT_MESH_LPN_CLEAR);
#else
  return false;
#endif
}

void bt_mesh_lpn_msg_received(struct bt_mesh_net_rx *rx);

void bt_mesh_lpn_group_add(uint16_t group);
void bt_mesh_lpn_group_del(const uint16_t *groups, size_t group_count);

void bt_mesh_lpn_disable(bool force);
void bt_mesh_lpn_friendship_end(void);

int bt_mesh_lpn_init(void);
