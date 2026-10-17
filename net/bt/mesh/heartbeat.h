/****************************************************************************
 * net/bt/mesh/heartbeat.h
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

static inline uint16_t bt_mesh_hb_pwr2(uint8_t val)
{
  if (!val)
    {
      return 0x0000;
    }
  else if (val == 0xff)
    {
      return 0xffff;
    }
  else
    {
      return (1 << (val - 1));
    }
}

static inline uint8_t bt_mesh_hb_log(uint32_t val)
{
  if (!val)
    {
      return 0x00;
    }
  else
    {
      return 32 - __builtin_clz(val);
    }
}

void bt_mesh_hb_init(void);
void bt_mesh_hb_start(void);
void bt_mesh_hb_suspend(void);
void bt_mesh_hb_resume(void);

int bt_mesh_hb_recv(struct bt_mesh_net_rx *rx, struct net_buf_simple *buf);
void bt_mesh_hb_feature_changed(uint16_t features);

uint8_t bt_mesh_hb_pub_set(struct bt_mesh_hb_pub *hb_pub);
uint8_t bt_mesh_hb_sub_set(uint16_t src, uint16_t dst, uint32_t period);
void bt_mesh_hb_sub_reset_count(void);
void bt_mesh_hb_pub_pending_store(void);
