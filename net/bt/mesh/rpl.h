/****************************************************************************
 * net/bt/mesh/rpl.h
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
 * Copyright (c) 2020 Lingao Meng
 *
 * SPDX-License-Identifier: Apache-2.0
 */

struct bt_mesh_rpl
{
  uint64_t src : 15;
  uint64_t old_iv : 1;
  uint64_t seq : 24;

  /** Sequence authentication value for the previous segmented
   *  message received from this address.
   *
   *  This value is used to manage the parallel RPL of the SeqAuth values in
   *  transport.
   */

  uint64_t seg : 24;
};

typedef void (*bt_mesh_rpl_func_t)(struct bt_mesh_rpl *rpl, void *user_data);

void bt_mesh_rpl_reset(void);
bool bt_mesh_rpl_check(struct bt_mesh_net_rx *rx,
                       struct bt_mesh_rpl **match, bool bridge);
void bt_mesh_rpl_clear(void);
void bt_mesh_rpl_update(struct bt_mesh_rpl *rpl, struct bt_mesh_net_rx *rx);
void bt_mesh_rpl_pending_store_all_nodes(void);
