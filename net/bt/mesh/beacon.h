/****************************************************************************
 * net/bt/mesh/beacon.h
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

void bt_mesh_beacon_enable(void);
void bt_mesh_beacon_disable(void);
void bt_mesh_beacon_cache_clear(struct bt_mesh_subnet *sub);
void bt_mesh_beacon_ivu_initiator(bool enable);

void bt_mesh_beacon_recv(struct net_buf_simple *buf);

int bt_mesh_beacon_create(struct bt_mesh_subnet *sub,
                          struct net_buf_simple *buf, bool priv);

void bt_mesh_beacon_init(void);
void bt_mesh_beacon_update(struct bt_mesh_subnet *sub);
