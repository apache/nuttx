/****************************************************************************
 * net/bt/mesh/statistic.h
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

#ifndef ZEPHYR_SUBSYS_BLUETOOTH_MESH_STATISTIC_H_
#define ZEPHYR_SUBSYS_BLUETOOTH_MESH_STATISTIC_H_

void bt_mesh_stat_planned_count(struct bt_mesh_adv_ctx *ctx);
void bt_mesh_stat_succeeded_count(struct bt_mesh_adv_ctx *ctx);
void bt_mesh_stat_rx(enum bt_mesh_net_if net_if);

#if defined(CONFIG_BT_MESH_LOW_POWER)
void bt_mesh_stat_lpn_timing_update_poll_tx(void);
void bt_mesh_stat_lpn_timing_update_scan_start(void);
void bt_mesh_stat_lpn_timing_update_response_rx(void);
void bt_mesh_stat_lpn_timing_update_win_expired(void);
#endif

#endif /* ZEPHYR_SUBSYS_BLUETOOTH_MESH_STATISTIC_H_ */
