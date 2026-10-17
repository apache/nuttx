/****************************************************************************
 * net/bt/host/adv.h
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

/* Copyright (c) 2017-2021 Nordic Semiconductor ASA
 * Copyright (c) 2015-2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdbool.h>

#include <nuttx/net/bluetooth/bluetooth.h>

struct bt_le_ext_adv *bt_le_adv_lookup_legacy(void);

void bt_le_adv_delete_legacy(void);
int bt_le_adv_set_enable(struct bt_le_ext_adv *adv, bool enable);

void bt_le_ext_adv_foreach(
  void (*func)(struct bt_le_ext_adv *adv, void *data), void *data);

int bt_le_adv_set_enable_ext(struct bt_le_ext_adv *adv, bool enable,
                             const struct bt_le_ext_adv_start_param *param);
int bt_le_adv_set_enable_legacy(struct bt_le_ext_adv *adv, bool enable);
int bt_le_lim_adv_cancel_timeout(struct bt_le_ext_adv *adv);
void bt_adv_reset_adv_pool(void);
