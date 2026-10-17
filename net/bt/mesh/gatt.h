/****************************************************************************
 * net/bt/mesh/gatt.h
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

/* Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define ADV_SLOW_INT                                                                               \
  .interval_min = BT_GAP_ADV_SLOW_INT_MIN, .interval_max = BT_GAP_ADV_SLOW_INT_MAX

#define ADV_FAST_INT                                                                               \
  .interval_min = BT_GAP_ADV_FAST_INT_MIN_2, .interval_max = BT_GAP_ADV_FAST_INT_MAX_2

#define BT_DEVICE_NAME                                                                             \
  (IS_ENABLED(CONFIG_BT_DEVICE_NAME_DYNAMIC) ? (const uint8_t *)bt_get_name()                      \
                                             : (const uint8_t *)CONFIG_BT_DEVICE_NAME)
#define BT_DEVICE_NAME_LEN                                                                         \
  (IS_ENABLED(CONFIG_BT_DEVICE_NAME_DYNAMIC) ? strlen(bt_get_name())                               \
                                             : (sizeof(CONFIG_BT_DEVICE_NAME) - 1))
