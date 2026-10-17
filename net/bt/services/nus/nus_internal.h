/****************************************************************************
 * net/bt/services/nus/nus_internal.h
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

/* Copyright (c) 2024 Croxel, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_BLUETOOTH_SERVICES_NUS_INTERNAL_H_
#define ZEPHYR_BLUETOOTH_SERVICES_NUS_INTERNAL_H_

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/net/bluetooth/services/nus.h>
#include <nuttx/net/bluetooth/services/nus/inst.h>

struct bt_nus_inst *bt_nus_inst_default(void);
struct bt_nus_inst *
bt_nus_inst_get_from_attr(const struct bt_gatt_attr *attr);

#endif /* ZEPHYR_BLUETOOTH_SERVICES_NUS_INTERNAL_H_ */
