/****************************************************************************
 * net/bt/mesh/pb_gatt_srv.h
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

/* Copyright (c) 2017 Intel Corporation Copyright (c) 2021 Lingao Meng
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_SUBSYS_BLUETOOTH_MESH_PB_GATT_SRV_H_
#define ZEPHYR_SUBSYS_BLUETOOTH_MESH_PB_GATT_SRV_H_

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/net/bluetooth/gatt.h>

int bt_mesh_pb_gatt_srv_enable(void);
int bt_mesh_pb_gatt_srv_disable(void);

int bt_mesh_pb_gatt_srv_adv_start(void);

#endif /* ZEPHYR_SUBSYS_BLUETOOTH_MESH_PB_GATT_SRV_H_ */
