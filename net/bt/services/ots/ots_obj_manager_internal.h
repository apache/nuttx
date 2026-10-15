/****************************************************************************
 * net/bt/services/ots/ots_obj_manager_internal.h
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

#ifndef BT_GATT_OTS_OBJ_MANAGER_H_
#define BT_GATT_OTS_OBJ_MANAGER_H_

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <sys/types.h>

int bt_gatt_ots_obj_manager_first_obj_get(void *obj_manager,
                                          struct bt_gatt_ots_object **obj);

int bt_gatt_ots_obj_manager_last_obj_get(void *obj_manager,
                                         struct bt_gatt_ots_object **obj);

int bt_gatt_ots_obj_manager_prev_obj_get(void *obj_manager,
  const struct bt_gatt_ots_object *cur_obj,
  struct bt_gatt_ots_object **prev_obj);

int bt_gatt_ots_obj_manager_next_obj_get(void *obj_manager,
  const struct bt_gatt_ots_object *cur_obj,
  struct bt_gatt_ots_object **next_obj);

int bt_gatt_ots_obj_manager_obj_get(void *obj_manager, uint64_t id,
                                    struct bt_gatt_ots_object **obj);

int bt_gatt_ots_obj_manager_obj_add(void *obj_manager,
                                    struct bt_gatt_ots_object **obj);

int bt_gatt_ots_obj_manager_obj_delete(struct bt_gatt_ots_object *obj);

bool bt_gatt_ots_obj_manager_obj_contains(void *obj_manager,
                                          struct bt_gatt_ots_object *obj);

void *bt_gatt_ots_obj_manager_assign(void);

#ifdef __cplusplus
}
#endif

#endif /* BT_GATT_OTS_OBJ_MANAGER_H_ */
