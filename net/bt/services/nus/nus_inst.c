/****************************************************************************
 * net/bt/services/nus/nus_inst.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "nus_internal.h"

#if defined(CONFIG_BT_ZEPHYR_NUS_DEFAULT_INSTANCE)
BT_NUS_INST_DEFINE(nus_def);

/****************************************************************************
 * Public Functions
 ****************************************************************************/

struct bt_nus_inst *bt_nus_inst_default(void)
{
  return &nus_def;
}
#else
struct bt_nus_inst *bt_nus_inst_default(void)
{
  return NULL;
}
#endif

struct bt_nus_inst *
bt_nus_inst_get_from_attr(const struct bt_gatt_attr *attr)
{
  STRUCT_SECTION_FOREACH(bt_nus_inst, instance)
  {
    for (size_t i = 0; i < instance->svc->attr_count; i++)
      {
        if (attr == &instance->svc->attrs[i])
          {
            return instance;
          }
      }
  }

  return NULL;
}
