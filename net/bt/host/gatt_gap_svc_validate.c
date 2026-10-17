/****************************************************************************
 * net/bt/host/gatt_gap_svc_validate.c
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

/* gatt_gap_svc_validate.c - GAP service inside GATT validation functions */

/* Copyright (c) 2025 Koppel Electronic
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <errno.h>

#include <nuttx/net/bluetooth/gatt.h>
#include <nuttx/net/bluetooth/uuid.h>
#include <nuttx/net/bluetooth/compat/logging/log.h>

#define LOG_LEVEL CONFIG_BT_GATT_LOG_LEVEL
LOG_MODULE_DECLARE(bt_gatt);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static uint8_t gap_service_check_cb(const struct bt_gatt_attr *attr,
                                    uint16_t handle,
                                    void *user_data)
{
  size_t *cnt = user_data;
  const struct bt_uuid *svc_uuid = attr->user_data;

  if (svc_uuid != NULL && bt_uuid_cmp(svc_uuid, BT_UUID_GAP) == 0)
    {
      (*cnt)++;
      LOG_DBG("GAP service found at handle: %u", handle);
    }

  return BT_GATT_ITER_CONTINUE;
}

int gatt_gap_svc_validate(void)
{
  size_t gap_svc_count = 0U;

  bt_gatt_foreach_attr_type(BT_ATT_FIRST_ATTRIBUTE_HANDLE,
          BT_ATT_LAST_ATTRIBUTE_HANDLE,
                            BT_UUID_GATT_PRIMARY, NULL, 0,
                            gap_service_check_cb, &gap_svc_count);

  if (gap_svc_count != 1U)
    {
      LOG_DBG("GAP service count invalid: %zu", gap_svc_count);
      return -EINVAL;
    }

  return 0;
}
