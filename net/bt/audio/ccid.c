/****************************************************************************
 * net/bt/audio/ccid.c
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

/*  Bluetooth Audio Content Control */

/* Copyright (c) 2020 Bose Corporation
 * Copyright (c) 2021-2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <errno.h>
#include <stddef.h>
#include <stdint.h>
#include <sys/types.h>

#include <nuttx/net/bluetooth/att.h>
#include <nuttx/net/bluetooth/audio/ccid.h>
#include <nuttx/net/bluetooth/gatt.h>
#include <nuttx/net/bluetooth/uuid.h>
#include <assert.h>
#include <nuttx/net/bluetooth/compat/toolchain.h>

struct ccid_search_param
{
  const struct bt_gatt_attr *attr;
  uint8_t ccid;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static uint8_t ccid_attr_cb(const struct bt_gatt_attr *attr,
                            uint16_t handle, void *user_data)
{
  struct ccid_search_param *search_param = user_data;

  ARG_UNUSED(handle);

  if (attr->read != NULL)
    {
      uint8_t ccid = 0U;
      ssize_t res;

      res = attr->read(NULL, attr, &ccid, sizeof(ccid), 0);

      if (res == sizeof(ccid) && search_param->ccid == ccid)
        {
          search_param->attr = attr;

          return BT_GATT_ITER_STOP;
        }
    }

  return BT_GATT_ITER_CONTINUE;
}

const struct bt_gatt_attr *bt_ccid_find_attr(uint8_t ccid)
{
  struct ccid_search_param search_param =
    {
          .attr = NULL,
          .ccid = ccid,
    };

  bt_gatt_foreach_attr_type(BT_ATT_FIRST_ATTRIBUTE_HANDLE,
                            BT_ATT_LAST_ATTRIBUTE_HANDLE,
                            BT_UUID_CCID, NULL, 0U, ccid_attr_cb,
                            &search_param);

  return search_param.attr;
}

int bt_ccid_alloc_value(void)
{
  static uint8_t next_ccid_value;
  const uint8_t tmp = next_ccid_value;

  /* Verify that the CCID is unused and increment until we reach an unused
   * value or until we reach our starting point This is not a perfect check
   * as there may be services that are not registered that have allocated a
   * CCID. Implementing a perfect solution would require a alloc and free API
   * where we need to keep track of all allocated CCIDs, which requires
   * additional memory for something that's very unlikely to be an issue.
   */

  while (bt_ccid_find_attr(next_ccid_value) != NULL)
    {
      next_ccid_value++;
      if (tmp == next_ccid_value)
        {
          return -ENOMEM;
        }
    }

  return next_ccid_value++;
}
