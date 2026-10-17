/****************************************************************************
 * net/bt/services/nus/nus.c
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

#include <nuttx/net/bluetooth/bluetooth.h>
#include <nuttx/net/bluetooth/gatt.h>
#include <nuttx/net/bluetooth/services/nus.h>
#include <nuttx/net/bluetooth/compat/sys/util.h>
#include "nus_internal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

ssize_t nus_bt_chr_write(
  struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf,
  uint16_t len, uint16_t offset, uint8_t flags)
{
  struct bt_nus_cb *listener = NULL;
  struct bt_nus_inst *instance = NULL;

  instance = bt_nus_inst_get_from_attr(attr);
  DEBUGASSERT(instance);

  list_for_every_container(instance->cbs, listener, _node)
  {
    if (listener->received)
      {
        listener->received(conn, buf, len, listener->ctx);
      }
  }

  return len;
}

void nus_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
  struct bt_nus_cb *listener = NULL;
  struct bt_nus_inst *instance = NULL;

  instance = bt_nus_inst_get_from_attr(attr);
  DEBUGASSERT(instance);

  list_for_every_container(instance->cbs, listener, _node)
  {
    if (listener->notif_enabled)
      {
        listener->notif_enabled((value == 1), listener->ctx);
      }
  }
}

int bt_nus_inst_cb_register(struct bt_nus_inst *instance,
                            struct bt_nus_cb *cb, void *ctx)
{
  if (!cb)
    {
      return -EINVAL;
    }

  if (!instance)
    {
      if (IS_ENABLED(CONFIG_BT_ZEPHYR_NUS_DEFAULT_INSTANCE))
        {
          instance = bt_nus_inst_default();
        }
      else
        {
          return -ENOTSUP;
        }
    }

  cb->ctx = ctx;
  list_add_tail(instance->cbs, &cb->_node);

  return 0;
}

int bt_nus_inst_send(
  struct bt_conn *conn, struct bt_nus_inst *instance, const void *data,
  uint16_t len)
{
  if (!data || !len)
    {
      return -EINVAL;
    }

  if (!instance)
    {
      if (IS_ENABLED(CONFIG_BT_ZEPHYR_NUS_DEFAULT_INSTANCE))
        {
          instance = bt_nus_inst_default();
        }
      else
        {
          return -ENOTSUP;
        }
    }

  return bt_gatt_notify(conn, &instance->svc->attrs[1], data, len);
}
