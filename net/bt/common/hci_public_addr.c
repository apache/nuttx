/****************************************************************************
 * net/bt/common/hci_public_addr.c
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

/* Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/net/bluetooth/addr.h>
#include <nuttx/net/bluetooth/compat/device.h>
#include <nuttx/net/bluetooth/compat/drivers/bluetooth.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* "No address" is BT_ADDR_ANY, as in the setup() op's parameters and, as the
 * address part of BT_ADDR_LE_ANY, in the Host's identity table, and not
 * BT_ADDR_NONE: zero-initialized driver data then needs no flag to say that
 * nothing has been set.
 */

void bt_hci_set_public_addr(const struct device *dev, const bt_addr_t *addr)
{
  struct bt_hci_driver_data *data = dev->data;

  bt_addr_copy(&data->public_addr, addr);
}

const bt_addr_t *bt_hci_get_public_addr(const struct device *dev)
{
  const struct bt_hci_driver_data *data = dev->data;

  return &data->public_addr;
}
