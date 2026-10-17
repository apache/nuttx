/****************************************************************************
 * include/nuttx/net/bluetooth/hci_driver_nuttx.h
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

/* SPDX-License-Identifier: Apache-2.0 */
#ifndef ZEPHYR_BT_HCI_DRIVER_NUTTX_H_
#define ZEPHYR_BT_HCI_DRIVER_NUTTX_H_

/* NuttX HCI driver integration for NuttX Bluetooth Host Stack.
 *
 * This provides the bridge between the Zephyr BT host (which expects the
 * Zephyr driver model) and NuttX's hardware drivers.
 *
 * To use with a specific HCI transport: 1. Register an HCI driver via
 * bt_hci_driver_register() 2. Driver calls bt_recv() when it receives data
 * from the controller 3. BT host calls the driver's send() when it has data
 * for the controller
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <nuttx/net/bluetooth/compat/kernel.h>

#ifdef __cplusplus
extern "C"
{
#endif

struct net_buf;
struct bt_driver_s;

/* HCI driver operations */

struct bt_hci_driver_ops
{
  int (*open)(void);
  int (*send)(struct net_buf *buf);
  int (*close)(void);
};

/* Register an HCI driver. Returns 0 on success. */

int bt_hci_driver_register(const struct bt_hci_driver_ops *ops);

/* Register a NuttX Bluetooth lower-half driver for the Zephyr BT host. */

int bt_hci_driver_register_btdev(struct bt_driver_s *driver);

/* Called by the HCI driver when it receives data from the controller */

int bt_recv(struct net_buf *buf);

/* Called by the HCI driver when it receives an ISO packet */

int bt_recv_iso(struct net_buf *buf);

/* Get a buffer for HCI command/event/ACL data */

struct net_buf *bt_buf_get_cmd_complete(clock_t timeout);
struct net_buf *bt_buf_get_evt(uint8_t evt,
                               bool discardable, clock_t timeout);
struct net_buf *bt_buf_get_acl(clock_t timeout);
struct net_buf *bt_buf_get_iso(clock_t timeout);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_BT_HCI_DRIVER_NUTTX_H_ */
