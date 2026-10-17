/****************************************************************************
 * include/nuttx/net/bluetooth/compat/drivers/bluetooth/hci_lockstep.h
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

/** @file
 *  @brief Bluetooth HCI lockstep command helper.
 */

/* Copyright (c) 2026 Silicon Laboratories Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __INCLUDE_NUTTX_NET_BLUETOOTH_COMPAT_DRIVERS_BLUETOOTH_HCI_LOCKSTEP_H
#define __INCLUDE_NUTTX_NET_BLUETOOTH_COMPAT_DRIVERS_BLUETOOTH_HCI_LOCKSTEP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <nuttx/spinlock.h>

#include <nuttx/net/bluetooth/hci_pkt.h>
#include <nuttx/net/bluetooth/compat/device.h>
#include <nuttx/net/bluetooth/compat/kernel.h>
#include <nuttx/mm/net_buf.h>

#ifdef __cplusplus
extern "C"
{
#endif

typedef int (*bt_hci_lockstep_send_t)(
  const struct device *dev, const uint8_t *pkt, size_t len);

struct bt_hci_lockstep
{
  const struct device *dev;
  bt_hci_lockstep_send_t send;
  sem_t changed;
  spinlock_t lock;
  struct net_buf_simple *rsp;
  uint16_t opcode;
  uint8_t state;
  uint8_t status;
  clock_t timeout;
};

void bt_hci_lockstep_init(
  struct bt_hci_lockstep *ls, const struct device *dev,
  bt_hci_lockstep_send_t send);
void bt_hci_lockstep_reset(struct bt_hci_lockstep *ls);
bool bt_hci_lockstep_feed(struct bt_hci_lockstep *ls,
                          const uint8_t *pkt, size_t len);
int bt_hci_lockstep_cmd_send_sync(
  struct bt_hci_lockstep *ls, uint16_t opcode, struct net_buf_simple *cmd,
  struct net_buf_simple *rsp);

#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_NET_BLUETOOTH_COMPAT_DRIVERS_BLUETOOTH_HCI_LOCKSTEP_H */
