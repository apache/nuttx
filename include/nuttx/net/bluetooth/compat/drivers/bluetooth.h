/****************************************************************************
 * include/nuttx/net/bluetooth/compat/drivers/bluetooth.h
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
#ifndef __INCLUDE_NUTTX_NET_BLUETOOTH_COMPAT_DRIVERS_BLUETOOTH_H
#define __INCLUDE_NUTTX_NET_BLUETOOTH_COMPAT_DRIVERS_BLUETOOTH_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <nuttx/net/bluetooth/compat/sys/util.h>

#ifdef __cplusplus
extern "C"
{
#endif

/* BT HCI driver interface - NuttX native */

#ifndef _BT_HCI_EVT_HDR_DEFINED
#define _BT_HCI_EVT_HDR_DEFINED
struct bt_hci_evt_hdr
{
  uint8_t evt;
  uint8_t len;
}
__attribute__((packed));
#endif

#ifndef _BT_HCI_ACL_HDR_DEFINED
#define _BT_HCI_ACL_HDR_DEFINED
struct bt_hci_acl_hdr
{
  uint16_t handle;
  uint16_t len;
}
__attribute__((packed));
#endif

#ifndef _BT_HCI_ISO_HDR_DEFINED
#define _BT_HCI_ISO_HDR_DEFINED
struct bt_hci_iso_hdr
{
  uint16_t handle;
  uint16_t len;
}
__attribute__((packed));
#endif

/* HCI driver vtable */

struct bt_hci_driver_api
{
  int (*open)(void);
  int (*send)(const uint8_t *data, size_t len);
  int (*close)(void);
};

/* HCI driver quirks (bitmask) */
#define BT_HCI_QUIRK_NO_RESET BIT(0)
#define BT_HCI_QUIRK_NO_AUTO_DLE BIT(1)
#define BT_HCI_QUIRK_NO_FLOW_CONTROL BIT(2)

/* Forward declarations */

struct device;
struct net_buf;
#include <nuttx/net/bluetooth/addr.h>

/* HCI driver data */

struct bt_hci_driver_data
{
  bt_addr_t public_addr;
};

/* HCI driver API functions */

typedef int (*bt_hci_recv_t)(const struct device *dev, struct net_buf *buf);

int bt_hci_send(const struct device *dev, struct net_buf *buf);
int bt_hci_open(const struct device *dev, bt_hci_recv_t recv);
int bt_hci_close(const struct device *dev);
bool bt_hci_can_close(const struct device *dev);
void bt_hci_set_public_addr(const struct device *dev, const bt_addr_t *addr);
const bt_addr_t *bt_hci_get_public_addr(const struct device *dev);

#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_NET_BLUETOOTH_COMPAT_DRIVERS_BLUETOOTH_H */
