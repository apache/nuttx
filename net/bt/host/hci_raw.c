/****************************************************************************
 * net/bt/host/hci_raw.c
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

/* hci_userchan.c - HCI user channel Bluetooth handling */

/* Copyright (c) 2015-2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <nuttx/net/bluetooth/buf.h>
#include <nuttx/net/bluetooth/hci.h>
#include <nuttx/net/bluetooth/hci_raw.h>
#include <nuttx/net/bluetooth/hci_types.h>
#include <nuttx/net/bluetooth/l2cap.h>
#include <nuttx/net/bluetooth/iso.h>
#include <nuttx/net/bluetooth/compat/device.h>
#include <nuttx/net/bluetooth/compat/drivers/bluetooth.h>
#include <nuttx/net/bluetooth/compat/kernel.h>
#include <nuttx/net/bluetooth/compat/logging/log.h>
#include <nuttx/mm/net_buf.h>
#include <nuttx/net/bluetooth/compat/sys/byteorder.h>
#include <nuttx/clock.h>
#include <nuttx/net/bluetooth/compat/toolchain.h>

#include "common/hci_common_internal.h"
#include "hci_raw_internal.h"
#include "monitor.h"

#define LOG_LEVEL CONFIG_BT_HCI_CORE_LOG_LEVEL
LOG_MODULE_REGISTER(bt_hci_raw);

static struct k_fifo *raw_rx;

static bt_buf_rx_freed_cb_t buf_rx_freed_cb;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void hci_rx_buf_destroy(struct net_buf *buf)
{
  net_buf_destroy(buf);

  if (buf_rx_freed_cb)
    {
      /* bt_buf_get_rx is used for all types of RX buffers */

      buf_rx_freed_cb(BT_BUF_EVT | BT_BUF_ACL_IN | BT_BUF_ISO_IN);
    }
}

NET_BUF_POOL_FIXED_DEFINE(hci_rx_pool,
                          BT_BUF_RX_COUNT,
                          BT_BUF_RX_SIZE,
                          0,
                          hci_rx_buf_destroy);
NET_BUF_POOL_FIXED_DEFINE(hci_cmd_pool, BT_BUF_CMD_TX_COUNT,
                          BT_BUF_CMD_SIZE(CONFIG_BT_BUF_CMD_TX_SIZE),
                              0,
                              NULL);
NET_BUF_POOL_FIXED_DEFINE(hci_acl_pool, CONFIG_BT_BUF_ACL_TX_COUNT,
                          BT_BUF_ACL_SIZE(CONFIG_BT_BUF_ACL_TX_SIZE),
                              0,
                              NULL);
#if defined(CONFIG_BT_ISO)
NET_BUF_POOL_FIXED_DEFINE(hci_iso_pool, CONFIG_BT_ISO_TX_BUF_COUNT,
                          BT_ISO_SDU_BUF_SIZE(CONFIG_BT_ISO_TX_MTU),
                              0,
                              NULL);
#endif /* CONFIG_BT_ISO */

#define BT_HCI_DEV NULL
#define BT_HCI_BUS 0
#define BT_HCI_NAME "nuttx_bt_hci"

struct bt_dev_raw bt_dev =
{
        .hci = BT_HCI_DEV,
};

struct net_buf *bt_buf_get_rx(enum bt_buf_type type, clock_t timeout)
{
  struct net_buf *buf;

  switch (type)
    {
      case BT_BUF_EVT:
      case BT_BUF_ACL_IN:
      case BT_BUF_ISO_IN:
        break;
      default:
        LOG_ERR("Invalid rx type: %u", type);
        return NULL;
    }

  buf = net_buf_alloc(&hci_rx_pool, timeout);
  if (!buf)
    {
      return buf;
    }

  net_buf_add_u8(buf, bt_buf_type_to_h4(type));

  return buf;
}

void bt_buf_rx_freed_cb_set(bt_buf_rx_freed_cb_t cb)
{
  buf_rx_freed_cb = cb;
}

struct net_buf *bt_buf_get_tx(enum bt_buf_type type,
                              clock_t timeout, const void *data, size_t size)
{
  struct net_buf_pool *pool;
  struct net_buf *buf;

  switch (type)
    {
      case BT_BUF_CMD:
        pool = &hci_cmd_pool;
        break;
      case BT_BUF_ACL_OUT:
        pool = &hci_acl_pool;
        break;
#if defined(CONFIG_BT_ISO)
      case BT_BUF_ISO_OUT:
        pool = &hci_iso_pool;
        break;
#endif /* CONFIG_BT_ISO */
      default:
        LOG_ERR("Invalid tx type: %u", type);
        return NULL;
    }

  buf = net_buf_alloc(pool, timeout);
  if (!buf)
    {
      return buf;
    }

  net_buf_add_u8(buf, bt_buf_type_to_h4(type));

  if (data && size)
    {
      if (net_buf_tailroom(buf) < size)
        {
          net_buf_unref(buf);
          return NULL;
        }

      net_buf_add_mem(buf, data, size);
    }

  return buf;
}

struct net_buf *bt_buf_get_evt(uint8_t evt,
                               bool discardable, clock_t timeout)
{
  ARG_UNUSED(evt);
  ARG_UNUSED(discardable);

  return bt_buf_get_rx(BT_BUF_EVT, timeout);
}

static int bt_recv(const struct device *dev, struct net_buf *buf)
{
  uint8_t type = buf->data[0];

  ARG_UNUSED(dev);

  LOG_DBG("buf %p len %u", buf, buf->len);

  bt_monitor_send(bt_monitor_opcode(type, BT_MONITOR_RX), buf->data + 1,
          buf->len - 1);

  k_fifo_put(raw_rx, buf);

  return 0;
}

int bt_send(struct net_buf *buf)
{
  if (buf->len == 0)
    {
      return BT_HCI_ERR_INVALID_PARAM;
    }

  LOG_DBG("buf %p type %u len %u", buf, buf->data[0], buf->len);

  bt_monitor_send(bt_monitor_opcode(buf->data[0], BT_MONITOR_TX),
          buf->data + 1, buf->len - 1);

  return bt_hci_send(bt_dev.hci, buf);
}

int bt_enable_raw(struct k_fifo *rx_queue)
{
  int err;

  LOG_DBG("");

  raw_rx = rx_queue;

  if (!device_is_ready(bt_dev.hci))
    {
      LOG_ERR("HCI driver is not ready");
      return -ENODEV;
    }

  bt_monitor_new_index(BT_MONITOR_TYPE_PRIMARY, BT_HCI_BUS, BT_ADDR_ANY,
          BT_HCI_NAME);

  err = bt_hci_open(bt_dev.hci, bt_recv);
  if (err)
    {
      LOG_ERR("HCI driver open failed (%d)", err);
      return err;
    }

  LOG_INF("Lower HCI transport: %s", BT_HCI_NAME);
  LOG_INF("Bluetooth enabled in RAW mode");

  return 0;
}
