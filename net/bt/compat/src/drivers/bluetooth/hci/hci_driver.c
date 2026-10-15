/****************************************************************************
 * net/bt/compat/src/drivers/bluetooth/hci/hci_driver.c
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

/* NuttX HCI driver integration.
 * This provides the bridge between Zephyr BT host and NuttX hardware. For
 * now this is a skeleton - actual HCI transport drivers (UART H4, etc.) need
 * to be implemented separately.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <string.h>
#include <errno.h>
#include <nuttx/net/bluetooth/compat/kernel.h>
#include <nuttx/mm/net_buf.h>
#include <nuttx/net/bluetooth/buf.h>
#include <nuttx/net/bluetooth/hci_driver_nuttx.h>
#include <nuttx/net/bluetooth/compat/drivers/bluetooth.h>
#include <nuttx/wireless/bluetooth/bt_driver.h>
#include <syslog.h>

static const struct bt_hci_driver_ops *g_hci_ops;
static bt_hci_recv_t g_hci_recv_cb;
static FAR struct bt_driver_s *g_btdev_driver;

static int btdev_hci_open(void);
static int btdev_hci_send(FAR struct net_buf *buf);
static int btdev_hci_close(void);

static const struct bt_hci_driver_ops g_btdev_hci_ops =
{
  .open = btdev_hci_open,
  .send = btdev_hci_send,
  .close = btdev_hci_close,
};

/* Buffer pools for HCI data */

NET_BUF_POOL_FIXED_DEFINE(hci_cmd_pool,
                          CONFIG_BT_BUF_CMD_TX_COUNT,
                          BT_BUF_CMD_TX_SIZE,
                          0,
                          NULL);
NET_BUF_POOL_FIXED_DEFINE(hci_acl_tx_pool, CONFIG_BT_BUF_ACL_TX_COUNT,
                          BT_BUF_ACL_SIZE(CONFIG_BT_BUF_ACL_TX_SIZE),
                          0,
                          NULL);
NET_BUF_POOL_FIXED_DEFINE(hci_acl_rx_pool,
                          BT_BUF_ACL_RX_COUNT,
                          BT_BUF_ACL_RX_SIZE,
                          0,
                          NULL);

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int bt_hci_driver_register(const struct bt_hci_driver_ops *ops)
{
  g_hci_ops = ops;
  return 0;
}

static int btdev_hci_open(void)
{
  if (g_btdev_driver == NULL || g_btdev_driver->open == NULL)
    {
      return 0;
    }

  return g_btdev_driver->open(g_btdev_driver);
}

static int btdev_hci_send(FAR struct net_buf *buf)
{
  FAR struct bt_driver_s *driver = g_btdev_driver;
  enum bt_buf_type_e type;
  int ret;

  if (driver == NULL || driver->send == NULL)
    {
      return -ENODEV;
    }

  if (buf == NULL || buf->len < 1)
    {
      return -EINVAL;
    }

  switch (buf->data[0])
    {
      case BT_HCI_H4_CMD:
        type = BT_CMD;
        break;
      case BT_HCI_H4_ACL:
        type = BT_ACL_OUT;
        break;
      case BT_HCI_H4_ISO:
        type = BT_ISO_OUT;
        break;
      default:
        return -EINVAL;
    }

  ret = driver->send(driver, type, buf->data + 1, buf->len - 1);
  if (ret < 0)
    {
      return ret;
    }

  net_buf_unref(buf);
  return 0;
}

static int btdev_hci_close(void)
{
  if (g_btdev_driver != NULL && g_btdev_driver->close != NULL)
    {
      g_btdev_driver->close(g_btdev_driver);
    }

  return 0;
}

static FAR struct net_buf *btdev_hci_rx_alloc(enum bt_buf_type_e type,
    FAR const uint8_t *data,
    size_t len)
{
  FAR struct net_buf *buf;

  switch (type)
    {
      case BT_EVT:
        if (len < sizeof(struct bt_hci_evt_hdr))
          {
            return NULL;
          }

        buf = bt_buf_get_evt(data[0], false, 0);
        break;
      case BT_ACL_IN:
        buf = bt_buf_get_rx(BT_BUF_ACL_IN, 0);
        break;
      case BT_ISO_IN:
#ifdef CONFIG_BT_ISO
        buf = bt_buf_get_rx(BT_BUF_ISO_IN, 0);
        break;
#else
        return NULL;
#endif
      default:
        return NULL;
    }

  if (buf == NULL)
    {
      return NULL;
    }

  if (net_buf_tailroom(buf) < len)
    {
      net_buf_unref(buf);
      return NULL;
    }

  net_buf_add_mem(buf, data, len);
  return buf;
}

static int btdev_hci_receive(FAR struct bt_driver_s *driver,
                             enum bt_buf_type_e type,
                             FAR void *data,
                             size_t len)
{
  FAR struct net_buf *buf;
  int ret;

  if (data == NULL || len == 0)
    {
      return -EINVAL;
    }

  buf = btdev_hci_rx_alloc(type, data, len);
  if (buf == NULL)
    {
      return -ENOBUFS;
    }

  ret = bt_recv(buf);
  if (ret < 0)
    {
      net_buf_unref(buf);
    }

  return ret;
}

int bt_hci_driver_register_btdev(FAR struct bt_driver_s *driver)
{
  if (driver == NULL)
    {
      return -EINVAL;
    }

  if (g_btdev_driver != NULL)
    {
      return -EBUSY;
    }

  driver->receive = btdev_hci_receive;
  g_btdev_driver = driver;

  return bt_hci_driver_register(&g_btdev_hci_ops);
}

struct net_buf *bt_buf_get_cmd_complete(clock_t timeout)
{
  return net_buf_alloc(&hci_cmd_pool, timeout);
}

struct net_buf *bt_buf_get_acl(clock_t timeout)
{
  return net_buf_alloc(&hci_acl_rx_pool, timeout);
}

struct net_buf *bt_buf_get_iso(clock_t timeout)
{
  (void)timeout;
  return NULL;
}

int bt_recv(struct net_buf *buf)
{
  if (g_hci_recv_cb)
    {
      return g_hci_recv_cb(NULL, buf);
    }

  net_buf_unref(buf);
  return -ENODEV;
}

int bt_recv_iso(struct net_buf *buf)
{
  (void)buf;
  return 0;
}

/* ===== HCI driver API (bt_hci_*) ===== */

int bt_hci_open(const struct device *dev, bt_hci_recv_t recv)
{
  g_hci_recv_cb = recv;
  if (g_hci_ops && g_hci_ops->open)
    {
      return g_hci_ops->open();
    }

  return 0;
}

int bt_hci_send(const struct device *dev, struct net_buf *buf)
{
  if (g_hci_ops && g_hci_ops->send)
    {
      return g_hci_ops->send(buf);
    }

  return -ENODEV;
}

int bt_hci_close(const struct device *dev)
{
  g_hci_recv_cb = NULL;
  if (g_hci_ops && g_hci_ops->close)
    {
      return g_hci_ops->close();
    }

  return 0;
}

bool bt_hci_can_close(const struct device *dev)
{
  return true;
}
