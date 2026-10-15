/****************************************************************************
 * net/bt/host/buf.c
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

/* Copyright (c) 2017 Nordic Semiconductor ASA
 * Copyright (c) 2015 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stddef.h>
#include <stdint.h>
#include <sched.h>
#include <string.h>

#include <nuttx/config.h>
#include <nuttx/net/bluetooth/hci.h>
#include <nuttx/net/bluetooth/buf.h>
#include <nuttx/net/bluetooth/hci_types.h>
#include <nuttx/net/bluetooth/l2cap.h>
#include <nuttx/net/bluetooth/compat/kernel.h>
#include <nuttx/net/bluetooth/compat/logging/log.h>
#include <nuttx/mm/net_buf.h>
#include <assert.h>
#include <nuttx/clock.h>
#include <nuttx/irq.h>

#include "buf_view.h"
#include "common/hci_common_internal.h"
#include "conn_internal.h"
#include "hci_core.h"
#include "iso_internal.h"

LOG_MODULE_REGISTER(bt_buf, CONFIG_BT_LOG_LEVEL);

/* Events have a length field of 1 byte. This size fits all events.
 *
 * It's true that we don't put all kinds of events there (yet). However, the
 * command complete event has an arbitrary payload, depending on opcode.
 */
#define SYNC_EVT_SIZE (BT_BUF_RESERVE + BT_HCI_EVT_HDR_SIZE + 255)

static void *buf_rx_freed_cb;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void buf_rx_freed_notify(enum bt_buf_type mask)
{
  bt_buf_rx_freed_cb_t cb;
  bool in_isr = up_interrupt_context();

  if (!in_isr)
    {
      sched_lock();
    }

  cb = (bt_buf_rx_freed_cb_t)atomic_ptr_read(&buf_rx_freed_cb);

  if (cb != NULL)
    {
      cb(mask);
    }

  if (!in_isr)
    {
      sched_unlock();
    }
}

#if defined(CONFIG_BT_ISO_RX)
static void iso_rx_freed_cb(void)
{
  buf_rx_freed_notify(BT_BUF_ISO_IN);
}
#endif

/* Pool for RX HCI buffers that are always freed by `bt_recv`
 * before it returns.
 *
 * A singleton buffer shall be sufficient for correct operation. The buffer
 * count may be increased as an optimization to allow the HCI transport to
 * fill buffers in parallel with `bt_recv` consuming them.
 */

NET_BUF_POOL_FIXED_DEFINE(sync_evt_pool,
                          CONFIG_BT_BUF_SYNC_EVT_POOL_COUNT,
                          SYNC_EVT_SIZE,
                          0,
                          NULL);

NET_BUF_POOL_FIXED_DEFINE(discardable_pool,
    CONFIG_BT_BUF_EVT_DISCARDABLE_COUNT,

                        BT_BUF_EVT_SIZE(CONFIG_BT_BUF_EVT_DISCARDABLE_SIZE),
                              0,
                              NULL);

#if defined(CONFIG_BT_HCI_ACL_FLOW_CONTROL)
static void acl_in_pool_destroy(struct net_buf *buf)
{
  bt_hci_host_num_completed_packets(buf);
  buf_rx_freed_notify(BT_BUF_ACL_IN);
}

static void evt_pool_destroy(struct net_buf *buf)
{
  net_buf_destroy(buf);
  buf_rx_freed_notify(BT_BUF_EVT);
}

NET_BUF_POOL_DEFINE(acl_in_pool,
                    (BT_BUF_ACL_RX_COUNT_EXTRA + BT_BUF_HCI_ACL_RX_COUNT),
                    BT_BUF_ACL_SIZE(CONFIG_BT_BUF_ACL_RX_SIZE),
                        sizeof(struct bt_conn_rx),
                    acl_in_pool_destroy);

NET_BUF_POOL_FIXED_DEFINE(evt_pool,
    CONFIG_BT_BUF_EVT_RX_COUNT,
    BT_BUF_EVT_RX_SIZE,
    0,

                          evt_pool_destroy);
#else
static void hci_rx_pool_destroy(struct net_buf *buf)
{
  net_buf_destroy(buf);

  /* When ACL Flow Control is disabled, a single pool is used for events and
   * acl data. Therefore the callback will always notify about both types of
   * buffers, BT_BUF_EVT and BT_BUF_ACL_IN.
   */

  buf_rx_freed_notify(BT_BUF_EVT | BT_BUF_ACL_IN);
}

NET_BUF_POOL_FIXED_DEFINE(hci_rx_pool,
                          BT_BUF_RX_COUNT,
                          BT_BUF_RX_SIZE,
                          sizeof(struct bt_conn_rx),
                          hci_rx_pool_destroy);
#endif /* CONFIG_BT_HCI_ACL_FLOW_CONTROL */

struct net_buf *bt_buf_get_rx(enum bt_buf_type type, clock_t timeout)
{
  struct net_buf *buf;

  DEBUGASSERT(type == BT_BUF_EVT
      || type == BT_BUF_ACL_IN
      || type == BT_BUF_ISO_IN);

  if (IS_ENABLED(CONFIG_BT_ISO_RX) && type == BT_BUF_ISO_IN)
    {
      return bt_iso_get_rx(timeout);
    }

#if defined(CONFIG_BT_HCI_ACL_FLOW_CONTROL)
  if (type == BT_BUF_EVT)
    {
      buf = net_buf_alloc(&evt_pool, timeout);
    }
  else
    {
      buf = net_buf_alloc(&acl_in_pool, timeout);
    }
#else
  buf = net_buf_alloc(&hci_rx_pool, timeout);
#endif
  if (buf)
    {
      net_buf_add_u8(buf, bt_buf_type_to_h4(type));
    }

  return buf;
}

void bt_buf_rx_freed_cb_set(bt_buf_rx_freed_cb_t cb)
{
  atomic_ptr_xchg(&buf_rx_freed_cb, (void *)cb);

#if defined(CONFIG_BT_ISO_RX)
  bt_iso_buf_rx_freed_cb_set(cb != NULL ? iso_rx_freed_cb : NULL);
#endif
}

struct net_buf *bt_buf_get_evt(uint8_t evt,
                               bool discardable, clock_t timeout)
{
  struct net_buf *buf;

  switch (evt)
    {
#if defined(CONFIG_BT_CONN) || defined(CONFIG_BT_ISO)
      case BT_HCI_EVT_NUM_COMPLETED_PACKETS:
#endif /* CONFIG_BT_CONN || CONFIG_BT_ISO */
      case BT_HCI_EVT_CMD_STATUS:
      case BT_HCI_EVT_CMD_COMPLETE:
        buf = net_buf_alloc(&sync_evt_pool, timeout);
        break;
      default:
        if (discardable)
          {
            /* Discardable, decided in Host-side HCI Transport driver. */

            buf = net_buf_alloc(&discardable_pool, timeout);
          }
        else
          {
            return bt_buf_get_rx(BT_BUF_EVT, timeout);
          }
    }

  if (buf)
    {
      net_buf_add_u8(buf, BT_HCI_H4_EVT);
    }

  return buf;
}

#ifdef ZTEST_UNITTEST
#if defined(CONFIG_BT_HCI_ACL_FLOW_CONTROL)
struct net_buf_pool *bt_buf_get_evt_pool(void)
{
  return &evt_pool;
}

struct net_buf_pool *bt_buf_get_acl_in_pool(void)
{
  return &acl_in_pool;
}
#else
struct net_buf_pool *bt_buf_get_hci_rx_pool(void)
{
  return &hci_rx_pool;
}
#endif /* CONFIG_BT_HCI_ACL_FLOW_CONTROL */

#if defined(CONFIG_BT_CONN) || defined(CONFIG_BT_ISO)
struct net_buf_pool *bt_buf_get_num_complete_pool(void)
{
  return &sync_evt_pool;
}
#endif /* CONFIG_BT_CONN || CONFIG_BT_ISO */
#endif /* ZTEST_UNITTEST */

struct net_buf *bt_buf_make_view(
  struct net_buf *view, struct net_buf *parent, size_t len,
  struct bt_buf_view_meta *meta)
{
  DEBUGASSERT(len);
  DEBUGASSERT(view);
  /* The whole point of this API is to allow prepending data. If the
   * headroom is 0, that will not happen.
   */

  DEBUGASSERT(net_buf_headroom(parent) > 0);

  DEBUGASSERT(!bt_buf_has_view(parent));

  LOG_DBG("make-view %p viewsize %zu meta %p", view, len, meta);

  net_buf_simple_clone(&parent->b, &view->b);
  view->size = net_buf_headroom(parent) + len;
  view->len = len;
  view->flags = NET_BUF_EXTERNAL_DATA;

  /* we have a view, eat `len`'s worth of data from the parent */

  (void)net_buf_pull(parent, len);

  meta->backup.data = parent->data;
  parent->data = NULL;

  meta->backup.size = parent->size;
  parent->size = 0;

  /* The ref to `parent` is moved in by passing `parent` as argument. */

  /* save backup & "clip" the buffer so the next `make_view` will fail */

  meta->parent = parent;
  parent = NULL;

  return view;
}

void bt_buf_destroy_view(struct net_buf *view, struct bt_buf_view_meta *meta)
{
  LOG_DBG("destroy-view %p meta %p", view, meta);
  DEBUGASSERT(meta->parent);

  /* "unclip" the parent buf */

  meta->parent->data = meta->backup.data;
  meta->parent->size = meta->backup.size;

  net_buf_unref(meta->parent);

  memset(meta, 0, sizeof(*meta));
  net_buf_destroy(view);
}

bool bt_buf_has_view(const struct net_buf *parent)
{
  /* This is enforced by `make_view`. see comment there. */

  return parent->size == 0 && parent->data == NULL;
}
