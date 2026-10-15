/****************************************************************************
 * net/bt/host/smp_null.c
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

/**
 * @file smp_null.c Security Manager Protocol stub
 */

/* Copyright (c) 2015-2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <errno.h>
#include <stddef.h>

#include <nuttx/net/bluetooth/bluetooth.h>
#include <nuttx/net/bluetooth/conn.h>
#include <nuttx/net/bluetooth/buf.h>
#include <nuttx/net/bluetooth/l2cap.h>
#include <nuttx/net/bluetooth/compat/kernel.h>
#include <nuttx/net/bluetooth/compat/logging/log.h>
#include <nuttx/mm/net_buf.h>
#include <nuttx/net/bluetooth/compat/sys/util.h>
#include <nuttx/net/bluetooth/compat/toolchain.h>

#include "conn_internal.h"
#include "hci_core.h"
#include "l2cap_internal.h"
#include "smp.h"

#define LOG_LEVEL CONFIG_BT_HCI_CORE_LOG_LEVEL
LOG_MODULE_REGISTER(bt_smp);

static struct bt_l2cap_le_chan bt_smp_pool[CONFIG_BT_MAX_CONN];

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int bt_smp_sign_verify(struct bt_conn *conn, struct net_buf *buf)
{
  return -ENOTSUP;
}

int bt_smp_sign(struct bt_conn *conn, struct net_buf *buf)
{
  return -ENOTSUP;
}

static int bt_smp_recv(struct bt_l2cap_chan *chan, struct net_buf *req_buf)
{
  struct bt_l2cap_le_chan *le_chan = BT_L2CAP_LE_CHAN(chan);
  struct bt_smp_pairing_fail *rsp;
  struct bt_smp_hdr *hdr;
  struct net_buf *buf;

  ARG_UNUSED(req_buf);

  /* If a device does not support pairing then it shall respond with
   * a Pairing Failed command with the reason set to "Pairing Not Supported"
   * when any command is received. Core Specification Vol. 3, Part H, 3.3
   */

  buf = bt_l2cap_create_pdu(NULL, 0);

  /* NULL is not a possible return due to ((clock_t)-1) */

  hdr = net_buf_add(buf, sizeof(*hdr));
  hdr->code = BT_SMP_CMD_PAIRING_FAIL;

  rsp = net_buf_add(buf, sizeof(*rsp));
  rsp->reason = BT_SMP_ERR_PAIRING_NOTSUPP;

  if (bt_l2cap_send_pdu(le_chan, buf, NULL, NULL))
    {
      net_buf_unref(buf);
    }

  return 0;
}

static int bt_smp_accept(struct bt_conn *conn, struct bt_l2cap_chan **chan)
{
  int i;
  static const struct bt_l2cap_chan_ops ops =
    {
          .recv = bt_smp_recv,
    };

  LOG_DBG("conn %p handle %u", conn, conn->handle);

  for (i = 0; i < ARRAY_SIZE(bt_smp_pool); i++)
    {
      struct bt_l2cap_le_chan *smp = &bt_smp_pool[i];

      if (smp->chan.conn)
        {
          continue;
        }

      smp->chan.ops = &ops;

      *chan = &smp->chan;

      return 0;
    }

  LOG_ERR("No available SMP context for conn %p", conn);

  return -ENOMEM;
}

BT_L2CAP_FIXED_CHANNEL_DEFINE(smp_fixed_chan) =
{
        .cid = BT_L2CAP_CID_SMP,
        .accept = bt_smp_accept,
};

int bt_smp_init(void)
{
  return 0;
}
