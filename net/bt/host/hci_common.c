/****************************************************************************
 * net/bt/host/hci_common.c
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

/* Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdbool.h>
#include <stdint.h>

#include <nuttx/net/bluetooth/buf.h>
#include <nuttx/net/bluetooth/hci_types.h>
#include <nuttx/net/bluetooth/compat/drivers/bluetooth.h>
#include <nuttx/net/bluetooth/compat/kernel.h>
#include <nuttx/mm/net_buf.h>
#include <nuttx/net/bluetooth/compat/sys/byteorder.h>

#include "common/assert.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

struct net_buf *bt_hci_evt_create(uint8_t evt, uint8_t len)
{
  struct bt_hci_evt_hdr *hdr;
  struct net_buf *buf;

  buf = bt_buf_get_evt(evt, false, ((clock_t)-1));

  BT_ASSERT(buf);

  hdr = net_buf_add(buf, sizeof(*hdr));
  hdr->evt = evt;
  hdr->len = len;

  return buf;
}

struct net_buf *bt_hci_cmd_complete_create(uint16_t op, uint8_t plen)
{
  struct net_buf *buf;
  struct bt_hci_evt_cmd_complete *cc;

  buf = bt_hci_evt_create(BT_HCI_EVT_CMD_COMPLETE, sizeof(*cc) + plen);

  cc = net_buf_add(buf, sizeof(*cc));

  /* The Num_HCI_Command_Packets parameter allows the Controller to
   * indicate the number of HCI command packets the Host can send to the
   * Controller. If the Controller requires the Host to stop sending
   * commands, Num_HCI_Command_Packets will be set to zero.
   *
   * NOTE: Zephyr Controller (and may be other Controllers) do not support
   * higher Number of HCI Command packets than 1.
   */

  cc->ncmd = 1U;

  cc->opcode = sys_cpu_to_le16(op);

  return buf;
}

struct net_buf *bt_hci_cmd_status_create(uint16_t op, uint8_t status)
{
  struct net_buf *buf;
  struct bt_hci_evt_cmd_status *cs;

  buf = bt_hci_evt_create(BT_HCI_EVT_CMD_STATUS, sizeof(*cs));

  cs = net_buf_add(buf, sizeof(*cs));
  cs->status = status;

  /* The Num_HCI_Command_Packets parameter allows the Controller to
   * indicate the number of HCI command packets the Host can send to the
   * Controller. If the Controller requires the Host to stop sending
   * commands, Num_HCI_Command_Packets will be set to zero.
   *
   * NOTE: Zephyr Controller (and may be other Controllers) do not support
   * higher Number of HCI Command packets than 1.
   */

  cs->ncmd = 1U;

  cs->opcode = sys_cpu_to_le16(op);

  return buf;
}
