/****************************************************************************
 * net/bt/mesh/sar_cfg.c
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

#include <nuttx/mm/net_buf.h>
#include <nuttx/net/bluetooth/bluetooth.h>
#include <nuttx/net/bluetooth/conn.h>
#include <nuttx/net/bluetooth/gatt.h>
#include <nuttx/net/bluetooth/mesh.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <sys/types.h>

#include "net.h"
#include "sar_cfg_internal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void bt_mesh_sar_tx_encode(struct net_buf_simple *buf,
                           const struct bt_mesh_sar_tx *tx)
{
  net_buf_simple_add_u8(buf, (tx->seg_int_step & 0xf) |
                        (tx->unicast_retrans_count << 4));
  net_buf_simple_add_u8(buf, (tx->unicast_retrans_without_prog_count & 0xf) |
                                     (tx->unicast_retrans_int_step << 4));
  net_buf_simple_add_u8(buf,
                        (tx->unicast_retrans_int_inc & 0xf) |
                        (tx->multicast_retrans_count << 4));
  net_buf_simple_add_u8(buf, tx->multicast_retrans_int & 0xf);
}

void bt_mesh_sar_rx_encode(struct net_buf_simple *buf,
                           const struct bt_mesh_sar_rx *rx)
{
  net_buf_simple_add_u8(buf, (rx->seg_thresh & 0x1f) |
                        (rx->ack_delay_inc << 5));
  net_buf_simple_add_u8(buf, (rx->discard_timeout & 0xf) |
                        ((rx->rx_seg_int_step & 0xf) << 4));
  net_buf_simple_add_u8(buf, (rx->ack_retrans_count & 0x3));
}

void bt_mesh_sar_tx_decode(struct net_buf_simple *buf,
                           struct bt_mesh_sar_tx *tx)
{
  uint8_t val;

  val = net_buf_simple_pull_u8(buf);
  tx->seg_int_step = (val & 0xf);
  tx->unicast_retrans_count = (val >> 4);
  val = net_buf_simple_pull_u8(buf);
  tx->unicast_retrans_without_prog_count = (val & 0xf);
  tx->unicast_retrans_int_step = (val >> 4);
  val = net_buf_simple_pull_u8(buf);
  tx->unicast_retrans_int_inc = (val & 0xf);
  tx->multicast_retrans_count = (val >> 4);
  val = net_buf_simple_pull_u8(buf);
  tx->multicast_retrans_int = (val & 0xf);
}

void bt_mesh_sar_rx_decode(struct net_buf_simple *buf,
                           struct bt_mesh_sar_rx *rx)
{
  uint8_t val;

  val = net_buf_simple_pull_u8(buf);
  rx->seg_thresh = (val & 0x1f);
  rx->ack_delay_inc = (val >> 5);
  val = net_buf_simple_pull_u8(buf);
  rx->discard_timeout = (val & 0xf);
  rx->rx_seg_int_step = (val >> 4);
  val = net_buf_simple_pull_u8(buf);
  rx->ack_retrans_count = (val & 0x3);
}
