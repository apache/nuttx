/****************************************************************************
 * net/bt/mesh/statistic.c
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

/* Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/net/bluetooth/mesh.h>

#include "net.h"
#include "statistic.h"

static struct bt_mesh_statistic stat;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void bt_mesh_stat_get(struct bt_mesh_statistic *st)
{
  memcpy(st, &stat, sizeof(struct bt_mesh_statistic));
}

void bt_mesh_stat_reset(void)
{
  memset(&stat, 0, sizeof(struct bt_mesh_statistic));
}

void bt_mesh_stat_planned_count(struct bt_mesh_adv_ctx *ctx)
{
  if (ctx->tag == BT_MESH_ADV_TAG_LOCAL)
    {
      stat.tx_local_planned++;
    }
  else if (ctx->tag == BT_MESH_ADV_TAG_RELAY)
    {
      stat.tx_adv_relay_planned++;
    }
  else if (ctx->tag == BT_MESH_ADV_TAG_FRIEND)
    {
      stat.tx_friend_planned++;
    }
}

void bt_mesh_stat_succeeded_count(struct bt_mesh_adv_ctx *ctx)
{
  if (ctx->tag == BT_MESH_ADV_TAG_LOCAL)
    {
      stat.tx_local_succeeded++;
    }
  else if (ctx->tag == BT_MESH_ADV_TAG_RELAY)
    {
      stat.tx_adv_relay_succeeded++;
    }
  else if (ctx->tag == BT_MESH_ADV_TAG_FRIEND)
    {
      stat.tx_friend_succeeded++;
    }
}

void bt_mesh_stat_rx(enum bt_mesh_net_if net_if)
{
  switch (net_if)
    {
      case BT_MESH_NET_IF_ADV:
        stat.rx_adv++;
        break;
      case BT_MESH_NET_IF_LOCAL:
        stat.rx_loopback++;
        break;
      case BT_MESH_NET_IF_PROXY:
      case BT_MESH_NET_IF_PROXY_CFG:
        stat.rx_proxy++;
        break;
      default:
        stat.rx_uknown++;
        break;
    }
}

#if defined(CONFIG_BT_MESH_LOW_POWER)

static struct bt_mesh_lpn_timing lpn_timing;

static int64_t poll_tx_end_ts;
static int64_t scan_start_ts;

void bt_mesh_stat_lpn_timing_update_poll_tx(void)
{
  poll_tx_end_ts = clock_systime_ticks();
}

void bt_mesh_stat_lpn_timing_update_scan_start(void)
{
  int64_t now = clock_systime_ticks();
  uint32_t delay;

  scan_start_ts = now;

  if (poll_tx_end_ts == 0)
    {
      return;
    }

  delay = TICK2USEC(now - poll_tx_end_ts);
  lpn_timing.recv_delay_us = delay;

  if (lpn_timing.cnt == 0 && lpn_timing.recv_delay_min_us == 0)
    {
      lpn_timing.recv_delay_min_us = delay;
      lpn_timing.recv_delay_max_us = delay;
    }
  else
    {
      if (delay < lpn_timing.recv_delay_min_us)
        {
          lpn_timing.recv_delay_min_us = delay;
        }

      if (delay > lpn_timing.recv_delay_max_us)
        {
          lpn_timing.recv_delay_max_us = delay;
        }
    }
}

void bt_mesh_stat_lpn_timing_update_response_rx(void)
{
  int64_t now = clock_systime_ticks();
  uint32_t win;

  if (scan_start_ts == 0)
    {
      return;
    }

  win = TICK2USEC(now - scan_start_ts);
  lpn_timing.recv_win_us = win;

  if (lpn_timing.cnt == 0)
    {
      lpn_timing.recv_win_min_us = win;
      lpn_timing.recv_win_max_us = win;
    }
  else
    {
      if (win < lpn_timing.recv_win_min_us)
        {
          lpn_timing.recv_win_min_us = win;
        }

      if (win > lpn_timing.recv_win_max_us)
        {
          lpn_timing.recv_win_max_us = win;
        }
    }

  lpn_timing.cnt++;
}

void bt_mesh_stat_lpn_timing_update_win_expired(void)
{
  int64_t now = clock_systime_ticks();

  if (scan_start_ts == 0)
    {
      return;
    }

  lpn_timing.recv_win_expected_us = TICK2USEC(now - scan_start_ts);
  lpn_timing.cnt_failed++;
}

int bt_mesh_stat_lpn_timing_get(struct bt_mesh_lpn_timing *timing)
{
  if (timing == NULL)
    {
      return -EINVAL;
    }

  memcpy(timing, &lpn_timing, sizeof(*timing));
  return 0;
}

void bt_mesh_stat_lpn_timing_reset(void)
{
  memset(&lpn_timing, 0, sizeof(lpn_timing));
  poll_tx_end_ts = 0;
  scan_start_ts = 0;
}

#else /* !CONFIG_BT_MESH_LOW_POWER */

int bt_mesh_stat_lpn_timing_get(struct bt_mesh_lpn_timing *timing)
{
  ARG_UNUSED(timing);
  return -ENOTSUP;
}

void bt_mesh_stat_lpn_timing_reset(void)
{
}

#endif /* CONFIG_BT_MESH_LOW_POWER */
