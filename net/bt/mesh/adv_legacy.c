/****************************************************************************
 * net/bt/mesh/adv_legacy.c
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

/* Copyright (c) 2018 Nordic Semiconductor ASA
 * Copyright (c) 2017 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

#include <nuttx/net/bluetooth/compat/kernel.h>
#include <nuttx/net/bluetooth/compat/sys/util.h>

#include <nuttx/kthread.h>

#include <nuttx/mm/net_buf.h>
#include <nuttx/net/bluetooth/bluetooth.h>
#include <nuttx/net/bluetooth/hci.h>
#include <nuttx/net/bluetooth/conn.h>
#include <nuttx/net/bluetooth/mesh.h>

#include "common/bt_str.h"

#include "host/hci_core.h"

#include "net.h"
#include "foundation.h"
#include "beacon.h"
#include "prov.h"
#include "solicitation.h"

#define LOG_LEVEL CONFIG_BT_MESH_ADV_LOG_LEVEL
#include <nuttx/net/bluetooth/compat/logging/log.h>
LOG_MODULE_REGISTER(bt_mesh_adv_legacy);

/* Pre-5.0 controllers enforce a minimum interval of 100ms
 * whereas 5.0+ controllers can go down to 20ms.
 */
#define ADV_INT_DEFAULT_MS 100
#define ADV_INT_FAST_MS 20

static pid_t adv_thread_pid = INVALID_PROCESS_ID;
static uint64_t adv_thread_stack[CONFIG_BT_MESH_ADV_STACK_SIZE /
                                 sizeof(uint64_t)];
static int32_t adv_timeout;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static bool is_mesh_suspended(void)
{
  return atomic_test_bit(bt_mesh.flags, BT_MESH_SUSPENDED);
}

static int bt_data_send(uint8_t num_events,
                        uint16_t adv_int,
                        const struct bt_data *ad,
                        size_t ad_len,
                        struct bt_mesh_adv_ctx *ctx)
{
  struct bt_le_adv_param param =
    {
    };

  uint64_t uptime = TICK2MSEC(clock_systime_ticks());
  uint16_t duration;
  int err;
  const int32_t adv_int_min =
          ((bt_dev.hci_version >=
              BT_HCI_VERSION_5_0) ? ADV_INT_FAST_MS : ADV_INT_DEFAULT_MS);

  adv_int = MAX(adv_int_min, adv_int);

  ARG_UNUSED(uptime);

  /* NuttX Bluetooth Low Energy Controller for mesh stack uses
   * pre-emptible continuous scanning, allowing advertising events to be
   * transmitted without delay when advertising is enabled. No need to
   * compensate with scan window duration. An advertising event could be
   * delayed by upto one interval when advertising is stopped and started in
   * quick succession, hence add advertising interval to the total
   * advertising duration.
   */

  duration = adv_int + num_events * (adv_int + 10);

  /* NuttX Bluetooth Low Energy Controller built for nRF51x SoCs use
   * CONFIG_BT_CTLR_LOW_LAT=y, and continuous scanning cannot be preempted,
   * hence, scanning will block advertising events from being transmitted.
   * Increase the advertising duration by the amount of scan window duration
   * to compensate for the blocked advertising events.
   */

  if (IS_ENABLED(CONFIG_BT_CTLR_LOW_LAT))
    {
      duration += BT_MESH_SCAN_WINDOW_MS;
    }

  LOG_DBG("count %u interval %ums duration %ums", num_events, adv_int,
          duration);

  if (IS_ENABLED(CONFIG_BT_MESH_DEBUG_USE_ID_ADDR))
    {
      param.options = BT_LE_ADV_OPT_USE_IDENTITY;
    }
  else
    {
      param.options = 0U;
    }

  param.id = BT_ID_DEFAULT;
  param.interval_min = BT_MESH_ADV_SCAN_UNIT(adv_int);
  param.interval_max = param.interval_min;

  err = bt_le_adv_start(&param, ad, ad_len, NULL, 0);

  if (err)
    {
      LOG_ERR("Advertising failed: err %d", err);
      return err;
    }

  LOG_DBG("Advertising started. Sleeping %u ms", duration);

  if (ctx)
    {
      bt_mesh_adv_send_start(duration, err, ctx);
    }

  if (!is_mesh_suspended())
    {
      nxsched_msleep(duration);
    }

  err = bt_le_adv_stop();
  if (err)
    {
      LOG_ERR("Stopping advertising failed: err %d", err);
      return err;
    }

  LOG_DBG("Advertising stopped (%u ms)",
          (uint32_t)(TICK2MSEC(clock_systime_ticks()) - uptime));

  return 0;
}

int bt_mesh_adv_bt_data_send(uint8_t num_events,
                             uint16_t adv_int,
                             const struct bt_data *ad,
                             size_t ad_len)
{
  return bt_data_send(num_events, adv_int, ad, ad_len, NULL);
}

static inline void adv_send(struct bt_mesh_adv *adv)
{
  uint16_t num_events = BT_MESH_TRANSMIT_COUNT(adv->ctx.xmit) + 1;
  uint16_t adv_int;
  struct bt_data ad;

  adv_int = BT_MESH_TRANSMIT_INT(adv->ctx.xmit);

  LOG_DBG("type %u len %u: "
          "%s", adv->ctx.type, adv->b.len, bt_hex(adv->b.data,
          adv->b.len));

  ad.type = bt_mesh_adv_type[adv->ctx.type];
  ad.data_len = adv->b.len;
  ad.data = adv->b.data;

  bt_data_send(num_events, adv_int, &ad, 1, &adv->ctx);
}

static void adv_pool_drain(void)
{
  struct bt_mesh_adv *adv;

  /* Empty the advertising pool when advertising is disabled */

  while ((adv = bt_mesh_adv_get(0)))
    {
      bt_mesh_adv_send_start(0, -ENODEV, &adv->ctx);
      bt_mesh_adv_unref(adv);
    }
}

static int adv_thread(int argc, char *argv[])
{
  ARG_UNUSED(argc);
  ARG_UNUSED(argv);

  LOG_DBG("started");
  struct bt_mesh_adv *adv;

  while (true)
    {
      if (is_mesh_suspended())
        {
          adv_pool_drain();
          nxsched_msleep(10);
          continue;
        }

      if (IS_ENABLED(CONFIG_BT_MESH_GATT_SERVER))
        {
          adv = bt_mesh_adv_get(0);
          if (IS_ENABLED(CONFIG_BT_MESH_PROXY_SOLICITATION) && !adv)
            {
              (void)bt_mesh_sol_send();
            }

          while (!adv && !is_mesh_suspended())
            {
              /* Adv timeout may be set by a call from proxy
               * to bt_mesh_adv_gatt_start:
               */

              adv_timeout = -1;
              (void)bt_mesh_adv_gatt_send();

              adv = bt_mesh_adv_get(adv_timeout == -1 ?
              (clock_t)-1 : MSEC2TICK(adv_timeout));
              bt_le_adv_stop();

              if (IS_ENABLED(CONFIG_BT_MESH_PROXY_SOLICITATION) && !adv)
                {
                  (void)bt_mesh_sol_send();
                }
            }

          if (is_mesh_suspended())
            {
              continue;
            }
        }
      else
        {
          adv = bt_mesh_adv_get(((clock_t)-1));
        }

      if (!adv)
        {
          continue;
        }

      /* busy == 0 means this was canceled */

      if (adv->ctx.busy)
        {
          adv->ctx.busy = 0U;
          adv_send(adv);
        }

      struct bt_mesh_adv_ctx ctx = adv->ctx;

      adv->ctx.started = 0;
      bt_mesh_adv_unref(adv);
      bt_mesh_adv_send_end(0, &ctx);

      /* Give other threads a chance to run */

      sched_yield();
    }

  return 0;
}

void bt_mesh_adv_local_ready(void)
{
  /* Will be handled automatically */
}

void bt_mesh_adv_relay_ready(void)
{
  /* Will be handled automatically */
}

void bt_mesh_adv_gatt_update(void)
{
  bt_mesh_adv_get_cancel();
}

int bt_mesh_adv_terminate(struct bt_mesh_adv *adv)
{
  ARG_UNUSED(adv);

  return 0;
}

void bt_mesh_adv_init(void)
{
  if (adv_thread_pid != INVALID_PROCESS_ID)
    {
      return;
    }

  adv_thread_pid =
          kthread_create_with_stack("BT Mesh adv",
                  K_PRIO_COOP(CONFIG_BT_MESH_ADV_PRIO),
                                    adv_thread_stack,
                                    sizeof(adv_thread_stack), adv_thread,
                                    NULL);
  DEBUGASSERT(adv_thread_pid > 0);
}

int bt_mesh_adv_enable(void)
{
  /* The advertiser thread relies on BT_MESH_SUSPENDED flag. No point in
   * starting the advertiser thread if the flag is not set.
   */

  if (is_mesh_suspended())
    {
      return -EINVAL;
    }

  return 0;
}

int bt_mesh_adv_disable(void)
{
  /* The advertiser thread is kept alive permanently. The flag is set by
   * the higher layer function, and the thread will observe it and drain any
   * pending advertising buffers without being joined or recreated.
   */

  if (!is_mesh_suspended())
    {
      return -EINVAL;
    }

  LOG_DBG("Advertising disabled");

  return 0;
}

int bt_mesh_adv_gatt_start(const struct bt_le_adv_param *param,
                           int32_t duration,
                           const struct bt_data *ad,
                           size_t ad_len,
                           const struct bt_data *sd,
                           size_t sd_len)
{
  adv_timeout = duration;
  return bt_le_adv_start(param, ad, ad_len, sd, sd_len);
}

int bt_mesh_wq_submit(struct k_work *work)
{
  return k_work_submit(work);
}
