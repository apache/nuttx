/****************************************************************************
 * net/bt/host/long_wq.c
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

/* long_work.c - Workqueue intended for long-running operations. */

/* Copyright (c) 2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/net/bluetooth/compat/kernel.h>
#include <nuttx/net/bluetooth/compat/init.h>
#include <nuttx/clock.h>
#include <stdint.h>

static uint64_t bt_lw_stack_area[CONFIG_BT_LONG_WQ_STACK_SIZE /
                                 sizeof(uint64_t)];
static struct k_work_q bt_long_wq;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int bt_long_wq_schedule(struct k_work_delayable *dwork, clock_t timeout)
{
  return k_work_schedule_for_queue(&bt_long_wq, dwork, timeout);
}

int bt_long_wq_reschedule(struct k_work_delayable *dwork, clock_t timeout)
{
  return k_work_reschedule_for_queue(&bt_long_wq, dwork, timeout);
}

int bt_long_wq_submit(struct k_work *work)
{
  return k_work_submit_to_queue(&bt_long_wq, work);
}

static int long_wq_init(void)
{
  const struct k_work_queue_config cfg =
    {
        .name = "BT LW WQ"
    };

  k_work_queue_init(&bt_long_wq);

  k_work_queue_start(&bt_long_wq, bt_lw_stack_area, sizeof(bt_lw_stack_area),
                     CONFIG_BT_LONG_WQ_PRIO, &cfg);

  return 0;
}

SYS_INIT(long_wq_init, POST_KERNEL, CONFIG_BT_LONG_WQ_INIT_PRIO);
