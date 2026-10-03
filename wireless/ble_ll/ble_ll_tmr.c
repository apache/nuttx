/****************************************************************************
 * wireless/ble_ll/ble_ll_tmr.c
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
 * Derived from Apache Mynewt NimBLE (porting/nimble/src/hal_timer.c).
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "ble_ll_os.h"
#include "ble_ll_tmr.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static TAILQ_HEAD(, ble_ll_tmr) g_ble_ll_tmr_q =
  TAILQ_HEAD_INITIALIZER(g_ble_ll_tmr_q);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ble_ll_tmr_rearm
 *
 * Description:
 *   Program the lower-half compare channel for the first queued timer.
 *   Must be called with interrupts disabled.
 *
 ****************************************************************************/

static void ble_ll_tmr_rearm(void)
{
  struct ble_ll_tmr *tmr;

  tmr = TAILQ_FIRST(&g_ble_ll_tmr_q);
  if (tmr != NULL)
    {
      BLE_PHY_OP(tmr_start, tmr->expiry);
    }
  else
    {
      BLE_PHY_OP(tmr_stop);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ble_ll_tmr_init
 *
 * Description:
 *   Initialize a Link Layer timer.
 *
 ****************************************************************************/

void ble_ll_tmr_init(struct ble_ll_tmr *tmr, ble_ll_tmr_cb *cb, void *arg)
{
  DEBUGASSERT(tmr != NULL && cb != NULL);

  memset(tmr, 0, sizeof(*tmr));
  tmr->cb  = cb;
  tmr->arg = arg;
}

/****************************************************************************
 * Name: ble_ll_tmr_start
 *
 * Description:
 *   Start a timer at an absolute time.  A running timer is restarted.  A
 *   target in the past expires as soon as possible.
 *
 ****************************************************************************/

void ble_ll_tmr_start(struct ble_ll_tmr *tmr, uint32_t tgt)
{
  struct ble_ll_tmr *entry;
  irqstate_t flags;

  flags = enter_critical_section();

  if (tmr->queued)
    {
      TAILQ_REMOVE(&g_ble_ll_tmr_q, tmr, link);
    }

  tmr->expiry = tgt;
  tmr->queued = true;

  TAILQ_FOREACH(entry, &g_ble_ll_tmr_q, link)
    {
      if (LL_TMR_LT(tgt, entry->expiry))
        {
          break;
        }
    }

  if (entry != NULL)
    {
      TAILQ_INSERT_BEFORE(entry, tmr, link);
    }
  else
    {
      TAILQ_INSERT_TAIL(&g_ble_ll_tmr_q, tmr, link);
    }

  if (TAILQ_FIRST(&g_ble_ll_tmr_q) == tmr)
    {
      ble_ll_tmr_rearm();
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: ble_ll_tmr_stop
 *
 * Description:
 *   Stop a timer.
 *
 ****************************************************************************/

void ble_ll_tmr_stop(struct ble_ll_tmr *tmr)
{
  irqstate_t flags;
  bool first;

  flags = enter_critical_section();

  if (tmr->queued)
    {
      first = TAILQ_FIRST(&g_ble_ll_tmr_q) == tmr;
      TAILQ_REMOVE(&g_ble_ll_tmr_q, tmr, link);
      tmr->queued = false;

      if (first)
        {
          ble_ll_tmr_rearm();
        }
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: ble_ll_tmr_isr
 *
 * Description:
 *   Run all expired timers.  Called by the lower half from the timer
 *   compare interrupt.
 *
 ****************************************************************************/

void ble_ll_tmr_isr(void)
{
  struct ble_ll_tmr *tmr;
  irqstate_t flags;

  flags = enter_critical_section();

  while ((tmr = TAILQ_FIRST(&g_ble_ll_tmr_q)) != NULL)
    {
      if (LL_TMR_GT(tmr->expiry, ble_ll_tmr_get()))
        {
          break;
        }

      TAILQ_REMOVE(&g_ble_ll_tmr_q, tmr, link);
      tmr->queued = false;
      tmr->cb(tmr->arg);
    }

  ble_ll_tmr_rearm();

  leave_critical_section(flags);
}
