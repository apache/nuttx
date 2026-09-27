/****************************************************************************
 * arch/arm/src/nrf54l/nrf54l_tickless_grtc.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <nuttx/debug.h>

#include <nuttx/arch.h>

#include "arm_internal.h"
#include "hardware/nrf54l_grtc.h"
#include "nrf54l_grtc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Check configuration */

#ifdef CONFIG_TIMER_ARCH
#  error CONFIG_TIMER_ARCH must be not set
#endif

/* The SYSCOUNTER runs at 1MHz so the counter value is in microseconds.
 * The compare value must be ahead of the counter for the event to fire,
 * so a small margin covers the register writes.
 */

#define NRF54L_GRTC_ALARM_MARGIN (10)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct nrf54l_tickless_dev_s
{
  struct nrf54l_grtc_dev_s *grtc; /* nrf54l GRTC driver */
  bool alarm_set;                 /* is the alarm set? */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int grtc_handler(int irq, void *context, void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct nrf54l_tickless_dev_s g_tickless_dev;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline void grtc_counter_to_ts(uint64_t counter, struct timespec *now)
{
  now->tv_sec  = counter / USEC_PER_SEC;
  now->tv_nsec = (counter % USEC_PER_SEC) * NSEC_PER_USEC;
}

static void grtc_prepare_alarm(uint64_t target)
{
  uint64_t counter;

  /* Set CC to the desired value. If the alarm is already due, the CC is
   * moved ahead of the counter so that the COMPARE event is guaranteed
   * to fire. We repeat this until the CC is ahead of the counter or
   * the event has already fired (as the counter may change in between).
   */

  for (; ; )
    {
      NRF54L_GRTC_GETCOUNTER(g_tickless_dev.grtc, &counter);
      if (target <= counter + NRF54L_GRTC_ALARM_MARGIN)
        {
          target = counter + NRF54L_GRTC_ALARM_MARGIN;
        }

      NRF54L_GRTC_SETCC(g_tickless_dev.grtc, NRF54L_GRTC_CC0, target);

      NRF54L_GRTC_GETCOUNTER(g_tickless_dev.grtc, &counter);
      if (target > counter ||
          NRF54L_GRTC_CHECKINT(g_tickless_dev.grtc,
                               NRF54L_GRTC_EVT_COMPARE0))
        {
          break;
        }
    }

  /* Enable interrupt */

  NRF54L_GRTC_ENABLEINT(g_tickless_dev.grtc, NRF54L_GRTC_EVT_COMPARE0);
}

/****************************************************************************
 * Name: grtc_cancel_ack
 ****************************************************************************/

static void grtc_cancel_ack(void)
{
  irqstate_t flags;

  flags = enter_critical_section();

  NRF54L_GRTC_DISABLEINT(g_tickless_dev.grtc, NRF54L_GRTC_EVT_COMPARE0);
  NRF54L_GRTC_DISABLECC(g_tickless_dev.grtc, NRF54L_GRTC_CC0);
  NRF54L_GRTC_ACKINT(g_tickless_dev.grtc, NRF54L_GRTC_EVT_COMPARE0);
  g_tickless_dev.alarm_set = false;

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: grtc_handler
 ****************************************************************************/

static int grtc_handler(int irq, void *context, void *arg)
{
  irqstate_t flags;

  flags = enter_critical_section();

  /* if the compare event fired */

  if (NRF54L_GRTC_CHECKINT(g_tickless_dev.grtc, NRF54L_GRTC_EVT_COMPARE0))
    {
      /* cancel the alarm and ack the event */

      grtc_cancel_ack();

      /* let scheduler now of alarm firing */

      nxsched_process_timer();
    }

  leave_critical_section(flags);

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_alarm_cancel
 ****************************************************************************/

int up_alarm_cancel(struct timespec *ts)
{
  uint64_t counter;
  irqstate_t flags;

  flags = enter_critical_section();

  NRF54L_GRTC_DISABLEINT(g_tickless_dev.grtc, NRF54L_GRTC_EVT_COMPARE0);
  NRF54L_GRTC_GETCOUNTER(g_tickless_dev.grtc, &counter);

  NRF54L_GRTC_DISABLECC(g_tickless_dev.grtc, NRF54L_GRTC_CC0);
  NRF54L_GRTC_ACKINT(g_tickless_dev.grtc, NRF54L_GRTC_EVT_COMPARE0);
  g_tickless_dev.alarm_set = false;

  leave_critical_section(flags);

  grtc_counter_to_ts(counter, ts);

  return OK;
}

/****************************************************************************
 * Name: up_alarm_start
 ****************************************************************************/

int up_alarm_start(const struct timespec *ts)
{
  uint64_t target;
  irqstate_t flags;

  flags = enter_critical_section();

  /* remember the alarm time */

  g_tickless_dev.alarm_set = true;

  /* Round up so the alarm never fires before the requested time. Clamp
   * to the counter range, the alarm is reevaluated before it can expire.
   */

  if (ts->tv_sec >= (GRTC_COUNTER_MAX / USEC_PER_SEC))
    {
      target = GRTC_COUNTER_MAX;
    }
  else
    {
      target = (uint64_t)ts->tv_sec * USEC_PER_SEC +
               (ts->tv_nsec + NSEC_PER_USEC - 1) / NSEC_PER_USEC;
    }

  grtc_prepare_alarm(target);

  leave_critical_section(flags);

  return OK;
}

/****************************************************************************
 * Name: up_timer_gettime
 ****************************************************************************/

int up_timer_gettime(struct timespec *ts)
{
  uint64_t counter;
  irqstate_t flags;

  flags = enter_critical_section();

  NRF54L_GRTC_GETCOUNTER(g_tickless_dev.grtc, &counter);
  grtc_counter_to_ts(counter, ts);

  leave_critical_section(flags);

  return OK;
}

/****************************************************************************
 * Name: up_timer_initialize
 ****************************************************************************/

void up_timer_initialize(void)
{
  memset(&g_tickless_dev, 0, sizeof(struct nrf54l_tickless_dev_s));

  g_tickless_dev.grtc = nrf54l_grtc_init(0);

  /* Ensure we have support for the GRTC instance */

  ASSERT(g_tickless_dev.grtc);

  /* Configure ISR */

  NRF54L_GRTC_SETISR(g_tickless_dev.grtc, grtc_handler, NULL);

  /* Start counting */

  NRF54L_GRTC_CLEAR(g_tickless_dev.grtc);
  NRF54L_GRTC_ACKINT(g_tickless_dev.grtc, NRF54L_GRTC_EVT_COMPARE0);
  NRF54L_GRTC_START(g_tickless_dev.grtc);

  /* kick off alarm scheduling */

  nxsched_process_timer();
}
