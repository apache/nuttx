/****************************************************************************
 * arch/arm/src/nrf52/nrf52_tickless_rtc.c
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
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>

#include "arm_arch.h"

#include "hardware/nrf52_rtc.h"
#include "nrf52_rtc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Check corresponding RTC support */

#if (CONFIG_NRF52_SYSTIMER_RTC_INSTANCE == 0) && !defined(CONFIG_NRF52_RTC0)
#  error "Support for RTC0 is not enabled"
#elif (CONFIG_NRF52_SYSTIMER_RTC_INSTANCE == 1) && !defined(CONFIG_NRF52_RTC1)
#  error "Support for RTC1 is not enabled"
#elif (CONFIG_NRF52_SYSTIMER_RTC_INSTANCE == 2) && !defined(CONFIG_NRF52_RTC2)
#  error "Support for RTC2 is not enabled"
#endif

/* TIMER configuration */

#if (CONFIG_NRF52_SYSTIMER_RTC_INSTANCE == 0)
#  define NRF52_RTC_BASE     NRF52_RTC0_BASE
#elif (CONFIG_NRF52_SYSTIMER_RTC_INSTANCE == 1)
#  define NRF52_RTC_BASE     NRF52_RTC1_BASE
#elif (CONFIG_NRF52_SYSTIMER_RTC_INSTANCE == 2)
#  define NRF52_RTC_BASE     NRF52_RTC2_BASE
#endif

#define NRF52_RTC_PERIOD   (512)
#define NRF52_RTC_MAX      (0x00ffffff)
#define NRF52_RTC_MAX_TIME (NRF52_RTC_MAX * 31)

/* Convert uS to timer count for f = 32768Hz, using more precision
 * when possible:
 * (1 / 32768) s ~ 30.51 uS ~ 31 uS
 * 512 * (1 / 32768) s = 0.015625 s = 15625 uS
 * So, instead of always dividing by 31, if t * 512 < (2**32 - 1), we can do:
 * (t * 512) / 15625 ~ t / 30.51
 */

#define USEC_TO_COUNTER(t) (t > 0x7fffff ? (t / 31) : ((t * 512) / 15625))

/* To convert from counter to uS we split the counter into one second worth
 * of counts (32768) and a fractional part we can safely multiply first
 * by (USEC_PER_SEC/8) and still be within 32 bit value
 */

#define COUNTER_TO_USEC(c) ((c / 32768) * USEC_PER_SEC) + \
                            (((c % 32768) * (USEC_PER_SEC / 8)) / (32768 / 8))

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct nrf52_tickless_dev_s
{
  FAR struct nrf52_rtc_dev_s *rtc;  /* nrf52 RTC driver */
  uint32_t periods;                 /* how many times the timer overflowed */
  bool alarm_set;                   /* is the alarm set? */
  struct timespec alarm;            /* absolute time of alarm */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int rtc_handler(int irq, void *context, void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct nrf52_tickless_dev_s g_tickless_dev;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void rtc_counter_to_ts(uint32_t counter, struct timespec *now)
{
  uint32_t usec;

  usec = COUNTER_TO_USEC(counter);
  now->tv_sec  = usec / USEC_PER_SEC;
  now->tv_nsec = (usec % USEC_PER_SEC) * NSEC_PER_USEC;

  now->tv_sec += g_tickless_dev.periods * NRF52_RTC_PERIOD;
}

static void rtc_prepare_alarm(void)
{
  struct timespec now;
  struct timespec delta;
  uint32_t usec;
  uint32_t counter;
  uint32_t target_counter;

  /* get current counter and absolute time */

  NRF52_RTC_GETCOUNTER(g_tickless_dev.rtc, &counter);
  rtc_counter_to_ts(counter, &now);

  /* obtain relative time to alarm */

  clock_timespec_subtract(&g_tickless_dev.alarm, &now, &delta);
  usec = delta.tv_sec * USEC_PER_SEC + delta.tv_nsec / NSEC_PER_USEC;

  /* if the alarm is to expire within one RTC period, we can set the CC */

  if (usec < NRF52_RTC_PERIOD * USEC_PER_SEC)
    {
      /* usec is the time w.r.t. now, so we compute the counter compare value
       * from current counter. as this may be "behind" the counter, we wrap
       * around one full timer period
       */

      target_counter = USEC_TO_COUNTER(usec);

      if (target_counter < 2)
        {
          /* ensure counter fires, from nRF52832_PS_v1.4 (p. 245):
           * "If the COUNTER is N, writing N+2 to a CC register is
           * guaranteed to trigger a COMPARE event at N+2."
           */

          target_counter = 2;
        }

      target_counter = (counter + target_counter) % (NRF52_RTC_MAX + 1);

      NRF52_RTC_SETCC(g_tickless_dev.rtc, NRF52_RTC_CC0, target_counter);
      NRF52_RTC_ENABLEINT(g_tickless_dev.rtc, NRF52_RTC_EVT_COMPARE0);
    }
}

/****************************************************************************
 * Name: rtc_handler
 ****************************************************************************/

static int rtc_handler(int irq, void *context, void *arg)
{
  irqstate_t flags;

  flags = enter_critical_section();

  /* if the timer wrapped-around */

  if (getreg32(NRF52_RTC_BASE + NRF52_RTC_EVENTS_OVRFLW_OFFSET))
    {
      /* ack interrupt */

      NRF52_RTC_ACKINT(g_tickless_dev.rtc, NRF52_RTC_EVT_OVRFLW);

      /* count one more period */

      g_tickless_dev.periods++;

      /* check if the currently set alarm is to fire in this new period */

      if (g_tickless_dev.alarm_set)
        {
          rtc_prepare_alarm();
        }
    }

  /* if the compare event fired */

  if (getreg32(NRF52_RTC_BASE + NRF52_RTC_EVENTS_COMPARE_OFFSET(0)))
    {
      struct timespec now;

      /* cancel alarm and get current time */

      up_alarm_cancel(&now);

      /* let scheduler now of alarm firing */

      nxsched_alarm_expiration(&now);
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

int up_alarm_cancel(FAR struct timespec *ts)
{
  if (g_tickless_dev.alarm_set)
    {
      uint32_t counter;
      irqstate_t flags;

      flags = enter_critical_section();

      NRF52_RTC_DISABLEINT(g_tickless_dev.rtc, NRF52_RTC_EVT_COMPARE0);
      NRF52_RTC_GETCOUNTER(g_tickless_dev.rtc, &counter);
      rtc_counter_to_ts(counter, ts);

      NRF52_RTC_ACKINT(g_tickless_dev.rtc, NRF52_RTC_EVT_COMPARE0);
      g_tickless_dev.alarm_set = false;

      leave_critical_section(flags);
    }

  return OK;
}

/****************************************************************************
 * Name: up_alarm_start
 ****************************************************************************/

int up_alarm_start(FAR const struct timespec *ts)
{
  irqstate_t flags;
  flags = enter_critical_section();

  /* remember the alarm time */

  g_tickless_dev.alarm_set = true;
  g_tickless_dev.alarm     = *ts;

  rtc_prepare_alarm();

  leave_critical_section(flags);

  return OK;
}

/****************************************************************************
 * Name: up_timer_gettime
 ****************************************************************************/

int up_timer_gettime(FAR struct timespec *ts)
{
  uint32_t counter;
  irqstate_t flags;

  flags = enter_critical_section();

  NRF52_RTC_GETCOUNTER(g_tickless_dev.rtc, &counter);
  rtc_counter_to_ts(counter, ts);

  leave_critical_section(flags);

  return OK;
}

/****************************************************************************
 * Name: up_timer_initialize
 ****************************************************************************/

void up_timer_initialize(void)
{
  struct timespec ts;

  g_tickless_dev.rtc = nrf52_rtc_init(CONFIG_NRF52_SYSTIMER_RTC_INSTANCE);
  g_tickless_dev.periods = 0;
  g_tickless_dev.alarm_set = false;

  /* Ensure we have support for the selected RTC instance */

  ASSERT(g_tickless_dev.rtc);

  /* Configure prescaler */

  NRF52_RTC_SETPRE(g_tickless_dev.rtc, 0);

  /* Configure ISR */

  NRF52_RTC_SETISR(g_tickless_dev.rtc, rtc_handler, NULL);

  /* Enable overflow interrupt */

  NRF52_RTC_ENABLEINT(g_tickless_dev.rtc, NRF52_RTC_EVT_OVRFLW);

  /* Start counting */

  NRF52_RTC_CLEAR(g_tickless_dev.rtc);
  NRF52_RTC_ACKINT(g_tickless_dev.rtc, NRF52_RTC_EVT_OVRFLW);
  NRF52_RTC_ACKINT(g_tickless_dev.rtc, NRF52_RTC_EVT_COMPARE0);
  NRF52_RTC_START(g_tickless_dev.rtc);

  /* kick off alarm scheduling */

  ts.tv_sec = ts.tv_nsec = 0;
  nxsched_alarm_expiration(&ts);
}
