/****************************************************************************
 * arch/arm/src/nrf53/nrf53_tickless_rtc.c
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
#include "hardware/nrf53_rtc.h"
#include "nrf53_rtc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Check configuration */

#ifdef CONFIG_TIMER_ARCH
#  error CONFIG_TIMER_ARCH must be not set
#endif

/* Check corresponding RTC support */

#if (CONFIG_NRF53_SYSTIMER_RTC_INSTANCE == 0) && !defined(CONFIG_NRF53_RTC0)
#  error "Support for RTC0 is not enabled"
#elif (CONFIG_NRF53_SYSTIMER_RTC_INSTANCE == 1) && !defined(CONFIG_NRF53_RTC1)
#  error "Support for RTC1 is not enabled"
#endif

#define NRF53_RTC_PERIOD   (512)
#define NRF53_RTC_FREQ     (32768)

/* A COMPARE event is guaranteed only for a CC value at least two counts
 * ahead of the COUNTER (nRF52832_PS_v1.4, p. 245; the nRF5340 RTC alike).
 */

#define NRF53_RTC_MIN_DELAY (2)

/* To convert from counter to uS we split the counter into one second worth
 * of counts (32768) and a fractional part we can safely multiply first
 * by (USEC_PER_SEC/8) and still be within 32 bit value
 */

#define COUNTER_TO_USEC(c) ((c / 32768) * USEC_PER_SEC) + \
                            (((c % 32768) * (USEC_PER_SEC / 8)) / (32768 / 8))

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct nrf53_tickless_dev_s
{
  struct nrf53_rtc_dev_s *rtc; /* nrf53 RTC driver */
  uint32_t periods;            /* how many times the timer overflowed */
  bool alarm_set;              /* is the alarm set? */
  struct timespec alarm;       /* absolute time of alarm */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int rtc_handler(int irq, void *context, void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct nrf53_tickless_dev_s g_tickless_dev;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Read the COUNTER and the absolute time it stands for.  Called with
 * interrupts disabled, so an overflow may not have been counted in
 * 'periods' yet: its OVRFLW event is then still set.  A COUNTER in its
 * lower half was read after that overflow, one in its upper half before.
 */

static uint32_t rtc_now(struct timespec *now)
{
  const uint32_t rtc_base = NRF53_RTC_GETBASE(g_tickless_dev.rtc);
  uint32_t periods = g_tickless_dev.periods;
  uint32_t counter;
  uint32_t usec;

  counter = NRF53_RTC_GETCOUNTER_REG(rtc_base);
  if (getreg32(rtc_base + NRF53_RTC_EVENTS_OVRFLW_OFFSET) != 0 &&
      counter <= RTC_COUNTER_MASK / 2)
    {
      periods++;
    }

  usec = COUNTER_TO_USEC(counter);
  now->tv_sec  = usec / USEC_PER_SEC + periods * NRF53_RTC_PERIOD;
  now->tv_nsec = (usec % USEC_PER_SEC) * NSEC_PER_USEC;

  return counter;
}

/* Convert a time shorter than one RTC period to counts, rounding up so
 * that an alarm never fires early.  32768 / 1000000 = 512 / 15625.
 */

static inline uint32_t rtc_ts_to_counts(const struct timespec *ts)
{
  uint32_t usec = (ts->tv_nsec + NSEC_PER_USEC - 1) / NSEC_PER_USEC;

  return (uint32_t)ts->tv_sec * NRF53_RTC_FREQ +
         (usec * 512 + 15624) / 15625;
}

static void rtc_prepare_alarm(void)
{
  struct timespec now;
  struct timespec delta;
  uint32_t counter;
  uint32_t elapsed;
  uint32_t counts;
  const uint32_t rtc_base = NRF53_RTC_GETBASE(g_tickless_dev.rtc);

  /* Time left until the alarm, zero if it has passed */

  counter = rtc_now(&now);
  clock_timespec_subtract(&g_tickless_dev.alarm, &now, &delta);

  /* An alarm further away is prepared again on a later overflow */

  if (delta.tv_sec >= NRF53_RTC_PERIOD)
    {
      return;
    }

  counts = rtc_ts_to_counts(&delta);
  if (counts < NRF53_RTC_MIN_DELAY)
    {
      counts = NRF53_RTC_MIN_DELAY;
    }

  /* Move CC0 as far ahead as it goes before enabling its interrupt */

  NRF53_RTC_SETCC_REG(rtc_base, 0, counter - 1);
  NRF53_RTC_ENABLEINT(g_tickless_dev.rtc, NRF53_RTC_EVT_COMPARE0);

  /* Set CC0 'counts' after 'counter'.  The COUNTER has kept running since
   * it was read: if it is now less than NRF53_RTC_MIN_DELAY short of CC0,
   * move CC0 to the earliest guaranteed match and check again.  Both
   * distances are taken modulo the 24-bit COUNTER, so this holds across
   * an overflow.
   */

  for (; ; )
    {
      NRF53_RTC_SETCC_REG(rtc_base, 0,
                          (counter + counts) & RTC_COUNTER_MASK);
      elapsed = (NRF53_RTC_GETCOUNTER_REG(rtc_base) - counter) &
                RTC_COUNTER_MASK;
      if (elapsed + NRF53_RTC_MIN_DELAY <= counts)
        {
          break;
        }

      counts = elapsed + NRF53_RTC_MIN_DELAY;
    }
}

/****************************************************************************
 * Name: rtc_cancel_ack
 ****************************************************************************/

static void rtc_cancel_ack(void)
{
  irqstate_t flags;

  flags = enter_critical_section();

  NRF53_RTC_DISABLEINT(g_tickless_dev.rtc, NRF53_RTC_EVT_COMPARE0);
  NRF53_RTC_ACKINT(g_tickless_dev.rtc, NRF53_RTC_EVT_COMPARE0);
  g_tickless_dev.alarm_set = false;

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: rtc_handler
 ****************************************************************************/

static int rtc_handler(int irq, void *context, void *arg)
{
  irqstate_t flags;

  flags = enter_critical_section();

  /* if the timer wrapped-around */

  if (NRF53_RTC_CHECKINT(g_tickless_dev.rtc, NRF53_RTC_EVT_OVRFLW))
    {
      /* ack interrupt */

      NRF53_RTC_ACKINT(g_tickless_dev.rtc, NRF53_RTC_EVT_OVRFLW);

      /* count one more period */

      g_tickless_dev.periods++;

      /* check if the currently set alarm is to fire in this new period */

      if (g_tickless_dev.alarm_set)
        {
          rtc_prepare_alarm();
        }
    }

  /* if the compare event fired */

  if (NRF53_RTC_CHECKINT(g_tickless_dev.rtc, NRF53_RTC_EVT_COMPARE0))
    {
      /* cancel the alarm and ack the event */

      rtc_cancel_ack();

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
  irqstate_t flags;

  flags = enter_critical_section();

  NRF53_RTC_DISABLEINT(g_tickless_dev.rtc, NRF53_RTC_EVT_COMPARE0);
  rtc_now(ts);

  NRF53_RTC_ACKINT(g_tickless_dev.rtc, NRF53_RTC_EVT_COMPARE0);
  g_tickless_dev.alarm_set = false;

  leave_critical_section(flags);

  return OK;
}

/****************************************************************************
 * Name: up_alarm_start
 ****************************************************************************/

int up_alarm_start(const struct timespec *ts)
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

int up_timer_gettime(struct timespec *ts)
{
  irqstate_t flags;

  flags = enter_critical_section();
  rtc_now(ts);
  leave_critical_section(flags);

  return OK;
}

/****************************************************************************
 * Name: up_timer_initialize
 ****************************************************************************/

void up_timer_initialize(void)
{
  memset(&g_tickless_dev, 0, sizeof(struct nrf53_tickless_dev_s));

  g_tickless_dev.rtc = nrf53_rtc_init(CONFIG_NRF53_SYSTIMER_RTC_INSTANCE);

  /* Ensure we have support for the selected RTC instance */

  ASSERT(g_tickless_dev.rtc);

  /* Configure prescaler */

  NRF53_RTC_SETPRE(g_tickless_dev.rtc, 0);

  /* Configure ISR */

  NRF53_RTC_SETISR(g_tickless_dev.rtc, rtc_handler, NULL);

  /* Enable overflow interrupt */

  NRF53_RTC_ENABLEINT(g_tickless_dev.rtc, NRF53_RTC_EVT_OVRFLW);

  /* Start counting */

  NRF53_RTC_CLEAR(g_tickless_dev.rtc);
  NRF53_RTC_ACKINT(g_tickless_dev.rtc, NRF53_RTC_EVT_OVRFLW);
  NRF53_RTC_ACKINT(g_tickless_dev.rtc, NRF53_RTC_EVT_COMPARE0);
  NRF53_RTC_START(g_tickless_dev.rtc);

  /* kick off alarm scheduling */

  nxsched_process_timer();
}
