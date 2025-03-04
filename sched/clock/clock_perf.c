/****************************************************************************
 * sched/clock/clock_perf.c
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

#include <stdint.h>

#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <nuttx/spinlock.h>
#include <nuttx/wdog.h>

#if defined(CONFIG_PERF_OVERFLOW_CORRECTION) && ULONG_MAX != UINT64_MAX

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct perf_s
{
  struct wdog_s wdog;
  spinlock_t lock;
  unsigned long last;
  unsigned long overflow;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct perf_s g_perf;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * perf_update
 ****************************************************************************/

static void perf_update(wdparm_t arg)
{
  clock_t tick = (clock_t)LONG_MAX * TICK_PER_SEC / up_perf_getfreq();

  perf_gettime();
  wd_start((FAR struct wdog_s *)arg, tick, perf_update, arg);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * perf_init
 ****************************************************************************/

void perf_init(void)
{
  FAR struct perf_s *perf = &g_perf;
  clock_t tick = (clock_t)LONG_MAX * TICK_PER_SEC / up_perf_getfreq();

  perf->last = up_perf_gettime();

  /* Periodic check for overflow */

  wd_start(&perf->wdog, tick, perf_update, (wdparm_t)perf);
}

/****************************************************************************
 * perf_gettime
 ****************************************************************************/

clock_t perf_gettime(void)
{
  FAR struct perf_s *perf = &g_perf;
  clock_t now = up_perf_gettime();
  irqstate_t flags = spin_lock_irqsave(&perf->lock);
  clock_t result;

  /* Check if overflow */

  if (now < perf->last)
    {
      perf->overflow++;
    }

  perf->last = now;
  result = (clock_t)now | (clock_t)perf->overflow << 32;
  spin_unlock_irqrestore(&perf->lock, flags);
  return result;
}

/****************************************************************************
 * perf_convert
 ****************************************************************************/

void perf_convert(clock_t elapsed, FAR struct timespec *ts)
{
  unsigned long freq = up_perf_getfreq();

  ts->tv_sec  = elapsed / freq;
  elapsed -= ts->tv_sec * freq;
  ts->tv_nsec = NSEC_PER_SEC * elapsed / freq;
}

/****************************************************************************
 * perf_getfreq
 ****************************************************************************/

unsigned long perf_getfreq(void)
{
  return up_perf_getfreq();
}

#elif defined(CONFIG_ALARM_ARCH) || defined (CONFIG_TIMER_ARCH) || \
      defined(CONFIG_ARCH_PERF_EVENTS)

/****************************************************************************
 * perf_init
 ****************************************************************************/

void perf_init(void)
{
}

/****************************************************************************
 * perf_gettime
 ****************************************************************************/

clock_t perf_gettime(void)
{
  return up_perf_gettime();
}

/****************************************************************************
 * perf_convert
 ****************************************************************************/

void perf_convert(clock_t elapsed, FAR struct timespec *ts)
{
  up_perf_convert(elapsed, ts);
}

/****************************************************************************
 * perf_getfreq
 ****************************************************************************/

unsigned long perf_getfreq(void)
{
  return up_perf_getfreq();
}

#else

/****************************************************************************
 * perf_init
 ****************************************************************************/

void perf_init(void)
{
}

/****************************************************************************
 * perf_gettime
 ****************************************************************************/

clock_t perf_gettime(void)
{
  return clock_systime_ticks();
}

/****************************************************************************
 * perf_convert
 ****************************************************************************/

void perf_convert(clock_t elapsed, FAR struct timespec *ts)
{
  clock_ticks2time(ts, elapsed);
}

/****************************************************************************
 * perf_getfreq
 ****************************************************************************/

unsigned long perf_getfreq(void)
{
  return TICK_PER_SEC;
}

#endif
