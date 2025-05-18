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

#ifndef CONFIG_ARCH_HAVE_PERF_EVENTS_USER_ACCESS

/****************************************************************************
 * Preprocessors
 ****************************************************************************/

#  if defined(CONFIG_PERF_OVERFLOW_CORRECTION) && ULONG_MAX != UINT64_MAX

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct perf_s
{
  struct wdog_s wdog;
  spinlock_t lock;
  unsigned long last;
  clock_t overflow;
  clock_t timeout;
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
  FAR struct perf_s *perf = (FAR struct perf_s *)arg;

  perf_gettime();
  wd_start_next(&perf->wdog, perf->timeout, perf_update, arg);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * perf_gettime
 ****************************************************************************/

clock_t perf_gettime(void)
{
  FAR struct perf_s *perf = &g_perf;
  irqstate_t flags = spin_lock_irqsave(&perf->lock);
  clock_t now = up_perf_gettime();
  clock_t result;

  if (perf->timeout == 0)
    {
      perf->timeout =
        ((clock_t)1 << (CONFIG_ARCH_PERF_COUNT_BITWIDTH - 1)) *
        TICK_PER_SEC / up_perf_getfreq();

      /* Periodic check for overflow */

      wd_start(&perf->wdog, perf->timeout, perf_update, (wdparm_t)perf);
    }
  else if (now < perf->last)
    {
      perf->overflow++;
    }

  perf->last = now;
  result = now | (perf->overflow << CONFIG_ARCH_PERF_COUNT_BITWIDTH);
  spin_unlock_irqrestore(&perf->lock, flags);
  return result;
}

#  elif defined(CONFIG_ALARM_ARCH) || defined (CONFIG_TIMER_ARCH) || \
        defined(CONFIG_ARCH_PERF_EVENTS)

/****************************************************************************
 * perf_gettime
 ****************************************************************************/

clock_t perf_gettime(void)
{
  return up_perf_gettime();
}

#  else

/****************************************************************************
 * perf_gettime
 ****************************************************************************/

clock_t perf_gettime(void)
{
  return clock_systime_ticks();
}

#  endif
#endif /* !CONFIG_ARCH_HAVE_PERF_EVENTS_USER_ACCESS */

#if defined(CONFIG_ALARM_ARCH) || defined (CONFIG_TIMER_ARCH) || \
    defined(CONFIG_ARCH_PERF_EVENTS)

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
