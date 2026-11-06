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
#include <nuttx/irq.h>
#include <nuttx/sched.h>
#include <nuttx/wdog.h>

/****************************************************************************
 * Preprocessors
 ****************************************************************************/

/* Use up_perf_*() only when they count.  Else use the system ticks. */

#if defined(CONFIG_ARCH_HAVE_PERF_EVENTS)
#  ifdef CONFIG_ARCH_PERF_EVENTS
#    define USE_UP_PERF /* The arch counter runs only when it is started */
#  endif
#elif defined(CONFIG_ALARM_ARCH) || defined(CONFIG_TIMER_ARCH)
#  define USE_UP_PERF /* arch_alarm.c or arch_timer.c gives up_perf_*() */
#endif

#ifndef CONFIG_ARCH_PERF_EVENTS_USER_ACCESS
#  if defined(USE_UP_PERF) && defined(CONFIG_PERF_OVERFLOW_CORRECTION) && \
      ULONG_MAX != UINT64_MAX

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct perf_s
{
  struct wdog_s wdog;
#ifdef CONFIG_SMP
  struct smp_call_data_s call;
#endif
  unsigned long last;
  clock_t overflow;
  clock_t timeout;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Each CPU has its own counter, so each CPU has its own overflow state */

static struct perf_s g_perf[CONFIG_SMP_NCPUS];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * perf_update_cpu
 ****************************************************************************/

#ifdef CONFIG_SMP
static int perf_update_cpu(FAR void *arg)
{
  perf_gettime();
  return OK;
}
#endif

/****************************************************************************
 * perf_update
 ****************************************************************************/

static void perf_update(wdparm_t arg)
{
  FAR struct perf_s *perf = (FAR struct perf_s *)arg;

#ifdef CONFIG_SMP
  /* The watchdog can run on any CPU: read the counter of the state's CPU */

  nxsched_smp_call_single_async(perf - g_perf, &perf->call);
#else
  perf_gettime();
#endif

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
  irqstate_t flags = up_irq_save();
  FAR struct perf_s *perf = &g_perf[this_cpu()];
  clock_t now = up_perf_gettime();
  clock_t result;

  if (perf->timeout == 0)
    {
      perf->timeout =
        ((clock_t)1 << (CONFIG_ARCH_PERF_COUNT_BITWIDTH - 1)) *
        TICK_PER_SEC / up_perf_getfreq();

#ifdef CONFIG_SMP
      nxsched_smp_call_init(&perf->call, perf_update_cpu, NULL);
#endif

      /* Periodic check for overflow */

      wd_start(&perf->wdog, perf->timeout, perf_update, (wdparm_t)perf);
    }
  else if (now < perf->last)
    {
      perf->overflow++;
    }

  perf->last = now;
  result = now | (perf->overflow << CONFIG_ARCH_PERF_COUNT_BITWIDTH);
  up_irq_restore(flags);
  return result;
}

#  elif defined(USE_UP_PERF)

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
#endif /* !CONFIG_ARCH_PERF_EVENTS_USER_ACCESS */

#ifdef USE_UP_PERF

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
