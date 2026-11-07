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
#include <nuttx/seqlock.h>
#include <nuttx/wdog.h>

#ifdef CONFIG_BUILD_PROTECTED
#  include <nuttx/userspace.h>
#endif

/****************************************************************************
 * Preprocessors
 ****************************************************************************/

#if defined(CONFIG_PERF_OVERFLOW_CORRECTION) && ULONG_MAX != UINT64_MAX && \
    !defined(CONFIG_SMP)

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Without SMP there is a single counter, so the state is the struct perf_s
 * declared in include/nuttx/perf.h.  In a protected build it is embedded in
 * the user-space data block; otherwise it is a global.  The watchdog is the
 * only writer, and the sequence counter lets readers sample the folded value
 * without taking a lock.
 */

static struct wdog_s g_wdog;
static unsigned int g_timeout;

/* The overflow state is one global in every build mode.  A flat build
 * shares the address space with the application, so user space reads it
 * directly; a protected or kernel build keeps it private to the kernel,
 * which is the only writer.  Either way the state lives here.
 */

#if !defined(CONFIG_BUILD_PROTECTED)
struct perf_s g_perf =
{
  .lock = SEQLOCK_INITIALIZER,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * perf_update
 ****************************************************************************/

static void perf_update(wdparm_t arg)
{
  FAR struct perf_s *perf = (FAR struct perf_s *)arg;
  irqstate_t flags = write_seqlock_irqsave(&perf->lock);
  clock_t now = up_perf_gettime();

  if (now < perf->last)
    {
      perf->overflow++;
    }

  perf->last = now;
  write_sequnlock_irqrestore(&perf->lock, flags);
  wd_start_next(&g_wdog, g_timeout, perf_update, arg);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * perf_setup
 ****************************************************************************/

void perf_setup(void)
{
  FAR struct perf_s *perf = &g_perf;

  /* Setup periodic-wdog here, this function can be called after
   * perf_gettime(), but suggested to be called in initialization phase.
   */

  g_timeout =
      ((clock_t)1 << (CONFIG_ARCH_PERF_COUNT_BITWIDTH - 1)) *
      TICK_PER_SEC / up_perf_getfreq();

  /* Periodic check for overflow */

  wd_start(&g_wdog, g_timeout, perf_update, (wdparm_t)perf);
}

/****************************************************************************
 * perf_gettime
 ****************************************************************************/

/* A flat build gets perf_gettime() from libs/libc/sched/clock_perf.c, which
 * reads the very same g_perf.  In a protected or kernel build the state is
 * private to the kernel, so the kernel has to provide the reader here.
 */

#if !defined(CONFIG_BUILD_FLAT)

clock_t noinstrument_function perf_gettime(void)
{
  clock_t overflow;
  clock_t result;
  uint32_t seq;

  do
    {
      seq = read_seqbegin(&g_perf.lock);
      clock_t now = up_perf_gettime();

      if (now < g_perf.last)
        {
          overflow = g_perf.overflow + 1;
        }
      else
        {
          overflow = g_perf.overflow;
        }

      result = now | (overflow << CONFIG_ARCH_PERF_COUNT_BITWIDTH);
    }
  while (read_seqretry(&g_perf.lock, seq));

  return result;
}

#endif /* !CONFIG_BUILD_FLAT */

#elif defined(CONFIG_PERF_OVERFLOW_CORRECTION) && ULONG_MAX != UINT64_MAX

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

clock_t noinstrument_function perf_gettime(void)
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

#elif defined(CONFIG_ALARM_ARCH) || defined (CONFIG_TIMER_ARCH) || \
      defined(CONFIG_ARCH_PERF_EVENTS)

/****************************************************************************
 * perf_gettime
 ****************************************************************************/

clock_t perf_gettime(void)
{
  return up_perf_gettime();
}

#else

/****************************************************************************
 * perf_gettime
 ****************************************************************************/

clock_t perf_gettime(void)
{
  return clock_systime_ticks();
}

#endif

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
