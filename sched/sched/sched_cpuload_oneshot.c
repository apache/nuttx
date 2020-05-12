/****************************************************************************
 * sched/sched/sched_cpuload_oneshot.c
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

#include <time.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <nuttx/lib/xorshift128.h>
#include <nuttx/timers/oneshot.h>

#include "clock/clock.h"

#ifdef CONFIG_CPULOAD_ONESHOT

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#if !defined(CONFIG_SCHED_CPULOAD) || !defined(CONFIG_SCHED_CPULOAD_EXTCLK)
#  error CONFIG_SCHED_CPULOAD and CONFIG_SCHED_CPULOAD_EXTCLK must be defined
#endif

/* CONFIG_SCHED_CPULOAD_TICKSPERSEC is the frequency of the external clock
 * source.
 */

#ifndef CONFIG_SCHED_CPULOAD_TICKSPERSEC
#  error CONFIG_SCHED_CPULOAD_TICKSPERSEC not defined
#endif

/* CONFIG_CPULOAD_ENTROPY determines that amount of random "jitter"
 * that will be added to the nominal sample interval.  Specified as a number
 * bits.
 */

#ifndef CONFIG_CPULOAD_ENTROPY
#  warning CONFIG_CPULOAD_ENTROPY not defined
#  define CONFIG_CPULOAD_ENTROPY 0
#endif

/* Calculate the nominal sample interval in microseconds:
 *
 * nominal = (1,000,000 usec/sec) / Frequency cycles/sec) = Period usec/cycle
 */

#define CPULOAD_ONESHOT_NOMINAL      (1000000 / CONFIG_SCHED_CPULOAD_TICKSPERSEC)

#if CPULOAD_ONESHOT_NOMINAL < 1 || CPULOAD_ONESHOT_NOMINAL > 0x7fffffff
#  error CPULOAD_ONESHOT_NOMINAL is out of range
#endif

/* Convert the entropy from number of bits to a numeric value */

#define CPULOAD_ONESHOT_ENTROPY      (1 << CONFIG_CPULOAD_ENTROPY)

#if CPULOAD_ONESHOT_NOMINAL < CPULOAD_ONESHOT_ENTROPY
#  error CPULOAD_ONESHOT_NOMINAL too small for CONFIG_CPULOAD_ENTROPY
#endif

#define CPULOAD_ONESHOT_ENTROPY_MASK (CPULOAD_ONESHOT_ENTROPY - 1)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct sched_oneshot_s
{
  FAR struct oneshot_lowerhalf_s *oneshot;
#if CONFIG_CPULOAD_ENTROPY > 0
  struct xorshift128_state_s prng;
  int32_t maxdelay;
  int32_t error;
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void nxsched_oneshot_start(void);
static void nxsched_oneshot_callback(FAR struct oneshot_lowerhalf_s *lower,
                                     FAR void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct sched_oneshot_s g_sched_oneshot;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  nxsched_oneshot_start
 *
 * Description:
 *   [Re-]start the oneshot timer, applying entropy as configured
 *
 * Input Parameters:
 *   None
 *   lower - An instance of the lower half driver
 *   arg   - The opaque argument provided when the interrupt was registered
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void nxsched_oneshot_start(void)
{
  struct timespec ts;
#if CONFIG_CPULOAD_ENTROPY > 0
  uint32_t entropy;
#endif
  int32_t secs;
  int32_t usecs;

  /* Get the next delay */

#if CONFIG_CPULOAD_ENTROPY > 0
  /* The one shot will be set to this interval:
   *
   *  CPULOAD_ONESHOT_NOMINAL - (CPULOAD_ONESHOT_ENTROPY / 2) + error
   *    + nrand(CPULOAD_ONESHOT_ENTROPY)
   */

  usecs   = (CPULOAD_ONESHOT_NOMINAL - CPULOAD_ONESHOT_ENTROPY / 2) +
            g_sched_oneshot.error;

  /* Add the random value in the range 0..(CPULOAD_ONESHOT_ENTRY - 1) */

  entropy = xorshift128(&g_sched_oneshot.prng);
  usecs  += (int32_t)(entropy & CPULOAD_ONESHOT_ENTROPY_MASK);

  DEBUGASSERT(usecs > 0); /* Check for overflow to negative or zero */

  /* Make sure that the accumulated value does not exceed the maximum delay */

  if (usecs > g_sched_oneshot.maxdelay)
    {
      tmrwarn("WARNING: Truncating\n");
      usecs = g_sched_oneshot.maxdelay;
    }

  /* Save the new error value */

  g_sched_oneshot.error = CPULOAD_ONESHOT_NOMINAL +
                          g_sched_oneshot.error - usecs;
#else
  /* No entropy */

  usecs = CPULOAD_ONESHOT_NOMINAL;
#endif

  /* Then re-start the oneshot timer */

  secs       = usecs / 1000000;
  usecs     -= 100000 * secs;

  ts.tv_sec  = secs;
  ts.tv_nsec = 1000 * usecs;

  DEBUGVERIFY(ONESHOT_START(g_sched_oneshot.oneshot,
                            nxsched_oneshot_callback, NULL, &ts));
}

/****************************************************************************
 * Name:  nxsched_oneshot_callback
 *
 * Description:
 *   This is the callback function that will be invoked when the oneshot
 *   timer expires.
 *
 * Input Parameters:
 *   lower - An instance of the lower half driver
 *   arg   - The opaque argument provided when the interrupt was registered
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void nxsched_oneshot_callback(FAR struct oneshot_lowerhalf_s *lower,
                                     FAR void *arg)
{
  /* Perform CPU load measurements */

#ifdef CONFIG_HAVE_WEAKFUNCTIONS
  if (nxsched_process_cpuload != NULL)
#endif
    {
      nxsched_process_cpuload();
    }

  /* Then restart the oneshot */

  nxsched_oneshot_start();
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  nxsched_oneshot_extclk
 *
 * Description:
 *   Configure to use a oneshot timer as described in
 *   include/nuttx/timers/oneshot.h to provide external clocking to assess
 *   the CPU load.
 *
 * Input Parameters:
 *   lower - An instance of the oneshot timer interface as defined in
 *           include/nuttx/timers/oneshot.h
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void nxsched_oneshot_extclk(FAR struct oneshot_lowerhalf_s *lower)
{
#if CONFIG_CPULOAD_ENTROPY > 0
  struct timespec ts;
#endif

  DEBUGASSERT(lower != NULL && lower->ops != NULL);
  DEBUGASSERT(lower->ops->start != NULL);

#if CONFIG_CPULOAD_ENTROPY > 0
  DEBUGASSERT(lower->ops->max_delay != NULL);

  /* Get the maximum delay */

  DEBUGVERIFY(ONESHOT_MAX_DELAY(lower, &ts));
  if (ts.tv_sec >= 0)
    {
      g_sched_oneshot.maxdelay = INT32_MAX;
    }
  else
    {
      g_sched_oneshot.maxdelay = ts.tv_nsec / 1000;
    }

  tmrinfo("madelay = %ld usec\n", (long)g_sched_oneshot.maxdelay);
  DEBUGASSERT(CPULOAD_ONESHOT_NOMINAL < g_sched_oneshot.maxdelay);

  /* Seed the PRNG */

  g_sched_oneshot.prng.w = 97;
  g_sched_oneshot.prng.x = 101;
  g_sched_oneshot.prng.y = g_sched_oneshot.prng.w << 17;
  g_sched_oneshot.prng.z = g_sched_oneshot.prng.x << 25;
#endif

  /* Then start the oneshot */

  g_sched_oneshot.oneshot = lower;
  nxsched_oneshot_start();
}
#endif
