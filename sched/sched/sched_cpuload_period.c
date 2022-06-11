/****************************************************************************
 * sched/sched/sched_cpuload_period.c
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

#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <nuttx/lib/xorshift128.h>
#include <nuttx/power/pm.h>
#include <nuttx/timers/timer.h>

#ifdef CONFIG_CPULOAD_PERIOD

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

#define CPULOAD_PERIOD_NOMINAL (1000000 / CONFIG_SCHED_CPULOAD_TICKSPERSEC)

/* Calculate the systick for one cpuload tick:
 *
 * tick = (Tick_per_sec) / Cpuload tick_per_sec) = Systick for one cpuload
 */

#define CPULOAD_PERIOD_TICKS   (TICK_PER_SEC / CONFIG_SCHED_CPULOAD_TICKSPERSEC)

#if CPULOAD_PERIOD_NOMINAL < 1 || CPULOAD_PERIOD_NOMINAL > 0x7fffffff
#  error CPULOAD_PERIOD_NOMINAL is out of range
#endif

/* Convert the entropy from number of bits to a numeric value */

#define CPULOAD_PERIOD_ENTROPY       (1 << CONFIG_CPULOAD_ENTROPY)

#if CPULOAD_PERIOD_NOMINAL < CPULOAD_PERIOD_ENTROPY
#  error CPULOAD_PERIOD_NOMINAL too small for CONFIG_CPULOAD_ENTROPY
#endif

#define CPULOAD_PERIOD_ENTROPY_MASK  (CPULOAD_PERIOD_ENTROPY - 1)

/****************************************************************************
 * Private Types
 ****************************************************************************/

#if CONFIG_CPULOAD_ENTROPY > 0 || defined(CONFIG_PM)
struct sched_period_s
{
#if CONFIG_CPULOAD_ENTROPY > 0
  struct xorshift128_state_s prng;
  uint32_t maxtimeout;
  int32_t error;
#endif
#ifdef CONFIG_PM
  FAR struct timer_lowerhalf_s *lower;
  struct pm_callback_s pm_cb;
  clock_t idle_start;
  clock_t idle_ticks;
#endif
};
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static bool nxsched_period_callback(FAR uint32_t *next_interval_us,
                                    FAR void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

#if CONFIG_CPULOAD_ENTROPY > 0 || defined(CONFIG_PM)
static struct sched_period_s g_sched_period;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  nxsched_period_callback
 *
 * Description:
 *   This is the callback function that will be invoked when the period
 *   timer expires.
 *
 * Input Parameters:
 *   next_interval_us - The timeout value for next interval
 *   arg   - The opaque argument provided when the callback was registered
 *
 * Returned Value:
 *   return false to stop the timer, true to continue the timing
 *
 ****************************************************************************/

static bool nxsched_period_callback(FAR uint32_t *next_interval_us,
                                    FAR void *arg)
{
  /* Get the next delay */

#if CONFIG_CPULOAD_ENTROPY > 0
  /* The period timer will be set to this interval:
   *
   *  CPULOAD_PERIOD_NOMINAL - (CPULOAD_PERIOD_ENTROPY / 2) + error
   *    + nrand(CPULOAD_PERIOD_ENTROPY)
   */

  *next_interval_us = CPULOAD_PERIOD_NOMINAL - CPULOAD_PERIOD_ENTROPY / 2 +
                      g_sched_period.error;

  /* Add the random value in the range 0..(CPULOAD_PERIOD_ENTROPY - 1) */

  *next_interval_us += xorshift128(&g_sched_period.prng) &
                       CPULOAD_PERIOD_ENTROPY_MASK;

  DEBUGASSERT(*next_interval_us > 0); /* Check for overflow to negative or zero */

  /* Make sure that the accumulated value does not exceed the maximum
   * timeout.
   */

  if (*next_interval_us > g_sched_period.maxtimeout)
    {
      tmrwarn("WARNING: Truncating\n");
      *next_interval_us = g_sched_period.maxtimeout;
    }

  /* Save the new error value */

  g_sched_period.error = CPULOAD_PERIOD_NOMINAL +
                         g_sched_period.error - *next_interval_us;
#endif

  /* Perform CPU load measurements */

#ifdef CONFIG_HAVE_WEAKFUNCTIONS
  if (nxsched_process_cpuload != NULL)
#endif
    {
      nxsched_process_cpuload();
    }

  /* Then continue the timing */

  return true;
}

#ifdef CONFIG_PM
static void nxsched_period_pmnotify(FAR struct pm_callback_s *cb, int domain,
                                    enum pm_state_e pmstate)
{
  if (domain == PM_IDLE_DOMAIN)
    {
      if (pmstate == PM_RESTORE)
        {
          g_sched_period.idle_ticks +=
            clock_systime_ticks() - g_sched_period.idle_start;

          if (g_sched_period.idle_ticks >= CPULOAD_PERIOD_TICKS)
            {
              nxsched_process_cpuload_ticks(
                  g_sched_period.idle_ticks / CPULOAD_PERIOD_TICKS);

              g_sched_period.idle_ticks %= CPULOAD_PERIOD_TICKS;
            }

          g_sched_period.lower->ops->start(g_sched_period.lower);
        }
      else
        {
          g_sched_period.lower->ops->stop(g_sched_period.lower);

          g_sched_period.idle_start = clock_systime_ticks();
        }
    }
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  nxsched_period_extclk
 *
 * Description:
 *   Configure to use a period timer as described in
 *   include/nuttx/timers/timer.h to provide external clocking to assess
 *   the CPU load.
 *
 * Input Parameters:
 *   lower - An instance of the period timer interface as defined in
 *           include/nuttx/timers/timer.h
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void nxsched_period_extclk(FAR struct timer_lowerhalf_s *lower)
{
  DEBUGASSERT(lower != NULL && lower->ops != NULL);
  DEBUGASSERT(lower->ops->setcallback != NULL);
  DEBUGASSERT(lower->ops->settimeout != NULL);
  DEBUGASSERT(lower->ops->start != NULL);

#if CONFIG_CPULOAD_ENTROPY > 0
  DEBUGASSERT(lower->ops->maxtimeout != NULL);

  /* Get the maximum timeout */

  DEBUGVERIFY(lower->ops->maxtimeout(lower, &g_sched_period.maxtimeout));
  tmrinfo("maxtimeout = %lu usec\n", (long)g_sched_period.maxtimeout);
  DEBUGASSERT(CPULOAD_PERIOD_NOMINAL < g_sched_period.maxtimeout);

  /* Seed the PRNG */

  g_sched_period.prng.w = 97;
  g_sched_period.prng.x = 101;
  g_sched_period.prng.y = g_sched_period.prng.w << 17;
  g_sched_period.prng.z = g_sched_period.prng.x << 25;
#endif

#ifdef CONFIG_PM
  g_sched_period.lower = lower;

  /* Register pm notify */

  g_sched_period.pm_cb.notify = nxsched_period_pmnotify;
  pm_register(&g_sched_period.pm_cb);
#endif

  /* Then start the period timer */

  lower->ops->setcallback(lower, nxsched_period_callback, NULL);
  lower->ops->settimeout(lower, CPULOAD_PERIOD_NOMINAL);
  lower->ops->start(lower);
}
#endif
