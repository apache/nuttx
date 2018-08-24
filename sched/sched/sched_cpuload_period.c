/****************************************************************************
 * sched/sched/sched_cpuload_period.c
 *
 *   Copyright (C) 2018 Pinecone Inc. All rights reserved.
 *   Author: Xiang Xiao <xiaoxiang@pinecone.net>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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

/* Calculate the nomimal sample interval in microseconds:
 *
 * nominal = (1,000,000 usec/sec) / Frequency cycles/sec) = Period usec/cycle
 */

#define CPULOAD_PERIOD_NOMINAL       (1000000 / CONFIG_SCHED_CPULOAD_TICKSPERSEC)

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

#if CONFIG_CPULOAD_ENTROPY > 0
struct sched_period_s
{
  struct xorshift128_state_s prng;
  uint32_t maxtimeout;
  int32_t error;
};
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static bool sched_period_callback(FAR uint32_t *next_interval_us,
                                  FAR void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

#if CONFIG_CPULOAD_ENTROPY > 0
static struct sched_period_s g_sched_period;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  sched_period_callback
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

static bool sched_period_callback(FAR uint32_t *next_interval_us,
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

  /* Make sure that the accumulated value does not exceed the maximum timeout */

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
  if (sched_process_cpuload != NULL)
#endif
    {
      sched_process_cpuload();
    }

  /* Then continue the timing */

  return true;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  sched_period_extclk
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

void sched_period_extclk(FAR struct timer_lowerhalf_s *lower)
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

  /* Then start the period timer */

  lower->ops->setcallback(lower, sched_period_callback, NULL);
  lower->ops->settimeout(lower, CPULOAD_PERIOD_NOMINAL);
  lower->ops->start(lower);
}
#endif
