/****************************************************************************
 * arch/x86_64/src/intel64/intel64_oneshot.c
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
#include <stdint.h>
#include <stdbool.h>
#include <sched.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/clock.h>
#include <nuttx/spinlock.h>

#include "intel64_hpet.h"
#include "intel64_oneshot.h"

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int intel64_oneshot_handler(int irg_num, void * context, void *arg);
static int intel64_allocate_handler(struct intel64_oneshot_s *oneshot);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct intel64_oneshot_s *g_oneshot[CONFIG_INTEL64_ONESHOT_MAXTIMERS];
static spinlock_t                g_oneshot_spin;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: intel64_oneshot_handler
 *
 * Description:
 *   Common timer interrupt callback.  When any oneshot timer interrupt
 *   expires, this function will be called.  It will forward the call to
 *   the next level up.
 *
 * Input Parameters:
 *   oneshot - The state associated with the expired timer
 *
 * Returned Value:
 *   Always returns OK
 *
 ****************************************************************************/

static int intel64_oneshot_handler(int irg_num, void * context, void *arg)
{
  struct intel64_oneshot_s *oneshot = (struct intel64_oneshot_s *)arg;
  void                     *oneshot_arg;
  oneshot_handler_t         oneshot_handler;

  DEBUGASSERT(oneshot != NULL);

  /* Additional check for spurious interrupts. We can't check interrupt
   * status here, because it doesn't work for FSB which is edge tirggered
   */

  if (oneshot->running)
    {
      DEBUGASSERT(oneshot->handler);
      tmrinfo("Expired...\n");

#ifndef CONFIG_INTEL64_HPET_FSB
      /* Disable any further interrupts. */

      INTEL64_TIM_DISABLEINT(oneshot->tch, oneshot->chan);
      INTEL64_TIM_SETISR(oneshot->tch, oneshot->chan, NULL, NULL, false);
      INTEL64_TIM_ACKINT(oneshot->tch, oneshot->chan);
#endif

      /* The timer is no longer running */

      oneshot->running = false;

      /* Forward the event, clearing out any vestiges */

      oneshot_handler  = (oneshot_handler_t)oneshot->handler;
      oneshot->handler = NULL;
      oneshot_arg      = (void *)oneshot->arg;
      oneshot->arg     = NULL;

      oneshot_handler(oneshot_arg);
    }
  else
    {
      tmrinfo("Spurious...\n");
    }

  return OK;
}

/****************************************************************************
 * Name: intel64_allocate_handler
 *
 * Description:
 *   Allocate a timer callback handler for the oneshot instance.
 *
 * Input Parameters:
 *   oneshot - The state instance the new oneshot timer
 *
 * Returned Value:
 *   Returns zero (OK) on success.  This can only fail if the number of
 *   timers exceeds CONFIG_INTEL64_ONESHOT_MAXTIMERS.
 *
 ****************************************************************************/

static int intel64_allocate_handler(struct intel64_oneshot_s *oneshot)
{
  int ret = -EBUSY;
  int i;

  /* Search for an unused handler */

  for (i = 0; i < CONFIG_INTEL64_ONESHOT_MAXTIMERS; i++)
    {
      /* Is this handler available? */

      if (g_oneshot[i] == NULL)
        {
          /* Yes... assign it to this oneshot */

          g_oneshot[i]   = oneshot;
          oneshot->cbndx = i;
          ret            = OK;
          break;
        }
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: intel64_oneshot_initialize
 *
 * Description:
 *   Initialize the oneshot timer wrapper
 *
 * Input Parameters:
 *   oneshot    Caller allocated instance of the oneshot state structure
 *   chan       Timer counter channel to be used.
 *   resolution The required resolution of the timer in units of
 *              microseconds.  NOTE that the range is restricted to the
 *              range of uint16_t (excluding zero).
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

int intel64_oneshot_initialize(struct intel64_oneshot_s *oneshot, int chan,
                               uint16_t resolution)
{
  /* HPET resolution can't be changed, but it is always in terms of ns */

  UNUSED(resolution);

  /* Get timer */

  oneshot->tch = intel64_hpet_init(CONFIG_INTEL64_HPET_BASE);
  if (oneshot->tch == NULL)
    {
      tmrerr("ERROR: Failed to allocate HPET timer %d\n", chan);
      return -EBUSY;
    }

  /* Get the timer period */

  oneshot->period = INTEL64_TIM_GETPERIOD(oneshot->tch);
  oneshot->frequency = 1e15 / oneshot->period;

  /* Calculations in intel64_oneshot_current() requires that HPET
   * frequency is less than 1GHz
   */

  DEBUGASSERT(oneshot->frequency < 1000000000);

  /* Initialize the remaining fields in the state structure. */

  oneshot->chan       = chan;
  oneshot->running    = false;
  oneshot->handler    = NULL;
  oneshot->arg        = NULL;

#ifdef CONFIG_INTEL64_HPET_FSB
  /* IMPORTENT: HPET in edge triggered mode is broken on some hardware
   * and generate spurious interrupts when we enable timer. On the other
   * hand FSB works only with edge triggered mode.
   *
   * We use the following work around for this:
   * 1. enable interrupts now for FSB so we can catch initial spurious
   *    interrupt.
   * 2. NEVER disable interrupts, so we avoid spurious interrupt when we
   *    re-enabled interrupt.
   * 3. The previous step requires that we must filter out unwanted
   *    interrupts for 32-bit mode HPET, that will happen with HPET
   *    period interval.
   */

  INTEL64_TIM_SETISR(oneshot->tch, oneshot->chan, intel64_oneshot_handler,
                     oneshot, false);
  INTEL64_TIM_SETCOMPARE(oneshot->tch, oneshot->chan, 0);
  INTEL64_TIM_ENABLEINT(oneshot->tch, oneshot->chan);
#endif

  /* Assign a callback handler to the oneshot */

  return intel64_allocate_handler(oneshot);
}

/****************************************************************************
 * Name: intel64_oneshot_max_delay
 *
 * Description:
 *   Determine the maximum delay of the one-shot timer (in microseconds)
 *
 ****************************************************************************/

int intel64_oneshot_max_delay(struct intel64_oneshot_s *oneshot,
                              uint64_t *usec)
{
  DEBUGASSERT(oneshot != NULL && usec != NULL);

  *usec = ((uint64_t)(UINT32_MAX / oneshot->frequency) *
           (uint64_t)USEC_PER_SEC);
  return OK;
}

/****************************************************************************
 * Name: intel64_oneshot_start
 *
 * Description:
 *   Start the oneshot timer
 *
 * Input Parameters:
 *   oneshot Caller allocated instance of the oneshot state structure.  This
 *           structure must have been previously initialized via a call to
 *           intel64_oneshot_initialize();
 *   handler The function to call when when the oneshot timer expires.
 *   arg     An opaque argument that will accompany the callback.
 *   ts      Provides the duration of the one shot timer.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

int intel64_oneshot_start(struct intel64_oneshot_s *oneshot,
                          oneshot_handler_t handler, void *arg,
                          const struct timespec *ts)
{
  uint64_t   usec    = 0;
  uint64_t   compare = 0;
  irqstate_t flags;

  tmrinfo("handler=%p arg=%p, ts=(%lu, %lu)\n",
         handler, arg, (unsigned long)ts->tv_sec,
         (unsigned long)ts->tv_nsec);

  DEBUGASSERT(oneshot && handler && ts);
  DEBUGASSERT(oneshot->tch);

  /* Was the oneshot already running? */

  flags = spin_lock_irqsave(&g_oneshot_spin);
  if (oneshot->running)
    {
      /* Yes.. then cancel it */

      tmrinfo("Already running... cancelling\n");
      intel64_oneshot_cancel(oneshot, NULL);
    }

  /* Save the new handler and its argument */

  oneshot->handler = handler;
  oneshot->arg     = arg;

  /* Express the delay in microseconds */

  usec = (uint64_t)ts->tv_sec * USEC_PER_SEC +
         (uint64_t)(ts->tv_nsec / NSEC_PER_USEC);

  /* HPET use free runnin up-counter and a comparators which generate events
   * only on a equal event. This can results in event miss if we set too
   * small delay. In that case we just set a minimum value for delay that
   * seem to work.
   */

  if (usec < CONFIG_INTEL64_HPET_MIN_DELAY)
    {
      usec = CONFIG_INTEL64_HPET_MIN_DELAY;
    }

  /* Get the timer counter frequency and determine the number of counts need
   * to achieve the requested delay.
   *
   *   frequency = ticks / second
   *   ticks     = seconds * frequency
   *             = (usecs * frequency) / USEC_PER_SEC;
   */

  compare = (usec * (uint64_t)oneshot->frequency) / USEC_PER_SEC;

#ifndef CONFIG_INTEL64_HPET_FSB
  /* Set up to receive the callback when the interrupt occurs */

  INTEL64_TIM_SETISR(oneshot->tch, oneshot->chan, intel64_oneshot_handler,
                     oneshot, false);
#endif

  /* Set comparator ahed of the current counter */

  compare += INTEL64_TIM_GETCOUNTER(oneshot->tch);
  INTEL64_TIM_SETCOMPARE(oneshot->tch, oneshot->chan, compare);

#ifndef CONFIG_INTEL64_HPET_FSB
  /* Enable interrupts.  We should get the callback when the interrupt
   * occurs.
   */

  INTEL64_TIM_ENABLEINT(oneshot->tch, oneshot->chan);
#endif

  oneshot->running = true;
  spin_unlock_irqrestore(&g_oneshot_spin, flags);
  return OK;
}

/****************************************************************************
 * Name: intel64_oneshot_cancel
 *
 * Description:
 *   Cancel the oneshot timer and return the time remaining on the timer.
 *
 *   NOTE: This function may execute at a high rate with no timer running (as
 *   when pre-emption is enabled and disabled).
 *
 * Input Parameters:
 *   oneshot Caller allocated instance of the oneshot state structure.  This
 *           structure must have been previously initialized via a call to
 *           intel64_oneshot_initialize();
 *   ts      The location in which to return the time remaining on the
 *           oneshot timer.  A time of zero is returned if the timer is
 *           not running.
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  A call to up_timer_cancel() when
 *   the timer is not active should also return success; a negated errno
 *   value is returned on any failure.
 *
 ****************************************************************************/

int intel64_oneshot_cancel(struct intel64_oneshot_s *oneshot,
                           struct timespec *ts)
{
  irqstate_t flags;
  uint64_t   usec;
  uint64_t   sec;
  uint64_t   nsec;
  uint64_t   counter;
  uint64_t   compare;

  /* Was the timer running? */

  flags = spin_lock_irqsave(&g_oneshot_spin);
  if (!oneshot->running)
    {
      /* No.. Just return zero timer remaining and successful cancellation.
       * This function may execute at a high rate with no timer running
       * (as when pre-emption is enabled and disabled).
       */

      ts->tv_sec  = 0;
      ts->tv_nsec = 0;
      spin_unlock_irqrestore(&g_oneshot_spin, flags);
      return OK;
    }

  /* Yes.. Get the timer counter and compare registers and stop the
   * counter.
   */

  tmrinfo("Cancelling...\n");

  counter = INTEL64_TIM_GETCOUNTER(oneshot->tch);
  compare = INTEL64_TIM_GETCOMPARE(oneshot->tch, oneshot->chan);

#ifndef CONFIG_INTEL64_HPET_FSB
  /* Now we can disable the interrupt and stop the timer. */

  INTEL64_TIM_DISABLEINT(oneshot->tch, oneshot->chan);
  INTEL64_TIM_SETISR(oneshot->tch, oneshot->chan, NULL, NULL, false);
#endif

  oneshot->running = false;
  oneshot->handler = NULL;
  oneshot->arg     = NULL;
  spin_unlock_irqrestore(&g_oneshot_spin, flags);

  /* Did the caller provide us with a location to return the time
   * remaining?
   */

  if (ts)
    {
      /* Yes.. then calculate and return the time remaining on the
       * oneshot timer.
       */

      /* The total time remaining is the difference.  Convert that
       * to units of microseconds.
       *
       *   frequency = ticks / second
       *   seconds   = ticks * frequency
       *   usecs     = (ticks * USEC_PER_SEC) / frequency;
       */

      usec        = ((compare - counter) * USEC_PER_SEC) /
                     oneshot->frequency;

      /* Return the time remaining in the correct form */

      sec         = usec / USEC_PER_SEC;
      nsec        = ((usec) - (sec * USEC_PER_SEC)) * NSEC_PER_USEC;

      ts->tv_sec  = (time_t)sec;
      ts->tv_nsec = (unsigned long)nsec;

      tmrinfo("remaining (%lu, %lu)\n",
             (unsigned long)ts->tv_sec, (unsigned long)ts->tv_nsec);
    }

  return OK;
}

/****************************************************************************
 * Name: intel64_oneshot_current
 *
 * Description:
 *   Get the current time.
 *
 ****************************************************************************/

int intel64_oneshot_current(struct intel64_oneshot_s *oneshot,
                            uint64_t *usec)
{
  uint64_t counter;

  /* Get the current counter value */

  counter = INTEL64_TIM_GETCOUNTER(oneshot->tch);

  /* Use nano seconds to avoid 64-bit overflow */

  *usec = (counter * (NSEC_PER_SEC / oneshot->frequency)) / 1000;

  return OK;
}
