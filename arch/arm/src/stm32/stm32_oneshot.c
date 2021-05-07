/****************************************************************************
 * arch/arm/src/stm32/stm32_oneshot.c
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

#include "stm32_oneshot.h"

#ifdef CONFIG_STM32_ONESHOT

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int stm32_oneshot_handler(int irg_num, void * context, void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct stm32_oneshot_s *g_oneshot[CONFIG_STM32_ONESHOT_MAXTIMERS];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_oneshot_handler
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

static int stm32_oneshot_handler(int irg_num, void * context, void *arg)
{
  struct stm32_oneshot_s * oneshot = (struct stm32_oneshot_s *) arg;
  oneshot_handler_t oneshot_handler;
  void *oneshot_arg;

  tmrinfo("Expired...\n");
  DEBUGASSERT(oneshot != NULL && oneshot->handler);

  /* The clock was stopped, but not disabled when the RC match occurred.
   * Disable the TC now and disable any further interrupts.
   */

  STM32_TIM_SETISR(oneshot->tch, NULL, NULL, 0);
  STM32_TIM_DISABLEINT(oneshot->tch, GTIM_DIER_UIE);
  STM32_TIM_SETMODE(oneshot->tch, STM32_TIM_MODE_DISABLED);
  STM32_TIM_ACKINT(oneshot->tch, GTIM_SR_UIF);

  /* The timer is no longer running */

  oneshot->running = false;

  /* Forward the event, clearing out any vestiges */

  oneshot_handler  = (oneshot_handler_t)oneshot->handler;
  oneshot->handler = NULL;
  oneshot_arg      = (void *)oneshot->arg;
  oneshot->arg     = NULL;

  oneshot_handler(oneshot_arg);
  return OK;
}

/****************************************************************************
 * Name: stm32_allocate_handler
 *
 * Description:
 *   Allocate a timer callback handler for the oneshot instance.
 *
 * Input Parameters:
 *   oneshot - The state instance the new oneshot timer
 *
 * Returned Value:
 *   Returns zero (OK) on success.  This can only fail if the number of
 *   timers exceeds CONFIG_STM32_ONESHOT_MAXTIMERS.
 *
 ****************************************************************************/

static inline int stm32_allocate_handler(struct stm32_oneshot_s *oneshot)
{
#if CONFIG_STM32_ONESHOT_MAXTIMERS > 1
  int ret = -EBUSY;
  int i;

  /* Search for an unused handler */

  sched_lock();
  for (i = 0; i < CONFIG_STM32_ONESHOT_MAXTIMERS; i++)
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

  sched_unlock();
  return ret;

#else
  if (g_oneshot[0] == NULL)
    {
      g_oneshot[0] = oneshot;
      return OK;
    }

  return -EBUSY;
#endif
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_oneshot_initialize
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

int stm32_oneshot_initialize(struct stm32_oneshot_s *oneshot, int chan,
                             uint16_t resolution)
{
  uint32_t frequency;

  tmrinfo("chan=%d resolution=%d usec\n", chan, resolution);
  DEBUGASSERT(oneshot && resolution > 0);

  /* Get the TC frequency the corresponds to the requested resolution */

  frequency = USEC_PER_SEC / (uint32_t)resolution;
  oneshot->frequency = frequency;

  oneshot->tch = stm32_tim_init(chan);
  if (!oneshot->tch)
    {
      tmrerr("ERROR: Failed to allocate TIM%d\n", chan);
      return -EBUSY;
    }

  STM32_TIM_SETCLOCK(oneshot->tch, frequency);

  /* Initialize the remaining fields in the state structure. */

  oneshot->chan       = chan;
  oneshot->running    = false;
  oneshot->handler    = NULL;
  oneshot->arg        = NULL;

  /* Assign a callback handler to the oneshot */

  return stm32_allocate_handler(oneshot);
}

/****************************************************************************
 * Name: stm32_oneshot_max_delay
 *
 * Description:
 *   Determine the maximum delay of the one-shot timer (in microseconds)
 *
 ****************************************************************************/

int stm32_oneshot_max_delay(struct stm32_oneshot_s *oneshot, uint64_t *usec)
{
  DEBUGASSERT(oneshot != NULL && usec != NULL);

  *usec = (uint64_t)(UINT32_MAX / oneshot->frequency) *
          (uint64_t)USEC_PER_SEC;
  return OK;
}

/****************************************************************************
 * Name: stm32_oneshot_start
 *
 * Description:
 *   Start the oneshot timer
 *
 * Input Parameters:
 *   oneshot Caller allocated instance of the oneshot state structure.  This
 *           structure must have been previously initialized via a call to
 *           stm32_oneshot_initialize();
 *   handler The function to call when when the oneshot timer expires.
 *   arg     An opaque argument that will accompany the callback.
 *   ts      Provides the duration of the one shot timer.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

int stm32_oneshot_start(struct stm32_oneshot_s *oneshot,
                        oneshot_handler_t handler, void *arg,
                        const struct timespec *ts)
{
  uint64_t usec;
  uint64_t period;
  irqstate_t flags;

  tmrinfo("handler=%p arg=%p, ts=(%lu, %lu)\n",
         handler, arg, (unsigned long)ts->tv_sec,
         (unsigned long)ts->tv_nsec);
  DEBUGASSERT(oneshot && handler && ts);
  DEBUGASSERT(oneshot->tch);

  /* Was the oneshot already running? */

  flags = enter_critical_section();
  if (oneshot->running)
    {
      /* Yes.. then cancel it */

      tmrinfo("Already running... cancelling\n");
      stm32_oneshot_cancel(oneshot, NULL);
    }

  /* Save the new handler and its argument */

  oneshot->handler = handler;
  oneshot->arg     = arg;

  /* Express the delay in microseconds */

  usec = (uint64_t)ts->tv_sec * USEC_PER_SEC +
         (uint64_t)(ts->tv_nsec / NSEC_PER_USEC);

  /* Get the timer counter frequency and determine the number of counts need
   * to achieve the requested delay.
   *
   *   frequency = ticks / second
   *   ticks     = seconds * frequency
   *             = (usecs * frequency) / USEC_PER_SEC;
   */

  period = (usec * (uint64_t)oneshot->frequency) / USEC_PER_SEC;

  tmrinfo("usec=%llu period=%08llx\n", usec, period);
  DEBUGASSERT(period <= UINT32_MAX);

  /* Set up to receive the callback when the interrupt occurs */

  STM32_TIM_SETISR(oneshot->tch, stm32_oneshot_handler, oneshot, 0);

  /* Set timer period */

  oneshot->period = (uint32_t)period;
  STM32_TIM_SETPERIOD(oneshot->tch, (uint32_t)period);

  /* Start the counter */

  STM32_TIM_SETMODE(oneshot->tch, STM32_TIM_MODE_PULSE);

  STM32_TIM_ACKINT(oneshot->tch, GTIM_SR_UIF);
  STM32_TIM_ENABLEINT(oneshot->tch, GTIM_DIER_UIE);

  /* Enable interrupts.  We should get the callback when the interrupt
   * occurs.
   */

  oneshot->running = true;
  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: stm32_oneshot_cancel
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
 *           stm32_oneshot_initialize();
 *   ts      The location in which to return the time remaining on the
 *           oneshot timer.  A time of zero is returned if the timer is
 *           not running.  ts may be zero in which case the time remaining
 *           is not returned.
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  A call to up_timer_cancel() when
 *   the timer is not active should also return success; a negated errno
 *   value is returned on any failure.
 *
 ****************************************************************************/

int stm32_oneshot_cancel(struct stm32_oneshot_s *oneshot,
                         struct timespec *ts)
{
  irqstate_t flags;
  uint64_t usec;
  uint64_t sec;
  uint64_t nsec;
  uint32_t count;
  uint32_t period;

  /* Was the timer running? */

  flags = enter_critical_section();
  if (!oneshot->running)
    {
      /* No.. Just return zero timer remaining and successful cancellation.
       * This function may execute at a high rate with no timer running
       * (as when pre-emption is enabled and disabled).
       */

      ts->tv_sec  = 0;
      ts->tv_nsec = 0;
      leave_critical_section(flags);
      return OK;
    }

  /* Yes.. Get the timer counter and period registers and stop the counter.
   * If the counter expires while we are doing this, the counter clock will
   * be stopped, but the clock will not be disabled.
   *
   * The expected behavior is that the counter register will freezes at
   * a value equal to the RC register when the timer expires.  The counter
   * should have values between 0 and RC in all other cased.
   *
   * REVISIT:  This does not appear to be the case.
   */

  tmrinfo("Cancelling...\n");

  count  = STM32_TIM_GETCOUNTER(oneshot->tch);
  period = oneshot->period;

  /* Now we can disable the interrupt and stop the timer. */

  STM32_TIM_DISABLEINT(oneshot->tch, GTIM_DIER_UIE);
  STM32_TIM_SETISR(oneshot->tch, NULL, NULL, 0);
  STM32_TIM_SETMODE(oneshot->tch, STM32_TIM_MODE_DISABLED);

  oneshot->running = false;
  oneshot->handler = NULL;
  oneshot->arg     = NULL;
  leave_critical_section(flags);

  /* Did the caller provide us with a location to return the time
   * remaining?
   */

  if (ts)
    {
      /* Yes.. then calculate and return the time remaining on the
       * oneshot timer.
       */

      tmrinfo("period=%lu count=%lu\n",
             (unsigned long)period, (unsigned long)count);

      /* REVISIT: I am not certain why the timer counter value sometimes
       * exceeds RC.  Might be a bug, or perhaps the counter does not stop
       * in all cases.
       */

      if (count >= period)
        {
          /* No time remaining (?) */

          ts->tv_sec  = 0;
          ts->tv_nsec = 0;
        }
      else
        {
          /* The total time remaining is the difference.  Convert that
           * to units of microseconds.
           *
           *   frequency = ticks / second
           *   seconds   = ticks * frequency
           *   usecs     = (ticks * USEC_PER_SEC) / frequency;
           */

          usec        = (((uint64_t)(period - count)) * USEC_PER_SEC) /
                        oneshot->frequency;

          /* Return the time remaining in the correct form */

          sec         = usec / USEC_PER_SEC;
          nsec        = ((usec) - (sec * USEC_PER_SEC)) * NSEC_PER_USEC;

          ts->tv_sec  = (time_t)sec;
          ts->tv_nsec = (unsigned long)nsec;
        }

      tmrinfo("remaining (%lu, %lu)\n",
             (unsigned long)ts->tv_sec, (unsigned long)ts->tv_nsec);
    }

  return OK;
}

#endif /* CONFIG_STM32_ONESHOT */
