/****************************************************************************
 * arch/arm/src/n32h7/n32_oneshot.c
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

#include "n32_oneshot.h"

#ifdef CONFIG_N32H7_ONESHOT

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int n32_oneshot_handler(int irg_num, void *context, void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct n32_oneshot_s *g_oneshot[CONFIG_N32H7_ONESHOT_MAXCHANNELS];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: n32_oneshot_handler
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

static int n32_oneshot_handler(int irg_num, void *context, void *arg)
{
  struct n32_oneshot_s *oneshot = (struct n32_oneshot_s *)arg;
  n32_oneshot_handler_t oneshot_handler;
  void *oneshot_arg;

  tmrinfo("Expired (Interrupt)...\n");
  DEBUGASSERT(oneshot != NULL && oneshot->handler);

  /* The clock was stopped, but not disabled when the CC match occurred.
   * Disable the TC now and disable any further interrupts.
   */

  N32_TIM_SETISR(oneshot->tch, NULL, NULL, N32_TIM_ISR_CC);
  N32_TIM_DISABLEINT(oneshot->tch, N32_ONESHOT_CCIE(oneshot->channel));
  N32_TIM_SETMODE(oneshot->tch, N32_TIM_MODE_DISABLED);
  N32_TIM_ACKINT(oneshot->tch, N32_ONESHOT_CCIF(oneshot->channel));

  /* The timer is no longer running */

  oneshot->running = false;

  /* Forward the event, clearing out any vestiges */

  oneshot_handler  = oneshot->handler;
  oneshot->handler = NULL;
  oneshot_arg      = (void *)oneshot->arg;
  oneshot->arg     = NULL;

  oneshot_handler(oneshot_arg);
  return OK;
}

/****************************************************************************
 * Name: n32_allocate_handler
 *
 * Description:
 *   Allocate a timer callback handler for the oneshot instance.
 *
 * Input Parameters:
 *   oneshot - The state instance the new oneshot timer
 *
 * Returned Value:
 *   Returns zero (OK) on success.  This can only fail if the number of
 *   timers exceeds CONFIG_N32H7_ONESHOT_MAXCHANNELS.
 *
 ****************************************************************************/

static inline int n32_allocate_handler(struct n32_oneshot_s *oneshot)
{
#if CONFIG_N32H7_ONESHOT_MAXCHANNELS > 1
  int ret = -EBUSY;
  int i;

  /* Search for an unused handler */

  for (i = 0; i < CONFIG_N32H7_ONESHOT_MAXCHANNELS; i++)
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
 * Name: n32_oneshot_initialize
 *
 * Description:
 *   Initialize the oneshot timer wrapper
 *
 * Input Parameters:
 *   oneshot    Caller allocated instance of the oneshot state structure
 *   timer      Timer number (1-18, see n32_tim_init mapping)
 *   channel    Capture/compare channel (1-4)
 *   resolution The required resolution of the timer in units of
 *              microseconds.  NOTE that the range is restricted to the
 *              range of uint16_t (excluding zero).
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

int n32_oneshot_initialize(struct n32_oneshot_s *oneshot, int timer,
                           int channel, uint16_t resolution)
{
  uint32_t frequency;

  tmrinfo("timer=%d channel=%d resolution=%u usec, USEC_PER_SEC:%ld\n",
          timer, channel, resolution, USEC_PER_SEC);
  DEBUGASSERT(oneshot && resolution > 0);
  DEBUGASSERT(timer >= 1 && timer <= 18);
  DEBUGASSERT(channel >= 1 && channel <= 4);

  /* Get the TC frequency that corresponds to the requested resolution */

  frequency = USEC_PER_SEC / (uint32_t)resolution;
  tmrinfo("frequency: %" PRIu32 "\n", frequency);
  oneshot->frequency = frequency;

  /* Store timer and channel information */

  oneshot->timer     = timer;
  oneshot->channel   = channel;

  /* Initialize the timer hardware.
   * n32_tim_init() takes the timer number (1-18).
   */

  oneshot->tch = n32_tim_init(timer);
  if (!oneshot->tch)
    {
      tmrerr("ERROR: Failed to allocate TIM%d\n", timer);
      return -EBUSY;
    }

  N32_TIM_SETCLOCK(oneshot->tch, frequency);

  /* Initialize the remaining fields in the state structure. */

  oneshot->running    = false;
  oneshot->handler    = NULL;
  oneshot->arg        = NULL;
  oneshot->period     = 0;

  /* Assign a callback handler to the oneshot */

  return n32_allocate_handler(oneshot);
}

/****************************************************************************
 * Name: n32_oneshot_max_delay
 *
 * Description:
 *   Determine the maximum delay of the one-shot timer (in microseconds)
 *
 ****************************************************************************/

int n32_oneshot_max_delay(struct n32_oneshot_s *oneshot, uint64_t *usec)
{
  int width;

  DEBUGASSERT(oneshot != NULL && usec != NULL);

  width = N32_TIM_GETWIDTH(oneshot->tch);

  tmrinfo("frequency: %" PRIu32 ", USEC_PER_SEC: %ld, width: %d\n",
          oneshot->frequency, USEC_PER_SEC, width);

  if (width == 32)
    {
      *usec = ((uint64_t)UINT32_MAX * (uint64_t)USEC_PER_SEC) /
              (uint64_t)oneshot->frequency;
    }
  else
    {
      *usec = ((uint64_t)UINT16_MAX * (uint64_t)USEC_PER_SEC) /
              (uint64_t)oneshot->frequency;
    }

  return OK;
}

/****************************************************************************
 * Name: n32_oneshot_start
 *
 * Description:
 *   Start the oneshot timer
 *
 * Input Parameters:
 *   oneshot Caller allocated instance of the oneshot state structure.  This
 *           structure must have been previously initialized via a call to
 *           n32_oneshot_initialize();
 *   handler The function to call when when the oneshot timer expires.
 *   arg     An opaque argument that will accompany the callback.
 *   ts      Provides the duration of the one shot timer.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

int n32_oneshot_start(struct n32_oneshot_s *oneshot,
                      n32_oneshot_handler_t handler, void *arg,
                      const struct timespec *ts)
{
  uint64_t usec;
  uint64_t period;
  irqstate_t flags;

  tmrinfo("handler=%p arg=%p, ts=(%lu, %lu)\n", handler, arg,
          (unsigned long)ts->tv_sec, (unsigned long)ts->tv_nsec);
  DEBUGASSERT(oneshot && handler && ts);
  DEBUGASSERT(oneshot->tch);

  /* Was the oneshot already running? */

  flags = enter_critical_section();
  if (oneshot->running)
    {
      /* Yes.. then cancel it */

      tmrinfo("Already running... cancelling\n");
      n32_oneshot_cancel(oneshot, NULL);
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

  N32_TIM_SETISR(oneshot->tch, n32_oneshot_handler, oneshot, N32_TIM_ISR_CC);

  /* Set timer period (auto-reload) to the calculated period */

  oneshot->period = (uint32_t)period;
  N32_TIM_SETPERIOD(oneshot->tch, (uint32_t)period);

  /* Set the compare register using the channel number */

  N32_TIM_SETCOMPARE(oneshot->tch, oneshot->channel, (uint32_t)period);

  /* Start the counter in up-count mode (one-shot pulse mode) */

  N32_TIM_SETMODE(oneshot->tch, N32_TIM_MODE_PULSE);

  /* Acknowledge any pending interrupt and enable the CC interrupt */

  N32_TIM_ACKINT(oneshot->tch, N32_ONESHOT_CCIF(oneshot->channel));
  N32_TIM_ENABLEINT(oneshot->tch, N32_ONESHOT_CCIE(oneshot->channel));

  /* Enable interrupts.  We should get the callback when the interrupt
   * occurs.
   */

  oneshot->running = true;
  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: n32_oneshot_cancel
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
 *           n32_oneshot_initialize();
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

int n32_oneshot_cancel(struct n32_oneshot_s *oneshot,
                       struct timespec *ts)
{
  irqstate_t flags;
  uint32_t count;
  uint32_t period;
  uint64_t usec;

  /* Was the timer running? */

  flags = enter_critical_section();
  if (!oneshot->running)
    {
      /* No.. Just return zero timer remaining and successful cancellation.
       * This function may execute at a high rate with no timer running
       * (as when pre-emption is enabled and disabled).
       */

      if (ts)
        {
          ts->tv_sec  = 0;
          ts->tv_nsec = 0;
        }

      leave_critical_section(flags);
      return OK;
    }

  tmrinfo("Cancelling...\n");

  /* Now we can disable the interrupt and stop the timer. */

  N32_TIM_DISABLEINT(oneshot->tch, N32_ONESHOT_CCIE(oneshot->channel));
  N32_TIM_SETISR(oneshot->tch, NULL, NULL, N32_TIM_ISR_CC);
  N32_TIM_SETMODE(oneshot->tch, N32_TIM_MODE_DISABLED);

  /* Get the current counter value and period */

  count  = N32_TIM_GETCOUNTER(oneshot->tch);
  period = oneshot->period;

  oneshot->running = false;
  oneshot->handler = NULL;
  oneshot->arg     = NULL;
  leave_critical_section(flags);

  /* Did the caller provide us with a location to return the time
   * remaining?
   */

  if (ts)
    {
      /* Calculate remaining time in microseconds */

      if (count < period)
        {
          usec = ((uint64_t)(period - count) * USEC_PER_SEC) /
                 oneshot->frequency;
        }
      else
        {
          usec = 0;
        }

      ts->tv_sec  = (time_t)(usec / USEC_PER_SEC);
      ts->tv_nsec = (long)((usec - (uint64_t)(ts->tv_sec * USEC_PER_SEC)) *
                           NSEC_PER_USEC);

      tmrinfo("Remaining: %lu sec, %lu nsec\n",
              (unsigned long)ts->tv_sec, (unsigned long)ts->tv_nsec);
    }

  return OK;
}

#endif /* CONFIG_N32H7_ONESHOT */
