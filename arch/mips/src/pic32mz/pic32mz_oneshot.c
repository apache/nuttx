/****************************************************************************
 * arch/mips/src/pic32mz/pic32mz_oneshot.c
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

#include "pic32mz_oneshot.h"

#ifdef CONFIG_PIC32MZ_ONESHOT

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int pic32mz_oneshot_handler(int irg_num, void * context, void *arg);
static inline
int pic32mz_allocate_handler(struct pic32mz_oneshot_s *oneshot);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct pic32mz_oneshot_s *g_oneshot[CONFIG_PIC32MZ_ONESHOT_MAXTIMERS];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pic32mz_oneshot_handler
 *
 * Description:
 *   Common timer interrupt callback.  When any oneshot timer interrupt
 *   expires, this function will be called.  It will forward the call to
 *   the next level up.
 *
 * Input Parameters:
 *   oneshot   The state associated with the expired timer
 *
 * Returned Value:
 *   Always returns OK
 *
 ****************************************************************************/

static int pic32mz_oneshot_handler(int irg_num, void * context, void *arg)
{
  struct pic32mz_oneshot_s * oneshot = (struct pic32mz_oneshot_s *) arg;
  oneshot_handler_t oneshot_handler;
  void *oneshot_arg;

  tmrinfo("Expired...\n");
  DEBUGASSERT(oneshot != NULL && oneshot->handler);

  /* Stop the timer and disable any further interrupts */

  PIC32MZ_TIMER_ACKINT(oneshot->timer);
  PIC32MZ_TIMER_SETISR(oneshot->timer, NULL, NULL);
  PIC32MZ_TIMER_STOP(oneshot->timer);

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
 * Name: pic32mz_allocate_handler
 *
 * Description:
 *   Allocate a timer callback handler for the oneshot instance.
 *
 * Input Parameters:
 *   oneshot   The state instance the new oneshot timer
 *
 * Returned Value:
 *   Returns zero (OK) on success.  This can only fail if the number of
 *   timers exceeds CONFIG_PIC32MZ_ONESHOT_MAXTIMERS.
 *
 ****************************************************************************/

static inline int pic32mz_allocate_handler(struct pic32mz_oneshot_s *oneshot)
{
#if CONFIG_PIC32MZ_ONESHOT_MAXTIMERS > 1
  int ret = -EBUSY;
  int i;

  /* Search for an unused handler */

  sched_lock();
  for (i = 0; i < CONFIG_PIC32MZ_ONESHOT_MAXTIMERS; i++)
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
 * Name: pic32mz_oneshot_initialize
 *
 * Description:
 *   Initialize the oneshot timer wrapper
 *
 * Input Parameters:
 *   oneshot     Caller allocated instance of the oneshot state structure
 *   chan        Timer counter channel to be used.
 *   resolution  The required resolution of the timer in units of
 *               microseconds. NOTE that the range is restricted to the
 *               range of uint16_t (excluding zero).
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

int pic32mz_oneshot_initialize(struct pic32mz_oneshot_s *oneshot, int chan,
                               uint16_t resolution)
{
  uint32_t freq;

  tmrinfo("chan=%d resolution=%d usec\n", chan, resolution);
  DEBUGASSERT(oneshot && resolution > 0);

  oneshot->timer = pic32mz_timer_init(chan);
  if (!oneshot->timer)
    {
      tmrerr("ERROR: Failed to allocate timer%d\n", chan);
      return -EBUSY;
    }

  /* Get the timer's frequency that corresponds to the requested resolution */

  freq = USEC_PER_SEC / (uint32_t)resolution;
  tmrinfo("Setting frequency=%luHz\n", freq);

  if (!PIC32MZ_TIMER_SETFREQ(oneshot->timer, freq))
    {
      tmrerr("Cannot set frequency=%luHz\n", freq);
      return -EAGAIN;
    }

  /* Initialize the remaining fields in the state structure.
   *
   * The timer's frequency might not be the same as requested,
   * due to the lack of prescale values.  Get it from the driver.
   */

  oneshot->freq     = PIC32MZ_TIMER_GETFREQ(oneshot->timer);
  oneshot->width    = PIC32MZ_TIMER_GETWIDTH(oneshot->timer);
  oneshot->chan     = chan;
  oneshot->running  = false;
  oneshot->handler  = NULL;
  oneshot->arg      = NULL;

  /* Assign a callback handler to the oneshot */

  return pic32mz_allocate_handler(oneshot);
}

/****************************************************************************
 * Name: pic32mz_oneshot_max_delay
 *
 * Description:
 *   Determine the maximum delay of the one-shot timer (in microseconds)
 *
 ****************************************************************************/

int pic32mz_oneshot_max_delay(struct pic32mz_oneshot_s *oneshot,
                              uint64_t *usec)
{
  uint32_t maxticks;

  maxticks = (1ull << oneshot->width) - 1ul;

  DEBUGASSERT(oneshot != NULL && usec != NULL);
  tmrinfo("width=%u freq=%lu max ticks=%lu\n",
           oneshot->width, oneshot->freq, maxticks);

  *usec = (maxticks / oneshot->freq) * USEC_PER_SEC;

  tmrinfo("max delay %lu\n", *usec);

  return OK;
}

/****************************************************************************
 * Name: pic32mz_oneshot_start
 *
 * Description:
 *   Start the oneshot timer
 *
 * Input Parameters:
 *   oneshot  Caller allocated instance of the oneshot state structure.
 *            This structure must have been previously initialized via
 *            a call to pic32mz_oneshot_initialize();
 *   handler  The function to call when when the oneshot timer expires.
 *   arg      An opaque argument that will accompany the callback.
 *   ts       Provides the duration of the one shot timer.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

int pic32mz_oneshot_start(struct pic32mz_oneshot_s *oneshot,
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
  DEBUGASSERT(oneshot->timer);

  flags = enter_critical_section();

  /* Was the oneshot already running? */

  if (oneshot->running)
    {
      /* Yes.. then cancel it */

      tmrinfo("Already running... cancelling\n");
      pic32mz_oneshot_cancel(oneshot, NULL);
    }

  /* Save the new handler and its argument */

  oneshot->handler = handler;
  oneshot->arg     = arg;

  /* Express the delay in microseconds */

  usec = (uint64_t)ts->tv_sec * USEC_PER_SEC +
         (uint64_t)(ts->tv_nsec / NSEC_PER_USEC);

  period = (usec * (uint64_t)oneshot->freq) / USEC_PER_SEC;

  tmrinfo("usec=%llu period=%08llx\n", usec, period);
  DEBUGASSERT(period <= ((1ull << oneshot->width) - 1ul));

  /* Set timer period */

  oneshot->period = (uint32_t)period;
  PIC32MZ_TIMER_SETPERIOD(oneshot->timer, (uint32_t)period);

  /* Set up to receive the callback and start the timer */

  PIC32MZ_TIMER_ACKINT(oneshot->timer);
  PIC32MZ_TIMER_SETISR(oneshot->timer, pic32mz_oneshot_handler, oneshot);
  PIC32MZ_TIMER_START(oneshot->timer);

  oneshot->running = true;
  leave_critical_section(flags);

  return OK;
}

/****************************************************************************
 * Name: pic32mz_oneshot_cancel
 *
 * Description:
 *   Cancel the oneshot timer and return the time remaining on the timer.
 *
 *   NOTE: This function may execute at a high rate with no timer running (as
 *   when pre-emption is enabled and disabled).
 *
 * Input Parameters:
 *   oneshot  Caller allocated instance of the oneshot state structure. This
 *            structure must have been previously initialized via a call to
 *            pic32mz_oneshot_initialize();
 *   ts       The location in which to return the time remaining on the
 *            oneshot timer.  A time of zero is returned if the timer is
 *            not running.  ts may be zero in which case the time remaining
 *            is not returned.
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  A call to up_timer_cancel() when
 *   the timer is not active should also return success; a negated errno
 *   value is returned on any failure.
 *
 ****************************************************************************/

int pic32mz_oneshot_cancel(struct pic32mz_oneshot_s *oneshot,
                           struct timespec *ts)
{
  irqstate_t flags;
  uint64_t usec;
  uint64_t sec;
  uint64_t nsec;
  uint32_t count;
  uint32_t period;

  flags = enter_critical_section();

  /* Was the timer running? */

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

  /* Yes.. Get the timer counter and period registers and stop the counter. */

  tmrinfo("Cancelling...\n");

  count  = PIC32MZ_TIMER_GETCOUNTER(oneshot->timer);
  period = oneshot->period;

  PIC32MZ_TIMER_SETISR(oneshot->timer, NULL, NULL);
  PIC32MZ_TIMER_STOP(oneshot->timer);

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
                        oneshot->freq;

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

#endif /* CONFIG_PIC32MZ_ONESHOT */
