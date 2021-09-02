/****************************************************************************
 * arch/arm/src/samd5e5/sam_oneshot.c
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
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/clock.h>

#include "sam_oneshot.h"
#include "sam_freerun.h"

#ifdef CONFIG_SAMD5E5_ONESHOT

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_oneshot_handler
 *
 * Description:
 *   Timer interrupt callback.  When the oneshot timer interrupt expires,
 *   this function will be called.  It will forward the call to the next
 *   level up.
 *
 * Input Parameters:
 *   tch - The handle that represents the timer state
 *   arg - An opaque argument provided when the interrupt was registered
 *   sr  - The value of the timer interrupt status register at the time
 *         that the interrupt occurred.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void sam_oneshot_handler(TC_HANDLE tch, void *arg, uint32_t sr)
{
  struct sam_oneshot_s *oneshot = (struct sam_oneshot_s *)arg;
  oneshot_handler_t oneshot_handler;
  void *oneshot_arg;

  tmrinfo("Expired...\n");
  DEBUGASSERT(oneshot && oneshot->handler);

  /* The clock was stopped, but not disabled when the RC match occurred.
   * Disable the TC now and disable any further interrupts.
   */

  sam_tc_detach(oneshot->tch);
  sam_tc_stop(oneshot->tch);

  /* The timer is no longer running */

  oneshot->running = false;

  /* Forward the event, clearing out any vestiges */

  oneshot_handler      = (oneshot_handler_t)oneshot->handler;
  oneshot->handler     = NULL;
  oneshot_arg          = (void *)oneshot->arg;
  oneshot->arg         = NULL;
#ifdef CONFIG_SAMD5E5_FREERUN
  oneshot->start_count = 0;
#endif

  oneshot_handler(oneshot_arg);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_oneshot_initialize
 *
 * Description:
 *   Initialize the oneshot timer wrapper
 *
 * Input Parameters:
 *   oneshot    Caller allocated instance of the oneshot state structure
 *   chan       Timer counter channel to be used.  See the TC_CHAN*
 *              definitions in arch/arm/src/samd5e5/sam_tc.h.
 *   resolution The required resolution of the timer in units of
 *              microseconds.  NOTE that the range is restricted to the
 *              range of uint16_t (excluding zero).
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

int sam_oneshot_initialize(struct sam_oneshot_s *oneshot, int chan,
                           uint16_t resolution)
{
  uint32_t frequency;

  tmrinfo("chan=%d resolution=%d usec\n", chan, resolution);
  DEBUGASSERT(oneshot && resolution > 0);

  /* Get the TC frequency the corresponds to the requested resolution */

  frequency = USEC_PER_SEC / (uint32_t)resolution;

  oneshot->tch = sam_tc_allocate(chan, frequency);
  if (!oneshot->tch)
    {
      tmrerr("ERROR: Failed to allocate timer channel %d\n", chan);
      return -EBUSY;
    }

  /* Initialize the remaining fields in the state structure and return
   * success.
   */

  oneshot->chan        = chan;
  oneshot->running     = false;
  oneshot->handler     = NULL;
  oneshot->arg         = NULL;
#ifdef CONFIG_SAMD5E5_FREERUN
  oneshot->start_count = 0;
#endif

  return OK;
}

/****************************************************************************
 * Name: sam_oneshot_max_delay
 *
 * Description:
 *   Return the maximum delay supported by the one shot timer (in
 *   microseconds).
 *
 * Input Parameters:
 *   oneshot Caller allocated instance of the oneshot state structure.  This
 *           structure must have been previously initialized via a call to
 *           sam_oneshot_initialize();
 *   usec    The location in which to return the maximum delay.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

int sam_oneshot_max_delay(struct sam_oneshot_s *oneshot, uint64_t *usec)
{
  DEBUGASSERT(oneshot != NULL && usec != NULL);
  *usec = (0xffffull * USEC_PER_SEC) /
          (uint64_t)sam_tc_divfreq(oneshot->tch);
  return OK;
}

/****************************************************************************
 * Name: sam_oneshot_start
 *
 * Description:
 *   Start the oneshot timer
 *
 * Input Parameters:
 *   oneshot Caller allocated instance of the oneshot state structure.  This
 *           structure must have been previously initialized via a call to
 *           sam_oneshot_initialize();
 *   handler The function to call when when the oneshot timer expires.
 *   arg     An opaque argument that will accompany the callback.
 *   ts      Provides the duration of the one shot timer.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

int sam_oneshot_start(struct sam_oneshot_s *oneshot,
                      struct sam_freerun_s *freerun,
                      oneshot_handler_t handler, void *arg,
                      const struct timespec *ts)
{
  uint64_t usec;
  uint64_t regval;
  irqstate_t flags;

  tmrinfo("handler=%p arg=%p, ts=(%lu, %lu)\n",
  handler, arg, (unsigned long)ts->tv_sec, (unsigned long)ts->tv_nsec);
  DEBUGASSERT(oneshot && handler && ts);

  /* Was the oneshot already running? */

  flags = enter_critical_section();
  if (oneshot->running)
    {
      /* Yes.. then cancel it */

      tmrinfo("Already running... cancelling\n");
      sam_oneshot_cancel(oneshot, freerun, NULL);
    }

  /* Save the new handler and its argument */

  oneshot->handler = handler;
  oneshot->arg     = arg;

  /* Express the delay in microseconds */

  usec = (uint64_t)ts->tv_sec * USEC_PER_SEC + (uint64_t)(ts->tv_nsec /
                                                        NSEC_PER_USEC);

  /* Get the timer counter frequency and determine
   * the number of counts need to achieve the requested delay.
   *
   *   frequency = ticks / second
   *   ticks     = seconds * frequency
   *             = (usecs * frequency) / USEC_PER_SEC;
   */

  regval = (usec * (uint64_t)sam_tc_divfreq(oneshot->tch)) / USEC_PER_SEC;

  tmrinfo("usec=%llu * %lu / %lu -> regval=0x%08llx\n", usec,
          sam_tc_divfreq(oneshot->tch), USEC_PER_SEC, regval);
  DEBUGASSERT(regval <= UINT32_MAX);

  /* Set up to receive the callback when the interrupt occurs */

  sam_tc_attach(oneshot->tch, sam_oneshot_handler, oneshot, TC_INTFLAG_MC0);

  /* Set CC0 so that an event will be triggered when COUNT register
   * counts up to CC0.
   */

  sam_tc_setregister(oneshot->tch, TC_REGCC0, (uint32_t)regval);
  sam_tc_setregister(oneshot->tch, TC_REGCC1, (uint32_t)0xffffffff);

  /* Start the counter */

  sam_tc_start(oneshot->tch);

#ifdef CONFIG_SAMD5E5_FREERUN
  /* The function sam_tc_start() starts the timer/counter by setting the
   * bits TC_CCR_CLKEN and TC_CCR_SWTRG in the channel control register.
   * The first one enables the timer/counter the latter performs an
   * software trigger, which starts the clock and sets the counter
   * register to zero. This reset is performed with the next valid edge
   * of the selected clock. Thus it can take up USEC_PER_TICK microseconds
   * until the counter register becomes zero.
   *
   * If the timer is canceled within this period the counter register holds
   * the counter value for the last timer/counter run. To circumvent this
   * the counter value of the freerun timer/counter is stored at each start
   * of the oneshot timer/counter.
   *
   * The function up_timer_gettime() could also be used for this but it takes
   * too long. If up_timer_gettime() is called within this function the
   * problem vanishes at least if compiled with no optimisation.
   */

  if (freerun != NULL)
    {
      oneshot->start_count = sam_tc_getcounter(freerun->tch);
    }
#endif

  /* Enable interrupts.  We should get the callback when the interrupt
   * occurs.
   */

  oneshot->running = true;
  leave_critical_section(flags);

  return OK;
}

/****************************************************************************
 * Name: sam_oneshot_cancel
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
 *           sam_oneshot_initialize();
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

int sam_oneshot_cancel(struct sam_oneshot_s *oneshot,
                       struct sam_freerun_s *freerun, struct timespec *ts)
{
  irqstate_t flags;
  uint64_t usec;
  uint64_t sec;
  uint64_t nsec;
  uint32_t count;
  uint32_t rc;

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

  /* Yes.. Get the timer counter and rc registers and stop the counter.  If
   * the counter expires while we are doing this, the counter clock will be
   * stopped, but the clock will not be disabled.
   *
   * The expected behavior is that the counter register will freezes at
   * a value equal to the RC register when the timer expires.  The counter
   * should have values between 0 and RC in all other cased.
   *
   * REVISIT:  This does not appear to be the case.
   */

  tmrinfo("Cancelling...\n");

  count = sam_tc_getcounter(oneshot->tch);
  rc    = sam_tc_getregister(oneshot->tch, TC_REGCC0);

#ifdef CONFIG_SAMD5E5_FREERUN
  /* In the case the timer/counter was canceled very short after its start,
   * the counter register can hold the wrong value (the value of the last
   * run). To prevent this the counter value is set to zero if not at
   * least on tick passed since the start of the timer/counter.
   */

  if (count > 0 && freerun != NULL &&
      sam_tc_getcounter(freerun->tch) == oneshot->start_count)
    {
      count = 0;
    }
#endif

  /* Now we can disable the interrupt and stop the timer. */

  sam_tc_attach(oneshot->tch);
  sam_tc_stop(oneshot->tch);

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

      tmrinfo("rc=%lu count=%lu usec=%lu\n",
              (unsigned long)rc, (unsigned long)count, (unsigned long)usec);

      /* REVISIT: I am not certain why the timer counter value sometimes
       * exceeds RC.  Might be a bug, or perhaps the counter does not stop
       * in all cases.
       */

      if (count >= rc)
        {
          /* No time remaining (?) */

          ts->tv_sec  = 0;
          ts->tv_nsec = 0;
        }
      else
        {
          /* The total time remaining is the difference.  Convert the that
           * to units of microseconds.
           *
           *   frequency = ticks / second
           *   seconds   = ticks * frequency
           *   usecs     = (ticks * USEC_PER_SEC) / frequency;
           */

          usec        = (((uint64_t)(rc - count)) * USEC_PER_SEC) /
                                                sam_tc_divfreq(oneshot->tch);

          /* Each time the timer/counter is canceled the time calculated from
           * the two registers (counter and REGC) is accurate up to an error
           * between 0 and USEC_PER_TICK microseconds. To correct this error
           * one tick which means USEC_PER_TICK microseconds are subtracted.
           */

          usec        = usec > USEC_PER_TICK ? usec - USEC_PER_TICK : 0;

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

#endif /* CONFIG_SAMD5E5_ONESHOT */
