/****************************************************************************
 * arch/arm/src/sam34/sam4cm_oneshot.c
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * References:
 *
 *   Atmel NoOS sample code.
 *
 * The Atmel sample code has a BSD compatible license that requires this
 * copyright notice:
 *
 *   Copyright (c) 2011, Atmel Corporation
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
 * 3. Neither the names NuttX nor Atmel nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
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

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/irq.h>
#include <nuttx/clock.h>

#include "sam4cm_oneshot.h"
#include "sam4cm_freerun.h"

#ifdef CONFIG_SAM34_ONESHOT

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

  sam_tc_attach(oneshot->tch, NULL, NULL, 0);
  sam_tc_stop(oneshot->tch);

  /* The timer is no longer running */

  oneshot->running = false;

  /* Forward the event, clearing out any vestiges */

  oneshot_handler      = (oneshot_handler_t)oneshot->handler;
  oneshot->handler     = NULL;
  oneshot_arg          = (void *)oneshot->arg;
  oneshot->arg         = NULL;
#ifdef CONFIG_SAM34_FREERUN
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
 *              definitions in arch/arm/src/sam34/sam_tc.h.
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
  uint32_t divisor;
  uint32_t cmr;
  int ret;

  tmrinfo("chan=%d resolution=%d usec\n", chan, resolution);
  DEBUGASSERT(oneshot && resolution > 0);

  /* Get the TC frequency the corresponds to the requested resolution */

  frequency = USEC_PER_SEC / (uint32_t)resolution;

  /* The pre-calculate values to use when we start the timer */

  ret = sam_tc_divisor(frequency, &divisor, &cmr);
  if (ret < 0)
    {
      tmrerr("ERROR: sam_tc_divisor failed: %d\n", ret);
      return ret;
    }

  tmrinfo("frequency=%lu, divisor=%lu, cmr=%08lx\n",
          (unsigned long)frequency, (unsigned long)divisor,
          (unsigned long)cmr);

  /* Allocate the timer/counter and select its mode of operation
   *
   *   CMR_TCCLKS          - Returned by sam_tc_divisor
   *   TC_CMR_CLKI=0       - Not inverted
   *   TC_CMR_BURST_NONE   - Not gated by an external signal
   *   TC_CMR_CPCSTOP=1    - Stop the clock on an RC compare event
   *   TC_CMR_CPCDIS=0     - Don't disable the clock on an RC compare event
   *   TC_CMR_EEVTEDG_NONE - No external events (and, hence, no edges
   *   TC_CMR_EEVT_TIOB    - ???? REVISIT
   *   TC_CMR_ENET=0       - External event trigger disabled
   *   TC_CMR_WAVSEL_UPRC  - TC_CV is incremented from 0 to the value of RC,
   *                         then automatically reset on a RC Compare
   *   TC_CMR_WAVE         - Waveform mode
   *   TC_CMR_ACPA_NONE    - RA compare has no effect on TIOA
   *   TC_CMR_ACPC_NONE    - RC compare has no effect on TIOA
   *   TC_CMR_AEEVT_NONE   - No external event effect on TIOA
   *   TC_CMR_ASWTRG_NONE  - No software trigger effect on TIOA
   *   TC_CMR_BCPB_NONE    - RB compare has no effect on TIOB
   *   TC_CMR_BCPC_NONE    - RC compare has no effect on TIOB
   *   TC_CMR_BEEVT_NONE   - No external event effect on TIOB
   *   TC_CMR_BSWTRG_NONE  - No software trigger effect on TIOB
   */

  cmr |= (TC_CMR_BURST_NOTGATED | TC_CMR_CPCSTOP       | TC_CMR_EEVTEDG_NONE |
          TC_CMR_EEVT_TIOB      | TC_CMR_WAVSEL_UPAUTO | TC_CMR_WAVE         |
          TC_CMR_ACPA_NONE      | TC_CMR_ACPC_NONE     | TC_CMR_AEEVT_NONE   |
          TC_CMR_ASWTRG_NONE    | TC_CMR_BCPB_NONE     | TC_CMR_BCPC_NONE    |
          TC_CMR_BEEVT_NONE     | TC_CMR_BSWTRG_NONE);

  oneshot->tch = sam_tc_allocate(chan, cmr);
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
#ifdef CONFIG_SAM34_FREERUN
  oneshot->start_count = 0;
#endif

  return OK;
}

/****************************************************************************
 * Name: sam_oneshot_max_delay
 *
 * Description:
 *   Determine the maximum delay of the one-shot timer (in microseconds)
 *
 ****************************************************************************/

int sam_oneshot_max_delay(struct sam_oneshot_s *oneshot, uint64_t *usec)
{
  DEBUGASSERT(oneshot && usec);
  *usec = (0xffffull * USEC_PER_SEC) / (uint64_t)sam_tc_divfreq(oneshot->tch);
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
      (void)sam_oneshot_cancel(oneshot, freerun, NULL);
    }

  /* Save the new handler and its argument */

  oneshot->handler = handler;
  oneshot->arg     = arg;

  /* Express the delay in microseconds */

  usec = (uint64_t)ts->tv_sec * USEC_PER_SEC + (uint64_t)(ts->tv_nsec / NSEC_PER_USEC);

  /* Get the timer counter frequency and determine the number of counts need to achieve the requested delay.
   *
   *   frequency = ticks / second
   *   ticks     = seconds * frequency
   *             = (usecs * frequency) / USEC_PER_SEC;
   */

  regval = (usec * (uint64_t)sam_tc_divfreq(oneshot->tch)) / USEC_PER_SEC;

  tmrinfo("usec=%llu regval=%08llx\n", usec, regval);
  DEBUGASSERT(regval <= UINT16_MAX);

  /* Set up to receive the callback when the interrupt occurs */

  (void)sam_tc_attach(oneshot->tch, sam_oneshot_handler, oneshot,
                      TC_INT_CPCS);

  /* Set RC so that an event will be triggered when TC_CV register counts
   * up to RC.
   */

  sam_tc_setregister(oneshot->tch, TC_REGC, (uint32_t)regval);

  /* Start the counter */

  sam_tc_start(oneshot->tch);

#ifdef CONFIG_SAM34_FREERUN
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
   * too long. If up_timer_gettime() is called within this function the problem
   * vanishes at least if compiled with no optimisation.
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
  rc    = sam_tc_getregister(oneshot->tch, TC_REGC);

#ifdef CONFIG_SAM34_FREERUN
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

  sam_tc_attach(oneshot->tch, NULL, NULL, 0);
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

#endif /* CONFIG_SAM34_ONESHOT */
