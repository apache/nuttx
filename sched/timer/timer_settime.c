/****************************************************************************
 * sched/timer/timer_settime.c
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

#include <stdint.h>
#include <time.h>
#include <string.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/irq.h>

#include "clock/clock.h"
#include "timer/timer.h"

#ifndef CONFIG_DISABLE_POSIX_TIMERS

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static inline void timer_signotify(FAR struct posix_timer_s *timer);
static inline void timer_restart(FAR struct posix_timer_s *timer,
                                 wdparm_t itimer);
static void timer_timeout(wdparm_t itimer);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: timer_signotify
 *
 * Description:
 *   This function basically re-implements nxsig_queue() so that the si_code
 *   can be correctly set to SI_TIMER
 *
 * Input Parameters:
 *   timer - A reference to the POSIX timer that just timed out
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   This function executes in the context of the watchod timer interrupt.
 *
 ****************************************************************************/

static inline void timer_signotify(FAR struct posix_timer_s *timer)
{
  DEBUGVERIFY(nxsig_notification(timer->pt_owner, &timer->pt_event,
                                 SI_TIMER, &timer->pt_work));
}

/****************************************************************************
 * Name: timer_restart
 *
 * Description:
 *   If a periodic timer has been selected, then restart the watchdog.
 *
 * Input Parameters:
 *   timer - A reference to the POSIX timer that just timed out
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   This function executes in the context of the watchdog timer interrupt.
 *
 ****************************************************************************/

static inline void timer_restart(FAR struct posix_timer_s *timer,
                                 wdparm_t itimer)
{
  /* If this is a repetitive timer, then restart the watchdog */

  if (timer->pt_delay)
    {
      wd_start(&timer->pt_wdog, timer->pt_delay, timer_timeout, itimer);
    }
}

/****************************************************************************
 * Name: timer_timeout
 *
 * Description:
 *   This function is called if the timeout elapses before the condition is
 *   signaled.
 *
 * Input Parameters:
 *   itimer - A reference to the POSIX timer that just timed out
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   This function executes in the context of the watchod timer interrupt.
 *
 ****************************************************************************/

static void timer_timeout(wdparm_t itimer)
{
  FAR struct posix_timer_s *timer = timer_gethandle((timer_t)itimer);

  if (timer == NULL)
    {
      return;
    }

  /* Send the specified signal to the specified task.   Increment the
   * reference count on the timer first so that will not be deleted until
   * after the signal handler returns.
   */

  timer->pt_crefs++;
  timer_signotify(timer);

  /* Release the reference.  timer_release will return nonzero if the timer
   * was not deleted.
   */

  if (timer_release(timer))
    {
      /* If this is a repetitive timer, the restart the watchdog */

      timer_restart(timer, itimer);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: timer_settime
 *
 * Description:
 *   The timer_settime() function sets the time until the next expiration of
 *   the timer specified by timerid from the it_value member of the value
 *   argument and arm the timer if the it_value member of value is non-zero.
 *   If the specified timer was already armed when timer_settime() is
 *   called, this call will reset the time until next expiration to the
 *   value specified. If the it_value member of value is zero, the timer
 *   will be disarmed. The effect of disarming or resetting a timer with
 *   pending expiration notifications is unspecified.
 *
 *   If the flag TIMER_ABSTIME is not set in the argument flags,
 *   timer_settime() will behave as if the time until next expiration is set
 *   to be equal to the interval specified by the it_value member of value.
 *   That is, the timer will expire in it_value nanoseconds from when the
 *   call is made. If the flag TIMER_ABSTIME is set in the argument flags,
 *   timer_settime() will behave as if the time until next expiration is set
 *   to be equal to the difference between the absolute time specified by
 *   the it_value member of value and the current value of the clock
 *   associated with timerid.  That is, the timer will expire when the clock
 *   reaches the value specified by the it_value member of value.  If the
 *   specified time has already passed, the function will succeed and the
 *   expiration notification will be made.
 *
 *   The reload value of the timer will be set to the value specified by the
 *   it_interval member of value.  When a timer is armed with a non-zero
 *   it_interval, a periodic (or repetitive) timer is specified.
 *
 *   Time values that are between two consecutive non-negative integer
 *   multiples of the resolution of the specified timer will be rounded up
 *   to the larger multiple of the resolution. Quantization error will not
 *   cause the timer to expire earlier than the rounded time value.
 *
 *   If the argument ovalue is not NULL, the timer_settime() function will
 *   store, in the location referenced by ovalue, a value representing the
 *   previous amount of time before the timer would have expired, or zero if
 *   the timer was disarmed, together with the previous timer reload value.
 *   Timers will not expire before their scheduled time.
 *
 * Input Parameters:
 *   timerid - The pre-thread timer, previously created by the call to
 *     timer_create(), to be be set.
 *   flags - Specifies characteristics of the timer (see above)
 *   value - Specifies the timer value to set
 *   ovalue - A location in which to return the time remaining from the
 *     previous timer setting.
 *
 * Returned Value:
 *   If the timer_settime() succeeds, a value of 0 (OK) will be returned.
 *   If an error occurs, the value -1 (ERROR) will be returned, and errno set
 *   to indicate the error.
 *
 *   EINVAL - The timerid argument does not correspond to an ID returned by
 *     timer_create() but not yet deleted by timer_delete().
 *   EINVAL - A value structure specified a nanosecond value less than zero
 *     or greater than or equal to 1000 million, and the it_value member of
 *     that structure did not specify zero seconds and nanoseconds.
 *
 * Assumptions:
 *
 ****************************************************************************/

int timer_settime(timer_t timerid, int flags,
                  FAR const struct itimerspec *value,
                  FAR struct itimerspec *ovalue)
{
  FAR struct posix_timer_s *timer = timer_gethandle(timerid);
  irqstate_t intflags;
  sclock_t delay;
  int ret = OK;

  /* Some sanity checks */

  if (!timer || !value)
    {
      set_errno(EINVAL);
      return ERROR;
    }

  if (ovalue)
    {
      /* Get the number of ticks before the underlying watchdog expires */

      delay = wd_gettime(&timer->pt_wdog);

      /* Convert that to a struct timespec and return it */

      clock_ticks2time(delay, &ovalue->it_value);
      clock_ticks2time(timer->pt_delay, &ovalue->it_interval);
    }

  /* Disarm the timer (in case the timer was already armed when
   * timer_settime() is called).
   */

  wd_cancel(&timer->pt_wdog);

  /* Cancel any pending notification */

  nxsig_cancel_notification(&timer->pt_work);

  /* If the it_value member of value is zero, the timer will not be
   * re-armed
   */

  if (value->it_value.tv_sec <= 0 && value->it_value.tv_nsec <= 0)
    {
      return OK;
    }

  /* Setup up any repetitive timer */

  if (value->it_interval.tv_sec > 0 || value->it_interval.tv_nsec > 0)
    {
      clock_time2ticks(&value->it_interval, &delay);

      /* REVISIT: Should pt_delay be sclock_t? */

      timer->pt_delay = (int)delay;
    }
  else
    {
      timer->pt_delay = 0;
    }

  /* We need to disable timer interrupts through the following section so
   * that the system timer is stable.
   */

  intflags = enter_critical_section();

  /* Check if abstime is selected */

  if ((flags & TIMER_ABSTIME) != 0)
    {
      /* Calculate a delay corresponding to the absolute time in 'value' */

      ret = clock_abstime2ticks(timer->pt_clock, &value->it_value, &delay);
    }
  else
    {
      /* Calculate a delay assuming that 'value' holds the relative time
       * to wait.  We have internal knowledge that clock_time2ticks always
       * returns success.
       */

      ret = clock_time2ticks(&value->it_value, &delay);
    }

  if (ret < 0)
    {
      goto errout;
    }

  /* If the specified time has already passed, the function shall succeed
   * and the expiration notification shall be made.
   */

  if (delay < 0)
    {
      delay = 0;
    }

  /* Then start the watchdog */

  if (delay >= 0)
    {
      ret = wd_start(&timer->pt_wdog, delay, timer_timeout, (wdparm_t)timer);
    }

errout:
  leave_critical_section(intflags);

  if (ret < 0)
    {
      set_errno(-ret);
      ret = ERROR;
    }

  return ret;
}

#endif /* CONFIG_DISABLE_POSIX_TIMERS */
