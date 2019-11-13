/****************************************************************************
 * sched/timer/timer_setitimer.c
 *
 *   Copyright (C) 2019 Xiaomi Inc. All rights reserved.
 *   Author: Xiang Xiao <xiaoxiang@xiaomi.com>
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

#include <errno.h>
#include <time.h>
#include <sys/time.h>

#include "sched/sched.h"

#ifndef CONFIG_DISABLE_POSIX_TIMERS

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: setitimer
 *
 * Description:
 *   The setitimer() function sets the time until the next expiration of
 *   the timer specified by which from the it_value member of the value
 *   argument and arm the timer if the it_value member of value is non-zero.
 *   If the specified timer was already armed when setitimer() is
 *   called, this call will reset the time until next expiration to the
 *   value specified. If the it_value member of value is zero, the timer
 *   will be disarmed. The effect of disarming or resetting a timer with
 *   pending expiration notifications is unspecified.
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
 *   If the argument ovalue is not NULL, the setitimer() function will
 *   store, in the location referenced by ovalue, a value representing the
 *   previous amount of time before the timer would have expired, or zero if
 *   the timer was disarmed, together with the previous timer reload value.
 *   Timers will not expire before their scheduled time.
 *
 * Input Parameters:
 *   which - The predefined timer id
 *   value - Specifies the timer value to set
 *   ovalue - A location in which to return the time remaining from the
 *     previous timer setting.
 *
 * Returned Value:
 *   If the setitimer() succeeds, a value of 0 (OK) will be returned.
 *   If an error occurs, the value -1 (ERROR) will be returned, and errno set
 *   to indicate the error.
 *
 *   EINVAL - The which argument does not correspond to an predefined ID.
 *   EINVAL - A value structure specified a microsecond value less than zero or
 *     greater than or equal to 1000 million, and the it_value member of that
 *     structure did not specify zero seconds and nanoseconds.
 *
 * Assumptions:
 *
 ****************************************************************************/

int setitimer(int which, FAR const struct itimerval *value,
              FAR struct itimerval *ovalue)
{
  FAR struct tcb_s *rtcb = this_task();
  struct itimerspec spec;
  struct itimerspec ospec;
  irqstate_t flags;
  int ret = OK;

  if (which != ITIMER_REAL || !value)
    {
      set_errno(EINVAL);
      return ERROR;
    }

  if (!rtcb->group->itimer)
    {
      flags = enter_critical_section();
      if (!rtcb->group->itimer)
        {
          ret = timer_create(CLOCK_REALTIME, NULL, &rtcb->group->itimer);
        }

      leave_critical_section(flags);

      if (ret != OK)
        {
          return ret;
        }
    }

  TIMEVAL_TO_TIMESPEC(&value->it_value, &spec.it_value);
  TIMEVAL_TO_TIMESPEC(&value->it_interval, &spec.it_interval);

  ret = timer_settime(rtcb->group->itimer, 0, &spec, ovalue ? &ospec : NULL);
  if (ret == OK && ovalue)
    {
      TIMESPEC_TO_TIMEVAL(&ovalue->it_value, &ospec.it_value);
      TIMESPEC_TO_TIMEVAL(&ovalue->it_interval, &ospec.it_interval);
    }

  return ret;
}

#endif /* CONFIG_DISABLE_POSIX_TIMERS */
