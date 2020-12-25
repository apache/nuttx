/****************************************************************************
 * sched/timer/timer_setitimer.c
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
 *   EINVAL - A value structure specified a microsecond value less than zero
 *     or greater than or equal to 1000 million, and the it_value member of
 *     that structure did not specify zero seconds and nanoseconds.
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
