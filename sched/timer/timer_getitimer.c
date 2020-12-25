/****************************************************************************
 * sched/timer/timer_getitimer.c
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
 * Name: getitimer
 *
 * Description:
 *   The getitimer() function will store the amount of time until the
 *   specified timer, which, expires and the reload value of the timer
 *   into the space pointed to by the value argument. The it_value member
 *   of this structure will contain the amount of time before the timer
 *   expires, or zero if the timer is disarmed. This value is returned as
 *   the interval until timer expiration. The it_interval member of value
 *   will contain the reload value last set by setitime().
 *
 * Input Parameters:
 *   which - The predefined timer id
 *   value - The current timer value
 *
 * Returned Value:
 *   If the getitimer() succeeds, a value of 0 (OK) will be returned.
 *   If an error occurs, the value -1 (ERROR) will be returned, and errno
 *   set to indicate the error.
 *
 *   EINVAL - The which argument does not correspond to an predefined ID.
 *
 * Assumptions/Limitations:
 *   Due to the asynchronous operation of this function, the time reported
 *   by this function could be significantly more than that actual time
 *   remaining on the timer at any time.
 *
 ****************************************************************************/

int getitimer(int which, FAR struct itimerval *value)
{
  FAR struct tcb_s *rtcb = this_task();
  struct itimerspec spec =
  {
    {0, 0},
    {0, 0}
  };

  int ret = OK;

  if (which != ITIMER_REAL || !value)
    {
      set_errno(EINVAL);
      return ERROR;
    }

  if (rtcb->group->itimer)
    {
      ret = timer_gettime(rtcb->group->itimer, &spec);
    }

  if (ret == OK)
    {
      TIMESPEC_TO_TIMEVAL(&value->it_value, &spec.it_value);
      TIMESPEC_TO_TIMEVAL(&value->it_interval, &spec.it_interval);
    }

  return ret;
}

#endif /* CONFIG_DISABLE_POSIX_TIMERS */
