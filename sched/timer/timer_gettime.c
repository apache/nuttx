/****************************************************************************
 * sched/timer/timer_gettime.c
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

#include <time.h>
#include <errno.h>

#include "clock/clock.h"
#include "timer/timer.h"

#ifndef CONFIG_DISABLE_POSIX_TIMERS

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: timer_gettime
 *
 * Description:
 *   The timer_gettime() function will store the amount of time until the
 *   specified timer, timerid, expires and the reload value of the timer
 *   into the space pointed to by the value argument. The it_value member
 *   of this structure will contain the amount of time before the timer
 *   expires, or zero if the timer is disarmed. This value is returned as
 *   the interval until timer expiration, even if the timer was armed with
 *   absolute time. The it_interval member of value will contain the reload
 *   value last set by timer_settime().
 *
 * Input Parameters:
 *   timerid - The pre-thread timer, previously created by the call to
 *   timer_create(), whose remaining time count will be returned..
 *
 * Returned Value:
 *   If the timer_gettime() succeeds, a value of 0 (OK) will be returned.
 *   If an error occurs, the value -1 (ERROR) will be returned, and errno
 *   set to indicate the error.
 *
 *   EINVAL - The timerid argument does not correspond to an ID returned by
 *     timer_create() but not yet deleted by timer_delete().
 *
 * Assumptions/Limitations:
 *   Due to the asynchronous operation of this function, the time reported
 *   by this function could be significantly more than that actual time
 *   remaining on the timer at any time.
 *
 ****************************************************************************/

int timer_gettime(timer_t timerid, FAR struct itimerspec *value)
{
  FAR struct posix_timer_s *timer = (FAR struct posix_timer_s *)timerid;
  sclock_t ticks;

  if (!timer || !value)
    {
      set_errno(EINVAL);
      return ERROR;
    }

  /* Get the number of ticks before the underlying watchdog expires */

  ticks = wd_gettime(&timer->pt_wdog);

  /* Convert that to a struct timespec and return it */

  clock_ticks2time(ticks, &value->it_value);
  clock_ticks2time(timer->pt_delay, &value->it_interval);
  return OK;
}

#endif /* CONFIG_DISABLE_POSIX_TIMERS */
