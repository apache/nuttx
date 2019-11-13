/****************************************************************************
 * sched/timer/timer_getitimer.c
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
