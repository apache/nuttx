/****************************************************************************
 * sched/timer/timer_getoverrun.c
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

#include "timer/timer.h"

#ifndef CONFIG_DISABLE_POSIX_TIMERS

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: timer_getoverrun
 *
 * Description:
 *   Only a single signal will be queued to the process for a given timer at
 *   any point in time.  When a timer for which a signal is still pending
 *   expires, no signal will be queued, and a timer overrun will occur. When
 *   a timer expiration signal is delivered to or accepted by a process, if
 *   the implementation  supports  the  Realtime Signals Extension, the
 *   timer_getoverrun() function will return the timer expiration overrun
 *   count for the specified timer. The overrun count returned contains the
 *   number of extra timer expirations that occurred between the time the
 *   signal was generated (queued) and when it was delivered or accepted, up
 *   to but not including an implementation-defined  maximum of
 *   DELAYTIMER_MAX. If the number of such extra expirations is greater than
 *   or equal to DELAYTIMER_MAX, then the overrun count will be set to
 *   DELAYTIMER_MAX. The value returned by timer_getoverrun() will apply to
 *   the most recent expiration signal delivery or acceptance for the timer.
 *   If no expiration signal has been delivered for the timer, or if the
 *   Realtime Signals Extension is not supported, the return value of
 *   timer_getoverrun() is unspecified.
 *
 * Input Parameters:
 *   timerid - The pre-thread timer, previously created by the call to
 *   timer_create(), whose overrun count will be returned..
 *
 * Returned Value:
 *   If the timer_getoverrun() function succeeds, it will return the timer
 *   expiration overrun count as explained above. timer_getoverrun() will
 *   fail if:
 *
 *   EINVAL - The timerid argument does not correspond to an ID returned by
 *     timer_create() but not yet deleted by timer_delete().
 *
 * Assumptions:
 *
 ****************************************************************************/

int timer_getoverrun(timer_t timerid)
{
  set_errno(ENOSYS);
  return ERROR;
}

#endif /* CONFIG_DISABLE_POSIX_TIMERS */
