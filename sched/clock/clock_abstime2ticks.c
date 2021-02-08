/****************************************************************************
 * sched/clock/clock_abstime2ticks.c
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
#include <debug.h>
#include "clock/clock.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: clock_timespec_compare
 *
 * Description:
 *    Return < 0 if time a is before time b
 *    Return > 0 if time b is before time a
 *    Return 0 if time a is the same as time b
 *
 ****************************************************************************/

int clock_timespec_compare(FAR const struct timespec *a,
                           FAR const struct timespec *b)
{
  if (a->tv_sec < b->tv_sec)
    {
      return -1;
    }

  if (a->tv_sec > b->tv_sec)
    {
      return 1;
    }

  return a->tv_nsec - b->tv_nsec;
}

/****************************************************************************
 * Name: clock_abstime2ticks
 *
 * Description:
 *   Convert an absolute timespec delay to system timer ticks.
 *
 * Input Parameters:
 *   clockid - The timing source to use in the conversion
 *   reltime - Convert this absolute time to system clock ticks.
 *   ticks - Return the converted number of ticks here.
 *
 * Returned Value:
 *   OK on success; A non-zero error number on failure
 *
 * Assumptions:
 *   Interrupts should be disabled so that the time is not changing during
 *   the calculation
 *
 ****************************************************************************/

int clock_abstime2ticks(clockid_t clockid,
                        FAR const struct timespec *abstime,
                        FAR sclock_t *ticks)
{
  struct timespec currtime;
  struct timespec reltime;
  int             ret;

  /* Convert the timespec to clock ticks.
   * NOTE: Here we use internal knowledge
   * that CLOCK_REALTIME is defined to be zero!
   */

  ret = clock_gettime(clockid, &currtime);
  if (ret != OK)
    {
      return EINVAL;
    }

  if (clock_timespec_compare(abstime, &currtime) < 0)
    {
      /* Every caller of clock_abstime2ticks check 'ticks < 0' to see if
       * absolute time is in the past. So lets just return negative tick
       * here.
       */

      *ticks = -1;
      return OK;
    }

  /* The relative time to wait is the absolute time minus the current time. */

  reltime.tv_nsec = (abstime->tv_nsec - currtime.tv_nsec);
  reltime.tv_sec  = (abstime->tv_sec  - currtime.tv_sec);

  /* Check if we were supposed to borrow from the seconds. */

  if (reltime.tv_nsec < 0)
    {
      reltime.tv_nsec += NSEC_PER_SEC;
      reltime.tv_sec  -= 1;
    }

  /* Convert this relative time into clock ticks. */

  return clock_time2ticks(&reltime, ticks);
}
