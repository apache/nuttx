/****************************************************************************
 * libs/libc/sched/clock_time2ticks.c
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
#include <assert.h>

#include <nuttx/clock.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: clock_time2ticks
 *
 * Description:
 *   Convert a timespec delay to system timer ticks.  This function is
 *   suitable for calculating relative time delays and does not depend on
 *   the other clock_* logic.
 *
 * Input Parameters:
 *   reltime - Convert this relative time to system clock ticks.
 *   ticks - Return the converted number of ticks here.
 *
 * Returned Value:
 *   Always returns OK
 *
 * Assumptions:
 *
 ****************************************************************************/

int clock_time2ticks(FAR const struct timespec *reltime,
                     FAR sclock_t *ticks)
{
#ifdef CONFIG_HAVE_LONG_LONG
  int64_t relnsec;

  /* Convert the relative time into nanoseconds.  The range of the int64_t
   * is sufficiently large that there is no real need for range checking.
   */

  relnsec = (int64_t)reltime->tv_sec * NSEC_PER_SEC +
            (int64_t)reltime->tv_nsec;

  /* Convert nanoseconds to clock ticks, rounding up to the smallest integer
   * that is greater than or equal to the exact number of tick.
   */

  *ticks = (sclock_t)((relnsec + NSEC_PER_TICK - 1) / NSEC_PER_TICK);
  return OK;
#else
  int32_t relusec;

  /* This function uses an int32_t to only the relative time in microseconds.
   * that means that the maximum supported relative time is 2,147,487.647
   * seconds
   */

#if 0 // overkill
  DEBUGASSERT(reltime->tv_sec  <  2147487 ||
              reltime->tv_sec  == 2147487 &&
              reltime->tv_nsec <= 647 * NSEC_PER_MSEC);
#endif

  /* Convert the relative time into microseconds, rounding up to the smallest
   * value that is greater than or equal to the exact number of microseconds.
   */

  relusec = reltime->tv_sec * USEC_PER_SEC +
            (reltime->tv_nsec + NSEC_PER_USEC - 1) / NSEC_PER_USEC;

  /* Convert microseconds to clock ticks, rounding up to the smallest integer
   * that is greater than or equal to the exact number of tick.
   */

  *ticks = (sclock_t)((relusec + USEC_PER_TICK - 1) / USEC_PER_TICK);
  return OK;
#endif
}
