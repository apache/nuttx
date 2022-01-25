/****************************************************************************
 * libs/libc/sched/clock_timespec_subtract.c
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

#include <nuttx/clock.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  clock_timespec_subtract
 *
 * Description:
 *   Subtract timespec ts2 from to1 and return the result in ts3.
 *   Zero is returned if the time difference is negative.
 *
 * Input Parameters:
 *   ts1 and ts2: The two timespecs to be subtracted (ts1 - ts2)
 *   ts3: The location to return the result (may be ts1 or ts2)
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void clock_timespec_subtract(FAR const struct timespec *ts1,
                             FAR const struct timespec *ts2,
                             FAR struct timespec *ts3)
{
  time_t sec;
  long nsec;

  if (ts1->tv_sec < ts2->tv_sec)
    {
      sec  = 0;
      nsec = 0;
    }
  else if (ts1->tv_sec == ts2->tv_sec && ts1->tv_nsec <= ts2->tv_nsec)
    {
      sec  = 0;
      nsec = 0;
    }
  else
    {
      sec = ts1->tv_sec - ts2->tv_sec;
      if (ts1->tv_nsec < ts2->tv_nsec)
        {
          nsec = (ts1->tv_nsec + NSEC_PER_SEC) - ts2->tv_nsec;
          sec--;
        }
      else
        {
          nsec = ts1->tv_nsec - ts2->tv_nsec;
        }
    }

  ts3->tv_sec = sec;
  ts3->tv_nsec = nsec;
}
