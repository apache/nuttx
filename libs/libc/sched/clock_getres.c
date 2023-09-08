/****************************************************************************
 * libs/libc/sched/clock_getres.c
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
#include <errno.h>
#include <debug.h>

#include <nuttx/clock.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: clock_getres
 *
 * Description:
 *   Clock Functions based on POSIX APIs
 *
 ****************************************************************************/

int clock_getres(clockid_t clock_id, struct timespec *res)
{
  clockid_t clock_type = clock_id & CLOCK_MASK;
  int       ret = OK;

  sinfo("clock_id=%d, clock_type=%d\n", clock_id, clock_type);

  switch (clock_type)
    {
      default:
        serr("Returning ERROR\n");
        set_errno(EINVAL);
        ret = ERROR;
        break;

      case CLOCK_MONOTONIC:
      case CLOCK_BOOTTIME:
      case CLOCK_REALTIME:
      case CLOCK_PROCESS_CPUTIME_ID:
      case CLOCK_THREAD_CPUTIME_ID:

        /* Form the timspec using clock resolution in nanoseconds */

        res->tv_sec  = 0;
        res->tv_nsec = NSEC_PER_TICK;

        sinfo("Returning res=(%d,%d)\n", (int)res->tv_sec,
                                         (int)res->tv_nsec);
        break;
    }

  return ret;
}
