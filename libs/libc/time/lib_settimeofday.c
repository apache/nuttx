/****************************************************************************
 * libs/libc/time/lib_settimeofday.c
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

#include <sys/time.h>
#include <errno.h>

#include <nuttx/clock.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: settimeofday
 *
 * Description:
 *   Set the current time
 *
 *   Conforming to SVr4, 4.3BSD. POSIX.1-2001 describes gettimeofday() but
 *   not settimeofday().
 *
 *   NuttX implements settimeofday() as a thin layer around clock_settime();
 *
 * Input Parameters:
 *   tv - The net to time to be set
 *   tz - Ignored
 *
 * Returned Value:
 *   Zero (OK) on success;  -1 is returned on failure with the errno variable
 *   set appropriately.
 *
 ****************************************************************************/

int settimeofday(FAR const struct timeval *tv, FAR const struct timezone *tz)
{
  struct timespec ts;

  UNUSED(tz);

#ifdef CONFIG_DEBUG_FEATURES
  if (!tv || tv->tv_usec >= USEC_PER_SEC)
    {
      set_errno(EINVAL);
      return ERROR;
    }
#endif

  /* Convert the timeval to a timespec */

  ts.tv_sec  = tv->tv_sec;
  ts.tv_nsec = tv->tv_usec * NSEC_PER_USEC;

  /* Let clock_settime do the work */

  return clock_settime(CLOCK_REALTIME, &ts);
}
