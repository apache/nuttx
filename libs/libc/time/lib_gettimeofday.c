/****************************************************************************
 * libs/libc/time/lib_gettimeofday.c
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
 * Name: gettimeofday
 *
 * Description:
 *   Get the current time
 *
 *   Conforming to SVr4, 4.3BSD. POSIX.1-2001 describes gettimeofday().
 *   POSIX.1-2008 marks gettimeofday() as obsolete, recommending the use of
 *   clock_gettime(2) instead.
 *
 *   NuttX implements gettimeofday() as a thin layer around clock_gettime();
 *
 * Input Parameters:
 *   tv - The location to return the current time
 *   tz - Ignored
 *
 * Returned Value:
 *   Zero (OK) on success;  -1 is returned on failure with the errno variable
 *   set appropriately.
 *
 ****************************************************************************/

int gettimeofday(FAR struct timeval *tv, FAR struct timezone *tz)
{
  struct timespec ts;
  int ret;

  UNUSED(tz);

#ifdef CONFIG_DEBUG_FEATURES
  if (!tv)
    {
      set_errno(EINVAL);
      return ERROR;
    }
#endif

  /* Let clock_gettime do most of the work */

  ret = clock_gettime(CLOCK_REALTIME, &ts);
  if (ret == OK)
    {
      /* Convert the struct timespec to a struct timeval */

      tv->tv_sec  = ts.tv_sec;
      tv->tv_usec = ts.tv_nsec / NSEC_PER_USEC;
    }

  return ret;
}
