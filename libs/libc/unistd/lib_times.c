/****************************************************************************
 * libs/libc/unistd/lib_times.c
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

#include <errno.h>
#include <string.h>
#include <sys/times.h>
#include <time.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: times
 *
 * Description:
 *   The times() function shall fill the tms structure pointed to by buffer
 *   with time-accounting information.
 *
 * Returned Value:
 *   Upon successful completion, times() shall return the elapsed real time,
 *   in clock ticks, since an arbitrary point in the past (for example,
 *   system start-up time). This point does not change from one invocation
 *   of times() within the process to another. The return value may overflow
 *   the possible range of type clock_t. If times() fails, (clock_t)-1 shall
 *   be returned and errno set to indicate the error.
 *
 ****************************************************************************/

clock_t times(FAR struct tms *buffer)
{
  if (buffer == NULL)
    {
      set_errno(EINVAL);
      return -1;
    }

  memset(buffer, 0, sizeof(*buffer));
  return clock();
}
