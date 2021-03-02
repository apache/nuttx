/****************************************************************************
 * libs/libc/time/lib_time.c
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

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  time
 *
 * Description:
 *   Get the current calendar time as a time_t object.  The function returns
 *   this value, and if the argument is not a null pointer, the value is also
 *   set to the object pointed by tloc.
 *
 *   Note that this function is just a thin wrapper around clock_gettime()
 *   and is provided for compatibility.  clock_gettime() is the preferred way
 *   to obtain system time.
 *
 * Input Parameters:
 *   Pointer to an object of type time_t, where the time value is stored.
 *   Alternatively, this parameter can be a null pointer, in which case the
 *   parameter is not used, but a time_t object is still returned by the
 *   function.
 *
 * Returned Value:
 *   The current calendar time as a time_t object.  If the argument is not
 *   a null pointer, the return value is the same as the one stored in the
 *   location pointed by the argument.
 *
 *   If the function could not retrieve the calendar time, it returns a -1
 *   value.
 *
 ****************************************************************************/

time_t time(time_t *tloc)
{
  struct timespec ts;
  int ret;

  /* Get the current time from the system */

  ret = clock_gettime(CLOCK_REALTIME, &ts);
  if (ret == OK)
    {
      /* Return the seconds since the epoch */

      if (tloc)
        {
          *tloc = ts.tv_sec;
        }

      return ts.tv_sec;
    }

  return (time_t)ERROR;
}
