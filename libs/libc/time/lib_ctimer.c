/****************************************************************************
 * libs/libc/time/lib_ctimer.c
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
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  ctime_r
 *
 * Description:
 *   ctime and ctime_r convert the time provided in seconds since the
 *   epoch to a string representation. ctime is not re-entrant; ctime_r is
 *   re-entrant.
 *
 * Input Parameters:
 *   timep - The current time represented as seconds since the epoch.
 *   buf   - A user provided buffer to receive the 26 character time string.
 *
 * Returned Value:
 *   On success, the pointer to the 'buf' is returned; on failure, NULL is
 *   returned.
 *
 ****************************************************************************/

FAR char *ctime_r(FAR const time_t *timep, FAR char *buf)
{
  struct tm tm;

  return asctime_r(localtime_r(timep, &tm), buf);
}
