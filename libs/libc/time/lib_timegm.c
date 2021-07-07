/****************************************************************************
 * libs/libc/time/lib_timegm.c
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
#include <debug.h>

#include <nuttx/time.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Type Declarations
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Public Constant Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  timegm
 *
 * Description:
 *   Time conversion (based on the POSIX API)
 *
 ****************************************************************************/

time_t timegm(FAR struct tm *tp)
{
  time_t ret;
  time_t jdn;

  /* Get the EPOCH-relative julian date from the calendar year,
   * month, and date
   */

  jdn = clock_calendar2utc(tp->tm_year + 1900, tp->tm_mon, tp->tm_mday);
  linfo("jdn=%d tm_year=%d tm_mon=%d tm_mday=%d\n",
        (int)jdn, tp->tm_year, tp->tm_mon, tp->tm_mday);

  /* Return the seconds into the julian day. */

  ret = ((jdn * 24 + tp->tm_hour) * 60 + tp->tm_min) * 60 + tp->tm_sec;
  linfo("ret=%d tm_hour=%d tm_min=%d tm_sec=%d\n",
        (int)ret, tp->tm_hour, tp->tm_min, tp->tm_sec);

  return ret;
}

time_t mktime(FAR struct tm *tp)
{
  return timegm(tp);
}
