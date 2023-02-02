/****************************************************************************
 * libs/libc/time/lib_asctimer.c
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

#include <stdio.h>

#include <sys/param.h>

#include <nuttx/time.h>

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Note: These strings duplicate other definitions in other files.  These
 * definitions could be combined to save a little FLASH space.
 */

static const char * const g_wday_name[7] =
{
  "Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"
};

static const char * const g_mon_name[12] =
{
  "Jan", "Feb", "Mar", "Apr", "May", "Jun",
  "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  asctime_r
 *
 * Description:
 *   asctime and asctime_r convert the time provided in a struct tm to a
 *   string representation.  asctime is not re-entrant; asctime_r is re-
 *   entrant.
 *
 * Input Parameters:
 *   tp  - Pointer to the time to be converted.
 *   buf - A user provided buffer to receive the 26 character time string.
 *
 * Returned Value:
 *   On success, the pointer to the 'buf' is returned; on failure, NULL is
 *   returned.
 *
 ****************************************************************************/

FAR char *asctime_r(FAR const struct tm *tp, FAR char *buf)
{
  char tmp[128];

  if (tp == NULL ||
      tp->tm_wday >= nitems(g_wday_name) ||
      tp->tm_mon >= nitems(g_mon_name))
    {
      return NULL;
    }

  snprintf(tmp, sizeof(tmp), "%.3s %.3s%3d %.2d:%.2d:%.2d %d\n",
           g_wday_name[tp->tm_wday], g_mon_name[tp->tm_mon],
           tp->tm_mday, tp->tm_hour, tp->tm_min, tp->tm_sec,
           TM_YEAR_BASE + tp->tm_year);
  strlcpy(buf, tmp, 26);

  return buf;
}
