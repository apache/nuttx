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

static const int g_mon_lengths[2][MONSPERYEAR] =
{
  {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31},
  {31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31}
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void adjust(FAR int *tenx, FAR int *x, int base)
{
  while (*x < 0)
    {
      *x += base;
      (*tenx)--;
    }

  while (*x > (base - 1))
    {
      *x -= base;
      (*tenx)++;
    }
}

static void normalize(FAR struct tm *tm)
{
  bool leapyear = false;
  int year;

  for (; ; )
    {
      /* Adjust mon field */

      adjust(&tm->tm_year, &tm->tm_mon, MONSPERYEAR);

      /* Get an absolute year */

      year = tm->tm_year + TM_YEAR_BASE;

      /* Is this a leap year? */

      leapyear = clock_isleapyear(year);

      /* Adjust mday field */

      while (tm->tm_mday < 1)
        {
          tm->tm_mon--;
          if (tm->tm_mon < 0)
            {
              tm->tm_mday += g_mon_lengths[leapyear][TM_DECEMBER];
              break;
            }

          tm->tm_mday += g_mon_lengths[leapyear][tm->tm_mon];
        }

      if (tm->tm_mon < 0)
        {
          continue;
        }

      while (tm->tm_mday > g_mon_lengths[leapyear][tm->tm_mon])
        {
          tm->tm_mday -= g_mon_lengths[leapyear][tm->tm_mon];
          tm->tm_mon++;
          if (tm->tm_mon > (MONSPERYEAR - 1))
            {
              break;
            }
        }

      if (tm->tm_mon > (MONSPERYEAR - 1))
        {
          continue;
        }

      /* Adjust seconds field */

      adjust(&tm->tm_min, &tm->tm_sec, SECSPERMIN);

      /* Adjust minutes field */

      adjust(&tm->tm_hour, &tm->tm_min, MINSPERHOUR);

      /* Adjust hours field */

      while (tm->tm_hour < 0)
        {
          tm->tm_hour += HOURSPERDAY;
          tm->tm_mday--;

          if (tm->tm_mday < 1)
            {
              break;
            }
        }

      if (tm->tm_mday < 1)
        {
          continue;
        }

      while (tm->tm_hour > (HOURSPERDAY - 1))
        {
          tm->tm_hour -= HOURSPERDAY;
          tm->tm_mday++;

          if (tm->tm_mday > g_mon_lengths[leapyear][tm->tm_mon])
            {
              break;
            }
        }

      if (tm->tm_mday > g_mon_lengths[leapyear][tm->tm_mon])
        {
          continue;
        }

      break;
    }

  /* Update the years field */

  tm->tm_year = year - TM_YEAR_BASE;

  /* Determine the day of the year; -1 because the mday is 1-indexed */

  tm->tm_yday = tm->tm_mday - 1 + clock_daysbeforemonth(tm->tm_mon,
                                                        leapyear);

  /* Finally calculate the weekday */

  tm->tm_wday = clock_dayoftheweek(tm->tm_mday, tm->tm_mon + 1, year);
}

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

  /* Normalize struct tm */

  normalize(tp);

  /* Get the EPOCH-relative julian date from the calendar year,
   * month, and date
   */

  jdn = clock_calendar2utc(tp->tm_year + TM_YEAR_BASE, tp->tm_mon,
                           tp->tm_mday);
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
