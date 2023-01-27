/****************************************************************************
 * libs/libc/time/lib_gmtimer.c
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

#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/time.h>
#include <nuttx/clock.h>

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Calendar/UTC conversion routines */

static void   clock_utc2calendar(time_t utc, FAR int *year, FAR int *month,
                                 FAR int *day);
#ifdef CONFIG_GREGORIAN_TIME
static void   clock_utc2gregorian(time_t jdn, FAR int *year, FAR int *month,
                                  FAR int *day);
#ifdef CONFIG_JULIAN_TIME
static void   clock_utc2julian(time_t jdn, FAR int *year, FAR int *month,
                               FAR int *day);
#endif /* CONFIG_JULIAN_TIME */
#endif /* CONFIG_GREGORIAN_TIME */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  clock_utc2calendar, clock_utc2gregorian, and clock_utc2julian
 *
 * Description:
 *    Calendar to UTC conversion routines.  These conversions
 *    are based on algorithms from p. 604 of Seidelman, P. K.
 *    1992.  Explanatory Supplement to the Astronomical
 *    Almanac.  University Science Books, Mill Valley.
 *
 ****************************************************************************/

#ifdef CONFIG_GREGORIAN_TIME
static void clock_utc2calendar(time_t utc, FAR int *year, FAR int *month,
                               FAR int *day)
{
#ifdef CONFIG_JULIAN_TIME

  if (utc >= GREG_DUTC)
    {
      clock_utc2gregorian(utc + JD_OF_EPOCH, year, month, day);
    }
  else
    {
      clock_utc2julian (utc + JD_OF_EPOCH, year, month, day);
    }

#else /* CONFIG_JULIAN_TIME */

  clock_utc2gregorian(utc + JD_OF_EPOCH, year, month, day);

#endif /* CONFIG_JULIAN_TIME */
}

static void clock_utc2gregorian(time_t jd, FAR int *year, FAR int *month,
                                FAR int *day)
{
  long l;
  long n;
  long i;
  long j;
  long d;
  long m;
  long y;

  l = jd + 68569;
  n = (4 * l) / 146097;
  l = l - (146097 * n + 3) / 4;
  i = (4000 * (l + 1)) / 1461001;
  l = l - (1461 * i) / 4 + 31;
  j = (80 * l) / 2447;
  d = l - (2447 * j) / 80;
  l = j / 11;
  m = j + 2 - 12 * l;
  y = 100 * (n - 49) + i + l;

  *year  = y;
  *month = m;
  *day   = d;
}

#ifdef CONFIG_JULIAN_TIME

static void clock_utc2julian(time_t jd, FAR int *year, FAR int *month,
                             FAR int *day)
{
  long j;
  long k;
  long l;
  long n;
  long d;
  long i;
  long m;
  long y;

  j = jd + 1402;
  k = (j - 1) / 1461;
  l = j - 1461 * k;
  n = (l - 1) / 365 - l / 1461;
  i = l - 365 * n + 30;
  j = (80 * i) / 2447;
  d = i - (2447 * j) / 80;
  i = j / 11;
  m = j + 2 - 12 * i;
  y = 4 * k + n + i - 4716;

  *year  = y;
  *month = m;
  *day   = d;
}

#endif /* CONFIG_JULIAN_TIME */
#else/* CONFIG_GREGORIAN_TIME */

/* Only handles dates since Jan 1, 1970 */

static void clock_utc2calendar(time_t days, FAR int *year, FAR int *month,
                               FAR int *day)
{
  int  value;
  int  min;
  int  max;
  int  tmp;
  bool leapyear;

  /* There is one leap year every four years, so we can get close with the
   * following:
   */

  value   = days  / (4 * DAYSPERNYEAR + 1); /* Number of 4-years periods since the epoch */
  days   -= value * (4 * DAYSPERNYEAR + 1); /* Remaining days */
  value <<= 2;                              /* Years since the epoch */

  /* Then we will brute force the next 0-3 years
   *
   * Is this year a leap year? (we'll need this later too)
   */

  leapyear = clock_isleapyear(value + EPOCH_YEAR);

  /* Get the number of days in the year */

  tmp = (leapyear ? DAYSPERLYEAR : DAYSPERNYEAR);

  /* Do we have that many days left to account for? */

  while (days >= tmp)
    {
      /* Yes.. bump up the year and subtract the number of days in the year */

      value++;
      days -= tmp;

      /* Is the next year a leap year? */

      leapyear = clock_isleapyear(value + EPOCH_YEAR);

      /* Get the number of days in the next year */

      tmp = (leapyear ? DAYSPERLYEAR : DAYSPERNYEAR);
    }

  /* At this point, 'value' has the years since 1970 and 'days' has number
   * of days into that year.  'leapyear' is true if the year in 'value' is
   * a leap year.
   */

  *year = EPOCH_YEAR + value;

  /* Handle the month (zero based) */

  min = 0;
  max = 11;

  do
    {
      /* Get the midpoint */

      value = (min + max) >> 1;

      /* Get the number of days that occurred before the beginning of the
       * month following the midpoint.
       */

      tmp = clock_daysbeforemonth(value + 1, leapyear);

      /* Does the number of days before this month that equal or exceed the
       * number of days we have remaining?
       */

      if (tmp > days)
        {
          /* Yes.. then the month we want is somewhere from 'min' and to the
           * midpoint, 'value'.  Could it be the midpoint?
           */

          tmp = clock_daysbeforemonth(value, leapyear);
          if (tmp > days)
            {
              /* No... The one we want is somewhere between min and value-1 */

              max = value - 1;
            }
          else
            {
              /* Yes.. 'value' contains the month that we want */

              break;
            }
        }
      else
        {
          /* No... The one we want is somewhere between value+1 and max */

          min = value + 1;
        }

      /* If we break out of the loop because min == max, then we want value
       * to be equal to min == max.
       */

      value = min;
    }
  while (min < max);

  /* The selected month number is in value. Subtract the number of days in
   * the selected month
   */

  days -= clock_daysbeforemonth(value, leapyear);

  /* At this point, value has the month into this year (zero based) and days
   * has number of days into this month (zero based)
   */

  *month = value + 1; /* 1-based */
  *day   = days + 1;  /* 1-based */
}

#endif /* CONFIG_GREGORIAN_TIME */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  gmtime_r
 *
 * Description:
 *  Time conversion (based on the POSIX API)
 *
 ****************************************************************************/

FAR struct tm *gmtime_r(FAR const time_t *timep, FAR struct tm *result)
{
  time_t epoch;
  time_t jdn;
  int    year;
  int    month;
  int    day;
  int    hour;
  int    min;
  int    sec;

  /* Get the seconds since the EPOCH */

  epoch = *timep;
  linfo("timer=%d\n", (int)epoch);

  /* Convert to days, hours, minutes, and seconds since the EPOCH */

  jdn    = epoch / SEC_PER_DAY;
  epoch -= SEC_PER_DAY * jdn;

  hour   = epoch / SEC_PER_HOUR;
  epoch -= SEC_PER_HOUR * hour;

  min    = epoch / SEC_PER_MIN;
  epoch -= SEC_PER_MIN * min;

  sec    = epoch;

  linfo("hour=%d min=%d sec=%d\n", hour, min, sec);

  /* Convert the days since the EPOCH to calendar day */

  clock_utc2calendar(jdn, &year, &month, &day);

  linfo("jdn=%d year=%d month=%d day=%d\n", (int)jdn, year, month, day);

  /* Then return the struct tm contents */

  result->tm_year   = year - TM_YEAR_BASE; /* Relative to 1900 */
  result->tm_mon    = month - 1;           /* zero-based */
  result->tm_mday   = day;                 /* one-based */
  result->tm_hour   = hour;
  result->tm_min    = min;
  result->tm_sec    = sec;

  result->tm_wday   = clock_dayoftheweek(day, month, year);
  result->tm_yday   = day - 1 +
                      clock_daysbeforemonth(result->tm_mon,
                                            clock_isleapyear(year));
  result->tm_isdst  = 0;
  result->tm_gmtoff = 0;
  result->tm_zone   = NULL;

  return result;
}

FAR struct tm *localtime_r(FAR const time_t *timep, FAR struct tm *result)
{
  return gmtime_r(timep, result);
}
