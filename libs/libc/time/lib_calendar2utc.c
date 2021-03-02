/****************************************************************************
 * libs/libc/time/lib_calendar2utc.c
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
#include <time.h>
#include <debug.h>

#include <nuttx/time.h>

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  clock_gregorian2utc, clock_julian2utc
 *
 * Description:
 *    UTC conversion routines.  These conversions are based
 *    on algorithms from p. 604 of Seidelman, P. K. 1992.
 *    Explanatory Supplement to the Astronomical Almanac.
 *    University Science Books, Mill Valley.
 *
 ****************************************************************************/

#ifdef CONFIG_GREGORIAN_TIME
static time_t clock_gregorian2utc(int year, int month, int day)
{
  int temp;

  /* temp = (month - 14)/12; */

  temp = (month <= 2 ? -1:0);

  return (1461 * (year + 4800 + temp)) / 4
    + (367 * (month - 2 - 12 * temp)) / 12
    - (3 * ((year + 4900 + temp) / 100)) / 4 + day - 32075;
}

#ifdef CONFIG_JULIAN_TIME
static time_t clock_julian2utc(int year, int month, int day)
{
  return 367 * year
    - (7 * (year + 5001 + (month - 9) / 7)) / 4
    + (275 * month) / 9
    + day + 1729777;
}
#endif /* CONFIG_JULIAN_TIME */
#endif /* CONFIG_GREGORIAN_TIME */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  clock_calendar2utc
 *
 * Description:
 *    Calendar/UTC conversion based on algorithms from p. 604
 *    of Seidelman, P. K. 1992.  Explanatory Supplement to
 *    the Astronomical Almanac.  University Science Books,
 *    Mill Valley.
 *
 ****************************************************************************/

#ifdef CONFIG_GREGORIAN_TIME
time_t clock_calendar2utc(int year, int month, int day)
{
  int dyear;
#ifdef CONFIG_JULIAN_TIME
  bool isgreg;
#endif /* CONFIG_JULIAN_TIME */

  /* Correct year & month ranges.  Shift month into range 1-12 */

  dyear = (month - 1) / 12;
  month -= 12 * dyear;
  year += dyear;

  if (month < 1)
    {
      month += 12;
      year -= 1;
    }

#ifdef CONFIG_JULIAN_TIME
  /* Determine which calendar to use */

  if (year > GREG_YEAR)
    {
      isgreg = true;
    }
  else if (year < GREG_YEAR)
    {
      isgreg = false;
    }
  else if (month > GREG_MONTH)
    {
      isgreg = true;
    }
  else if (month < GREG_MONTH)
    {
      isgreg = false;
    }
  else
    {
      isgreg = (day >= GREG_DAY);
    }

  /* Calculate and return date */

  if (isgreg)
    {
      return clock_gregorian2utc(year, month, day) - JD_OF_EPOCH;
    }
  else
    {
      return clock_julian2utc (year, month, day) - JD_OF_EPOCH;
    }

#else /* CONFIG_JULIAN_TIME */

  return clock_gregorian2utc(year, month, day) - JD_OF_EPOCH;

#endif /* CONFIG_JULIAN_TIME */
}
#else

/* A highly simplified version that only handles days in the time
 * since Jan 1, 1970.
 */

time_t clock_calendar2utc(int year, int month, int day)
{
  time_t days;

  /* Years since epoch in units of days (ignoring leap years). */

  days = (year - 1970) * 365;

  /* Add in the extra days for the leap years prior to the current year. */

  days += (year - 1969) >> 2;

  /* Add in the days up to the beginning of this month. */

  days += (time_t)clock_daysbeforemonth(month, clock_isleapyear(year));

  /* Add in the days since the beginning of this month (days are 1-based). */

  days += day - 1;

  /* Then convert the seconds and add in hours, minutes, and seconds */

  return days;
}
#endif /* CONFIG_GREGORIAN_TIME */
