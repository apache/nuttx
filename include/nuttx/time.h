/****************************************************************************
 * include/nuttx/time.h
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

#ifndef __INCLUDE_NUTTX_TIME_H
#define __INCLUDE_NUTTX_TIME_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <time.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* If Gregorian time is not supported, then neither is Julian */

#ifndef CONFIG_GREGORIAN_TIME
#  undef CONFIG_JULIAN_TIME
#else
#  define JD_OF_EPOCH           2440588    /* Julian Date of noon, J1970 */

#  ifdef CONFIG_JULIAN_TIME
#    define GREG_DUTC           -141427    /* Default is October 15, 1582 */
#    define GREG_YEAR            1582
#    define GREG_MONTH           10
#    define GREG_DAY             15
#  endif /* CONFIG_JULIAN_TIME */
#endif /* !CONFIG_GREGORIAN_TIME */

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name:  clock_isleapyear
 *
 * Description:
 *    Return true if the specified year is a leap year
 *
 ****************************************************************************/

int clock_isleapyear(int year);

/****************************************************************************
 * Name:  clock_daysbeforemonth
 *
 * Description:
 *    Get the number of days that occurred before the beginning of the month.
 *
 ****************************************************************************/

int clock_daysbeforemonth(int month, bool leapyear);

/****************************************************************************
 * Name:  clock_dayoftheweek
 *
 * Description:
 *    Get the day of the week
 *
 * Input Parameters:
 *   mday  - The day of the month 1 - 31
 *   month - The month of the year 1 - 12
 *   year  - the year including the 1900
 *
 * Returned Value:
 *   Zero based day of the week 0-6, 0 = Sunday, 1 = Monday... 6 = Saturday
 *
 ****************************************************************************/

int clock_dayoftheweek(int mday, int month, int year);

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

time_t clock_calendar2utc(int year, int month, int day);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_TIME_H */
