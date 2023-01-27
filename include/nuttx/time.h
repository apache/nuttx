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
#  define JD_OF_EPOCH       2440588    /* Julian Date of noon, J1970 */

#  ifdef CONFIG_JULIAN_TIME
#    define GREG_DUTC       -141427    /* Default is October 15, 1582 */
#    define GREG_YEAR       1582
#    define GREG_MONTH      10
#    define GREG_DAY        15
#  endif /* CONFIG_JULIAN_TIME */
#endif /* !CONFIG_GREGORIAN_TIME */

#define SECSPERMIN          60
#define MINSPERHOUR         60
#define HOURSPERDAY         24
#define DAYSPERWEEK         7
#define DAYSPERNYEAR        365
#define DAYSPERLYEAR        366
#define MONSPERYEAR         12

#define TM_SUNDAY           0
#define TM_MONDAY           1
#define TM_TUESDAY          2
#define TM_WEDNESDAY        3
#define TM_THURSDAY         4
#define TM_FRIDAY           5
#define TM_SATURDAY         6

#define TM_JANUARY          0
#define TM_FEBRUARY         1
#define TM_MARCH            2
#define TM_APRIL            3
#define TM_MAY              4
#define TM_JUNE             5
#define TM_JULY             6
#define TM_AUGUST           7
#define TM_SEPTEMBER        8
#define TM_OCTOBER          9
#define TM_NOVEMBER         10
#define TM_DECEMBER         11

#define TM_YEAR_BASE        1900
#define TM_WDAY_BASE        TM_MONDAY

#define EPOCH_YEAR          1970
#define EPOCH_WDAY          TM_THURSDAY

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
