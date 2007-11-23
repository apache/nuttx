/************************************************************
 * gmtime_r.c
 *
 *   Copyright (C) 2007 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name Gregory Nutt nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ************************************************************/

/************************************************************
 * Included Files
 ************************************************************/

#include <sys/types.h>
#include <time.h>
#include <errno.h>
#include <debug.h>
#include "clock_internal.h"

/************************************************************
 * Definitions
 ************************************************************/

/************************************************************
 * Private Type Declarations
 ************************************************************/

/************************************************************
 * Private Function Prototypes
 ************************************************************/

/* Calendar/UTC conversion routines */

static void   clock_utc2calendar(time_t utc, int *year, int *month, int *day);
static void   clock_utc2gregorian (time_t jdn, int *year, int *month, int *day);

#ifdef CONFIG_JULIAN_TIME
static void   clock_utc2julian(time_t jdn, int *year, int *month, int *day);
#endif /* CONFIG_JULIAN_TIME */

/**********************************************************
 * Public Constant Data
 **********************************************************/

/************************************************************
 * Public Variables
 ************************************************************/

/**********************************************************
 * Private Variables
 **********************************************************/

/************************************************************
 * Private Functions
 ************************************************************/

/************************************************************
 * Function:  clock_calendar2utc, clock_gregorian2utc,
 *            and clock_julian2utc
 *
 * Description:
 *    Calendar to UTC conversion routines.  These conversions
 *    are based on algorithms from p. 604 of Seidelman, P. K.
 *    1992.  Explanatory Supplement to the Astronomical
 *    Almanac.  University Science Books, Mill Valley. 
 *
 ************************************************************/

static void clock_utc2calendar(time_t utc, int *year, int *month, int *day)
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

static void clock_utc2gregorian(time_t jd, int *year, int *month, int *day)
{
  long	l, n, i, j, d, m, y;

  l = jd + 68569;
  n = (4*l) / 146097;
  l = l - (146097*n + 3)/4;
  i = (4000*(l+1))/1461001;
  l = l - (1461*i)/4 + 31;
  j = (80*l)/2447;
  d = l - (2447*j)/80;
  l = j/11;
  m = j + 2 - 12*l;
  y = 100*(n-49) + i + l;

  *year  = y;
  *month = m;
  *day   = d;
}


#ifdef CONFIG_JULIAN_TIME
static void clock_utc2julian(time_t jd, int *year, int *month, int *day)
{
  long	j, k, l, n, d, i, m, y;

  j = jd + 1402;
  k = (j-1)/1461;
  l = j - 1461*k;
  n = (l-1)/365 - l/1461;
  i = l - 365*n + 30;
  j = (80*i)/2447;
  d = i - (2447*j)/80;
  i = j/11;
  m = j + 2 - 12*i;
  y = 4*k + n + i - 4716;

  *year  = y;
  *month = m;
  *day   = d;
}
#endif /* CONFIG_JULIAN_TIME */

/************************************************************
 * Public Functions
 ************************************************************/

/************************************************************
 * Function:  gmtime_r
 *
 * Description:
 *  Time conversion (based on the POSIX API)
 *
 ************************************************************/

struct tm *gmtime_r(const time_t *clock, struct tm *result)
{
  time_t time;
  time_t jdn;
  int year, month, day;
  int hour, min, sec;

  /* Get the seconds since the EPOCH */

  time = *clock;
  sdbg("clock=%d\n", (int)time);

  /* Convert to days, hours, minutes, and seconds since the EPOCH */

  jdn   = time / (24*60*60);
  time -= (24*60*60) * jdn;

  hour  = time / (60*60);
  time -= (60*60) * hour;

  min   = time / 60;
  time -= 60 * min;

  sec   = time;

  sdbg("hour=%d min=%d sec=%d\n",
       (int)hour, (int)min, (int)sec);

  /* Convert the days since the EPOCH to calendar day */

  clock_utc2calendar(jdn, &year, &month, &day);

  sdbg("jdn=%d year=%d month=%d day=%d\n",
       (int)jdn, (int)year, (int)month, (int)day);

  /* Then return the struct tm contents */

  result->tm_year = (int)year - 1900;
  result->tm_mon  = (int)month - 1;
  result->tm_mday = (int)day;
  result->tm_hour = (int)hour;
  result->tm_min  = (int)min;
  result->tm_sec  = (int)sec;

  return result;
}
