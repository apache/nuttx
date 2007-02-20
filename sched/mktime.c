/************************************************************
 * mktime.c
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
 * Function:  clock_gregorian2utc, clock_julian2utc
 *
 * Description:
 *    UTC conversion routines.  These conversions are based
 *    on algorithms from p. 604 of Seidelman, P. K. 1992.
 *    Explanatory Supplement to the Astronomical Almanac.
 *    University Science Books, Mill Valley. 
 *
 ************************************************************/

static time_t clock_gregorian2utc(int year, int month, int day)
{
  int temp;

  /* temp = (month - 14)/12; */

  temp = (month <= 2 ? -1:0);

  return (1461*(year + 4800 + temp))/4
    + (367*(month - 2 - 12*temp))/12
    - (3*((year + 4900 + temp)/100))/4 + day - 32075;
}

#ifdef CONFIG_JULIAN_TIME
static time_t clock_julian2utc(int year, int month, int day)
{
  return 367*year
    - (7*(year + 5001 + (month-9)/7))/4
    + (275*month)/9
    + day + 1729777;
}
#endif

/************************************************************
 * Public Functions
 ************************************************************/

/************************************************************
 * Function:  mktime
 *
 * Description:
 *  Time conversion (based on the POSIX API)
 *
 ************************************************************/

time_t mktime(struct tm *tp)
{
  time_t ret;
  time_t jdn;

  /* Get the EPOCH-relative julian date from the calendar year,
   * month, and date
   */

  jdn = clock_calendar2utc(tp->tm_year+1900, tp->tm_mon+1, tp->tm_mday);
  dbg("jdn=%d tm_year=%d tm_mon=%d tm_mday=%d\n",
      (int)jdn, tp->tm_year, tp->tm_mon, tp->tm_mday);

  /* Return the seconds into the julian day. */

  ret = ((jdn*24 + tp->tm_hour)*60 + tp->tm_min)*60 + tp->tm_sec;
  dbg("%s:\tret=%d tm_hour=%d tm_min=%d tm_sec=%d\n",
      (int)ret, tp->tm_hour, tp->tm_min, tp->tm_sec);

  return ret;
}

/************************************************************
 * Function:  clock_calendar2utc
 *
 * Description:
 *    Calendar/UTC conversion based on algorithms from p. 604
 *    of Seidelman, P. K. 1992.  Explanatory Supplement to
 *    the Astronomical Almanac.  University Science Books,
 *    Mill Valley. 
 *
 ************************************************************/

time_t clock_calendar2utc(int year, int month, int day)
{
  int dyear;
#ifdef CONFIG_JULIAN_TIME
  int isgreg;
#endif /* CONFIG_JULIAN_TIME */

  /* Correct year & month ranges.  Shift month into range 1-12 */

  dyear = (month-1) / 12;	
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
      isgreg = TRUE;
    }
  else if (year < GREG_YEAR)
    {
      isgreg = FALSE;
    }
  else if (month > GREG_MONTH)
    {
      isgreg = TRUE;
    }
  else if (month < GREG_MONTH)
    {
      isgreg = FALSE;
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
