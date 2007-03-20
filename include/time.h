/********************************************************************************
 * time.h
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
 ********************************************************************************/

#ifndef _TIME_H_
#define _TIME_H_

/********************************************************************************
 * Included Files
 ********************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>

/********************************************************************************
 * Compilations Switches
 ********************************************************************************/

/********************************************************************************
 * Definitions
 ********************************************************************************/

/* Clock tick of the system */

#define CLK_TCK 100

/* This is the only clock_id supported by the "Clock and Timer
 * Functions."
 */

#define CLOCK_REALTIME 0
#define CLOCK_ABSTIME

/********************************************************************************
 * Global Type Declarations
 ********************************************************************************/

typedef uint32 time_t;
typedef ubyte  clockid_t;
typedef ubyte  timer_t;

struct timespec
{
  time_t tv_sec;                   /* Seconds */
  long   tv_nsec;                  /* Nanoseconds */
};

struct timeval
{
  time_t tv_sec;                   /* Seconds */
  long tv_usec;                    /* Microseconds */
};

struct tm
{
  int tm_sec;     /* second (0-61, allows for leap seconds) */
  int tm_min;     /* minute (0-59) */
  int tm_hour;    /* hour (0-23) */
  int tm_mday;    /* day of the month (1-31) */
  int tm_mon;     /* month (0-11) */
  int tm_year;    /* years since 1900 */
#if 0 /* not supported */
  int tm_wday;    /* day of the week (0-6) */
  int tm_yday;    /* day of the year (0-365) */
  int tm_isdst;   /* non-0 if daylight savings time is in effect */
#endif
};

/* Struct itimerspec is used to define settings for an interval timer */

struct itimerspec
{
  struct timespec it_value;    /* First time */
  struct timespec it_interval; /* and thereafter */
};

/* forward reference (defined in signal.h) */

struct sigevent;

/********************************************************************************
 * Global Variables
 ********************************************************************************/

/* extern char *tznames[]; not supported */

/********************************************************************************
 * Global Function Prototypes
 ********************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

EXTERN int clock_settime(clockid_t clockid, const struct timespec *tp);
EXTERN int clock_gettime(clockid_t clockid, struct timespec *tp);
EXTERN int clock_getres(clockid_t clockid, struct timespec *res);

EXTERN time_t mktime(struct tm *tp);
EXTERN struct tm *gmtime_r(const time_t *clock, struct tm *result);
#define localtime_r(c,r) gmtime_r(c,r)

EXTERN int timer_create(clockid_t clockid, struct sigevent *evp, timer_t *timerid);
EXTERN int timer_delete(timer_t timerid);
EXTERN int timer_settime(timer_t timerid, int flags, const struct itimerspec *value,
                         struct itimerspec *ovalue);
EXTERN int timer_gettime(timer_t timerid, struct itimerspec *value);
EXTERN int timer_getoverrun(timer_t timerid);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif  /* _TIME_H_ */
