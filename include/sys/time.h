/****************************************************************************
 * include/sys/time.h
 *
 *   Copyright (C) 2009, 2015-2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 * 3. Neither the name NuttX nor the names of its contributors may be
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
 ****************************************************************************/

#ifndef __INCLUDE_SYS_TIME_H
#define __INCLUDE_SYS_TIME_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <time.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The following are non-standard interfaces in the sense that they are not
 * in POSIX.1-2001 nor are they specified at OpenGroup.org. These interfaces
 * are present on most BSD derivatives, however, including Linux.
 */

/* void timeradd(FAR struct timeval *a, FAR struct timeval *b,
 *               FAR struct timeval *res);
 */

#define timeradd(tvp, uvp, vvp) \
  do \
    { \
      (vvp)->tv_sec = (tvp)->tv_sec + (uvp)->tv_sec; \
      (vvp)->tv_usec = (tvp)->tv_usec + (uvp)->tv_usec; \
      if ((vvp)->tv_usec >= 1000000) \
        { \
          (vvp)->tv_sec++; \
          (vvp)->tv_usec -= 1000000; \
        } \
    } \
  while (0)

/* void timersub(FAR struct timeval *a, FAR struct timeval *b,
 *               FAR struct timeval *res);
 */

#define timersub(tvp, uvp, vvp) \
  do \
    { \
      (vvp)->tv_sec = (tvp)->tv_sec - (uvp)->tv_sec; \
      (vvp)->tv_usec = (tvp)->tv_usec - (uvp)->tv_usec; \
      if ((vvp)->tv_usec < 0) \
        { \
          (vvp)->tv_sec--; \
          (vvp)->tv_usec += 1000000; \
        } \
    } \
  while (0)

/* void timerclear(FAR struct timeval *tvp); */

#define timerclear(tvp) \
  do \
    { \
      (tvp)->tv_sec = 0; \
      (tvp)->tv_usec = 0; \
    } \
  while (0)

/* int timerisset(FAR struct timeval *tvp); */

#define timerisset(tvp) \
  ((tvp)->tv_sec != 0 || (tvp)->tv_usec != 0)

/* int timercmp(FAR struct timeval *a, FAR struct timeval *b, CMP); */

#define timercmp(tvp, uvp, cmp) \
  (((tvp)->tv_sec == (uvp)->tv_sec) ? \
   ((tvp)->tv_usec cmp (uvp)->tv_usec) : \
   ((tvp)->tv_sec cmp (uvp)->tv_sec))

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

/* struct timeval represents time as seconds plus microseconds */

struct timeval
{
  time_t tv_sec;         /* Seconds */
  long tv_usec;          /* Microseconds */
};

/* The use of the struct timezone  is obsolete; the tz argument should
 * normally be specified as NULL (and is ignored in any event).
 */

struct timezone
{
  int tz_minuteswest;     /* Minutes west of Greenwich */
  int tz_dsttime;         /* Type of DST correction */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: gettimeofday
 *
 * Description:
 *   Get the current time
 *
 *   Conforming to SVr4, 4.3BSD. POSIX.1-2001 describes gettimeofday().
 *   POSIX.1-2008 marks gettimeofday() as obsolete, recommending the use of
 *   clock_gettime(2) instead.
 *
 *   NuttX implements gettimeofday() as a thin layer around clock_gettime();
 *
 * Input Parameters:
 *   tv - The location to return the current time
 *   tz - Ignored
 *
 * Returned Value:
 *   Zero (OK) on success;  -1 is returned on failure with the errno variable
 *   set appropriately.
 *
 ****************************************************************************/

int gettimeofday(FAR struct timeval *tv, FAR struct timezone *tz);

/****************************************************************************
 * Name: settimeofday
 *
 * Description:
 *   Set the current time
 *
 *   Conforming to SVr4, 4.3BSD. POSIX.1-2001 describes gettimeofday() but
 *   not settimeofday().
 *
 *   NuttX implements settimeofday() as a thin layer around clock_settime();
 *
 * Input Parameters:
 *   tv - The net to time to be set
 *   tz - Ignored
 *
 * Returned Value:
 *   Zero (OK) on success;  -1 is returned on failure with the errno variable
 *   set appropriately.
 *
 ****************************************************************************/

int settimeofday(FAR const struct timeval *tv, FAR struct timezone *tz);

/****************************************************************************
 * Name: adjtime
 *
 * Description:
 *   The adjtime() function gradually adjusts the system clock (as returned
 *   by gettimeofday(2)).  The amount of time by which the clock is to be
 *   adjusted is specified in the structure pointed to by delta.
 *
 *   This structure has the following form:
 *
 *     struct timeval
 *       {
 *         time_t      tv_sec;     (seconds)
 *         suseconds_t tv_usec;    (microseconds)
 *       };
 *
 *   If the adjustment in delta is positive, then the system clock is
 *   speeded up by some small percentage (i.e., by adding a small amount of
 *   time to the clock value in each second) until the adjustment has been
 *   completed.  If the adjustment in delta is negative, then the clock is
 *   slowed down in a similar fashion.
 *
 *   If a clock adjustment from an earlier adjtime() call is already in
 *   progress at the time of a later adjtime() call, and delta is not NULL
 *   for the later call, then the earlier adjustment is stopped, but any
 *   already completed part of that adjustment is not undone.
 *
 *   If olddelta is not NULL, then the buffer that it points to is used to
 *   return the amount of time remaining from any previous adjustment that
 *   has not yet been completed.
 *
 *   NOTE: This is not a POSIX interface but derives from 4.3BSD, System V.
 *   It is also supported for Linux compatibility.
 *
 ****************************************************************************/

#ifdef CONFIG_CLOCK_TIMEKEEPING
int adjtime(FAR const struct timeval *delta, FAR struct timeval *olddelta);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_SYS_TIME_H */
