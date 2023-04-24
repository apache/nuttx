/****************************************************************************
 * include/sys/time.h
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

#ifndef __INCLUDE_SYS_TIME_H
#define __INCLUDE_SYS_TIME_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <time.h>
#include <sys/select.h>
#include <sys/types.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ITIMER_REAL    0 /* Timers run in real time. */
#define ITIMER_VIRTUAL 1 /* Timers run only when the process is executing. */
#define ITIMER_PROF    2 /* Timers run when the process is executing and when
                          * the system is executing on behalf of the process.
                          */

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

/* Macros for converting between `struct timeval' and `struct timespec'.  */

#define TIMEVAL_TO_TIMESPEC(tv, ts) \
  do \
    { \
      (ts)->tv_sec = (tv)->tv_sec; \
      (ts)->tv_nsec = (tv)->tv_usec * 1000; \
    } \
  while (0)

#define TIMESPEC_TO_TIMEVAL(tv, ts) \
  do \
    { \
      (tv)->tv_sec = (ts)->tv_sec; \
      (tv)->tv_usec = (ts)->tv_nsec / 1000; \
    } \
  while (0)

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

typedef clock_t hrtime_t;

/* struct timeval represents time as seconds plus microseconds */

struct timeval
{
  time_t tv_sec;         /* Seconds */
  long tv_usec;          /* Microseconds */
};

/* Type of the second argument to `getitimer' and
 * the second and third arguments `setitimer'.
 */

struct itimerval
{
  struct timeval it_interval; /* Interval for periodic timer */
  struct timeval it_value;    /* Time until next expiration */
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

int settimeofday(FAR const struct timeval *tv,
                 FAR const struct timezone *tz);

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

#if defined(CONFIG_CLOCK_TIMEKEEPING) || defined(CONFIG_CLOCK_ADJTIME)
int adjtime(FAR const struct timeval *delta, FAR struct timeval *olddelta);
#endif

/****************************************************************************
 * Name: getitimer
 *
 * Description:
 *   The getitimer() function will store the amount of time until the
 *   specified timer, which, expires and the reload value of the timer
 *   into the space pointed to by the value argument. The it_value member
 *   of this structure will contain the amount of time before the timer
 *   expires, or zero if the timer is disarmed. This value is returned as
 *   the interval until timer expiration. The it_interval member of value
 *   will contain the reload value last set by setitime().
 *
 * Input Parameters:
 *   which - The predefined timer id
 *   value - The current timer value
 *
 * Returned Value:
 *   If the getitimer() succeeds, a value of 0 (OK) will be returned.
 *   If an error occurs, the value -1 (ERROR) will be returned, and errno
 *   set to indicate the error.
 *
 *   EINVAL - The which argument does not correspond to an predefined ID.
 *
 * Assumptions/Limitations:
 *   Due to the asynchronous operation of this function, the time reported
 *   by this function could be significantly more than that actual time
 *   remaining on the timer at any time.
 *
 ****************************************************************************/

int getitimer(int which, FAR struct itimerval *value);

/****************************************************************************
 * Name: setitimer
 *
 * Description:
 *   The setitimer() function sets the time until the next expiration of
 *   the timer specified by which from the it_value member of the value
 *   argument and arm the timer if the it_value member of value is non-zero.
 *   If the specified timer was already armed when setitimer() is
 *   called, this call will reset the time until next expiration to the
 *   value specified. If the it_value member of value is zero, the timer
 *   will be disarmed. The effect of disarming or resetting a timer with
 *   pending expiration notifications is unspecified.
 *
 *   The reload value of the timer will be set to the value specified by the
 *   it_interval member of value.  When a timer is armed with a non-zero
 *   it_interval, a periodic (or repetitive) timer is specified.
 *
 *   Time values that are between two consecutive non-negative integer
 *   multiples of the resolution of the specified timer will be rounded up
 *   to the larger multiple of the resolution. Quantization error will not
 *   cause the timer to expire earlier than the rounded time value.
 *
 *   If the argument ovalue is not NULL, the setitimer() function will
 *   store, in the location referenced by ovalue, a value representing the
 *   previous amount of time before the timer would have expired, or zero if
 *   the timer was disarmed, together with the previous timer reload value.
 *   Timers will not expire before their scheduled time.
 *
 * Input Parameters:
 *   which - The predefined timer id
 *   value - Specifies the timer value to set
 *   ovalue - A location in which to return the time remaining from the
 *     previous timer setting.
 *
 * Returned Value:
 *   If the setitimer() succeeds, a value of 0 (OK) will be returned.
 *   If an error occurs, the value -1 (ERROR) will be returned, and errno set
 *   to indicate the error.
 *
 *   EINVAL - The which argument does not correspond to an predefined ID.
 *   EINVAL - A value structure specified a microsecond value less than zero
 *     or greater than or equal to 1000 million, and the it_value member of
 *     that structure did not specify zero seconds and nanoseconds.
 *
 * Assumptions:
 *
 ****************************************************************************/

int setitimer(int which, FAR const struct itimerval *value,
              FAR struct itimerval *ovalue);

/****************************************************************************
 * Name: utimes
 *
 * Description:
 *   The utimes() function shall set the access and modification times of
 *   the file pointed to by the path argument to the value of the times
 *   argument. utimes() function allows time specifications accurate to
 *   the microsecond.
 *
 *   For utimes(), the times argument is an array of timeval structures.
 *   The first array member represents the date and time of last access,
 *   and the second member represents the date and time of last
 *   modification. The times in the timeval structure are measured in
 *   seconds and microseconds since the Epoch, although rounding toward
 *   the nearest second may occur.
 *
 *   If the times argument is a null pointer, the access and modification
 *   times of the file shall be set to the current time. The effective
 *   user ID of the process shall match the owner of the file, has write
 *   access to the file or appropriate privileges to use this call in this
 *   manner. Upon completion, utimes() shall mark the time of the last
 *   file status change, st_ctime, for update.
 *
 * Input Parameters:
 *   path  - Specifies the file to be modified
 *   times - Specifies the time value to set
 *
 * Returned Value:
 *   Upon successful completion, 0 shall be returned. Otherwise, -1 shall
 *   be returned and errno shall be set to indicate the error, and the file
 *   times shall not be affected.
 *
 ****************************************************************************/

int utimes(FAR const char *path, const struct timeval times[2]);
int lutimes(FAR const char *path, const struct timeval times[2]);
int futimesat(int dirfd, FAR const char *path,
              const struct timeval times[2]);

/****************************************************************************
 * Name: futimes
 *
 * Description:
 *   futimes() update the timestamps of a file with microsecond precision.
 *   With futimes() the file whose timestamps are to be updated is specified
 *   via an open file descriptor, fd.
 *
 * Input Parameters:
 *   fd  - Specifies the fd to be modified
 *   times - Specifies the time value to set
 *
 * Returned Value:
 *   On success, futimes() return 0.
 *   On error, -1 is returned and errno is set to indicate the error.
 *
 ****************************************************************************/

int futimes(int fd, const struct timeval tv[2]);

/****************************************************************************
 * Name: gethrtime
 *
 * Description:
 *   Get the current time
 *
 * Returned Value:
 *   The current value of the system time in ns
 *
 ****************************************************************************/

hrtime_t gethrtime(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_SYS_TIME_H */
