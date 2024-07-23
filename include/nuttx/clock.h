/****************************************************************************
 * include/nuttx/clock.h
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

#ifndef __INCLUDE_NUTTX_CLOCK_H
#define __INCLUDE_NUTTX_CLOCK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdbool.h>
#include <stdint.h>
#include <time.h>

#include <nuttx/compiler.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Efficient, direct access to OS global timer variables will be supported
 * if the execution environment has direct access to kernel global data.
 * The code in this execution context can access the kernel global data
 * directly if:
 *
 * 1. We are not running tick-less (in which case there is no global timer
 *    data),
 * 2. This is an un-protected, non-kernel build,
 * 3. This is a protected build, but this code is being built for execution
 *    within the kernel space.
 * 4. It we are building with SYSCALLs enabled, but not in a kernel build,
 *    then we can't know a priori whether the code has access to the
 *    global variables or not.  In that case we have to assume not.
 */

#undef __HAVE_KERNEL_GLOBALS
#if defined(CONFIG_SCHED_TICKLESS)
  /* Case 1: There is no global timer data */

#elif defined(__KERNEL__)
  /* Case 3: Kernel mode of protected/kernel build */

#  define __HAVE_KERNEL_GLOBALS 1

#elif defined(CONFIG_LIB_SYSCALL)
  /* Case 4: Building with SYSCALLs enabled, but not part of a kernel build */

#else
  /* Case 2: Un-protected, non-kernel build */

#  define __HAVE_KERNEL_GLOBALS 1
#endif

/* If CONFIG_SYSTEM_TIME64 is selected and the CPU supports long long types,
 * then a 64-bit system time will be used.
 */

#ifndef CONFIG_HAVE_LONG_LONG
#  undef CONFIG_SYSTEM_TIME64
#endif

/* The following are the bit fields of the clockid_t
 * bit 0~2: the clock type
 * CLOCK_REALTIME           - 0
 * CLOCK_MONOTONIC          - 1
 * CLOCK_PROCESS_CPUTIME_ID - 2
 * CLOCK_THREAD_CPUTIME_ID  - 3
 * CLOCK_BOOTTIME           - 4
 * bit 3~32: the pid or tid value
 *
 * The CLOCK_MASK are using to extract the clock_type from the clockid_t
 */

#define CLOCK_MASK            7
#define CLOCK_SHIFT           3

/* Timing constants *********************************************************/

#define NSEC_PER_SEC          1000000000L /* Seconds */
#define USEC_PER_SEC             1000000L
#define MSEC_PER_SEC                1000
#define DSEC_PER_SEC                  10
#define HSEC_PER_SEC                   2

#define NSEC_PER_HSEC          500000000L /* Half seconds */
#define USEC_PER_HSEC             500000L
#define MSEC_PER_HSEC                500
#define DSEC_PER_HSEC                  5

#define NSEC_PER_DSEC          100000000L /* Deciseconds */
#define USEC_PER_DSEC             100000L
#define MSEC_PER_DSEC                100

#define NSEC_PER_MSEC            1000000L /* Milliseconds */
#define USEC_PER_MSEC               1000

#define NSEC_PER_USEC               1000  /* Microseconds */

#define SEC_PER_MIN                   60
#define NSEC_PER_MIN           (NSEC_PER_SEC * SEC_PER_MIN)
#define USEC_PER_MIN           (USEC_PER_SEC * SEC_PER_MIN)
#define MSEC_PER_MIN           (MSEC_PER_SEC * SEC_PER_MIN)
#define DSEC_PER_MIN           (DSEC_PER_SEC * SEC_PER_MIN)
#define HSEC_PER_MIN           (HSEC_PER_SEC * SEC_PER_MIN)

#define MIN_PER_HOUR                  60
#define NSEC_PER_HOUR          (NSEC_PER_MIN * MIN_PER_HOUR)
#define USEC_PER_HOUR          (USEC_PER_MIN * MIN_PER_HOUR)
#define MSEC_PER_HOUR          (MSEC_PER_MIN * MIN_PER_HOUR)
#define DSEC_PER_HOUR          (DSEC_PER_MIN * MIN_PER_HOUR)
#define HSEC_PER_HOUR          (HSEC_PER_MIN * MIN_PER_HOUR)
#define SEC_PER_HOUR           (SEC_PER_MIN  * MIN_PER_HOUR)

#define HOURS_PER_DAY                 24
#define SEC_PER_DAY            (HOURS_PER_DAY * SEC_PER_HOUR)

/* If CONFIG_SCHED_TICKLESS is not defined, then the interrupt interval of
 * the system timer is given by USEC_PER_TICK.  This is the expected number
 * of microseconds between calls from the processor-specific logic to
 * nxsched_process_timer().  The default value of USEC_PER_TICK is 10000
 * microseconds (100 Hz).  However, this default setting can be overridden
 * by defining the interval in microseconds as CONFIG_USEC_PER_TICK in the
 * NuttX configuration file.
 *
 * The following calculations are only accurate when (1) there is no
 * truncation involved and (2) the underlying system timer is an even
 * multiple of microseconds.  If (2) is not true, you will probably want
 * to redefine all of the following.
 */

#ifdef CONFIG_USEC_PER_TICK
#  define USEC_PER_TICK       (CONFIG_USEC_PER_TICK)
#else
#  define USEC_PER_TICK       (10000)
#endif

/* MSEC_PER_TICK can be very inaccurate if CONFIG_USEC_PER_TICK is not an
 * even multiple of milliseconds.  Calculations using USEC_PER_TICK are
 * preferred for that reason (at the risk of overflow)
 */

/* TICK_PER_* truncates! */

#define TICK_PER_HOUR         (USEC_PER_HOUR / USEC_PER_TICK)
#define TICK_PER_MIN          (USEC_PER_MIN  / USEC_PER_TICK)
#define TICK_PER_SEC          (USEC_PER_SEC  / USEC_PER_TICK)
#define TICK_PER_MSEC         (USEC_PER_MSEC / USEC_PER_TICK)
#define TICK_PER_DSEC         (USEC_PER_DSEC / USEC_PER_TICK)
#define TICK_PER_HSEC         (USEC_PER_HSEC / USEC_PER_TICK)

/* MSEC_PER_TICK truncates! */

#define MSEC_PER_TICK         (USEC_PER_TICK / USEC_PER_MSEC)
#define NSEC_PER_TICK         (USEC_PER_TICK * NSEC_PER_USEC)

/* ?SEC2TIC rounds up */

#define NSEC2TICK(nsec)       (((nsec) + (NSEC_PER_TICK - 1)) / NSEC_PER_TICK)
#define USEC2TICK(usec)       (((usec) + (USEC_PER_TICK - 1)) / USEC_PER_TICK)

#if (MSEC_PER_TICK * USEC_PER_MSEC) == USEC_PER_TICK
#  define MSEC2TICK(msec)     (((msec) + (MSEC_PER_TICK - 1)) / MSEC_PER_TICK)
#else
#  define MSEC2TICK(msec)     USEC2TICK((msec) * USEC_PER_MSEC)
#endif

#define DSEC2TICK(dsec)       MSEC2TICK((dsec) * MSEC_PER_DSEC)
#define HSEC2TICK(dsec)       MSEC2TICK((dsec) * MSEC_PER_HSEC)
#define SEC2TICK(sec)         MSEC2TICK((sec)  * MSEC_PER_SEC)

#define TICK2NSEC(tick)       ((tick) * NSEC_PER_TICK)
#define TICK2USEC(tick)       ((tick) * USEC_PER_TICK)

#if (MSEC_PER_TICK * USEC_PER_MSEC) == USEC_PER_TICK
#  define TICK2MSEC(tick)     ((tick) * MSEC_PER_TICK)
#else
#  define TICK2MSEC(tick)     (((tick) * USEC_PER_TICK) / USEC_PER_MSEC)
#endif

/* TIC2?SEC rounds to nearest */

#define TICK2DSEC(tick)       (((tick) + (TICK_PER_DSEC / 2)) / TICK_PER_DSEC)
#define TICK2HSEC(tick)       (((tick) + (TICK_PER_HSEC / 2)) / TICK_PER_HSEC)
#define TICK2SEC(tick)        (((tick) + (TICK_PER_SEC / 2)) / TICK_PER_SEC)

#if defined(CONFIG_DEBUG_SCHED) && defined(CONFIG_SYSTEM_TIME64) && \
    !defined(CONFIG_SCHED_TICKLESS)
/* Initial system timer ticks value close to maximum 32-bit value, to test
 * 64-bit system-timer after going over 32-bit value. This is to make errors
 * of casting 64-bit system-timer to 32-bit variables more visible.
 */

#  define INITIAL_SYSTEM_TIMER_TICKS \
    ((uint64_t)(UINT32_MAX - (TICK_PER_SEC * 5)))
#else
#  define INITIAL_SYSTEM_TIMER_TICKS 0
#endif

/* If Gregorian time is not supported, then neither is Julian */

#ifndef CONFIG_GREGORIAN_TIME
#  undef CONFIG_JULIAN_TIME
#else
#  define JD_OF_EPOCH         2440588    /* Julian Date of noon, J1970 */

#  ifdef CONFIG_JULIAN_TIME
#    define GREG_DUTC         -141427    /* Default is October 15, 1582 */
#    define GREG_YEAR         1582
#    define GREG_MONTH        10
#    define GREG_DAY          15
#  endif /* CONFIG_JULIAN_TIME */
#endif /* !CONFIG_GREGORIAN_TIME */

#define SECSPERMIN            60
#define MINSPERHOUR           60
#define HOURSPERDAY           24
#define DAYSPERWEEK           7
#define DAYSPERNYEAR          365
#define DAYSPERLYEAR          366
#define MONSPERYEAR           12

#define TM_SUNDAY             0
#define TM_MONDAY             1
#define TM_TUESDAY            2
#define TM_WEDNESDAY          3
#define TM_THURSDAY           4
#define TM_FRIDAY             5
#define TM_SATURDAY           6

#define TM_JANUARY            0
#define TM_FEBRUARY           1
#define TM_MARCH              2
#define TM_APRIL              3
#define TM_MAY                4
#define TM_JUNE               5
#define TM_JULY               6
#define TM_AUGUST             7
#define TM_SEPTEMBER          8
#define TM_OCTOBER            9
#define TM_NOVEMBER           10
#define TM_DECEMBER           11

#define TM_YEAR_BASE          (1900)
#define TM_WDAY_BASE          TM_MONDAY

#define EPOCH_YEAR            1970
#define EPOCH_WDAY            TM_THURSDAY

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This structure is used to report CPU usage for a particular thread */

#ifndef CONFIG_SCHED_CPULOAD_NONE
struct cpuload_s
{
  volatile clock_t total;   /* Total number of clock ticks */
  volatile clock_t active;  /* Number of ticks while this thread was active */
};
#endif

/* This non-standard type used to hold relative clock ticks that may take
 * negative values.  Because of its non-portable nature the type sclock_t
 * should be used only within the OS proper and not by portable applications.
 */

#ifdef CONFIG_SYSTEM_TIME64
typedef int64_t sclock_t;
#else
typedef int32_t sclock_t;
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/* Access to raw system clock ***********************************************/

/* Direct access to the system timer/counter is supported only if (1) the
 * system timer counter is available (i.e., we are not configured to use
 * a hardware periodic timer), and (2) the execution environment has direct
 * access to kernel global data
 */

#ifdef __HAVE_KERNEL_GLOBALS
EXTERN volatile clock_t g_system_ticks;

#  ifndef CONFIG_SYSTEM_TIME64
#    define clock_systime_ticks() g_system_ticks
#  endif
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#define timespec_from_tick(ts, tick) \
  do \
    { \
      clock_t _tick = (tick); \
      (ts)->tv_sec = _tick / TICK_PER_SEC; \
      _tick -= (clock_t)(ts)->tv_sec * TICK_PER_SEC; \
      (ts)->tv_nsec = _tick * NSEC_PER_TICK; \
    } \
  while (0)

#define timespec_to_tick(ts) \
  ((clock_t)(ts)->tv_sec * TICK_PER_SEC + (ts)->tv_nsec / NSEC_PER_TICK)

/****************************************************************************
 * Name: clock_timespec_compare
 *
 * Description:
 *    Return < 0 if time ts1 is before time ts2
 *    Return > 0 if time ts2 is before time ts1
 *    Return 0 if time ts1 is the same as time ts2
 *
 ****************************************************************************/

int clock_timespec_compare(FAR const struct timespec *ts1,
                           FAR const struct timespec *ts2);

/****************************************************************************
 * Name:  clock_timespec_add
 *
 * Description:
 *   Add timespec ts1 to to2 and return the result in ts3
 *
 * Input Parameters:
 *   ts1 and ts2: The two timespecs to be added
 *   t23: The location to return the result (may be ts1 or ts2)
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void clock_timespec_add(FAR const struct timespec *ts1,
                        FAR const struct timespec *ts2,
                        FAR struct timespec *ts3);

/****************************************************************************
 * Name:  clock_timespec_subtract
 *
 * Description:
 *   Subtract timespec ts2 from to1 and return the result in ts3.
 *   Zero is returned if the time difference is negative.
 *
 * Input Parameters:
 *   ts1 and ts2: The two timespecs to be subtracted (ts1 - ts2)
 *   t23: The location to return the result (may be ts1 or ts2)
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void clock_timespec_subtract(FAR const struct timespec *ts1,
                             FAR const struct timespec *ts2,
                             FAR struct timespec *ts3);

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

/****************************************************************************
 * Name:  clock_synchronize
 *
 * Description:
 *   Synchronize the system timer to a hardware RTC.  This operation is
 *   normally performed automatically by the system during clock
 *   initialization.  However, the user may also need to explicitly re-
 *   synchronize the system timer to the RTC under certain conditions where
 *   the system timer is known to be in error.  For example, in certain low-
 *   power states, the system timer may be stopped but the RTC will continue
 *   keep correct time.  After recovering from such low-power state, this
 *   function should be called to restore the correct system time.
 *
 *   Calling this function could result in system time going "backward" in
 *   time, especially with certain lower resolution RTC implementations.
 *   Time going backward could have bad consequences if there are ongoing
 *   timers and delays.  So use this interface with care.
 *
 * Input Parameters:
 *   tp: rtc time should be synced, set NULL to re-get time
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

#ifdef CONFIG_RTC
void clock_synchronize(FAR const struct timespec *tp);
#endif

/****************************************************************************
 * Name: clock_resynchronize
 *
 * Description:
 *   Resynchronize the system timer to a hardware RTC.  The user can
 *   explicitly re-synchronize the system timer to the RTC under certain
 *   conditions where the system timer is known to be in error.  For example,
 *   in certain low-power states, the system timer may be stopped but the
 *   RTC will continue keep correct time.  After recovering from such
 *   low-power state, this function should be called to restore the correct
 *   system time. Function also keeps monotonic clock at rate of RTC.
 *
 *   Calling this function will not result in system time going "backward" in
 *   time. If setting system time with RTC would result time going "backward"
 *   then resynchronization is not performed.
 *
 * Input Parameters:
 *   diff:  amount of time system-time is adjusted forward with RTC
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

#if defined(CONFIG_RTC) && !defined(CONFIG_SCHED_TICKLESS)
void clock_resynchronize(FAR struct timespec *rtc_diff);
#endif

/****************************************************************************
 * Name: clock_systime_ticks
 *
 * Description:
 *   Return the current value of the 32/64-bit system timer counter.
 *
 *   Indirect access to the system timer counter is required through this
 *   function if the execution environment does not have direct access to
 *   kernel global data.
 *
 *   Use of this function is also required to assure atomic access to the
 *   64-bit system timer.
 *
 *   NOTE:  This is an internal OS interface and should not be called from
 *   application code.  Rather, the functionally equivalent, standard
 *   interface clock() should be used.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   The current value of the system timer counter
 *
 ****************************************************************************/

#if !defined(__HAVE_KERNEL_GLOBALS) || defined(CONFIG_SYSTEM_TIME64)
clock_t clock_systime_ticks(void);
#endif

/****************************************************************************
 * Name: clock_time2ticks
 *
 * Description:
 *   Return the given struct timespec as systime ticks.
 *
 *   NOTE:  This is an internal OS interface and should not be called from
 *   application code.
 *
 * Input Parameters:
 *   reltime - Pointer to the time presented as struct timespec
 *
 * Output Parameters:
 *   ticks - Pointer to receive the time value presented as systime ticks
 *
 * Returned Value:
 *   Always returns OK (0)
 *
 ****************************************************************************/

int clock_time2ticks(FAR const struct timespec *reltime,
                     FAR sclock_t *ticks);

/****************************************************************************
 * Name: clock_ticks2time
 *
 * Description:
 *   Return the given systime ticks as a struct timespec.
 *
 *   NOTE:  This is an internal OS interface and should not be called from
 *   application code.
 *
 * Input Parameters:
 *   ticks - Time presented as systime ticks
 *
 * Output Parameters:
 *   reltime - Pointer to receive the time value presented as struct timespec
 *
 * Returned Value:
 *   Always returns OK (0)
 *
 ****************************************************************************/

int clock_ticks2time(sclock_t ticks, FAR struct timespec *reltime);

/****************************************************************************
 * Name: clock_systime_timespec
 *
 * Description:
 *   Return the current value of the system timer counter as a struct
 *   timespec.
 *
 * Input Parameters:
 *   ts - Location to return the time
 *
 * Returned Value:
 *   OK (0) on success; a negated errno value on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

int clock_systime_timespec(FAR struct timespec *ts);

/****************************************************************************
 * Name:  clock_cpuload
 *
 * Description:
 *   Return load measurement data for the select PID.
 *
 * Input Parameters:
 *   pid - The task ID of the thread of interest. pid == 0 is IDLE thread.
 *   cpuload - The location to return the CPU load
 *
 * Returned Value:
 *   OK (0) on success; a negated errno value on failure.  The only reason
 *   that this function can fail is if 'pid' no longer refers to a valid
 *   thread.
 *
 * Assumptions:
 *
 ****************************************************************************/

#ifndef CONFIG_SCHED_CPULOAD_NONE
int clock_cpuload(int pid, FAR struct cpuload_s *cpuload);
#endif

/****************************************************************************
 * Name:  nxsched_oneshot_extclk
 *
 * Description:
 *   Configure to use a oneshot timer as described in
 *   include/nuttx/timers/oneshot.h to provide external clocking to assess
 *   the CPU load.
 *
 * Input Parameters:
 *   lower - An instance of the oneshot timer interface as defined in
 *           include/nuttx/timers/oneshot.h
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_CPULOAD_ONESHOT
struct oneshot_lowerhalf_s;
void nxsched_oneshot_extclk(FAR struct oneshot_lowerhalf_s *lower);
#endif

/****************************************************************************
 * Name:  nxsched_period_extclk
 *
 * Description:
 *   Configure to use a period timer as described in
 *   include/nuttx/timers/timer.h to provide external clocking to assess
 *   the CPU load.
 *
 * Input Parameters:
 *   lower - An instance of the period timer interface as defined in
 *           include/nuttx/timers/timer.h
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_CPULOAD_PERIOD
struct timer_lowerhalf_s;
void nxsched_period_extclk(FAR struct timer_lowerhalf_s *lower);
#endif

/****************************************************************************
 * perf_gettime
 ****************************************************************************/

clock_t perf_gettime(void);

/****************************************************************************
 * perf_convert
 ****************************************************************************/

void perf_convert(clock_t elapsed, FAR struct timespec *ts);

/****************************************************************************
 * perf_gettfreq
 ****************************************************************************/

unsigned long perf_getfreq(void);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_CLOCK_H */
