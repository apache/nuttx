/****************************************************************************
 * include/nuttx/clock.h
 *
 *   Copyright (C) 2007-2009, 2011-2012, 2014, 2016-2018 Gregory Nutt.
             All rights reserved.
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

#ifndef __INCLUDE_NUTTX_CLOCK_H
#define __INCLUDE_NUTTX_CLOCK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
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

#elif defined(CONFIG_BUILD_PROTECTED) && defined(__KERNEL__)
   /* Case 3: Kernel mode of protected kernel build */

#    define __HAVE_KERNEL_GLOBALS 1

#elif defined(CONFIG_BUILD_KERNEL) && defined(__KERNEL__)
   /* Case 3: Kernel only build */

#    define __HAVE_KERNEL_GLOBALS 1

#elif defined(CONFIG_LIB_SYSCALL)
   /* Case 4: Building with SYSCALLs enabled, but not part of a kernel build */

#else
   /* Case 2: Un-protected, non-kernel build */

#    define __HAVE_KERNEL_GLOBALS 1
#endif

/* If CONFIG_SYSTEM_TIME64 is selected and the CPU supports long long types,
 * then a 64-bit system time will be used.
 */

#ifndef CONFIG_HAVE_LONG_LONG
#  undef CONFIG_SYSTEM_TIME64
#endif

/* Timing constants *********************************************************/

#define NSEC_PER_SEC          1000000000L /* Seconds */
#define USEC_PER_SEC             1000000L
#define MSEC_PER_SEC                1000L
#define DSEC_PER_SEC                  10L
#define HSEC_PER_SEC                   2L

#define NSEC_PER_HSEC          500000000L /* Half seconds */
#define USEC_PER_HSEC             500000L
#define MSEC_PER_HSEC                500L
#define DSEC_PER_HSEC                  5L

#define NSEC_PER_DSEC          100000000L /* Deciseconds */
#define USEC_PER_DSEC             100000L
#define MSEC_PER_DSEC                100L

#define NSEC_PER_MSEC            1000000L /* Milliseconds */
#define USEC_PER_MSEC               1000L

#define NSEC_PER_USEC               1000L /* Microseconds */

#define SEC_PER_MIN                   60L
#define NSEC_PER_MIN           (NSEC_PER_SEC * SEC_PER_MIN)
#define USEC_PER_MIN           (USEC_PER_SEC * SEC_PER_MIN)
#define MSEC_PER_MIN           (MSEC_PER_SEC * SEC_PER_MIN)
#define DSEC_PER_MIN           (HSEC_PER_SEC * SEC_PER_MIN)
#define HSEC_PER_MIN           (HSEC_PER_SEC * SEC_PER_MIN)

#define MIN_PER_HOUR                  60L
#define NSEC_PER_HOUR          (NSEC_PER_MIN * MIN_PER_HOUR)
#define USEC_PER_HOUR          (USEC_PER_MIN * MIN_PER_HOUR)
#define MSEC_PER_HOUR          (MSEC_PER_MIN * MIN_PER_HOUR)
#define DSEC_PER_HOUR          (HSEC_PER_SEC * MIN_PER_HOUR)
#define HSEC_PER_HOUR          (DSEC_PER_MIN * MIN_PER_HOUR)
#define SEC_PER_HOUR           (SEC_PER_MIN  * MIN_PER_HOUR)

#define HOURS_PER_DAY                 24L
#define SEC_PER_DAY            (HOURS_PER_DAY * SEC_PER_HOUR)

/* If CONFIG_SCHED_TICKLESS is not defined, then the interrupt interval of
 * the system timer is given by USEC_PER_TICK.  This is the expected number
 * of microseconds between calls from the processor-specific logic to
 * nxsched_process_timer().  The default value of USEC_PER_TICK is 10000
 * microseconds (100KHz).  However, this default setting can be overridden
 * by defining the interval in microseconds as CONFIG_USEC_PER_TICK in the
 * NuttX configuration file.
 *
 * The following calculations are only accurate when (1) there is no
 * truncation involved and (2) the underlying system timer is an even
 * multiple of microseconds.  If (2) is not true, you will probably want
 * to redefine all of the following.
 */

#ifdef CONFIG_USEC_PER_TICK
# define USEC_PER_TICK        (CONFIG_USEC_PER_TICK)
#else
# define USEC_PER_TICK        (10000)
#endif

/* MSEC_PER_TICK can be very inaccurate if CONFIG_USEC_PER_TICK is not an
 * even multiple of milliseconds.  Calculations using USEC_PER_TICK are
 * preferred for that reason (at the risk of overflow)
 */

#define TICK_PER_HOUR         (USEC_PER_HOUR / USEC_PER_TICK)            /* Truncates! */
#define TICK_PER_MIN          (USEC_PER_MIN  / USEC_PER_TICK)            /* Truncates! */
#define TICK_PER_SEC          (USEC_PER_SEC  / USEC_PER_TICK)            /* Truncates! */
#define TICK_PER_MSEC         (USEC_PER_MSEC / USEC_PER_TICK)            /* Truncates! */
#define TICK_PER_DSEC         (USEC_PER_DSEC / USEC_PER_TICK)            /* Truncates! */
#define TICK_PER_HSEC         (USEC_PER_HSEC / USEC_PER_TICK)            /* Truncates! */

#define MSEC_PER_TICK         (USEC_PER_TICK / USEC_PER_MSEC)            /* Truncates! */
#define NSEC_PER_TICK         (USEC_PER_TICK * NSEC_PER_USEC)            /* Exact */

#define NSEC2TICK(nsec)       (((nsec)+(NSEC_PER_TICK/2))/NSEC_PER_TICK) /* Rounds */
#define USEC2TICK(usec)       (((usec)+(USEC_PER_TICK/2))/USEC_PER_TICK) /* Rounds */

#if (MSEC_PER_TICK * USEC_PER_MSEC) == USEC_PER_TICK
#  define MSEC2TICK(msec)     (((msec)+(MSEC_PER_TICK/2))/MSEC_PER_TICK) /* Rounds */
#else
#  define MSEC2TICK(msec)     USEC2TICK((msec) * USEC_PER_MSEC)          /* Rounds */
#endif

#define DSEC2TICK(dsec)       MSEC2TICK((dsec) * MSEC_PER_DSEC)          /* Rounds */
#define HSEC2TICK(dsec)       MSEC2TICK((dsec) * MSEC_PER_HSEC)          /* Rounds */
#define SEC2TICK(sec)         MSEC2TICK((sec)  * MSEC_PER_SEC)           /* Rounds */

#define TICK2NSEC(tick)       ((tick) * NSEC_PER_TICK)                   /* Exact */
#define TICK2USEC(tick)       ((tick) * USEC_PER_TICK)                   /* Exact */

#if (MSEC_PER_TICK * USEC_PER_MSEC) == USEC_PER_TICK
#  define TICK2MSEC(tick)     ((tick)*MSEC_PER_TICK)                     /* Exact */
#else
#  define TICK2MSEC(tick)     (((tick)*USEC_PER_TICK)/USEC_PER_MSEC)     /* Rounds */
#endif

#define TICK2DSEC(tick)       (((tick)+(TICK_PER_DSEC/2))/TICK_PER_DSEC) /* Rounds */
#define TICK2HSEC(tick)       (((tick)+(TICK_PER_HSEC/2))/TICK_PER_HSEC) /* Rounds */
#define TICK2SEC(tick)        (((tick)+(TICK_PER_SEC/2))/TICK_PER_SEC)   /* Rounds */

#if defined(CONFIG_DEBUG_FEATURES) && defined(CONFIG_SYSTEM_TIME64) && \
    defined(CONFIG_CLOCK_MONOTONIC) && !defined(CONFIG_SCHED_TICKLESS)
/* Initial system timer ticks value close to maximum 32-bit value, to test
 * 64-bit system-timer after going over 32-bit value. This is to make errors
 * of casting 64-bit system-timer to 32-bit variables more visible.
 */

#  define INITIAL_SYSTEM_TIMER_TICKS \
    ((uint64_t)(UINT32_MAX - (TICK_PER_SEC * 5)))
#else
#  define INITIAL_SYSTEM_TIMER_TICKS 0
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This structure is used to report CPU usage for a particular thread */

#ifdef CONFIG_SCHED_CPULOAD
struct cpuload_s
{
  volatile uint32_t total;   /* Total number of clock ticks */
  volatile uint32_t active;  /* Number of ticks while this thread was active */
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
EXTERN volatile clock_t g_system_timer;

#ifndef CONFIG_SYSTEM_TIME64
#  define clock_systimer() g_system_timer
#endif
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

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
 *   None
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

#ifdef CONFIG_RTC
void clock_synchronize(void);
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
 * Name: clock_systimer
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
clock_t clock_systimer(void);
#endif

/****************************************************************************
 * Name: clock_systimespec
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

int clock_systimespec(FAR struct timespec *ts);

/****************************************************************************
 * Name:  clock_cpuload
 *
 * Description:
 *   Return load measurement data for the select PID.
 *
 * Input Parameters:
 *   pid - The task ID of the thread of interest.  pid == 0 is the IDLE thread.
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

#ifdef CONFIG_SCHED_CPULOAD
int clock_cpuload(int pid, FAR struct cpuload_s *cpuload);
#endif

/****************************************************************************
 * Name:  sched_oneshot_extclk
 *
 * Description:
 *   Configure to use a oneshot timer as described in
 *   include/nuttx/timers/oneshot.h to provid external clocking to assess
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
void sched_oneshot_extclk(FAR struct oneshot_lowerhalf_s *lower);
#endif

/****************************************************************************
 * Name:  sched_period_extclk
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
void sched_period_extclk(FAR struct timer_lowerhalf_s *lower);
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_CLOCK_H */
