/****************************************************************************
 * include/nuttx/timers/oneshot.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifndef __INCLUDE_NUTTX_TIMERS_ONESHOT_H
#define __INCLUDE_NUTTX_TIMERS_ONESHOT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <assert.h>
#include <errno.h>
#include <signal.h>
#include <stdint.h>
#include <time.h>

#include <nuttx/irq.h>
#include <nuttx/clock.h>
#include <nuttx/timers/clkcnt.h>
#include <nuttx/fs/ioctl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* IOCTL commands ***********************************************************/

/* These commands are used by applications to access the oneshot lower-half
 * logic via the oneshot character driver IOCTL command.  Since the oneshot
 * driver is a device control interface and not a data transfer interface,
 * the majority of the functionality is implemented in driver IOCTL calls.
 * The oneshot IOCTL commands are listed below:
 *
 * These are detected and handled by the "upper half" timer driver.
 *
 * OSIOC_MAXDELAY   - Return the maximum delay that can be supported by
 *                    this timer.
 *                    Argument: A reference to a struct timespec in which
 *                    the maximum time will be returned.
 * OSIOC_START      - Start the oneshot timer
 *                    Argument: A reference to struct oneshot_start_s
 * OSIOC_CANCEL     - Stop the timer
 *                    Argument: A reference to a struct timespec in which
 *                    the time remaining will be returned.
 * OSIOC_CURRENT    - Get the current time
 *                    Argument: A reference to a struct timespec in which
 *                    the current time will be returned.
 *
 * NOTE: _TCIOC(0x0020) through _TCIOC(0x003f) are reserved for use by the
 * oneshot driver to assure that the values are unique.  Other timer drivers
 * must not use IOCTL commands in this numeric range.
 */

#define OSIOC_MAXDELAY   _TCIOC(0x0020)
#define OSIOC_START      _TCIOC(0x0021)
#define OSIOC_CANCEL     _TCIOC(0x0022)
#define OSIOC_CURRENT    _TCIOC(0x0023)

/* Method access helper macros **********************************************/

/****************************************************************************
 * Name: ONESHOT_MAX_DELAY
 *
 * Description:
 *   Determine the maximum delay of the one-shot timer (in microseconds)
 *
 * Input Parameters:
 *   lower   An instance of the lower-half oneshot state structure.  This
 *           structure must have been previously initialized via a call to
 *           oneshot_initialize();
 *   ts      The location in which to return the maximum delay.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

#define ONESHOT_MAX_DELAY(l,t) oneshot_max_delay(l,t)
#define ONESHOT_TICK_MAX_DELAY(l,t)  oneshot_tick_max_delay(l,t)

/****************************************************************************
 * Name: ONESHOT_START
 *
 * Description:
 *   Start the oneshot timer
 *
 * Input Parameters:
 *   lower   An instance of the lower-half oneshot state structure.  This
 *           structure must have been previously initialized via a call to
 *           oneshot_initialize();
 *   handler The function to call when when the oneshot timer expires.
 *   arg     An opaque argument that will accompany the callback.
 *   ts      Provides the duration of the one shot timer.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

#define ONESHOT_START(l,t) oneshot_start(l,t)
#define ONESHOT_TICK_START(l,t) oneshot_tick_start(l,t)

#define ONESHOT_ABSOLUTE(l,t) oneshot_start_absolute(l,t)
#define ONESHOT_TICK_ABSOLUTE(l,t) oneshot_tick_absolute(l,t)

/****************************************************************************
 * Name: ONESHOT_CANCEL
 *
 * Description:
 *   Cancel the oneshot timer and return the time remaining on the timer.
 *
 *   NOTE: This function may execute at a high rate with no timer running (as
 *   when pre-emption is enabled and disabled).
 *
 * Input Parameters:
 *   lower   Caller allocated instance of the oneshot state structure.  This
 *           structure must have been previously initialized via a call to
 *           oneshot_initialize();
 *   ts      The location in which to return the time remaining on the
 *           oneshot timer.  A time of zero is returned if the timer is
 *           not running.
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  A call to up_timer_cancel() when
 *   the timer is not active should also return success; a negated errno
 *   value is returned on any failure.
 *
 ****************************************************************************/

#define ONESHOT_CANCEL(l,t) oneshot_cancel(l,t)
#define ONESHOT_TICK_CANCEL(l,t) oneshot_tick_cancel(l,t)

/****************************************************************************
 * Name: ONESHOT_CURRENT
 *
 * Description:
 *  Get the current time.
 *
 * Input Parameters:
 *   lower   Caller allocated instance of the oneshot state structure.  This
 *           structure must have been previously initialized via a call to
 *           oneshot_initialize();
 *   ts      The location in which to return the current time. A time of zero
 *           is returned for the initialization moment.
 *
 * Returned Value:
 *   Zero (OK) is returned on success, a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

#define ONESHOT_CURRENT(l,t) oneshot_current(l,t)
#define ONESHOT_TICK_CURRENT(l,t) oneshot_tick_current(l,t)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This describes the callback function that will be invoked when the oneshot
 * timer expires.  The oneshot fires, the client will receive:
 *
 *   lower - An instance of the lower half driver
 *   arg   - The opaque argument provided when the interrupt was registered
 */

struct oneshot_lowerhalf_s;
typedef CODE void (*oneshot_callback_t)
                       (FAR struct oneshot_lowerhalf_s *lower,
                        FAR void *arg);

/* The oneshot operations supported by the lower half driver */

struct timespec;
struct oneshot_operations_s
{
#ifdef CONFIG_ONESHOT_COUNT
  /* New clkcnt interfaces with better performance, overflow-free timing
   * conversion, and the theoretical optimal timing accuracy.
   */

  CODE clkcnt_t (*current)(FAR struct oneshot_lowerhalf_s *lower);
  CODE void     (*start)(FAR struct oneshot_lowerhalf_s *lower,
                         clkcnt_t delay);
  CODE void     (*start_absolute)(FAR struct oneshot_lowerhalf_s *lower,
                                  clkcnt_t cnt);
  CODE void     (*cancel)(FAR struct oneshot_lowerhalf_s *lower);
  CODE clkcnt_t (*max_delay)(FAR struct oneshot_lowerhalf_s *lower);
#else
  /* Deprecated interfaces, just for compatiable-usage. */

  CODE int (*max_delay)(FAR struct oneshot_lowerhalf_s *lower,
                        FAR struct timespec *ts);
  CODE int (*start)(FAR struct oneshot_lowerhalf_s *lower,
                    FAR const struct timespec *ts);
  CODE int (*cancel)(FAR struct oneshot_lowerhalf_s *lower,
                     FAR struct timespec *ts);
  CODE int (*current)(FAR struct oneshot_lowerhalf_s *lower,
                      FAR struct timespec *ts);
#endif
};

/* This structure describes the state of the oneshot timer lower-half
 * driver
 */

struct oneshot_lowerhalf_s
{
  /* This is the part of the lower half driver that is visible to the upper-
   * half client of the driver.
   */

  FAR const struct oneshot_operations_s *ops;

  FAR oneshot_callback_t callback;
  FAR void *arg;

#ifdef CONFIG_ONESHOT_COUNT
  uint32_t frequency;

  uint32_t cnt2nsec_mult;
  uint32_t cnt2nsec_shift;
#endif

#ifdef CONFIG_ONESHOT_FAST_DIVISION
  invdiv_param64_t invdiv_freq;
#endif

  /* Private lower half data may follow */
};

#ifdef CONFIG_ONESHOT
/* Argument to OSIOC_START IOCTL command */

struct oneshot_start_s
{
  pid_t pid;             /* PID of task to be signalled (0 means calling task) */
  struct sigevent event; /* Describe the way a task is to be notified */
  struct timespec ts;    /* Delay until time expiration */
};
#endif

/****************************************************************************
 * Public Data
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
 * Inline Functions
 ****************************************************************************/

#ifdef CONFIG_ONESHOT_COUNT
static inline_function
void oneshot_count_init(FAR struct oneshot_lowerhalf_s *lower,
                        uint32_t frequency)
{
  clkcnt_t result;
  DEBUGASSERT(lower && frequency);

  lower->frequency = frequency;

  clkcnt_best_multshift(frequency, NSEC_PER_SEC,
                        &lower->cnt2nsec_mult,
                        &lower->cnt2nsec_shift);

  /* Ensure the maximum error of the mult-shift is less than 5ns. */

  result = clkcnt_delta_cnt2nsec_fast(frequency, lower->cnt2nsec_mult,
                                      lower->cnt2nsec_shift);

  ASSERT(NSEC_PER_SEC - 5 <= result && NSEC_PER_SEC + 5 >= result);

#  ifdef CONFIG_ONESHOT_FAST_DIVISION
  /* invdiv requires the invariant-divsor > 1. */

  ASSERT(frequency > 1);

  invdiv_init_param64(frequency, &lower->invdiv_freq);
#  endif
}

static inline_function
uint32_t oneshot_delta_cnt2nsec(FAR struct oneshot_lowerhalf_s *lower,
                                clkcnt_t delta)
{
  DEBUGASSERT(delta <= lower->frequency);

  /* Here we use a multiply-shift method to convert the clock
   * count to nanoseconds. This will reduce at least one division
   * operation and improve the performance. Note that this is an
   * approximate method that trades accuracy for performance, it may lead
   * to 1-3 nanoseconds of error when converting the cycles that
   * represent less than 1 second. If extremely high resolution time is
   * required, then this option should be disabled.
   */

  return clkcnt_delta_cnt2nsec_fast(delta, lower->cnt2nsec_mult,
                                    lower->cnt2nsec_shift);
}

static inline_function
clock_t oneshot_delta_cnt2tick(FAR struct oneshot_lowerhalf_s *lower,
                               clkcnt_t delta)
{
  uint32_t nsec;

  DEBUGASSERT(delta <= lower->frequency);

  /* Be careful of using mult-shift fast converting here.
   * Since ticks are related to the scheduling, inaccurate converting
   * results may lead to wrong scheduling.
   */

  nsec = clkcnt_delta_cnt2nsec_fast(delta, lower->cnt2nsec_mult,
                                    lower->cnt2nsec_shift);

  return div_const(nsec, NSEC_PER_TICK);
}

static inline_function
uint64_t oneshot_cnt2sec(FAR struct oneshot_lowerhalf_s *lower,
                         clkcnt_t cnt)
{
#  ifdef CONFIG_ONESHOT_FAST_DIVISION
  return clkcnt_delta_cnt2time_invdiv(cnt, 1, &lower->invdiv_freq);
#  else
  return clkcnt_cnt2sec(cnt, lower->frequency);
#  endif
}
#endif

static inline_function
int oneshot_max_delay(FAR struct oneshot_lowerhalf_s *lower,
                      FAR struct timespec *ts)
{
  int ret = OK;
#ifdef CONFIG_ONESHOT_COUNT
  clkcnt_t max = lower->ops->max_delay(lower);
  clkcnt_max_timespec(max, lower->frequency, ts);
#else
  ret = lower->ops->max_delay(lower, ts);
#endif
  return ret;
}

/****************************************************************************
 * Name: oneshot_current
 *
 * Description:
 *   Get the current time.
 *
 * Input Parameters:
 *   ops - The oneshot interface.
 *   lower - The oneshot lowerhalf data.
 *   ts - The pointer to the current time.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static inline_function
int oneshot_current(FAR struct oneshot_lowerhalf_s *lower,
                    FAR struct timespec *ts)
{
  int ret = OK;
#ifdef CONFIG_ONESHOT_COUNT
  clkcnt_t cnt  = lower->ops->current(lower);
  uint32_t freq = lower->frequency;
  uint64_t sec  = oneshot_cnt2sec(lower, cnt);

  cnt          -= sec * freq;
  ts->tv_nsec   = oneshot_delta_cnt2nsec(lower, cnt);
  ts->tv_sec    = sec;
#else
  ret = lower->ops->current(lower, ts);
#endif
  return ret;
}

/****************************************************************************
 * Name: oneshot_cancel
 *
 * Description:
 *   Cancel the timer
 *
 * Input Parameters:
 *   ops - The oneshot interface.
 *   ts - The delta time in timespec.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static inline_function
int oneshot_cancel(FAR struct oneshot_lowerhalf_s *lower,
                   FAR struct timespec *ts)
{
  int ret = OK;
#ifdef CONFIG_ONESHOT_COUNT
  lower->ops->cancel(lower);
  oneshot_current(lower, ts);
#else
  ret = lower->ops->cancel(lower, ts);
#endif
  return ret;
}

/****************************************************************************
 * Name: oneshot_start
 *
 * Description:
 *   Set the relative time in timespec to trigger the clockevent.
 *
 * Input Parameters:
 *   ops - The oneshot interface.
 *   ts - The delta time in timespec.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static inline_function
int oneshot_start(FAR struct oneshot_lowerhalf_s *lower,
                  FAR const struct timespec *ts)
{
  int ret = OK;
#ifdef CONFIG_ONESHOT_COUNT
  clkcnt_t freq = lower->frequency;
  clkcnt_t cnt  = clkcnt_delta_time2cnt(ts->tv_nsec, freq, NSEC_PER_SEC) +
                  ts->tv_sec * freq;

  lower->ops->start(lower, cnt);
#else
  ret = lower->ops->start(lower, ts);
#endif
  return ret;
}

/****************************************************************************
 * Name: oneshot_start_absolute
 *
 * Description:
 *   Set the absolute time to trigger the clockevent.
 *
 * Input Parameters:
 *   ops - The oneshot interface.
 *   expected - The expected time count.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static inline_function
int oneshot_start_absolute(FAR struct oneshot_lowerhalf_s *lower,
                           FAR const struct timespec *ts)
{
  int ret = OK;
#ifdef CONFIG_ONESHOT_COUNT
  uint32_t freq     = lower->frequency;
  clkcnt_t expected = ts->tv_sec * freq +
                      clkcnt_delta_time2cnt(ts->tv_nsec, freq, NSEC_PER_SEC);

  if (lower->ops->start_absolute)
    {
      lower->ops->start_absolute(lower, expected);
    }
  else
    {
      /* IRQ should be disable or the timer will be fired too late. */

      irqstate_t flags = up_irq_save();
      clkcnt_t   delay = expected - lower->ops->current(lower);
      lower->ops->start(lower, delay);
      up_irq_restore(flags);
    }
#else
  struct timespec curr =
  {
    0
  };

  /* Some timer drivers may not have current() function.
   * Since only arch_alarm uses the function, it should be OK.
   */

  DEBUGASSERT(lower->ops->current);

  ret = lower->ops->current(lower, &curr);
  clock_timespec_subtract(ts, &curr, &curr);
  ret = lower->ops->start(lower, &curr);
#endif
  return ret;
}

/* Tick-based compatible layer for oneshot */

static inline_function
int oneshot_tick_max_delay(FAR struct oneshot_lowerhalf_s *lower,
                           FAR clock_t *tick)
{
  int ret = OK;
#ifdef CONFIG_ONESHOT_COUNT
  clkcnt_t max = lower->ops->max_delay(lower);
  *tick = clkcnt_max_tick(max, lower->frequency);
#else
  struct timespec ts =
  {
    0
  };

  ret = lower->ops->max_delay(lower, &ts);
  *tick = clock_time2ticks(&ts);
#endif
  return ret;
}

/****************************************************************************
 * Name: oneshot_tick_start
 *
 * Description:
 *   Set the relative time in ticks to trigger the clockevent.
 *
 * Input Parameters:
 *   ops - The oneshot interface.
 *   tick   - The delta time in ticks.

 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static inline_function
int oneshot_tick_start(FAR struct oneshot_lowerhalf_s *lower,
                       clock_t tick)
{
  int ret = OK;
#ifdef CONFIG_ONESHOT_COUNT
  clkcnt_t cnt = clkcnt_tick2cnt(tick, lower->frequency);
  lower->ops->start(lower, cnt);
#else
  struct timespec ts;
  clock_ticks2time(&ts, tick);
  ret = lower->ops->start(lower, &ts);
#endif
  return ret;
}

/****************************************************************************
 * Name: oneshot_tick_current
 *
 * Description:
 *   Get the current system tick.
 *
 * Input Parameters:
 *   ops - The oneshot interface.
 *   lower - The oneshot lowerhalf data.
 *
 * Returned Value:
 *   The current system tick.
 *
 ****************************************************************************/

static inline_function
int oneshot_tick_current(FAR struct oneshot_lowerhalf_s *lower,
                         FAR clock_t *tick)
{
  int ret = OK;
#ifdef CONFIG_ONESHOT_COUNT
  clkcnt_t cnt  = lower->ops->current(lower);
  uint32_t freq = lower->frequency;
  uint64_t sec  = oneshot_cnt2sec(lower, cnt);

  cnt   -= sec * freq;
  *tick  = sec * TICK_PER_SEC + oneshot_delta_cnt2tick(lower, cnt);
#else
  struct timespec ts =
  {
    0
  };

  /* Some timer drivers may not have current() function.
   * Since only arch_alarm uses the function, it should be OK.
   */

  DEBUGASSERT(lower->ops->current);

  ret = lower->ops->current(lower, &ts);
  *tick = clock_time2ticks_floor(&ts);
#endif
  return ret;
}

static inline_function
int oneshot_tick_absolute(FAR struct oneshot_lowerhalf_s *lower,
                          clock_t tick)
{
  int ret = OK;
#ifdef CONFIG_ONESHOT_COUNT
  clkcnt_t expected = clkcnt_tick2cnt(tick, lower->frequency);
  if (lower->ops->start_absolute)
    {
      lower->ops->start_absolute(lower, expected);
    }
  else
    {
      /* IRQ should be disable or the timer will be fired too late. */

      irqstate_t flags = up_irq_save();
      clkcnt_t   delay = expected - lower->ops->current(lower);
      lower->ops->start(lower, delay);
      up_irq_restore(flags);
    }
#else
  struct timespec ts;
  clock_ticks2time(&ts, tick);
  ret = oneshot_start_absolute(lower, &ts);
#endif
  return ret;
}

/****************************************************************************
 * Name: oneshot_tick_cancel
 *
 * Description:
 *   Cancel the timer.
 *
 * Input Parameters:
 *   ops - The oneshot interface.
 *   lower - The oneshot lowerhalf data.
 *
 * Returned Value:
 *   The current system tick.
 *
 ****************************************************************************/

static inline_function
int oneshot_tick_cancel(FAR struct oneshot_lowerhalf_s *lower,
                        FAR clock_t *tick)
{
  int ret = OK;
#ifdef CONFIG_ONESHOT_COUNT
  lower->ops->cancel(lower);
  oneshot_tick_current(lower, tick);
#else
  struct timespec ts =
  {
    0
  };

  ret = lower->ops->cancel(lower, &ts);

  /* Converting timespec to ticks may overflow. */

  *tick = clock_time2ticks_floor(&ts);
#endif
  return ret;
}

static inline_function
void oneshot_process_callback(FAR struct oneshot_lowerhalf_s *lower)
{
  if (lower->callback)
    {
      lower->callback(lower, lower->arg);
    }
}

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: oneshot_initialize
 *
 * Description:
 *   Initialize the oneshot timer and return a oneshot lower half driver
 *   instance.
 *
 * Input Parameters:
 *   chan       Timer counter channel to be used.
 *   resolution The required resolution of the timer in units of
 *              microseconds.  NOTE that the range is restricted to the
 *              range of uint16_t (excluding zero).
 *
 * Returned Value:
 *   On success, a non-NULL instance of the oneshot lower-half driver is
 *   returned.  NULL is return on any failure.
 *
 ****************************************************************************/

FAR struct oneshot_lowerhalf_s *oneshot_initialize(int chan,
                                                   uint16_t resolution);

/****************************************************************************
 * Name: oneshot_register
 *
 * Description:
 *   Register the oneshot device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/oneshot0"
 *   lower - An instance of the lower half interface
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.  The following
 *   possible error values may be returned (most are returned by
 *   register_driver()):
 *
 *   EINVAL - 'path' is invalid for this operation
 *   EEXIST - An inode already exists at 'path'
 *   ENOMEM - Failed to allocate in-memory resources for the operation
 *
 ****************************************************************************/

#ifdef CONFIG_ONESHOT
int oneshot_register(FAR const char *devname,
                     FAR struct oneshot_lowerhalf_s *lower);
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_TIMERS_ONESHOT_H */
