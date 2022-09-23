/****************************************************************************
 * include/nuttx/timers/oneshot.h
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

#include <errno.h>
#include <signal.h>
#include <stdint.h>
#include <time.h>

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

#define ONESHOT_MAX_DELAY(l,t) \
  ((l)->ops->max_delay ? (l)->ops->max_delay(l,t) : oneshot_max_delay(l,t))
#define ONESHOT_TICK_MAX_DELAY(l,t) \
  ((l)->ops->tick_max_delay ? (l)->ops->tick_max_delay(l,t) : oneshot_tick_max_delay(l,t))

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

#define ONESHOT_START(l,h,a,t) \
  ((l)->ops->start ? (l)->ops->start(l,h,a,t) : oneshot_start(l,h,a,t))
#define ONESHOT_TICK_START(l,h,a,t) \
  ((l)->ops->tick_start ? (l)->ops->tick_start(l,h,a,t) : oneshot_tick_start(l,h,a,t))

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

#define ONESHOT_CANCEL(l,t) \
  ((l)->ops->cancel ? (l)->ops->cancel(l,t) : oneshot_cancel(l,t))
#define ONESHOT_TICK_CANCEL(l,t) \
  ((l)->ops->tick_cancel ? (l)->ops->tick_cancel(l,t) : oneshot_tick_cancel(l,t))

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

#define ONESHOT_CURRENT(l,t) \
  ((l)->ops->current ? (l)->ops->current(l,t) : oneshot_current(l,t))
#define ONESHOT_TICK_CURRENT(l,t) \
  ((l)->ops->tick_current ? (l)->ops->tick_current(l,t) : oneshot_tick_current(l,t))

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

/* The one short operations supported by the lower half driver */

struct timespec;
struct oneshot_operations_s
{
  CODE int (*max_delay)(FAR struct oneshot_lowerhalf_s *lower,
                        FAR struct timespec *ts);
  CODE int (*start)(FAR struct oneshot_lowerhalf_s *lower,
                    oneshot_callback_t callback, FAR void *arg,
                    FAR const struct timespec *ts);
  CODE int (*cancel)(FAR struct oneshot_lowerhalf_s *lower,
                     FAR struct timespec *ts);
  CODE int (*current)(FAR struct oneshot_lowerhalf_s *lower,
                      FAR struct timespec *ts);
  CODE int (*tick_max_delay)(FAR struct oneshot_lowerhalf_s *lower,
                             FAR clock_t *ticks);
  CODE int (*tick_start)(FAR struct oneshot_lowerhalf_s *lower,
                         oneshot_callback_t callback, FAR void *arg,
                         clock_t ticks);
  CODE int (*tick_cancel)(FAR struct oneshot_lowerhalf_s *lower,
                          FAR clock_t *ticks);
  CODE int (*tick_current)(FAR struct oneshot_lowerhalf_s *lower,
                           FAR clock_t *ticks);
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
 * Public Function Prototypes
 ****************************************************************************/

static inline
int oneshot_max_delay(FAR struct oneshot_lowerhalf_s *lower,
                      FAR struct timespec *ts)
{
  clock_t tick;
  int ret;

  DEBUGASSERT(lower->ops->tick_max_delay);

  ret = lower->ops->tick_max_delay(lower, &tick);
  timespec_from_tick(ts, tick);
  return ret;
}

static inline
int oneshot_start(FAR struct oneshot_lowerhalf_s *lower,
                  oneshot_callback_t callback, FAR void *arg,
                  FAR const struct timespec *ts)
{
  clock_t tick;

  DEBUGASSERT(lower->ops->tick_start);

  tick = timespec_to_tick(ts);
  return lower->ops->tick_start(lower, callback, arg, tick);
}

static inline
int oneshot_cancel(FAR struct oneshot_lowerhalf_s *lower,
                   FAR struct timespec *ts)
{
  clock_t tick;
  int ret;

  DEBUGASSERT(lower->ops->tick_cancel);

  ret = lower->ops->tick_cancel(lower, &tick);
  timespec_from_tick(ts, tick);

  return ret;
}

static inline
int oneshot_current(FAR struct oneshot_lowerhalf_s *lower,
                    FAR struct timespec *ts)
{
  clock_t tick;
  int ret;

  DEBUGASSERT(lower->ops->tick_current);

  ret = lower->ops->tick_current(lower, &tick);
  timespec_from_tick(ts, tick);

  return ret;
}

static inline
int oneshot_tick_max_delay(FAR struct oneshot_lowerhalf_s *lower,
                           FAR clock_t *ticks)
{
  struct timespec ts;
  int ret;

  DEBUGASSERT(lower->ops->max_delay);

  ret = lower->ops->max_delay(lower, &ts);
  *ticks = timespec_to_tick(&ts);
  return ret;
}

static inline
int oneshot_tick_start(FAR struct oneshot_lowerhalf_s *lower,
                       oneshot_callback_t callback, FAR void *arg,
                       clock_t ticks)
{
  struct timespec ts;

  DEBUGASSERT(lower->ops->start);

  timespec_from_tick(&ts, ticks);
  return lower->ops->start(lower, callback, arg, &ts);
}

static inline
int oneshot_tick_cancel(FAR struct oneshot_lowerhalf_s *lower,
                        FAR clock_t *ticks)
{
  struct timespec ts;
  int ret;

  DEBUGASSERT(lower->ops->cancel);

  ret = lower->ops->cancel(lower, &ts);
  *ticks = timespec_to_tick(&ts);

  return ret;
}

static inline
int oneshot_tick_current(FAR struct oneshot_lowerhalf_s *lower,
                         FAR clock_t *ticks)
{
  struct timespec ts;
  int ret;

  DEBUGASSERT(lower->ops->current);

  ret = lower->ops->current(lower, &ts);
  *ticks = timespec_to_tick(&ts);

  return ret;
}

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
