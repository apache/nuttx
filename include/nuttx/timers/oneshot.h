/****************************************************************************
 * include/nuttx/timers/oneshot.h
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
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

#ifndef __INCLUDE_NUTTX_TIMERS_ONESHOT_H
#define __INCLUDE_NUTTX_TIMERS_ONESHOT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
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
 *                    Argument: A referenct to a struct timespec in which
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
 * NOTE: _TCIOC(0x0020) througn _TCIOC(0x003f) are reserved for use by the
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
 *   ts      The location in which to return the maxumum delay.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

#define ONESHOT_MAX_DELAY(l,t) ((l)->ops->max_delay(l,t))

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

#define ONESHOT_START(l,h,a,t) ((l)->ops->start(l,h,a,t))

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

#define ONESHOT_CANCEL(l,t) ((l)->ops->cancel(l,t))

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

#define ONESHOT_CURRENT(l,t) ((l)->ops->current ? (l)->ops->current(l,t) : -ENOSYS)

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
typedef void (*oneshot_callback_t)(FAR struct oneshot_lowerhalf_s *lower,
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
  CODE int (*cancel)(struct oneshot_lowerhalf_s *lower,
                     FAR struct timespec *ts);
  CODE int (*current)(struct oneshot_lowerhalf_s *lower,
                      FAR struct timespec *ts);
};

/* This structure describes the state of the oneshot timer lower-half driver */

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
  pid_t pid;          /* PID of task to be signalled (0 means calling task) */
  int signo;          /* Signal number to use */
  FAR void *arg;      /* Signal value argument */
  struct timespec ts; /* Delay until time expiration */
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
