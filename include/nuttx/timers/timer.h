/****************************************************************************
 * include/nuttx/timers/timer.h
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

#ifndef __INCLUDE_NUTTX_TIMERS_TIMER_H
#define __INCLUDE_NUTTX_TIMERS_TIMER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <nuttx/irq.h>
#include <nuttx/fs/ioctl.h>
#include <signal.h>
#include <stdbool.h>
#include <sys/types.h>

#ifdef CONFIG_TIMER

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* IOCTL Commands ***********************************************************/

/* The timer driver uses a standard character driver framework.  However,
 * since the timer driver is a device control interface and not a data
 * transfer interface, the majority of the functionality is implemented in
 * driver ioctl calls.  The timer ioctl commands are listed below:
 *
 * These are detected and handled by the "upper half" timer driver.
 *
 * TCIOC_START        - Start the timer
 *                      Argument: Ignored
 * TCIOC_STOP         - Stop the timer
 *                      Argument: Ignored
 * TCIOC_GETSTATUS    - Get the status of the timer.
 *                      Argument:  A writeable pointer to struct
 *                      timer_status_s.
 * TCIOC_SETTIMEOUT   - Reset the timer timeout to this value
 *                      Argument: A 32-bit timeout value in microseconds.
 * TCIOC_NOTIFICATION - Set up to notify an application via a signal when
 *                      the timer expires.
 *                      Argument: A read-only pointer to an instance of
 *                      struct timer_notify_s.
 * TCIOC_MAXTIMEOUT   - Get the maximum supported timeout value
 *                      Argument: A 32-bit timeout value in microseconds.
 *
 * WARNING: May change TCIOC_SETTIMEOUT to pass pointer to 64bit nanoseconds
 * or timespec structure.
 *
 * NOTE: _TCIOC(0x0001) through _TCIOC(0x001f) are reserved for use by the
 * timer driver to assure that the values are unique.  Other timer drivers,
 * such as the oneshot timer, must not use IOCTL commands in this numeric
 * range.
 */

#define TCIOC_START        _TCIOC(0x0001)
#define TCIOC_STOP         _TCIOC(0x0002)
#define TCIOC_GETSTATUS    _TCIOC(0x0003)
#define TCIOC_SETTIMEOUT   _TCIOC(0x0004)
#define TCIOC_NOTIFICATION _TCIOC(0x0005)
#define TCIOC_MAXTIMEOUT   _TCIOC(0x0006)

/* Bit Settings *************************************************************/

/* Bit settings for the struct timer_status_s flags field */

#define TCFLAGS_ACTIVE     (1 << 0) /* 1=The timer is running */
#define TCFLAGS_HANDLER    (1 << 1) /* 1=Call the user function when the
                                     *   timer expires */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Upper half callback prototype. Returns true to reload the timer, and the
 * function can modify the next interval if desired.
 */

typedef CODE bool (*tccb_t)(FAR uint32_t *next_interval_us, FAR void *arg);

/* This is the type of the argument passed to the TCIOC_GETSTATUS ioctl and
 * and returned by the "lower half" getstatus() method.
 */

struct timer_status_s
{
  uint32_t  flags;          /* See TCFLAGS_* definitions above */
  uint32_t  timeout;        /* The current timeout setting (in microseconds) */
  uint32_t  timeleft;       /* Time left until the timer expiration
                             * (in microseconds) */
};

/* This is the type of the argument passed to the TCIOC_NOTIFICATION ioctl */

struct timer_notify_s
{
  struct sigevent event;    /* Describe the way a task is to be notified */
  pid_t           pid;      /* The ID of the task/thread to receive the signal */
  bool            oneshot;  /* Single notification or periodic notifications */
};

/* This structure provides the "lower-half" driver operations available to
 * the "upper-half" driver.
 */

struct timer_lowerhalf_s;
struct timer_ops_s
{
  /* Required methods *******************************************************/

  /* Start the timer, resetting the time to the current timeout */

  CODE int (*start)(FAR struct timer_lowerhalf_s *lower);

  /* Stop the timer */

  CODE int (*stop)(FAR struct timer_lowerhalf_s *lower);

  /* Get the current timer status */

  CODE int (*getstatus)(FAR struct timer_lowerhalf_s *lower,
                        FAR struct timer_status_s *status);

  /* Set a new timeout value (and reset the timer) */

  CODE int (*settimeout)(FAR struct timer_lowerhalf_s *lower,
                         uint32_t timeout);

  /* Call the NuttX INTERNAL timeout callback on timeout.
   * NOTE:  Providing callback==NULL disable.
   * NOT to call back into applications.
   */

  CODE void (*setcallback)(FAR struct timer_lowerhalf_s *lower,
                           CODE tccb_t callback, FAR void *arg);

  /* Any ioctl commands that are not recognized by the "upper-half" driver
   * are forwarded to the lower half driver through this method.
   */

  CODE int (*ioctl)(FAR struct timer_lowerhalf_s *lower, int cmd,
                    unsigned long arg);

  /* Get the maximum supported timeout value */

  CODE int (*maxtimeout)(FAR struct timer_lowerhalf_s *lower,
                         FAR uint32_t *maxtimeout);
};

/* This structure provides the publicly visible representation of the
 * "lower-half" driver state structure.  "lower half" drivers will have an
 * internal structure definition that will be cast-compatible with this
 * structure definitions.
 */

struct timer_lowerhalf_s
{
  /* Publicly visible portion of the "lower-half" driver state structure. */

  FAR const struct timer_ops_s  *ops;  /* Lower half operations */

  /* The remainder of the structure is used by the "lower-half" driver
   * for whatever state storage that it may need.
   */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * "Upper-Half" Timer Driver Interfaces
 ****************************************************************************/

/****************************************************************************
 * Name: timer_register
 *
 * Description:
 *   This function binds an instance of a "lower half" timer driver with the
 *   "upper half" timer device and registers that device so that can be used
 *   by application code.
 *
 *   When this function is called, the "lower half" driver should be in the
 *   disabled state (as if the stop() method had already been called).
 *
 *   NOTE:  Normally, this function would not be called by application code.
 *   Rather it is called indirectly through the architecture-specific
 *   initialization.
 *
 * Input Parameters:
 *   dev path - The full path to the driver to be registered in the NuttX
 *     pseudo-filesystem.  The recommended convention is to name all timer
 *     drivers as "/dev/timer0", "/dev/timer1", etc.  where the driver
 *     path differs only in the "minor" number at the end of the device name.
 *   lower - A pointer to an instance of lower half timer driver.  This
 *     instance is bound to the timer driver and must persists as long as
 *     the driver persists.
 *
 * Returned Value:
 *   On success, a non-NULL handle is returned to the caller.  In the event
 *   of any failure, a NULL value is returned.
 *
 ****************************************************************************/

FAR void *timer_register(FAR const char *path,
                         FAR struct timer_lowerhalf_s *lower);

/****************************************************************************
 * Name: timer_unregister
 *
 * Description:
 *   This function can be called to disable and unregister the timer
 *   device driver.
 *
 * Input Parameters:
 *   handle - This is the handle that was returned by timer_register()
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void timer_unregister(FAR void *handle);

/****************************************************************************
 * Kernel internal interfaces.  Thse may not be used by application logic
 ****************************************************************************/

/****************************************************************************
 * Name: timer_setcallback
 *
 * Description:
 *   This function can be called to add a callback into driver-related code
 *   to handle timer expirations.  This is a strictly OS internal interface
 *   and may NOT be used by application code.
 *
 * Input Parameters:
 *   handle   - This is the handle that was returned by timer_register()
 *   callback - The new timer interrupt callback
 *   arg      - Argument provided when the callback is called.
 *
 * Returned Value:
 *   Zero (OK), if the callback was successfully set, or -ENOSYS if the lower
 *   half driver does not support the operation.
 *
 ****************************************************************************/

int timer_setcallback(FAR void *handle, tccb_t callback, FAR void *arg);

/****************************************************************************
 * Platform-Independent "Lower-Half" Timer Driver Interfaces
 ****************************************************************************/

/****************************************************************************
 * Architecture-specific Application Interfaces
 ****************************************************************************/

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_TIMER */
#endif /* __INCLUDE_NUTTX_TIMERS_TIMER_H */
