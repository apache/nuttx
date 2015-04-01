/****************************************************************************
 * include/nuttx/timers/timer.h
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            Bob Doiron
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

#ifndef __INCLUDE_NUTTX_TIMERS_TIMER_H
#define __INCLUDE_NUTTX_TIMERS_TIMER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <nuttx/irq.h>
#include <nuttx/fs/ioctl.h>

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
 * TCIOC_START      - Start the timer
 *                    Argument: Ignored
 * TCIOC_STOP       - Stop the timer
 *                    Argument: Ignored
 * TCIOC_GETSTATUS  - Get the status of the timer.
 *                    Argument:  A writeable pointer to struct timer_status_s.
 * TCIOC_SETTIMEOUT - Reset the timer timeout to this value
 *                    Argument: A 32-bit timeout value in microseconds.
 * TCIOC_SETHANDLER - Call this handler on timer expiration
 *                    Argument: A pointer to struct timer_sethandler_s.
 *
 * WARNING: May change TCIOC_SETTIMEOUT to pass pointer to 64bit nanoseconds
 * or timespec structure.
 *
 * NOTE: The TCIOC_SETHANDLER ioctl cannot be supported in the kernel build
 * mode. In that case direct callbacks from kernel space into user space is
 * forbidden.
 */

#define TCIOC_START      _TCIOC(0x001)
#define TCIOC_STOP       _TCIOC(0x002)
#define TCIOC_GETSTATUS  _TCIOC(0x003)
#define TCIOC_SETTIMEOUT _TCIOC(0x004)
#define TCIOC_SETHANDLER _TCIOC(0x005)

/* Bit Settings *************************************************************/
/* Bit settings for the struct timer_status_s flags field */

#define TCFLAGS_ACTIVE   (1 << 0) /* 1=The timer is running */
#define TCFLAGS_HANDLER  (1 << 1) /* 1=Call the user function when the
                                   *   timer expires */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* User function prototype. Returns true to reload the timer, and the
 * function can modify the next interval if desired.
 */

typedef bool (*tccb_t)(FAR uint32_t *next_interval_us);

/* This is the type of the argument passed to the TCIOC_SETHANDLER ioctl */

struct timer_sethandler_s
{
  CODE tccb_t newhandler;   /* The new timer interrupt handler */
  CODE tccb_t oldhandler;   /* The previous timer interrupt handler (if any) */
};

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

/* This structure provides the "lower-half" driver operations available to
 * the "upper-half" driver.
 */

struct timer_lowerhalf_s;
struct timer_ops_s
{
  /* Required methods ********************************************************/
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

  /* Call this user provider timeout handler on timeout.
   * NOTE:  Providing handler==NULL disable.
   */

  CODE tccb_t (*sethandler)(FAR struct timer_lowerhalf_s *lower,
                            CODE tccb_t handler);

  /* Any ioctl commands that are not recognized by the "upper-half" driver
   * are forwarded to the lower half driver through this method.
   */

  CODE int (*ioctl)(FAR struct timer_lowerhalf_s *lower, int cmd,
                    unsigned long arg);
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
 * Input parameters:
 *   dev path - The full path to the driver to be registers in the NuttX
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
 * Input parameters:
 *   handle - This is the handle that was returned by timer_register()
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void timer_unregister(FAR void *handle);

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
#endif  /* __INCLUDE_NUTTX_TIMERS_TIMER_H */
