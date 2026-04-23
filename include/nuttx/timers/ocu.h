/****************************************************************************
 * include/nuttx/timers/ocu.h
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

/* The OCU (Output Compare Unit) driver provides a standard interface for
 * hardware output compare functionality. An OCU channel compares a
 * free-running counter against a programmable threshold and performs
 * configurable actions (pin toggle, notification callback) on match.
 *
 * The driver is split into two parts:
 *
 * 1) An "upper half", generic driver that provides the common OCU interface
 *    to application level code, and
 * 2) A "lower half", platform-specific driver that implements the low-level
 *    timer/compare controls to implement the OCU functionality.
 */

#ifndef __INCLUDE_NUTTX_TIMERS_OCU_H
#define __INCLUDE_NUTTX_TIMERS_OCU_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/signal.h>

#include <stdbool.h>
#include <stdint.h>

#ifdef CONFIG_OCU

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* IOCTL Commands ***********************************************************/

/* The OCU module uses a standard character driver framework. The majority
 * of the functionality is implemented in driver ioctl calls.
 *
 * OCUIOC_START              - Start the OCU channel, enabling compare match
 *                             actions.
 *                             Argument: Ignored
 * OCUIOC_STOP               - Stop the OCU channel, halting compare match
 *                             actions. The free-running counter is NOT
 *                             stopped.
 *                             Argument: Ignored
 * OCUIOC_GETSTATUS          - Get the current status of the OCU channel.
 *                             Argument: A writeable pointer to struct
 *                             ocu_status_s.
 * OCUIOC_GETCOUNTER         - Read the current counter value in ticks.
 *                             Argument: A writeable pointer to uint32_t.
 * OCUIOC_SETABSTHRESHOLD    - Set the compare threshold to an absolute
 *                             value.
 *                             Argument: A read-only pointer to struct
 *                             ocu_abs_threshold_s.
 * OCUIOC_SETRELTHRESHOLD    - Set the compare threshold relative to the
 *                             current counter value.
 *                             Argument: A read-only pointer to struct
 *                             ocu_rel_threshold_s.
 * OCUIOC_SETPINSTATE        - Set the output pin level immediately (only
 *                             when channel is STOPPED).
 *                             Argument: enum ocu_pinstate_e value.
 * OCUIOC_SETPINACTION       - Set the automatic pin action on compare
 *                             match.
 *                             Argument: enum ocu_pinaction_e value.
 * OCUIOC_SETADVANCETHRESHOLD- Set the compare threshold relative to the
 *                             current threshold value (advance by step).
 *                             new_threshold = current_threshold + step.
 *                             Argument: A read-only pointer to struct
 *                             ocu_advance_threshold_s.
 * OCUIOC_NOTIFICATION       - Enable or disable compare match signal
 *                             notification to user space.  When enabled,
 *                             the upper-half automatically registers an
 *                             internal notifier callback with the
 *                             lower-half; on compare match, a signal is
 *                             sent to the specified process.
 *                             Argument: A read-only pointer to struct
 *                             ocu_notify_s.
 *
 * NOTE: There is no OCUIOC_SETCALLBACK ioctl.  Kernel-internal drivers
 * that need a direct ISR callback must use the OCU_SETCALLBACK() macro
 * to register directly with the lower-half, bypassing the upper-half
 * (same pattern as TIMER_SETCALLBACK in timer.h).
 */

#define OCUIOC_START               _OCUIOC(0x0001)
#define OCUIOC_STOP                _OCUIOC(0x0002)
#define OCUIOC_GETSTATUS           _OCUIOC(0x0003)
#define OCUIOC_GETCOUNTER          _OCUIOC(0x0004)
#define OCUIOC_SETABSTHRESHOLD     _OCUIOC(0x0005)
#define OCUIOC_SETRELTHRESHOLD     _OCUIOC(0x0006)
#define OCUIOC_SETADVANCETHRESHOLD _OCUIOC(0x0007)
#define OCUIOC_SETPINSTATE         _OCUIOC(0x0008)
#define OCUIOC_SETPINACTION        _OCUIOC(0x0009)
#define OCUIOC_NOTIFICATION        _OCUIOC(0x000a)

/* Bit Settings *************************************************************/

/* Bit settings for the struct ocu_status_s flags field */

#define OCUFLAGS_ACTIVE            (1 << 0) /* 1=The channel is running */
#define OCUFLAGS_NOTIF_EN          (1 << 1) /* 1=Notification is enabled */
#define OCUFLAGS_PIN_AVAIL         (1 << 2) /* 1=Output pin is available */

/* Method access helper macros **********************************************/

#define OCU_START(l)               ((l)->ops->start ? \
                                    (l)->ops->start(l) : -ENOTSUP)
#define OCU_STOP(l)                ((l)->ops->stop ? \
                                    (l)->ops->stop(l) : -ENOTSUP)
#define OCU_GETSTATUS(l,s)         ((l)->ops->getstatus ? \
                                    (l)->ops->getstatus(l,s) : -ENOTSUP)
#define OCU_GETCOUNTER(l,v)        ((l)->ops->getcounter ? \
                                    (l)->ops->getcounter(l,v) : -ENOTSUP)
#define OCU_SETABSTHRESHOLD(l,t)   ((l)->ops->setabsthreshold ? \
                                    (l)->ops->setabsthreshold(l,t) : -ENOTSUP)
#define OCU_SETRELTHRESHOLD(l,t)   ((l)->ops->setrelthreshold ? \
                                    (l)->ops->setrelthreshold(l,t) : -ENOTSUP)
#define OCU_SETPINSTATE(l,s)       ((l)->ops->setpinstate ? \
                                    (l)->ops->setpinstate(l,s) : -ENOTSUP)
#define OCU_SETPINACTION(l,a)      ((l)->ops->setpinaction ? \
                                    (l)->ops->setpinaction(l,a) : -ENOTSUP)
#define OCU_SETCALLBACK(l,c,a)     do \
                                     { \
                                       if ((l)->ops->setcallback) \
                                         { \
                                           (l)->ops->setcallback(l,c,a); \
                                         } \
                                     } \
                                   while (0)
#define OCU_NOTIFICATION(l,en)     ((l)->ops->notification ? \
                                    (l)->ops->notification(l,en) : -ENOTSUP)
#define OCU_SETADVANCETHRESHOLD(l,t) \
                                   ((l)->ops->setadvancethreshold ? \
                                    (l)->ops->setadvancethreshold(l,t) : \
                                    -ENOTSUP)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Output pin state */

enum ocu_pinstate_e
{
  OCU_PINSTATE_LOW  = 0,  /* Output pin is LOW */
  OCU_PINSTATE_HIGH = 1,  /* Output pin is HIGH */
};

/* Automatic pin action on compare match */

enum ocu_pinaction_e
{
  OCU_PINACTION_DISABLE  = 0,  /* No pin action on compare match */
  OCU_PINACTION_SET_LOW  = 1,  /* Set pin LOW on compare match */
  OCU_PINACTION_SET_HIGH = 2,  /* Set pin HIGH on compare match */
  OCU_PINACTION_TOGGLE   = 3,  /* Toggle pin on compare match */
};

/* Return type for threshold setting APIs */

enum ocu_threshold_ret_e
{
  OCU_THRESHOLD_IN_REF_INTERVAL  = 0,  /* Compare match will occur in the
                                        * current reference interval */
  OCU_THRESHOLD_OUT_REF_INTERVAL = 1,  /* Compare match will NOT occur in the
                                        * current reference interval */
};

/* OCU channel status */

struct ocu_status_s
{
  uint32_t flags;           /* See OCUFLAGS_* definitions above */
  uint32_t counter;         /* Current counter value in ticks */
  uint32_t threshold;       /* Current threshold value in ticks */
  uint32_t maxcounter;      /* Maximum counter value */
};

/* Absolute threshold setting */

struct ocu_abs_threshold_s
{
  uint32_t reference;                   /* Reference value */
  uint32_t absolute;                    /* Absolute threshold in ticks */
  enum ocu_threshold_ret_e result;      /* [out] Match in ref interval? */
};

/* Relative threshold setting (relative to current counter) */

struct ocu_rel_threshold_s
{
  uint32_t relative;                    /* Relative offset from counter */
  enum ocu_threshold_ret_e result;      /* [out] Match in ref interval? */
};

/* Advance threshold setting (relative to current threshold) */

struct ocu_advance_threshold_s
{
  uint32_t step;                        /* Step from current threshold */
  enum ocu_threshold_ret_e result;      /* [out] Match in ref interval? */
};

/* Notification configuration */

struct ocu_notify_s
{
  bool     enable;          /* true=enable, false=disable notification */
  pid_t    pid;             /* PID of the task to notify */
  struct sigevent event;    /* Signal event for notification */
};

/* Compare match callback type.
 * Called from interrupt context when a compare match occurs.
 * Returns true to automatically re-arm with the same threshold.
 */

typedef CODE bool (*ocu_callback_t)(FAR uint32_t *next_threshold,
                                    FAR void *arg);

/* This structure provides the "lower-half" driver operations available to
 * the "upper-half" driver.
 */

struct ocu_lowerhalf_s;
struct ocu_ops_s
{
  /* This method is called when the driver is first opened. The lower half
   * driver should configure and initialize the hardware.
   */

  CODE int (*setup)(FAR struct ocu_lowerhalf_s *lower);

  /* This method is called when the last file descriptor is closed. The
   * lower half driver should stop output, free resources, and disable
   * the timer hardware.
   */

  CODE int (*shutdown)(FAR struct ocu_lowerhalf_s *lower);

  /* Start the OCU channel, enabling compare match actions. */

  CODE int (*start)(FAR struct ocu_lowerhalf_s *lower);

  /* Stop the OCU channel. The free-running counter is NOT stopped. */

  CODE int (*stop)(FAR struct ocu_lowerhalf_s *lower);

  /* Get the current status of the OCU channel. */

  CODE int (*getstatus)(FAR struct ocu_lowerhalf_s *lower,
                        FAR struct ocu_status_s *status);

  /* Read the current counter value. */

  CODE int (*getcounter)(FAR struct ocu_lowerhalf_s *lower,
                         FAR uint32_t *counter);

  /* Set the compare threshold to an absolute value. */

  CODE int (*setabsthreshold)(FAR struct ocu_lowerhalf_s *lower,
                              FAR struct ocu_abs_threshold_s *threshold);

  /* Set the compare threshold relative to the current counter value. */

  CODE int (*setrelthreshold)(FAR struct ocu_lowerhalf_s *lower,
                              FAR struct ocu_rel_threshold_s *threshold);

  /* Set the compare threshold relative to the current threshold value
   * (advance by step).  new_threshold = current_threshold + step.
   */

  CODE int (*setadvancethreshold)(FAR struct ocu_lowerhalf_s *lower,
                                  FAR struct ocu_advance_threshold_s *t);

  /* Set the compare match callback (timer-style setcallback pattern).
   * NOTE: Providing callback==NULL disables the callback.
   * The lower-half driver holds the callback pointer and calls it directly
   * from the compare match ISR.
   */

  CODE void (*setcallback)(FAR struct ocu_lowerhalf_s *lower,
                           CODE ocu_callback_t callback, FAR void *arg);

  /* Set the output pin level immediately (only when channel is STOPPED) */

  CODE int (*setpinstate)(FAR struct ocu_lowerhalf_s *lower,
                          enum ocu_pinstate_e state);

  /* Set the automatic pin action on compare match */

  CODE int (*setpinaction)(FAR struct ocu_lowerhalf_s *lower,
                           enum ocu_pinaction_e action);

  /* Enable or disable the hardware compare-match notification (interrupt).
   * This is independent of setcallback(): the upper-half registers the
   * notifier callback via setcallback() and then enables/disables the
   * hardware notification via this method (both driven from the
   * OCUIOC_NOTIFICATION ioctl).
   */

  CODE int (*notification)(FAR struct ocu_lowerhalf_s *lower, bool enable);

  /* Any ioctl commands that are not recognized by the "upper-half" driver
   * are forwarded to the lower half driver through this method.
   */

  CODE int (*ioctl)(FAR struct ocu_lowerhalf_s *lower, int cmd,
                    unsigned long arg);
};

/* This structure provides the publicly visible representation of the
 * "lower-half" driver state structure. "lower half" drivers will have an
 * internal structure definition that will be cast-compatible with this
 * structure definitions.
 */

struct ocu_lowerhalf_s
{
  /* Publicly visible portion of the "lower-half" driver state structure. */

  FAR const struct ocu_ops_s *ops;  /* Lower half operations */
};

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

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: ocu_register
 *
 * Description:
 *   This function binds an instance of a "lower half" OCU driver with the
 *   "upper half" OCU device and registers that device so that it can be
 *   used by application code.
 *
 * Input Parameters:
 *   devpath - The full path to the driver to be registered in the NuttX
 *     pseudo-filesystem. The recommended convention is to name all OCU
 *     drivers as "/dev/ocu0", "/dev/ocu1", etc.
 *   lower - A pointer to an instance of lower half OCU driver. This
 *     instance is bound to the OCU driver and must persist as long as the
 *     driver persists.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int ocu_register(FAR const char *devpath,
                 FAR struct ocu_lowerhalf_s *lower);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_OCU */
#endif /* __INCLUDE_NUTTX_TIMERS_OCU_H */
