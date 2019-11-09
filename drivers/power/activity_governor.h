/****************************************************************************
 * activity_governor.h
 *
 *   Copyright (C) 2011-2012, 2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *   Author: Matias Nitsche <mnitsche@dc.uba.ar>
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

#ifndef __DRIVERS_POWER_ACTIVITY_GOVERNER_H
#define __DRIVERS_POWER_ACTIVITY_GOVERNER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/power/pm.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* CONFIG_PM_GOVERNOR_SLICEMS. The activity based governor collects activity
 * counts in time slices.  At the end of the time slice, the count accumulated
 * during that interval is applied to an averaging algorithm to determine
 * the activity level.
 *
 * CONFIG_PM_GOVERNOR_SLICEMS provides the duration of that time slice in
 * milliseconds.  Default: 100 Milliseconds
 */

#ifndef CONFIG_PM_GOVERNOR_SLICEMS
#  define CONFIG_PM_GOVERNOR_SLICEMS  100 /* Default is 100 msec */
#endif

#if CONFIG_PM_GOVERNOR_SLICEMS < 1
#  error CONFIG_PM_GOVERNOR_SLICEMS invalid
#endif

/* The averaging algorithm is simply: Y = (An*X + SUM(Ai*Yi))/SUM(Aj), where
 * i = 1..n-1 and j= 1..n, n is the length of the "memory", Ai is the
 * weight applied to each value, and X is the current activity.  These weights
 * may be negative and a limited to the range of int16_t.
 *
 * CONFIG_PM_GOVERNOR_MEMORY:
 *  provides the memory for the algorithm.  Default: 2
 * CONFIG_PM_GOVERNOR_COEFn:
 *  provides weight for each sample.  Default: 1
 *
 * Setting CONFIG_PM_GOVERNOR_MEMORY=1 disables all smoothing.
 */

#ifndef CONFIG_PM_GOVERNOR_MEMORY
#  define CONFIG_PM_GOVERNOR_MEMORY 2
#endif

#if CONFIG_PM_GOVERNOR_MEMORY < 1
#  error "CONFIG_PM_GOVERNOR_MEMORY must be >= 1"
#endif

#ifndef CONFIG_PM_GOVERNOR_COEFN
#  define CONFIG_PM_GOVERNOR_COEFN 1
#endif

#if CONFIG_PM_GOVERNOR_MEMORY > 1 && !defined(CONFIG_PM_GOVERNOR_COEF1)
#  define CONFIG_PM_GOVERNOR_COEF1 1
#endif

#if CONFIG_PM_GOVERNOR_MEMORY > 2 && !defined(CONFIG_PM_GOVERNOR_COEF2)
#  define CONFIG_PM_GOVERNOR_COEF2 1
#endif

#if CONFIG_PM_GOVERNOR_MEMORY > 3 && !defined(CONFIG_PM_GOVERNOR_COEF3)
#  define CONFIG_PM_GOVERNOR_COEF3 1
#endif

#if CONFIG_PM_GOVERNOR_MEMORY > 4 && !defined(CONFIG_PM_GOVERNOR_COEF4)
#  define CONFIG_PM_GOVERNOR_COEF4 1
#endif

#if CONFIG_PM_GOVERNOR_MEMORY > 5 && !defined(CONFIG_PM_GOVERNOR_COEF5)
#  define CONFIG_PM_GOVERNOR_COEF5 1
#endif

#if CONFIG_PM_GOVERNOR_MEMORY > 6
#  warning "This logic needs to be extended"
#endif

/* State changes then occur when the weight activity account crosses
 * threshold values for certain periods of time (time slice count).
 *
 * CONFIG_PM_GOVERNOR_xxxENTER_THRESH is the threshold value for entering
 * state xxx.
 * CONFIG_PM_GOVERNOR_xxxENTER_COUNT is the count for entering state xxx.
 *
 * Resuming to normal state, on the other hand, is usually immediate and
 * controlled by wakeup conditions established by the platform.  The PM
 * module only recommends reduced power states.
 */

#ifndef CONFIG_PM_GOVERNOR_IDLEENTER_THRESH
#  define CONFIG_PM_GOVERNOR_IDLEENTER_THRESH    1   /* <=1: Essentially no activity */
#endif

#ifndef CONFIG_PM_GOVERNOR_IDLEEXIT_THRESH
#  define CONFIG_PM_GOVERNOR_IDLEEXIT_THRESH     2   /* >=2: Active */
#endif

#if CONFIG_PM_GOVERNOR_IDLEENTER_THRESH >= CONFIG_PM_GOVERNOR_IDLEEXIT_THRESH
#  error "Must have CONFIG_PM_GOVERNOR_IDLEENTER_THRESH < CONFIG_PM_GOVERNOR_IDLEEXIT_THRESH"
#endif

#ifndef CONFIG_PM_GOVERNOR_IDLEENTER_COUNT
#  define CONFIG_PM_GOVERNOR_IDLEENTER_COUNT     30  /* Thirty IDLE slices to enter
                                                      * IDLE mode from normal
                                                      */
#endif

#ifndef CONFIG_PM_GOVERNOR_STANDBYENTER_THRESH
#  define CONFIG_PM_GOVERNOR_STANDBYENTER_THRESH 1   /*  <=1: Essentially no activity */
#endif

#ifndef CONFIG_PM_GOVERNOR_STANDBYEXIT_THRESH
#  define CONFIG_PM_GOVERNOR_STANDBYEXIT_THRESH  2   /* >=2: Active */
#endif

#if CONFIG_PM_GOVERNOR_STANDBYENTER_THRESH >= CONFIG_PM_GOVERNOR_STANDBYEXIT_THRESH
#  error "Must have CONFIG_PM_GOVERNOR_STANDBYENTER_THRESH < CONFIG_PM_GOVERNOR_STANDBYEXIT_THRESH"
#endif

#ifndef CONFIG_PM_GOVERNOR_STANDBYENTER_COUNT
#  define CONFIG_PM_GOVERNOR_STANDBYENTER_COUNT  50  /* Fifty IDLE slices to enter
                                                      * STANDBY mode from IDLE
                                                      */
#endif

#ifndef CONFIG_PM_GOVERNOR_SLEEPENTER_THRESH
#  define CONFIG_PM_GOVERNOR_SLEEPENTER_THRESH   1   /*  <=1: Essentially no activity */
#endif

#ifndef CONFIG_PM_GOVERNOR_SLEEPEXIT_THRESH
#  define CONFIG_PM_GOVERNOR_SLEEPEXIT_THRESH    2   /* >=2: Active */
#endif

#if CONFIG_PM_GOVERNOR_SLEEPENTER_THRESH >= CONFIG_PM_GOVERNOR_SLEEPEXIT_THRESH
#  error "Must have CONFIG_PM_GOVERNOR_SLEEPENTER_THRESH < CONFIG_PM_GOVERNOR_SLEEPEXIT_THRESH"
#endif

#ifndef CONFIG_PM_GOVERNOR_SLEEPENTER_COUNT
#  define CONFIG_PM_GOVERNOR_SLEEPENTER_COUNT    70  /* 70 IDLE slices to enter SLEEP
                                                      * mode from STANDBY
                                                      */
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

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

FAR struct pm_governor_s *pm_activity_governor_initialize(void);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif // __DRIVERS_POWER_ACTIVITY_GOVERNER_H
