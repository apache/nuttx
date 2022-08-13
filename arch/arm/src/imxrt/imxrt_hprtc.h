/****************************************************************************
 * arch/arm/src/imxrt/imxrt_hprtc.h
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_IMXRT_IMXRT_HPRTC_H
#define __ARCH_ARM_SRC_IMXRT_IMXRT_HPRTC_H

#include <nuttx/config.h>

#include "chip.h"

#ifdef CONFIG_IMXRT_SNVS_HPRTC

/****************************************************************************
 * Preprocessor Definitions
 ****************************************************************************/

#ifdef CONFIG_RTC_DATETIME
#  error CONFIG_RTC_DATETIME should not be selected with this driver
#endif

#ifdef CONFIG_RTC_PERIODIC
#  error CONFIG_RTC_PERIODIC should not be selected with this driver
#endif

/* REVISIT: This is probably supportable.  The 47 bit timer does have
 * accuracy greater than 1 second.
 */

#ifdef CONFIG_RTC_HIRES
#  error CONFIG_RTC_PERIODIC should not be selected with this driver
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Callback type used by the HPRTC log to notify the RTC driver when the
 * alarm expires.
 */

typedef void (*hprtc_alarm_callback_t)(void);

/****************************************************************************
 * Public Data
 ****************************************************************************/

#if !defined(CONFIG_IMXRT_SNVS_LPSRTC) && defined(CONFIG_RTC_DRIVER)
bool g_hprtc_timset;  /* True:  time has been set since power up */
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Functions used only for HPRTC
 ****************************************************************************/

/****************************************************************************
 * Logic Common to LPSRTC and HPRTC
 ****************************************************************************/

/****************************************************************************
 * Name: imxrt_rtc_lowerhalf
 *
 * Description:
 *   Instantiate the RTC lower half driver for the i.MXRT.  General usage:
 *
 *     #include <nuttx/timers/rtc.h>
 *     #include "imxrt_hprtc.h"
 *
 *     struct rtc_lowerhalf_s *lower;
 *     lower = imxrt_hprtc_lowerhalf();
 *     rtc_initialize(0, lower);
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   On success, a non-NULL RTC lower interface is returned.  NULL is
 *   returned on any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_DRIVER
struct rtc_lowerhalf_s;
struct rtc_lowerhalf_s *imxrt_rtc_lowerhalf(void);
#endif

/****************************************************************************
 * Name: imxrt_hprtc_initialize
 *
 * Description:
 *   Initialize the LPSRTC per the selected configuration.
 *    This function is called via up_rtc_initialize (see imxrt_hprtc.c).
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

int imxrt_hprtc_initialize(void);

/****************************************************************************
 * Name: imxrt_hprtc_synchronize
 *
 * Description:
 *   Synchronize the HPRTC to the LPSRTC and enable the HPRTC timer.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_IMXRT_SNVS_LPSRTC
void imxrt_hprtc_synchronize(void);
#endif

/****************************************************************************
 * Name: imxrt_hprtc_time
 *
 * Description:
 *   Get the current time in seconds.
 *   This is the underlying implementation of the up_rtc_tim() function that
 *   is used by the RTOS during initialization to set up the system time.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   The current time in seconds
 *
 ****************************************************************************/

uint32_t imxrt_hprtc_time(void);

/****************************************************************************
 * Name: imxrt_hprtc_getalarm
 *
 * Description:
 *   Get the current alarm setting in seconds.
 *   This is only used by the lower half RTC driver.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   The current alarm setting in seconds
 *
 ****************************************************************************/

#if defined(CONFIG_RTC_ALARM) && defined(CONFIG_RTC_DRIVER)
uint32_t imxrt_hprtc_getalarm(void);
#endif

/****************************************************************************
 * Name: imxrt_hprtc_setalarm
 *
 * Description:
 *   Set the alarm (in seconds) and enable alarm interrupts.
 *   This is only used by the lower half RTC driver.
 *
 * Input Parameters:
 *   sec - The new alarm setting
 *
 * Returned Value:
 *   The current alarm setting in seconds
 *
 ****************************************************************************/

#if defined(CONFIG_RTC_ALARM) && defined(CONFIG_RTC_DRIVER)
int imxrt_hprtc_setalarm(struct timespec *ts,
                         hprtc_alarm_callback_t cb);
#endif

/****************************************************************************
 * Name: imxrt_hprtc_alarmdisable
 *
 * Description:
 *    Disable alarm interrupts.
 *    Used internally after the receipt of the alarm interrupt.
 *    Also called by the lower-half RTC driver in order to cancel an alarm.
 *
 * Input Parameters:
 *    None
 *
 * Returned Value:
 *    None
 *
 ****************************************************************************/

#if defined(CONFIG_RTC_ALARM) && defined(CONFIG_RTC_DRIVER)
void imxrt_hprtc_alarmdisable(void);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __ASSEMBLY__ */

#endif /* CONFIG_IMXRT_SNVS_HPRTC */
#endif /* __ARCH_ARM_SRC_IMXRT_IMXRT_HPRTC_H */
