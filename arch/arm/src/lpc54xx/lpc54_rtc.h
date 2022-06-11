/****************************************************************************
 * arch/arm/src/lpc54xx/lpc54_rtc.h
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

#ifndef __ARCH_ARM_SRC_LPC54XX_LPC54_RTC_H
#define __ARCH_ARM_SRC_LPC54XX_LPC54_RTC_H

#include <nuttx/config.h>

#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RTC_MAGIC               (0xfacefeed)
#define RTC_MAGIC_REG            LPC54_RTC_GPREG(0)

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

#ifdef CONFIG_RTC_ALARM
/* The form of an alarm callback */

typedef void (*alarmcb_t)(void);
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
 * Public Functions Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: lpc54_rtc_setalarm
 *
 * Description:
 *   Set up an alarm.
 *
 * Input Parameters:
 *   tp - the time to set the alarm
 *   callback - the function to call when the alarm expires.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_ALARM
struct timespec;
int lpc54_rtc_setalarm(const struct timespec *tp, alarmcb_t callback);
#endif

/****************************************************************************
 * Name: lpc54_rtc_rdalarm
 *
 * Description:
 *   Query an alarm configured in hardware.
 *
 * Input Parameters:
 *  time - Current alarm setting.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_ALARM
struct tm;
int lpc54_rtc_rdalarm(struct tm *time);
#endif

/****************************************************************************
 * Name: lpc54_rtc_cancelalarm
 *
 * Description:
 *   Cancel a pending alarm alarm
 *
 * Input Parameters:
 *   none
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_ALARM
int lpc54_rtc_cancelalarm(void);
#endif

/****************************************************************************
 * Name: lpc54_rtc_lowerhalf
 *
 * Description:
 *   Instantiate the RTC lower half driver for the LPC54.  General usage:
 *
 *     #include <nuttx/timers/rtc.h>
 *     #include "lpc54_rtc.h"
 *
 *     struct rtc_lowerhalf_s *lower;
 *     lower = lpc54_rtc_lowerhalf();
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
struct rtc_lowerhalf_s *lpc54_rtc_lowerhalf(void);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_LPC54XX_LPC54_RTC_H */
