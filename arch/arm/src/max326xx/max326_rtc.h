/****************************************************************************
 * arch/arm/src/max326xx/max326_rtc.h
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

#ifndef __ARCH_ARM_SRC_MAX326XX_MAX326_RTC_H
#define __ARCH_ARM_SRC_MAX326XX_MAX326_RTC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <fixedmath.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/* Type of callback invoked when an ALARM interrupt occurs */

typedef void (*alm_callback_t)(void *arg, unsigned int alarmid);

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
 * Name: max326_rtc_setalarm
 *
 * Description:
 *   Set an alarm to an absolute time using associated hardware.
 *
 * Input Parameters:
 *  ts  - Alarm time
 *  id  - A
 *  cb  - Callback invoked when alarm expires
 *  arg - Argument passed with the alarm
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_ALARM
int max326_rtc_setalarm(struct timespec *ts, alm_callback_t cb,
                        void *arg);
#endif

/****************************************************************************
 * Name: max326_rtc_rdalarm
 *
 * Description:
 *   Query an alarm configured in hardware.
 *
 * Input Parameters:
 *  ftime - Location to return the current alarm setting.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_ALARM
int max326_rtc_rdalarm(b32_t *ftime);
#endif

/****************************************************************************
 * Name: max326_rtc_cancelalarm
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
int max326_rtc_cancelalarm(void);
#endif

/****************************************************************************
 * Name: max326_rtc_lowerhalf
 *
 * Description:
 *   Instantiate the RTC lower half driver for the MAX326XX.  General usage:
 *
 *     #include <nuttx/timers/rtc.h>
 *     #include "max326_rtc.h"
 *
 *     struct rtc_lowerhalf_s *lower;
 *     lower = max326_rtc_lowerhalf();
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
struct rtc_lowerhalf_s *max326_rtc_lowerhalf(void);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_MAX326XX_MAX326_RTC_H */
