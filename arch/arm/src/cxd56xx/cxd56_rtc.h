/****************************************************************************
 * arch/arm/src/cxd56xx/cxd56_rtc.h
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

#ifndef __ARCH_ARM_SRC_CXD56XX_CXD56_RTC_H
#define __ARCH_ARM_SRC_CXD56XX_CXD56_RTC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <time.h>
#include <nuttx/timers/rtc.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

#ifdef CONFIG_RTC_ALARM

/* The form of an alarm callback */

typedef void (*alm_callback_t)(void *arg, unsigned int alarmid);

enum alm_id_e
{
  RTC_ALARM0 = 0,           /* RTC ALARM 0 */
  RTC_ALARM_LAST,
  RTC_ALARM1 = 1,           /* RTC ALARM 1 */
  RTC_ALARM2,               /* RTC ALARM 2 (relative) */
};

/* Structure used to pass parameters to set an alarm */

struct alm_setalarm_s
{
  int             as_id;    /* enum alm_id_e */
  struct timespec as_time;  /* Alarm expiration time */
  alm_callback_t  as_cb;    /* Callback (if non-NULL) */
  void       *as_arg;       /* Argument for callback */
};

#endif /* CONFIG_RTC_ALARM */

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
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Name: cxd56_rtc_count
 *
 * Description:
 *   Get RTC raw counter value
 *
 * Returned Value:
 *   64bit counter value running at 32kHz
 *
 ****************************************************************************/

uint64_t cxd56_rtc_count(void);

/****************************************************************************
 * Name: cxd56_rtc_almcount
 *
 * Description:
 *   Get RTC raw alarm counter value
 *
 * Returned Value:
 *   64bit alarm counter value running at 32kHz
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_ALARM
uint64_t cxd56_rtc_almcount(void);
#endif /* CONFIG_RTC_ALARM */

/****************************************************************************
 * Name: cxd56_rtc_setalarm
 *
 * Description:
 *   Set up an alarm.
 *
 * Input Parameters:
 *   alminfo - Information about the alarm configuration.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_ALARM
int cxd56_rtc_setalarm(struct alm_setalarm_s *alminfo);
#endif /* CONFIG_RTC_ALARM */

/****************************************************************************
 * Name: cxd56_rtc_cancelalarm
 *
 * Description:
 *   Cancel an alaram.
 *
 * Input Parameters:
 *  alarmid - Identifies the alarm to be cancelled
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_ALARM
int cxd56_rtc_cancelalarm(enum alm_id_e alarmid);
#endif /* CONFIG_RTC_ALARM */

/****************************************************************************
 * Name: cxd56_rtc_lowerhalf
 *
 * Description:
 *   Instantiate the RTC lower half driver for the STM32L4.  General usage:
 *
 *     #include <nuttx/timers/rtc.h>
 *     #include "stm32l4_rtc.h>
 *
 *     struct rtc_lowerhalf_s *lower;
 *     lower = stm32l4_rtc_lowerhalf();
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
struct rtc_lowerhalf_s *cxd56_rtc_lowerhalf(void);
#endif /* CONFIG_RTC_DRIVER */

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_CXD56XX_CXD56_RTC_H */
