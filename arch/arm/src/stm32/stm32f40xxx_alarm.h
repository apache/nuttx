/****************************************************************************
 * arch/arm/src/stm32/stm32f40xxx_alarm.h
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

#ifndef __ARCH_ARM_SRC_STM32_STM32F40XXX_ALARM_H
#define __ARCH_ARM_SRC_STM32_STM32F40XXX_ALARM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <time.h>

#ifdef CONFIG_RTC_ALARM

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef void (*alm_callback_t)(void *arg, unsigned int alarmid);

/* These features are known to map to STM32 RTC from stm32F4xx and appear to
 * map to beyond stm32F4xx and stm32L0xx there appears to be a small variant
 * with stm32F3 but do not map to stm32F0, F1, F2
 */

enum alm_id_e
{
  RTC_ALARMA = 0,            /* RTC ALARM A */
  RTC_ALARMB,                /* RTC ALARM B */
  RTC_ALARM_LAST
};

/* Structure used to pass parameters to set an alarm */

struct alm_setalarm_s
{
  int as_id;                /* enum alm_id_e */
  struct tm as_time;        /* Alarm expiration time */
  alm_callback_t as_cb;     /* Callback (if non-NULL) */
  void *as_arg;             /* Argument for callback */
};

/* Structure used to pass parameters to query an alarm */

struct alm_rdalarm_s
{
  int ar_id;                /* enum alm_id_e */
  struct rtc_time *ar_time; /* Argument for storing ALARM RTC time */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_rtc_setalarm
 *
 * Description:
 *   Set an alarm to an absolute time using associated hardware.
 *
 * Input Parameters:
 *  alminfo - Information about the alarm configuration.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

int stm32_rtc_setalarm(struct alm_setalarm_s *alminfo);

/****************************************************************************
 * Name: stm32_rtc_rdalarm
 *
 * Description:
 *   Query an alarm configured in hardware.
 *
 * Input Parameters:
 *  alminfo - Information about the alarm configuration.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

int stm32_rtc_rdalarm(struct alm_rdalarm_s *alminfo);

/****************************************************************************
 * Name: stm32_rtc_cancelalarm
 *
 * Description:
 *   Cancel an alarm.
 *
 * Input Parameters:
 *  alarmid - Identifies the alarm to be cancelled
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

int stm32_rtc_cancelalarm(enum alm_id_e alarmid);

#endif /* CONFIG_RTC_ALARM */
#endif /* __ARCH_ARM_SRC_STM32_STM32F40XXX_ALARM_H */
