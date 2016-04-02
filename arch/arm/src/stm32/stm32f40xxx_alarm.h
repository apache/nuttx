/****************************************************************************
 * arch/arm/src/include/stm32/stm32f0xxx_alarm.h
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Author: Neil hancock - delegated to Gregory Nutt Mar 30, 2016
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

#ifndef __ARCH_ARM_SRC_STM32_STM32F0XXX_ALARM_H
#define __ARCH_ARM_SRC_STM32_STM32F0XXX_ALARM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifdef CONFIG_RTC_ALARM

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RTC_ALRMR_DIS_MASK            (RTC_ALRMR_MSK4 | RTC_ALRMR_MSK3 | \
                                       RTC_ALRMR_MSK2 | RTC_ALRMR_MSK1)
#define RTC_ALRMR_DIS_DATE_HOURS_MASK (RTC_ALRMR_MSK4 | RTC_ALRMR_MSK3)
#define RTC_ALRMR_DIS_DATE_MASK       (RTC_ALRMR_MSK4 )

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef CODE void (*rtc_ext_cb_t)(void);
typedef unsigned int rtc_alarmreg_t;

/* These features are known to map to STM32 RTC from stm32F4xx and appear to
 * map to beyond stm32F4xx, & stm32L0xx there appears to be a small variant
 * with stm32F3 but do not map to stm32F0, F1, F2
 */

enum rtc_ext_type_e
{
  RTC_ALARMA_REL,              /* EXTI RTC_ALARM Event */
  RTC_ALARMB_REL,              /* EXTI RTC_ALARM Event */
  RTC_EXT_LAST
};

union rtc_ext_param_u
{
  int alm_minutes;

  /* Other param can share the space */
};

struct up_alarm_update_s
{
  int alm_type;                /* enum rtc_ext_type_e */
  union rtc_ext_param_u param;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_rtc_ext_set_cb
 *
 * Description:
 *   Set up an alarm callback for alarm A /B
 *
 * Input Parameters:
 *  callback - function address
 *
 * Returned Value:
 *  OK if completed else -1 if failed set the callback
 *
 ****************************************************************************/

int stm32_rtc_ext_set_cb(int rtc_ext_type_e, rtc_ext_cb_t callback);

/****************************************************************************
 * Name: stm32_rtc_ext_update
 *
 * Description:
 *   Set up the STM32 RTC hardware. This includes
 *   - two alarms  (ALARM A and ALARM B).
 *   - other functions that it can be extended to as required
 *
 * Input Parameters:
 *  alm_setup - the details of the alarm
 *      alm_type RTC_ALARMA_REL/RTC_ALARMB_REL - set a relative alarm in minutes using associated hardware
 *      all other types not implemented
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

int stm32_rtc_ext_update(struct up_alarm_update_s *alm_setup);

#endif /* CONFIG_RTC_ALARM */
#endif /* __ARCH_ARM_SRC_STM32_STM32F0XXX_ALARM_H */
