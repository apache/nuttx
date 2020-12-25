/****************************************************************************
 * arch/xtensa/src/esp32/esp32_pm.h
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

#ifndef __ARCH_XTENSA_SRC_ESP32_ESP32_PMSLEEP_H
#define __ARCH_XTENSA_SRC_ESP32_ESP32_PMSLEEP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#ifdef CONFIG_PM

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Sleep wakeup cause */

enum esp32_sleep_source_e
{
/* In case of deep sleep, reset was not caused by exit from deep sleep */

  ESP_SLEEP_WAKEUP_UNDEFINED,

/* Not a wakeup cause, used to disable all wakeup sources with
 * esp_sleep_disable_wakeup_source
 */

  ESP_SLEEP_WAKEUP_ALL,

/* Wakeup caused by external signal using RTC_IO */

  ESP_SLEEP_WAKEUP_EXT0,

/* Wakeup caused by external signal using RTC_CNTL */

  ESP_SLEEP_WAKEUP_EXT1,

/* Wakeup caused by timer */

  ESP_SLEEP_WAKEUP_TIMER,

/* Wakeup caused by touchpad */

  ESP_SLEEP_WAKEUP_TOUCHPAD,

/* Wakeup caused by ULP program */

  ESP_SLEEP_WAKEUP_ULP,

/* Wakeup caused by GPIO (light sleep only) */

  ESP_SLEEP_WAKEUP_GPIO,

/* Wakeup caused by UART (light sleep only) */

  ESP_SLEEP_WAKEUP_UART,
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name:  esp32_sleep_enable_timer_wakeup
 *
 * Description:
 *   Configure wake-up interval
 *
 * Input Parameters:
 *   time_in_us - Configure wake-up time interval
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void esp32_sleep_enable_timer_wakeup(uint64_t time_in_us);

/****************************************************************************
 * Name:  esp32_light_sleep_start
 *
 * Description:
 *   Enter sleep mode
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   0 is returned on success or a negated errno value is returned
 *
 ****************************************************************************/

int esp32_light_sleep_start(void);

/****************************************************************************
 * Name: esp32_pminit
 *
 * Description:
 *   Initialize force sleep parameters.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void esp32_pminit(void);

/****************************************************************************
 * Name: esp32_pmstandby
 *
 * Description:
 *   Enter force sleep time interval.
 *
 * Input Parameters:
 *   time_in_us - force sleep time interval
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void esp32_pmstandby(uint64_t time_in_us);

/****************************************************************************
 * Name: esp32_sleep_get_wakeup_cause
 *
 * Description:
 *   Get the wakeup source which caused wakeup from sleep.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   enum esp32_sleep_source_e - Cause of wake up from last sleep.
 *
 ****************************************************************************/

enum esp32_sleep_source_e esp32_sleep_get_wakeup_cause(void);

/****************************************************************************
 * Name:  esp32_deep_sleep_start
 *
 * Description:
 *   Enter deep sleep mode
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void esp32_deep_sleep_start(void);

/****************************************************************************
 * Name: esp32_pmsleep
 *
 * Description:
 *   Enter deep sleep.
 *
 * Input Parameters:
 *   time_in_us - deep sleep time interval
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void esp32_pmsleep(uint64_t time_in_us);

#endif /* CONFIG_PM */

#ifdef __cplusplus
}
#endif
#undef EXTERN

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_XTENSA_SRC_ESP32_ESP32_PMSLEEP_H */
