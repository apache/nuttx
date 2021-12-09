/****************************************************************************
 * arch/risc-v/src/esp32c3/esp32c3_pm.h
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

#ifndef __ARCH_RISCV_SRC_ESP32C3_ESP32C3_PM_H
#define __ARCH_RISCV_SRC_ESP32C3_ESP32C3_PM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <stdbool.h>

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

/* Callback function type for peripherals to
 * know light sleep wakeup overhead.
 */

typedef void (*inform_out_sleep_overhead_cb_t)(uint32_t);

/* Callback function type for peripherals to skip light sleep. */

typedef bool (*skip_light_sleep_cb_t)(void);

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name:  esp32c3_sleep_enable_rtc_timer_wakeup
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

void esp32c3_sleep_enable_rtc_timer_wakeup(uint64_t time_in_us);

/****************************************************************************
 * Name:  esp32c3_light_sleep_start
 *
 * Description:
 *   Enter light sleep mode
 *
 * Input Parameters:
 *   sleep_time - Actual sleep time
 *
 * Returned Value:
 *   0 is returned on success or a negated errno value is returned
 *
 ****************************************************************************/

int esp32c3_light_sleep_start(uint64_t *sleep_time);

/****************************************************************************
 * Name: esp32c3_pmstandby
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

void esp32c3_pmstandby(uint64_t time_in_us);

/****************************************************************************
 * Name:  esp32c3_deep_sleep_start
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

void esp32c3_deep_sleep_start(void);

/****************************************************************************
 * Name: esp32c3_pmsleep
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

void esp32c3_pmsleep(uint64_t time_in_us);

/****************************************************************************
 * Name: esp32c3_pm_lockacquire
 *
 * Description:
 *   Take a power management lock
 *
 ****************************************************************************/

void esp32c3_pm_lockacquire(void);

/****************************************************************************
 * Name: esp32c3_pm_lockrelease
 *
 * Description:
 *   Release the lock taken using esp32c3_pm_lockacquire.
 *
 ****************************************************************************/

void esp32c3_pm_lockrelease(void);

/****************************************************************************
 * Name: esp32c3_pm_lockstatus
 *
 * Description:
 *   Return power management lock status.
 *
 ****************************************************************************/

uint32_t esp32c3_pm_lockstatus(void);

/****************************************************************************
 * Name:  esp32c3_sleep_enable_wifi_wakeup
 *
 * Description:
 *   Configure Wi-Fi wake-up source
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void esp32c3_sleep_enable_wifi_wakeup(void);

/****************************************************************************
 * Name:  esp32c3_should_skip_light_sleep
 *
 * Description:
 *   Indicates if light sleep shoule be skipped.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   True is returned on success.  Otherwise false.
 *
 ****************************************************************************/

bool esp32c3_should_skip_light_sleep(void);

/****************************************************************************
 * Name:  esp32c3_pm_register_inform_out_sleep_overhead_callback
 *
 * Description:
 *   Register informing peripherals of light sleep wakeup overhead time
 *   callback function.
 *
 * Input Parameters:
 *   cb - callback function
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  Otherwise -1 (ERROR).
 *
 ****************************************************************************/

int esp32c3_pm_register_inform_out_sleep_overhead_callback(
                            inform_out_sleep_overhead_cb_t cb);

/****************************************************************************
 * Name:  esp32c3_pm_unregister_inform_out_sleep_overhead_callback
 *
 * Description:
 *   Unregister informing peripherals of light sleep wakeup overhead time
 *   callback function.
 *
 * Input Parameters:
 *   cb - callback function
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

int esp32c3_pm_unregister_inform_out_sleep_overhead_callback(
                              inform_out_sleep_overhead_cb_t cb);

/****************************************************************************
 * Name:  esp32c3_pm_register_skip_sleep_callback
 *
 * Description:
 *   Unregister callback function of skipping light sleep.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  Otherwise -1 (ERROR).
 *
 ****************************************************************************/

int esp32c3_pm_register_skip_sleep_callback(skip_light_sleep_cb_t cb);

/****************************************************************************
 * Name:  esp32c3_pm_unregister_skip_sleep_callback
 *
 * Description:
 *   Register callback function of skipping light sleep.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  Otherwise -1 (ERROR).
 *
 ****************************************************************************/

int esp32c3_pm_unregister_skip_sleep_callback(skip_light_sleep_cb_t cb);

#endif /* CONFIG_PM */

#ifdef __cplusplus
}
#endif
#undef EXTERN

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_RISCV_SRC_ESP32C3_ESP32C3_PM_H */
