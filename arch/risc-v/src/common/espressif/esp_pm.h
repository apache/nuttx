/****************************************************************************
 * arch/risc-v/src/common/espressif/esp_pm.h
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

#ifndef __ARCH_RISC_V_SRC_COMMON_ESPRESSIF_ESP_PM_H
#define __ARCH_RISC_V_SRC_COMMON_ESPRESSIF_ESP_PM_H

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
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: esp_pm_light_sleep_start
 *
 * Description:
 *   Enter light sleep mode
 *
 * Input Parameters:
 *   sleep_time - Reference of uint64_t value to return actual sleep duration
 *                in microseconds. Use NULL if not needed.
 *
 * Returned Value:
 *   OK on success or a negated errno value if fails.
 *
 ****************************************************************************/

int esp_pm_light_sleep_start(uint64_t *sleep_time);

/****************************************************************************
 * Name:  esp_pm_deep_sleep_start
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

void esp_pm_deep_sleep_start(void);

/****************************************************************************
 * Name: esp_pm_pmstandby
 *
 * Description:
 *   Enter pm standby (light sleep) mode.
 *
 * Input Parameters:
 *   time_in_us - The maximum time to sleep in microseconds.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void esp_pmstandby(uint64_t time_in_us);

/****************************************************************************
 * Name: esp_pmsleep
 *
 * Description:
 *   Enter pm sleep (deep sleep) mode.
 *
 * Input Parameters:
 *   time_in_us - The maximum time to sleep in microseconds.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void esp_pmsleep(uint64_t time_in_us);

#endif /* CONFIG_PM */

#ifdef __cplusplus
}
#endif
#undef EXTERN

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_RISC_V_SRC_COMMON_ESPRESSIF_ESP_PM_H */
