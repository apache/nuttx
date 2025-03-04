/****************************************************************************
 * arch/xtensa/src/common/espressif/esp_mcpwm.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifndef __ARCH_XTENSA_SRC_COMMON_ESPRESSIF_ESP_MCPWM_H
#define __ARCH_XTENSA_SRC_COMMON_ESPRESSIF_ESP_MCPWM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#ifdef CONFIG_ESP_MCPWM

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

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
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: esp_motor_bdc_initialize
 *
 * Description:
 *   This function initializes the MCPWM peripheral and configures the
 *   motor control driver.
 *
 * Input Parameters:
 *   channel   - Channel to be initialized (only 0 available for now).
 *   frequency - PWM output frequency in Hertz.
 *   pwm_a_pin - GPIO pin number for PWM0_A output.
 *   pwm_b_pin - GPIO pin number for PWM0_B output.
 *   fault_pin - GPIO pin number for fault signal input.
 *
 * Returned Value:
 *   On success, this function returns a valid pointer to the Capture device
 *   structure. If the initialization fails, it returns NULL.
 *
 ****************************************************************************/

#ifdef CONFIG_ESP_MCPWM_MOTOR_BDC
struct motor_lowerhalf_s *esp_motor_bdc_initialize(int channel,
  uint32_t frequency, int pwm_a_pin, int pwm_b_pin, int fault_pin);
#endif

/****************************************************************************
 * Name: esp_mcpwm_capture_initialize
 *
 * Description:
 *   This function initializes the specified MCPWM peripheral and the capture
 *   submodule with the provided configuration.
 *
 * Input Parameters:
 *   channel - Channel to be initialized [0-3].
 *   pin     - GPIO pin assigned to this channel.
 *
 * Returned Value:
 *   On success, this function returns a valid pointer to the Capture device
 *   structure. If the initialization fails, it returns NULL.
 *
 ****************************************************************************/

#ifdef CONFIG_ESP_MCPWM_CAPTURE
struct cap_lowerhalf_s *esp_mcpwm_capture_initialize(int channel, int pin);
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_ESP_MCPWM */
#endif /* __ARCH_XTENSA_SRC_COMMON_ESPRESSIF_ESP_MCPWM_H */
