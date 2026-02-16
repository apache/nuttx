/****************************************************************************
 * arch/risc-v/src/common/espressif/esp_rtc_gpio.h
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

#ifndef __ARCH_RISC_V_SRC_COMMON_ESPRESSIF_ESP_RTC_GPIO_H
#define __ARCH_RISC_V_SRC_COMMON_ESPRESSIF_ESP_RTC_GPIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef enum esp_rtc_gpio_mode_e
{
  ESP_RTC_GPIO_MODE_INPUT,
  ESP_RTC_GPIO_MODE_OUTPUT,
  ESP_RTC_GPIO_MODE_INPUT_OUTPUT,
  ESP_RTC_GPIO_MODE_DISABLED,
  ESP_RTC_GPIO_MODE_OUTPUT_OD,
  ESP_RTC_GPIO_MODE_INPUT_OUTPUT_OD
} esp_rtc_gpio_mode_t;

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: esp_rtcioirqinitialize
 *
 * Description:
 *   Initialize logic to support a second level of interrupt decoding for
 *   RTC IRQs.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#ifdef CONFIG_ESPRESSIF_RTCIO_IRQ
void esp_rtcioirqinitialize(void);
#else
#  define esp_rtcioirqinitialize()
#endif

/****************************************************************************
 * Name: esp_rtcioirqenable
 *
 * Description:
 *   Enable the interrupt for the specified RTC peripheral IRQ.
 *
 * Input Parameters:
 *   irq - The IRQ number.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#ifdef CONFIG_ESPRESSIF_RTCIO_IRQ
void esp_rtcioirqenable(int irq);
#else
#  define esp_rtcioirqenable(irq)
#endif

/****************************************************************************
 * Name: esp_rtcioirqdisable
 *
 * Description:
 *   Disable the interrupt for the specified RTC peripheral IRQ.
 *
 * Input Parameters:
 *   irq - The IRQ number.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#ifdef CONFIG_ESPRESSIF_RTCIO_IRQ
void esp_rtcioirqdisable(int irq);
#else
#  define esp_rtcioirqdisable(irq)
#endif

#ifdef CONFIG_ARCH_CHIP_ESP32C6
/****************************************************************************
 * Name: esp_rtcio_config_gpio
 *
 * Description:
 *   Configure a RTC GPIO pin based on encoded pin attributes
 *
 * Input Parameters:
 *   pin  - RTC GPIO pin to be configured.
 *   mode - Attributes to be configured for the selected RTC GPIO pin.
 *
 * Returned Value:
 *   Zero (OK) on success, or -1 (ERROR) in case of failure.
 *
 ****************************************************************************/

int esp_rtcio_config_gpio(int pin, enum esp_rtc_gpio_mode_e mode);

/****************************************************************************
 * Name: esp_rtcio_read
 *
 * Description:
 *   Read one or zero from the selected RTC GPIO pin
 *
 * Input Parameters:
 *   pin - RTC GPIO pin to be read.
 *
 * Returned Value:
 *   The boolean representation of the input value (true/false).
 *
 ****************************************************************************/

int esp_rtcio_read(int pin);

/****************************************************************************
 * Name: esp_rtcio_write
 *
 * Description:
 *   Write one or zero to the selected RTC GPIO pin
 *
 * Input Parameters:
 *   pin   - GPIO pin to be modified.
 *   value - The value to be written (0 or 1).
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void esp_rtcio_write(int pin, bool value);
#endif /* CONFIG_ARCH_CHIP_ESP32C6 */

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_RISC_V_SRC_COMMON_ESPRESSIF_ESP_RTC_GPIO_H */
