/****************************************************************************
 * boards/arm/stm32h7/devebox-stm32h743/src/stm32_gpio_helper.h
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

#ifndef __STM32_GPIO_HELPER_H
#define __STM32_GPIO_HELPER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <stdbool.h>

/* Pin modes */

enum gpio_helper_mode_e
{
  GPIO_HELPER_OUTPUT = 0,
  GPIO_HELPER_INPUT,
  GPIO_HELPER_INTERRUPT_RISING,
  GPIO_HELPER_INTERRUPT_FALLING,
  GPIO_HELPER_INTERRUPT_BOTH,
  GPIO_HELPER_INTERRUPT_HIGH,
  GPIO_HELPER_INTERRUPT_LOW,
};

/* Pull configuration */

enum gpio_helper_pull_e
{
  GPIO_HELPER_PULL_NONE = 0,
  GPIO_HELPER_PULL_UP,
  GPIO_HELPER_PULL_DOWN,
};

/* Register a GPIO pin as output */

int gpio_helper_reg_output(uint32_t pin, const char *devpath);

/* Register a GPIO pin as input (with optional pull) */

int gpio_helper_reg_input(uint32_t pin, const char *devpath,
                          enum gpio_helper_pull_e pull);

/* Register a GPIO pin as interrupt input (with optional pull and
 * trigger mode)
 */

int gpio_helper_reg_input_interrupt(uint32_t pin, const char *devpath,
                                   enum gpio_helper_pull_e pull,
                                   enum gpio_helper_mode_e mode);

#endif /* __STM32_GPIO_HELPER_H */