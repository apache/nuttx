/****************************************************************************
 * boards/arm/stm32u0/stm32u083c-dk/src/stm32u083c-dk.h
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

#ifndef __BOARDS_ARM_STM32U0_STM32U083C_DK_SRC_STM32U083C_DK_H
#define __BOARDS_ARM_STM32U0_STM32U083C_DK_SRC_STM32U083C_DK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

#include "stm32_gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* STM32U083C-DK GPIOs ******************************************************/

/* User LEDs:
 *  LD3 (green) - PC13
 *  LD4 (blue)  - PA5
 *  LD5 (red)   - PB2
 */

#define GPIO_LD3        (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_MEDIUM | \
                         GPIO_OUTPUT_CLEAR | GPIO_PORTC | GPIO_PIN13)
#define GPIO_LD4        (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_MEDIUM | \
                         GPIO_OUTPUT_CLEAR | GPIO_PORTA | GPIO_PIN5)
#define GPIO_LD5        (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_MEDIUM | \
                         GPIO_OUTPUT_CLEAR | GPIO_PORTB | GPIO_PIN2)

#define LED_DRIVER_PATH "/dev/userleds"

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Functions Definitions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_bringup
 *
 * Description:
 *   Perform architecture-specific initialization
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y :
 *     Called from board_late_initialize().
 *
 ****************************************************************************/

int stm32_bringup(void);

/****************************************************************************
 * Name: stm32_adc_setup
 *
 * Description:
 *   Initialize ADC and register the ADC driver.
 *
 ****************************************************************************/

#ifdef CONFIG_ADC
int stm32_adc_setup(void);
#endif

/****************************************************************************
 * Name: stm32_djoy_initialize
 *
 * Description:
 *   Initialize and register the discrete joystick driver
 *
 ****************************************************************************/

#if defined(CONFIG_INPUT_DJOYSTICK) && defined(CONFIG_ADC)
int stm32_djoy_initialize(void);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_STM32U0_STM32U083C_DK_SRC_STM32U083C_DK_H */
