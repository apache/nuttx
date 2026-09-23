/****************************************************************************
 * boards/arm/stm32h7/stm32h735g-dk/src/stm32h735g-dk.h
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

#ifndef __BOARDS_ARM_STM32H7_STM32H735G_DK_SRC_STM32H735G_DK_H
#define __BOARDS_ARM_STM32H7_STM32H735G_DK_SRC_STM32H735G_DK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "stm32_gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define GPIO_LED_GREEN (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_2MHz | \
                        GPIO_OUTPUT_SET | GPIO_PORTC | GPIO_PIN3)
#define GPIO_LED_RED   (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_2MHz | \
                        GPIO_OUTPUT_SET | GPIO_PORTC | GPIO_PIN2)
#define GPIO_BTN_USER  (GPIO_INPUT | GPIO_FLOAT | GPIO_EXTI | \
                        GPIO_PORTC | GPIO_PIN13)

/* Display control pins */

#define GPIO_LCD_DISP    (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_2MHz | \
                          GPIO_OUTPUT_CLEAR | GPIO_PORTD | GPIO_PIN10)
#define GPIO_LCD_BL      (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_2MHz | \
                          GPIO_OUTPUT_CLEAR | GPIO_PORTG | GPIO_PIN15)
#define GPIO_TOUCH_INT   (GPIO_INPUT | GPIO_FLOAT | GPIO_EXTI | \
                          GPIO_PORTG | GPIO_PIN2)

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_STM32_LTDC
int stm32_hyperram_initialize(void);
#endif

int stm32_bringup(void);

#ifdef CONFIG_INPUT_GT9XX
int stm32_tsc_setup(int minor);
#endif

#endif /* __BOARDS_ARM_STM32H7_STM32H735G_DK_SRC_STM32H735G_DK_H */
