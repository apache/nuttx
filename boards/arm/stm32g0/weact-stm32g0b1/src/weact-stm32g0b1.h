/****************************************************************************
 * boards/arm/stm32g0/weact-stm32g0b1/src/weact-stm32g0b1.h
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

#ifndef __BOARDS_ARM_STM32F0L0G0_WEACT_STM32G0B1_SRC_WEACT_STM32G0B1_H
#define __BOARDS_ARM_STM32F0L0G0_WEACT_STM32G0B1_SRC_WEACT_STM32G0B1_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* LEDS
 *
 * The WeAct STM32G0B1 board has only one Blue LED controlled by GPIO.
 * LED: connected to PC6
 */

#define GPIO_LED1      (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_HIGH | \
                        GPIO_OUTPUT_CLEAR | GPIO_PORTC | GPIO_PIN6)

#define LED_DRIVER_PATH "/dev/userleds"

/* BUTTONS
 *
 * The pushbutton is connected to GPIO PC13.
 */

#define MIN_IRQBUTTON  BUTTON_USER
#define MAX_IRQBUTTON  BUTTON_USER
#define NUM_IRQBUTTONS 1

#define GPIO_BTN_USER  (GPIO_INPUT | GPIO_FLOAT | GPIO_EXTI | GPIO_PORTC | GPIO_PIN13)

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
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

#endif /* __BOARDS_ARM_STM32F0L0G0_WEACT_STM32G0B1_SRC_WEACT_STM32G0B1_H */
