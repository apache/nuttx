/****************************************************************************
 * boards/arm/efr32/xg24-ek2703a/src/xg24-ek2703a.h
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

#ifndef __BOARDS_ARM_EFR32_XG24_EK2703A_SRC_XG24_EK2703A_H
#define __BOARDS_ARM_EFR32_XG24_EK2703A_SRC_XG24_EK2703A_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include "efr32_gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* LEDs: LED0 on PA4, LED1 on PA7, active high */

#define GPIO_LED0    (GPIO_PORTA | GPIO_PIN(4) | GPIO_OUTPUT | \
                      GPIO_OUTPUT_CLEAR)
#define GPIO_LED1    (GPIO_PORTA | GPIO_PIN(7) | GPIO_OUTPUT | \
                      GPIO_OUTPUT_CLEAR)

/* Buttons: BTN0 on PB2, BTN1 on PB3, low when pressed and debounced by
 * an RC filter on the board.  The internal pull-up keeps them high when
 * released, and the glitch filter is on as the manual advises for pins
 * used as interrupts.  Both edges interrupt (EXTI2 and EXTI3).
 */

#define GPIO_BTN0    (GPIO_PORTB | GPIO_PIN(2) | GPIO_INPUT_PULLFILTER | \
                      GPIO_PULLUP | GPIO_INT_BOTH)
#define GPIO_BTN1    (GPIO_PORTB | GPIO_PIN(3) | GPIO_INPUT_PULLFILTER | \
                      GPIO_PULLUP | GPIO_INT_BOTH)

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Name: efr32_bringup
 *
 * Description:
 *   Perform architecture-specific initialization
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y :
 *     Called from board_late_initialize().
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=n && CONFIG_BOARDCTL=y :
 *     Called from the NSH library via board_app_initialize()
 *
 ****************************************************************************/

int efr32_bringup(void);

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_EFR32_XG24_EK2703A_SRC_XG24_EK2703A_H */
