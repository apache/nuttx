/****************************************************************************
 * boards/arm/stm32/stm32vldiscovery/src/stm32vldiscovery.h
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

#ifndef __BOARDS_ARM_STM32_STM32VL_DISCOVERY_SRC_STM32VLDISCOVERY_H
#define __BOARDS_ARM_STM32_STM32VL_DISCOVERY_SRC_STM32VLDISCOVERY_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* LED - assume it is on PC9 */

#define GPIO_LED1       (GPIO_OUTPUT | GPIO_CNF_OUTPP | GPIO_MODE_50MHz | \
                         GPIO_OUTPUT_CLEAR | GPIO_PORTC | GPIO_PIN9)

/* BUTTON - assume it is on PA0 */

#define MIN_IRQBUTTON   BUTTON_0
#define MAX_IRQBUTTON   BUTTON_0
#define NUM_IRQBUTTONS  1

#define GPIO_BTN_0      (GPIO_INPUT | GPIO_CNF_INFLOAT | GPIO_MODE_INPUT | \
                         GPIO_PORTA | GPIO_PIN0)

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
 *   CONFIG_BOARD_LATE_INITIALIZE=y && CONFIG_BOARDCTL=y :
 *     Called from the NSH library
 *
 ****************************************************************************/

int stm32_bringup(void);

/****************************************************************************
 * Name: stm32_led_initialize
 ****************************************************************************/

#ifdef CONFIG_ARCH_LEDS
void stm32_led_initialize(void);
#endif

#endif /* __BOARDS_ARM_STM32_STM32VL_DISCOVERY_SRC_STM32VLDISCOVERY_H */
