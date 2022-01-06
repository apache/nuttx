/****************************************************************************
 * boards/arm/stm32f0l0g0/stm32f072-discovery/src/stm32f072-discovery.h
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

#ifndef __BOARDS_ARM_STM32F0L0G0_STM32F072_DISCOVERY_SRC_STM32F072_DISCOVERY_H
#define __BOARDS_ARM_STM32F0L0G0_STM32F072_DISCOVERY_SRC_STM32F072_DISCOVERY_H

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

/* Configuration ************************************************************/

/* How many SPI modules does this chip support? */

#if STM32_NSPI < 1
#  undef CONFIG_STM32F0L0G0_SPI1
#  undef CONFIG_STM32F0L0G0_SPI2
#  undef CONFIG_STM32F0L0G0_SPI3
#elif STM32_NSPI < 2
#  undef CONFIG_STM32F0L0G0_SPI2
#  undef CONFIG_STM32F0L0G0_SPI3
#elif STM32_NSPI < 3
#  undef CONFIG_STM32F0L0G0_SPI3
#endif

/* STM32F0Discovery GPIOs ***************************************************/

/* The STM32F0Discovery board has four LEDs.
 * Two of these are controlled by logic on the board and
 * are not available for software control:
 *
 * LD1 COM:   LD2 default status is red. LD2 turns to green to indicate that
 *            communications are in progress between the PC and
 *            the ST-LINK/V2.
 * LD2 PWR:   Red LED indicates that the board is powered.
 *
 * And two LEDs can be controlled by software:
 *
 * User LEDs connected to the STM32F072 MCU:
 * User LD_U:  Green LED is connected to PC6 I/O
 * User LD_D:  Blue LED  is connected to PC7 I/O
 * User LD_L:  Green LED is connected to PC8 I/O
 * User LD_R:  Blue LED  is connected to PC9 I/O
 *
 * The other side of the LED connects to ground so high value will illuminate
 * the LED.
 */

#define GPIO_LED1       (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_MEDIUM | \
                         GPIO_OUTPUT_CLEAR | GPIO_PORTC | GPIO_PIN6)
#define GPIO_LED2       (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_MEDIUM | \
                         GPIO_OUTPUT_CLEAR | GPIO_PORTC | GPIO_PIN7)
#define GPIO_LED3       (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_MEDIUM | \
                         GPIO_OUTPUT_CLEAR | GPIO_PORTC | GPIO_PIN8)
#define GPIO_LED4       (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_MEDIUM | \
                         GPIO_OUTPUT_CLEAR | GPIO_PORTC | GPIO_PIN9)

/* Button definitions *******************************************************/

/* The STM32F0Discovery supports two buttons;
 * only one button is controllable by software:
 *
 *   B1 USER: user and wake-up button connected to thePA0 I/O.
 *   B2 RESET: pushbutton connected to NRST is used to RESET
 *             the STM32F303VCT6.
 *
 * NOTE that  EXTI interrupts are configured
 */

#define MIN_IRQBUTTON   BUTTON_USER
#define MAX_IRQBUTTON   BUTTON_USER
#define NUM_IRQBUTTONS  1

#define GPIO_BTN_USER   (GPIO_INPUT | GPIO_FLOAT | GPIO_EXTI | GPIO_PORTA | GPIO_PIN0)

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
 *   CONFIG_BOARD_LATE_INITIALIZE=n && CONFIG_BOARDCTL=y :
 *     Called from the NSH library
 *
 ****************************************************************************/

int stm32_bringup(void);

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_STM32F0L0G0_STM32F072_DISCOVERY_SRC_STM32F072_DISCOVERY_H */
