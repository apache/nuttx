/****************************************************************************
 * boards/arm/stm32f0l0g0/nucleo-f091rc/src/nucleo-f091rc.h
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

#ifndef __BOARDS_ARM_STM32F0L0G0_NUCLEO_F091RC_SRC_NUCLEO_F091RC_H
#define __BOARDS_ARM_STM32F0L0G0_NUCLEO_F091RC_SRC_NUCLEO_F091RC_H

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

/* Nucleo-F091RC GPIOs ******************************************************/

/* LED.  User LD2: the green LED is a user LED connected to Arduino signal
 * D13 corresponding to MCU I/O PA5 (pin 21) or PB13 (pin 34) depending on
 * the STM32 target.
 *
 * - When the I/O is HIGH value, the LED is on.
 * - When the I/O is LOW, the LED is off.
 */

#define GPIO_LD2        (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_MEDIUM | \
                         GPIO_OUTPUT_CLEAR | GPIO_PORTA | GPIO_PIN5)

/* Button definitions *******************************************************/

/* B1 USER: the user button is connected to the I/O PC13 (pin 2) of the STM32
 * microcontroller.
 */

#define MIN_IRQBUTTON   BUTTON_USER
#define MAX_IRQBUTTON   BUTTON_USER
#define NUM_IRQBUTTONS  1

#define GPIO_BTN_USER   (GPIO_INPUT | GPIO_FLOAT | GPIO_EXTI | \
                         GPIO_PORTC | GPIO_PIN13)

/* Dragino LORA shield (v1.4) - RF98 module (based on SX127X)
 * RESET - PC7  (D9)
 * CS    - PB6  (D10)
 * DIO0  - PA10 (D2)
 */

#define GPIO_SX127X_RESET (GPIO_PORTC | GPIO_PIN7)
#define GPIO_SX127X_CS    (GPIO_OUTPUT | GPIO_SPEED_HIGH |        \
                           GPIO_OUTPUT_SET | GPIO_PORTB | GPIO_PIN6)
#define GPIO_SX127X_DIO0  (GPIO_INPUT | GPIO_FLOAT | GPIO_EXTI |  \
                           GPIO_PORTA | GPIO_PIN10)

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

/****************************************************************************
 * Name: stm32_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the Nucleo-H743ZI
 *   board.
 *
 ****************************************************************************/

#ifdef CONFIG_STM32F0L0G0_SPI
void stm32_spidev_initialize(void);
#endif

/****************************************************************************
 * Name: stm32_lpwaninitialize
 *
 * Description:
 *   Initialize SX127X LPWAN interaface.
 ****************************************************************************/

#ifdef CONFIG_LPWAN_SX127X
int stm32_lpwaninitialize(void);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_STM32F0L0G0_NUCLEO_F091RC_SRC_NUCLEO_F091RC_H */
