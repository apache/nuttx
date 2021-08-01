/****************************************************************************
 * boards/arm/stm32l5/stm32l562e-dk/src/stm32l562e-dk.h
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

#ifndef __BOARDS_ARM_STM32L5_STM32L562E_DK_SRC_STM32L562E_DK_H
#define __BOARDS_ARM_STM32L5_STM32L562E_DK_SRC_STM32L562E_DK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

#include "stm32l5_gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#define HAVE_PROC             1
#define HAVE_RTC_DRIVER       1

#if !defined(CONFIG_FS_PROCFS)
#  undef HAVE_PROC
#endif

#if defined(HAVE_PROC) && defined(CONFIG_DISABLE_MOUNTPOINT)
#  warning Mountpoints disabled.  No procfs support
#  undef HAVE_PROC
#endif

/* Check if we can support the RTC driver */

#if !defined(CONFIG_RTC) || !defined(CONFIG_RTC_DRIVER)
#  undef HAVE_RTC_DRIVER
#endif

/* STM32L562E-DK GPIOs ******************************************************/

/* LED  I/O  Color
 * LD9  PD3  Red
 * LD10 PG12 Green
 *
 * - When the I/O is LOW, the LED is on.
 * - When the I/O is HIGH value, the LED is off
 */

#define GPIO_LD9        (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_2MHZ | \
                         GPIO_OUTPUT_SET | GPIO_PORTD | GPIO_PIN3)
#define GPIO_LD10       (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_2MHZ | \
                         GPIO_OUTPUT_SET | GPIO_PORTG | GPIO_PIN12)

/* Button definitions *******************************************************/

/* B1 USER: the user button is connected to the I/O PC13 (pin 2) of the STM32
 * microcontroller.
 */

#define MIN_IRQBUTTON   BUTTON_USER
#define MAX_IRQBUTTON   BUTTON_USER
#define NUM_IRQBUTTONS  1

#define GPIO_BTN_USER   (GPIO_INPUT | GPIO_FLOAT | GPIO_EXTI | \
                         GPIO_PORTC | GPIO_PIN13)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

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
 *   CONFIG_BOARD_LATE_INITIALIZE=n && CONFIG_BOARDCTL=y :
 *     Called from the NSH library
 *
 ****************************************************************************/

int stm32_bringup(void);

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_STM32L5_STM32L562E_DK_SRC_STM32L562E_DK_H */
