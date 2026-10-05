/****************************************************************************
 * boards/arm/stm32h7/stm32h7s8-dk/src/stm32h7s8-dk.h
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

#ifndef __BOARDS_ARM_STM32H7_STM32H7S8_DK_SRC_STM32H7S8_DK_H
#define __BOARDS_ARM_STM32H7_STM32H7S8_DK_SRC_STM32H7S8_DK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include "stm32_gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* LEDS
 *
 * The green LD1, orange LD2, red LD3 LED, and blue LD4 LEDs are connected
 * to PO1, PO5, PM2, PM3 respectively.  A high output illuminates each LED.
 */

#define GPIO_LD1        (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_2MHz | \
                         GPIO_OUTPUT_CLEAR | GPIO_PORTO | GPIO_PIN1)
#define GPIO_LD2        (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_2MHz | \
                         GPIO_OUTPUT_CLEAR | GPIO_PORTO | GPIO_PIN5)
#define GPIO_LD3        (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_2MHz | \
                         GPIO_OUTPUT_CLEAR | GPIO_PORTM | GPIO_PIN2)
#define GPIO_LD4        (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_2MHz | \
                         GPIO_OUTPUT_CLEAR | GPIO_PORTM | GPIO_PIN3)

#define GPIO_LED_GREEN  GPIO_LD1
#define GPIO_LED_ORANGE GPIO_LD2
#define GPIO_LED_RED    GPIO_LD3
#define GPIO_LED_BLUE   GPIO_LD4

/* BUTTONS
 *
 * The blue B2 user button is connected to PC13.  The input is low while the
 * button is pressed.  GPIO_EXTI permits interrupt button support.
 */

#define GPIO_BTN_USER   (GPIO_INPUT | GPIO_PULLDOWN | GPIO_EXTI | \
                         GPIO_PORTC | GPIO_PIN13)

/* procfs file system */

#ifdef CONFIG_FS_PROCFS
#  ifdef CONFIG_NSH_PROC_MOUNTPOINT
#    define STM32_PROCFS_MOUNTPOINT CONFIG_NSH_PROC_MOUNTPOINT
#  else
#    define STM32_PROCFS_MOUNTPOINT "/proc"
#  endif
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_bringup
 *
 * Description:
 *   Perform architecture-specific initialization.
 *
 ****************************************************************************/

int stm32_bringup(void);

#endif /* __BOARDS_ARM_STM32H7_STM32H7S8_DK_SRC_STM32H7S8_DK_H */
