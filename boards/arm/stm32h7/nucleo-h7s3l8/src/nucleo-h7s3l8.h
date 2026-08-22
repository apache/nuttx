/****************************************************************************
 * boards/arm/stm32h7/nucleo-h7s3l8/src/nucleo-h7s3l8.h
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

#ifndef __BOARDS_ARM_STM32H7_NUCLEO_H7S3L8_SRC_NUCLEO_H7S3L8_H
#define __BOARDS_ARM_STM32H7_NUCLEO_H7S3L8_SRC_NUCLEO_H7S3L8_H

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
 * The green LD1, yellow LD2 and red LD3 LEDs are connected to PD10, PD13 and
 * PB7 respectively.  A high output illuminates each LED.
 */

#define GPIO_LD1        (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_2MHz | \
                         GPIO_OUTPUT_CLEAR | GPIO_PORTD | GPIO_PIN10)
#define GPIO_LD2        (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_2MHz | \
                         GPIO_OUTPUT_CLEAR | GPIO_PORTD | GPIO_PIN13)
#define GPIO_LD3        (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_2MHz | \
                         GPIO_OUTPUT_CLEAR | GPIO_PORTB | GPIO_PIN7)

#define GPIO_LED_GREEN  GPIO_LD1
#define GPIO_LED_YELLOW GPIO_LD2
#define GPIO_LED_RED    GPIO_LD3

/* BUTTONS
 *
 * The blue B2 user button is connected to PC13.  The input is low while the
 * button is pressed.  GPIO_EXTI permits interrupt button support.
 */

#define GPIO_BTN_USER   (GPIO_INPUT | GPIO_PULLUP | GPIO_EXTI | \
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

/****************************************************************************
 * Name: stm32_xspi_flash_initialize
 *
 * Description:
 *   Initialize the external MX25UW25645G flash for memory-mapped access.
 *
 ****************************************************************************/

#ifdef CONFIG_NUCLEO_H7S3L8_XIP_BOOTLOADER
int stm32_xspi_flash_initialize(void);
int stm32_xip_boot(int argc, char *argv[]);
#endif

#endif /* __BOARDS_ARM_STM32H7_NUCLEO_H7S3L8_SRC_NUCLEO_H7S3L8_H */
