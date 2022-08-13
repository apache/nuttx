/****************************************************************************
 * boards/arm/stm32l4/steval-stlcs01v1/src/steval-stlcs01v1.h
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

#ifndef __BOARDS_ARM_STM32L4_STEVAL_STLCS01V1_SRC_STEVAL_STLCS01V1_H
#define __BOARDS_ARM_STM32L4_STEVAL_STLCS01V1_SRC_STEVAL_STLCS01V1_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <stdint.h>

#include "stm32l4_gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* LEDs */

#define GPIO_LD1 (GPIO_PORTG | GPIO_PIN12 | GPIO_OUTPUT_CLEAR | \
                  GPIO_OUTPUT | GPIO_SPEED_50MHz)

/* SPI chip selects */

#define GPIO_LSM6DS3H_CS    (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz |   \
                             GPIO_OUTPUT_SET | GPIO_PORTB | GPIO_PIN12)
#define GPIO_LPS22HB_CS     (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz |  \
                             GPIO_OUTPUT_SET | GPIO_PORTA | GPIO_PIN3)
#define GPIO_LSM303AGR_M_CS (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz |  \
                                GPIO_OUTPUT_SET | GPIO_PORTB | GPIO_PIN1)
#define GPIO_LSM303AGR_A_CS (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz |  \
                             GPIO_OUTPUT_SET | GPIO_PORTC | GPIO_PIN4)
#define GPIO_BLUE_CS        (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz |  \
                             GPIO_OUTPUT_SET | GPIO_PORTB | GPIO_PIN2)

/* IRQs */

#define GPIO_BLUE_IRQ        (GPIO_INPUT | GPIO_FLOAT | GPIO_EXTI |         \
                              GPIO_SPEED_50MHz | GPIO_PORTC | GPIO_PIN5)

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Definitions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32l4_bringup
 *
 * Description:
 *   Perform architecture specific initialization
 *
 *   CONFIG_BOARDCTL=y:
 *     If CONFIG_NSH_ARCHINITIALIZE=y:
 *       Called from the NSH library (or other application)
 *     Otherwise, assumed to be called from some other application.
 *
 *   Otherwise CONFIG_BOARD_LATE_INITIALIZE=y:
 *     Called from board_late_initialize().
 *
 *   Otherwise, bad news:  Never called
 *
 ****************************************************************************/

int stm32l4_bringup(void);

/****************************************************************************
 * Name: stm32l4_usbinitialize
 *
 * Description:
 *   Called from stm32_usbinitialize very early in initialization to setup
 *   USB-related GPIO pins for the Olimex-STM32-H405 board.
 *
 ****************************************************************************/

#ifdef CONFIG_STM32L4_OTGFS
void weak_function stm32l4_usbinitialize(void);
#endif

#endif /* __BOARDS_ARM_STM32L4_STEVAL_STLCS01V1_SRC_STEVAL_STLCS01V1_H */
