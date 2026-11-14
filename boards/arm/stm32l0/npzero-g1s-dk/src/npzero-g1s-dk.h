/****************************************************************************
 * boards/arm/stm32l0/npzero-g1s-dk/src/npzero-g1s-dk.h
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

#ifndef __BOARDS_ARM_STM32L0_NPZERO_G1S_DK_SRC_NPZERO_G1S_DK_H
#define __BOARDS_ARM_STM32L0_NPZERO_G1S_DK_SRC_NPZERO_G1S_DK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* LED definitions **********************************************************/

/* The STM32 MCU host has one LED connected to PA12, lit when PA12 is high */

#define GPIO_LED1      (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_LOW | \
                        GPIO_OUTPUT_CLEAR | GPIO_PORTA | GPIO_PIN12)

/* nPZero G1S ***************************************************************/

/* The nPZero G1S is on I2C1 */

#define NPZERO_I2C_BUS  1
#define NPZERO_DEVPATH  "/dev/npzero0"

/* I2C1 pins as open-drain GPIOs, used to power the PMOD peripherals before
 * the I2C driver is initialized (see stm32_bringup.c).
 */

#define GPIO_NPZERO_SCL (GPIO_OUTPUT | GPIO_OPENDRAIN | GPIO_PULLUP | \
                         GPIO_SPEED_LOW | GPIO_OUTPUT_SET | GPIO_PORTB | \
                         GPIO_PIN6)
#define GPIO_NPZERO_SDA (GPIO_OUTPUT | GPIO_OPENDRAIN | GPIO_PULLUP | \
                         GPIO_SPEED_LOW | GPIO_OUTPUT_SET | GPIO_PORTB | \
                         GPIO_PIN7)

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

#endif /* __BOARDS_ARM_STM32L0_NPZERO_G1S_DK_SRC_NPZERO_G1S_DK_H */
