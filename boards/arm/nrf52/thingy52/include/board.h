/****************************************************************************
 * boards/arm/nrf52/thingy52/include/board.h
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

#ifndef __BOARDS_ARM_NRF52_THINGY52_INCLUDE_BOARD_H
#define __BOARDS_ARM_NRF52_THINGY52_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdbool.h>

#if defined(CONFIG_ARCH_IRQBUTTONS) && defined(CONFIG_NRF52_GPIOTE)
#  include <nuttx/irq.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

#define BOARD_SYSTICK_CLOCK         (64000000)

/* Button definitions *******************************************************/

/* Board supports four buttons. */

#define BUTTON_BTN1       0
#define NUM_BUTTONS       1

#define BUTTON_BTN1_BIT  (1 << BUTTON_BTN1)

/* UART Pins ****************************************************************/

/* UART0:
 *   RX - P0.02
 *   TX - P0.03
 */

#define BOARD_UART0_RX_PIN  (GPIO_INPUT  | GPIO_PORT0 | GPIO_PIN(2))
#define BOARD_UART0_TX_PIN  (GPIO_OUTPUT | GPIO_VALUE_ONE | GPIO_PORT0 | GPIO_PIN(3))

/* I2C Pins *****************************************************************/

/* I2C0 (TWI0)
 *   I2C0_SCL - P0.07
 *   I2C0_SDA - P0.08
 */

#define BOARD_I2C0_SCL_PIN (GPIO_OUTPUT | GPIO_PORT0 | GPIO_PIN(7))
#define BOARD_I2C0_SDA_PIN (GPIO_INPUT  | GPIO_PORT0 | GPIO_PIN(8))

#endif /* __BOARDS_ARM_NRF52_THINGY52_INCLUDE_BOARD_H */
