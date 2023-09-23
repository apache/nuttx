/****************************************************************************
 * boards/arm/nrf91/thingy91/include/board.h
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

#ifndef __BOARDS_ARM_NRF91_THINGY91_INCLUDE_BOARD_H
#define __BOARDS_ARM_NRF91_THINGY91_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdbool.h>

#if defined(CONFIG_ARCH_IRQBUTTONS) && defined(CONFIG_NRF91_GPIOTE)
#  include <nuttx/irq.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

#define BOARD_SYSTICK_CLOCK         (64000000)

/* Button definitions *******************************************************/

/* Board supports 2 buttons. */

#define BUTTON_BTN1       0
#define NUM_BUTTONS       1

#define BUTTON_BTN1_BIT  (1 << BUTTON_BTN1)

/* UART Pins ****************************************************************/

/* UART0 (TWI0) - connected to nRF52840:
 *   UART0_RX - P0-18 (MCU_IF0)
 *   UART0_TX - P0-19 (MCU_IF1)
 */

#define BOARD_UART0_RX_PIN  (GPIO_INPUT  | GPIO_PORT0 | GPIO_PIN(18))
#define BOARD_UART0_TX_PIN  (GPIO_OUTPUT | GPIO_VALUE_ONE | GPIO_PORT0 | GPIO_PIN(19))

/* SPI Pins *****************************************************************/

/* SPI1 (TWI1)
 *   SPI1_SCK  - P0.03
 *   SPI1_MOSI - P0.04
 *   SPI1_MISO - P0.05
 */

#define BOARD_SPI1_SCK_PIN  (GPIO_OUTPUT | GPIO_VALUE_ONE | GPIO_PORT0 | GPIO_PIN(3))
#define BOARD_SPI1_MOSI_PIN (GPIO_OUTPUT | GPIO_PORT0 | GPIO_PIN(4))
#define BOARD_SPI1_MISO_PIN (GPIO_INPUT  | GPIO_PORT0 | GPIO_PIN(5))

/* I2C Pins *****************************************************************/

/* I2C2 (TWI2)
 *   I2C2_SCL - P0.12
 *   I2C2_SDA - P0.11
 */

#define BOARD_I2C2_SCL_PIN (GPIO_OUTPUT | GPIO_PORT0 | GPIO_PIN(12))
#define BOARD_I2C2_SDA_PIN (GPIO_INPUT  | GPIO_PORT0 | GPIO_PIN(11))

#endif /* __BOARDS_ARM_NRF91_THINGY91_INCLUDE_BOARD_H */
