/****************************************************************************
 * boards/arm/nrf53/thingy53/include/board.h
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

#ifndef __BOARDS_ARM_NRF53_THINGY53_INCLUDE_BOARD_H
#define __BOARDS_ARM_NRF53_THINGY53_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdbool.h>

#if defined(CONFIG_ARCH_IRQBUTTONS) && defined(CONFIG_NRF53_GPIOTE)
#  include <nuttx/irq.h>
#endif

#include "hardware/nrf53_osc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

#define BOARD_SYSTICK_CLOCK         (64000000)
#define BOARD_OSC_XOSC32KI_INTCAP   (OSC_XOSC32KI_INTCAP_C7PF)

/* Button definitions *******************************************************/

/* Board supports four buttons. */

#define BUTTON_BTN1       0
#define BUTTON_BTN2       1

#define BUTTON_BTN1_BIT  (1 << BUTTON_BTN1)
#define BUTTON_BTN2_BIT  (1 << BUTTON_BTN2)

/* UART Pins ****************************************************************/

/* UART0 APP:
 *   UART0_RX - P0-12
 *   UART0_TX - P0-11
 *
 * UART0 NET:
 *   UART0_RX - P0-10
 *   UART0_TX - P0-09
 */

#ifdef CONFIG_ARCH_CHIP_NRF5340_CPUAPP
#  define BOARD_UART0_RX_PIN  (GPIO_MCUSEL_APP | GPIO_INPUT  | \
                               GPIO_PORT0 | GPIO_PIN(12))
#  define BOARD_UART0_TX_PIN  (GPIO_MCUSEL_APP | GPIO_OUTPUT | \
                               GPIO_VALUE_ONE | GPIO_PORT0 | GPIO_PIN(11))

/* Declarations for the Net core */

#  define BOARD_NET_UART0_RX_PIN  (GPIO_MCUSEL_NET | GPIO_INPUT  |  \
                                   GPIO_PORT0 | GPIO_PIN(10))
#  define BOARD_NET_UART0_TX_PIN  (GPIO_MCUSEL_NET | GPIO_OUTPUT |      \
                                   GPIO_VALUE_ONE | GPIO_PORT0 | GPIO_PIN(9))
#endif

#ifdef CONFIG_ARCH_CHIP_NRF5340_CPUNET
#  define BOARD_UART0_RX_PIN  (GPIO_MCUSEL_NET | GPIO_INPUT  | \
                               GPIO_PORT0 | GPIO_PIN(10))
#  define BOARD_UART0_TX_PIN  (GPIO_MCUSEL_NET | GPIO_OUTPUT | \
                               GPIO_VALUE_ONE | GPIO_PORT0 | GPIO_PIN(9))
#endif

/* SPI Pins *****************************************************************/

/* SPI0
 *   SPI0_SCK  - P0.29
 *   SPI0_MOSI - P0.28
 *   SPI0_MISO - P0.26
 */

#define BOARD_SPI0_SCK_PIN  (GPIO_OUTPUT | GPIO_VALUE_ONE | GPIO_PORT0 | GPIO_PIN(29))
#define BOARD_SPI0_MOSI_PIN (GPIO_OUTPUT | GPIO_PORT0 | GPIO_PIN(28))
#define BOARD_SPI0_MISO_PIN (GPIO_INPUT  | GPIO_PORT0 | GPIO_PIN(26))

/* I2C Pins *****************************************************************/

/* I2C0 (TWI0)
 *    I2C0_SCL - P1.02
 *    I2C0_SDA - P1.03
 */

#define BOARD_I2C0_SCL_PIN (GPIO_OUTPUT | GPIO_PORT1 | GPIO_PIN(2))
#define BOARD_I2C0_SDA_PIN (GPIO_INPUT  | GPIO_PORT1 | GPIO_PIN(3))

/* ADC Pins *****************************************************************/

/* ADC
 *   ADC CH0 - P0.06 - AIN2 - BAT_MEAS
 */

#define NRF52_ADC_CH0_PIN (GPIO_INPUT | GPIO_PORT0 | GPIO_PIN(6))

/* QSPI Pins ****************************************************************/

/* QSPI0
 *   QSPI CS  - P0.18
 *   QSPI SCK - P0.17
 *   QSPI IO0 - P0.13
 *   QSPI IO1 - P0.14
 */

#define NRF52_QSPI0_CSN_PIN (GPIO_OUTPUT | GPIO_VALUE_ONE | GPIO_PORT0 | GPIO_PIN(18))
#define NRF52_QSPI0_SCK_PIN (GPIO_OUTPUT | GPIO_PORT0 | GPIO_PIN(17))
#define NRF52_QSPI0_IO0_PIN (GPIO_OUTPUT | GPIO_PORT0 | GPIO_PIN(13))
#define NRF52_QSPI0_IO1_PIN (GPIO_OUTPUT | GPIO_PORT0 | GPIO_PIN(14))

#endif /* __BOARDS_ARM_NRF53_THINGY53_INCLUDE_BOARD_H */
