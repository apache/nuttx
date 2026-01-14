/****************************************************************************
 * boards/arm/nrf52/arduino-nano-33ble-rev2/include/board.h
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

#ifndef __BOARDS_ARM_NRF52_ARDUINO_NANO_33BLE_REV2_INCLUDE_BOARD_H
#define __BOARDS_ARM_NRF52_ARDUINO_NANO_33BLE_REV2_INCLUDE_BOARD_H

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

/* LED definitions **********************************************************/

/* LED index values for use with board_userled()
 */

#define BOARD_LED1        0
#define BOARD_LED2        1
#define BOARD_LED3        2
#define BOARD_LED4        3
#define BOARD_LED5        4
#define BOARD_NLEDS       5

/* LED bits for use with board_userled_all() */

#define BOARD_LED1_BIT    (1 << BOARD_LED1)
#define BOARD_LED2_BIT    (1 << BOARD_LED2)
#define BOARD_LED3_BIT    (1 << BOARD_LED3)
#define BOARD_LED4_BIT    (1 << BOARD_LED4)
#define BOARD_LED5_BIT    (1 << BOARD_LED5)

/* If CONFIG_ARCH_LEDS is defined, the LED will be controlled as follows
 * for NuttX debug functionality.
 */

#define LED_STARTED                0
#define LED_HEAPALLOCATE           1
#define LED_IRQSENABLED            2
#define LED_STACKCREATED           3
#define LED_INIRQ                  4
#define LED_SIGNAL                 5
#define LED_ASSERTION              6
#define LED_PANIC                  7

/* If CONFIG_ARCH_LEDS is not defined, then the LEDs are completely under
 * control of the application.  The following interfaces are then available
 * for application control of the LEDs:
 *
 *  uint32_t board_userled_initialize(void);
 *  void board_userled(int led, bool ledon);
 *  void board_userled_all(uint32_t ledset);
 */

/* Button definitions *******************************************************/

/* Board supports no buttons. */

#define NUM_BUTTONS       0

/* UART Pins ****************************************************************/

/* The following definitions must be provided so that the NRF52 serial
 * driver can set up the UART for the serial console properly.
 */

#define BOARD_UART0_RX_PIN  (GPIO_INPUT  | GPIO_PORT1 | GPIO_PIN(10))
#define BOARD_UART0_TX_PIN  (GPIO_OUTPUT | GPIO_VALUE_ONE | GPIO_PORT1 | GPIO_PIN(3))

/* Sensor Power Pin *********************************************************/

/* The VDD_ENV must be driven high in order for sensors to receive power. */

#define BOARD_VDD_ENV_PIN   (GPIO_OUTPUT | GPIO_VALUE_ONE | GPIO_PORT0 | GPIO_PIN(22))

/* I2C Pins *****************************************************************/

/* I2C0 (TWI0) - Internal I2C
 *    I2C0_SCL - P0.15
 *    I2C0_SDA - P0.14
 */

#define BOARD_I2C0_SCL_PIN    (GPIO_OUTPUT | GPIO_PORT0 | GPIO_PIN(15))
#define BOARD_I2C0_SDA_PIN    (GPIO_INPUT  | GPIO_PORT0 | GPIO_PIN(14))
#define BOARD_I2C0_PULLUP_PIN (GPIO_OUTPUT | GPIO_VALUE_ONE | GPIO_PORT1 | GPIO_PIN(0))

/* I2C1 (TWI1) - External I2C
 *    I2C1_SCL - P0.2
 *    I2C1_SDA - P0.31
 */

#define BOARD_I2C1_SCL_PIN (GPIO_OUTPUT | GPIO_PORT0 | GPIO_PIN(2))
#define BOARD_I2C1_SDA_PIN (GPIO_INPUT  | GPIO_PORT0 | GPIO_PIN(31))

#endif /* __BOARDS_ARM_NRF52_ARDUINO_NANO_33BLE_REV2_INCLUDE_BOARD_H */
