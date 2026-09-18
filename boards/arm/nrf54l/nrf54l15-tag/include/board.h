/****************************************************************************
 * boards/arm/nrf54l/nrf54l15-tag/include/board.h
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

#ifndef __BOARDS_ARM_NRF54L_NRF54L15_TAG_INCLUDE_BOARD_H
#define __BOARDS_ARM_NRF54L_NRF54L15_TAG_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

#define BOARD_SYSTICK_CLOCK    128000000
#define BOARD_HFXO_CAPACITANCE 15000

/* LED definitions **********************************************************/

#define BOARD_LED1  0
#define BOARD_LED2  1
#define BOARD_LED3  2
#define BOARD_NLEDS 3

/* LED bits for use with board_userled_all() */

#define BOARD_LED1_BIT (1 << BOARD_LED1)
#define BOARD_LED2_BIT (1 << BOARD_LED2)
#define BOARD_LED3_BIT (1 << BOARD_LED3)

/* LED indications when CONFIG_ARCH_LEDS is selected */

#define LED_STARTED      0 /* OFF */
#define LED_HEAPALLOCATE 0 /* OFF */
#define LED_IRQSENABLED  0 /* OFF */
#define LED_STACKCREATED 1 /* ON */
#define LED_INIRQ        2 /* No change */
#define LED_SIGNAL       2 /* No change */
#define LED_ASSERTION    2 /* No change */
#define LED_PANIC        3 /* Flashing */

/* Buttons ******************************************************************/

#define BUTTON_BTN1     0
#define NUM_BUTTONS     1
#define BUTTON_BTN1_BIT (1 << BUTTON_BTN1)

/* SPI22: BMI270 ************************************************************/

#define BOARD_SPI2_SCK_PIN  (GPIO_OUTPUT | GPIO_VALUE_ONE | GPIO_PORT1 | GPIO_PIN(8))
#define BOARD_SPI2_MOSI_PIN (GPIO_OUTPUT | GPIO_PORT1 | GPIO_PIN(6))
#define BOARD_SPI2_MISO_PIN (GPIO_INPUT | GPIO_PORT1 | GPIO_PIN(5))

/* TWIM21: BME688 and ADXL367 ***********************************************/

#define BOARD_I2C1_SCL_PIN (GPIO_OUTPUT | GPIO_PORT1 | GPIO_PIN(11))
#define BOARD_I2C1_SDA_PIN (GPIO_INPUT | GPIO_PORT1 | GPIO_PIN(12))

#endif /* __BOARDS_ARM_NRF54L_NRF54L15_TAG_INCLUDE_BOARD_H */
