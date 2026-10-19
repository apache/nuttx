/****************************************************************************
 * arch/arm/src/gd32w51x/hardware/gd32w51x_pinmap.h
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

#ifndef __ARCH_ARM_SRC_GD32W51X_HARDWARE_GD32W51X_PINMAP_H
#define __ARCH_ARM_SRC_GD32W51X_HARDWARE_GD32W51X_PINMAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Alternate function pin selections for the GD32W515xx.  The alternate
 * function numbers are taken from the "alternate functions summary" tables
 * (Table 2-5, 2-6 and 2-7) of the GD32W515xx datasheet (Rev 1.9).
 *
 * Each peripheral signal that has more than one possible pin gets one
 * definition per pin, in port and pin order.  The board.h file of each
 * board must select the one that matches the board wiring, for example:
 *
 *   #define GPIO_SPI0_SCK  GPIO_SPI0_SCK_3
 *
 * These pin configurations are bit encoded as described in
 * gd32w51x_gpio.h and are consumed by gd32_gpio_config().
 */

/* SPI0 */

#define GPIO_SPI0_SCK_1           (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                                   GPIO_CFG_PP | GPIO_CFG_SPEED_MAX | \
                                   GPIO_CFG_AF_5 | GPIO_CFG_PORT_A | \
                                   GPIO_CFG_PIN_5)
#define GPIO_SPI0_SCK_2           (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                                   GPIO_CFG_PP | GPIO_CFG_SPEED_MAX | \
                                   GPIO_CFG_AF_7 | GPIO_CFG_PORT_A | \
                                   GPIO_CFG_PIN_6)
#define GPIO_SPI0_SCK_3           (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                                   GPIO_CFG_PP | GPIO_CFG_SPEED_MAX | \
                                   GPIO_CFG_AF_0 | GPIO_CFG_PORT_A | \
                                   GPIO_CFG_PIN_11)
#define GPIO_SPI0_SCK_4           (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                                   GPIO_CFG_PP | GPIO_CFG_SPEED_MAX | \
                                   GPIO_CFG_AF_5 | GPIO_CFG_PORT_B | \
                                   GPIO_CFG_PIN_3)

#define GPIO_SPI0_MISO_1          (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                                   GPIO_CFG_PP | GPIO_CFG_SPEED_MAX | \
                                   GPIO_CFG_AF_4 | GPIO_CFG_PORT_A | \
                                   GPIO_CFG_PIN_5)
#define GPIO_SPI0_MISO_2          (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                                   GPIO_CFG_PP | GPIO_CFG_SPEED_MAX | \
                                   GPIO_CFG_AF_5 | GPIO_CFG_PORT_A | \
                                   GPIO_CFG_PIN_6)
#define GPIO_SPI0_MISO_3          (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                                   GPIO_CFG_PP | GPIO_CFG_SPEED_MAX | \
                                   GPIO_CFG_AF_0 | GPIO_CFG_PORT_A | \
                                   GPIO_CFG_PIN_10)
#define GPIO_SPI0_MISO_4          (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                                   GPIO_CFG_PP | GPIO_CFG_SPEED_MAX | \
                                   GPIO_CFG_AF_5 | GPIO_CFG_PORT_B | \
                                   GPIO_CFG_PIN_4)

#define GPIO_SPI0_MOSI_1          (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                                   GPIO_CFG_PP | GPIO_CFG_SPEED_MAX | \
                                   GPIO_CFG_AF_2 | GPIO_CFG_PORT_A | \
                                   GPIO_CFG_PIN_4)
#define GPIO_SPI0_MOSI_2          (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                                   GPIO_CFG_PP | GPIO_CFG_SPEED_MAX | \
                                   GPIO_CFG_AF_5 | GPIO_CFG_PORT_A | \
                                   GPIO_CFG_PIN_7)
#define GPIO_SPI0_MOSI_3          (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                                   GPIO_CFG_PP | GPIO_CFG_SPEED_MAX | \
                                   GPIO_CFG_AF_0 | GPIO_CFG_PORT_A | \
                                   GPIO_CFG_PIN_9)
#define GPIO_SPI0_MOSI_4          (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                                   GPIO_CFG_PP | GPIO_CFG_SPEED_MAX | \
                                   GPIO_CFG_AF_5 | GPIO_CFG_PORT_B | \
                                   GPIO_CFG_PIN_5)

#define GPIO_SPI0_NSS_1           (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                                   GPIO_CFG_PP | GPIO_CFG_SPEED_MAX | \
                                   GPIO_CFG_AF_5 | GPIO_CFG_PORT_A | \
                                   GPIO_CFG_PIN_4)
#define GPIO_SPI0_NSS_2           (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                                   GPIO_CFG_PP | GPIO_CFG_SPEED_MAX | \
                                   GPIO_CFG_AF_4 | GPIO_CFG_PORT_A | \
                                   GPIO_CFG_PIN_7)
#define GPIO_SPI0_NSS_3           (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                                   GPIO_CFG_PP | GPIO_CFG_SPEED_MAX | \
                                   GPIO_CFG_AF_6 | GPIO_CFG_PORT_A | \
                                   GPIO_CFG_PIN_12)

/* SPI1 */

#define GPIO_SPI1_SCK_1           (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                                   GPIO_CFG_PP | GPIO_CFG_SPEED_MAX | \
                                   GPIO_CFG_AF_5 | GPIO_CFG_PORT_A | \
                                   GPIO_CFG_PIN_9)
#define GPIO_SPI1_SCK_2           (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                                   GPIO_CFG_PP | GPIO_CFG_SPEED_MAX | \
                                   GPIO_CFG_AF_5 | GPIO_CFG_PORT_B | \
                                   GPIO_CFG_PIN_8)
#define GPIO_SPI1_SCK_3           (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                                   GPIO_CFG_PP | GPIO_CFG_SPEED_MAX | \
                                   GPIO_CFG_AF_5 | GPIO_CFG_PORT_B | \
                                   GPIO_CFG_PIN_13)
#define GPIO_SPI1_SCK_4           (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                                   GPIO_CFG_PP | GPIO_CFG_SPEED_MAX | \
                                   GPIO_CFG_AF_5 | GPIO_CFG_PORT_C | \
                                   GPIO_CFG_PIN_7)

#define GPIO_SPI1_MISO_1          (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                                   GPIO_CFG_PP | GPIO_CFG_SPEED_MAX | \
                                   GPIO_CFG_AF_5 | GPIO_CFG_PORT_B | \
                                   GPIO_CFG_PIN_6)
#define GPIO_SPI1_MISO_2          (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                                   GPIO_CFG_PP | GPIO_CFG_SPEED_MAX | \
                                   GPIO_CFG_AF_5 | GPIO_CFG_PORT_B | \
                                   GPIO_CFG_PIN_14)
#define GPIO_SPI1_MISO_3          (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                                   GPIO_CFG_PP | GPIO_CFG_SPEED_MAX | \
                                   GPIO_CFG_AF_5 | GPIO_CFG_PORT_C | \
                                   GPIO_CFG_PIN_2)

#define GPIO_SPI1_MOSI_1          (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                                   GPIO_CFG_PP | GPIO_CFG_SPEED_MAX | \
                                   GPIO_CFG_AF_6 | GPIO_CFG_PORT_A | \
                                   GPIO_CFG_PIN_4)
#define GPIO_SPI1_MOSI_2          (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                                   GPIO_CFG_PP | GPIO_CFG_SPEED_MAX | \
                                   GPIO_CFG_AF_5 | GPIO_CFG_PORT_B | \
                                   GPIO_CFG_PIN_9)
#define GPIO_SPI1_MOSI_3          (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                                   GPIO_CFG_PP | GPIO_CFG_SPEED_MAX | \
                                   GPIO_CFG_AF_5 | GPIO_CFG_PORT_B | \
                                   GPIO_CFG_PIN_15)
#define GPIO_SPI1_MOSI_4          (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                                   GPIO_CFG_PP | GPIO_CFG_SPEED_MAX | \
                                   GPIO_CFG_AF_7 | GPIO_CFG_PORT_C | \
                                   GPIO_CFG_PIN_1)
#define GPIO_SPI1_MOSI_5          (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                                   GPIO_CFG_PP | GPIO_CFG_SPEED_MAX | \
                                   GPIO_CFG_AF_5 | GPIO_CFG_PORT_C | \
                                   GPIO_CFG_PIN_3)

#define GPIO_SPI1_NSS_1           (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                                   GPIO_CFG_PP | GPIO_CFG_SPEED_MAX | \
                                   GPIO_CFG_AF_0 | GPIO_CFG_PORT_A | \
                                   GPIO_CFG_PIN_7)
#define GPIO_SPI1_NSS_2           (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                                   GPIO_CFG_PP | GPIO_CFG_SPEED_MAX | \
                                   GPIO_CFG_AF_5 | GPIO_CFG_PORT_B | \
                                   GPIO_CFG_PIN_7)
#define GPIO_SPI1_NSS_3           (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                                   GPIO_CFG_PP | GPIO_CFG_SPEED_MAX | \
                                   GPIO_CFG_AF_5 | GPIO_CFG_PORT_B | \
                                   GPIO_CFG_PIN_12)

/* QSPI */

#define GPIO_QSPI_SCK_1           (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                                   GPIO_CFG_PP | GPIO_CFG_SPEED_MAX | \
                                   GPIO_CFG_AF_3 | GPIO_CFG_PORT_A | \
                                   GPIO_CFG_PIN_4)
#define GPIO_QSPI_SCK_2           (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                                   GPIO_CFG_PP | GPIO_CFG_SPEED_MAX | \
                                   GPIO_CFG_AF_4 | GPIO_CFG_PORT_A | \
                                   GPIO_CFG_PIN_9)

#define GPIO_QSPI_CS_1            (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                                   GPIO_CFG_PP | GPIO_CFG_SPEED_MAX | \
                                   GPIO_CFG_AF_3 | GPIO_CFG_PORT_A | \
                                   GPIO_CFG_PIN_5)
#define GPIO_QSPI_CS_2            (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                                   GPIO_CFG_PP | GPIO_CFG_SPEED_MAX | \
                                   GPIO_CFG_AF_4 | GPIO_CFG_PORT_A | \
                                   GPIO_CFG_PIN_10)

#define GPIO_QSPI_IO0_1           (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                                   GPIO_CFG_PP | GPIO_CFG_SPEED_MAX | \
                                   GPIO_CFG_AF_3 | GPIO_CFG_PORT_A | \
                                   GPIO_CFG_PIN_6)
#define GPIO_QSPI_IO0_2           (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                                   GPIO_CFG_PP | GPIO_CFG_SPEED_MAX | \
                                   GPIO_CFG_AF_4 | GPIO_CFG_PORT_A | \
                                   GPIO_CFG_PIN_11)

#define GPIO_QSPI_IO1_1           (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                                   GPIO_CFG_PP | GPIO_CFG_SPEED_MAX | \
                                   GPIO_CFG_AF_3 | GPIO_CFG_PORT_A | \
                                   GPIO_CFG_PIN_7)
#define GPIO_QSPI_IO1_2           (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                                   GPIO_CFG_PP | GPIO_CFG_SPEED_MAX | \
                                   GPIO_CFG_AF_4 | GPIO_CFG_PORT_A | \
                                   GPIO_CFG_PIN_12)

#define GPIO_QSPI_IO2_1           (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                                   GPIO_CFG_PP | GPIO_CFG_SPEED_MAX | \
                                   GPIO_CFG_AF_3 | GPIO_CFG_PORT_B | \
                                   GPIO_CFG_PIN_3)
#define GPIO_QSPI_IO2_2           (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                                   GPIO_CFG_PP | GPIO_CFG_SPEED_MAX | \
                                   GPIO_CFG_AF_3 | GPIO_CFG_PORT_C | \
                                   GPIO_CFG_PIN_4)

#define GPIO_QSPI_IO3_1           (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                                   GPIO_CFG_PP | GPIO_CFG_SPEED_MAX | \
                                   GPIO_CFG_AF_3 | GPIO_CFG_PORT_B | \
                                   GPIO_CFG_PIN_4)
#define GPIO_QSPI_IO3_2           (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                                   GPIO_CFG_PP | GPIO_CFG_SPEED_MAX | \
                                   GPIO_CFG_AF_3 | GPIO_CFG_PORT_C | \
                                   GPIO_CFG_PIN_5)

/* I2C0 */

#define GPIO_I2C0_SCL_1           (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_PULLUP | \
                                   GPIO_CFG_OD | GPIO_CFG_SPEED_MAX | \
                                   GPIO_CFG_AF_4 | GPIO_CFG_PORT_B | \
                                   GPIO_CFG_PIN_15)
#define GPIO_I2C0_SCL_2           (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_PULLUP | \
                                   GPIO_CFG_OD | GPIO_CFG_SPEED_MAX | \
                                   GPIO_CFG_AF_4 | GPIO_CFG_PORT_C | \
                                   GPIO_CFG_PIN_2)

#define GPIO_I2C0_SDA_1           (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_PULLUP | \
                                   GPIO_CFG_OD | GPIO_CFG_SPEED_MAX | \
                                   GPIO_CFG_AF_5 | GPIO_CFG_PORT_A | \
                                   GPIO_CFG_PIN_8)
#define GPIO_I2C0_SDA_2           (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_PULLUP | \
                                   GPIO_CFG_OD | GPIO_CFG_SPEED_MAX | \
                                   GPIO_CFG_AF_4 | GPIO_CFG_PORT_B | \
                                   GPIO_CFG_PIN_7)
#define GPIO_I2C0_SDA_3           (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_PULLUP | \
                                   GPIO_CFG_OD | GPIO_CFG_SPEED_MAX | \
                                   GPIO_CFG_AF_4 | GPIO_CFG_PORT_C | \
                                   GPIO_CFG_PIN_3)
#define GPIO_I2C0_SDA_4           (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_PULLUP | \
                                   GPIO_CFG_OD | GPIO_CFG_SPEED_MAX | \
                                   GPIO_CFG_AF_5 | GPIO_CFG_PORT_C | \
                                   GPIO_CFG_PIN_8)

/* I2C1 */

#define GPIO_I2C1_SCL_1           (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_PULLUP | \
                                   GPIO_CFG_OD | GPIO_CFG_SPEED_MAX | \
                                   GPIO_CFG_AF_6 | GPIO_CFG_PORT_B | \
                                   GPIO_CFG_PIN_15)
#define GPIO_I2C1_SCL_2           (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_PULLUP | \
                                   GPIO_CFG_OD | GPIO_CFG_SPEED_MAX | \
                                   GPIO_CFG_AF_6 | GPIO_CFG_PORT_C | \
                                   GPIO_CFG_PIN_3)

#define GPIO_I2C1_SDA_1           (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_PULLUP | \
                                   GPIO_CFG_OD | GPIO_CFG_SPEED_MAX | \
                                   GPIO_CFG_AF_6 | GPIO_CFG_PORT_A | \
                                   GPIO_CFG_PIN_8)
#define GPIO_I2C1_SDA_2           (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_PULLUP | \
                                   GPIO_CFG_OD | GPIO_CFG_SPEED_MAX | \
                                   GPIO_CFG_AF_9 | GPIO_CFG_PORT_C | \
                                   GPIO_CFG_PIN_2)
#define GPIO_I2C1_SDA_3           (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_PULLUP | \
                                   GPIO_CFG_OD | GPIO_CFG_SPEED_MAX | \
                                   GPIO_CFG_AF_6 | GPIO_CFG_PORT_C | \
                                   GPIO_CFG_PIN_8)

/* TIMER0 */

#define GPIO_TIMER0_CH0OUT_1      (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                                   GPIO_CFG_PP | GPIO_CFG_SPEED_MAX | \
                                   GPIO_CFG_AF_6 | GPIO_CFG_PORT_A | \
                                   GPIO_CFG_PIN_2)
#define GPIO_TIMER0_CH0OUT_2      (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                                   GPIO_CFG_PP | GPIO_CFG_SPEED_MAX | \
                                   GPIO_CFG_AF_1 | GPIO_CFG_PORT_A | \
                                   GPIO_CFG_PIN_8)
#define GPIO_TIMER0_CH0OUT_3      (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                                   GPIO_CFG_PP | GPIO_CFG_SPEED_MAX | \
                                   GPIO_CFG_AF_1 | GPIO_CFG_PORT_C | \
                                   GPIO_CFG_PIN_2)

#define GPIO_TIMER0_CH0IN_1       (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                                   GPIO_CFG_AF_6 | GPIO_CFG_PORT_A | \
                                   GPIO_CFG_PIN_2)
#define GPIO_TIMER0_CH0IN_2       (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                                   GPIO_CFG_AF_1 | GPIO_CFG_PORT_A | \
                                   GPIO_CFG_PIN_8)
#define GPIO_TIMER0_CH0IN_3       (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                                   GPIO_CFG_AF_1 | GPIO_CFG_PORT_C | \
                                   GPIO_CFG_PIN_2)

#define GPIO_TIMER0_CH1OUT_1      (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                                   GPIO_CFG_PP | GPIO_CFG_SPEED_MAX | \
                                   GPIO_CFG_AF_8 | GPIO_CFG_PORT_A | \
                                   GPIO_CFG_PIN_4)
#define GPIO_TIMER0_CH1OUT_2      (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                                   GPIO_CFG_PP | GPIO_CFG_SPEED_MAX | \
                                   GPIO_CFG_AF_8 | GPIO_CFG_PORT_A | \
                                   GPIO_CFG_PIN_6)
#define GPIO_TIMER0_CH1OUT_3      (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                                   GPIO_CFG_PP | GPIO_CFG_SPEED_MAX | \
                                   GPIO_CFG_AF_1 | GPIO_CFG_PORT_A | \
                                   GPIO_CFG_PIN_9)
#define GPIO_TIMER0_CH1OUT_4      (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                                   GPIO_CFG_PP | GPIO_CFG_SPEED_MAX | \
                                   GPIO_CFG_AF_8 | GPIO_CFG_PORT_B | \
                                   GPIO_CFG_PIN_10)
#define GPIO_TIMER0_CH1OUT_5      (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                                   GPIO_CFG_PP | GPIO_CFG_SPEED_MAX | \
                                   GPIO_CFG_AF_8 | GPIO_CFG_PORT_C | \
                                   GPIO_CFG_PIN_6)

#define GPIO_TIMER0_CH2OUT_1      (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                                   GPIO_CFG_PP | GPIO_CFG_SPEED_MAX | \
                                   GPIO_CFG_AF_1 | GPIO_CFG_PORT_A | \
                                   GPIO_CFG_PIN_10)

#define GPIO_TIMER0_CH3OUT_1      (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                                   GPIO_CFG_PP | GPIO_CFG_SPEED_MAX | \
                                   GPIO_CFG_AF_1 | GPIO_CFG_PORT_A | \
                                   GPIO_CFG_PIN_11)
#define GPIO_TIMER0_CH3OUT_2      (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                                   GPIO_CFG_PP | GPIO_CFG_SPEED_MAX | \
                                   GPIO_CFG_AF_2 | GPIO_CFG_PORT_B | \
                                   GPIO_CFG_PIN_12)

#define GPIO_TIMER0_CH0NOUT_1     (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                                   GPIO_CFG_PP | GPIO_CFG_SPEED_MAX | \
                                   GPIO_CFG_AF_8 | GPIO_CFG_PORT_A | \
                                   GPIO_CFG_PIN_3)
#define GPIO_TIMER0_CH0NOUT_2     (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                                   GPIO_CFG_PP | GPIO_CFG_SPEED_MAX | \
                                   GPIO_CFG_AF_1 | GPIO_CFG_PORT_A | \
                                   GPIO_CFG_PIN_7)
#define GPIO_TIMER0_CH0NOUT_3     (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                                   GPIO_CFG_PP | GPIO_CFG_SPEED_MAX | \
                                   GPIO_CFG_AF_1 | GPIO_CFG_PORT_B | \
                                   GPIO_CFG_PIN_13)
#define GPIO_TIMER0_CH0NOUT_4     (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                                   GPIO_CFG_PP | GPIO_CFG_SPEED_MAX | \
                                   GPIO_CFG_AF_8 | GPIO_CFG_PORT_C | \
                                   GPIO_CFG_PIN_3)

#define GPIO_TIMER0_CH1NOUT_1     (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                                   GPIO_CFG_PP | GPIO_CFG_SPEED_MAX | \
                                   GPIO_CFG_AF_8 | GPIO_CFG_PORT_A | \
                                   GPIO_CFG_PIN_5)
#define GPIO_TIMER0_CH1NOUT_2     (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                                   GPIO_CFG_PP | GPIO_CFG_SPEED_MAX | \
                                   GPIO_CFG_AF_6 | GPIO_CFG_PORT_A | \
                                   GPIO_CFG_PIN_7)
#define GPIO_TIMER0_CH1NOUT_3     (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                                   GPIO_CFG_PP | GPIO_CFG_SPEED_MAX | \
                                   GPIO_CFG_AF_1 | GPIO_CFG_PORT_B | \
                                   GPIO_CFG_PIN_0)
#define GPIO_TIMER0_CH1NOUT_4     (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                                   GPIO_CFG_PP | GPIO_CFG_SPEED_MAX | \
                                   GPIO_CFG_AF_2 | GPIO_CFG_PORT_B | \
                                   GPIO_CFG_PIN_11)
#define GPIO_TIMER0_CH1NOUT_5     (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                                   GPIO_CFG_PP | GPIO_CFG_SPEED_MAX | \
                                   GPIO_CFG_AF_1 | GPIO_CFG_PORT_B | \
                                   GPIO_CFG_PIN_14)
#define GPIO_TIMER0_CH1NOUT_6     (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                                   GPIO_CFG_PP | GPIO_CFG_SPEED_MAX | \
                                   GPIO_CFG_AF_3 | GPIO_CFG_PORT_C | \
                                   GPIO_CFG_PIN_7)

#define GPIO_TIMER0_CH2NOUT_1     (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                                   GPIO_CFG_PP | GPIO_CFG_SPEED_MAX | \
                                   GPIO_CFG_AF_1 | GPIO_CFG_PORT_B | \
                                   GPIO_CFG_PIN_1)
#define GPIO_TIMER0_CH2NOUT_2     (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                                   GPIO_CFG_PP | GPIO_CFG_SPEED_MAX | \
                                   GPIO_CFG_AF_1 | GPIO_CFG_PORT_B | \
                                   GPIO_CFG_PIN_15)

/* TIMER1 */

#define GPIO_TIMER1_CH0OUT_1      (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                                   GPIO_CFG_PP | GPIO_CFG_SPEED_MAX | \
                                   GPIO_CFG_AF_1 | GPIO_CFG_PORT_A | \
                                   GPIO_CFG_PIN_0)
#define GPIO_TIMER1_CH0OUT_2      (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                                   GPIO_CFG_PP | GPIO_CFG_SPEED_MAX | \
                                   GPIO_CFG_AF_1 | GPIO_CFG_PORT_B | \
                                   GPIO_CFG_PIN_4)
#define GPIO_TIMER1_CH0OUT_3      (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                                   GPIO_CFG_PP | GPIO_CFG_SPEED_MAX | \
                                   GPIO_CFG_AF_1 | GPIO_CFG_PORT_C | \
                                   GPIO_CFG_PIN_3)

#define GPIO_TIMER1_CH0IN_1       (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                                   GPIO_CFG_AF_1 | GPIO_CFG_PORT_A | \
                                   GPIO_CFG_PIN_0)
#define GPIO_TIMER1_CH0IN_2       (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                                   GPIO_CFG_AF_1 | GPIO_CFG_PORT_B | \
                                   GPIO_CFG_PIN_4)
#define GPIO_TIMER1_CH0IN_3       (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                                   GPIO_CFG_AF_1 | GPIO_CFG_PORT_C | \
                                   GPIO_CFG_PIN_3)

#define GPIO_TIMER1_CH1OUT_1      (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                                   GPIO_CFG_PP | GPIO_CFG_SPEED_MAX | \
                                   GPIO_CFG_AF_1 | GPIO_CFG_PORT_A | \
                                   GPIO_CFG_PIN_1)
#define GPIO_TIMER1_CH1OUT_2      (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                                   GPIO_CFG_PP | GPIO_CFG_SPEED_MAX | \
                                   GPIO_CFG_AF_9 | GPIO_CFG_PORT_A | \
                                   GPIO_CFG_PIN_6)
#define GPIO_TIMER1_CH1OUT_3      (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                                   GPIO_CFG_PP | GPIO_CFG_SPEED_MAX | \
                                   GPIO_CFG_AF_1 | GPIO_CFG_PORT_B | \
                                   GPIO_CFG_PIN_3)
#define GPIO_TIMER1_CH1OUT_4      (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                                   GPIO_CFG_PP | GPIO_CFG_SPEED_MAX | \
                                   GPIO_CFG_AF_1 | GPIO_CFG_PORT_B | \
                                   GPIO_CFG_PIN_9)
#define GPIO_TIMER1_CH1OUT_5      (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                                   GPIO_CFG_PP | GPIO_CFG_SPEED_MAX | \
                                   GPIO_CFG_AF_9 | GPIO_CFG_PORT_C | \
                                   GPIO_CFG_PIN_6)

#define GPIO_TIMER1_CH2OUT_1      (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                                   GPIO_CFG_PP | GPIO_CFG_SPEED_MAX | \
                                   GPIO_CFG_AF_1 | GPIO_CFG_PORT_A | \
                                   GPIO_CFG_PIN_2)
#define GPIO_TIMER1_CH2OUT_2      (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                                   GPIO_CFG_PP | GPIO_CFG_SPEED_MAX | \
                                   GPIO_CFG_AF_9 | GPIO_CFG_PORT_A | \
                                   GPIO_CFG_PIN_7)
#define GPIO_TIMER1_CH2OUT_3      (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                                   GPIO_CFG_PP | GPIO_CFG_SPEED_MAX | \
                                   GPIO_CFG_AF_1 | GPIO_CFG_PORT_B | \
                                   GPIO_CFG_PIN_10)
#define GPIO_TIMER1_CH2OUT_4      (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                                   GPIO_CFG_PP | GPIO_CFG_SPEED_MAX | \
                                   GPIO_CFG_AF_9 | GPIO_CFG_PORT_C | \
                                   GPIO_CFG_PIN_7)

#define GPIO_TIMER1_CH3OUT_1      (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                                   GPIO_CFG_PP | GPIO_CFG_SPEED_MAX | \
                                   GPIO_CFG_AF_1 | GPIO_CFG_PORT_A | \
                                   GPIO_CFG_PIN_3)
#define GPIO_TIMER1_CH3OUT_2      (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                                   GPIO_CFG_PP | GPIO_CFG_SPEED_MAX | \
                                   GPIO_CFG_AF_1 | GPIO_CFG_PORT_B | \
                                   GPIO_CFG_PIN_2)
#define GPIO_TIMER1_CH3OUT_3      (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                                   GPIO_CFG_PP | GPIO_CFG_SPEED_MAX | \
                                   GPIO_CFG_AF_1 | GPIO_CFG_PORT_B | \
                                   GPIO_CFG_PIN_5)

/* TIMER2 */

#define GPIO_TIMER2_CH0OUT_1      (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                                   GPIO_CFG_PP | GPIO_CFG_SPEED_MAX | \
                                   GPIO_CFG_AF_2 | GPIO_CFG_PORT_A | \
                                   GPIO_CFG_PIN_6)
#define GPIO_TIMER2_CH0OUT_2      (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                                   GPIO_CFG_PP | GPIO_CFG_SPEED_MAX | \
                                   GPIO_CFG_AF_1 | GPIO_CFG_PORT_C | \
                                   GPIO_CFG_PIN_5)
#define GPIO_TIMER2_CH0OUT_3      (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                                   GPIO_CFG_PP | GPIO_CFG_SPEED_MAX | \
                                   GPIO_CFG_AF_2 | GPIO_CFG_PORT_C | \
                                   GPIO_CFG_PIN_6)

#define GPIO_TIMER2_CH0IN_1       (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                                   GPIO_CFG_AF_2 | GPIO_CFG_PORT_A | \
                                   GPIO_CFG_PIN_6)
#define GPIO_TIMER2_CH0IN_2       (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                                   GPIO_CFG_AF_1 | GPIO_CFG_PORT_C | \
                                   GPIO_CFG_PIN_5)
#define GPIO_TIMER2_CH0IN_3       (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                                   GPIO_CFG_AF_2 | GPIO_CFG_PORT_C | \
                                   GPIO_CFG_PIN_6)

#define GPIO_TIMER2_CH1OUT_1      (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                                   GPIO_CFG_PP | GPIO_CFG_SPEED_MAX | \
                                   GPIO_CFG_AF_2 | GPIO_CFG_PORT_A | \
                                   GPIO_CFG_PIN_7)
#define GPIO_TIMER2_CH1OUT_2      (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                                   GPIO_CFG_PP | GPIO_CFG_SPEED_MAX | \
                                   GPIO_CFG_AF_2 | GPIO_CFG_PORT_B | \
                                   GPIO_CFG_PIN_0)
#define GPIO_TIMER2_CH1OUT_3      (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                                   GPIO_CFG_PP | GPIO_CFG_SPEED_MAX | \
                                   GPIO_CFG_AF_2 | GPIO_CFG_PORT_C | \
                                   GPIO_CFG_PIN_7)

#define GPIO_TIMER2_CH2OUT_1      (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                                   GPIO_CFG_PP | GPIO_CFG_SPEED_MAX | \
                                   GPIO_CFG_AF_3 | GPIO_CFG_PORT_B | \
                                   GPIO_CFG_PIN_1)
#define GPIO_TIMER2_CH2OUT_2      (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                                   GPIO_CFG_PP | GPIO_CFG_SPEED_MAX | \
                                   GPIO_CFG_AF_2 | GPIO_CFG_PORT_C | \
                                   GPIO_CFG_PIN_4)
#define GPIO_TIMER2_CH2OUT_3      (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                                   GPIO_CFG_PP | GPIO_CFG_SPEED_MAX | \
                                   GPIO_CFG_AF_2 | GPIO_CFG_PORT_C | \
                                   GPIO_CFG_PIN_8)

#define GPIO_TIMER2_CH3OUT_1      (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                                   GPIO_CFG_PP | GPIO_CFG_SPEED_MAX | \
                                   GPIO_CFG_AF_3 | GPIO_CFG_PORT_B | \
                                   GPIO_CFG_PIN_2)
#define GPIO_TIMER2_CH3OUT_2      (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                                   GPIO_CFG_PP | GPIO_CFG_SPEED_MAX | \
                                   GPIO_CFG_AF_2 | GPIO_CFG_PORT_C | \
                                   GPIO_CFG_PIN_5)

/* TIMER15 */

#define GPIO_TIMER15_CH0OUT_1     (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                                   GPIO_CFG_PP | GPIO_CFG_SPEED_MAX | \
                                   GPIO_CFG_AF_8 | GPIO_CFG_PORT_A | \
                                   GPIO_CFG_PIN_8)
#define GPIO_TIMER15_CH0OUT_2     (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                                   GPIO_CFG_PP | GPIO_CFG_SPEED_MAX | \
                                   GPIO_CFG_AF_8 | GPIO_CFG_PORT_B | \
                                   GPIO_CFG_PIN_13)

#define GPIO_TIMER15_CH0NOUT_1    (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                                   GPIO_CFG_PP | GPIO_CFG_SPEED_MAX | \
                                   GPIO_CFG_AF_8 | GPIO_CFG_PORT_A | \
                                   GPIO_CFG_PIN_9)

/* TIMER16 */

#define GPIO_TIMER16_CH0OUT_1     (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                                   GPIO_CFG_PP | GPIO_CFG_SPEED_MAX | \
                                   GPIO_CFG_AF_7 | GPIO_CFG_PORT_A | \
                                   GPIO_CFG_PIN_10)
#define GPIO_TIMER16_CH0OUT_2     (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                                   GPIO_CFG_PP | GPIO_CFG_SPEED_MAX | \
                                   GPIO_CFG_AF_7 | GPIO_CFG_PORT_C | \
                                   GPIO_CFG_PIN_7)

#define GPIO_TIMER16_CH0NOUT_1    (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                                   GPIO_CFG_PP | GPIO_CFG_SPEED_MAX | \
                                   GPIO_CFG_AF_10 | GPIO_CFG_PORT_A | \
                                   GPIO_CFG_PIN_12)

/* TSI (the board adds GPIO_CFG_OD to the pin of the sampling capacitor) */

#define GPIO_TSI_G0_IO0_1         (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                                   GPIO_CFG_PP | GPIO_CFG_SPEED_2MHZ | \
                                   GPIO_CFG_AF_4 | GPIO_CFG_PORT_A | \
                                   GPIO_CFG_PIN_0)

#define GPIO_TSI_G0_IO1_1         (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                                   GPIO_CFG_PP | GPIO_CFG_SPEED_2MHZ | \
                                   GPIO_CFG_AF_4 | GPIO_CFG_PORT_A | \
                                   GPIO_CFG_PIN_1)

#define GPIO_TSI_G0_IO2_1         (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                                   GPIO_CFG_PP | GPIO_CFG_SPEED_2MHZ | \
                                   GPIO_CFG_AF_4 | GPIO_CFG_PORT_A | \
                                   GPIO_CFG_PIN_2)

#define GPIO_TSI_G0_IO3_1         (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                                   GPIO_CFG_PP | GPIO_CFG_SPEED_2MHZ | \
                                   GPIO_CFG_AF_4 | GPIO_CFG_PORT_A | \
                                   GPIO_CFG_PIN_3)

#define GPIO_TSI_G1_IO0_1         (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                                   GPIO_CFG_PP | GPIO_CFG_SPEED_2MHZ | \
                                   GPIO_CFG_AF_4 | GPIO_CFG_PORT_B | \
                                   GPIO_CFG_PIN_0)

#define GPIO_TSI_G1_IO1_1         (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                                   GPIO_CFG_PP | GPIO_CFG_SPEED_2MHZ | \
                                   GPIO_CFG_AF_4 | GPIO_CFG_PORT_B | \
                                   GPIO_CFG_PIN_1)

#define GPIO_TSI_G1_IO2_1         (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                                   GPIO_CFG_PP | GPIO_CFG_SPEED_2MHZ | \
                                   GPIO_CFG_AF_4 | GPIO_CFG_PORT_B | \
                                   GPIO_CFG_PIN_2)

#define GPIO_TSI_G1_IO3_1         (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                                   GPIO_CFG_PP | GPIO_CFG_SPEED_2MHZ | \
                                   GPIO_CFG_AF_4 | GPIO_CFG_PORT_B | \
                                   GPIO_CFG_PIN_10)

#define GPIO_TSI_G2_IO0_1         (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                                   GPIO_CFG_PP | GPIO_CFG_SPEED_2MHZ | \
                                   GPIO_CFG_AF_4 | GPIO_CFG_PORT_B | \
                                   GPIO_CFG_PIN_11)

#define GPIO_TSI_G2_IO1_1         (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                                   GPIO_CFG_PP | GPIO_CFG_SPEED_2MHZ | \
                                   GPIO_CFG_AF_4 | GPIO_CFG_PORT_B | \
                                   GPIO_CFG_PIN_12)

#define GPIO_TSI_G2_IO2_1         (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                                   GPIO_CFG_PP | GPIO_CFG_SPEED_2MHZ | \
                                   GPIO_CFG_AF_4 | GPIO_CFG_PORT_B | \
                                   GPIO_CFG_PIN_13)

#define GPIO_TSI_G2_IO3_1         (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                                   GPIO_CFG_PP | GPIO_CFG_SPEED_2MHZ | \
                                   GPIO_CFG_AF_4 | GPIO_CFG_PORT_B | \
                                   GPIO_CFG_PIN_14)

/* ADC inputs (analog mode, no alternate function) */

#define GPIO_ADC_IN0      (GPIO_CFG_MODE_ANALOG | GPIO_CFG_PUPD_NONE | \
                           GPIO_CFG_PORT_A | GPIO_CFG_PIN_0)
#define GPIO_ADC_IN1      (GPIO_CFG_MODE_ANALOG | GPIO_CFG_PUPD_NONE | \
                           GPIO_CFG_PORT_A | GPIO_CFG_PIN_1)
#define GPIO_ADC_IN2      (GPIO_CFG_MODE_ANALOG | GPIO_CFG_PUPD_NONE | \
                           GPIO_CFG_PORT_A | GPIO_CFG_PIN_2)
#define GPIO_ADC_IN3      (GPIO_CFG_MODE_ANALOG | GPIO_CFG_PUPD_NONE | \
                           GPIO_CFG_PORT_A | GPIO_CFG_PIN_3)
#define GPIO_ADC_IN4      (GPIO_CFG_MODE_ANALOG | GPIO_CFG_PUPD_NONE | \
                           GPIO_CFG_PORT_C | GPIO_CFG_PIN_0)
#define GPIO_ADC_IN5      (GPIO_CFG_MODE_ANALOG | GPIO_CFG_PUPD_NONE | \
                           GPIO_CFG_PORT_C | GPIO_CFG_PIN_1)
#define GPIO_ADC_IN6      (GPIO_CFG_MODE_ANALOG | GPIO_CFG_PUPD_NONE | \
                           GPIO_CFG_PORT_C | GPIO_CFG_PIN_2)
#define GPIO_ADC_IN7      (GPIO_CFG_MODE_ANALOG | GPIO_CFG_PUPD_NONE | \
                           GPIO_CFG_PORT_C | GPIO_CFG_PIN_3)
#define GPIO_ADC_IN8      (GPIO_CFG_MODE_ANALOG | GPIO_CFG_PUPD_NONE | \
                           GPIO_CFG_PORT_A | GPIO_CFG_PIN_4)

#endif /* __ARCH_ARM_SRC_GD32W51X_HARDWARE_GD32W51X_PINMAP_H */
