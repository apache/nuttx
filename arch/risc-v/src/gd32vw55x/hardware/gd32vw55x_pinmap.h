/****************************************************************************
 * arch/risc-v/src/gd32vw55x/hardware/gd32vw55x_pinmap.h
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

#ifndef __ARCH_RISCV_SRC_GD32VW55X_HARDWARE_GD32VW55X_PINMAP_H
#define __ARCH_RISCV_SRC_GD32VW55X_HARDWARE_GD32VW55X_PINMAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Alternate function pin selections for the GD32VW553xx.  The alternate
 * function numbers are taken from the "pin alternate functions summary"
 * tables (Table 2-5, 2-6 and 2-7) of the GD32VW553xx datasheet (Rev 1.5).
 *
 * Each peripheral signal that has more than one possible pin gets one
 * definition per pin.  The board.h file of each board must select the one
 * that matches the board wiring, for example:
 *
 *   #define GPIO_SPI_SCK   GPIO_SPI_SCK_1
 *
 * These pin configurations are bit encoded as described in
 * gd32vw55x_gpio.h and are consumed by gd32_gpio_config().
 */

/* SPI */

/* The GD32VW55x provides a single SPI peripheral */

#define GPIO_SPI_SCK_1    (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                           GPIO_CFG_PP | GPIO_CFG_SPEED_MAX | \
                           GPIO_CFG_AF_5 | GPIO_CFG_PORT_A | GPIO_CFG_PIN_2)
#define GPIO_SPI_SCK_2    (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                           GPIO_CFG_PP | GPIO_CFG_SPEED_MAX | \
                           GPIO_CFG_AF_5 | GPIO_CFG_PORT_A | GPIO_CFG_PIN_5)
#define GPIO_SPI_SCK_3    (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                           GPIO_CFG_PP | GPIO_CFG_SPEED_MAX | \
                           GPIO_CFG_AF_7 | GPIO_CFG_PORT_A | GPIO_CFG_PIN_6)
#define GPIO_SPI_SCK_4    (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                           GPIO_CFG_PP | GPIO_CFG_SPEED_MAX | \
                           GPIO_CFG_AF_0 | GPIO_CFG_PORT_A | GPIO_CFG_PIN_11)

#define GPIO_SPI_MISO_1   (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                           GPIO_CFG_PP | GPIO_CFG_SPEED_MAX | \
                           GPIO_CFG_AF_5 | GPIO_CFG_PORT_A | GPIO_CFG_PIN_1)
#define GPIO_SPI_MISO_2   (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                           GPIO_CFG_PP | GPIO_CFG_SPEED_MAX | \
                           GPIO_CFG_AF_4 | GPIO_CFG_PORT_A | GPIO_CFG_PIN_5)
#define GPIO_SPI_MISO_3   (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                           GPIO_CFG_PP | GPIO_CFG_SPEED_MAX | \
                           GPIO_CFG_AF_5 | GPIO_CFG_PORT_A | GPIO_CFG_PIN_6)
#define GPIO_SPI_MISO_4   (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                           GPIO_CFG_PP | GPIO_CFG_SPEED_MAX | \
                           GPIO_CFG_AF_0 | GPIO_CFG_PORT_A | GPIO_CFG_PIN_10)

#define GPIO_SPI_MOSI_1   (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                           GPIO_CFG_PP | GPIO_CFG_SPEED_MAX | \
                           GPIO_CFG_AF_5 | GPIO_CFG_PORT_A | GPIO_CFG_PIN_0)
#define GPIO_SPI_MOSI_2   (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                           GPIO_CFG_PP | GPIO_CFG_SPEED_MAX | \
                           GPIO_CFG_AF_2 | GPIO_CFG_PORT_A | GPIO_CFG_PIN_4)
#define GPIO_SPI_MOSI_3   (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                           GPIO_CFG_PP | GPIO_CFG_SPEED_MAX | \
                           GPIO_CFG_AF_5 | GPIO_CFG_PORT_A | GPIO_CFG_PIN_7)
#define GPIO_SPI_MOSI_4   (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                           GPIO_CFG_PP | GPIO_CFG_SPEED_MAX | \
                           GPIO_CFG_AF_0 | GPIO_CFG_PORT_A | GPIO_CFG_PIN_9)

#define GPIO_SPI_NSS_1    (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                           GPIO_CFG_PP | GPIO_CFG_SPEED_MAX | \
                           GPIO_CFG_AF_5 | GPIO_CFG_PORT_A | GPIO_CFG_PIN_3)
#define GPIO_SPI_NSS_2    (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                           GPIO_CFG_PP | GPIO_CFG_SPEED_MAX | \
                           GPIO_CFG_AF_5 | GPIO_CFG_PORT_A | GPIO_CFG_PIN_4)
#define GPIO_SPI_NSS_3    (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                           GPIO_CFG_PP | GPIO_CFG_SPEED_MAX | \
                           GPIO_CFG_AF_4 | GPIO_CFG_PORT_A | GPIO_CFG_PIN_7)
#define GPIO_SPI_NSS_4    (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                           GPIO_CFG_PP | GPIO_CFG_SPEED_MAX | \
                           GPIO_CFG_AF_6 | GPIO_CFG_PORT_A | GPIO_CFG_PIN_12)

/* I2C0 */

#define GPIO_I2C0_SCL_1   (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_PULLUP | \
                           GPIO_CFG_OD | GPIO_CFG_SPEED_MAX | \
                           GPIO_CFG_AF_4 | GPIO_CFG_PORT_A | GPIO_CFG_PIN_2)
#define GPIO_I2C0_SCL_2   (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_PULLUP | \
                           GPIO_CFG_OD | GPIO_CFG_SPEED_MAX | \
                           GPIO_CFG_AF_4 | GPIO_CFG_PORT_A | GPIO_CFG_PIN_15)
#define GPIO_I2C0_SCL_3   (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_PULLUP | \
                           GPIO_CFG_OD | GPIO_CFG_SPEED_MAX | \
                           GPIO_CFG_AF_6 | GPIO_CFG_PORT_B | GPIO_CFG_PIN_0)
#define GPIO_I2C0_SCL_4   (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_PULLUP | \
                           GPIO_CFG_OD | GPIO_CFG_SPEED_MAX | \
                           GPIO_CFG_AF_4 | GPIO_CFG_PORT_B | GPIO_CFG_PIN_15)

#define GPIO_I2C0_SDA_1   (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_PULLUP | \
                           GPIO_CFG_OD | GPIO_CFG_SPEED_MAX | \
                           GPIO_CFG_AF_4 | GPIO_CFG_PORT_A | GPIO_CFG_PIN_3)
#define GPIO_I2C0_SDA_2   (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_PULLUP | \
                           GPIO_CFG_OD | GPIO_CFG_SPEED_MAX | \
                           GPIO_CFG_AF_5 | GPIO_CFG_PORT_A | GPIO_CFG_PIN_8)
#define GPIO_I2C0_SDA_3   (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_PULLUP | \
                           GPIO_CFG_OD | GPIO_CFG_SPEED_MAX | \
                           GPIO_CFG_AF_6 | GPIO_CFG_PORT_B | GPIO_CFG_PIN_1)
#define GPIO_I2C0_SDA_4   (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_PULLUP | \
                           GPIO_CFG_OD | GPIO_CFG_SPEED_MAX | \
                           GPIO_CFG_AF_4 | GPIO_CFG_PORT_C | GPIO_CFG_PIN_8)

#define GPIO_I2C0_SMBA    (GPIO_CFG_MODE_AF | GPIO_CFG_PP | \
                           GPIO_CFG_SPEED_MAX | GPIO_CFG_AF_4 | \
                           GPIO_CFG_PORT_A | GPIO_CFG_PIN_13)

/* I2C1 */

#define GPIO_I2C1_SCL_1   (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_PULLUP | \
                           GPIO_CFG_OD | GPIO_CFG_SPEED_MAX | \
                           GPIO_CFG_AF_4 | GPIO_CFG_PORT_A | GPIO_CFG_PIN_6)
#define GPIO_I2C1_SCL_2   (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_PULLUP | \
                           GPIO_CFG_OD | GPIO_CFG_SPEED_MAX | \
                           GPIO_CFG_AF_6 | GPIO_CFG_PORT_A | GPIO_CFG_PIN_13)
#define GPIO_I2C1_SCL_3   (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_PULLUP | \
                           GPIO_CFG_OD | GPIO_CFG_SPEED_MAX | \
                           GPIO_CFG_AF_6 | GPIO_CFG_PORT_A | GPIO_CFG_PIN_15)
#define GPIO_I2C1_SCL_4   (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_PULLUP | \
                           GPIO_CFG_OD | GPIO_CFG_SPEED_MAX | \
                           GPIO_CFG_AF_6 | GPIO_CFG_PORT_B | GPIO_CFG_PIN_12)
#define GPIO_I2C1_SCL_5   (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_PULLUP | \
                           GPIO_CFG_OD | GPIO_CFG_SPEED_MAX | \
                           GPIO_CFG_AF_6 | GPIO_CFG_PORT_B | GPIO_CFG_PIN_15)

#define GPIO_I2C1_SDA_1   (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_PULLUP | \
                           GPIO_CFG_OD | GPIO_CFG_SPEED_MAX | \
                           GPIO_CFG_AF_0 | GPIO_CFG_PORT_A | GPIO_CFG_PIN_7)
#define GPIO_I2C1_SDA_2   (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_PULLUP | \
                           GPIO_CFG_OD | GPIO_CFG_SPEED_MAX | \
                           GPIO_CFG_AF_6 | GPIO_CFG_PORT_A | GPIO_CFG_PIN_8)
#define GPIO_I2C1_SDA_3   (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_PULLUP | \
                           GPIO_CFG_OD | GPIO_CFG_SPEED_MAX | \
                           GPIO_CFG_AF_6 | GPIO_CFG_PORT_A | GPIO_CFG_PIN_14)
#define GPIO_I2C1_SDA_4   (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_PULLUP | \
                           GPIO_CFG_OD | GPIO_CFG_SPEED_MAX | \
                           GPIO_CFG_AF_6 | GPIO_CFG_PORT_B | GPIO_CFG_PIN_13)
#define GPIO_I2C1_SDA_5   (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_PULLUP | \
                           GPIO_CFG_OD | GPIO_CFG_SPEED_MAX | \
                           GPIO_CFG_AF_6 | GPIO_CFG_PORT_C | GPIO_CFG_PIN_8)

#define GPIO_I2C1_SMBA    (GPIO_CFG_MODE_AF | GPIO_CFG_PP | \
                           GPIO_CFG_SPEED_MAX | GPIO_CFG_AF_4 | \
                           GPIO_CFG_PORT_A | GPIO_CFG_PIN_14)

#endif /* __ARCH_RISCV_SRC_GD32VW55X_HARDWARE_GD32VW55X_PINMAP_H */
