/****************************************************************************
 * arch/arm/src/stm32f0l0g0/hardware/stm32f05x_pinmap_legacy.h
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

#ifndef __ARCH_ARM_SRC_STM32F0L0G0_HARDWARE_STM32F05X_PINMAP_LEGACY_H
#define __ARCH_ARM_SRC_STM32F0L0G0_HARDWARE_STM32F05X_PINMAP_LEGACY_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "stm32_gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Alternate Pin Functions.
 *
 * Alternative pin selections are provided with a numeric suffix like _1,
 * _2, etc.  Drivers, however, will use the pin selection without the numeric
 * suffix.  Additional definitions are required in the board.h file.  For
 * example, if USART1_TX connects via PA9 on some board, then the following
 * definition should appear in the board.h header file for that board:
 *
 * #define GPIO_USART1_TX GPIO_USART1_TX_1
 *
 * The driver will then automatically configure PD0 as the CAN1 RX pin.
 */

/* WARNING!!! WARNING!!! WARNING!!! WARNING!!! WARNING!!! WARNING!!!
 * Additional effort is required to select specific GPIO options such as
 * frequency, open-drain/push-pull, and pull-up/down!  Just the basics are
 * defined for most pins in this file.
 */

/* ADC 1 */

#define GPIO_ADC1_IN0       (GPIO_ANALOG | GPIO_PORTA | GPIO_PIN0)
#define GPIO_ADC1_IN1       (GPIO_ANALOG | GPIO_PORTA | GPIO_PIN1)
#define GPIO_ADC1_IN2       (GPIO_ANALOG | GPIO_PORTA | GPIO_PIN2)
#define GPIO_ADC1_IN3       (GPIO_ANALOG | GPIO_PORTA | GPIO_PIN3)
#define GPIO_ADC1_IN4       (GPIO_ANALOG | GPIO_PORTA | GPIO_PIN4)
#define GPIO_ADC1_IN5       (GPIO_ANALOG | GPIO_PORTA | GPIO_PIN5)
#define GPIO_ADC1_IN6       (GPIO_ANALOG | GPIO_PORTA | GPIO_PIN6)
#define GPIO_ADC1_IN7       (GPIO_ANALOG | GPIO_PORTA | GPIO_PIN7)
#define GPIO_ADC1_IN8       (GPIO_ANALOG | GPIO_PORTB | GPIO_PIN0)
#define GPIO_ADC1_IN9       (GPIO_ANALOG | GPIO_PORTB | GPIO_PIN1)
#define GPIO_ADC1_IN10      (GPIO_ANALOG | GPIO_PORTC | GPIO_PIN0)
#define GPIO_ADC1_IN11      (GPIO_ANALOG | GPIO_PORTC | GPIO_PIN1)
#define GPIO_ADC1_IN12      (GPIO_ANALOG | GPIO_PORTC | GPIO_PIN2)
#define GPIO_ADC1_IN13      (GPIO_ANALOG | GPIO_PORTC | GPIO_PIN3)
#define GPIO_ADC1_IN14      (GPIO_ANALOG | GPIO_PORTC | GPIO_PIN4)
#define GPIO_ADC1_IN15      (GPIO_ANALOG | GPIO_PORTC | GPIO_PIN5)

/* TIMERS */

/* TODO: Define TIMx pins here */

/* USART */

#define GPIO_USART1_TX_1    (GPIO_ALT | GPIO_AF1 | GPIO_PORTA | GPIO_PIN9)
#define GPIO_USART1_TX_2    (GPIO_ALT | GPIO_AF0 | GPIO_PORTB | GPIO_PIN6)
#define GPIO_USART1_RX_1    (GPIO_ALT | GPIO_AF1 | GPIO_PORTA | GPIO_PIN10)
#define GPIO_USART1_RX_2    (GPIO_ALT | GPIO_AF0 | GPIO_PORTB | GPIO_PIN7)
#define GPIO_USART1_CTS     (GPIO_ALT | GPIO_AF1 | GPIO_PORTA | GPIO_PIN11)
#define GPIO_USART1_RTS     (GPIO_ALT | GPIO_AF1 | GPIO_PORTA | GPIO_PIN12)
#define GPIO_USART1_CK      (GPIO_ALT | GPIO_AF1 | GPIO_PORTA | GPIO_PIN8)

#define GPIO_USART2_CTS     (GPIO_ALT | GPIO_AF1 | GPIO_PORTA | GPIO_PIN0)
#define GPIO_USART2_RTS     (GPIO_ALT | GPIO_AF1 | GPIO_PORTA | GPIO_PIN1)
#define GPIO_USART2_TX      (GPIO_ALT | GPIO_AF1 | GPIO_PORTA | GPIO_PIN2)
#define GPIO_USART2_RX      (GPIO_ALT | GPIO_AF1 | GPIO_PORTA | GPIO_PIN3)
#define GPIO_USART2_CK      (GPIO_ALT | GPIO_AF1 | GPIO_PORTA | GPIO_PIN4)

/* SPI */

#define GPIO_SPI1_NSS_1     (GPIO_ALT | GPIO_AF0 | GPIO_PORTA | GPIO_PIN4)
#define GPIO_SPI1_NSS_2     (GPIO_ALT | GPIO_AF0 | GPIO_PORTA | GPIO_PIN15)
#define GPIO_SPI1_SCK_1     (GPIO_ALT | GPIO_AF0 | GPIO_PORTA | GPIO_PIN5)
#define GPIO_SPI1_SCK_2     (GPIO_ALT | GPIO_AF0 | GPIO_PORTB | GPIO_PIN3)
#define GPIO_SPI1_MISO_1    (GPIO_ALT | GPIO_AF0 | GPIO_PORTA | GPIO_PIN6)
#define GPIO_SPI1_MISO_2    (GPIO_ALT | GPIO_AF0 | GPIO_PORTB | GPIO_PIN4)
#define GPIO_SPI1_MOSI_1    (GPIO_ALT | GPIO_AF0 | GPIO_PORTA | GPIO_PIN7)
#define GPIO_SPI1_MOSI_2    (GPIO_ALT | GPIO_AF0 | GPIO_PORTB | GPIO_PIN5)
#define GPIO_SPI2_NSS       (GPIO_ALT | GPIO_AF0 | GPIO_PORTB | GPIO_PIN12)
#define GPIO_SPI2_SCK       (GPIO_ALT | GPIO_AF0 | GPIO_PORTB | GPIO_PIN13)
#define GPIO_SPI2_MISO      (GPIO_ALT | GPIO_AF0 | GPIO_PORTB | GPIO_PIN14)
#define GPIO_SPI2_MOSI      (GPIO_ALT | GPIO_AF0 | GPIO_PORTB | GPIO_PIN15)

/* I2C */

#define GPIO_I2C1_SCL_1     (GPIO_ALT | GPIO_AF1 | GPIO_OPENDRAIN | GPIO_SPEED_HIGH | GPIO_PORTB | GPIO_PIN6)
#define GPIO_I2C1_SCL_2     (GPIO_ALT | GPIO_AF1 | GPIO_OPENDRAIN | GPIO_SPEED_HIGH | GPIO_PORTB | GPIO_PIN8)
#define GPIO_I2C1_SDA_1     (GPIO_ALT | GPIO_AF1 | GPIO_OPENDRAIN | GPIO_SPEED_HIGH | GPIO_PORTB | GPIO_PIN7)
#define GPIO_I2C1_SDA_2     (GPIO_ALT | GPIO_AF1 | GPIO_OPENDRAIN | GPIO_SPEED_HIGH | GPIO_PORTB | GPIO_PIN9)
#define GPIO_I2C1_SMBA      (GPIO_ALT | GPIO_AF3 | GPIO_FLOAT | GPIO_SPEED_HIGH | GPIO_PORTB | GPIO_PIN5)

#define GPIO_I2C2_SCL       (GPIO_ALT | GPIO_AF1 | GPIO_OPENDRAIN | GPIO_SPEED_HIGH | GPIO_PORTB | GPIO_PIN10)
#define GPIO_I2C2_SDA       (GPIO_ALT | GPIO_AF1 | GPIO_OPENDRAIN | GPIO_SPEED_HIGH | GPIO_PORTB | GPIO_PIN11)

#endif /* __ARCH_ARM_SRC_STM32F0L0G0_HARDWARE_STM32F05X_PINMAP_LEGACY_H */
