/************************************************************************************
 * arch/arm/src/stm32f0/chip/stm32f05x_pinmap.h
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *           Alan Carvalho de Assis <acassis@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_STM32F0_CHIP_STM32F05X_PINMAP_H
#define __ARCH_ARM_SRC_STM32F0_CHIP_STM32F05X_PINMAP_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "stm32f0_gpio.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Alternate Pin Functions.
 *
 * Alternative pin selections are provided with a numeric suffix like _1,
 * _2, etc.  Drivers, however, will use the pin selection without the numeric
 * suffix.  Additional definitions are required in the board.h file.  For
 * example, if USART1_TX connects vis PA9 on some board, then the following
 * definition should appear inthe board.h header file for that board:
 *
 * #define GPIO_USART1_TX GPIO_USART1_TX_1
 *
 * The driver will then automatically configre PD0 as the CAN1 RX pin.
 */

/* WARNING!!! WARNING!!! WARNING!!! WARNING!!! WARNING!!! WARNING!!! WARNING!!!
 * Additional effort is required to select specific GPIO options such as frequency,
 * open-drain/push-pull, and pull-up/down!  Just the basics are defined for most
 * pins in this file.
 */

/* ADC */

#define GPIO_ADC_IN0        (GPIO_ANALOG | GPIO_PORTA | GPIO_PIN0)
#define GPIO_ADC_IN1        (GPIO_ANALOG | GPIO_PORTA | GPIO_PIN1)
#define GPIO_ADC_IN2        (GPIO_ANALOG | GPIO_PORTA | GPIO_PIN2)
#define GPIO_ADC_IN3        (GPIO_ANALOG | GPIO_PORTA | GPIO_PIN3)
#define GPIO_ADC_IN4        (GPIO_ANALOG | GPIO_PORTA | GPIO_PIN4)
#define GPIO_ADC_IN5        (GPIO_ANALOG | GPIO_PORTA | GPIO_PIN5)
#define GPIO_ADC_IN6        (GPIO_ANALOG | GPIO_PORTA | GPIO_PIN6)
#define GPIO_ADC_IN7        (GPIO_ANALOG | GPIO_PORTA | GPIO_PIN7)
#define GPIO_ADC_IN8        (GPIO_ANALOG | GPIO_PORTB | GPIO_PIN0)
#define GPIO_ADC_IN9        (GPIO_ANALOG | GPIO_PORTB | GPIO_PIN1)
#define GPIO_ADC_IN10       (GPIO_ANALOG | GPIO_PORTC | GPIO_PIN0)
#define GPIO_ADC_IN11       (GPIO_ANALOG | GPIO_PORTC | GPIO_PIN1)
#define GPIO_ADC_IN12       (GPIO_ANALOG | GPIO_PORTC | GPIO_PIN2)
#define GPIO_ADC_IN13       (GPIO_ANALOG | GPIO_PORTC | GPIO_PIN3)
#define GPIO_ADC_IN14       (GPIO_ANALOG | GPIO_PORTC | GPIO_PIN4)
#define GPIO_ADC_IN15       (GPIO_ANALOG | GPIO_PORTC | GPIO_PIN5)

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

#define GPIO_I2C1_SCL_1     (GPIO_ALT | GPIO_AF1 | GPIO_OPENDRAIN | GPIO_SPEED_50MHz | GPIO_PORTB | GPIO_PIN6)
#define GPIO_I2C1_SCL_2     (GPIO_ALT | GPIO_AF1 | GPIO_OPENDRAIN | GPIO_SPEED_50MHz | GPIO_PORTB | GPIO_PIN8)
#define GPIO_I2C1_SDA_1     (GPIO_ALT | GPIO_AF1 | GPIO_OPENDRAIN | GPIO_SPEED_50MHz | GPIO_PORTB | GPIO_PIN7)
#define GPIO_I2C1_SDA_2     (GPIO_ALT | GPIO_AF1 | GPIO_OPENDRAIN | GPIO_SPEED_50MHz | GPIO_PORTB | GPIO_PIN9)
#define GPIO_I2C1_SMBA      (GPIO_ALT | GPIO_AF3 | GPIO_FLOAT | GPIO_SPEED_50MHz | GPIO_PORTB | GPIO_PIN5)

#define GPIO_I2C2_SCL       (GPIO_ALT | GPIO_AF1 | GPIO_OPENDRAIN | GPIO_SPEED_50MHz | GPIO_PORTB | GPIO_PIN10)
#define GPIO_I2C2_SDA       (GPIO_ALT | GPIO_AF1 | GPIO_OPENDRAIN | GPIO_SPEED_50MHz | GPIO_PORTB | GPIO_PIN11)

#endif /* __ARCH_ARM_SRC_STM32F0_CHIP_STM32F05X_PINMAP_H */
