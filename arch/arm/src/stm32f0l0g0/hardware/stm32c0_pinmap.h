/****************************************************************************
 * arch/arm/src/stm32f0l0g0/hardware/stm32c0_pinmap.h
 *
 * SPDX-License-Identifier: Apache-2.0
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
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_STM32F0L0G0_HARDWARE_STM32C0_PINMAP_H
#define __ARCH_ARM_SRC_STM32F0L0G0_HARDWARE_STM32C0_PINMAP_H

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
 * Alternative pin selections are provided with a numeric suffix like _1, _2,
 * etc. Drivers, however, will use the pin selection without the numeric
 * suffix. Additional definitions are required in the board.h file.  For
 * example, if CAN1_RX connects via PA11 on some board, then the following
 * definitions should appear in the board.h header file for that board:
 *
 * #define GPIO_I2C1_SCL GPIO_I2C1_SCL_1
 *
 * The driver will then automatically configure PB6 as the I2C1 SCL pin.
 */

/* WARNING!!! WARNING!!! WARNING!!! WARNING!!! WARNING!!! WARNING!!!
 * Additional effort is required to select specific GPIO options such as
 * frequency, open-drain/push-pull, and pull-up/down!  Just the basics are
 * defined for most pins in this file.
 */

/* ADC */

#define GPIO_ADC1_IN0_0     (GPIO_ANALOG | GPIO_PORTA | GPIO_PIN0)
#define GPIO_ADC1_IN1_0     (GPIO_ANALOG | GPIO_PORTA | GPIO_PIN1)
#define GPIO_ADC1_IN2_0     (GPIO_ANALOG | GPIO_PORTA | GPIO_PIN2)
#define GPIO_ADC1_IN3_0     (GPIO_ANALOG | GPIO_PORTA | GPIO_PIN3)
#define GPIO_ADC1_IN4_0     (GPIO_ANALOG | GPIO_PORTA | GPIO_PIN4)
#define GPIO_ADC1_IN5_0     (GPIO_ANALOG | GPIO_PORTA | GPIO_PIN5)
#define GPIO_ADC1_IN6_0     (GPIO_ANALOG | GPIO_PORTA | GPIO_PIN6)
#define GPIO_ADC1_IN7_0     (GPIO_ANALOG | GPIO_PORTA | GPIO_PIN7)
#define GPIO_ADC1_IN8_0     (GPIO_ANALOG | GPIO_PORTA | GPIO_PIN8)

#define GPIO_ADC1_IN11_0    (GPIO_ANALOG | GPIO_PORTC | GPIO_PIN4)
#define GPIO_ADC1_IN12_0    (GPIO_ANALOG | GPIO_PORTC | GPIO_PIN5)
#define GPIO_ADC1_IN13_0    (GPIO_ANALOG | GPIO_PORTA | GPIO_PIN13)
#define GPIO_ADC1_IN14_0    (GPIO_ANALOG | GPIO_PORTA | GPIO_PIN14)

#define GPIO_ADC1_IN17_0    (GPIO_ANALOG | GPIO_PORTB | GPIO_PIN0)
#define GPIO_ADC1_IN18_0    (GPIO_ANALOG | GPIO_PORTB | GPIO_PIN1)
#define GPIO_ADC1_IN19_0    (GPIO_ANALOG | GPIO_PORTB | GPIO_PIN2)
#define GPIO_ADC1_IN20_0    (GPIO_ANALOG | GPIO_PORTB | GPIO_PIN10)
#define GPIO_ADC1_IN21_0    (GPIO_ANALOG | GPIO_PORTB | GPIO_PIN11)
#define GPIO_ADC1_IN22_0    (GPIO_ANALOG | GPIO_PORTB | GPIO_PIN12)

/* USART */

#define GPIO_USART1_CTS_1    (GPIO_ALT | GPIO_AF1 | GPIO_PORTA | GPIO_PIN11)
#define GPIO_USART1_CTS_2    (GPIO_ALT | GPIO_AF4 | GPIO_PORTB | GPIO_PIN4)
#define GPIO_USART1_RTS_1    (GPIO_ALT | GPIO_AF1 | GPIO_PORTA | GPIO_PIN12)
#define GPIO_USART1_RTS_2    (GPIO_ALT | GPIO_AF4 | GPIO_PORTB | GPIO_PIN3)
#define GPIO_USART1_RX_1     (GPIO_ALT | GPIO_PULLUP | GPIO_AF1 | GPIO_PUSHPULL | GPIO_PORTA | GPIO_PIN10)
#define GPIO_USART1_RX_2     (GPIO_ALT | GPIO_PULLUP | GPIO_AF0 | GPIO_PUSHPULL | GPIO_PORTB | GPIO_PIN7)
#define GPIO_USART1_RX_3     (GPIO_ALT | GPIO_PULLUP | GPIO_AF1 | GPIO_PUSHPULL | GPIO_PORTC | GPIO_PIN5)
#define GPIO_USART1_TX_1     (GPIO_ALT | GPIO_PULLUP | GPIO_AF1 | GPIO_PUSHPULL | GPIO_PORTA | GPIO_PIN9)
#define GPIO_USART1_TX_2     (GPIO_ALT | GPIO_PULLUP | GPIO_AF0 | GPIO_PUSHPULL | GPIO_PORTB | GPIO_PIN6)
#define GPIO_USART1_TX_3     (GPIO_ALT | GPIO_PULLUP | GPIO_AF1 | GPIO_PUSHPULL | GPIO_PORTC | GPIO_PIN4)

#define GPIO_USART2_CTS_1   (GPIO_ALT | GPIO_AF1 | GPIO_PORTA | GPIO_PIN0)
#define GPIO_USART2_CTS_2   (GPIO_ALT | GPIO_AF0 | GPIO_PORTD | GPIO_PIN3)
#define GPIO_USART2_RTS_1   (GPIO_ALT | GPIO_AF1 | GPIO_PORTA | GPIO_PIN1)
#define GPIO_USART2_RTS_2   (GPIO_ALT | GPIO_AF0 | GPIO_PORTD | GPIO_PIN4)
#define GPIO_USART2_RX_1    (GPIO_ALT | GPIO_PULLUP | GPIO_AF1 | GPIO_PUSHPULL | GPIO_PORTA | GPIO_PIN3)
#define GPIO_USART2_RX_2    (GPIO_ALT | GPIO_PULLUP | GPIO_AF1 | GPIO_PUSHPULL | GPIO_PORTA | GPIO_PIN15)
#define GPIO_USART2_RX_3    (GPIO_ALT | GPIO_PULLUP | GPIO_AF1 | GPIO_PUSHPULL | GPIO_PORTD | GPIO_PIN6)
#define GPIO_USART2_TX_1    (GPIO_ALT | GPIO_PULLUP | GPIO_AF1 | GPIO_PUSHPULL | GPIO_PORTA | GPIO_PIN2)
#define GPIO_USART2_TX_2    (GPIO_ALT | GPIO_PULLUP | GPIO_AF1 | GPIO_PUSHPULL | GPIO_PORTA | GPIO_PIN14)
#define GPIO_USART2_TX_3    (GPIO_ALT | GPIO_PULLUP | GPIO_AF1 | GPIO_PUSHPULL | GPIO_PORTD | GPIO_PIN6)

/* FDCAN1 */

#define GPIO_FDCAN1_RX_1               (GPIO_ALT | GPIO_AF4 | GPIO_PUSHPULL | GPIO_PORTA | GPIO_PIN11)
#define GPIO_FDCAN1_RX_2               (GPIO_ALT | GPIO_AF3 | GPIO_PUSHPULL | GPIO_PORTB | GPIO_PIN0)
#define GPIO_FDCAN1_RX_3               (GPIO_ALT | GPIO_AF4 | GPIO_PUSHPULL | GPIO_PORTB | GPIO_PIN5)
#define GPIO_FDCAN1_RX_4               (GPIO_ALT | GPIO_AF4 | GPIO_PUSHPULL | GPIO_PORTB | GPIO_PIN12)
#define GPIO_FDCAN1_RX_5               (GPIO_ALT | GPIO_AF8 | GPIO_PUSHPULL | GPIO_PORTB | GPIO_PIN8)
#define GPIO_FDCAN1_RX_6               (GPIO_ALT | GPIO_AF4 | GPIO_PUSHPULL | GPIO_PORTC | GPIO_PIN2)
#define GPIO_FDCAN1_RX_7               (GPIO_ALT | GPIO_AF4 | GPIO_PUSHPULL | GPIO_PORTC | GPIO_PIN4)
#define GPIO_FDCAN1_RX_8               (GPIO_ALT | GPIO_AF4 | GPIO_PUSHPULL | GPIO_PORTD | GPIO_PIN0)

#define GPIO_FDCAN1_TX_1               (GPIO_ALT | GPIO_AF4 | GPIO_PUSHPULL | GPIO_PORTA | GPIO_PIN12)
#define GPIO_FDCAN1_TX_2               (GPIO_ALT | GPIO_AF3 | GPIO_PUSHPULL | GPIO_PORTB | GPIO_PIN1)
#define GPIO_FDCAN1_TX_3               (GPIO_ALT | GPIO_AF4 | GPIO_PUSHPULL | GPIO_PORTB | GPIO_PIN13)
#define GPIO_FDCAN1_TX_4               (GPIO_ALT | GPIO_AF15 | GPIO_PUSHPULL | GPIO_PORTB | GPIO_PIN6)
#define GPIO_FDCAN1_TX_5               (GPIO_ALT | GPIO_AF8 | GPIO_PUSHPULL | GPIO_PORTB | GPIO_PIN9)
#define GPIO_FDCAN1_TX_6               (GPIO_ALT | GPIO_AF4 | GPIO_PUSHPULL | GPIO_PORTC | GPIO_PIN3)
#define GPIO_FDCAN1_TX_7               (GPIO_ALT | GPIO_AF4 | GPIO_PUSHPULL | GPIO_PORTC | GPIO_PIN5)
#define GPIO_FDCAN1_TX_8               (GPIO_ALT | GPIO_AF13 | GPIO_PUSHPULL | GPIO_PORTC | GPIO_PIN14)
#define GPIO_FDCAN1_TX_9               (GPIO_ALT | GPIO_AF4 | GPIO_PUSHPULL | GPIO_PORTD | GPIO_PIN1)

/* TODO: missing pinmaps */

#endif /* __ARCH_ARM_SRC_STM32F0L0G0_HARDWARE_STM32C0_PINMAP_H */
