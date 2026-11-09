/****************************************************************************
 * arch/arm/src/stm32n6/stm32_uart.h
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

#ifndef __ARCH_ARM_SRC_STM32N6_STM32_UART_H
#define __ARCH_ARM_SRC_STM32N6_STM32_UART_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/serial/serial.h>

#include "chip.h"

#include "hardware/stm32n6xxx_uart.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Sanity checks */

#if !defined(CONFIG_STM32_LPUART1)
#  undef CONFIG_STM32_LPUART1_SERIALDRIVER
#  undef CONFIG_STM32_LPUART1_1WIREDRIVER
#endif

#if !defined(CONFIG_STM32_USART1)
#  undef CONFIG_STM32_USART1_SERIALDRIVER
#  undef CONFIG_STM32_USART1_1WIREDRIVER
#endif

#if !defined(CONFIG_STM32_USART2)
#  undef CONFIG_STM32_USART2_SERIALDRIVER
#  undef CONFIG_STM32_USART2_1WIREDRIVER
#endif

#if !defined(CONFIG_STM32_USART3)
#  undef CONFIG_STM32_USART3_SERIALDRIVER
#  undef CONFIG_STM32_USART3_1WIREDRIVER
#endif

#if !defined(CONFIG_STM32_UART4)
#  undef CONFIG_STM32_UART4_SERIALDRIVER
#  undef CONFIG_STM32_UART4_1WIREDRIVER
#endif

#if !defined(CONFIG_STM32_UART5)
#  undef CONFIG_STM32_UART5_SERIALDRIVER
#  undef CONFIG_STM32_UART5_1WIREDRIVER
#endif

#if !defined(CONFIG_STM32_USART6)
#  undef CONFIG_STM32_USART6_SERIALDRIVER
#  undef CONFIG_STM32_USART6_1WIREDRIVER
#endif

#if !defined(CONFIG_STM32_UART7)
#  undef CONFIG_STM32_UART7_SERIALDRIVER
#  undef CONFIG_STM32_UART7_1WIREDRIVER
#endif

#if !defined(CONFIG_STM32_UART8)
#  undef CONFIG_STM32_UART8_SERIALDRIVER
#  undef CONFIG_STM32_UART8_1WIREDRIVER
#endif

#if !defined(CONFIG_STM32_UART9)
#  undef CONFIG_STM32_UART9_SERIALDRIVER
#  undef CONFIG_STM32_UART9_1WIREDRIVER
#endif

#if !defined(CONFIG_STM32_USART10)
#  undef CONFIG_STM32_USART10_SERIALDRIVER
#  undef CONFIG_STM32_USART10_1WIREDRIVER
#endif

/* Is there a USART enabled? */

#if defined(CONFIG_STM32_LPUART1_SERIALDRIVER) || \
    defined(CONFIG_STM32_USART1_SERIALDRIVER) || \
    defined(CONFIG_STM32_USART2_SERIALDRIVER) || \
    defined(CONFIG_STM32_USART3_SERIALDRIVER) || \
    defined(CONFIG_STM32_UART4_SERIALDRIVER) || \
    defined(CONFIG_STM32_UART5_SERIALDRIVER) || \
    defined(CONFIG_STM32_USART6_SERIALDRIVER) || \
    defined(CONFIG_STM32_UART7_SERIALDRIVER) || \
    defined(CONFIG_STM32_UART8_SERIALDRIVER) || \
    defined(CONFIG_STM32_UART9_SERIALDRIVER) || \
    defined(CONFIG_STM32_USART10_SERIALDRIVER)
#  define HAVE_UART 1
#endif

/* Is there a serial console?  CONSOLE_UART is the 1-based index of the
 * console in the driver port table.
 */

#if defined(CONFIG_LPUART1_SERIAL_CONSOLE) && \
    defined(CONFIG_STM32_LPUART1_SERIALDRIVER)
#  define CONSOLE_UART 1
#  define HAVE_CONSOLE 1
#elif defined(CONFIG_USART1_SERIAL_CONSOLE) && \
    defined(CONFIG_STM32_USART1_SERIALDRIVER)
#  define CONSOLE_UART 2
#  define HAVE_CONSOLE 1
#elif defined(CONFIG_USART2_SERIAL_CONSOLE) && \
    defined(CONFIG_STM32_USART2_SERIALDRIVER)
#  define CONSOLE_UART 3
#  define HAVE_CONSOLE 1
#elif defined(CONFIG_USART3_SERIAL_CONSOLE) && \
    defined(CONFIG_STM32_USART3_SERIALDRIVER)
#  define CONSOLE_UART 4
#  define HAVE_CONSOLE 1
#elif defined(CONFIG_UART4_SERIAL_CONSOLE) && \
    defined(CONFIG_STM32_UART4_SERIALDRIVER)
#  define CONSOLE_UART 5
#  define HAVE_CONSOLE 1
#elif defined(CONFIG_UART5_SERIAL_CONSOLE) && \
    defined(CONFIG_STM32_UART5_SERIALDRIVER)
#  define CONSOLE_UART 6
#  define HAVE_CONSOLE 1
#elif defined(CONFIG_USART6_SERIAL_CONSOLE) && \
    defined(CONFIG_STM32_USART6_SERIALDRIVER)
#  define CONSOLE_UART 7
#  define HAVE_CONSOLE 1
#elif defined(CONFIG_UART7_SERIAL_CONSOLE) && \
    defined(CONFIG_STM32_UART7_SERIALDRIVER)
#  define CONSOLE_UART 8
#  define HAVE_CONSOLE 1
#elif defined(CONFIG_UART8_SERIAL_CONSOLE) && \
    defined(CONFIG_STM32_UART8_SERIALDRIVER)
#  define CONSOLE_UART 9
#  define HAVE_CONSOLE 1
#elif defined(CONFIG_UART9_SERIAL_CONSOLE) && \
    defined(CONFIG_STM32_UART9_SERIALDRIVER)
#  define CONSOLE_UART 10
#  define HAVE_CONSOLE 1
#elif defined(CONFIG_USART10_SERIAL_CONSOLE) && \
    defined(CONFIG_STM32_USART10_SERIALDRIVER)
#  define CONSOLE_UART 11
#  define HAVE_CONSOLE 1
#else
#  define CONSOLE_UART 0
#  undef HAVE_CONSOLE
#endif

#define USART_CR1_USED_INTS    (USART_CR1_RXNEIE | USART_CR1_TXEIE | USART_CR1_PEIE)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

#if defined(__cplusplus)
extern "C"
{
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_STM32N6_STM32_UART_H */
