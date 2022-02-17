/****************************************************************************
 * arch/xtensa/src/esp32/esp32_config.h
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

#ifndef __ARCH_XTENSA_SRC_ESP32_ESP32_CONFIG_H
#define __ARCH_XTENSA_SRC_ESP32_ESP32_CONFIG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <arch/chip/chip.h>
#include <arch/board/board.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* GPIO IRQs ****************************************************************/

/* UARTs ********************************************************************/

/* Don't enable UARTs not supported by the chip. */

#if ESP32_NUARTS < 1
#  undef CONFIG_ESP32_UART0
#  undef CONFIG_ESP32_UART1
#  undef CONFIG_ESP32_UART2
#elif ESP32_NUARTS < 2
#  undef CONFIG_ESP32_UART1
#  undef CONFIG_ESP32_UART2
#elif ESP32_NUARTS < 3
#  undef CONFIG_ESP32_UART2
#endif

/* Are any UARTs enabled? */

#undef HAVE_UART_DEVICE
#if defined(CONFIG_ESP32_UART0) || defined(CONFIG_ESP32_UART1) || \
    defined(CONFIG_ESP32_UART2)
#  define HAVE_UART_DEVICE 1
#endif

/* Is RS-485 used? */

#if defined(CONFIG_ESP32_UART0_RS485) || \
    defined(CONFIG_ESP32_UART1_RS485) || \
    defined(CONFIG_ESP32_UART2_RS485)
#  define HAVE_RS485 1
#endif

/* UART Flow Control ********************************************************/

#ifndef CONFIG_ESP32_UART0
#  undef CONFIG_UART0_IFLOWCONTROL
#endif
#ifndef CONFIG_ESP32_UART1
#  undef CONFIG_UART1_IFLOWCONTROL
#endif
#ifndef CONFIG_ESP32_UART2
#  undef CONFIG_UART2_IFLOWCONTROL
#endif

/* Serial Console ***********************************************************/

/* Is there a serial console?  There should be no more than one defined.  It
 * could be on any UARTn, n=1,..,ESP32_NUART, or USARTn, n=1,.., ESP32_NUSART
 */

#undef HAVE_SERIAL_CONSOLE
#if defined(CONFIG_UART0_SERIAL_CONSOLE) && defined(CONFIG_ESP32_UART0)
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  undef CONFIG_UART2_SERIAL_CONSOLE
#  define HAVE_SERIAL_CONSOLE 1
#elif defined(CONFIG_UART1_SERIAL_CONSOLE) && defined(CONFIG_ESP32_UART1)
#  undef CONFIG_UART0_SERIAL_CONSOLE
#  undef CONFIG_UART2_SERIAL_CONSOLE
#  define HAVE_SERIAL_CONSOLE 1
#elif defined(CONFIG_UART2_SERIAL_CONSOLE) && defined(CONFIG_ESP32_UART2)
#  undef CONFIG_UART0_SERIAL_CONSOLE
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  define HAVE_SERIAL_CONSOLE 1
#else
#  undef CONFIG_UART0_SERIAL_CONSOLE
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  undef CONFIG_UART2_SERIAL_CONSOLE
#endif

/* SPI **********************************************************************/

/* Don't enable SPI peripherals not supported by the chip. */

#if ESP32_NSPI < 1
#  undef CONFIG_ESP32_SPI0
#  undef CONFIG_ESP32_SPI1
#  undef CONFIG_ESP32_SPI2
#  undef CONFIG_ESP32_SPI3
#elif ESP32_NSPI < 2
#  undef CONFIG_ESP32_SPI1
#  undef CONFIG_ESP32_SPI2
#  undef CONFIG_ESP32_SPI3
#elif ESP32_NSPI < 3
#  undef CONFIG_ESP32_SPI2
#  undef CONFIG_ESP32_SPI3
#elif ESP32_NSPI < 4
#  undef CONFIG_ESP32_SPI3
#endif

#endif /* __ARCH_XTENSA_SRC_ESP32_ESP32_CONFIG_H */
