/****************************************************************************
 * arch/xtensa/src/esp32s2/esp32s2_config.h
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

#ifndef __ARCH_XTENSA_SRC_ESP32S2_ESP32S2_CONFIG_H
#define __ARCH_XTENSA_SRC_ESP32S2_ESP32S2_CONFIG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <arch/chip/chip.h>
#include <arch/board/board.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* UARTs ********************************************************************/

/* Are any UARTs enabled? */

#undef HAVE_UART_DEVICE
#ifdef CONFIG_ESP32S2_UART
#  define HAVE_UART_DEVICE 1
#endif

/* Serial Console ***********************************************************/

/* Is there a serial console?  There should be no more than one defined.  It
 * could be on any UARTn. n E {0,1}
 */

#undef HAVE_SERIAL_CONSOLE
#if defined(CONFIG_UART0_SERIAL_CONSOLE) && defined(CONFIG_ESP32S2_UART0)
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  define HAVE_SERIAL_CONSOLE 1
#elif defined(CONFIG_UART1_SERIAL_CONSOLE) && defined(CONFIG_ESP32S2_UART1)
#  undef CONFIG_UART0_SERIAL_CONSOLE
#  define HAVE_SERIAL_CONSOLE 1
#else
#  undef CONFIG_UART0_SERIAL_CONSOLE
#  undef CONFIG_UART1_SERIAL_CONSOLE
#endif

/* SPI **********************************************************************/

/* Don't enable SPI peripherals not supported by the chip. */

#if ESP32S2_NSPI < 1
#  undef CONFIG_ESP32S2_SPI0
#  undef CONFIG_ESP32S2_SPI1
#  undef CONFIG_ESP32S2_SPI2
#  undef CONFIG_ESP32S2_SPI3
#elif ESP32S2_NSPI < 2
#  undef CONFIG_ESP32S2_SPI1
#  undef CONFIG_ESP32S2_SPI2
#  undef CONFIG_ESP32S2_SPI3
#elif ESP32S2_NSPI < 3
#  undef CONFIG_ESP32S2_SPI2
#  undef CONFIG_ESP32S2_SPI3
#elif ESP32S2_NSPI < 4
#  undef CONFIG_ESP32S2_SPI3
#endif

#endif /* __ARCH_XTENSA_SRC_ESP32S2_ESP32S2_CONFIG_H */
