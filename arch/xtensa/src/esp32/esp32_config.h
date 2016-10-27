/****************************************************************************
 * arch/xtensa/src/esp32/esp32_config.h
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

/* SPI ******************************************************************************/
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
