/****************************************************************************
 * arch/arm/src/efm32/efm32_config.h
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_EFM32_EFM32_CONFIG_H
#define __ARCH_ARM_SRC_EFM32_EFM32_CONFIG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/
/* Make sure that the configuration does not enable UARTs that the MCU does
 * not have.
 */

#ifndef CONFIG_EFM32_HAVE_USART2
#  undef CONFIG_EFM32_USART2
#endif

#ifndef CONFIG_EFM32_HAVE_UART0
#  undef CONFIG_EFM32_UART0
#endif

#ifndef CONFIG_EFM32_HAVE_UART1
#  undef CONFIG_EFM32_UART1
#endif

#ifndef CONFIG_EFM32_HAVE_LEUART1
#  undef CONFIG_EFM32_LEUART1
#endif

/* Is there a UART device?  Or an SPI device? */

#ifndef CONFIG_EFM32_USART_ISUART
#  undef CONFIG_EFM32_USART0_ISUART
#  undef CONFIG_EFM32_USART1_ISUART
#  undef CONFIG_EFM32_USART2_ISUART
#endif

#ifndef CONFIG_EFM32_UART
#  undef CONFIG_EFM32_UART0
#  undef CONFIG_EFM32_UART1
#endif

#ifndef CONFIG_EFM32_USART0
#  undef CONFIG_EFM32_USART0_ISUART
#  undef CONFIG_EFM32_USART0_ISSPI
#endif

#ifndef CONFIG_EFM32_USART1
#  undef CONFIG_EFM32_USART1_ISUART
#  undef CONFIG_EFM32_USART1_ISSPI
#endif

#ifndef CONFIG_EFM32_USART2
#  undef CONFIG_EFM32_USART2_ISUART
#  undef CONFIG_EFM32_USART2_ISSPI
#endif

#undef HAVE_UART_DEVICE
#if defined(CONFIG_EFM32_USART0_ISUART) || \
    defined(CONFIG_EFM32_USART1_ISUART) || \
    defined(CONFIG_EFM32_USART2_ISUART) ||  \
    defined(CONFIG_EFM32_UART0) || \
    defined(CONFIG_EFM32_UART1)
#  define HAVE_UART_DEVICE 1
#endif

#undef HAVE_SPI_DEVICE
#if defined(CONFIG_EFM32_USART0_ISSPI) || \
    defined(CONFIG_EFM32_USART1_ISSPI) || \
    defined(CONFIG_EFM32_USART2_ISSPI)
#  define HAVE_SPI_DEVICE 1
#endif

/* Is there an LEUART device?  */

#ifndef CONFIG_EFM32_LEUART
#  undef CONFIG_EFM32_LEUART0
#  undef CONFIG_EFM32_LEUART1
#endif

#undef HAVE_LEUART_DEVICE
#if defined(CONFIG_EFM32_LEUART0) ||  defined(CONFIG_EFM32_LEUART1)
#  define HAVE_LEUART_DEVICE 1
#endif

/* Is there a serial console? (Low energy UARTs not yet supported) */

#undef HAVE_UART_CONSOLE
#undef HAVE_LEUART_CONSOLE

#if defined(CONFIG_CONSOLE_SYSLOG)
#  undef CONFIG_USART1_SERIAL_CONSOLE
#  undef CONFIG_USART2_SERIAL_CONSOLE
#  undef CONFIG_UART0_SERIAL_CONSOLE
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  undef CONFIG_LEUART0_SERIAL_CONSOLE
#  undef CONFIG_LEUART1_SERIAL_CONSOLE
#else
#  if defined(CONFIG_USART0_SERIAL_CONSOLE) && defined(CONFIG_EFM32_USART0_ISUART)
#    undef CONFIG_USART1_SERIAL_CONSOLE
#    undef CONFIG_USART2_SERIAL_CONSOLE
#    undef CONFIG_UART0_SERIAL_CONSOLE
#    undef CONFIG_UART1_SERIAL_CONSOLE
#    undef CONFIG_LEUART0_SERIAL_CONSOLE
#    undef CONFIG_LEUART1_SERIAL_CONSOLE
#    define HAVE_UART_CONSOLE 1
#  elif defined(CONFIG_USART1_SERIAL_CONSOLE) && defined(CONFIG_EFM32_USART1_ISUART)
#    undef CONFIG_USART0_SERIAL_CONSOLE
#    undef CONFIG_USART2_SERIAL_CONSOLE
#    undef CONFIG_UART0_SERIAL_CONSOLE
#    undef CONFIG_UART1_SERIAL_CONSOLE
#    undef CONFIG_LEUART0_SERIAL_CONSOLE
#    undef CONFIG_LEUART1_SERIAL_CONSOLE
#    define HAVE_UART_CONSOLE 1
#  elif defined(CONFIG_USART2_SERIAL_CONSOLE) && defined(CONFIG_EFM32_USART2_ISUART)
#    undef CONFIG_USART0_SERIAL_CONSOLE
#    undef CONFIG_USART1_SERIAL_CONSOLE
#    undef CONFIG_UART0_SERIAL_CONSOLE
#    undef CONFIG_UART1_SERIAL_CONSOLE
#    undef CONFIG_LEUART0_SERIAL_CONSOLE
#    undef CONFIG_LEUART1_SERIAL_CONSOLE
#    define HAVE_UART_CONSOLE 1
#  elif defined(CONFIG_UART0_SERIAL_CONSOLE) && defined(CONFIG_EFM32_UART0)
#    undef CONFIG_USART0_SERIAL_CONSOLE
#    undef CONFIG_USART1_SERIAL_CONSOLE
#    undef CONFIG_USART2_SERIAL_CONSOLE
#    undef CONFIG_UART1_SERIAL_CONSOLE
#    undef CONFIG_LEUART0_SERIAL_CONSOLE
#    undef CONFIG_LEUART1_SERIAL_CONSOLE
#    define HAVE_UART_CONSOLE 1
#  elif defined(CONFIG_UART1_SERIAL_CONSOLE) && defined(CONFIG_EFM32_UART1)
#    undef CONFIG_USART0_SERIAL_CONSOLE
#    undef CONFIG_USART1_SERIAL_CONSOLE
#    undef CONFIG_USART2_SERIAL_CONSOLE
#    undef CONFIG_UART0_SERIAL_CONSOLE
#    undef CONFIG_LEUART0_SERIAL_CONSOLE
#    undef CONFIG_LEUART1_SERIAL_CONSOLE
#    define HAVE_UART_CONSOLE 1
#  elif defined(CONFIG_LEUART0_SERIAL_CONSOLE) && defined(CONFIG_EFM32_LEUART0)
#    undef CONFIG_USART0_SERIAL_CONSOLE
#    undef CONFIG_USART1_SERIAL_CONSOLE
#    undef CONFIG_USART2_SERIAL_CONSOLE
#    undef CONFIG_UART0_SERIAL_CONSOLE
#    undef CONFIG_UART1_SERIAL_CONSOLE
#    undef CONFIG_LEUART1_SERIAL_CONSOLE
#    define HAVE_LEUART_CONSOLE 1
#  elif defined(CONFIG_LEUART1_SERIAL_CONSOLE) && defined(CONFIG_EFM32_LEUART1)
#    undef CONFIG_USART0_SERIAL_CONSOLE
#    undef CONFIG_USART1_SERIAL_CONSOLE
#    undef CONFIG_USART2_SERIAL_CONSOLE
#    undef CONFIG_UART0_SERIAL_CONSOLE
#    undef CONFIG_UART1_SERIAL_CONSOLE
#    undef CONFIG_LEUART0_SERIAL_CONSOLE
#    define HAVE_LEUART_CONSOLE 1
#  else
#    ifdef CONFIG_DEV_CONSOLE
#      warning "No valid CONFIG_U[S]ART[n]_SERIAL_CONSOLE Setting"
#    endif
#    undef CONFIG_USART0_SERIAL_CONSOLE
#    undef CONFIG_USART1_SERIAL_CONSOLE
#    undef CONFIG_USART2_SERIAL_CONSOLE
#    undef CONFIG_UART0_SERIAL_CONSOLE
#    undef CONFIG_UART1_SERIAL_CONSOLE
#    undef CONFIG_LEUART0_SERIAL_CONSOLE
#    undef CONFIG_LEUART1_SERIAL_CONSOLE
#  endif
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Data
 ****************************************************************************/

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
#endif /* __ARCH_ARM_SRC_EFM32_EFM32_CONFIG_H */
