/****************************************************************************
 * arch/arm/src/efm32/efm32_config.h
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
