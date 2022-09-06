/****************************************************************************
 * arch/arm/src/gd32f4/gd32f4xx_usart.h
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

#ifndef __ARCH_ARM_GD32F4_GD32F4XX_USART_H
#define __ARCH_ARM_GD32F4_GD32F4XX_USART_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/serial/serial.h>

#include "chip.h"

#if defined(CONFIG_GD32F4_GD32F4XX)
#  include "hardware/gd32f4xx_uart.h"
#else
#  error "Unsupported GD32 UART"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Make sure that we have not enabled more U[S]ARTs than are supported by the
 * device.
 */

#if GD32_NUSART < 8 || !defined(CONFIG_GD32F4_HAVE_UART7)
#  undef CONFIG_GD32F4_UART7
#endif
#if GD32_NUSART < 7 || !defined(CONFIG_GD32F4_HAVE_UART6)
#  undef CONFIG_GD32F4_UART6
#endif

#if GD32_NUSART < 6
#  undef CONFIG_GD32F4_USART5
#endif
#if GD32_NUSART < 5
#  undef CONFIG_GD32F4_UART4
#endif
#if GD32_NUSART < 4
#  undef CONFIG_GD32F4_UART3
#endif
#if GD32_NUSART < 3
#  undef CONFIG_GD32F4_USART2
#endif
#if GD32_NUSART < 2
#  undef CONFIG_GD32F4_USART1
#endif
#if GD32_NUSART < 1
#  undef CONFIG_GD32F4_USART0
#endif

/* Sanity checks */

#if !defined(CONFIG_GD32F4_USART0)
#  undef CONFIG_GD32F4_USART0_SERIALDRIVER
#  undef CONFIG_GD32F4_USART0_1WIREDRIVER
#endif
#if !defined(CONFIG_GD32F4_USART1)
#  undef CONFIG_GD32F4_USART1_SERIALDRIVER
#  undef CONFIG_GD32F4_USART1_1WIREDRIVER
#endif
#if !defined(CONFIG_GD32F4_USART2)
#  undef CONFIG_GD32F4_USART2_SERIALDRIVER
#  undef CONFIG_GD32F4_USART2_1WIREDRIVER
#endif
#if !defined(CONFIG_GD32F4_UART3)
#  undef CONFIG_GD32F4_UART3_SERIALDRIVER
#  undef CONFIG_GD32F4_UART3_1WIREDRIVER
#endif
#if !defined(CONFIG_GD32F4_UART4)
#  undef CONFIG_GD32F4_UART4_SERIALDRIVER
#  undef CONFIG_GD32F4_UART4_1WIREDRIVER
#endif
#if !defined(CONFIG_GD32F4_USART5)
#  undef CONFIG_GD32F4_USART5_SERIALDRIVER
#  undef CONFIG_GD32F4_USART5_1WIREDRIVER
#endif
#if !defined(CONFIG_GD32F4_UART6)
#  undef CONFIG_GD32F4_UART6_SERIALDRIVER
#  undef CONFIG_GD32F4_UART6_1WIREDRIVER
#endif
#if !defined(CONFIG_GD32F4_UART7)
#  undef CONFIG_GD32F4_UART7_SERIALDRIVER
#  undef CONFIG_GD32F4_UART7_1WIREDRIVER
#endif

/* Check 1-Wire and U(S)ART conflicts */

#if defined(CONFIG_GD32F4_USART0_1WIREDRIVER) && defined(CONFIG_GD32F4_USART0_SERIALDRIVER)
#  error "Both CONFIG_GD32F4_USART0_1WIREDRIVER and CONFIG_GD32F4_USART0_SERIALDRIVER defined"
#  undef CONFIG_GD32F4_USART0_1WIREDRIVER
#endif
#if defined(CONFIG_GD32F4_USART1_1WIREDRIVER) && defined(CONFIG_GD32F4_USART1_SERIALDRIVER)
#  error "Both CONFIG_GD32F4_USART1_1WIREDRIVER and CONFIG_GD32F4_USART1_SERIALDRIVER defined"
#  undef CONFIG_GD32F4_USART1_1WIREDRIVER
#endif
#if defined(CONFIG_GD32F4_USART2_1WIREDRIVER) && defined(CONFIG_GD32F4_USART2_SERIALDRIVER)
#  error "Both CONFIG_GD32F4_USART2_1WIREDRIVER and CONFIG_GD32F4_USART2_SERIALDRIVER defined"
#  undef CONFIG_GD32F4_USART2_1WIREDRIVER
#endif
#if defined(CONFIG_GD32F4_UART3_1WIREDRIVER) && defined(CONFIG_GD32F4_UART3_SERIALDRIVER)
#  error "Both CONFIG_GD32F4_UART3_1WIREDRIVER and CONFIG_GD32F4_UART3_SERIALDRIVER defined"
#  undef CONFIG_GD32F4_UART3_1WIREDRIVER
#endif
#if defined(CONFIG_GD32F4_UART4_1WIREDRIVER) && defined(CONFIG_GD32F4_UART4_SERIALDRIVER)
#  error "Both CONFIG_GD32F4_UART4_1WIREDRIVER and CONFIG_GD32F4_UART4_SERIALDRIVER defined"
#  undef CONFIG_GD32F4_UART4_1WIREDRIVER
#endif
#if defined(CONFIG_GD32F4_USART5_1WIREDRIVER) && defined(CONFIG_GD32F4_USART5_SERIALDRIVER)
#  error "Both CONFIG_GD32F4_USART5_1WIREDRIVER and CONFIG_GD32F4_USART5_SERIALDRIVER defined"
#  undef CONFIG_GD32F4_USART5_1WIREDRIVER
#endif
#if defined(CONFIG_GD32F4_UART6_1WIREDRIVER) && defined(CONFIG_GD32F4_UART6_SERIALDRIVER)
#  error "Both CONFIG_GD32F4_UART6_1WIREDRIVER and CONFIG_GD32F4_UART6_SERIALDRIVER defined"
#  undef CONFIG_GD32F4_UART6_1WIREDRIVER
#endif
#if defined(CONFIG_GD32F4_UART7_1WIREDRIVER) && defined(CONFIG_GD32F4_UART7_SERIALDRIVER)
#  error "Both CONFIG_GD32F4_UART7_1WIREDRIVER and CONFIG_GD32F4_UART7_SERIALDRIVER defined"
#  undef CONFIG_GD32F4_UART7_1WIREDRIVER
#endif

/* Is the serial driver enabled? */

#if defined(CONFIG_GD32F4_USART0_SERIALDRIVER) || defined(CONFIG_GD32F4_USART1_SERIALDRIVER) || \
    defined(CONFIG_GD32F4_USART2_SERIALDRIVER) || defined(CONFIG_GD32F4_UART3_SERIALDRIVER)  || \
    defined(CONFIG_GD32F4_UART4_SERIALDRIVER)  || defined(CONFIG_GD32F4_USART5_SERIALDRIVER) || \
    defined(CONFIG_GD32F4_UART6_SERIALDRIVER)  || defined(CONFIG_GD32F4_UART7_SERIALDRIVER)
#  define HAVE_SERIALDRIVER 1
#endif

/* Is the 1-Wire driver? */

#if defined(CONFIG_GD32F4_USART0_1WIREDRIVER) || defined(CONFIG_GD32F4_USART1_1WIREDRIVER) || \
    defined(CONFIG_GD32F4_USART2_1WIREDRIVER) || defined(CONFIG_GD32F4_UART3_1WIREDRIVER) || \
    defined(CONFIG_GD32F4_UART4_1WIREDRIVER) || defined(CONFIG_GD32F4_USART5_1WIREDRIVER) || \
    defined(CONFIG_GD32F4_UART6_1WIREDRIVER) || defined(CONFIG_GD32F4_UART7_1WIREDRIVER)
#  define HAVE_1WIREDRIVER 1
#endif

/* Is there a serial console? */

#if defined(CONFIG_USART0_SERIAL_CONSOLE) && defined(CONFIG_GD32F4_USART0_SERIALDRIVER)
#  undef CONFIG_USART1_SERIAL_CONSOLE
#  undef CONFIG_USART2_SERIAL_CONSOLE
#  undef CONFIG_UART3_SERIAL_CONSOLE
#  undef CONFIG_UART4_SERIAL_CONSOLE
#  undef CONFIG_USART5_SERIAL_CONSOLE
#  undef CONFIG_UART6_SERIAL_CONSOLE
#  undef CONFIG_UART7_SERIAL_CONSOLE
#  define CONSOLE_UART 0
#  define HAVE_CONSOLE 1
#elif defined(CONFIG_USART1_SERIAL_CONSOLE) && defined(CONFIG_GD32F4_USART1_SERIALDRIVER)
#  undef CONFIG_USART0_SERIAL_CONSOLE
#  undef CONFIG_USART2_SERIAL_CONSOLE
#  undef CONFIG_UART3_SERIAL_CONSOLE
#  undef CONFIG_UART4_SERIAL_CONSOLE
#  undef CONFIG_USART5_SERIAL_CONSOLE
#  undef CONFIG_UART6_SERIAL_CONSOLE
#  undef CONFIG_UART7_SERIAL_CONSOLE
#  define CONSOLE_UART 1
#  define HAVE_CONSOLE 1
#elif defined(CONFIG_USART2_SERIAL_CONSOLE) && defined(CONFIG_GD32F4_USART2_SERIALDRIVER)
#  undef CONFIG_USART0_SERIAL_CONSOLE
#  undef CONFIG_USART1_SERIAL_CONSOLE
#  undef CONFIG_UART3_SERIAL_CONSOLE
#  undef CONFIG_UART4_SERIAL_CONSOLE
#  undef CONFIG_USART5_SERIAL_CONSOLE
#  undef CONFIG_UART6_SERIAL_CONSOLE
#  undef CONFIG_UART7_SERIAL_CONSOLE
#  define CONSOLE_UART 2
#  define HAVE_CONSOLE 1
#elif defined(CONFIG_UART3_SERIAL_CONSOLE) && defined(CONFIG_GD32F4_UART3_SERIALDRIVER)
#  undef CONFIG_USART0_SERIAL_CONSOLE
#  undef CONFIG_USART1_SERIAL_CONSOLE
#  undef CONFIG_USART2_SERIAL_CONSOLE
#  undef CONFIG_UART4_SERIAL_CONSOLE
#  undef CONFIG_USART5_SERIAL_CONSOLE
#  undef CONFIG_UART6_SERIAL_CONSOLE
#  undef CONFIG_UART7_SERIAL_CONSOLE
#  define CONSOLE_UART 3
#  define HAVE_CONSOLE 1
#elif defined(CONFIG_UART4_SERIAL_CONSOLE) && defined(CONFIG_GD32F4_UART4_SERIALDRIVER)
#  undef CONFIG_USART0_SERIAL_CONSOLE
#  undef CONFIG_USART1_SERIAL_CONSOLE
#  undef CONFIG_USART2_SERIAL_CONSOLE
#  undef CONFIG_UART3_SERIAL_CONSOLE
#  undef CONFIG_USART5_SERIAL_CONSOLE
#  undef CONFIG_UART6_SERIAL_CONSOLE
#  undef CONFIG_UART7_SERIAL_CONSOLE
#  define CONSOLE_UART 4
#  define HAVE_CONSOLE 1
#elif defined(CONFIG_USART5_SERIAL_CONSOLE) && defined(CONFIG_GD32F4_USART5_SERIALDRIVER)
#  undef CONFIG_USART0_SERIAL_CONSOLE
#  undef CONFIG_USART1_SERIAL_CONSOLE
#  undef CONFIG_USART2_SERIAL_CONSOLE
#  undef CONFIG_UART3_SERIAL_CONSOLE
#  undef CONFIG_UART4_SERIAL_CONSOLE
#  undef CONFIG_UART6_SERIAL_CONSOLE
#  undef CONFIG_UART7_SERIAL_CONSOLE
#  define CONSOLE_UART 5
#  define HAVE_CONSOLE 1
#elif defined(CONFIG_UART6_SERIAL_CONSOLE) && defined(CONFIG_GD32F4_UART6_SERIALDRIVER)
#  undef CONFIG_USART0_SERIAL_CONSOLE
#  undef CONFIG_USART1_SERIAL_CONSOLE
#  undef CONFIG_USART2_SERIAL_CONSOLE
#  undef CONFIG_UART3_SERIAL_CONSOLE
#  undef CONFIG_UART4_SERIAL_CONSOLE
#  undef CONFIG_USART5_SERIAL_CONSOLE
#  undef CONFIG_UART7_SERIAL_CONSOLE
#  define CONSOLE_UART 6
#  define HAVE_CONSOLE 1
#elif defined(CONFIG_UART7_SERIAL_CONSOLE) && defined(CONFIG_GD32F4_UART7_SERIALDRIVER)
#  undef CONFIG_USART0_SERIAL_CONSOLE
#  undef CONFIG_USART1_SERIAL_CONSOLE
#  undef CONFIG_USART2_SERIAL_CONSOLE
#  undef CONFIG_UART3_SERIAL_CONSOLE
#  undef CONFIG_UART4_SERIAL_CONSOLE
#  undef CONFIG_USART5_SERIAL_CONSOLE
#  undef CONFIG_UART6_SERIAL_CONSOLE
#  define CONSOLE_UART 7
#  define HAVE_CONSOLE 1
#else
#  undef CONFIG_USART0_SERIAL_CONSOLE
#  undef CONFIG_USART1_SERIAL_CONSOLE
#  undef CONFIG_USART2_SERIAL_CONSOLE
#  undef CONFIG_UART3_SERIAL_CONSOLE
#  undef CONFIG_UART4_SERIAL_CONSOLE
#  undef CONFIG_USART5_SERIAL_CONSOLE
#  undef CONFIG_UART6_SERIAL_CONSOLE
#  undef CONFIG_UART7_SERIAL_CONSOLE
#  undef CONSOLE_UART
#  undef HAVE_CONSOLE
#endif

/* DMA support is only provided if CONFIG_ARCH_DMA is in the
 * NuttX configuration
 */

#if !defined(HAVE_SERIALDRIVER) || !defined(CONFIG_ARCH_DMA)
#  undef CONFIG_GD32F4_USART0_RXDMA
#  undef CONFIG_GD32F4_USART0_TXDMA
#  undef CONFIG_GD32F4_USART1_RXDMA
#  undef CONFIG_GD32F4_USART1_TXDMA
#  undef CONFIG_GD32F4_USART2_RXDMA
#  undef CONFIG_GD32F4_USART2_TXDMA
#  undef CONFIG_GD32F4_UART3_RXDMA
#  undef CONFIG_GD32F4_UART3_TXDMA
#  undef CONFIG_GD32F4_UART4_RXDMA
#  undef CONFIG_GD32F4_UART4_TXDMA
#  undef CONFIG_GD32F4_USART5_RXDMA
#  undef CONFIG_GD32F4_USART5_TXDMA
#  undef CONFIG_GD32F4_UART6_RXDMA
#  undef CONFIG_GD32F4_UART6_TXDMA
#  undef CONFIG_GD32F4_UART7_RXDMA
#  undef CONFIG_GD32F4_UART7_TXDMA
#endif

/* Disable the DMA configuration on all unused USARTs */

#ifndef CONFIG_GD32F4_USART0_SERIALDRIVER
#  undef CONFIG_GD32F4_USART0_RXDMA
#  undef CONFIG_GD32F4_USART0_TXDMA
#endif

#ifndef CONFIG_GD32F4_USART1_SERIALDRIVER
#  undef CONFIG_GD32F4_USART1_RXDMA
#  undef CONFIG_GD32F4_USART1_TXDMA
#endif

#ifndef CONFIG_GD32F4_USART2_SERIALDRIVER
#  undef CONFIG_GD32F4_USART2_RXDMA
#  undef CONFIG_GD32F4_USART2_TXDMA
#endif

#ifndef CONFIG_GD32F4_UART3_SERIALDRIVER
#  undef CONFIG_GD32F4_UART3_RXDMA
#  undef CONFIG_GD32F4_UART3_TXDMA
#endif

#ifndef CONFIG_GD32F4_UART4_SERIALDRIVER
#  undef CONFIG_GD32F4_UART4_RXDMA
#  undef CONFIG_GD32F4_UART4_TXDMA
#endif

#ifndef CONFIG_GD32F4_USART5_SERIALDRIVER
#  undef CONFIG_GD32F4_USART5_RXDMA
#  undef CONFIG_GD32F4_USART5_TXDMA
#endif

#ifndef CONFIG_GD32F4_UART6_SERIALDRIVER
#  undef CONFIG_GD32F4_UART6_RXDMA
#  undef CONFIG_GD32F4_UART6_TXDMA
#endif

#ifndef CONFIG_GD32F4_UART7_SERIALDRIVER
#  undef CONFIG_GD32F4_UART7_RXDMA
#  undef CONFIG_GD32F4_UART7_TXDMA
#endif

/* Is DMA available on any (enabled) USART? */

#undef SERIAL_HAVE_RXDMA
#if defined(CONFIG_GD32F4_USART0_RXDMA) || defined(CONFIG_GD32F4_USART1_RXDMA) || \
    defined(CONFIG_GD32F4_USART2_RXDMA) || defined(CONFIG_GD32F4_UART3_RXDMA)  || \
    defined(CONFIG_GD32F4_UART4_RXDMA)  || defined(CONFIG_GD32F4_USART5_RXDMA) || \
    defined(CONFIG_GD32F4_UART6_RXDMA)  || defined(CONFIG_GD32F4_UART7_RXDMA)
#  define SERIAL_HAVE_RXDMA 1
#endif

/* Is TX DMA available on any (enabled) USART? */

#undef SERIAL_HAVE_TXDMA
#if defined(CONFIG_GD32F4_USART0_TXDMA) || defined(CONFIG_GD32F4_USART1_TXDMA) || \
  defined(CONFIG_GD32F4_USART2_TXDMA) || defined(CONFIG_GD32F4_UART3_TXDMA)  ||   \
  defined(CONFIG_GD32F4_UART4_TXDMA)  || defined(CONFIG_GD32F4_USART5_TXDMA) ||   \
  defined(CONFIG_GD32F4_UART6_TXDMA)  || defined(CONFIG_GD32F4_UART7_TXDMA)
#  define SERIAL_HAVE_TXDMA 1
#endif

/* Is RX DMA used on the console UART? */

#undef SERIAL_HAVE_CONSOLE_RXDMA
#if defined(CONFIG_USART0_SERIAL_CONSOLE) && defined(CONFIG_GD32F4_USART0_RXDMA)
#  define SERIAL_HAVE_CONSOLE_RXDMA 1
#elif defined(CONFIG_USART1_SERIAL_CONSOLE) && defined(CONFIG_GD32F4_USART1_RXDMA)
#  define SERIAL_HAVE_CONSOLE_RXDMA 1
#elif defined(CONFIG_USART2_SERIAL_CONSOLE) && defined(CONFIG_GD32F4_USART2_RXDMA)
#  define SERIAL_HAVE_CONSOLE_RXDMA 1
#elif defined(CONFIG_UART3_SERIAL_CONSOLE) && defined(CONFIG_GD32F4_UART3_RXDMA)
#  define SERIAL_HAVE_CONSOLE_RXDMA 1
#elif defined(CONFIG_UART4_SERIAL_CONSOLE) && defined(CONFIG_GD32F4_UART4_RXDMA)
#  define SERIAL_HAVE_CONSOLE_RXDMA 1
#elif defined(CONFIG_USART5_SERIAL_CONSOLE) && defined(CONFIG_GD32F4_USART5_RXDMA)
#  define SERIAL_HAVE_CONSOLE_RXDMA 1
#elif defined(CONFIG_UART6_SERIAL_CONSOLE) && defined(CONFIG_GD32F4_UART6_RXDMA)
#  define SERIAL_HAVE_CONSOLE_RXDMA 1
#elif defined(CONFIG_UART7_SERIAL_CONSOLE) && defined(CONFIG_GD32F4_UART7_RXDMA)
#  define SERIAL_HAVE_CONSOLE_RXDMA 1
#endif

/* Is TX DMA used on the console UART? */

#undef SERIAL_HAVE_CONSOLE_TXDMA
#if defined(CONFIG_USART0_SERIAL_CONSOLE) && defined(CONFIG_GD32F4_USART0_TXDMA)
#  define SERIAL_HAVE_CONSOLE_TXDMA 1
#elif defined(CONFIG_USART1_SERIAL_CONSOLE) && defined(CONFIG_GD32F4_USART1_TXDMA)
#  define SERIAL_HAVE_CONSOLE_TXDMA 1
#elif defined(CONFIG_USART2_SERIAL_CONSOLE) && defined(CONFIG_GD32F4_USART2_TXDMA)
#  define SERIAL_HAVE_CONSOLE_TXDMA 1
#elif defined(CONFIG_UART3_SERIAL_CONSOLE) && defined(CONFIG_GD32F4_UART3_TXDMA)
#  define SERIAL_HAVE_CONSOLE_TXDMA 1
#elif defined(CONFIG_UART4_SERIAL_CONSOLE) && defined(CONFIG_GD32F4_UART4_TXDMA)
#  define SERIAL_HAVE_CONSOLE_TXDMA 1
#elif defined(CONFIG_USART5_SERIAL_CONSOLE) && defined(CONFIG_GD32F4_USART5_TXDMA)
#  define SERIAL_HAVE_CONSOLE_TXDMA 1
#elif defined(CONFIG_UART6_SERIAL_CONSOLE) && defined(CONFIG_GD32F4_UART6_TXDMA)
#  define SERIAL_HAVE_CONSOLE_TXDMA 1
#elif defined(CONFIG_UART7_SERIAL_CONSOLE) && defined(CONFIG_GD32F4_UART7_TXDMA)
#  define SERIAL_HAVE_CONSOLE_TXDMA 1
#endif

/* No DMA */

#undef SERIAL_NOT_HAVE_DMA
#if defined(CONFIG_GD32F4_USART0) && !defined(CONFIG_GD32F4_USART0_RXDMA) &&   \
    !defined(CONFIG_GD32F4_USART0_TXDMA)
#  define SERIAL_NOT_HAVE_DMA
#elif defined(CONFIG_GD32F4_USART1) && !defined(CONFIG_GD32F4_USART1_RXDMA) && \
    !defined(CONFIG_GD32F4_USART1_TXDMA)
#  define SERIAL_NOT_HAVE_DMA
#elif defined(CONFIG_GD32F4_USART2) && !defined(CONFIG_GD32F4_USART2_RXDMA) && \
    !defined(CONFIG_GD32F4_USART2_TXDMA)
#  define SERIAL_NOT_HAVE_DMA
#elif defined(CONFIG_GD32F4_UART3) && !defined(CONFIG_GD32F4_UART3_RXDMA) &&  \
    !defined(CONFIG_GD32F4_UART3_TXDMA)
#  define SERIAL_NOT_HAVE_DMA
#elif defined(CONFIG_GD32F4_UART4) && !defined(CONFIG_GD32F4_UART4_RXDMA) &&  \
    !defined(CONFIG_GD32F4_UART4_TXDMA)
#  define SERIAL_NOT_HAVE_DMA
#elif defined(CONFIG_GD32F4_USART5) && !defined(CONFIG_GD32F4_USART5_RXDMA) && \
    !defined(CONFIG_GD32F4_USART5_TXDMA)
#  define SERIAL_NOT_HAVE_DMA
#elif defined(CONFIG_GD32F4_UART6) && !defined(CONFIG_GD32F4_UART6_RXDMA) &&  \
    !defined(CONFIG_GD32F4_UART6_TXDMA)
#  define SERIAL_NOT_HAVE_DMA
#elif defined(CONFIG_GD32F4_UART7) && !defined(CONFIG_GD32F4_UART7_RXDMA) &&  \
    !defined(CONFIG_GD32F4_UART7_TXDMA)
#  define SERIAL_NOT_HAVE_DMA
#endif

/* RX DMA */

#undef SERIAL_HAVE_RX_DMA
#if defined(CONFIG_GD32F4_USART0_RXDMA) && !defined(CONFIG_GD32F4_USART0_TXDMA)
#  define SERIAL_HAVE_RX_DMA
#elif defined(CONFIG_GD32F4_USART1_RXDMA) && !defined(CONFIG_GD32F4_USART1_TXDMA)
#  define SERIAL_HAVE_RX_DMA
#elif defined(CONFIG_GD32F4_USART2_RXDMA) && !defined(CONFIG_GD32F4_USART2_TXDMA)
#  define SERIAL_HAVE_RX_DMA
#elif defined(CONFIG_GD32F4_UART3_RXDMA) && !defined(CONFIG_GD32F4_UART3_TXDMA)
#  define SERIAL_HAVE_RX_DMA
#elif defined(CONFIG_GD32F4_UART4_RXDMA) && !defined(CONFIG_GD32F4_UART4_TXDMA)
#  define SERIAL_HAVE_RX_DMA
#elif defined(CONFIG_GD32F4_USART5_RXDMA) && !defined(CONFIG_GD32F4_USART5_TXDMA)
#  define SERIAL_HAVE_RX_DMA
#elif defined(CONFIG_GD32F4_UART6_RXDMA) && !defined(CONFIG_GD32F4_UART6_TXDMA)
#  define SERIAL_HAVE_RX_DMA
#elif defined(CONFIG_GD32F4_UART7_RXDMA) && !defined(CONFIG_GD32F4_UART7_TXDMA)
#  define SERIAL_HAVE_RX_DMA
#endif

/* TX DMA */

#undef SERIAL_HAVE_TX_DMA
#if !defined(CONFIG_GD32F4_USART0_RXDMA) && defined(CONFIG_GD32F4_USART0_TXDMA)
#  define SERIAL_HAVE_TX_DMA
#elif !defined(CONFIG_GD32F4_USART1_RXDMA) && defined(CONFIG_GD32F4_USART1_TXDMA)
#  define SERIAL_HAVE_TX_DMA
#elif !defined(CONFIG_GD32F4_USART2_RXDMA) && defined(CONFIG_GD32F4_USART2_TXDMA)
#  define SERIAL_HAVE_TX_DMA
#elif !defined(CONFIG_GD32F4_UART3_RXDMA) && defined(CONFIG_GD32F4_UART3_TXDMA)
#  define SERIAL_HAVE_TX_DMA
#elif !defined(CONFIG_GD32F4_UART4_RXDMA) && defined(CONFIG_GD32F4_UART4_TXDMA)
#  define SERIAL_HAVE_TX_DMA
#elif !defined(CONFIG_GD32F4_USART5_RXDMA) && defined(CONFIG_GD32F4_USART5_TXDMA)
#  define SERIAL_HAVE_TX_DMA
#elif !defined(CONFIG_GD32F4_UART6_RXDMA) && defined(CONFIG_GD32F4_UART6_TXDMA)
#  define SERIAL_HAVE_TX_DMA
#elif !defined(CONFIG_GD32F4_UART7_RXDMA) && defined(CONFIG_GD32F4_UART7_TXDMA)
#  define SERIAL_HAVE_TX_DMA
#endif

/* RX and TX DMA */

#undef SERIAL_HAVE_RXTX_DMA
#if defined(CONFIG_GD32F4_USART0_RXDMA) && defined(CONFIG_GD32F4_USART0_TXDMA)
#  define SERIAL_HAVE_RXTX_DMA
#elif defined(CONFIG_GD32F4_USART1_RXDMA) && defined(CONFIG_GD32F4_USART1_TXDMA)
#  define SERIAL_HAVE_RXTX_DMA
#elif defined(CONFIG_GD32F4_USART2_RXDMA) && defined(CONFIG_GD32F4_USART2_TXDMA)
#  define SERIAL_HAVE_RXTX_DMA
#elif defined(CONFIG_GD32F4_UART3_RXDMA) && defined(CONFIG_GD32F4_UART3_TXDMA)
#  define SERIAL_HAVE_RXTX_DMA
#elif defined(CONFIG_GD32F4_UART4_RXDMA) && defined(CONFIG_GD32F4_UART4_TXDMA)
#  define SERIAL_HAVE_RXTX_DMA
#elif defined(CONFIG_GD32F4_USART5_RXDMA) && defined(CONFIG_GD32F4_USART5_TXDMA)
#  define SERIAL_HAVE_RXTX_DMA
#elif defined(CONFIG_GD32F4_UART6_RXDMA) && defined(CONFIG_GD32F4_UART6_TXDMA)
#  define SERIAL_HAVE_RXTX_DMA
#elif defined(CONFIG_GD32F4_UART7_RXDMA) && defined(CONFIG_GD32F4_UART7_TXDMA)
#  define SERIAL_HAVE_RXTX_DMA
#endif

/* Is RS-485 used? */

#if defined(CONFIG_USART0_RS485) || defined(CONFIG_USART1_RS485) || \
    defined(CONFIG_USART2_RS485) || defined(CONFIG_UART3_RS485)  || \
    defined(CONFIG_UART4_RS485)  || defined(CONFIG_USART5_RS485)  || \
    defined(CONFIG_UART6_RS485)  || defined(CONFIG_UART7_RS485)
#  define HAVE_RS485 1
#endif

/* Usart oversample mode. */

#define USART_OVSMOD_16            (0)
#define USART_OVSMOD_8             (USART_CTL0_OVSMOD)

/* USART interrupts maps, each interrupt of the USART can be individually
 * configured by software. The following definitions provide the bit
 * encodingthat used to define the interrupt and control register offset.
 *
 * 24-bit Encoding:       2222 2222 1111 1111 1100 0000 0000
 *                        7654 3210 9876 5432 1098 7654 3210
 * ENCODING               SHIF
 *
 * CTL SHIFT: Bit24-27, CTL2 int: Bit8-23, CTL3 int: Bit6-7,
 *  CTL1 int: Bit5, CTL0 int: Bit0-4,
 */

#define USART_CFG_SHIFT            (24)
#define USART_CFG_CTL0_INT         (1)
#define USART_CFG_CTL1_INT         (2)
#define USART_CFG_CTL2_INT         (4)
#define USART_CFG_CTL3_INT         (8)
#define USART_CFG_CTL_MASK         (0xf)

/* USART interrupt in CTL0 register */
#define USART_CFG_CTL0_INT_SHIFT     (4)
#  define USART_CFG_CTL0_INT_IDLIE   ((USART_CFG_CTL0_INT << USART_CFG_SHIFT) | (USART_CTL0_IDLEIE >> USART_CFG_CTL0_INT_SHIFT))
#  define USART_CFG_CTL0_INT_RBNEIE  ((USART_CFG_CTL0_INT << USART_CFG_SHIFT) | (USART_CTL0_RBNEIE >> USART_CFG_CTL0_INT_SHIFT))
#  define USART_CFG_CTL0_INT_TCIE    ((USART_CFG_CTL0_INT << USART_CFG_SHIFT) | (USART_CTL0_TCIE >> USART_CFG_CTL0_INT_SHIFT))
#  define USART_CFG_CTL0_INT_TBEIE   ((USART_CFG_CTL0_INT << USART_CFG_SHIFT) | (USART_CTL0_TBEIE >> USART_CFG_CTL0_INT_SHIFT))
#  define USART_CFG_CTL0_INT_PERRIE  ((USART_CFG_CTL0_INT << USART_CFG_SHIFT) | (USART_CTL0_PERRIE >> USART_CFG_CTL0_INT_SHIFT))

/* USART interrupt in CTL1 register */

#define USART_CFG_CTL1_INT_SHIFT   (1)
#  define USART_CFG_CTL1_INT_LBDIE ((USART_CFG_CTL1_INT << USART_CFG_SHIFT) | (USART_CTL1_LBDIE >> USART_CFG_CTL0_INT_SHIFT))

/* USART interrupt in CTL2 register */

#define USART_CFG_CTL2_INT_SHIFT   (8)
#  define USART_CFG_CTL2_INT_ERRIE ((USART_CFG_CTL2_INT << USART_CFG_SHIFT) | (USART_CTL2_ERRIE << USART_CFG_CTL0_INT_SHIFT))
#  define USART_CFG_CTL2_INT_CTSIE ((USART_CFG_CTL2_INT << USART_CFG_SHIFT) | (USART_CTL2_CTSIE << USART_CFG_CTL0_INT_SHIFT))

/* USART interrupt in CTL3 register */

#define USART_CFG_CTL3_INT_SHIFT   (2)
#  define USART_CFG_CTL3_INT_RTIE  ((USART_CFG_CTL3_INT << USART_CFG_SHIFT) | USART_CTL3_RTIE << USART_CFG_CTL3_INT_SHIFT)
#  define USART_CFG_CTL3_INT_EBIE  ((USART_CFG_CTL3_INT << USART_CFG_SHIFT) | USART_CTL3_EBIE << USART_CFG_CTL3_INT_SHIFT)

#define USART_CTL0_USED_INTS       USART_CTL0_INT_MASK
#define USART_CTL1_USED_INTS       USART_CTL1_INT_MASK
#define USART_CTL2_USED_INTS       USART_CTL2_INT_MASK
#define USART_CTL3_USED_INTS       USART_CTL3_INT_MASK

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_serial_get_uart
 *
 * Description:
 *   Get serial driver structure for GD32 USART
 *
 ****************************************************************************/

uart_dev_t *gd32_serial_get_uart(int uart_num);

/****************************************************************************
 * Name: gd32_serial_dma_poll
 *
 * Description:
 *   Must be called periodically if any GD32 UART is configured for DMA.
 *   The DMA callback is triggered for each fifo size/2 bytes, but this can
 *   result in some bytes being transferred but not collected if the incoming
 *   data is not a whole multiple of half the FIFO size.
 *
 *   May be safely called from either interrupt or thread context.
 *
 ****************************************************************************/

#ifdef SERIAL_HAVE_RXDMA
void gd32_serial_dma_poll(void);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_GD32F4_GD32F4XX_USART_H */
