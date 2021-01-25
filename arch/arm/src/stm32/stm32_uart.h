/************************************************************************************
 * arch/arm/src/stm32/stm32_uart.h
 *
 *   Copyright (C) 2009, 2012-2013, 2015 Gregory Nutt. All rights reserved.
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
 ************************************************************************************/

#ifndef __ARCH_ARM_STC_STM32_STM32_UART_H
#define __ARCH_ARM_STC_STM32_STM32_UART_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/serial/serial.h>

#include "chip.h"

#if defined(CONFIG_STM32_STM32L15XX)
#  include "hardware/stm32l15xxx_uart.h"
#elif defined(CONFIG_STM32_STM32F10XX)
#  include "hardware/stm32f10xxx_uart.h"
#elif defined(CONFIG_STM32_STM32F20XX)
#  include "hardware/stm32f20xxx_uart.h"
#elif defined(CONFIG_STM32_STM32F30XX) || defined(CONFIG_STM32_STM32F33XX) || \
    defined(CONFIG_STM32_STM32F37XX)
#  include "hardware/stm32f30xxx_uart.h"
#elif defined(CONFIG_STM32_STM32F4XXX)
#  include "hardware/stm32f40xxx_uart.h"
#elif defined(CONFIG_STM32_STM32G4XXX)
#  include "hardware/stm32g4xxxx_uart.h"
#else
#  error "Unsupported STM32 UART"
#endif

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Make sure that we have not enabled more U[S]ARTs than are supported by the
 * device.
 */

#if STM32_NUSART < 8 || !defined(CONFIG_STM32_HAVE_UART8)
#  undef CONFIG_STM32_UART8
#endif
#if STM32_NUSART < 7 || !defined(CONFIG_STM32_HAVE_UART7)
#  undef CONFIG_STM32_UART7
#endif
#if STM32_NUSART < 6 || !defined(CONFIG_STM32_HAVE_USART6)
#  undef CONFIG_STM32_USART6
#endif
#if STM32_NUSART < 5 || !defined(CONFIG_STM32_HAVE_UART5)
#  undef CONFIG_STM32_UART5
#endif
#if STM32_NUSART < 4 || !defined(CONFIG_STM32_HAVE_UART4)
#  undef CONFIG_STM32_UART4
#endif
#if STM32_NUSART < 3 || !defined(CONFIG_STM32_HAVE_USART3)
#  undef CONFIG_STM32_USART3
#endif
#if STM32_NUSART < 2
#  undef CONFIG_STM32_USART2
#endif
#if STM32_NUSART < 1
#  undef CONFIG_STM32_USART1
#endif

/* Sanity checks */

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

/* Check 1-Wire and U(S)ART conflicts */

#if defined(CONFIG_STM32_USART1_1WIREDRIVER) && defined(CONFIG_STM32_USART1_SERIALDRIVER)
#  error Both CONFIG_STM32_USART1_1WIREDRIVER and CONFIG_STM32_USART1_SERIALDRIVER defined
#  undef CONFIG_STM32_USART1_1WIREDRIVER
#endif
#if defined(CONFIG_STM32_USART2_1WIREDRIVER) && defined(CONFIG_STM32_USART2_SERIALDRIVER)
#  error Both CONFIG_STM32_USART2_1WIREDRIVER and CONFIG_STM32_USART2_SERIALDRIVER defined
#  undef CONFIG_STM32_USART2_1WIREDRIVER
#endif
#if defined(CONFIG_STM32_USART3_1WIREDRIVER) && defined(CONFIG_STM32_USART3_SERIALDRIVER)
#  error Both CONFIG_STM32_USART3_1WIREDRIVER and CONFIG_STM32_USART3_SERIALDRIVER defined
#  undef CONFIG_STM32_USART3_1WIREDRIVER
#endif
#if defined(CONFIG_STM32_UART4_1WIREDRIVER) && defined(CONFIG_STM32_UART4_SERIALDRIVER)
#  error Both CONFIG_STM32_UART4_1WIREDRIVER and CONFIG_STM32_UART4_SERIALDRIVER defined
#  undef CONFIG_STM32_UART4_1WIREDRIVER
#endif
#if defined(CONFIG_STM32_UART5_1WIREDRIVER) && defined(CONFIG_STM32_UART5_SERIALDRIVER)
#  error Both CONFIG_STM32_UART5_1WIREDRIVER and CONFIG_STM32_UART5_SERIALDRIVER defined
#  undef CONFIG_STM32_UART5_1WIREDRIVER
#endif
#if defined(CONFIG_STM32_USART6_1WIREDRIVER) && defined(CONFIG_STM32_USART6_SERIALDRIVER)
#  error Both CONFIG_STM32_USART6_1WIREDRIVER and CONFIG_STM32_USART6_SERIALDRIVER defined
#  undef CONFIG_STM32_USART6_1WIREDRIVER
#endif
#if defined(CONFIG_STM32_UART7_1WIREDRIVER) && defined(CONFIG_STM32_UART7_SERIALDRIVER)
#  error Both CONFIG_STM32_UART7_1WIREDRIVER and CONFIG_STM32_UART7_SERIALDRIVER defined
#  undef CONFIG_STM32_UART7_1WIREDRIVER
#endif
#if defined(CONFIG_STM32_UART8_1WIREDRIVER) && defined(CONFIG_STM32_UART8_SERIALDRIVER)
#  error Both CONFIG_STM32_UART8_1WIREDRIVER and CONFIG_STM32_UART8_SERIALDRIVER defined
#  undef CONFIG_STM32_UART8_1WIREDRIVER
#endif

/* Is the serial driver enabled? */

#if defined(CONFIG_STM32_USART1_SERIALDRIVER) || defined(CONFIG_STM32_USART2_SERIALDRIVER) || \
    defined(CONFIG_STM32_USART3_SERIALDRIVER) || defined(CONFIG_STM32_UART4_SERIALDRIVER)  || \
    defined(CONFIG_STM32_UART5_SERIALDRIVER)  || defined(CONFIG_STM32_USART6_SERIALDRIVER) || \
    defined(CONFIG_STM32_UART7_SERIALDRIVER)  || defined(CONFIG_STM32_UART8_SERIALDRIVER)
#  define HAVE_SERIALDRIVER 1
#endif

/* Is the 1-Wire driver? */

#if defined(CONFIG_STM32_USART1_1WIREDRIVER) || defined(CONFIG_STM32_USART2_1WIREDRIVER) || \
    defined(CONFIG_STM32_USART3_1WIREDRIVER) || defined(CONFIG_STM32_UART4_1WIREDRIVER) || \
    defined(CONFIG_STM32_UART5_1WIREDRIVER) || defined(CONFIG_STM32_USART6_1WIREDRIVER) || \
    defined(CONFIG_STM32_UART7_1WIREDRIVER) || defined(CONFIG_STM32_UART8_1WIREDRIVER)
#  define HAVE_1WIREDRIVER 1
#endif

/* Is there a serial console? */

#if defined(CONFIG_USART1_SERIAL_CONSOLE) && defined(CONFIG_STM32_USART1_SERIALDRIVER)
#  undef CONFIG_USART2_SERIAL_CONSOLE
#  undef CONFIG_USART3_SERIAL_CONSOLE
#  undef CONFIG_UART4_SERIAL_CONSOLE
#  undef CONFIG_UART5_SERIAL_CONSOLE
#  undef CONFIG_USART6_SERIAL_CONSOLE
#  undef CONFIG_UART7_SERIAL_CONSOLE
#  undef CONFIG_UART8_SERIAL_CONSOLE
#  define CONSOLE_UART 1
#  define HAVE_CONSOLE 1
#elif defined(CONFIG_USART2_SERIAL_CONSOLE) && defined(CONFIG_STM32_USART2_SERIALDRIVER)
#  undef CONFIG_USART1_SERIAL_CONSOLE
#  undef CONFIG_USART3_SERIAL_CONSOLE
#  undef CONFIG_UART4_SERIAL_CONSOLE
#  undef CONFIG_UART5_SERIAL_CONSOLE
#  undef CONFIG_USART6_SERIAL_CONSOLE
#  undef CONFIG_UART7_SERIAL_CONSOLE
#  undef CONFIG_UART8_SERIAL_CONSOLE
#  define CONSOLE_UART 2
#  define HAVE_CONSOLE 1
#elif defined(CONFIG_USART3_SERIAL_CONSOLE) && defined(CONFIG_STM32_USART3_SERIALDRIVER)
#  undef CONFIG_USART1_SERIAL_CONSOLE
#  undef CONFIG_USART2_SERIAL_CONSOLE
#  undef CONFIG_UART4_SERIAL_CONSOLE
#  undef CONFIG_UART5_SERIAL_CONSOLE
#  undef CONFIG_USART6_SERIAL_CONSOLE
#  undef CONFIG_UART7_SERIAL_CONSOLE
#  undef CONFIG_UART8_SERIAL_CONSOLE
#  define CONSOLE_UART 3
#  define HAVE_CONSOLE 1
#elif defined(CONFIG_UART4_SERIAL_CONSOLE) && defined(CONFIG_STM32_UART4_SERIALDRIVER)
#  undef CONFIG_USART1_SERIAL_CONSOLE
#  undef CONFIG_USART2_SERIAL_CONSOLE
#  undef CONFIG_USART3_SERIAL_CONSOLE
#  undef CONFIG_UART5_SERIAL_CONSOLE
#  undef CONFIG_USART6_SERIAL_CONSOLE
#  undef CONFIG_UART7_SERIAL_CONSOLE
#  undef CONFIG_UART8_SERIAL_CONSOLE
#  define CONSOLE_UART 4
#  define HAVE_CONSOLE 1
#elif defined(CONFIG_UART5_SERIAL_CONSOLE) && defined(CONFIG_STM32_UART5_SERIALDRIVER)
#  undef CONFIG_USART1_SERIAL_CONSOLE
#  undef CONFIG_USART2_SERIAL_CONSOLE
#  undef CONFIG_USART3_SERIAL_CONSOLE
#  undef CONFIG_UART4_SERIAL_CONSOLE
#  undef CONFIG_USART6_SERIAL_CONSOLE
#  undef CONFIG_UART7_SERIAL_CONSOLE
#  undef CONFIG_UART8_SERIAL_CONSOLE
#  define CONSOLE_UART 5
#  define HAVE_CONSOLE 1
#elif defined(CONFIG_USART6_SERIAL_CONSOLE) && defined(CONFIG_STM32_USART6_SERIALDRIVER)
#  undef CONFIG_USART1_SERIAL_CONSOLE
#  undef CONFIG_USART2_SERIAL_CONSOLE
#  undef CONFIG_USART3_SERIAL_CONSOLE
#  undef CONFIG_UART4_SERIAL_CONSOLE
#  undef CONFIG_UART5_SERIAL_CONSOLE
#  undef CONFIG_UART7_SERIAL_CONSOLE
#  undef CONFIG_UART8_SERIAL_CONSOLE
#  define CONSOLE_UART 6
#  define HAVE_CONSOLE 1
#elif defined(CONFIG_UART7_SERIAL_CONSOLE) && defined(CONFIG_STM32_UART7_SERIALDRIVER)
#  undef CONFIG_USART1_SERIAL_CONSOLE
#  undef CONFIG_USART2_SERIAL_CONSOLE
#  undef CONFIG_USART3_SERIAL_CONSOLE
#  undef CONFIG_UART4_SERIAL_CONSOLE
#  undef CONFIG_UART5_SERIAL_CONSOLE
#  undef CONFIG_USART6_SERIAL_CONSOLE
#  undef CONFIG_UART8_SERIAL_CONSOLE
#  define CONSOLE_UART 7
#  define HAVE_CONSOLE 1
#elif defined(CONFIG_UART8_SERIAL_CONSOLE) && defined(CONFIG_STM32_UART8_SERIALDRIVER)
#  undef CONFIG_USART1_SERIAL_CONSOLE
#  undef CONFIG_USART2_SERIAL_CONSOLE
#  undef CONFIG_USART3_SERIAL_CONSOLE
#  undef CONFIG_UART4_SERIAL_CONSOLE
#  undef CONFIG_UART5_SERIAL_CONSOLE
#  undef CONFIG_USART6_SERIAL_CONSOLE
#  undef CONFIG_UART7_SERIAL_CONSOLE
#  define CONSOLE_UART 8
#  define HAVE_CONSOLE 1
#else
#  undef CONFIG_USART1_SERIAL_CONSOLE
#  undef CONFIG_USART2_SERIAL_CONSOLE
#  undef CONFIG_USART3_SERIAL_CONSOLE
#  undef CONFIG_UART4_SERIAL_CONSOLE
#  undef CONFIG_UART5_SERIAL_CONSOLE
#  undef CONFIG_USART6_SERIAL_CONSOLE
#  undef CONFIG_UART7_SERIAL_CONSOLE
#  undef CONFIG_UART8_SERIAL_CONSOLE
#  define CONSOLE_UART 0
#  undef HAVE_CONSOLE
#endif

/* DMA support is only provided if CONFIG_ARCH_DMA is in the NuttX configuration */

#if !defined(HAVE_SERIALDRIVER) || !defined(CONFIG_ARCH_DMA)
#  undef CONFIG_USART1_RXDMA
#  undef CONFIG_USART2_RXDMA
#  undef CONFIG_USART3_RXDMA
#  undef CONFIG_UART4_RXDMA
#  undef CONFIG_UART5_RXDMA
#  undef CONFIG_USART6_RXDMA
#  undef CONFIG_UART7_RXDMA
#  undef CONFIG_UART8_RXDMA
#endif

/* Disable the DMA configuration on all unused USARTs */

#ifndef CONFIG_STM32_USART1_SERIALDRIVER
#  undef CONFIG_USART1_RXDMA
#endif

#ifndef CONFIG_STM32_USART2_SERIALDRIVER
#  undef CONFIG_USART2_RXDMA
#endif

#ifndef CONFIG_STM32_USART3_SERIALDRIVER
#  undef CONFIG_USART3_RXDMA
#endif

#ifndef CONFIG_STM32_UART4_SERIALDRIVER
#  undef CONFIG_UART4_RXDMA
#endif

#ifndef CONFIG_STM32_UART5_SERIALDRIVER
#  undef CONFIG_UART5_RXDMA
#endif

#ifndef CONFIG_STM32_USART6_SERIALDRIVER
#  undef CONFIG_USART6_RXDMA
#endif

#ifndef CONFIG_STM32_UART7_SERIALDRIVER
#  undef CONFIG_UART7_RXDMA
#endif

#ifndef CONFIG_STM32_UART8_SERIALDRIVER
#  undef CONFIG_UART8_RXDMA
#endif

/* Is DMA available on any (enabled) USART? */

#undef SERIAL_HAVE_RXDMA
#if defined(CONFIG_USART1_RXDMA) || defined(CONFIG_USART2_RXDMA) || \
    defined(CONFIG_USART3_RXDMA) || defined(CONFIG_UART4_RXDMA)  || \
    defined(CONFIG_UART5_RXDMA)  || defined(CONFIG_USART6_RXDMA) || \
    defined(CONFIG_UART7_RXDMA)  || defined(CONFIG_UART8_RXDMA)
#  define SERIAL_HAVE_RXDMA 1
#endif

/* Is DMA used on the console UART? */

#undef SERIAL_HAVE_CONSOLE_DMA
#if defined(CONFIG_USART1_SERIAL_CONSOLE) && defined(CONFIG_USART1_RXDMA)
#  define SERIAL_HAVE_CONSOLE_DMA 1
#elif defined(CONFIG_USART2_SERIAL_CONSOLE) && defined(CONFIG_USART2_RXDMA)
#  define SERIAL_HAVE_CONSOLE_DMA 1
#elif defined(CONFIG_USART3_SERIAL_CONSOLE) && defined(CONFIG_USART3_RXDMA)
#  define SERIAL_HAVE_CONSOLE_DMA 1
#elif defined(CONFIG_UART4_SERIAL_CONSOLE) && defined(CONFIG_UART4_RXDMA)
#  define SERIAL_HAVE_CONSOLE_DMA 1
#elif defined(CONFIG_UART5_SERIAL_CONSOLE) && defined(CONFIG_UART5_RXDMA)
#  define SERIAL_HAVE_CONSOLE_DMA 1
#elif defined(CONFIG_USART6_SERIAL_CONSOLE) && defined(CONFIG_USART6_RXDMA)
#  define SERIAL_HAVE_CONSOLE_DMA 1
#elif defined(CONFIG_UART7_SERIAL_CONSOLE) && defined(CONFIG_UART7_RXDMA)
#  define SERIAL_HAVE_CONSOLE_DMA 1
#elif defined(CONFIG_UART8_SERIAL_CONSOLE) && defined(CONFIG_UART8_RXDMA)
#  define SERIAL_HAVE_CONSOLE_DMA 1
#endif

/* Is DMA used on all (enabled) USARTs */

#define SERIAL_HAVE_ONLY_DMA 1
#if defined(CONFIG_STM32_USART1_SERIALDRIVER) && !defined(CONFIG_USART1_RXDMA)
#  undef SERIAL_HAVE_ONLY_DMA
#elif defined(CONFIG_STM32_USART2_SERIALDRIVER) && !defined(CONFIG_USART2_RXDMA)
#  undef SERIAL_HAVE_ONLY_DMA
#elif defined(CONFIG_STM32_USART3_SERIALDRIVER) && !defined(CONFIG_USART3_RXDMA)
#  undef SERIAL_HAVE_ONLY_DMA
#elif defined(CONFIG_STM32_UART4_SERIALDRIVER) && !defined(CONFIG_UART4_RXDMA)
#  undef SERIAL_HAVE_ONLY_DMA
#elif defined(CONFIG_STM32_UART5_SERIALDRIVER) && !defined(CONFIG_UART5_RXDMA)
#  undef SERIAL_HAVE_ONLY_DMA
#elif defined(CONFIG_STM32_USART6_SERIALDRIVER) && !defined(CONFIG_USART6_RXDMA)
#  undef SERIAL_HAVE_ONLY_DMA
#elif defined(CONFIG_STM32_UART7_SERIALDRIVER) && !defined(CONFIG_UART7_RXDMA)
#  undef SERIAL_HAVE_ONLY_DMA
#elif defined(CONFIG_STM32_UART8_SERIALDRIVER) && !defined(CONFIG_UART8_RXDMA)
#  undef SERIAL_HAVE_ONLY_DMA
#endif

/* Is RS-485 used? */

#if defined(CONFIG_USART1_RS485) || defined(CONFIG_USART2_RS485) || \
    defined(CONFIG_USART3_RS485) || defined(CONFIG_UART4_RS485)  || \
    defined(CONFIG_UART5_RS485)  || defined(CONFIG_USART6_RS485)  || \
    defined(CONFIG_UART7_RS485)  || defined(CONFIG_UART8_RS485)
#  define HAVE_RS485 1
#endif

#ifdef HAVE_RS485
#  define USART_CR1_USED_INTS    (USART_CR1_RXNEIE | USART_CR1_TXEIE | USART_CR1_PEIE | USART_CR1_TCIE)
#else
#  define USART_CR1_USED_INTS    (USART_CR1_RXNEIE | USART_CR1_TXEIE | USART_CR1_PEIE)
#endif

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/************************************************************************************
 * Public Functions Prototypes
 ************************************************************************************/

/************************************************************************************
 * Name: stm32_serial_get_uart
 *
 * Description:
 *   Get serial driver structure for STM32 USART
 *
 ************************************************************************************/

FAR uart_dev_t *stm32_serial_get_uart(int uart_num);

/************************************************************************************
 * Name: stm32_serial_dma_poll
 *
 * Description:
 *   Must be called periodically if any STM32 UART is configured for DMA.  The DMA
 *   callback is triggered for each fifo size/2 bytes, but this can result in some
 *   bytes being transferred but not collected if the incoming data is not a whole
 *   multiple of half the FIFO size.
 *
 *   May be safely called from either interrupt or thread context.
 *
 ************************************************************************************/

#ifdef SERIAL_HAVE_RXDMA
void stm32_serial_dma_poll(void);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_STC_STM32_STM32_UART_H */
