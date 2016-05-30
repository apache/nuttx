/****************************************************************************
 * arch/arm/src/stm32f7/stm32_config.h
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_STM32F7_STM32_CONFIG_H
#define __ARCH_ARM_SRC_STM32F7_STM32_CONFIG_H

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

#ifndef CONFIG_STM32F7_GPIO_IRQ
#  undef CONFIG_STM32F7_GPIOA_IRQ
#  undef CONFIG_STM32F7_GPIOB_IRQ
#  undef CONFIG_STM32F7_GPIOC_IRQ
#  undef CONFIG_STM32F7_GPIOD_IRQ
#  undef CONFIG_STM32F7_GPIOE_IRQ
#endif

#if STM32F7_NPORTS < 1
#  undef CONFIG_STM32F7_GPIOA_IRQ
#endif
#if STM32F7_NPORTS < 2
#  undef CONFIG_STM32F7_GPIOB_IRQ
#endif
#if STM32F7_NPORTS < 3
#  undef CONFIG_STM32F7_GPIOC_IRQ
#endif
#if STM32F7_NPORTS < 4
#  undef CONFIG_STM32F7_GPIOD_IRQ
#endif
#if STM32F7_NPORTS < 5
#  undef CONFIG_STM32F7_GPIOE_IRQ
#endif

/* UARTs ********************************************************************/
/* Don't enable UARTs not supported by the chip. */

#if STM32F7_NUART < 1
#  undef CONFIG_STM32F7_UART0
#  undef CONFIG_STM32F7_UART1
#  undef CONFIG_STM32F7_UART2
#  undef CONFIG_STM32F7_UART3
#  undef CONFIG_STM32F7_UART4
#elif STM32F7_NUART < 2
#  undef CONFIG_STM32F7_UART1
#  undef CONFIG_STM32F7_UART2
#  undef CONFIG_STM32F7_UART3
#  undef CONFIG_STM32F7_UART4
#elif STM32F7_NUART < 3
#  undef CONFIG_STM32F7_UART2
#  undef CONFIG_STM32F7_UART3
#  undef CONFIG_STM32F7_UART4
#elif STM32F7_NUART < 4
#  undef CONFIG_STM32F7_UART3
#  undef CONFIG_STM32F7_UART4
#elif STM32F7_NUART < 5
#  undef CONFIG_STM32F7_UART4
#endif

/* Are any UARTs enabled? */

#undef HAVE_UART_DEVICE
#if defined(CONFIG_STM32F7_UART0) || defined(CONFIG_STM32F7_UART1) || \
    defined(CONFIG_STM32F7_UART2) || defined(CONFIG_STM32F7_UART3) || \
    defined(CONFIG_STM32F7_UART4)
#  define HAVE_UART_DEVICE 1
#endif

/* USARTs *******************************************************************/
/* If the USART is not being used as a UART, then it really isn't enabled
 * for our purposes.
 */

#ifndef CONFIG_USART0_SERIALDRIVER
#  undef CONFIG_STM32F7_USART0
#endif
#ifndef CONFIG_USART1_SERIALDRIVER
#  undef CONFIG_STM32F7_USART1
#endif
#ifndef CONFIG_USART2_SERIALDRIVER
#  undef CONFIG_STM32F7_USART2
#endif

/* Don't enable USARTs not supported by the chip. */

#if STM32F7_NUSART < 1
#  undef CONFIG_STM32F7_USART0
#  undef CONFIG_STM32F7_USART1
#  undef CONFIG_STM32F7_USART2
#elif STM32F7_NUSART < 2
#  undef CONFIG_STM32F7_USART1
#  undef CONFIG_STM32F7_USART2
#elif STM32F7_NUSART < 3
#  undef CONFIG_STM32F7_USART2
#endif

/* Are any USARTs enabled? */

#if defined(CONFIG_STM32F7_USART0) || defined(CONFIG_STM32F7_USART1) || \
    defined(CONFIG_STM32F7_USART2)
#  undef  HAVE_UART_DEVICE
#  define HAVE_UART_DEVICE 1
#endif

/* UART Flow Control ********************************************************/
/* UARTs do not support flow control */

#undef CONFIG_UART0_IFLOWCONTROL
#undef CONFIG_UART1_IFLOWCONTROL
#undef CONFIG_UART2_IFLOWCONTROL
#undef CONFIG_UART3_IFLOWCONTROL
#undef CONFIG_UART4_IFLOWCONTROL

/* Hardware flow control requires using DMAC channel (not yet supported) */

#ifdef CONFIG_SERIAL_IFLOWCONTROL
#  warning PDC or DMAC support is required for RTS hardware flow control
#  undef CONFIG_SERIAL_IFLOWCONTROL
#  undef CONFIG_USART0_IFLOWCONTROL
#  undef CONFIG_USART1_IFLOWCONTROL
#  undef CONFIG_USART2_IFLOWCONTROL
#endif

/* Serial Console ***********************************************************/
/* Is there a serial console?  There should be no more than one defined.  It
 * could be on any UARTn, n=1,..,STM32F7_NUART, or USARTn, n=1,.., STM32F7_NUSART
 */

#undef HAVE_SERIAL_CONSOLE
#if defined(CONFIG_UART0_SERIAL_CONSOLE) && defined(CONFIG_STM32F7_UART0)
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  undef CONFIG_UART2_SERIAL_CONSOLE
#  undef CONFIG_UART3_SERIAL_CONSOLE
#  undef CONFIG_UART4_SERIAL_CONSOLE
#  undef CONFIG_USART0_SERIAL_CONSOLE
#  undef CONFIG_USART1_SERIAL_CONSOLE
#  undef CONFIG_USART2_SERIAL_CONSOLE
#  define HAVE_SERIAL_CONSOLE 1
#elif defined(CONFIG_UART1_SERIAL_CONSOLE) && defined(CONFIG_STM32F7_UART1)
#  undef CONFIG_UART0_SERIAL_CONSOLE
#  undef CONFIG_UART2_SERIAL_CONSOLE
#  undef CONFIG_UART3_SERIAL_CONSOLE
#  undef CONFIG_UART4_SERIAL_CONSOLE
#  undef CONFIG_USART0_SERIAL_CONSOLE
#  undef CONFIG_USART1_SERIAL_CONSOLE
#  undef CONFIG_USART2_SERIAL_CONSOLE
#  define HAVE_SERIAL_CONSOLE 1
#elif defined(CONFIG_UART2_SERIAL_CONSOLE) && defined(CONFIG_STM32F7_UART2)
#  undef CONFIG_UART0_SERIAL_CONSOLE
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  undef CONFIG_UART3_SERIAL_CONSOLE
#  undef CONFIG_UART4_SERIAL_CONSOLE
#  undef CONFIG_USART0_SERIAL_CONSOLE
#  undef CONFIG_USART1_SERIAL_CONSOLE
#  undef CONFIG_USART2_SERIAL_CONSOLE
#  define HAVE_SERIAL_CONSOLE 1
#elif defined(CONFIG_UART3_SERIAL_CONSOLE) && defined(CONFIG_STM32F7_UART3)
#  undef CONFIG_UART0_SERIAL_CONSOLE
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  undef CONFIG_UART2_SERIAL_CONSOLE
#  undef CONFIG_UART4_SERIAL_CONSOLE
#  undef CONFIG_USART0_SERIAL_CONSOLE
#  undef CONFIG_USART1_SERIAL_CONSOLE
#  undef CONFIG_USART2_SERIAL_CONSOLE
#  define HAVE_SERIAL_CONSOLE 1
#elif defined(CONFIG_UART4_SERIAL_CONSOLE) && defined(CONFIG_STM32F7_UART4)
#  undef CONFIG_UART0_SERIAL_CONSOLE
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  undef CONFIG_UART2_SERIAL_CONSOLE
#  undef CONFIG_UART3_SERIAL_CONSOLE
#  undef CONFIG_USART0_SERIAL_CONSOLE
#  undef CONFIG_USART1_SERIAL_CONSOLE
#  undef CONFIG_USART2_SERIAL_CONSOLE
#  define HAVE_SERIAL_CONSOLE 1
#elif defined(CONFIG_USART0_SERIAL_CONSOLE) && defined(CONFIG_STM32F7_USART0)
#  undef CONFIG_UART0_SERIAL_CONSOLE
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  undef CONFIG_UART2_SERIAL_CONSOLE
#  undef CONFIG_UART3_SERIAL_CONSOLE
#  undef CONFIG_UART4_SERIAL_CONSOLE
#  undef CONFIG_USART1_SERIAL_CONSOLE
#  undef CONFIG_USART2_SERIAL_CONSOLE
#  define HAVE_SERIAL_CONSOLE 1
#elif defined(CONFIG_USART1_SERIAL_CONSOLE) && defined(CONFIG_STM32F7_USART1)
#  undef CONFIG_UART0_SERIAL_CONSOLE
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  undef CONFIG_UART2_SERIAL_CONSOLE
#  undef CONFIG_UART3_SERIAL_CONSOLE
#  undef CONFIG_UART4_SERIAL_CONSOLE
#  undef CONFIG_USART0_SERIAL_CONSOLE
#  undef CONFIG_USART2_SERIAL_CONSOLE
#  define HAVE_SERIAL_CONSOLE 1
#elif defined(CONFIG_USART2_SERIAL_CONSOLE) && defined(CONFIG_STM32F7_USART2)
#  undef CONFIG_UART0_SERIAL_CONSOLE
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  undef CONFIG_UART2_SERIAL_CONSOLE
#  undef CONFIG_UART3_SERIAL_CONSOLE
#  undef CONFIG_UART4_SERIAL_CONSOLE
#  undef CONFIG_USART0_SERIAL_CONSOLE
#  undef CONFIG_USART1_SERIAL_CONSOLE
#  define HAVE_SERIAL_CONSOLE 1
#else
#  undef CONFIG_UART0_SERIAL_CONSOLE
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  undef CONFIG_UART2_SERIAL_CONSOLE
#  undef CONFIG_UART3_SERIAL_CONSOLE
#  undef CONFIG_UART4_SERIAL_CONSOLE
#  undef CONFIG_USART0_SERIAL_CONSOLE
#  undef CONFIG_USART1_SERIAL_CONSOLE
#  undef CONFIG_USART2_SERIAL_CONSOLE
#endif

/* SPI ******************************************************************************/
/* Don't enable SPI peripherals not supported by the chip. */

#if CHIP_NSPI < 1
#  undef CONFIG_STM32F7_SPI0
#  undef CONFIG_STM32F7_SPI1
#elif CHIP_NSPI < 2
#  undef CONFIG_STM32F7_SPI1
#endif

#ifndef CONFIG_STM32F7_HAVE_SPI
#  undef CONFIG_STM32F7_SPI0
#  undef CONFIG_STM32F7_SPI1
#endif

/* Are any SPI peripherals enabled? */

#if !defined(CONFIG_STM32F7_SPI0) && !defined(CONFIG_STM32F7_SPI0)
#  undef CONFIG_STM32F7_HAVE_SPI
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_STM32F7_STM32_CONFIG_H */
