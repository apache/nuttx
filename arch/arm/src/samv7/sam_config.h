/****************************************************************************
 * arch/arm/src/samv7/sam_config.h
 *
 *   Copyright (C) 2015, 2018 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_SAMV7_SAMV7_CONFIG_H
#define __ARCH_ARM_SRC_SAMV7_SAMV7_CONFIG_H

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

#ifndef CONFIG_SAMV7_GPIO_IRQ
#  undef CONFIG_SAMV7_GPIOA_IRQ
#  undef CONFIG_SAMV7_GPIOB_IRQ
#  undef CONFIG_SAMV7_GPIOC_IRQ
#  undef CONFIG_SAMV7_GPIOD_IRQ
#  undef CONFIG_SAMV7_GPIOE_IRQ
#endif

#if SAMV7_NPIO < 1
#  undef CONFIG_SAMV7_GPIOA_IRQ
#endif
#if SAMV7_NPIO < 2
#  undef CONFIG_SAMV7_GPIOB_IRQ
#endif
#if SAMV7_NPIO < 3
#  undef CONFIG_SAMV7_GPIOC_IRQ
#endif
#if SAMV7_NPIO < 4
#  undef CONFIG_SAMV7_GPIOD_IRQ
#endif
#if SAMV7_NPIO < 5
#  undef CONFIG_SAMV7_GPIOE_IRQ
#endif

/* UARTs ********************************************************************/
/* Don't enable UARTs not supported by the chip. */

#if SAMV7_NUART < 1
#  undef CONFIG_SAMV7_UART0
#  undef CONFIG_SAMV7_UART1
#  undef CONFIG_SAMV7_UART2
#  undef CONFIG_SAMV7_UART3
#  undef CONFIG_SAMV7_UART4
#elif SAMV7_NUART < 2
#  undef CONFIG_SAMV7_UART1
#  undef CONFIG_SAMV7_UART2
#  undef CONFIG_SAMV7_UART3
#  undef CONFIG_SAMV7_UART4
#elif SAMV7_NUART < 3
#  undef CONFIG_SAMV7_UART2
#  undef CONFIG_SAMV7_UART3
#  undef CONFIG_SAMV7_UART4
#elif SAMV7_NUART < 4
#  undef CONFIG_SAMV7_UART3
#  undef CONFIG_SAMV7_UART4
#elif SAMV7_NUART < 5
#  undef CONFIG_SAMV7_UART4
#endif

/* Are any UARTs enabled? */

#undef HAVE_UART_DEVICE
#if defined(CONFIG_SAMV7_UART0) || defined(CONFIG_SAMV7_UART1) || \
    defined(CONFIG_SAMV7_UART2) || defined(CONFIG_SAMV7_UART3) || \
    defined(CONFIG_SAMV7_UART4)
#  define HAVE_UART_DEVICE 1
#endif

/* USARTs *******************************************************************/
/* If the USART is not being used as a UART or for SPI, then it really isn't
 * enabled for our purposes.
 */

#if !defined(CONFIG_USART0_SERIALDRIVER) && !defined(CONFIG_USART0_ISSPI)
#  undef CONFIG_SAMV7_USART0
#  undef CONFIG_USART0_SERIAL_CONSOLE
#  undef CONFIG_USART0_IFLOWCONTROL
#endif

#if !defined(CONFIG_USART1_SERIALDRIVER) && !defined(CONFIG_USART1_ISSPI)
#  undef CONFIG_SAMV7_USART1
#  undef CONFIG_USART1_SERIAL_CONSOLE
#  undef CONFIG_USART1_IFLOWCONTROL
#endif

#if !defined(CONFIG_USART2_SERIALDRIVER) && !defined(CONFIG_USART2_ISSPI)
#  undef CONFIG_SAMV7_USART2
#  undef CONFIG_USART2_SERIAL_CONSOLE
#  undef CONFIG_USART2_IFLOWCONTROL
#endif

/* Don't enable USARTs not supported by the chip. */

#if SAMV7_NUSART < 1
#  undef CONFIG_SAMV7_USART0
#  undef CONFIG_SAMV7_USART1
#  undef CONFIG_SAMV7_USART2
#elif SAMV7_NUSART < 2
#  undef CONFIG_SAMV7_USART1
#  undef CONFIG_SAMV7_USART2
#elif SAMV7_NUSART < 3
#  undef CONFIG_SAMV7_USART2
#endif

/* Are any USARTs enabled?
 *
 * REVISIT: Setting HAVE_UART_DEVICE only makes sense of the USART is being
 * used as a UART.
 */

#if defined(CONFIG_SAMV7_USART0) || defined(CONFIG_SAMV7_USART1) || \
    defined(CONFIG_SAMV7_USART2)
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

/* Hardware flow control requires using a DMAC channel (not yet supported) */

#ifdef CONFIG_SERIAL_IFLOWCONTROL
#  warning XDMAC support is required for RTS hardware flow control
#  undef CONFIG_SERIAL_IFLOWCONTROL
#  undef CONFIG_USART0_IFLOWCONTROL
#  undef CONFIG_USART1_IFLOWCONTROL
#  undef CONFIG_USART2_IFLOWCONTROL
#endif

/* Serial Console ***********************************************************/
/* Is there a serial console?  There should be no more than one defined.  It
 * could be on any UARTn, n=1,..,SAMV7_NUART, or USARTn, n=1,.., SAMV7_NUSART
 */

#undef HAVE_SERIAL_CONSOLE
#if defined(CONFIG_UART0_SERIAL_CONSOLE) && defined(CONFIG_SAMV7_UART0)
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  undef CONFIG_UART2_SERIAL_CONSOLE
#  undef CONFIG_UART3_SERIAL_CONSOLE
#  undef CONFIG_UART4_SERIAL_CONSOLE
#  undef CONFIG_USART0_SERIAL_CONSOLE
#  undef CONFIG_USART1_SERIAL_CONSOLE
#  undef CONFIG_USART2_SERIAL_CONSOLE
#  define HAVE_SERIAL_CONSOLE 1
#elif defined(CONFIG_UART1_SERIAL_CONSOLE) && defined(CONFIG_SAMV7_UART1)
#  undef CONFIG_UART0_SERIAL_CONSOLE
#  undef CONFIG_UART2_SERIAL_CONSOLE
#  undef CONFIG_UART3_SERIAL_CONSOLE
#  undef CONFIG_UART4_SERIAL_CONSOLE
#  undef CONFIG_USART0_SERIAL_CONSOLE
#  undef CONFIG_USART1_SERIAL_CONSOLE
#  undef CONFIG_USART2_SERIAL_CONSOLE
#  define HAVE_SERIAL_CONSOLE 1
#elif defined(CONFIG_UART2_SERIAL_CONSOLE) && defined(CONFIG_SAMV7_UART2)
#  undef CONFIG_UART0_SERIAL_CONSOLE
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  undef CONFIG_UART3_SERIAL_CONSOLE
#  undef CONFIG_UART4_SERIAL_CONSOLE
#  undef CONFIG_USART0_SERIAL_CONSOLE
#  undef CONFIG_USART1_SERIAL_CONSOLE
#  undef CONFIG_USART2_SERIAL_CONSOLE
#  define HAVE_SERIAL_CONSOLE 1
#elif defined(CONFIG_UART3_SERIAL_CONSOLE) && defined(CONFIG_SAMV7_UART3)
#  undef CONFIG_UART0_SERIAL_CONSOLE
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  undef CONFIG_UART2_SERIAL_CONSOLE
#  undef CONFIG_UART4_SERIAL_CONSOLE
#  undef CONFIG_USART0_SERIAL_CONSOLE
#  undef CONFIG_USART1_SERIAL_CONSOLE
#  undef CONFIG_USART2_SERIAL_CONSOLE
#  define HAVE_SERIAL_CONSOLE 1
#elif defined(CONFIG_UART4_SERIAL_CONSOLE) && defined(CONFIG_SAMV7_UART4)
#  undef CONFIG_UART0_SERIAL_CONSOLE
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  undef CONFIG_UART2_SERIAL_CONSOLE
#  undef CONFIG_UART3_SERIAL_CONSOLE
#  undef CONFIG_USART0_SERIAL_CONSOLE
#  undef CONFIG_USART1_SERIAL_CONSOLE
#  undef CONFIG_USART2_SERIAL_CONSOLE
#  define HAVE_SERIAL_CONSOLE 1
#elif defined(CONFIG_USART0_SERIAL_CONSOLE) && defined(CONFIG_SAMV7_USART0)
#  undef CONFIG_UART0_SERIAL_CONSOLE
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  undef CONFIG_UART2_SERIAL_CONSOLE
#  undef CONFIG_UART3_SERIAL_CONSOLE
#  undef CONFIG_UART4_SERIAL_CONSOLE
#  undef CONFIG_USART1_SERIAL_CONSOLE
#  undef CONFIG_USART2_SERIAL_CONSOLE
#  define HAVE_SERIAL_CONSOLE 1
#elif defined(CONFIG_USART1_SERIAL_CONSOLE) && defined(CONFIG_SAMV7_USART1)
#  undef CONFIG_UART0_SERIAL_CONSOLE
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  undef CONFIG_UART2_SERIAL_CONSOLE
#  undef CONFIG_UART3_SERIAL_CONSOLE
#  undef CONFIG_UART4_SERIAL_CONSOLE
#  undef CONFIG_USART0_SERIAL_CONSOLE
#  undef CONFIG_USART2_SERIAL_CONSOLE
#  define HAVE_SERIAL_CONSOLE 1
#elif defined(CONFIG_USART2_SERIAL_CONSOLE) && defined(CONFIG_SAMV7_USART2)
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

#if SAMV7_NSPI < 1
#  undef CONFIG_SAMV7_SPI0
#  undef CONFIG_SAMV7_SPI0_MASTER
#  undef CONFIG_SAMV7_SPI0_SLAVE
#  undef CONFIG_SAMV7_SPI1
#  undef CONFIG_SAMV7_SPI1_MASTER
#  undef CONFIG_SAMV7_SPI1_SLAVE
#elif SAMV7_NSPI < 2
#  undef CONFIG_SAMV7_SPI1
#  undef CONFIG_SAMV7_SPI1_MASTER
#  undef CONFIG_SAMV7_SPI1_SLAVE
#endif

#ifndef CONFIG_SAMV7_SPI
#  undef CONFIG_SAMV7_SPI0
#  undef CONFIG_SAMV7_SPI0_MASTER
#  undef CONFIG_SAMV7_SPI0_SLAVE
#  undef CONFIG_SAMV7_SPI1
#  undef CONFIG_SAMV7_SPI1_MASTER
#  undef CONFIG_SAMV7_SPI1_SLAVE
#endif

/* Are any SPI peripherals enabled? */

#if !defined(CONFIG_SAMV7_SPI0) && !defined(CONFIG_SAMV7_SPI0)
#  undef CONFIG_SAMV7_SPI
#  undef CONFIG_SAMV7_SPI_MASTER
#  undef CONFIG_SAMV7_SPI_SLAVE
#endif

/* Each SPI peripheral must be enabled as a MASTER or as a SLAVE */

#ifndef CONFIG_SAMV7_SPI_MASTER
#  undef CONFIG_SAMV7_SPI0_MASTER
#  undef CONFIG_SAMV7_SPI1_MASTER
#endif

#if !defined(CONFIG_SAMV7_SPI0_MASTER) && !defined(CONFIG_SAMV7_SPI1_MASTER)
#  undef CONFIG_SAMV7_SPI_MASTER
#endif

#ifndef CONFIG_SAMV7_SPI_SLAVE
#  undef CONFIG_SAMV7_SPI0_SLAVE
#  undef CONFIG_SAMV7_SPI1_SLAVE
#endif

#if !defined(CONFIG_SAMV7_SPI0_SLAVE) && !defined(CONFIG_SAMV7_SPI1_SLAVE)
#  undef CONFIG_SAMV7_SPI_SLAVE
#endif

#if !defined(CONFIG_SAMV7_SPI_MASTER) && !defined(CONFIG_SAMV7_SPI_SLAVE)
#  undef CONFIG_SAMV7_SPI
#  undef CONFIG_SAMV7_SPI0
#  undef CONFIG_SAMV7_SPI1
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

#endif /* __ARCH_ARM_SRC_SAMV7_SAMV7_CONFIG_H */
