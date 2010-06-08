/************************************************************************************
 * arch/arm/src/lpc17xx/lpc17_serial.h
 *
 *   Copyright (C) 2010 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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

#ifndef __ARCH_ARM_SRC_LPC17XX_LPC17_SERIAL_H
#define __ARCH_ARM_SRC_LPC17XX_LPC17_SERIAL_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Configuration *********************************************************************/

/* Is there a serial console? It could be on any UARTn, n=0,1,2,3 */

#if defined(CONFIG_UART0_SERIAL_CONSOLE) && defined(CONFIG_LPC17_UART0)
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  undef CONFIG_UART2_SERIAL_CONSOLE
#  undef CONFIG_UART3_SERIAL_CONSOLE
#  define HAVE_CONSOLE 1
#elif defined(CONFIG_UART1_SERIAL_CONSOLE) && defined(CONFIG_LPC17_UART1)
#  undef CONFIG_UART0_SERIAL_CONSOLE
#  undef CONFIG_UART2_SERIAL_CONSOLE
#  undef CONFIG_UART3_SERIAL_CONSOLE
#  define HAVE_CONSOLE 1
#elif defined(CONFIG_UART2_SERIAL_CONSOLE) && defined(CONFIG_LPC17_UART2)
#  undef CONFIG_UART0_SERIAL_CONSOLE
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  undef CONFIG_UART3_SERIAL_CONSOLE
#  define HAVE_CONSOLE 1
#elif defined(CONFIG_UART3_SERIAL_CONSOLE) && defined(CONFIG_LPC17_UART3)
#  undef CONFIG_UART0_SERIAL_CONSOLE
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  undef CONFIG_UART2_SERIAL_CONSOLE
#  define HAVE_CONSOLE 1
#else
#  undef CONFIG_UART0_SERIAL_CONSOLE
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  undef CONFIG_UART2_SERIAL_CONSOLE
#  undef CONFIG_UART3_SERIAL_CONSOLE
#  undef HAVE_CONSOLE
#endif

/* Select UART parameters for the selected console */

#if defined(CONFIG_UART0_SERIAL_CONSOLE)
#  define CONSOLE_BASE     LPC17_UART0_BASE
#  define CONSOLE_BAUD     CONFIG_UART0_BAUD
#  define CONSOLE_BITS     CONFIG_UART0_BITS
#  define CONSOLE_PARITY   CONFIG_UART0_PARITY
#  define CONSOLE_2STOP    CONFIG_UART0_2STOP
#elif defined(CONFIG_UART1_SERIAL_CONSOLE)
#  define CONSOLE_BASE     LPC17_UART1_BASE
#  define CONSOLE_BAUD     CONFIG_UART1_BAUD
#  define CONSOLE_BITS     CONFIG_UART1_BITS
#  define CONSOLE_PARITY   CONFIG_UART1_PARITY
#  define CONSOLE_2STOP    CONFIG_UART1_2STOP
#elif defined(CONFIG_UART2_SERIAL_CONSOLE)
#  define CONSOLE_BASE     LPC17_UART2_BASE
#  define CONSOLE_BAUD     CONFIG_UART2_BAUD
#  define CONSOLE_BITS     CONFIG_UART2_BITS
#  define CONSOLE_PARITY   CONFIG_UART2_PARITY
#  define CONSOLE_2STOP    CONFIG_UART2_2STOP
#elif defined(CONFIG_UART3_SERIAL_CONSOLE)
#  define CONSOLE_BASE     LPC17_UART3_BASE
#  define CONSOLE_BAUD     CONFIG_UART3_BAUD
#  define CONSOLE_BITS     CONFIG_UART3_BITS
#  define CONSOLE_PARITY   CONFIG_UART3_PARITY
#  define CONSOLE_2STOP    CONFIG_UART3_2STOP
#else
#  error "No CONFIG_UARTn_SERIAL_CONSOLE Setting"
#endif

/* Get word length setting for the console UART and UART0-3 */

#if CONSOLE_BITS == 5
#  define CONSOLE_LCR_WLS UART_LCR_WLS_5BIT
#elif CONSOLE_BITS == 6
#  define CONSOLE_LCR_WLS UART_LCR_WLS_6BIT
#elif CONSOLE_BITS == 7
#  define CONSOLE_LCR_WLS UART_LCR_WLS_7BIT
#elif CONSOLE_BITS == 8
#  define CONSOLE_LCR_WLS UART_LCR_WLS_8BIT
#else
#  error "Invalid CONFIG_UARTn_BITS setting for console "
#endif

#ifdef CONFIG_LPC17_UART0
#  if CONFIG_UART0_BITS == 5
#    define UART0_LCR_WLS UART_LCR_WLS_5BIT
#  elif CONFIG_UART0_BITS == 6
#    define UART0_LCR_WLS UART_LCR_WLS_6BIT
#  elif CONFIG_UART0_BITS == 7
#    define UART0_LCR_WLS UART_LCR_WLS_7BIT
#  elif CONFIG_UART0_BITS == 8
#    define UART0_LCR_WLS UART_LCR_WLS_8BIT
#  else
#    error "Invalid CONFIG_UARTn_BITS setting for UART0 "
#  endif
#endif

#ifdef CONFIG_LPC17_UART1
#  if CONFIG_UART1_BITS == 5
#    define UART1_LCR_WLS UART_LCR_WLS_5BIT
#  elif CONFIG_UART1_BITS == 6
#    define UART1_LCR_WLS UART_LCR_WLS_6BIT
#  elif CONFIG_UART1_BITS == 7
#    define UART1_LCR_WLS UART_LCR_WLS_7BIT
#  elif CONFIG_UART1_BITS == 8
#    define UART1_LCR_WLS UART_LCR_WLS_8BIT
#  else
#    error "Invalid CONFIG_UARTn_BITS setting for UART1 "
#  endif
#endif

#ifdef CONFIG_LPC17_UART2
#  if CONFIG_UART2_BITS == 5
#    define UART2_LCR_WLS UART_LCR_WLS_5BIT
#  elif CONFIG_UART2_BITS == 6
#    define UART2_LCR_WLS UART_LCR_WLS_6BIT
#  elif CONFIG_UART2_BITS == 7
#    define UART2_LCR_WLS UART_LCR_WLS_7BIT
#  elif CONFIG_UART2_BITS == 8
#    define UART2_LCR_WLS UART_LCR_WLS_8BIT
#  else
#    error "Invalid CONFIG_UARTn_BITS setting for UART2 "
#  endif
#endif

#ifdef CONFIG_LPC17_UART3
#  if CONFIG_UART3_BITS == 5
#    define UART3_LCR_WLS UART_LCR_WLS_5BIT
#  elif CONFIG_UART3_BITS == 6
#    define UART3_LCR_WLS UART_LCR_WLS_6BIT
#  elif CONFIG_UART3_BITS == 7
#    define UART3_LCR_WLS UART_LCR_WLS_7BIT
#  elif CONFIG_UART3_BITS == 8
#    define UART3_LCR_WLS UART_LCR_WLS_8BIT
#  else
#    error "Invalid CONFIG_UARTn_BITS setting for UART3 "
#  endif
#endif

/* Get parity setting for the console UART and UART0-3 */

#if CONSOLE_PARITY == 0
#  define CONSOLE_LCR_PAR 0
#elif CONSOLE_PARITY == 1
#  define CONSOLE_LCR_PAR (UART_LCR_PE|UART_LCR_PS_ODD)
#elif CONSOLE_PARITY == 2
#  define CONSOLE_LCR_PAR (UART_LCR_PE|UART_LCR_PS_EVEN)
#elif CONSOLE_PARITY == 3
#  define CONSOLE_LCR_PAR (UART_LCR_PE|UART_LCR_PS_STICK1)
#elif CONSOLE_PARITY == 4
#  define CONSOLE_LCR_PAR (UART_LCR_PE|UART_LCR_PS_STICK0)
#else
#    error "Invalid CONFIG_UARTn_PARITY setting for CONSOLE"
#endif

#ifdef CONFIG_LPC17_UART0
#  if CONFIG_UART0_PARITY == 0
#    define UART0_LCR_PAR 0
#  elif CONFIG_UART0_PARITY == 1
#    define UART0_LCR_PAR (UART_LCR_PE|UART_LCR_PS_ODD)
#  elif CONFIG_UART0_PARITY == 2
#    define UART0_LCR_PAR (UART_LCR_PE|UART_LCR_PS_EVEN)
#  elif CONFIG_UART0_PARITY == 3
#    define UART0_LCR_PAR (UART_LCR_PE|UART_LCR_PS_STICK1)
#  elif CONFIG_UART0_PARITY == 4
#    define UART0_LCR_PAR (UART_LCR_PE|UART_LCR_PS_STICK0)
#  else
#    error "Invalid CONFIG_UARTn_PARITY setting for UART0"
#  endif
#endif

#ifdef CONFIG_LPC17_UART1
#  if CONFIG_UART1_PARITY == 0
#    define UART1_LCR_PAR 0
#  elif CONFIG_UART1_PARITY == 1
#    define UART1_LCR_PAR (UART_LCR_PE|UART_LCR_PS_ODD)
#  elif CONFIG_UART1_PARITY == 2
#    define UART1_LCR_PAR (UART_LCR_PE|UART_LCR_PS_EVEN)
#  elif CONFIG_UART1_PARITY == 3
#    define UART1_LCR_PAR (UART_LCR_PE|UART_LCR_PS_STICK1)
#  elif CONFIG_UART1_PARITY == 4
#    define UART1_LCR_PAR (UART_LCR_PE|UART_LCR_PS_STICK0)
#  else
#    error "Invalid CONFIG_UARTn_PARITY setting for UART1"
#  endif
#endif

#ifdef CONFIG_LPC17_UART2
#  if CONFIG_UART2_PARITY == 0
#    define UART2_LCR_PAR 0
#  elif CONFIG_UART2_PARITY == 1
#    define UART2_LCR_PAR (UART_LCR_PE|UART_LCR_PS_ODD)
#  elif CONFIG_UART2_PARITY == 2
#    define UART2_LCR_PAR (UART_LCR_PE|UART_LCR_PS_EVEN)
#  elif CONFIG_UART2_PARITY == 3
#    define UART2_LCR_PAR (UART_LCR_PE|UART_LCR_PS_STICK1)
#  elif CONFIG_UART2_PARITY == 4
#    define UART2_LCR_PAR (UART_LCR_PE|UART_LCR_PS_STICK0)
#  else
#    error "Invalid CONFIG_UARTn_PARITY setting for UART2"
#  endif
#endif

#ifdef CONFIG_LPC17_UART3
#  if CONFIG_UART3_PARITY == 0
#    define UART3_LCR_PAR 0
#  elif CONFIG_UART3_PARITY == 1
#    define UART3_LCR_PAR (UART_LCR_PE|UART_LCR_PS_ODD)
#  elif CONFIG_UART3_PARITY == 2
#    define UART3_LCR_PAR (UART_LCR_PE|UART_LCR_PS_EVEN)
#  elif CONFIG_UART3_PARITY == 3
#    define UART3_LCR_PAR (UART_LCR_PE|UART_LCR_PS_STICK1)
#  elif CONFIG_UART3_PARITY == 4
#    define UART3_LCR_PAR (UART_LCR_PE|UART_LCR_PS_STICK0)
#  else
#    error "Invalid CONFIG_UARTn_PARITY setting for UART3"
#  endif
#endif

/* Get stop-bit setting for the console UART and UART0-3 */

#if CONSOLE_2STOP != 0
#  define CONSOLE_LCR_STOP LPC214X_LCR_STOP_2
#else
#  define CONSOLE_LCR_STOP LPC214X_LCR_STOP_1
#endif

#if CONFIG_UART0_2STOP != 0
#  define UART0_LCR_STOP LPC214X_LCR_STOP_2
#else
#  define UART0_LCR_STOP LPC214X_LCR_STOP_1
#endif

#if CONFIG_UART1_2STOP != 0
#  define UART1_LCR_STOP LPC214X_LCR_STOP_2
#else
#  define UART1_LCR_STOP LPC214X_LCR_STOP_1
#endif

#if CONFIG_UART2_2STOP != 0
#  define UART2_LCR_STOP LPC214X_LCR_STOP_2
#else
#  define UART2_LCR_STOP LPC214X_LCR_STOP_1
#endif

#if CONFIG_UART3_2STOP != 0
#  define UART3_LCR_STOP LPC214X_LCR_STOP_2
#else
#  define UART3_LCR_STOP LPC214X_LCR_STOP_1
#endif

/* LCR and FCR values */

#define CONSOLE_LCR_VALUE (CONSOLE_LCR_WLS | CONSOLE_LCR_PAR | CONSOLE_LCR_STOP)
#define CONSOLE_FCR_VALUE (UART_FCR_RXTRIGGER_8 | UART_FCR_TXRST |\
                           UART_FCR_RXRST | UART_FCR_FIFOEN)

#define UART0_LCR_VALUE   (UART0_LCR_WLS | UART0_LCR_PAR | UART0_LCR_STOP)
#define UART0_FCR_VALUE   (UART_FCR_RXTRIGGER_8 | UART_FCR_TXRST |\
                           UART_FCR_RXRST | UART_FCR_FIFOEN)

#define UART1_LCR_VALUE   (UART1_LCR_WLS | UART1_LCR_PAR | UART1_LCR_STOP)
#define UART1_FCR_VALUE   (UART_FCR_RXTRIGGER_8 | UART_FCR_TXRST |\
                           UART_FCR_RXRST | UART_FCR_FIFOEN)

#define UART2_LCR_VALUE   (UART2_LCR_WLS | UART2_LCR_PAR | UART2_LCR_STOP)
#define UART2_FCR_VALUE   (UART_FCR_RXTRIGGER_8 | UART_FCR_TXRST |\
                           UART_FCR_RXRST | UART_FCR_FIFOEN)

#define UART3_LCR_VALUE   (UART3_LCR_WLS | UART3_LCR_PAR | UART3_LCR_STOP)
#define UART3_FCR_VALUE   (UART_FCR_RXTRIGGER_8 | UART_FCR_TXRST |\
                           UART_FCR_RXRST | UART_FCR_FIFOEN)

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_LPC17XX_LPC17_SERIAL_H */
