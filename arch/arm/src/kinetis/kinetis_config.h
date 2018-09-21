/************************************************************************************
 * arch/arm/src/kinetis/kinetis_config.h
 *
 *   Copyright (C) 2011, 2017-2018 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            David Sidrane<david_s5@nscdg.com>
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

#ifndef __ARCH_ARM_SRC_KINETIS_KINETIS_CONFIG_H
#define __ARCH_ARM_SRC_KINETIS_KINETIS_CONFIG_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <arch/board/board.h>

#include "chip.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Configuration *********************************************************************/
/* Make sure that no unsupported UARTs are enabled */

#ifndef KINETIS_NLPUART
#  define KINETIS_NLPUART 0
#endif

#ifndef KINETIS_NISO7816
#  define KINETIS_NISO7816 0
#endif

#if (KINETIS_NISO7816 + KINETIS_NUART) < 6
#  undef CONFIG_KINETIS_UART5
#  if (KINETIS_NISO7816 + KINETIS_NUART) < 5
#    undef CONFIG_KINETIS_UART4
#    if (KINETIS_NISO7816 + KINETIS_NUART) < 4
#      undef CONFIG_KINETIS_UART3
#      if (KINETIS_NISO7816 + KINETIS_NUART) < 3
#        undef CONFIG_KINETIS_UART2
#        if (KINETIS_NISO7816 + KINETIS_NUART) < 2
#          undef CONFIG_KINETIS_UART1
#          if (KINETIS_NISO7816 + KINETIS_NUART) < 1
#            undef CONFIG_KINETIS_UART0
#          endif
#        endif
#      endif
#    endif
#  endif
#endif

#if KINETIS_NLPUART < 1
#  undef CONFIG_KINETIS_LPUART0
#endif
#if KINETIS_NLPUART < 2
#  undef CONFIG_KINETIS_LPUART1
#endif
#if KINETIS_NLPUART < 3
#  undef CONFIG_KINETIS_LPUART2
#endif
#if KINETIS_NLPUART < 4
#  undef CONFIG_KINETIS_LPUART3
#endif
#if KINETIS_NLPUART < 5
#  undef CONFIG_KINETIS_LPUART4
#endif

/* Are any UARTs or LPUARTs enabled? */

#undef HAVE_UART_DEVICE
#if defined(CONFIG_KINETIS_UART0) || defined(CONFIG_KINETIS_UART1) || \
    defined(CONFIG_KINETIS_UART2) || defined(CONFIG_KINETIS_UART3) || \
    defined(CONFIG_KINETIS_UART4) || defined(CONFIG_KINETIS_UART5)
#  define HAVE_UART_DEVICE 1
#endif

#undef HAVE_LPUART_DEVICE
#if defined(CONFIG_KINETIS_LPUART0) || defined(CONFIG_KINETIS_LPUART1) || \
    defined(CONFIG_KINETIS_LPUART2) || defined(CONFIG_KINETIS_LPUART3) || \
    defined(CONFIG_KINETIS_LPUART4)
#  define HAVE_LPUART_DEVICE 1
#endif

/* Is there a serial console? There should be at most one defined.  It could be on
 * any UARTn, n=0,1,2,3,4,5
 */

#undef HAVE_UART_CONSOLE
#undef HAVE_LPUART_CONSOLE

#if defined(CONFIG_CONSOLE_SYSLOG)
#    undef CONFIG_UART0_SERIAL_CONSOLE
#    undef CONFIG_UART1_SERIAL_CONSOLE
#    undef CONFIG_UART2_SERIAL_CONSOLE
#    undef CONFIG_UART3_SERIAL_CONSOLE
#    undef CONFIG_UART4_SERIAL_CONSOLE
#    undef CONFIG_UART5_SERIAL_CONSOLE
#    undef CONFIG_LPUART0_SERIAL_CONSOLE
#    undef CONFIG_LPUART1_SERIAL_CONSOLE
#    undef CONFIG_LPUART2_SERIAL_CONSOLE
#    undef CONFIG_LPUART3_SERIAL_CONSOLE
#    undef CONFIG_LPUART4_SERIAL_CONSOLE
#else
#  if defined(CONFIG_UART0_SERIAL_CONSOLE) && defined(CONFIG_KINETIS_UART0)
#    undef CONFIG_UART1_SERIAL_CONSOLE
#    undef CONFIG_UART2_SERIAL_CONSOLE
#    undef CONFIG_UART3_SERIAL_CONSOLE
#    undef CONFIG_UART4_SERIAL_CONSOLE
#    undef CONFIG_UART5_SERIAL_CONSOLE
#    undef CONFIG_LPUART0_SERIAL_CONSOLE
#    undef CONFIG_LPUART1_SERIAL_CONSOLE
#    undef CONFIG_LPUART2_SERIAL_CONSOLE
#    undef CONFIG_LPUART3_SERIAL_CONSOLE
#    undef CONFIG_LPUART4_SERIAL_CONSOLE
#    define HAVE_UART_CONSOLE 1
#  elif defined(CONFIG_UART1_SERIAL_CONSOLE) && defined(CONFIG_KINETIS_UART1)
#    undef CONFIG_UART0_SERIAL_CONSOLE
#    undef CONFIG_UART2_SERIAL_CONSOLE
#    undef CONFIG_UART3_SERIAL_CONSOLE
#    undef CONFIG_UART4_SERIAL_CONSOLE
#    undef CONFIG_UART5_SERIAL_CONSOLE
#    undef CONFIG_LPUART0_SERIAL_CONSOLE
#    undef CONFIG_LPUART1_SERIAL_CONSOLE
#    undef CONFIG_LPUART2_SERIAL_CONSOLE
#    undef CONFIG_LPUART3_SERIAL_CONSOLE
#    undef CONFIG_LPUART4_SERIAL_CONSOLE
#    define HAVE_UART_CONSOLE 1
#  elif defined(CONFIG_UART2_SERIAL_CONSOLE) && defined(CONFIG_KINETIS_UART2)
#    undef CONFIG_UART0_SERIAL_CONSOLE
#    undef CONFIG_UART1_SERIAL_CONSOLE
#    undef CONFIG_UART3_SERIAL_CONSOLE
#    undef CONFIG_UART4_SERIAL_CONSOLE
#    undef CONFIG_UART5_SERIAL_CONSOLE
#    undef CONFIG_LPUART0_SERIAL_CONSOLE
#    undef CONFIG_LPUART1_SERIAL_CONSOLE
#    undef CONFIG_LPUART2_SERIAL_CONSOLE
#    undef CONFIG_LPUART3_SERIAL_CONSOLE
#    undef CONFIG_LPUART4_SERIAL_CONSOLE
#    define HAVE_UART_CONSOLE 1
#  elif defined(CONFIG_UART3_SERIAL_CONSOLE) && defined(CONFIG_KINETIS_UART3)
#    undef CONFIG_UART0_SERIAL_CONSOLE
#    undef CONFIG_UART1_SERIAL_CONSOLE
#    undef CONFIG_UART2_SERIAL_CONSOLE
#    undef CONFIG_UART4_SERIAL_CONSOLE
#    undef CONFIG_UART5_SERIAL_CONSOLE
#    undef CONFIG_LPUART0_SERIAL_CONSOLE
#    undef CONFIG_LPUART1_SERIAL_CONSOLE
#    undef CONFIG_LPUART2_SERIAL_CONSOLE
#    undef CONFIG_LPUART3_SERIAL_CONSOLE
#    undef CONFIG_LPUART4_SERIAL_CONSOLE
#    define HAVE_UART_CONSOLE 1
#  elif defined(CONFIG_UART4_SERIAL_CONSOLE) && defined(CONFIG_KINETIS_UART4)
#    undef CONFIG_UART0_SERIAL_CONSOLE
#    undef CONFIG_UART1_SERIAL_CONSOLE
#    undef CONFIG_UART2_SERIAL_CONSOLE
#    undef CONFIG_UART3_SERIAL_CONSOLE
#    undef CONFIG_UART5_SERIAL_CONSOLE
#    undef CONFIG_LPUART0_SERIAL_CONSOLE
#    undef CONFIG_LPUART1_SERIAL_CONSOLE
#    undef CONFIG_LPUART2_SERIAL_CONSOLE
#    undef CONFIG_LPUART3_SERIAL_CONSOLE
#    undef CONFIG_LPUART4_SERIAL_CONSOLE
#    define HAVE_UART_CONSOLE 1
#  elif defined(CONFIG_UART5_SERIAL_CONSOLE) && defined(CONFIG_KINETIS_UART5)
#    undef CONFIG_UART0_SERIAL_CONSOLE
#    undef CONFIG_UART1_SERIAL_CONSOLE
#    undef CONFIG_UART2_SERIAL_CONSOLE
#    undef CONFIG_UART3_SERIAL_CONSOLE
#    undef CONFIG_UART4_SERIAL_CONSOLE
#    undef CONFIG_LPUART0_SERIAL_CONSOLE
#    undef CONFIG_LPUART1_SERIAL_CONSOLE
#    undef CONFIG_LPUART2_SERIAL_CONSOLE
#    undef CONFIG_LPUART3_SERIAL_CONSOLE
#    undef CONFIG_LPUART4_SERIAL_CONSOLE
#    define HAVE_UART_CONSOLE 1
#  elif defined(CONFIG_LPUART0_SERIAL_CONSOLE) && defined(CONFIG_KINETIS_LPUART0)
#    undef CONFIG_UART0_SERIAL_CONSOLE
#    undef CONFIG_UART1_SERIAL_CONSOLE
#    undef CONFIG_UART2_SERIAL_CONSOLE
#    undef CONFIG_UART3_SERIAL_CONSOLE
#    undef CONFIG_UART4_SERIAL_CONSOLE
#    undef CONFIG_UART5_SERIAL_CONSOLE
#    undef CONFIG_LPUART1_SERIAL_CONSOLE
#    undef CONFIG_LPUART2_SERIAL_CONSOLE
#    undef CONFIG_LPUART3_SERIAL_CONSOLE
#    undef CONFIG_LPUART4_SERIAL_CONSOLE
#    define HAVE_LPUART_CONSOLE 1
#  elif defined(CONFIG_LPUART1_SERIAL_CONSOLE) && defined(CONFIG_KINETIS_LPUART1)
#    undef CONFIG_UART0_SERIAL_CONSOLE
#    undef CONFIG_UART1_SERIAL_CONSOLE
#    undef CONFIG_UART2_SERIAL_CONSOLE
#    undef CONFIG_UART3_SERIAL_CONSOLE
#    undef CONFIG_UART4_SERIAL_CONSOLE
#    undef CONFIG_UART5_SERIAL_CONSOLE
#    undef CONFIG_LPUART0_SERIAL_CONSOLE
#    undef CONFIG_LPUART2_SERIAL_CONSOLE
#    undef CONFIG_LPUART3_SERIAL_CONSOLE
#    undef CONFIG_LPUART4_SERIAL_CONSOLE
#    define HAVE_LPUART_CONSOLE 1
#  elif defined(CONFIG_LPUART2_SERIAL_CONSOLE) && defined(CONFIG_KINETIS_LPUART2)
#    undef CONFIG_UART0_SERIAL_CONSOLE
#    undef CONFIG_UART1_SERIAL_CONSOLE
#    undef CONFIG_UART2_SERIAL_CONSOLE
#    undef CONFIG_UART3_SERIAL_CONSOLE
#    undef CONFIG_UART4_SERIAL_CONSOLE
#    undef CONFIG_UART5_SERIAL_CONSOLE
#    undef CONFIG_LPUART0_SERIAL_CONSOLE
#    undef CONFIG_LPUART1_SERIAL_CONSOLE
#    undef CONFIG_LPUART3_SERIAL_CONSOLE
#    undef CONFIG_LPUART4_SERIAL_CONSOLE
#    define HAVE_LPUART_CONSOLE 1
#  elif defined(CONFIG_LPUART3_SERIAL_CONSOLE) && defined(CONFIG_KINETIS_LPUART3)
#    undef CONFIG_UART0_SERIAL_CONSOLE
#    undef CONFIG_UART1_SERIAL_CONSOLE
#    undef CONFIG_UART2_SERIAL_CONSOLE
#    undef CONFIG_UART3_SERIAL_CONSOLE
#    undef CONFIG_UART4_SERIAL_CONSOLE
#    undef CONFIG_UART5_SERIAL_CONSOLE
#    undef CONFIG_LPUART0_SERIAL_CONSOLE
#    undef CONFIG_LPUART1_SERIAL_CONSOLE
#    undef CONFIG_LPUART2_SERIAL_CONSOLE
#    undef CONFIG_LPUART4_SERIAL_CONSOLE
#    define HAVE_LPUART_CONSOLE 1
#  elif defined(CONFIG_LPUART4_SERIAL_CONSOLE) && defined(CONFIG_KINETIS_LPUART4)
#    undef CONFIG_UART0_SERIAL_CONSOLE
#    undef CONFIG_UART1_SERIAL_CONSOLE
#    undef CONFIG_UART2_SERIAL_CONSOLE
#    undef CONFIG_UART3_SERIAL_CONSOLE
#    undef CONFIG_UART4_SERIAL_CONSOLE
#    undef CONFIG_UART5_SERIAL_CONSOLE
#    undef CONFIG_LPUART0_SERIAL_CONSOLE
#    undef CONFIG_LPUART1_SERIAL_CONSOLE
#    undef CONFIG_LPUART2_SERIAL_CONSOLE
#    undef CONFIG_LPUART3_SERIAL_CONSOLE
#    define HAVE_LPUART_CONSOLE 1
#  else
#    ifdef CONFIG_DEV_CONSOLE
#      warning "No valid CONFIG_[LP]UART[n]_SERIAL_CONSOLE Setting"
#    endif
#    undef CONFIG_UART0_SERIAL_CONSOLE
#    undef CONFIG_UART1_SERIAL_CONSOLE
#    undef CONFIG_UART2_SERIAL_CONSOLE
#    undef CONFIG_UART3_SERIAL_CONSOLE
#    undef CONFIG_UART4_SERIAL_CONSOLE
#    undef CONFIG_UART5_SERIAL_CONSOLE
#    undef CONFIG_LPUART0_SERIAL_CONSOLE
#    undef CONFIG_LPUART1_SERIAL_CONSOLE
#    undef CONFIG_LPUART2_SERIAL_CONSOLE
#    undef CONFIG_LPUART3_SERIAL_CONSOLE
#    undef CONFIG_LPUART4_SERIAL_CONSOLE
#  endif
#endif

/* Which version of up_putc() should be built?
 *
 * --------------------+-------------------+-----------------+------------------
 *                      HAVE_UART_DEVICE && HAVE_UART_DEVICE  HAVE_LPUART_DEVICE
 *                      HAVE_LPUART_DEVICE  (only)            (only)
 * --------------------+-------------------+-----------------+------------------
 * HAVE_UART_CONSOLE    kinetis_serial      kinetis_serial    (impossible)
 * HAVE_LPUART_CONSOLE  kinetis_lpserial    (impossible)      kinetis_lpserial
 * No serial console    kinetis_serial      kinetis_serial    kinetis_lpserial
 * --------------------+-------------------+-----------------+------------------
 */

#undef HAVE_UART_PUTC
#undef HAVE_LPUART_PUTC

#if defined(HAVE_LPUART_CONSOLE)
#  define HAVE_LPUART_PUTC 1
#elif defined(HAVE_UART_CONSOLE)
#  define HAVE_UART_PUTC   1
#elif defined(HAVE_UART_DEVICE)
#  define HAVE_UART_PUTC   1
#elif defined(HAVE_LPUART_DEVICE)
#  define HAVE_LPUART_PUTC 1
#endif

/* Check UART flow control (Not yet supported) */

# undef CONFIG_UART0_FLOWCONTROL
# undef CONFIG_UART1_FLOWCONTROL
# undef CONFIG_UART2_FLOWCONTROL
# undef CONFIG_UART3_FLOWCONTROL
# undef CONFIG_UART4_FLOWCONTROL
# undef CONFIG_UART5_FLOWCONTROL
# undef CONFIG_LPUART0_FLOWCONTROL
# undef CONFIG_LPUART1_FLOWCONTROL
# undef CONFIG_LPUART2_FLOWCONTROL
# undef CONFIG_LPUART3_FLOWCONTROL
# undef CONFIG_LPUART4_FLOWCONTROL

/* UART FIFO support is not fully implemented.
 *
 * NOTE:  UART0 has an 8-byte deep FIFO; the other UARTs have no FIFOs
 * (1-deep).  There appears to be no way to know when the FIFO is not
 * full (other than reading the FIFO length and comparing the FIFO count).
 * Hence, the FIFOs are not used in this implementation and, as a result
 * TDRE indeed mean that the single output buffer is available.
 *
 * Performance on UART0 could be improved by enabling the FIFO and by
 * redesigning all of the FIFO status logic.
 */

#undef CONFIG_KINETIS_UARTFIFOS

/* Ethernet controller configuration */

#ifndef CONFIG_ENET_NRXBUFFERS
#  define CONFIG_ENET_NRXBUFFERS 6
#endif
#ifndef CONFIG_ENET_NTXBUFFERS
#  define CONFIG_ENET_NTXBUFFERS 2
#endif

#ifndef CONFIG_ENET_PHYADDR
#  define CONFIG_ENET_PHYADDR 1
#endif

#ifndef CONFIG_ENETNETHIFS
# define CONFIG_ENETNETHIFS 1
#endif

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Inline Functions
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_KINETIS_KINETIS_CONFIG_H */
