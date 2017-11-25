/************************************************************************************
 * arch/arm/src/xmc4/xmc4_config.h
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_XMC4_XMC4_CONFIG_H
#define __ARCH_ARM_SRC_XMC4_XMC4_CONFIG_H

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

#ifndef CONFIG_XMC4_USIC0
#  undef CONFIG_XMC4_USIC0_CHAN0_ISUART
#  undef CONFIG_XMC4_USIC0_CHAN1_ISUART
#endif
#ifndef CONFIG_XMC4_USIC1
#  undef CONFIG_XMC4_USIC1_CHAN0_ISUART
#  undef CONFIG_XMC4_USIC1_CHAN1_ISUART
#endif
#ifndef CONFIG_XMC4_USIC2
#  undef CONFIG_XMC4_USIC2_CHAN0_ISUART
#  undef CONFIG_XMC4_USIC2_CHAN1_ISUART
#endif

/* Map logical UART names (Just for simplicity of naming) */

#undef HAVE_UART0
#undef HAVE_UART1
#undef HAVE_UART2
#undef HAVE_UART3
#undef HAVE_UART4
#undef HAVE_UART5

#ifdef CONFIG_XMC4_USIC0_CHAN0_ISUART
#  define HAVE_UART0
#endif
#ifdef CONFIG_XMC4_USIC0_CHAN1_ISUART
#  define HAVE_UART1
#endif
#ifdef CONFIG_XMC4_USIC1_CHAN0_ISUART
#  define HAVE_UART2
#endif
#ifdef CONFIG_XMC4_USIC1_CHAN1_ISUART
#  define HAVE_UART3
#endif
#ifdef CONFIG_XMC4_USIC2_CHAN0_ISUART
#  define HAVE_UART4
#endif
#ifdef CONFIG_XMC4_USIC2_CHAN1_ISUART
#  define HAVE_UART5
#endif

/* Are any UARTs enabled? */

#undef HAVE_UART_DEVICE
#if defined(HAVE_UART0) || defined(HAVE_UART1) || defined(HAVE_UART2) || \
    defined(HAVE_UART3) || defined(HAVE_UART4) || defined(HAVE_UART5)
#  define HAVE_UART_DEVICE 1
#endif

/* Is there a serial console? There should be at most one defined.  It could be on
 * any UARTn, n=0,1,2,3,4,5
 */

#undef HAVE_UART_CONSOLE

#if defined(CONFIG_CONSOLE_SYSLOG)
#    undef CONFIG_UART0_SERIAL_CONSOLE
#    undef CONFIG_UART1_SERIAL_CONSOLE
#    undef CONFIG_UART2_SERIAL_CONSOLE
#    undef CONFIG_UART3_SERIAL_CONSOLE
#    undef CONFIG_UART4_SERIAL_CONSOLE
#    undef CONFIG_UART5_SERIAL_CONSOLE
#else
#  if defined(CONFIG_UART0_SERIAL_CONSOLE) && defined(HAVE_UART0)
#    undef CONFIG_UART1_SERIAL_CONSOLE
#    undef CONFIG_UART2_SERIAL_CONSOLE
#    undef CONFIG_UART3_SERIAL_CONSOLE
#    undef CONFIG_UART4_SERIAL_CONSOLE
#    undef CONFIG_UART5_SERIAL_CONSOLE
#    define HAVE_UART_CONSOLE 1
#  elif defined(CONFIG_UART1_SERIAL_CONSOLE) && defined(HAVE_UART1)
#    undef CONFIG_UART0_SERIAL_CONSOLE
#    undef CONFIG_UART2_SERIAL_CONSOLE
#    undef CONFIG_UART3_SERIAL_CONSOLE
#    undef CONFIG_UART4_SERIAL_CONSOLE
#    undef CONFIG_UART5_SERIAL_CONSOLE
#    define HAVE_UART_CONSOLE 1
#  elif defined(CONFIG_UART2_SERIAL_CONSOLE) && defined(HAVE_UART2)
#    undef CONFIG_UART0_SERIAL_CONSOLE
#    undef CONFIG_UART1_SERIAL_CONSOLE
#    undef CONFIG_UART3_SERIAL_CONSOLE
#    undef CONFIG_UART4_SERIAL_CONSOLE
#    undef CONFIG_UART5_SERIAL_CONSOLE
#    define HAVE_UART_CONSOLE 1
#  elif defined(CONFIG_UART3_SERIAL_CONSOLE) && defined(HAVE_UART3)
#    undef CONFIG_UART0_SERIAL_CONSOLE
#    undef CONFIG_UART1_SERIAL_CONSOLE
#    undef CONFIG_UART2_SERIAL_CONSOLE
#    undef CONFIG_UART4_SERIAL_CONSOLE
#    undef CONFIG_UART5_SERIAL_CONSOLE
#    define HAVE_UART_CONSOLE 1
#  elif defined(CONFIG_UART4_SERIAL_CONSOLE) && defined(HAVE_UART4)
#    undef CONFIG_UART0_SERIAL_CONSOLE
#    undef CONFIG_UART1_SERIAL_CONSOLE
#    undef CONFIG_UART2_SERIAL_CONSOLE
#    undef CONFIG_UART3_SERIAL_CONSOLE
#    undef CONFIG_UART5_SERIAL_CONSOLE
#    define HAVE_UART_CONSOLE 1
#  elif defined(CONFIG_UART5_SERIAL_CONSOLE) && defined(HAVE_UART5)
#    undef CONFIG_UART0_SERIAL_CONSOLE
#    undef CONFIG_UART1_SERIAL_CONSOLE
#    undef CONFIG_UART2_SERIAL_CONSOLE
#    undef CONFIG_UART3_SERIAL_CONSOLE
#    undef CONFIG_UART4_SERIAL_CONSOLE
#    define HAVE_UART_CONSOLE 1
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
#  endif
#endif

/* Check UART flow control (Not yet supported) */

# undef CONFIG_UART0_FLOWCONTROL
# undef CONFIG_UART1_FLOWCONTROL
# undef CONFIG_UART2_FLOWCONTROL
# undef CONFIG_UART3_FLOWCONTROL
# undef CONFIG_UART4_FLOWCONTROL
# undef CONFIG_UART5_FLOWCONTROL

/* UART Default Interrupt Priorities */

#ifndef CONFIG_XMC4_UART0PRIO
#  define CONFIG_XMC4_UART0PRIO NVIC_SYSH_PRIORITY_DEFAULT
#endif
#ifndef CONFIG_XMC4_UART1PRIO
#  define CONFIG_XMC4_UART1PRIO NVIC_SYSH_PRIORITY_DEFAULT
#endif
#ifndef CONFIG_XMC4_UART2PRIO
#  define CONFIG_XMC4_UART2PRIO NVIC_SYSH_PRIORITY_DEFAULT
#endif
#ifndef CONFIG_XMC4_UART3PRIO
#  define CONFIG_XMC4_UART3PRIO NVIC_SYSH_PRIORITY_DEFAULT
#endif
#ifndef CONFIG_XMC4_UART4PRIO
#  define CONFIG_XMC4_UART4PRIO NVIC_SYSH_PRIORITY_DEFAULT
#endif
#ifndef CONFIG_XMC4_UART5PRIO
#  define CONFIG_XMC4_UART5PRIO NVIC_SYSH_PRIORITY_DEFAULT
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

#endif /* __ARCH_ARM_SRC_XMC4_XMC4_CONFIG_H */
