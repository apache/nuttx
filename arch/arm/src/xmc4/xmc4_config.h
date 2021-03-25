/****************************************************************************
 * arch/arm/src/xmc4/xmc4_config.h
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

#ifndef __ARCH_ARM_SRC_XMC4_XMC4_CONFIG_H
#define __ARCH_ARM_SRC_XMC4_XMC4_CONFIG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <arch/board/board.h>

#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

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

/* Is there a serial console? There should be at most one defined.
 * It could be on any UARTn, n=0,1,2,3,4,5
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

#endif /* __ARCH_ARM_SRC_XMC4_XMC4_CONFIG_H */
