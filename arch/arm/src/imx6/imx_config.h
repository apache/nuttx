/****************************************************************************
 * arch/arm/src/imx6/imx_config.h
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_IMX6_IMX_CONFIG_H
#define __ARCH_ARM_SRC_IMX6_IMX_CONFIG_H 1

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration **********************************************************/

/* Is there a UART enabled? */

#if defined(CONFIG_IMX6_UART1) || defined(CONFIG_IMX6_UART2) || \
    defined(CONFIG_IMX6_UART3) || defined(CONFIG_IMX6_UART4) || \
    defined(CONFIG_IMX6_UART5)
#  define IMX_HAVE_UART
#endif

#undef SUPPRESS_CONSOLE_CONFIG
#ifdef CONFIG_SUPPRESS_UART_CONFIG
#  define SUPPRESS_CONSOLE_CONFIG 1
#endif

/* Is there a serial console?  It could be on UART1-5 */

#if defined(CONFIG_UART1_SERIAL_CONSOLE) && defined(CONFIG_IMX6_UART1)
#  undef  CONFIG_UART2_SERIAL_CONSOLE
#  undef  CONFIG_UART3_SERIAL_CONSOLE
#  undef  CONFIG_UART4_SERIAL_CONSOLE
#  undef  CONFIG_UART5_SERIAL_CONSOLE
#  define IMX_HAVE_UART_CONSOLE 1
#elif defined(CONFIG_UART2_SERIAL_CONSOLE) && defined(CONFIG_IMX6_UART2)
#  undef  CONFIG_UART1_SERIAL_CONSOLE
#  undef  CONFIG_UART3_SERIAL_CONSOLE
#  undef  CONFIG_UART4_SERIAL_CONSOLE
#  undef  CONFIG_UART5_SERIAL_CONSOLE
#  define IMX_HAVE_UART_CONSOLE 1
#elif defined(CONFIG_UART3_SERIAL_CONSOLE) && defined(CONFIG_IMX6_UART3)
#  undef  CONFIG_UART1_SERIAL_CONSOLE
#  undef  CONFIG_UART2_SERIAL_CONSOLE
#  undef  CONFIG_UART4_SERIAL_CONSOLE
#  undef  CONFIG_UART5_SERIAL_CONSOLE
#  define IMX_HAVE_UART_CONSOLE 1
#elif defined(CONFIG_UART4_SERIAL_CONSOLE) && defined(CONFIG_IMX6_UART4)
#  undef  CONFIG_UART1_SERIAL_CONSOLE
#  undef  CONFIG_UART2_SERIAL_CONSOLE
#  undef  CONFIG_UART3_SERIAL_CONSOLE
#  undef  CONFIG_UART5_SERIAL_CONSOLE
#  define IMX_HAVE_UART_CONSOLE 1
#elif defined(CONFIG_UART5_SERIAL_CONSOLE) && defined(CONFIG_IMX6_UART5)
#  undef  CONFIG_UART1_SERIAL_CONSOLE
#  undef  CONFIG_UART2_SERIAL_CONSOLE
#  undef  CONFIG_UART3_SERIAL_CONSOLE
#  undef  CONFIG_UART4_SERIAL_CONSOLE
#  define IMX_HAVE_UART_CONSOLE 1
#else
#  warning "No valid CONFIG_UARTn/USARTn_SERIAL_CONSOLE Setting"
#  undef  CONFIG_UART1_SERIAL_CONSOLE
#  undef  CONFIG_UART2_SERIAL_CONSOLE
#  undef  CONFIG_UART3_SERIAL_CONSOLE
#  undef  CONFIG_UART4_SERIAL_CONSOLE
#  undef  CONFIG_UART5_SERIAL_CONSOLE
#  undef  IMX_HAVE_UART_CONSOLE
#endif

#endif /* __ARCH_ARM_SRC_IMX6_IMX_CONFIG_H */
