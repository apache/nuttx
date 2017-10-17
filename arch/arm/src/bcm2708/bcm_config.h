/****************************************************************************
 * arch/arm/src/bcm2708/bcm_config.h
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
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_BCM2708_BCM_CONFIG_H
#define __ARCH_ARM_SRC_BCM2708_BCM_CONFIG_H 1

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Is there a UART enabled?  The BCM2835 device has two UARTS. On mini UART
 * and and PL011 UART.
 */

#if defined(CONFIG_BCM2708_MINI_UART) || defined(CONFIG_BCM2708_PL011_UART)
#  define BCM_HAVE_UART
#endif

#undef SUPPRESS_CONSOLE_CONFIG
#ifdef CONFIG_SUPPRESS_UART_CONFIG
#  define SUPPRESS_CONSOLE_CONFIG 1
#endif

/* Is there a serial console?  It could be on UART1-5 */

#if defined(CONFIG_BCM2708_MINI_UART_SERIAL_CONSOLE) && defined(CONFIG_BCM2708_MINI_UART)
#  undef  CONFIG_BCM2708_PL011_UART_SERIAL_CONSOLE
#  define BCM_HAVE_UART_CONSOLE 1
#elif defined(CONFIG_BCM2708_PL011_UART_SERIAL_CONSOLE) && defined(CONFIG_BCM2708_PL011_UART)
#  undef  CONFIG_BCM2708_MINI_UART_SERIAL_CONSOLE
#  define BCM_HAVE_UART_CONSOLE 1
#else
#  warning "No valid serial console Setting"
#  undef  CONFIG_BCM2708_MINI_UART_SERIAL_CONSOLE
#  undef  CONFIG_BCM2708_PL011_UART_SERIAL_CONSOLE
#  undef  BCM_HAVE_UART_CONSOLE
#endif

#endif /* __ARCH_ARM_SRC_BCM2708_BCM_CONFIG_H */
