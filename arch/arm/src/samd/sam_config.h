/************************************************************************************
 * arch/arm/src/samd/sam_config.h
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_SAMD_SAM_CONFIG_H
#define __ARCH_ARM_SRC_SAMD_SAM_CONFIG_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <arch/samd/chip.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* How many SERCOM peripherals are configured as peripherals */

#define SAMD_HAVE_UART0 1
#define SAMD_HAVE_UART1 1
#define SAMD_HAVE_UART2 1
#define SAMD_HAVE_UART3 1
#define SAMD_HAVE_UART4 1
#define SAMD_HAVE_UART5 1

#if !defined(CONFIG_SAMD_SERCOM0) || !defined(CONFIG_SAMD_SERCOM0_UART)
#  undef SAMD_HAVE_UART0
#  undef CONFIG_SAMD_SERCOM0_UART
#  undef CONFIG_UART0_SERIAL_CONSOLE
#  undef CONFIG_UART0_FLOW_CONTROL
#  undef CONFIG_UART0_IRDAMODE
#  undef CONFIG_UART0_RS485MODE
#endif

#if !defined(CONFIG_SAMD_SERCOM1) || !defined(CONFIG_SAMD_SERCOM1_UART)
#  undef SAMD_HAVE_UART1
#  undef CONFIG_SAMD_SERCOM1_UART
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  undef CONFIG_UART1_FLOW_CONTROL
#  undef CONFIG_UART1_IRDAMODE
#  undef CONFIG_UART1_RS485MODE
#endif

#if !defined(CONFIG_SAMD_SERCOM2) || !defined(CONFIG_SAMD_SERCOM2_UART)
#  undef SAMD_HAVE_UART2
#  undef CONFIG_SAMD_SERCOM2_UART
#  undef CONFIG_UART2_SERIAL_CONSOLE
#  undef CONFIG_UART2_FLOW_CONTROL
#  undef CONFIG_UART2_IRDAMODE
#  undef CONFIG_UART2_RS485MODE
#endif

#if !defined(CONFIG_SAMD_SERCOM3) || !defined(CONFIG_SAMD_SERCOM3_UART)
#  undef SAMD_HAVE_UART3
#  undef CONFIG_SAMD_SERCOM3_UART
#  undef CONFIG_UART3_SERIAL_CONSOLE
#  undef CONFIG_UART3_FLOW_CONTROL
#  undef CONFIG_UART3_IRDAMODE
#  undef CONFIG_UART3_RS485MODE
#endif

#if !defined(CONFIG_SAMD_SERCOM4) || !defined(CONFIG_SAMD_SERCOM4_UART)
#  undef SAMD_HAVE_UART4
#  undef CONFIG_SAMD_SERCOM4_UART
#  undef CONFIG_UART4_SERIAL_CONSOLE
#  undef CONFIG_UART4_FLOW_CONTROL
#  undef CONFIG_UART4_IRDAMODE
#  undef CONFIG_UART4_RS485MODE
#endif

#if !defined(CONFIG_SAMD_SERCOM5) || !defined(CONFIG_SAMD_SERCOM5_UART)
#  undef SAMD_HAVE_UART5
#  undef CONFIG_SAMD_SERCOM5_UART
#  undef CONFIG_UART5_SERIAL_CONSOLE
#  undef CONFIG_UART5_FLOW_CONTROL
#  undef CONFIG_UART5_IRDAMODE
#  undef CONFIG_UART5_RS485MODE
#endif

/* Are any UARTs enabled? */

#undef HAVE_UART
#if defined(SAMD_HAVE_UART0) || defined(SAMD_HAVE_UART1) || \
    defined(SAMD_HAVE_UART2) || defined(SAMD_HAVE_UART3) || \
    defined(SAMD_HAVE_UART4) || defined(SAMD_HAVE_UART5)
#  define HAVE_UART 1
#endif

/* Is there a serial console? There should be at most one defined.  It could be on
 * any UARTn, n=0,1,2 - OR - there might not be any serial console at all.
 */

#if defined(CONFIG_UART0_SERIAL_CONSOLE)
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  undef CONFIG_UART2_SERIAL_CONSOLE
#  undef CONFIG_UART3_SERIAL_CONSOLE
#  undef CONFIG_UART4_SERIAL_CONSOLE
#  undef CONFIG_UART5_SERIAL_CONSOLE
#  define HAVE_SERIAL_CONSOLE 1
#elif defined(CONFIG_UART1_SERIAL_CONSOLE)
#  undef CONFIG_UART0_SERIAL_CONSOLE
#  undef CONFIG_UART2_SERIAL_CONSOLE
#  undef CONFIG_UART3_SERIAL_CONSOLE
#  undef CONFIG_UART4_SERIAL_CONSOLE
#  undef CONFIG_UART5_SERIAL_CONSOLE
#  define HAVE_SERIAL_CONSOLE 1
#elif defined(CONFIG_UART2_SERIAL_CONSOLE)
#  undef CONFIG_UART0_SERIAL_CONSOLE
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  undef CONFIG_UART3_SERIAL_CONSOLE
#  undef CONFIG_UART4_SERIAL_CONSOLE
#  undef CONFIG_UART5_SERIAL_CONSOLE
#  define HAVE_SERIAL_CONSOLE 1
#elif defined(CONFIG_UART3_SERIAL_CONSOLE)
#  undef CONFIG_UART0_SERIAL_CONSOLE
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  undef CONFIG_UART2_SERIAL_CONSOLE
#  undef CONFIG_UART4_SERIAL_CONSOLE
#  undef CONFIG_UART5_SERIAL_CONSOLE
#  define HAVE_SERIAL_CONSOLE 1
#elif defined(CONFIG_UART4_SERIAL_CONSOLE)
#  undef CONFIG_UART0_SERIAL_CONSOLE
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  undef CONFIG_UART2_SERIAL_CONSOLE
#  undef CONFIG_UART3_SERIAL_CONSOLE
#  undef CONFIG_UART5_SERIAL_CONSOLE
#  define HAVE_SERIAL_CONSOLE 1
#elif defined(CONFIG_UART5_SERIAL_CONSOLE)
#  undef CONFIG_UART0_SERIAL_CONSOLE
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  undef CONFIG_UART2_SERIAL_CONSOLE
#  undef CONFIG_UART3_SERIAL_CONSOLE
#  undef CONFIG_UART4_SERIAL_CONSOLE
#  define HAVE_SERIAL_CONSOLE 1
#else
#  undef CONFIG_UART0_SERIAL_CONSOLE
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  undef CONFIG_UART2_SERIAL_CONSOLE
#  undef CONFIG_UART3_SERIAL_CONSOLE
#  undef CONFIG_UART4_SERIAL_CONSOLE
#  undef CONFIG_UART5_SERIAL_CONSOLE
#  undef HAVE_SERIAL_CONSOLE
#endif

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_SAMD_SAM_CONFIG_H */
