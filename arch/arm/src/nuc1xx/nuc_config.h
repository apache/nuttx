/************************************************************************************
 * arch/arm/src/nuc1xx/nuc_config.h
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_NUC1XX_NUC_CONFIG_H
#define __ARCH_ARM_SRC_NUC1XX_NUC_CONFIG_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <arch/nuc1xx/chip.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* Limit the number of enabled UARTs to those supported by the chip architecture */

#if NUC_NUARTS < 3
#  undef CONFIG_NUC_UART2
#endif

#if NUC_NUARTS < 2
#  undef CONFIG_NUC_UART1
#endif

#if NUC_NUARTS < 1
#  undef CONFIG_NUC_UART0
#endif

/* Are any UARTs enabled? */

#undef HAVE_UART
#if defined(CONFIG_NUC_UART0) || defined(CONFIG_NUC_UART1) || defined(CONFIG_NUC_UART2)
#  define HAVE_UART 1
#endif

/* Make sure all features are disabled for disabled U[S]ARTs.  This simplifies
 * checking later.
 */

#ifndef CONFIG_NUC_UART0
#  undef CONFIG_UART0_SERIAL_CONSOLE
#  undef CONFIG_UART0_FLOWCONTROL
#  undef CONFIG_UART0_IRDAMODE
#  undef CONFIG_UART0_RS485MODE
#endif

#ifndef CONFIG_NUC_UART1
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  undef CONFIG_UART1_FLOWCONTROL
#  undef CONFIG_UART1_IRDAMODE
#  undef CONFIG_UART1_RS485MODE
#endif

#undef CONFIG_UART2_FLOWCONTROL  /* UART2 does not support flow control */
#ifndef CONFIG_NUC_UART2
#  undef CONFIG_UART2_SERIAL_CONSOLE
#  undef CONFIG_UART2_IRDAMODE
#  undef CONFIG_UART2_RS485MODE
#endif

/* Is there a serial console? There should be at most one defined.  It could be on
 * any UARTn, n=0,1,2 - OR - there might not be any serial console at all.
 */

#if defined(CONFIG_UART0_SERIAL_CONSOLE)
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  undef CONFIG_UART2_SERIAL_CONSOLE
#  define HAVE_SERIAL_CONSOLE 1
#elif defined(CONFIG_UART1_SERIAL_CONSOLE)
#  undef CONFIG_UART0_SERIAL_CONSOLE
#  undef CONFIG_UART2_SERIAL_CONSOLE
#  define HAVE_SERIAL_CONSOLE 1
#elif defined(CONFIG_UART2_SERIAL_CONSOLE)
#  undef CONFIG_UART0_SERIAL_CONSOLE
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  define HAVE_SERIAL_CONSOLE 1
#else
#  undef CONFIG_UART0_SERIAL_CONSOLE
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  undef CONFIG_UART2_SERIAL_CONSOLE
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

#endif /* __ARCH_ARM_SRC_NUC1XX_NUC_CONFIG_H */
