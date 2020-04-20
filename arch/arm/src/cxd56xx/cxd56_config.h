/****************************************************************************
 * arch/arm/src/cxd56xx/cxd56_config.h
 *
 *   Copyright 2018 Sony Semiconductor Solutions Corporation
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
 * 3. Neither the name of Sony Semiconductor Solutions Corporation nor
 *    the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
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

#ifndef __ARCH_ARM_SRC_CXD56XX_CXD56_CONFIG_H
#define __ARCH_ARM_SRC_CXD56XX_CXD56_CONFIG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Required configuration settings */

/* Are any UARTs enabled? */

#undef HAVE_UART
#if defined(CONFIG_CXD56_UART0) || defined(CONFIG_CXD56_UART1) || \
    defined(CONFIG_CXD56_UART2)
#  define HAVE_UART 1
#endif

/* Make sure all features are disabled for disabled U[S]ARTs.
 * This simplifies checking later.
 */

#ifndef CONFIG_CXD56_UART0
#  undef CONFIG_UART0_SERIAL_CONSOLE
#  undef CONFIG_UART0_RS485MODE
#  undef CONFIG_UART0_RS485_DTRDIR
#endif

#ifndef CONFIG_CXD56_UART1
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  undef CONFIG_UART1_RS485MODE
#  undef CONFIG_UART1_RS485_DTRDIR
#endif

#ifndef CONFIG_CXD56_UART2
#  undef CONFIG_UART2_SERIAL_CONSOLE
#  undef CONFIG_UART2_RS485MODE
#  undef CONFIG_UART2_RS485_DTRDIR
#endif

/* Is there a serial console? There should be at most one defined.  It could
 * be on any UARTn, n=0,1,2,3 - OR - there might not be any serial console at
 * all.
 */

#if defined(CONFIG_UART0_SERIAL_CONSOLE)
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  undef CONFIG_UART2_SERIAL_CONSOLE
#  undef CONFIG_UART3_SERIAL_CONSOLE
#  define HAVE_CONSOLE 1
#elif defined(CONFIG_UART1_SERIAL_CONSOLE)
#  undef CONFIG_UART0_SERIAL_CONSOLE
#  undef CONFIG_UART2_SERIAL_CONSOLE
#  undef CONFIG_UART3_SERIAL_CONSOLE
#  define HAVE_CONSOLE 1
#elif defined(CONFIG_UART2_SERIAL_CONSOLE)
#  undef CONFIG_UART0_SERIAL_CONSOLE
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  undef CONFIG_UART3_SERIAL_CONSOLE
#  define HAVE_CONSOLE 1
#else
#  undef CONFIG_UART0_SERIAL_CONSOLE
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  undef CONFIG_UART2_SERIAL_CONSOLE
#  undef CONFIG_UART3_SERIAL_CONSOLE
#  undef HAVE_CONSOLE
#endif

/* Check UART flow control (Only supported by UART1) */

# undef CONFIG_UART0_FLOWCONTROL
# undef CONFIG_UART2_FLOWCONTROL
# undef CONFIG_UART3_FLOWCONTROL
#ifndef CONFIG_CXD56_UART1
# undef CONFIG_UART1_FLOWCONTROL
#endif

/* Get Firmware version */

#define GET_SBL_VERSION()           (BKUP->sbl_version)
#define GET_SYSFW_VERSION()         (BKUP->sysfw_version)
#define GET_SYSFW_VERSION_MAJOR()   ((GET_SYSFW_VERSION() >> 28) & 0xf)
#define GET_SYSFW_VERSION_MINOR()   ((GET_SYSFW_VERSION() >> 20) & 0xff)
#define GET_SYSFW_VERSION_BUILD()   (GET_SYSFW_VERSION() & 0xfffff)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_CXD56XX_CXD56_CONFIG_H */
