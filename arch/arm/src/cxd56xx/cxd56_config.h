/****************************************************************************
 * arch/arm/src/cxd56xx/cxd56_config.h
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
