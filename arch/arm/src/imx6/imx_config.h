/****************************************************************************
 * arch/arm/src/imx6/imx_config.h
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

#ifndef __ARCH_ARM_SRC_IMX6_IMX_CONFIG_H
#define __ARCH_ARM_SRC_IMX6_IMX_CONFIG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

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
