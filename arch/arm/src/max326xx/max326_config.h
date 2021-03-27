/****************************************************************************
 * arch/arm/src/max326xx/max326_config.h
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

#ifndef __ARCH_ARM_SRC_MAX326XX_MAX326_CONFIG_H
#define __ARCH_ARM_SRC_MAX326XX_MAX326_CONFIG_H

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

/* Check if we have a UART device */

#if MAX326_NUART < 4
#  undef CONFIG_MAX326XX_UART3
#if MAX326_NUART < 3
#  undef CONFIG_MAX326XX_UART2
#if MAX326_NUART < 2
#  undef CONFIG_MAX326XX_UART1
#if MAX326_NUART < 1
#  undef CONFIG_MAX326XX_UART0
#endif
#endif
#endif
#endif

#undef HAVE_UART_DEVICE

#if defined(CONFIG_MAX326XX_UART0) || defined(CONFIG_MAX326XX_UART2) || \
    defined(CONFIG_MAX326XX_UART1) || defined(CONFIG_MAX326XX_UART3)
#  define HAVE_UART_DEVICE 1
#endif

/* Is there a serial console? There should be at most one defined.
 * It could be on any UARTn, n=0,1,2,3
 */

#undef HAVE_UART_CONSOLE

#if defined(CONFIG_UART0_SERIAL_CONSOLE)
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  undef CONFIG_UART2_SERIAL_CONSOLE
#  undef CONFIG_UART3_SERIAL_CONSOLE
#  define HAVE_UART_CONSOLE 1
#elif defined(CONFIG_UART1_SERIAL_CONSOLE)
#  undef CONFIG_UART0_SERIAL_CONSOLE
#  undef CONFIG_UART2_SERIAL_CONSOLE
#  undef CONFIG_UART3_SERIAL_CONSOLE
#  define HAVE_UART_CONSOLE 1
#elif defined(CONFIG_UART2_SERIAL_CONSOLE)
#  undef CONFIG_UART0_SERIAL_CONSOLE
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  undef CONFIG_UART3_SERIAL_CONSOLE
#  define HAVE_UART_CONSOLE 1
#elif defined(CONFIG_UART3_SERIAL_CONSOLE)
#  undef CONFIG_UART0_SERIAL_CONSOLE
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  undef CONFIG_UART2_SERIAL_CONSOLE
#  define HAVE_UART_CONSOLE 1
#else
#  ifdef CONFIG_DEV_CONSOLE
#    warning "No valid CONFIG_[LP]UART[n]_SERIAL_CONSOLE Setting"
#  endif
#  undef CONFIG_UART0_SERIAL_CONSOLE
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  undef CONFIG_UART2_SERIAL_CONSOLE
#  undef CONFIG_UART3_SERIAL_CONSOLE
#endif

/* Check UART flow control (Not yet supported) */

# undef CONFIG_UART0_FLOWCONTROL
# undef CONFIG_UART1_FLOWCONTROL
# undef CONFIG_UART2_FLOWCONTROL
# undef CONFIG_UART3_FLOWCONTROL
# undef CONFIG_UART4_FLOWCONTROL
# undef CONFIG_UART5_FLOWCONTROL
# undef CONFIG_UART6_FLOWCONTROL
# undef CONFIG_UART7_FLOWCONTROL
# undef CONFIG_UART8_FLOWCONTROL
# undef CONFIG_UART9_FLOWCONTROL

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_MAX326XX_MAX326_CONFIG_H */
