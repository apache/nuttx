/****************************************************************************
 * arch/arm/src/lpc43xx/lpc43_config.h
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

#ifndef __ARCH_ARM_SRC_LPC43XX_LPC43_CONFIG_H
#define __ARCH_ARM_SRC_LPC43XX_LPC43_CONFIG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Are any UARTs enabled? */

#undef HAVE_UART
#if defined(CONFIG_LPC43_USART0) || defined(CONFIG_LPC43_UART1) || \
    defined(CONFIG_LPC43_USART2) || defined(CONFIG_LPC43_USART3)
#  define HAVE_UART 1
#endif

/* Make sure all features are disabled for disabled U[S]ARTs.
 * This simplifies checking later.
 */

#ifndef CONFIG_LPC43_USART0
#  undef CONFIG_USART0_SERIAL_CONSOLE
#  undef CONFIG_USART0_RS485MODE
#endif

#ifndef CONFIG_LPC43_UART1
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  undef CONFIG_UART1_RS485MODE
#  undef CONFIG_UART1_RS485_DTRDIR
#endif

#ifndef CONFIG_LPC43_USART2
#  undef CONFIG_USART2_SERIAL_CONSOLE
#  undef CONFIG_USART2_RS485MODE
#endif

#ifndef CONFIG_LPC43_USART3
#  undef CONFIG_USART3_SERIAL_CONSOLE
#  undef CONFIG_USART3_RS485MODE
#endif

/* Is there a serial console? There should be at most one defined.
 * It could be on any UARTn, n=0,1,2,3 - OR - there might not be any serial
 * console at all.
 */

#if defined(CONFIG_USART0_SERIAL_CONSOLE)
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  undef CONFIG_USART2_SERIAL_CONSOLE
#  undef CONFIG_USART3_SERIAL_CONSOLE
#  define HAVE_SERIAL_CONSOLE 1
#elif defined(CONFIG_UART1_SERIAL_CONSOLE)
#  undef CONFIG_USART0_SERIAL_CONSOLE
#  undef CONFIG_USART2_SERIAL_CONSOLE
#  undef CONFIG_USART3_SERIAL_CONSOLE
#  define HAVE_SERIAL_CONSOLE 1
#elif defined(CONFIG_USART2_SERIAL_CONSOLE)
#  undef CONFIG_USART0_SERIAL_CONSOLE
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  undef CONFIG_USART3_SERIAL_CONSOLE
#  define HAVE_SERIAL_CONSOLE 1
#elif defined(CONFIG_USART3_SERIAL_CONSOLE)
#  undef CONFIG_USART0_SERIAL_CONSOLE
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  undef CONFIG_USART2_SERIAL_CONSOLE
#  define HAVE_SERIAL_CONSOLE 1
#else
#  undef CONFIG_USART0_SERIAL_CONSOLE
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  undef CONFIG_USART2_SERIAL_CONSOLE
#  undef CONFIG_USART3_SERIAL_CONSOLE
#  undef HAVE_SERIAL_CONSOLE
#endif

/* Check UART flow control (Only supported by UART1) */

# undef CONFIG_USART0_FLOWCONTROL
# undef CONFIG_USART2_FLOWCONTROL
# undef CONFIG_USART3_FLOWCONTROL
#ifndef CONFIG_LPC43_UART1
# undef CONFIG_UART1_FLOWCONTROL
#endif

/* Check for RS-485 support (All USARTS & UART1) */

#undef HAVE_RS485
#if defined(CONFIG_USART0_RS485MODE) || defined(CONFIG_UART1_RS485MODE) || \
    defined(CONFIG_USART2_RS485MODE) || defined(CONFIG_USART3_RS485MODE)
#  define HAVE_RS485 1
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_LPC43XX_LPC43_CONFIG_H */
