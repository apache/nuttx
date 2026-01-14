/****************************************************************************
 * arch/avr/src/avrdx/avrdx_config.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifndef __ARCH_AVR_SRC_AVRDX_AVRDX_CONFIG_H
#define __ARCH_AVR_SRC_AVRDX_AVRDX_CONFIG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* USARTs *******************************************************************/

/* Is there a serial console? There should be at most one defined (Kconfig
 * takes care of that) and it can be on any enabled USARTn
 */

#if defined(CONFIG_USART0_SERIAL_CONSOLE)
#  define AVRDX_SERIAL_CONSOLE_USART_N 0
#elif defined(CONFIG_USART1_SERIAL_CONSOLE)
#  define AVRDX_SERIAL_CONSOLE_USART_N 1
#elif defined(CONFIG_USART2_SERIAL_CONSOLE)
#  define AVRDX_SERIAL_CONSOLE_USART_N 2
#elif defined(CONFIG_USART3_SERIAL_CONSOLE)
#  define AVRDX_SERIAL_CONSOLE_USART_N 3
#elif defined(CONFIG_USART4_SERIAL_CONSOLE)
#  define AVRDX_SERIAL_CONSOLE_USART_N 4
#elif defined(CONFIG_USART5_SERIAL_CONSOLE)
#  define AVRDX_SERIAL_CONSOLE_USART_N 5
#else
  /* For a good measure, should not be set anywhere else */
#  undef AVRDX_SERIAL_CONSOLE_USART_N
#endif

/* If some other device is used as the console, then the serial driver may
 * still be needed.  Let's assume that if the upper half serial driver is
 * built, then the lower half will also be needed.  There is no need for
 * the early serial initialization in this case.
 */

#if defined(CONFIG_MCU_SERIAL) || defined(CONFIG_STANDARD_SERIAL)
#  define USE_SERIALDRIVER 1
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes Prototypes
 ****************************************************************************/

#endif /* __ARCH_AVR_SRC_AVRDX_AVRDX_CONFIG_H */
