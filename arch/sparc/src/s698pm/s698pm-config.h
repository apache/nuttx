/****************************************************************************
 * arch/sparc/src/s698pm/s698pm-config.h
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

#ifndef __ARCH_SPARC_SRC_S698PM_CONFIG_H
#define __ARCH_SPARC_SRC_S698PM_CONFIG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <arch/board/board.h>

#include "chip.h"

#include "s698pm-memorymap.h"
#include "s698pm-uart.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* UARTs ********************************************************************/

/* Don't enable UARTs not supported by the chip. */

#if CHIP_NUARTS < 1
#  undef CONFIG_S698PM_UART1
#  undef CONFIG_S698PM_UART2
#  undef CONFIG_S698PM_UART3
#  undef CONFIG_S698PM_UART4
#elif CHIP_NUARTS < 2
#  undef CONFIG_S698PM_UART2
#  undef CONFIG_S698PM_UART3
#  undef CONFIG_S698PM_UART4
#elif CHIP_NUARTS < 3
#  undef CONFIG_S698PM_UART3
#  undef CONFIG_S698PM_UART4
#elif CHIP_NUARTS < 4
#  undef CONFIG_S698PM_UART4
#endif

/* Are any UARTs enabled? */

#undef HAVE_UART_DEVICE
#if defined(CONFIG_S698PM_UART1) || defined(CONFIG_S698PM_UART2) || \
    defined(CONFIG_S698PM_UART3) || defined(CONFIG_S698PM_UART4)
#  define HAVE_UART_DEVICE 1
#endif

/* Is there a serial console?  There should be no more than one defined.  It
 * could be on any UARTn, n=1,.. CHIP_NUARTS
 */

#if defined(CONFIG_UART1_SERIAL_CONSOLE) && defined(CONFIG_S698PM_UART1)
#  undef CONFIG_UART2_SERIAL_CONSOLE
#  undef CONFIG_UART3_SERIAL_CONSOLE
#  undef CONFIG_UART4_SERIAL_CONSOLE
#  define HAVE_SERIAL_CONSOLE 1
#elif defined(CONFIG_UART2_SERIAL_CONSOLE) && defined(CONFIG_S698PM_UART2)
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  undef CONFIG_UART3_SERIAL_CONSOLE
#  undef CONFIG_UART4_SERIAL_CONSOLE
#  define HAVE_SERIAL_CONSOLE 1
#elif defined(CONFIG_UART3_SERIAL_CONSOLE) && defined(CONFIG_S698PM_UART3)
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  undef CONFIG_UART2_SERIAL_CONSOLE
#  undef CONFIG_UART4_SERIAL_CONSOLE
#  define HAVE_SERIAL_CONSOLE 1
#elif defined(CONFIG_UART4_SERIAL_CONSOLE) && defined(CONFIG_S698PM_UART4)
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  undef CONFIG_UART2_SERIAL_CONSOLE
#  undef CONFIG_UART3_SERIAL_CONSOLE
#  define HAVE_SERIAL_CONSOLE 1
#else
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  undef CONFIG_UART2_SERIAL_CONSOLE
#  undef CONFIG_UART3_SERIAL_CONSOLE
#  undef CONFIG_UART4_SERIAL_CONSOLE
#  undef HAVE_SERIAL_CONSOLE
#endif

#endif /* __ARCH_SPARC_SRC_S698PM_CONFIG_H */
