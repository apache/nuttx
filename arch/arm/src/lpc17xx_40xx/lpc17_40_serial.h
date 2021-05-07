/****************************************************************************
 * arch/arm/src/lpc17xx_40xx/lpc17_40_serial.h
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

#ifndef __ARCH_ARM_SRC_LPC17XX_40XX_LPC17_40_SERIAL_H
#define __ARCH_ARM_SRC_LPC17XX_40XX_LPC17_40_SERIAL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <arch/board/board.h>

#include "hardware/lpc17_40_uart.h"
#include "hardware/lpc17_40_syscon.h"

#include "lpc17_40_gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Are any UARTs enabled? */

#undef HAVE_UART
#if defined(CONFIG_LPC17_40_UART0) || defined(CONFIG_LPC17_40_UART1) || \
    defined(CONFIG_LPC17_40_UART2) || defined(CONFIG_LPC17_40_UART3)
#  define HAVE_UART 1
#endif

/* Is there a serial console? There should be at most one defined.
 * It could be on any UARTn, n=0,1,2,3
 */

#if defined(CONFIG_UART0_SERIAL_CONSOLE) && defined(CONFIG_LPC17_40_UART0)
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  undef CONFIG_UART2_SERIAL_CONSOLE
#  undef CONFIG_UART3_SERIAL_CONSOLE
#  define HAVE_CONSOLE 1
#elif defined(CONFIG_UART1_SERIAL_CONSOLE) && defined(CONFIG_LPC17_40_UART1)
#  undef CONFIG_UART0_SERIAL_CONSOLE
#  undef CONFIG_UART2_SERIAL_CONSOLE
#  undef CONFIG_UART3_SERIAL_CONSOLE
#  define HAVE_CONSOLE 1
#elif defined(CONFIG_UART2_SERIAL_CONSOLE) && defined(CONFIG_LPC17_40_UART2)
#  undef CONFIG_UART0_SERIAL_CONSOLE
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  undef CONFIG_UART3_SERIAL_CONSOLE
#  define HAVE_CONSOLE 1
#elif defined(CONFIG_UART3_SERIAL_CONSOLE) && defined(CONFIG_LPC17_40_UART3)
#  undef CONFIG_UART0_SERIAL_CONSOLE
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  undef CONFIG_UART2_SERIAL_CONSOLE
#  define HAVE_CONSOLE 1
#else
#  undef CONFIG_UART0_SERIAL_CONSOLE
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  undef CONFIG_UART2_SERIAL_CONSOLE
#  undef CONFIG_UART3_SERIAL_CONSOLE
#  undef HAVE_CONSOLE
#endif

/* Check UART flow control (Only supported by UART1) */

# undef CONFIG_UART0_IFLOWCONTROL
# undef CONFIG_UART0_OFLOWCONTROL
# undef CONFIG_UART2_IFLOWCONTROL
# undef CONFIG_UART2_OFLOWCONTROL
# undef CONFIG_UART3_IFLOWCONTROL
# undef CONFIG_UART3_OFLOWCONTROL
#ifndef CONFIG_LPC17_40_UART1
# undef CONFIG_UART1_IFLOWCONTROL
# undef CONFIG_UART1_OFLOWCONTROL
#endif

/* We cannot allow the DLM/DLL divisor to become to small or will will lose
 * too much accuracy.  This following is a "fudge factor" that represents the
 * minimum value of the divisor that we will permit.
 */

#define UART_MINDL 32

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

void up_setbaud(uintptr_t uartbase, uint32_t basefreq, uint32_t baud);

#endif /* __ARCH_ARM_SRC_LPC17XX_40XX_LPC17_40_SERIAL_H */
