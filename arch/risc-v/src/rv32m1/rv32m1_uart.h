/****************************************************************************
 * arch/risc-v/src/rv32m1/rv32m1_uart.h
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

#ifndef __ARCH_RISCV_SRC_RV32M1_RV32M1_UART_H
#define __ARCH_RISCV_SRC_RV32M1_RV32M1_UART_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/serial/serial.h>
#include "chip.h"

#include "hardware/rv32m1_lpuart.h"

#undef HAVE_UART
#if defined(CONFIG_RV32M1_LPUART0) || \
    defined(CONFIG_RV32M1_LPUART1) || \
    defined(CONFIG_RV32M1_LPUART2) || \
    defined(CONFIG_RV32M1_LPUART3)
#  define HAVE_UART 1
#endif

#if defined(CONFIG_LPUART0_SERIAL_CONSOLE) && defined(CONFIG_RV32M1_LPUART0)
#  undef CONFIG_LPUART1_SERIAL_CONSOLE
#  undef CONFIG_LPUART2_SERIAL_CONSOLE
#  undef CONFIG_LPUART3_SERIAL_CONSOLE
#  define HAVE_SERIAL_CONSOLE 1
#elif defined(CONFIG_LPUART1_SERIAL_CONSOLE) && defined(CONFIG_RV32M1_LPUART1)
#  undef CONFIG_LPUART0_SERIAL_CONSOLE
#  undef CONFIG_LPUART2_SERIAL_CONSOLE
#  undef CONFIG_LPUART3_SERIAL_CONSOLE
#  define HAVE_SERIAL_CONSOLE 1
#elif defined(CONFIG_LPUART2_SERIAL_CONSOLE) && defined(CONFIG_RV32M1_LPUART2)
#  undef CONFIG_LPUART0_SERIAL_CONSOLE
#  undef CONFIG_LPUART1_SERIAL_CONSOLE
#  undef CONFIG_LPUART3_SERIAL_CONSOLE
#  define HAVE_SERIAL_CONSOLE 1
#elif defined(CONFIG_LPUART3_SERIAL_CONSOLE) && defined(CONFIG_RV32M1_LPUART3)
#  undef CONFIG_LPUART0_SERIAL_CONSOLE
#  undef CONFIG_LPUART1_SERIAL_CONSOLE
#  undef CONFIG_LPUART2_SERIAL_CONSOLE
#  define HAVE_SERIAL_CONSOLE 1
#else
#  undef CONFIG_LPUART0_SERIAL_CONSOLE
#  undef CONFIG_LPUART1_SERIAL_CONSOLE
#  undef CONFIG_LPUART2_SERIAL_CONSOLE
#  undef CONFIG_LPUART3_SERIAL_CONSOLE
#  undef HAVE_SERIAL_CONSOLE
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#if defined(HAVE_SERIAL_CONSOLE)
/****************************************************************************
 * Name: rv32m1_console_uart_setup
 ****************************************************************************/

EXTERN void rv32m1_console_uart_setup(void);

/****************************************************************************
 * Name: rv32m1_console_uart_putc
 ****************************************************************************/

EXTERN void rv32m1_console_uart_putc(char);
#endif /* HAVE_SERIAL_CONSOLE */

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_RISCV_SRC_RV32M1_RV32M1_UART_H */
