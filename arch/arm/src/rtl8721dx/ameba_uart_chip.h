/****************************************************************************
 * arch/arm/src/rtl8721dx/ameba_uart_chip.h
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

#ifndef __ARCH_ARM_SRC_RTL8721DX_AMEBA_UART_CHIP_H
#define __ARCH_ARM_SRC_RTL8721DX_AMEBA_UART_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <nuttx/irq.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Per-chip UART wiring for RTL8721DX (amebadplus).  The shared driver
 * (arch/arm/src/common/ameba/ameba_uart.c) includes this header to learn how
 * many general-purpose UARTs the chip exposes and, for each, its register
 * base, peripheral-clock masks, NVIC vector and crossbar pad-mux codes.
 *
 * Contract for the other Ameba chips (amebalite / amebasmart / amebagreen2 /
 * RTL8720F): supply a same-named header on the chip include path with the
 * macros below.  The instance count, bases, IRQs and pin-mux codes differ
 * per chip.  Two points are worth noting for a future port:
 *
 *   1. The "function" and "clock" APBPeriph masks are two separate lists
 *      because on some chips (e.g. amebasmart) they are not the same value;
 *      RCC_PeriphClockCmd() takes them as distinct arguments.  On this chip
 *      the two lists are identical.
 *
 *   2. Chips whose pad mux uses a single generic UART function code instead
 *      of per-direction TXD/RXD codes (e.g. amebasmart) simply set both the
 *      TX and RX code lists to that same value; the driver programs each pad
 *      from the list and needs no change.
 */

#define AMEBA_NUART               2

#define AMEBA_UART_PORT_BASES     { 0x4100c000ul, 0x4100d000ul }

#define AMEBA_UART_PORT_IRQS      { RTL8721DX_IRQ_UART0, RTL8721DX_IRQ_UART1 }

/* APBPeriph_UARTx (function) and APBPeriph_UARTx_CLOCK masks.  Equal on this
 * chip; kept as two lists so chips where they differ can supply both.
 */

#define AMEBA_UART_APBPERIPH      \
        { (((uint32_t)1 << 30) | ((uint32_t)1 << 6)), \
          (((uint32_t)1 << 30) | ((uint32_t)1 << 7)) }

#define AMEBA_UART_APBPERIPH_CLK  \
        { (((uint32_t)1 << 30) | ((uint32_t)1 << 6)), \
          (((uint32_t)1 << 30) | ((uint32_t)1 << 7)) }

/* Direction-specific crossbar pad-mux function codes (PINMUX_FUNCTION_*). */

#define AMEBA_UART_TXFID          { 19, 23 }  /* UART0_TXD, UART1_TXD */
#define AMEBA_UART_RXFID          { 20, 24 }  /* UART0_RXD, UART1_RXD */

#endif /* __ARCH_ARM_SRC_RTL8721DX_AMEBA_UART_CHIP_H */
