/****************************************************************************
 * arch/arm/src/rtl8720f/ameba_uart_chip.h
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

#ifndef __ARCH_ARM_SRC_RTL8720F_AMEBA_UART_CHIP_H
#define __ARCH_ARM_SRC_RTL8720F_AMEBA_UART_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <nuttx/irq.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Per-chip UART wiring for RTL8720F.  The shared driver
 * (arch/arm/src/common/ameba/ameba_uart.c) includes this header to learn how
 * many general-purpose UARTs the chip exposes and, for each, its register
 * base, peripheral-clock masks, NVIC vector and crossbar pad-mux codes.  See
 * the RTL8721DX version of this header for the full contract; the notes
 * below cover only what differs on this chip.
 *
 * The AP core is km4tz (TrustZone secure) here, but the fwlib UART_DEV_TABLE
 * (fwlib/ram_common/ameba_uart.c) points at the *non-secure* UARTx_DEV bases
 * unconditionally, so -- exactly as on RTL8721DX/RTL8721F -- the driver
 * programs the controllers through the non-secure alias (0x401C_3000 /
 * 0x401C_4000).  The chip exposes UART0/UART1/UART2, but the LOG-UART owns
 * the console and only UART0/UART1 are wired here.
 */

#define AMEBA_NUART               2

#define AMEBA_UART_PORT_BASES     { 0x401c3000ul, 0x401c4000ul }

#define AMEBA_UART_PORT_IRQS      { RTL8720F_IRQ_UART0, RTL8720F_IRQ_UART1 }

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

#define AMEBA_UART_TXFID          { 65, 69 }   /* UART0_TXD, UART1_TXD */
#define AMEBA_UART_RXFID          { 66, 70 }   /* UART0_RXD, UART1_RXD */

#endif /* __ARCH_ARM_SRC_RTL8720F_AMEBA_UART_CHIP_H */
