/****************************************************************************
 * arch/arm/src/rtl8721f/ameba_uart_chip.h
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

#ifndef __ARCH_ARM_SRC_RTL8721F_AMEBA_UART_CHIP_H
#define __ARCH_ARM_SRC_RTL8721F_AMEBA_UART_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <nuttx/irq.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Per-chip UART wiring for RTL8721F (amebagreen2).  The shared driver
 * (arch/arm/src/common/ameba/ameba_uart.c) includes this header to learn how
 * many general-purpose UARTs the chip exposes and, for each, its register
 * base, peripheral-clock masks, NVIC vector and crossbar pad-mux codes.  See
 * the RTL8721DX version of this header for the full contract; the notes
 * below cover only what differs on this chip.
 *
 * The AP core is km4tz (TrustZone secure) here, but the fwlib UART_DEV_TABLE
 * (fwlib/ram_common/ameba_uart.c) points at the *non-secure* UARTx_DEV bases
 * unconditionally, so -- exactly as on RTL8721DX -- the driver programs the
 * controllers through the non-secure alias (0x4080_C000 / 0x4080_D000).  The
 * APBPeriph function/clock masks match RTL8721DX; only the register bases,
 * NVIC vectors and pad-mux function codes are chip-specific.
 */

#define AMEBA_NUART               2

#define AMEBA_UART_PORT_BASES     { 0x4080c000ul, 0x4080d000ul }

#define AMEBA_UART_PORT_IRQS      { RTL8721F_IRQ_UART0, RTL8721F_IRQ_UART1 }

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

#define AMEBA_UART_TXFID          { 95, 99 }   /* UART0_TXD, UART1_TXD */
#define AMEBA_UART_RXFID          { 96, 100 }  /* UART0_RXD, UART1_RXD */

#endif /* __ARCH_ARM_SRC_RTL8721F_AMEBA_UART_CHIP_H */
