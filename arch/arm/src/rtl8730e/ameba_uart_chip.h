/****************************************************************************
 * arch/arm/src/rtl8730e/ameba_uart_chip.h
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

#ifndef __ARCH_ARM_SRC_RTL8730E_AMEBA_UART_CHIP_H
#define __ARCH_ARM_SRC_RTL8730E_AMEBA_UART_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <nuttx/irq.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Per-chip UART wiring for RTL8730E (amebasmart CA32).  The shared driver
 * (arch/arm/src/common/ameba/ameba_uart.c) includes this header to learn how
 * many general-purpose UARTs the chip exposes and, for each, its register
 * base, peripheral-clock masks, GIC interrupt vector and pad-mux codes.
 *
 * RTL8730E exposes UART0-2 to the CA32 domain.  UART3 is reserved for
 * Bluetooth and is not registered here.  The LOG-UART (LOGUART_REG_BASE
 * 0x4200c000) is the NuttX console (/dev/ttyS0) and is managed by
 * rtl8730e_serial.c; UART0-2 appear as /dev/ttyS1-3.
 *
 * GIC SPI numbers come from SDK ameba_vector_table.h (CA32 section):
 *   UART0_IRQ = 50, UART1_IRQ = 51, UART2_IRQ = 52.
 * NuttX IRQ = GIC SPI number + 32.
 *
 * APBPeriph and APBPeriph_CLOCK values differ on amebasmart; both lists are
 * supplied so RCC_PeriphClockCmd() receives correct distinct arguments.
 * Source: component/soc/amebasmart/fwlib/include/sysreg_lsys.h.
 *
 * amebasmart uses a single PINMUX_FUNCTION_UART (1) for both TX and RX;
 * there are no separate TXD/RXD function codes (unlike amebalite/green2).
 */

#define AMEBA_NUART               3

#define AMEBA_UART_PORT_BASES     \
        { 0x41004000ul, 0x41005000ul, 0x41006000ul }

/* NuttX IRQ = GIC SPI + 32 */

#define AMEBA_UART_PORT_IRQS      \
        { 82, 83, 84 }

/* APBPeriph_UARTx (function reset) masks:
 *   UART0: (2<<30)|(1<<4)|(1<<0)
 *   UART1: (2<<30)|(1<<5)|(1<<0)
 *   UART2: (2<<30)|(1<<6)|(1<<0)
 */

#define AMEBA_UART_APBPERIPH      \
        { ((uint32_t)(2 << 30) | (1u << 4) | (1u << 0)), \
          ((uint32_t)(2 << 30) | (1u << 5) | (1u << 0)), \
          ((uint32_t)(2 << 30) | (1u << 6) | (1u << 0)) }

/* APBPeriph_UARTx_CLOCK masks:
 *   UART0: (2<<30)|(1<<0)
 *   UART1: (2<<30)|(1<<1)
 *   UART2: (2<<30)|(1<<2)
 */

#define AMEBA_UART_APBPERIPH_CLK  \
        { ((uint32_t)(2 << 30) | (1u << 0)), \
          ((uint32_t)(2 << 30) | (1u << 1)), \
          ((uint32_t)(2 << 30) | (1u << 2)) }

/* amebasmart uses PINMUX_FUNCTION_UART (1) for both TX and RX pads.
 * No direction-specific codes exist on this chip family.
 */

#define AMEBA_UART_TXFID          { 1, 1, 1 }  /* PINMUX_FUNCTION_UART */
#define AMEBA_UART_RXFID          { 1, 1, 1 }  /* PINMUX_FUNCTION_UART */

#endif /* __ARCH_ARM_SRC_RTL8730E_AMEBA_UART_CHIP_H */
