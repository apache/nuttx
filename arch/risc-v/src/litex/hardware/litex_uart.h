/****************************************************************************
 * arch/risc-v/src/litex/hardware/litex_uart.h
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

#ifndef ARCH_RISCV_SRC_LITEX_CHIP_LITEX_UART_H
#define ARCH_RISCV_SRC_LITEX_CHIP_LITEX_UART_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define UART_RXTX_OFFSET        0x00
#define UART_TXFULL_OFFSET      0x04
#define UART_RXEMPTY_OFFSET     0x08
#define UART_EV_STATUS_OFFSET   0x0c
#define UART_EV_PENDING_OFFSET  0x10
#define UART_EV_ENABLE_OFFSET   0x14

#ifdef CONFIG_LITEX_UART0
#  define LITEX_UART0_RXTX          (LITEX_UART0_BASE + UART_RXTX_OFFSET)
#  define LITEX_UART0_TXFULL        (LITEX_UART0_BASE + UART_TXFULL_OFFSET)
#  define LITEX_UART0_RXEMPTY       (LITEX_UART0_BASE + UART_RXEMPTY_OFFSET)
#  define LITEX_UART0_EV_STATUS     (LITEX_UART0_BASE + UART_EV_STATUS_OFFSET)
#  define LITEX_UART0_EV_PENDING    (LITEX_UART0_BASE + UART_EV_PENDING_OFFSET)
#  define LITEX_UART0_EV_ENABLE     (LITEX_UART0_BASE + UART_EV_ENABLE_OFFSET)
#  define LITEX_UART0_PHY_TUNING_WORD   0xf0002000L
#endif

#define UART_EV_TX	0x1
#define UART_EV_RX	0x2

#endif /* _ARCH_RISCV_SRC_LITEX_CHIP_LITEX_UART_H */
