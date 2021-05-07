/****************************************************************************
 * arch/risc-v/src/fe310/hardware/fe310_uart.h
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

#ifndef ARCH_RISCV_SRC_FE310_CHIP_FE310_UART_H
#define ARCH_RISCV_SRC_FE310_CHIP_FE310_UART_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define UART_TXDATA_OFFSET   0x00
#define UART_RXDATA_OFFSET   0x04
#define UART_TXCTL_OFFSET    0x08
#define UART_RXCTL_OFFSET    0x0c
#define UART_IE_OFFSET       0x10
#define UART_IP_OFFSET       0x14
#define UART_DIV_OFFSET      0x18

#ifdef CONFIG_FE310_UART0
#  define FE310_UART0_TXDATA  (FE310_UART0_BASE + UART_TXDATA_OFFSET)
#  define FE310_UART0_RXDATA  (FE310_UART0_BASE + UART_RXDATA_OFFSET)
#  define FE310_UART0_TXCTRL  (FE310_UART0_BASE + UART_TXCTRL_OFFSET)
#  define FE310_UART0_RXCTRL  (FE310_UART0_BASE + UART_RXCTRL_OFFSET)
#  define FE310_UART0_IE      (FE310_UART0_BASE + UART_IE_OFFSET)
#  define FE310_UART0_IP      (FE310_UART0_BASE + UART_IP_OFFSET)
#  define FE310_UART0_DIV     (FE310_UART0_BASE + UART_DIV_OFFSET)
#endif

#define UART_TX_FULL   (1 << 31) /* TX FIFO full  (in TXDATA) */
#define UART_RX_EMPTY  (1 << 31) /* RX FIFO empty (in RXDATA) */

#define UART_TX_EN     (1 << 0)  /* Enable TX (in TXCTL) */
#define UART_NSTOP     (1 << 1)  /* Number of stop bits (in TXCTL) */

#define UART_RX_EN     (1 << 0)  /* Enable RX (in RXCTL) */

#define UART_IE_TXWM   (1 << 0)  /* Enable TX wartermark int (in IE) */
#define UART_IE_RXWM   (1 << 1)  /* Enable RX wartermark int (in IE) */

#define UART_IP_TXWM   (1 << 0)  /* TX wartermark pending (in IP) */
#define UART_IP_RXWM   (1 << 1)  /* RX wartermark pending (in IP) */

#endif /* _ARCH_RISCV_SRC_FE310_CHIP_FE310_UART_H */
