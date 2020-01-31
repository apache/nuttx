/****************************************************************************
 * arch/risc-v/src/k210/hardware/k210_uart.h
 *
 *   Copyright (C) 2019 Masayuki Ishikawa. All rights reserved.
 *   Author: Masayuki Ishikawa <masayuki.ishikawa@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef ARCH_RISCV_SRC_K210_CHIP_K210_UART_H
#define ARCH_RISCV_SRC_K210_CHIP_K210_UART_H

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

#ifdef CONFIG_K210_UART0
#  define K210_UART0_TXDATA  (K210_UART0_BASE + UART_TXDATA_OFFSET)
#  define K210_UART0_RXDATA  (K210_UART0_BASE + UART_RXDATA_OFFSET)
#  define K210_UART0_TXCTRL  (K210_UART0_BASE + UART_TXCTRL_OFFSET)
#  define K210_UART0_RXCTRL  (K210_UART0_BASE + UART_RXCTRL_OFFSET)
#  define K210_UART0_IE      (K210_UART0_BASE + UART_IE_OFFSET)
#  define K210_UART0_IP      (K210_UART0_BASE + UART_IP_OFFSET)
#  define K210_UART0_DIV     (K210_UART0_BASE + UART_DIV_OFFSET)
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

#endif /* _ARCH_RISCV_SRC_K210_CHIP_K210_UART_H */
