/****************************************************************************
 * arch/risc-v/src/litex/hardware/litex_uart.h
 *
 *   Copyright (C) 2020 Gregory Nutt. All rights reserved.
 *   Author: hctang <aenrbesaen@126.com>
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
