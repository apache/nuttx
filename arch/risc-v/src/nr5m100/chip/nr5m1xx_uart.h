/************************************************************************************
 * arch/risc-v/src/nr5m100/chip/nr5_uart.h
 *
 *   Copyright (C) 2016 Ken Pettit. All rights reserved.
 *   Author: Ken Pettit <pettitkd@gmail.com>
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
 ************************************************************************************/

#ifndef ARCH_RISCV_SRC_NR5M100_CHIP_NR5M1XX_UART_H
#define ARCH_RISCV_SRC_NR5M100_CHIP_NR5M1XX_UART_H

/* The UART in NR5M100 is a very small (i.e. dumb) peripheral.  It
 * only supports the most common mode ever used:
 *
 *   8 Data bits
 *   1 Stop bit
 *   No parity.
 *
 *   It has a programmable baud rate and RX / TX interrupt capability
 *   and that's about it.  The primary goal for the UART is to provide a
 *   debug console to the part.
 */

#define NR5_UART_BAUD_RATE_OFFSET         0x000
#define NR5_UART_TX_REG_OFFSET            0x004
#define NR5_UART_RX_REG_OFFSET            0x008
#define NR5_UART_STATUS_REG_OFFSET        0x00C
#define NR5_UART_CTRL_REG_OFFSET          0x010

#ifdef CONFIG_NR5_UART1
#  define NR5_UART1_BAUD_RATE_REG         (NR5_UART1_BASE+NR5_UART_BAUD_RATE_OFFSET)
#  define NR5_UART1_TX_REG                (NR5_UART1_BASE+NR5_UART_TX_REG_OFFSET)
#  define NR5_UART1_RX_REG                (NR5_UART1_BASE+NR5_UART_RX_REG_OFFSET)
#  define NR5_UART1_STATUS_REG            (NR5_UART1_BASE+NR5_UART_STATUS_REG_OFFSET)
#  define NR5_UART1_CTRL_REG              (NR5_UART1_BASE+NR5_UART_CTRL_REG_OFFSET)
#endif

/* Status Register Bit definitions */

#define NR5_UART_STATUS_TX_EMPTY          0x01
#define NR5_UART_STATUS_RX_AVAIL          0x02
#define NR5_UART_STATUS_RX_OVERRUN        0x04
#define NR5_UART_RX_IRQ_PENDING           0x08
#define NR5_UART_TX_IRQ_PENDING           0x10

/* Control Register Bit definitions */

#define NR5_UART_CTRL_ENABLE_RX_IRQ       0x01
#define NR5_UART_CTRL_ENABLE_TX_IRQ       0x02

#endif  /* _ARCH_RISCV_SRC_NR5M100_CHIP_NR5M1XX_UART_H */
