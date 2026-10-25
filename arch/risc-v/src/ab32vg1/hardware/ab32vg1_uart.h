/****************************************************************************
 * arch/risc-v/src/ab32vg1/hardware/ab32vg1_uart.h
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

#ifndef __ARCH_RISCV_SRC_AB32VG1_HARDWARE_AB32VG1_UART_H
#define __ARCH_RISCV_SRC_AB32VG1_HARDWARE_AB32VG1_UART_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/ab32vg1_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* UART0/1/2 (register manual, chapter 8).  Each UART has the same four
 * consecutive registers.
 */

#define AB32VG1_UART0_BASE      AB32VG1_SFR(0, 0x10)
#define AB32VG1_UART1_BASE      AB32VG1_SFR(0, 0x30)
#define AB32VG1_UART2_BASE      AB32VG1_SFR(9, 0x18)

#define AB32VG1_UART_CON_OFFSET   0x00
#define AB32VG1_UART_CPND_OFFSET  0x04
#define AB32VG1_UART_BAUD_OFFSET  0x08
#define AB32VG1_UART_DATA_OFFSET  0x0c

/* UARTxCON */

#define UART_CON_UTEN           (1 << 0)   /* UART enable */
#define UART_CON_BIT9EN         (1 << 1)   /* 9-bit mode */
#define UART_CON_RXIE           (1 << 2)   /* RX interrupt enable */
#define UART_CON_TXIE           (1 << 3)   /* TX interrupt enable */
#define UART_CON_SB2EN          (1 << 4)   /* 2 stop bits */
#define UART_CON_CLKSRC         (1 << 5)   /* 0: system clock, 1: uart_inc */
#define UART_CON_ONELINE        (1 << 6)   /* TX/RX on one pin */
#define UART_CON_RXEN           (1 << 7)   /* RX enable */
#define UART_CON_TXPND          (1 << 8)   /* TX of one byte finished */
#define UART_CON_RXPND          (1 << 9)   /* RX of one byte finished */

/* UARTxCPND */

#define UART_CPND_CTXPND        (1 << 8)
#define UART_CPND_CRXPND        (1 << 9)
#define UART_CPND_CKEYPND       (1 << 16)
#define UART_CPND_CRSTKEYPND    (1 << 17)

/* UARTxBAUD: baud = Fclk / (n + 1); RX in [31:16], TX in [15:0].  With
 * CLKSRC = 1 the clock is uart_inc = XOSC 26 MHz / 2.
 */

#define UART_INC_FREQUENCY      13000000
#define UART_BAUD(n)            (((uint32_t)(n) << 16) | (uint32_t)(n))

#endif /* __ARCH_RISCV_SRC_AB32VG1_HARDWARE_AB32VG1_UART_H */
