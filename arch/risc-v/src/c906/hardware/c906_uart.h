/****************************************************************************
 * arch/risc-v/src/c906/hardware/c906_uart.h
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

#ifndef ARCH_RISCV_SRC_C906_CHIP_C906_UART_H
#define ARCH_RISCV_SRC_C906_CHIP_C906_UART_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define UART_TXDATA_OFFSET   0x00
#define UART_RXDATA_OFFSET   0x00
#define UART_TXCTL_OFFSET    0x04
#define UART_RXCTL_OFFSET    0x04
#define UART_IE_OFFSET       0x04
#define UART_IIR_OFFSET      0x08
#define UART_DIV_OFFSET      0x04
#define UART_LSR_OFFSET      0x14
#define UART_MSR_OFFSET      0x18
#define UART_USR_OFFSET      0x7c

#ifdef CONFIG_C906_UART0
#  define C906_UART0_TXDATA  (C906_UART0_BASE + UART_TXDATA_OFFSET)
#  define C906_UART0_RXDATA  (C906_UART0_BASE + UART_RXDATA_OFFSET)
#  define C906_UART0_TXCTRL  (C906_UART0_BASE + UART_TXCTRL_OFFSET)
#  define C906_UART0_RXCTRL  (C906_UART0_BASE + UART_RXCTRL_OFFSET)
#  define C906_UART0_IE      (C906_UART0_BASE + UART_IE_OFFSET)
#  define C906_UART0_IP      (C906_UART0_BASE + UART_IP_OFFSET)
#  define C906_UART0_DIV     (C906_UART0_BASE + UART_DIV_OFFSET)
#  define C906_UART0_LSR     (C906_UART0_BASE + UART_LSR_OFFSET)
#  define C906_UART0_MSR     (C906_UART0_BASE + UART_MSR_OFFSET)
#  define C906_UART0_USR     (C906_UART0_BASE + UART_USR_OFFSET)
#endif

#define USR_UART_BUSY           0x01
#define USR_UART_TFE            0x04
#define USR_UART_RFNE           0x08
#define LSR_DATA_READY          0x01
#define LSR_THR_EMPTY           0x20
#define IER_RDA_INT_ENABLE      0x01
#define IER_THRE_INT_ENABLE     0x02
#define IIR_RECV_LINE_ENABLE    0x04
#define IIR_NO_ISQ_PEND         0x01

#define LCR_SET_DLAB            0x80   /* enable r/w DLR to set the baud rate */
#define LCR_PARITY_ENABLE       0x08   /* parity enabled */
#define LCR_PARITY_EVEN         0x10   /* Even parity enabled */
#define LCR_PARITY_ODD          0xef   /* Odd parity enabled */
#define LCR_WORD_SIZE_5         0xfc   /* the data length is 5 bits */
#define LCR_WORD_SIZE_6         0x01   /* the data length is 6 bits */
#define LCR_WORD_SIZE_7         0x02   /* the data length is 7 bits */
#define LCR_WORD_SIZE_8         0x03   /* the data length is 8 bits */
#define LCR_STOP_BIT1           0xfb   /* 1 stop bit */
#define LCR_STOP_BIT2           0x04   /* 1.5 stop bit */

#define DW_LSR_PFE              0x80
#define DW_LSR_TEMT             0x40
#define DW_LSR_THRE             0x40
#define DW_LSR_BI               0x10
#define DW_LSR_FE               0x08
#define DW_LSR_PE               0x04
#define DW_LSR_OE               0x02
#define DW_LSR_DR               0x01
#define DW_LSR_TRANS_EMPTY      0x20

#define DW_IIR_THR_EMPTY        0x02    /* threshold empty */
#define DW_IIR_RECV_DATA        0x04    /* received data available */
#define DW_IIR_RECV_LINE        0x06    /* receiver line status */
#define DW_IIR_CHAR_TIMEOUT     0x0c    /* character timeout */

#endif /* ARCH_RISCV_SRC_C906_CHIP_C906_UART_H */
