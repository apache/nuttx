/****************************************************************************
 * arch/arm/src/moxart/chip.h
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

#ifndef __ARCH_ARM_SRC_MOXART_CHIP_H
#define __ARCH_ARM_SRC_MOXART_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#define UART0_BASE           0x98200000

#define UART_THR             0x00
#define UART_LSR             0x14
#define UART_LSR_THRE        0x20

/* Common UART Registers.  Expressed as offsets from the BASE address */

#define UART_RHR_OFFS        0x00000000 /* Rcv Holding Register */
#define UART_THR_OFFS        0x00000000 /* Xmit Holding Register */
#define UART_FCR_OFFS        0x00000002 /* FIFO Control Register */
#define UART_RFCR_OFFS       0x00000002 /* Rcv FIFO Control Register */
#define UART_TFCR_OFFS       0x00000002 /* Xmit FIFO Control Register */
#define UART_SCR_OFFS        0x00000010 /* Status Control Register */
#define UART_LCR_OFFS        0x00000003 /* Line Control Register */
#define UART_LSR_OFFS        0x00000005 /* Line Status Register */
#define UART_SSR_OFFS        0x00000011 /* Supplementary Status Register */
#define UART_MCR_OFFS        0x00000004 /* Modem Control Register */
#define UART_MSR_OFFS        0x00000006 /* Modem Status Register */
#define UART_IER_OFFS        0x00000001 /* Interrupt Enable Register */
#define UART_ISR_OFFS        0x00000002 /* Interrupt Status Register */
#define UART_EFR_OFFS        0x00000002 /* Enhanced Feature Register */
#define UART_XON1_OFFS       0x00000004 /* XON1 Character Register */
#define UART_XON2_OFFS       0x00000005 /* XON2 Character Register */
#define UART_XOFF1_OFFS      0x00000006 /* XOFF1 Character Register */
#define UART_XOFF2_OFFS      0x00000007 /* XOFF2 Character Register */
#define UART_SPR_OFFS        0x00000007 /* Scratch-pad Register */
#define UART_DIV_LOW_OFFS    0x00000000 /* Divisor for baud generation */
#define UART_DIV_HIGH_OFFS   0x00000001
#define UART_TCR_OFFS        0x00000006 /* Transmission Control Register */
#define UART_TLR_OFFS        0x00000007 /* Trigger Level Register */
#define UART_MDR_OFFS        0x00000008 /* Mode Definition Register */

/* UART Settings ************************************************************/

/* Miscellaneous UART settings. */

#define IRQ_UART             1
#define UART_REGISTER_BITS   8
#define UART_IRQ             IRQ_UART

#define UART_RX_FIFO_NOEMPTY 0x00000001
#define UART_SSR_TXFULL      0x00000001
#define UART_LSR_TREF        0x00000020

#define UART_XMIT_FIFO_SIZE      64
#define UART_IRDA_XMIT_FIFO_SIZE 64

/* UART_LCR Register */

                                        /* Bits 31-7: Reserved */
#define UART_LCR_BOC         0x00000040 /* Bit 6: Break Control */
                                        /* Bit 5: Parity Type 2 */
#define UART_LCR_PAREVEN     0x00000010 /* Bit 4: Parity Type 1 */
#define UART_LCR_PARODD      0x00000000
#define UART_LCR_PARMARK     0x00000010
#define UART_LCR_PARSPACE    0x00000011
#define UART_LCR_PAREN       0x00000008 /* Bit 3: Paity Enable */
#define UART_LCR_PARDIS      0x00000000
#define UART_LCR_2STOP       0x00000004 /* Bit 2: Number of stop bits */
#define UART_LCR_1STOP       0x00000000
#define UART_LCR_5BITS       0x00000000 /* Bits 0-1: Word-length */
#define UART_LCR_6BITS       0x00000001
#define UART_LCR_7BITS       0x00000002
#define UART_LCR_8BITS       0x00000003

#define UART_FCR_FTL         0x000000f0
#define UART_FCR_FIFO_EN     0x00000001
#define UART_FCR_TX_CLR      0x00000002
#define UART_FCR_RX_CLR      0x00000004

#define UART_IER_RECVINT     0x00000001
#define UART_IER_XMITINT     0x00000002
#define UART_IER_LINESTSINT  0x00000004
#define UART_IER_MODEMSTSINT 0x00000008 /* IrDA UART only */
#define UART_IER_XOFFINT     0x00000020
#define UART_IER_RTSINT      0x00000040 /* IrDA UART only */
#define UART_IER_CTSINT      0x00000080 /* IrDA UART only */
#define UART_IER_INTMASK     0x000000ff

#define BAUD_115200          0x00000007
#define BAUD_57600           0x00000014
#define BAUD_38400           0x00000021
#define BAUD_19200           0x00000006
#define BAUD_9600            0x0000000C
#define BAUD_4800            0x00000018
#define BAUD_2400            0x00000030
#define BAUD_1200            0x00000060

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_MOXART_CHIP_H */
