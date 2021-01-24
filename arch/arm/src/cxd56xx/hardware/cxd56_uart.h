/****************************************************************************
 * arch/arm/src/cxd56xx/hardware/cxd56_uart.h
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

#ifndef __ARCH_ARM_SRC_CXD56XX_HARDWARE_CXD56_UART_H
#define __ARCH_ARM_SRC_CXD56XX_HARDWARE_CXD56_UART_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

/* Common Register Offsets */

#define CXD56_UART_DR       0x000    /* Data register */
#define CXD56_UART_RSR_ECR  0x004    /* Receive status/error clear register */
#define CXD56_UART_FR       0x018    /* Flag register */
#define CXD56_UART_ILPR     0x020    /* IrDA low-power counter register */
#define CXD56_UART_IBRD     0x024    /* Integer baud rate register */
#define CXD56_UART_FBRD     0x028    /* Fractional baud rate register */
#define CXD56_UART_LCR_H    0x02c    /* Line control register */
#define CXD56_UART_CR       0x030    /* Control register */
#define CXD56_UART_IFLS     0x034    /* Interrupt FIFO level select register */
#define CXD56_UART_IMSC     0x038    /* Interrupt mask set/clear register */
#define CXD56_UART_RIS      0x03c    /* Raw Interrupt status register */
#define CXD56_UART_MIS      0x040    /* Masked interrupt status register */
#define CXD56_UART_ICR      0x044    /* Interrupt clear register */
#define CXD56_UART_DMACR    0x048    /* DMA control register */

#define  UART_FR_RI         (0x1 << 8)
#define  UART_FR_TXFE       (0x1 << 7)
#define  UART_FR_RXFF       (0x1 << 6)
#define  UART_FR_TXFF       (0x1 << 5)
#define  UART_FR_RXFE       (0x1 << 4)
#define  UART_FR_BUSY       (0x1 << 3)
#define  UART_FR_DCD        (0x1 << 2)
#define  UART_FR_DSR        (0x1 << 1)
#define  UART_FR_CTS        (0x1 << 0)

#define UART_LCR_SPS        (1u << 7)
#define UART_LCR_WLEN(x)    ((((x)-5)&3)<<5)
#define UART_LCR_FEN        (1u << 4)
#define UART_LCR_STP2       (1u << 3)
#define UART_LCR_EPS        (1u << 2)
#define UART_LCR_PEN        (1u << 1)
#define UART_LCR_BRK        (1u << 0)

#define UART_CR_CTSEN       (1u << 15)
#define UART_CR_RTSEN       (1u << 14)
#define UART_CR_OUT2        (1u << 13)
#define UART_CR_OUT1        (1u << 12)
#define UART_CR_RTS         (1u << 11)
#define UART_CR_DTR         (1u << 10)
#define UART_CR_RXE         (1u << 9)
#define UART_CR_TXE         (1u << 8)
#define UART_CR_LBE         (1u << 7)
#define UART_CR_SIRLP       (1u << 2)
#define UART_CR_SIREN       (1u << 1)
#define UART_CR_EN          (1u << 0)

#define UART_INTR_RI        (1u << 0)  /* nUARTRI modem interrupt */
#define UART_INTR_CTS       (1u << 1)  /* nUARTCTS modem interrupt */
#define UART_INTR_DCD       (1u << 2)  /* nUARTDCD modem interrupt */
#define UART_INTR_DSR       (1u << 3)  /* nUARTDSR modem interrupt */
#define UART_INTR_RX        (1u << 4)  /* Receive interrupt */
#define UART_INTR_TX        (1u << 5)  /* Transmit interrupt */
#define UART_INTR_RT        (1u << 6)  /* Receive timeout interrupt */
#define UART_INTR_FE        (1u << 7)  /* Framing error interrupt */
#define UART_INTR_PE        (1u << 8)  /* Parity error interrupt */
#define UART_INTR_BE        (1u << 9)  /* Break error interrupt */
#define UART_INTR_OE        (1u << 10) /* Overrun error interrupt */
#define UART_INTR_ALL       (0x7ff)    /* All of interrupts */

#define UART_FLAG_RI        (1u << 8)  /* Ring indicator */
#define UART_FLAG_TXFE      (1u << 7)  /* Transmit FIFO empty */
#define UART_FLAG_RXFF      (1u << 6)  /* Receive FIFO full */
#define UART_FLAG_TXFF      (1u << 5)  /* Transmit FIFO full */
#define UART_FLAG_RXFE      (1u << 4)  /* Receive FIFO empty */
#define UART_FLAG_BUSY      (1u << 3)  /* UART busy */
#define UART_FLAG_DCD       (1u << 2)  /* Data carrier detect */
#define UART_FLAG_DSR       (1u << 1)  /* Data set ready */
#define UART_FLAG_CTS       (1u << 0)  /* Cleart to send */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_CXD56XX_HARDWARE_CXD56_UART_H */
