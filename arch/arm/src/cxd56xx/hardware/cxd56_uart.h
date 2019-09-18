/****************************************************************************
 * arch/arm/src/cxd56xx/hardware/cxd56_uart.h
 *
 *   Copyright 2018 Sony Semiconductor Solutions Corporation
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
 * 3. Neither the name of Sony Semiconductor Solutions Corporation nor
 *    the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
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
#define UART_INTR_ALL       (0x7ff)  /* All of interrupts */

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
 * Public Functions
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_CXD56XX_HARDWARE_CXD56_UART_H */
