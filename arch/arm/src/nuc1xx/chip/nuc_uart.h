/********************************************************************************************
 * arch/arm/src/nuc1xx/chip/nuc_uart.h
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
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
 ********************************************************************************************/

#ifndef __ARCH_ARM_SRC_NUC1XX_CHIP_NUC_UART_H
#define __ARCH_ARM_SRC_NUC1XX_CHIP_NUC_UART_H

/********************************************************************************************
 * Included Files
 ********************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/********************************************************************************************
 * Pre-processor Definitions
 ********************************************************************************************/
/* Register offsets *************************************************************************/

#define NUC_UART_RBR_OFFSET        0x0000 /* UART receive buffer register */
#define NUC_UART_THR_OFFSET        0x0000 /* UART transmit holding register */
#define NUC_UART_IER_OFFSET        0x0004 /* UART interrupt enable register */
#define NUC_UART_FCR_OFFSET        0x0008 /* UART FIFO control register */
#define NUC_UART_LCR_OFFSET        0x000c /* UART line control register */
#define NUC_UART_MCR_OFFSET        0x0010 /* UART modem control register */
#define NUC_UART_MSR_OFFSET        0x0014 /* UART modem status register */
#define NUC_UART_FSR_OFFSET        0x0018 /* UART FIFO status register */
#define NUC_UART_ISR_OFFSET        0x001c /* UART interrupt status register */
#define NUC_UART_TOR_OFFSET        0x0020 /* UART time out register */
#define NUC_UART_BAUD_OFFSET       0x0024 /* UART BAUD rate divisor register */
#define NUC_UART_IRCR_OFFSET       0x0028 /* UART IrDA control register */
#define NUC_UART_ALT_CSR_OFFSET    0x002c /* UART alternate control/status register */
#define NUC_UART_FUN_SEL_OFFSET    0x0030 /* UART function select register */

/* Register addresses ***********************************************************************/

#define NUC_UART0_RBR              (NUC_UART0_BASE+NUC_UART_RBR_OFFSET)
#define NUC_UART0_THR              (NUC_UART0_BASE+NUC_UART_THR_OFFSET)
#define NUC_UART0_IER              (NUC_UART0_BASE+NUC_UART_IER_OFFSET)
#define NUC_UART0_FCR              (NUC_UART0_BASE+NUC_UART_FCR_OFFSET)
#define NUC_UART0_LCR              (NUC_UART0_BASE+NUC_UART_LCR_OFFSET)
#define NUC_UART0_MCR              (NUC_UART0_BASE+NUC_UART_MCR_OFFSET)
#define NUC_UART0_MSR              (NUC_UART0_BASE+NUC_UART_MSR_OFFSET)
#define NUC_UART0_FSR              (NUC_UART0_BASE+NUC_UART_FSR_OFFSET)
#define NUC_UART0_ISR              (NUC_UART0_BASE+NUC_UART_ISR_OFFSET)
#define NUC_UART0_TOR              (NUC_UART0_BASE+NUC_UART_TOR_OFFSET)
#define NUC_UART0_BAUD             (NUC_UART0_BASE+NUC_UART_BAUD_OFFSET)
#define NUC_UART0_IRCR             (NUC_UART0_BASE+NUC_UART_IRCR_OFFSET)
#define NUC_UART0_ALT_CSR          (NUC_UART0_BASE+NUC_UART_ALT_CSR_OFFSET)
#define NUC_UART0_FUN_SEL          (NUC_UART0_BASE+NUC_UART_FUN_SEL_OFFSET)

#define NUC_UART1_RBR              (NUC_UART1_BASE+NUC_UART_RBR_OFFSET)
#define NUC_UART1_THR              (NUC_UART1_BASE+NUC_UART_THR_OFFSET)
#define NUC_UART1_IER              (NUC_UART1_BASE+NUC_UART_IER_OFFSET)
#define NUC_UART1_FCR              (NUC_UART1_BASE+NUC_UART_FCR_OFFSET)
#define NUC_UART1_LCR              (NUC_UART1_BASE+NUC_UART_LCR_OFFSET)
#define NUC_UART1_MCR              (NUC_UART1_BASE+NUC_UART_MCR_OFFSET)
#define NUC_UART1_MSR              (NUC_UART1_BASE+NUC_UART_MSR_OFFSET)
#define NUC_UART1_FSR              (NUC_UART1_BASE+NUC_UART_FSR_OFFSET)
#define NUC_UART1_ISR              (NUC_UART1_BASE+NUC_UART_ISR_OFFSET)
#define NUC_UART1_TOR              (NUC_UART1_BASE+NUC_UART_TOR_OFFSET)
#define NUC_UART1_BAUD             (NUC_UART1_BASE+NUC_UART_BAUD_OFFSET)
#define NUC_UART1_IRCR             (NUC_UART1_BASE+NUC_UART_IRCR_OFFSET)
#define NUC_UART1_ALT_CSR          (NUC_UART1_BASE+NUC_UART_ALT_CSR_OFFSET)
#define NUC_UART1_FUN_SEL          (NUC_UART1_BASE+NUC_UART_FUN_SEL_OFFSET)

#define NUC_UART2_RBR              (NUC_UART2_BASE+NUC_UART_RBR_OFFSET)
#define NUC_UART2_THR              (NUC_UART2_BASE+NUC_UART_THR_OFFSET)
#define NUC_UART2_IER              (NUC_UART2_BASE+NUC_UART_IER_OFFSET)
#define NUC_UART2_FCR              (NUC_UART2_BASE+NUC_UART_FCR_OFFSET)
#define NUC_UART2_LCR              (NUC_UART2_BASE+NUC_UART_LCR_OFFSET)
#define NUC_UART2_MCR              (NUC_UART2_BASE+NUC_UART_MCR_OFFSET)
#define NUC_UART2_MSR              (NUC_UART2_BASE+NUC_UART_MSR_OFFSET)
#define NUC_UART2_FSR              (NUC_UART2_BASE+NUC_UART_FSR_OFFSET)
#define NUC_UART2_ISR              (NUC_UART2_BASE+NUC_UART_ISR_OFFSET)
#define NUC_UART2_TOR              (NUC_UART2_BASE+NUC_UART_TOR_OFFSET)
#define NUC_UART2_BAUD             (NUC_UART2_BASE+NUC_UART_BAUD_OFFSET)
#define NUC_UART2_IRCR             (NUC_UART2_BASE+NUC_UART_IRCR_OFFSET)
#define NUC_UART2_ALT_CSR          (NUC_UART2_BASE+NUC_UART_ALT_CSR_OFFSET)
#define NUC_UART2_FUN_SEL          (NUC_UART2_BASE+NUC_UART_FUN_SEL_OFFSET)

/* Register bit-field definitions ***********************************************************/

/* UART receive buffer register */

#define UART_RBR_MASK              (0xff)

/* UART transmit holding register */

#define UART_THR_MASK              (0xff)

/* UART interrupt enable register */

#define UART_IER_RDA_IEN           (1 << 0)  /* Bit 0:  Receive data avaiable interrupt enable */
#define UART_IER_THRE_IEN          (1 << 1)  /* Bit 1:  Transmit holding register empty interrupt enable */
#define UART_IER_RLS_IEN           (1 << 2)  /* Bit 2:  Receive line status interrupt enable */
#define UART_IER_MODEM_IEN         (1 << 3)  /* Bit 3:  Modem status interrupt enable (UART0/1) */
#define UART_IER_RTO_IEN           (1 << 4)  /* Bit 4:  RX timeout interrupt enable */
#define UART_IER_BUF_ERR_IEN       (1 << 5)  /* Bit 5:  Buffer error interrupt enable */
#define UART_IER_WAKE_EN           (1 << 6)  /* Bit 6:  UART wake-up function enable (UART0/1) */
#define UART_IER_TIME_OUT_EN       (1 << 11) /* Bit 11: Time out counter enable */
#define UART_IER_AUTO_RTS_EN       (1 << 12) /* Bit 12: RTS auto flow control enable (UART0/1) */
#define UART_IER_AUTO_CTS_EN       (1 << 13) /* Bit 13: CTS auto flow control enable (UART0/1) */
#define UART_IER_DMA_TX_EN         (1 << 14) /* Bit 14: TX DMA enable (UART0/1) */
#define UART_IER_DMA_RX_EN         (1 << 15) /* Bit 15: RX DMA enable (UART0/1) */

#define UART_IER_ALLIE             (0x0000003f)
#define UART_IER_ALLBITS           (0x0000f87f)

/* UART FIFO control register */

#define UART_FCR_RFR               (1 << 1)  /* Bit 1:  RX FIFO software reset */
#define UART_FCR_TFR               (1 << 2)  /* Bit 2:  TX FIFO software reset */
#define UART_FCR_RFITL_SHIFT       (4)       /* Bits 4-7: RX FIFO interrupt trigger level */
#define UART_FCR_RFITL_MASK        (15 << UART_FCR_RFITL_SHIFT)
#  define UART_FCR_RFITL_1         (0 << UART_FCR_RFITL_SHIFT)
#  define UART_FCR_RFITL_4         (1 << UART_FCR_RFITL_SHIFT)
#  define UART_FCR_RFITL_8         (2 << UART_FCR_RFITL_SHIFT)
#  define UART_FCR_RFITL_14        (3 << UART_FCR_RFITL_SHIFT)
#  define UART_FCR_RFITL_30        (4 << UART_FCR_RFITL_SHIFT) /* High speed */
#  define UART_FCR_RFITL_46        (5 << UART_FCR_RFITL_SHIFT) /* High speed */
#  define UART_FCR_RFITL_62        (6 << UART_FCR_RFITL_SHIFT) /* High speed */
#define UART_FCR_RX_DIS            (1 << 8)  /* Bit 8:  Recive disable register */
#define UART_FCR_RTS_TRI_LEV_SHIFT (16)      /* Bits 16-19: RTS trigger level for auto flow control */
#define UART_FCR_RTS_TRI_LEV_MASK  (15 << UART_FCR_RTS_TRI_LEV_SHIFT)
#  define UART_FCR_RTS_TRI_LEV_1   (0 << UART_FCR_RTS_TRI_LEV_SHIFT)
#  define UART_FCR_RTS_TRI_LEV_4   (1 << UART_FCR_RTS_TRI_LEV_SHIFT)
#  define UART_FCR_RTS_TRI_LEV_8   (2 << UART_FCR_RTS_TRI_LEV_SHIFT)
#  define UART_FCR_RTS_TRI_LEV_14  (3 << UART_FCR_RTS_TRI_LEV_SHIFT)
#  define UART_FCR_RTS_TRI_LEV_30  (4 << UART_FCR_RTS_TRI_LEV_SHIFT) /* High speed */
#  define UART_FCR_RTS_TRI_LEV_46  (5 << UART_FCR_RTS_TRI_LEV_SHIFT) /* High speed */
#  define UART_FCR_RTS_TRI_LEV_62  (6 << UART_FCR_RTS_TRI_LEV_SHIFT) /* High speed */

/* UART line control register */

#define UART_LCR_WLS_SHIFT         (0)       /* Bits 0-1: Word length select */
#define UART_LCR_WLS_MASK          (3 << UART_LCR_WLS_SHIFT)
#  define UART_LCR_WLS_5           (0 << UART_LCR_WLS_SHIFT)
#  define UART_LCR_WLS_6           (1 << UART_LCR_WLS_SHIFT)
#  define UART_LCR_WLS_7           (2 << UART_LCR_WLS_SHIFT)
#  define UART_LCR_WLS_8           (3 << UART_LCR_WLS_SHIFT)
#define UART_LCR_NSB               (1 << 2)  /* Bit 2:  Number of stop bits */
#define UART_LCR_PBE               (1 << 3)  /* Bit 3:  Parity enable bit */
#define UART_LCR_EPE               (1 << 4)  /* Bit 4:  Even parity enable bit */
#define UART_LCR_SPE               (1 << 5)  /* Bit 5:  Sticky parity enable bit */
#define UART_LCR_BCB               (1 << 6)  /* Bit 6:  Break control bit */

/* UART modem control register */

#define UART_MCR_RTS               (1 << 1)  /* Bit 1:  RTS signal (UART0/1) */
#define UART_MCR_LEV_RTS           (1 << 9)  /* Bit 9:  RTS trigger level (UART0/1) */
#define UART_MCR_RTS_ST            (1 << 13) /* Bit 13: RTS pin state (UART0/1) */

/* UART modem status register */

#define UART_MSR_DCTSF             (1 << 0)  /* Bit 0:  CTS state change detected (UART0/1) */
#define UART_MSR_CTS_ST            (1 << 4)  /* Bit 4:  CTS pin status (UART0/1) */
#define UART_MSR_LEV_CTS           (1 << 8)  /* Bit 8:  CTS trigger level (UART0/1) */

/* UART FIFO status register */

#define UART0_FIFO_DEPTH           64
#define UART1_FIFO_DEPTH           16
#define UART2_FIFO_DEPTH           16

#define UART_FSR_RX_OVER_IF        (1 << 0)  /* Bit 0:  RX overflow error interrupt flag */
#define UART_FSR_RS485_ADD_DETF    (1 << 3)  /* Bit 3:  RS-485 address byte detection flag */
#define UART_FSR_PEF               (1 << 4)  /* Bit 4:  Parity error flag */
#define UART_FSR_FEF               (1 << 5)  /* Bit 5:  Framing error flag */
#define UART_FSR_BIF               (1 << 6)  /* Bit 6:  Break interrupt flag */
#define UART_FSR_RX_POINTER_SHIFT  (8)       /* Bits 8-13: RX FIFO pointer */
#define UART_FSR_RX_POINTER_MASK   (0x3f << UART_FSR_RX_POINTER_SHIFT)
#define UART_FSR_RX_EMPTY          (1 << 14) /* Bit 14: Receiver FIFO empty */
#define UART_FSR_TX_POINTER_SHIFT  (16)      /* Bits 16-21: TX FIFO pointer */
#define UART_FSR_TX_POINTER_MASK   (0x3f << UART_FSR_TX_POINTER_SHIFT)
#define UART_FSR_TX_EMPTY          (1 << 22) /* Bit 22: Transmitter FIFO empty flag */
#define UART_FSR_TX_OVER_IF        (1 << 24) /* Bit 24: TX overflow error interrupt flag */
#define UART_FSR_TE_FLAG           (1 << 28) /* Bit 28: Transmitter empty flag */

/* UART interrupt status register */

#define UART_ISR_RDA_IF            (1 << 0)  /* Bit 0:  Receive data available interrupt flag */
#define UART_ISR_THRE_IF           (1 << 1)  /* Bit 1:  Transmit holding register empty interrupt flag */
#define UART_ISR_RLS_IF            (1 << 2)  /* Bit 2:  Receive line status interrupt flag */
#define UART_ISR_MODEM_IF          (1 << 3)  /* Bit 3:  Modem interrupt flag (UART0/1) */
#define UART_ISR_TOUT_IF           (1 << 4)  /* Bit 4:  Timeout interrupt flag */
#define UART_ISR_BUF_ERR_IF        (1 << 5)  /* Bit 5:  Buffer error interrupt flag */
#define UART_ISR_RDA_INT           (1 << 8)  /* Bit 8:  Receive data avaiable interrupt indicator */
#define UART_ISR_THRE_INT          (1 << 9)  /* Bit 9:  Transmit holding register empty interrupt indicator */
#define UART_ISR_RLS_INT           (1 << 10) /* Bit 10: Receive line status interrupt indicator */
#define UART_ISR_MODEM_INT         (1 << 11) /* Bit 11: Modem interrupt indicator (UART0/1) */
#define UART_ISR_TOUT_INT          (1 << 12) /* Bit 12: Timeout interrupt indicator */
#define UART_ISR_BUF_ERR_INT       (1 << 13) /* Bit 13: Buffer error interrupt indicator */
#define UART_ISR_HW_RLS_IF         (1 << 18) /* Bit 18: DMA mode receive line status interrupt flag */
#define UART_ISR_HW_MODEM_IF       (1 << 19) /* Bit 19: DMA mode modem interrupt flag (UART0/1) */
#define UART_ISR_HW_TOUT_IF        (1 << 20) /* Bit 20: DMA mode timeout interrupt flag */
#define UART_ISR_HW_BUF_ERR_IF     (1 << 21) /* Bit 21: DMA mode buffer error interrupt flag */
#define UART_ISR_HW_RLS_INT        (1 << 26) /* Bit 26: DMA mode receive line status interrupt indicator */
#define UART_ISR_HW_MODEM_INT      (1 << 27) /* Bit 27: DMA mode modem interrupt indicator (UART0/1) */
#define UART_ISR_HW_TOUT_INT       (1 << 28) /* Bit 28: DMA mode timeout interrupt indicator */
#define UART_ISR_HW_BUF_ERR_INT    (1 << 29) /* Bit 29: DMA mode buffer error interrupt indicator */

/* UART time out register */

#define UART_TOR_TOIC_SHIFT        (0)       /* Bits 0-7: Time out interrupt comparator */
#define UART_TOR_TOIC_MASK         (0xff << UART_TOR_TOIC_SHIFT)
#  define UART_TOR_TOIC(t)         ((t) << UART_TOR_TOIC_SHIFT)
#define UART_TOR_DLY_SHIFT         (8)       /* Bits 8-15: TX delay time value */
#define UART_TOR_DLY_MASK          (0xff << UART_TOR_DLY_SHIFT)
#  define UART_TOR_DLY(d)          ((d) << UART_TOR_DLY_SHIFT)

/* UART BAUD rate divisor register */

#define UART_BAUD_BRD_SHIFT        (0)       /* Bits 0-15: Baud rate divider */
#define UART_BAUD_BRD_MASK         (0xffff << UART_BAUD_BRD_SHIFT)
#  define UART_BAUD_BRD(b)         ((b) << UART_BAUD_BRD_SHIFT)
#define UART_BAUD_DIVIDER_X_SHIFT  (24)      /* Bits 24-27: Divider X */
#define UART_BAUD_DIVIDER_X_MASK   (15 << UART_BAUD_DIVIDER_X_SHIFT)
#  define UART_BAUD_DIVIDER_X(x)   ((x) << UART_BAUD_DIVIDER_X_SHIFT)
#define UART_BAUD_DIV_X_ONE        (1 << 28) /* Bit 28: Divider X equals one */
#define UART_BAUD_DIV_X_EN         (1 << 29) /* Bit 29: Divider X enable */

/* UART IrDA control register */

#define UART_IRCR_TX_SELECT        (1 << 0)  /* Bit 0:  Invert RX input signal */
#define UART_IRCR_INV_TX           (1 << 1)  /* Bit 1:  Invert TX output signal */
#define UART_IRCR_INV_RX           (1 << 2)  /* Bit 2:  Enable IrDA transmitter */

/* UART alternate control/status register */

#define UART_ALT_CSR_RS485_NMM     (1 << 8)  /* Bit 8:  RS-485 normal multi-drop operation mode (NMM) */
#define UART_ALT_CSR_RS485_AAD     (1 << 9)  /* Bit 9:  RS-485 auto address detection operation mode (AAD) */
#define UART_ALT_CSR_RS485_AUD     (1 << 10) /* Bit 10: RS-485 auto direction mode (AUD) */
#define UART_ALT_CSR_RS485_ADD_EN  (1 << 15) /* Bit 15: RS-485 address detection enable */
#define UART_ALT_CSR_ADDR_MATCH_SHIFT (24)   /* Bits 24-31: Address match value */
#define UART_ALT_CSR_ADDR_MATCH_MASK  (0xff << UART_ALT_CSR_ADDR_MATCH_SHIFT)

/* UART function select register */

#define UART_FUN_SEL_SHIFT         (0)       /* Bits 0-1: Function select enable */
#define UART_FUN_SEL_MASK          (3 << UART_FUN_SEL_SHIFT)
#  define UART_FUN_SEL_UART        (0 << UART_FUN_SEL_SHIFT)
#  define UART_FUN_SEL_IRDA        (2 << UART_FUN_SEL_SHIFT)
#  define UART_FUN_SEL_RS485       (3 << UART_FUN_SEL_SHIFT) /* Low density only */

/********************************************************************************************
 * Public Types
 ********************************************************************************************/

/********************************************************************************************
 * Public Data
 ********************************************************************************************/

/********************************************************************************************
 * Public Functions
 ********************************************************************************************/

#endif /* __ARCH_ARM_SRC_NUC1XX_CHIP_NUC_UART_H */
