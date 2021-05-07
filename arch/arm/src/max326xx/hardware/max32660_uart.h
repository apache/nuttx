/****************************************************************************
 * arch/arm/src/max326xx/hardware/max32660_uart.h
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

#ifndef __ARCH_ARM_SRC_MAX326XX_HARDWARE_MAX32660_UART_H
#define __ARCH_ARM_SRC_MAX326XX_HARDWARE_MAX32660_UART_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/max326_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MAX326_UART_TXFIFO_DEPTH    32      /* 32 bytes */
#define MAX326_UART_RXFIFO_DEPTH    32      /* 32 bytes */

/* Register Offsets *********************************************************/

#define MAX326_UART_CTRL0_OFFSET    0x0000  /* UART Control 0 Register */
#define MAX326_UART_CTRL1_OFFSET    0x0004  /* UART Control 1 Register */
#define MAX326_UART_STAT_OFFSET     0x0008  /* UART Status Register */
#define MAX326_UART_INTEN_OFFSET    0x000c  /* UART Interrupt Enable Register */
#define MAX326_UART_INTFL_OFFSET    0x0010  /* UART Interrupt Flag Register */
#define MAX326_UART_BAUD0_OFFSET    0x0014  /* UART Baud Rate Integer Register */
#define MAX326_UART_BAUD1_OFFSET    0x0018  /* UART Baud Rate Decimal Register */
#define MAX326_UART_FIFO_OFFSET     0x001c  /* UART FIFO Read/Write Register */
#define MAX326_UART_DMA_OFFSET      0x0020  /* UART DMA Configuration Register */
#define MAX326_UART_TXFIFO_OFFSET   0x0024  /* UART TX FIFO Register */

/* Register Addresses *******************************************************/

#define MAX326_UART0_CTRL0          (MAX326_UART0_BASE + MAX326_UART_CTRL0_OFFSET)
#define MAX326_UART0_CTRL1          (MAX326_UART0_BASE + MAX326_UART_CTRL1_OFFSET)
#define MAX326_UART0_STAT           (MAX326_UART0_BASE + MAX326_UART_STAT_OFFSET)
#define MAX326_UART0_INTEN          (MAX326_UART0_BASE + MAX326_UART_INTEN_OFFSET)
#define MAX326_UART0_INTFL          (MAX326_UART0_BASE + MAX326_UART_INTFL_OFFSET)
#define MAX326_UART0_BAUD0          (MAX326_UART0_BASE + MAX326_UART_BAUD0_OFFSET)
#define MAX326_UART0_BAUD1          (MAX326_UART0_BASE + MAX326_UART_BAUD1_OFFSET)
#define MAX326_UART0_FIFO           (MAX326_UART0_BASE + MAX326_UART_FIFO_OFFSET)
#define MAX326_UART0_DMA            (MAX326_UART0_BASE + MAX326_UART_DMA_OFFSET)
#define MAX326_UART0_TXFIFO         (MAX326_UART0_BASE + MAX326_UART_TXFIFO_OFFSET)

#define MAX326_UART1_CTRL0          (MAX326_UART1_BASE + MAX326_UART_CTRL0_OFFSET)
#define MAX326_UART1_CTRL1          (MAX326_UART1_BASE + MAX326_UART_CTRL1_OFFSET)
#define MAX326_UART1_STAT           (MAX326_UART1_BASE + MAX326_UART_STAT_OFFSET)
#define MAX326_UART1_INTEN          (MAX326_UART1_BASE + MAX326_UART_INTEN_OFFSET)
#define MAX326_UART1_INTFL          (MAX326_UART1_BASE + MAX326_UART_INTFL_OFFSET)
#define MAX326_UART1_BAUD0          (MAX326_UART1_BASE + MAX326_UART_BAUD0_OFFSET)
#define MAX326_UART1_BAUD1          (MAX326_UART1_BASE + MAX326_UART_BAUD1_OFFSET)
#define MAX326_UART1_FIFO           (MAX326_UART1_BASE + MAX326_UART_FIFO_OFFSET)
#define MAX326_UART1_DMA            (MAX326_UART1_BASE + MAX326_UART_DMA_OFFSET)
#define MAX326_UART1_TXFIFO         (MAX326_UART1_BASE + MAX326_UART_TXFIFO_OFFSET)

/* Register Bit-field Definitions *******************************************/

/* UART Control 0 Register */

#define UART_CTRL0_ENABLE           (1 << 0)  /* Bit 0:  UART Enable */
#define UART_CTRL0_PARITYEN         (1 << 1)  /* Bit 1:  Parity Enable */
#define UART_CTRL0_PARITYMODE_SHIFT (2)       /* Bits 2-3: Parity Mode Select */
#define UART_CTRL0_PARITYMODE_MASK  (3 << UART_CTRL0_PARITYMODE_SHIFT)
#  define UART_CTRL0_PARITY_EVEN    (0 << UART_CTRL0_PARITYMODE_SHIFT) /* Even parity */
#  define UART_CTRL0_PARITY_ODD     (1 << UART_CTRL0_PARITYMODE_SHIFT) /* Odd Parity */
#  define UART_CTRL0_PARITY_MARK    (2 << UART_CTRL0_PARITYMODE_SHIFT) /* Mark parity */
#  define UART_CTRL0_PARITY_SPACE   (3 << UART_CTRL0_PARITYMODE_SHIFT) /* Space parity */

#define UART_CTRL0_PARITYLVL        (1 << 4)  /* Bit 4:  Parity Level Select */
#define UART_CTRL0_TXFLUSH          (1 << 5)  /* Bit 5:  Transmit FIFO Flush */
#define UART_CTRL0_RXFLUSH          (1 << 6)  /* Bit 6:  Receive FIFO Flush */
#define UART_CTRL0_BITACC           (1 << 7)  /* Bit 7:  Frame or Bit Accuracy Select */
#define UART_CTRL0_SIZE_SHIFT       (8)       /* Bits 8-9: Character Size */
#define UART_CTRL0_SIZE_MASK        (3 << UART_CTRL0_SIZE_SHIFT)
#  define UART_CTRL0_SIZE_5BITS     (0 << UART_CTRL0_SIZE_SHIFT) /* 5 data bits */
#  define UART_CTRL0_SIZE_6BITS     (1 << UART_CTRL0_SIZE_SHIFT) /* 6 data bits */
#  define UART_CTRL0_SIZE_7BITS     (2 << UART_CTRL0_SIZE_SHIFT) /* 7 data bits */
#  define UART_CTRL0_SIZE_8BITS     (3 << UART_CTRL0_SIZE_SHIFT) /* 8 data bits */

#define UART_CTRL0_STOP             (1 << 10) /* Bit 10: STOP Bit Mode Select */
#define UART_CTRL0_FLOW             (1 << 11) /* Bit 11: Hardware Flow Control Enable */
#define UART_CTRL0_FLOWPOL          (1 << 12) /* Bit 12: RTS/CTS Polarity */
#define UART_CTRL0_NULLMOD          (1 << 13) /* Bit 13: Null Modem Support */
#define UART_CTRL0_BREAK            (1 << 14) /* Bit 14: Transmit BREAK Frame */
#define UART_CTRL0_CLKSEL           (1 << 15) /* Bit 15: Bit Rate Clock Source Select */
#define UART_CTRL0_TOCNT_SHIFT      (16)      /* Bits 16-23: RX Timeout Frame Count */
#define UART_CTRL0_TOCNT_MASK       (0xff << UART_CTRL0_TOCNT_SHIFT)
#define UART_CTRL0_TOCNT(n)         (((n) & 0xff) << UART_CTRL0_TOCNT_SHIFT) /* n=1..256 */

/* UART Control 1 Register */

#define UART_CTRL1_RXFIFOLVL_SHIFT  (0)       /* Bits 0-5: RX FIFO Threshold Level */
#define UART_CTRL1_RXFIFOLVL_MASK   (0x3f << UART_CTRL1_RXFIFOLVL_SHIFT)
#  define UART_CTRL1_RXFIFOLVL(n)   ((uint32_t)(n) << UART_CTRL1_RXFIFOLVL_SHIFT)
#define UART_CTRL1_TXFIFOLVL_SHIFT  (8)       /* Bits 8-13: TX FIFO Threshold Level */
#define UART_CTRL1_TXFIFOLVL_MASK   (0x3f << UART_CTRL1_TXFIFOLVL_SHIFT)
#  define UART_CTRL1_TXFIFOLVL(n)   ((uint32_t)(n) << UART_CTRL1_TXFIFOLVL_SHIFT)
#define UART_CTRL1_RTSFIFOLVL_SHIFT (16)      /* Bits 16-21: RTS RX FIFO Threshold Level */
#define UART_CTRL1_RTSFIFOLVL_MASK  (0x3f << UART_CTRL1_RTSFIFOLVL_SHIFT)
#  define UART_CTRL1_RTSFIFOLVL(n)  ((uint32_t)(n) << UART_CTRL1_RTSFIFOLVL_SHIFT)

/* UART Status Register */

#define UART_STAT_TXBUSY            (1 << 0)  /* Bit 0:  TX Busy */
#define UART_STAT_RXBUSY            (1 << 1)  /* Bit 1:  RX Busy */
#define UART_STAT_PARITY            (1 << 2)  /* Bit 2:  Parity Bit State */
#define UART_STAT_BREAK             (1 << 3)  /* Bit 3:  Break Flag */
#define UART_STAT_RXEMPTY           (1 << 4)  /* Bit 4:  RX FIFO Empty Flag */
#define UART_STAT_RXFULL            (1 << 5)  /* Bit 5:  RX FIFO Full Flag */
#define UART_STAT_TXEMPTY           (1 << 6)  /* Bit 6:  TX FIFO Empty Flag */
#define UART_STAT_TXFULL            (1 << 7)  /* Bit 7:  TX FIFO Full Status Flag */
#define UART_STAT_RXNUM_SHIFT       (8)       /* Bits 8-13: Number of Bytes in the RX FIFO */
#define UART_STAT_RXNUM_MASK        (0x3f << UART_STAT_RXNUM_SHIFT)
#define UART_STAT_TXNUM_SHIFT       (16)      /* Bits 16-21: Number of Bytes in the TX FIFO */
#define UART_STAT_TXNUM_MASK        (0x3f << UART_STAT_TXNUM_SHIFT)
#define UART_STAT_RXTO              (1 << 24) /* Bit 24: RX Timeout */

/* UART Interrupt Enable Register and UART Interrupt Flag Register */

#define UART_INT_FRAME              (1 << 0)  /* Bit 0:  RX Frame Error Interrupt */
#define UART_INT_PARITY             (1 << 1)  /* Bit 1:  RX Parity Error Interrupt */
#define UART_INT_CTS                (1 << 2)  /* Bit 2:  CTS State Change Interrupt */
#define UART_INT_RXOVR              (1 << 3)  /* Bit 3:  RX FIFO Overrun Interrupt */
#define UART_INT_RXFIFOLVL          (1 << 4)  /* Bit 4:  RX FIFO Threshold Level Interrupt */
#define UART_INT_TXFIFOAE           (1 << 5)  /* Bit 5:  TX FIFO Almost Empty Interrupt */
#define UART_INT_TXFIFOLVL          (1 << 6)  /* Bit 6:  TX FIFO Threshold Level Interrupt */
#define UART_INT_BREAK              (1 << 7)  /* Bit 7:  Received BREAK Interrupt */
#define UART_INT_RXTO               (1 << 8)  /* Bit 8:  RX Timeout Interrupt */
#define UART_INT_LASTBREAK          (1 << 9)  /* Bit 9:  Last Break Interrupt */

/* UART Baud Rate Integer Register */

#define UART_BAUD0_IBAUD_SHIFT      (0)       /* Bits 0-11: Integer Portion of Baud Rate Divisor */
#define UART_BAUD0_IBAUD_MASK       (0xfff << UART_BAUD0_IBAUD_SHIFT)
#  define UART_BAUD0_IBAUD(n)       ((uint32_t)(n) << UART_BAUD0_IBAUD_SHIFT)
#define UART_BAUD0_CLKDIV_SHIFT     (16)      /* Bits 16-18:  Bit Rate Clock Divisor */
#define UART_BAUD0_CLKDIV_MASK      (7 << UART_BAUD0_CLKDIV_SHIFT)
#  define UART_BAUD0_CLKDIV(n)      ((uint32_t)(n) << UART_BAUD0_CLKDIV_SHIFT)
#  define UART_BAUD0_CLKDIV_128     (0 << UART_BAUD0_CLKDIV_SHIFT)
#  define UART_BAUD0_CLKDIV_64      (1 << UART_BAUD0_CLKDIV_SHIFT)
#  define UART_BAUD0_CLKDIV_32      (2 << UART_BAUD0_CLKDIV_SHIFT)
#  define UART_BAUD0_CLKDIV_16      (3 << UART_BAUD0_CLKDIV_SHIFT)
#  define UART_BAUD0_CLKDIV_8       (4 << UART_BAUD0_CLKDIV_SHIFT)

/* UART Baud Rate Decimal Register */

#define UART_BAUD1_MASK             (0xfff)   /* Bits 0-11: Decimal Portion of Baud Rate Divisor */

/* UART FIFO Read/Write Register */

#define UART_FIFO_MASK              (0xff)    /* Bits 0-7: UART FIFO Register */

/* UART DMA Configuration Register */

#define UART_DMA_TXDMAEN            (1 << 0)  /* Bit 0:  TX FIFO DMA Channel Enable */
#define UART_DMA_RXDMAEN            (1 << 1)  /* Bit 1:  RX FIFO DMA Channel Enable */
#define UART_DMA_TXDMALVL_SHIFT     (8)       /* Bits 8-13: TX FIFO Level DMA Trigger */
#define UART_DMA_TXDMALVL_MASK      (0x3f << UART_DMA_TXDMALVL_SHIFT)
#  define UART_DMA_TXDMALVL(n)      ((uint32_t)(n) << UART_DMA_TXDMALVL_SHIFT)
#define UART_DMA_RXDMALVL_SHIFT     (16)      /* Bits 16-21: RX FIFO Level DMA Trigger */
#define UART_DMA_RXDMALVL_MASK      (0x3f << UART_DMA_RXDMALVL_SHIFT)
#  define UART_DMA_RXDMALVL(n)      ((uint32_t)(n) << UART_DMA_RXDMALVL_SHIFT)

/* UART TX FIFO Register */

#define UART_TXFIFO_MASK            (0xff)    /* Bits 0-7: TX FIFO Data Output Peek Register */

#endif /* __ARCH_ARM_SRC_MAX326XX_HARDWARE_MAX32660_UART_H */
