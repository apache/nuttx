/************************************************************************************
 * arch/arm/src/bcm2708/chip/bcm2708_aux.h
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Author: Alan Carvalho de Assis <acassis@gmail.com>
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
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_BCM2708_CHIP_BCM2708_AUX_H
#define __ARCH_ARM_SRC_BCM2708_CHIP_BCM2708_AUX_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "chip/bcm2708_memorymap.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* UART/AUX Register Offsets ********************************************************/

#define BCM_AUX_IRQ_OFFSET              0x0000 /* Auxiliary Interrupt status */
#define BCM_AUX_ENB_OFFSET              0x0004 /* Auxiliary enables */
#define BCM_AUX_MU_IO_OFFSET            0x0040 /* Mini Uart I/O Data */
#define BCM_AUX_MU_IER_OFFSET           0x0044 /* Mini Uart Interrupt Enable */
#define BCM_AUX_MU_IIR_OFFSET           0x0048 /* Mini Uart Interrupt Identify */
#define BCM_AUX_MU_LCR_OFFSET           0x004c /* Mini Uart Line Control */
#define BCM_AUX_MU_MCR_OFFSET           0x0050 /* Mini Uart Modem Control */
#define BCM_AUX_MU_LSR_OFFSET           0x0054 /* Mini Uart Line Status */
#define BCM_AUX_MU_MSR_OFFSET           0x0058 /* Mini Uart Modem Status */
#define BCM_AUX_MU_SCRATCH_OFFSET       0x005c /* Mini Uart Scratch */
#define BCM_AUX_MU_CNTL_OFFSET          0x0060 /* Mini Uart Extra Control */
#define BCM_AUX_MU_STAT_OFFSET          0x0064 /* Mini Uart Extra Status */
#define BCM_AUX_MU_BAUD_OFFSET          0x0068 /* Mini Uart Baudrate */
#define BCM_AUX_SPI0_CNTL0_OFFSET       0x0080 /* SPI 1 Control register 0 */
#define BCM_AUX_SPI0_CNTL1_OFFSET       0x0084 /* SPI 1 Control register 1 */
#define BCM_AUX_SPI0_STAT_OFFSET        0x0088 /* SPI 1 Status */
#define BCM_AUX_SPI0_IO_OFFSET          0x0090 /* SPI 1 Data */
#define BCM_AUX_SPI0_PEEK_OFFSET        0x0094 /* SPI 1 Peek */
#define BCM_AUX_SPI1_CNTL0_OFFSET       0x00c0 /* SPI 2 Control register 0 */
#define BCM_AUX_SPI1_CNTL1_OFFSET       0x00c4 /* SPI 2 Control register 1 */
#define BCM_AUX_SPI1_STAT_OFFSET        0x00c8 /* SPI 2 Status */
#define BCM_AUX_SPI1_IO_OFFSET          0x00d0 /* SPI 2 Data */
#define BCM_AUX_SPI1_PEEK_OFFSET        0x00d4 /* SPI 2 Peek */


/* UART Register Addresses **********************************************************/

#define BCM_AUX_IRQ                     (BCM_AUX_VBASE+BCM_AUX_IRQ_OFFSET)
#define BCM_AUX_ENB                     (BCM_AUX_VBASE+BCM_AUX_ENB_OFFSET)
#define BCM_AUX_MU_IO                   (BCM_AUX_VBASE+BCM_AUX_MU_IO_OFFSET)
#define BCM_AUX_MU_IER                  (BCM_AUX_VBASE+BCM_AUX_MU_IER_OFFSET)
#define BCM_AUX_MU_IIR                  (BCM_AUX_VBASE+BCM_AUX_MU_IIR_OFFSET)
#define BCM_AUX_MU_LCR                  (BCM_AUX_VBASE+BCM_AUX_MU_LCR_OFFSET)
#define BCM_AUX_MU_MCR                  (BCM_AUX_VBASE+BCM_AUX_MU_MCR_OFFSET)
#define BCM_AUX_MU_LSR                  (BCM_AUX_VBASE+BCM_AUX_MU_LSR_OFFSET)
#define BCM_AUX_MU_MSR                  (BCM_AUX_VBASE+BCM_AUX_MU_MSR_OFFSET)
#define BCM_AUX_MU_SCRATCH              (BCM_AUX_VBASE+BCM_AUX_MU_SCRATCH_OFFSET)
#define BCM_AUX_MU_CNTL                 (BCM_AUX_VBASE+BCM_AUX_MU_CNTL_OFFSET)
#define BCM_AUX_MU_STAT                 (BCM_AUX_VBASE+BCM_AUX_MU_STAT_OFFSET)
#define BCM_AUX_MU_BAUD                 (BCM_AUX_VBASE+BCM_AUX_MU_BAUD_OFFSET)
#define BCM_AUX_SPI0_CNTL0              (BCM_AUX_VBASE+BCM_AUX_SPI0_CNTL0_OFFSET)
#define BCM_AUX_SPI0_CNTL1              (BCM_AUX_VBASE+BCM_AUX_SPI0_CNTL1_OFFSET)
#define BCM_AUX_SPI0_STAT               (BCM_AUX_VBASE+BCM_AUX_SPI0_STAT_OFFSET)
#define BCM_AUX_SPI0_IO                 (BCM_AUX_VBASE+BCM_AUX_SPI0_IO_OFFSET)
#define BCM_AUX_SPI0_PEEK               (BCM_AUX_VBASE+BCM_AUX_SPI0_PEEK_OFFSET)
#define BCM_AUX_SPI1_CNTL0              (BCM_AUX_VBASE+BCM_AUX_SPI1_CNTL0_OFFSET)
#define BCM_AUX_SPI1_CNTL1              (BCM_AUX_VBASE+BCM_AUX_SPI1_CNTL1_OFFSET)
#define BCM_AUX_SPI1_STAT               (BCM_AUX_VBASE+BCM_AUX_SPI1_STAT_OFFSET)
#define BCM_AUX_SPI1_IO                 (BCM_AUX_VBASE+BCM_AUX_SPI1_IO_OFFSET)
#define BCM_AUX_SPI1_PEEK               (BCM_AUX_VBASE+BCM_AUX_SPI1_PEEK_OFFSET)


/* UART Register Bit Definitions ****************************************************/

#define BCM_AUX_IRQ_MU                  (1 << 0)  /* Mini UART IRQ pending */
#define BCM_AUX_IRQ_SPI1                (1 << 1)  /* SPI 1 IRQ pending */
#define BCM_AUX_IRQ_SPI2                (1 << 2)  /* SPI 2 IRQ pending */

#define BCM_AUX_ENB_MU                  (1 << 0)  /* Mini UART Enable */
#define BCM_AUX_ENB_SPI1                (1 << 1)  /* SPI 1 Enable */
#define BCM_AUX_ENB_SPI2                (1 << 2)  /* SPI 2 Enable */

#define BCM_AUX_MU_IO_SHIFT             0         /* LSB 8-bit of baudrate or TXD/RXD */
#define BCM_AUX_MU_IO_BAUDRATE          (0xff << BCM_AUX_MU_IO_SHIFT)  /* If DLAB = 1 */
#define BCM_AUX_MU_IO_TXD               (0 << BCM_AUX_MU_IO_SHIFT)     /* If DLAB = 0 */
#define BCM_AUX_MU_IO_RXD               (1 << BCM_AUX_MU_IO_SHIFT)     /* If DLAB = 0 */

#define BCM_AUX_MU_IER_SHIFT            0         /* MSB 8-bit of baudrate or IRQTXE/IRQRXE */
#define BCM_AUX_MU_IER_BAUDRATE         (0xff << BCM_AUX_MU_IER_SHIFT) /* If DLAB = 1 */
#define BCM_AUX_MU_IER_TXD              (1 << 0)  /* Enable Transmit Interrupt */
#define BCM_AUX_MU_IER_RXD              (1 << 1)  /* Enable Receive Interrupt */

#define BCM_AUX_MU_IIR_PEND             (1 << 0)  /* This bit is clear whenever an IRQ is pending */
#define BCM_AUX_MU_IIR_SHIFT            1         /* On read this register shows the interrupt ID bit */
#define BCM_AUX_MU_IIR_MASK             (3 << BCM_AUX_MU_IIR_SHIFT)
#  define BCM_AUX_MU_IIR_NONE           (0 << BCM_AUX_MU_IIR_SHIFT) /* No interrupts */
#  define BCM_AUX_MU_IIR_TXEMPTY        (1 << BCM_AUX_MU_IIR_SHIFT) /* TX FIFO empty (read) */
#  define BCM_AUX_MU_IIR_RXDATA         (2 << BCM_AUX_MU_IIR_SHIFT) /* Data in RX FIFO (read) */
#  define BCM_AUX_MU_IIR_TXCLEAR        (1 << BCM_AUX_MU_IIR_SHIFT) /* Clear RX FIFO (write) */
#  define BCM_AUX_MU_IIR_RXCLEAR        (2 << BCM_AUX_MU_IIR_SHIFT) /* Clear TX FIFO (write) */

#define BCM_AUX_MU_LCR_DATA8BIT         (1 << 0)  /* 1 = UART 8-bit 0 = UART 7-bit */
#define BCM_AUX_MU_LCR_BREAK            (1 << 6)  /* Set UART TX line to low, breaks if at least 12 bits times */
#define BCM_AUX_MU_LCR_DLAB             (1 << 7)  /* If set access Baudrate register */

#define BCM_AUX_MU_MCR_RTS              (1 << 1)  /* if set RTS line is low */

#define BCM_AUX_MU_LSR_DTREADY          (1 << 0)  /* This bit is set if FIFO has data */
#define BCM_AUX_MU_LSR_RXOVR            (1 << 1)  /* Receiver Overrun */
#define BCM_AUX_MU_LSR_TXEMPTY          (1 << 5)  /* TX FIFO can accept at least 1 byte */
#define BCM_AUX_MU_LSR_TXIDLE           (1 << 6)  /* TX FIFO empty and transmitter idle */

#define BCM_AUX_MU_MSR_CTS              (1 << 5)  /* Holds the inverse of UART CTS status */

#define BCM_AUX_MU_CNTL_RXEN            (1 << 0)  /* Receiver Enable */
#define BCM_AUX_MU_CNTL_TXEN            (1 << 1)  /* Transmitter Enable */
#define BCM_AUX_MU_CNTL_RXAUTOFLOW      (1 << 2)  /* Enable RXD Auto-flow using RTS */
#define BCM_AUX_MU_CNTL_TXAUTOFLOW      (1 << 3)  /* Enable TXD Auto-flow using CTS */
#define BCM_AUX_MU_CNTL_RTSLEVEL_SHIFT  4         /* RX FIFO level to deassert RTS */
#define BCM_AUX_MU_CNTL_RTSLEVEL_MASK   (3 << BCM_AUX_MU_CNTL_RTSLEVEL_SHIFT)
#  define BCM_AUX_MU_CNTL_RTSLEVEL_3    (0 << BCM_AUX_MU_CNTL_RTSLEVEL_SHIFT)
#  define BCM_AUX_MU_CNTL_RTSLEVEL_2    (1 << BCM_AUX_MU_CNTL_RTSLEVEL_SHIFT)
#  define BCM_AUX_MU_CNTL_RTSLEVEL_1    (2 << BCM_AUX_MU_CNTL_RTSLEVEL_SHIFT)
#  define BCM_AUX_MU_CNTL_RTSLEVEL_4    (3 << BCM_AUX_MU_CNTL_RTSLEVEL_SHIFT)
#define BCM_AUX_MU_CNTL_RTS_ASSERT      (1 << 6)  /* Invert RTS auto-flow polarity */
#define BCM_AUX_MU_CNTL_CTS_ASSERT      (1 << 7)  /* Invert CTS auto-flow polarity */

#define BCM_AUX_MU_STAT_SYM_AVAIL       (1 << 0)  /* There is at least one symbol in the RX FIFO */
#define BCM_AUX_MU_STAT_SPC_AVAIL       (1 << 1)  /* There is at least one free position in the TX FIFO */
#define BCM_AUX_MU_STAT_RX_IDLE         (1 << 2)  /* The RX is Idle */
#define BCM_AUX_MU_STAT_TX_IDLE         (1 << 3)  /* The TX is Idle */
#define BCM_AUX_MU_STAT_RX_OVR          (1 << 4)  /* Receiver overrun */
#define BCM_AUX_MU_STAT_TX_FULL         (1 << 5)  /* This is the inverse of bit 1 */
#define BCM_AUX_MU_STAT_RTS_STAT        (1 << 6)  /* Status of RTS line */
#define BCM_AUX_MU_STAT_CTS_STAT        (1 << 7)  /* Status of CTS line */
#define BCM_AUX_MU_STAT_TX_EMPTY        (1 << 8)  /* TX FIFO is Empty */
#define BCM_AUX_MU_STAT_TX_DONE         (1 << 9)  /* TX is Idle and FIFO is Empty */
#define BCM_AUX_MU_STAT_RX_LEVEL_SHIFT  16        /* Bits 19-16: how many symbols in RX FIFO */
#define BCM_AUX_MU_STAT_RX_LEVEL_MASK   (0xf << BCM_AUX_MU_STAT_RX_LEVEL_SHIFT)
#define BCM_AUX_MU_STAT_TX_LEVEL_SHIFT  24        /* Bits 27-24: how many symbols in TX FIFO */
#define BCM_AUX_MU_STAT_TX_LEVEL_MASK   (0xf << BCM_AUX_MU_STAT_TX_LEVEL_SHIFT)

#define BCM_AUX_SPI_CNTL0_SFT_LEN_SHIFT 0         /* Specifies the number of bits to shift */
#define BCM_AUX_SPI_CNTL0_SFT_LEN_MASK  (0x3f << BCM_AUX_SPI_CNTL0_SFT_LEN_SHIFT)
#define BCM_AUX_SPI_CNTL0_MSB_FIRST     (1 << 6)  /* If 1 the data is shifted out starting with the MS bit */
#define BCM_AUX_SPI_CNTL0_INV_SPICLK    (1 << 7)  /* If 1 the 'idle' clock line state is high */
#define BCM_AUX_SPI_CNTL0_OUT_RISING    (1 << 8)  /* if 1 data is clocked out on the rising edge of the SPI clock */
#define BCM_AUX_SPI_CNTL0_CLR_FIFO      (1 << 9)  /* Clear RX and TX FIFOs */
#define BCM_AUX_SPI_CNTL0_IN_RISING     (1 << 10) /* If 1 data is clocked in on the rising edge of the SPI clock */
#define BCM_AUX_SPI_CNTL0_ENABLE        (1 << 11) /* Enables the SPI interface. */
#define BCM_AUX_SPI_CNTL0_DOHT_SHIFT    12        /* Bit 13-12: DOUT Hold Time */
#define BCM_AUX_SPI_CNTL0_DOHT_MASK     (3 << BCM_AUX_SPI_CNTL0_DOHT_SHIFT)
#  define BCM_AUX_SPI_CNTL0_DOHT_NONE   (0 << BCM_AUX_SPI_CNTL0_DOHT_SHIFT)
#  define BCM_AUX_SPI_CNTL0_DOHT_1CLK   (1 << BCM_AUX_SPI_CNTL0_DOHT_SHIFT)
#  define BCM_AUX_SPI_CNTL0_DOHT_4CLK   (2 << BCM_AUX_SPI_CNTL0_DOHT_SHIFT)
#  define BCM_AUX_SPI_CNTL0_DOHT_7CLK   (3 << BCM_AUX_SPI_CNTL0_DOHT_SHIFT)
#define BCM_AUX_SPI_CNTL0_VAR_WIDTH     (1 << 14) /* Variable Width based on TX FIFO */
#define BCM_AUX_SPI_CNTL0_VAR_CS        (1 << 15) /* CS pattern and data from TX FIFO */
#define BCM_AUX_SPI_CNTL0_PIM           (1 << 16) /* Post Input Mode */
#define BCM_AUX_SPI_CNTL0_CS_SHIFT      17        /* The pattern output on the CS pins when active */
#define BCM_AUX_SPI_CNTL0_CS_MASK       (7 << BCM_AUX_SPI_CNTL0_CS_SHIFT)
#define BCM_AUX_SPI_CNTL0_SPEED_SHIFT   20
#define BCM_AUX_SPI_CNTL0_SPEED_MASK    (0xfff << BCM_AUX_SPI_CNTL0_SPEED_SHIFT)

#define BCM_AUX_SPI_CNTL1_KEEP_IN       (1 << 0)  /* Keep input, shift register is not cleared */
#define BCM_AUX_SPI_CNTL1_MSB_FIRST     (1 << 1)  /* Shift data from MSB */
#define BCM_AUX_SPI_CNTL1_DONE_IRQ      (1 << 6)  /* If 1 the interrupt line is high when the interface is idle */
#define BCM_AUX_SPI_CNTL1_TXEMPTY_IRQ   (1 << 7)  /* Enable IRQ Line when FIFO is Empty */
#define BCM_AUX_SPI_CNTL1_CSHT_SHIFT    8         /* Bits 10-8: Additional CS High Time */
#define BCM_AUX_SPI_CNTL1_CSHT_MASK     (7 << BCM_AUX_SPI_CNTL1_CSHT_SHIFT)

#define BCM_AUX_SPI_STAT_BITCNT_SHIFT   0         /* Bits 5-0: The number of bits still to be processed */
#define BCM_AUX_SPI_STAT_BITCNT_MASK    (0x3f << BCM_AUX_SPI_STAT_BITCNT_SHIFT)
#define BCM_AUX_SPI_STAT_BUSY           (1 << 6)  /* The module is busy transferring data */
#define BCM_AUX_SPI_STAT_RXEMPTY        (1 << 7)  /* RX FIFO is empty */
#define BCM_AUX_SPI_STAT_TXEMPTY        (1 << 8)  /* TX FIFO is empty */
#define BCM_AUX_SPI_STAT_TXFULL         (1 << 9)  /* TX FIFO is full */
#define BCM_AUX_SPI_STAT_RXLEVEL_SHIFT  12        /* Data units in the RX FIFO */
#define BCM_AUX_SPI_STAT_RXLEVEL_MASK   (0xfff << BCM_AUX_SPI_STAT_RXLEVEL_SHIFT)
#define BCM_AUX_SPI_STAT_TXLEVEL_SHIFT  24        /* Data units in the FX FIFO */
#define BCM_AUX_SPI_STAT_TXLEVEL_MASK   (0xff << BCM_AUX_SPI_STAT_TXLEVEL_SHIFT)

#endif /* __ARCH_ARM_SRC_BCM2708_CHIP_BCM2708_AUX_H */
