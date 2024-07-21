/****************************************************************************
 * arch/arm64/src/bcm2711/hardware/bcm2711_aux.h
 *
 * Author: Matteo Golin <matteo.golin@gmail.com>
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

#ifndef __ARCH_ARM64_SRC_BCM2711_AUX_H
#define __ARCH_ARM64_SRC_BCM2711_AUX_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "bcm2711_memmap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* BCM2711 auxiliary register offsets. **************************************/

#define BCM_AUX_IRQ_OFFSET 0x00     /* Auxiliary interrupt status */
#define BCM_AUX_ENABLES_OFFSET 0x04 /* Auxiliary enables */

/* BCM2711 mini UART register offsets. */

#define BCM_AUX_MU_IO_REG_OFFSET 0x40   /* Mini UART I/O Data */
#define BCM_AUX_MU_IER_REG_OFFSET 0x44  /* Mini UART interrupt enable */
#define BCM_AUX_MU_IIR_REG_OFFSET 0x48  /* Mini UART interrupt identify */
#define BCM_AUX_MU_LCR_REG_OFFSET 0x4c  /* Mini UART line control */
#define BCM_AUX_MU_MCR_REG_OFFSET 0x50  /* Mini UART modem control */
#define BCM_AUX_MU_LSR_REG_OFFSET 0x54  /* Mini UART Line Status */
#define BCM_AUX_MU_MSR_REG_OFFSET 0x58  /* Mini UART Modem Status */
#define BCM_AUX_MU_SCRATCH_OFFSET 0x5c  /* Mini UART Scratch */
#define BCM_AUX_MU_CNTL_REG_OFFSET 0x60 /* Mini UART Extra Control */
#define BCM_AUX_MU_STAT_REG_OFFSET 0x64 /* Mini UART Extra Status */
#define BCM_AUX_MU_BAUD_REG_OFFSET 0x68 /* Mini UART Baudrate */

/* BCM2711 SPI registers. */

#define BCM_AUX_SPI1_OFFSET (0x80)
#define BCM_AUX_SPI1_BASEADDR (BCM_AUX_BASEADDR + BCM_AUX_SPI1_OFFSET)

#define BCM_AUX_SPI2_OFFSET (0xc0)
#define BCM_AUX_SPI2_BASEADDR (BCM_AUX_BASEADDR + BCM_AUX_SPI2_OFFSET)

/* BCM2711 SPI register offsets (offset from SPI base register) */

#define BCM_AUX_SPI_CNTL0_REG_OFFSET 0x00   /* SPI Control register 0 */
#define BCM_AUX_SPI_CNTL1_REG_OFFSET 0x04   /* SPI Control register 1 */
#define BCM_AUX_SPI_STAT_REG_OFFSET 0x08    /* SPI Status */
#define BCM_AUX_SPI_PEEK_REG_OFFSET 0x0c    /* SPI Peek */
#define BCM_AUX_SPI_IO_REGa_OFFSET 0x20     /* SPI Data */
#define BCM_AUX_SPI_IO_REGb_OFFSET 0x24     /* SPI Data */
#define BCM_AUX_SPI_IO_REGc_OFFSET 0x28     /* SPI Data */
#define BCM_AUX_SPI_IO_REGd_OFFSET 0x2c     /* SPI Data */
#define BCM_AUX_SPI_TXHOLD_REGa_OFFSET 0x30 /* SPI Extended Data */
#define BCM_AUX_SPI_TXHOLD_REGb_OFFSET 0x34 /* SPI Extended Data */
#define BCM_AUX_SPI_TXHOLD_REGc_OFFSET 0x38 /* SPI Extended Data */
#define BCM_AUX_SPI_TXHOLD_REGd_OFFSET 0x3c /* SPI Extended Data */

/* BCM2711 auxiliary registers. *********************************************/

#define BCM_AUX_REG(offset) (BCM_AUX_BASEADDR + (offset))

/* BCM2711 mini UART registers. */

#define BCM_AUX_IRQ BCM_AUX_REG(BCM_AUX_IRQ_OFFSET)
#define BCM_AUX_ENABLES BCM_AUX_REG(BCM_AUX_ENABLES_OFFSET)
#define BCM_AUX_MU_IO_REG BCM_AUX_REG(BCM_AUX_MU_IO_REG_OFFSET)
#define BCM_AUX_MU_IER_REG BCM_AUX_REG(BCM_AUX_MU_IER_REG_OFFSET)
#define BCM_AUX_MU_IIR_REG BCM_AUX_REG(BCM_AUX_MU_IIR_REG_OFFSET)
#define BCM_AUX_MU_LCR_REG BCM_AUX_REG(BCM_AUX_MU_LCR_REG_OFFSET)
#define BCM_AUX_MU_MCR_REG BCM_AUX_REG(BCM_AUX_MU_MCR_REG_OFFSET)
#define BCM_AUX_MU_LSR_REG BCM_AUX_REG(BCM_AUX_MU_LSR_REG_OFFSET)
#define BCM_AUX_MU_MSR_REG BCM_AUX_REG(BCM_AUX_MU_MSR_REG_OFFSET)
#define BCM_AUX_MU_SCRATCH BCM_AUX_REG(BCM_AUX_MU_SCRATCH_OFFSET)
#define BCM_AUX_MU_CNTL_REG BCM_AUX_REG(BCM_AUX_MU_CNTL_REG_OFFSET)
#define BCM_AUX_MU_STAT_REG BCM_AUX_REG(BCM_AUX_MU_STAT_REG_OFFSET)
#define BCM_AUX_MU_BAUD_REG BCM_AUX_REG(BCM_AUX_MU_BAUD_REG_OFFSET)

/* BCM2711 SPI registers. */

#define BCM_SPI_CNTL0_REG(base) ((base) + BCM_AUX_SPI_CNTL0_REG_OFFSET)
#define BCM_SPI_CNTL1_REG(base) ((base) + BCM_AUX_SPI_CNTL1_REG_OFFSET)
#define BCM_SPI_STAT_REG(base) ((base) + BCM_AUX_SPI_STAT_REG_OFFSET)
#define BCM_SPI_PEEK_REG(base) ((base) + BCM_AUX_SPI_PEEK_REG_OFFSET)
#define BCM_SPI_IO_REGa(base) ((base) + BCM_AUX_SPI_IO_REGa_OFFSET)
#define BCM_SPI_IO_REGb(base) ((base) + BCM_AUX_SPI_IO_REGb_OFFSET)
#define BCM_SPI_IO_REGc(base) ((base) + BCM_AUX_SPI_IO_REGc_OFFSET)
#define BCM_SPI_IO_REGd(base) ((base) + BCM_AUX_SPI_IO_REGd_OFFSET)
#define BCM_SPI_TXHOLD_REGa(base) ((base) + BCM_AUX_SPI_TXHOLD_REGa_OFFSET)
#define BCM_SPI_TXHOLD_REGb(base) ((base) + BCM_AUX_SPI_TXHOLD_REGb_OFFSET)
#define BCM_SPI_TXHOLD_REGc(base) ((base) + BCM_AUX_SPI_TXHOLD_REGc_OFFSET)
#define BCM_SPI_TXHOLD_REGd(base) ((base) + BCM_AUX_SPI_TXHOLD_REGd_OFFSET)

/* BCM2711 auxiliary register bit definitions. */

#define BCM_AUX_IRQ_MU (1 << 0)   /* Mini UART interrupt pending */
#define BCM_AUX_IRQ_SPI1 (1 << 1) /* SPI1 interrupt pending */
#define BCM_AUX_IRQ_SPI2 (1 << 2) /* SPI2 interrupt pending */

#define BCM_AUX_ENABLE_MU (1 << 0)   /* Mini UART enable */
#define BCM_AUX_ENABLE_SPI1 (1 << 1) /* SPI1 enable */
#define BCM_AUX_ENABLE_SPI2 (1 << 2) /* SPI2 enable */

#define BCM_AUX_MU_IO_BAUDRATE (0xff) /* LS 8 bits of baudrate register */
#define BCM_AUX_MU_IO_TXD (0xff)      /* If DLAB=0, write-only */
#define BCM_AUX_MU_IO_RXD (0xff)      /* If DLAB=0, read-only */

#define BCM_AUX_MU_IER_BAUDRATE (0xff) /* MS 8 bits of baudrate register */

/* NOTE: The RXD and TXD interrupts here are swapped when compared to what is
 * visible on the BCM2711 datasheet. This is because the data sheet contains
 * an error.
 */

#define BCM_AUX_MU_IER_RXD (1 << 0) /* Enable receive interrupt */
#define BCM_AUX_MU_IER_TXD (1 << 1) /* Enable transmit interrupt */

#define BCM_AUX_MU_IIR_PENDING (1 << 0) /* Clear when interrupt pending */
#define BCM_AUX_MU_IIR_MASK (0x03 << 1) /* Mask interrupt ID bits */
#define BCM_AUX_MU_IIR_TXEMPTY (1 << 1) /* TX holding register empty (RO) */
#define BCM_AUX_MU_IIR_RXBYTE (2 << 1)  /* RX holding valid byte (RO) */
#define BCM_AUX_MU_IIR_NONE (0 << 1)    /* No interrupts (RO) */
#define BCM_AUX_MU_IIR_RXCLEAR (1 << 1) /* Clear RX FIFO (WO) */
#define BCM_AUX_MU_IIR_TXCLEAR (1 << 2) /* Clear TX FIFO (WO) */

#define BCM_AUX_MU_LCR_DLAB (1 << 7)   /* Gives access to baudrate */
#define BCM_AUX_MU_LCR_BREAK (1 << 6)  /* Pull TX line low */
#define BCM_AUX_MU_LCR_DATA8B (1 << 0) /* 7-bit if clear, 8-bit if set */

#define BCM_AUX_MU_MCR_RTS (1 << 1) /* RTS high = 0, RTS low = 1 */

#define BCM_AUX_MU_LSR_TXIDLE (1 << 6)    /* Transmitter is idle */
#define BCM_AUX_MU_LSR_TXEMPTY (1 << 5)   /* Transmitter FIFO has space */
#define BCM_AUX_MU_LSR_RXOVERRUN (1 << 1) /* Receiver overrun */
#define BCM_AUX_MU_LSR_DREADY (1 << 0)    /* RX data is ready */

#define BCM_AUX_MU_MSR_CTS (1 << 4) /* CTS low = 1, CTS high = 0 */

#define BCM_AUX_MU_SCRATCHMASK (0xff) /* Byte of extra storage */

#define BCM_AUX_MU_CNTL_CTSLVL (1 << 7)       /* CTS flow assert low = 1 */
#define BCM_AUX_MU_CNTL_RTSLVL (1 << 6)       /* RTS flow assert low = 1 */
#define BCM_AUX_MU_CNTL_RTSFLOWMSK (0x3 << 4) /* RTS auto flow level mask */
#define BCM_AUX_MU_CNTL_FLOWLVL4 (3 << 4)     /* De-assert FIFO 4 empty */
#define BCM_AUX_MU_CNTL_FLOWLVL3 (0 << 4)     /* De-assert FIFO 3 empty */
#define BCM_AUX_MU_CNTL_FLOWLVL2 (1 << 4)     /* De-assert FIFO 2 empty */
#define BCM_AUX_MU_CNTL_FLOWLVL1 (2 << 4)     /* De-assert FIFO 1 empty */
#define BCM_AUX_MU_CNTL_TXAUTOFLOW (1 << 3)   /* Enable TX auto flow */
#define BCM_AUX_MU_CNTL_RXAUTOFLOW (1 << 2)   /* Enable RX auto flow */
#define BCM_AUX_MU_CNTL_TXENABLE (1 << 1)     /* Enable transmitter */
#define BCM_AUX_MU_CNTL_RXENABLE (1 << 0)     /* Enable receiver */

#define BCM_AUX_MU_STAT_TXFIFOFILL (0xf << 24) /* How many symbols 0-8 */
#define BCM_AUX_MU_STAT_RXFIFOFILL (0xf << 16) /* How many symbols 0-8 */
#define BCM_AUX_MU_STAT_TXDONE (1 << 9)        /* TX idle and FIFO empty */
#define BCM_AUX_MU_STAT_TXEMPTY (1 << 8)       /* TX FIFO is empty */
#define BCM_AUX_MU_STAT_CTSLINE (1 << 7)       /* Status of CTS line */
#define BCM_AUX_MU_STAT_RTSLINE (1 << 6)       /* Status of RTS line */
#define BCM_AUX_MU_STAT_TXFULL (1 << 5)        /* TX FIFO full */
#define BCM_AUX_MU_STAT_RXOVERRUN (1 << 4)     /* RX overrun */
#define BCM_AUX_MU_STAT_TXIDLE (1 << 3)        /* TX idle */
#define BCM_AUX_MU_STAT_RXIDLE (1 << 2)        /* RX idle */
#define BCM_AUX_MU_STAT_SPACEAVAIL (1 << 1)    /* TX has space */
#define BCM_AUX_MU_STAT_SYMAVAIL (1 << 0)      /* RX has symbol */

#define BCM_AUX_MU_BAUD_MASK (0xffff) /* Baudrate counter */

#define BCM_SPI_CNTL0_SPEED (0xfff << 20)  /* SPI clock speed */
#define BCM_SPI_CNTL0_CS (0x7 << 17)       /* SPI clock speed */
#define BCM_SPI_CNTL0_PIMODE (1 << 16)     /* Post input mode */
#define BCM_SPI_CNTL0_VARCS (1 << 15)      /* Variable chip select */
#define BCM_SPI_CNTL0_VARWIDTH (1 << 14)   /* Variable width */
#define BCM_SPI_CNTL0_DOUTHOLD (0x3 << 12) /* DOUT hold time mask */
#define BCM_SPI_CNTL0_DOUTNONE (0 << 12)   /* No hold */
#define BCM_SPI_CNTL0_DOUT1 (1 << 12)      /* 1 system clock hold */
#define BCM_SPI_CNTL0_DOUT4 (2 << 12)      /* 4 system clocks hold */
#define BCM_SPI_CNTL0_DOUT7 (3 << 12)      /* 7 system clocks hold */
#define BCM_SPI_CNTL0_EN (1 << 11)         /* Enable SPI interface */
#define BCM_SPI_CNTL0_INRISE (1 << 10)     /* Data clock on rising */
#define BCM_SPI_CNTL0_FIFOCLR (1 << 9)     /* Clear FIFOs */
#define BCM_SPI_CNTL0_OUTRISE (1 << 8)     /* Data clock on rising */
#define BCM_SPI_CNTL0_CLKINV (1 << 7)      /* Invert SPI clock */
#define BCM_SPI_CNTL0_SHIFTMS (1 << 6)     /* Shift out MS bit */
#define BCM_SPI_CNTL0_SHIFTLEN (0x2f)      /* Bit shift count mask */

#define BCM_SPI_CNTL1_CSHTIME (0x7 << 8)  /* CS high time */
#define BCM_SPI_CNTL1_TXEMPTYIRQ (1 << 7) /* Int line high = 1 */
#define BCM_SPI_CNTL1_DONEIRQ (1 << 6)    /* Interrupt while idle = 1 */
#define BCM_SPI_CNTL1_SHIFTMS (1 << 1)    /* Shift in MS bit first */
#define BCM_SPI_CNTL1_KEEPIN (1 << 0)     /* Do not clear RX shift reg */

#define BCM_SPI_STAT_TXLVL (0xf << 24) /* TX FIFO level mask */
#define BCM_SPI_STAT_RXLVL (0xf << 16) /* RX FIFO level mask */
#define BCM_SPI_STAT_TXFULL (1 << 10)  /* TX FIFO full */
#define BCM_SPI_STAT_TXEMPTY (1 << 9)  /* TX FIFO empty */
#define BCM_SPI_STAT_RXFULL (1 << 8)   /* RX FIFO empty */
#define BCM_SPI_STAT_RXEMPTY (1 << 7)  /* RX FIFO empty */
#define BCM_SPI_STAT_BUSY (1 << 6)     /* Module is busy */
#define BCM_SPI_STAT_BITCOUNT (0x3f)   /* Bits to be processed */

#define BMX_SPI_PEEK_DATA (0xffff) /* Data mask */

#define BMX_SPI_IO_DATA (0xffff) /* Data mask */

#define BMX_SPI_TXHOLD_DATA (0xffff) /* Data mask */

#endif /* __ARCH_ARM64_SRC_BCM2711_AUX_H */
