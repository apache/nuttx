/****************************************************************************
 * arch/arm/src/max326xx/hardware/max32660_spi.h
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

#ifndef __ARCH_ARM_SRC_MAX326XX_HARDWARE_MAX32660_SPI_H
#define __ARCH_ARM_SRC_MAX326XX_HARDWARE_MAX32660_SPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/max326_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MAX326_SPI_FIFO_DEPTH       32

/* Register Offsets *********************************************************/

#define MAX326_SPI_DATA_OFFSET      0x0000  /* SPI FIFO Data Register */
#define MAX326_SPI_CTRL0_OFFSET     0x0004  /* SPI Master Signals Control Register */
#define MAX326_SPI_CTRL1_OFFSET     0x0008  /* SPI Transmit Packet Size Register */
#define MAX326_SPI_CTRL2_OFFSET     0x000c  /* SPI Static Configuration Register */
#define MAX326_SPI_SSTIME_OFFSET    0x0010  /* SPI Slave Select Timing Register */
#define MAX326_SPI_CLKCFG_OFFSET    0x0014  /* SPI Master Clock Configuration Register */
#define MAX326_SPI_DMA_OFFSET       0x001c  /* SPI DMA Control Register */
#define MAX326_SPI_INTFL_OFFSET     0x0020  /* SPI Interrupt Status Flags Register */
#define MAX326_SPI_INTEN_OFFSET     0x0024  /* SPI Interrupt Enable Register */
#define MAX326_SPI_WAKEFL_OFFSET    0x0028  /* SPI Wakeup Status Flags Register */
#define MAX326_SPI_WAKEEN_OFFSET    0x002c  /* SPI Wakeup Enable Register */
#define MAX326_SPI_STAT_OFFSET      0x0030  /* SPI Active Status Register */

/* Register Addresses *******************************************************/

#define MAX326_SPI0_DATA            (MAX326_SPI0_BASE + MAX326_SPI_DATA_OFFSET)
#define MAX326_SPI0_CTRL0           (MAX326_SPI0_BASE + MAX326_SPI_CTRL0_OFFSET)
#define MAX326_SPI0_CTRL1           (MAX326_SPI0_BASE + MAX326_SPI_CTRL1_OFFSET)
#define MAX326_SPI0_CTRL2           (MAX326_SPI0_BASE + MAX326_SPI_CTRL2_OFFSET)
#define MAX326_SPI0_SSTIME          (MAX326_SPI0_BASE + MAX326_SPI_SSTIME_OFFSET)
#define MAX326_SPI0_CLKCFG          (MAX326_SPI0_BASE + MAX326_SPI_CLKCFG_OFFSET)
#define MAX326_SPI0_DMA             (MAX326_SPI0_BASE + MAX326_SPI_DMA_OFFSET)
#define MAX326_SPI0_INTFL           (MAX326_SPI0_BASE + MAX326_SPI_INTFL_OFFSET)
#define MAX326_SPI0_INTEN           (MAX326_SPI0_BASE + MAX326_SPI_INTEN_OFFSET)
#define MAX326_SPI0_WAKEFL          (MAX326_SPI0_BASE + MAX326_SPI_WAKEFL_OFFSET)
#define MAX326_SPI0_WAKEEN          (MAX326_SPI0_BASE + MAX326_SPI_WAKEEN_OFFSET)
#define MAX326_SPI0_STAT            (MAX326_SPI0_BASE + MAX326_SPI_STAT_OFFSET)

/* Register Bit-field Definitions *******************************************/

/* SPI FIFO Data Register (SPI data up to 32-bits wide) */

/* SPI Master Signals Control Register */

#define SPI_CTRL0_SPIEN             (1 << 0 ) /* Bit 0:  SPI Enable/Disable */
#define SPI_CTRL0_MMEN              (1 << 1)  /* Bit 1:  SPI Master Mode Enable */
#define SPI_CTRL0_SSIO              (1 << 4)  /* Bit 4:  Slave Select Output (master) */
#define SPI_CTRL0_START             (1 << 5)  /* Bit 5:  Start Data Transmission (master) */
#define SPI_CTRL0_SSCTRL            (1 << 8)  /* Bit 8:  Slave Select Control (master) */

#define SPI_CTRL0_SSSEL(n)          (1 << ((n) + 16)) /* Bits 16-19: Slave Select n
                                                       *             Enable, n=0 */

/* SPI Transmit Packet Size Register */

#define SPI_CTRL1_TXNUMCH_SHIFT     (0)       /* Bits 0-15: Number of Transmit Characters */
#define SPI_CTRL1_TXNUMCH_MASK      (0xffff << SPI_CTRL1_TXNUMCH_SHIFT)
#  define SPI_CTRL1_TXNUMCH(n)      ((uint32_t)(n) << SPI_CTRL1_TXNUMCH_SHIFT)
#define SPI_CTRL1_RXNUMCH_SHIFT     (16)      /* Bits 16-31: Number of Receive Characters */
#define SPI_CTRL1_RXNUMCH_MASK      (0xffff << SPI_CTRL1_RXNUMCH_SHIFT)
#  define SPI_CTRL1_RXNUMCH(n)      ((uint32_t)(n) << SPI_CTRL1_RXNUMCH_SHIFT)

/* SPI Static Configuration Register */

#define SPI_CTRL2_CLKPHA            (1 << 0)  /* Bit 0:  Clock Phase */
#define SPI_CTRL2_CLKPOL            (1 << 1)  /* Bit 1:  Clock Polarity */
#define SPI_CTRL2_NUMBITS_SHIFT     (8)       /* Bits 8-11: Number of Bits per Character */
#define SPI_CTRL2_NUMBITS_MASK      (15 << SPI_CTRL2_NUMBITS_SHIFT)
#  define SPI_CTRL2_NUMBITS(n)      ((uint32_t)((n) & 15) << SPI_CTRL2_NUMBITS_SHIFT)
#define SPI_CTRL2_DATWIDTH_SHIFT    (12)      /* Bits 12-13: SPI Data Width */
#define SPI_CTRL2_DATWIDTH_MASK     (3 << SPI_CTRL2_DATWIDTH_SHIFT)
#  define SPI_CTRL2_DATWIDTH_SINGLE (0 << SPI_CTRL2_DATWIDTH_SHIFT) /* MOSI */
#  define SPI_CTRL2_DATWIDTH_DUAL   (1 << SPI_CTRL2_DATWIDTH_SHIFT) /* MOSI/MISO */

#define SPI_CTRL2_3WIRE             (1 << 15) /* Bit 15: Three-Wire Mode Enable */
#define SPI_CTRL2_SSPOL             (1 << 16) /* Bit 16: Slave Select Polarity */

/* SPI Slave Select Timing Register */

#define SPI_SSTIME_SSACT1_SHIFT     (0)       /* Bits 0-7: Slave Select Active to First
                                               *            SCLK */
#define SPI_SSTIME_SSACT1_MASK      (0xff << SPI_SSTIME_SSACT1_SHIFT)
#  define SPI_SSTIME_SSACT1(n)      ((uint32_t)((n) & 0xff) << SPI_SSTIME_SSACT1_SHIFT)
#define SPI_SSTIME_SSACT2_SHIFT     (8)       /* Bits 8-15: Slave Select Active After Last
                                               *            SCLK */
#define SPI_SSTIME_SSACT2_MASK      (0xff << SPI_SSTIME_SSACT2_SHIFT)
#  define SPI_SSTIME_SSACT2(n)      ((uint32_t)((n) & 0xff) << SPI_SSTIME_SSACT2_SHIFT)
#define SPI_SSTIME_SSINACT_SHIFT    (16)      /* Bits 16-23: SS Inactive Clock Delay */
#define SPI_SSTIME_SSINACT_MASK     (0xff << SPI_SSTIME_SSINACT_SHIFT)
#  define SPI_SSTIME_SSINACT(n)     ((uint32_t)((n) & 0xff) << SPI_SSTIME_SSINACT_SHIFT)

/* SPI Master Clock Configuration Register */

#define SPI_CLKCFG_LO_SHIFT         (0)       /* Bits 0-7: SCLK Low Clock Cycles Control */
#define SPI_CLKCFG_LO_MASK          (0xff << SPI_CLKCFG_LO_SHIFT)
#  define SPI_CLKCFG_LO(n)          ((uint32_t)(n) << SPI_CLKCFG_LO_SHIFT)
#define SPI_CLKCFG_HI_SHIFT         (8)       /* Bits 8-15: SCLK Hi Clock Cycles Control */
#define SPI_CLKCFG_HI_MASK          (0xff << SPI_CLKCFG_HI_SHIFT)
#  define SPI_CLKCFG_HI_DISABLE     (0 << SPI_CLKCFG_HI_SHIFT)
#  define SPI_CLKCFG_HI(n)          ((uint32_t)(n) << SPI_CLKCFG_HI_SHIFT)
#define SPI_CLKCFG_SCALE_SHIFT      (16)      /* Bits 16-19: System Clock to SPI Clock
                                               *             Scale Factor */
#define SPI_CLKCFG_SCALE_MASK       (0xff << SPI_CLKCFG_SCALE_SHIFT)
#  define SPI_CLKCFG_SCALE_DISABLE  (0 << SPI_CLKCFG_SCALE_SHIFT)
#  define SPI_CLKCFG_SCALE(n)       ((uint32_t)(n) << SPI_CLKCFG_SCALE_SHIFT)

/* SPI DMA Control Register */

#define SPI_DMA_TXFIFOLVL_SHIFT     (0)       /* Bits 0-4: TX FIFO Threshold Level */
#define SPI_DMA_TXFIFOLVL_MASK      (0x1f << SPI_DMA_TXFIFOLVL_SHIFT)
#  define SPI_DMA_TXFIFOLVL(n)      ((uint32_t)((n) & 0x1f) << SPI_DMA_TXFIFOLVL_SHIFT)
#define SPI_DMA_TXFIFOEN            (1 << 6)  /* Bit 6:  TX FIFO Enabled */
#define SPI_DMA_TXFIFOCLR           (1 << 7)  /* Bit 7:  Clear the TX FIFO */
#define SPI_DMA_TXFIFOCNT_SHIFT     (8)       /* Bits 8-13: Number of Bytes in the TX
                                               *            FIFO */
#define SPI_DMA_TXFIFOCNT_MASK      (0x3f << SPI_DMA_TXFIFOCNT_SHIFT)
#define SPI_DMA_TXDMAEN             (1 << 15) /* Bit 15: TX DMA Enable */
#define SPI_DMA_RXFIFOLVL_SHIFT     (16)      /* Bits 16-20: RX FIFO Threshold Level */
#define SPI_DMA_RXFIFOLVL_MASK      (0x1f << SPI_DMA_RXFIFOLVL_SHIFT)
#  define SPI_DMA_RXFIFOLVL(n)      ((uint32_t)((n) & 0x1f) << SPI_DMA_RXFIFOLVL_SHIFT)
#define SPI_DMA_RXFIFOEN            (1 << 22) /* Bit 22: RX FIFO Enabled */
#define SPI_DMA_RXFIFOCLR           (1 << 23) /* Bit 23: Clear the RX FIFO */
#define SPI_DMA_RXFIFOCNT_SHIFT     (24)      /* Bits 24-29: Number of Bytes in the RX
                                               *             FIFO */
#define SPI_DMA_RXFIFOCNT_MASK      (0x3f << SPI_DMA_RXFIFOCNT_SHIFT)
#define SPI_DMA_RXDMAEN             (1 << 31) /* Bit 31: RX DMA Enable */

/* SPI Interrupt Status Flags Register and SPI Interrupt Enable Register */

#define SPI_INT_TXLEVEL             (1 << 0)  /* Bit 0:  TX FIFO Threshold Level Crossed */
#define SPI_INT_TXEMPTY             (1 << 1)  /* Bit 1:  TX FIFO Empty */
#define SPI_INT_RXLEVEL             (1 << 2)  /* Bit 2:  RX FIFO Threshold Level Crossed */
#define SPI_INT_RXFULL              (1 << 3)  /* Bit 3:  RX FIFO Full */
#define SPI_INT_SSA                 (1 << 4)  /* Bit 4:  Slave Select Asserted */
#define SPI_INT_SSD                 (1 << 5)  /* Bit 5:  Slave Select Deasserted */
#define SPI_INT_FAULT               (1 << 8)  /* Bit 8:  Multi-Master Fault Interrupt */
#define SPI_INT_ABORT               (1 << 9)  /* Bit 9:  Slave Mode Transaction Abort
                                               *         Detected */
#define SPI_INT_MDONE               (1 << 11) /* Bit 11: Master Data Transmission
                                               *         Complete */
#define SPI_INT_TXOVR               (1 << 12) /* Bit 12: TX FIFO Overrun */
#define SPI_INT_TXUND               (1 << 13) /* Bit 13: TX FIFO Underrun */
#define SPI_INT_RXOVR               (1 << 14) /* Bit 14: RX FIFO Overrun */
#define SPI_INT_RXUND               (1 << 15) /* Bit 15: RX FIFO Underrun */

/* SPI Wakeup Status Flags Register and SPI Wakeup Enable Register */

#define SPI_WAKE_TXLEVEL            (1 << 0)  /* Bit 0:  Wake on TX FIFO Threshold Level
                                               *         Crossed */
#define SPI_WAKE_TXEMPTY            (1 << 1)  /* Bit 1:  Wake on TX FIFO Empty */
#define SPI_WAKE_RXLEVEL            (1 << 2)  /* Bit 2:  Wake on RX FIFO Threshold Level
                                               *         Crossed */
#define SPI_WAKE_RXFULL             (1 << 3)  /* Bit 3:  Wake on RX FIFO Full */

/* SPI Active Status Register */

#define SPI_STAT_BUSY               (1 << 0)  /* Bit 0:  SPI Active Status */

#endif /* __ARCH_ARM_SRC_MAX326XX_HARDWARE_MAX32660_SPI_H */
