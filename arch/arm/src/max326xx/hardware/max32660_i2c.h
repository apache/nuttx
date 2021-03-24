/****************************************************************************
 * arch/arm/src/max326xx/hardware/max32660_i2c.h
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

#ifndef __ARCH_ARM_SRC_MAX326XX_HARDWARE_MAX32660_I2C_H
#define __ARCH_ARM_SRC_MAX326XX_HARDWARE_MAX32660_I2C_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/max326_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define MAX326_I2C_CTRL0_OFFSET     0x0000  /* I2C Control 0 Register */
#define MAX326_I2C_STATUS_OFFSET    0x0004  /* I2C Status Register */
#define MAX326_I2C_INTFL0_OFFSET    0x0008  /* I2C Interrupt Flags 0 Register */
#define MAX326_I2C_INTEN0_OFFSET    0x000c  /* I2C Interrupt Enable 0 Register */
#define MAX326_I2C_INTFL1_OFFSET    0x0010  /* I2C Interrupts Flags 1 Register */
#define MAX326_I2C_INTEN1_OFFSET    0x0014  /* I2C Interrupts Enable 1 Register */
#define MAX326_I2C_FIFOLEN_OFFSET   0x0018  /* I2C FIFO Length Register */
#define MAX326_I2C_RXCTRL0_OFFSET   0x001c  /* I2C Receive Control 0 Register */
#define MAX326_I2C_RXCTRL1_OFFSET   0x0020  /* I2C Receive Control 1 Register 1 */
#define MAX326_I2C_TXCTRL0_OFFSET   0x0024  /* I2C Transmit Control 0 Register 0 */
#define MAX326_I2C_TXCTRL1_OFFSET   0x0028  /* I2C Transmit Control 1 Register 1 */
#define MAX326_I2C_FIFO_OFFSET      0x002c  /* I2C Transmit and Receive FIFO Register */
#define MAX326_I2C_MSTRMODE_OFFSET  0x0030  /* I2C Master Mode Register */
#define MAX326_I2C_CLKLO_OFFSET     0x0034  /* I2C Clock Low Time Register */
#define MAX326_I2C_CLKHI_OFFSET     0x0038  /* I2C Clock High Time Register */
#define MAX326_I2C_HSCLK_OFFSET     0x003c  /* I2C Hs-Mode Clock Control Register */
#define MAX326_I2C_TIMEOUT_OFFSET   0x0040  /* I2C Timeout Register */
#define MAX326_I2C_SLADDR_OFFSET    0x0044  /* I2C Slave Address Register */
#define MAX326_I2C_DMA_OFFSET       0x0048  /* I2C DMA Enable Register */

/* Register Addresses *******************************************************/

#define MAX326_I2C0_CTRL0           (MAX326_I2C0_BASE + MAX326_I2C_CTRL0_OFFSET)
#define MAX326_I2C0_STATUS          (MAX326_I2C0_BASE + MAX326_I2C_STATUS_OFFSET)
#define MAX326_I2C0_INTFL0          (MAX326_I2C0_BASE + MAX326_I2C_INTFL0_OFFSET)
#define MAX326_I2C0_INTEN0          (MAX326_I2C0_BASE + MAX326_I2C_INTEN0_OFFSET)
#define MAX326_I2C0_INTFL1          (MAX326_I2C0_BASE + MAX326_I2C_INTFL1_OFFSET)
#define MAX326_I2C0_INTEN1          (MAX326_I2C0_BASE + MAX326_I2C_INTEN1_OFFSET)
#define MAX326_I2C0_FIFOLEN         (MAX326_I2C0_BASE + MAX326_I2C_FIFOLEN_OFFSET)
#define MAX326_I2C0_RXCTRL0         (MAX326_I2C0_BASE + MAX326_I2C_RXCTRL0_OFFSET)
#define MAX326_I2C0_RXCTRL1         (MAX326_I2C0_BASE + MAX326_I2C_RXCTRL1_OFFSET)
#define MAX326_I2C0_TXCTRL0         (MAX326_I2C0_BASE + MAX326_I2C_TXCTRL0_OFFSET)
#define MAX326_I2C0_TXCTRL1         (MAX326_I2C0_BASE + MAX326_I2C_TXCTRL1_OFFSET)
#define MAX326_I2C0_FIFO            (MAX326_I2C0_BASE + MAX326_I2C_FIFO_OFFSET)
#define MAX326_I2C0_MSTRMODE        (MAX326_I2C0_BASE + MAX326_I2C_MSTRMODE_OFFSET)
#define MAX326_I2C0_CLKLO           (MAX326_I2C0_BASE + MAX326_I2C_CLKLO_OFFSET)
#define MAX326_I2C0_CLKHI           (MAX326_I2C0_BASE + MAX326_I2C_CLKHI_OFFSET)
#define MAX326_I2C0_HSCLK           (MAX326_I2C0_BASE + MAX326_I2C_HSCLK_OFFSET)
#define MAX326_I2C0_TIMEOUT         (MAX326_I2C0_BASE + MAX326_I2C_TIMEOUT_OFFSET)
#define MAX326_I2C0_SLADDR          (MAX326_I2C0_BASE + MAX326_I2C_SLADDR_OFFSET)
#define MAX326_I2C0_DMA             (MAX326_I2C0_BASE + MAX326_I2C_DMA_OFFSET)

#define MAX326_I2C1_CTRL0           (MAX326_I2C1_BASE + MAX326_I2C_CTRL0_OFFSET)
#define MAX326_I2C1_STATUS          (MAX326_I2C1_BASE + MAX326_I2C_STATUS_OFFSET)
#define MAX326_I2C1_INTFL0          (MAX326_I2C1_BASE + MAX326_I2C_INTFL0_OFFSET)
#define MAX326_I2C1_INTEN0          (MAX326_I2C1_BASE + MAX326_I2C_INTEN0_OFFSET)
#define MAX326_I2C1_INTFL1          (MAX326_I2C1_BASE + MAX326_I2C_INTFL1_OFFSET)
#define MAX326_I2C1_INTEN1          (MAX326_I2C1_BASE + MAX326_I2C_INTEN1_OFFSET)
#define MAX326_I2C1_FIFOLEN         (MAX326_I2C1_BASE + MAX326_I2C_FIFOLEN_OFFSET)
#define MAX326_I2C1_RXCTRL0         (MAX326_I2C1_BASE + MAX326_I2C_RXCTRL0_OFFSET)
#define MAX326_I2C1_RXCTRL1         (MAX326_I2C1_BASE + MAX326_I2C_RXCTRL1_OFFSET)
#define MAX326_I2C1_TXCTRL0         (MAX326_I2C1_BASE + MAX326_I2C_TXCTRL0_OFFSET)
#define MAX326_I2C1_TXCTRL1         (MAX326_I2C1_BASE + MAX326_I2C_TXCTRL1_OFFSET)
#define MAX326_I2C1_FIFO            (MAX326_I2C1_BASE + MAX326_I2C_FIFO_OFFSET)
#define MAX326_I2C1_MSTRMODE        (MAX326_I2C1_BASE + MAX326_I2C_MSTRMODE_OFFSET)
#define MAX326_I2C1_CLKLO           (MAX326_I2C1_BASE + MAX326_I2C_CLKLO_OFFSET)
#define MAX326_I2C1_CLKHI           (MAX326_I2C1_BASE + MAX326_I2C_CLKHI_OFFSET)
#define MAX326_I2C1_HSCLK           (MAX326_I2C1_BASE + MAX326_I2C_HSCLK_OFFSET)
#define MAX326_I2C1_TIMEOUT         (MAX326_I2C1_BASE + MAX326_I2C_TIMEOUT_OFFSET)
#define MAX326_I2C1_SLADDR          (MAX326_I2C1_BASE + MAX326_I2C_SLADDR_OFFSET)
#define MAX326_I2C1_DMA             (MAX326_I2C1_BASE + MAX326_I2C_DMA_OFFSET)

/* Register Bit-field Definitions *******************************************/

/* I2C Control 0 Register */

#define I2C_CTRL0_I2CEN             (1 << 0)  /* Bit 0:  I2C Enable */
#define I2C_CTRL0_MST               (1 << 1)  /* Bit 1:  Master Mode Enable */
#define I2C_CTRL0_GCEN              (1 << 2)  /* Bit 2:  General Call Address Enable */
#define I2C_CTRL0_IRXM              (1 << 3)  /* Bit 3:  Interactive Receive Mode (IRXM) */
#define I2C_CTRL0_ACK               (1 << 4)  /* Bit 4:  Interactive Receive Mode (IRXM)
                                               *         Acknowledge */
#define I2C_CTRL0_SCLO              (1 << 6)  /* Bit 6:  SCL Pin Control */
#define I2C_CTRL0_SDAO              (1 << 7)  /* Bit 7:  SDA Pin Control */
#define I2C_CTRL0_SCL               (1 << 8)  /* Bit 8:  SCL Status */
#define I2C_CTRL0_SDA               (1 << 9)  /* Bit 9:  SDA Status */
#define I2C_CTRL0_SWOE              (1 << 10) /* Bit 10: Software output Enabled */
#define I2C_CTRL0_READ              (1 << 11) /* Bit 11: Read/Write Bit Status */
#define I2C_CTRL0_SCLSTRD           (1 << 12) /* Bit 12: SCL Clock Stretch Control */
#define I2C_CTRL0_SCLPPM            (1 << 13) /* Bit 13: SCL Push-Pull Mode Enable */
#define I2C_CTRL0_HSMODE            (1 << 15) /* Bit 15: High Speed Mode */

/* I2C Status Register */

#define I2C_STATUS_BUSY             (1 << 0)  /* Bit 0:  Bus Busy */
#define I2C_STATUS_RXE              (1 << 1)  /* Bit 1:  RX FIFO Empty */
#define I2C_STATUS_RXF              (1 << 2)  /* Bit 2:  RX FIFO Full */
#define I2C_STATUS_TXE              (1 << 3)  /* Bit 3:  TX FIFO Empty */
#define I2C_STATUS_TXF              (1 << 4)  /* Bit 4:  TX FIFO Full */
#define I2C_STATUS_CKMD             (1 << 5)  /* Bit 5:  SCL Drive Status */
#define I2C_STATUS_STAT_SHIFT       (8)       /* Bits 8-11: I2C Controller Status */
#define I2C_STATUS_STAT_MASK        (15 << I2C_STATUS_STAT_SHIFT)

#  define I2C_STATUS_STAT_IDLE        (0 << I2C_STATUS_STAT_SHIFT)  /* Idle */
#  define I2C_STATUS_STAT_MTXADDR     (1 << I2C_STATUS_STAT_SHIFT)  /* Master Transmit
                                                                     * address */
#  define I2C_STATUS_STAT_MRXACK      (2 << I2C_STATUS_STAT_SHIFT)  /* Master Receive
                                                                     * address ACK */
#  define I2C_STATUS_STAT_MTXEXTADDR  (3 << I2C_STATUS_STAT_SHIFT)  /* Master Transmit
                                                                     * extended address */
#  define I2C_STATUS_STAT_MRXEXTACK   (4 << I2C_STATUS_STAT_SHIFT)  /* Master Receive
                                                                     * extended address
                                                                     * ACK */
#  define I2C_STATUS_STAT_SRXADDR     (5 << I2C_STATUS_STAT_SHIFT)  /* Slave Receive
                                                                     * address */
#  define I2C_STATUS_STAT_STXRACK     (6 << I2C_STATUS_STAT_SHIFT)  /* Slave Transmit
                                                                     * address ACK */
#  define I2C_STATUS_STAT_SRXEXTADDR  (7 << I2C_STATUS_STAT_SHIFT)  /* Slave Receive
                                                                     * extended address */
#  define I2C_STATUS_STAT_STXEXTACK   (8 << I2C_STATUS_STAT_SHIFT)  /* Slave transit
                                                                     * extended address
                                                                     * ACK */
#  define I2C_STATUS_STAT_TXDATA      (9 << I2C_STATUS_STAT_SHIFT)  /* Transmit data
                                                                     * (Master or Slave) */
#  define I2C_STATUS_STAT_RXACK       (10 << I2C_STATUS_STAT_SHIFT) /* Receive data ACK
                                                                     * (Master or Slave) */
#  define I2C_STATUS_STAT_RXDATA      (11 << I2C_STATUS_STAT_SHIFT) /* Receive data
                                                                     * (Master or Slave) */
#  define I2C_STATUS_STAT_TXACK       (12 << I2C_STATUS_STAT_SHIFT) /* Transmit data ACK
                                                                     * (Master or Slave) */
#  define I2C_STATUS_STAT_NACK        (13 << I2C_STATUS_STAT_SHIFT) /* NACK stage (Master
                                                                     * or Slave) */
#  define I2C_STATUS_STAT_MBUSY       (15 << I2C_STATUS_STAT_SHIFT) /* Another master is
                                                                     * addressing another
                                                                     * slave. */

/* I2C Interrupt Flags 0 Register and I2C Interrupt Enable 0 Register */

#define I2C_INT0_DONEI              (1 << 0)  /* Bit 0:  Transfer Complete Interrupt */
#define I2C_INT0_IRXMI              (1 << 1)  /* Bit 1:  Interactive Receive Mode
                                               *         Interrupt */
#define I2C_INT0_GCI                (1 << 2)  /* Bit 2:  General Call Address Match
                                               *         Received Interrupt (slave) */
#define I2C_INT0_AMI                (1 << 3)  /* Bit 3:  Address Match Status Interrupt
                                               *         (slave) */
#define I2C_INT0_RXTHI              (1 << 4)  /* Bit 4:  RX FIFO Threshold Level Interrupt */
#define I2C_INT0_TXTHI              (1 << 5)  /* Bit 5:  TX FIFO Threshold Level Interrupt */
#define I2C_INT0_STOPI              (1 << 6)  /* Bit 6:  Slave Mode: STOP Condition
                                               *         Interrupt (slave) */
#define I2C_INT0_ADRACKI            (1 << 7)  /* Bit 7:  Address ACK from External Slave
                                               *         Interrupt (master) */
#define I2C_INT0_ARBERI             (1 << 8)  /* Bit 8:  Arbitration Lost Interrupt
                                               *         (master) */
#define I2C_INT0_TOERI              (1 << 9)  /* Bit 9:  Timeout Error Interrupt */
#define I2C_INT0_ADRERI             (1 << 10) /* Bit 10: Address NACK from Slave Error
                                               *         (master) */
#define I2C_INT0_DATERI             (1 << 11) /* Bit 11: Data NACK from External Slave
                                               *         Interrupt (master) */
#define I2C_INT0_DNRERI             (1 << 12) /* Bit 12: Slave Mode Do Not Respond
                                               *         Interrupt */
#define I2C_INT0_STRTERI            (1 << 13) /* Bit 13: Out of Sequence START Interrupt */
#define I2C_INT0_STOPERI            (1 << 14) /* Bit 14: Out of Sequence STOP Interrupt */
#define I2C_INT0_TXLOI              (1 << 15) /* Bit 15: TX FIFO Locked Interrupt */

/* I2C Interrupts Flags 1 Register and I2C Interrupts Enable 1 Register */

#define I2C_INT1_RXOFI              (1 << 0)  /* Bit 0:  RX FIFO Overflow Interrupt
                                               *         (slave) */
#define I2C_INT1_TXUFI              (1 << 1)  /* Bit 1:  TX FIFO Underflow Interrupt
                                               *         (slave) */

/* I2C FIFO Length Register */

#define I2C_FIFOLEN_RXLEN_SHIFT     (0)       /* Bits 0-7: RX FIFO Length */
#define I2C_FIFOLEN_RXLEN_MASK      (0xff << I2C_FIFOLEN_RXLEN_SHIFT)
#define I2C_FIFOLEN_TXLEN_SHIFT     (8)       /* Bits 8-15: TX FIFO Length */
#define I2C_FIFOLEN_TXLEN_MASK      (0xff << I2C_FIFOLEN_TXLEN_SHIFT)

/* I2C Receive Control 0 Register */

#define I2C_RXCTRL0_DNR             (1 << 0)  /* Bit 0:  Do Not Respond (slave) */
#define I2C_RXCTRL0_RXFSH           (1 << 7)  /* Bit 7:  Flush RX FIFO */
#define I2C_RXCTRL0_RXTH_SHIFT      (8)       /* Bits 8-11: RX FIFO Threshold Level */
#define I2C_RXCTRL0_RXTH_MASK       (15 << I2C_RXCTRL0_RXTH_SHIFT)

/* I2C Receive Control 1 Register 1 */

#define I2C_RXCTRL1_RXCNT_SHIFT     (0)       /* Bits 0-7: RX FIFO Transaction Byte Count */
#define I2C_RXCTRL1_RXCNT_MASK      (0xff << I2C_RXCTRL1_RXCNT_SHIFT)
#  define I2C_RXCTRL1_RXCNT(n)      ((uint16_t)((n) & 0xff) << I2C_RXCTRL1_RXFIFO_SHIFT)
#define I2C_RXCTRL1_RXFIFO_SHIFT    (8)       /* Bits 8-11: RX FIFO Byte Count */
#define I2C_RXCTRL1_RXFIFO_MASK     (15 << I2C_RXCTRL1_RXFIFO_SHIFT)

/* I2C Transmit Control 0 Register 0 */

#define I2C_TXCTRL0_TXPRELD         (1 << 0)  /* Bit 0:  TX FIFO Preload Mode Enable */
#define I2C_TXCTRL0_TXFSH           (1 << 7)  /* Bit 7:  Flush TX FIFO */
#define I2C_TXCTRL0_TXTH_SHIFT      (8)       /* Bits 8-11: TX FIFO Threshold Level */
#define I2C_TXCTRL0_TXTH_MASK       (15 << I2C_TXCTRL0_TXTH_SHIFT)

/* I2C Transmit Control 1 Register 1 */

#define I2C_TXCTRL1_TXRDY           (1 << 0)  /* Bit 0:  Transmit FIFO Preload Ready
                                               *         Status */
#define I2C_TXCTRL1_TXLAST          (1 << 1)  /* Bit 1:  Transmit Last (slave) */
#define I2C_TXCTRL1_FLSH_GCADDRDIS  (1 << 2)  /* Bit 2:  TX FIFO Auto Flush Disable on
                                               *         General Call Address Match */
#define I2C_TXCTRL1_FLSH_SLADDRDIS  (1 << 4)  /* Bit 4:  TX FIFO Auto Flush Disable for
                                               *         Slave Address Match */
#define I2C_TXCTRL1_FLSH_NACKDIS    (1 << 5)  /* Bit 5:  TX FIFO Auto Flush Disable for
                                               *         NACK */
#define I2C_TXCTRL1_TXFIFO_SHIFT    (8)       /* Bits 8-11: TX FIFO Byte Count */
#define I2C_TXCTRL1_TXFIFO_MASK     (15 << I2C_TXCTRL1_TXFIFO_SHIFT)

/* I2C Transmit and Receive FIFO Register */

#define I2C_FIFO_MASK               (0xff)    /* Bits 0-7: I2C FIFO Data */

/* I2C Master Mode Register */

#define I2C_MSTRMODE_START          (1 << 0)  /* Bit 0:  Start Master Mode Transfer */
#define I2C_MSTRMODE_RESTART        (1 << 1)  /* Bit 1:  Send Repeated START Condition */
#define I2C_MSTRMODE_STOP           (1 << 2)  /* Bit 2:  Send STOP Condition */
#define I2C_MSTRMODE_SEA            (1 << 7)  /* Bit 7:  Slave Extended Addressing */

/* I2C Clock Low Time Register */

#define I2C_CLKLO_MASK              (0x1ff)   /* Bits 0-8:  Clock Low Time */

/* I2C Clock High Time Register */

#define I2C_CLKHI_MASK              (0x1ff)   /* Bits 0-8:  Clock High Time */

/* I2C Hs-Mode Clock Control Register */

#define I2C_HSCLK_HSCLKLO_SHIFT     (0)       /* Bits 0-7: Hs-Mode Clock Low Time */
#define I2C_HSCLK_HSCLKLO_MASK      (0xff << I2C_HSCLK_HSCLKLO_SHIFT)
#  define I2C_HSCLK_HSCLKLO(n)      ((uint32_t)((n) - 1) << I2C_HSCLK_HSCLKLO_SHIFT)
#define I2C_HSCLK_HSCLKHI_SHIFT     (8)       /* Bits 8-15: Hs-Mode Clock High Time */
#define I2C_HSCLK_HSCLKHI_MASK      (0xff << I2C_HSCLK_HSCLKHI_SHIFT)
#  define I2C_HSCLK_HSCLKHI(n)      ((uint32_t)((n) - 1) << I2C_HSCLK_HSCLKHI_SHIFT)

/* I2C Timeout Register */

#define I2C_TIMEOUT_MASK            (0xffff)  /* Bits 0-15: Bus Error SCL Timeout Period */

/* I2C Slave Address Register */

#define I2C_SLADDR_SLA_SHIFT        (0)       /* Bits 0-9: Slave Mode Slave Address */
#define I2C_SLADDR_SLA_MASK         (0x3ff << I2C_SLADDR_SLA_SHIFT)
#  define I2C_SLADDR_SLA(n)         ((uint32_t)(n) << I2C_SLADDR_SLA_SHIFT)
#define I2C_SLADDR_EA               (1 << 15) /* Bit 15: Slave Mode Extended Address
                                               *         Select */

/* I2C DMA Enable Register */

#define I2C_DMA_TXEN                (1 << 0)  /* Bit 0:  TX DMA Channel Enable */
#define I2C_DMA_RXEN                (1 << 1)  /* Bit 1:  RX DMA Channel Enable */

#endif /* __ARCH_ARM_SRC_MAX326XX_HARDWARE_MAX32660_I2C_H */
