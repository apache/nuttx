/****************************************************************************
 * arch/risc-v/src/gd32vw55x/hardware/gd32vw55x_spi.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifndef __ARCH_RISCV_SRC_GD32VW55X_HARDWARE_GD32VW55X_SPI_H
#define __ARCH_RISCV_SRC_GD32VW55X_HARDWARE_GD32VW55X_SPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "gd32vw55x_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The GD32VW55x has a single SPI peripheral.  Its register interface is
 * identical to the SPI of the GD32F4 family (no FIFO, 8/16-bit frames).
 */

/* Register offsets *********************************************************/

#define GD32VW55X_SPI_CTL0_OFFSET     0x0000  /* Control register 0 */
#define GD32VW55X_SPI_CTL1_OFFSET     0x0004  /* Control register 1 */
#define GD32VW55X_SPI_STAT_OFFSET     0x0008  /* Status register */
#define GD32VW55X_SPI_DATA_OFFSET     0x000c  /* Data register */
#define GD32VW55X_SPI_CRCPOLY_OFFSET  0x0010  /* CRC polynomial register */
#define GD32VW55X_SPI_RCRC_OFFSET     0x0014  /* RX CRC register */
#define GD32VW55X_SPI_TCRC_OFFSET     0x0018  /* TX CRC register */

/* Register addresses *******************************************************/

#define GD32VW55X_SPI_CTL0(b)         ((b) + GD32VW55X_SPI_CTL0_OFFSET)
#define GD32VW55X_SPI_CTL1(b)         ((b) + GD32VW55X_SPI_CTL1_OFFSET)
#define GD32VW55X_SPI_STAT(b)         ((b) + GD32VW55X_SPI_STAT_OFFSET)
#define GD32VW55X_SPI_DATA(b)         ((b) + GD32VW55X_SPI_DATA_OFFSET)
#define GD32VW55X_SPI_CRCPOLY(b)      ((b) + GD32VW55X_SPI_CRCPOLY_OFFSET)
#define GD32VW55X_SPI_RCRC(b)         ((b) + GD32VW55X_SPI_RCRC_OFFSET)
#define GD32VW55X_SPI_TCRC(b)         ((b) + GD32VW55X_SPI_TCRC_OFFSET)

/* Register bit definitions *************************************************/

/* Control register 0 */

#define SPI_CTL0_CKPH                 (1 << 0)   /* Clock phase selection */
#define SPI_CTL0_CKPL                 (1 << 1)   /* Clock polarity */
#define SPI_CTL0_MSTMOD               (1 << 2)   /* Master mode enable */
#define SPI_CTL0_PSC_SHIFT            (3)        /* Bits 3-5: Prescaler */
#define SPI_CTL0_PSC_MASK             (7 << SPI_CTL0_PSC_SHIFT)
#  define SPI_CTL0_PSC_2              (0 << SPI_CTL0_PSC_SHIFT)
#  define SPI_CTL0_PSC_4              (1 << SPI_CTL0_PSC_SHIFT)
#  define SPI_CTL0_PSC_8              (2 << SPI_CTL0_PSC_SHIFT)
#  define SPI_CTL0_PSC_16             (3 << SPI_CTL0_PSC_SHIFT)
#  define SPI_CTL0_PSC_32             (4 << SPI_CTL0_PSC_SHIFT)
#  define SPI_CTL0_PSC_64             (5 << SPI_CTL0_PSC_SHIFT)
#  define SPI_CTL0_PSC_128            (6 << SPI_CTL0_PSC_SHIFT)
#  define SPI_CTL0_PSC_256            (7 << SPI_CTL0_PSC_SHIFT)
#define SPI_CTL0_SPIEN                (1 << 6)   /* SPI enable */
#define SPI_CTL0_LF                   (1 << 7)   /* LSB first mode */
#define SPI_CTL0_SWNSS                (1 << 8)   /* NSS pin level in SW mode */
#define SPI_CTL0_SWNSSEN              (1 << 9)   /* NSS software mode */
#define SPI_CTL0_RO                   (1 << 10)  /* Receive only */
#define SPI_CTL0_FF16                 (1 << 11)  /* 16-bit data frame */
#define SPI_CTL0_CRCNT                (1 << 12)  /* CRC next transfer */
#define SPI_CTL0_CRCEN                (1 << 13)  /* CRC calculation enable */
#define SPI_CTL0_BDOEN                (1 << 14)  /* Bidir. transmit output */
#define SPI_CTL0_BDEN                 (1 << 15)  /* Bidirectional enable */

/* Control register 1 */

#define SPI_CTL1_DMAREN               (1 << 0)   /* RX buffer DMA enable */
#define SPI_CTL1_DMATEN               (1 << 1)   /* TX buffer DMA enable */
#define SPI_CTL1_NSSDRV               (1 << 2)   /* Drive NSS output */
#define SPI_CTL1_TMOD                 (1 << 4)   /* TI mode enable */
#define SPI_CTL1_ERRIE                (1 << 5)   /* Error interrupt enable */
#define SPI_CTL1_RBNEIE               (1 << 6)   /* RBNE interrupt enable */
#define SPI_CTL1_TBEIE                (1 << 7)   /* TBE interrupt enable */

/* Status register */

#define SPI_STAT_RBNE                 (1 << 0)   /* RX buffer not empty */
#define SPI_STAT_TBE                  (1 << 1)   /* TX buffer empty */
#define SPI_STAT_CRCERR               (1 << 4)   /* CRC error */
#define SPI_STAT_CONFERR              (1 << 5)   /* Configuration error */
#define SPI_STAT_RXORERR              (1 << 6)   /* Reception overrun error */
#define SPI_STAT_TRANS                (1 << 7)   /* Transmitting on-going */
#define SPI_STAT_FERR                 (1 << 8)   /* Format error (TI mode) */

/* Data register */

#define SPI_DATA_MASK                 (0xffff)   /* Data value */

/* CRC registers */

#define SPI_CRCPOLY_MASK              (0xffff)   /* CRC polynomial value */
#define SPI_RCRC_MASK                 (0xffff)   /* RX CRC value */
#define SPI_TCRC_MASK                 (0xffff)   /* TX CRC value */

/* SPI mode selections (as written to CTL0) */

#define SPI_MASTER                    (SPI_CTL0_MSTMOD | SPI_CTL0_SWNSS)
#define SPI_SLAVE                     (0)

#endif /* __ARCH_RISCV_SRC_GD32VW55X_HARDWARE_GD32VW55X_SPI_H */
