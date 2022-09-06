/****************************************************************************
 * arch/arm/src/gd32f4/hardware/gd32f4xx_spi.h
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

#ifndef __ARCH_ARM_SRC_GD32F4_HARDWARE_GD32F4XX_SPI_H
#define __ARCH_ARM_SRC_GD32F4_HARDWARE_GD32F4XX_SPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* SPIx(x=0,1,2,3,4,5) definitions */
#define GD32_SPI0_BASE             (GD32_SPI_BASE+0x0000F800)      /* SPI0 base address */
#define GD32_SPI1_BASE             (GD32_SPI_BASE+0x00000000)      /* SPI1 base address */
#define GD32_SPI2_BASE             (GD32_SPI_BASE+0x00000400)      /* SPI2 base address */
#define GD32_SPI3_BASE             (GD32_SPI_BASE+0x0000FC00)      /* SPI3 base address */
#define GD32_SPI4_BASE             (GD32_SPI_BASE+0x00011800)      /* SPI4 base address */
#define GD32_SPI5_BASE             (GD32_SPI_BASE+0x00011C00)      /* SPI5 base address */

/* I2Sx_ADD(x=1,2) definitions */
#define GD32_I2S1_ADD_BASE         (GD32_I2S_ADD_BASE+0x00000000)  /* I2S1_ADD base address */
#define GD32_I2S2_ADD_BASE         (GD32_I2S_ADD_BASE+0x00000C00)  /* I2S2_ADD base address */

/* Register Offsets *********************************************************/
#define GD32_SPI_CTL0_OFFSET       0x0000      /* SPI control register 0 offset */
#define GD32_SPI_CTL1_OFFSET       0x0004      /* SPI control register 1 offset */
#define GD32_SPI_STAT_OFFSET       0x0008      /* SPI status register offset */
#define GD32_SPI_DATA_OFFSET       0x000C      /* SPI data register offset */
#define GD32_SPI_CRCPOLY_OFFSET    0x0010      /* SPI CRC polynomial register offset */
#define GD32_SPI_RCRC_OFFSET       0x0014      /* SPI receive CRC register offset */
#define GD32_SPI_TCRC_OFFSET       0x0018      /* SPI transmit CRC register offset */
#define GD32_SPI_I2SCTL_OFFSET     0x001C      /* SPI I2S control register offset */
#define GD32_SPI_I2SPSC_OFFSET     0x0020      /* SPI I2S clock prescaler register offset */
#define GD32_SPI_QCTL_OFFSET       0x0080      /* SPI quad mode control register offset */

/* Register Addresses *******************************************************/

#define GD32_SPI0                  GD32_SPI0_BASE
#define GD32_SPI1                  GD32_SPI1_BASE
#define GD32_SPI2                  GD32_SPI2_BASE
#define GD32_SPI3                  GD32_SPI3_BASE
#define GD32_SPI4                  GD32_SPI4_BASE
#define GD32_SPI5                  GD32_SPI5_BASE

#define GD32_I2S1_ADD              GD32_I2S1_ADD_BASE
#define GD32_I2S2_ADD              GD32_I2S2_ADD_BASE

/* SPI registers definitions */
#define GD32_SPI_CTL0(spix)        ((spix)+GD32_SPI_CTL0_OFFSET)      /* SPI control register 0 */
#define GD32_SPI_CTL1(spix)        ((spix)+GD32_SPI_CTL1_OFFSET)      /* SPI control register 1*/
#define GD32_SPI_STAT(spix)        ((spix)+GD32_SPI_STAT_OFFSET)      /* SPI status register */
#define GD32_SPI_DATA(spix)        ((spix)+GD32_SPI_DATA_OFFSET)      /* SPI data register */
#define GD32_SPI_CRCPOLY(spix)     ((spix)+GD32_SPI_CRCPOLY_OFFSET)   /* SPI CRC polynomial register */
#define GD32_SPI_RCRC(spix)        ((spix)+GD32_SPI_RCRC_OFFSET)      /* SPI receive CRC register */
#define GD32_SPI_TCRC(spix)        ((spix)+GD32_SPI_TCRC_OFFSET)      /* SPI transmit CRC register */
#define GD32_SPI_I2SCTL(spix)      ((spix)+GD32_SPI_I2SCTL_OFFSET)    /* SPI I2S control register */
#define GD32_SPI_I2SPSC(spix)      ((spix)+GD32_SPI_I2SPSC_OFFSET)    /* SPI I2S clock prescaler register */
#define GD32_SPI_QCTL(spix)        ((spix)+GD32_SPI_QCTL_OFFSET)      /* SPI quad mode control register */

/* I2S_ADD registers definitions */
#define I2S_ADD_CTL0(i2sx_add)     ((i2sx_add)+GD32_SPI_CTL0_OFFSET)     /* I2S_ADD control register 0 */
#define I2S_ADD_CTL1(i2sx_add)     ((i2sx_add)+GD32_SPI_CTL1_OFFSET)     /* I2S_ADD control register 1*/
#define I2S_ADD_STAT(i2sx_add)     ((i2sx_add)+GD32_SPI_STAT_OFFSET)     /* I2S_ADD status register */
#define I2S_ADD_DATA(i2sx_add)     ((i2sx_add)+GD32_SPI_DATA_OFFSET)     /* I2S_ADD data register */
#define I2S_ADD_CRCPOLY(i2sx_add)  ((i2sx_add)+GD32_SPI_CRCPOLY_OFFSET)  /* I2S_ADD CRC polynomial register */
#define I2S_ADD_RCRC(i2sx_add)     ((i2sx_add)+GD32_SPI_RCRC_OFFSET)     /* I2S_ADD receive CRC register */
#define I2S_ADD_TCRC(i2sx_add)     ((i2sx_add)+GD32_SPI_TCRC_OFFSET)     /* I2S_ADD transmit CRC register */
#define I2S_ADD_I2SCTL(i2sx_add)   ((i2sx_add)+GD32_SPI_I2SCTL_OFFSET)   /* I2S_ADD I2S control register */
#define I2S_ADD_I2SPSC(i2sx_add)   ((i2sx_add)+GD32_SPI_I2SPSC_OFFSET)   /* I2S_ADD I2S clock prescaler register */

/* Register Bitfield Definitions ********************************************/

/* Control register 0 */

#define SPI_CTL0_CKPH              (1 << 0)            /* Bit 0: clock phase selection*/
#define SPI_CTL0_CKPL              (1 << 1)            /* Bit 1: clock polarity selection */
#define SPI_CTL0_MSTMOD            (1 << 2)            /* Bit 2: master mode enable */
#define SPI_CTL0_PSC_SHIFT         (3)                 /* Bit 3-5: master clock prescaler selection */ 
#define SPI_CTL0_PSC_MASK          (7 << SPI_CTL0_PSC_SHIFT)
#define SPI_CTL0_PSC(n)            ((n) << SPI_CTL0_PSC_SHIFT)
#  define SPI_CTL0_PSC_2           SPI_CTL0_PSC(0)     /* 000: SPI clock prescale factor is 2 */
#  define SPI_CTL0_PSC_4           SPI_CTL0_PSC(1)     /* 001: SPI clock prescale factor is 4 */
#  define SPI_CTL0_PSC_8           SPI_CTL0_PSC(2)     /* 010: SPI clock prescale factor is 8 */
#  define SPI_CTL0_PSC_16          SPI_CTL0_PSC(3)     /* 011: SPI clock prescale factor is 16 */
#  define SPI_CTL0_PSC_32          SPI_CTL0_PSC(4)     /* 100: SPI clock prescale factor is 32 */
#  define SPI_CTL0_PSC_64          SPI_CTL0_PSC(5)     /* 101: SPI clock prescale factor is 64 */
#  define SPI_CTL0_PSC_128         SPI_CTL0_PSC(6)     /* 110: SPI clock prescale factor is 128 */
#  define SPI_CTL0_PSC_256         SPI_CTL0_PSC(7)     /* 111: SPI clock prescale factor is 256 */

#define SPI_CTL0_SPIEN             (1 << 6)            /* Bit 6: SPI enable*/
#define SPI_CTL0_LF                (1 << 7)            /* Bit 7: lsb first mode */
#define SPI_CTL0_SWNSS             (1 << 8)            /* Bit 8: nss pin selection in nss software mode */
#define SPI_CTL0_SWNSSEN           (1 << 9)            /* Bit 9: nss software mode selection */
#define SPI_CTL0_RO                (1 << 10)           /* Bit 10: receive only */
#define SPI_CTL0_FF16              (1 << 11)           /* Bit 11: data frame size */
#define SPI_CTL0_CRCNT             (1 << 12)           /* Bit 12: CRC next transfer */
#define SPI_CTL0_CRCEN             (1 << 13)           /* Bit 13: CRC calculation enable */
#define SPI_CTL0_BDOEN             (1 << 14)           /* Bit 14: bidirectional transmit output enable*/
#define SPI_CTL0_BDEN              (1 << 15)           /* Bit 15: bidirectional enable */

/* SPI_CTL1 */
#define SPI_CTL1_DMAREN            (1 << 0)            /* Bit 0: receive buffer dma enable */
#define SPI_CTL1_DMATEN            (1 << 1)            /* Bit 1: transmit buffer dma enable */
#define SPI_CTL1_NSSDRV            (1 << 2)            /* Bit 2: drive nss output */
#define SPI_CTL1_TMOD              (1 << 4)            /* Bit 4: SPI TI mode enable */
#define SPI_CTL1_ERRIE             (1 << 5)            /* Bit 5: errors interrupt enable */
#define SPI_CTL1_RBNEIE            (1 << 6)            /* Bit 6: receive buffer not empty interrupt enable */
#define SPI_CTL1_TBEIE             (1 << 7)            /* Bit 7: transmit buffer empty interrupt enable */

/* SPI_STAT */
#define SPI_STAT_RBNE              (1 << 0)            /* Bit 0: receive buffer not empty */
#define SPI_STAT_TBE               (1 << 1)            /* Bit 1: transmit buffer empty */
#define SPI_STAT_I2SCH             (1 << 2)            /* Bit 2: I2S channel side */
#define SPI_STAT_TXURERR           (1 << 3)            /* Bit 3: I2S transmission underrun error bit */
#define SPI_STAT_CRCERR            (1 << 4)            /* Bit 4: SPI CRC error bit */
#define SPI_STAT_CONFERR           (1 << 5)            /* Bit 5: SPI configuration error bit */
#define SPI_STAT_RXORERR           (1 << 6)            /* Bit 6: SPI reception overrun error bit */
#define SPI_STAT_TRANS             (1 << 7)            /* Bit 7: transmitting on-going bit */
#define SPI_STAT_FERR              (1 << 8)            /* Bit 8: format error bit */

/* SPI_DATA */
#define SPI_DATA_SHIFT             (0)                 /* Bits 0-15: data transfer register */
#define SPI_DATA_MASK              (0xffff << SPI_DATA_SHIFT)

/* SPI_CRCPOLY */
#define SPI_CRCPOLY_CPR_SHIFT      (0)                 /* Bits 0-15: CRC polynomial register */
#define SPI_CRCPOLY_CPR__MASK      (0xffff << SPI_CRCPOLY_CPR_SHIFT)

/* SPI_RCRC */
#define SPI_RCRC_RCR_SHIFT         (0)                 /* Bits 0-15: CRX CRC register */
#define SPI_RCRC_RCR_MASK          (0xffff << SPI_RCRC_RCR_SHIFT)

/* SPI_TCRC */
#define SPI_TCRC_TCR_SHIFT         (0)                 /* Bits 0-15: TX CRC register */
#define SPI_TCRC_TCR_MASK          (0xffff << SPI_TCRC_TCR_SHIFT)

/* SPI_I2SCTL */
#define SPI_I2SCTL_CHLEN           (1 << 0)            /* Bit 0: channel length */
#define SPI_I2SCTL_DTLEN_SHIFT     (1)                 /* Bit 1-2: data length */
#define SPI_I2SCTL_DTLEN_MASK      (3 << SPI_I2SCTL_DTLEN_SHIFT)
#define SPI_I2SCTL_DTLEN(n)        ((n) << SPI_I2SCTL_DTLEN_SHIFT)
#  define SPI_I2SCTL_DTLEN_DT16B   SPI_I2SCTL_DTLEN(0) /* 00: 16 bits */
#  define SPI_I2SCTL_DTLEN_DT24B   SPI_I2SCTL_DTLEN(1) /* 00: 24 bits */
#  define SPI_I2SCTL_DTLEN_DT32B   SPI_I2SCTL_DTLEN(2) /* 00: 32 bits */

#define SPI_I2SCTL_CKPL            (1 << 3)            /* Bit 3: idle state clock polarity */
#define SPI_I2SCTL_I2SSTD_SHIFT    (4)                 /* Bit 4-5: I2S standard selection */
#define SPI_I2SCTL_I2SSTD_MASK     (3 << SPI_I2SCTL_I2SSTD_SHIFT)
#define SPI_I2SCTL_I2SSTD(n)       ((n) << SPI_I2SCTL_I2SSTD_SHIFT)
#  define SPI_I2SSTD_PHILLIPS      SPI_I2SCTL_I2SSTD(0)    /* 00: I2S Phillips standard */
#  define SPI_I2SSTD_MSB           SPI_I2SCTL_I2SSTD(0)    /* 01: MSB justified standard */
#  define SPI_I2SSTD_LSB           SPI_I2SCTL_I2SSTD(0)    /* 10: LSB justified standard */
#  define SPI_I2SSTD_PCM           SPI_I2SCTL_I2SSTD(0)    /* 11: PCM standard*/

#define SPI_I2SCTL_PCMSMOD         (1 << 7)            /* Bit 7: PCM frame synchronization mode */

#define SPI_I2SCTL_I2SOPMOD_SHIIFT (8)                 /* Bit 8-9: I2S operation mode */
#define SPI_I2SCTL_I2SOPMOD_MASK   (3 << SPI_I2SCTL_I2SOPMOD_SHIIFT)
#define SPI_I2SCTL_I2SOPMOD(n)     ((n) << SPI_I2SCTL_I2SOPMOD_SHIIFT)
#  define SPI_I2SCTL_I2SOPMSTRAN   SPI_I2SCTL_I2SOPMOD(0)  /* 00: Slave transmission mode */
#  define SPI_I2SCTL_I2SOPMSRECV   SPI_I2SCTL_I2SOPMOD(1)  /* 01: Slave reception mode */
#  define SPI_I2SCTL_I2SOPMMTRAN   SPI_I2SCTL_I2SOPMOD(2)  /* 10: Master transmission mode */
#  define SPI_I2SCTL_I2SOPMMRECV   SPI_I2SCTL_I2SOPMOD(3)  /* 11: Master reception mode */

#define SPI_I2SCTL_I2SEN           (1 << 10)            /* Bit 10: I2S enable */
#define SPI_I2SCTL_I2SSEL          (1 << 11)            /* Bit 11: I2S mode selection */

/* SPI_I2S_PSC */

#define SPI_I2SPSC_DIV_SHIFT       (0)                 /* Bit0-7: dividing factor for the prescaler */
#define SPI_I2SPSC_DIV_MASK        (0xff << SPI_I2SPSC_DIV_SHIFT)
#define SPI_I2SPSC_DIV(n)          ((n) << SPI_I2SPSC_DIV_SHIFT)  /* Real divider value is DIV * 2 + OF. */

#define SPI_I2SPSC_OF              (1 << 8)                 /* Bit8: odd factor for the prescaler */
#define SPI_I2SPSC_MCKOEN          (1 << 9)                 /* Bit9: I2S MCK output enable */

/* SPI_SPI_QCTL(only SPI5) */
#define SPI_QCTL_QMOD              (1 << 0)                 /* Bit0: quad-SPI mode enable */
#define SPI_QCTL_QRD               (1 << 1)                 /* Bit1: quad-SPI mode read select */
#define SPI_QCTL_IO23_DRV          (1 << 2)                 /* Bit2: drive SPI_IO2 and SPI_IO3 enable */

/* SPI DMA enable */
#define SPI_CTL1_DMAEN             (SPI_CTL1_DMAREN | SPI_CTL1_DMATEN) /* SPI RX/TX DMA enable */

/* SPI mode definitions */
#define SPI_MASTER                 (SPI_CTL0_MSTMOD | SPI_CTL0_SWNSS)  /* SPI as master, SWNSS=1 to prevent CONFERR */
#define SPI_SLAVE                  (0x00000000)                        /* SPI as slave */

/* SPI bidirectional transfer direction */
#define SPI_BIDIRECTIONAL_TRANSMIT SPI_CTL0_BDOEN                      /* SPI work in transmit-only mode */
#define SPI_BIDIRECTIONAL_RECEIVE  SPI_CTL0_BDOEN)                     /* SPI work in receive-only mode */

/* SPI transmit type */
#define SPI_TRANSMODE_FULLDUPLEX   (00000000)                          /* SPI receive and send data at fullduplex communication */
#define SPI_TRANSMODE_RECEIVEONLY  SPI_CTL0_RO                         /* SPI only receive data */
#define SPI_TRANSMODE_BDRECEIVE    SPI_CTL0_BDEN                       /* bidirectional receive data */
#define SPI_TRANSMODE_BDTRANSMIT   (SPI_CTL0_BDEN | SPI_CTL0_BDOEN)    /* bidirectional transmit data*/

#endif /* __ARCH_ARM_SRC_GD32F4_HARDWARE_GD32F4XX_SPI_H */
