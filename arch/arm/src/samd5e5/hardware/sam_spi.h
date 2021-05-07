/****************************************************************************
 * arch/arm/src/samd5e5/hardware/sam_spi.h
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

#ifndef __ARCH_ARM_SRC_SAMD5E5_HARDWARE_SAM_SPI_H
#define __ARCH_ARM_SRC_SAMD5E5_HARDWARE_SAM_SPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "hardware/sam_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* SPI register offsets *****************************************************/

#define SAM_SPI_CTRLA_OFFSET       0x0000  /* Control A register */
#define SAM_SPI_CTRLB_OFFSET       0x0004  /* Control B register */
#define SAM_SPI_CTRLC_OFFSET       0x0008  /* Control C register */
#define SAM_SPI_BAUD_OFFSET        0x000c  /* Baud register */
#define SAM_SPI_INTENCLR_OFFSET    0x0014  /* Interrupt enable clear register */
#define SAM_SPI_INTENSET_OFFSET    0x0016  /* Interrupt enable set register */
#define SAM_SPI_INTFLAG_OFFSET     0x0018  /* Interrupt flag and status clear register */
#define SAM_SPI_STATUS_OFFSET      0x001a  /* Status register */
#define SAM_SPI_SYNCBUSY_OFFSET    0x001c  /* Synchronization busy register */
#define SAM_SPI_LENGTH_OFFSET      0x0022  /* Length register */
#define SAM_SPI_ADDR_OFFSET        0x0024  /* Address register */
#define SAM_SPI_DATA_OFFSET        0x0028  /* Data register */
#define SAM_SPI_DBGCTRL_OFFSET     0x0030  /* Debug control register */

/* SPI register addresses ***************************************************/

#define SAM_SPI0_CTRLA             (SAM_SERCOM0_BASE + SAM_SPI_CTRLA_OFFSET)
#define SAM_SPI0_CTRLB             (SAM_SERCOM0_BASE + SAM_SPI_CTRLB_OFFSET)
#define SAM_SPI0_CTRLC             (SAM_SERCOM0_BASE + SAM_SPI_CTRLC_OFFSET)
#define SAM_SPI0_BAUD              (SAM_SERCOM0_BASE + SAM_SPI_BAUD_OFFSET)
#define SAM_SPI0_INTENCLR          (SAM_SERCOM0_BASE + SAM_SPI_INTENCLR_OFFSET)
#define SAM_SPI0_INTENSET          (SAM_SERCOM0_BASE + SAM_SPI_INTENSET_OFFSET)
#define SAM_SPI0_INTFLAG           (SAM_SERCOM0_BASE + SAM_SPI_INTFLAG_OFFSET)
#define SAM_SPI0_STATUS            (SAM_SERCOM0_BASE + SAM_SPI_STATUS_OFFSET)
#define SAM_SPI0_SYNCBUSY          (SAM_SERCOM0_BASE + SAM_SPI_SYNCBUSY_OFFSET)
#define SAM_SPI0_LENGTH            (SAM_SERCOM0_BASE + SAM_SPI_LENGTH_OFFSET)
#define SAM_SPI0_ADDR              (SAM_SERCOM0_BASE + SAM_SPI_ADDR_OFFSET)
#define SAM_SPI0_DATA              (SAM_SERCOM0_BASE + SAM_SPI_DATA_OFFSET)
#define SAM_SPI0_DBGCTRL           (SAM_SERCOM0_BASE + SAM_SPI_DBGCTRL_OFFSET)

#define SAM_SPI1_CTRLA             (SAM_SERCOM1_BASE + SAM_SPI_CTRLA_OFFSET)
#define SAM_SPI1_CTRLB             (SAM_SERCOM1_BASE + SAM_SPI_CTRLB_OFFSET)
#define SAM_SPI1_CTRLC             (SAM_SERCOM1_BASE + SAM_SPI_CTRLC_OFFSET)
#define SAM_SPI1_BAUD              (SAM_SERCOM1_BASE + SAM_SPI_BAUD_OFFSET)
#define SAM_SPI1_INTENCLR          (SAM_SERCOM1_BASE + SAM_SPI_INTENCLR_OFFSET)
#define SAM_SPI1_INTENSET          (SAM_SERCOM1_BASE + SAM_SPI_INTENSET_OFFSET)
#define SAM_SPI1_INTFLAG           (SAM_SERCOM1_BASE + SAM_SPI_INTFLAG_OFFSET)
#define SAM_SPI1_STATUS            (SAM_SERCOM1_BASE + SAM_SPI_STATUS_OFFSET)
#define SAM_SPI1_SYNCBUSY          (SAM_SERCOM1_BASE + SAM_SPI_SYNCBUSY_OFFSET)
#define SAM_SPI1_LENGTH            (SAM_SERCOM1_BASE + SAM_SPI_LENGTH_OFFSET)
#define SAM_SPI1_ADDR              (SAM_SERCOM1_BASE + SAM_SPI_ADDR_OFFSET)
#define SAM_SPI1_DATA              (SAM_SERCOM1_BASE + SAM_SPI_DATA_OFFSET)
#define SAM_SPI1_DBGCTRL           (SAM_SERCOM1_BASE + SAM_SPI_DBGCTRL_OFFSET)

#define SAM_SPI2_CTRLA             (SAM_SERCOM2_BASE + SAM_SPI_CTRLA_OFFSET)
#define SAM_SPI2_CTRLB             (SAM_SERCOM2_BASE + SAM_SPI_CTRLB_OFFSET)
#define SAM_SPI2_CTRLC             (SAM_SERCOM2_BASE + SAM_SPI_CTRLC_OFFSET)
#define SAM_SPI2_BAUD              (SAM_SERCOM2_BASE + SAM_SPI_BAUD_OFFSET)
#define SAM_SPI2_INTENCLR          (SAM_SERCOM2_BASE + SAM_SPI_INTENCLR_OFFSET)
#define SAM_SPI2_INTENSET          (SAM_SERCOM2_BASE + SAM_SPI_INTENSET_OFFSET)
#define SAM_SPI2_INTFLAG           (SAM_SERCOM2_BASE + SAM_SPI_INTFLAG_OFFSET)
#define SAM_SPI2_STATUS            (SAM_SERCOM2_BASE + SAM_SPI_STATUS_OFFSET)
#define SAM_SPI2_SYNCBUSY          (SAM_SERCOM2_BASE + SAM_SPI_SYNCBUSY_OFFSET)
#define SAM_SPI2_LENGTH            (SAM_SERCOM2_BASE + SAM_SPI_LENGTH_OFFSET)
#define SAM_SPI2_ADDR              (SAM_SERCOM2_BASE + SAM_SPI_ADDR_OFFSET)
#define SAM_SPI2_DATA              (SAM_SERCOM2_BASE + SAM_SPI_DATA_OFFSET)
#define SAM_SPI2_DBGCTRL           (SAM_SERCOM2_BASE + SAM_SPI_DBGCTRL_OFFSET)

#define SAM_SPI3_CTRLA             (SAM_SERCOM3_BASE + SAM_SPI_CTRLA_OFFSET)
#define SAM_SPI3_CTRLB             (SAM_SERCOM3_BASE + SAM_SPI_CTRLB_OFFSET)
#define SAM_SPI3_CTRLC             (SAM_SERCOM3_BASE + SAM_SPI_CTRLC_OFFSET)
#define SAM_SPI3_BAUD              (SAM_SERCOM3_BASE + SAM_SPI_BAUD_OFFSET)
#define SAM_SPI3_INTENCLR          (SAM_SERCOM3_BASE + SAM_SPI_INTENCLR_OFFSET)
#define SAM_SPI3_INTENSET          (SAM_SERCOM3_BASE + SAM_SPI_INTENSET_OFFSET)
#define SAM_SPI3_INTFLAG           (SAM_SERCOM3_BASE + SAM_SPI_INTFLAG_OFFSET)
#define SAM_SPI3_STATUS            (SAM_SERCOM3_BASE + SAM_SPI_STATUS_OFFSET)
#define SAM_SPI3_SYNCBUSY          (SAM_SERCOM3_BASE + SAM_SPI_SYNCBUSY_OFFSET)
#define SAM_SPI3_LENGTH            (SAM_SERCOM3_BASE + SAM_SPI_LENGTH_OFFSET)
#define SAM_SPI3_ADDR              (SAM_SERCOM3_BASE + SAM_SPI_ADDR_OFFSET)
#define SAM_SPI3_DATA              (SAM_SERCOM3_BASE + SAM_SPI_DATA_OFFSET)
#define SAM_SPI3_DBGCTRL           (SAM_SERCOM3_BASE + SAM_SPI_DBGCTRL_OFFSET)

#define SAM_SPI4_CTRLA             (SAM_SERCOM4_BASE + SAM_SPI_CTRLA_OFFSET)
#define SAM_SPI4_CTRLB             (SAM_SERCOM4_BASE + SAM_SPI_CTRLB_OFFSET)
#define SAM_SPI4_CTRLC             (SAM_SERCOM4_BASE + SAM_SPI_CTRLC_OFFSET)
#define SAM_SPI4_BAUD              (SAM_SERCOM4_BASE + SAM_SPI_BAUD_OFFSET)
#define SAM_SPI4_INTENCLR          (SAM_SERCOM4_BASE + SAM_SPI_INTENCLR_OFFSET)
#define SAM_SPI4_INTENSET          (SAM_SERCOM4_BASE + SAM_SPI_INTENSET_OFFSET)
#define SAM_SPI4_INTFLAG           (SAM_SERCOM4_BASE + SAM_SPI_INTFLAG_OFFSET)
#define SAM_SPI4_STATUS            (SAM_SERCOM4_BASE + SAM_SPI_STATUS_OFFSET)
#define SAM_SPI4_SYNCBUSY          (SAM_SERCOM4_BASE + SAM_SPI_SYNCBUSY_OFFSET)
#define SAM_SPI4_LENGTH            (SAM_SERCOM4_BASE + SAM_SPI_LENGTH_OFFSET)
#define SAM_SPI4_ADDR              (SAM_SERCOM4_BASE + SAM_SPI_ADDR_OFFSET)
#define SAM_SPI4_DATA              (SAM_SERCOM4_BASE + SAM_SPI_DATA_OFFSET)
#define SAM_SPI4_DBGCTRL           (SAM_SERCOM4_BASE + SAM_SPI_DBGCTRL_OFFSET)

#define SAM_SPI5_CTRLA             (SAM_SERCOM5_BASE + SAM_SPI_CTRLA_OFFSET)
#define SAM_SPI5_CTRLB             (SAM_SERCOM5_BASE + SAM_SPI_CTRLB_OFFSET)
#define SAM_SPI5_BAUD              (SAM_SERCOM5_BASE + SAM_SPI_BAUD_OFFSET)
#define SAM_SPI5_CTRLC             (SAM_SERCOM5_BASE + SAM_SPI_CTRLC_OFFSET)
#define SAM_SPI5_INTENCLR          (SAM_SERCOM5_BASE + SAM_SPI_INTENCLR_OFFSET)
#define SAM_SPI5_INTENSET          (SAM_SERCOM5_BASE + SAM_SPI_INTENSET_OFFSET)
#define SAM_SPI5_INTFLAG           (SAM_SERCOM5_BASE + SAM_SPI_INTFLAG_OFFSET)
#define SAM_SPI5_STATUS            (SAM_SERCOM5_BASE + SAM_SPI_STATUS_OFFSET)
#define SAM_SPI5_SYNCBUSY          (SAM_SERCOM5_BASE + SAM_SPI_SYNCBUSY_OFFSET)
#define SAM_SPI5_LENGTH            (SAM_SERCOM5_BASE + SAM_SPI_LENGTH_OFFSET)
#define SAM_SPI5_ADDR              (SAM_SERCOM5_BASE + SAM_SPI_ADDR_OFFSET)
#define SAM_SPI5_DATA              (SAM_SERCOM5_BASE + SAM_SPI_DATA_OFFSET)
#define SAM_SPI5_DBGCTRL           (SAM_SERCOM5_BASE + SAM_SPI_DBGCTRL_OFFSET)

#define SAM_SPI6_CTRLA             (SAM_SERCOM6_BASE + SAM_SPI_CTRLA_OFFSET)
#define SAM_SPI6_CTRLB             (SAM_SERCOM6_BASE + SAM_SPI_CTRLB_OFFSET)
#define SAM_SPI6_BAUD              (SAM_SERCOM6_BASE + SAM_SPI_BAUD_OFFSET)
#define SAM_SPI6_CTRLC             (SAM_SERCOM6_BASE + SAM_SPI_CTRLC_OFFSET)
#define SAM_SPI6_INTENCLR          (SAM_SERCOM6_BASE + SAM_SPI_INTENCLR_OFFSET)
#define SAM_SPI6_INTENSET          (SAM_SERCOM6_BASE + SAM_SPI_INTENSET_OFFSET)
#define SAM_SPI6_INTFLAG           (SAM_SERCOM6_BASE + SAM_SPI_INTFLAG_OFFSET)
#define SAM_SPI6_STATUS            (SAM_SERCOM6_BASE + SAM_SPI_STATUS_OFFSET)
#define SAM_SPI6_SYNCBUSY          (SAM_SERCOM6_BASE + SAM_SPI_SYNCBUSY_OFFSET)
#define SAM_SPI6_LENGTH            (SAM_SERCOM6_BASE + SAM_SPI_LENGTH_OFFSET)
#define SAM_SPI6_ADDR              (SAM_SERCOM6_BASE + SAM_SPI_ADDR_OFFSET)
#define SAM_SPI6_DATA              (SAM_SERCOM6_BASE + SAM_SPI_DATA_OFFSET)
#define SAM_SPI6_DBGCTRL           (SAM_SERCOM6_BASE + SAM_SPI_DBGCTRL_OFFSET)

#define SAM_SPI7_CTRLA             (SAM_SERCOM7_BASE + SAM_SPI_CTRLA_OFFSET)
#define SAM_SPI7_CTRLB             (SAM_SERCOM7_BASE + SAM_SPI_CTRLB_OFFSET)
#define SAM_SPI7_BAUD              (SAM_SERCOM7_BASE + SAM_SPI_BAUD_OFFSET)
#define SAM_SPI7_CTRLC             (SAM_SERCOM7_BASE + SAM_SPI_CTRLC_OFFSET)
#define SAM_SPI7_INTENCLR          (SAM_SERCOM7_BASE + SAM_SPI_INTENCLR_OFFSET)
#define SAM_SPI7_INTENSET          (SAM_SERCOM7_BASE + SAM_SPI_INTENSET_OFFSET)
#define SAM_SPI7_INTFLAG           (SAM_SERCOM7_BASE + SAM_SPI_INTFLAG_OFFSET)
#define SAM_SPI7_STATUS            (SAM_SERCOM7_BASE + SAM_SPI_STATUS_OFFSET)
#define SAM_SPI7_SYNCBUSY          (SAM_SERCOM7_BASE + SAM_SPI_SYNCBUSY_OFFSET)
#define SAM_SPI7_LENGTH            (SAM_SERCOM7_BASE + SAM_SPI_LENGTH_OFFSET)
#define SAM_SPI7_ADDR              (SAM_SERCOM7_BASE + SAM_SPI_ADDR_OFFSET)
#define SAM_SPI7_DATA              (SAM_SERCOM7_BASE + SAM_SPI_DATA_OFFSET)
#define SAM_SPI7_DBGCTRL           (SAM_SERCOM7_BASE + SAM_SPI_DBGCTRL_OFFSET)

/* SPI register bit definitions *********************************************/

/* Control A register */

#define SPI_CTRLA_SWRST            (1 << 0)  /* Bit 0:  Software reset */
#define SPI_CTRLA_ENABLE           (1 << 1)  /* Bit 1:  Enable */
#define SPI_CTRLA_MODE_SHIFT       (2)       /* Bits 2-4: Operating Mode */
#define SPI_CTRLA_MODE_MASK        (7 << SPI_CTRLA_MODE_SHIFT)
#  define SPI_CTRLA_MODE_SLAVE     (2 << SPI_CTRLA_MODE_SHIFT) /* SPI slave operation */
#  define SPI_CTRLA_MODE_MASTER    (3 << SPI_CTRLA_MODE_SHIFT) /* SPI master operation */

#define SPI_CTRLA_RUNSTDBY         (1 << 7)  /* Bit 7:  Run in standby */
#define SPI_CTRLA_IBON             (1 << 8)  /* Bit 8:  Immediate BUFOVF notification */
#define SPI_CTRLA_DOPO_SHIFT       (16)      /* Bit 16-17: Data out pinout */

#define SPI_CTRLA_DOPO_MASK        (3 << SPI_CTRLA_DOPO_SHIFT) /* Bit 16-17: Data out pinout */
#  define SPI_CTRLA_DOPO_DOPAD012  (0 << SPI_CTRLA_DOPO_SHIFT) /* D0=PAD0 SCK=PAD1 SS=PAD2 */
#  define SPI_CTRLA_DOPO_DOPAD312  (2 << SPI_CTRLA_DOPO_SHIFT) /* D0=PAD3 SCK=PAD1 SS=PAD2 */

#define SPI_CTRLA_DIPO_SHIFT       (20)      /* Bits 20-21: Data in pinout */
#define SPI_CTRLA_DIPO_MASK        (3 << SPI_CTRLA_DIPO_SHIFT)
#  define SPI_CTRLA_DIPAD0         (0 << SPI_CTRLA_DIPO_SHIFT) /* SERCOM PAD0 for DI */
#  define SPI_CTRLA_DIPAD1         (1 << SPI_CTRLA_DIPO_SHIFT) /* SERCOM PAD1 for DI */
#  define SPI_CTRLA_DIPAD2         (2 << SPI_CTRLA_DIPO_SHIFT) /* SERCOM PAD2 for DI */
#  define SPI_CTRLA_DIPAD3         (3 << SPI_CTRLA_DIPO_SHIFT) /* SERCOM PAD3 for DI */

#define SPI_CTRLA_FORM_SHIFT       (24)      /* Bits 24-27: Frame format */
#define SPI_CTRLA_FORM_MASK        (7 << SPI_CTRLA_FORM_SHIFT)
#  define SPI_CTRLA_FORM_SPI       (0 << SPI_CTRLA_FORM_SHIFT) /* SPI frame (no address) */
#  define SPI_CTRLA_FORM_ADDR      (2 << SPI_CTRLA_FORM_SHIFT) /* SPI frame (w/address) */

#define SPI_CTRLA_CPHA             (1 << 28)  /* Bit 28: Clock phase */
#define SPI_CTRLA_CPOL             (1 << 29)  /* Bit 29: Clock polarity */
#define SPI_CTRLA_DORD             (1 << 30)  /* Bit 30: Data order */
#  define SPI_CTRLA_MSBFIRST       (0)
#  define SPI_CTRLA_LSBFIRST       SPI_CTRLA_DORD

/* Control B register */

#define SPI_CTRLB_CHSIZE_SHIFT     (0)       /* Bits 0-2: Character Size */
#define SPI_CTRLB_CHSIZE_MASK      (7 << SPI_CTRLB_CHSIZE_SHIFT)
#  define SPI_CTRLB_CHSIZE_8BITS   (0 << SPI_CTRLB_CHSIZE_SHIFT) /* 8 bits */
#  define SPI_CTRLB_CHSIZE_9BITS   (1 << SPI_CTRLB_CHSIZE_SHIFT) /* 9 bits */

#define SPI_CTRLB_PLOADEN          (1 << 6)  /* Bit 6:  Slave Data Preload Enable */
#define SPI_CTRLB_SSDE             (1 << 9)  /* Bit 9:  Slave select low detect enable */
#define SPI_CTRLB_MSSEN            (1 << 13) /* Bit 13: Master slave select enable */
#define SPI_CTRLB_AMODE_SHIFT      (14)      /* Bits 14-15: Address Mode */
#define SPI_CTRLB_AMODE_MASK       (3 << SPI_CTRLB_AMODE_SHIFT)
#  define SPI_CTRLB_AMODE_ADDRMASK (0 << SPI_CTRLB_AMODE_SHIFT) /* ADDRMASK used to mask ADDR */
#  define SPI_CTRLB_AMODE_2ADDRS   (1 << SPI_CTRLB_AMODE_SHIFT) /* Slave 2 addresses: ADDR & ADDRMASK */
#  define SPI_CTRLB_AMODE_RANGE    (2 << SPI_CTRLB_AMODE_SHIFT) /* Slave range of addresses: ADDRMASK-ADDR */

#define SPI_CTRLB_RXEN             (1 << 17)  /* Bit 17: Receiver enable */

/* Control C register */
#
#define SPI_CTRLC_ICSPACE_SHIFT      (0)      /* Bits 0-2: Inter-Character Spacing */
#define SPI_CTRLC_ICSPACE_MASK       (7 << SPI_CTRLC_ICSPACE_SHIFT)
#  define SPI_CTRLC_ICSPACE_DISABLE  (0 << SPI_CTRLC_ICSPACE_SHIFT)
#  define SPI_CTRLC_ICSPACE(n)       ((uint32_t)(n) << SPI_CTRLC_ICSPACE_SHIFT)
#define SPI_CTRLC_DATA32B          (1 << 24   /* Bit 24: Data 32 Bit */

#  define SPI_CTRLC_DATA32B_8BIT   (0)               /* DATA register is 8-bit */
#  define SPI_CTRLC_DATA32B_32BIT  SPI_CTRLC_DATA32B /* DATA register is 32-bit */

/* Baud register (8-bit baud value) */

/* Interrupt enable clear, interrupt enable set, interrupt enable set,
 * interrupt flag and status clear registers.
 */

#define SPI_INT_DRE                (1 << 0)  /* Bit 0:  Data register empty interrupt */
#define SPI_INT_TXC                (1 << 1)  /* Bit 1:  Transmit complete interrupt */
#define SPI_INT_RXC                (1 << 2)  /* Bit 2:  Receive complete interrupt */
#define SPI_INT_SSL                (1 << 3)  /* Bit 3:  Slave select low interrupt */
#define SPI_INT_ERROR              (1 << 7)  /* Bit 7:  Error interrupt */

#define SPI_INT_ALL                (0x8f)

/* Status register */

#define SPI_STATUS_BUFOVF          (1 << 2)  /* Bit 2:  Buffer overflow */
#define SPI_STATUS_LENERR          (1 << 11) /* Bit 11: Transaction Length Error */

#define SPI_STATUS_CLRALL          (SPI_STATUS_BUFOVF | SPI_STATUS_LENERR)

/* Synchronization busy register */

#define SPI_SYNCBUSY_SWRST         (1 << 0)  /* Bit 0:  Software reset synchronization busy */
#define SPI_SYNCBUSY_ENABLE        (1 << 1)  /* Bit 1:  SERCOM enable synchronization busy */
#define SPI_SYNCBUSY_CTRLB         (1 << 2)  /* Bit 2:  CTRLB synchronization busy */
#define SPI_SYNCBUSY_LENGTH        (1 << 4)  /* Bit 4:  LENGTH synchronization busy */

#define SPI_SYNCBUSY_ALL           0x017

/* Length register */

#define SPI_LENGTH_LEN_SHIFT       (0)       /* Bit 0-7: Data length enable */
#define SPI_LENGTH_LEN_MASK        (0xff << SPI_LENGTH_LEN_SHIFT)
#  define SPI_LENGTHLEN(n)         ((uint32_t)(n) << SPI_LENGTH_LEN_SHIFT)
#define SPI_LENGTH_LENEN           (1 << 8)  /* Bit 8: Data Length Enable */

/* Address register */

#define SPI_ADDR_SHIFT             (0)       /* Bits 0-7: Address */
#define SPI_ADDR_MASK              (0xff << SPI_ADDR_SHIFT)
#  define SPI_ADDR(n)              ((uint32_t)(n) << SPI_ADDR_SHIFT)
#define SPI_ADDRMASK_SHIFT         (16)      /* Bits 16-23: Address Mask */
#define SPI_ADDRMASK_MASK          (0xff << SPI_ADDRMASK_SHIFT)
#  define SPI_ADDRMASK(n)          ((uint32_t)(n) << SPI_ADDRMASK_SHIFT)

/* Data register (8-, 9- or 32-bit data) */

#define SPI_DATA_MASK9             (0x1ff)   /* Bits 0-8: 8- or 9-bit Data */

/* Debug control register */

#define SPI_DBGCTRL_DBGSTOP        (1 << 0)  /* Bit 0: Debug stop mode */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_SAMD5E5_HARDWARE_SAM_SPI_H */
