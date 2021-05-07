/****************************************************************************
 * arch/arm/src/sama5/hardware/sam_dbgu.h
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

#ifndef __ARCH_ARM_SRC_SAMA5_HARDWARE_SAM_DBGU_H
#define __ARCH_ARM_SRC_SAMA5_HARDWARE_SAM_DBGU_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "hardware/sam_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* DBGU register offsets ****************************************************/

#define SAM_DBGU_CR_OFFSET           0x0000 /* Control Register */
#define SAM_DBGU_MR_OFFSET           0x0004 /* Mode Register */
#define SAM_DBGU_IER_OFFSET          0x0008 /* Interrupt Enable Register */
#define SAM_DBGU_IDR_OFFSET          0x000c /* Interrupt Disable Register */
#define SAM_DBGU_IMR_OFFSET          0x0010 /* Interrupt Mask Register */
#define SAM_DBGU_SR_OFFSET           0x0014 /* [Channel] Status Register */
#define SAM_DBGU_RHR_OFFSET          0x0018 /* Receive Holding Register */
#define SAM_DBGU_THR_OFFSET          0x001c /* Transmit Holding Register */
#define SAM_DBGU_BRGR_OFFSET         0x0020 /* Baud Rate Generator Register */
                                            /* 0x0024-0x003c: Reserved */
#define SAM_DBGU_CIDR_OFFSET         0x0040 /* Chip ID Register */
#define SAM_DBGU_EXID_OFFSET         0x0044 /* Chip ID Extension Register */
#define SAM_DBGU_FNR_OFFSET          0x0048 /* Force NTRST Register */
                                            /* 0x004c-0x00fc: Reserved */

/* DBGU register addresses **************************************************/

#define SAM_DBGU_CR                  (SAM_DBGU_VBASE+SAM_DBGU_CR_OFFSET)
#define SAM_DBGU_MR                  (SAM_DBGU_VBASE+SAM_DBGU_MR_OFFSET)
#define SAM_DBGU_IER                 (SAM_DBGU_VBASE+SAM_DBGU_IER_OFFSET)
#define SAM_DBGU_IDR                 (SAM_DBGU_VBASE+SAM_DBGU_IDR_OFFSET)
#define SAM_DBGU_IMR                 (SAM_DBGU_VBASE+SAM_DBGU_IMR_OFFSET)
#define SAM_DBGU_SR                  (SAM_DBGU_VBASE+SAM_DBGU_SR_OFFSET)
#define SAM_DBGU_RHR                 (SAM_DBGU_VBASE+SAM_DBGU_RHR_OFFSET)
#define SAM_DBGU_THR                 (SAM_DBGU_VBASE+SAM_DBGU_THR_OFFSET)
#define SAM_DBGU_BRGR                (SAM_DBGU_VBASE+SAM_DBGU_BRGR_OFFSET)
#define SAM_DBGU_CIDR                (SAM_DBGU_VBASE+SAM_DBGU_CIDR_OFFSET)
#define SAM_DBGU_EXID                (SAM_DBGU_VBASE+SAM_DBGU_EXID_OFFSET)
#define SAM_DBGU_FNR                 (SAM_DBGU_VBASE+SAM_DBGU_FNR_OFFSET)

/* DBGU register bit definitions ********************************************/

/* DBGU Control Register */

#define DBGU_CR_RSTRX                (1 << 2)  /* Bit 2:  Reset Receiver */
#define DBGU_CR_RSTTX                (1 << 3)  /* Bit 3:  Reset Transmitter */
#define DBGU_CR_RXEN                 (1 << 4)  /* Bit 4:  Receiver Enable */
#define DBGU_CR_RXDIS                (1 << 5)  /* Bit 5:  Receiver Disable */
#define DBGU_CR_TXEN                 (1 << 6)  /* Bit 6:  Transmitter Enable */
#define DBGU_CR_TXDIS                (1 << 7)  /* Bit 7:  Transmitter Disable */
#define DBGU_CR_RSTSTA               (1 << 8)  /* Bit 8:  Reset Status Bits */

/* DBGU Mode Register */

#ifdef ATSAMA5D4
#  define DBGU_MR_FILTER_SHIFT       (1 << 4)  /* Bit 4: FILTER: Receiver Digital Filter */
#endif

#define DBGU_MR_PAR_SHIFT            (9)       /* Bits 9-11: Parity Type */
#define DBGU_MR_PAR_MASK             (7 << DBGU_MR_PAR_SHIFT)
#  define DBGU_MR_PAR_EVEN           (0 << DBGU_MR_PAR_SHIFT) /* Even parity */
#  define DBGU_MR_PAR_ODD            (1 << DBGU_MR_PAR_SHIFT) /* Odd parity */
#  define DBGU_MR_PAR_SPACE          (2 << DBGU_MR_PAR_SHIFT) /* Space: parity forced to 0 */
#  define DBGU_MR_PAR_MARK           (3 << DBGU_MR_PAR_SHIFT) /* Mark: parity forced to 1 */
#  define DBGU_MR_PAR_NONE           (4 << DBGU_MR_PAR_SHIFT) /* No parity */

#define DBGU_MR_CHMODE_SHIFT         (14)      /* Bits 14-15: Channel Mode */
#define DBGU_MR_CHMODE_MASK          (3 << DBGU_MR_CHMODE_SHIFT)
#  define DBGU_MR_CHMODE_NORMAL      (0 << DBGU_MR_CHMODE_SHIFT) /* Normal Mode */
#  define DBGU_MR_CHMODE_ECHO        (1 << DBGU_MR_CHMODE_SHIFT) /* Automatic Echo */
#  define DBGU_MR_CHMODE_LLPBK       (2 << DBGU_MR_CHMODE_SHIFT) /* Local Loopback */
#  define DBGU_MR_CHMODE_RLPBK       (3 << DBGU_MR_CHMODE_SHIFT) /* Remote Loopback */

/* DBGU Interrupt Enable Register,
 * DBGU Interrupt Disable Register, DBGU Interrupt Mask
 * Register, and DBGU Status Register common bit field definitions
 */

#define DBGU_INT_RXRDY               (1 << 0)  /* Bit 0:  RXRDY Interrupt */
#define DBGU_INT_TXRDY               (1 << 1)  /* Bit 1:  TXRDY Interrupt */
#define DBGU_INT_OVRE                (1 << 5)  /* Bit 5:  Overrun Error Interrupt */
#define DBGU_INT_FRAME               (1 << 6)  /* Bit 6:  Framing Error Interrupt */
#define DBGU_INT_PARE                (1 << 7)  /* Bit 7:  Parity Error Interrupt */
#define DBGU_INT_TXEMPTY             (1 << 9)  /* Bit 9:  TXEMPTY Interrupt */
#define DBGU_INT_COMMTX              (1 << 30) /* Bit 30: COMMTX (from ARM) Interrupt */
#define DBGU_INT_COMMRX              (1 << 31) /* Bit 31: COMMRX (from ARM) Interrupt */

#define DBGU_INT_ALLINTS             (0xc00002e3)

/* DBGU Receiver Holding Register */

#define DBGU_RHR_RXCHR_SHIFT         (0)       /* Bits 0-7: Received Character */
#define DBGU_RHR_RXCHR_MASK          (0xff << DBGU_RHR_RXCHR_SHIFT)

/* DBGU Transmit Holding Register */

#define DBGU_THR_TXCHR_SHIFT         (0)       /* Bits 0-7: Character to be Transmitted (DBGU only) */
#define DBGU_THR_TXCHR_MASK          (0xff << DBGU_THR_TXCHR_SHIFT)

/* DBGU Baud Rate Generator Register */

#define DBGU_BRGR_CD_SHIFT           (0)      /* Bits 0-15: Clock Divisor */
#define DBGU_BRGR_CD_MASK            (0xffff << DBGU_BRGR_CD_SHIFT)
#  define DBGU_BRGR_CD_DISABLE       (0 << DBGU_BRGR_CD_SHIFT)
#  define DBGU_BRGR_CD(n)            ((uint32_t)(n) << DBGU_BRGR_CD_SHIFT)

/* Chip ID Register */

#define DBGU_CIDR_VERSION_SHIFT      (0)      /* Bits 0-4: Version of the Device */
#define DBGU_CIDR_VERSION_MASK       (31 << DBGU_CIDR_VERSION_SHIFT)
#define DBGU_CIDR_EPROC_SHIFT        (5)      /* Bits 5-7: Embedded Processor */
#define DBGU_CIDR_EPROC_MASK         (7 << DBGU_CIDR_EPROC_SHIFT)
#  define DBGU_CIDR_EPROC_ARM946ES   (1 << DBGU_CIDR_EPROC_SHIFT) /* ARM946ES */
#  define DBGU_CIDR_EPROC_ARM7TDMI   (2 << DBGU_CIDR_EPROC_SHIFT) /* ARM7TDMI */
#  define DBGU_CIDR_EPROC_CM3        (3 << DBGU_CIDR_EPROC_SHIFT) /* Cortex-M3 */
#  define DBGU_CIDR_EPROC_ARM920T    (4 << DBGU_CIDR_EPROC_SHIFT) /* ARM920T */
#  define DBGU_CIDR_EPROC_ARM926EJS  (5 << DBGU_CIDR_EPROC_SHIFT) /* ARM926EJS */
#  define DBGU_CIDR_EPROC_CA5        (6 << DBGU_CIDR_EPROC_SHIFT) /* Cortex-A5 */

#define DBGU_CIDR_NVPSIZ_SHIFT       (8)      /* Bits 8-11: Nonvolatile Program Memory Size */
#define DBGU_CIDR_NVPSIZ_MASK        (15 << DBGU_CIDR_NVPSIZ_SHIFT)
#  define DBGU_CIDR_NVPSIZ_NONE      (0 << DBGU_CIDR_NVPSIZ_SHIFT)  /* None */
#  define DBGU_CIDR_NVPSIZ_8K        (1 << DBGU_CIDR_NVPSIZ_SHIFT)  /* 8 Kbytes */
#  define DBGU_CIDR_NVPSIZ_16K       (2 << DBGU_CIDR_NVPSIZ_SHIFT)  /* 16 Kbytes */
#  define DBGU_CIDR_NVPSIZ_32K       (3 << DBGU_CIDR_NVPSIZ_SHIFT)  /* 32 Kbytes */
#  define DBGU_CIDR_NVPSIZ_64K       (5 << DBGU_CIDR_NVPSIZ_SHIFT)  /* 64 Kbytes */
#  define DBGU_CIDR_NVPSIZ_128K      (7 << DBGU_CIDR_NVPSIZ_SHIFT)  /* 128 Kbytes */
#  define DBGU_CIDR_NVPSIZ_256K      (9 << DBGU_CIDR_NVPSIZ_SHIFT)  /* 256 Kbytes */
#  define DBGU_CIDR_NVPSIZ_512K      (10 << DBGU_CIDR_NVPSIZ_SHIFT) /* 512 Kbytes */
#  define DBGU_CIDR_NVPSIZ_1M        (12 << DBGU_CIDR_NVPSIZ_SHIFT) /* 1024 Kbytes */
#  define DBGU_CIDR_NVPSIZ_2M        (14 << DBGU_CIDR_NVPSIZ_SHIFT) /* 2048 Kbytes */

#define DBGU_CIDR_NVPSIZ2_SHIFT      (12)     /* Bits 12-15: Second Nonvolatile Program Memory Size */
#define DBGU_CIDR_NVPSIZ2_MASK       (15 << DBGU_CIDR_NVPSIZ2_SHIFT)
#  define DBGU_CIDR_NVPSIZ2_NONE     (0 << DBGU_CIDR_NVPSIZ2_SHIFT)  /* None */
#  define DBGU_CIDR_NVPSIZ2_8K       (1 << DBGU_CIDR_NVPSIZ2_SHIFT)  /* 8 Kbytes */
#  define DBGU_CIDR_NVPSIZ2_16K      (2 << DBGU_CIDR_NVPSIZ2_SHIFT)  /* 16 Kbytes */
#  define DBGU_CIDR_NVPSIZ2_32K      (3 << DBGU_CIDR_NVPSIZ2_SHIFT)  /* 32 Kbytes */
#  define DBGU_CIDR_NVPSIZ2_64K      (5 << DBGU_CIDR_NVPSIZ2_SHIFT)  /* 64 Kbytes */
#  define DBGU_CIDR_NVPSIZ2_128K     (7 << DBGU_CIDR_NVPSIZ2_SHIFT)  /* 128 Kbytes */
#  define DBGU_CIDR_NVPSIZ2_256K     (9 << DBGU_CIDR_NVPSIZ2_SHIFT)  /* 256 Kbytes */
#  define DBGU_CIDR_NVPSIZ2_512K     (10 << DBGU_CIDR_NVPSIZ2_SHIFT) /* 512 Kbytes */
#  define DBGU_CIDR_NVPSIZ2_1M       (12 << DBGU_CIDR_NVPSIZ2_SHIFT) /* 1024 Kbytes */
#  define DBGU_CIDR_NVPSIZ2_2M       (14 << DBGU_CIDR_NVPSIZ2_SHIFT) /* 2048 Kbytes */

#define DBGU_CIDR_SRAMSIZ_SHIFT      (16)     /* Bits 16-19: Internal SRAM Size */
#define DBGU_CIDR_SRAMSIZ_MASK       (15 << DBGU_CIDR_SRAMSIZ_SHIFT)
#  define DBGU_CIDR_SRAMSIZ_1K       (1 << DBGU_CIDR_SRAMSIZ_SHIFT)  /* 1 Kbytes */
#  define DBGU_CIDR_SRAMSIZ_2K       (2 << DBGU_CIDR_SRAMSIZ_SHIFT)  /* 2 Kbytes */
#  define DBGU_CIDR_SRAMSIZ_6K       (3 << DBGU_CIDR_SRAMSIZ_SHIFT)  /* 6 Kbytes */
#  define DBGU_CIDR_SRAMSIZ_112K     (4 << DBGU_CIDR_SRAMSIZ_SHIFT)  /* 112 Kbytes */
#  define DBGU_CIDR_SRAMSIZ_4K       (5 << DBGU_CIDR_SRAMSIZ_SHIFT)  /* 4 Kbytes */
#  define DBGU_CIDR_SRAMSIZ_80K      (6 << DBGU_CIDR_SRAMSIZ_SHIFT)  /* 80 Kbytes */
#  define DBGU_CIDR_SRAMSIZ_160K     (7 << DBGU_CIDR_SRAMSIZ_SHIFT)  /* 160 Kbytes */
#  define DBGU_CIDR_SRAMSIZ_8K       (8 << DBGU_CIDR_SRAMSIZ_SHIFT)  /* 8 Kbytes */
#  define DBGU_CIDR_SRAMSIZ_16K      (9 << DBGU_CIDR_SRAMSIZ_SHIFT)  /* 16 Kbytes */
#  define DBGU_CIDR_SRAMSIZ_32K      (10 << DBGU_CIDR_SRAMSIZ_SHIFT) /* 32 Kbytes */
#  define DBGU_CIDR_SRAMSIZ_64K      (11 << DBGU_CIDR_SRAMSIZ_SHIFT) /* 64 Kbytes */
#  define DBGU_CIDR_SRAMSIZ_128K     (12 << DBGU_CIDR_SRAMSIZ_SHIFT) /* 128 Kbytes */
#  define DBGU_CIDR_SRAMSIZ_256K     (13 << DBGU_CIDR_SRAMSIZ_SHIFT) /* 256 Kbytes */
#  define DBGU_CIDR_SRAMSIZ_96K      (14 << DBGU_CIDR_SRAMSIZ_SHIFT) /* 96 Kbytes */
#  define DBGU_CIDR_SRAMSIZ_512K     (15 << DBGU_CIDR_SRAMSIZ_SHIFT) /* 512 Kbytes */

#define DBGU_CIDR_ARCH_SHIFT         (20)     /* Bits 20-23: Architecture Identifier */
#define DBGU_CIDR_ARCH_MASK          (15 << DBGU_CIDR_ARCH_SHIFT)
#  define DBGU_CIDR_ARCH_AT91SAM9xx    (0x19 << DBGU_CIDR_ARCH_SHIFT) /* AT91SAM9xx Series */
#  define DBGU_CIDR_ARCH_AT91SAM9XExx  (0x29 << DBGU_CIDR_ARCH_SHIFT) /* AT91SAM9XExx Series */
#  define DBGU_CIDR_ARCH_AT91x34       (0x34 << DBGU_CIDR_ARCH_SHIFT) /* AT91x34 Series */
#  define DBGU_CIDR_ARCH_CAP7          (0x37 << DBGU_CIDR_ARCH_SHIFT) /* CAP7 Series */
#  define DBGU_CIDR_ARCH_CAP9          (0x39 << DBGU_CIDR_ARCH_SHIFT) /* CAP9 Series */
#  define DBGU_CIDR_ARCH_CAP11         (0x3b << DBGU_CIDR_ARCH_SHIFT) /* CAP11 Series */
#  define DBGU_CIDR_ARCH_AT91x40       (0x40 << DBGU_CIDR_ARCH_SHIFT) /* AT91x40 Series */
#  define DBGU_CIDR_ARCH_AT91x42       (0x42 << DBGU_CIDR_ARCH_SHIFT) /* AT91x42 Series */
#  define DBGU_CIDR_ARCH_AT91x55       (0x55 << DBGU_CIDR_ARCH_SHIFT) /* AT91x55 Series */
#  define DBGU_CIDR_ARCH_AT91SAM7Axx   (0x60 << DBGU_CIDR_ARCH_SHIFT) /* AT91SAM7Axx Series */
#  define DBGU_CIDR_ARCH_AT91SAM7AQxx  (0x61 << DBGU_CIDR_ARCH_SHIFT) /* AT91SAM7AQxx Series */
#  define DBGU_CIDR_ARCH_AT91x63       (0x63 << DBGU_CIDR_ARCH_SHIFT) /* AT91x63 Series */
#  define DBGU_CIDR_ARCH_AT91SAM7Sxx   (0x70 << DBGU_CIDR_ARCH_SHIFT) /* AT91SAM7Sxx Series */
#  define DBGU_CIDR_ARCH_AT91SAM7XCxx  (0x71 << DBGU_CIDR_ARCH_SHIFT) /* AT91SAM7XCxx Series */
#  define DBGU_CIDR_ARCH_AT91SAM7SExx  (0x72 << DBGU_CIDR_ARCH_SHIFT) /* AT91SAM7SExx Series */
#  define DBGU_CIDR_ARCH_AT91SAM7Lxx   (0x73 << DBGU_CIDR_ARCH_SHIFT) /* AT91SAM7Lxx Series */
#  define DBGU_CIDR_ARCH_AT91SAM7Xxx   (0x75 << DBGU_CIDR_ARCH_SHIFT) /* AT91SAM7Xxx Series */
#  define DBGU_CIDR_ARCH_AT91SAM7SLxx  (0x76 << DBGU_CIDR_ARCH_SHIFT) /* AT91SAM7SLxx Series */
#  define DBGU_CIDR_ARCH_ATSAM3UxC     (0x80 << DBGU_CIDR_ARCH_SHIFT) /* ATSAM3UxC Series (100-pin) */
#  define DBGU_CIDR_ARCH_ATSAM3UxE     (0x81 << DBGU_CIDR_ARCH_SHIFT) /* ATSAM3UxE Series (144-pin) */
#  define DBGU_CIDR_ARCH_ATSAM3AxC     (0x83 << DBGU_CIDR_ARCH_SHIFT) /* ATSAM3AxC Series (100-pin) */
#  define DBGU_CIDR_ARCH_ATSAM3XxC     (0x84 << DBGU_CIDR_ARCH_SHIFT) /* ATSAM3XxC Series (100-pin) */
#  define DBGU_CIDR_ARCH_ATSAM3XxE     (0x85 << DBGU_CIDR_ARCH_SHIFT) /* ATSAM3XxE Series (144-pin) */
#  define DBGU_CIDR_ARCH_ATSAM3XxG     (0x86 << DBGU_CIDR_ARCH_SHIFT) /* ATSAM3XxG Series (208/217-pin) */
#  define DBGU_CIDR_ARCH_ATSAM3SxA     (0x88 << DBGU_CIDR_ARCH_SHIFT) /* ATSAM3SxA Series (48-pin) */
#  define DBGU_CIDR_ARCH_ATSAM3SxB     (0x89 << DBGU_CIDR_ARCH_SHIFT) /* ATSAM3SxB Series (64-pin) */
#  define DBGU_CIDR_ARCH_ATSAM3SxC     (0x8a << DBGU_CIDR_ARCH_SHIFT) /* ATSAM3SxC Series (100-pin) */
#  define DBGU_CIDR_ARCH_AT91x92       (0x92 << DBGU_CIDR_ARCH_SHIFT) /* AT91x92 Series */
#  define DBGU_CIDR_ARCH_ATSAM3NxA     (0x93 << DBGU_CIDR_ARCH_SHIFT) /* ATSAM3NxA Series (48-pin) */
#  define DBGU_CIDR_ARCH_ATSAM3NxB     (0x94 << DBGU_CIDR_ARCH_SHIFT) /* ATSAM3NxB Series (64-pin) */
#  define DBGU_CIDR_ARCH_ATSAM3NxC     (0x95 << DBGU_CIDR_ARCH_SHIFT) /* ATSAM3NxC Series (100-pin) */
#  define DBGU_CIDR_ARCH_ATSAM3SDxA    (0x98 << DBGU_CIDR_ARCH_SHIFT) /* ATSAM3SDxA Series (48-pin) */
#  define DBGU_CIDR_ARCH_ATSAM3SDxB    (0x99 << DBGU_CIDR_ARCH_SHIFT) /* ATSAM3SDxB Series (64-pin) */
#  define DBGU_CIDR_ARCH_ATSAM3SDxC    (0x9a << DBGU_CIDR_ARCH_SHIFT) /* ATSAM3SDxC Series (100-pin) */
#  define DBGU_CIDR_ARCH_ATSAMA5xx     (0xa5 << DBGU_CIDR_ARCH_SHIFT) /* ATSAMA5xx Series */
#  define DBGU_CIDR_ARCH_AT75Cxx       (0xf0 << DBGU_CIDR_ARCH_SHIFT) /* AT75Cxx Series */

#define DBGU_CIDR_NVPTYP_SHIFT       (28)     /* Bits 28-30: Nonvolatile Program Memory Type */
#define DBGU_CIDR_NVPTYP_MASK        (7 << DBGU_CIDR_NVPTYP_SHIFT)
#  define DBGU_CIDR_NVPTYP_ROM       (0 << DBGU_CIDR_NVPTYP_SHIFT) /* ROM */
#  define DBGU_CIDR_NVPTYP_ROMLESS   (1 << DBGU_CIDR_NVPTYP_SHIFT) /* ROMless or on-chip Flash */
#  define DBGU_CIDR_NVPTYP_SRAM      (4 << DBGU_CIDR_NVPTYP_SHIFT) /* SRAM emulating ROM */
#  define DBGU_CIDR_NVPTYP_FLASH     (2 << DBGU_CIDR_NVPTYP_SHIFT) /* Embedded Flash Memory */
#  define DBGU_CIDR_NVPTYP_ROMFLASH  (3 << DBGU_CIDR_NVPTYP_SHIFT) /* ROM and Embedded Flash Memory */

#define DBGU_CIDR_EXT                (1 << 31) /* Bit 31:  Extension Flag */

/* Chip ID Extension Register (32-bit ID */

/* Force NTRST Register */

#define DBGU_FNR_FNTRST              (1 << 0)  /* Bit 0:  Force NTRST */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_SAMA5_HARDWARE_SAM_DBGU_H */
