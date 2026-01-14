/****************************************************************************
 * arch/arm/src/stm32h5/hardware/stm32_qspi.h
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

#ifndef __ARCH_ARM_SRC_STM32H5_HARDWARE_STM32_QUADSPI_H
#define __ARCH_ARM_SRC_STM32H5_HARDWARE_STM32_QUADSPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <arch/stm32h5/chip.h>

#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* General Characteristics **************************************************/

#define STM32H5_QSPI_MINBITS          8         /* Minimum word width */
#define STM32H5_QSPI_MAXBITS          32        /* Maximum word width */

/* QSPI register offsets ****************************************************/

#define STM32_QUADSPI_CR_OFFSET       0x0000    /* Control Register */
#define STM32_QUADSPI_DCR1_OFFSET     0x0008    /* Device Configuration Register 1 */
#define STM32_QUADSPI_DCR2_OFFSET     0x000c    /* Device Configuration Register 2 */
#define STM32_QUADSPI_DCR3_OFFSET     0x0010    /* Device Configuration Register 3 */
#define STM32_QUADSPI_DCR4_OFFSET     0x0014    /* Device Configuration Register 4 */
#define STM32_QUADSPI_SR_OFFSET       0x0020    /* Status Register */
#define STM32_QUADSPI_FCR_OFFSET      0x0024    /* Flag Clear Register */
#define STM32_QUADSPI_DLR_OFFSET      0x0040    /* Data Length Register */
#define STM32_QUADSPI_AR_OFFSET       0x0048    /* Address Register */
#define STM32_QUADSPI_DR_OFFSET       0x0050    /* Data Register */
#define STM32_QUADSPI_PSMKR_OFFSET    0x0080    /* Polling Status mask Register */
#define STM32_QUADSPI_PSMAR_OFFSET    0x0088    /* Polling Status match Register */
#define STM32_QUADSPI_PIR_OFFSET      0x0090    /* Polling Interval Register */
#define STM32_QUADSPI_CCR_OFFSET      0x0100    /* Communication Configuration Register */
#define STM32_QUADSPI_TCR_OFFSET      0x0108    /* Timing Configuration Register */
#define STM32_QUADSPI_IR_OFFSET       0x0110    /* Instruction Register */
#define STM32_QUADSPI_ABR_OFFSET      0x0120    /* Alternate Bytes Register */
#define STM32_QUADSPI_LPTR_OFFSET     0x0130    /* Low-Power Timeout Register */
#define STM32_QUADSPI_WPCCR_OFFSET    0x0140    /* Wrap Communication Configuration Register */
#define STM32_QUADSPI_WPTCR_OFFSET    0x0148    /* Wrap Timing Configuration Register */
#define STM32_QUADSPI_WPIR_OFFSET     0x0150    /* Wrap Instruction Register */
#define STM32_QUADSPI_WPABR_OFFSET    0x0160    /* Wrap Alternate Bytes Register */
#define STM32_QUADSPI_WCCR_OFFSET     0x0180    /* Write Communication Configuration Register */
#define STM32_QUADSPI_WTCR_OFFSET     0x0188    /* Write Configuration Register */
#define STM32_QUADSPI_WIR_OFFSET      0x0190    /* Write Instruction Register */
#define STM32_QUADSPI_WABR_OFFSET     0x01a0    /* Write Alternate Bytes Register */
#define STM32_QUADSPI_HLCR_OFFSET     0x0200    /* HyperBus Latency Configuration Register */

/* QSPI register addresses **************************************************/

#define STM32_QUADSPI_CR     (STM32_OCTOSPI1_BASE+STM32_QUADSPI_CR_OFFSET)     /* Control Register */
#define STM32_QUADSPI_DCR1   (STM32_OCTOSPI1_BASE+STM32_QUADSPI_DCR1_OFFSET)   /* Device Configuration Register 1 */
#define STM32_QUADSPI_DCR2   (STM32_OCTOSPI1_BASE+STM32_QUADSPI_DCR2_OFFSET)   /* Device Configuration Register 2 */
#define STM32_QUADSPI_DCR3   (STM32_OCTOSPI1_BASE+STM32_QUADSPI_DCR3_OFFSET)   /* Device Configuration Register 3 */
#define STM32_QUADSPI_DCR4   (STM32_OCTOSPI1_BASE+STM32_QUADSPI_DCR4_OFFSET)   /* Device Configuration Register 4 */
#define STM32_QUADSPI_SR     (STM32_OCTOSPI1_BASE+STM32_QUADSPI_SR_OFFSET)     /* Status Register */
#define STM32_QUADSPI_FCR    (STM32_OCTOSPI1_BASE+STM32_QUADSPI_FCR_OFFSET)    /* Flag Clear Register */
#define STM32_QUADSPI_DLR    (STM32_OCTOSPI1_BASE+STM32_QUADSPI_DLR_OFFSET)    /* Data Length Register */
#define STM32_QUADSPI_AR     (STM32_OCTOSPI1_BASE+STM32_QUADSPI_AR_OFFSET)     /* Address Register */
#define STM32_QUADSPI_DR     (STM32_OCTOSPI1_BASE+STM32_QUADSPI_DR_OFFSET)     /* Data Register */
#define STM32_QUADSPI_PSMKR  (STM32_OCTOSPI1_BASE+STM32_QUADSPI_PSMKR_OFFSET)  /* Polling Status mask Register */
#define STM32_QUADSPI_PSMAR  (STM32_OCTOSPI1_BASE+STM32_QUADSPI_PSMAR_OFFSET)  /* Polling Status match Register */
#define STM32_QUADSPI_PIR    (STM32_OCTOSPI1_BASE+STM32_QUADSPI_PIR_OFFSET)    /* Polling Interval Register */
#define STM32_QUADSPI_CCR    (STM32_OCTOSPI1_BASE+STM32_QUADSPI_CCR_OFFSET)    /* Communication Configuration Register */
#define STM32_QUADSPI_TCR    (STM32_OCTOSPI1_BASE+STM32_QUADSPI_TCR_OFFSET)    /* Timing Configuration Register */
#define STM32_QUADSPI_IR     (STM32_OCTOSPI1_BASE+STM32_QUADSPI_IR_OFFSET)     /* Instruction Register */
#define STM32_QUADSPI_ABR    (STM32_OCTOSPI1_BASE+STM32_QUADSPI_ABR_OFFSET)    /* Alternate Bytes Register */
#define STM32_QUADSPI_LPTR   (STM32_OCTOSPI1_BASE+STM32_QUADSPI_LPTR_OFFSET)   /* Low-Power Timeout Register */
#define STM32_QUADSPI_WPCCR  (STM32_OCTOSPI1_BASE+STM32_QUADSPI_WPCCR_OFFSET)  /* Wrap Communication Configuration Register */
#define STM32_QUADSPI_WPTCR  (STM32_OCTOSPI1_BASE+STM32_QUADSPI_WPTCR_OFFSET)  /* Wrap Timing Configuration Register */
#define STM32_QUADSPI_WPIR   (STM32_OCTOSPI1_BASE+STM32_QUADSPI_WPIR_OFFSET)   /* Wrap Instruction Register */
#define STM32_QUADSPI_WPABR  (STM32_OCTOSPI1_BASE+STM32_QUADSPI_WPABR_OFFSET)  /* Wrap Alternate Bytes Register */
#define STM32_QUADSPI_WCCR   (STM32_OCTOSPI1_BASE+STM32_QUADSPI_WCCR_OFFSET)   /* Write Communication Configuration Register */
#define STM32_QUADSPI_WTCR   (STM32_OCTOSPI1_BASE+STM32_QUADSPI_WTCR_OFFSET)   /* Write Configuration Register */
#define STM32_QUADSPI_WIR    (STM32_OCTOSPI1_BASE+STM32_QUADSPI_WIR_OFFSET)    /* Write Instruction Register */
#define STM32_QUADSPI_WABR   (STM32_OCTOSPI1_BASE+STM32_QUADSPI_WABR_OFFSET)   /* Write Alternate Bytes Register */
#define STM32_QUADSPI_HLCR   (STM32_OCTOSPI1_BASE+STM32_QUADSPI_HLCR_OFFSET)   /* HyperBus Latency Configuration Register */

/* QSPI register bit definitions ********************************************/

/* Control Register */

#define QSPI_CR_EN                 (1 << 0)   /* Bit 0:  QSPI Enable */
#define QSPI_CR_ABORT              (1 << 1)   /* Bit 1:  Abort request */
#define QSPI_CR_DMAEN              (1 << 2)   /* Bit 2:  DMA enable */
#define QSPI_CR_TCEN               (1 << 3)   /* Bit 3:  Timeout counter enable */
#define QSPI_CR_DMM                (1 << 6)   /* Bit 6:  DMM: Dual-memory mode */
#define QSPI_CR_MSEL               (1 << 7)   /* Bit 7:  MSEL: Memory selection */
#define QSPI_CR_FTHRES_SHIFT       (8)        /* Bits 8-11: FIFO threshold level */
#define QSPI_CR_FTHRES_MASK        (0x1f << QSPI_CR_FTHRES_SHIFT)
#define QSPI_CR_TEIE               (1 << 16)  /* Bit 16:  Transfer error interrupt enable */
#define QSPI_CR_TCIE               (1 << 17)  /* Bit 17:  Transfer complete interrupt enable */
#define QSPI_CR_FTIE               (1 << 18)  /* Bit 18:  FIFO threshold interrupt enable */
#define QSPI_CR_SMIE               (1 << 19)  /* Bit 19:  Status match interrupt enable */
#define QSPI_CR_TOIE               (1 << 20)  /* Bit 20:  TimeOut interrupt enable */
#define QSPI_CR_APMS               (1 << 22)  /* Bit 22:  Automatic poll mode stop */
#define QSPI_CR_PMM                (1 << 23)  /* Bit 23:  Polling match mode */

#define QSPI_CR_FMODE_SHIFT       (28)        /* Bits 28-29: Functional mode */
#define QSPI_CR_FMODE_MASK        (0x3 << QSPI_CR_FMODE_SHIFT)
#  define QSPI_CR_FMODE(n)        ((uint32_t)(n) << QSPI_CR_FMODE_SHIFT)

#define CR_FMODE_INDWR     0   /* Indirect write mode */
#define CR_FMODE_INDRD     1   /* Indirect read mode */
#define CR_FMODE_AUTOPOLL  2   /* Automatic polling mode */
#define CR_FMODE_MEMMAP    3   /* Memory-mapped mode */

/* Device Configuration Register 1 */

#define QSPI_DCR1_CKMODE             (1 << 0)   /* Bit 0:  Mode 0 / mode 3 */
#define QSPI_DCR1_FRCK               (1 << 1)   /* Bit 1:  Free Running Clock */
#define QSPI_DCR1_DLYBYP             (1 << 2)   /* Bit 2:  Delay block bypass */

#define QSPI_DCR1_CSHT_SHIFT         (8)        /* Bits 8-10: Chip select high time */
#define QSPI_DCR1_CSHT_MASK          (0x3f << QSPI_DCR1_CSHT_SHIFT)
#define QSPI_DCR1_DEVSIZE_SHIFT      (16)       /* Bits 16-20: Flash memory size */

#define QSPI_DCR1_DEVSIZE_MASK       (0x1f << QSPI_DCR1_DEVSIZE_SHIFT)
#define QSPI_DCR1_MTYP_SHIFT         (24)       /* Bits 24-26: Memory Type */
#define QSPI_DCR1_MTYP_MASK          (0x7 << QSPI_DCR1_MTYP_SHIFT)
#define QSPI_DCR1_MTYP_MICRON_MODE   (0x0 << QSPI_DCR1_MTYP_SHIFT)
#define QSPI_DCR1_MTYP_MACRNX_MODE   (0x1 << QSPI_DCR1_MTYP_SHIFT)
#define QSPI_DCR1_MTYP_STD_MODE      (0x2 << QSPI_DCR1_MTYP_SHIFT)
#define QSPI_DCR1_MTYP_MACRNX_RAM    (0x3 << QSPI_DCR1_MTYP_SHIFT)
#define QSPI_DCR1_MTYP_HYPBUS_MEM    (0x4 << QSPI_DCR1_MTYP_SHIFT)
#define QSPI_DCR1_MTYP_HYPBUS_REG    (0x5 << QSPI_DCR1_MTYP_SHIFT)

/* Device Configuration Register 2 */

#define QSPI_DCR2_PRESCALER_SHIFT    (0)       /* Bits 0-7: Clock prescaler */
#define QSPI_DCR2_PRESCALER_MASK     (0xff << QSPI_DCR2_PRESCALER_SHIFT)

#define QSPI_DCR2_WRAPSIZE_SHIFT    (16)                              /* Bits 16-18: Wrap Size */
#define QSPI_DCR2_WRAPSIZE_MASK     (0x7 << QSPI_DCR2_WRAPSIZE_SHIFT)
#define QSPI_DCR2_WRAPSIZE_DIS      (0x0 << QSPI_DCR2_WRAPSIZE_SHIFT) /* Wrapped reads not supported */
#define QSPI_DCR2_WRAPSIZE_16B      (0x2 << QSPI_DCR2_WRAPSIZE_SHIFT) /* Wrap Size 16 bytes */
#define QSPI_DCR2_WRAPSIZE_32B      (0x3 << QSPI_DCR2_WRAPSIZE_SHIFT) /* Wrap Size 32 bytes */
#define QSPI_DCR2_WRAPSIZE_64B      (0x4 << QSPI_DCR2_WRAPSIZE_SHIFT) /* Wrap Size 64 bytes */
#define QSPI_DCR2_WRAPSIZE_128B     (0x5 << QSPI_DCR2_WRAPSIZE_SHIFT) /* Wrap Size 128 bytes */

/* Device Configuration Register 3 */

#define QSPI_DCR3_CSBOUND_SHIFT    (16)       /* Bits 16-20: Clock prescaler */
#define QSPI_DCR3_CSBOUND_MASK     (0x1f << QSPI_CR_CSBOUND_SHIFT)
#define QSPI_DCR3_CSBOUND(n)       ((n << QSPI_CR_CSBOUND_SHIFT) & OCSOSPI_DCR3_CSBOUND_MASK) /* NCS boundary = 2^n bytes when n = 1-31 */

/* Device Configuration Register 4 */

/* Status Register */

#define QSPI_SR_TEF                (1 << 0)   /* Bit 0:  Transfer error flag */
#define QSPI_SR_TCF                (1 << 1)   /* Bit 1:  Transfer complete flag */
#define QSPI_SR_FTF                (1 << 2)   /* Bit 2:  FIFO threshold flag */
#define QSPI_SR_SMF                (1 << 3)   /* Bit 3:  Status match flag */
#define QSPI_SR_TOF                (1 << 4)   /* Bit 4:  Timeout flag */
#define QSPI_SR_BUSY               (1 << 5)   /* Bit 5:  Busy */
#define QSPI_SR_FLEVEL_SHIFT       (8)        /* Bits 8-12: FIFO threshold level */
#define QSPI_SR_FLEVEL_MASK        (0x3f << QSPI_SR_FLEVEL_SHIFT)

/* Flag Clear Register */

#define QSPI_FCR_CTEF              (1 << 0)   /* Bit 0:  Clear Transfer error flag */
#define QSPI_FCR_CTCF              (1 << 1)   /* Bit 1:  Clear Transfer complete flag */
#define QSPI_FCR_CSMF              (1 << 3)   /* Bit 3:  Clear Status match flag */
#define QSPI_FCR_CTOF              (1 << 4)   /* Bit 4:  Clear Timeout flag */

/* Data Length Register */

/* Instruction Register */

/* Communication Configuration Register */

#define CCR_IMODE_NONE      0   /* No instruction */
#define CCR_IMODE_SINGLE    1   /* Instruction on a single line */
#define CCR_IMODE_DUAL      2   /* Instruction on two lines */
#define CCR_IMODE_QUAD      3   /* Instruction on four lines */

#define CCR_ADMODE_NONE     0   /* No address */
#define CCR_ADMODE_SINGLE   1   /* Address on a single line */
#define CCR_ADMODE_DUAL     2   /* Address on two lines */
#define CCR_ADMODE_QUAD     3   /* Address on four lines */

#define CCR_ADSIZE_8        0   /* 8-bit address */
#define CCR_ADSIZE_16       1   /* 16-bit address */
#define CCR_ADSIZE_24       2   /* 24-bit address */
#define CCR_ADSIZE_32       3   /* 32-bit address */

#define CCR_ABMODE_NONE     0   /* No alternate bytes */
#define CCR_ABMODE_SINGLE   1   /* Alternate bytes on a single line */
#define CCR_ABMODE_DUAL     2   /* Alternate bytes on two lines */
#define CCR_ABMODE_QUAD     3   /* Alternate bytes on four lines */

#define CCR_ABSIZE_8        0   /* 8-bit alternate byte */
#define CCR_ABSIZE_16       1   /* 16-bit alternate bytes */
#define CCR_ABSIZE_24       2   /* 24-bit alternate bytes */
#define CCR_ABSIZE_32       3   /* 32-bit alternate bytes */

#define CCR_DMODE_NONE      0   /* No data */
#define CCR_DMODE_SINGLE    1   /* Data on a single line */
#define CCR_DMODE_DUAL      2   /* Data on two lines */
#define CCR_DMODE_QUAD      3   /* Data on four lines */

#define QSPI_CCR_IMODE_SHIFT       (0)        /* Bits 0-2: Instruction mode */
#define QSPI_CCR_IMODE_MASK        (0x7 << QSPI_CCR_IMODE_SHIFT)
#  define QSPI_CCR_IMODE(n)        ((uint32_t)(n) << QSPI_CCR_IMODE_SHIFT)

#define QSPI_CCR_IDTR              (1 << 3)   /* Bit 3: Instruction double transfer rate */

#define QSPI_CCR_ISIZE_SHIFT       (4)        /* Bits 4-5: Instruction size */
#define QSPI_CCR_ISIZE_MASK        (0x3 << QSPI_CCR_ISIZE_SHIFT)
#  define QSPI_CCR_ISIZE_8b        (0 << QSPI_CCR_ISIZE_SHIFT)
#  define QSPI_CCR_ISIZE_16b       (1 << QSPI_CCR_ISIZE_SHIFT)
#  define QSPI_CCR_ISIZE_24b       (2 << QSPI_CCR_ISIZE_SHIFT)
#  define QSPI_CCR_ISIZE_32b       (3 << QSPI_CCR_ISIZE_SHIFT)

#define QSPI_CCR_ADMODE_SHIFT      (8)        /* Bits 10-11: Address mode */
#define QSPI_CCR_ADMODE_MASK       (0x7 << QSPI_CCR_ADMODE_SHIFT)
#  define QSPI_CCR_ADMODE(n)       ((uint32_t)(n) << QSPI_CCR_ADMODE_SHIFT)

#define QSPI_CCR_ADDTR             (1 << 11)   /* Bit 11: Address double transfer rate */

#define QSPI_CCR_ADSIZE_SHIFT      (12)        /* Bits 12-13: Address size */
#define QSPI_CCR_ADSIZE_MASK       (0x3 << QSPI_CCR_ADSIZE_SHIFT)
#  define QSPI_CCR_ADSIZE(n)       ((uint32_t)(n) << QSPI_CCR_ADSIZE_SHIFT)
#  define QSPI_CCR_ADSIZE_8b       (0 << QSPI_CCR_ADSIZE_SHIFT)
#  define QSPI_CCR_ADSIZE_16b      (1 << QSPI_CCR_ADSIZE_SHIFT)
#  define QSPI_CCR_ADSIZE_24b      (2 << QSPI_CCR_ADSIZE_SHIFT)
#  define QSPI_CCR_ADSIZE_32b      (3 << QSPI_CCR_ADSIZE_SHIFT)

#define QSPI_CCR_ABMODE_SHIFT      (16)        /* Bits 16-18: Alternate bytes mode */
#define QSPI_CCR_ABMODE_MASK       (0x7 << QSPI_CCR_ABMODE_SHIFT)
#  define QSPI_CCR_ABMODE(n)       ((uint32_t)(n) << QSPI_CCR_ABMODE_SHIFT)

#define QSPI_CCR_ABDTR              (1 << 19)   /* Bit 19: Alternate-byte double transfer rate */

#define QSPI_CCR_ABSIZE_SHIFT      (20)        /* Bits 20-21: Alternate bytes size */
#define QSPI_CCR_ABSIZE_MASK       (0x3 << QSPI_CCR_ABSIZE_SHIFT)
#  define QSPI_CCR_ABSIZE(n)       ((uint32_t)(n) << QSPI_CCR_ABSIZE_SHIFT)

#define QSPI_CCR_DMODE_SHIFT       (24)        /* Bits 24-25: Data mode */
#define QSPI_CCR_DMODE_MASK        (0x3 << QSPI_CCR_DMODE_SHIFT)
#  define QSPI_CCR_DMODE(n)        ((uint32_t)(n) << QSPI_CCR_DMODE_SHIFT)

#define QSPI_CCR_DDTR              (1 << 27)   /* Bit 27: Data double transfer rate */
#define QSPI_CCR_DQSE              (1 << 29)   /* Bit 29: DQS enable */

/* Timing Configuration Register */

#define QSPI_TCR_DCYC_SHIFT        (0)        /* Bits 0-4: Number of dummy cycles */
#define QSPI_TCR_DCYC_MASK         (0x1f << QSPI_TCR_DCYC_SHIFT)
#  define QSPI_TCR_DCYC(n)         ((uint32_t)(n) << QSPI_TCR_DCYC_SHIFT)

#define QSPI_TCR_DHQC              (1 << 28)   /* Bit 28: Delay hold quarter cycle */
#define QSPI_TCR_SSHIFT            (1 << 30)   /* Bit 30: Sample Shift */

/* Address Register */

/* Alternate Bytes Register */

/* Data Register */

/* Polling Status mask Register */

/* Polling Status match Register */

/* Polling Interval Register */

#define QSPI_PIR_INTERVAL_SHIFT    (0)        /* Bits 0-15: Polling interval */
#define QSPI_PIR_INTERVAL_MASK     (0xFFff << QSPI_PIR_INTERVAL_SHIFT)

/* Low-Power Timeout Register */

#define QSPI_LPTR_TIMEOUT_SHIFT    (0)        /* Bits 0-15: Timeout period */
#define QSPI_LPTR_TIMEOUT_MASK     (0xFFff << QSPI_LPTR_TIMEOUT_SHIFT)

/* Wrap Communication Configuration Register */

#define WPCCR_IMODE_NONE      0   /* No instruction */
#define WPCCR_IMODE_SINGLE    1   /* Instruction on a single line */
#define WPCCR_IMODE_DUAL      2   /* Instruction on two lines */
#define WPCCR_IMODE_QUAD      3   /* Instruction on four lines */

#define WPCCR_ADMODE_NONE     0   /* No address */
#define WPCCR_ADMODE_SINGLE   1   /* Address on a single line */
#define WPCCR_ADMODE_DUAL     2   /* Address on two lines */
#define WPCCR_ADMODE_QUAD     3   /* Address on four lines */

#define WPCCR_ADSIZE_8        0   /* 8-bit address */
#define WPCCR_ADSIZE_16       1   /* 16-bit address */
#define WPCCR_ADSIZE_24       2   /* 24-bit address */
#define WPCCR_ADSIZE_32       3   /* 32-bit address */

#define WPCCR_ABMODE_NONE     0   /* No alternate bytes */
#define WPCCR_ABMODE_SINGLE   1   /* Alternate bytes on a single line */
#define WPCCR_ABMODE_DUAL     2   /* Alternate bytes on two lines */
#define WPCCR_ABMODE_QUAD     3   /* Alternate bytes on four lines */

#define WPCCR_ABSIZE_8        0   /* 8-bit alternate byte */
#define WPCCR_ABSIZE_16       1   /* 16-bit alternate bytes */
#define WPCCR_ABSIZE_24       2   /* 24-bit alternate bytes */
#define WPCCR_ABSIZE_32       3   /* 32-bit alternate bytes */

#define WPCCR_DMODE_NONE      0   /* No data */
#define WPCCR_DMODE_SINGLE    1   /* Data on a single line */
#define WPCCR_DMODE_DUAL      2   /* Data on two lines */
#define WPCCR_DMODE_QUAD      3   /* Data on four lines */

#define QSPI_WPCCR_IMODE_SHIFT       (0)        /* Bits 0-2: Instruction mode */
#define QSPI_WPCCR_IMODE_MASK        (0x7 << QSPI_WPCCR_IMODE_SHIFT)
#  define QSPI_WPCCR_IMODE(n)        ((uint32_t)(n) << QSPI_WPCCR_IMODE_SHIFT)

#define QSPI_WPCCR_IDTR              (1 << 3)   /* Bit 3: Instruction double transfer rate */

#define QSPI_WPCCR_ISIZE_SHIFT       (4)        /* Bits 4-5: Instruction size */
#define QSPI_WPCCR_ISIZE_MASK        (0x3 << QSPI_WPCCR_ISIZE_SHIFT)
#  define QSPI_WPCCR_ISIZE_8b        (0 << QSPI_WPCCR_ISIZE_SHIFT)
#  define QSPI_WPCCR_ISIZE_16b       (1 << QSPI_WPCCR_ISIZE_SHIFT)
#  define QSPI_WPCCR_ISIZE_24b       (2 << QSPI_WPCCR_ISIZE_SHIFT)
#  define QSPI_WPCCR_ISIZE_32b       (3 << QSPI_WPCCR_ISIZE_SHIFT)

#define QSPI_WPCCR_ADMODE_SHIFT      (8)        /* Bits 10-11: Address mode */
#define QSPI_WPCCR_ADMODE_MASK       (0x7 << QSPI_WPCCR_ADMODE_SHIFT)
#  define QSPI_WPCCR_ADMODE(n)       ((uint32_t)(n) << QSPI_WPCCR_ADMODE_SHIFT)

#define QSPI_WPCCR_ADTR              (1 << 11)   /* Bit 11: Address double transfer rate */

#define QSPI_WPCCR_ADSIZE_SHIFT      (12)        /* Bits 12-13: Address size */
#define QSPI_WPCCR_ADSIZE_MASK       (0x3 << QSPI_WPCCR_ADSIZE_SHIFT)
#  define QSPI_WPCCR_ADSIZE(n)       ((uint32_t)(n) << QSPI_WPCCR_ADSIZE_SHIFT)
#  define QSPI_WPCCR_ADSIZE_8b       (0 << QSPI_WPCCR_ADSIZE_SHIFT)
#  define QSPI_WPCCR_ADSIZE_16b      (1 << QSPI_WPCCR_ADSIZE_SHIFT)
#  define QSPI_WPCCR_ADSIZE_24b      (2 << QSPI_WPCCR_ADSIZE_SHIFT)
#  define QSPI_WPCCR_ADSIZE_32b      (3 << QSPI_WPCCR_ADSIZE_SHIFT)

#define QSPI_WPCCR_ABMODE_SHIFT      (16)        /* Bits 16-18: Alternate bytes mode */
#define QSPI_WPCCR_ABMODE_MASK       (0x7 << QSPI_WPCCR_ABMODE_SHIFT)
#  define QSPI_WPCCR_ABMODE(n)       ((uint32_t)(n) << QSPI_WPCCR_ABMODE_SHIFT)

#define QSPI_WPCCR_ABDTR              (1 << 19)   /* Bit 19: Alternate-byte double transfer rate */

#define QSPI_WPCCR_ABSIZE_SHIFT      (20)        /* Bits 20-21: Alternate bytes size */
#define QSPI_WPCCR_ABSIZE_MASK       (0x3 << QSPI_WPCCR_ABSIZE_SHIFT)
#  define QSPI_WPCCR_ABSIZE(n)       ((uint32_t)(n) << QSPI_WPCCR_ABSIZE_SHIFT)

#define QSPI_WPCCR_DMODE_SHIFT       (24)        /* Bits 24-25: Data mode */
#define QSPI_WPCCR_DMODE_MASK        (0x3 << QSPI_WPCCR_DMODE_SHIFT)
#  define QSPI_WPCCR_DMODE(n)        ((uint32_t)(n) << QSPI_WPCCR_DMODE_SHIFT)

#define QSPI_WPCCR_DDTR              (1 << 27)   /* Bit 27: Data double transfer rate */
#define QSPI_WPCCR_DQSE              (1 << 29)   /* Bit 29: DQS enable */

/* Wrap Timing Configuration Register */

#define QSPI_WPTCR_DCYC_SHIFT        (0)        /* Bits 0-4: Number of dummy cycles */
#define QSPI_WPTCR_DCYC_MASK         (0x1f << QSPI_WPTCR_DCYC_SHIFT)
#  define QSPI_WPTCR_DCYC(n)         ((uint32_t)(n) << QSPI_WPTCR_DCYC_SHIFT)

#define QSPI_WPTCR_DHQC              (1 << 28)   /* Bit 28: Delay hold quarter cycle */
#define QSPI_WPTCR_SSHIFT            (1 << 30)   /* Bit 30: Sample Shift */

/* Write Communication Configuration Register */

#define WCCR_IMODE_NONE      0   /* No instruction */
#define WCCR_IMODE_SINGLE    1   /* Instruction on a single line */
#define WCCR_IMODE_DUAL      2   /* Instruction on two lines */
#define WCCR_IMODE_QUAD      3   /* Instruction on four lines */

#define WCCR_ADMODE_NONE     0   /* No address */
#define WCCR_ADMODE_SINGLE   1   /* Address on a single line */
#define WCCR_ADMODE_DUAL     2   /* Address on two lines */
#define WCCR_ADMODE_QUAD     3   /* Address on four lines */

#define WCCR_ADSIZE_8        0   /* 8-bit address */
#define WCCR_ADSIZE_16       1   /* 16-bit address */
#define WCCR_ADSIZE_24       2   /* 24-bit address */
#define WCCR_ADSIZE_32       3   /* 32-bit address */

#define WCCR_ABMODE_NONE     0   /* No alternate bytes */
#define WCCR_ABMODE_SINGLE   1   /* Alternate bytes on a single line */
#define WCCR_ABMODE_DUAL     2   /* Alternate bytes on two lines */
#define WCCR_ABMODE_QUAD     3   /* Alternate bytes on four lines */

#define WCCR_ABSIZE_8        0   /* 8-bit alternate byte */
#define WCCR_ABSIZE_16       1   /* 16-bit alternate bytes */
#define WCCR_ABSIZE_24       2   /* 24-bit alternate bytes */
#define WCCR_ABSIZE_32       3   /* 32-bit alternate bytes */

#define WCCR_DMODE_NONE      0   /* No data */
#define WCCR_DMODE_SINGLE    1   /* Data on a single line */
#define WCCR_DMODE_DUAL      2   /* Data on two lines */
#define WCCR_DMODE_QUAD      3   /* Data on four lines */

#define QSPI_WCCR_IMODE_SHIFT       (0)        /* Bits 0-2: Instruction mode */
#define QSPI_WCCR_IMODE_MASK        (0x7 << QSPI_WCCR_IMODE_SHIFT)
#  define QSPI_WCCR_IMODE(n)        ((uint32_t)(n) << QSPI_WCCR_IMODE_SHIFT)

#define QSPI_WCCR_IDTR              (1 << 3)   /* Bit 3: Instruction double transfer rate */

#define QSPI_WCCR_ISIZE_SHIFT       (4)        /* Bits 4-5: Instruction size */
#define QSPI_WCCR_ISIZE_MASK        (0x3 << QSPI_WCCR_ISIZE_SHIFT)
#  define QSPI_WCCR_ISIZE_8b        (0 << QSPI_WCCR_ISIZE_SHIFT)
#  define QSPI_WCCR_ISIZE_16b       (1 << QSPI_WCCR_ISIZE_SHIFT)
#  define QSPI_WCCR_ISIZE_24b       (2 << QSPI_WCCR_ISIZE_SHIFT)
#  define QSPI_WCCR_ISIZE_32b       (3 << QSPI_WCCR_ISIZE_SHIFT)

#define QSPI_WCCR_ADMODE_SHIFT      (8)        /* Bits 10-11: Address mode */
#define QSPI_WCCR_ADMODE_MASK       (0x7 << QSPI_WCCR_ADMODE_SHIFT)
#  define QSPI_WCCR_ADMODE(n)       ((uint32_t)(n) << QSPI_WCCR_ADMODE_SHIFT)

#define QSPI_WCCR_ADTR              (1 << 11)   /* Bit 11: Address double transfer rate */

#define QSPI_WCCR_ADSIZE_SHIFT      (12)        /* Bits 12-13: Address size */
#define QSPI_WCCR_ADSIZE_MASK       (0x3 << QSPI_WCCR_ADSIZE_SHIFT)
#  define QSPI_WCCR_ADSIZE(n)       ((uint32_t)(n) << QSPI_WCCR_ADSIZE_SHIFT)
#  define QSPI_WCCR_ADSIZE_8b       (0 << QSPI_WCCR_ADSIZE_SHIFT)
#  define QSPI_WCCR_ADSIZE_16b      (1 << QSPI_WCCR_ADSIZE_SHIFT)
#  define QSPI_WCCR_ADSIZE_24b      (2 << QSPI_WCCR_ADSIZE_SHIFT)
#  define QSPI_WCCR_ADSIZE_32b      (3 << QSPI_WCCR_ADSIZE_SHIFT)

#define QSPI_WCCR_ABMODE_SHIFT      (16)        /* Bits 16-18: Alternate bytes mode */
#define QSPI_WCCR_ABMODE_MASK       (0x7 << QSPI_WCCR_ABMODE_SHIFT)
#  define QSPI_WCCR_ABMODE(n)       ((uint32_t)(n) << QSPI_WCCR_ABMODE_SHIFT)

#define QSPI_WCCR_ABDTR              (1 << 19)   /* Bit 19: Alternate-byte double transfer rate */

#define QSPI_WCCR_ABSIZE_SHIFT      (20)        /* Bits 20-21: Alternate bytes size */
#define QSPI_WCCR_ABSIZE_MASK       (0x3 << QSPI_WCCR_ABSIZE_SHIFT)
#  define QSPI_WCCR_ABSIZE(n)       ((uint32_t)(n) << QSPI_WCCR_ABSIZE_SHIFT)

#define QSPI_WCCR_DMODE_SHIFT       (24)        /* Bits 24-25: Data mode */
#define QSPI_WCCR_DMODE_MASK        (0x3 << QSPI_WCCR_DMODE_SHIFT)
#  define QSPI_WCCR_DMODE(n)        ((uint32_t)(n) << QSPI_WCCR_DMODE_SHIFT)

#define QSPI_WCCR_DDTR              (1 << 27)   /* Bit 27: Data double transfer rate */
#define QSPI_WCCR_DQSE              (1 << 29)   /* Bit 29: DQS enable */

/* Writing Timing Configuration Register */

#define QSPI_WTCR_DCYC_SHIFT        (0)        /* Bits 0-4: Number of dummy cycles */
#define QSPI_WTCR_DCYC_MASK         (0x1f << QSPI_WTCR_DCYC_SHIFT)
#  define QSPI_WTCR_DCYC(n)         ((uint32_t)(n) << QSPI_WTCR_DCYC_SHIFT)

/* HyperBus Latency Configuration Register */

#define QSPI_HLCR_LM                (1 << 0) /* Bit 0: Latency Mode */
#define QSPI_HLCR_WZL               (1 << 1) /* Bit 1: Write Zero Latency */

#define QSPI_HLCR_TACC_SHIFT       (8)        /* Bits 8-15: Access Timing */
#define QSPI_HLCR_TACC_MASK        (0xff << QSPI_HLCR_TACC_SHIFT)
#  define QSPI_HLCR_TACC(n)        ((uint32_t)(n) << QSPI_HLCR_TACC_SHIFT)

#define QSPI_HLCR_TRWR_SHIFT       (16)        /* Bits 16-23: Read-write minimum recovery time */
#define QSPI_HLCR_TRWR_MASK        (0xff << QSPI_HLCR_TRWR_SHIFT)
#  define QSPI_HLCR_TRWR(n)        ((uint32_t)(n) << QSPI_HLCR_TRWR_SHIFT)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_STM32H5_HARDWARE_STM32_QUADSPI_H */
