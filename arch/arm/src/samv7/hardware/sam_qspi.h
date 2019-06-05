/****************************************************************************************
 * arch/arm/src/samv7/hardware/sam_qspi.h
 * Quad SPI (QSPI) definitions for the SAMV71
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 ****************************************************************************************/

#ifndef __ARCH_ARM_SRC_SAMV7_HARDWARE_SAM_QSPI_H
#define __ARCH_ARM_SRC_SAMV7_HARDWARE_SAM_QSPI_H

/****************************************************************************************
 * Included Files
 ****************************************************************************************/

#include <nuttx/config.h>
#include <arch/samv7/chip.h>

#include "hardware/sam_memorymap.h"

#if SAMV7_NQSPI > 0

/****************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************/
/* General Characteristics **************************************************************/

#define SAM_QSPI_MINBITS           8      /* Minimum word width */
#define SAM_QSPI_MAXBITS           16     /* Maximum word width */

/* QSPI register offsets ****************************************************************/

#define SAM_QSPI_CR_OFFSET         0x0000 /* Control Register */
#define SAM_QSPI_MR_OFFSET         0x0004 /* Mode Register */
#define SAM_QSPI_RDR_OFFSET        0x0008 /* Receive Data Register */
#define SAM_QSPI_TDR_OFFSET        0x000c /* Transmit Data Register */
#define SAM_QSPI_SR_OFFSET         0x0010 /* Status Register */
#define SAM_QSPI_IER_OFFSET        0x0014 /* Interrupt Enable Register */
#define SAM_QSPI_IDR_OFFSET        0x0018 /* Interrupt Disable Register */
#define SAM_QSPI_IMR_OFFSET        0x001c /* Interrupt Mask Register */
#define SAM_QSPI_SCR_OFFSET        0x0020 /* Serial Clock Register */
#define SAM_QSPI_IAR_OFFSET        0x0030 /* Instruction Address Register */
#define SAM_QSPI_ICR_OFFSET        0x0034 /* Instruction Code Register */
#define SAM_QSPI_IFR_OFFSET        0x0038 /* Instruction Frame Register */
                                          /* 0x003c Reserved */
#define SAM_QSPI_SMR_OFFSET        0x0040 /* Scrambling Mode Register */
#define SAM_QSPI_SKR_OFFSET        0x0044 /* Scrambling Key Register */
                                          /* 0x0048–0x00e0 Reserved */
#define SAM_QSPI_WPCR_OFFSET       0x00e4 /* Write Protection Control Register */
#define SAM_QSPI_WPSR_OFFSET       0x00e8 /* Write Protection Status Register */
                                         /* 0xec-0xfc: Reserved */

/* QSPI register addresses **************************************************************/

#define SAM_QSPI0_CR               (SAM_QSPI0_BASE+SAM_QSPI_CR_OFFSET)   /* Control Register */
#define SAM_QSPI0_MR               (SAM_QSPI0_BASE+SAM_QSPI_MR_OFFSET)   /* Mode Register */
#define SAM_QSPI0_RDR              (SAM_QSPI0_BASE+SAM_QSPI_RDR_OFFSET)  /* Receive Data Register */
#define SAM_QSPI0_TDR              (SAM_QSPI0_BASE+SAM_QSPI_TDR_OFFSET)  /* Transmit Data Register */
#define SAM_QSPI0_SR               (SAM_QSPI0_BASE+SAM_QSPI_SR_OFFSET)   /* Status Register */
#define SAM_QSPI0_IER              (SAM_QSPI0_BASE+SAM_QSPI_IER_OFFSET)  /* Interrupt Enable Register */
#define SAM_QSPI0_IDR              (SAM_QSPI0_BASE+SAM_QSPI_IDR_OFFSET)  /* Interrupt Disable Register */
#define SAM_QSPI0_IMR              (SAM_QSPI0_BASE+SAM_QSPI_IMR_OFFSET)  /* Interrupt Mask Register */
#define SAM_QSPI0_SCR              (SAM_QSPI0_BASE+SAM_QSPI_SCR_OFFSET)  /* Serial Clock Register */
#define SAM_QSPI0_IAR              (SAM_QSPI0_BASE+SAM_QSPI_IAR_OFFSET)  /* Instruction Address Register */
#define SAM_QSPI0_ICR              (SAM_QSPI0_BASE+SAM_QSPI_ICR_OFFSET)  /* Instruction Code Register */
#define SAM_QSPI0_IFR              (SAM_QSPI0_BASE+SAM_QSPI_IFR_OFFSET)  /* Instruction Frame Register */
#define SAM_QSPI0_SMR              (SAM_QSPI0_BASE+SAM_QSPI_SMR_OFFSET)  /* Scrambling Mode Register */
#define SAM_QSPI0_SKR              (SAM_QSPI0_BASE+SAM_QSPI_SKR_OFFSET)  /* Scrambling Key Register */
#define SAM_QSPI0_WPCR             (SAM_QSPI0_BASE+SAM_QSPI_WPCR_OFFSET) /* Write Protection Control Register */
#define SAM_QSPI0_WPSR             (SAM_QSPI0_BASE+SAM_QSPI_WPSR_OFFSET) /* Write Protection Status Register */

#if SAMV7_NQSPI > 1
#  define SAM_QSPI1_CR             (SAM_QSPI1_BASE+SAM_QSPI_CR_OFFSET)   /* Control Register */
#  define SAM_QSPI1_MR             (SAM_QSPI1_BASE+SAM_QSPI_MR_OFFSET)   /* Mode Register */
#  define SAM_QSPI1_RDR            (SAM_QSPI1_BASE+SAM_QSPI_RDR_OFFSET)  /* Receive Data Register */
#  define SAM_QSPI1_TDR            (SAM_QSPI1_BASE+SAM_QSPI_TDR_OFFSET)  /* Transmit Data Register */
#  define SAM_QSPI1_SR             (SAM_QSPI1_BASE+SAM_QSPI_SR_OFFSET)   /* Status Register */
#  define SAM_QSPI1_IER            (SAM_QSPI1_BASE+SAM_QSPI_IER_OFFSET)  /* Interrupt Enable Register */
#  define SAM_QSPI1_IDR            (SAM_QSPI1_BASE+SAM_QSPI_IDR_OFFSET)  /* Interrupt Disable Register */
#  define SAM_QSPI1_IMR            (SAM_QSPI1_BASE+SAM_QSPI_IMR_OFFSET)  /* Interrupt Mask Register */
#  define SAM_QSPI1_SCR            (SAM_QSPI1_BASE+SAM_QSPI_SCR_OFFSET)  /* Serial Clock Register */
#  define SAM_QSPI1_IAR            (SAM_QSPI1_BASE+SAM_QSPI_IAR_OFFSET)  /* Instruction Address Register */
#  define SAM_QSPI1_ICR            (SAM_QSPI1_BASE+SAM_QSPI_ICR_OFFSET)  /* Instruction Code Register */
#  define SAM_QSPI1_IFR            (SAM_QSPI1_BASE+SAM_QSPI_IFR_OFFSET)  /* Instruction Frame Register */
#  define SAM_QSPI1_SMR            (SAM_QSPI1_BASE+SAM_QSPI_SMR_OFFSET)  /* Scrambling Mode Register */
#  define SAM_QSPI1_SKR            (SAM_QSPI1_BASE+SAM_QSPI_SKR_OFFSET)  /* Scrambling Key Register */
#  define SAM_QSPI1_WPCR           (SAM_QSPI1_BASE+SAM_QSPI_WPCR_OFFSET) /* Write Protection Control Register */
#  define SAM_QSPI1_WPSR           (SAM_QSPI1_BASE+SAM_QSPI_WPSR_OFFSET) /* Write Protection Status Register */
#endif

/* QSPI register bit definitions ********************************************************/

/* QSPI Control Register */

#define QSPI_CR_QSPIEN             (1 << 0)  /* Bit 0:  QSPI Enable */
#define QSPI_CR_QSPIDIS            (1 << 1)  /* Bit 1:  QSPI Disable */
#define QSPI_CR_SWRST              (1 << 7)  /* Bit 7:  QSPI Software Reset */
#define QSPI_CR_LASTXFER           (1 << 24) /* Bit 24: Last Transfer */

/* QSPI Mode Register */

#define QSPI_MR_SMM                (1 << 0)  /* Bit 0:  Serial Memory Mode */
#define QSPI_MR_LLB                (1 << 1)  /* Bit 1:  Local Loopback Enable */
#define QSPI_MR_WDRBT              (1 << 2)  /* Bit 2:  Wait Data Read Before Transfer */
#define QSPI_MR_CSMODE_SHIFT       (4)       /* Bits 4-5: Chip Select Mode */
#define QSPI_MR_CSMODE_MASK        (3 << QSPI_MR_CSMODE_SHIFT)
#  define QSPI_MR_CSMODE_NRELOAD   (0 << QSPI_MR_CSMODE_SHIFT) /* CS deasserted if TD not reloaded */
#  define QSPI_MR_CSMODE_LASTXFER  (1 << QSPI_MR_CSMODE_SHIFT) /* CS deasserted when LASTXFER transferred */
#  define QSPI_MR_CSMODE_SYSTEM    (2 << QSPI_MR_CSMODE_SHIFT) /* CS deasserted after each transfer */
#define QSPI_MR_NBBITS_SHIFT       (8)       /* Bits 8-11: Number Of Bits Per Transfer */
#define QSPI_MR_NBBITS_MASK        (15 << QSPI_MR_NBBITS_SHIFT)
#  define QSPI_MR_NBBITS(n)        ((uint32_t)((n)-SAM_QSPI_MINBITS) << QSPI_MR_NBBITS_SHIFT)
#  define QSPI_MR_NBBITS_8BIT      (0 << QSPI_MR_NBBITS_SHIFT) /* 8 bits for transfer */
#  define QSPI_MR_NBBITS_9BIT      (1 << QSPI_MR_NBBITS_SHIFT) /* 9 bits for transfer */
#  define QSPI_MR_NBBITS_10BIT     (2 << QSPI_MR_NBBITS_SHIFT) /* 10 bits for transfer */
#  define QSPI_MR_NBBITS_11BIT     (3 << QSPI_MR_NBBITS_SHIFT) /* 11 bits for transfer */
#  define QSPI_MR_NBBITS_12BIT     (4 << QSPI_MR_NBBITS_SHIFT) /* 12 bits for transfer */
#  define QSPI_MR_NBBITS_13BIT     (5 << QSPI_MR_NBBITS_SHIFT) /* 13 bits for transfer */
#  define QSPI_MR_NBBITS_14BIT     (6 << QSPI_MR_NBBITS_SHIFT) /* 14 bits for transfer */
#  define QSPI_MR_NBBITS_15BIT     (7 << QSPI_MR_NBBITS_SHIFT) /* 15 bits for transfer */
#  define QSPI_MR_NBBITS_16BIT     (8 << QSPI_MR_NBBITS_SHIFT) /* 16 bits for transfer */
#define QSPI_MR_DLYBCT_SHIFT       (16)      /* Bits 16-23: Delay Between Consecutive Transfers */
#define QSPI_MR_DLYBCT_MASK        (0xff << QSPI_MR_DLYBCT_SHIFT)
#  define QSPI_MR_DLYBCT(n)        ((uint32_t)(n) << QSPI_MR_DLYBCT_SHIFT)
#define QSPI_MR_DLYCS_SHIFT        (24)      /* Bits 24-31: Minimum Inactive QCS Delay */
#define QSPI_MR_DLYCS_MASK         (0xff << QSPI_MR_DLYCS_SHIFT)
#  define QSPI_MR_DLYCS(n)         ((uint32_t)(n) << QSPI_MR_DLYCS_SHIFT)

/* QSPI Receive Data Register */

#define QSPI_RDR_RD_SHIFT          (0)       /* Bits 0-15: Receive Data */
#define QSPI_RDR_RD_MASK           (0xffff << QSPI_RDR_RD_SHIFT)

/* QSPI Transmit Data Register */

#define QSPI_TDR_TD_SHIFT          (0)       /* Bits 0-15:  Transmit Data */
#define QSPI_TDR_TD_MASK           (0xffff << QSPI_TDR_TD_SHIFT)

/* QSPI Status Register, QSPI Interrupt Enable Register, QSPI Interrupt Disable Register,
 * and QSPI Interrupt Mask Register (common bit fields)
 */

#define QSPI_INT_RDRF              (1 << 0)  /* Bit 0:  Receive Data Register Full Interrupt */
#define QSPI_INT_TDRE              (1 << 1)  /* Bit 1:  Transmit Data Register Empty Interrupt */
#define QSPI_INT_TXEMPTY           (1 << 2)  /* Bit 2:  Transmission Registers Empty Interrupt */
#define QSPI_INT_OVRES             (1 << 3)  /* Bit 3:  Overrun Error Interrupt */
#define QSPI_INT_CSR               (1 << 8)  /* Bit 8:  Chip Select Rise Interrupt */
#define QSPI_SR_CSS                (1 << 9)  /* Bit 9:  Chip Select Status Interrupt */
#define QSPI_SR_INSTRE             (1 << 10) /* Bit 10: Instruction End Status Interrupt */
#define QSPI_SR_QSPIENS            (1 << 24) /* Bit 24: QSPI Enable Status (SR only) */

#define QSPI_INT_ALL               (0x0000070f)

/* Serial Clock Register */

#define QSPI_SCR_CPOL              (1 << 0)  /* Bit 0:  Clock Polarity */
#define QSPI_SCR_CPHA              (1 << 1)  /* Bit 1:  Clock Phase */
#define QSPI_SCR_SCBR_SHIFT        (8)       /* Bits 8-15: Serial Clock Baud Rate */
#define QSPI_SCR_SCBR_MASK         (0xff << QSPI_SCR_SCBR_SHIFT)
#  define QSPI_SCR_SCBR(n)         ((uint32_t)(n) << QSPI_SCR_SCBR_SHIFT)
#define QSPI_SCR_DLYBS_SHIFT       (16)      /* Bits 16-23: Delay Before QSCK */
#define QSPI_SCR_DLYBS_MASK        (0xff << QSPI_SCR_DLYBS_SHIFT)
#  define QSPI_SCR_DLYBS(n)        ((uint32_t)(n) << QSPI_SCR_DLYBS_SHIFT)

/* Instruction Address Register (32-bit value) */

/* Instruction Code Register */

#define QSPI_ICR_INST_SHIFT        (0)       /* Bits 0-7: Instruction Code */
#define QSPI_ICR_INST_MASK         (0xff << QSPI_ICR_INST_SHIFT)
#  define QSPI_ICR_INST(n)         ((uint32_t)(n) << QSPI_ICR_INST_SHIFT)
#define QSPI_ICR_OPT_SHIFT         (16)      /*  Bits 16-23: Option Code */
#define QSPI_ICR_OPT_MASK          (0xff << QSPI_ICR_OPT_SHIFT)
#  define QSPI_ICR_OPT(n)          ((uint32_t)(n) << QSPI_ICR_OPT_SHIFT)

/* Instruction Frame Register */

#define QSPI_IFR_WIDTH_SHIFT       (0)       /* Bits 0-2: Width of Instruction Code,
                                              * Address, Option Code and Data */
#define QSPI_IFR_WIDTH_MASK        (7 << QSPI_IFR_WIDTH_SHIFT)
                                                               /* Instruction Address-Option Data */
#  define QSPI_IFR_WIDTH_SINGLE    (0 << QSPI_IFR_WIDTH_SHIFT) /* Single-bit  Single-bit     Single-bit */
#  define QSPI_IFR_WIDTH_DUALOUT   (1 << QSPI_IFR_WIDTH_SHIFT) /* Single-bit  Single-bit     Dual */
#  define QSPI_IFR_WIDTH_QUADOUT   (2 << QSPI_IFR_WIDTH_SHIFT) /* Single-bit  Single-bit     Quad */
#  define QSPI_IFR_WIDTH_DUALIO    (3 << QSPI_IFR_WIDTH_SHIFT) /* Single-bit  Dual           Dual */
#  define QSPI_IFR_WIDTH_QUADIO    (4 << QSPI_IFR_WIDTH_SHIFT) /* Single-bit  Quad           Quad */
#  define QSPI_IFR_WIDTH_DUALCMD   (5 << QSPI_IFR_WIDTH_SHIFT) /* Dual        Dual           Dual */
#  define QSPI_IFR_WIDTH_QUADCMD   (6 << QSPI_IFR_WIDTH_SHIFT) /* Quad        Quad           Quad */
#define QSPI_IFR_INSTEN            (1 << 4)  /* Bit 4:  Instruction Enable */
#define QSPI_IFR_ADDREN            (1 << 5)  /* Bit 5:  Address Enable */
#define QSPI_IFR_OPTEN             (1 << 6)  /* Bit 6:  Option Enable */
#define QSPI_IFR_DATAEN            (1 << 7)  /* Bit 7:  Data Enable */
#define QSPI_IFR_OPTL_SHIFT        (8)       /* Bits 8-9: Option Code Length */
#define QSPI_IFR_OPTL_MASK         (3 << QSPI_IFR_OPTL_SHIFT)
#  define QSPI_IFR_OPTL_1BIT       (0 << QSPI_IFR_OPTL_SHIFT) /* Option is 1 bit */
#  define QSPI_IFR_OPTL_2BIT       (1 << QSPI_IFR_OPTL_SHIFT) /* Option is 2 bits */
#  define QSPI_IFR_OPTL_4BIT       (2 << QSPI_IFR_OPTL_SHIFT) /* Option is 4 bits */
#  define QSPI_IFR_OPTL_8BIT       (3 << QSPI_IFR_OPTL_SHIFT) /* Option is 8 bits */
#define QSPI_IFR_ADDRL             (1 << 10) /* Bit 10: Address Length */
#  define QSPI_IFR_ADDRL_24BIT     (0 << 10) /*   0=24-bit */
#  define QSPI_IFR_ADDRL_32BIT     (1 << 10) /*   1=32-bit */
#define QSPI_IFR_TFRTYP_SHIFT      (12)      /* Bits 12-13: Data Transfer Type */
#define QSPI_IFR_TFRTYP_MASK       (3 << QSPI_IFR_TFRTYP_SHIFT)
#  define QSPI_IFR_TFRTYP_READ     (0 << QSPI_IFR_TFRTYP_SHIFT) /* Read transfer from serial memory */
#  define QSPI_IFR_TFRTYP_RDMEM    (1 << QSPI_IFR_TFRTYP_SHIFT) /* Read data transfer from serial memory */
#  define QSPI_IFR_TFRTYP_WRITE    (2 << QSPI_IFR_TFRTYP_SHIFT) /* Write transfer into serial memory */
#  define QSPI_IFR_TFRTYP_WRMEM    (3 << QSPI_IFR_TFRTYP_SHIFT) /* Write data transfer the serial memory */
#define QSPI_IFR_CRM               (1 << 14) /* Bit 14: Continuous Read Mode */
#define QSPI_IFR_NBDUM_SHIFT       (16)      /* Bits 16-20: Number Of Dummy Cycles */
#define QSPI_IFR_NBDUM_MASK        (31 << QSPI_IFR_NBDUM_SHIFT)
#  define QSPI_IFR_NBDUM(n)        ((uint32_t)(n) << QSPI_IFR_NBDUM_SHIFT)

/* Scrambling Mode Register */

#define QSPI_SMR_SCREN             (1 << 0)  /* Bit 0:  Scrambling/Unscrambling Enable */
#define QSPI_SMR_RVDIS             (1 << 1)  /* Bit 1:  Scrambling/Unscrambling Random Value Disable */

/* Scrambling Key Register (32-bit value) */

/* QSPI Write Protection Control Register */

#define QSPI_WPCR_WPEN             (1 << 0)  /* Bit 0:  QSPI Write Protection Enable */
#define QSPI_WPCR_WPKEY_SHIFT      (8)       /* Bits 8-31: QSPI Write Protection Key Password */
#define QSPI_WPCR_WPKEY_MASK       (0x00ffffff << QSPI_WPCR_WPKEY_SHIFT)
#  define QSPI_WPCR_WPKEY          (0x00515350 << QSPI_WPCR_WPKEY_SHIFT)

/* QSPI Write Protection Status Register */

#define QSPI_WPSR_WPVS             (1 << 0) /* Bit 0: QSPI Write Protection Violation Status */
#define QSPI_WPSR_WPVSRC_SHIFT     (8)      /* Bits 8-15: QSPI Write Protection Violation Source */
#define QSPI_WPSR_WPVSRC_MASK      (0xff << QSPI_WPSR_WPVSRC_SHIFT)

/****************************************************************************************
 * Public Types
 ****************************************************************************************/

/****************************************************************************************
 * Public Data
 ****************************************************************************************/

/****************************************************************************************
 * Public Functions
 ****************************************************************************************/

#endif /* SAMV7_NQSPI > 0 */
#endif /* __ARCH_ARM_SRC_SAMV7_HARDWARE_SAM_QSPI_H */
