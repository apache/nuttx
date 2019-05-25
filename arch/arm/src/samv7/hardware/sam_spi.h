/****************************************************************************************
 * arch/arm/src/samv7/hardware/sam_spi.h
 * Serial Peripheral Interface (SPI) definitions for the SAMV71
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

#ifndef __ARCH_ARM_SRC_SAMV7_HARDWARE_SAM_SPI_H
#define __ARCH_ARM_SRC_SAMV7_HARDWARE_SAM_SPI_H

/****************************************************************************************
 * Included Files
 ****************************************************************************************/

#include <nuttx/config.h>
#include <arch/samv7/chip.h>

#include "hardware/sam_memorymap.h"

#if SAMV7_NSPI > 0

/****************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************/
/* General definitions ******************************************************************/

#define SAM_SPI_NCS               4    /* Four chip selects */

/* SPI register offsets *****************************************************************/

#define SAM_SPI_CR_OFFSET         0x0000 /* Control Register */
#define SAM_SPI_MR_OFFSET         0x0004 /* Mode Register */
#define SAM_SPI_RDR_OFFSET        0x0008 /* Receive Data Register */
#define SAM_SPI_TDR_OFFSET        0x000c /* Transmit Data Register */
#define SAM_SPI_SR_OFFSET         0x0010 /* Status Register */
#define SAM_SPI_IER_OFFSET        0x0014 /* Interrupt Enable Register */
#define SAM_SPI_IDR_OFFSET        0x0018 /* Interrupt Disable Register */
#define SAM_SPI_IMR_OFFSET        0x001c /* Interrupt Mask Register */
                                         /* 0x20-0x2c: Reserved */
#define SAM_SPI_CSR0_OFFSET       0x0030 /* Chip Select Register 0 */
#define SAM_SPI_CSR1_OFFSET       0x0034 /* Chip Select Register 1 */
#define SAM_SPI_CSR2_OFFSET       0x0038 /* Chip Select Register 2 */
#define SAM_SPI_CSR3_OFFSET       0x003c /* Chip Select Register 3 */
                                         /* 0x40-0xe0: Reserved */
#define SAM_SPI_WPCR_OFFSET       0x00e4 /* Write Protection Control Register */
#define SAM_SPI_WPSR_OFFSET       0x00e8 /* Write Protection Status Register */
                                         /* 0xec-0xfc: Reserved */

/* SPI register addresses ***************************************************************/

#define SAM_SPI0_CR               (SAM_SPI0_BASE+SAM_SPI_CR_OFFSET)   /* Control Register */
#define SAM_SPI0_MR               (SAM_SPI0_BASE+SAM_SPI_MR_OFFSET)   /* Mode Register */
#define SAM_SPI0_RDR              (SAM_SPI0_BASE+SAM_SPI_RDR_OFFSET)  /* Receive Data Register */
#define SAM_SPI0_TDR              (SAM_SPI0_BASE+SAM_SPI_TDR_OFFSET)  /* Transmit Data Register */
#define SAM_SPI0_SR               (SAM_SPI0_BASE+SAM_SPI_SR_OFFSET)   /* Status Register */
#define SAM_SPI0_IER              (SAM_SPI0_BASE+SAM_SPI_IER_OFFSET)  /* Interrupt Enable Register */
#define SAM_SPI0_IDR              (SAM_SPI0_BASE+SAM_SPI_IDR_OFFSET)  /* Interrupt Disable Register */
#define SAM_SPI0_IMR              (SAM_SPI0_BASE+SAM_SPI_IMR_OFFSET)  /* Interrupt Mask Register */
#define SAM_SPI0_CSR0             (SAM_SPI0_BASE+SAM_SPI_CSR0_OFFSET) /* Chip Select Register 0 */
#define SAM_SPI0_CSR1             (SAM_SPI0_BASE+SAM_SPI_CSR1_OFFSET) /* Chip Select Register 1 */
#define SAM_SPI0_CSR2             (SAM_SPI0_BASE+SAM_SPI_CSR2_OFFSET) /* Chip Select Register 2 */
#define SAM_SPI0_CSR3             (SAM_SPI0_BASE+SAM_SPI_CSR3_OFFSET) /* Chip Select Register 3 */
#define SAM_SPI0_WPCR             (SAM_SPI0_BASE+SAM_SPI_WPCR_OFFSET) /* Write Protection Control Register */
#define SAM_SPI0_WPSR             (SAM_SPI0_BASE+SAM_SPI_WPSR_OFFSET) /* Write Protection Status Register */

#if SAMV7_NSPI > 1
#  define SAM_SPI1_CR             (SAM_SPI1_BASE+SAM_SPI_CR_OFFSET)   /* Control Register */
#  define SAM_SPI1_MR             (SAM_SPI1_BASE+SAM_SPI_MR_OFFSET)   /* Mode Register */
#  define SAM_SPI1_RDR            (SAM_SPI1_BASE+SAM_SPI_RDR_OFFSET)  /* Receive Data Register */
#  define SAM_SPI1_TDR            (SAM_SPI1_BASE+SAM_SPI_TDR_OFFSET)  /* Transmit Data Register */
#  define SAM_SPI1_SR             (SAM_SPI1_BASE+SAM_SPI_SR_OFFSET)   /* Status Register */
#  define SAM_SPI1_IER            (SAM_SPI1_BASE+SAM_SPI_IER_OFFSET)  /* Interrupt Enable Register */
#  define SAM_SPI1_IDR            (SAM_SPI1_BASE+SAM_SPI_IDR_OFFSET)  /* Interrupt Disable Register */
#  define SAM_SPI1_IMR            (SAM_SPI1_BASE+SAM_SPI_IMR_OFFSET)  /* Interrupt Mask Register */
#  define SAM_SPI1_CSR0           (SAM_SPI1_BASE+SAM_SPI_CSR0_OFFSET) /* Chip Select Register 0 */
#  define SAM_SPI1_CSR1           (SAM_SPI1_BASE+SAM_SPI_CSR1_OFFSET) /* Chip Select Register 1 */
#  define SAM_SPI1_CSR2           (SAM_SPI1_BASE+SAM_SPI_CSR2_OFFSET) /* Chip Select Register 2 */
#  define SAM_SPI1_CSR3           (SAM_SPI1_BASE+SAM_SPI_CSR3_OFFSET) /* Chip Select Register 3 */
#  define SAM_SPI1_WPCR           (SAM_SPI1_BASE+SAM_SPI_WPCR_OFFSET) /* Write Protection Control Register */
#  define SAM_SPI1_WPSR           (SAM_SPI1_BASE+SAM_SPI_WPSR_OFFSET) /* Write Protection Status Register */
#endif

/* SPI register bit definitions *********************************************************/

/* SPI Control Register */

#define SPI_CR_SPIEN              (1 << 0)  /* Bit 0:  SPI Enable */
#define SPI_CR_SPIDIS             (1 << 1)  /* Bit 1:  SPI Disable */
#define SPI_CR_SWRST              (1 << 7)  /* Bit 7:  SPI Software Reset */
#define SPI_CR_LASTXFER           (1 << 24) /* Bit 24: Last Transfer */

/* SPI Mode Register */

#define SPI_MR_MSTR               (1 << 0)  /* Bit 0:  1=Master Mode */
#  define SPI_MR_SLAVE            (0)       /*         0=Slave Mode */
#define SPI_MR_PS                 (1 << 1)  /* Bit 1:  Peripheral Select */
#define SPI_MR_PCSDEC             (1 << 2)  /* Bit 2:  Chip Select Decode */
#define SPI_MR_MODFDIS            (1 << 4)  /* Bit 4:  Mode Fault Detection */
#define SPI_MR_WDRBT              (1 << 5)  /* Bit 5:  Wait Data Read Before Transfer */
#define SPI_MR_LLB                (1 << 7)  /* Bit 7:  Local Loopback Enable */
#define SPI_MR_PCS_SHIFT          (16)      /* Bits 16-19: Peripheral Chip Select */
#define SPI_MR_PCS_MASK           (15 << SPI_MR_PCS_SHIFT)
#  define SPI_MR_PCS0             (0 << SPI_MR_PCS_SHIFT) /* NPCS[3:0] = 1110 (w/PCSDEC=0) */
#  define SPI_MR_PCS1             (1 << SPI_MR_PCS_SHIFT) /* NPCS[3:0] = 1101 (w/PCSDEC=0) */
#  define SPI_MR_PCS2             (3 << SPI_MR_PCS_SHIFT) /* NPCS[3:0] = 1011 (w/PCSDEC=0) */
#  define SPI_MR_PCS3             (7 << SPI_MR_PCS_SHIFT) /* NPCS[3:0] = 0111 (w/PCSDEC=0) */
#define SPI_MR_DLYBCS_SHIFT       (24)      /* Bits 24-31: Delay Between Chip Selects */
#define SPI_MR_DLYBCS_MASK        (0xff << SPI_MR_DLYBCS_SHIFT)
#  define SPI_MR_DLYBCS(n)        ((uint32_t)(n) << SPI_MR_DLYBCS_SHIFT)

/* SPI Receive Data Register */

#define SPI_RDR_RD_SHIFT          (0)       /* Bits 0-15: Receive Data */
#define SPI_RDR_RD_MASK           (0xffff << SPI_RDR_RD_SHIFT)
#define SPI_RDR_PCS_SHIFT         (16)      /* Bits 16-19: Peripheral Chip Select */
#define SPI_RDR_PCS_MASK          (15 << SPI_RDR_PCS_SHIFT)
#  define SPI_RDR_PCS0            (0 << SPI_RDR_PCS_SHIFT) /* NPCS[3:0] = 1110 (w/PCSDEC=0) */
#  define SPI_RDR_PCS1            (1 << SPI_RDR_PCS_SHIFT) /* NPCS[3:0] = 1101 (w/PCSDEC=0) */
#  define SPI_RDR_PCS2            (3 << SPI_RDR_PCS_SHIFT) /* NPCS[3:0] = 1011 (w/PCSDEC=0) */
#  define SPI_RDR_PCS3            (7 << SPI_RDR_PCS_SHIFT) /* NPCS[3:0] = 0111 (w/PCSDEC=0) */

/* SPI Transmit Data Register */

#define SPI_TDR_TD_SHIFT          (0)       /* Bits 0-15:  Transmit Data */
#define SPI_TDR_TD_MASK           (0xffff << SPI_TDR_TD_SHIFT)
#define SPI_TDR_PCS_SHIFT         (16)      /* Bits 16-19: Peripheral Chip Select */
#define SPI_TDR_PCS_MASK          (15 << SPI_TDR_PCS_SHIFT)
#  define SPI_TDR_PCS0            (0 << SPI_TDR_PCS_SHIFT) /* NPCS[3:0] = 1110 (w/PCSDEC=0) */
#  define SPI_TDR_PCS1            (1 << SPI_TDR_PCS_SHIFT) /* NPCS[3:0] = 1101 (w/PCSDEC=0) */
#  define SPI_TDR_PCS2            (3 << SPI_TDR_PCS_SHIFT) /* NPCS[3:0] = 1011 (w/PCSDEC=0) */
#  define SPI_TDR_PCS3            (7 << SPI_TDR_PCS_SHIFT) /* NPCS[3:0] = 0111 (w/PCSDEC=0) */
#define SPI_TDR_LASTXFER          (1 << 24) /* Bit 24: Last Transfer */

/* SPI Status Register, SPI Interrupt Enable Register, SPI Interrupt Disable Register,
 * and SPI Interrupt Mask Register (common bit fields)
 */

#define SPI_INT_RDRF              (1 << 0)  /* Bit 0:  Receive Data Register Full Interrupt */
#define SPI_INT_TDRE              (1 << 1)  /* Bit 1:  Transmit Data Register Empty Interrupt */
#define SPI_INT_MODF              (1 << 2)  /* Bit 2:  Mode Fault Error Interrupt */
#define SPI_INT_OVRES             (1 << 3)  /* Bit 3:  Overrun Error Interrupt */
#define SPI_INT_NSSR              (1 << 8)  /* Bit 8:  NSS Rising Interrupt */
#define SPI_INT_TXEMPTY           (1 << 9)  /* Bit 9:  Transmission Registers Empty Interrupt */
#define SPI_INT_UNDES             (1 << 10) /* Bit 10: Underrun Error Status Interrupt (slave) */
#define SPI_SR_SPIENS             (1 << 16) /* Bit 16: SPI Enable Status (SR only) */

#define SPI_INT_ALL               (0x0000070f)

/* SPI Chip Select Registers 0-3 */

#define SPI_CSR_CPOL              (1 << 0)  /* Bit 0:  Clock Polarity */
#define SPI_CSR_NCPHA             (1 << 1)  /* Bit 1:  Clock Phase */
#define SPI_CSR_CSNAAT            (1 << 2)  /* Bit 2:  Chip Select Not Active After Transfer */
#define SPI_CSR_CSAAT             (1 << 3)  /* Bit 3:  Chip Select Active After Transfer */
#define SPI_CSR_BITS_SHIFT        (4)       /* Bits 4-7: Bits Per Transfer */
#define SPI_CSR_BITS_MASK         (15 << SPI_CSR_BITS_SHIFT)
#  define SPI_CSR_BITS(n)         (((n)-8) << SPI_CSR_BITS_SHIFT) /* n, n=8-16 */
#  define SPI_CSR_BITS8           (0 << SPI_CSR_BITS_SHIFT) /* 8 */
#  define SPI_CSR_BITS9           (1 << SPI_CSR_BITS_SHIFT) /* 9 */
#  define SPI_CSR_BITS10          (2 << SPI_CSR_BITS_SHIFT) /* 10 */
#  define SPI_CSR_BITS11          (3 << SPI_CSR_BITS_SHIFT) /* 11 */
#  define SPI_CSR_BITS12          (4 << SPI_CSR_BITS_SHIFT) /* 12 */
#  define SPI_CSR_BITS13          (5 << SPI_CSR_BITS_SHIFT) /* 13 */
#  define SPI_CSR_BITS14          (6 << SPI_CSR_BITS_SHIFT) /* 14 */
#  define SPI_CSR_BITS15          (7 << SPI_CSR_BITS_SHIFT) /* 15 */
#  define SPI_CSR_BITS16          (8 << SPI_CSR_BITS_SHIFT) /* 16 */
#define SPI_CSR_SCBR_SHIFT        (8)       /* Bits 8-15: Serial Clock Baud Rate */
#define SPI_CSR_SCBR_MASK         (0xff << SPI_CSR_SCBR_SHIFT)
#  define SPI_CSR_SCBR(n)         ((uint32_t)(n) << SPI_CSR_SCBR_SHIFT)
#define SPI_CSR_DLYBS_SHIFT       (16)      /* Bits 16-23: Delay Before SPCK */
#define SPI_CSR_DLYBS_MASK        (0xff << SPI_CSR_DLYBS_SHIFT)
#  define SPI_CSR_DLYBS(n)        ((uint32_t)(n) << SPI_CSR_DLYBS_SHIFT)
#define SPI_CSR_DLYBCT_SHIFT      (24)      /* Bits 24-31: Delay Between Consecutive Transfers */
#define SPI_CSR_DLYBCT_MASK       (0xff << SPI_CSR_DLYBCT_SHIFT)
#  define SPI_CSR_DLYBCT(n)       ((uint32_t)(n) << SPI_CSR_DLYBCT_SHIFT)

/* SPI Write Protection Control Register */

#define SPI_WPCR_WPEN             (1 << 0)  /* Bit 0:  SPI Write Protection Enable */
#define SPI_WPCR_WPKEY_SHIFT      (8)       /* Bits 8-31: SPI Write Protection Key Password */
#define SPI_WPCR_WPKEY_MASK       (0x00ffffff << SPI_WPCR_WPKEY_SHIFT)
#  define SPI_WPCR_WPKEY          (0x00535049 << SPI_WPCR_WPKEY_SHIFT)

/* SPI Write Protection Status Register */

#define SPI_WPSR_WPVS             (1 << 0) /* Bit 0: SPI Write Protection Violation Status */
#define SPI_WPSR_WPVSRC_SHIFT     (8)      /* Bits 8-15: SPI Write Protection Violation Source */
#define SPI_WPSR_WPVSRC_MASK      (0xff << SPI_WPSR_WPVSRC_SHIFT)

/****************************************************************************************
 * Public Types
 ****************************************************************************************/

/****************************************************************************************
 * Public Data
 ****************************************************************************************/

/****************************************************************************************
 * Public Functions
 ****************************************************************************************/

#endif /* SAMV7_NSPI > 0 */
#endif /* __ARCH_ARM_SRC_SAMV7_HARDWARE_SAM_SPI_H */
