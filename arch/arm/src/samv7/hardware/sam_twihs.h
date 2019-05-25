/****************************************************************************************
 * arch/arm/src/samv7/hardware/sam_twihs.h
 * Two-wire Interface (TWIHS) definitions for the SAMV71
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
 * FOR SAMV7_NTWIHS PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
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

#ifndef __ARCH_ARM_SRC_SAMV7_HARDWARE_SAM_TWIHS_H
#define __ARCH_ARM_SRC_SAMV7_HARDWARE_SAM_TWIHS_H

/****************************************************************************************
 * Included Files
 ****************************************************************************************/

#include <nuttx/config.h>
#include <arch/samv7/chip.h>

#include "hardware/sam_memorymap.h"

#if SAMV7_NTWIHS > 0

/****************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************/

/* TWIHS register offsets *****************************************************************/

#define SAM_TWIHS_CR_OFFSET        0x0000 /* Control Register */
#define SAM_TWIHS_MMR_OFFSET       0x0004 /* Master Mode Register */
#define SAM_TWIHS_SMR_OFFSET       0x0008 /* Slave Mode Register */
#define SAM_TWIHS_IADR_OFFSET      0x000c /* Internal Address Register */
#define SAM_TWIHS_CWGR_OFFSET      0x0010 /* Clock Waveform Generator Register */
                                          /* 0x0014-0x001c: Reserved */
#define SAM_TWIHS_SR_OFFSET        0x0020 /* Status Register */
#define SAM_TWIHS_IER_OFFSET       0x0024 /* Interrupt Enable Register */
#define SAM_TWIHS_IDR_OFFSET       0x0028 /* Interrupt Disable Register */
#define SAM_TWIHS_IMR_OFFSET       0x002c /* Interrupt Mask Register */
#define SAM_TWIHS_RHR_OFFSET       0x0030 /* Receive Holding Register */
#define SAM_TWIHS_THR_OFFSET       0x0034 /* Transmit Holding Register */
#define SAM_TWIHS_SMBTR_OFFSET     0x0038 /* SMBus Timing Register */
#define SAM_TWIHS_FILTR_OFFSET     0x0044 /* Filter Register */
#define SAM_TWIHS_SWMR_OFFSET      0x004c /* SleepWalking Matching Register */
                                          /* 0x3c-0xe0: Reserved */
#define SAM_TWIHS_WPMR_OFFSET      0x00e4 /* Protection Mode Register */
#define SAM_TWIHS_WPSR_OFFSET      0x00e8 /* Protection Status Register */

/* TWIHS register addresses ***************************************************************/

#define SAM_TWIHS0_CR              (SAM_TWIHS0_BASE+SAM_TWIHS_CR_OFFSET)
#define SAM_TWIHS0_MMR             (SAM_TWIHS0_BASE+SAM_TWIHS_MMR_OFFSET)
#define SAM_TWIHS0_SMR             (SAM_TWIHS0_BASE+SAM_TWIHS_SMR_OFFSET)
#define SAM_TWIHS0_IADR            (SAM_TWIHS0_BASE+SAM_TWIHS_IADR_OFFSET)
#define SAM_TWIHS0_CWGR            (SAM_TWIHS0_BASE+SAM_TWIHS_CWGR_OFFSET)
#define SAM_TWIHS0_SR              (SAM_TWIHS0_BASE+SAM_TWIHS_SR_OFFSET)
#define SAM_TWIHS0_IER             (SAM_TWIHS0_BASE+SAM_TWIHS_IER_OFFSET)
#define SAM_TWIHS0_IDR             (SAM_TWIHS0_BASE+SAM_TWIHS_IDR_OFFSET)
#define SAM_TWIHS0_IMR             (SAM_TWIHS0_BASE+SAM_TWIHS_IMR_OFFSET)
#define SAM_TWIHS0_RHR             (SAM_TWIHS0_BASE+SAM_TWIHS_RHR_OFFSET)
#define SAM_TWIHS0_THR             (SAM_TWIHS0_BASE+SAM_TWIHS_THR_OFFSET)
#define SAM_TWIHS0_SMBTR           (SAM_TWIHS0_BASE+SAM_TWIHS_SMBTR_OFFSET)
#define SAM_TWIHS0_FILTR           (SAM_TWIHS0_BASE+SAM_TWIHS_FILTR_OFFSET)
#define SAM_TWIHS0_SWMR            (SAM_TWIHS0_BASE+SAM_TWIHS_SWMR_OFFSET)
#define SAM_TWIHS0_WPMR            (SAM_TWIHS0_BASE+SAM_TWIHS_WPMR_OFFSET)
#define SAM_TWIHS0_WPSR            (SAM_TWIHS0_BASE)+SAM_TWIHS_WPSR_OFFSET)

#if SAMV7_NTWIHS > 1
#  define SAM_TWIHS1_CR            (SAM_TWIHS1_BASE+SAM_TWIHS_CR_OFFSET)
#  define SAM_TWIHS1_MMR           (SAM_TWIHS1_BASE+SAM_TWIHS_MMR_OFFSET)
#  define SAM_TWIHS1_SMR           (SAM_TWIHS1_BASE+SAM_TWIHS_SMR_OFFSET)
#  define SAM_TWIHS1_IADR          (SAM_TWIHS1_BASE+SAM_TWIHS_IADR_OFFSET)
#  define SAM_TWIHS1_CWGR          (SAM_TWIHS1_BASE+SAM_TWIHS_CWGR_OFFSET)
#  define SAM_TWIHS1_SR            (SAM_TWIHS1_BASE+SAM_TWIHS_SR_OFFSET)
#  define SAM_TWIHS1_IER           (SAM_TWIHS1_BASE+SAM_TWIHS_IER_OFFSET)
#  define SAM_TWIHS1_IDR           (SAM_TWIHS1_BASE+SAM_TWIHS_IDR_OFFSET)
#  define SAM_TWIHS1_IMR           (SAM_TWIHS1_BASE+SAM_TWIHS_IMR_OFFSET)
#  define SAM_TWIHS1_RHR           (SAM_TWIHS1_BASE+SAM_TWIHS_RHR_OFFSET)
#  define SAM_TWIHS1_THR           (SAM_TWIHS1_BASE+SAM_TWIHS_THR_OFFSET)
#  define SAM_TWIHS1_SMBTR         (SAM_TWIHS1_BASE+SAM_TWIHS_SMBTR_OFFSET)
#  define SAM_TWIHS1_FILTR         (SAM_TWIHS1_BASE+SAM_TWIHS_FILTR_OFFSET)
#  define SAM_TWIHS1_SWMR          (SAM_TWIHS1_BASE+SAM_TWIHS_SWMR_OFFSET)
#  define SAM_TWIHS1_WPMR          (SAM_TWIHS1_BASE+SAM_TWIHS_WPMR_OFFSET)
#  define SAM_TWIHS1_WPSR          (SAM_TWIHS1_BASE)+SAM_TWIHS_WPSR_OFFSET)
#endif

#if SAMV7_NTWIHS > 2
#  define SAM_TWIHS2_CR            (SAM_TWIHS2_BASE+SAM_TWIHS_CR_OFFSET)
#  define SAM_TWIHS2_MMR           (SAM_TWIHS2_BASE+SAM_TWIHS_MMR_OFFSET)
#  define SAM_TWIHS2_SMR           (SAM_TWIHS2_BASE+SAM_TWIHS_SMR_OFFSET)
#  define SAM_TWIHS2_IADR          (SAM_TWIHS2_BASE+SAM_TWIHS_IADR_OFFSET)
#  define SAM_TWIHS2_CWGR          (SAM_TWIHS2_BASE+SAM_TWIHS_CWGR_OFFSET)
#  define SAM_TWIHS2_SR            (SAM_TWIHS2_BASE+SAM_TWIHS_SR_OFFSET)
#  define SAM_TWIHS2_IER           (SAM_TWIHS2_BASE+SAM_TWIHS_IER_OFFSET)
#  define SAM_TWIHS2_IDR           (SAM_TWIHS2_BASE+SAM_TWIHS_IDR_OFFSET)
#  define SAM_TWIHS2_IMR           (SAM_TWIHS2_BASE+SAM_TWIHS_IMR_OFFSET)
#  define SAM_TWIHS2_RHR           (SAM_TWIHS2_BASE+SAM_TWIHS_RHR_OFFSET)
#  define SAM_TWIHS2_THR           (SAM_TWIHS2_BASE+SAM_TWIHS_THR_OFFSET)
#  define SAM_TWIHS2_SMBTR         (SAM_TWIHS2_BASE+SAM_TWIHS_SMBTR_OFFSET)
#  define SAM_TWIHS2_FILTR         (SAM_TWIHS2_BASE+SAM_TWIHS_FILTR_OFFSET)
#  define SAM_TWIHS2_SWMR          (SAM_TWIHS2_BASE+SAM_TWIHS_SWMR_OFFSET)
#  define SAM_TWIHS2_WPMR          (SAM_TWIHS2_BASE+SAM_TWIHS_WPMR_OFFSET)
#  define SAM_TWIHS2_WPSR          (SAM_TWIHS2_BASE)+SAM_TWIHS_WPSR_OFFSET)
#endif

/* TWIHS register bit definitions *********************************************************/
/* TWIHS Control Register */

#define TWIHS_CR_START             (1 << 0)  /* Bit 0:  Send SAMV7_NTWIHS START Condition */
#define TWIHS_CR_STOP              (1 << 1)  /* Bit 1:  Send SAMV7_NTWIHS STOP Condition */
#define TWIHS_CR_MSEN              (1 << 2)  /* Bit 2:  TWIHS Master Mode Enabled */
#define TWIHS_CR_MSDIS             (1 << 3)  /* Bit 3:  TWIHS Master Mode Disabled */
#define TWIHS_CR_SVEN              (1 << 4)  /* Bit 4:  TWIHS Slave Mode Enabled */
#define TWIHS_CR_SVDIS             (1 << 5)  /* Bit 5:  TWIHS Slave Mode Disabled */
#define TWIHS_CR_QUICK             (1 << 6)  /* Bit 6:  SMBUS Quick Command */
#define TWIHS_CR_SWRST             (1 << 7)  /* Bit 7:  Software Reset */
#define TWIHS_CR_HSEN              (1 << 8)  /* Bit 8:  TWIHS High-Speed Mode Enabled */
#define TWIHS_CR_HSDIS             (1 << 9)  /* Bit 9:  TWIHS High-Speed Mode Disabled */
#define TWIHS_CR_SMBEN             (1 << 10) /* Bit 10: SMBus Mode Enabled */
#define TWIHS_CR_SMBDIS            (1 << 11) /* Bit 11: SMBus Mode Disabled */
#define TWIHS_CR_PECEN             (1 << 12) /* Bit 12: Packet Error Checking Enable */
#define TWIHS_CR_PECDIS            (1 << 13) /* Bit 13: Packet Error Checking Disable */
#define TWIHS_CR_PECRQ             (1 << 14) /* Bit 14: PEC Request */
#define TWIHS_CR_CLEAR             (1 << 15) /* Bit 15: Bus CLEAR Command */

/* TWIHS Master Mode Register */

#define TWIHS_MMR_IADRSZ_SHIFT     (8)      /* Bits 8-9:  Internal Device Address Size */
#define TWIHS_MMR_IADRSZ_MASK      (3 << TWIHS_MMR_IADRSZ_SHIFT)
#  define TWIHS_MMR_IADRSZ_NONE    (0 << TWIHS_MMR_IADRSZ_SHIFT) /* No internal device address */
#  define TWIHS_MMR_IADRSZ_1BYTE   (1 << TWIHS_MMR_IADRSZ_SHIFT) /* One-byte internal device address */
#  define TWIHS_MMR_IADRSZ_2BYTE   (2 << TWIHS_MMR_IADRSZ_SHIFT) /* Two-byte internal device address */
#  define TWIHS_MMR_IADRSZ_3BYTE   (3 << TWIHS_MMR_IADRSZ_SHIFT) /* Three-byte internal device address */
#define TWIHS_MMR_MREAD            (1 << 12) /* Bit 12: Master Read Direction */
#define TWIHS_MMR_DADR_SHIFT       (16)      /* Bits 16-22:  Device Address */
#define TWIHS_MMR_DADR_MASK        (0x7f << TWIHS_MMR_DADR_SHIFT)
#  define TWIHS_MMR_DADR(n)        ((uint32_t)(n) << TWIHS_MMR_DADR_SHIFT)

/* TWIHS Slave Mode Register */

#define TWIHS_SMR_NACKEN           (1 << 0)  /* Bit 0:  Slave Receiver Data Phase NACK enable */
#define TWIHS_SMR_SMDA             (1 << 2)  /* Bit 2:  SMBus Default Address */
#define TWIHS_SMR_SMHH             (1 << 3)  /* Bit 3:  SMBus Host Header */
#define TWIHS_SMR_SCLWSDIS         (1 << 6)  /* Bit 6:  Clock Wait State Disable */
#define TWIHS_SMR_MASK_SHIFT       (8)       /* Bits 8-14: Slave Address Mask */
#define TWIHS_SMR_MASK_MASK        (0x7f << TWIHS_SMR_MASK_SHIFT)
#  define TWIHS_SMR_MASK(n)        ((uint32_t)(n) << TWIHS_SMR_MASK_SHIFT)
#define TWIHS_SMR_SADR_SHIFT       (16)      /* Bits 16-22:  Slave Address */
#define TWIHS_SMR_SADR_MASK        (0x7f << TWIHS_SMR_SADR_SHIFT)
#  define TWIHS_SMR_SADR(n)        ((uint32_t)(n) << TWIHS_SMR_SADR_SHIFT)
#define TWIHS_SMR_SADR1EN          (1 << 28) /* Bit 28:  Slave Address 1 Enable */
#define TWIHS_SMR_SADR2EN          (1 << 29) /* Bit 29:  Slave Address 2 Enable */
#define TWIHS_SMR_SADR3EN          (1 << 20) /* Bit 20:  Slave Address 3 Enable */
#define TWIHS_SMR_DATAMEN          (1 << 21) /* Bit 21:  Data Matching Enable */

/* TWIHS Internal Address Register */

#define TWIHS_IADR_SHIFT           (0)      /* Bits 0-23:  Internal Address */
#define TWIHS_IADR_MASK            (0x00ffffff << TWIHS_IADR_SHIFT)

/* TWIHS Clock Waveform Generator Register */

#define TWIHS_CWGR_CLDIV_SHIFT     (0)       /* Bits 0-7:  Clock Low Divider */
#define TWIHS_CWGR_CLDIV_MASK      (0xff << TWIHS_CWGR_CLDIV_SHIFT)
#  define TWIHS_CWGR_CLDIV(n)      ((uint32_t)(n) << TWIHS_CWGR_CLDIV_SHIFT)
#define TWIHS_CWGR_CHDIV_SHIFT     (8)       /* Bits 8-15:  Clock High Divider */
#define TWIHS_CWGR_CHDIV_MASK      (0xff << TWIHS_CWGR_CLDIV_SHIFT)
#  define TWIHS_CWGR_CHDIV(n)      ((uint32_t)(n) << TWIHS_CWGR_CLDIV_SHIFT)
#define TWIHS_CWGR_CKDIV_SHIFT     (16)      /* Bits 16-18:  Clock Divider */
#define TWIHS_CWGR_CKDIV_MASK      (7 << TWIHS_CWGR_CLDIV_SHIFT)
#  define TWIHS_CWGR_CKDIV(n)      ((uint32_t)(n) << TWIHS_CWGR_CLDIV_SHIFT)
#define TWIHS_CWGR_HOLD_SHIFT      (24)      /* Bits 24-28: TWD Hold Time Versus TWCK Falling */
#define TWIHS_CWGR_HOLD_MASK       (31 << TWIHS_CWGR_HOLD_SHIFT)
#  define TWIHS_CWGR_HOLD(n)       ((uint32_t)(n) << TWIHS_CWGR_HOLD_SHIFT)

/* TWIHS Status Register, TWIHS Interrupt Enable Register, TWIHS Interrupt Disable
 * Register, and TWIHS Interrupt Mask Register common bit fields.
 */

#define TWIHS_INT_TXCOMP           (1 << 0)  /* Bit 0:  Transmission Completed */
#define TWIHS_INT_RXRDY            (1 << 1)  /* Bit 1:  Receive Holding Register */
#define TWIHS_INT_TXRDY            (1 << 2)  /* Bit 2:  Transmit Holding Register Ready */
#define TWIHS_SR_SVREAD            (1 << 3)  /* Bit 3:  Slave Read (SR only) */
#define TWIHS_INT_SVACC            (1 << 4)  /* Bit 4:  Slave Access */
#define TWIHS_INT_GACC             (1 << 5)  /* Bit 5:  General Call Access */
#define TWIHS_INT_OVRE             (1 << 6)  /* Bit 6:  Overrun Error */
#define TWIHS_INT_UNRE             (1 << 7)  /* Bit 7:  Underrun Error */
#define TWIHS_INT_NACK             (1 << 8)  /* Bit 8:  Not Acknowledged */
#define TWIHS_INT_ARBLST           (1 << 9)  /* Bit 9:  Arbitration Lost */
#define TWIHS_INT_SCLWS            (1 << 10) /* Bit 10: Clock Wait State */
#define TWIHS_INT_EOSACC           (1 << 11) /* Bit 11: End Of Slave Access */
#define TWIHS_INT_MCACK            (1 << 16) /* Bit 16: Master Code Acknowledge */
#define TWIHS_INT_TOUT             (1 << 18) /* Bit 18: Timeout Error */
#define TWIHS_INT_PECERR           (1 << 19) /* Bit 19: PEC Error */
#define TWIHS_INT_SMBDAM           (1 << 20) /* Bit 20: SMBus Default Address Match */
#define TWIHS_INT_SMBHHM           (1 << 21) /* Bit 21: SMBus Host Header Address Match */
#define TWIHS_INT_SCL              (1 << 24) /* Bit 24: SCL Line Value (SR only) */
#define TWIHS_INT_SDA              (1 << 25) /* Bit 25: SDA Line Value (SR only) */

#define TWIHS_INT_ERRORS           0x000c03c0
#define TWIHS_INT_ALL              0x033d0fff

/* TWIHS Receive Holding Register */

#define TWIHS_RHR_RXDATA_SHIFT     (0)       /* Bits 0-7: Master or Slave Receive Holding Data */
#define TWIHS_RHR_RXDATA_MASK      (0xff << TWIHS_RHR_RXDATA_SHIFT)

/* TWIHS Transmit Holding Register */

#define TWIHS_THR_TXDATA_SHIFT     (0)       /* Bits 0-7: Master or Slave Transmit Holding Data */
#define TWIHS_THR_TXDATA_MASK      (0xff << TWIHS_THR_TXDATA_SHIFT)


/* SMBus Timing Register */

#define TWIHS_SMBTR_PRESC_SHIFT    (0)       /* Bits 0-3: SMBus Clock Prescaler */
#define TWIHS_SMBTR_PRESC_MASK     (15 << TWIHS_SMBTR_PRESC_SHIFT)
#  define TWIHS_SMBTR_PRESC(n)     ((uint32_t)(n) << TWIHS_SMBTR_PRESC_SHIFT)
#define TWIHS_SMBTR_TLOWS_SHIFT    (8)       /* Bits 8-15: Slave Clock Stretch Maximum  */Cycles
#define TWIHS_SMBTR_TLOWS_MASK     (0xff << TWIHS_SMBTR_TLOWS_SHIFT)
#  define TWIHS_SMBTR_TLOWS(n)     ((uint32_t)(n) << TWIHS_SMBTR_TLOWS_SHIFT)
#define TWIHS_SMBTR_TLOWM_SHIFT    (16)      /* Bits 16-23: Master Clock Stretch  */Maximum Cycles
#define TWIHS_SMBTR_TLOWM_MASK     (0xff << TWIHS_SMBTR_TLOWM_SHIFT)
#  define TWIHS_SMBTR_TLOWM(n)     ((uint32_t)(n) << TWIHS_SMBTR_TLOWM_SHIFT)
#define TWIHS_SMBTR_THMAX_SHIFT    (24)      /* Bits 24-24: Clock High Maximum Cycles */
#define TWIHS_SMBTR_THMAX_MASK     (0xff << TWIHS_SMBTR_THMAX_SHIFT)
#  define TWIHS_SMBTR_THMAX(n)     ((uint32_t)(n) << TWIHS_SMBTR_THMAX_SHIFT)

/* Filter Register */

#define TWIHS_FILTR_FILT           (1 << 0)  /* Bit 0:  RX Digital Filter */
#define TWIHS_FILTR_PADFEN         (1 << 1)  /* Bit 1:  PAD Filter Enable */
#define TWIHS_FILTR_PADFCFG        (1 << 2)  /* Bit 2:  PAD Filter Config */
#define TWIHS_FILTR_THRES_SHIFT    (8)       /* Bits 8-10: Digital Filter Threshold */
#define TWIHS_FILTR_THRES_MASK     (7 << TWIHS_FILTR_THRES_SHIFT)
#  define TWIHS_FILTR_THRES(n)     ((uint32_t)(n) << TWIHS_FILTR_THRES_SHIFT)

/* SleepWalking Matching Register */

#define TWIHS_SWMR_SADR1_SHIFT     (0)       /* Bits 0-6:  Slave Address 1 */
#define TWIHS_SWMR_SADR1_MASK      (0x7f << TWIHS_SWMR_SADR1_SHIFT)
#  define TWIHS_SWMR_SADR1(n)      ((uint32_t)(n) << TWIHS_SWMR_SADR1_SHIFT)
#define TWIHS_SWMR_SADR2_SHIFT     (8)       /* Bits 8-24:  Slave Address 2 */
#define TWIHS_SWMR_SADR2_MASK      (0x7f << TWIHS_SWMR_SADR2_SHIFT)
#  define TWIHS_SWMR_SADR2(n)      ((uint32_t)(n) << TWIHS_SWMR_SADR2_SHIFT)
#define TWIHS_SWMR_SADR3_SHIFT     (16)      /* Bits 16-22:  Slave Address 3 */
#define TWIHS_SWMR_SADR3_MASK      (0x7f << TWIHS_SWMR_SADR3_SHIFT)
#  define TWIHS_SWMR_SADR3(n)      ((uint32_t)(n) << TWIHS_SWMR_SADR3_SHIFT)
#define TWIHS_SWMR_DATAM_SHIFT     (24)      /* Bits 24-31:  Data Match */
#define TWIHS_SWMR_DATAM_MASK      (0xff << TWIHS_SWMR_DATAM_SHIFT)
#  define TWIHS_SWMR_DATAM(n)      ((uint32_t)(n) << TWIHS_SWMR_DATAM_SHIFT)

/* Protection Mode Register */

#define TWIHS_WPMR_WPEN            (1 << 0)  /* Bit 0:  Write Protect Enable */
#define TWIHS_WPMR_WPKEY_SHIFT     (8)       /* Bits 8-31: Write Protect Key */
#define TWIHS_WPMR_WPKEY_MASK      (0x00ffffff << TWIHS_WPMR_WPKEY_SHIFT)
#  define TWIHS_WPMR_WPKEY         (0x00545749 << TWIHS_WPMR_WPKEY_SHIFT)

/* Protection Status Register */

#define TWIHS_WPSR_WPVS            (1 << 0)  /* Bit 0:  Write Protect Violation Status */
#define TWIHS_WPSR_WPVSRC_SHIFT    (8)       /* Bits 8-23: Write Protect Violation Source */
#define TWIHS_WPSR_WPVSRC_MASK     (0xffff << TWIHS_WPSR_WPVSRC_SHIFT)

/****************************************************************************************
 * Public Types
 ****************************************************************************************/

/****************************************************************************************
 * Public Data
 ****************************************************************************************/

/****************************************************************************************
 * Public Functions
 ****************************************************************************************/

#endif /* SAMV7_NTWIHS > 0 */
#endif /* __ARCH_ARM_SRC_SAMV7_HARDWARE_SAM_TWIHS_H */
