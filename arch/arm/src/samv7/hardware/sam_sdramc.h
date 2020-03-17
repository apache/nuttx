/****************************************************************************************
 * arch/arm/src/samv7/hardware/sam_sdramc.h
 * SDRAM Controller (SDRAMC) definitions for the SAMV71
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

#ifndef __ARCH_ARM_SRC_SAMV7_HARDWARE_SAM_SDRAMC_H
#define __ARCH_ARM_SRC_SAMV7_HARDWARE_SAM_SDRAMC_H

/****************************************************************************************
 * Included Files
 ****************************************************************************************/

#include <nuttx/config.h>
#include <arch/samv7/chip.h>

#include "hardware/sam_memorymap.h"

/****************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************/

/* SDRAMC register offsets **************************************************************/

#define SAM_SDRAMC_MR_OFFSET         0x0000 /* SDRAMC Mode Register */
#define SAM_SDRAMC_TR_OFFSET         0x0004 /* SDRAMC Refresh Timer Register */
#define SAM_SDRAMC_CR_OFFSET         0x0008 /* SDRAMC Configuration Register */
#define SAM_SDRAMC_LPR_OFFSET        0x0010 /* SDRAMC Low Power Register */
#define SAM_SDRAMC_IER_OFFSET        0x0014 /* SDRAMC Interrupt Enable Register */
#define SAM_SDRAMC_IDR_OFFSET        0x0018 /* SDRAMC Interrupt Disable Register */
#define SAM_SDRAMC_IMR_OFFSET        0x001c /* SDRAMC Interrupt Mask Register */
#define SAM_SDRAMC_ISR_OFFSET        0x0020 /* SDRAMC Interrupt Status Register */
#define SAM_SDRAMC_MDR_OFFSET        0x0024 /* SDRAMC Memory Device Register */
#define SAM_SDRAMC_CFR1_OFFSET       0x0028 /* SDRAMC Configuration Register 1 */
#define SAM_SDRAMC_OCMS_OFFSET       0x002c /* SDRAMC OCMS Register */
#define SAM_SDRAMC_OCMS_KEY1_OFFSET  0x0030 /* SDRAMC OCMS KEY1 Register */
#define SAM_SDRAMC_OCMS_KEY2_OFFSET  0x0034 /* SDRAMC OCMS KEY2 Register */

/* SDRAMC register addresses ************************************************************/

#define SAM_SDRAMC_MR                (SAM_SDRAMC_BASE+SAM_SDRAMC_MR_OFFSET)
#define SAM_SDRAMC_TR                (SAM_SDRAMC_BASE+SAM_SDRAMC_TR_OFFSET)
#define SAM_SDRAMC_CR                (SAM_SDRAMC_BASE+SAM_SDRAMC_CR_OFFSET)
#define SAM_SDRAMC_LPR               (SAM_SDRAMC_BASE+SAM_SDRAMC_LPR_OFFSET)
#define SAM_SDRAMC_IER               (SAM_SDRAMC_BASE+SAM_SDRAMC_IER_OFFSET)
#define SAM_SDRAMC_IDR               (SAM_SDRAMC_BASE+SAM_SDRAMC_IDR_OFFSET)
#define SAM_SDRAMC_IMR               (SAM_SDRAMC_BASE+SAM_SDRAMC_IMR_OFFSET)
#define SAM_SDRAMC_ISR               (SAM_SDRAMC_BASE+SAM_SDRAMC_ISR_OFFSET)
#define SAM_SDRAMC_MDR               (SAM_SDRAMC_BASE+SAM_SDRAMC_MDR_OFFSET)
#define SAM_SDRAMC_CFR1              (SAM_SDRAMC_BASE+SAM_SDRAMC_CFR1_OFFSET)
#define SAM_SDRAMC_OCMS              (SAM_SDRAMC_BASE+SAM_SDRAMC_OCMS_OFFSET)
#define SAM_SDRAMC_OCMS_KEY1         (SAM_SDRAMC_BASE+SAM_SDRAMC_OCMS_KEY1_OFFSET)
#define SAM_SDRAMC_OCMS_KEY2         (SAM_SDRAMC_BASE+SAM_SDRAMC_OCMS_KEY2_OFFSET)

/* SDRAMC register bit definitions ******************************************************/

/* SDRAMC Mode Register */

#define SDRAMC_MR_MODE_SHIFT         (0)       /* Bits 0-2: SDRAMC Command Mode */
#define SDRAMC_MR_MODE_MASK          (7 << SDRAMC_MR_MODE_SHIFT)
#  define SDRAMC_MR_MODE_NORMAL      (0 << SDRAMC_MR_MODE_SHIFT) /* Normal mode */
#  define SDRAMC_MR_MODE_NOP         (1 << SDRAMC_MR_MODE_SHIFT) /* NOP when SDRAM accessed */
#  define SDRAMC_MR_MODE_PRECHARGE   (2 << SDRAMC_MR_MODE_SHIFT) /* All Banks Precharge when SDRAM accessed */
#  define SDRAMC_MR_MODE_LOADMODE    (3 << SDRAMC_MR_MODE_SHIFT) /* Load Mode Register when SDRAM accessed */
#  define SDRAMC_MR_MODE_AUTOREFRESH (4 << SDRAMC_MR_MODE_SHIFT) /* Auto-Refresh when SDRAM accessed */
#  define SDRAMC_MR_MODE_EXTLOADMODE (5 << SDRAMC_MR_MODE_SHIFT) /* Extended Load Mode Register when SDRAM accessed */
#  define SDRAMC_MR_MODE_PWRDOWN     (6 << SDRAMC_MR_MODE_SHIFT) /* Deep power-down mode */

/* SDRAMC Refresh Timer Register */

#define SDRAMC_TR_MASK               0x00000fff /* Bits 0-11: SDRAMC Refresh Timer Count */

/* SDRAMC Configuration Register */

#define SDRAMC_CR_NC_SHIFT           (0)       /* Bits 0-1: Number of Column Bits */
#define SDRAMC_CR_NC_MASK            (3 << SDRAMC_CR_NC_SHIFT)
#  define SDRAMC_CR_NC_COL8          (0 << SDRAMC_CR_NC_SHIFT) /* 8 column bits */
#  define SDRAMC_CR_NC_COL9          (1 << SDRAMC_CR_NC_SHIFT) /* 9 column bits */
#  define SDRAMC_CR_NC_COL10         (2 << SDRAMC_CR_NC_SHIFT) /* 10 column bits */
#  define SDRAMC_CR_NC_COL11         (3 << SDRAMC_CR_NC_SHIFT) /* 11 column bits */
#define SDRAMC_CR_NR_SHIFT           (2)       /* Bits 2-3: Number of Row Bits */
#define SDRAMC_CR_NR_MASK            (3 << SDRAMC_CR_NR_SHIFT)
#  define SDRAMC_CR_NR_ROW11         (0 << SDRAMC_CR_NR_SHIFT) /* 11 row bits */
#  define SDRAMC_CR_NR_ROW12         (1 << SDRAMC_CR_NR_SHIFT) /* 12 row bits */
#  define SDRAMC_CR_NR_ROW13         (2 << SDRAMC_CR_NR_SHIFT) /* 13 row bits */
#define SDRAMC_CR_NB                 (1 << 4)  /* Bit 4:  Number of Banks */
#  define SDRAMC_CR_NB_BANK2         (0 << 4)  /*  0=2 banks */
#  define SDRAMC_CR_NB_BANK4         (1 << 4)  /*  1=4 banks */
#define SDRAMC_CR_CAS_SHIFT          (5)       /* Bits 5-6: CAS Latency */
#define SDRAMC_CR_CAS_MASK           (3 << SDRAMC_CR_CAS_SHIFT)
#  define SDRAMC_CR_CAS_LATENCY1     (0 << SDRAMC_CR_CAS_SHIFT) /* 1 cycle CAS latency */
#  define SDRAMC_CR_CAS_LATENCY2     (1 << SDRAMC_CR_CAS_SHIFT) /* 2 cycle CAS latency */
#  define SDRAMC_CR_CAS_LATENCY3     (2 << SDRAMC_CR_CAS_SHIFT) /* 3 cycle CAS latency */
#define SDRAMC_CR_DBW                (1 << 7)  /* Bit 7:  Data Bus Width */
#define SDRAMC_CR_TWR_SHIFT          (8)       /* Bits 8-11: Write Recovery Delay */
#define SDRAMC_CR_TWR_MASK           (15 << SDRAMC_CR_TWR_SHIFT)
#  define SDRAMC_CR_TWR(n)           ((uint32_t)(n) << SDRAMC_CR_TWR_SHIFT)
#define SDRAMC_CR_TRCTRFC_SHIFT      (12)      /* Bits 12-15: Row Cycle Delay and Row Refresh Cycle */
#define SDRAMC_CR_TRCTRFC_MASK       (15 << SDRAMC_CR_TRCTRFC_SHIFT)
#  define SDRAMC_CR_TRCTRFC(n)       ((uint32_t)(n) << SDRAMC_CR_TRCTRFC_SHIFT)
#define SDRAMC_CR_TRP_SHIFT          (16)      /* Bits 16-19: Row Precharge Delay */
#define SDRAMC_CR_TRP_MASK           (15 << SDRAMC_CR_TRP_SHIFT)
#  define SDRAMC_CR_TRP(n)           ((uint32_t)(n) << SDRAMC_CR_TRP_SHIFT)
#define SDRAMC_CR_TRCD_SHIFT         (20)      /* Bits 20-23: Row to Column Delay */
#define SDRAMC_CR_TRCD_MASK          (15 << SDRAMC_CR_TRCD_SHIFT)
#  define SDRAMC_CR_TRCD(n)          ((uint32_t)(n) << SDRAMC_CR_TRCD_SHIFT)
#define SDRAMC_CR_TRAS_SHIFT         (24)      /* Bits 24-27: Active to Precharge Delay */
#define SDRAMC_CR_TRAS_MASK          (15 << SDRAMC_CR_TRAS_SHIFT)
#  define SDRAMC_CR_TRAS(n)          ((uint32_t)(n) << SDRAMC_CR_TRAS_SHIFT)
#define SDRAMC_CR_TXSR_SHIFT         (28)      /* Bits 28-31: Exit Self Refresh to Active Delay */
#define SDRAMC_CR_TXSR_MASK          (15 << SDRAMC_CR_TXSR_SHIFT)
#  define SDRAMC_CR_TXSR(n)          ((uint32_t)(n) << SDRAMC_CR_TXSR_SHIFT)

/* SDRAMC Low Power Register */

#define SDRAMC_LPR_LPCB_SHIFT        (0)       /* Bits 0-1: Low-power Configuration Bits */
#define SDRAMC_LPR_LPCB_MASK         (3 << SDRAMC_LPR_LPCB_SHIFT)
#  define SDRAMC_LPR_LPCB_DISABLED   (0 << SDRAMC_LPR_LPCB_SHIFT) /* Low Power Feature is inhibited */
#  define SDRAMC_LPR_LPCB_REFRESH    (1 << SDRAMC_LPR_LPCB_SHIFT) /* Self-refresh to SDRAM device */
#  define SDRAMC_LPR_LPCB_PWRDOWN    (2 << SDRAMC_LPR_LPCB_SHIFT) /* Power-down to SDRAM after accesses */
#  define SDRAMC_LPR_LPCB_DPPWRDOWN  (3 << SDRAMC_LPR_LPCB_SHIFT) /* Deep Power-down the SDRAM device */
#define SDRAMC_LPR_PASR_SHIFT        (4)       /* Bits 4-6: Partial Array Self-refresh */
#define SDRAMC_LPR_PASR_MASK         (7 << SDRAMC_LPR_PASR_SHIFT)
#define SDRAMC_LPR_TCSR_SHIFT        (8)       /* Bits 8-9:  Temperature Compensated Self-Refresh */
#define SDRAMC_LPR_TCSR_MASK         (3 << SDRAMC_LPR_TCSR_SHIFT)
#  define SDRAMC_LPR_TCSR(n)         ((uint32_t)(n) << SDRAMC_LPR_TCSR_SHIFT)
#define SDRAMC_LPR_DS_SHIFT          (10)      /* Bits 10-11: Drive Strength */
#define SDRAMC_LPR_DS_MASK           (3 << SDRAMC_LPR_DS_SHIFT)
#  define SDRAMC_LPR_DS(n)           ((uint32_t)(n) << SDRAMC_LPR_DS_SHIFT)
#define SDRAMC_LPR_TIMEOUT_SHIFT     (12)      /* Bits 12-13: Time to Define When Low-power Mode Is Enabled */
#define SDRAMC_LPR_TIMEOUT_MASK      (3 << SDRAMC_LPR_TIMEOUT_SHIFT)
#  define SDRAMC_LPR_TIMEOUT_LP      (0 << SDRAMC_LPR_TIMEOUT_SHIFT) /* SDRAM low-power mode immediately */
#  define SDRAMC_LPR_TIMEOUT_LP64    (1 << SDRAMC_LPR_TIMEOUT_SHIFT) /* SDRAM low-power mode after 64 cycles */
#  define SDRAMC_LPR_TIMEOUT_LP128   (2 << SDRAMC_LPR_TIMEOUT_SHIFT) /* SDRAM low-power mode 128 cycles */

/* SDRAMC Interrupt Enable Register,  SDRAMC Interrupt Disable Register, SDRAMC
 * Interrupt Mask Register, and SDRAMC Interrupt Status Register.
 */

#define SDRAMC_INT_RES               (1 << 0)  /* Bit 0:  Refresh Error */

/* SDRAMC Memory Device Register */

#define SDRAMC_MDR_SHIFT             (0)       /* Bits 0-1:  Memory Device Type */
#define SDRAMC_MDR_MASK              (3 << SDRAMC_MDR_SHIFT)
#  define SDRAMC_MDR_SDRAM           (0 << SDRAMC_MDR_SHIFT) /* SDRAM */
#  define SDRAMC_MDR_LPSDRAM         (1 << SDRAMC_MDR_SHIFT) /* Low-power SDRAM */

/* SDRAMC Configuration Register 1 */

#define SDRAMC_CFR1_TMRD_SHIFT       (0)       /* Bits 0-3: Load Mode Register to Active/Refresh Command */
#define SDRAMC_CFR1_TMRD_MASK        (15 << SDRAMC_CFR1_TMRD_SHIFT)
#  define SDRAMC_CFR1_TMRD(n)        ((uint32_t)(n) << SDRAMC_CFR1_TMRD_SHIFT)
#define SDRAMC_CFR1_UNAL             (1 << 8)  /* Bit 8: Support Unaligned Access */
#  define SDRAMC_CFR1_UNAL_UNSUPP    (0 << 8)  /*   0=Unaligned access is not supported */
#  define SDRAMC_CFR1_UNAL_SUPPORTED (1 << 8)  /*   1=Unaligned access is supported */

/* SDRAMC OCMS Register */

#define SDRAMC_OCMS_SDRSE            (1 << 0)  /* Bit 9:  SDRAM Memory Controller Scrambling Enable */

/* SDRAMC OCMS KEY1 Register (32-bit value) */
/* SDRAMC OCMS KEY2 Register (32-bit value) */

/****************************************************************************************
 * Public Types
 ****************************************************************************************/

/****************************************************************************************
 * Public Data
 ****************************************************************************************/

/****************************************************************************************
 * Public Functions
 ****************************************************************************************/

#endif /* __ARCH_ARM_SRC_SAMV7_HARDWARE_SAM_SDRAMC_H */
