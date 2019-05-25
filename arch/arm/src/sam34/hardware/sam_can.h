/****************************************************************************************
 * arch/arm/src/sam34/hardware/sam_can.h
 * Controller Area Network (CAN) for the SAM4E
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_SAM34_HARDWARE_SAM_CAN_H
#define __ARCH_ARM_SRC_SAM34_HARDWARE_SAM_CAN_H

/****************************************************************************************
 * Included Files
 ****************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "hardware/sam_memorymap.h"

/****************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************/

#define SAM_CAN_NMBOXES              8      /* 8 Mailboxes */
#define SAM_CAN_MBOX(n)              (n)
#define SAM_CAN_MBOX0                0
#define SAM_CAN_MBOX1                1
#define SAM_CAN_MBOX2                2
#define SAM_CAN_MBOX3                3
#define SAM_CAN_MBOX4                4
#define SAM_CAN_MBOX5                5
#define SAM_CAN_MBOX6                6
#define SAM_CAN_MBOX7                7

/* CAN register offsets *****************************************************************/

#define SAM_CAN_MR_OFFSET            0x0000 /* Mode Register */
#define SAM_CAN_IER_OFFSET           0x0004 /* Interrupt Enable Register */
#define SAM_CAN_IDR_OFFSET           0x0008 /* Interrupt Disable Register */
#define SAM_CAN_IMR_OFFSET           0x000c /* Interrupt Mask Register */
#define SAM_CAN_SR_OFFSET            0x0010 /* Status Register */
#define SAM_CAN_BR_OFFSET            0x0014 /* Baudrate Register */
#define SAM_CAN_TIM_OFFSET           0x0018 /* Timer Register */
#define SAM_CAN_TIMESTP_OFFSET       0x001c /* Timestamp Register */
#define SAM_CAN_ECR_OFFSET           0x0020 /* Error Counter Register */
#define SAM_CAN_TCR_OFFSET           0x0024 /* Transfer Command Register */
#define SAM_CAN_ACR_OFFSET           0x0028 /* Abort Command Register */
                                     /* 0x002c-0x00e0: Reserved */
#define SAM_CAN_WPMR_OFFSET          0x00e4 /* Write Protect Mode Register */
#define SAM_CAN_WPSR_OFFSET          0x00e8 /* Write Protect Status Register */
                                     /* 0x00eC-0x01fc: Reserved */
/* Mailbox Registers */

#define SAM_CAN_MBOX_OFFSET(n)       (0x0200+((n) << 5))
#define SAM_CAN_MMR_OFFSET           0x0000 /* Mailbox Mode Register */
#define SAM_CAN_MAM_OFFSET           0x0004 /* Mailbox Acceptance Mask Register */
#define SAM_CAN_MID_OFFSET           0x0008 /* Mailbox ID Register */
#define SAM_CAN_MFID_OFFSET          0x000c /* Mailbox Family ID Register */
#define SAM_CAN_MSR_OFFSET           0x0010 /* Mailbox Status Register */
#define SAM_CAN_MDL_OFFSET           0x0014 /* Mailbox Data Low Register */
#define SAM_CAN_MDH_OFFSET           0x0018 /* Mailbox Data High Register */
#define SAM_CAN_MCR_OFFSET           0x001c /* Mailbox Control Register */

/* CAN register addresses ***************************************************************/

#define SAM_CAN0_MR                  (SAM_CAN0_BASE+SAM_CAN_MR_OFFSET)
#define SAM_CAN0_IER                 (SAM_CAN0_BASE+SAM_CAN_IER_OFFSET)
#define SAM_CAN0_IDR                 (SAM_CAN0_BASE+SAM_CAN_IDR_OFFSET)
#define SAM_CAN0_IMR                 (SAM_CAN0_BASE+SAM_CAN_IMR_OFFSET)
#define SAM_CAN0_SR                  (SAM_CAN0_BASE+SAM_CAN_SR_OFFSET)
#define SAM_CAN0_BR                  (SAM_CAN0_BASE+SAM_CAN_BR_OFFSET)
#define SAM_CAN0_TIM                 (SAM_CAN0_BASE+SAM_CAN_TIM_OFFSET)
#define SAM_CAN0_TIMESTP             (SAM_CAN0_BASE+SAM_CAN_TIMESTP_OFFSET)
#define SAM_CAN0_ECR                 (SAM_CAN0_BASE+SAM_CAN_ECR_OFFSET)
#define SAM_CAN0_TCR                 (SAM_CAN0_BASE+SAM_CAN_TCR_OFFSET)
#define SAM_CAN0_ACR                 (SAM_CAN0_BASE+SAM_CAN_ACR_OFFSET)
#define SAM_CAN0_WPMR                (SAM_CAN0_BASE+SAM_CAN_WPMR_OFFSET)
#define SAM_CAN0_WPSR                (SAM_CAN0_BASE+SAM_CAN_WPSR_OFFSET)

/* Mailbox Registers */

#define SAM_CAN0_MBOX_BASE(n)        (SAM_CAN0_BASE+SAM_CAN_MBOX_OFFSET(n))
#define SAM_CAN0_MMR(n)              (SAM_CAN0_MBOX_BASE(n)+SAM_CAN_MMR_OFFSET)
#define SAM_CAN0_MAM(n)              (SAM_CAN0_MBOX_BASE(n)+SAM_CAN_MAM_OFFSET)
#define SAM_CAN0_MID(n)              (SAM_CAN0_MBOX_BASE(n)+SAM_CAN_MID_OFFSET)
#define SAM_CAN0_MFID(n)             (SAM_CAN0_MBOX_BASE(n)+SAM_CAN_MFID_OFFSET)
#define SAM_CAN0_MSR(n)              (SAM_CAN0_MBOX_BASE(n)+SAM_CAN_MSR_OFFSET)
#define SAM_CAN0_MDL(n)              (SAM_CAN0_MBOX_BASE(n)+SAM_CAN_MDL_OFFSET)
#define SAM_CAN0_MDH(n)              (SAM_CAN0_MBOX_BASE(n)+SAM_CAN_MDH_OFFSET)
#define SAM_CAN0_MCR(n)              (SAM_CAN0_MBOX_BASE(n)+SAM_CAN_MCR_OFFSET)

#define SAM_CAN1_MR                  (SAM_CAN1_BASE+SAM_CAN_MR_OFFSET)
#define SAM_CAN1_IER                 (SAM_CAN1_BASE+SAM_CAN_IER_OFFSET)
#define SAM_CAN1_IDR                 (SAM_CAN1_BASE+SAM_CAN_IDR_OFFSET)
#define SAM_CAN1_IMR                 (SAM_CAN1_BASE+SAM_CAN_IMR_OFFSET)
#define SAM_CAN1_SR                  (SAM_CAN1_BASE+SAM_CAN_SR_OFFSET)
#define SAM_CAN1_BR                  (SAM_CAN1_BASE+SAM_CAN_BR_OFFSET)
#define SAM_CAN1_TIM                 (SAM_CAN1_BASE+SAM_CAN_TIM_OFFSET)
#define SAM_CAN1_TIMESTP             (SAM_CAN1_BASE+SAM_CAN_TIMESTP_OFFSET)
#define SAM_CAN1_ECR                 (SAM_CAN1_BASE+SAM_CAN_ECR_OFFSET)
#define SAM_CAN1_TCR                 (SAM_CAN1_BASE+SAM_CAN_TCR_OFFSET)
#define SAM_CAN1_ACR                 (SAM_CAN1_BASE+SAM_CAN_ACR_OFFSET)
#define SAM_CAN1_WPMR                (SAM_CAN1_BASE+SAM_CAN_WPMR_OFFSET)
#define SAM_CAN1_WPSR                (SAM_CAN1_BASE+SAM_CAN_WPSR_OFFSET)

/* Mailbox Registers */

#define SAM_CAN1_MBOX_BASE(n)        (SAM_CAN1_BASE+SAM_CAN_MBOX_OFFSET(n))
#define SAM_CAN1_MMR(n)              (SAM_CAN1_MBOX_BASE(n)+SAM_CAN_MMR_OFFSET)
#define SAM_CAN1_MAM(n)              (SAM_CAN1_MBOX_BASE(n)+SAM_CAN_MAM_OFFSET)
#define SAM_CAN1_MID(n)              (SAM_CAN1_MBOX_BASE(n)+SAM_CAN_MID_OFFSET)
#define SAM_CAN1_MFID(n)             (SAM_CAN1_MBOX_BASE(n)+SAM_CAN_MFID_OFFSET)
#define SAM_CAN1_MSR(n)              (SAM_CAN1_MBOX_BASE(n)+SAM_CAN_MSR_OFFSET)
#define SAM_CAN1_MDL(n)              (SAM_CAN1_MBOX_BASE(n)+SAM_CAN_MDL_OFFSET)
#define SAM_CAN1_MDH(n)              (SAM_CAN1_MBOX_BASE(n)+SAM_CAN_MDH_OFFSET)
#define SAM_CAN1_MCR(n)              (SAM_CAN1_MBOX_BASE(n)+SAM_CAN_MCR_OFFSET)

/* CAN register bit definitions *********************************************************/

/* Mode Register */

#define CAN_MR_CANEN                 (1 << 0)  /* Bit 0:  CAN controller enable */
#define CAN_MR_LPM                   (1 << 1)  /* Bit 1:  Disable/enable low power mode */
#define CAN_MR_ABM                   (1 << 2)  /* Bit 2:  Disable/enable autobaud/listen mode */
#define CAN_MR_OVL                   (1 << 3)  /* Bit 3:  Disable/enable overload frame */
#define CAN_MR_TEOF                  (1 << 4)  /* Bit 4:  Timestamp messages at each end of frame */
#define CAN_MR_TTM                   (1 << 5)  /* Bit 5:  Disable/enable time triggered mode */
#define CAN_MR_TIMFRZ                (1 << 6)  /* Bit 6:  Enable timer freeze */
#define CAN_MR_DRPT                  (1 << 7)  /* Bit 7:  Disable repeat */

/* Interrupt Enable, Interrupt Disable, Interrupt Mask and Status Register */

#define CAN_INT_MB(n)                (1 << (n)) /* Bit n: Mailbox n Interrupt */
#define CAN_INT_ERRA                 (1 << 16) /* Bit 16: Error Active Mode Interrupt */
#define CAN_INT_WARN                 (1 << 17) /* Bit 17: Warning Limit Interrupt */
#define CAN_INT_ERRP                 (1 << 18) /* Bit 18: Error Passive Mode Interrupt */
#define CAN_INT_BOFF                 (1 << 19) /* Bit 19: Bus Off Mode Interrupt */
#define CAN_INT_SLEEP                (1 << 20) /* Bit 20: Sleep Interrupt */
#define CAN_INT_WAKEUP               (1 << 21) /* Bit 21: Wake-up Interrupt */
#define CAN_INT_TOVF                 (1 << 22) /* Bit 22: Timer Overflow Interrupt */
#define CAN_INT_TSTP                 (1 << 23) /* Bit 23: TimeStamp Interrupt */
#define CAN_INT_CERR                 (1 << 24) /* Bit 24: CRC Error Interrupt */
#define CAN_INT_SERR                 (1 << 25) /* Bit 25: Stuffing Error Interrupt */
#define CAN_INT_AERR                 (1 << 26) /* Bit 26: Acknowledgement Error Interrupt */
#define CAN_INT_FERR                 (1 << 27) /* Bit 27: Form Error Interrupt */
#define CAN_INT_BERR                 (1 << 28) /* Bit 28: Bit Error Interrupt */

#define CAN_SR_RBSY                  (1 << 29) /* Bit 29: Receiver busy (SR only) */
#define CAN_SR_TBSY                  (1 << 30) /* Bit 30: Transmitter busy (SR only) */
#define CAN_SR_OVLSY                 (1 << 31) /* Bit 31: Overload busy (SR only) */

/* Baudrate Register */

#define CAN_BR_PHASE2_SHIFT          (0)       /* Bits 0-2: Phase 2 segment */
#define CAN_BR_PHASE2_MASK           (7 << CAN_BR_PHASE2_SHIFT)
#  define CAN_BR_PHASE2(n)           ((uint32_t)(n) << CAN_BR_PHASE2_SHIFT)
#define CAN_BR_PHASE1_SHIFT          (4)       /* Bits 4-6: Phase 1 segment */
#define CAN_BR_PHASE1_MASK           (7 << CAN_BR_PHASE1_SHIFT)
#  define CAN_BR_PHASE1(n)           ((uint32_t)(n) << CAN_BR_PHASE1_SHIFT)
#define CAN_BR_PROPAG_SHIFT          (8)       /* Bits 8-10: Programming time segment */
#define CAN_BR_PROPAG_MASK           (7 << CAN_BR_PROPAG_SHIFT)
#  define CAN_BR_PROPAG(n)           ((uint32_t)(n) << CAN_BR_PROPAG_SHIFT)
#define CAN_BR_SJW_SHIFT             (12)      /* Bits 12-13: Re-synchronization jump width */
#define CAN_BR_SJW_MASK              (3 << CAN_BR_SJW_SHIFT)
#  define CAN_BR_SJW(n)              ((uint32_t)(n) << CAN_BR_SJW_SHIFT)
#define CAN_BR_BRP_SHIFT             (16)      /* Bits 16-22: Baudrate Prescaler */
#define CAN_BR_BRP_MASK              (127 << CAN_BR_BRP_SHIFT)
#  define CAN_BR_BRP(n)              ((uint32_t)(n) << CAN_BR_BRP_SHIFT)
#define CAN_BR_SMP                   (1 << 24) /* Bit 24: Sampling Mode

/* Timer Register */

#define CAN_TIM_MASK                 (0x0000ffff) /* Bits 0-15: Timer */

/* Timestamp Register */

#define CAN_TIMESTP_MASK             (0x0000ffff) /* Bits 0-15: Timestamp */

/* Error Counter Register */

#define CAN_ECR_REC_SHIFT            (0)       /* Bits 0-7: Receive Error Counter */
#define CAN_ECR_REC_MASK             (0xff << CAN_ECR_REC_SHIFT)
#  define CAN_ECR_REC(n)             ((uint32_t)(n) << CAN_ECR_REC_SHIFT)
#define CAN_ECR_TEC_SHIFT            (16)      /* Bits 16-23: Transmit Error Counter */
#define CAN_ECR_TEC_MASK             (0xff << CAN_ECR_TEC_SHIFT)
#  define CAN_ECR_TEC(n)             ((uint32_t)(n) << CAN_ECR_TEC_SHIFT)

/* Transfer Command Register */

#define CAN_TCR_MB(n)                (1 << (n)) /* Bit (n): Transfer Request for Mailbox n */
#define CAN_TCR_TIMRST               (1 << 31)  /* Bit 31: Timer Reset */

/* Abort Command Register */

#define CAN_ACR_MB(n)                (1 << (n)) /* Bit (n): Abort Request for Mailbox n */

/* Write Protect Mode Register */

#define CAN_WPMR_WPEN                (1 << 0)  /* Bit 0: Write Protection Enable */
#define CAN_WPMR_WPKEY_SHIFT         (8)       /* Bits 8-31: SPI Write Protection Key Password */
#define CAN_WPMR_WPKEY_MASK          (0x00ffffff << CAN_WPMR_WPKEY_SHIFT)
#  define CAN_WPMR_WPKEY             (0x0043414e << CAN_WPMR_WPKEY_SHIFT)

/* Write Protect Status Register */

#define CAN_WPSR_WPVS                (1 << 0)  /* Bit 0: Write Protection Violation Status */
#define CAN_WPSR_WPVSRC_SHIFT        (8)       /* Bits 8-23: Write Protection Violation Source */
#define CAN_WPSR_WPVSRC_MASK         (0x0000ffff << CAN_WPSR_WPVSRC_SHIFT)

/* Mailbox Registers */

/* Mailbox Mode Register */

#define CAN_MMR_MTIMEMARK_SHIFT      (0)       /* Bits 0-15: Mailbox Timemark */
#define CAN_MMR_MTIMEMARK_MASK       (0x0000ffff << CAN_MMR_MTIMEMARK_SHIFT)
#  define CAN_MMR_MTIMEMARK(n)       ((uint32_t)(n) << CAN_MMR_MTIMEMARK_SHIFT)
#define CAN_MMR_PRIOR_SHIFT          (16)       /* Bits 16-19: Mailbox Priority */
#define CAN_MMR_PRIOR_MASK           (15 << CAN_MMR_PRIOR_SHIFT)
#  define CAN_MMR_PRIOR(n)           ((uint32_t)(n) << CAN_MMR_PRIOR_SHIFT)
#define CAN_MMR_MOT_SHIFT            (24)       /* Bits 24-26: Mailbox Object Type */
#define CAN_MMR_MOT_MASK             (7 << CAN_MMR_MOT_SHIFT)
#  define CAN_MMR_MOT_DISABLED       (0 << CAN_MMR_MOT_SHIFT) /* Mailbox is disabled */
#  define CAN_MMR_MOT_RX             (1 << CAN_MMR_MOT_SHIFT) /* Reception Mailbox */
#  define CAN_MMR_MOT_RXOVR          (2 << CAN_MMR_MOT_SHIFT) /* Reception mailbox with overwrite */
#  define CAN_MMR_MOT_TX             (3 << CAN_MMR_MOT_SHIFT) /* Transmit mailbox */
#  define CAN_MMR_MOT_CONSUMER       (4 << CAN_MMR_MOT_SHIFT) /* Consumer Mailbox */
#  define CAN_MMR_MOT_PRODUCER       (5 << CAN_MMR_MOT_SHIFT) /* Producer Mailbox */

/* Mailbox Acceptance Mask Register */

#define CAN_MAM_MIDvB_SHIFT          (0)       /* Bits 0-18: Complementary bits for ID in extended frame mode */
#define CAN_MAM_MIDvB_MASK           (0x0003ffff << CAN_MAM_MIDvB_SHIFT)
#  define CAN_MAM_MIDvB(n)           ((uint32_t)(n) << CAN_MAM_MIDvB_SHIFT)
#define CAN_MAM_MIDvA_SHIFT          (18)      /* Bits 18-28: ID for standard frame mode */
#define CAN_MAM_MIDvA_MASK           (0x000007ff << CAN_MAM_MIDvA_SHIFT)
#  define CAN_MAM_MIDvA(n)           ((uint32_t)(n) << CAN_MAM_MIDvA_SHIFT)
#define CAN_MAM_MIDE                 (1 << 29) /* Bit 29: ID Version */

/* Mailbox ID Register */

#define CAN_MID_MIDvB_SHIFT          (0)       /* Bits 0-18: Complementary bits for ID in extended frame mode */
#define CAN_MID_MIDvB_MASK           (0x0003ffff << CAN_MID_MIDvB_SHIFT)
#  define CAN_MID_MIDvB(n)           ((uint32_t)(n) << CAN_MID_MIDvB_SHIFT)
#define CAN_MID_MIDvA_SHIFT          (18)      /* Bits 18-28: ID for standard frame mode */
#define CAN_MID_MIDvA_MASK           (0x000007ff << CAN_MID_MIDvA_SHIFT)
#  define CAN_MID_MIDvA(n)           ((uint32_t)(n) << CAN_MID_MIDvA_SHIFT)
#define CAN_MID_MIDE                 (1 << 29) /* Bit 29: ID Version */

/* Mailbox Family ID Register */

#define CAN_MFID_MASK                (0x1fffffff) /* Bit 0-28: Family ID */

/* Mailbox Status Register */

#define CAN_MSR_MTIMESTAMP_SHIFT     (0)       /* Bits 0-15: Timer value */
#define CAN_MSR_MTIMESTAMP_MASK      (0x0000ffff << CAN_MSR_MTIMESTAMP_SHIFT)
#  define CAN_MSR_MTIMESTAMP(n)      ((uint32_t)(n) << CAN_MSR_MTIMESTAMP_SHIFT)
#define CAN_MSR_MDLC_SHIFT           (16)       /* Bits 16-19: Mailbox Data Length Code */
#define CAN_MSR_MDLC_MASK            (15 << CAN_MSR_MDLC_SHIFT)
#  define CAN_MSR_MDLC(n)            ((uint32_t)(n) << CAN_MSR_MDLC_SHIFT)
#define CAN_MSR_MRTR                 (1 << 20) /* Bit 20: Mailbox Remote Transmission Request */
#define CAN_MSR_MABT                 (1 << 22) /* Bit 22: Mailbox Message Abort */
#define CAN_MSR_MRDY                 (1 << 23) /* Bit 23: Mailbox Ready */
#define CAN_MSR_MMI                  (1 << 24) /* Bit 24: Mailbox Message Ignored */

/* Mailbox Data Low Register (32-bit value) */
/* Mailbox Data High Register (32-bit value) */

/* Mailbox Control Register */

#define CAN_MCR_MDLC_SHIFT           (16)      /* Bits 16-19: Mailbox Data Length Code */
#define CAN_MCR_MDLC_MASK            (15 << CAN_MCR_MDLC_SHIFT)
#  define CAN_MCR_MDLC(n)            ((uint32_t)(n) << CAN_MCR_MDLC_SHIFT)
#define CAN_MCR_MRTR                 (1 << 20) /* Bit 20: Mailbox Remote Transmission Request */
#define CAN_MCR_MACR                 (1 << 22) /* Bit 22: Abort Request for Mailbox n */
#define CAN_MCR_MTCR                 (1 << 23) /* Bit 23: Mailbox Transfer Command */

/****************************************************************************************
 * Public Types
 ****************************************************************************************/

/****************************************************************************************
 * Public Data
 ****************************************************************************************/

/****************************************************************************************
 * Public Functions
 ****************************************************************************************/

#endif /* __ARCH_ARM_SRC_SAM34_HARDWARE_SAM_CAN_H */
