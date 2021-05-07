/****************************************************************************
 * arch/arm/src/sama5/hardware/sam_can.h
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

#ifndef __ARCH_ARM_SRC_SAMA5_HARDWARE_SAM_CAN_H
#define __ARCH_ARM_SRC_SAMA5_HARDWARE_SAM_CAN_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/sam_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SAM_CAN_NMAILBOXES       8
#define SAM_CAN_MAXPERCLK        66000000

/* CAN Register Offsets *****************************************************/

#define SAM_CAN_MR_OFFSET        0x0000 /* Mode Register */
#define SAM_CAN_IER_OFFSET       0x0004 /* Interrupt Enable Register */
#define SAM_CAN_IDR_OFFSET       0x0008 /* Interrupt Disable Register */
#define SAM_CAN_IMR_OFFSET       0x000c /* Interrupt Mask Register */
#define SAM_CAN_SR_OFFSET        0x0010 /* Status Register */
#define SAM_CAN_BR_OFFSET        0x0014 /* Baudrate Register */
#define SAM_CAN_TIM_OFFSET       0x0018 /* Timer Register */
#define SAM_CAN_TIMESTP_OFFSET   0x001c /* Timestamp Register */
#define SAM_CAN_ECR_OFFSET       0x0020 /* Error Counter Register */
#define SAM_CAN_TCR_OFFSET       0x0024 /* Transfer Command Register */
#define SAM_CAN_ACR_OFFSET       0x0028 /* Abort Command Register */

#define SAM_CAN_WPMR_OFFSET      0x00e4 /* Write Protect Mode Register */
#define SAM_CAN_WPSR_OFFSET      0x00e8 /* Write Protect Status Register */

#define SAM_CAN_MBN_OFFSET(n)    (0x0200 + ((n) << 5))
#define SAM_CAN_MMR_OFFSET       0x0000 /* Mailbox Mode Register */
#define SAM_CAN_MAM_OFFSET       0x0004 /* Mailbox Acceptance Mask Register */
#define SAM_CAN_MID_OFFSET       0x0008 /* Mailbox ID Register */
#define SAM_CAN_MFID_OFFSET      0x000c /* Mailbox Family ID Register */
#define SAM_CAN_MSR_OFFSET       0x0010 /* Mailbox Status Register */
#define SAM_CAN_MDL_OFFSET       0x0014 /* Mailbox Data Low Register */
#define SAM_CAN_MDH_OFFSET       0x0018 /* Mailbox Data High Register */
#define SAM_CAN_MCR_OFFSET       0x001c /* Mailbox Control Register */

#define SAM_CAN_MNMR_OFFSET(n)   (SAM_CAN_MBN_OFFSET(n)+SAM_CAN_MMR_OFFSET)
#define SAM_CAN_MNAM_OFFSET(n)   (SAM_CAN_MBN_OFFSET(n)+SAM_CAN_MAM_OFFSET)
#define SAM_CAN_MNID_OFFSET(n)   (SAM_CAN_MBN_OFFSET(n)+SAM_CAN_MID_OFFSET)
#define SAM_CAN_MNFID_OFFSET(n)  (SAM_CAN_MBN_OFFSET(n)+SAM_CAN_MFID_OFFSET)
#define SAM_CAN_MNSR_OFFSET(n)   (SAM_CAN_MBN_OFFSET(n)+SAM_CAN_MSR_OFFSET)
#define SAM_CAN_MNDL_OFFSET(n)   (SAM_CAN_MBN_OFFSET(n)+SAM_CAN_MDL_OFFSET)
#define SAM_CAN_MNDH_OFFSET(n)   (SAM_CAN_MBN_OFFSET(n)+SAM_CAN_MDH_OFFSET)
#define SAM_CAN_MNCR_OFFSET(n)   (SAM_CAN_MBN_OFFSET(n)+SAM_CAN_MCR_OFFSET)

/* CAN Register Addresses ***************************************************/

#define SAM_CAN0_MR              (SAM_CAN0_VBASE+SAM_CAN_MR_OFFSET)
#define SAM_CAN0_IER             (SAM_CAN0_VBASE+SAM_CAN_IER_OFFSET)
#define SAM_CAN0_IDR             (SAM_CAN0_VBASE+SAM_CAN_IDR_OFFSET)
#define SAM_CAN0_IMR             (SAM_CAN0_VBASE+SAM_CAN_IMR_OFFSET)
#define SAM_CAN0_SR              (SAM_CAN0_VBASE+SAM_CAN_SR_OFFSET)
#define SAM_CAN0_BR              (SAM_CAN0_VBASE+SAM_CAN_BR_OFFSET)
#define SAM_CAN0_TIM             (SAM_CAN0_VBASE+SAM_CAN_TIM_OFFSET)
#define SAM_CAN0_TIMESTP         (SAM_CAN0_VBASE+SAM_CAN_TIMESTP_OFFSET)
#define SAM_CAN0_ECR             (SAM_CAN0_VBASE+SAM_CAN_ECR_OFFSET)
#define SAM_CAN0_TCR             (SAM_CAN0_VBASE+SAM_CAN_TCR_OFFSET)
#define SAM_CAN0_ACR             (SAM_CAN0_VBASE+SAM_CAN_ACR_OFFSET)

#define SAM_CAN0_WPMR            (SAM_CAN0_VBASE+SAM_CAN_WPMR_OFFSET)
#define SAM_CAN0_WPSR            (SAM_CAN0_VBASE+SAM_CAN_WPSR_OFFSET)

#define SAM_CAN0_MB_BASE(n)      (SAM_CAN0_VBASE+SAM_CAN_MBN_OFFSET(n))
#define SAM_CAN0_MMR(n)          (SAM_CAN0_VBASE+SAM_CAN_MNMR_OFFSET(n))
#define SAM_CAN0_MAM(n)          (SAM_CAN0_VBASE+SAM_CAN_MNAM_OFFSET(n))
#define SAM_CAN0_MID(n)          (SAM_CAN0_VBASE+SAM_CAN_MNID_OFFSET(n))
#define SAM_CAN0_MFID(n)         (SAM_CAN0_VBASE+SAM_CAN_MNFID_OFFSET(n))
#define SAM_CAN0_MSR(n)          (SAM_CAN0_VBASE+SAM_CAN_MNSR_OFFSET(n))
#define SAM_CAN0_MDL(n)          (SAM_CAN0_VBASE+SAM_CAN_MNDL_OFFSET(n))
#define SAM_CAN0_MDH(n)          (SAM_CAN0_VBASE+SAM_CAN_MNDH_OFFSET(n))
#define SAM_CAN0_MCR(n)          (SAM_CAN0_VBASE+SAM_CAN_MNCR_OFFSET(n))

#define SAM_CAN1_MR              (SAM_CAN1_VBASE+SAM_CAN_MR_OFFSET)
#define SAM_CAN1_IER             (SAM_CAN1_VBASE+SAM_CAN_IER_OFFSET)
#define SAM_CAN1_IDR             (SAM_CAN1_VBASE+SAM_CAN_IDR_OFFSET)
#define SAM_CAN1_IMR             (SAM_CAN1_VBASE+SAM_CAN_IMR_OFFSET)
#define SAM_CAN1_SR              (SAM_CAN1_VBASE+SAM_CAN_SR_OFFSET)
#define SAM_CAN1_BR              (SAM_CAN1_VBASE+SAM_CAN_BR_OFFSET)
#define SAM_CAN1_TIM             (SAM_CAN1_VBASE+SAM_CAN_TIM_OFFSET)
#define SAM_CAN1_TIMESTP         (SAM_CAN1_VBASE+SAM_CAN_TIMESTP_OFFSET)
#define SAM_CAN1_ECR             (SAM_CAN1_VBASE+SAM_CAN_ECR_OFFSET)
#define SAM_CAN1_TCR             (SAM_CAN1_VBASE+SAM_CAN_TCR_OFFSET)
#define SAM_CAN1_ACR             (SAM_CAN1_VBASE+SAM_CAN_ACR_OFFSET)

#define SAM_CAN1_WPMR            (SAM_CAN1_VBASE+SAM_CAN_WPMR_OFFSET)
#define SAM_CAN1_WPSR            (SAM_CAN1_VBASE+SAM_CAN_WPSR_OFFSET)

#define SAM_CAN1_MB_BASE(n)      (SAM_CAN1_VBASE+SAM_CAN_MBN_OFFSET(n))
#define SAM_CAN1_MMR(n)          (SAM_CAN1_VBASE+SAM_CAN_MNMR_OFFSET(n))
#define SAM_CAN1_MAM(n)          (SAM_CAN1_VBASE+SAM_CAN_MNAM_OFFSET(n))
#define SAM_CAN1_MID(n)          (SAM_CAN1_VBASE+SAM_CAN_MNID_OFFSET(n))
#define SAM_CAN1_MFID(n)         (SAM_CAN1_VBASE+SAM_CAN_MNFID_OFFSET(n))
#define SAM_CAN1_MSR(n)          (SAM_CAN1_VBASE+SAM_CAN_MNSR_OFFSET(n))
#define SAM_CAN1_MDL(n)          (SAM_CAN1_VBASE+SAM_CAN_MNDL_OFFSET(n))
#define SAM_CAN1_MDH(n)          (SAM_CAN1_VBASE+SAM_CAN_MNDH_OFFSET(n))
#define SAM_CAN1_MCR(n)          (SAM_CAN1_VBASE+SAM_CAN_MNCR_OFFSET(n))

/* CAN Register Bit Definitions *********************************************/

/* Mode Register */

#define CAN_MR_CANEN             (1 << 0)  /* Bit 0:  CAN Controller Enable */
#define CAN_MR_LPM               (1 << 1)  /* Bit 1:  Disable/Enable Low-power Mode */
#define CAN_MR_ABM               (1 << 2)  /* Bit 2:  Disable/Enable Autobaud/Listen mode */
#define CAN_MR_OVL               (1 << 3)  /* Bit 3:  Disable/Enable Overload Frame */
#define CAN_MR_TEOF              (1 << 4)  /* Bit 4:  Timestamp Messages at each End of Frame */
#define CAN_MR_TTM               (1 << 5)  /* Bit 5:  Disable/Enable Time Triggered Mode */
#define CAN_MR_TIMFRZ            (1 << 6)  /* Bit 6:  Enable Timer Freeze */
#define CAN_MR_DRPT              (1 << 7)  /* Bit 7:  Disable Repeat */

/* Interrupt Enable Register, Interrupt Disable Register,
 * Interrupt Mask Register, and Status Register
 */

#define CAN_INT_MB(n)            (1 << (n)) /* Bit n:  Mailbox n Event */
#define CAN_INT_MB0              (1 << 0)   /* Bit 0:  Mailbox 0 Event */
#define CAN_INT_MB1              (1 << 1)   /* Bit 1:  Mailbox 1 Event */
#define CAN_INT_MB2              (1 << 2)   /* Bit 2:  Mailbox 2 Event */
#define CAN_INT_MB3              (1 << 3)   /* Bit 3:  Mailbox 3 Event */
#define CAN_INT_MB4              (1 << 4)   /* Bit 4:  Mailbox 4 Event */
#define CAN_INT_MB5              (1 << 5)   /* Bit 5:  Mailbox 5 Event */
#define CAN_INT_MB6              (1 << 6)   /* Bit 6:  Mailbox 6 Event */
#define CAN_INT_MB7              (1 << 7)   /* Bit 7:  Mailbox 7 Event */
#define CAN_INT_MBALL            (0x000000ff)

#define CAN_INT_ERRA             (1 << 16) /* Bit 16: Error Active Mode */
#define CAN_INT_WARN             (1 << 17) /* Bit 17: Warning Limit */
#define CAN_INT_ERRP             (1 << 18) /* Bit 18: Error Passive Mode */
#define CAN_INT_BOFF             (1 << 19) /* Bit 19: Bus Off Mode */
#define CAN_INT_SLEEP            (1 << 20) /* Bit 20: CAN Controller in Low-power Mode */
#define CAN_INT_WAKEUP           (1 << 21) /* Bit 21: Wake-up Interrupt */
#define CAN_INT_TOVF             (1 << 22) /* Bit 22: Timer Overflow */
#define CAN_INT_TSTP             (1 << 23) /* Bit 23: Timestamp */
#define CAN_INT_CERR             (1 << 24) /* Bit 24: Mailbox CRC Error */
#define CAN_INT_SERR             (1 << 25) /* Bit 25: Mailbox Stuffing Error */
#define CAN_INT_AERR             (1 << 26) /* Bit 26: Acknowledgment Error */
#define CAN_INT_FERR             (1 << 27) /* Bit 27: Form Error */
#define CAN_INT_BERR             (1 << 28) /* Bit 28: Bit Error */
#define CAN_INT_ALLERRORS        (0x1f000000)
#define CAN_INT_ALL              (0x1fff00ff)

#define CAN_SR_RBSY              (1 << 29) /* Bit 29: Receiver busy */
#define CAN_SR_TBSY              (1 << 30) /* Bit 30: Transmitter busy */
#define CAN_SR_OVLSY             (1 << 31) /* Bit 31: Overload busy */

/* Baudrate Register */

#define CAN_BR_PHASE2_SHIFT      (0)      /* Bits 0-2: Phase 2 segment */
#define CAN_BR_PHASE2_MASK       (7 << CAN_BR_PHASE2_SHIFT)
#  define CAN_BR_PHASE2(n)       ((uint32_t)(n) << CAN_BR_PHASE2_SHIFT)
#define CAN_BR_PHASE1_SHIFT      (4)      /* Bits 4-6: Phase 1 segment */
#define CAN_BR_PHASE1_MASK       (7 << CAN_BR_PHASE1_SHIFT)
#  define CAN_BR_PHASE1(n)       ((uint32_t)(n) << CAN_BR_PHASE1_SHIFT)
#define CAN_BR_PROPAG_SHIFT      (8)      /* Bits 8-10: Programming time segment */
#define CAN_BR_PROPAG_MASK       (7 << CAN_BR_PROPAG_SHIFT)
#  define CAN_BR_PROPAG(n)       ((uint32_t)(n) << CAN_BR_PROPAG_SHIFT)
#define CAN_BR_SJW_SHIFT         (12)     /* Bits 12-13: Re-synchronization jump width */
#define CAN_BR_SJW_MASK          (3 << CAN_BR_SJW_SHIFT)
#  define CAN_BR_SJW(n)          ((uint32_t)(n) << CAN_BR_SJW_SHIFT)
#define CAN_BR_BRP_SHIFT         (16)     /* Bits 16-22: Baudrate Prescaler */
#define CAN_BR_BRP_MASK          (0x7f << CAN_BR_BRP_SHIFT)
#  define CAN_BR_BRP(n)          ((uint32_t)(n) << CAN_BR_BRP_SHIFT)
#define CAN_BR_SMP               (1 << 24)  /* Bit 24: Sampling Mode */
#  define CAN_BR_ONCE            (0)        /* Bit 24: 0:Bit stream sampled once at sample point */
#  define CAN_BR_THREE           CAN_BR_SMP /* Bit 24: 1:Sampling three times */

/* Timer Register */

#define CAN_TIM_MASK             (0xffff) /* Bit 0-15: Timer */

/* Timestamp Register */

#define CAN_TIMESTP_MASK         (0xffff) /* Bit 0-15: Timestamp */

/* Error Counter Register */

#define CAN_ECR_REC_SHIFT        (0)      /* Bits 0-7: Receive Error Counter */
#define CAN_ECR_REC_MASK         (0xff << CAN_ECR_REC_SHIFT)
#define CAN_ECR_TEC_SHIFT        (16)     /* Bits 16-24: Transmit Error Counter */
#define CAN_ECR_TEC_MASK         (0x1ff << CAN_ECR_TEC_SHIFT)

/* Transfer Command Register */

#define CAN_TCR_MB(n)            (1 << (n)) /* Bit n: Transfer Request for Mailbox n */

#define CAN_TCR_MB0              (1 << 0)  /* Bit 0:  Transfer Request for Mailbox 0 */
#define CAN_TCR_MB1              (1 << 1)  /* Bit 1:  Transfer Request for Mailbox 1 */
#define CAN_TCR_MB2              (1 << 2)  /* Bit 2:  Transfer Request for Mailbox 2 */
#define CAN_TCR_MB3              (1 << 3)  /* Bit 3:  Transfer Request for Mailbox 3 */
#define CAN_TCR_MB4              (1 << 4)  /* Bit 4:  Transfer Request for Mailbox 4 */
#define CAN_TCR_MB5              (1 << 5)  /* Bit 5:  Transfer Request for Mailbox 5 */
#define CAN_TCR_MB6              (1 << 6)  /* Bit 6:  Transfer Request for Mailbox 6 */
#define CAN_TCR_MB7              (1 << 7)  /* Bit 7:  Transfer Request for Mailbox 7 */
#define CAN_TCR_TIMRST           (1 << 31) /* Bit 31: Timer Reset */

/* Abort Command Register */

#define CAN_ACR_MB(n)            (1 << (n)) /* Bit n: Abort Request for Mailbox n */

#define CAN_ACR_MB0              (1 << 0)  /* Bit 0:  Abort Request for Mailbox 0 */
#define CAN_ACR_MB1              (1 << 1)  /* Bit 1:  Abort Request for Mailbox 1 */
#define CAN_ACR_MB2              (1 << 2)  /* Bit 2:  Abort Request for Mailbox 2 */
#define CAN_ACR_MB3              (1 << 3)  /* Bit 3:  Abort Request for Mailbox 3 */
#define CAN_ACR_MB4              (1 << 4)  /* Bit 4:  Abort Request for Mailbox 4 */
#define CAN_ACR_MB5              (1 << 5)  /* Bit 5:  Abort Request for Mailbox 5 */
#define CAN_ACR_MB6              (1 << 6)  /* Bit 6:  Abort Request for Mailbox 6 */
#define CAN_ACR_MB7              (1 << 7)  /* Bit 6:  Abort Request for Mailbox 7 */

/* Write Protect Mode Register */

#define CAN_WPMR_WPEN            (1 << 0)  /* Bit 0:  Write Protection Enable */
#define CAN_WPMR_WPKEY_SHIFT     (8)       /* Bits 8-31: CAN Write Protection Key Password */
#define CAN_WPMR_WPKEY_MASK      (0xffffff << CAN_WPMR_WPKEY_SHIFT)
#  define CAN_WPMR_WPKEY         (0x43414e << CAN_WPMR_WPKEY_SHIFT) /* "CAN" in ASCII */

/* Write Protect Status Register */

#define CAN_WPSR_WPVS            (1 << 0)  /* Bit 0:  Write Protection Violation Status */
#define CAN_WPSR_WPVSRC_SHIFT    (8)       /* Bits 8-15: Write Protection Violation Source */
#define CAN_WPSR_WPVSRC_MASK     (0xff << CAN_WPSR_WPVSRC_SHIFT)

/* Mailbox Mode Register */

#define CAN_MMR_MTIMEMARK_SHIFT  (0)      /* Bits 0-15: Mailbox Timemark */
#define CAN_MMR_MTIMEMARK_MASK   (0xffff << CAN_MMR_MTIMEMARK_SHIFT)
#  define CAN_MMR_MTIMEMARK(n)   ((uint32_t)(n) << 19)
#define CAN_MMR_PRIOR_SHIFT      (16)      /* Bits 16-19: Mailbox Priority */
#define CAN_MMR_PRIOR_MASK       (15 << CAN_MMR_PRIOR_SHIFT)
#  define CAN_MMR_PRIOR(n)       ((uint32_t)(n) << CAN_MMR_PRIOR_SHIFT)
#define CAN_MMR_MOT_SHIFT        (24)      /* Bits 24-26: Mailbox Object Type */
#define CAN_MMR_MOT_MASK         (7 << CAN_MMR_MOT_SHIFT)
#  define CAN_MMR_MOT_DISABLED   (0 << CAN_MMR_MOT_SHIFT) /* Mailbox is disabled */
#  define CAN_MMR_MOT_RX         (1 << CAN_MMR_MOT_SHIFT) /* Reception Mailbox */
#  define CAN_MMR_MOT_RXOVRWR    (2 << CAN_MMR_MOT_SHIFT) /* Reception mailbox with overwrite */
#  define CAN_MMR_MOT_TX         (3 << CAN_MMR_MOT_SHIFT) /* Transmit mailbox */
#  define CAN_MMR_MOT_CONSUMER   (4 << CAN_MMR_MOT_SHIFT) /* Consumer Mailbox */
#  define CAN_MMR_MOT_PRODUCER   (5 << CAN_MMR_MOT_SHIFT) /* Producer Mailbox */

/* Mailbox Acceptance Mask Register */

#define CAN_MAM_MIDvB_SHIFT      (0)       /* Bits 0-17: Complementary bits for identifier */
#define CAN_MAM_MIDvB_MASK       (0x3ffff << CAN_MAM_MIDvB_SHIFT)
#  define CAN_MAM_MIDvB(n)       ((uint32_t)(n) << CAN_MAM_MIDvB_SHIFT)
#define CAN_MAM_MIDvA_SHIFT      (18)      /* Bits 18-28: Identifier for standard frame mode */
#define CAN_MAM_MIDvA_MASK       (0x7ff << CAN_MAM_MIDvA_SHIFT)
#  define CAN_MAM_MIDvA(n)       ((uint32_t)(n) << CAN_MAM_MIDvA_SHIFT)
#define CAN_MAM_MIDE             (1 << 29) /* Bit 29: Identifier Version */

#define CAN_MAM_EXTID_SHIFT      (0)       /* Bits 0-28: 29-bit extended address */
#define CAN_MAM_EXTID_MASK       (0x1fffffff << CAN_MAM_EXTID_SHIFT)
#  define CAN_MAM_EXTID(n)       (((uint32_t)(n) << CAN_MAM_EXTID_SHIFT) | CAN_MAM_MIDE)
#define CAN_MAM_STDID_SHIFT      (18)      /* Bits 18-28: 11-bit standard address */
#define CAN_MAM_STDID_MASK       (0x7ff << CAN_MAM_STDID_SHIFT)
#  define CAN_MAM_STDID(n)       ((uint32_t)(n) << CAN_MAM_STDID_SHIFT)

/* Mailbox ID Register */

#define CAN_MID_MIDvB_SHIFT      (0)       /* Bits 0-17: Complementary bits for identifier */
#define CAN_MID_MIDvB_MASK       (0x3ffff << CAN_MID_MIDvB_SHIFT)
#  define CAN_MID_MIDvB(n)       ((uint32_t)(n) << CAN_MID_MIDvB_SHIFT)
#define CAN_MID_MIDvA_SHIFT      (18)      /* Bits 18-28: Identifier for standard frame mode */
#define CAN_MID_MIDvA_MASK       (0x7ff << CAN_MID_MIDvA_SHIFT)
#  define CAN_MID_MIDvA(n)       ((uint32_t)(n) << CAN_MID_MIDvA_SHIFT)
#define CAN_MID_MIDE             (1 << 29) /* Bit 19: Identifier Version */

#define CAN_MID_EXTID_SHIFT      (0)       /* Bits 0-28: 29-bit extended address */
#define CAN_MID_EXTID_MASK       (0x1fffffff << CAN_MID_EXTID_SHIFT)
#  define CAN_MID_EXTID(n)       (((uint32_t)(n) << CAN_MID_EXTID_SHIFT) | CAN_MID_MIDE)
#define CAN_MID_STDID_SHIFT      (18)      /* Bits 18-28: 11-bit standard address */
#define CAN_MID_STDID_MASK       (0x7ff << CAN_MID_STDID_SHIFT)
#  define CAN_MID_STDID(n)       ((uint32_t)(n) << CAN_MID_STDID_SHIFT)

/* Mailbox Family ID Register */

#define CAN_MFID_MASK            (0x1fffffff)

/* Mailbox Status Register */

#define CAN_MSR_MTIMESTAMP_SHIFT (0)      /* Bits 0-15: Timer value */
#define CAN_MSR_MTIMESTAMP_MASK  (0xffff << CAN_MSR_MTIMESTAMP_SHIFT)
#define CAN_MSR_MDLC_SHIFT       (16)      /* Bits 16-19: Mailbox Data Length Code */
#define CAN_MSR_MDLC_MASK        (15 << CAN_MSR_MDLC_SHIFT)
#define CAN_MSR_MRTR             (1 << 20) /* Bit 20: Mailbox Remote Transmission Request */
#define CAN_MSR_MABT             (1 << 22) /* Bit 22: Mailbox Message Abort */
#define CAN_MSR_MRDY             (1 << 23) /* Bit 23: Mailbox Ready */
#define CAN_MSR_MMI              (1 << 24) /* Bit 24: Mailbox Message Ignored */

/* Mailbox Data Low Register and Mailbox Data High Register.
 * Bytes are received/sent on the bus in the following order:
 *
 *   1. CAN_MDL[7:0]
 *   2. CAN_MDL[15:8]
 *   3. CAN_MDL[23:16]
 *   4. CAN_MDL[31:24]
 *   5. CAN_MDH[7:0]
 *   6. CAN_MDH[15:8]
 *   7. CAN_MDH[23:16]
 *   8. CAN_MDH[31:24]
 */

#define CAN_MDL0_SHIFT           (0)       /* Bits 0-7:   Byte 0 */
#define CAN_MDL0_MASK            (0xff << CAN_MDL0_SHIFT)
#  define CAN_MDL0(n)            ((uint32_t)(n) << CAN_MDL0_SHIFT)
#define CAN_MDL1_SHIFT           (8)       /* Bits 8-15:  Byte 1 */
#define CAN_MDL1_MASK            (0xff << CAN_MDL1_SHIFT)
#  define CAN_MDL1(n)            ((uint32_t)(n) << CAN_MDL1_SHIFT)
#define CAN_MDL2_SHIFT           (16)      /* Bits 16-23: Byte 2 */
#define CAN_MDL2_MASK            (0xff << CAN_MDL2_SHIFT)
#  define CAN_MDL2(n)            ((uint32_t)(n) << CAN_MDL2_SHIFT)
#define CAN_MDL3_SHIFT           (24)      /* Bits 24-31: Byte 3 */
#define CAN_MDL3_MASK            (0xff << CAN_MDL3_SHIFT)
#  define CAN_MDL3(n)            ((uint32_t)(n) << CAN_MDL3_SHIFT)

#define CAN_MDH4_SHIFT           (0)       /* Bits 0-7:   Byte 4 */
#define CAN_MDH4_MASK            (0xff << CAN_MDH4_SHIFT)
#  define CAN_MDH4(n)            ((uint32_t)(n) << CAN_MDH4_SHIFT)
#define CAN_MDH5_SHIFT           (8)       /* Bits 8-15:  Byte 5 */
#define CAN_MDH5_MASK            (0xff << CAN_MDH5_SHIFT)
#  define CAN_MDH5(n)            ((uint32_t)(n) << CAN_MDH5_SHIFT)
#define CAN_MDH6_SHIFT           (16)      /* Bits 16-23: Byte 6 */
#define CAN_MDH6_MASK            (0xff << CAN_MDH6_SHIFT)
#  define CAN_MDH6(n)            ((uint32_t)(n) << CAN_MDH6_SHIFT)
#define CAN_MDH7_SHIFT           (24)      /* Bits 24-31: Byte 7 */
#define CAN_MDH7_MASK            (0xff << CAN_MDH7_SHIFT)
#  define CAN_MDH7(n)            ((uint32_t)(n) << CAN_MDH7_SHIFT)

/* Mailbox Control Register */

#define CAN_MCR_MDLC_SHIFT       (16)      /* Bits 16-19: Mailbox Data Length Code */
#define CAN_MCR_MDLC_MASK        (15 << CAN_MCR_MDLC_SHIFT)
#  define CAN_MCR_MDLC(n)        ((uint32_t)(n) << CAN_MCR_MDLC_SHIFT)
#define CAN_MCR_MRTR             (1 << 20) /* Bit 20: Mailbox Remote Transmission Request */
#define CAN_MCR_MACR             (1 << 22) /* Bit 22: Abort Request for Mailbox n */
#define CAN_MCR_MTCR             (1 << 23) /* Bit 23: Mailbox Transfer Command */

#endif /* __ARCH_ARM_SRC_SAMA5_HARDWARE_SAM_CAN_H */
