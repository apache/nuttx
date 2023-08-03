/****************************************************************************
 * arch/arm/src/imxrt/hardware/imxrt_flexcan.h
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

#ifndef __ARCH_ARM_SRC_IMXRT_HARDWARE_IMXRT_FLEXCAN_H
#define __ARCH_ARM_SRC_IMXRT_HARDWARE_IMXRT_FLEXCAN_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define IMXRT_CAN_MCR_OFFSET      0x0000 /* Module Configuration Register */
#define IMXRT_CAN_CTRL1_OFFSET    0x0004 /* Control 1 Register */
#define IMXRT_CAN_TIMER_OFFSET    0x0008 /* Free Running Timer */
#define IMXRT_CAN_RXMGMASK_OFFSET 0x0010 /* Rx Mailboxes Global Mask Register */
#define IMXRT_CAN_RX14MASK_OFFSET 0x0014 /* Rx 14 Mask Register */
#define IMXRT_CAN_RX15MASK_OFFSET 0x0018 /* Rx 15 Mask Register */
#define IMXRT_CAN_ECR_OFFSET      0x001c /* Error Counter */
#define IMXRT_CAN_ESR1_OFFSET     0x0020 /* Error and Status 1 Register */
#define IMXRT_CAN_IMASK2_OFFSET   0x0024 /* Interrupt Masks 2 Register */
#define IMXRT_CAN_IMASK1_OFFSET   0x0028 /* Interrupt Masks 1 Register */
#define IMXRT_CAN_IFLAG2_OFFSET   0x002c /* Interrupt Flags 2 Register */
#define IMXRT_CAN_IFLAG1_OFFSET   0x0030 /* Interrupt Flags 1 Register */
#define IMXRT_CAN_CTRL2_OFFSET    0x0034 /* Control 2 Register */
#define IMXRT_CAN_ESR2_OFFSET     0x0038 /* Error and Status 2 Register */
#define IMXRT_CAN_CRCR_OFFSET     0x0044 /* CRC Register */
#define IMXRT_CAN_RXFGMASK_OFFSET 0x0048 /* Rx FIFO Global Mask Register */
#define IMXRT_CAN_RXFIR_OFFSET    0x004c /* Rx FIFO Information Register */
#define IMXRT_CAN_CBT_OFFSET      0x0050 /* CAN Bit Timing Register */

#define IMXRT_CAN_MB_OFFSET       0x0080 /* CAN MB register */
#define IMXRT_CAN_MB_SIZE         0x0A60
#define IMXRT_CAN_MB_END          (IMXRT_CAN_MB_OFFSET + IMXRT_CAN_MB_SIZE)

#define IMXRT_CAN_MB2_OFFSET      0x0F28 /* CAN MB2 register */
#define IMXRT_CAN_MB2_SIZE        0x00D8
#define IMXRT_CAN_MB2_END         (IMXRT_CAN_MB2_OFFSET + IMXRT_CAN_MB2_SIZE)

#define IMXRT_CAN_RXIMR_OFFSET(n) (0x0880+((n)<<2)) /* Rn Individual Mask Registers */
#define IMXRT_CAN_RXIMR0_OFFSET   0x0880            /* R0 Individual Mask Registers */
#define IMXRT_CAN_RXIMR1_OFFSET   0x0884            /* R1 Individual Mask Registers */
#define IMXRT_CAN_RXIMR2_OFFSET   0x0888            /* R2 Individual Mask Registers */
#define IMXRT_CAN_RXIMR3_OFFSET   0x088c            /* R3 Individual Mask Registers */
#define IMXRT_CAN_RXIMR4_OFFSET   0x0890            /* R4 Individual Mask Registers */
#define IMXRT_CAN_RXIMR5_OFFSET   0x0894            /* R5 Individual Mask Registers */
#define IMXRT_CAN_RXIMR6_OFFSET   0x0898            /* R6 Individual Mask Registers */
#define IMXRT_CAN_RXIMR7_OFFSET   0x089c            /* R7 Individual Mask Registers */
#define IMXRT_CAN_RXIMR8_OFFSET   0x08a0            /* R8 Individual Mask Registers */
#define IMXRT_CAN_RXIMR9_OFFSET   0x08a4            /* R9 Individual Mask Registers */
#define IMXRT_CAN_RXIMR10_OFFSET  0x08a8            /* R10 Individual Mask Registers */
#define IMXRT_CAN_RXIMR11_OFFSET  0x08ac            /* R11 Individual Mask Registers */
#define IMXRT_CAN_RXIMR12_OFFSET  0x08b0            /* R12 Individual Mask Registers */
#define IMXRT_CAN_RXIMR13_OFFSET  0x08b4            /* R13 Individual Mask Registers */
#define IMXRT_CAN_RXIMR14_OFFSET  0x08b8            /* R14 Individual Mask Registers */
#define IMXRT_CAN_RXIMR15_OFFSET  0x08bc            /* R15 Individual Mask Registers */
#define IMXRT_CAN_RXIMR16_OFFSET  0x08c0            /* R16 Individual Mask Registers */
#define IMXRT_CAN_RXIMR17_OFFSET  0x08c4            /* R17 Individual Mask Registers */
#define IMXRT_CAN_RXIMR18_OFFSET  0x08c8            /* R18 Individual Mask Registers */
#define IMXRT_CAN_RXIMR19_OFFSET  0x08cc            /* R19 Individual Mask Registers */
#define IMXRT_CAN_RXIMR20_OFFSET  0x08d0            /* R20 Individual Mask Registers */
#define IMXRT_CAN_RXIMR21_OFFSET  0x08d4            /* R21 Individual Mask Registers */
#define IMXRT_CAN_RXIMR22_OFFSET  0x08d8            /* R22 Individual Mask Registers */
#define IMXRT_CAN_RXIMR23_OFFSET  0x08dc            /* R23 Individual Mask Registers */
#define IMXRT_CAN_RXIMR24_OFFSET  0x08e0            /* R24 Individual Mask Registers */
#define IMXRT_CAN_RXIMR25_OFFSET  0x08e4            /* R25 Individual Mask Registers */
#define IMXRT_CAN_RXIMR26_OFFSET  0x08e8            /* R26 Individual Mask Registers */
#define IMXRT_CAN_RXIMR27_OFFSET  0x08ec            /* R27 Individual Mask Registers */
#define IMXRT_CAN_RXIMR28_OFFSET  0x08f0            /* R28 Individual Mask Registers */
#define IMXRT_CAN_RXIMR29_OFFSET  0x08f4            /* R29 Individual Mask Registers */
#define IMXRT_CAN_RXIMR30_OFFSET  0x08f8            /* R30 Individual Mask Registers */
#define IMXRT_CAN_RXIMR31_OFFSET  0x08fc            /* R31 Individual Mask Registers */
#define IMXRT_CAN_RXIMR32_OFFSET  0x0900            /* R32 Individual Mask Registers */
#define IMXRT_CAN_RXIMR33_OFFSET  0x0904            /* R33 Individual Mask Registers */
#define IMXRT_CAN_RXIMR34_OFFSET  0x0908            /* R34 Individual Mask Registers */
#define IMXRT_CAN_RXIMR35_OFFSET  0x090c            /* R35 Individual Mask Registers */
#define IMXRT_CAN_RXIMR36_OFFSET  0x0910            /* R36 Individual Mask Registers */
#define IMXRT_CAN_RXIMR37_OFFSET  0x0914            /* R37 Individual Mask Registers */
#define IMXRT_CAN_RXIMR38_OFFSET  0x0918            /* R38 Individual Mask Registers */
#define IMXRT_CAN_RXIMR39_OFFSET  0x091c            /* R39 Individual Mask Registers */
#define IMXRT_CAN_RXIMR40_OFFSET  0x0920            /* R40 Individual Mask Registers */
#define IMXRT_CAN_RXIMR41_OFFSET  0x0924            /* R41 Individual Mask Registers */
#define IMXRT_CAN_RXIMR42_OFFSET  0x0928            /* R42 Individual Mask Registers */
#define IMXRT_CAN_RXIMR43_OFFSET  0x092c            /* R43 Individual Mask Registers */
#define IMXRT_CAN_RXIMR44_OFFSET  0x0930            /* R44 Individual Mask Registers */
#define IMXRT_CAN_RXIMR45_OFFSET  0x0934            /* R45 Individual Mask Registers */
#define IMXRT_CAN_RXIMR46_OFFSET  0x0938            /* R46 Individual Mask Registers */
#define IMXRT_CAN_RXIMR47_OFFSET  0x093c            /* R47 Individual Mask Registers */
#define IMXRT_CAN_RXIMR48_OFFSET  0x0940            /* R48 Individual Mask Registers */
#define IMXRT_CAN_RXIMR49_OFFSET  0x0944            /* R49 Individual Mask Registers */
#define IMXRT_CAN_RXIMR50_OFFSET  0x0948            /* R50 Individual Mask Registers */
#define IMXRT_CAN_RXIMR51_OFFSET  0x094c            /* R51 Individual Mask Registers */
#define IMXRT_CAN_RXIMR52_OFFSET  0x0950            /* R52 Individual Mask Registers */
#define IMXRT_CAN_RXIMR53_OFFSET  0x0954            /* R53 Individual Mask Registers */
#define IMXRT_CAN_RXIMR54_OFFSET  0x0958            /* R54 Individual Mask Registers */
#define IMXRT_CAN_RXIMR55_OFFSET  0x095c            /* R55 Individual Mask Registers */
#define IMXRT_CAN_RXIMR56_OFFSET  0x0960            /* R56 Individual Mask Registers */
#define IMXRT_CAN_RXIMR57_OFFSET  0x0964            /* R57 Individual Mask Registers */
#define IMXRT_CAN_RXIMR58_OFFSET  0x0968            /* R58 Individual Mask Registers */
#define IMXRT_CAN_RXIMR59_OFFSET  0x096c            /* R59 Individual Mask Registers */
#define IMXRT_CAN_RXIMR60_OFFSET  0x0970            /* R60 Individual Mask Registers */
#define IMXRT_CAN_RXIMR61_OFFSET  0x0974            /* R61 Individual Mask Registers */
#define IMXRT_CAN_RXIMR62_OFFSET  0x0978            /* R62 Individual Mask Registers */
#define IMXRT_CAN_RXIMR63_OFFSET  0x097c            /* R63 Individual Mask Registers */

#define IMXRT_CAN_FDCTRL_OFFSET   0x0c00            /* CAN FD Control Register */
#define IMXRT_CAN_FDCBT_OFFSET    0x0c04            /* CAN FD Bit Timing Register */
#define IMXRT_CAN_FDCRC_OFFSET    0x0c08            /* CAN FD CRC register */

/* Register Bit Definitions *************************************************/

/* Module Configuration Register */

#define CAN_MCR_MAXMB_SHIFT        (0)       /* Bits 0-6: Number of the Last Message Buffer */
#define CAN_MCR_MAXMB_MASK         (0x7f << CAN_MCR_MAXMB_SHIFT)
                                             /* Bit 7:  Reserved */
#define CAN_MCR_IDAM_SHIFT         (8)       /* Bits 8-9: ID Acceptance Mode */
#define CAN_MCR_IDAM_MASK          (3 << CAN_MCR_IDAM_SHIFT)
#  define CAN_MCR_IDAM_FMTA        (0 << CAN_MCR_IDAM_SHIFT) /* Format A: One full ID  */
#  define CAN_MCR_IDAM_FMTB        (1 << CAN_MCR_IDAM_SHIFT) /* Format B: Two full (or partial) IDs */
#  define CAN_MCR_IDAM_FMTC        (2 << CAN_MCR_IDAM_SHIFT) /* Format C: Four partial IDs */
#  define CAN_MCR_IDAM_FMTD        (3 << CAN_MCR_IDAM_SHIFT) /* Format D: All frames rejected */

                                             /* Bit 10: Reserved */
#define CAN_MCR_FDEN               (1 << 11) /* Bit 11: CAN FD Operation Enable */
                                             /* Bit 11: Reserved for FlexCAN1 and FlexCAN2 */
#define CAN_MCR_AEN                (1 << 12) /* Bit 12: Abort Enable */
#define CAN_MCR_LPRIOEN            (1 << 13) /* Bit 13: Local Priority Enable */
                                             /* Bit 14: Reserved */
#define CAN_MCR_DMA                (1 << 15) /* Bit 15: DMA Enable */
                                             /* Bit 15: Reserved for FlexCAN1 and FlexCAN2 */
#define CAN_MCR_IRMQ               (1 << 16) /* Bit 16: Individual Rx Masking and Queue Enable */
#define CAN_MCR_SRXDIS             (1 << 17) /* Bit 17: Self Reception Disable */
#define CAN_MCR_DOZE               (1 << 18) /* Bit 18: Doze Mode Enable */
                                             /* Bit 18: Reserved for FlexCAN1 and FlexCAN2 */
#define CAN_MCR_WAKSRC             (1 << 19) /* Bit 19: Wake Up Source */
#define CAN_MCR_LPMACK             (1 << 20) /* Bit 20: Low Power Mode Acknowledge */
#define CAN_MCR_WRNEN              (1 << 21) /* Bit 21: Warning Interrupt Enable */
#define CAN_MCR_SLFWAK             (1 << 22) /* Bit 22: Self Wake Up */
#define CAN_MCR_SUPV               (1 << 23) /* Bit 23: Supervisor Mode */
#define CAN_MCR_FRZACK             (1 << 24) /* Bit 24: Freeze Mode Acknowledge */
#define CAN_MCR_SOFTRST            (1 << 25) /* Bit 25: Soft Reset */
#define CAN_MCR_WAKMSK             (1 << 26) /* Bit 26: Wake Up Interrupt Mask */
#define CAN_MCR_NOTRDY             (1 << 27) /* Bit 27: FlexCAN Not Ready */
#define CAN_MCR_HALT               (1 << 28) /* Bit 28: Halt FlexCAN */
#define CAN_MCR_RFEN               (1 << 29) /* Bit 29: Rx FIFO Enable */
#define CAN_MCR_FRZ                (1 << 30) /* Bit 30: Freeze Enable */
#define CAN_MCR_MDIS               (1 << 31) /* Bit 31: Module Disable */

/* Control 1 Register */

#define CAN_CTRL1_PROPSEG_SHIFT    (0)       /* Bits 0-2: Propagation Segment */
#define CAN_CTRL1_PROPSEG_MASK     (7 << CAN_CTRL1_PROPSEG_SHIFT)
#define CAN_CTRL1_PROPSEG(x)       (((uint32_t)(((uint32_t)(x)) << CAN_CTRL1_PROPSEG_SHIFT)) & CAN_CTRL1_PROPSEG_MASK)
#define CAN_CTRL1_LOM              (1 << 3)  /* Bit 3:  Listen-Only Mode */
#define CAN_CTRL1_LBUF             (1 << 4)  /* Bit 4:  Lowest Buffer Transmitted First */
#define CAN_CTRL1_TSYN             (1 << 5)  /* Bit 5:  Timer Sync */
#define CAN_CTRL1_BOFFREC          (1 << 6)  /* Bit 6:  Bus Off Recovery */
#define CAN_CTRL1_SMP              (1 << 7)  /* Bit 7:  CAN Bit Sampling */
                                             /* Bits 8-9: Reserved */
#define CAN_CTRL1_RWRNMSK          (1 << 10) /* Bit 10: Rx Warning Interrupt Mask */
#define CAN_CTRL1_TWRNMSK          (1 << 11) /* Bit 11: Tx Warning Interrupt Mask */
#define CAN_CTRL1_LPB              (1 << 12) /* Bit 12: Loop Back Mode */
#define CAN_CTRL1_CLKSRC           (1 << 13) /* Bit 13: CAN Engine Clock Source */
                                             /* Bit 13: Reserved for FlexCAN1 and FlexCAN2 */
#define CAN_CTRL1_ERRMSK           (1 << 14) /* Bit 14: Error Mask */
#define CAN_CTRL1_BOFFMSK          (1 << 15) /* Bit 15: Bus Off Mask */
#define CAN_CTRL1_PSEG2_SHIFT      (16)      /* Bits 16-18: Phase Segment 2 */
#define CAN_CTRL1_PSEG2_MASK       (7 << CAN_CTRL1_PSEG2_SHIFT)
#define CAN_CTRL1_PSEG2(x)         (((uint32_t)(((uint32_t)(x)) << CAN_CTRL1_PSEG2_SHIFT)) & CAN_CTRL1_PSEG2_MASK)
#define CAN_CTRL1_PSEG1_SHIFT      (19)      /* Bits 19-21: Phase Segment 1 */
#define CAN_CTRL1_PSEG1_MASK       (7 << CAN_CTRL1_PSEG1_SHIFT)
#define CAN_CTRL1_PSEG1(x)         (((uint32_t)(((uint32_t)(x)) << CAN_CTRL1_PSEG1_SHIFT)) & CAN_CTRL1_PSEG1_MASK)
#define CAN_CTRL1_RJW_SHIFT        (22)      /* Bits 22-23: Resync Jump Width */
#define CAN_CTRL1_RJW_MASK         (3 << CAN_CTRL1_RJW_SHIFT)
#define CAN_CTRL1_RJW(x)           (((uint32_t)(((uint32_t)(x)) << CAN_CTRL1_RJW_SHIFT)) & CAN_CTRL1_RJW_MASK)
#define CAN_CTRL1_PRESDIV_SHIFT    (24)      /* Bits 24-31: Prescaler Division Factor */
#define CAN_CTRL1_PRESDIV_MASK     (0xff << CAN_CTRL1_PRESDIV_SHIFT)
#define CAN_CTRL1_PRESDIV(x)       (((uint32_t)(((uint32_t)(x)) << CAN_CTRL1_PRESDIV_SHIFT)) & CAN_CTRL1_PRESDIV_MASK)

/* Free Running Timer */

#define CAN_TIMER_SHIFT            (0)       /* Bits 0-15: Timer value */
#define CAN_TIMER_MASK             (0xffff << CAN_TIMER_SHIFT)
                                             /* Bits 16-31: Reserved */

/* Rx Mailboxes Global Mask Register (32 Rx Mailboxes Global Mask Bits) */

#define CAN_RXMGMASK(n)            (1 << (n)) /* Bit n: Rx Mailboxe n Global Mask Bit */

/* Rx 14 Mask Register */

#define CAN_RX14MASK(n)            (1 << (n)) /* Bit n: Rx Buffer 14 Mask Bit n */

/* Rx 15 Mask Register */

#define CAN_RX15MASK(n)            (1 << (n)) /* Bit n: Rx Buffer 15 Mask Bit n */

/* Error Counter */

#define CAN_ECR_TXERRCNT_SHIFT     (0)       /* Bits 0-7: Transmit Error Counter */
#define CAN_ECR_TXERRCNT_MASK      (0xff << CAN_ECR_TXERRCNT_SHIFT)
#define CAN_ECR_RXERRCNT_SHIFT     (8)       /* Bits 8-15: Receive Error Counter */
#define CAN_ECR_RXERRCNT_MASK      (0xff << CAN_ECR_RXERRCNT_SHIFT)
#define CAN_ECR_TXERRCNTFAST_SHIFT (16)      /* Bits 16-23: Transmit Error Counter for fast bits */
#define CAN_ECR_TXERRCNTFAST_MASK  (0xff << CAN_ECR_TXERRCNTFAST_SHIFT)
#define CAN_ECR_RXERRCNTFAST_SHIFT (24)      /* Bits 24-31: Receive Error Counter for fast bits */
#define CAN_ECR_RXERRCNTFAST_MASK  (0xff << CAN_ECR_RXERRCNTFAST_SHIFT)
                                             /* Bits 16-31: Reserved for FlexCAN1 and FlexCAN2 */

/* Error and Status 1 Register */

#define CAN_ESR1_WAKINT            (1 << 0)  /* Bit 0:  Wake-Up Interrupt */
#define CAN_ESR1_ERRINT            (1 << 1)  /* Bit 1:  Error Interrupt */
#define CAN_ESR1_BOFFINT           (1 << 2)  /* Bit 2:  'Bus Off' Interrupt */
#define CAN_ESR1_RX                (1 << 3)  /* Bit 3:  FlexCAN in Reception */
#define CAN_ESR1_FLTCONF_SHIFT     (4)       /* Bits 4-5: Fault Confinement State */
#define CAN_ESR1_FLTCONF_MASK      (3 << CAN_ESR1_FLTCONF_SHIFT)
#  define CAN_ESR1_FLTCONF_ACTV    (0 << CAN_ESR1_FLTCONF_SHIFT)
                                             /* Error Active */
#  define CAN_ESR1_FLTCONF_PASV    (1 << CAN_ESR1_FLTCONF_SHIFT)
                                             /* Error Passive */
#  define CAN_ESR1_FLTCONF_OFF     (2 << CAN_ESR1_FLTCONF_SHIFT)
                                             /* Bus Off */
#define CAN_ESR1_TX                (1 << 6)  /* Bit 6:  FlexCAN in Transmission */
#define CAN_ESR1_IDLE              (1 << 7)  /* Bit 7:  CAN bus is in IDLE state */
#define CAN_ESR1_RXWRN             (1 << 8)  /* Bit 8:  Rx Error Warning */
#define CAN_ESR1_TXWRN             (1 << 9)  /* Bit 9:  TX Error Warning */
#define CAN_ESR1_STFERR            (1 << 10) /* Bit 10: Stuffing Error */
#define CAN_ESR1_FRMERR            (1 << 11) /* Bit 11: Form Error */
#define CAN_ESR1_CRCERR            (1 << 12) /* Bit 12: Cyclic Redundancy Check Error */
#define CAN_ESR1_ACKERR            (1 << 13) /* Bit 13: Acknowledge Error */
#define CAN_ESR1_BIT0ERR           (1 << 14) /* Bit 14: Bit0 Error */
#define CAN_ESR1_BIT1ERR           (1 << 15) /* Bit 15: Bit1 Error */
#define CAN_ESR1_RWRNINT           (1 << 16) /* Bit 16: Rx Warning Interrupt Flag */
#define CAN_ESR1_TWRNINT           (1 << 17) /* Bit 17: Tx Warning Interrupt Flag */
#define CAN_ESR1_SYNCH             (1 << 18) /* Bit 18: CAN Synchronization Status */
#define CAN_ESR1_BOFFDONEINT       (1 << 19) /* Bit 19: Bus Off Done Interrupt */
#define CAN_ESR1_ERRINTFAST        (1 << 20) /* Bit 20: Error Iterrupt for Errors Detected in Data Phase of CAN FD frames */
#define CAN_ESR1_ERROVR            (1 << 21) /* Bit 21: Error Overrun */
                                             /* Bits 21-25: Reserved */
#define CAN_ESR1_STFERRFAST        (1 << 26) /* Bit 26: Stuffing Error in the Data Phase of CAN FD frames */
#define CAN_ESR1_FRMERRFAST        (1 << 27) /* Bit 27: Form Error in the Data Phase of CAN FD frames */
#define CAN_ESR1_CRCERRFAST        (1 << 28) /* Bit 28: Cyclic Redundancy Check Error in the CRC field of CAN FD frames */
                                             /* Bit 29: Reserved */
#define CAN_ESR1_BIT0ERRFAST       (1 << 30) /* Bit 30: Bit0 Error in the Data Phase of CAN FD frames */
#define CAN_ESR1_BIT1ERRFAST       (1 << 31) /* Bit 31: Bit1 Error in the Data Phase of CAN FD frames */
                                             /* Bits 19-31: Reserved for FlexCAN1 and FlexCAN2 */

/* Interrupt Masks 2 Register */

#define CAN_IMASK2(n)              (1 << (n)) /* Bit n: Buffer MBn Mask */

/* Interrupt Masks 1 Register */

#define CAN_IMASK1(n)              (1 << (n)) /* Bit n: Buffer MBn Mask */

/* Interrupt Flags 2 Register */

#define CAN_IFLAG2(n)              (1 << (n)) /* Bit n: Buffer MBn Interrupt */

/* Interrupt Flags 1 Register */

#define CAN_IFLAG1(n)              (1 << (n)) /* Bit n: Buffer MBn Interrupt, n=0..4,8..31 */

/* Control 2 Register */

                                             /* Bits 0-10: Reserved */
#define CAN_CTRL2_EDFLTDIS         (1 << 11) /* Bit 11: Edge Filter Disable */
#define CAN_CTRL2_ISOCANFDEN       (1 << 12) /* Bit 12: ISO CAN FD Enable */
                                             /* Bit 13: Reserved */
#define CAN_CTRL2_PREXCEN          (1 << 14) /* Bit 14: Protocol Exception Enable */
#define CAN_CTRL2_TIMERSRC         (1 << 15) /* Bit 15: Timer Source */
                                             /* Bits 0-15: Reserved for FlexCAN1 and FlexCAN2 */
#define CAN_CTRL2_EACEN            (1 << 16) /* Bit 16: Entire Frame Arbitration Field Comparison Enable (Rx) */
#define CAN_CTRL2_RRS              (1 << 17) /* Bit 17: Remote Request Storing */
#define CAN_CTRL2_MRP              (1 << 18) /* Bit 18: Mailboxes Reception Priority */
#define CAN_CTRL2_TASD_SHIFT       (19)      /* Bits 19-23: Tx Arbitration Start Delay */
#define CAN_CTRL2_TASD_MASK        (31 << CAN_CTRL2_TASD_SHIFT)
#define CAN_CTRL2_RFFN_SHIFT       (24)      /* Bits 24-27: Number of Rx FIFO Filters */
#define CAN_CTRL2_RFFN_MASK        (15 << CAN_CTRL2_RFFN_SHIFT)
#  define CAN_CTRL2_RFFN_8MB       (0 << CAN_CTRL2_RFFN_SHIFT)
#  define CAN_CTRL2_RFFN_16MB      (1 << CAN_CTRL2_RFFN_SHIFT)
#  define CAN_CTRL2_RFFN_24MB      (2 << CAN_CTRL2_RFFN_SHIFT)
#  define CAN_CTRL2_RFFN_32MB      (3 << CAN_CTRL2_RFFN_SHIFT)
#  define CAN_CTRL2_RFFN_40MB      (4 << CAN_CTRL2_RFFN_SHIFT)
#  define CAN_CTRL2_RFFN_48MB      (5 << CAN_CTRL2_RFFN_SHIFT)
#  define CAN_CTRL2_RFFN_56MB      (6 << CAN_CTRL2_RFFN_SHIFT)
#  define CAN_CTRL2_RFFN_64MB      (7 << CAN_CTRL2_RFFN_SHIFT)
#  define CAN_CTRL2_RFFN_72MB      (8 << CAN_CTRL2_RFFN_SHIFT)
#  define CAN_CTRL2_RFFN_80MB      (9 << CAN_CTRL2_RFFN_SHIFT)
#  define CAN_CTRL2_RFFN_88MB      (10 << CAN_CTRL2_RFFN_SHIFT)
#  define CAN_CTRL2_RFFN_96MB      (11 << CAN_CTRL2_RFFN_SHIFT)
#  define CAN_CTRL2_RFFN_104MB     (12 << CAN_CTRL2_RFFN_SHIFT)
#  define CAN_CTRL2_RFFN_112MB     (13 << CAN_CTRL2_RFFN_SHIFT)
#  define CAN_CTRL2_RFFN_120MB     (14 << CAN_CTRL2_RFFN_SHIFT)
#  define CAN_CTRL2_RFFN_128MB     (15 << CAN_CTRL2_RFFN_SHIFT)
                                             /* Bits 28-29: Reserved */
#define CAN_CTRL2_BOFFDONEMSK      (1 << 30) /* Bit 30: Bus Off Done Interrupt Mask */
#define CAN_CTRL2_ERRMSKFAST       (1 << 31) /* Bit 31: Error Interrupt for Errors Detected in the Data Phase of CAN FD frames */
#define CAN_CTRL2_WRMFRZ           (1 << 28) /* Bit 28: Enable unrestricted write access to FlexCAN memory in Freeze mode */
                                             /* Bits 29-31: Reserved */

/* Error and Status 2 Register */

                                             /* Bits 0-12: Reserved */
#define CAN_ESR2_IMB               (1 << 13) /* Bit 13: Inactive Mailbox */
#define CAN_ESR2_VPS               (1 << 14) /* Bit 14: Valid Priority Status */
                                             /* Bit 15: Reserved */
#define CAN_ESR2_LPTM_SHIFT        (16)      /* Bits 16-22: Lowest Priority Tx Mailbox */
#define CAN_ESR2_LPTM_MASK         (0x7f << CAN_ESR2_LPTM_SHIFT)
                                             /* Bits 23-31: Reserved */

/* CRC Register */

#define CAN_CRCR_TXCRC_SHIFT       (0)       /* Bits 0-14: CRC Transmitted */
#define CAN_CRCR_TXCRC_MASK        (0x7fff << CAN_CRCR_TXCRC_SHIFT)
                                             /* Bit  15: Reserved */
#define CAN_CRCR_MBCRC_SHIFT       (16)      /* Bits 16-22: CRC Mailbox */
#define CAN_CRCR_MBCRC_MASK        (0x7f << CAN_CRCR_MBCRC_SHIFT)
                                             /* Bits 23-31: Reserved */

/* Rx FIFO Global Mask Register (32 Rx FIFO Global Mask Bits) */

#define CAN_RXFGMASK(n)            (1 << (n)) /* Bit n: Rx FIFO n Global Mask Bit */

/* Rx FIFO Information Register */

                                             /* Bits 9-31: Reserved */
#define CAN_RXFIR_IDHIT_SHIFT      (0)       /* Bits 0-8: Identifier Acceptance Filter Hit Indicator */
#define CAN_RXFIR_IDHIT_MASK       (0x1ff << CAN_RXFIR_IDHIT_SHIFT)

/* Rx Individual Mask Registers */

#define CAN_RXIMR(n)               (1 << (n)) /* Bit n: Individual Mask Bits */

/* CAN Bit Timing register */

#define CAN_CBT_EPSEG2_SHIFT       (0)       /* Bits 0-4: Extended Phase Segment 2 */
#define CAN_CBT_EPSEG2_MASK        (0x1f << CAN_CBT_EPSEG2_SHIFT)
#define CAN_CBT_EPSEG2(x)          (((uint32_t)(((uint32_t)(x)) << CAN_CBT_EPSEG2_SHIFT)) & CAN_CBT_EPSEG2_MASK)
#define CAN_CBT_EPSEG1_SHIFT       (5)       /* Bits 5-9: Extended Phase Segment 1 */
#define CAN_CBT_EPSEG1_MASK        (0x1f << CAN_CBT_EPSEG1_SHIFT)
#define CAN_CBT_EPSEG1(x)          (((uint32_t)(((uint32_t)(x)) << CAN_CBT_EPSEG1_SHIFT)) & CAN_CBT_EPSEG1_MASK)
#define CAN_CBT_EPROPSEG_SHIFT     (10)      /* Bits 10-15: Extended Propagation Segment */
#define CAN_CBT_EPROPSEG_MASK      (0x1f << CAN_CBT_EPROPSEG_SHIFT)
#define CAN_CBT_EPROPSEG(x)        (((uint32_t)(((uint32_t)(x)) << CAN_CBT_EPROPSEG_SHIFT)) & CAN_CBT_EPROPSEG_MASK)
#define CAN_CBT_ERJW_SHIFT         (16)      /* Bits 16-20: Extended Resync Jump Width */
#define CAN_CBT_ERJW_MASK          (0x1f << CAN_CBT_ERJW_SHIFT)
#define CAN_CBT_ERJW(x)            (((uint32_t)(((uint32_t)(x)) << CAN_CBT_ERJW_SHIFT)) & CAN_CBT_ERJW_MASK)
#define CAN_CBT_EPRESDIV_SHIFT     (21)      /* Bits 21-30: Extendet Prescaler Division Factor */
#define CAN_CBT_EPRESDIV_MASK      (0x3ff << CAN_CBT_EPRESDIV_SHIFT)
#define CAN_CBT_EPRESDIV(x)        (((uint32_t)(((uint32_t)(x)) << CAN_CBT_EPRESDIV_SHIFT)) & CAN_CBT_EPRESDIV_MASK)
#define CAN_CBT_BTF                (1 << 31) /* Bit 31: Bit Timing Format Enable */

/* CAN FD Control register */

#define CAN_FDCTRL_TDCVAL_SHIFT    (0)       /* Bits 0-5: Transceiver Delay Compensation Value */
#define CAN_FDCTRL_TDCVAL_MASK     (0x3f << CAN_FDCTRL_TDCVAL_SHIFT)
                                             /* Bits 6-7: Reserved */
#define CAN_FDCTRL_TDCOFF_SHIFT    (8)       /* Bits 8-12: Transceiver Delay Compensation Offset */
#define CAN_FDCTRL_TDCOFF_MASK     (0x3f << CAN_FDCTRL_TDCOFF_SHIFT)
#define CAN_FDCTRL_TDCOFF(x)       (((uint32_t)(((uint32_t)(x)) << CAN_FDCTRL_TDCOFF_SHIFT)) & CAN_FDCTRL_TDCOFF_MASK)
                                             /* Bit 13: Reserved */
#define CAN_FDCTRL_TDCFAIL         (1 << 14) /* Bit 14: Transceiver Delay Compensation Fail */
#define CAN_FDCTRL_TDCEN           (1 << 15) /* Bit 15: Transceiver Delay Compensation Enable */
#define CAN_FDCTRL_MBDSR0_SHIFT    (16)      /* Bits 16-17: Message Buffer Data Size for Region 0 */
#define CAN_FDCTRL_MBDSR0_MASK     (0x3 << CAN_FDCTRL_MBDSR0_SHIFT)
#define CAN_FDCTRL_MSBSR0(x)       (((uint32_t)(((uint32_t)(x)) << CAN_FDCTRL_MBDSR0_SHIFT)) & CAN_FDCTRL_MBDSR0_MASK)
                                             /* Bit 18: Reserved */
#define CAN_FDCTRL_MBDSR1_SHIFT    (19)      /* Bits 19-20: Message Buffer Data Size for Region 2 */
#define CAN_FDCTRL_MBDSR1_MASK     (0x3 << CAN_FDCTRL_MBDSR1_SHIFT)
#define CAN_FDCTRL_MSBSR1(x)       (((uint32_t)(((uint32_t)(x)) << CAN_FDCTRL_MBDSR1_SHIFT)) & CAN_FDCTRL_MBDSR1_MASK)
                                             /* Bits 21-30: Reserved */
#define CAN_FDCTRL_FDRATE          (1 << 31) /* Bit 31: Bit Rate Switch Enable */

/* CAN FD Bit Timing register */

#define CAN_FDCBT_FPSEG2_SHIFT     (0)      /* Bits 0-2: Fast Phase Segment 2 */
#define CAN_FDCBT_FPSEG2_MASK      (0x7 << CAN_FDCBT_FPSEG2_SHIFT)
#define CAN_FDCBT_FPSEG2(x)        (((uint32_t)(((uint32_t)(x)) << CAN_FDCBT_FPSEG2_SHIFT)) & CAN_FDCBT_FPSEG2_MASK)
                                            /* Bits 3-4: Reserved */
#define CAN_FDCBT_FPSEG1_SHIFT     (5)      /* Bits 5-7: Fast Phase Segment 1 */
#define CAN_FDCBT_FPSEG1_MASK      (0x7 << CAN_FDCBT_FPSEG1_SHIFT)
#define CAN_FDCBT_FPSEG1(x)        (((uint32_t)(((uint32_t)(x)) << CAN_FDCBT_FPSEG1_SHIFT)) & CAN_FDCBT_FPSEG1_MASK)
                                            /* Bits 8-9: Reserved */
#define CAN_FDCBT_FPROPSEG_SHIFT   (10)     /* Bits 10-14: Fast Propagation Segment */
#define CAN_FDCBT_FPROPSEG_MASK    (0x1f << CAN_FDCBT_FPROPSEG_SHIFT)
#define CAN_FDCBT_FPROPSEG(x)      (((uint32_t)(((uint32_t)(x)) << CAN_FDCBT_FPROPSEG_SHIFT)) & CAN_FDCBT_FPROPSEG_MASK)
                                            /* Bit 15: Reserved */
#define CAN_FDCBT_FRJW_SHIFT       (16)     /* Bits 16-18: Fast Propagation Segment */
#define CAN_FDCBT_FRJW_MASK        (0x7 << CAN_FDCBT_FRJW_SHIFT)
#define CAN_FDCBT_FRJW(x)          (((uint32_t)(((uint32_t)(x)) << CAN_FDCBT_FRJW_SHIFT)) & CAN_FDCBT_FRJW_MASK)
                                            /* Bit 19: Reserved */
#define CAN_FDCBT_FPRESDIV_SHIFT   (20)     /* Bits 20-29: Fast Propagation Segment */
#define CAN_FDCBT_FPRESDIV_MASK    (0x3ff << CAN_FDCBT_FPRESDIV_SHIFT)
#define CAN_FDCBT_FPRESDIV(x)      (((uint32_t)(((uint32_t)(x)) << CAN_FDCBT_FPRESDIV_SHIFT)) & CAN_FDCBT_FPRESDIV_MASK)
                                            /* Bits 30-31: Reserved */

/* CAN FD CRC register */

#define CAN_FDCRC_FD_TXCRC_SHIFT   (0)      /* Bits 0-20: Extended Transmitted CRC value */
#define CAN_FDCRC_FD_TXCRC_MASK    (0x1fffff << CAN_FDCRC_FD_TXCRC_SHIFT)
#define CAN_FDCRC_FD_TXCRC(x)      (((uint32_t)(((uint32_t)(x)) << CAN_FDCRC_FD_TXCRC_SHIFT)) & CAN_FDCRC_FD_TXCRC_MASK)
                                            /* Bits 21-23: Reserved */
#define CAN_FDCRC_FD_MBCRC_SHIFT   (24)     /* Bits 24-30: Extended Transmitted CRC value */
#define CAN_FDCRC_FD_MBCRC_MASK    (0x7f << CAN_FDCRC_FD_MBCRC_SHIFT)
#define CAN_FDCRC_FD_MBCRC(x)      (((uint32_t)(((uint32_t)(x)) << CAN_FDCRC_FD_MBCRC_SHIFT)) & CAN_FDCRC_FD_MBCRC_MASK)
                                            /* Bit 31: Reserved */

/* CAN MB TX codes */
#define CAN_TXMB_INACTIVE          0x8        /* MB is not active. */
#define CAN_TXMB_ABORT             0x9        /* MB is aborted. */
#define CAN_TXMB_DATAORREMOTE      0xC        /* MB is a TX Data Frame(when MB RTR = 0) or */
                                              /* MB is a TX Remote Request Frame (when MB RTR = 1). */
#define CAN_TXMB_TANSWER           0xE        /* MB is a TX Response Request Frame from */
                                              /* an incoming Remote Request Frame. */
#define CAN_TXMB_NOTUSED           0xF        /* Not used.*/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_IMXRT_HARDWARE_IMXRT_FLEXCAN_H */
