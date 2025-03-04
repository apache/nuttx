/****************************************************************************
 * arch/arm64/src/imx9/hardware/imx9_flexcan.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifndef __ARCH_ARM64_SRC_IMX9_HARDWARE_IMX9_FLEXCAN_H
#define __ARCH_ARM64_SRC_IMX9_HARDWARE_IMX9_FLEXCAN_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define IMX9_CAN_MCR_OFFSET      0x0000 /* Module Configuration Register */
#define IMX9_CAN_CTRL1_OFFSET    0x0004 /* Control 1 Register */
#define IMX9_CAN_TIMER_OFFSET    0x0008 /* Free Running Timer */
#define IMX9_CAN_RXMGMASK_OFFSET 0x0010 /* Rx Mailboxes Global Mask Register */
#define IMX9_CAN_RX14MASK_OFFSET 0x0014 /* Rx 14 Mask Register */
#define IMX9_CAN_RX15MASK_OFFSET 0x0018 /* Rx 15 Mask Register */
#define IMX9_CAN_ECR_OFFSET      0x001c /* Error Counter */
#define IMX9_CAN_ESR1_OFFSET     0x0020 /* Error and Status 1 Register */
#define IMX9_CAN_IMASK2_OFFSET   0x0024 /* Interrupt Masks 2 Register */
#define IMX9_CAN_IMASK1_OFFSET   0x0028 /* Interrupt Masks 1 Register */
#define IMX9_CAN_IFLAG2_OFFSET   0x002c /* Interrupt Flags 2 Register */
#define IMX9_CAN_IFLAG1_OFFSET   0x0030 /* Interrupt Flags 1 Register */
#define IMX9_CAN_CTRL2_OFFSET    0x0034 /* Control 2 Register */
#define IMX9_CAN_ESR2_OFFSET     0x0038 /* Error and Status 2 Register */
#define IMX9_CAN_CRCR_OFFSET     0x0044 /* CRC Register */
#define IMX9_CAN_RXFGMASK_OFFSET 0x0048 /* Rx FIFO Global Mask Register */
#define IMX9_CAN_RXFIR_OFFSET    0x004c /* Rx FIFO Information Register */
#define IMX9_CAN_CBT_OFFSET      0x0050 /* CAN Bit Timing Register */
#define IMX9_CAN_IMASK3_OFFSET   0x006C /* Interrupt Masks 3 Register */
#define IMX9_CAN_IFLAG3_OFFSET   0x0074 /* Interrupt Flags 3 Register */
#define IMX9_CAN_ET_OFFSET       0x0078 /* External Timer Register */
#define IMX9_CAN_FLTC_IE_OFFSET  0x007c /* Fault Confinement Interrupt Enable Register */

#define IMX9_CAN_MB_OFFSET       0x0080 /* CAN MB start */
#define IMX9_CAN_MB_SIZE         0x0600
#define IMX9_CAN_MB_END          (IMX9_CAN_MB_OFFSET + IMX9_CAN_MB_SIZE)

#define IMX9_CAN_RXIMR_OFFSET(n) (0x0880+((n)<<2)) /* Rn Individual Mask Registers */
#define IMX9_CAN_N_RXIMR         96                /* RXIMR 0-95 */

#define IMX9_CAN_RXFMB_OFFSET(n) (0x0A80+((n)<<2)) /* Legacy FIFO Information memory area */
#define IMX9_CAN_N_RXFMB         6                 /* Number of RXFIR registers */

#define IMX9_CAN_TXSMB_OFFSET    0x0ab0 /* Tx_SMB */
#define IMX9_CAN_RXSMB0_OFFSET   0x0ac0 /* Rx_SMB0 */
#define IMX9_CAN_TXSMB1_OFFSET   0x0ad0 /* Rx_SMB1 */
#define IMX9_CAN_TXSMBFD_OFFSET  0x0f28 /* Tx_SMB for can-fd */
#define IMX9_CAN_RXSMB0FD_OFFSET 0x0f70 /* Rx_SMB0 for can-fd */
#define IMX9_CAN_TXSMB1FD_OFFSET 0x0fb8 /* Rx_SMB1 for can-fd */

#define IMX9_CAN_MECR_OFFSET     0x0ae0 /* Memory Error Control Register */
#define IMX9_CAN_ERRIAR_OFFSET   0x0ae4 /* Error Injection Address Register */
#define IMX9_CAN_ERRIDPR_OFFSET  0x0ae8 /* Error Injection Data Pattern Register */
#define IMX9_CAN_ERRIPPR_OFFSET  0x0aec /* Error Injection Parity Pattern Register */
#define IMX9_CAN_RERRAR_OFFSET   0x0af0 /* Error Report Address Register */
#define IMX9_CAN_RERRDR_OFFSET   0x0af4 /* Error Report Data Register */
#define IMX9_CAN_RERRSYNR_OFFSET 0x0af8 /* Error Report Syndrome Register */
#define IMX9_CAN_ERRSR_OFFSET    0x0afc /* Error Status Register */
#define IMX9_CAN_EPRS_OFFSET     0x0bf0 /* Enhanced CAN Bit Timing Prescalers Register */
#define IMX9_CAN_ENCBT_OFFSET    0x0bf4 /* Enhanced Nominal CAN Bit Timing Register */
#define IMX9_CAN_EDCBT_OFFSET    0x0bf8 /* Enhanced Data Phase CAN Bit Timing Register */
#define IMX9_CAN_ETDC_OFFSET     0x0bfc /* Enhanced Transceiver Delay Compensation Register */

#define IMX9_CAN_FDCTRL_OFFSET   0x0c00 /* CAN FD Control Register */
#define IMX9_CAN_FDCBT_OFFSET    0x0c04 /* CAN FD Bit Timing Register */
#define IMX9_CAN_FDCRC_OFFSET    0x0c08 /* CAN FD CRC register */

/* Register Bit Definitions *************************************************/

/* Module Configuration Register */

#define CAN_MCR_MAXMB_SHIFT        (0)       /* Bits 0-6: Number of the Last Message Buffer */
#define CAN_MCR_MAXMB_MASK         (0x7f << CAN_MCR_MAXMB_SHIFT)
#define CAN_MCR_TPOE               (1 << 7)  /* Bit 7: TX Pin Override Enable */

#define CAN_MCR_IDAM_SHIFT         (8)       /* Bits 8-9: ID Acceptance Mode */
#define CAN_MCR_IDAM_MASK          (3 << CAN_MCR_IDAM_SHIFT)
#  define CAN_MCR_IDAM_FMTA        (0 << CAN_MCR_IDAM_SHIFT) /* Format A: One full ID  */
#  define CAN_MCR_IDAM_FMTB        (1 << CAN_MCR_IDAM_SHIFT) /* Format B: Two full (or partial) IDs */
#  define CAN_MCR_IDAM_FMTC        (2 << CAN_MCR_IDAM_SHIFT) /* Format C: Four partial IDs */
#  define CAN_MCR_IDAM_FMTD        (3 << CAN_MCR_IDAM_SHIFT) /* Format D: All frames rejected */

#define CAN_MCR_TPOV               (1 << 10) /* Bit 10: TX Pin Override Value */
#define CAN_MCR_FDEN               (1 << 11) /* Bit 11: CAN FD Operation Enable */
#define CAN_MCR_AEN                (1 << 12) /* Bit 12: Abort Enable */
#define CAN_MCR_LPRIOEN            (1 << 13) /* Bit 13: Local Priority Enable */
                                             /* Bit 14: Reserved */
#define CAN_MCR_DMA                (1 << 15) /* Bit 15: DMA Enable */
#define CAN_MCR_IRMQ               (1 << 16) /* Bit 16: Individual Rx Masking and Queue Enable */
#define CAN_MCR_SRXDIS             (1 << 17) /* Bit 17: Self Reception Disable */
#define CAN_MCR_DOZE               (1 << 18) /* Bit 18: Doze Mode Enable */
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
#define CAN_MCR_RFEN               (1 << 29) /* Bit 29: Legacy Rx FIFO Enable */
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
#define CAN_CTRL1_ROM              (1 << 7)  /* Bit 8:  Restricted Operation Mode */
                                             /* Bit 9:  Reserved */
#define CAN_CTRL1_RWRNMSK          (1 << 10) /* Bit 10: Rx Warning Interrupt Mask */
#define CAN_CTRL1_TWRNMSK          (1 << 11) /* Bit 11: Tx Warning Interrupt Mask */
#define CAN_CTRL1_LPB              (1 << 12) /* Bit 12: Loop Back Mode */
#define CAN_CTRL1_CLKSRC           (1 << 13) /* Bit 13: CAN Engine Clock Source */
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
#define CAN_ESR1_ATP               (1 << 22) /* Bit 22: Active to Passive State */
#define CAN_ESR1_PTA               (1 << 23) /* Bit 23: Passive to Active State */
                                             /* Bits 24-25: Reserved */
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

                                             /* Bits 0-1: Reserved */
#define CAN_CTRL2_RETRY_SHIFT      (2)       /* Bits 2-4: Number of Retransmission Requests */
#define CAN_CTRL2_RETRY_MASK       (7 << CAN_CTRL2_RETRY_SHIFT)
#define CAN_CTRL2_RETRY(x)         (((x) << CAN_CTRL2_RETRY_SHIFT) & CAN_CTRL2_RETRY_MASK)
                                             /* Bit 5: Reserved */
#define CAN_CTRL2_TSTAMPCAP_SHIFT  (6)       /* Bits 6-7: Timestamp Capture Point */
#define CAN_CTRL2_TSTAMPCAP_MASK   (3 << CAN_CTRL2_TSTAMPCAP_SHIFT)
#  define CAN_CTRL2_TSTAMPCAP_DIS  (0 << CAN_CTRL2_TSTAMPCAP_SHIFT) /* Disabled */
#  define CAN_CTRL2_TSTAMPCAP_EOF  (1 << CAN_CTRL2_TSTAMPCAP_SHIFT) /* End of CAN frame */
#  define CAN_CTRL2_TSTAMPCAP_SOF  (2 << CAN_CTRL2_TSTAMPCAP_SHIFT) /* Start of CAN frame */
#  define CAN_CTRL2_TSTAMPCAP_CSOF (3 << CAN_CTRL2_TSTAMPCAP_SHIFT) /* Start of classical CAN frame or res for CAN FD */

#define CAN_CTRL2_MBTSBASE_SHIFT   (8)       /* Bits 8-9: Message Buffer Timestamp Base */
#define CAN_CTRL2_MBTSBASE_MASK    (3 << CAN_CTRL2_MBTSBASE_SHIFT)
#  define CAN_CTRL2_MBTSBASE_TIMER (0 << CAN_CTRL2_MBTSBASE_SHIFT) /* TIMER */
#  define CAN_CTRL2_MBTSBASE_LOWER (1 << CAN_CTRL2_MBTSBASE_SHIFT) /* Lower 16 bits of high-resolution timer */
#  define CAN_CTRL2_MBTSBASE_UPPER (2 << CAN_CTRL2_MBTSBASE_SHIFT) /* Upper 16 bits of high-resolution timer */

#define CAN_CTRL2_FLTRXN           (1 << 10) /* Bit 10: Fault reaction */
#define CAN_CTRL2_EDFLTDIS         (1 << 11) /* Bit 11: Edge Filter Disable */
#define CAN_CTRL2_ISOCANFDEN       (1 << 12) /* Bit 12: ISO CAN FD Enable */
#define CAN_CTRL2_BTE              (1 << 13) /* Bit 13: Bit Timing Expansion Enable */
#define CAN_CTRL2_PREXCEN          (1 << 14) /* Bit 14: Protocol Exception Enable */
#define CAN_CTRL2_TIMERSRC         (1 << 15) /* Bit 15: Timer Source */
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

#define CAN_CTRL2_WRMFRZ           (1 << 28) /* Bit 28: Write Access to Memory in Freeze Mode */
#define CAN_CTRL2_ECRWRE           (1 << 29) /* Bit 28: Error Correction Configuration Register Write Enable */
#define CAN_CTRL2_BOFFDONEMSK      (1 << 30) /* Bit 30: Bus Off Done Interrupt Mask */
#define CAN_CTRL2_ERRMSKFAST       (1 << 31) /* Bit 31: Error Interrupt for Errors Detected in the Data Phase of CAN FD frames */
#define CAN_CTRL2_WRMFRZ           (1 << 28) /* Bit 28: Enable unrestricted write access to FlexCAN memory in Freeze mode */
                                             /* Bits 29-31: Reserved */

/* Error and Status 2 Register */

                                             /* Bits 0-11: Reserved */
#define CAN_ESR2_RX_PIN_ST         (1 << 12) /* Bit 12: RX Pin Status */
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

/* Interrupt Masks 3 Register */

#define CAN_IMASK3(n)              (1 << (n)) /* Bit n: Buffer MBn Mask (95TO64) */

/* Interrupt Flags 3 Register */

#define CAN_IFLAG3(n)              (1 << (n)) /* Bit n: Buffer MBn Interrupt (95TO64) */

#define IMX9_CAN_FLTC_ATP_IE       (1 << 0)  /* Active to passive interrupt enable */
#define IMX9_CAN_FLTC_PTA_IE       (1 << 1)  /* Passive to active interrupt enable */

/* TODO: add bit definitions for FLTCONF_IO to ETDC registers when needed */

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
#define CAN_FDCTRL_MBDSR0(x)       (((uint32_t)(((uint32_t)(x)) << CAN_FDCTRL_MBDSR0_SHIFT)) & CAN_FDCTRL_MBDSR0_MASK)
                                             /* Bit 18: Reserved */
#define CAN_FDCTRL_MBDSR1_SHIFT    (19)      /* Bits 19-20: Message Buffer Data Size for Region 1 */
#define CAN_FDCTRL_MBDSR1_MASK     (0x3 << CAN_FDCTRL_MBDSR1_SHIFT)
#define CAN_FDCTRL_MBDSR1(x)       (((uint32_t)(((uint32_t)(x)) << CAN_FDCTRL_MBDSR1_SHIFT)) & CAN_FDCTRL_MBDSR1_MASK)

#define CAN_FDCTRL_MBDSR2_SHIFT    (22)      /* Bits 22-23: Message Buffer Data Size for Region 2 */
#define CAN_FDCTRL_MBDSR2_MASK     (0x3 << CAN_FDCTRL_MBDSR2_SHIFT)
#define CAN_FDCTRL_MBDSR2(x)       (((uint32_t)(((uint32_t)(x)) << CAN_FDCTRL_MBDSR2_SHIFT)) & CAN_FDCTRL_MBDSR2_MASK)
                                             /* Bits 24-30: Reserved */
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

/* CAN MB TX & RX codes */

#define CAN_RXMB_INACTIVE          0x0        /* MB is inactive */
#define CAN_RXMB_FULL              0x2        /* MB is full */
#define CAN_RXMB_EMPTY             0x4        /* MB is empty */
#define CAN_RXMB_OVERRUN           0x6        /* overrun */
#define CAN_RXMB_BUSY_BIT          0x1        /* BUSY, orred with any of the above */

/* CAN MB TX codes */

#define CAN_TXMB_INACTIVE          0x8        /* MB is not active. */
#define CAN_TXMB_ABORT             0x9        /* MB is aborted. */
#define CAN_TXMB_DATAORREMOTE      0xC        /* MB is a TX Data Frame(when MB RTR = 0) or */
                                              /* MB is a TX Remote Request Frame (when MB RTR = 1). */
#define CAN_TXMB_TANSWER           0xE        /* MB is a TX Response Request Frame from */
                                              /* an incoming Remote Request Frame. */

/* CAN MB CS fields (1st 32-bit word of header) */

#define CAN_MB_CS_TIMESTAMP_SHIFT  0          /* Free-Running Counter Timestamp */
#define CAN_MB_CS_TIMESTAMP_MASK   (0xffff << CAN_MB_CS_TIMESTAMP_SHIFT)
#define CAN_MB_CS_TIMESTAMP(x)     (((x) & CAN_MB_CS_TIMESTAMP_MASK) >> CAN_MB_CS_TIMESTAMP_SHIFT)
#define CAN_MB_CS_DLC_SHIFT 16                /* Length of Data in Bytes */
#define CAN_MB_CS_DLC_MASK         (0xf << CAN_MB_CS_DLC_SHIFT)
#define CAN_MB_CS_DLC(x)           (((x) & CAN_MB_CS_DLC_MASK) >> CAN_MB_CS_DLC_SHIFT)
#define CAN_MB_CS_RTR              (1 << 20)  /* Remote Transmission Request */
#define CAN_MB_CS_IDE              (1 << 21)  /* ID Extended Bit */
#define CAN_MB_CS_SSR              (1 << 22)  /* Substitute Remote Request */
                                              /* Bit 23: Reserved */
#define CAN_MB_CS_CODE_SHIFT       24         /* Message buffer code */
#define CAN_MB_CS_CODE_MASK        (0xf << CAN_MB_CS_CODE_SHIFT)
                                              /* Bit 28: Reserved */
#define CAN_MB_CS_CODE(x)          (((x) & CAN_MB_CS_CODE_MASK) >> CAN_MB_CS_CODE_SHIFT)
#define CAN_MB_CS_ESI              (1 << 29)  /* Error State Indicator */
#define CAN_MB_CS_BRS              (1 << 30)  /* Bit Rate Switch */
#define CAN_MB_CS_EDL              (1 << 31)  /* Extended Data Length */

/* CAN MB PRIO and ID fields (2nd 32-bit word of header) */

#define CAN_MB_ID_ID_SHIFT         0
#define CAN_MB_ID_ID_MASK          (0x1fffffff << CAN_MB_ID_ID_SHIFT)
#define CAN_MB_ID_ID_STD_SHIFT     18
#define CAN_MB_ID_ID_STD_MASK      (0x7ff << CAN_MB_ID_ID_STD_SHIFT)
#define CAN_MB_ID_PRIO_SHIFT       29
#define CAN_MB_ID_PRIO_MASK        (0x7 << CAN_MB_ID_PRIO_SHIFT)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#endif /* __ARCH_ARM64_SRC_IMX9_HARDWARE_IMX9_FLEXCAN_H */
