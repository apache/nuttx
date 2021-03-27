/****************************************************************************
 * arch/arm/src/s32k1xx/hardware/s32k1xx_flexcan.h
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

#ifndef __ARCH_ARM_SRC_S32K1XX_HARDWARE_S32K1XX_FLEXCAN_H
#define __ARCH_ARM_SRC_S32K1XX_HARDWARE_S32K1XX_FLEXCAN_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <hardware/s32k1xx_memorymap.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define S32K1XX_CAN_MCR_OFFSET        0x0000  /* Module Configuration Register */
#define S32K1XX_CAN_CTRL1_OFFSET      0x0004  /* Control 1 Register */
#define S32K1XX_CAN_TIMER_OFFSET      0x0008  /* Free Running Timer */
#define S32K1XX_CAN_RXMGMASK_OFFSET   0x0010  /* Rx Mailboxes Global Mask Register */
#define S32K1XX_CAN_RX14MASK_OFFSET   0x0014  /* Rx 14 Mask Register */
#define S32K1XX_CAN_RX15MASK_OFFSET   0x0018  /* Rx 15 Mask Register */
#define S32K1XX_CAN_ECR_OFFSET        0x001c  /* Error Counter */
#define S32K1XX_CAN_ESR1_OFFSET       0x0020  /* Error and Status 1 Register */
#define S32K1XX_CAN_IMASK2_OFFSET     0x0024  /* Interrupt Masks 2 Register */
#define S32K1XX_CAN_IMASK1_OFFSET     0x0028  /* Interrupt Masks 1 Register */
#define S32K1XX_CAN_IFLAG2_OFFSET     0x002c  /* Interrupt Flags 2 Register */
#define S32K1XX_CAN_IFLAG1_OFFSET     0x0030  /* Interrupt Flags 1 Register */
#define S32K1XX_CAN_CTRL2_OFFSET      0x0034  /* Control 2 Register */
#define S32K1XX_CAN_ESR2_OFFSET       0x0038  /* Error and Status 2 Register */
#define S32K1XX_CAN_CRCR_OFFSET       0x0044  /* CRC Register */
#define S32K1XX_CAN_RXFGMASK_OFFSET   0x0048  /* Rx FIFO Global Mask Register */
#define S32K1XX_CAN_RXFIR_OFFSET      0x004c  /* Rx FIFO Information Register */
#define S32K1XX_CAN_CBT_OFFSET        0x0050  /* CAN Bit Timing register */

#define S32K1XX_CAN_MB_OFFSET         0x0080  /* CAN MB register */

#define S32K1XX_CAN_RXIMR_OFFSET(n)   (0x0880 + ((n) << 2))
#  define S32K1XX_CAN_RXIMR0_OFFSET   0x0880  /* R0 Individual Mask Registers */
#  define S32K1XX_CAN_RXIMR1_OFFSET   0x0884  /* R1 Individual Mask Registers */
#  define S32K1XX_CAN_RXIMR2_OFFSET   0x0888  /* R2 Individual Mask Registers */
#  define S32K1XX_CAN_RXIMR3_OFFSET   0x088c  /* R3 Individual Mask Registers */
#  define S32K1XX_CAN_RXIMR4_OFFSET   0x0890  /* R4 Individual Mask Registers */
#  define S32K1XX_CAN_RXIMR5_OFFSET   0x0894  /* R5 Individual Mask Registers */
#  define S32K1XX_CAN_RXIMR6_OFFSET   0x0898  /* R6 Individual Mask Registers */
#  define S32K1XX_CAN_RXIMR7_OFFSET   0x089c  /* R7 Individual Mask Registers */
#  define S32K1XX_CAN_RXIMR8_OFFSET   0x08a0  /* R8 Individual Mask Registers */
#  define S32K1XX_CAN_RXIMR9_OFFSET   0x08a4  /* R9 Individual Mask Registers */
#  define S32K1XX_CAN_RXIMR10_OFFSET  0x08a8  /* R10 Individual Mask Registers */
#  define S32K1XX_CAN_RXIMR11_OFFSET  0x08ac  /* R11 Individual Mask Registers */
#  define S32K1XX_CAN_RXIMR12_OFFSET  0x08b0  /* R12 Individual Mask Registers */
#  define S32K1XX_CAN_RXIMR13_OFFSET  0x08b4  /* R13 Individual Mask Registers */
#  define S32K1XX_CAN_RXIMR14_OFFSET  0x08b8  /* R14 Individual Mask Registers */
#  define S32K1XX_CAN_RXIMR15_OFFSET  0x08bc  /* R15 Individual Mask Registers */
#  define S32K1XX_CAN_RXIMR16_OFFSET  0x08c0  /* R16 Individual Mask Registers */
#  define S32K1XX_CAN_RXIMR17_OFFSET  0x08c4  /* R17 Individual Mask Registers */
#  define S32K1XX_CAN_RXIMR18_OFFSET  0x08c8  /* R18 Individual Mask Registers */
#  define S32K1XX_CAN_RXIMR19_OFFSET  0x08cc  /* R19 Individual Mask Registers */
#  define S32K1XX_CAN_RXIMR20_OFFSET  0x08d0  /* R20 Individual Mask Registers */
#  define S32K1XX_CAN_RXIMR21_OFFSET  0x08d4  /* R21 Individual Mask Registers */
#  define S32K1XX_CAN_RXIMR22_OFFSET  0x08d8  /* R22 Individual Mask Registers */
#  define S32K1XX_CAN_RXIMR23_OFFSET  0x08dc  /* R23 Individual Mask Registers */
#  define S32K1XX_CAN_RXIMR24_OFFSET  0x08e0  /* R24 Individual Mask Registers */
#  define S32K1XX_CAN_RXIMR25_OFFSET  0x08e4  /* R25 Individual Mask Registers */
#  define S32K1XX_CAN_RXIMR26_OFFSET  0x08e8  /* R26 Individual Mask Registers */
#  define S32K1XX_CAN_RXIMR27_OFFSET  0x08ec  /* R27 Individual Mask Registers */
#  define S32K1XX_CAN_RXIMR28_OFFSET  0x08f0  /* R28 Individual Mask Registers */
#  define S32K1XX_CAN_RXIMR29_OFFSET  0x08f4  /* R29 Individual Mask Registers */
#  define S32K1XX_CAN_RXIMR30_OFFSET  0x08f8  /* R30 Individual Mask Registers */
#  define S32K1XX_CAN_RXIMR31_OFFSET  0x08fc  /* R31 Individual Mask Registers */
#define S32K1XX_CAN_RXIMR_COUNT       32      /* Individual Mask Registers Count */

#define S32K1XX_CAN_CTRL1_PN_OFFSET   0x0b00  /* Pretended Networking Control 1 register */
#define S32K1XX_CAN_CTRL2_PN_OFFSET   0x0b04  /* Pretended Networking Control 2 register */
#define S32K1XX_CAN_WU_MTC_OFFSET     0x0b08  /* Pretended Networking Wake Up Match register */
#define S32K1XX_CAN_FLT_ID1_OFFSET    0x0b0c  /* Pretended Networking ID Filter 1 register */
#define S32K1XX_CAN_FLT_DLC_OFFSET    0x0b10  /* Pretended Networking DLC Filter register */
#define S32K1XX_CAN_PL1_LO_OFFSET     0x0b14  /* Pretended Networking Payload Low Filter 1 register */
#define S32K1XX_CAN_PL1_HI_OFFSET     0x0b18  /* Pretended Networking Payload High Filter 1 register */

#define S32K1XX_CAN_FLT_ID2_IDMASK_OFFSET 0x0b1c  /* Pretended Networking ID Filter 2 /
                                                   * ID Mask register */
#define S32K1XX_CAN_PL2_PLMASK_LO_OFFSET  0x0b20  /* Pretended Networking Payload Low Filter 2 /
                                                   * Payload Low Mask register */
#define S32K1XX_CAN_PL2_PLMASK_HI_OFFSET  0x0b24  /* Pretended Networking Payload High Filter 2
                                                   * low order bits / Payload High Mask register */

#define S32K1XX_CAN_WMB_OFFSET(n)     (0x0b40 + ((n) << 4)
#  define S32K1XX_CAN_WMB_CS_OFFSET   0x0000  /* Wake Up Message Buffer register for C/S */
#  define S32K1XX_CAN_WMB_ID_OFFSET   0x0004  /* Wake Up Message Buffer Register for ID */
#  define S32K1XX_CAN_WMB_D03_OFFSET  0x0008  /* Wake Up Message Buffer Register for Data 0-3 */
#  define S32K1XX_CAN_WMB_D47_OFFSET  0x000c  /* Wake Up Message Buffer Register Data 4-7 */

#  define S32K1XX_CAN_WMB0_CS_OFFSET  0x0b40  /* Wake Up Message Buffer register for C/S */
#  define S32K1XX_CAN_WMB0_ID_OFFSET  0x0b44  /* Wake Up Message Buffer Register for ID */
#  define S32K1XX_CAN_WMB0_D03_OFFSET 0x0b48  /* Wake Up Message Buffer Register for Data 0-3 */
#  define S32K1XX_CAN_WMB0_D47_OFFSET 0x0b4c  /* Wake Up Message Buffer Register Data 4-7 */
#  define S32K1XX_CAN_WMB1_CS_OFFSET  0x0b50  /* Wake Up Message Buffer register for C/S */
#  define S32K1XX_CAN_WMB1_ID_OFFSET  0x0b54  /* Wake Up Message Buffer Register for ID */
#  define S32K1XX_CAN_WMB1_D03_OFFSET 0x0b58  /* Wake Up Message Buffer Register for Data 0-3 */
#  define S32K1XX_CAN_WMB1_D47_OFFSET 0x0b5c  /* Wake Up Message Buffer Register Data 4-7 */
#  define S32K1XX_CAN_WMB2_CS_OFFSET  0x0b60  /* Wake Up Message Buffer register for C/S */
#  define S32K1XX_CAN_WMB2_ID_OFFSET  0x0b64  /* Wake Up Message Buffer Register for ID */
#  define S32K1XX_CAN_WMB2_D03_OFFSET 0x0b68  /* Wake Up Message Buffer Register for Data 0-3 */
#  define S32K1XX_CAN_WMB2_D47_OFFSET 0x0b6c  /* Wake Up Message Buffer Register Data 4-7 */
#  define S32K1XX_CAN_WMB3_CS_OFFSET  0x0b70  /* Wake Up Message Buffer register for C/S */
#  define S32K1XX_CAN_WMB3_ID_OFFSET  0x0b74  /* Wake Up Message Buffer Register for ID */
#  define S32K1XX_CAN_WMB3_D03_OFFSET 0x0b78  /* Wake Up Message Buffer Register for Data 0-3 */
#  define S32K1XX_CAN_WMB3_D47_OFFSET 0x0b7c  /* Wake Up Message Buffer Register Data 4-7 */

#define S32K1XX_CAN_FDCTRL_OFFSET     0x0c00  /* CAN FD Control register */
#define S32K1XX_CAN_FDCBT_OFFSET      0x0c04  /* CAN FD Bit Timing register */
#define S32K1XX_CAN_FDCRC_OFFSET      0x0c08  /* CAN FD CRC register */

/* Register Addresses *******************************************************/

#define S32K1XX_CAN0_MCR              (S32K1XX_FLEXCAN0_BASE + S32K1XX_CAN_MCR_OFFSET)
#define S32K1XX_CAN0_CTRL1            (S32K1XX_FLEXCAN0_BASE + S32K1XX_CAN_CTRL1_OFFSET)
#define S32K1XX_CAN0_TIMER            (S32K1XX_FLEXCAN0_BASE + S32K1XX_CAN_TIMER_OFFSET)
#define S32K1XX_CAN0_RXMGMASK         (S32K1XX_FLEXCAN0_BASE + S32K1XX_CAN_RXMGMASK_OFFSET)
#define S32K1XX_CAN0_RX14MASK         (S32K1XX_FLEXCAN0_BASE + S32K1XX_CAN_RX14MASK_OFFSET)
#define S32K1XX_CAN0_RX15MASK         (S32K1XX_FLEXCAN0_BASE + S32K1XX_CAN_RX15MASK_OFFSET)
#define S32K1XX_CAN0_ECR              (S32K1XX_FLEXCAN0_BASE + S32K1XX_CAN_ECR_OFFSET)
#define S32K1XX_CAN0_ESR1             (S32K1XX_FLEXCAN0_BASE + S32K1XX_CAN_ESR1_OFFSET)
#define S32K1XX_CAN0_IMASK2           (S32K1XX_FLEXCAN0_BASE + S32K1XX_CAN_IMASK2_OFFSET)
#define S32K1XX_CAN0_IMASK1           (S32K1XX_FLEXCAN0_BASE + S32K1XX_CAN_IMASK1_OFFSET)
#define S32K1XX_CAN0_IFLAG2           (S32K1XX_FLEXCAN0_BASE + S32K1XX_CAN_IFLAG2_OFFSET)
#define S32K1XX_CAN0_IFLAG1           (S32K1XX_FLEXCAN0_BASE + S32K1XX_CAN_IFLAG1_OFFSET)
#define S32K1XX_CAN0_CTRL2            (S32K1XX_FLEXCAN0_BASE + S32K1XX_CAN_CTRL2_OFFSET)
#define S32K1XX_CAN0_ESR2             (S32K1XX_FLEXCAN0_BASE + S32K1XX_CAN_ESR2_OFFSET)
#define S32K1XX_CAN0_CRCR             (S32K1XX_FLEXCAN0_BASE + S32K1XX_CAN_CRCR_OFFSET)
#define S32K1XX_CAN0_RXFGMASK         (S32K1XX_FLEXCAN0_BASE + S32K1XX_CAN_RXFGMASK_OFFSET)
#define S32K1XX_CAN0_RXFIR            (S32K1XX_FLEXCAN0_BASE + S32K1XX_CAN_RXFIR_OFFSET)
#define S32K1XX_CAN0_CBT              (S32K1XX_FLEXCAN0_BASE + S32K1XX_CAN_CBT_OFFSET)
#define S32K1XX_CAN0_MB               (S32K1XX_FLEXCAN0_BASE + S32K1XX_CAN_MB_OFFSET)
#define S32K1XX_CAN0_FDCTRL           (S32K1XX_FLEXCAN0_BASE + S32K1XX_CAN_FDCTRL_OFFSET)
#define S32K1XX_CAN0_FDCBT            (S32K1XX_FLEXCAN0_BASE + S32K1XX_CAN_FDCBT_OFFSET)
#define S32K1XX_CAN0_FDCRC            (S32K1XX_FLEXCAN0_BASE + S32K1XX_CAN_FDCRC_OFFSET)

#define S32K1XX_CAN0_RXIMR(n)         (S32K1XX_FLEXCAN0_BASE + S32K1XX_CAN_RXIMR_OFFSET(n))
#  define S32K1XX_CAN0_RXIMR0         (S32K1XX_FLEXCAN0_BASE + S32K1XX_CAN_RXIMR0_OFFSET)
#  define S32K1XX_CAN0_RXIMR1         (S32K1XX_FLEXCAN0_BASE + S32K1XX_CAN_RXIMR1_OFFSET)
#  define S32K1XX_CAN0_RXIMR2         (S32K1XX_FLEXCAN0_BASE + S32K1XX_CAN_RXIMR2_OFFSET)
#  define S32K1XX_CAN0_RXIMR3         (S32K1XX_FLEXCAN0_BASE + S32K1XX_CAN_RXIMR3_OFFSET)
#  define S32K1XX_CAN0_RXIMR4         (S32K1XX_FLEXCAN0_BASE + S32K1XX_CAN_RXIMR4_OFFSET)
#  define S32K1XX_CAN0_RXIMR5         (S32K1XX_FLEXCAN0_BASE + S32K1XX_CAN_RXIMR5_OFFSET)
#  define S32K1XX_CAN0_RXIMR6         (S32K1XX_FLEXCAN0_BASE + S32K1XX_CAN_RXIMR6_OFFSET)
#  define S32K1XX_CAN0_RXIMR7         (S32K1XX_FLEXCAN0_BASE + S32K1XX_CAN_RXIMR7_OFFSET)
#  define S32K1XX_CAN0_RXIMR8         (S32K1XX_FLEXCAN0_BASE + S32K1XX_CAN_RXIMR8_OFFSET)
#  define S32K1XX_CAN0_RXIMR9         (S32K1XX_FLEXCAN0_BASE + S32K1XX_CAN_RXIMR9_OFFSET)
#  define S32K1XX_CAN0_RXIMR10        (S32K1XX_FLEXCAN0_BASE + S32K1XX_CAN_RXIMR10_OFFSET)
#  define S32K1XX_CAN0_RXIMR11        (S32K1XX_FLEXCAN0_BASE + S32K1XX_CAN_RXIMR11_OFFSET)
#  define S32K1XX_CAN0_RXIMR12        (S32K1XX_FLEXCAN0_BASE + S32K1XX_CAN_RXIMR12_OFFSET)
#  define S32K1XX_CAN0_RXIMR13        (S32K1XX_FLEXCAN0_BASE + S32K1XX_CAN_RXIMR13_OFFSET)
#  define S32K1XX_CAN0_RXIMR14        (S32K1XX_FLEXCAN0_BASE + S32K1XX_CAN_RXIMR14_OFFSET)
#  define S32K1XX_CAN0_RXIMR15        (S32K1XX_FLEXCAN0_BASE + S32K1XX_CAN_RXIMR15_OFFSET)
#  define S32K1XX_CAN0_RXIMR16        (S32K1XX_FLEXCAN0_BASE + S32K1XX_CAN_RXIMR16_OFFSET)
#  define S32K1XX_CAN0_RXIMR17        (S32K1XX_FLEXCAN0_BASE + S32K1XX_CAN_RXIMR17_OFFSET)
#  define S32K1XX_CAN0_RXIMR18        (S32K1XX_FLEXCAN0_BASE + S32K1XX_CAN_RXIMR18_OFFSET)
#  define S32K1XX_CAN0_RXIMR19        (S32K1XX_FLEXCAN0_BASE + S32K1XX_CAN_RXIMR19_OFFSET)
#  define S32K1XX_CAN0_RXIMR20        (S32K1XX_FLEXCAN0_BASE + S32K1XX_CAN_RXIMR20_OFFSET)
#  define S32K1XX_CAN0_RXIMR21        (S32K1XX_FLEXCAN0_BASE + S32K1XX_CAN_RXIMR21_OFFSET)
#  define S32K1XX_CAN0_RXIMR22        (S32K1XX_FLEXCAN0_BASE + S32K1XX_CAN_RXIMR22_OFFSET)
#  define S32K1XX_CAN0_RXIMR23        (S32K1XX_FLEXCAN0_BASE + S32K1XX_CAN_RXIMR23_OFFSET)
#  define S32K1XX_CAN0_RXIMR24        (S32K1XX_FLEXCAN0_BASE + S32K1XX_CAN_RXIMR24_OFFSET)
#  define S32K1XX_CAN0_RXIMR25        (S32K1XX_FLEXCAN0_BASE + S32K1XX_CAN_RXIMR25_OFFSET)
#  define S32K1XX_CAN0_RXIMR26        (S32K1XX_FLEXCAN0_BASE + S32K1XX_CAN_RXIMR26_OFFSET)
#  define S32K1XX_CAN0_RXIMR27        (S32K1XX_FLEXCAN0_BASE + S32K1XX_CAN_RXIMR27_OFFSET)
#  define S32K1XX_CAN0_RXIMR28        (S32K1XX_FLEXCAN0_BASE + S32K1XX_CAN_RXIMR28_OFFSET)
#  define S32K1XX_CAN0_RXIMR29        (S32K1XX_FLEXCAN0_BASE + S32K1XX_CAN_RXIMR29_OFFSET)
#  define S32K1XX_CAN0_RXIMR30        (S32K1XX_FLEXCAN0_BASE + S32K1XX_CAN_RXIMR30_OFFSET)
#  define S32K1XX_CAN0_RXIMR31        (S32K1XX_FLEXCAN0_BASE + S32K1XX_CAN_RXIMR31_OFFSET)

#define S32K1XX_CAN0_CTRL1_PN         (S32K1XX_FLEXCAN0_BASE + S32K1XX_CAN_CTRL1_PN_OFFSET)
#define S32K1XX_CAN0_CTRL2_PN         (S32K1XX_FLEXCAN0_BASE + S32K1XX_CAN_CTRL2_PN_OFFSET)
#define S32K1XX_CAN0_WU_MTC           (S32K1XX_FLEXCAN0_BASE + S32K1XX_CAN_WU_MTC_OFFSET)
#define S32K1XX_CAN0_FLT_ID1          (S32K1XX_FLEXCAN0_BASE + S32K1XX_CAN_FLT_ID1_OFFSET)
#define S32K1XX_CAN0_FLT_DLC          (S32K1XX_FLEXCAN0_BASE + S32K1XX_CAN_FLT_DLC_OFFSET)
#define S32K1XX_CAN0_PL1_LO           (S32K1XX_FLEXCAN0_BASE + S32K1XX_CAN_PL1_LO_OFFSET)
#define S32K1XX_CAN0_PL1_HI           (S32K1XX_FLEXCAN0_BASE + S32K1XX_CAN_PL1_HI_OFFSET)

#define S32K1XX_CAN0_FLT_ID2_IDMASK   (S32K1XX_FLEXCAN0_BASE + S32K1XX_CAN_FLT_ID2_IDMASK_OFFSET)
#define S32K1XX_CAN0_PL2_PLMASK_LO    (S32K1XX_FLEXCAN0_BASE + S32K1XX_CAN_PL2_PLMASK_LO_OFFSET)
#define S32K1XX_CAN0_PL2_PLMASK_HI    (S32K1XX_FLEXCAN0_BASE + S32K1XX_CAN_PL2_PLMASK_HI_OFFSET)

#define S32K1XX_CAN0_WMB_BASE(n)      (S32K1XX_FLEXCAN0_BASE + S32K1XX_CAN_WMB_OFFSET(n))
#  define S32K1XX_CAN0_WMB_CS(n)      (S32K1XX_CAN_WMB_BASE(n) + S32K1XX_CAN_WMB_CS_OFFSET)
#  define S32K1XX_CAN0_WMB_ID(n)      (S32K1XX_CAN_WMB_BASE(n) + S32K1XX_CAN_WMB_ID_OFFSET)
#  define S32K1XX_CAN0_WMB_D03(n)     (S32K1XX_CAN_WMB_BASE(n) + S32K1XX_CAN_WMB_D03_OFFSET)
#  define S32K1XX_CAN0_WMB_D47(n)     (S32K1XX_CAN_WMB_BASE(n) + S32K1XX_CAN_WMB_D47_OFFSET)

#  define S32K1XX_CAN0_WMB0_CS        (S32K1XX_FLEXCAN0_BASE + S32K1XX_CAN_WMB0_CS_OFFSET)
#  define S32K1XX_CAN0_WMB0_ID        (S32K1XX_FLEXCAN0_BASE + S32K1XX_CAN_WMB0_ID_OFFSET)
#  define S32K1XX_CAN0_WMB0_D03       (S32K1XX_FLEXCAN0_BASE + S32K1XX_CAN_WMB0_D03_OFFSET)
#  define S32K1XX_CAN0_WMB0_D47       (S32K1XX_FLEXCAN0_BASE + S32K1XX_CAN_WMB0_D47_OFFSET)
#  define S32K1XX_CAN0_WMB1_CS        (S32K1XX_FLEXCAN0_BASE + S32K1XX_CAN_WMB1_CS_OFFSET)
#  define S32K1XX_CAN0_WMB1_ID        (S32K1XX_FLEXCAN0_BASE + S32K1XX_CAN_WMB1_ID_OFFSET)
#  define S32K1XX_CAN0_WMB1_D03       (S32K1XX_FLEXCAN0_BASE + S32K1XX_CAN_WMB1_D03_OFFSET)
#  define S32K1XX_CAN0_WMB1_D47       (S32K1XX_FLEXCAN0_BASE + S32K1XX_CAN_WMB1_D47_OFFSET)
#  define S32K1XX_CAN0_WMB2_CS        (S32K1XX_FLEXCAN0_BASE + S32K1XX_CAN_WMB2_CS_OFFSET)
#  define S32K1XX_CAN0_WMB2_ID        (S32K1XX_FLEXCAN0_BASE + S32K1XX_CAN_WMB2_ID_OFFSET)
#  define S32K1XX_CAN0_WMB2_D03       (S32K1XX_FLEXCAN0_BASE + S32K1XX_CAN_WMB2_D03_OFFSET)
#  define S32K1XX_CAN0_WMB2_D47       (S32K1XX_FLEXCAN0_BASE + S32K1XX_CAN_WMB2_D47_OFFSET)
#  define S32K1XX_CAN0_WMB3_CS        (S32K1XX_FLEXCAN0_BASE + S32K1XX_CAN_WMB3_CS_OFFSET)
#  define S32K1XX_CAN0_WMB3_ID        (S32K1XX_FLEXCAN0_BASE + S32K1XX_CAN_WMB3_ID_OFFSET)
#  define S32K1XX_CAN0_WMB3_D03       (S32K1XX_FLEXCAN0_BASE + S32K1XX_CAN_WMB3_D03_OFFSET)
#  define S32K1XX_CAN0_WMB3_D47       (S32K1XX_FLEXCAN0_BASE + S32K1XX_CAN_WMB3_D47_OFFSET)

#define S32K1XX_CAN0_FDCTRL           (S32K1XX_FLEXCAN0_BASE + S32K1XX_CAN_FDCTRL_OFFSET)
#define S32K1XX_CAN0_FDCBT            (S32K1XX_FLEXCAN0_BASE + S32K1XX_CAN_FDCBT_OFFSET)
#define S32K1XX_CAN0_FDCRC            (S32K1XX_FLEXCAN0_BASE + S32K1XX_CAN_FDCRC_OFFSET)

#define S32K1XX_CAN1_MCR              (S32K1XX_FLEXCAN1_BASE + S32K1XX_CAN_MCR_OFFSET)
#define S32K1XX_CAN1_CTRL1            (S32K1XX_FLEXCAN1_BASE + S32K1XX_CAN_CTRL1_OFFSET)
#define S32K1XX_CAN1_TIMER            (S32K1XX_FLEXCAN1_BASE + S32K1XX_CAN_TIMER_OFFSET)
#define S32K1XX_CAN1_RXMGMASK         (S32K1XX_FLEXCAN1_BASE + S32K1XX_CAN_RXMGMASK_OFFSET)
#define S32K1XX_CAN1_RX14MASK         (S32K1XX_FLEXCAN1_BASE + S32K1XX_CAN_RX14MASK_OFFSET)
#define S32K1XX_CAN1_RX15MASK         (S32K1XX_FLEXCAN1_BASE + S32K1XX_CAN_RX15MASK_OFFSET)
#define S32K1XX_CAN1_ECR              (S32K1XX_FLEXCAN1_BASE + S32K1XX_CAN_ECR_OFFSET)
#define S32K1XX_CAN1_ESR1             (S32K1XX_FLEXCAN1_BASE + S32K1XX_CAN_ESR1_OFFSET)
#define S32K1XX_CAN1_IMASK2           (S32K1XX_FLEXCAN1_BASE + S32K1XX_CAN_IMASK2_OFFSET)
#define S32K1XX_CAN1_IMASK1           (S32K1XX_FLEXCAN1_BASE + S32K1XX_CAN_IMASK1_OFFSET)
#define S32K1XX_CAN1_IFLAG2           (S32K1XX_FLEXCAN1_BASE + S32K1XX_CAN_IFLAG2_OFFSET)
#define S32K1XX_CAN1_IFLAG1           (S32K1XX_FLEXCAN1_BASE + S32K1XX_CAN_IFLAG1_OFFSET)
#define S32K1XX_CAN1_CTRL2            (S32K1XX_FLEXCAN1_BASE + S32K1XX_CAN_CTRL2_OFFSET)
#define S32K1XX_CAN1_ESR2             (S32K1XX_FLEXCAN1_BASE + S32K1XX_CAN_ESR2_OFFSET)
#define S32K1XX_CAN1_CRCR             (S32K1XX_FLEXCAN1_BASE + S32K1XX_CAN_CRCR_OFFSET)
#define S32K1XX_CAN1_RXFGMASK         (S32K1XX_FLEXCAN1_BASE + S32K1XX_CAN_RXFGMASK_OFFSET)
#define S32K1XX_CAN1_RXFIR            (S32K1XX_FLEXCAN1_BASE + S32K1XX_CAN_RXFIR_OFFSET)

#define S32K1XX_CAN1_RXIMR(n)         (S32K1XX_FLEXCAN1_BASE + S32K1XX_CAN_RXIMR_OFFSET(n))
#  define S32K1XX_CAN1_RXIMR0         (S32K1XX_FLEXCAN1_BASE + S32K1XX_CAN_RXIMR0_OFFSET)
#  define S32K1XX_CAN1_RXIMR1         (S32K1XX_FLEXCAN1_BASE + S32K1XX_CAN_RXIMR1_OFFSET)
#  define S32K1XX_CAN1_RXIMR2         (S32K1XX_FLEXCAN1_BASE + S32K1XX_CAN_RXIMR2_OFFSET)
#  define S32K1XX_CAN1_RXIMR3         (S32K1XX_FLEXCAN1_BASE + S32K1XX_CAN_RXIMR3_OFFSET)
#  define S32K1XX_CAN1_RXIMR4         (S32K1XX_FLEXCAN1_BASE + S32K1XX_CAN_RXIMR4_OFFSET)
#  define S32K1XX_CAN1_RXIMR5         (S32K1XX_FLEXCAN1_BASE + S32K1XX_CAN_RXIMR5_OFFSET)
#  define S32K1XX_CAN1_RXIMR6         (S32K1XX_FLEXCAN1_BASE + S32K1XX_CAN_RXIMR6_OFFSET)
#  define S32K1XX_CAN1_RXIMR7         (S32K1XX_FLEXCAN1_BASE + S32K1XX_CAN_RXIMR7_OFFSET)
#  define S32K1XX_CAN1_RXIMR8         (S32K1XX_FLEXCAN1_BASE + S32K1XX_CAN_RXIMR8_OFFSET)
#  define S32K1XX_CAN1_RXIMR9         (S32K1XX_FLEXCAN1_BASE + S32K1XX_CAN_RXIMR9_OFFSET)
#  define S32K1XX_CAN1_RXIMR10        (S32K1XX_FLEXCAN1_BASE + S32K1XX_CAN_RXIMR10_OFFSET)
#  define S32K1XX_CAN1_RXIMR11        (S32K1XX_FLEXCAN1_BASE + S32K1XX_CAN_RXIMR11_OFFSET)
#  define S32K1XX_CAN1_RXIMR12        (S32K1XX_FLEXCAN1_BASE + S32K1XX_CAN_RXIMR12_OFFSET)
#  define S32K1XX_CAN1_RXIMR13        (S32K1XX_FLEXCAN1_BASE + S32K1XX_CAN_RXIMR13_OFFSET)
#  define S32K1XX_CAN1_RXIMR14        (S32K1XX_FLEXCAN1_BASE + S32K1XX_CAN_RXIMR14_OFFSET)
#  define S32K1XX_CAN1_RXIMR15        (S32K1XX_FLEXCAN1_BASE + S32K1XX_CAN_RXIMR15_OFFSET)
#  define S32K1XX_CAN1_RXIMR16        (S32K1XX_FLEXCAN1_BASE + S32K1XX_CAN_RXIMR16_OFFSET)
#  define S32K1XX_CAN1_RXIMR17        (S32K1XX_FLEXCAN1_BASE + S32K1XX_CAN_RXIMR17_OFFSET)
#  define S32K1XX_CAN1_RXIMR18        (S32K1XX_FLEXCAN1_BASE + S32K1XX_CAN_RXIMR18_OFFSET)
#  define S32K1XX_CAN1_RXIMR19        (S32K1XX_FLEXCAN1_BASE + S32K1XX_CAN_RXIMR19_OFFSET)
#  define S32K1XX_CAN1_RXIMR20        (S32K1XX_FLEXCAN1_BASE + S32K1XX_CAN_RXIMR20_OFFSET)
#  define S32K1XX_CAN1_RXIMR21        (S32K1XX_FLEXCAN1_BASE + S32K1XX_CAN_RXIMR21_OFFSET)
#  define S32K1XX_CAN1_RXIMR22        (S32K1XX_FLEXCAN1_BASE + S32K1XX_CAN_RXIMR22_OFFSET)
#  define S32K1XX_CAN1_RXIMR23        (S32K1XX_FLEXCAN1_BASE + S32K1XX_CAN_RXIMR23_OFFSET)
#  define S32K1XX_CAN1_RXIMR24        (S32K1XX_FLEXCAN1_BASE + S32K1XX_CAN_RXIMR24_OFFSET)
#  define S32K1XX_CAN1_RXIMR25        (S32K1XX_FLEXCAN1_BASE + S32K1XX_CAN_RXIMR25_OFFSET)
#  define S32K1XX_CAN1_RXIMR26        (S32K1XX_FLEXCAN1_BASE + S32K1XX_CAN_RXIMR26_OFFSET)
#  define S32K1XX_CAN1_RXIMR27        (S32K1XX_FLEXCAN1_BASE + S32K1XX_CAN_RXIMR27_OFFSET)
#  define S32K1XX_CAN1_RXIMR28        (S32K1XX_FLEXCAN1_BASE + S32K1XX_CAN_RXIMR28_OFFSET)
#  define S32K1XX_CAN1_RXIMR29        (S32K1XX_FLEXCAN1_BASE + S32K1XX_CAN_RXIMR29_OFFSET)
#  define S32K1XX_CAN1_RXIMR30        (S32K1XX_FLEXCAN1_BASE + S32K1XX_CAN_RXIMR30_OFFSET)
#  define S32K1XX_CAN1_RXIMR31        (S32K1XX_FLEXCAN1_BASE + S32K1XX_CAN_RXIMR31_OFFSET)

#define S32K1XX_CAN1_CTRL1_PN         (S32K1XX_FLEXCAN1_BASE + S32K1XX_CAN_CTRL1_PN_OFFSET)
#define S32K1XX_CAN1_CTRL2_PN         (S32K1XX_FLEXCAN1_BASE + S32K1XX_CAN_CTRL2_PN_OFFSET)
#define S32K1XX_CAN1_WU_MTC           (S32K1XX_FLEXCAN1_BASE + S32K1XX_CAN_WU_MTC_OFFSET)
#define S32K1XX_CAN1_FLT_ID1          (S32K1XX_FLEXCAN1_BASE + S32K1XX_CAN_FLT_ID1_OFFSET)
#define S32K1XX_CAN1_FLT_DLC          (S32K1XX_FLEXCAN1_BASE + S32K1XX_CAN_FLT_DLC_OFFSET)
#define S32K1XX_CAN1_PL1_LO           (S32K1XX_FLEXCAN1_BASE + S32K1XX_CAN_PL1_LO_OFFSET)
#define S32K1XX_CAN1_PL1_HI           (S32K1XX_FLEXCAN1_BASE + S32K1XX_CAN_PL1_HI_OFFSET)

#define S32K1XX_CAN1_FLT_ID2_IDMASK   (S32K1XX_FLEXCAN1_BASE + S32K1XX_CAN_FLT_ID2_IDMASK_OFFSET)
#define S32K1XX_CAN1_PL2_PLMASK_LO    (S32K1XX_FLEXCAN1_BASE + S32K1XX_CAN_PL2_PLMASK_LO_OFFSET)
#define S32K1XX_CAN1_PL2_PLMASK_HI    (S32K1XX_FLEXCAN1_BASE + S32K1XX_CAN_PL2_PLMASK_HI_OFFSET)

#define S32K1XX_CAN1_WMB_BASE(n)      (S32K1XX_FLEXCAN1_BASE + S32K1XX_CAN_WMB_OFFSET(n))
#  define S32K1XX_CAN1_WMB_CS(n)      (S32K1XX_CAN_WMB_BASE(n) + S32K1XX_CAN_WMB_CS_OFFSET)
#  define S32K1XX_CAN1_WMB_ID(n)      (S32K1XX_CAN_WMB_BASE(n) + S32K1XX_CAN_WMB_ID_OFFSET)
#  define S32K1XX_CAN1_WMB_D03(n)     (S32K1XX_CAN_WMB_BASE(n) + S32K1XX_CAN_WMB_D03_OFFSET)
#  define S32K1XX_CAN1_WMB_D47(n)     (S32K1XX_CAN_WMB_BASE(n) + S32K1XX_CAN_WMB_D47_OFFSET)

#  define S32K1XX_CAN1_WMB0_CS        (S32K1XX_FLEXCAN1_BASE + S32K1XX_CAN_WMB0_CS_OFFSET)
#  define S32K1XX_CAN1_WMB0_ID        (S32K1XX_FLEXCAN1_BASE + S32K1XX_CAN_WMB0_ID_OFFSET)
#  define S32K1XX_CAN1_WMB0_D03       (S32K1XX_FLEXCAN1_BASE + S32K1XX_CAN_WMB0_D03_OFFSET)
#  define S32K1XX_CAN1_WMB0_D47       (S32K1XX_FLEXCAN1_BASE + S32K1XX_CAN_WMB0_D47_OFFSET)
#  define S32K1XX_CAN1_WMB1_CS        (S32K1XX_FLEXCAN1_BASE + S32K1XX_CAN_WMB1_CS_OFFSET)
#  define S32K1XX_CAN1_WMB1_ID        (S32K1XX_FLEXCAN1_BASE + S32K1XX_CAN_WMB1_ID_OFFSET)
#  define S32K1XX_CAN1_WMB1_D03       (S32K1XX_FLEXCAN1_BASE + S32K1XX_CAN_WMB1_D03_OFFSET)
#  define S32K1XX_CAN1_WMB1_D47       (S32K1XX_FLEXCAN1_BASE + S32K1XX_CAN_WMB1_D47_OFFSET)
#  define S32K1XX_CAN1_WMB2_CS        (S32K1XX_FLEXCAN1_BASE + S32K1XX_CAN_WMB2_CS_OFFSET)
#  define S32K1XX_CAN1_WMB2_ID        (S32K1XX_FLEXCAN1_BASE + S32K1XX_CAN_WMB2_ID_OFFSET)
#  define S32K1XX_CAN1_WMB2_D03       (S32K1XX_FLEXCAN1_BASE + S32K1XX_CAN_WMB2_D03_OFFSET)
#  define S32K1XX_CAN1_WMB2_D47       (S32K1XX_FLEXCAN1_BASE + S32K1XX_CAN_WMB2_D47_OFFSET)
#  define S32K1XX_CAN1_WMB3_CS        (S32K1XX_FLEXCAN1_BASE + S32K1XX_CAN_WMB3_CS_OFFSET)
#  define S32K1XX_CAN1_WMB3_ID        (S32K1XX_FLEXCAN1_BASE + S32K1XX_CAN_WMB3_ID_OFFSET)
#  define S32K1XX_CAN1_WMB3_D03       (S32K1XX_FLEXCAN1_BASE + S32K1XX_CAN_WMB3_D03_OFFSET)
#  define S32K1XX_CAN1_WMB3_D47       (S32K1XX_FLEXCAN1_BASE + S32K1XX_CAN_WMB3_D47_OFFSET)

#define S32K1XX_CAN1_FDCTRL           (S32K1XX_FLEXCAN1_BASE + S32K1XX_CAN_FDCTRL_OFFSET)
#define S32K1XX_CAN1_FDCBT            (S32K1XX_FLEXCAN1_BASE + S32K1XX_CAN_FDCBT_OFFSET)
#define S32K1XX_CAN1_FDCRC            (S32K1XX_FLEXCAN1_BASE + S32K1XX_CAN_FDCRC_OFFSET)

/* Register Bit Definitions *************************************************/

/* Module Configuration Register */

#define CAN_MCR_MAXMB_SHIFT           (0)       /* Bits 0-6: Number of the Last Message Buffer */
#define CAN_MCR_MAXMB_MASK            (0x7f << CAN_MCR_MAXMB_SHIFT)
                                                /* Bit 7:  Reserved */
#define CAN_MCR_IDAM_SHIFT            (8)       /* Bits 8-9: ID Acceptance Mode */

#define CAN_MCR_IDAM_MASK             (3 << CAN_MCR_IDAM_SHIFT)
#  define CAN_MCR_IDAM_FMTA           (0 << CAN_MCR_IDAM_SHIFT) /* Format A: One full ID  */
#  define CAN_MCR_IDAM_FMTB           (1 << CAN_MCR_IDAM_SHIFT) /* Format B: Two full (or partial) IDs */
#  define CAN_MCR_IDAM_FMTC           (2 << CAN_MCR_IDAM_SHIFT) /* Format C: Four partial IDs */
#  define CAN_MCR_IDAM_FMTD           (3 << CAN_MCR_IDAM_SHIFT) /* Format D: All frames rejected */

                                                /* Bit 10: Reserved */
#define CAN_MCR_FDEN                  (1 << 11) /* Bit 11: CAN FD operation enable */
#define CAN_MCR_AEN                   (1 << 12) /* Bit 12: Abort Enable */
#define CAN_MCR_LPRIOEN               (1 << 13) /* Bit 13: Local Priority Enable */
                                                /* Bits 14-15: Reserved */
#define CAN_MCR_IRMQ                  (1 << 16) /* Bit 16: Individual Rx Masking and Queue Enable */
#define CAN_MCR_SRXDIS                (1 << 17) /* Bit 17: Self Reception Disable */
#define CAN_MCR_DOZE                  (1 << 18) /* Bit 18: Doze Mode Enable */
                                                /* Bit 19: Reserved */
#define CAN_MCR_LPMACK                (1 << 20) /* Bit 20: Low Power Mode Acknowledge */
#define CAN_MCR_WRNEN                 (1 << 21) /* Bit 21: Warning Interrupt Enable */
#define CAN_MCR_SLFWAK                (1 << 22) /* Bit 22: Self Wake Up */
#define CAN_MCR_SUPV                  (1 << 23) /* Bit 23: Supervisor Mode */
#define CAN_MCR_FRZACK                (1 << 24) /* Bit 24: Freeze Mode Acknowledge */
#define CAN_MCR_SOFTRST               (1 << 25) /* Bit 25: Soft Reset */
#define CAN_MCR_WAKMSK                (1 << 26) /* Bit 26: Wake Up Interrupt Mask */
#define CAN_MCR_NOTRDY                (1 << 27) /* Bit 27: FlexCAN Not Ready */
#define CAN_MCR_HALT                  (1 << 28) /* Bit 28: Halt FlexCAN */
#define CAN_MCR_RFEN                  (1 << 29) /* Bit 29: Rx FIFO Enable */
#define CAN_MCR_FRZ                   (1 << 30) /* Bit 30: Freeze Enable */
#define CAN_MCR_MDIS                  (1 << 31) /* Bit 31: Module Disable */

/* Control 1 Register */

#define CAN_CTRL1_PROPSEG(x)          (((uint32_t)(((uint32_t)(x)) << 0)) & 0x7)
#define CAN_CTRL1_LOM                 (1 << 3)  /* Bit 3:  Listen-Only Mode */
#define CAN_CTRL1_LBUF                (1 << 4)  /* Bit 4:  Lowest Buffer Transmitted First */
#define CAN_CTRL1_TSYN                (1 << 5)  /* Bit 5:  Timer Sync */
#define CAN_CTRL1_BOFFREC             (1 << 6)  /* Bit 6:  Bus Off Recovery */
#define CAN_CTRL1_SMP                 (1 << 7)  /* Bit 7:  CAN Bit Sampling */
                                                /* Bits 8-9: Reserved */
#define CAN_CTRL1_RWRNMSK             (1 << 10) /* Bit 10: Rx Warning Interrupt Mask */
#define CAN_CTRL1_TWRNMSK             (1 << 11) /* Bit 11: Tx Warning Interrupt Mask */
#define CAN_CTRL1_LPB                 (1 << 12) /* Bit 12: Loop Back Mode */
#define CAN_CTRL1_CLKSRC              (1 << 13) /* Bit 13: CAN Engine Clock Source */
#define CAN_CTRL1_ERRMSK              (1 << 14) /* Bit 14: Error Mask */
#define CAN_CTRL1_BOFFMSK             (1 << 15) /* Bit 15: Bus Off Mask */
#define CAN_CTRL1_TIMINGMSK           (0xFFFF << 16)
#define CAN_CTRL1_PSEG2(x)            (((uint32_t)(((uint32_t)(x)) << 16)) & 0x70000)
#define CAN_CTRL1_PSEG1(x)            (((uint32_t)(((uint32_t)(x)) << 19)) & 0x380000)
#define CAN_CTRL1_RJW(x)              (((uint32_t)(((uint32_t)(x)) << 22)) & 0xC00000)
#define CAN_CTRL1_PRESDIV(x)          (((uint32_t)(((uint32_t)(x)) << 24)) & 0xFF000000)

/* Free Running Timer */

#define CAN_TIMER_SHIFT               (0)       /* Bits 0-15: Timer value */
#define CAN_TIMER_MASK                (0xffff << CAN_TIMER_SHIFT)
                                                /* Bits 16-31: Reserved */

/* Rx Mailboxes Global Mask Register (32 Rx Mailboxes Global Mask Bits) */

#define CAN_RXMGMASK(n)               (1 << (n)) /* Bit n: Rx Mailboxe n Global Mask Bit */

/* Rx 14 Mask Register */

#define CAN_RX14MASK(n)               (1 << (n)) /* Bit n: Rx Buffer 14 Mask Bit n */

/* Rx 15 Mask Register */

#define CAN_RX15MASK(n)               (1 << (n)) /* Bit n: Rx Buffer 15 Mask Bit n */

/* Error Counter */

#define CAN_ECR_TXERRCNT_SHIFT        (0)       /* Bits 0-7: Transmit Error Counter */
#define CAN_ECR_TXERRCNT_MASK         (0xff << CAN_ECR_TXERRCNT_SHIFT)
#define CAN_ECR_RXERRCNT_SHIFT        (8)       /* Bits 8-15: Receive Error Counter */
#define CAN_ECR_RXERRCNT_MASK         (0xff << CAN_ECR_RXERRCNT_SHIFT)
                                                /* Bits 16-31: Reserved */

/* Error and Status 1 Register */

#define CAN_ESR1_WAKINT               (1 << 0)  /* Bit 0:  Wake-Up Interrupt */
#define CAN_ESR1_ERRINT               (1 << 1)  /* Bit 1:  Error Interrupt */
#define CAN_ESR1_BOFFINT              (1 << 2)  /* Bit 2:  'Bus Off' Interrupt */
#define CAN_ESR1_RX                   (1 << 3)  /* Bit 3:  FlexCAN in Reception */
#define CAN_ESR1_FLTCONF_SHIFT        (4)       /* Bits 4-5: Fault Confinement State */

#define CAN_ESR1_FLTCONF_MASK         (3 << CAN_ESR1_FLTCONF_SHIFT)
#  define CAN_ESR1_FLTCONF_ACTV       (0 << CAN_ESR1_FLTCONF_SHIFT) /* Error Active */
#  define CAN_ESR1_FLTCONF_PASV       (1 << CAN_ESR1_FLTCONF_SHIFT) /* Error Passive */
#  define CAN_ESR1_FLTCONF_OFF        (2 << CAN_ESR1_FLTCONF_SHIFT) /* Bus Off */

#define CAN_ESR1_TX                   (1 << 6)  /* Bit 6:  FlexCAN in Transmission */
#define CAN_ESR1_IDLE                 (1 << 7)  /* Bit 7:  CAN bus is in IDLE state */
#define CAN_ESR1_RXWRN                (1 << 8)  /* Bit 8:  Rx Error Warning */
#define CAN_ESR1_TXWRN                (1 << 9)  /* Bit 9:  TX Error Warning */
#define CAN_ESR1_STFERR               (1 << 10) /* Bit 10: Stuffing Error */
#define CAN_ESR1_FRMERR               (1 << 11) /* Bit 11: Form Error */
#define CAN_ESR1_CRCERR               (1 << 12) /* Bit 12: Cyclic Redundancy Check Error */
#define CAN_ESR1_ACKERR               (1 << 13) /* Bit 13: Acknowledge Error */
#define CAN_ESR1_BIT0ERR              (1 << 14) /* Bit 14: Bit0 Error */
#define CAN_ESR1_BIT1ERR              (1 << 15) /* Bit 15: Bit1 Error */
#define CAN_ESR1_RWRNINT              (1 << 16) /* Bit 16: Rx Warning Interrupt Flag */
#define CAN_ESR1_TWRNINT              (1 << 17) /* Bit 17: Tx Warning Interrupt Flag */
#define CAN_ESR1_SYNCH                (1 << 18) /* Bit 18: CAN Synchronization Status */
                                                /* Bits 19-31: Reserved */

/* Interrupt Masks 2 Register */

#define CAN_IMASK2(n)                 (1 << (n)) /* Bit n: Buffer MBn Mask */

/* Interrupt Masks 1 Register */

#define CAN_IMASK1(n)                 (1 << (n)) /* Bit n: Buffer MBn Mask */

/* Interrupt Flags 2 Register */

#define CAN_IFLAG2(n)                 (1 << (n)) /* Bit n: Buffer MBn Interrupt */

/* Interrupt Flags 1 Register */

#define CAN_IFLAG1(n)                 (1 << (n)) /* Bit n: Buffer MBn Interrupt, n=0..4,8..31 */

/* Control 2 Register */

                                                /* Bits 0-10: Reserved */
#define CAN_CTRL2_EDFLTDIS            (1 << 11) /* Bit 11:  Edge Filter Disable */
#define CAN_CTRL2_ISOCANFDEN          (1 << 12) /* Bit 12:  ISO CAN FD Enable */
                                                /* Bit 13:  Reserved */
#define CAN_CTRL2_PREXCEN             (1 << 14) /* Bit 14:  Protocol Exception Enable */
#define CAN_CTRL2_TIMER_SRC           (1 << 15) /* Bit 15:  Timer Source */
#define CAN_CTRL2_EACEN               (1 << 16) /* Bit 16:  Entire Frame Arbitration Field Comparison Enable (Rx) */
#define CAN_CTRL2_RRS                 (1 << 17) /* Bit 17:  Remote Request Storing */
#define CAN_CTRL2_MRP                 (1 << 18) /* Bit 18:  Mailboxes Reception Priority */
#define CAN_CTRL2_TASD_SHIFT          (19)      /* Bits 19-23: Tx Arbitration Start Delay */
#define CAN_CTRL2_TASD_MASK           (31 << CAN_CTRL2_TASD_SHIFT)
#define CAN_CTRL2_RFFN_SHIFT          (24)      /* Bits 24-27: Number of Rx FIFO Filters */
#define CAN_CTRL2_RFFN_MASK           (15 << CAN_CTRL2_RFFN_SHIFT)
#  define CAN_CTRL2_RFFN_8MB          (0 << CAN_CTRL2_RFFN_SHIFT)
#  define CAN_CTRL2_RFFN_16MB         (1 << CAN_CTRL2_RFFN_SHIFT)
#  define CAN_CTRL2_RFFN_24MB         (2 << CAN_CTRL2_RFFN_SHIFT)
#  define CAN_CTRL2_RFFN_32MB         (3 << CAN_CTRL2_RFFN_SHIFT)
#  define CAN_CTRL2_RFFN_40MB         (4 << CAN_CTRL2_RFFN_SHIFT)
#  define CAN_CTRL2_RFFN_48MB         (5 << CAN_CTRL2_RFFN_SHIFT)
#  define CAN_CTRL2_RFFN_56MB         (6 << CAN_CTRL2_RFFN_SHIFT)
#  define CAN_CTRL2_RFFN_64MB         (7 << CAN_CTRL2_RFFN_SHIFT)
#  define CAN_CTRL2_RFFN_72MB         (8 << CAN_CTRL2_RFFN_SHIFT)
#  define CAN_CTRL2_RFFN_80MB         (9 << CAN_CTRL2_RFFN_SHIFT)
#  define CAN_CTRL2_RFFN_88MB         (10 << CAN_CTRL2_RFFN_SHIFT)
#  define CAN_CTRL2_RFFN_96MB         (11 << CAN_CTRL2_RFFN_SHIFT)
#  define CAN_CTRL2_RFFN_104MB        (12 << CAN_CTRL2_RFFN_SHIFT)
#  define CAN_CTRL2_RFFN_112MB        (13 << CAN_CTRL2_RFFN_SHIFT)
#  define CAN_CTRL2_RFFN_120MB        (14 << CAN_CTRL2_RFFN_SHIFT)
#  define CAN_CTRL2_RFFN_128MB        (15 << CAN_CTRL2_RFFN_SHIFT)
#define CAN_CTRL2_WRMFRZ              (1 << 28) /* Bit 28: Write-Access to Memory in Freeze mode */
                                                /* Bits 29-31: Reserved */

/* Error and Status 2 Register */

                                                /* Bits 0-12: Reserved */
#define CAN_ESR2_IMB                  (1 << 13) /* Bit 13: Inactive Mailbox */
#define CAN_ESR2_VPS                  (1 << 14) /* Bit 14: Valid Priority Status */
                                                /* Bit 15: Reserved */
#define CAN_ESR2_LPTM_SHIFT           (16)      /* Bits 16-22: Lowest Priority Tx Mailbox */
#define CAN_ESR2_LPTM_MASK            (0x7f << CAN_ESR2_LPTM_SHIFT)
                                                /* Bits 23-31: Reserved */

/* CRC Register */

#define CAN_CRCR_TXCRC_SHIFT          (0)       /* Bits 0-14: CRC Transmitted */
#define CAN_CRCR_TXCRC_MASK           (0x7fff << CAN_CRCR_TXCRC_SHIFT)
                                                /* Bit  15: Reserved */
#define CAN_CRCR_MBCRC_SHIFT          (16)      /* Bits 16-22: CRC Mailbox */
#define CAN_CRCR_MBCRC_MASK           (0x7f << CAN_CRCR_MBCRC_SHIFT)
                                                /* Bits 23-31: Reserved */

/* Rx FIFO Global Mask Register (32 Rx FIFO Global Mask Bits) */

/* Rx FIFO Information Register */

                                                /* Bits 9-31: Reserved */
#define CAN_RXFIR_IDHIT_SHIFT         (0)       /* Bits 0-8: Identifier Acceptance Filter Hit Indicator */
#define CAN_RXFIR_IDHIT_MASK          (0x1ff << CAN_RXFIR_IDHIT_SHIFT)

/* CAN Bit Timing register (CBT) */

/* CBT Bit Fields */
#define CAN_CBT_EPSEG2(x)             (((uint32_t)(((uint32_t)(x)) << 0))  & 0x1F)
#define CAN_CBT_EPSEG1(x)             (((uint32_t)(((uint32_t)(x)) << 5))  & 0x3E0)
#define CAN_CBT_EPROPSEG(x)           (((uint32_t)(((uint32_t)(x)) << 10)) & 0xFC00)
#define CAN_CBT_ERJW(x)               (((uint32_t)(((uint32_t)(x)) << 16)) & 0x1F0000)
#define CAN_CBT_EPRESDIV(x)           (((uint32_t)(((uint32_t)(x)) << 21)) & 0x7FE00000)
#define CAN_CBT_BTF                   (1 << 31) /* Bit 31: Bit Timing Format Enable */

/* CAN MB TX codes */
#define CAN_TXMB_INACTIVE             0x8       /* MB is not active. */
#define CAN_TXMB_ABORT                0x9       /* MB is aborted. */
#define CAN_TXMB_DATAORREMOTE         0xC       /* MB is a TX Data Frame(when MB RTR = 0) or */
                                                /* MB is a TX Remote Request Frame (when MB RTR = 1). */
#define CAN_TXMB_TANSWER              0xE       /* MB is a TX Response Request Frame from */
                                                /* an incoming Remote Request Frame. */
#define CAN_TXMB_NOTUSED              0xF       /* Not used.*/

/* CAN FD Control register (FDCTRL) */
#define CAN_FDCTRL_TDCVAL(x)          (((uint32_t)(((uint32_t)(x)) << 0))  & 0x3F)
#define CAN_FDCTRL_TDCOFF(x)          (((uint32_t)(((uint32_t)(x)) << 8))  & 0x1F00)
#define CAN_FDCTRL_TDCF               (1 << 14) /* Bit 14: TDC fail */
#define CAN_FDCTRL_TDCEN              (1 << 15) /* Bit 15: TDC enable */
#define CAN_FDCTRL_MBDSR0(x)          (((uint32_t)(((uint32_t)(x)) << 16)) & 0x30000)
#define CAN_FDCTRL_FDRATE             (1 << 31) /* Bit 31: FD rate */

/* FDCBT Bit Fields */
#define CAN_FDCBT_FPSEG2(x)           (((uint32_t)(((uint32_t)(x)) << 0))  & 0x7)
#define CAN_FDCBT_FPSEG1(x)           (((uint32_t)(((uint32_t)(x)) << 5))  & 0xE0)
#define CAN_FDCBT_FPROPSEG(x)         (((uint32_t)(((uint32_t)(x)) << 10)) & 0x7C00)
#define CAN_FDCBT_FRJW(x)             (((uint32_t)(((uint32_t)(x)) << 16)) & 0x70000)
#define CAN_FDCBT_FPRESDIV(x)         (((uint32_t)(((uint32_t)(x)) << 20)) & 0x3FF00000)

/* FDCRC Bit Fields */
#define CAN_FDCRC_FD_TXCRC(x)         (((uint32_t)(((uint32_t)(x)) << 0))  & 0x1FFFFF)
#define CAN_FDCRC_FD_MBCRC(x)         (((uint32_t)(((uint32_t)(x)) << 24)) & 0x7F000000)

/* Rn Individual Mask Registers */

#define CAN_RXIMR(n)                  (1 << (n)) /* Bit n: Individual Mask Bits */

/* Pretended Networking Control 1 register */
#define CAN_CTRL1_PN_

/* Pretended Networking Control 2 register */
#define CAN_CTRL2_PN_

/* Pretended Networking Wake Up Match register */
#define CAN_WU_MTC_

/* Pretended Networking ID Filter 1 register */
#define CAN_FLT_ID1_

/* Pretended Networking DLC Filter register */
#define CAN_FLT_DLC_

/* Pretended Networking Payload Low Filter 1 register */
#define CAN_PL1_LO_

/* Pretended Networking Payload High Filter 1 register */
#define CAN_PL1_HI_

/* Pretended Networking ID Filter 2 ID Mask register */
#define CAN_FLT_ID2_IDMASK_

/* Pretended Networking Payload Low Filter 2 Payload Low Mask register */
#define CAN_PL2_PLMASK_LO_

/* Pretended Networking Payload High Filter 2 low order bits /
 * Payload High Mask register
 */
#define CAN_PL2_PLMASK_HI_

/* Wake Up Message Buffer register for C/S */
#  define CAN_WMB_CS_

/* Wake Up Message Buffer Register for ID */
#  define CAN_WMB_ID_

/* Wake Up Message Buffer Register for Data 0-3 */
#  define CAN_WMB_D03_

/* Wake Up Message Buffer Register Data 4-7 */
#  define CAN_WMB_D47_

/* CAN FD Control register */
#define CAN_FDCTRL_

/* CAN FD Bit Timing register */
#define CAN_FDCBT_

/* CAN FD CRC register */
#define CAN_FDCRC_

#endif /* __ARCH_ARM_SRC_S32K1XX_HARDWARE_S32K1XX_FLEXCAN_H */
