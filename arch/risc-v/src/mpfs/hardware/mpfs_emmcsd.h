/****************************************************************************
 * arch/risc-v/src/mpfs/hardware/mpfs_emmcsd.h
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

#ifndef __ARCH_RISCV_SRC_MPFS_HARDWARE_MPFS_EMMCSD_H
#define __ARCH_RISCV_SRC_MPFS_HARDWARE_MPFS_EMMCSD_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MPFS_EMMCSD_HRS00_OFFSET      0x00
#define MPFS_EMMCSD_HRS01_OFFSET      0x04
#define MPFS_EMMCSD_HRS02_OFFSET      0x08
#define MPFS_EMMCSD_HRS03_OFFSET      0x0c
#define MPFS_EMMCSD_HRS04_OFFSET      0x10
#define MPFS_EMMCSD_HRS06_OFFSET      0x18
#define MPFS_EMMCSD_HRS07_OFFSET      0x1c
#define MPFS_EMMCSD_HRS30_OFFSET      0x78
#define MPFS_EMMCSD_HRS31_OFFSET      0x7c
#define MPFS_EMMCSD_HRS32_OFFSET      0x80
#define MPFS_EMMCSD_HRS33_OFFSET      0x84
#define MPFS_EMMCSD_HRS34_OFFSET      0x88
#define MPFS_EMMCSD_HRS35_OFFSET      0x8c
#define MPFS_EMMCSD_HRS36_OFFSET      0x90
#define MPFS_EMMCSD_HRS37_OFFSET      0x94
#define MPFS_EMMCSD_HRS38_OFFSET      0x98

#define MPFS_EMMCSD_CRS63_OFFSET      0xfc

#define MPFS_EMMCSD_SRS00_OFFSET      0x200
#define MPFS_EMMCSD_SRS01_OFFSET      0x204
#define MPFS_EMMCSD_SRS02_OFFSET      0x208
#define MPFS_EMMCSD_SRS03_OFFSET      0x20c
#define MPFS_EMMCSD_SRS04_OFFSET      0x210
#define MPFS_EMMCSD_SRS05_OFFSET      0x214
#define MPFS_EMMCSD_SRS06_OFFSET      0x218
#define MPFS_EMMCSD_SRS07_OFFSET      0x21c
#define MPFS_EMMCSD_SRS08_OFFSET      0x220
#define MPFS_EMMCSD_SRS09_OFFSET      0x224
#define MPFS_EMMCSD_SRS10_OFFSET      0x228
#define MPFS_EMMCSD_SRS11_OFFSET      0x22c
#define MPFS_EMMCSD_SRS12_OFFSET      0x230
#define MPFS_EMMCSD_SRS13_OFFSET      0x234
#define MPFS_EMMCSD_SRS14_OFFSET      0x238
#define MPFS_EMMCSD_SRS15_OFFSET      0x23c
#define MPFS_EMMCSD_SRS16_OFFSET      0x240
#define MPFS_EMMCSD_SRS17_OFFSET      0x244
#define MPFS_EMMCSD_SRS18_OFFSET      0x248
#define MPFS_EMMCSD_SRS19_OFFSET      0x24c
#define MPFS_EMMCSD_SRS20_OFFSET      0x250
#define MPFS_EMMCSD_SRS21_OFFSET      0x254
#define MPFS_EMMCSD_SRS22_OFFSET      0x258
#define MPFS_EMMCSD_SRS23_OFFSET      0x25c

#define MPFS_EMMCSD_SRS24_OFFSET      0x260
#define MPFS_EMMCSD_SRS25_OFFSET      0x264
#define MPFS_EMMCSD_SRS26_OFFSET      0x268
#define MPFS_EMMCSD_SRS27_OFFSET      0x26c
#define MPFS_EMMCSD_SRS29_OFFSET      0x274

#define MPFS_EMMCSD_CQRS00_OFFSET     0x400
#define MPFS_EMMCSD_CQRS01_OFFSET     0x404
#define MPFS_EMMCSD_CQRS02_OFFSET     0x408
#define MPFS_EMMCSD_CQRS03_OFFSET     0x40c
#define MPFS_EMMCSD_CQRS04_OFFSET     0x410
#define MPFS_EMMCSD_CQRS05_OFFSET     0x414
#define MPFS_EMMCSD_CQRS06_OFFSET     0x418
#define MPFS_EMMCSD_CQRS07_OFFSET     0x41c
#define MPFS_EMMCSD_CQRS08_OFFSET     0x420
#define MPFS_EMMCSD_CQRS09_OFFSET     0x424
#define MPFS_EMMCSD_CQRS10_OFFSET     0x428
#define MPFS_EMMCSD_CQRS11_OFFSET     0x42c
#define MPFS_EMMCSD_CQRS12_OFFSET     0x430
#define MPFS_EMMCSD_CQRS13_OFFSET     0x434
#define MPFS_EMMCSD_CQRS14_OFFSET     0x438
#define MPFS_EMMCSD_CQRS16_OFFSET     0x440
#define MPFS_EMMCSD_CQRS17_OFFSET     0x444
#define MPFS_EMMCSD_CQRS18_OFFSET     0x448
#define MPFS_EMMCSD_CQRS20_OFFSET     0x450
#define MPFS_EMMCSD_CQRS21_OFFSET     0x454
#define MPFS_EMMCSD_CQRS22_OFFSET     0x458
#define MPFS_EMMCSD_CQRS23_OFFSET     0x45c

/* HRS00 register */

#define MPFS_EMMCSD_HRS00_HWINIT1       (0xff << 24)
#define MPFS_EMMCSD_HRS00_SAV           (0xff << 16)
#define MPFS_EMMCSD_HRS00_HWINIT0       (0x7fff << 1)
#define MPFS_EMMCSD_HRS00_SWR           (1 << 0)

/* HRS01 register */

#define MPFS_EMMCSD_HRS01_HWINIT0       (0xff << 24)
#define MPFS_EMMCSD_HRS01_DP            (0xffffff << 0)

/* HRS02 register */

#define MPFS_EMMCSD_HRS02_HWINIT1       (0x3fff << 18)
#define MPFS_EMMCSD_FRS02_OTN           (0x3 << 16)
#define MPFS_EMMCSD_HRS02_HWINIT0       (0xfff << 4)
#define MPFS_EMMCSD_HRS02_PBL           (0xf << 0)

/* HRS03 register */

#define MPFS_EMMCSD_HRS03_AER_IEBS      (1 << 19)
#define MPFS_EMMCSD_HRS03_AER_IEBD      (1 << 18)
#define MPFS_EMMCSD_HRS03_AER_IERS      (1 << 17)
#define MPFS_EMMCSD_HRS03_AER_IERD      (1 << 16)
#define MPFS_EMMCSD_HRS03_AER_SENBS     (1 << 11)
#define MPFS_EMMCSD_HRS03_AER_SENBD     (1 << 10)
#define MPFS_EMMCSD_HRS03_AER_SENRS     (1 << 9)
#define MPFS_EMMCSD_HRS03_AER_SENRD     (1 << 8)
#define MPFS_EMMCSD_HRS03_AER_BS        (1 << 3)
#define MPFS_EMMCSD_HRS03_AER_BD        (1 << 2)
#define MPFS_EMMCSD_HRS03_AER_RS        (1 << 1)
#define MPFS_EMMCSD_HRS03_AER_RD        (1 << 0)

/* HRS04 register */

#define MPFS_EMMCSD_HRS04_UIS_ACK       (1 << 26)
#define MPFS_EMMCSD_HRS04_UIS_RD        (1 << 25)
#define MPFS_EMMCSD_HRS04_UIS_WR        (1 << 24)
#define MPFS_EMMCSD_HRS04_UIS_RDATA     (0xff << 16)
#define MPFS_EMMCSD_HRS04_UIS_WDATA     (0xff << 8)
#define MPFS_EMMCSD_HRS04_UIS_ADDR      (0x3f << 0)

/* HRS06 register */

#define MPFS_EMMCSD_HRS06_ETR           (1 << 15)
#define MPFS_EMMCSD_HRS06_ETV           (0x3f << 8)
#define MPFS_EMMCSD_HRS06_EMM           (0x7 << 0)

/* HRS07 register */

#define MPFS_EMMCSD_HRS07_ODELAY_VAL    (0x1f << 16)
#define MPFS_EMMCSD_HRS07_IDELAY_VAL    (0x1f << 0)

/* HRS30 register */

#define MPFS_EMMCSD_HRS30_HS400ESSUP    (1 << 1)
#define MPFS_EMMCSD_HRS30_CQSUP         (1 << 0)

/* HRS31 register */

#define MPFS_EMMCSD_HRS31_HOSTCTRLVER   (0xfff << 16)
#define MPFS_EMMCSD_HRS31_HOSTFIXVER    (0xff << 0)

/* HRS32 register */

#define MPFS_EMMCSD_HRS32_LOAD          (1 << 31)
#define MPFS_EMMCSD_HRS32_ADDR          (0x7fff << 16)
#define MPFS_EMMCSD_HRS32_DATA          (0xffff << 0)

/* HRS33 register */

/* #define MPFS_EMMCSD_HRS33_STAT0 */

/* HRS34 register */

#define MPFS_EMMCSD_HRS33_STAT1         (0xff << 0)

/* HRS35 register */

#define MPFS_EMMCSD_HRS35_TFR           (1 << 31)
#define MPFS_EMMCSD_HRS35_TFV           (0x3f << 16)
#define MPFS_EMMCSD_HRS35_TVAL          (0x3f << 0)

/* HRS36 register */

#define MPFS_EMMCSD_HRS36_BOOT_EDE      (1 << 5)
#define MPFS_EMMCSD_HRS36_BOOT_EDC      (1 << 4)
#define MPFS_EMMCSD_HRS36_BOOT_EDT      (1 << 3)
#define MPFS_EMMCSD_HRS36_BOOT_EAI      (1 << 2)
#define MPFS_EMMCSD_HRS36_BOOT_EAT      (1 << 1)
#define MPFS_EMMCSD_HRS36_BOOT_ACT      (1 << 0)

/* HRS37 register */

#define MPFS_EMMCSD_HRS37_RGB_COEFF_IFM (0x3f << 0)

/* HRS38 register */

#define MPFS_EMMCSD_HRS38_RGB_COEFF     (0x0f << 0)

/* CRS63 register */

#define MPFS_EMMCSD_CRS63_HWINIT1       (0xff << 24)
#define MPFS_EMMCSD_CRS63_SVN           (0xff << 16)
#define MPFS_EMMCSD_CRS63_HWINIT0       (0xff << 8)
#define MPFS_EMMCSD_CRS63_ISES          (0xff << 0)

/* SRS01 register */

#define MPFS_EMMCSD_SRS01_BCCT          (0xff << 16)
#define MPFS_EMMCSD_SRS01_SDMABB        (0x7 << 12)
#define MPFS_EMMCSD_SRS01_TBS           (0xfff << 0)

#define MPFS_EMMCSD_SRS01_DMA_SZ_512KB  0x00007000

/* SRS03 register */

#define MPFS_EMMCSD_SRS03_CIDX          (0x3f << 24)
#define MPFS_EMMCSD_SRS03_CT            (0x03 << 22)
#define MPFS_EMMCSD_SRS03_DPS           (1 << 21)
#define MPFS_EMMCSD_SRS03_CICE          (1 << 20)
#define MPFS_EMMCSD_SRS03_CRCCE         (1 << 19)
#define MPFS_EMMCSD_SRS03_RTS           (0x3 << 16)
#define MPFS_EMMCSD_SRS03_RID           (1 << 8)
#define MPFS_EMMCSD_SRS03_RECE          (1 << 7)
#define MPFS_EMMCSD_SRS03_RECT          (1 << 6)
#define MPFS_EMMCSD_SRS03_MSBS          (1 << 5)
#define MPFS_EMMCSD_SRS03_DTDS          (1 << 4)
#define MPFS_EMMCSD_SRS03_ACE           (0x3 << 2)
#define MPFS_EMMCSD_SRS03_BCE           (1 << 1)
#define MPFS_EMMCSD_SRS03_DMAE          (1 << 0)

#define MPFS_EMMCSD_SRS03_NO_RESPONSE   (0 << 16)
#define MPFS_EMMCSD_SRS03_RESP_L136     (0x1 << 16)
#define MPFS_EMMCSD_SRS03_RESP_L48      (0x2 << 16)
#define MPFS_EMMCSD_SRS03_RESP_L48B     (0x3 << 16)

/* SRS09 register */

#define MPFS_EMMCSD_SRS09_CMDSL         (1 << 24)
#define MPFS_EMMCSD_SRS09_DATSL1        (0xf << 20)
#define MPFS_EMMCSD_SRS09_WPSL          (1 << 19)
#define MPFS_EMMCSD_SRS09_CDSL          (1 << 18)
#define MPFS_EMMCSD_SRS09_CSS           (1 << 17)
#define MPFS_EMMCSD_SRS09_CI            (1 << 16)
#define MPFS_EMMCSD_SRS09_BRE           (1 << 11)
#define MPFS_EMMCSD_SRS09_BWE           (1 << 10)
#define MPFS_EMMCSD_SRS09_RTA           (1 << 9)
#define MPFS_EMMCSD_SRS09_WTA           (1 << 8)
#define MPFS_EMMCSD_SRS09_DATSL2        (0xf << 4)
#define MPFS_EMMCSD_SRS09_DLA           (1 << 2)
#define MPFS_EMMCSD_SRS09_CIDAT         (1 << 1)
#define MPFS_EMMCSD_SRS09_CICMD         (1 << 0)

/* SRS10 register */

#define MPFS_EMMCSD_SRS10_WORM          (1 << 26)
#define MPFS_EMMCSD_SRS10_WOIS          (1 << 25)
#define MPFS_EMMCSD_SRS10_WOIQ          (1 << 24)
#define MPFS_EMMCSD_SRS10_IBG           (1 << 19)
#define MPFS_EMMCSD_SRS10_RWC           (1 << 18)
#define MPFS_EMMCSD_SRS10_CREQ          (1 << 17)
#define MPFS_EMMCSD_SRS10_SBGR          (1 << 16)
#define MPFS_EMMCSD_SRS10_BVS2          (0x7 << 13)
#define MPFS_EMMCSD_SRS10_BP2           (1 << 12)
#define MPFS_EMMCSD_SRS10_BVS           (0x7 << 9)
#define MPFS_EMMCSD_SRS10_BP            (1 << 8)
#define MPFS_EMMCSD_SRS10_CDSS          (1 << 7)
#define MPFS_EMMCSD_SRS10_CDTL          (1 << 6)
#define MPFS_EMMCSD_SRS10_EDTW          (1 << 5)
#define MPFS_EMMCSD_SRS10_DMASEL        (0x3 << 3)
#define MPFS_EMMCSD_SRS10_HSE           (1 << 2)
#define MPFS_EMMCSD_SRS10_DTW           (1 << 1)
#define MPFS_EMMCSD_SRS10_LEDC          (1 << 0)

/* SRS11 register */

#define MPFS_EMMCSD_SRS11_SRDAT         (1 << 26)
#define MPFS_EMMCSD_SRS11_SRCMD         (1 << 25)
#define MPFS_EMMCSD_SRS11_SRFA          (1 << 24)
#define MPFS_EMMCSD_SRS11_DTCV          (0xf << 16)
#define MPFS_EMMCSD_SRS11_SDCFSL        (0xff << 8)
#define MPFS_EMMCSD_SRS11_SDCFSH        (0x3 << 6)
#define MPFS_EMMCSD_SRS11_CLKGENSEL     (1 << 5) /* Not documented! */
#define MPFS_EMMCSD_SRS11_SDCE          (1 << 2)
#define MPFS_EMMCSD_SRS11_ICS           (1 << 1)
#define MPFS_EMMCSD_SRS11_ICE           (1 << 0)

/* SRS12 register */

#define MPFS_EMMCSD_SRS12_ERSP          (1 << 27)
#define MPFS_EMMCSD_SRS12_EADMA         (1 << 25)
#define MPFS_EMMCSD_SRS12_EAC           (1 << 24)
#define MPFS_EMMCSD_SRS12_ECL           (1 << 23)
#define MPFS_EMMCSD_SRS12_EDEB          (1 << 22)
#define MPFS_EMMCSD_SRS12_EDCRC         (1 << 21)
#define MPFS_EMMCSD_SRS12_EDT           (1 << 20)
#define MPFS_EMMCSD_SRS12_ECI           (1 << 19)
#define MPFS_EMMCSD_SRS12_ECEB          (1 << 18)
#define MPFS_EMMCSD_SRS12_ECCRC         (1 << 17)
#define MPFS_EMMCSD_SRS12_ECT           (1 << 16)
#define MPFS_EMMCSD_SRS12_EINT          (1 << 15)
#define MPFS_EMMCSD_SRS12_CQINT         (1 << 14)
#define MPFS_EMMCSD_SRS12_CINT          (1 << 8)
#define MPFS_EMMCSD_SRS12_CR            (1 << 7)
#define MPFS_EMMCSD_SRS12_CIN           (1 << 6)
#define MPFS_EMMCSD_SRS12_BRR           (1 << 5)
#define MPFS_EMMCSD_SRS12_BWR           (1 << 4)
#define MPFS_EMMCSD_SRS12_DMAINT        (1 << 3)
#define MPFS_EMMCSD_SRS12_BGE           (1 << 2)
#define MPFS_EMMCSD_SRS12_TC            (1 << 1)
#define MPFS_EMMCSD_SRS12_CC            (1 << 0)

#define MPFS_EMMCSD_SRS12_ESTAT_MASK    (0xFFFF8000u)
#define MPFS_EMMCSD_SRS12_STAT_CLEAR    0xFFFFFFFFu

/* SRS13 register */

#define MPFS_EMMCSD_SRS13_ERSP_SE       (1 << 27)
#define MPFS_EMMCSD_SRS13_TUNING_ERR_SE (1 << 26)
#define MPFS_EMMCSD_SRS13_EADMA_SE      (1 << 25)
#define MPFS_EMMCSD_SRS13_EAC_SE        (1 << 24)
#define MPFS_EMMCSD_SRS13_ECL_SE        (1 << 23)
#define MPFS_EMMCSD_SRS13_EDEB_SE       (1 << 22)
#define MPFS_EMMCSD_SRS13_EDCRC_SE      (1 << 21)
#define MPFS_EMMCSD_SRS13_EDT_SE        (1 << 20)
#define MPFS_EMMCSD_SRS13_ECI_SE        (1 << 19)
#define MPFS_EMMCSD_SRS13_ECEB_SE       (1 << 18)
#define MPFS_EMMCSD_SRS13_ECCRC_SE      (1 << 17)
#define MPFS_EMMCSD_SRS13_ECT_SE        (1 << 16)
#define MPFS_EMMCSD_SRS13_CQINT_SE      (1 << 14)
#define MPFS_EMMCSD_SRS13_RT_SE         (1 << 12) /* Undocumented */
#define MPFS_EMMCSD_SRS13_INT_ON_C_SE   (1 << 11) /* Undocumented */
#define MPFS_EMMCSD_SRS13_INT_ON_B_SE   (1 << 10) /* Undocumented */
#define MPFS_EMMCSD_SRS13_INT_ON_A_SE   (1 << 9)  /* Undocumented */
#define MPFS_EMMCSD_SRS13_CINT_SE       (1 << 8)
#define MPFS_EMMCSD_SRS13_CR_SE         (1 << 7)
#define MPFS_EMMCSD_SRS13_CIN_SE        (1 << 6)
#define MPFS_EMMCSD_SRS13_BRR_SE        (1 << 5)
#define MPFS_EMMCSD_SRS13_BWR_SE        (1 << 4)
#define MPFS_EMMCSD_SRS13_DMAINT_SE     (1 << 3)
#define MPFS_EMMCSD_SRS13_BGE_SE        (1 << 2)
#define MPFS_EMMCSD_SRS13_TC_SE         (1 << 1)
#define MPFS_EMMCSD_SRS13_CC_SE         (1 << 0)

#define MPFS_EMMCSD_SRS13_STATUS_EN (MPFS_EMMCSD_SRS13_ERSP_SE |       \
                                     MPFS_EMMCSD_SRS13_TUNING_ERR_SE | \
                                     MPFS_EMMCSD_SRS13_EADMA_SE |      \
                                     MPFS_EMMCSD_SRS13_EAC_SE |        \
                                     MPFS_EMMCSD_SRS13_ECL_SE |        \
                                     MPFS_EMMCSD_SRS13_EDEB_SE |       \
                                     MPFS_EMMCSD_SRS13_EDCRC_SE |      \
                                     MPFS_EMMCSD_SRS13_EDT_SE |        \
                                     MPFS_EMMCSD_SRS13_ECI_SE |        \
                                     MPFS_EMMCSD_SRS13_ECEB_SE |       \
                                     MPFS_EMMCSD_SRS13_ECCRC_SE |      \
                                     MPFS_EMMCSD_SRS13_ECT_SE |        \
                                     MPFS_EMMCSD_SRS13_CQINT_SE |      \
                                     MPFS_EMMCSD_SRS13_RT_SE |         \
                                     MPFS_EMMCSD_SRS13_INT_ON_C_SE |   \
                                     MPFS_EMMCSD_SRS13_INT_ON_B_SE |   \
                                     MPFS_EMMCSD_SRS13_INT_ON_A_SE |   \
                                     MPFS_EMMCSD_SRS13_CR_SE |         \
                                     MPFS_EMMCSD_SRS13_CIN_SE |        \
                                     MPFS_EMMCSD_SRS13_BRR_SE |        \
                                     MPFS_EMMCSD_SRS13_BWR_SE |        \
                                     MPFS_EMMCSD_SRS13_DMAINT_SE |     \
                                     MPFS_EMMCSD_SRS13_BGE_SE |        \
                                     MPFS_EMMCSD_SRS13_TC_SE |         \
                                     MPFS_EMMCSD_SRS13_CC_SE)

/* SRS14 register */

#define MPFS_EMMCSD_SRS14_ERSP_IE       (1 << 27)
#define MPFS_EMMCSD_SRS14_EADMA_IE      (1 << 25)
#define MPFS_EMMCSD_SRS14_EAC_IE        (1 << 24)
#define MPFS_EMMCSD_SRS14_ECL_IE        (1 << 23)
#define MPFS_EMMCSD_SRS14_EDEB_IE       (1 << 22)
#define MPFS_EMMCSD_SRS14_EDCRC_IE      (1 << 21)
#define MPFS_EMMCSD_SRS14_EDT_IE        (1 << 20)
#define MPFS_EMMCSD_SRS14_ECI_IE        (1 << 19)
#define MPFS_EMMCSD_SRS14_ECEB_IE       (1 << 18)
#define MPFS_EMMCSD_SRS14_ECCRC_IE      (1 << 17)
#define MPFS_EMMCSD_SRS14_ECT_IE        (1 << 16)
#define MPFS_EMMCSD_SRS14_CQINT_IE      (1 << 14)
#define MPFS_EMMCSD_SRS14_CINT_IE       (1 << 8)
#define MPFS_EMMCSD_SRS14_CR_IE         (1 << 7)
#define MPFS_EMMCSD_SRS14_CIN_IE        (1 << 6)
#define MPFS_EMMCSD_SRS14_BRR_IE        (1 << 5)
#define MPFS_EMMCSD_SRS14_BWR_IE        (1 << 4)
#define MPFS_EMMCSD_SRS14_DMAINT_IE     (1 << 3)
#define MPFS_EMMCSD_SRS14_BGE_IE        (1 << 2)
#define MPFS_EMMCSD_SRS14_TC_IE         (1 << 1)
#define MPFS_EMMCSD_SRS14_CC_IE         (1 << 0)

/* SRS15 register */

#define MPFS_EMMCSD_SRS15_PVE           (1 << 31)
#define MPFS_EMMCSD_SRS15_A64B          (1 << 29)
#define MPFS_EMMCSD_SRS15_HV4E          (1 << 28)
#define MPFS_EMMCSD_SRS15_SCS           (1 << 23)
#define MPFS_EMMCSD_SRS15_EXTNG         (1 << 22)
#define MPFS_EMMCSD_SRS15_DSS           (0x3 << 20)
#define MPFS_EMMCSD_SRS15_V18SE         (1 << 19)
#define MPFS_EMMCSD_SRS15_UMS           (0x7 << 16)
#define MPFS_EMMCSD_SRS15_CNIACE        (1 << 7)
#define MPFS_EMMCSD_SRS15_ACRE          (1 << 5)
#define MPFS_EMMCSD_SRS15_ACIE          (1 << 4)
#define MPFS_EMMCSD_SRS15_ACEBE         (1 << 3)
#define MPFS_EMMCSD_SRS15_ACCE          (1 << 2)
#define MPFS_EMMCSD_SRS15_ACTE          (1 << 1)
#define MPFS_EMMCSD_SRS15_ACNE          (1 << 0)

/* SRS16 register */

#define MPFS_EMMCSD_SRS16_SLT           (0x3 << 30)
#define MPFS_EMMCSD_SRS16_AIS           (1 << 29)
#define MPFS_EMMCSD_SRS16_A64S          (1 << 28)
#define MPFS_EMMCSD_SRS16_HWINIT1       (1 << 27)
#define MPFS_EMMCSD_SRS16_VS18          (1 << 26)
#define MPFS_EMMCSD_SRS16_VS30          (1 << 25)
#define MPFS_EMMCSD_SRS16_VS33          (1 << 24)
#define MPFS_EMMCSD_SRS16_SRS           (1 << 23)
#define MPFS_EMMCSD_SRS16_DMAS          (1 << 22)
#define MPFS_EMMCSD_SRS16_HSS           (1 << 21)
#define MPFS_EMMCSD_SRS16_ADMA1S        (1 << 20)
#define MPFS_EMMCSD_SRS16_ADMA2S        (1 << 19)
#define MPFS_EMMCSD_SRS16_EDS8          (1 << 18)
#define MPFS_EMMCSD_SRS16_MBL           (0x3 << 16)
#define MPFS_EMMCSD_SRS16_BCSDCLK       (0xff << 8)
#define MPFS_EMMCSD_SRS16_TCU           (1 << 7)
#define MPFS_EMMCSD_SRS16_HWINIT0       (1 << 6)
#define MPFS_EMMCSD_SRS16_TCF           (0x3f << 0)

/* SRS17 register */

#define MPFS_EMMCSD_SRS17_HWINIT3       (0x7 << 29)
#define MPFS_EMMCSD_SRS17_VDD2S         (1 << 28)
#define MPFS_EMMCSD_SRS17_HWINIT2       (0xf << 24)
#define MPFS_EMMCSD_SRS17_CLKMPR        (0xff << 16)
#define MPFS_EMMCSD_SRS17_RTNGM         (0x3 << 14)
#define MPFS_EMMCSD_SRS17_UTSM50        (1 << 13)
#define MPFS_EMMCSD_SRS17_HWINIT1       (1 << 12)
#define MPFS_EMMCSD_SRS17_RTNGCNT       (0xf << 8)
#define MPFS_EMMCSD_SRS17_HWINIT0       (1 << 7)
#define MPFS_EMMCSD_SRS17_DRVD          (1 << 6)
#define MPFS_EMMCSD_SRS17_DRVC          (1 << 5)
#define MPFS_EMMCSD_SRS17_DRVA          (1 << 4)
#define MPFS_EMMCSD_SRS17_UHSII         (1 << 3)
#define MPFS_EMMCSD_SRS17_DDR50         (1 << 2)
#define MPFS_EMMCSD_SRS17_SDR104        (1 << 1)
#define MPFS_EMMCSD_SRS17_SDR50         (1 << 0)

/* SRS18 register */

#define MPFS_EMMCSD_SRS18_HWINIT0       (0xff << 24)
#define MPFS_EMMCSD_SRS18_MC18          (0xff << 16)
#define MPFS_EMMCSD_SRS18_MC30          (0xff << 8)
#define MPFS_EMMCSD_SRS18_MC33          (0xff << 0)

/* SRS19 register */

#define MPFS_EMMCSD_SRS19_HWINIT0       (0xffffff << 8)
#define MPFS_EMMCSD_SRS19_MC18V2        (0xff << 0)

/* SRS20 register */

#define MPFS_EMMCSD_SRS20_ERESP_FE      (1 << 27)
#define MPFS_EMMCSD_SRS20_ETUNE_FE      (1 << 26)
#define MPFS_EMMCSD_SRS20_EADMA_FE      (1 << 25)
#define MPFS_EMMCSD_SRS20_EAC_FE        (1 << 24)
#define MPFS_EMMCSD_SRS20_ECL_FE        (1 << 23)
#define MPFS_EMMCSD_SRS20_EDEB_FE       (1 << 22)
#define MPFS_EMMCSD_SRS20_EDCRC_FE      (1 << 21)
#define MPFS_EMMCSD_SRS20_EDT_FE        (1 << 20)
#define MPFS_EMMCSD_SRS20_ECI_FE        (1 << 19)
#define MPFS_EMMCSD_SRS20_ECEB_FE       (1 << 18)
#define MPFS_EMMCSD_SRS20_ECCRC_FE      (1 << 17)
#define MPFS_EMMCSD_SRS20_ECT_FE        (1 << 16)
#define MPFS_EMMCSD_SRS20_CNIACE_FE     (1 << 7)
#define MPFS_EMMCSD_SRS20_ACIE_FE       (1 << 4)
#define MPFS_EMMCSD_SRS20_ACEBE_FE      (1 << 3)
#define MPFS_EMMCSD_SRS20_ACCE_FE       (1 << 2)
#define MPFS_EMMCSD_SRS20_ACTE_FE       (1 << 1)
#define MPFS_EMMCSD_SRS20_ACNE_FE       (1 << 0)

/* SRS21 register */

#define MPFS_EMMCSD_SRS21_EADMAL        (1 << 2)
#define MPFS_EMMCSD_SRS21_EADMAS        (0x3 << 0)

/* SRS24 register */

#define MPFS_EMMCSD_SRS24_DSSPV_31_30   (0x3 << 30)
#define MPFS_EMMCSD_SRS24_HWINIT1       (0xf << 26)
#define MPFS_EMMCSD_SRS24_SDCFSPV_25_16 (0x3ff << 16)
#define MPFS_EMMCSD_SRS24_HWINIT0       (0xffff << 0)

/* SRS25 register */

#define MPFS_EMMCSD_SRS25_DSSPV_31_30   (0x3 << 30)
#define MPFS_EMMCSD_SRS25_HWINIT1       (0xf << 26)
#define MPFS_EMMCSD_SRS25_SDCFSPV_25_16 (0x3ff << 16)
#define MPFS_EMMCSD_SRS25_DSSPV_15_14   (0x3 << 14)
#define MPFS_EMMCSD_SRS25_HWINIT0       (0xf << 10)
#define MPFS_EMMCSD_SRS25_SDCFSPV_09_00 (0x3ff << 0)

/* SRS26 register */

#define MPFS_EMMCSD_SRS26_DSSPV_31_30   (0x3 << 30)
#define MPFS_EMMCSD_SRS26_HWINIT1       (0xf << 26)
#define MPFS_EMMCSD_SRS26_SDCFSPV_25_16 (0x3ff << 16)
#define MPFS_EMMCSD_SRS26_DSSPV_15_14   (0x3 << 14)
#define MPFS_EMMCSD_SRS26_HWINIT0       (0x7 << 11)
#define MPFS_EMMCSD_SRS26_CGSPV_10      (1 << 10)
#define MPFS_EMMCSD_SRS26_SDCFSPV_09_00 (0x3ff << 0)

/* SRS27 register */

#define MPFS_EMMCSD_SRS27_DSSPV_31_30   (0x3 << 30)
#define MPFS_EMMCSD_SRS27_HWINIT1       (0xf << 26)
#define MPFS_EMMCSD_SRS27_SDCFSPV_25_16 (0x3ff << 16)
#define MPFS_EMMCSD_SRS27_DSSPV_15_14   (0x3 << 14)
#define MPFS_EMMCSD_SRS27_HWINIT0       (0xf << 10)
#define MPFS_EMMCSD_SRS27_SDCFSPV_09_00 (0x3ff << 0)

/* SRS29 register */

#define MPFS_EMMCSD_SRS29_HWINIT1       (0xffff << 16)
#define MPFS_EMMCSD_SRS29_DSSPV_15_14   (0x3 << 14)
#define MPFS_EMMCSD_SRS29_HWINIT0       (0xf << 10)
#define MPFS_EMMCSD_SRS29_SDCFSPV_09_00 (0x3ff << 0)

/* CQRS00 register */

#define MPFS_EMMCSD_CQRS00_CQVN1        (0xf << 8)
#define MPFS_EMMCSD_CQRS00_CQVN2        (0xf << 4)
#define MPFS_EMMCSD_CQRS00_CQVN3        (0xf << 0)

/* CQRS01 register */

#define MPFS_EMMCSD_CQRS01_ITCFMUL      (0xf << 12)
#define MPFS_EMMCSD_CQRS01_ITCFVAL      (0x3ff << 0)

/* CQRS02 register */

#define MPFS_EMMCSD_CQRS02_CQDCE        (1 << 12)
#define MPFS_EMMCSD_CQRS02_CQTDS        (1 << 8)
#define MPFS_EMMCSD_CQRS02_CQE          (1 << 0)

/* CQRS03 register */

#define MPFS_EMMCSD_CQRS03_CQCAT        (1 << 8)
#define MPFS_EMMCSD_CQRS03_CQHLT        (1 << 0)

/* CQRS04 register */

#define MPFS_EMMCSD_CQRS04_CQTCL        (1 << 3)
#define MPFS_EMMCSD_CQRS04_CQREDI       (1 << 2)
#define MPFS_EMMCSD_CQRS04_CQTCC        (1 << 1)
#define MPFS_EMMCSD_CQRS04_CQHAC        (1 << 0)

/* CQRS05 register */

#define MPFS_EMMCSD_CQRS05_CQTCLST      (1 << 3)
#define MPFS_EMMCSD_CQRS05_CQREDST      (1 << 2)
#define MPFS_EMMCSD_CQRS05_CQTCCST      (1 << 1)
#define MPFS_EMMCSD_CQRS05_CQHACST      (1 << 0)

/* CQRS06 register */

#define MPFS_EMMCSD_CQRS06_CQTCLSI      (1 << 3)
#define MPFS_EMMCSD_CQRS06_CQREDSI      (1 << 2)
#define MPFS_EMMCSD_CQRS06_CQTCCSI      (1 << 1)
#define MPFS_EMMCSD_CQRS06_CQHACSI      (1 << 0)

/* CQRS07 register */

#define MPFS_EMMCSD_CQRS07_CQICED       (1 << 31)
#define MPFS_EMMCSD_CQRS07_CQICSB       (1 << 20)
#define MPFS_EMMCSD_CQRS07_CQICCTR      (1 << 16)
#define MPFS_EMMCSD_CQRS07_CQICCTHWEN   (1 << 15)
#define MPFS_EMMCSD_CQRS07_CQICCTH      (0x1f << 8)
#define MPFS_EMMCSD_CQRS07_CQICTOVALEN  (1 << 7)
#define MPFS_EMMCSD_CQRS07_CQICTOVAL    (0x7f << 0)

/* CQRS10 register */

#define MPFS_EMMCSD_CQRS10_CQTD31       (1 << 31)
#define MPFS_EMMCSD_CQRS10_CQTD30       (1 << 30)
#define MPFS_EMMCSD_CQRS10_CQTD29       (1 << 29)
#define MPFS_EMMCSD_CQRS10_CQTD28       (1 << 28)
#define MPFS_EMMCSD_CQRS10_CQTD27       (1 << 27)
#define MPFS_EMMCSD_CQRS10_CQTD26       (1 << 26)
#define MPFS_EMMCSD_CQRS10_CQTD25       (1 << 25)
#define MPFS_EMMCSD_CQRS10_CQTD24       (1 << 24)
#define MPFS_EMMCSD_CQRS10_CQTD23       (1 << 23)
#define MPFS_EMMCSD_CQRS10_CQTD22       (1 << 22)
#define MPFS_EMMCSD_CQRS10_CQTD21       (1 << 21)
#define MPFS_EMMCSD_CQRS10_CQTD20       (1 << 20)
#define MPFS_EMMCSD_CQRS10_CQTD19       (1 << 19)
#define MPFS_EMMCSD_CQRS10_CQTD18       (1 << 18)
#define MPFS_EMMCSD_CQRS10_CQTD17       (1 << 17)
#define MPFS_EMMCSD_CQRS10_CQTD16       (1 << 16)
#define MPFS_EMMCSD_CQRS10_CQTD15       (1 << 15)
#define MPFS_EMMCSD_CQRS10_CQTD14       (1 << 14)
#define MPFS_EMMCSD_CQRS10_CQTD13       (1 << 13)
#define MPFS_EMMCSD_CQRS10_CQTD12       (1 << 12)
#define MPFS_EMMCSD_CQRS10_CQTD11       (1 << 11)
#define MPFS_EMMCSD_CQRS10_CQTD10       (1 << 10)
#define MPFS_EMMCSD_CQRS10_CQTD09       (1 << 9)
#define MPFS_EMMCSD_CQRS10_CQTD08       (1 << 8)
#define MPFS_EMMCSD_CQRS10_CQTD07       (1 << 7)
#define MPFS_EMMCSD_CQRS10_CQTD06       (1 << 6)
#define MPFS_EMMCSD_CQRS10_CQTD05       (1 << 5)
#define MPFS_EMMCSD_CQRS10_CQTD04       (1 << 4)
#define MPFS_EMMCSD_CQRS10_CQTD03       (1 << 3)
#define MPFS_EMMCSD_CQRS10_CQTD02       (1 << 2)
#define MPFS_EMMCSD_CQRS10_CQTD01       (1 << 1)
#define MPFS_EMMCSD_CQRS10_CQTD00       (1 << 0)

/* CQRS11 register */

#define MPFS_EMMCSD_CQRS11_CQTCN31      (1 << 31)
#define MPFS_EMMCSD_CQRS11_CQTCN30      (1 << 30)
#define MPFS_EMMCSD_CQRS11_CQTCN29      (1 << 29)
#define MPFS_EMMCSD_CQRS11_CQTCN28      (1 << 28)
#define MPFS_EMMCSD_CQRS11_CQTCN27      (1 << 27)
#define MPFS_EMMCSD_CQRS11_CQTCN26      (1 << 26)
#define MPFS_EMMCSD_CQRS11_CQTCN25      (1 << 25)
#define MPFS_EMMCSD_CQRS11_CQTCN24      (1 << 24)
#define MPFS_EMMCSD_CQRS11_CQTCN23      (1 << 23)
#define MPFS_EMMCSD_CQRS11_CQTCN22      (1 << 22)
#define MPFS_EMMCSD_CQRS11_CQTCN21      (1 << 21)
#define MPFS_EMMCSD_CQRS11_CQTCN20      (1 << 20)
#define MPFS_EMMCSD_CQRS11_CQTCN19      (1 << 19)
#define MPFS_EMMCSD_CQRS11_CQTCN18      (1 << 18)
#define MPFS_EMMCSD_CQRS11_CQTCN17      (1 << 17)
#define MPFS_EMMCSD_CQRS11_CQTCN16      (1 << 16)
#define MPFS_EMMCSD_CQRS11_CQTCN15      (1 << 15)
#define MPFS_EMMCSD_CQRS11_CQTCN14      (1 << 14)
#define MPFS_EMMCSD_CQRS11_CQTCN13      (1 << 13)
#define MPFS_EMMCSD_CQRS11_CQTCN12      (1 << 12)
#define MPFS_EMMCSD_CQRS11_CQTCN11      (1 << 11)
#define MPFS_EMMCSD_CQRS11_CQTCN10      (1 << 10)
#define MPFS_EMMCSD_CQRS11_CQTCN09      (1 << 9)
#define MPFS_EMMCSD_CQRS11_CQTCN08      (1 << 8)
#define MPFS_EMMCSD_CQRS11_CQTCN07      (1 << 7)
#define MPFS_EMMCSD_CQRS11_CQTCN06      (1 << 6)
#define MPFS_EMMCSD_CQRS11_CQTCN05      (1 << 5)
#define MPFS_EMMCSD_CQRS11_CQTCN04      (1 << 4)
#define MPFS_EMMCSD_CQRS11_CQTCN03      (1 << 3)
#define MPFS_EMMCSD_CQRS11_CQTCN02      (1 << 2)
#define MPFS_EMMCSD_CQRS11_CQTCN01      (1 << 1)
#define MPFS_EMMCSD_CQRS11_CQTCN00      (1 << 0)

/* CQRS13 register */

#define MPFS_EMMCSD_CQRS13_CQDPT31      (1 << 31)
#define MPFS_EMMCSD_CQRS13_CQDPT30      (1 << 30)
#define MPFS_EMMCSD_CQRS13_CQDPT29      (1 << 29)
#define MPFS_EMMCSD_CQRS13_CQDPT28      (1 << 28)
#define MPFS_EMMCSD_CQRS13_CQDPT27      (1 << 27)
#define MPFS_EMMCSD_CQRS13_CQDPT26      (1 << 26)
#define MPFS_EMMCSD_CQRS13_CQDPT25      (1 << 25)
#define MPFS_EMMCSD_CQRS13_CQDPT24      (1 << 24)
#define MPFS_EMMCSD_CQRS13_CQDPT23      (1 << 23)
#define MPFS_EMMCSD_CQRS13_CQDPT22      (1 << 22)
#define MPFS_EMMCSD_CQRS13_CQDPT21      (1 << 21)
#define MPFS_EMMCSD_CQRS13_CQDPT20      (1 << 20)
#define MPFS_EMMCSD_CQRS13_CQDPT19      (1 << 19)
#define MPFS_EMMCSD_CQRS13_CQDPT18      (1 << 18)
#define MPFS_EMMCSD_CQRS13_CQDPT17      (1 << 17)
#define MPFS_EMMCSD_CQRS13_CQDPT16      (1 << 16)
#define MPFS_EMMCSD_CQRS13_CQDPT15      (1 << 15)
#define MPFS_EMMCSD_CQRS13_CQDPT14      (1 << 14)
#define MPFS_EMMCSD_CQRS13_CQDPT13      (1 << 13)
#define MPFS_EMMCSD_CQRS13_CQDPT12      (1 << 12)
#define MPFS_EMMCSD_CQRS13_CQDPT11      (1 << 11)
#define MPFS_EMMCSD_CQRS13_CQDPT10      (1 << 10)
#define MPFS_EMMCSD_CQRS13_CQDPT09      (1 << 9)
#define MPFS_EMMCSD_CQRS13_CQDPT08      (1 << 8)
#define MPFS_EMMCSD_CQRS13_CQDPT07      (1 << 7)
#define MPFS_EMMCSD_CQRS13_CQDPT06      (1 << 6)
#define MPFS_EMMCSD_CQRS13_CQDPT05      (1 << 5)
#define MPFS_EMMCSD_CQRS13_CQDPT04      (1 << 4)
#define MPFS_EMMCSD_CQRS13_CQDPT03      (1 << 3)
#define MPFS_EMMCSD_CQRS13_CQDPT02      (1 << 2)
#define MPFS_EMMCSD_CQRS13_CQDPT01      (1 << 1)
#define MPFS_EMMCSD_CQRS13_CQDPT00      (1 << 0)

/* CQRS14 register */

#define MPFS_EMMCSD_CQRS14_CQTC31      (1 << 31)
#define MPFS_EMMCSD_CQRS14_CQTC30      (1 << 30)
#define MPFS_EMMCSD_CQRS14_CQTC29      (1 << 29)
#define MPFS_EMMCSD_CQRS14_CQTC28      (1 << 28)
#define MPFS_EMMCSD_CQRS14_CQTC27      (1 << 27)
#define MPFS_EMMCSD_CQRS14_CQTC26      (1 << 26)
#define MPFS_EMMCSD_CQRS14_CQTC25      (1 << 25)
#define MPFS_EMMCSD_CQRS14_CQTC24      (1 << 24)
#define MPFS_EMMCSD_CQRS14_CQTC23      (1 << 23)
#define MPFS_EMMCSD_CQRS14_CQTC22      (1 << 22)
#define MPFS_EMMCSD_CQRS14_CQTC21      (1 << 21)
#define MPFS_EMMCSD_CQRS14_CQTC20      (1 << 20)
#define MPFS_EMMCSD_CQRS14_CQTC19      (1 << 19)
#define MPFS_EMMCSD_CQRS14_CQTC18      (1 << 18)
#define MPFS_EMMCSD_CQRS14_CQTC17      (1 << 17)
#define MPFS_EMMCSD_CQRS14_CQTC16      (1 << 16)
#define MPFS_EMMCSD_CQRS14_CQTC15      (1 << 15)
#define MPFS_EMMCSD_CQRS14_CQTC14      (1 << 14)
#define MPFS_EMMCSD_CQRS14_CQTC13      (1 << 13)
#define MPFS_EMMCSD_CQRS14_CQTC12      (1 << 12)
#define MPFS_EMMCSD_CQRS14_CQTC11      (1 << 11)
#define MPFS_EMMCSD_CQRS14_CQTC10      (1 << 10)
#define MPFS_EMMCSD_CQRS14_CQTC09      (1 << 9)
#define MPFS_EMMCSD_CQRS14_CQTC08      (1 << 8)
#define MPFS_EMMCSD_CQRS14_CQTC07      (1 << 7)
#define MPFS_EMMCSD_CQRS14_CQTC06      (1 << 6)
#define MPFS_EMMCSD_CQRS14_CQTC05      (1 << 5)
#define MPFS_EMMCSD_CQRS14_CQTC04      (1 << 4)
#define MPFS_EMMCSD_CQRS14_CQTC03      (1 << 3)
#define MPFS_EMMCSD_CQRS14_CQTC02      (1 << 2)
#define MPFS_EMMCSD_CQRS14_CQTC01      (1 << 1)
#define MPFS_EMMCSD_CQRS14_CQTC00      (1 << 0)

/* CQRS16 register */

#define MPFS_EMMCSD_CQRS16_CQSSCBC     (0xf << 16)
#define MPFS_EMMCSD_CQRS16_CQSSCIT     (0xffff << 0)

/* CQRS21 register */

#define MPFS_EMMCSD_CQRS21_CQDTEFV     (1 << 31)
#define MPFS_EMMCSD_CQRS21_CQDTETID    (0x1f << 24)
#define MPFS_EMMCSD_CQRS21_CQDTECI     (0x3f << 16)
#define MPFS_EMMCSD_CQRS21_CQRMEFV     (1 << 15)
#define MPFS_EMMCSD_CQRS21_CQRMETID    (0x1f << 8)
#define MPFS_EMMCSD_CQRS21_CQRMECI     (0x3f << 0)

/* CQRS22 register */

#define MPFS_EMMCSD_CQRS22_CQLCRI      (0x3f << 0)

#endif /* __ARCH_RISCV_SRC_MPFS_HARDWARE_MPFS_EMMCSD_H */
