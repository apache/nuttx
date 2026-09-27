/****************************************************************************
 * arch/arm/src/n32h7/hardware/n32h7_sdmmc.h
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

#ifndef __ARCH_ARM_SRC_N32H7_HARDWARE_N32H7_SDMMC_H
#define __ARCH_ARM_SRC_N32H7_HARDWARE_N32H7_SDMMC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* SDMMC Configuration Register Offsets (CFG) *******************************/

#define N32_SDMMC_CFG1_OFFSET        0x00 /* SDMMC Configuration 1 */
#define N32_SDMMC_CFG2_OFFSET        0x04 /* SDMMC Configuration 2 (Capabilities) */
#define N32_SDMMC_CFG3_OFFSET        0x08 /* SDMMC Configuration 3 */
#define N32_SDMMC_PV0CTRL_OFFSET     0x0C /* Preset Value 0 Control */
#define N32_SDMMC_PV1CTRL_OFFSET     0x10 /* Preset Value 1 Control */
#define N32_SDMMC_PV2CTRL_OFFSET     0x14 /* Preset Value 2 Control */
#define N32_SDMMC_PV3CTRL_OFFSET     0x18 /* Preset Value 3 Control */
#define N32_SDMMC_DLYCTRL_OFFSET     0x20 /* TX/RX Delay Control */

/* SDHOST Register Offsets **************************************************/

#define N32_SDHOST_DSADD_OFFSET      0x00 /* SDMA System Address / Parameter 2 */
#define N32_SDHOST_BLKCFG_OFFSET     0x04 /* Block Count & Size Configuration */
#define N32_SDHOST_CMDARG1_OFFSET    0x08 /* Command Argument 1 */
#define N32_SDHOST_TMODE_OFFSET      0x0C /* Transfer Mode */
#define N32_SDHOST_CMDRSP0_OFFSET    0x10 /* Command Response 0 (31:0) */
#define N32_SDHOST_CMDRSP1_OFFSET    0x14 /* Command Response 1 (63:32) */
#define N32_SDHOST_CMDRSP2_OFFSET    0x18 /* Command Response 2 (95:64) */
#define N32_SDHOST_CMDRSP3_OFFSET    0x1C /* Command Response 3 (127:96) */
#define N32_SDHOST_BUFDAT_OFFSET     0x20 /* Buffer Data Port (FIFO) */
#define N32_SDHOST_PRESTS_OFFSET     0x24 /* Present State */
#define N32_SDHOST_CTRL1_OFFSET      0x28 /* Control 1 */
#define N32_SDHOST_CTRL2_OFFSET      0x2C /* Control 2 */
#define N32_SDHOST_INTSTS_OFFSET     0x30 /* Interrupt Status */
#define N32_SDHOST_IE_OFFSET         0x34 /* Interrupt Enable */
#define N32_SDHOST_ISE_OFFSET        0x38 /* Interrupt Signal Enable */
#define N32_SDHOST_CTRLSTS_OFFSET    0x3C /* Control Status */
#define N32_SDHOST_CAP0STS_OFFSET    0x40 /* Capabilities 0 */
#define N32_SDHOST_CAP1STS_OFFSET    0x44 /* Capabilities 1 */
#define N32_SDHOST_STSFE_OFFSET      0x50 /* Status Force Event */
#define N32_SDHOST_ADMAESTS_OFFSET   0x54 /* ADMA Error Status */
#define N32_SDHOST_ASADD0_OFFSET     0x58 /* ADMA System Address 0 (low 32-bit) */
#define N32_SDHOST_ASADD1_OFFSET     0x5C /* ADMA System Address 1 (high 32-bit) */
#define N32_SDHOST_PV0STS_OFFSET     0x60 /* Preset Value 0 Status */
#define N32_SDHOST_PV1STS_OFFSET     0x64 /* Preset Value 1 Status */
#define N32_SDHOST_PV2STS_OFFSET     0x68 /* Preset Value 2 Status */
#define N32_SDHOST_PV3STS_OFFSET     0x6C /* Preset Value 3 Status */
#define N32_SDHOST_BOOTCTRL_OFFSET   0x70 /* Boot Timeout Control */

/* SDMMC CFG1 Register Bit Definitions **************************************/

#define N32_SDMMC_CFG1_WSGM               (1 << 0)   /* Bit 0: Wakeup Signal Generation Mode */
#define N32_SDMMC_CFG1_TCNT_SHIFT         (1)        /* Bits 1-6: Tuning Count */
#define N32_SDMMC_CFG1_TCNT_MASK          (0x3F << N32_SDMMC_CFG1_TCNT_SHIFT)
#define N32_SDMMC_CFG1_TCNT(n)            ((n) << N32_SDMMC_CFG1_TCNT_SHIFT)
#define N32_SDMMC_CFG1_TCLKU              (1 << 13)  /* Bit 13: Timeout Clock Unit (0=KHz, 1=MHz) */
#define N32_SDMMC_CFG1_BCLKF_SHIFT        (14)       /* Bits 14-21: Base Clock Frequency (MHz) */
#define N32_SDMMC_CFG1_BCLKF_MASK         (0xFF << N32_SDMMC_CFG1_BCLKF_SHIFT)
#define N32_SDMMC_CFG1_BCLKF(n)           ((n) << N32_SDMMC_CFG1_BCLKF_SHIFT)
#define N32_SDMMC_CFG1_MBL_SHIFT          (22)       /* Bits 22-23: Max Block Length */
#define N32_SDMMC_CFG1_MBL_MASK           (3 << N32_SDMMC_CFG1_MBL_SHIFT)
#define N32_SDMMC_CFG1_MBL_512            (0 << N32_SDMMC_CFG1_MBL_SHIFT) /* 512 bytes */
#define N32_SDMMC_CFG1_MBL_1024           (1 << N32_SDMMC_CFG1_MBL_SHIFT) /* 1024 bytes */
#define N32_SDMMC_CFG1_MBL_2048           (2 << N32_SDMMC_CFG1_MBL_SHIFT) /* 2048 bytes */

/* SDMMC CFG2 Register Bit Definitions (Capabilities) ***********************/

#define N32_SDMMC_CFG2_EMBUS             (1 << 0)   /* Bit 0: 8-bit Support for Embedded Device */
#define N32_SDMMC_CFG2_ADMA2             (1 << 1)   /* Bit 1: ADMA2 Support */
#define N32_SDMMC_CFG2_HS                (1 << 2)   /* Bit 2: High Speed Support */
#define N32_SDMMC_CFG2_SDMA              (1 << 3)   /* Bit 3: SDMA Support */
#define N32_SDMMC_CFG2_SRS               (1 << 4)   /* Bit 4: Suspend/Resume Support */
#define N32_SDMMC_CFG2_VS33              (1 << 5)   /* Bit 5: 3.3V Support */
#define N32_SDMMC_CFG2_ASYNCIINT         (1 << 9)   /* Bit 9: Asynchronous Interrupt Support */
#define N32_SDMMC_CFG2_STYP_SHIFT        (10)       /* Bits 10-11: Slot Type */
#define N32_SDMMC_CFG2_STYP_MASK         (3 << N32_SDMMC_CFG2_STYP_SHIFT)
#define N32_SDMMC_CFG2_STYP_REMOVABLE    (0 << N32_SDMMC_CFG2_STYP_SHIFT) /* Removable card slot */
#define N32_SDMMC_CFG2_STYP_EMBEDDED     (1 << N32_SDMMC_CFG2_STYP_SHIFT) /* Embedded slot */
#define N32_SDMMC_CFG2_SDR50             (1 << 12)                        /* Bit 12: SDR50 Support */
#define N32_SDMMC_CFG2_SDR104            (1 << 13)                        /* Bit 13: SDR104 Support */

/* SDMMC CFG3 Register Bit Definitions **************************************/

#define N32_SDMMC_CFG3_UTFSDR50          (1 << 4)   /* Bit 4: Use Tuning for SDR50 */

/* SDMMC Preset Value Control Registers Bit Definitions *********************/

/* PV0CTRL: Default Speed and Initialization */
#define N32_SDMMC_PV0CTRL_CLKFS_INIT_SHIFT  (0)     /* Bits 0-9: Init clock */
#define N32_SDMMC_PV0CTRL_CLKFS_INIT_MASK   (0x3FF << N32_SDMMC_PV0CTRL_CLKFS_INIT_SHIFT)
#define N32_SDMMC_PV0CTRL_CLKFS_INIT(n)     ((n) << N32_SDMMC_PV0CTRL_CLKFS_INIT_SHIFT)
#define N32_SDMMC_PV0CTRL_CLKFS_DS_SHIFT    (13)   /* Bits 13-22: Default speed clock */
#define N32_SDMMC_PV0CTRL_CLKFS_DS_MASK     (0x3FF << N32_SDMMC_PV0CTRL_CLKFS_DS_SHIFT)
#define N32_SDMMC_PV0CTRL_CLKFS_DS(n)       ((n) << N32_SDMMC_PV0CTRL_CLKFS_DS_SHIFT)

/* PV1CTRL: SDR12 and High Speed */
#define N32_SDMMC_PV1CTRL_CLKFS_HS_SHIFT    (0)     /* Bits 0-9: HS clock */
#define N32_SDMMC_PV1CTRL_CLKFS_HS_MASK     (0x3FF << N32_SDMMC_PV1CTRL_CLKFS_HS_SHIFT)
#define N32_SDMMC_PV1CTRL_CLKFS_HS(n)       ((n) << N32_SDMMC_PV1CTRL_CLKFS_HS_SHIFT)
#define N32_SDMMC_PV1CTRL_CLKFS_SDR12_SHIFT (13)   /* Bits 13-22: SDR12 clock */
#define N32_SDMMC_PV1CTRL_CLKFS_SDR12_MASK  (0x3FF << N32_SDMMC_PV1CTRL_CLKFS_SDR12_SHIFT)
#define N32_SDMMC_PV1CTRL_CLKFS_SDR12(n)    ((n) << N32_SDMMC_PV1CTRL_CLKFS_SDR12_SHIFT)

/* PV2CTRL: SDR50 and SDR25 */
#define N32_SDMMC_PV2CTRL_CLKFS_SDR25_SHIFT (0)     /* Bits 0-9: SDR25 clock */
#define N32_SDMMC_PV2CTRL_CLKFS_SDR25_MASK  (0x3FF << N32_SDMMC_PV2CTRL_CLKFS_SDR25_SHIFT)
#define N32_SDMMC_PV2CTRL_CLKFS_SDR25(n)    ((n) << N32_SDMMC_PV2CTRL_CLKFS_SDR25_SHIFT)
#define N32_SDMMC_PV2CTRL_CLKFS_SDR50_SHIFT (13)   /* Bits 13-22: SDR50 clock */
#define N32_SDMMC_PV2CTRL_CLKFS_SDR50_MASK  (0x3FF << N32_SDMMC_PV2CTRL_CLKFS_SDR50_SHIFT)
#define N32_SDMMC_PV2CTRL_CLKFS_SDR50(n)    ((n) << N32_SDMMC_PV2CTRL_CLKFS_SDR50_SHIFT)

/* PV3CTRL: DDR50 and SDR104 */
#define N32_SDMMC_PV3CTRL_CLKFS_DDR50_SHIFT (0)     /* Bits 0-9: DDR50 clock */
#define N32_SDMMC_PV3CTRL_CLKFS_DDR50_MASK  (0x3FF << N32_SDMMC_PV3CTRL_CLKFS_DDR50_SHIFT)
#define N32_SDMMC_PV3CTRL_CLKFS_DDR50(n)    ((n) << N32_SDMMC_PV3CTRL_CLKFS_DDR50_SHIFT)
#define N32_SDMMC_PV3CTRL_CLKFS_SDR104_SHIFT (13)  /* Bits 13-22: SDR104 clock */
#define N32_SDMMC_PV3CTRL_CLKFS_SDR104_MASK (0x3FF << N32_SDMMC_PV3CTRL_CLKFS_SDR104_SHIFT)
#define N32_SDMMC_PV3CTRL_CLKFS_SDR104(n)   ((n) << N32_SDMMC_PV3CTRL_CLKFS_SDR104_SHIFT)

/* SDMMC DLYCTRL Register Bit Definitions ***********************************/

#define N32_SDMMC_DLYCTRL_ITDS_SHIFT      (0)        /* Bits 0-4: Input Tap Delay Select */
#define N32_SDMMC_DLYCTRL_ITDS_MASK       (0x1F << N32_SDMMC_DLYCTRL_ITDS_SHIFT)
#define N32_SDMMC_DLYCTRL_ITDS(n)         ((n) << N32_SDMMC_DLYCTRL_ITDS_SHIFT)
#define N32_SDMMC_DLYCTRL_ITDE            (1 << 5)   /* Bit 5: Input Tap Delay Enable */
#define N32_SDMMC_DLYCTRL_ITCW            (1 << 6)   /* Bit 6: Input Tap Change Window */
#define N32_SDMMC_DLYCTRL_OTDS_SHIFT      (7)        /* Bits 7-10: Output Tap Delay Select */
#define N32_SDMMC_DLYCTRL_OTDS_MASK       (0xF << N32_SDMMC_DLYCTRL_OTDS_SHIFT)
#define N32_SDMMC_DLYCTRL_OTDS(n)         ((n) << N32_SDMMC_DLYCTRL_OTDS_SHIFT)
#define N32_SDMMC_DLYCTRL_OTDE            (1 << 11)  /* Bit 11: Output Tap Delay Enable */

/* SDHOST DSADD Register Bit Definitions ************************************/

#define N32_SDHOST_DSADD_ADD_SHIFT        (0)        /* Bits 0-31: Address or CMD23 parameter */
#define N32_SDHOST_DSADD_ADD_MASK         (0xFFFFFFFFUL << N32_SDHOST_DSADD_ADD_SHIFT)

/* SDHOST BLKCFG Register Bit Definitions ***********************************/

#define N32_SDHOST_BLKCFG_SIZE_SHIFT      (0)        /* Bits 0-11: Block Size */
#define N32_SDHOST_BLKCFG_SIZE_MASK       (0xFFF << N32_SDHOST_BLKCFG_SIZE_SHIFT)
#define N32_SDHOST_BLKCFG_SIZE(n)         ((n) << N32_SDHOST_BLKCFG_SIZE_SHIFT)
#define N32_SDHOST_BLKCFG_HDBS_SHIFT      (12)       /* Bits 12-14: SDMA Buffer Size */
#define N32_SDHOST_BLKCFG_HDBS_MASK       (7 << N32_SDHOST_BLKCFG_HDBS_SHIFT)
#define N32_SDHOST_BLKCFG_HDBS_4KB        (0 << N32_SDHOST_BLKCFG_HDBS_SHIFT)
#define N32_SDHOST_BLKCFG_HDBS_8KB        (1 << N32_SDHOST_BLKCFG_HDBS_SHIFT)
#define N32_SDHOST_BLKCFG_HDBS_16KB       (2 << N32_SDHOST_BLKCFG_HDBS_SHIFT)
#define N32_SDHOST_BLKCFG_HDBS_32KB       (3 << N32_SDHOST_BLKCFG_HDBS_SHIFT)
#define N32_SDHOST_BLKCFG_HDBS_64KB       (4 << N32_SDHOST_BLKCFG_HDBS_SHIFT)
#define N32_SDHOST_BLKCFG_HDBS_128KB      (5 << N32_SDHOST_BLKCFG_HDBS_SHIFT)
#define N32_SDHOST_BLKCFG_HDBS_256KB      (6 << N32_SDHOST_BLKCFG_HDBS_SHIFT)
#define N32_SDHOST_BLKCFG_HDBS_512KB      (7 << N32_SDHOST_BLKCFG_HDBS_SHIFT)
#define N32_SDHOST_BLKCFG_CNT_SHIFT       (16)       /* Bits 16-31: Block Count */
#define N32_SDHOST_BLKCFG_CNT_MASK        (0xFFFF << N32_SDHOST_BLKCFG_CNT_SHIFT)
#define N32_SDHOST_BLKCFG_CNT(n)          ((n) << N32_SDHOST_BLKCFG_CNT_SHIFT)

/* SDHOST CMDARG1 Register Bit Definitions **********************************/

#define N32_SDHOST_CMDARG1_ARG1_SHIFT     (0)        /* Bits 0-31: Command Argument */
#define N32_SDHOST_CMDARG1_ARG1_MASK      (0xFFFFFFFFUL << N32_SDHOST_CMDARG1_ARG1_SHIFT)

/* SDHOST TMODE Register Bit Definitions ************************************/

#define N32_SDHOST_TMODE_DMAE             (1 << 0)   /* Bit 0: DMA Enable */
#define N32_SDHOST_TMODE_BCNTE            (1 << 1)   /* Bit 1: Block Count Enable */
#define N32_SDHOST_TMODE_ACMDE_SHIFT      (2)        /* Bits 2-3: Auto CMD Enable */
#define N32_SDHOST_TMODE_ACMDE_MASK       (3 << N32_SDHOST_TMODE_ACMDE_SHIFT)
#define N32_SDHOST_TMODE_ACMDE_DISABLE    (0 << N32_SDHOST_TMODE_ACMDE_SHIFT)
#define N32_SDHOST_TMODE_ACMDE_CMD12      (1 << N32_SDHOST_TMODE_ACMDE_SHIFT)
#define N32_SDHOST_TMODE_ACMDE_CMD23      (2 << N32_SDHOST_TMODE_ACMDE_SHIFT)
#define N32_SDHOST_TMODE_DATDIR           (1 << 4)   /* Bit 4: Data Direction (0=Write, 1=Read) */
#define N32_SDHOST_TMODE_BLKSEL           (1 << 5)   /* Bit 5: Multi/Single Block Select (1=Multi) */
#define N32_SDHOST_TMODE_RTYPES_SHIFT     (16)       /* Bits 16-17: Response Type */
#define N32_SDHOST_TMODE_RTYPES_MASK      (3 << N32_SDHOST_TMODE_RTYPES_SHIFT)
#define N32_SDHOST_TMODE_RTYPES_NONE      (0 << N32_SDHOST_TMODE_RTYPES_SHIFT) /* No response */
#define N32_SDHOST_TMODE_RTYPES_136       (1 << N32_SDHOST_TMODE_RTYPES_SHIFT) /* Response length 136 bits */
#define N32_SDHOST_TMODE_RTYPES_48        (2 << N32_SDHOST_TMODE_RTYPES_SHIFT) /* Response length 48 bits */
#define N32_SDHOST_TMODE_RTYPES_48_BUSY   (3 << N32_SDHOST_TMODE_RTYPES_SHIFT) /* 48 bits with busy */
#define N32_SDHOST_TMODE_CRCCK            (1 << 19)                            /* Bit 19: Command CRC Check Enable */
#define N32_SDHOST_TMODE_CMDXCK           (1 << 20)                            /* Bit 20: Command Index Check Enable */
#define N32_SDHOST_TMODE_DPSEL            (1 << 21)                            /* Bit 21: Data Present Select */
#define N32_SDHOST_TMODE_TYPE_SHIFT       (22)                                 /* Bits 22-23: Command Type */
#define N32_SDHOST_TMODE_TYPE_MASK        (3 << N32_SDHOST_TMODE_TYPE_SHIFT)
#define N32_SDHOST_TMODE_TYPE_NORMAL      (0 << N32_SDHOST_TMODE_TYPE_SHIFT)   /* Normal */
#define N32_SDHOST_TMODE_TYPE_SUSPEND     (1 << N32_SDHOST_TMODE_TYPE_SHIFT)   /* Suspend */
#define N32_SDHOST_TMODE_TYPE_RESUME      (2 << N32_SDHOST_TMODE_TYPE_SHIFT)   /* Resume */
#define N32_SDHOST_TMODE_TYPE_ABORT       (3 << N32_SDHOST_TMODE_TYPE_SHIFT)   /* Abort */
#define N32_SDHOST_TMODE_CMDINDEX_SHIFT   (24)                                 /* Bits 24-29: Command Index */
#define N32_SDHOST_TMODE_CMDINDEX_MASK    (0x3F << N32_SDHOST_TMODE_CMDINDEX_SHIFT)
#define N32_SDHOST_TMODE_CMDINDEX(n)      ((n) << N32_SDHOST_TMODE_CMDINDEX_SHIFT)

/* SDHOST Response Registers ************************************************/

#define N32_SDHOST_CMDRSP0_RESP0_SHIFT    (0)
#define N32_SDHOST_CMDRSP0_RESP0_MASK     (0xFFFFFFFFUL << N32_SDHOST_CMDRSP0_RESP0_SHIFT)
#define N32_SDHOST_CMDRSP1_RESP1_SHIFT    (0)
#define N32_SDHOST_CMDRSP1_RESP1_MASK     (0xFFFFFFFFUL << N32_SDHOST_CMDRSP1_RESP1_SHIFT)
#define N32_SDHOST_CMDRSP2_RESP2_SHIFT    (0)
#define N32_SDHOST_CMDRSP2_RESP2_MASK     (0xFFFFFFFFUL << N32_SDHOST_CMDRSP2_RESP2_SHIFT)
#define N32_SDHOST_CMDRSP3_RESP3_SHIFT    (0)
#define N32_SDHOST_CMDRSP3_RESP3_MASK     (0xFFFFFFFFUL << N32_SDHOST_CMDRSP3_RESP3_SHIFT)

/* SDHOST BUFDAT Register Bit Definitions ***********************************/

#define N32_SDHOST_BUFDAT_DAT_SHIFT       (0)
#define N32_SDHOST_BUFDAT_DAT_MASK        (0xFFFFFFFFUL << N32_SDHOST_BUFDAT_DAT_SHIFT)

/* SDHOST PRESTS Register Bit Definitions ***********************************/

#define N32_SDHOST_PRESTS_CMDINHC         (1 << 0)   /* Bit 0: Command Inhibit (CMD) */
#define N32_SDHOST_PRESTS_CMDINHD         (1 << 1)   /* Bit 1: Command Inhibit (DAT) */
#define N32_SDHOST_PRESTS_DLACT           (1 << 2)   /* Bit 2: DAT Line Active */
#define N32_SDHOST_PRESTS_RETUNREQ        (1 << 3)   /* Bit 3: Re-Tuning Request */
#define N32_SDHOST_PRESTS_WTRANACT        (1 << 8)   /* Bit 8: Write Transfer Active */
#define N32_SDHOST_PRESTS_RTRANACT        (1 << 9)   /* Bit 9: Read Transfer Active */
#define N32_SDHOST_PRESTS_BUFW            (1 << 10)  /* Bit 10: Buffer Write Enable */
#define N32_SDHOST_PRESTS_BUFR            (1 << 11)  /* Bit 11: Buffer Read Enable */
#define N32_SDHOST_PRESTS_CINS            (1 << 16)  /* Bit 16: Card Inserted */
#define N32_SDHOST_PRESTS_CSTSL           (1 << 17)  /* Bit 17: Card State Stable */
#define N32_SDHOST_PRESTS_SDCDL           (1 << 18)  /* Bit 18: Card Detect Pin Level */
#define N32_SDHOST_PRESTS_SDWPL           (1 << 19)  /* Bit 19: Write Protect Switch Pin Level */
#define N32_SDHOST_PRESTS_DATLH0_SHIFT    (20)       /* Bits 20-23: DAT[3:0] Line Signal Level */
#define N32_SDHOST_PRESTS_DATLH0_MASK     (0xF << N32_SDHOST_PRESTS_DATLH0_SHIFT)
#define N32_SDHOST_PRESTS_CMDL            (1 << 24)  /* Bit 24: CMD Line Signal Level */
#define N32_SDHOST_PRESTS_DATLH1_SHIFT    (25)       /* Bits 25-28: DAT[7:4] Line Signal Level */
#define N32_SDHOST_PRESTS_DATLH1_MASK     (0xF << N32_SDHOST_PRESTS_DATLH1_SHIFT)

/* SDHOST CTRL1 Register Bit Definitions ************************************/

#define N32_SDHOST_CTRL1_LEDCTRL          (1 << 0)   /* Bit 0: LED Control */
#define N32_SDHOST_CTRL1_DTWIDTH          (1 << 1)   /* Bit 1: Data Transfer Width (0=1-bit, 1=4-bit) */
#define N32_SDHOST_CTRL1_HSEN             (1 << 2)   /* Bit 2: High Speed Enable */
#define N32_SDHOST_CTRL1_DMASEL_SHIFT     (3)        /* Bits 3-4: DMA Select */
#define N32_SDHOST_CTRL1_DMASEL_MASK      (3 << N32_SDHOST_CTRL1_DMASEL_SHIFT)
#define N32_SDHOST_CTRL1_DMASEL_SDMA      (0 << N32_SDHOST_CTRL1_DMASEL_SHIFT)
#define N32_SDHOST_CTRL1_DMASEL_ADMA2     (2 << N32_SDHOST_CTRL1_DMASEL_SHIFT)
#define N32_SDHOST_CTRL1_EDTWIDTH         (1 << 5)   /* Bit 5: Extended Data Transfer Width (8-bit) */
#define N32_SDHOST_CTRL1_CDTL             (1 << 6)   /* Bit 6: Card Detect Test Level */
#define N32_SDHOST_CTRL1_CDSD             (1 << 7)   /* Bit 7: Card Detect Signal Detection */
#define N32_SDHOST_CTRL1_SDPWR            (1 << 8)   /* Bit 8: SD Bus Power */
#define N32_SDHOST_CTRL1_SDBVSEL_SHIFT    (9)        /* Bits 9-11: SD Bus Voltage Select */
#define N32_SDHOST_CTRL1_SDBVSEL_MASK     (7 << N32_SDHOST_CTRL1_SDBVSEL_SHIFT)
#define N32_SDHOST_CTRL1_SDBVSEL_3V3      (7 << N32_SDHOST_CTRL1_SDBVSEL_SHIFT)
#define N32_SDHOST_CTRL1_HWRST            (1 << 12)  /* Bit 12: Hardware Reset (eMMC) */
#define N32_SDHOST_CTRL1_SABGREQ          (1 << 16)  /* Bit 16: Stop At Block Gap Request */
#define N32_SDHOST_CTRL1_CONTREQ          (1 << 17)  /* Bit 17: Continue Request */
#define N32_SDHOST_CTRL1_RWAITCTRL        (1 << 18)  /* Bit 18: Read Wait Control */
#define N32_SDHOST_CTRL1_INTATBG          (1 << 19)  /* Bit 19: Interrupt At Block Gap */
#define N32_SDHOST_CTRL1_SPIMODE          (1 << 20)  /* Bit 20: SPI Mode Enable */
#define N32_SDHOST_CTRL1_BOOTEN           (1 << 21)  /* Bit 21: Boot Enable */
#define N32_SDHOST_CTRL1_BOOTINALT        (1 << 22)  /* Bit 22: Boot in Alternative Mode */
#define N32_SDHOST_CTRL1_BOOTACKC         (1 << 23)  /* Bit 23: Boot Ack Check */
#define N32_SDHOST_CTRL1_INTWKU           (1 << 24)  /* Bit 24: Card Interrupt Wakeup Enable */
#define N32_SDHOST_CTRL1_INSTWKU          (1 << 25)  /* Bit 25: Insertion Wakeup Enable */
#define N32_SDHOST_CTRL1_RMVWKU           (1 << 26)  /* Bit 26: Removal Wakeup Enable */

/* SDHOST CTRL2 Register Bit Definitions ************************************/

#define N32_SDHOST_CTRL2_INCLKE           (1 << 0)   /* Bit 0: Internal Clock Enable */
#define N32_SDHOST_CTRL2_INCLKSTS         (1 << 1)   /* Bit 1: Internal Clock Stable */
#define N32_SDHOST_CTRL2_SDCLKE           (1 << 2)   /* Bit 2: SD Clock Enable */
#define N32_SDHOST_CTRL2_SDCLKSEL70_SHIFT (8)        /* Bits 8-15: SDCLK Frequency Select [7:0] */
#define N32_SDHOST_CTRL2_SDCLKSEL70_MASK  (0xFF << N32_SDHOST_CTRL2_SDCLKSEL70_SHIFT)
#define N32_SDHOST_CTRL2_SDCLKSEL98_SHIFT (6)        /* Bits 6-7: SDCLK Frequency Select [9:8] */
#define N32_SDHOST_CTRL2_SDCLKSEL98_MASK  (0x3 << N32_SDHOST_CTRL2_SDCLKSEL98_SHIFT)
#define N32_SDHOST_CTRL2_SDCLKSEL(n)      (((n & 0xFF) << N32_SDHOST_CTRL2_SDCLKSEL70_SHIFT) | (((n >> 8) & 0x3) << N32_SDHOST_CTRL2_SDCLKSEL98_SHIFT))
#define N32_SDHOST_CTRL2_DTCNT_SHIFT      (16)       /* Bits 16-19: Data Timeout Counter Value */
#define N32_SDHOST_CTRL2_DTCNT_MASK       (0xF << N32_SDHOST_CTRL2_DTCNT_SHIFT)
#define N32_SDHOST_CTRL2_DTCNT(n)         ((n) << N32_SDHOST_CTRL2_DTCNT_SHIFT)
#define N32_SDHOST_CTRL2_SWRSTALL         (1 << 24)  /* Bit 24: Software Reset All */
#define N32_SDHOST_CTRL2_SWRSTCMD         (1 << 25)  /* Bit 25: Software Reset CMD Line */
#define N32_SDHOST_CTRL2_SWRSTDAT         (1 << 26)  /* Bit 26: Software Reset DAT Line */

/* SDHOST INTSTS Register Bit Definitions (Interrupt Status) ****************/

#define N32_SDHOST_INTSTS_CMDC            (1 << 0)   /* Bit 0: Command Complete */
#define N32_SDHOST_INTSTS_TC              (1 << 1)   /* Bit 1: Transfer Complete */
#define N32_SDHOST_INTSTS_BLKGAPE         (1 << 2)   /* Bit 2: Block Gap Event */
#define N32_SDHOST_INTSTS_DMAINT          (1 << 3)   /* Bit 3: DMA Interrupt (SDMA) */
#define N32_SDHOST_INTSTS_BUFWRDY         (1 << 4)   /* Bit 4: Buffer Write Ready */
#define N32_SDHOST_INTSTS_BUFRRDY         (1 << 5)   /* Bit 5: Buffer Read Ready */
#define N32_SDHOST_INTSTS_CINS            (1 << 6)   /* Bit 6: Card Insertion */
#define N32_SDHOST_INTSTS_CRMV            (1 << 7)   /* Bit 7: Card Removal */
#define N32_SDHOST_INTSTS_CINT            (1 << 8)   /* Bit 8: Card Interrupt */
#define N32_SDHOST_INTSTS_RETUNE          (1 << 12)  /* Bit 12: Re-Tuning Event */
#define N32_SDHOST_INTSTS_BOOTACKR        (1 << 13)  /* Bit 13: Boot Ack Received */
#define N32_SDHOST_INTSTS_BOOTTER         (1 << 14)  /* Bit 14: Boot Terminate Interrupt */
#define N32_SDHOST_INTSTS_ALLERR          (1 << 15)  /* Bit 15: Error Interrupt (OR of all errors) */
#define N32_SDHOST_INTSTS_CTERR           (1 << 16)  /* Bit 16: Command Timeout Error */
#define N32_SDHOST_INTSTS_CCRCERR         (1 << 17)  /* Bit 17: Command CRC Error */
#define N32_SDHOST_INTSTS_CENDBERR        (1 << 18)  /* Bit 18: Command End Bit Error */
#define N32_SDHOST_INTSTS_CINXERR         (1 << 19)  /* Bit 19: Command Index Error */
#define N32_SDHOST_INTSTS_DTERR           (1 << 20)  /* Bit 20: Data Timeout Error */
#define N32_SDHOST_INTSTS_DCRERR          (1 << 21)  /* Bit 21: Data CRC Error */
#define N32_SDHOST_INTSTS_DENDERR         (1 << 22)  /* Bit 22: Data End Bit Error */
#define N32_SDHOST_INTSTS_ACMDERR         (1 << 24)  /* Bit 24: Auto CMD Error */
#define N32_SDHOST_INTSTS_ADMAERR         (1 << 25)  /* Bit 25: ADMA Error */
#define N32_SDHOST_INTSTS_TRGRERR         (1 << 28)  /* Bit 28: Target Response Error (DMA) */

/* SDHOST IE Register Bit Definitions (Interrupt Enable) ********************/

#define N32_SDHOST_IE_CMDCE              (1 << 0)   /* Bit 0: Command Complete Enable */
#define N32_SDHOST_IE_TCE                (1 << 1)   /* Bit 1: Transfer Complete Enable */
#define N32_SDHOST_IE_BLKGAPEE           (1 << 2)   /* Bit 2: Block Gap Event Enable */
#define N32_SDHOST_IE_DMAINTE            (1 << 3)   /* Bit 3: DMA Interrupt Enable */
#define N32_SDHOST_IE_BUFWRDYE           (1 << 4)   /* Bit 4: Buffer Write Ready Enable */
#define N32_SDHOST_IE_BUFRRDYE           (1 << 5)   /* Bit 5: Buffer Read Ready Enable */
#define N32_SDHOST_IE_CINSE              (1 << 6)   /* Bit 6: Card Insertion Enable */
#define N32_SDHOST_IE_CRMVE              (1 << 7)   /* Bit 7: Card Removal Enable */
#define N32_SDHOST_IE_CINTE              (1 << 8)   /* Bit 8: Card Interrupt Enable */
#define N32_SDHOST_IE_RTUNSE             (1 << 12)  /* Bit 12: Re-Tuning Event Enable */
#define N32_SDHOST_IE_BOOTACKRE          (1 << 13)  /* Bit 13: Boot Ack Received Enable */
#define N32_SDHOST_IE_BOOTTIE            (1 << 14)  /* Bit 14: Boot Terminate Enable */
#define N32_SDHOST_IE_CTEE               (1 << 16)  /* Bit 16: Command Timeout Error Enable */
#define N32_SDHOST_IE_CCRCEE             (1 << 17)  /* Bit 17: Command CRC Error Enable */
#define N32_SDHOST_IE_CENDBEE            (1 << 18)  /* Bit 18: Command End Bit Error Enable */
#define N32_SDHOST_IE_CINXEE             (1 << 19)  /* Bit 19: Command Index Error Enable */
#define N32_SDHOST_IE_DTEE               (1 << 20)  /* Bit 20: Data Timeout Error Enable */
#define N32_SDHOST_IE_DCRCEE             (1 << 21)  /* Bit 21: Data CRC Error Enable */
#define N32_SDHOST_IE_DENDEE             (1 << 22)  /* Bit 22: Data End Bit Error Enable */
#define N32_SDHOST_IE_ACMDE              (1 << 24)  /* Bit 24: Auto CMD Error Enable */
#define N32_SDHOST_IE_ADMAEE             (1 << 25)  /* Bit 25: ADMA Error Enable */
#define N32_SDHOST_IE_TRGREE             (1 << 28)  /* Bit 28: Target Response Error Enable */

/* SDHOST ISE Register Bit Definitions (Interrupt Signal Enable) ************/

#define N32_SDHOST_ISE_CMDCS             (1 << 0)   /* Bit 0: Command Complete Signal Enable */
#define N32_SDHOST_ISE_TCSE              (1 << 1)   /* Bit 1: Transfer Complete Signal Enable */
#define N32_SDHOST_ISE_BLKGAPESE         (1 << 2)   /* Bit 2: Block Gap Event Signal Enable */
#define N32_SDHOST_ISE_DMAINTSE          (1 << 3)   /* Bit 3: DMA Interrupt Signal Enable */
#define N32_SDHOST_ISE_BUFWRDYSE         (1 << 4)   /* Bit 4: Buffer Write Ready Signal Enable */
#define N32_SDHOST_ISE_BUFRRDYSE         (1 << 5)   /* Bit 5: Buffer Read Ready Signal Enable */
#define N32_SDHOST_ISE_CINSSE            (1 << 6)   /* Bit 6: Card Insertion Signal Enable */
#define N32_SDHOST_ISE_CRMVSE            (1 << 7)   /* Bit 7: Card Removal Signal Enable */
#define N32_SDHOST_ISE_CINTSE            (1 << 8)   /* Bit 8: Card Interrupt Signal Enable */
#define N32_SDHOST_ISE_RTUNSE            (1 << 12)  /* Bit 12: Re-Tuning Event Signal Enable */
#define N32_SDHOST_ISE_BOOTACKRSE        (1 << 13)  /* Bit 13: Boot Ack Received Signal Enable */
#define N32_SDHOST_ISE_BOOTTISE          (1 << 14)  /* Bit 14: Boot Terminate Signal Enable */
#define N32_SDHOST_ISE_CTESE             (1 << 16)  /* Bit 16: Command Timeout Error Signal Enable */
#define N32_SDHOST_ISE_CCRCESE           (1 << 17)  /* Bit 17: Command CRC Error Signal Enable */
#define N32_SDHOST_ISE_CENDBESE          (1 << 18)  /* Bit 18: Command End Bit Error Signal Enable */
#define N32_SDHOST_ISE_CINXESE           (1 << 19)  /* Bit 19: Command Index Error Signal Enable */
#define N32_SDHOST_ISE_DTESE             (1 << 20)  /* Bit 20: Data Timeout Error Signal Enable */
#define N32_SDHOST_ISE_DCRCESE           (1 << 21)  /* Bit 21: Data CRC Error Signal Enable */
#define N32_SDHOST_ISE_DENDESE           (1 << 22)  /* Bit 22: Data End Bit Error Signal Enable */
#define N32_SDHOST_ISE_ACMDESE           (1 << 24)  /* Bit 24: Auto CMD Error Signal Enable */
#define N32_SDHOST_ISE_ADMAESE           (1 << 25)  /* Bit 25: ADMA Error Signal Enable */
#define N32_SDHOST_ISE_TRGRESE           (1 << 28)  /* Bit 28: Target Response Error Signal Enable */

/* SDHOST CTRLSTS Register Bit Definitions **********************************/

#define N32_SDHOST_CTRLSTS_ACMD12NE      (1 << 0)   /* Bit 0: Auto CMD12 Not Executed */
#define N32_SDHOST_CTRLSTS_ACMDTE        (1 << 1)   /* Bit 1: Auto CMD Timeout Error */
#define N32_SDHOST_CTRLSTS_ACMDCRCE      (1 << 2)   /* Bit 2: Auto CMD CRC Error */
#define N32_SDHOST_CTRLSTS_ACMDEBE       (1 << 3)   /* Bit 3: Auto CMD End Bit Error */
#define N32_SDHOST_CTRLSTS_ACMDINXE      (1 << 4)   /* Bit 4: Auto CMD Index Error */
#define N32_SDHOST_CTRLSTS_ACMD12E       (1 << 7)   /* Bit 7: Auto CMD12 Not Issued */
#define N32_SDHOST_CTRLSTS_UHSMOD_SHIFT  (16)       /* Bits 16-18: UHS Mode Select */
#define N32_SDHOST_CTRLSTS_UHSMOD_MASK   (7 << N32_SDHOST_CTRLSTS_UHSMOD_SHIFT)
#define N32_SDHOST_CTRLSTS_UHSMOD_SDR12  (0 << N32_SDHOST_CTRLSTS_UHSMOD_SHIFT)
#define N32_SDHOST_CTRLSTS_UHSMOD_SDR25  (1 << N32_SDHOST_CTRLSTS_UHSMOD_SHIFT)
#define N32_SDHOST_CTRLSTS_UHSMOD_SDR50  (2 << N32_SDHOST_CTRLSTS_UHSMOD_SHIFT)
#define N32_SDHOST_CTRLSTS_UHSMOD_SDR104 (3 << N32_SDHOST_CTRLSTS_UHSMOD_SHIFT)
#define N32_SDHOST_CTRLSTS_UHSMOD_DDR50  (4 << N32_SDHOST_CTRLSTS_UHSMOD_SHIFT)
#define N32_SDHOST_CTRLSTS_V18SE         (1 << 19)  /* Bit 19: 1.8V Signaling Enable */
#define N32_SDHOST_CTRLSTS_ETUN          (1 << 22)  /* Bit 22: Execute Tuning */
#define N32_SDHOST_CTRLSTS_SCS           (1 << 23)  /* Bit 23: Sampling Clock Select */
#define N32_SDHOST_CTRLSTS_ASYNCIE       (1 << 30)  /* Bit 30: Asynchronous Interrupt Enable */
#define N32_SDHOST_CTRLSTS_PREVE         (1 << 31)  /* Bit 31: Preset Value Enable */

/* SDHOST CAP0STS Register Bit Definitions (Capabilities 0) *****************/

#define N32_SDHOST_CAP0STS_TCLKU         (1 << 7)   /* Bit 7: Timeout Clock Unit (0=KHz, 1=MHz) */
#define N32_SDHOST_CAP0STS_BCLKF_SHIFT   (8)        /* Bits 8-15: Base Clock Frequency (MHz) */
#define N32_SDHOST_CAP0STS_BCLKF_MASK    (0xFF << N32_SDHOST_CAP0STS_BCLKF_SHIFT)
#define N32_SDHOST_CAP0STS_MBL_SHIFT     (16)       /* Bits 16-17: Max Block Length */
#define N32_SDHOST_CAP0STS_MBL_MASK      (3 << N32_SDHOST_CAP0STS_MBL_SHIFT)
#define N32_SDHOST_CAP0STS_MBL_512       (0 << N32_SDHOST_CAP0STS_MBL_SHIFT)
#define N32_SDHOST_CAP0STS_MBL_1024      (1 << N32_SDHOST_CAP0STS_MBL_SHIFT)
#define N32_SDHOST_CAP0STS_MBL_2048      (2 << N32_SDHOST_CAP0STS_MBL_SHIFT)
#define N32_SDHOST_CAP0STS_EMBUS         (1 << 18)  /* Bit 18: 8-bit Support for Embedded Device */
#define N32_SDHOST_CAP0STS_ADMA2         (1 << 19)  /* Bit 19: ADMA2 Support */
#define N32_SDHOST_CAP0STS_HS            (1 << 21)  /* Bit 21: High Speed Support */
#define N32_SDHOST_CAP0STS_SDMA          (1 << 22)  /* Bit 22: SDMA Support */
#define N32_SDHOST_CAP0STS_SRS           (1 << 23)  /* Bit 23: Suspend/Resume Support */
#define N32_SDHOST_CAP0STS_VS33          (1 << 24)  /* Bit 24: 3.3V Support */
#define N32_SDHOST_CAP0STS_ASYNCIINT     (1 << 29)  /* Bit 29: Asynchronous Interrupt Support */
#define N32_SDHOST_CAP0STS_STYP_SHIFT    (30)       /* Bits 30-31: Slot Type */
#define N32_SDHOST_CAP0STS_STYP_MASK     (3 << N32_SDHOST_CAP0STS_STYP_SHIFT)
#define N32_SDHOST_CAP0STS_STYP_REMOVABLE (0 << N32_SDHOST_CAP0STS_STYP_SHIFT)
#define N32_SDHOST_CAP0STS_STYP_EMBEDDED (1 << N32_SDHOST_CAP0STS_STYP_SHIFT)

/* SDHOST CAP1STS Register Bit Definitions (Capabilities 1) *****************/

#define N32_SDHOST_CAP1STS_SDR50         (1 << 0)   /* Bit 0: SDR50 Support */
#define N32_SDHOST_CAP1STS_SDR104        (1 << 1)   /* Bit 1: SDR104 Support */
#define N32_SDHOST_CAP1STS_DDR50         (1 << 2)   /* Bit 2: DDR50 Support */
#define N32_SDHOST_CAP1STS_UTFSDR50      (1 << 13)  /* Bit 13: Use Tuning for SDR50 */
#define N32_SDHOST_CAP1STS_SPIMOD        (1 << 24)  /* Bit 24: SPI Mode Support */
#define N32_SDHOST_CAP1STS_SPIBMOD       (1 << 25)  /* Bit 25: SPI Block Mode Support */

/* SDHOST STSFE Register Bit Definitions (Force Event) **********************/

#define N32_SDHOST_STSFE_ACMD12NE        (1 << 0)   /* Bit 0: Force Auto CMD12 Not Executed */
#define N32_SDHOST_STSFE_ACMDTE          (1 << 1)   /* Bit 1: Force Auto CMD Timeout Error */
#define N32_SDHOST_STSFE_ACMDCRCE        (1 << 2)   /* Bit 2: Force Auto CMD CRC Error */
#define N32_SDHOST_STSFE_ACMDEBE         (1 << 3)   /* Bit 3: Force Auto CMD End Bit Error */
#define N32_SDHOST_STSFE_ACMDINXE        (1 << 4)   /* Bit 4: Force Auto CMD Index Error */
#define N32_SDHOST_STSFE_ACMD12E         (1 << 7)   /* Bit 7: Force Auto CMD12 Not Issued */
#define N32_SDHOST_STSFE_CTE             (1 << 16)  /* Bit 16: Force Command Timeout Error */
#define N32_SDHOST_STSFE_CCRCE           (1 << 17)  /* Bit 17: Force Command CRC Error */
#define N32_SDHOST_STSFE_CEBE            (1 << 18)  /* Bit 18: Force Command End Bit Error */
#define N32_SDHOST_STSFE_CINXE           (1 << 19)  /* Bit 19: Force Command Index Error */
#define N32_SDHOST_STSFE_DTE             (1 << 20)  /* Bit 20: Force Data Timeout Error */
#define N32_SDHOST_STSFE_DCRCE           (1 << 21)  /* Bit 21: Force Data CRC Error */
#define N32_SDHOST_STSFE_DEBE            (1 << 22)  /* Bit 22: Force Data End Bit Error */
#define N32_SDHOST_STSFE_ACMDE           (1 << 24)  /* Bit 24: Force Auto CMD Error */
#define N32_SDHOST_STSFE_ADMAE           (1 << 25)  /* Bit 25: Force ADMA Error */
#define N32_SDHOST_STSFE_TRGRE           (1 << 28)  /* Bit 28: Force Target Response Error */

/* SDHOST ADMAESTS Register Bit Definitions *********************************/

#define N32_SDHOST_ADMAESTS_ADMAE_SHIFT  (0)                                    /* Bits 0-1: ADMA Error State */
#define N32_SDHOST_ADMAESTS_ADMAE_MASK   (3 << N32_SDHOST_ADMAESTS_ADMAE_SHIFT)
#define N32_SDHOST_ADMAESTS_ST_STOP      (0 << N32_SDHOST_ADMAESTS_ADMAE_SHIFT) /* Stop DMA */
#define N32_SDHOST_ADMAESTS_ST_FDS       (1 << N32_SDHOST_ADMAESTS_ADMAE_SHIFT) /* Fetch Descriptor */
#define N32_SDHOST_ADMAESTS_ST_TFR       (3 << N32_SDHOST_ADMAESTS_ADMAE_SHIFT) /* Transfer Data */
#define N32_SDHOST_ADMAESTS_ADMALME      (1 << 2)                               /* Bit 2: ADMA Length Mismatch Error */

/* SDHOST ASADD0/1 Register Bit Definitions *********************************/

#define N32_SDHOST_ASADD0_ADD_SHIFT      (0)        /* Bits 0-31: Low 32-bit address */
#define N32_SDHOST_ASADD0_ADD_MASK       (0xFFFFFFFFUL << N32_SDHOST_ASADD0_ADD_SHIFT)
#define N32_SDHOST_ASADD1_ADD_SHIFT      (0)        /* Bits 0-31: High 32-bit address */
#define N32_SDHOST_ASADD1_ADD_MASK       (0xFFFFFFFFUL << N32_SDHOST_ASADD1_ADD_SHIFT)

/* SDHOST Preset Value Status Registers (same format as control) ************/

#define N32_SDHOST_PV0STS_CLKFS_INIT_SHIFT (0)
#define N32_SDHOST_PV0STS_CLKFS_INIT_MASK  (0x3FF << N32_SDHOST_PV0STS_CLKFS_INIT_SHIFT)
#define N32_SDHOST_PV0STS_CLKFS_DS_SHIFT   (16)
#define N32_SDHOST_PV0STS_CLKFS_DS_MASK    (0x3FF << N32_SDHOST_PV0STS_CLKFS_DS_SHIFT)

#define N32_SDHOST_PV1STS_CLKFS_HS_SHIFT   (0)
#define N32_SDHOST_PV1STS_CLKFS_HS_MASK    (0x3FF << N32_SDHOST_PV1STS_CLKFS_HS_SHIFT)
#define N32_SDHOST_PV1STS_CLKFS_SDR12_SHIFT (16)
#define N32_SDHOST_PV1STS_CLKFS_SDR12_MASK (0x3FF << N32_SDHOST_PV1STS_CLKFS_SDR12_SHIFT)

#define N32_SDHOST_PV2STS_CLKFS_SDR25_SHIFT (0)
#define N32_SDHOST_PV2STS_CLKFS_SDR25_MASK  (0x3FF << N32_SDHOST_PV2STS_CLKFS_SDR25_SHIFT)
#define N32_SDHOST_PV2STS_CLKFS_SDR50_SHIFT (16)
#define N32_SDHOST_PV2STS_CLKFS_SDR50_MASK  (0x3FF << N32_SDHOST_PV2STS_CLKFS_SDR50_SHIFT)

#define N32_SDHOST_PV3STS_CLKFS_DDR50_SHIFT (0)
#define N32_SDHOST_PV3STS_CLKFS_DDR50_MASK  (0x3FF << N32_SDHOST_PV3STS_CLKFS_DDR50_SHIFT)
#define N32_SDHOST_PV3STS_CLKFS_SDR104_SHIFT (16)
#define N32_SDHOST_PV3STS_CLKFS_SDR104_MASK (0x3FF << N32_SDHOST_PV3STS_CLKFS_SDR104_SHIFT)

/* SDHOST BOOTCTRL Register Bit Definitions *********************************/

#define N32_SDHOST_BOOTCTRL_CNT_SHIFT    (0)        /* Bits 0-31: Boot Data Timeout Counter Value */
#define N32_SDHOST_BOOTCTRL_CNT_MASK     (0xFFFFFFFFUL << N32_SDHOST_BOOTCTRL_CNT_SHIFT)
#define N32_SDHOST_BOOTCTRL_CNT(n)       ((n) << N32_SDHOST_BOOTCTRL_CNT_SHIFT)

/* ADMA2 Descriptor Structure (32-bit addressing) ***************************/

#define N32_ADMA_ACT_SHIFT               (4)                       /* Bits 4-5: Action */
#define N32_ADMA_ACT_MASK                (3 << N32_ADMA_ACT_SHIFT)
#define N32_ADMA_ACT_NOP                 (0 << N32_ADMA_ACT_SHIFT) /* No operation */
#define N32_ADMA_ACT_RSV                 (1 << N32_ADMA_ACT_SHIFT) /* Reserved */
#define N32_ADMA_ACT_TRAN                (2 << N32_ADMA_ACT_SHIFT) /* Transfer data */
#define N32_ADMA_ACT_LINK                (3 << N32_ADMA_ACT_SHIFT) /* Link to another table */
#define N32_ADMA_ENTRY_INT               (1 << 2)                  /* Bit 2: Generate DMA Interrupt */
#define N32_ADMA_ENTRY_END               (1 << 1)                  /* Bit 1: End of descriptor list */
#define N32_ADMA_ENTRY_VALID             (1 << 0)                  /* Bit 0: Descriptor valid */

struct n32_sdmmc_adma_desc_s
{
  uint16_t attr;       /* Attributes (ACT, INT, END, VALID) */
  uint16_t len;        /* Transfer length */
  uint32_t *addr;      /* 32-bit system address */
};

#endif /* __ARCH_ARM_SRC_N32H7_HARDWARE_N32H7_SDMMC_H */
