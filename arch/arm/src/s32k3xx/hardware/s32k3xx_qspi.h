/****************************************************************************
 * arch/arm/src/s32k3xx/hardware/s32k3xx_qspi.h
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

/* Copyright 2022 NXP */

#ifndef __ARCH_ARM_SRC_S32K3XX_HARDWARE_S32K3XX_QSPI_H
#define __ARCH_ARM_SRC_S32K3XX_HARDWARE_S32K3XX_QSPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <hardware/s32k3xx_memorymap.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* QSPI Register Offsets ****************************************************/

#define S32K3XX_QSPI_MCR_OFFSET           (0x0000) /* Module Configuration Register (MCR) */
#define S32K3XX_QSPI_IPCR_OFFSET          (0x0008) /* IP Configuration Register (IPCR) */
#define S32K3XX_QSPI_FLSHCR_OFFSET        (0x000c) /* Flash Memory Configuration Register (FLSHCR) */
#define S32K3XX_QSPI_BUF0CR_OFFSET        (0x0010) /* Buffer 0 Configuration Register (BUF0CR) */
#define S32K3XX_QSPI_BUF1CR_OFFSET        (0x0014) /* Buffer 1 Configuration Register (BUF1CR) */
#define S32K3XX_QSPI_BUF2CR_OFFSET        (0x0018) /* Buffer 2 Configuration Register (BUF2CR) */
#define S32K3XX_QSPI_BUF3CR_OFFSET        (0x001c) /* Buffer 3 Configuration Register (BUF3CR) */
#define S32K3XX_QSPI_BFGENCR_OFFSET       (0x0020) /* Buffer Generic Configuration Register (BFGENCR) */
#define S32K3XX_QSPI_SOCCR_OFFSET         (0x0024) /* SOC Configuration Register (SOCCR) */
#define S32K3XX_QSPI_BUF0IND_OFFSET       (0x0030) /* Buffer 0 Top Index Register (BUF0IND) */
#define S32K3XX_QSPI_BUF1IND_OFFSET       (0x0034) /* Buffer 1 Top Index Register (BUF1IND) */
#define S32K3XX_QSPI_BUF2IND_OFFSET       (0x0038) /* Buffer 2 Top Index Register (BUF2IND) */
#define S32K3XX_QSPI_DLLCRA_OFFSET        (0x0060) /* DLL Flash Memory A Configuration Register (DLLCRA) */
#define S32K3XX_QSPI_SFAR_OFFSET          (0x0100) /* Serial Flash Memory Address Register (SFAR) */
#define S32K3XX_QSPI_SMPR_OFFSET          (0x0108) /* Sampling Register (SMPR) */
#define S32K3XX_QSPI_RBSR_OFFSET          (0x010c) /* RX Buffer Status Register (RBSR) */
#define S32K3XX_QSPI_RBCT_OFFSET          (0x0110) /* RX Buffer Control Register (RBCT) */
#define S32K3XX_QSPI_DLSR_FA_OFFSET       (0x0134) /* Data Learning Status Flash Memory A Register (DLSR_FA) */
#define S32K3XX_QSPI_TBSR_OFFSET          (0x0150) /* TX Buffer Status Register (TBSR) */
#define S32K3XX_QSPI_TBDR_OFFSET          (0x0154) /* TX Buffer Data Register (TBDR) */
#define S32K3XX_QSPI_TBCT_OFFSET          (0x0158) /* TX Buffer Control Register (TBCT) */
#define S32K3XX_QSPI_SR_OFFSET            (0x015c) /* Status Register (SR) */
#define S32K3XX_QSPI_FR_OFFSET            (0x0160) /* Flag Register (FR) */
#define S32K3XX_QSPI_RSER_OFFSET          (0x0164) /* Interrupt and DMA Request Select and Enable Register (RSER) */
#define S32K3XX_QSPI_SPTRCLR_OFFSET       (0x016c) /* Sequence Pointer Clear Register (SPTRCLR) */
#define S32K3XX_QSPI_SFA1AD_OFFSET        (0x0180) /* Serial Flash Memory A1 Top Address Register (SFA1AD) */
#define S32K3XX_QSPI_SFA2AD_OFFSET        (0x0184) /* Serial Flash Memory A2 Top Address Register (SFA2AD) */
#define S32K3XX_QSPI_SFB1AD_OFFSET        (0x0188) /* Serial Flash Memory B1 Top Address Register (SFB1AD) */
#define S32K3XX_QSPI_SFB2AD_OFFSET        (0x018c) /* Serial Flash Memory B2 Top Address Register (SFB2AD) */

#define S32K3XX_QSPI_RBDR_OFFSET(n)       (0x0200 + ((n) << 2)) /* RX Buffer Data Register (RBDRn, n=0,...,63) */

#define S32K3XX_QSPI_LUTKEY_OFFSET        (0x0300) /* LUT Key Register (LUTKEY) */
#define S32K3XX_QSPI_LCKCR_OFFSET         (0x0304) /* LUT Lock Configuration Register (LKCR) */
#define S32K3XX_QSPI_LUT0_OFFSET          (0x0310) /* LUT Register 0 (LUT0) */
#define S32K3XX_QSPI_LUT1_OFFSET          (0x0314) /* LUT Register 1 (LUT1) */
#define S32K3XX_QSPI_LUT2_OFFSET          (0x0318) /* LUT Register 2 (LUT2) */
#define S32K3XX_QSPI_LUT3_OFFSET          (0x031c) /* LUT Register 3 (LUT3) */
#define S32K3XX_QSPI_LUT4_OFFSET          (0x0320) /* LUT Register 4 (LUT4) */
#define S32K3XX_QSPI_LUT5_OFFSET          (0x0324) /* LUT Register 5 (LUT5) */
#define S32K3XX_QSPI_LUT6_OFFSET          (0x0328) /* LUT Register 6 (LUT6) */
#define S32K3XX_QSPI_LUT7_OFFSET          (0x032c) /* LUT Register 7 (LUT7) */
#define S32K3XX_QSPI_LUT8_OFFSET          (0x0330) /* LUT Register 8 (LUT8) */
#define S32K3XX_QSPI_LUT9_OFFSET          (0x0334) /* LUT Register 9 (LUT9) */
#define S32K3XX_QSPI_LUT10_OFFSET         (0x0338) /* LUT Register 10 (LUT10) */
#define S32K3XX_QSPI_LUT11_OFFSET         (0x033c) /* LUT Register 11 (LUT11) */
#define S32K3XX_QSPI_LUT12_OFFSET         (0x0340) /* LUT Register 12 (LUT12) */
#define S32K3XX_QSPI_LUT13_OFFSET         (0x0344) /* LUT Register 13 (LUT13) */
#define S32K3XX_QSPI_LUT14_OFFSET         (0x0348) /* LUT Register 14 (LUT14) */
#define S32K3XX_QSPI_LUT15_OFFSET         (0x034c) /* LUT Register 15 (LUT15) */
#define S32K3XX_QSPI_LUT16_OFFSET         (0x0350) /* LUT Register 16 (LUT16) */
#define S32K3XX_QSPI_LUT17_OFFSET         (0x0354) /* LUT Register 17 (LUT17) */
#define S32K3XX_QSPI_LUT18_OFFSET         (0x0358) /* LUT Register 18 (LUT18) */
#define S32K3XX_QSPI_LUT19_OFFSET         (0x035c) /* LUT Register 19 (LUT19) */

/* QSPI Register Addresses **************************************************/

#define S32K3XX_QSPI_MCR                  (S32K3XX_QSPI_BASE + S32K3XX_QSPI_MCR_OFFSET)
#define S32K3XX_QSPI_IPCR                 (S32K3XX_QSPI_BASE + S32K3XX_QSPI_IPCR_OFFSET)
#define S32K3XX_QSPI_FLSHCR               (S32K3XX_QSPI_BASE + S32K3XX_QSPI_FLSHCR_OFFSET)
#define S32K3XX_QSPI_BUF0CR               (S32K3XX_QSPI_BASE + S32K3XX_QSPI_BUF0CR_OFFSET)
#define S32K3XX_QSPI_BUF1CR               (S32K3XX_QSPI_BASE + S32K3XX_QSPI_BUF1CR_OFFSET)
#define S32K3XX_QSPI_BUF2CR               (S32K3XX_QSPI_BASE + S32K3XX_QSPI_BUF2CR_OFFSET)
#define S32K3XX_QSPI_BUF3CR               (S32K3XX_QSPI_BASE + S32K3XX_QSPI_BUF3CR_OFFSET)
#define S32K3XX_QSPI_BFGENCR              (S32K3XX_QSPI_BASE + S32K3XX_QSPI_BFGENCR_OFFSET)
#define S32K3XX_QSPI_SOCCR                (S32K3XX_QSPI_BASE + S32K3XX_QSPI_SOCCR_OFFSET)
#define S32K3XX_QSPI_BUF0IND              (S32K3XX_QSPI_BASE + S32K3XX_QSPI_BUF0IND_OFFSET)
#define S32K3XX_QSPI_BUF1IND              (S32K3XX_QSPI_BASE + S32K3XX_QSPI_BUF1IND_OFFSET)
#define S32K3XX_QSPI_BUF2IND              (S32K3XX_QSPI_BASE + S32K3XX_QSPI_BUF2IND_OFFSET)
#define S32K3XX_QSPI_DLLCRA               (S32K3XX_QSPI_BASE + S32K3XX_QSPI_DLLCRA_OFFSET)
#define S32K3XX_QSPI_SFAR                 (S32K3XX_QSPI_BASE + S32K3XX_QSPI_SFAR_OFFSET)
#define S32K3XX_QSPI_SMPR                 (S32K3XX_QSPI_BASE + S32K3XX_QSPI_SMPR_OFFSET)
#define S32K3XX_QSPI_RBSR                 (S32K3XX_QSPI_BASE + S32K3XX_QSPI_RBSR_OFFSET)
#define S32K3XX_QSPI_RBCT                 (S32K3XX_QSPI_BASE + S32K3XX_QSPI_RBCT_OFFSET)
#define S32K3XX_QSPI_DLSR_FA              (S32K3XX_QSPI_BASE + S32K3XX_QSPI_DLSR_FA_OFFSET)
#define S32K3XX_QSPI_TBSR                 (S32K3XX_QSPI_BASE + S32K3XX_QSPI_TBSR_OFFSET)
#define S32K3XX_QSPI_TBDR                 (S32K3XX_QSPI_BASE + S32K3XX_QSPI_TBDR_OFFSET)
#define S32K3XX_QSPI_TBCT                 (S32K3XX_QSPI_BASE + S32K3XX_QSPI_TBCT_OFFSET)
#define S32K3XX_QSPI_SR                   (S32K3XX_QSPI_BASE + S32K3XX_QSPI_SR_OFFSET)
#define S32K3XX_QSPI_FR                   (S32K3XX_QSPI_BASE + S32K3XX_QSPI_FR_OFFSET)
#define S32K3XX_QSPI_RSER                 (S32K3XX_QSPI_BASE + S32K3XX_QSPI_RSER_OFFSET)
#define S32K3XX_QSPI_SPTRCLR              (S32K3XX_QSPI_BASE + S32K3XX_QSPI_SPTRCLR_OFFSET)
#define S32K3XX_QSPI_SFA1AD               (S32K3XX_QSPI_BASE + S32K3XX_QSPI_SFA1AD_OFFSET)
#define S32K3XX_QSPI_SFA2AD               (S32K3XX_QSPI_BASE + S32K3XX_QSPI_SFA2AD_OFFSET)
#define S32K3XX_QSPI_SFB1AD               (S32K3XX_QSPI_BASE + S32K3XX_QSPI_SFB1AD_OFFSET)
#define S32K3XX_QSPI_SFB2AD               (S32K3XX_QSPI_BASE + S32K3XX_QSPI_SFB2AD_OFFSET)
#define S32K3XX_QSPI_RBDR(n)              (S32K3XX_QSPI_BASE + S32K3XX_QSPI_RBDR_OFFSET(n))
#define S32K3XX_QSPI_LUTKEY               (S32K3XX_QSPI_BASE + S32K3XX_QSPI_LUTKEY_OFFSET)
#define S32K3XX_QSPI_LCKCR                (S32K3XX_QSPI_BASE + S32K3XX_QSPI_LCKCR_OFFSET)
#define S32K3XX_QSPI_LUT0                 (S32K3XX_QSPI_BASE + S32K3XX_QSPI_LUT0_OFFSET)
#define S32K3XX_QSPI_LUT1                 (S32K3XX_QSPI_BASE + S32K3XX_QSPI_LUT1_OFFSET)
#define S32K3XX_QSPI_LUT2                 (S32K3XX_QSPI_BASE + S32K3XX_QSPI_LUT2_OFFSET)
#define S32K3XX_QSPI_LUT3                 (S32K3XX_QSPI_BASE + S32K3XX_QSPI_LUT3_OFFSET)
#define S32K3XX_QSPI_LUT4                 (S32K3XX_QSPI_BASE + S32K3XX_QSPI_LUT4_OFFSET)
#define S32K3XX_QSPI_LUT5                 (S32K3XX_QSPI_BASE + S32K3XX_QSPI_LUT5_OFFSET)
#define S32K3XX_QSPI_LUT6                 (S32K3XX_QSPI_BASE + S32K3XX_QSPI_LUT6_OFFSET)
#define S32K3XX_QSPI_LUT7                 (S32K3XX_QSPI_BASE + S32K3XX_QSPI_LUT7_OFFSET)
#define S32K3XX_QSPI_LUT8                 (S32K3XX_QSPI_BASE + S32K3XX_QSPI_LUT8_OFFSET)
#define S32K3XX_QSPI_LUT9                 (S32K3XX_QSPI_BASE + S32K3XX_QSPI_LUT9_OFFSET)
#define S32K3XX_QSPI_LUT10                (S32K3XX_QSPI_BASE + S32K3XX_QSPI_LUT10_OFFSET)
#define S32K3XX_QSPI_LUT11                (S32K3XX_QSPI_BASE + S32K3XX_QSPI_LUT11_OFFSET)
#define S32K3XX_QSPI_LUT12                (S32K3XX_QSPI_BASE + S32K3XX_QSPI_LUT12_OFFSET)
#define S32K3XX_QSPI_LUT13                (S32K3XX_QSPI_BASE + S32K3XX_QSPI_LUT13_OFFSET)
#define S32K3XX_QSPI_LUT14                (S32K3XX_QSPI_BASE + S32K3XX_QSPI_LUT14_OFFSET)
#define S32K3XX_QSPI_LUT15                (S32K3XX_QSPI_BASE + S32K3XX_QSPI_LUT15_OFFSET)
#define S32K3XX_QSPI_LUT16                (S32K3XX_QSPI_BASE + S32K3XX_QSPI_LUT16_OFFSET)
#define S32K3XX_QSPI_LUT17                (S32K3XX_QSPI_BASE + S32K3XX_QSPI_LUT17_OFFSET)
#define S32K3XX_QSPI_LUT18                (S32K3XX_QSPI_BASE + S32K3XX_QSPI_LUT18_OFFSET)
#define S32K3XX_QSPI_LUT19                (S32K3XX_QSPI_BASE + S32K3XX_QSPI_LUT19_OFFSET)

#define S32K3XX_QSPI_LUT(n)               (S32K3XX_QSPI_BASE + S32K3XX_QSPI_LUT0_OFFSET + (n*4))
#define S32K3XX_QSPI_LUT_COUNT            20

/* QSPI Register Bitfield Definitions ***************************************/

/* Module Configuration Register (MCR) */

#define QSPI_MCR_SWRSTSD                  (1 << 0)  /* Bit 0: Software reset for serial flash memory domain (SWRSTSD) */
#define QSPI_MCR_SWRSTHD                  (1 << 1)  /* Bit 1: Software reset fo AHB domain (SWRSTHD) */
                                                    /* Bits 2-9: Reserved */
#define QSPI_MCR_CLR_RXF                  (1 << 10) /* Bit 10: Clear RX FIFO (CLR_RXF) */
#define QSPI_MCR_CLR_TXF                  (1 << 11) /* Bit 11: Clear TX FIFO/buffer (CLR_TXF) */
                                                    /* Bits 12-13: Reserved */
#define QSPI_MCR_MDIS                     (1 << 14) /* Bit 14: Module disable (MDIS) */
                                                    /* Bits 15-23: Reserved */
#define QSPI_MCR_DQS_FA_SEL_SHIFT         (24)      /* Bits 24-25: DQS clock for sampling read data at flash memory A (DQS_FA_SEL) */
#define QSPI_MCR_DQS_FA_SEL_MASK          (0x03 << QSPI_MCR_DQS_FA_SEL_SHIFT)
#define QSPI_MCR_DQS_FA_SEL_INTERNAL_DQS  ((0x00 << QSPI_MCR_DQS_FA_SEL_SHIFT) & QSPI_MCR_DQS_FA_SEL_MASK)
#define QSPI_MCR_DQS_FA_SEL_LOOPBACK      ((0x01 << QSPI_MCR_DQS_FA_SEL_SHIFT) & QSPI_MCR_DQS_FA_SEL_MASK)
#define QSPI_MCR_DQS_FA_SEL_LOOPBACK_DQS  ((0x02 << QSPI_MCR_DQS_FA_SEL_SHIFT) & QSPI_MCR_DQS_FA_SEL_MASK)
                                                    /* Bits 26-31: Reserved */

/* IP Configuration Register (IPCR) */

#define QSPICR_IDATSZ_SHIFT            (0)       /* Bits 0-15: IP data transfer size (IDATSZ) */
#define QSPICR_IDATSZ_MASK             (0xffff << QSPICR_IDATSZ_SHIFT)
#define QSPICR_IDATSZ(n)               (((n) << QSPICR_IDATSZ_SHIFT) & QSPICR_IDATSZ_MASK)
                                                    /* Bits 16-23: Reserved */
#define QSPICR_SEQID_SHIFT             (24)         /* Bits 24-27: Points to a sequence in the LUT (SEQID) */
#define QSPICR_SEQID_MASK              (0x0f << QSPICR_SEQID_SHIFT)
#define QSPICR_SEQID(n)                (((n) << QSPICR_SEQID_SHIFT) & QSPICR_SEQID_MASK)
                                                    /* Bits 28-31: Reserved */

/* Flash Memory Configuration Register (FLSHCR) */

#define QSPI_FLSHCR_TCSS_SHIFT            (0)       /* Bits 0-3: Serial flash memory CS setup time (TCSS) */
#define QSPI_FLSHCR_TCSS_MASK             (0x0f << QSPI_FLSHCR_TCSS_SHIFT)
#define QSPI_FLSHCR_TCSS(n)               (n & QSPI_FLSHCR_TCSS_MASK)
#define QSPI_FLSHCR_TCSH_SHIFT            (8)       /* Bits 8-11: Serial flash memory CS hold time (TCSH) */
#define QSPI_FLSHCR_TCSH_MASK             (0x0f << QSPI_FLSHCR_TCSH_SHIFT)
#define QSPI_FLSHCR_TCSH(n)               (((n) << QSPI_FLSHCR_TCSH_SHIFT) & QSPI_FLSHCR_TCSH_MASK)
                                                    /* Bits 12-31: Reserved */

/* Buffer n Configuration Register (BUFnCR) */

#define QSPI_BUFCR_MSTRID_SHIFT           (0)       /* Bits 0-3: Master ID (MSTRID) */
#define QSPI_BUFCR_MSTRID_MASK            (0x0f << QSPI_BUFCR_MSTRID_SHIFT)
#define QSPI_BUFCR_MSTRID(n)              (((n) << QSPI_BUFCR_MSTRID_SHIFT) & QSPI_BUFCR_MSTRID_MASK)
                                                    /* Bits 4-7: Reserved */
#define QSPI_BUFCR_ADATSZ_SHIFT           (8)       /* Bits 8-13: AHB data transfer size (ADATSZ) */
#define QSPI_BUFCR_ADATSZ_MASK            (0x3f << QSPI_BUFCR_ADATSZ_SHIFT)
#define QSPI_BUFCR_ADATSZ(n)              (((n) << QSPI_BUFCR_ADATSZ_SHIFT) & QSPI_BUFCR_ADATSZ_MASK)
                                                    /* Bits 14-31: Reserved */
#define QSPI_BUF3CR_ALLMST                (1 << 31) /* Bit 31: All master enable (ALLMST) */

/* Buffer Generic Configuration Register (BFGENCR) */

                                                    /* Bits 0-11: Reserved */
#define QSPI_BFGENCR_SEQID_SHIFT          (12)      /* Bits 12-15: Points to a sequence in the LUT (SEQID) */
#define QSPI_BFGENCR_SEQID_MASK           (0x0f << QSPI_BFGENCR_SEQID_SHIFT)
                                                    /* Bits 16-31: Reserved */

/* SOC Configuration Register (SOCCR) */

#define QSPI_SOCCR_SOCCFG_SHIFT           (0)       /* Bits 0-31: SOC configuration (SOCCFG) */
#define QSPI_SOCCR_SOCCFG_MASK            (0xffffffff << QSPI_SOCCR_SOCCFG_SHIFT)
#define QSPI_SOCCR_OBE_PULL_TMG_RLX       (1 << 0) /* Bit 0: obe_pull_timing_relax_b*/
#define QSPI_SOCCR_IBE                    (1 << 1) /* Bit 1: IBE */
#define QSPI_SOCCR_OBE                    (1 << 2) /* Bit 2: OBE */
#define QSPI_SOCCR_DSE                    (1 << 3) /* Bit 3: DSE */
#define QSPI_SOCCR_PUE                    (1 << 4) /* Bit 4: PUE */
#define QSPI_SOCCR_PUS                    (1 << 5) /* Bit 5: PUS */
#define QSPI_SOCCR_SRE                    (1 << 6) /* Bit 6: SRE */

/* Buffer n Top Index Register (BUFnIND) */

                                                    /* Bits 0-2: Reserved */
#define QSPI_BUFIND_TPINDX_SHIFT          (3)       /* Bits 3-8: Top index of buffer n (TPINDXn) */
#define QSPI_BUFIND_TPINDX_MASK           (0x3f << QSPI_BUFIND_TPINDX_SHIFT)
                                                    /* Bits 9-31: Reserved */

/* DLL Flash Memory A Configuration Register (DLLCRA) */

#define QSPI_DLLCRA_SLV_UPD               (1 << 0)  /* Bit 0: Slave update (SLV_UPD) */
#define QSPI_DLLCRA_SLV_DLL_BYPASS        (1 << 1)  /* Bit 1: Slave DLL bypass (SLV_DLL_BYPASS) */
#define QSPI_DLLCRA_SLV_EN                (1 << 2)  /* Bit 2: Slave enable (SLV_EN) */
                                                    /* Bits 3-7: Reserved */

#define QSPI_DLLCRA_SLV_DLY_COARSE_SHIFT  (8)       /* Bits 8-11: Delay elements in each delay tap (SLV_DLY_COARSE) */
#define QSPI_DLLCRA_SLV_DLY_COARSE_MASK   (0x0f << QSPI_DLLCRA_SLV_DLY_COARSE_SHIFT)
#define QSPI_DLLCRA_SLV_DLY_COARSE(n)     (((n) << QSPI_DLLCRA_SLV_DLY_COARSE_SHIFT) & QSPI_DLLCRA_SLV_DLY_COARSE_MASK)
#define QSPI_DLLCRA_SLV_DLY_OFFSET_SHIFT  (12)      /* Bits 12-14: T/16 offset delay elements in incoming DQS (SLV_DLY_OFFSET) */
#define QSPI_DLLCRA_SLV_DLY_OFFSET_MASK   (0x07 << QSPI_DLLCRA_SLV_DLY_OFFSET_SHIFT)
#define QSPI_DLLCRA_SLV_DLY(n)     (((n) << QSPI_DLLCRA_SLV_DLY_OFFSET_SHIFT) & QSPI_DLLCRA_SLV_DLY_OFFSET_MASK)
                                                    /* Bit 15: Reserved */
#define QSPI_DLLCRA_SLV_FINE_OFFSET_SHIFT (16)      /* Bits 16-19: Fine offset delay elements in incoming DQS (SLV_FINE_OFFSET) */
#define QSPI_DLLCRA_SLV_FINE_OFFSET_MASK  (0x0f << QSPI_DLLCRA_SLV_FINE_OFFSET_SHIFT)
                                                    /* Bits 20-29: Reserved */
#define QSPI_DLLCRA_FREQEN                (1 << 30) /* Bit 30: Frequency enable (FREQEN) */
                                                    /* Bit 31: Reserved */

/* Serial Flash Memory Address Register (SFAR) */

#define QSPI_SFAR_SFADR_SHIFT             (0)       /* Bits 0-31: Serial flash memory address (SFADR) */
#define QSPI_SFAR_SFADR_MASK              (0xffffffff << QSPI_SFAR_SFADR_SHIFT)

/* Sampling Register (SMPR) */

                                                    /* Bits 0-4: Reserved */
#define QSPI_SMPR_FSPHS                   (1 << 5)  /* Bit 5: Full speed phase selection for SDR instructions (FSPHS) */
#define QSPI_SMPR_FSDLY                   (1 << 6)  /* Bit 6: Full speed delay section for SDR instructions (FSDLY) */
                                                    /* Bits 7-23: Reserved */
#define QSPI_SMPR_DLLFSMPFA_SHIFT         (24)      /* Bits 24-26: Selects the nth tap provided by slave delay chain for flash memory A (DLLFSMPFA) */
#define QSPI_SMPR_DLLFSMPFA_MASK          (0x07 << QSPI_SMPR_DLLFSMPFA_SHIFT)
#define QSPI_SMPR_DLLFSMPFA(n)            (((n) << QSPI_SMPR_DLLFSMPFA_SHIFT) & QSPI_SMPR_DLLFSMPFA_MASK)
                                                    /* Bits 27-31: Reserved */

/* RX Buffer Status Register (RBSR) */

#define QSPI_RBSR_RDBFL_SHIFT             (0)       /* Bits 0-7: RX buffer fill level (RDBFL) */
#define QSPI_RBSR_RDBFL_MASK              (0xff << QSPI_RBSR_RDBFL_SHIFT)
                                                    /* Bits 8-15: Reserved */
#define QSPI_RBSR_RDCTR_SHIFT             (16)      /* Bits 16-31: Read counter (RDCTR) */
#define QSPI_RBSR_RDCTR_MASK              (0xffff << QSPI_RBSR_RDCTR_SHIFT)

/* RX Buffer Control Register (RBCT) */

#define QSPI_RBCT_WMRK_SHIFT              (0)       /* Bits 0-6: RX buffer watermark (WMRK) */
#define QSPI_RBCT_WMRK_MASK               (0x7f << QSPI_RBCT_WMRK_SHIFT)
#define QSPI_RBCT_WMRK(n)                 (((n) << QSPI_RBCT_WMRK_SHIFT) & QSPI_RBCT_WMRK_MASK)
                                                    /* Bit 7: Reserved */
#define QSPI_RBCT_RXBRD                   (1 << 8)  /* Bit 8: RX buffer readout (RXBRD) */
#  define QSPI_RBCT_RXBRD_AHB             (0 << 8)  /*        RX buffer content is read using the AHB bus registers */
#  define QSPI_RBCT_RXBRD_IP              (1 << 8)  /*        RX buffer content is read using the IP bus registers */
                                                    /* Bits 9-31: Reserved */

/* Data Learning Status Flash Memory A Register (DLSR_FA) */

#define QSPI_DLSR_FA_NEG_EDGE_SHIFT       (0)       /* Bits 0-7: DLP negative edge match signature for flash memory A (NEG_EDGE) */
#define QSPI_DLSR_FA_NEG_EDGE_MASK        (0xff << QSPI_DLSR_FA_NEG_EDGE_SHIFT)
#define QSPI_DLSR_FA_POS_EDGE_SHIFT       (8)       /* Bits 8-15: DLP positive edge match signature for flash memory A (POS_EDGE) */
#define QSPI_DLSR_FA_POS_EDGE_MASK        (0xff << QSPI_DLSR_FA_POS_EDGE_SHIFT)
                                                    /* Bits 16-30: Reserved */
#define QSPI_DLSR_FA_DLPFFA               (1 << 31) /* Bit 31: Data learning pattern fail (DLPFFA) */

/* TX Buffer Status Register (TBSR) */

#define QSPI_TBSR_TRBLF_SHIFT             (0)       /* Bits 0-5: TX buffer fill level (TRBFL) */
#define QSPI_TBSR_TRBLF_MASK              (0x3f << QSPI_TBSR_TRBLF_SHIFT)
                                                    /* Bits 6-15: Reserved */
#define QSPI_TBSR_TRCTR_SHIFT             (16)      /* Bits 16-31: Transmit counter (TRCTR) */
#define QSPI_TBSR_TRCTR_MASK              (0xffff << QSPI_TBSR_TRCTR_SHIFT)

/* TX Buffer Data Register (TBDR) */

#define QSPI_TBDR_TXDATA_SHIFT            (0)       /* Bits 0-31: TX data (TXDATA) */
#define QSPI_TBDR_TXDATA_MASK             (0xffffffff << QSPI_TBDR_TXDATA_SHIFT)

/* TX Buffer Control Register (TBCT) */

#define QSPI_TBCT_WMRK_SHIFT              (0)       /* Bits 0-4: Watermark for TX buffer (WMRK) */
#define QSPI_TBCT_WMRK_MASK               (0x1f << QSPI_TBCT_WMRK_SHIFT)
#define QSPI_TBCT_WMRK(n)                 (((n) << QSPI_TBCT_WMRK_SHIFT) & QSPI_TBCT_WMRK_MASK)
                                                    /* Bits 5-31: Reserved */

/* Status Register (SR) */

#define QSPI_SR_BUSY                      (1 << 0)  /* Bit 0: Module busy (BUSY) */
#define QSPI_SR_IP_ACC                    (1 << 1)  /* Bit 1: IP access (IP_ACC) */
#define QSPI_SR_AHB_ACC                   (1 << 2)  /* Bit 2: AHB read access (AHB_ACC) */
                                                    /* Bits 3-5: Reserved */
#define QSPI_SR_AHBTRN                    (1 << 6)  /* Bit 6: AHB access transaction pending (AHBTRN) */
#define QSPI_SR_AHB0NE                    (1 << 7)  /* Bit 7: AHB 0 buffer not empty (AHB0NE) */
#define QSPI_SR_AHB1NE                    (1 << 8)  /* Bit 8: AHB 1 buffer not empty (AHB1NE) */
#define QSPI_SR_AHB2NE                    (1 << 9)  /* Bit 9: AHB 2 buffer not empty (AHB2NE) */
#define QSPI_SR_AHB3NE                    (1 << 10) /* Bit 10: AHB 3 buffer not empty (AHB3NE) */
#define QSPI_SR_AHB0FUL                   (1 << 11) /* Bit 11: AHB 0 buffer full (AHB0FUL) */
#define QSPI_SR_AHB1FUL                   (1 << 12) /* Bit 12: AHB 1 buffer full (AHB1FUL) */
#define QSPI_SR_AHB2FUL                   (1 << 13) /* Bit 13: AHB 2 buffer full (AHB2FUL) */
#define QSPI_SR_AHB3FUL                   (1 << 14) /* Bit 14: AHB 3 buffer full (AHB3FUL) */
                                                    /* Bit 15: Reserved */
#define QSPI_SR_RXWE                      (1 << 16) /* Bit 16: RX buffer watermark exceeded (RXWE) */
                                                    /* Bits 17-18: Reserved */
#define QSPI_SR_RXFULL                    (1 << 19) /* Bit 19: RX buffer full (RXFULL) */
                                                    /* Bits 20-22: Reserved */
#define QSPI_SR_RXDMA                     (1 << 23) /* Bit 23: RX buffer DMA (RXDMA) */
#define QSPI_SR_TXNE                      (1 << 24) /* Bit 24: TX buffer not empty (TXNE) */
#define QSPI_SR_TXWA                      (1 << 25) /* Bit 25: TX buffer watermark available (TXWA) */
#define QSPI_SR_TXDMA                     (1 << 26) /* Bit 26: TX buffer DMA (TXDMA) */
#define QSPI_SR_TXFUL                     (1 << 27) /* Bit 27: TX buffer full (TXFULL) */
                                                    /* Bits 28-31: Reserved */

/* Flag Register (FR) */

#define QSPI_FR_TFF                       (1 << 0)  /* Bit 0: IP command transaction finished flag (TFF) */
                                                    /* Bits 1-5: Reserved */
#define QSPI_FR_IPIEF                     (1 << 6)  /* Bit 6: IP command trigger could not be executed error flag (IPIEF) */
#define QSPI_FR_IPAEF                     (1 << 7)  /* Bit 7: IP command trigger during AHB access error flag (IPAEF) */
                                                    /* Bits 8-11: Reserved */
#define QSPI_FR_ABOF                      (1 << 12) /* Bit 12: AHB buffer overflow flag (ABOF) */
#define QSPI_FR_AIBSEF                    (1 << 13) /* Bit 13: AHB illegal burst size error flag (AIBSEF) */
#define QSPI_FR_AITEF                     (1 << 14) /* Bit 14: AHB illegal transaction error flag (AITEF) */
                                                    /* Bit 15: Reserved */
#define QSPI_FR_RBDF                      (1 << 16) /* Bit 16: RX buffer drain flag (RBDF) */
#define QSPI_FR_RBOF                      (1 << 17) /* Bit 17: RX buffer overflow flag (RBOF) */
                                                    /* Bits 18-22: Reserved */
#define QSPI_FR_ILLINE                    (1 << 23) /* Bit 23: Illegal instruction error flag (ILLINE) */
                                                    /* Bits 24-25: Reserved */
#define QSPI_FR_TBUF                      (1 << 26) /* Bit 26: TX buffer underrun flag (TBUF) */
#define QSPI_FR_TBFF                      (1 << 27) /* Bit 27: TX buffer fill flag (TBFF) */
                                                    /* Bits 28-31: Reserved */

/* Interrupt and DMA Request Select and Enable Register (RSER) */

#define QSPI_RSER_TFIE                    (1 << 0)  /* Bit 0: Transaction finished interrupt enable flag (TFIE) */
                                                    /* Bits 1-5: Reserved */
#define QSPI_RSER_IPIEIE                  (1 << 6)  /* Bit 6: IP command trigger during IP access error interrupt enable flag (IPIEIE) */
#define QSPI_RSER_IPAEIE                  (1 << 7)  /* Bit 7: IP command trigger during AHB read access error interrupt enable flag (IPAEIE) */
                                                    /* Bits 8-11: Reserved */
#define QSPI_RSER_ABOIE                   (1 << 12) /* Bit 12: AHB buffer overflow interrupt enable flag (ABOIE) */
#define QSPI_RSER_AIBSIE                  (1 << 13) /* Bit 13: AHB illegal burst size interrupt enable flag (AIBSIE) */
#define QSPI_RSER_AITIE                   (1 << 14) /* Bit 14: AHB illegal transaction interrupt enable flag (AITIE) */
                                                    /* Bit 15: Reserved */
#define QSPI_RSER_RBDIE                   (1 << 16) /* Bit 16: RX buffer drain interrupt enable (RBDIE) */
#define QSPI_RSER_RBOIE                   (1 << 17) /* Bit 17: RX buffer overflow interrupt enable (RBOIE) */
                                                    /* Bits 18-20: Reserved */
#define QSPI_RSER_RBDDE                   (1 << 21) /* Bit 21: RX buffer drain DMA enable (RBDDE) */
                                                    /* Bit 22: Reserved */
#define QSPI_RSER_ILLINIE                 (1 << 23) /* Bit 23: Illegal instruction error interrupt enable (ILLINIE) */
                                                    /* Bit 24: Reserved */
#define QSPI_RSER_TBFDE                   (1 << 25) /* Bit 25: TX buffer fill DMA enable (TBFDE) */
#define QSPI_RSER_TBUIE                   (1 << 26) /* Bit 26: TX buffer underrun interrupt enable flag (TBUIE) */
#define QSPI_RSER_TBFIE                   (1 << 27) /* Bit 27: TX buffer fill interrupt enable flag (TBFIE) */
                                                    /* Bits 28-31: Reserved */

/* Sequence Pointer Clear Register (SPTRCLR) */

#define QSPI_SPTRCLR_BFPTRC               (1 << 0)  /* Bit 0: Buffer pointer clear (BFPTRC) */
                                                    /* Bits 1-7: Reserved */
#define QSPI_SPTRCLR_IPPTRC               (1 << 8)  /* Bit 8: IP pointer clear (IPPTRC) */
                                                    /* Bits 9-31: Reserved */

/* Serial Flash Memory An/Bn Top Address Register (SFAnAD/SFBnAD) */

                                                    /* Bits 0-9: Reserved */
#define QSPI_SFAD_TPAD_SHIFT              (10)      /* Bits 10-31: Top address for serial flash memory An/Bn (TPADAn/TPADBn) */
#define QSPI_SFAD_TPAD_MASK               (0x3fffff << QSPI_SFAD_TPAD_SHIFT)
#define QSPI_SFAD_TPAD(n)                 (((n) << QSPI_SFAD_TPAD_SHIFT) & QSPI_SFAD_TPAD_MASK)

/* RX Buffer Data Register (RBDRn, n=0,...,63) */

#define QSPI_RBDR_RXDATA_SHIFT            (0)       /* Bits 0-31: RX data (RXDATA) */
#define QSPI_RBDR_RXDATA_MASK             (0xffffffff << QSPI_RBDR_RXDATA_SHIFT)

/* LUT Key Register (LUTKEY) */

#define QSPI_LUTKEY_KEY_SHIFT             (0)       /* Bits 0-31: Key to lock or unlock the LUT (KEY) */
#define QSPI_LUTKEY_KEY_MASK              (0xffffffff << QSPI_LUTKEY_KEY_SHIFT)
#define QSPI_LUTKEY_KEY                   (0x5AF05AF0ul)

/* LUT Lock Configuration Register (LKCR) */

#define QSPI_LKCR_LOCK                    (1 << 0)  /* Bit 0: Lock LUT (LOCK) */
#define QSPI_LKCR_UNLOCK                  (1 << 1)  /* Bit 1: Unlock LUT (UNLOCK) */
                                                    /* Bits 2-31: Reserved */

/* LUT Register (LUTn) */

#define QSPI_LUT_OPRND0_SHIFT             (0)       /* Bits 0-7: Operand for INSTR0 (OPRND0) */
#define QSPI_LUT_OPRND0_MASK              (0xff << QSPI_LUT_OPRND0_SHIFT)
#define QSPI_LUT_OPRND0(n)                (((n) << QSPI_LUT_OPRND0_SHIFT) & QSPI_LUT_OPRND0_MASK)
#define QSPI_LUT_PAD0_SHIFT               (8)       /* Bits 8-9: Pad information for INSTR0 (PAD0) */
#define QSPI_LUT_PAD0_MASK                (0x03 << QSPI_LUT_PAD0_SHIFT)
#  define QSPI_LUT_PAD0_1                 (0x00 << QSPI_LUT_PAD0_SHIFT) /* 1 Pad */ 
#  define QSPI_LUT_PAD0_2                 (0x01 << QSPI_LUT_PAD0_SHIFT) /* 2 Pad */ 
#  define QSPI_LUT_PAD0_4                 (0x02 << QSPI_LUT_PAD0_SHIFT) /* 4 Pad */ 

#define QSPI_LUT_INSTR0_SHIFT             (10)      /* Bits 10-15: Instruction 0 (INSTR0) */
#define QSPI_LUT_INSTR0_MASK              (0x3f << QSPI_LUT_INSTR0_SHIFT)
#define QSPI_LUT_INSTR0(n)                (((n) << QSPI_LUT_INSTR0_SHIFT) & QSPI_LUT_INSTR0_MASK)

#define QSPI_LUT_OPRND1_SHIFT             (16)       /* Bits 16-23: Operand for INSTR1 (OPRND1) */
#define QSPI_LUT_OPRND1_MASK              (0xff << QSPI_LUT_OPRND1_SHIFT)
#define QSPI_LUT_OPRND1(n)                (((n) << QSPI_LUT_OPRND1_SHIFT) & QSPI_LUT_OPRND1_MASK)
#define QSPI_LUT_PAD1_SHIFT               (24)       /* Bits 24-25: Pad information for INSTR1 (PAD1) */
#define QSPI_LUT_PAD1_MASK                (0x03 << QSPI_LUT_PAD1_SHIFT)
#  define QSPI_LUT_PAD1_1                 (0x00 << QSPI_LUT_PAD1_SHIFT) /* 1 Pad */ 
#  define QSPI_LUT_PAD1_2                 (0x01 << QSPI_LUT_PAD1_SHIFT) /* 2 Pad */ 
#  define QSPI_LUT_PAD1_4                 (0x02 << QSPI_LUT_PAD1_SHIFT) /* 4 Pad */ 

#define QSPI_LUT_INSTR1_SHIFT             (26)      /* Bits 26-31: Instruction 1 (INSTR1) */
#define QSPI_LUT_INSTR1_MASK              (0x3f << QSPI_LUT_INSTR1_SHIFT)
#define QSPI_LUT_INSTR1(n)                (((n) << QSPI_LUT_INSTR1_SHIFT) & QSPI_LUT_INSTR1_MASK)

/* External Memory Base Address */

#define QSPI_AMBA_BASE    0x68000000

/* flash connection to the QSPI module */

typedef enum
{
  QSPI_SIDE_A1    = 0x00u,  /* Serial flash connected on side A1 */
  QSPI_SIDE_A2    = 0x01u,  /* Serial flash connected on side A2 */
  QSPI_SIDE_B1    = 0x02u,  /* Serial flash connected on side B1 */
  QSPI_SIDE_B2    = 0x03u,  /* Serial flash connected on side B2 */
} s32k3xx_qspi_connectiontype;

/* flash operation type */

typedef enum
{
  QSPI_OP_TYPE_CMD          = 0x00u,  /* Simple command                         */
  QSPI_OP_TYPE_WRITE_REG    = 0x01u,  /* Write value in external flash register */
  QSPI_OP_TYPE_RMW_REG      = 0x02u,  /* RMW command on external flash register */
  QSPI_OP_TYPE_READ_REG     = 0x03u,  /* Read external flash register until expected value is read */
  QSPI_OP_TYPE_QSPI_CFG     = 0x04u,  /* Re-configure QSPI controller           */
} s32k3xx_qspi_optype;

/* Lut commands */

typedef enum
{
  QSPI_LUT_INSTR_STOP            = (0u << 10u),    /* End of sequence                           */
  QSPI_LUT_INSTR_CMD             = (1u << 10u),    /* Command                                   */
  QSPI_LUT_INSTR_ADDR            = (2u << 10u),    /* Address                                   */
  QSPI_LUT_INSTR_DUMMY           = (3u << 10u),    /* Dummy cycles                              */
  QSPI_LUT_INSTR_MODE            = (4u << 10u),    /* 8-bit mode                                */
  QSPI_LUT_INSTR_MODE2           = (5u << 10u),    /* 2-bit mode                                */
  QSPI_LUT_INSTR_MODE4           = (6u << 10u),    /* 4-bit mode                                */
  QSPI_LUT_INSTR_READ            = (7u << 10u),    /* Read data                                 */
  QSPI_LUT_INSTR_WRITE           = (8u << 10u),    /* Write data                                */
  QSPI_LUT_INSTR_JMP_ON_CS       = (9u << 10u),    /* Jump on chip select deassert and stop     */
  QSPI_LUT_INSTR_ADDR_DDR        = (10u << 10u),   /* Address - DDR mode                        */
  QSPI_LUT_INSTR_MODE_DDR        = (11u << 10u),   /* 8-bit mode - DDR mode                     */
  QSPI_LUT_INSTR_MODE2_DDR       = (12u << 10u),   /* 2-bit mode - DDR mode                     */
  QSPI_LUT_INSTR_MODE4_DDR       = (13u << 10u),   /* 4-bit mode - DDR mode                     */
  QSPI_LUT_INSTR_READ_DDR        = (14u << 10u),   /* Read data - DDR mode                      */
  QSPI_LUT_INSTR_WRITE_DDR       = (15u << 10u),   /* Write data - DDR mode                     */
  QSPI_LUT_INSTR_DATA_LEARN      = (16u << 10u),   /* Data learning pattern                     */
  QSPI_LUT_INSTR_CMD_DDR         = (17u << 10u),   /* Command - DDR mode                        */
  QSPI_LUT_INSTR_CADDR           = (18u << 10u),   /* Column address                            */
  QSPI_LUT_INSTR_CADDR_DDR       = (19u << 10u),   /* Column address - DDR mode                 */
  QSPI_LUT_INSTR_JMP_TO_SEQ      = (20u << 10u),   /* Jump on chip select deassert and continue */
} s32k3xx_qspi_lutcommandstype;

/* Lut pad options */

typedef enum
{
  QSPI_LUT_PADS_1              = (0u << 8u),    /* 1 Pad */
  QSPI_LUT_PADS_2              = (1u << 8u),    /* 2 Pads */
  QSPI_LUT_PADS_4              = (2u << 8u),    /* 4 Pads */
  QSPI_LUT_PADS_8              = (3u << 8u),    /* 8 Pads */
} s32k3xx_qspi_lutpadstype;

#endif /* __ARCH_ARM_SRC_S32K3XX_HARDWARE_S32K3XX_QSPI_H */
