/****************************************************************************
 * arch/arm/src/s32k3xx/hardware/s32k3xx_mscm.h
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

#ifndef __ARCH_ARM_SRC_S32K3XX_HARDWARE_S32K3XX_MSCM_H
#define __ARCH_ARM_SRC_S32K3XX_HARDWARE_S32K3XX_MSCM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <hardware/s32k3xx_memorymap.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* MSCM Register Offsets ****************************************************/

#define S32K3XX_MSCM_CPXTYPE_OFFSET       (0x0000) /* Processor X Type Register (CPXTYPE) */
#define S32K3XX_MSCM_CPXNUM_OFFSET        (0x0004) /* Processor X Number Register (CPXNUM) */
#define S32K3XX_MSCM_CPXREV_OFFSET        (0x0008) /* Processor X Revision Register (CPXREV) */
#define S32K3XX_MSCM_CPXCFG0_OFFSET       (0x000c) /* Processor X Configuration 0 Register (CPXCFG0) */
#define S32K3XX_MSCM_CPXCFG1_OFFSET       (0x0010) /* Processor X Configuration 1 Register (CPXCFG1) */
#define S32K3XX_MSCM_CPXCFG2_OFFSET       (0x0014) /* Processor X Configuration 2 Register (CPXCFG2) */
#define S32K3XX_MSCM_CPXCFG3_OFFSET       (0x0018) /* Processor X Configuration 3 Register (CPXCFG3) */
#define S32K3XX_MSCM_CP0TYPE_OFFSET       (0x0020) /* Processor 0 Type Register (CP0TYPE) */
#define S32K3XX_MSCM_CP0NUM_OFFSET        (0x0024) /* Processor 0 Number Register (CP0NUM) */
#define S32K3XX_MSCM_CP0REV_OFFSET        (0x0028) /* Processor 0 Revision Register (CP0REV) */
#define S32K3XX_MSCM_CP0CFG0_OFFSET       (0x002c) /* Processor 0 Configuration 0 Register (CP0CFG0) */
#define S32K3XX_MSCM_CP0CFG1_OFFSET       (0x0030) /* Processor 0 Configuration 1 Register (CP0CFG1) */
#define S32K3XX_MSCM_CP0CFG2_OFFSET       (0x0034) /* Processor 0 Configuration 2 Register (CP0CFG2) */
#define S32K3XX_MSCM_CP0CFG3_OFFSET       (0x0038) /* Processor 0 Configuration 3 Register (CP0CFG3) */
#define S32K3XX_MSCM_CP1TYPE_OFFSET       (0x0040) /* Processor 1 Type Register (CP1TYPE) */
#define S32K3XX_MSCM_CP1NUM_OFFSET        (0x0044) /* Processor 1 Number Register (CP1NUM) */
#define S32K3XX_MSCM_CP1REV_OFFSET        (0x0048) /* Processor 1 Revision Register (CP1REV) */
#define S32K3XX_MSCM_CP1CFG0_OFFSET       (0x004c) /* Processor 1 Configuration 0 Register (CP1CFG0) */
#define S32K3XX_MSCM_CP1CFG1_OFFSET       (0x0050) /* Processor 1 Configuration 1 Register (CP1CFG1) */
#define S32K3XX_MSCM_CP1CFG2_OFFSET       (0x0054) /* Processor 1 Configuration 2 Register (CP1CFG2) */
#define S32K3XX_MSCM_CP1CFG3_OFFSET       (0x0058) /* Processor 1 Configuration 3 Register (CP1CFG3) */
#define S32K3XX_MSCM_IRCP0ISR0_OFFSET     (0x0200) /* Interrupt Router CP0 Interrupt Status Register 0 (IRCP0ISR0) */
#define S32K3XX_MSCM_IRCP0IGR0_OFFSET     (0x0204) /* Interrupt Router CP0 Interrupt Generation Register 0 (IRCP0IGR0) */
#define S32K3XX_MSCM_IRCP0ISR1_OFFSET     (0x0208) /* Interrupt Router CP0 Interrupt Status Register 1 (IRCP0ISR1) */
#define S32K3XX_MSCM_IRCP0IGR1_OFFSET     (0x020c) /* Interrupt Router CP0 Interrupt Generation Register 1 (IRCP0IGR1) */
#define S32K3XX_MSCM_IRCP0ISR2_OFFSET     (0x0210) /* Interrupt Router CP0 Interrupt Status Register 2 (IRCP0ISR2) */
#define S32K3XX_MSCM_IRCP0IGR2_OFFSET     (0x0214) /* Interrupt Router CP0 Interrupt Generation Register 2 (IRCP0IGR2) */
#define S32K3XX_MSCM_IRCP0ISR3_OFFSET     (0x0218) /* Interrupt Router CP0 Interrupt Status Register 3 (IRCP0ISR3) */
#define S32K3XX_MSCM_IRCP0IGR3_OFFSET     (0x021c) /* Interrupt Router CP0 Interrupt Generation Register 3 (IRCP0IGR3) */
#define S32K3XX_MSCM_IRCP1ISR0_OFFSET     (0x0220) /* Interrupt Router CP1 Interrupt Status Register 0 (IRCP1ISR0) */
#define S32K3XX_MSCM_IRCP1IGR0_OFFSET     (0x0224) /* Interrupt Router CP1 Interrupt Generation Register 0 (IRCP1IGR0) */
#define S32K3XX_MSCM_IRCP1ISR1_OFFSET     (0x0228) /* Interrupt Router CP1 Interrupt Status Register 1 (IRCP1ISR1) */
#define S32K3XX_MSCM_IRCP1IGR1_OFFSET     (0x022c) /* Interrupt Router CP1 Interrupt Generation Register 1 (IRCP1IGR1) */
#define S32K3XX_MSCM_IRCP1ISR2_OFFSET     (0x0230) /* Interrupt Router CP1 Interrupt Status Register 2 (IRCP1ISR2) */
#define S32K3XX_MSCM_IRCP1IGR2_OFFSET     (0x0234) /* Interrupt Router CP1 Interrupt Generation Register 2 (IRCP1IGR2) */
#define S32K3XX_MSCM_IRCP1ISR3_OFFSET     (0x0238) /* Interrupt Router CP1 Interrupt Status Register 3 (IRCP1ISR3) */
#define S32K3XX_MSCM_IRCP1IGR3_OFFSET     (0x023c) /* Interrupt Router CP1 Interrupt Generation Register 3 (IRCP1IGR3) */
#define S32K3XX_MSCM_IRCPCFG_OFFSET       (0x0400) /* Interrupt Router Configuration Register (IRCPCFG) */
#define S32K3XX_MSCM_ENEDC_OFFSET         (0x0600) /* Enable Interconnect Error Detection Register (ENEDC) */
#define S32K3XX_MSCM_IAHBCFGREG_OFFSET    (0x0700) /* AHB Gasket Configuration Register (IAHBCFGREG) */

#define S32K3XX_MSCM_IRSPRC_OFFSET(n)     (0x0880 + ((n) << 1)) /* Interrupt Router Shared Peripheral Routing Control n=0..239 Register (IRSPRCn) */

/* MSCM Register Addresses **************************************************/

#define S32K3XX_MSCM_CPXTYPE              (S32K3XX_MSCM_BASE + S32K3XX_MSCM_CPXTYPE_OFFSET)
#define S32K3XX_MSCM_CPXNUM               (S32K3XX_MSCM_BASE + S32K3XX_MSCM_CPXNUM_OFFSET)
#define S32K3XX_MSCM_CPXREV               (S32K3XX_MSCM_BASE + S32K3XX_MSCM_CPXREV_OFFSET)
#define S32K3XX_MSCM_CPXCFG0              (S32K3XX_MSCM_BASE + S32K3XX_MSCM_CPXCFG0_OFFSET)
#define S32K3XX_MSCM_CPXCFG1              (S32K3XX_MSCM_BASE + S32K3XX_MSCM_CPXCFG1_OFFSET)
#define S32K3XX_MSCM_CPXCFG2              (S32K3XX_MSCM_BASE + S32K3XX_MSCM_CPXCFG2_OFFSET)
#define S32K3XX_MSCM_CPXCFG3              (S32K3XX_MSCM_BASE + S32K3XX_MSCM_CPXCFG3_OFFSET)
#define S32K3XX_MSCM_CP0TYPE              (S32K3XX_MSCM_BASE + S32K3XX_MSCM_CP0TYPE_OFFSET)
#define S32K3XX_MSCM_CP0NUM               (S32K3XX_MSCM_BASE + S32K3XX_MSCM_CP0NUM_OFFSET)
#define S32K3XX_MSCM_CP0REV               (S32K3XX_MSCM_BASE + S32K3XX_MSCM_CP0REV_OFFSET)
#define S32K3XX_MSCM_CP0CFG0              (S32K3XX_MSCM_BASE + S32K3XX_MSCM_CP0CFG0_OFFSET)
#define S32K3XX_MSCM_CP0CFG1              (S32K3XX_MSCM_BASE + S32K3XX_MSCM_CP0CFG1_OFFSET)
#define S32K3XX_MSCM_CP0CFG2              (S32K3XX_MSCM_BASE + S32K3XX_MSCM_CP0CFG2_OFFSET)
#define S32K3XX_MSCM_CP0CFG3              (S32K3XX_MSCM_BASE + S32K3XX_MSCM_CP0CFG3_OFFSET)
#define S32K3XX_MSCM_CP1TYPE              (S32K3XX_MSCM_BASE + S32K3XX_MSCM_CP1TYPE_OFFSET)
#define S32K3XX_MSCM_CP1NUM               (S32K3XX_MSCM_BASE + S32K3XX_MSCM_CP1NUM_OFFSET)
#define S32K3XX_MSCM_CP1REV               (S32K3XX_MSCM_BASE + S32K3XX_MSCM_CP1REV_OFFSET)
#define S32K3XX_MSCM_CP1CFG0              (S32K3XX_MSCM_BASE + S32K3XX_MSCM_CP1CFG0_OFFSET)
#define S32K3XX_MSCM_CP1CFG1              (S32K3XX_MSCM_BASE + S32K3XX_MSCM_CP1CFG1_OFFSET)
#define S32K3XX_MSCM_CP1CFG2              (S32K3XX_MSCM_BASE + S32K3XX_MSCM_CP1CFG2_OFFSET)
#define S32K3XX_MSCM_CP1CFG3              (S32K3XX_MSCM_BASE + S32K3XX_MSCM_CP1CFG3_OFFSET)
#define S32K3XX_MSCM_IRCP0ISR0            (S32K3XX_MSCM_BASE + S32K3XX_MSCM_IRCP0ISR0_OFFSET)
#define S32K3XX_MSCM_IRCP0IGR0            (S32K3XX_MSCM_BASE + S32K3XX_MSCM_IRCP0IGR0_OFFSET)
#define S32K3XX_MSCM_IRCP0ISR1            (S32K3XX_MSCM_BASE + S32K3XX_MSCM_IRCP0ISR1_OFFSET)
#define S32K3XX_MSCM_IRCP0IGR1            (S32K3XX_MSCM_BASE + S32K3XX_MSCM_IRCP0IGR1_OFFSET)
#define S32K3XX_MSCM_IRCP0ISR2            (S32K3XX_MSCM_BASE + S32K3XX_MSCM_IRCP0ISR2_OFFSET)
#define S32K3XX_MSCM_IRCP0IGR2            (S32K3XX_MSCM_BASE + S32K3XX_MSCM_IRCP0IGR2_OFFSET)
#define S32K3XX_MSCM_IRCP0ISR3            (S32K3XX_MSCM_BASE + S32K3XX_MSCM_IRCP0ISR3_OFFSET)
#define S32K3XX_MSCM_IRCP0IGR3            (S32K3XX_MSCM_BASE + S32K3XX_MSCM_IRCP0IGR3_OFFSET)
#define S32K3XX_MSCM_IRCP1ISR0            (S32K3XX_MSCM_BASE + S32K3XX_MSCM_IRCP1ISR0_OFFSET)
#define S32K3XX_MSCM_IRCP1IGR0            (S32K3XX_MSCM_BASE + S32K3XX_MSCM_IRCP1IGR0_OFFSET)
#define S32K3XX_MSCM_IRCP1ISR1            (S32K3XX_MSCM_BASE + S32K3XX_MSCM_IRCP1ISR1_OFFSET)
#define S32K3XX_MSCM_IRCP1IGR1            (S32K3XX_MSCM_BASE + S32K3XX_MSCM_IRCP1IGR1_OFFSET)
#define S32K3XX_MSCM_IRCP1ISR2            (S32K3XX_MSCM_BASE + S32K3XX_MSCM_IRCP1ISR2_OFFSET)
#define S32K3XX_MSCM_IRCP1IGR2            (S32K3XX_MSCM_BASE + S32K3XX_MSCM_IRCP1IGR2_OFFSET)
#define S32K3XX_MSCM_IRCP1ISR3            (S32K3XX_MSCM_BASE + S32K3XX_MSCM_IRCP1ISR3_OFFSET)
#define S32K3XX_MSCM_IRCP1IGR3            (S32K3XX_MSCM_BASE + S32K3XX_MSCM_IRCP1IGR3_OFFSET)
#define S32K3XX_MSCM_IRCPCFG              (S32K3XX_MSCM_BASE + S32K3XX_MSCM_IRCPCFG_OFFSET)
#define S32K3XX_MSCM_ENEDC                (S32K3XX_MSCM_BASE + S32K3XX_MSCM_ENEDC_OFFSET)
#define S32K3XX_MSCM_IAHBCFGREG           (S32K3XX_MSCM_BASE + S32K3XX_MSCM_IAHBCFGREG_OFFSET)
#define S32K3XX_MSCM_IRSPRC(n)            (S32K3XX_MSCM_BASE + S32K3XX_MSCM_IRSPRC_OFFSET(n))

/* MSCM Register Bitfield Definitions ***************************************/

/* Processor x Type Register (CPXTYPE) */

#define MSCM_CPXTYPE_PERSONALITY_SHIFT    (0)       /* Bits 0-31: Personality of CPx (PERSONALITY) */
#define MSCM_CPXTYPE_PERSONALITY_MASK     (0xffffffff << MSCM_CPXTYPE_PERSONALITY_SHIFT)
#  define MSCM_CPXTYPE_PERSONALITY_M7_0   (0x434d3730 << MSCM_CPXTYPE_PERSONALITY_SHIFT) /* Cortex-M7 core 0 */
#  define MSCM_CPXTYPE_PERSONALITY_M7_1   (0x434d3731 << MSCM_CPXTYPE_PERSONALITY_SHIFT) /* Cortex-M7 core 1 */

/* Processor x Number Register (CPXNUM) */

#define MSCM_CPXNUM_CPN_SHIFT             (0)       /* Bits 0-1: Processor Number (CPN) */
#define MSCM_CPXNUM_CPN_MASK              (0x03 << MSCM_CPXNUM_CPN_SHIFT)
#  define MSCM_CPXNUM_CPN_M7_0            (0x00 << MSCM_CPXNUM_CPN_SHIFT) /* Cortex-M7 core 0 */
#  define MSCM_CPXNUM_CPN_M7_1            (0x01 << MSCM_CPXNUM_CPN_SHIFT) /* Cortex-M7 core 1 */

                                                    /* Bits 2-31: Reserved */

/* Processor x Revision Register (CPXREV) */

#define MSCM_CPXREV_RYPZ_SHIFT            (0)       /* Bits 0-7: Processor Revision (RYPZ) */
#define MSCM_CPXREV_RYPZ_MASK             (0xff << MSCM_CPXREV_RYPZ_SHIFT)

                                                    /* Bits 8-31: Reserved */

/* Processor x Configuration 0 Register (CPXCFG0) */

#define MSCM_CPXCFG0_DCWY_SHIFT           (0)       /* Bits 0-7: L1 Data Cache Ways (DCWY) */
#define MSCM_CPXCFG0_DCWY_MASK            (0xff << MSCM_CPXCFG0_DCWY_SHIFT)
#define MSCM_CPXCFG0_DCSZ_SHIFT           (8)       /* Bits 8-15: L1 Data Cache Size (DCSZ) */
#define MSCM_CPXCFG0_DCSZ_MASK            (0xff << MSCM_CPXCFG0_DCSZ_SHIFT)
#define MSCM_CPXCFG0_ICWY_SHIFT           (16)      /* Bits 16-23: L1 Instruction Cache Ways (ICWY) */
#define MSCM_CPXCFG0_ICWY_MASK            (0xff << MSCM_CPXCFG0_ICWY_SHIFT)
#define MSCM_CPXCFG0_ICSZ_SHIFT           (24)      /* Bits 24-31: L1 Instruction Cache Size (ICSZ) */
#define MSCM_CPXCFG0_ICSZ_MASK            (0xff << MSCM_CPXCFG0_ICSZ_SHIFT)

/* Processor x Configuration 1 Register (CPXCFG1) */

                                                    /* Bits 0-15: Reserved */
#define MSCM_CPXCFG1_L2WY_SHIFT           (16)      /* Bits 16-23: L2 Cache Ways (L2WY) */
#define MSCM_CPXCFG1_L2WY_MASK            (0xff << MSCM_CPXCFG1_L2WY_SHIFT)
#define MSCM_CPXCFG1_L2SZ_SHIFT           (24)      /* Bits 24-31: L2 Cache Size (L2SZ) */
#define MSCM_CPXCFG1_L2SZ_MASK            (0xff << MSCM_CPXCFG1_L2SZ_SHIFT)

/* Processor x Configuration 2 Register (CPXCFG2) */

                                                    /* Bits 0-15: Reserved */

#define MSCM_CPXCFG2_ITCMSZ_SHIFT         (16)      /* Bits 16-23: Instruction Tightly Coupled Memory Size (ITCMSZ) */
#define MSCM_CPXCFG2_ITCMSZ_MASK          (0xff << MSCM_CPXCFG2_ITCMSZ_SHIFT)
#define MSCM_CPXCFG2_DTCMSZ_SHIFT         (24)      /* Bits 24-31: Data Tightly Coupled Memory Size (DTCMSZ) */
#define MSCM_CPXCFG2_DTCMSZ_MASK          (0xff << MSCM_CPXCFG2_DTCMSZ_SHIFT)

/* Processor x Configuration 3 Register (CPXCFG3) */

#define MSCM_CPXCFG3_FPU                  (1 << 0)  /* Bit 0: Floating Point Unit (FPU) */
#define MSCM_CPXCFG3_SIMD                 (1 << 1)  /* Bit 1: SIMD/NEON Instruction Support (SIMD) */
#define MSCM_CPXCFG3_MMU                  (1 << 2)  /* Bit 2: Memory Mangement Unit (MMU) */
#define MSCM_CPXCFG3_CMP                  (1 << 3)  /* Bit 3: Core Memory Protection Unit (CMP) */
#define MSCM_CPXCFG3_CPY                  (1 << 4)  /* Bit 4: Cryptography (CPY) */
                                                    /* Bits 5-31: Reserved */

/* Interrupt Router CPn Interrupt Status Register m (IRCPnISRm) */

#define MSCM_IRCPISR_CP0_INT              (1 << 0)  /* Bit 0: CP0-to-CPn Interrupt (CP0_INT) */
#define MSCM_IRCPISR_CP1_INT              (1 << 1)  /* Bit 1: CP1-to-CPn Interrupt (CP1_INT) */
                                                    /* Bit 2-31: Reserved */

/* Interrupt Router CPn Interrupt Generation Register m (IRCPnIGRm) */

#define MSCM_IRCPIGR_INT_EN               (1 << 0)  /* Bit 0: Interrupt Enable (INT_EN) */
                                                    /* Bit 1-31: Reserved */

/* Interrupt Router Configuration Register (IRCPCFG) */

#define MSCM_IRCPCFG_CP0_TR               (1 << 0)  /* Bit 0: CP0 as Trusted Core (CP0_TR) */
#define MSCM_IRCPCFG_CP1_TR               (1 << 1)  /* Bit 1: CP1 as Trusted Core (CP1_TR) */
                                                    /* Bits 2-30: Reserved */
#define MSCM_IRCPCFG_LOCK                 (1 << 31) /* Bit 31: Lock (LOCK) */

/* Enable Interconnect Error Detection Register (ENEDC) */

#define MSCM_ENEDC_EN_RD_CM7_0_AHBM       (1 << 0)  /* Bit 0: Enable Read Data Check Cortex-M7_0_AHBM (EN_RD_CM7_0_AHBM) */
#define MSCM_ENEDC_EN_RD_CM7_0_AHBP       (1 << 1)  /* Bit 1: Enable Read Data Check Cortex-M7_0_AHBP (EN_RD_CM7_0_AHBP) */
#define MSCM_ENEDC_EN_RD_EDMA             (1 << 2)  /* Bit 2: Enable Read Data Check eDMA (EN_RD_EDMA) */
                                                    /* Bit 3: Reserved */
#define MSCM_ENEDC_EN_RD_HSE              (1 << 4)  /* Bit 4: Enable Read Data Check HSE (EN_RD_HSE) */
#define MSCM_ENEDC_EN_RD_EMAC             (1 << 5)  /* Bit 5: Enable Read Data Check EMAC (EN_RD_EMAC) */
#define MSCM_ENEDC_EN_RD_CM7_1_AHBM       (1 << 6)  /* Bit 6: Enable Read Data Check Cortex-M7_1_AHBM (EN_RD_CM7_1_AHBM) */
#define MSCM_ENEDC_EN_RD_CM7_1_AHBP       (1 << 7)  /* Bit 7: Enable Read Data Check Cortex-M7_1_AHBP (EN_RD_CM7_1_AHBP) */
#define MSCM_ENEDC_EN_RD_TCM              (1 << 8)  /* Bit 8: Enable Read Data Check TCM (EN_RD_TCM) */
#define MSCM_ENEDC_EN_ADD_PFLASH_PORT0    (1 << 9)  /* Bit 9: Enable Address Check P_FLASH_PORT0 (EN_ADD_PFLASH_PORT0) */
#define MSCM_ENEDC_EN_ADD_PFLASH_PORT1    (1 << 10) /* Bit 10: Enable Address Check P_FLASH_PORT1 (EN_ADD_PFLASH_PORT1) */
#define MSCM_ENEDC_EN_ADD_PFLASH_PORT2    (1 << 11) /* Bit 11: Enable Address Check P_FLASH_PORT2 (EN_ADD_PFLASH_PORT2) */
#define MSCM_ENEDC_EN_WR_PRAM0            (1 << 12) /* Bit 12: Enable Write Data Check PRAM0 (EN_WR_PRAM0) */
#define MSCM_ENEDC_EN_ADD_PRAM0           (1 << 13) /* Bit 13: Enable Address Check PRAM0 (EN_ADD_PRAM0) */
#define MSCM_ENEDC_EN_WR_PRAM1            (1 << 14) /* Bit 14: Enable Write Data Check PRAM1 (EN_WR_PRAM1) */
#define MSCM_ENEDC_EN_ADD_PRAM1           (1 << 15) /* Bit 15: Enable Address Check PRAM1 (EN_ADD_PRAM1) */
#define MSCM_ENEDC_EN_WR_TCM              (1 << 16) /* Bit 16: Enable Write Data Check TCM (EN_WR_TCM) */
#define MSCM_ENEDC_EN_ADD_TCM             (1 << 17) /* Bit 17: Enable Address Check TCM (EN_ADD_TCM) */
                                                    /* Bit 18: Reserved */
#define MSCM_ENEDC_EN_ADD_QSPI            (1 << 19) /* Bit 19: Enable Address Check QuadSPI (EN_ADD_QSPI) */
#define MSCM_ENEDC_EN_WR_AIPS0            (1 << 20) /* Bit 20: Enable Write Data Check AIPS0 (EN_WR_AIPS0) */
#define MSCM_ENEDC_EN_ADD_AIPS0           (1 << 21) /* Bit 21: Enable Address Check AIPS0 (EN_ADD_AIPS0) */
#define MSCM_ENEDC_EN_WR_AIPS1            (1 << 22) /* Bit 22: Enable Write Data Check AIPS1 (EN_WR_AIPS1) */
#define MSCM_ENEDC_EN_ADD_AIPS1           (1 << 23) /* Bit 23: Enable Address Check AIPS1 (EN_ADD_AIPS1) */
#define MSCM_ENEDC_EN_WR_AIPS2            (1 << 24) /* Bit 24: Enable Write Data Check AIPS2 (EN_WR_AIPS2) */
#define MSCM_ENEDC_EN_ADD_AIPS2           (1 << 25) /* Bit 25: Enable Address Check AIPS2 (EN_ADD_AIPS2) */
#define MSCM_ENEDC_EN_WR_CM7_0_TCM        (1 << 26) /* Bit 26: Enable Write Data Check Cortex-M7_0_TCM (EN_WR_CM7_0_TCM) */
#define MSCM_ENEDC_EN_ADD_CM7_0_TCM       (1 << 27) /* Bit 27: Enable Address Check Cortex-M7_0_TCM (EN_ADD_CM7_0_TCM) */
#define MSCM_ENEDC_EN_WR_CM7_1_TCM        (1 << 28) /* Bit 28: Enable Write Data Check Cortex-M7_1_TCM (EN_WR_CM7_1_TCM) */
#define MSCM_ENEDC_EN_ADD_CM7_1_TCM       (1 << 29) /* Bit 29: Enable Address Check Cortex-M7_1_TCM (EN_ADD_CM7_1_TCM) */
                                                    /* Bits 30-31: Reserved */

/* AHB Gasket Configuration Register (IAHBCFGREG) */

#define MSCM_IAHBCFG_EMAC_DIS_WR_OPT      (1 << 0)  /* Bit 0: EMAC AHB gasket write burst optimizations disabled (EMAC_DIS_WR_OPT) */
                                                    /* Bits 1-3: Reserved */

#define MSCM_IAHBCFG_DMA_AXBS_S0_DIS_WR_OPT (1 << 4)  /* Bit 4: DMA AXBS S0 AHB gasket write burst optimizations disabled (DMA_AXBS_S0_DIS_WR_OPT) */

                                                    /* Bits 5-7: Reserved */

#define MSCM_IAHBCFG_DMA_AXBS_S1_DIS_WR_OPT (1 << 8)  /* Bit 8: DMA AXBS S1 AHB gasket write burst optimizations disabled (DMA_AXBS_S1_DIS_WR_OPT) */

                                                    /* Bits 9-11: Reserved */

#define MSCM_IAHBCFG_DMA_AXBS_S2_DIS_WR_OPT (1 << 12) /* Bit 12: DMA AXBS S2 AHB gasket write burst optimizations disabled (DMA_AXBS_S2_DIS_WR_OPT) */

                                                    /* Bits 13-15: Reserved */
#define MSCM_IAHBCFG_TCM_DIS_WR_OPT       (1 << 16) /* Bit 16: TCM AHB gasket write burst optimizations disabled (TCM_DIS_WR_OPT) */
                                                    /* Bits 17-19: Reserved */
#define MSCM_IAHBCFG_QSPI_DIS_WR_OPT      (1 << 20) /* Bit 20: QuadSPI AHB gasket write burst optimizations disabled (QSPI_DIS_WR_OPT) */
                                                    /* Bits 21-23: Reserved */
#define MSCM_IAHBCFG_AIPS1_DIS_WR_OPT     (1 << 24) /* Bit 24: AIPS1 AHB gasket write burst optimizations disabled (AIPS1_DIS_WR_OPT) */
                                                    /* Bits 25-27: Reserved */
#define MSCM_IAHBCFG_AIPS2_DIS_WR_OPT     (1 << 28) /* Bit 28: AIPS2 AHB gasket write burst optimizations disabled (AIPS2_DIS_WR_OPT) */
                                                    /* Bits 29-31: Reserved */

/* Interrupt Router Shared Peripheral Routing Control n=0..239 (IRSPRCn) */

#define MSCM_IRSPRC_M7_0                  (1 << 0)  /* Bit 0: Enable Cortex-M7_0 Interrupt Steering (M7_0) */
#define MSCM_IRSPRC_M7_1                  (1 << 1)  /* Bit 1: Enable Cortex-M7_1 Interrupt Steering (M7_1) */
                                                    /* Bits 2-14: Reserved */
#define MSCM_IRSPRC_LOCK                  (1 << 15) /* Bit 15: Lock (LOCK) */

#endif /* __ARCH_ARM_SRC_S32K3XX_HARDWARE_S32K3XX_MSCM_H */
