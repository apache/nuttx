/****************************************************************************
 * arch/arm/src/s32k3xx/hardware/s32k3xx_fmu.h
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

#ifndef __ARCH_ARM_SRC_S32K3XX_HARDWARE_S32K3XX_FMU_H
#define __ARCH_ARM_SRC_S32K3XX_HARDWARE_S32K3XX_FMU_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <hardware/s32k3xx_memorymap.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* FMU Register Offsets *****************************************************/

#define S32K3XX_FMU_MCR_OFFSET       (0x0000) /* Module Configuration Register (MCR) */
#define S32K3XX_FMU_MCRS_OFFSET      (0x0004) /* Module Configuration Status Register (MCRS) */
#define S32K3XX_FMU_MCRE_OFFSET      (0x0008) /* Extended Module Configuration Register (MCRE) */
#define S32K3XX_FMU_CTL_OFFSET       (0x000c) /* Module Control Register (CTL) */
#define S32K3XX_FMU_ADR_OFFSET       (0x0010) /* Address Register (ADR) */
#define S32K3XX_FMU_PEADR_OFFSET     (0x0014) /* Program and Erase Address Register (PEADR) */
#define S32K3XX_FMU_SPELOCK_OFFSET   (0x0050) /* Sector Program and Erase Hardware Lock (SPELOCK) */
#define S32K3XX_FMU_SSPELOCK_OFFSET  (0x0054) /* Super Sector Program and Erase Hardware Lock (SSPELOCK) */
#define S32K3XX_FMU_XSPELOCK_OFFSET  (0x0070) /* Express Sector Program and Erase Hardware Lock (XSPELOCK) */
#define S32K3XX_FMU_XSSPELOCK_OFFSET (0x0074) /* Express Super Sector Program and Erase Hardware Lock (XSSPELOCK) */
#define S32K3XX_FMU_TMD_OFFSET       (0x0090) /* Test Mode Disable Password Check (TMD) */
#define S32K3XX_FMU_UT0_OFFSET       (0x0094) /* UTest 0 (UT0) */
#define S32K3XX_FMU_UM0_OFFSET       (0x0098) /* UMISR0 (UM0) */
#define S32K3XX_FMU_UM1_OFFSET       (0x009c) /* UMISR1 (UM1) */
#define S32K3XX_FMU_UM2_OFFSET       (0x00a0) /* UMISR2 (UM2) */
#define S32K3XX_FMU_UM3_OFFSET       (0x00a4) /* UMISR3 (UM3) */
#define S32K3XX_FMU_UM4_OFFSET       (0x00a8) /* UMISR4 (UM4) */
#define S32K3XX_FMU_UM5_OFFSET       (0x00ac) /* UMISR5 (UM5) */
#define S32K3XX_FMU_UM6_OFFSET       (0x00b0) /* UMISR6 (UM6) */
#define S32K3XX_FMU_UM7_OFFSET       (0x00b4) /* UMISR7 (UM7) */
#define S32K3XX_FMU_UM8_OFFSET       (0x00b8) /* UMISR8 (UM8) */
#define S32K3XX_FMU_UM9_OFFSET       (0x00bc) /* UMISR9 (UM9) */
#define S32K3XX_FMU_UD0_OFFSET       (0x00d0) /* UTest Data 0 (UD0) */
#define S32K3XX_FMU_UD1_OFFSET       (0x00d4) /* UTest Data 1 (UD1) */
#define S32K3XX_FMU_UD2_OFFSET       (0x00d8) /* UTest Data 2 (UD2) */
#define S32K3XX_FMU_UD3_OFFSET       (0x00dc) /* UTest Data 3 (UD3) */
#define S32K3XX_FMU_UD4_OFFSET       (0x00e0) /* UTest Data 4 (UD4) */
#define S32K3XX_FMU_UD5_OFFSET       (0x00e4) /* UTest Data 5 (UD5) */
#define S32K3XX_FMU_UA0_OFFSET       (0x00e8) /* UTest Address 0 (UA0) */
#define S32K3XX_FMU_UA1_OFFSET       (0x00ec) /* UTest Address 1 (UA1) */
#define S32K3XX_FMU_XMCR_OFFSET      (0x00f0) /* Express Module Configuration Register (XMCR) */
#define S32K3XX_FMU_XPEADR_OFFSET    (0x00f4) /* Express Program Address Register (XPEADR) */

#define S32K3XX_FMU_DATA_OFFSET(n)   (0x0100 + ((n) << 2)) /* Program Data (DATAn) */

/* FMU Register Addresses ***************************************************/

#define S32K3XX_FMU_MCR              (S32K3XX_FMU_BASE + S32K3XX_FMU_MCR_OFFSET)
#define S32K3XX_FMU_MCRS             (S32K3XX_FMU_BASE + S32K3XX_FMU_MCRS_OFFSET)
#define S32K3XX_FMU_MCRE             (S32K3XX_FMU_BASE + S32K3XX_FMU_MCRE_OFFSET)
#define S32K3XX_FMU_CTL              (S32K3XX_FMU_BASE + S32K3XX_FMU_CTL_OFFSET)
#define S32K3XX_FMU_ADR              (S32K3XX_FMU_BASE + S32K3XX_FMU_ADR_OFFSET)
#define S32K3XX_FMU_PEADR            (S32K3XX_FMU_BASE + S32K3XX_FMU_PEADR_OFFSET)
#define S32K3XX_FMU_SPELOCK          (S32K3XX_FMU_BASE + S32K3XX_FMU_SPELOCK_OFFSET)
#define S32K3XX_FMU_SSPELOCK         (S32K3XX_FMU_BASE + S32K3XX_FMU_SSPELOCK_OFFSET)
#define S32K3XX_FMU_XSPELOCK         (S32K3XX_FMU_BASE + S32K3XX_FMU_XSPELOCK_OFFSET)
#define S32K3XX_FMU_XSSPELOCK        (S32K3XX_FMU_BASE + S32K3XX_FMU_XSSPELOCK_OFFSET)
#define S32K3XX_FMU_TMD              (S32K3XX_FMU_BASE + S32K3XX_FMU_TMD_OFFSET)
#define S32K3XX_FMU_UT0              (S32K3XX_FMU_BASE + S32K3XX_FMU_UT0_OFFSET)
#define S32K3XX_FMU_UM0              (S32K3XX_FMU_BASE + S32K3XX_FMU_UM0_OFFSET)
#define S32K3XX_FMU_UM1              (S32K3XX_FMU_BASE + S32K3XX_FMU_UM1_OFFSET)
#define S32K3XX_FMU_UM2              (S32K3XX_FMU_BASE + S32K3XX_FMU_UM2_OFFSET)
#define S32K3XX_FMU_UM3              (S32K3XX_FMU_BASE + S32K3XX_FMU_UM3_OFFSET)
#define S32K3XX_FMU_UM4              (S32K3XX_FMU_BASE + S32K3XX_FMU_UM4_OFFSET)
#define S32K3XX_FMU_UM5              (S32K3XX_FMU_BASE + S32K3XX_FMU_UM5_OFFSET)
#define S32K3XX_FMU_UM6              (S32K3XX_FMU_BASE + S32K3XX_FMU_UM6_OFFSET)
#define S32K3XX_FMU_UM7              (S32K3XX_FMU_BASE + S32K3XX_FMU_UM7_OFFSET)
#define S32K3XX_FMU_UM8              (S32K3XX_FMU_BASE + S32K3XX_FMU_UM8_OFFSET)
#define S32K3XX_FMU_UM9              (S32K3XX_FMU_BASE + S32K3XX_FMU_UM9_OFFSET)
#define S32K3XX_FMU_UD0              (S32K3XX_FMU_BASE + S32K3XX_FMU_UD0_OFFSET)
#define S32K3XX_FMU_UD1              (S32K3XX_FMU_BASE + S32K3XX_FMU_UD1_OFFSET)
#define S32K3XX_FMU_UD2              (S32K3XX_FMU_BASE + S32K3XX_FMU_UD2_OFFSET)
#define S32K3XX_FMU_UD3              (S32K3XX_FMU_BASE + S32K3XX_FMU_UD3_OFFSET)
#define S32K3XX_FMU_UD4              (S32K3XX_FMU_BASE + S32K3XX_FMU_UD4_OFFSET)
#define S32K3XX_FMU_UD5              (S32K3XX_FMU_BASE + S32K3XX_FMU_UD5_OFFSET)
#define S32K3XX_FMU_UA0              (S32K3XX_FMU_BASE + S32K3XX_FMU_UA0_OFFSET)
#define S32K3XX_FMU_UA1              (S32K3XX_FMU_BASE + S32K3XX_FMU_UA1_OFFSET)
#define S32K3XX_FMU_XMCR             (S32K3XX_FMU_BASE + S32K3XX_FMU_XMCR_OFFSET)
#define S32K3XX_FMU_XPEADR           (S32K3XX_FMU_BASE + S32K3XX_FMU_XPEADR_OFFSET)

#define S32K3XX_FMU_DATA(n)          (S32K3XX_FMU_BASE + S32K3XX_FMU_DATA_OFFSET(n))

/* FMU Register Bitfield Definitions ****************************************/

/* Module Configuration Register (MCR) */

#define FMU_MCR_EHV                  (1 << 0)  /* Bit 0: Enable High Voltage (EHV) */
#  define FMU_MCR_EHV_DIS            (0 << 0)  /*        Flash memory is not enabled to perform a high voltage operation */
#  define FMU_MCR_EHV_ENA            (1 << 0)  /*        Flash memory is enabled to perform a high voltage operation */
                                               /* Bits 1-3: Reserved */
#define FMU_MCR_ERS                  (1 << 4)  /* Bit 4: Erase (ERS) */
#define FMU_MCR_ESS                  (1 << 5)  /* Bit 5: Erase Size Select (ESS) */
#  define FMU_MCR_ESS_SECTOR         (0 << 5)  /*        Flash memory erase is on a sector */
#  define FMU_MCR_ESS_BLOCK          (1 << 5)  /*        Flash memory erase is on a block */
                                               /* Bits 6-7: Reserved */
#define FMU_MCR_PGM                  (1 << 8)  /* Bit 8: Program (PGM) */
#  define FMU_MCR_PGM_NOTEXEC        (0 << 8)  /*        Flash memory not executing a program sequence */
#  define FMU_MCR_PGM_EXEC           (1 << 8)  /*        Flash memory executing a program sequence */
                                               /* Bits 9-11: Reserved */
#define FMU_MCR_WDIE                 (1 << 12) /* Bit 12: Watch Dog Interrupt Enable (WDIE) */
                                               /* Bits 13-14: Reserved */
#define FMU_MCR_PECIE                (1 << 15) /* Bit 15: Program/Erase Complete Interrupt Enable (PECIE) */
#define FMU_MCR_PEID_SHIFT           (16)      /* Bits 16-23: Program and Erase Master/Domain ID (PEID) */
#define FMU_MCR_PEID_MASK            (0xff << FMU_MCR_PEID_SHIFT)
                                               /* Bits 24-31: Reserved */

/* Module Configuration Status Register (MCRS) */

#define FMU_MCRS_RE                  (1 << 0)  /* Bit 0: Reset Error (RE) */
                                               /* Bits 1-7: Reserved */
#define FMU_MCRS_TSPELOCK            (1 << 8)  /* Bit 8: UTest NVM Program and Erase Lock (TSPELOCK) */
#define FMU_MCRS_EPEG                (1 << 9)  /* Bit 9: ECC Enabled Program/Erase Good (EPEG) */
                                               /* Bits 10-11: Reserved */
#define FMU_MCRS_WDI                 (1 << 12) /* Bit 12: Watch Dog Interrupt (WDI) */
                                               /* Bit 13: Reserved */
#define FMU_MCRS_PEG                 (1 << 14) /* Bit 14: Program/Erase Good (PEG) */
#define FMU_MCRS_DONE                (1 << 15) /* Bit 15: State Machine Status (DONE) */
#define FMU_MCRS_PES                 (1 << 16) /* Bit 16: Program and Erase Sequence Error (PES) */
#define FMU_MCRS_PEP                 (1 << 17) /* Bit 17: Program and Erase Protection Error (PEP) */
                                               /* Bits 18-19: Reserved */
#define FMU_MCRS_RWE                 (1 << 20) /* Bit 20: Read-While-Write Event Error (RWE) */
                                               /* Bits 21-23: Reserved */
#define FMU_MCRS_RRE                 (1 << 24) /* Bit 24: Read Reference Error (RRE) */
#define FMU_MCRS_RVE                 (1 << 25) /* Bit 25: Read Voltage Error (RVE) */
                                               /* Bits 26-27: Reserved */
#define FMU_MCRS_EEE                 (1 << 28) /* Bit 28: EDC after ECC Error (EEE) */
#define FMU_MCRS_AEE                 (1 << 29) /* Bit 29: Address Encode Error (AEE) */
#define FMU_MCRS_SBC                 (1 << 30) /* Bit 30: Single Bit Correction (SBC) */
#define FMU_MCRS_EER                 (1 << 31) /* Bit 31: ECC Event Error (EER) */

/* Extended Module Configuration Register (MCRE) */

                                               /* Bits 0-5: Reserved */
#define FMU_MCRE_N256K_SHIFT         (6)       /* Bits 6-7: Number of 256 KB Blocks (N256K) */
#define FMU_MCRE_N256K_MASK          (0x03 << FMU_MCRE_N256K_SHIFT)
#  define FMU_MCRE_N256K_0           (0x00 << FMU_MCRE_N256K_SHIFT) /* Zero 256 KB blocks */
#  define FMU_MCRE_N256K_1           (0x01 << FMU_MCRE_N256K_SHIFT) /* One 256 KB block */
#  define FMU_MCRE_N256K_2           (0x02 << FMU_MCRE_N256K_SHIFT) /* Two 256 KB blocks */
#  define FMU_MCRE_N256K_4           (0x03 << FMU_MCRE_N256K_SHIFT) /* Four 256 KB blocks */

                                               /* Bits 8-13: Reserved */
#define FMU_MCRE_N512K_SHIFT         (14)      /* Bits 14-15: Number of 512 KB Blocks (N512K) */
#define FMU_MCRE_N512K_MASK          (0x03 << FMU_MCRE_N512K_SHIFT)
#  define FMU_MCRE_N512K_0           (0x00 << FMU_MCRE_N512K_SHIFT) /* Zero 512 KB blocks */
#  define FMU_MCRE_N512K_1           (0x01 << FMU_MCRE_N512K_SHIFT) /* One 512 KB block */
#  define FMU_MCRE_N512K_2           (0x02 << FMU_MCRE_N512K_SHIFT) /* Two 512 KB blocks */
#  define FMU_MCRE_N512K_4           (0x03 << FMU_MCRE_N512K_SHIFT) /* Four 512 KB blocks */

                                               /* Bits 16-20: Reserved */
#define FMU_MCRE_N1M_SHIFT           (14)      /* Bits 21-23: Number of 1 MB Blocks (N1M) */
#define FMU_MCRE_N1M_MASK            (0x07 << FMU_MCRE_N1M_SHIFT)
#  define FMU_MCRE_N1M_0             (0x00 << FMU_MCRE_N1M_SHIFT) /* Zero 1 MB blocks */
#  define FMU_MCRE_N1M_1             (0x01 << FMU_MCRE_N1M_SHIFT) /* One 1 MB block */
#  define FMU_MCRE_N1M_2             (0x02 << FMU_MCRE_N1M_SHIFT) /* Two 1 MB blocks */
#  define FMU_MCRE_N1M_3             (0x03 << FMU_MCRE_N1M_SHIFT) /* Three 1 MB blocks */
#  define FMU_MCRE_N1M_4             (0x04 << FMU_MCRE_N1M_SHIFT) /* Four 1 MB blocks */

                                               /* Bits 24-31: Reserved */

/* Module Control Register (CTL) */

                                               /* Bits 0-7: Reserved */
#define FMU_CTL_RWSC_SHIFT           (8)       /* Bits 8-12: Wait State Control (RWSC) */
#define FMU_CTL_RWSC_MASK            (0x1f << FMU_CTL_RWSC_SHIFT)
#  define FMU_CTL_RWSC(n)            (((n) << FMU_CTL_RWSC_SHIFT) & FMU_CTL_RWSC_MASK)
#  define FMU_CTL_RWSC1              (0x01 << FMU_CTL_RWSC_SHIFT) /* One additional wait state is added */
#  define FMU_CTL_RWSC2              (0x02 << FMU_CTL_RWSC_SHIFT) /* Two additional wait states are added */
#  define FMU_CTL_RWSC3              (0x03 << FMU_CTL_RWSC_SHIFT) /* Three additional wait states are added */
#  define FMU_CTL_RWSC4              (0x04 << FMU_CTL_RWSC_SHIFT) /* Four additional wait states are added */
#  define FMU_CTL_RWSC5              (0x05 << FMU_CTL_RWSC_SHIFT) /* Five additional wait states are added */
#  define FMU_CTL_RWSC6              (0x06 << FMU_CTL_RWSC_SHIFT) /* Six additional wait states are added */
#  define FMU_CTL_RWSC7              (0x07 << FMU_CTL_RWSC_SHIFT) /* Seven additional wait states are added */
#  define FMU_CTL_RWSC8              (0x08 << FMU_CTL_RWSC_SHIFT) /* Eight additional wait states are added */

                                               /* Bits 13-14: Reserved */
#define FMU_CTL_RWSL                 (1 << 15) /* Bit 15: Read Wait State Lock (RWSL) */
                                               /* Bits 16-31: Reserved */

/* Address Register (ADR) */

                                               /* Bit 0: Reserved */
#define FMU_ADR_ADDR_SHIFT           (1)       /* Bits 1-18: Address (ADDR) */
#define FMU_ADR_ADDR_MASK            (0x03ffff << FMU_ADR_ADDR_SHIFT)
#define FMU_ADR_A0                   (1 << 19) /* Bit 19: Address Region 0 (A0) */
#define FMU_ADR_A1                   (1 << 20) /* Bit 20: Address Region 1 (A1) */
#define FMU_ADR_A2                   (1 << 21) /* Bit 21: Address Region 2 (A2) */
#define FMU_ADR_A3                   (1 << 22) /* Bit 22: Address Region 3 (A3) */
#define FMU_ADR_A4                   (1 << 23) /* Bit 23: Address Region 4 (A4) */
#define FMU_ADR_A5                   (1 << 24) /* Bit 24: Address Region 5 (A5) */
                                               /* Bits 25-30: Reserved */
#define FMU_ADR_SAD                  (1 << 31) /* Bit 31: UTest NVM Address (SAD) */

/* Program and Erase Address Register (PEADR) */

                                               /* Bits 0-4: Reserved */
#define FMU_PEADR_PEADDR_SHIFT       (5)       /* Bits 5-18: Program and Erase Address (PEADDR) */
#define FMU_PEADR_PEADDR_MASK        (0x3fff << FMU_PEADR_PEADDR_SHIFT)
#define FMU_PEADR_PEA0               (1 << 19) /* Bit 19: Program and Erase Address Region 0 (PEA0) */
#define FMU_PEADR_PEA1               (1 << 20) /* Bit 20: Program and Erase Address Region 1 (PEA1) */
#define FMU_PEADR_PEA2               (1 << 21) /* Bit 21: Program and Erase Address Region 2 (PEA2) */
#define FMU_PEADR_PEA3               (1 << 22) /* Bit 22: Program and Erase Address Region 3 (PEA3) */
#define FMU_PEADR_PEA4               (1 << 23) /* Bit 23: Program and Erase Address Region 4 (PEA4) */
#define FMU_PEADR_PEA5               (1 << 24) /* Bit 24: Program and Erase Address Region 5 (PEA5) */
                                               /* Bits 25-30: Reserved */
#define FMU_PEADR_PEASAD             (1 << 31) /* Bit 31: UTest NVM Program and Erase Address (PEASAD) */

/* Sector Program and Erase Hardware Lock (SPELOCK) */

#define FMU_SPELOCK_SHIFT            (0)       /* Bits 0-31: Sector Program and Erase Lock (SPELOCK) */
#define FMU_SPELOCK_MASK             (0xffffffff << FMU_SPELOCK_SHIFT)

/* Super Sector Program and Erase Hardware Lock (SSPELOCK) */

#define FMU_SSPELOCK_SHIFT           (0)       /* Bits 0-11: Super Sector Program and Erase Lock (SSPELOCK) */
#define FMU_SSPELOCK_MASK            (0x0fff << FMU_SSPELOCK_SHIFT)
                                               /* Bits 12-31: Reserved */

/* Express Sector Program and Erase Hardware Lock (XSPELOCK) */

#define FMU_XSPELOCK_SHIFT           (0)       /* Bits 0-31: Express Sector Program and Erase Lock (XSPELOCK) */
#define FMU_XSPELOCK_MASK            (0xffffffff << FMU_XSPELOCK_SHIFT)

/* Express Super Sector Program and Erase Hardware Lock (XSSPELOCK) */

#define FMU_XSSPELOCK_SHIFT          (0)       /* Bits 0-11: Super Sector Program and Erase Lock (XSSPELOCK) */
#define FMU_XSSPELOCK_MASK           (0x0fff << FMU_XSSPELOCK_SHIFT)
                                               /* Bits 12-31: Reserved */

/* Test Mode Disable Password Check (TMD) */

#define FMU_TMD_PWD_SHIFT            (0)       /* Bits 0-31: Password Challenge (PWD) */
#define FMU_TMD_PWD_MASK             (0xffffffff << FMU_TMD_PWD_SHIFT)

/* UTest 0 (UT0) */

#define FMU_UT0_AID                  (1 << 0)  /* Bit 0: Array Integrity Done (AID) */
#define FMU_UT0_AIE                  (1 << 1)  /* Bit 1: Array Integrity Enable (AIE) */
#define FMU_UT0_AIS                  (1 << 2)  /* Bit 2: Array Integrity Sequence (AIS) */
                                               /* Bit 3: Reserved */
#define FMU_UT0_MRV                  (1 << 4)  /* Bit 4: Margin Read Value (MRV) */
#define FMU_UT0_MRE                  (1 << 5)  /* Bit 5: Margin Read Enable (MRE) */
#define FMU_UT0_AISUS                (1 << 6)  /* Bit 6: Array Integrity Suspend (AISUS) */
                                               /* Bit 7: Reserved */
#define FMU_UT0_AIBPE                (1 << 8)  /* Bit 8: Array Integrity Break Point Enable (AIBPE) */
#define FMU_UT0_NAIBP                (1 << 9)  /* Bit 9: Next Array Integrity Break Point (NAIBP) */
                                               /* Bits 10-11: Reserved */
#define FMU_UT0_EIE                  (1 << 12) /* Bit 12: ECC Data Input Enable (EIE) */
#define FMU_UT0_EDIE                 (1 << 13) /* Bit 13: EDC after ECC Data Input Enable (EDIE) */
#define FMU_UT0_AEIE                 (1 << 14) /* Bit 14: Address Encode Invert Enable (AEIE) */
#define FMU_UT0_RRIE                 (1 << 15) /* Bit 15: Read Reference Input Enable (RRIE) */
                                               /* Bits 16-29: Reserved */
#define FMU_UT0_SBCE                 (1 << 30) /* Bit 30: Single Bit Correction Enable (SBCE) */
#define FMU_UT0_UTE                  (1 << 31) /* Bit 31: UTest Enable (UTE) */

/* UMISRn (UMn) */

#define FMU_UM_MISR_SHIFT            (0)       /* Bits 0-31: MISR */
#define FMU_UM_MISR_MASK             (0xffffffff << FMU_UM_MISR_SHIFT)

/* UMISR9 (UM9) */

#define FMU_UM9_MISR                 (1 << 0)  /* Bit 0: MISR */
                                               /* Bits 1-31: Reserved */

/* UTest Data n (UDn, n=0,1,3,4) */

#define FMU_UD0134_EDATA_SHIFT       (0)       /* Bits 0-31: ECC Data (EDATA) */
#define FMU_UD0134_EDATA_MASK        (0xffffffff << FMU_UD0134_EDATA_SHIFT)

/* UTest Data n (UDn, n=2,5) */

#define FMU_UD25_EDATAC_SHIFT        (0)       /* Bits 0-7: ECC Data Check Bits (EDATAC) */
#define FMU_UD25_EDATAC_MASK         (0xff << FMU_UD25_EDATAC_SHIFT)
                                               /* Bits 8-23: Reserved */
#define FMU_UD25_ED0                 (1 << 24) /* Bit 24: ECC Logic Check Double Word 0 (ED0) */
#define FMU_UD25_ED1                 (1 << 25) /* Bit 25: ECC Logic Check Double Word 1 (ED1) */
#define FMU_UD25_ED2                 (1 << 26) /* Bit 26: ECC Logic Check Double Word 2 (ED2) */
#define FMU_UD25_ED3                 (1 << 27) /* Bit 27: ECC Logic Check Double Word 3 (ED3) */
                                               /* Bits 28-31: Reserved */

/* UTest Address 0 (UA0) */

#define FMU_UA0_SHIFT                (0)       /* Bits 0-31: Address Encode Invert (AEI) */
#define FMU_UA0_MASK                 (0xffffffff << FMU_UA0_SHIFT)

/* UTest Address 1 (UA1) */

#define FMU_UA1_SHIFT                (0)       /* Bits 0-19: Address Encode Invert (AEI) */
#define FMU_UA1_MASK                 (0x0fffff << FMU_UA1_SHIFT)
                                               /* Bits 20-31: Reserved */

/* Express Module Configuration Register (XMCR) */

#define FMU_XMCR_XEHV                (1 << 0)  /* Bit 0: Express Enable High Voltage (XEHV) */
                                               /* Bits 1-7: Reserved */
#define FMU_XMCR_XPGM                (1 << 8)  /* Bit 8: Express Program (XPGM) */
#define FMU_XMCR_XEPEG               (1 << 9)  /* Bit 9: Express ECC Enabled Program Good (XEPEG) */
                                               /* Bit 10: Reserved */
#define FMU_XMCR_XWDIE               (1 << 11) /* Bit 11: Express Watch Dog Interrupt Enable (XWDIE) */
#define FMU_XMCR_XWDI                (1 << 12) /* Bit 12: Express Watch Dog Interrupt (XWDI) */
#define FMU_XMCR_XDOK                (1 << 13) /* Bit 13: Express Data OK (XDOK) */
#define FMU_XMCR_XPEG                (1 << 14) /* Bit 14: Express Program Good (XPEG) */
#define FMU_XMCR_XDONE               (1 << 15) /* Bit 15: Express State Machine Status (XDONE) */
#define FMU_XMCR_XPEID_SHIFT         (16)      /* Bits 16-23: Express Program Master/Domain ID */
#define FMU_XMCR_XPEID_MASK          (0xff << FMU_XMCR_XPEID_SHIFT)
                                               /* Bits 24-31: Reserved */

/* Express Program Address Register (XPEADR) */

                                               /* Bits 0-4: Reserved */
#define FMU_XPEADR_XPEADDR_SHIFT     (5)       /* Bits 5-18: Express Program Address (XPEADDR) */
#define FMU_XPEADR_XPEADDR_MASK      (0x3fff << FMU_XPEADR_XPEADDR_SHIFT)
#define FMU_XPEADR_XPEA0             (1 << 19) /* Bit 19: Express Program and Erase Address Region 0 (XPEA0) */
#define FMU_XPEADR_XPEA1             (1 << 20) /* Bit 20: Express Program and Erase Address Region 1 (XPEA1) */
#define FMU_XPEADR_XPEA2             (1 << 21) /* Bit 21: Express Program and Erase Address Region 2 (XPEA2) */
#define FMU_XPEADR_XPEA3             (1 << 22) /* Bit 22: Express Program and Erase Address Region 3 (XPEA3) */
#define FMU_XPEADR_XPEA4             (1 << 23) /* Bit 23: Express Program and Erase Address Region 4 (XPEA4) */
#define FMU_XPEADR_XPEA5             (1 << 24) /* Bit 24: Express Program and Erase Address Region 5 (XPEA5) */
                                               /* Bits 25-31: Reserved */

/* Program Data (DATAn) */

#define FMU_DATA_SHIFT               (0)       /* Bits 0-31: Program Data (PDATA) */
#define FMU_DATA_MASK                (0xffffffff << FMU_DATA_SHIFT)

#endif /* __ARCH_ARM_SRC_S32K3XX_HARDWARE_S32K3XX_FMU_H */
