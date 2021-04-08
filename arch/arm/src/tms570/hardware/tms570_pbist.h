/****************************************************************************
 * arch/arm/src/tms570/hardware/tms570_pbist.h
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

/* References:
 * TMS570LS04x/03x 16/32-Bit RISC Flash Microcontroller,
 * Technical Reference Manual, Texas Instruments,
 * Literature Number: SPNU517A, September 2013
 */

#ifndef __ARCH_ARM_SRC_TMS570_HARDWARE_TMS570_PBIST_H
#define __ARCH_ARM_SRC_TMS570_HARDWARE_TMS570_PBIST_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/tms570_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* PBIST RAM Groups */

#define PBIST_PBIST_ROM_GROUP          1   /* ROM */
#define PBIST_STC_ROM_GROUP            2   /* ROM */
#define PBIST_DCAN1_RAM_GROUP          3   /* Dual-port */
#define PBIST_DCAN2_RAM_GROUP          4   /* Dual-port */
#define PBIST_ESRAM1_RAM_GROUP         6   /* Single-port */
#define PBIST_MIBSPI1_RAM_GROUP        7   /* Dual-port */
#define PBIST_VIM_RAM_GROUP            10  /* Dual-port */
#define PBIST_MIBADC_RAM_GROUP         11  /* Dual-port */
#define PBIST_N2HET_RAM_GROUP          13  /* Dual-port */
#define PBIST_HET_TU_RAM_GROUP         14  /* Dual-port */

/* RAM Group Select */

#define PBIST_PBIST_ROM_RGS            1   /* ROM */
#define PBIST_STC_ROM_RGS              2   /* ROM */
#define PBIST_DCAN1_RAM_RGS            3   /* Dual-port */
#define PBIST_DCAN2_RAM_RGS            4   /* Dual-port */
#define PBIST_ESRAM1_RAM_RGS           6   /* Single-port */
#define PBIST_MIBSPI1_RAM_RGS          7   /* Dual-port */
#define PBIST_VIM_RAM_RGS              8   /* Dual-port */
#define PBIST_MIBADC_RAM_RGS           9   /* Dual-port */
#define PBIST_N2HET_RAM_RGS            11  /* Dual-port */
#define PBIST_HET_TU_RAM_RGS           12  /* Dual-port */

/* Register Offsets *********************************************************/

#define TMS570_PBIST_RAMT_OFFSET       0x0160 /* RAM Configuration Register */
#define TMS570_PBIST_DLR_OFFSET        0x0164 /* Datalogger Register */
#define TMS570_PBIST_PCR_OFFSET        0x016c /* Program Control Register */
#define TMS570_PBIST_PACT_OFFSET       0x0180 /* PBIST Activate/ROM Clock Enable Register */
#define TMS570_PBIST_PBISTID_OFFSET    0x0184 /* PBIST ID Register */
#define TMS570_PBIST_OVER_OFFSET       0x0188 /* Override Register */
#define TMS570_PBIST_FSRF0_OFFSET      0x0190 /* Fail Status Fail Register 0 */
#define TMS570_PBIST_FSRF1_OFFSET      0x0194 /* Fail Status Fail Register 1 */
#define TMS570_PBIST_FSRC0_OFFSET      0x0198 /* Fail Status Count Register 0 */
#define TMS570_PBIST_FSRC1_OFFSET      0x019c /* Fail Status Count Register 1 */
#define TMS570_PBIST_FSRA0_OFFSET      0x01a0 /* Fail Status Address 0 Register */
#define TMS570_PBIST_FSRA1_OFFSET      0x01a4 /* Fail Status Address 1 Register */
#define TMS570_PBIST_FSRDL0_OFFSET     0x01a8 /* Fail Status Data Register 0 */
#define TMS570_PBIST_FSRDL1_OFFSET     0x01b0 /* Fail Status Data Register 1 */
#define TMS570_PBIST_ROM_OFFSET        0x01c0 /* ROM Mask Register */
#define TMS570_PBIST_ALGO_OFFSET       0x01c4 /* ROM Algorithm Mask Register */
#define TMS570_PBIST_RINFOL_OFFSET     0x01c8 /* RAM Info Mask Lower Register */
#define TMS570_PBIST_RINFOU_OFFSET     0x01cc /* RAM Info Mask Upper Register */

/* Register Addresses *******************************************************/

#define TMS570_PBIST_RAMT              (TMS570_PBIST_BASE+TMS570_PBIST_RAMT_OFFSET)
#define TMS570_PBIST_DLR               (TMS570_PBIST_BASE+TMS570_PBIST_DLR_OFFSET)
#define TMS570_PBIST_PCR               (TMS570_PBIST_BASE+TMS570_PBIST_PCR_OFFSET)
#define TMS570_PBIST_PACT              (TMS570_PBIST_BASE+TMS570_PBIST_PACT_OFFSET)
#define TMS570_PBIST_PBISTID           (TMS570_PBIST_BASE+TMS570_PBIST_PBISTID_OFFSET)
#define TMS570_PBIST_OVER              (TMS570_PBIST_BASE+TMS570_PBIST_OVER_OFFSET)
#define TMS570_PBIST_FSRF0             (TMS570_PBIST_BASE+TMS570_PBIST_FSRF0_OFFSET)
#define TMS570_PBIST_FSRF1             (TMS570_PBIST_BASE+TMS570_PBIST_FSRF1_OFFSET)
#define TMS570_PBIST_FSRC0             (TMS570_PBIST_BASE+TMS570_PBIST_FSRC0_OFFSET)
#define TMS570_PBIST_FSRC1             (TMS570_PBIST_BASE+TMS570_PBIST_FSRC1_OFFSET)
#define TMS570_PBIST_FSRA0             (TMS570_PBIST_BASE+TMS570_PBIST_FSRA0_OFFSET)
#define TMS570_PBIST_FSRA1             (TMS570_PBIST_BASE+TMS570_PBIST_FSRA1_OFFSET)
#define TMS570_PBIST_FSRDL0            (TMS570_PBIST_BASE+TMS570_PBIST_FSRDL0_OFFSET)
#define TMS570_PBIST_FSRDL1            (TMS570_PBIST_BASE+TMS570_PBIST_FSRDL1_OFFSET)
#define TMS570_PBIST_ROM               (TMS570_PBIST_BASE+TMS570_PBIST_ROM_OFFSET)
#define TMS570_PBIST_ALGO              (TMS570_PBIST_BASE+TMS570_PBIST_ALGO_OFFSET)
#define TMS570_PBIST_RINFOL            (TMS570_PBIST_BASE+TMS570_PBIST_RINFOL_OFFSET)
#define TMS570_PBIST_RINFOU            (TMS570_PBIST_BASE+TMS570_PBIST_RINFOU_OFFSET)

/* Register Bit-Field Definitions *******************************************/

/* RAM Configuration Register */

#define PBIST_RAMT_RLS_SHIFT           (0)       /* Bits 0-1: RAM Latency Select */
#define PBIST_RAMT_RLS_MASK            (3 << PBIST_RAMT_RLS_SHIFT)
#  define PBIST_RAMT_RLS(n)            ((uint32_t)(n) << PBIST_RAMT_RLS_SHIFT)
#define PBIST_RAMT_PLS_SHIFT           (2)       /* Bits 2-5: Pipeline Latency Select */
#define PBIST_RAMT_PLS_MASK            (15 << PBIST_RAMT_PLS_SHIFT)
#  define PBIST_RAMT_PLS(n)            ((uint32_t)(n) << PBIST_RAMT_PLS_SHIFT)
#define PBIST_RAMT_SMS_SHIFT           (6)       /* Bits 6-7: Sense Margin Select Register */
#define PBIST_RAMT_SMS_MASK            (3 << PBIST_RAMT_SMS_SHIFT)
#  define PBIST_RAMT_SMS(n)            ((uint32_t)(n) << PBIST_RAMT_SMS_SHIFT)
#define PBIST_RAMT_DWR_SHIFT           (8)       /* Bits 8-15: Data Width Register */
#define PBIST_RAMT_DWR_MASK            (0xff << PBIST_RAMT_DWR_SHIFT)
#  define PBIST_RAMT_DWR(n)            ((uint32_t)(n) << PBIST_RAMT_DWR_SHIFT)
#define PBIST_RAMT_RDS_SHIFT           (16)      /* Bits 16-23: Return Data Select */
#define PBIST_RAMT_RDS_MASK            (0xff << PBIST_RAMT_RDS_SHIFT)
#  define PBIST_RAMT_RDS(n)            ((uint32_t)(n) << PBIST_RAMT_RDS_SHIFT)
#define PBIST_RAMT_RGS_SHIFT           (14)      /* Bits 24-31: Ram Group Select */
#define PBIST_RAMT_RGS_MASK            (0xff << PBIST_RAMT_RGS_SHIFT)
#  define PBIST_RAMT_RGS(n)            ((uint32_t)(n) << PBIST_RAMT_RGS_SHIFT)
#  define PBIST_RAMT_RGS_PBIST_ROM     (PBIST_PBIST_ROM_RGS << PBIST_RAMT_RGS_SHIFT)
#  define PBIST_RAMT_RGS_STC_ROM       (PBIST_STC_ROM_RGS << PBIST_RAMT_RGS_SHIFT)
#  define PBIST_RAMT_RGS_DCAN1_RAM     (PBIST_DCAN1_RAM_RGS << PBIST_RAMT_RGS_SHIFT)
#  define PBIST_RAMT_RGS_DCAN2_RAM     (PBIST_DCAN2_RAM_RGS << PBIST_RAMT_RGS_SHIFT)
#  define PBIST_RAMT_RGS_ESRAM1_RAM    (PBIST_ESRAM1_RAM_RGS << PBIST_RAMT_RGS_SHIFT)
#  define PBIST_RAMT_RGS_MIBSPI1_RAM   (PBIST_MIBSPI1_RAM_RGS << PBIST_RAMT_RGS_SHIFT)
#  define PBIST_RAMT_RGS_VIM_RAM       (PBIST_VIM_RAM_RGS << PBIST_RAMT_RGS_SHIFT)
#  define PBIST_RAMT_RGS_MIBADC_RAM    (PBIST_MIBADC_RAM_RGS << PBIST_RAMT_RGS_SHIFT)
#  define PBIST_RAMT_RGS_N2HET_RAM     (PBIST_N2HET_RAM_RGS << PBIST_RAMT_RGS_SHIFT)
#  define PBIST_RAMT_RGS_HET_TU_RAM    (PBIST_HET_TU_RAM_RGS << PBIST_RAMT_RGS_SHIFT)

/* Datalogger Register */

#define PBIST_DLR_DLR2                 (1 << 2)  /* Bit 2:  ROM-based testing */
#define PBIST_DLR_DLR4                 (1 << 4)  /* Bit 4:  Configuration access */

/* Program Control Register */

#define PBIST_PCR_STR_SHIFT            (0)       /* Bits 0-4: PBIST Controller Mode */
#define PBIST_PCR_STR_MASK             (0x1f << PBIST_PCR_STR_SHIFT)
#  define PBIST_PCR_STR_START          (1 << PBIST_PCR_STR_SHIFT)  /* Start / Time Stamp mode restart */
#  define PBIST_PCR_STR_RESUME         (2 << PBIST_PCR_STR_SHIFT)  /* Resume / Emulation read */
#  define PBIST_PCR_STR_STOP           (4 << PBIST_PCR_STR_SHIFT)  /* Stop */
#  define PBIST_PCR_STR_STEP           (8 << PBIST_PCR_STR_SHIFT)  /* Step / Step for emulation mode */
#  define PBIST_PCR_STR_MISR           (16 << PBIST_PCR_STR_SHIFT) /* Check MISR mode */

/* PBIST Activate/ROM Clock Enable Register */

#define PBIST_PACT_PACT0               (1 << 0)  /* Bit 0: ROM Clock Enable */
#define PBIST_PACT_PACT1               (1 << 1)  /* Bit 1: PBIST Activate */

/* PBIST ID Register */

#define PBIST_PBISTID_SHIFT            (0)       /* Bits 0-7: PBIST controller ID */
#define PBIST_PBISTID_MASK             (0xff << PBIST_PBISTID_SHIFT)
#  define PBIST_PBISTID(n)             ((uint32_t)(n) << PBIST_PBISTID_SHIFT)

/* Override Register */

#define PBIST_OVER_OVER0               (1 << 0)  /* Bit 0: RINFO Override Bit */

/* Fail Status Fail Register 0/1 */

#define PBIST_FSRF                     (1 << 0)  /* Bit 0:  Fail Status */

/* Fail Status Count Register 0/1 */

#define PBIST_FSRC_SHIFT               (0)       /* Bits 0-7: Failure status count */
#define PBIST_FSRC_MASK                (0xff << PBIST_FSRC0_SHIFT)

/* Fail Status Address 0/1 Register */

#define PBIST_FSRA_SHIFT               (0)       /* Bits 0-15: Failure status address */
#define PBIST_FSRA_MASK                (0xffff << PBIST_FSRA_SHIFT)

/* Fail Status Data Register 0/1 (32-bit data) */

/* ROM Mask Register */

#define PBIST_ROM_SHIFT                (0)    /* Bits 0-1: ROM Mask */
#define PBIST_ROM_MASK                 (3 << PBIST_ROM_SHIFT)
#  define PBIST_ROM_NONE               (0 << PBIST_ROM_SHIFT) /* No information used from ROM */
#  define PBIST_ROM_RAMINFO            (1 << PBIST_ROM_SHIFT) /* Only RAM Group information from ROM */
#  define PBIST_ROM_ALGOINFO           (2 << PBIST_ROM_SHIFT) /* Only Algorithm information from ROM */
#  define PBIST_ROM_BOTH               (3 << PBIST_ROM_SHIFT) /* Both Algorithm and RAM information from ROM */

/* ROM Algorithm Mask Register */

#define PBIST_ALGO_TripleReadSlow      (1 << 0)
#define PBIST_ALGO_TripleReadFast      (1 << 1)
#define PBIST_ALGO_MARCH13N_DP         (1 << 2)
#define PBIST_ALGO_MARCH13N_SP         (1 << 3)
#define PBIST_ALGO_DOWN1a_DP           (1 << 4)
#define PBIST_ALGO_DOWN1a_SP           (1 << 5)
#define PBIST_ALGO_MapColumn_DP        (1 << 6)
#define PBIST_ALGO_MapColumn_SP        (1 << 7)
#define PBIST_ALGO_Precharge_DP        (1 << 8)
#define PBIST_ALGO_Precharge_SP        (1 << 9)
#define PBIST_ALGO_DTXN2a_DP           (1 << 10)
#define PBIST_ALGO_DTXN2a_SP           (1 << 11)
#define PBIST_ALGO_PMOSOpen_DP         (1 << 12)
#define PBIST_ALGO_PMOSOpen_SP         (1 << 13)
#define PBIST_ALGO_PPMOSOpenSlice1_DP  (1 << 14)
#define PBIST_ALGO_PPMOSOpenSlice1_SP  (1 << 15)
#define PBIST_ALGO_PPMOSOpenSlice2_DP  (1 << 16)
#define PBIST_ALGO_PPMOSOpenSlice2_SP  (1 << 17)

/* RAM Info Mask Lower Register */

#define PBIST_RINFOL(n)                (1 << ((n)-1)) /* Bit n: Select RAM group n+1 */
#  define PBIST_RINFOL_PBIST_ROM       PBIST_RINFOL(PBIST_PBIST_ROM_GROUP)
#  define PBIST_RINFOL_STC_ROM         PBIST_RINFOL(PBIST_STC_ROM_GROUP)
#  define PBIST_RINFOL_DCAN1_RAM       PBIST_RINFOL(PBIST_DCAN1_RAM_GROUP)
#  define PBIST_RINFOL_DCAN2_RAM       PBIST_RINFOL(PBIST_DCAN2_RAM_GROUP)
#  define PBIST_RINFOL_ESRAM1_RAM      PBIST_RINFOL(PBIST_ESRAM1_RAM_GROUP)
#  define PBIST_RINFOL_MIBSPI1_RAM     PBIST_RINFOL(PBIST_MIBSPI1_RAM_GROUP)
#  define PBIST_RINFOL_VIM_RAM         PBIST_RINFOL(PBIST_VIM_RAM_GROUP)
#  define PBIST_RINFOL_MIBADC_RAM      PBIST_RINFOL(PBIST_MIBADC_RAM_GROUP)
#  define PBIST_RINFOL_N2HET_RAM       PBIST_RINFOL(PBIST_N2HET_RAM_GROUP)
#  define PBIST_RINFOL_HET_TU_RAM      PBIST_RINFOL(PBIST_HET_TU_RAM_GROUP)

/* RAM Info Mask Upper Register */

#endif /* __ARCH_ARM_SRC_TMS570_HARDWARE_TMS570_PBIST_H */
