/****************************************************************************
 * arch/arm/src/sama5/hardware/sam_hsmc.h
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

#ifndef __ARCH_ARM_SRC_SAMA5_HARDWARE_SAM_HSMC_H
#define __ARCH_ARM_SRC_SAMA5_HARDWARE_SAM_HSMC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/sam_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Chip Select Definitions **************************************************/

#define HSMC_CS0                      0
#define HSMC_CS1                      1
#define HSMC_CS2                      2
#define HSMC_CS3                      3

#define NFCSRAM_BASE                  SAM_NFCSRAM_VSECTION
#define NFCCMD_BASE                   SAM_NFCCR_VSECTION

/* SMC Register Offsets *****************************************************/

#define SAM_HSMC_CFG_OFFSET           0x0000 /* HSMC NFC Configuration Register */
#define SAM_HSMC_CTRL_OFFSET          0x0004 /* HSMC NFC Control Register */
#define SAM_HSMC_SR_OFFSET            0x0008 /* HSMC NFC Status Register */
#define SAM_HSMC_IER_OFFSET           0x000c /* HSMC NFC Interrupt Enable Register */
#define SAM_HSMC_IDR_OFFSET           0x0010 /* HSMC NFC Interrupt Disable Register */
#define SAM_HSMC_IMR_OFFSET           0x0014 /* HSMC NFC Interrupt Mask Register */
#define SAM_HSMC_ADDR_OFFSET          0x0018 /* HSMC NFC Address Cycle Zero Register */
#define SAM_HSMC_BANK_OFFSET          0x001c /* HSMC Bank Address Register */
                                             /* 0x0020-0x006c Reserved */
#define SAM_HSMC_PMECCFG_OFFSET       0x0070 /* PMECC Configuration Register */
#define SAM_HSMC_PMECCSAREA_OFFSET    0x0074 /* PMECC Spare Area Size Register */
#define SAM_HSMC_PMECCSADDR_OFFSET    0x0078 /* PMECC Start Address Register */
#define SAM_HSMC_PMECCEADDR_OFFSET    0x007c /* PMECC End Address Register */
                                             /* 0x0080 Reserved */
#define SAM_HSMC_PMECCTRL_OFFSET      0x0084 /* PMECC Control Register */
#define SAM_HSMC_PMECCSR_OFFSET       0x0088 /* PMECC Status Register */
#define SAM_HSMC_PMECCIER_OFFSET      0x008c /* PMECC Interrupt Enable Register */
#define SAM_HSMC_PMECCIDR_OFFSET      0x0090 /* PMECC Interrupt Disable Register */
#define SAM_HSMC_PMECCIMR_OFFSET      0x0094 /* PMECC Interrupt Mask Register */
#define SAM_HSMC_PMECCISR_OFFSET      0x0098 /* PMECC Interrupt Status Register */
                                             /* 0x009c-0x00ac Reserved */

#define SAM_HSMC_PMECC_OFFSET(n)      (0x00b0 + ((n) << 6)) /* PMECC sector offset */
#  define SAM_HSMC_PMECC0_OFFSET(n)   (0x00b0 + ((n) << 6)) /* PMECC Redundancy 0 Register */
#  define SAM_HSMC_PMECC1_OFFSET(n)   (0x00b4 + ((n) << 6)) /* PMECC Redundancy 1 Register */
#  define SAM_HSMC_PMECC2_OFFSET(n)   (0x00b8 + ((n) << 6)) /* PMECC Redundancy 2 Register */
#  define SAM_HSMC_PMECC3_OFFSET(n)   (0x00bc + ((n) << 6)) /* PMECC Redundancy 3 Register */
#  define SAM_HSMC_PMECC4_OFFSET(n)   (0x00c0 + ((n) << 6)) /* PMECC Redundancy 4 Register */
#  define SAM_HSMC_PMECC5_OFFSET(n)   (0x00c4 + ((n) << 6)) /* PMECC Redundancy 5 Register */
#  define SAM_HSMC_PMECC6_OFFSET(n)   (0x00c8 + ((n) << 6)) /* PMECC Redundancy 6 Register */
#  define SAM_HSMC_PMECC7_OFFSET(n)   (0x00cc + ((n) << 6)) /* PMECC Redundancy 7 Register */
#  define SAM_HSMC_PMECC8_OFFSET(n)   (0x00d0 + ((n) << 6)) /* PMECC Redundancy 8 Register */
#  define SAM_HSMC_PMECC9_OFFSET(n)   (0x00d4 + ((n) << 6)) /* PMECC Redundancy 9 Register */
#  define SAM_HSMC_PMECC10_OFFSET(n)  (0x00d8 + ((n) << 6)) /* PMECC Redundancy 10 Register */
#define SAM_HSMC_REM_OFFSET(n)        (0x02b0 + ((n) << 6)) /* PMECC Remainder offset */
#  define SAM_HSMC_REM0_OFFSET(n)     (0x02b0 + ((n) << 6)) /* PMECC Remainder 0 Register */
#  define SAM_HSMC_REM1_OFFSET(n)     (0x02b4 + ((n) << 6)) /* PMECC Remainder 1 Register */
#  define SAM_HSMC_REM2_OFFSET(n)     (0x02b8 + ((n) << 6)) /* PMECC Remainder 2 Register */
#  define SAM_HSMC_REM3_OFFSET(n)     (0x02bc + ((n) << 6)) /* PMECC Remainder 3 Register */
#  define SAM_HSMC_REM4_OFFSET(n)     (0x02b0 + ((n) << 6)) /* PMECC Remainder 4 Register */
#  define SAM_HSMC_REM5_OFFSET(n)     (0x02b4 + ((n) << 6)) /* PMECC Remainder 5 Register */
#  define SAM_HSMC_REM6_OFFSET(n)     (0x02b8 + ((n) << 6)) /* PMECC Remainder 6 Register */
#  define SAM_HSMC_REM7_OFFSET(n)     (0x02bc + ((n) << 6)) /* PMECC Remainder 7 Register */
#  define SAM_HSMC_REM8_OFFSET(n)     (0x02b0 + ((n) << 6)) /* PMECC Remainder 8 Register */
#  define SAM_HSMC_REM9_OFFSET(n)     (0x02b4 + ((n) << 6)) /* PMECC Remainder 9 Register */
#  define SAM_HSMC_REM10_OFFSET(n)    (0x02b8 + ((n) << 6)) /* PMECC Remainder 10 Register */
#  define SAM_HSMC_REM11_OFFSET(n)    (0x02bc + ((n) << 6)) /* PMECC Remainder 11 Register */

                                             /* 0x04a0-0x04fc Reserved */
#define SAM_HSMC_ELCFG_OFFSET         0x0500 /* PMECC Error Location Configuration Register */
#define SAM_HSMC_ELPRIM_OFFSET        0x0504 /* PMECC Error Location Primitive Register */
#define SAM_HSMC_ELEN_OFFSET          0x0508 /* PMECC Error Location Enable Register */
#define SAM_HSMC_ELDIS_OFFSET         0x050c /* PMECC Error Location Disable Register */
#define SAM_HSMC_ELSR_OFFSET          0x0510 /* PMECC Error Location Status Register */
#define SAM_HSMC_ELIER_OFFSET         0x0514 /* PMECC Error Location Interrupt Enable Register */
#define SAM_HSMC_ELIDR_OFFSET         0x0518 /* PMECC Error Location Interrupt Disable Register */
#define SAM_HSMC_ELIMR_OFFSET         0x051c /* PMECC Error Location Interrupt Mask Register */
#define SAM_HSMC_ELISR_OFFSET         0x0520 /* PMECC Error Location Interrupt Status Register */

                                             /* 0x0524-0x052c Reserved */
#define SAM_HSMC_SIGMA_OFFSET(n)      (0x0528 + ((n) << 2)) /* PMECC Error Location SIGMA n Register */

#  define SAM_HSMC_SIGMA0_OFFSET      0x0528 /* PMECC Error Location SIGMA 0 Register */
#  define SAM_HSMC_SIGMA1_OFFSET      0x052c /* PMECC Error Location SIGMA 1 Register */
#  define SAM_HSMC_SIGMA2_OFFSET      0x0530 /* PMECC Error Location SIGMA 2 Register */
#  define SAM_HSMC_SIGMA3_OFFSET      0x0534 /* PMECC Error Location SIGMA 3 Register */
#  define SAM_HSMC_SIGMA4_OFFSET      0x0538 /* PMECC Error Location SIGMA 4 Register */
#  define SAM_HSMC_SIGMA5_OFFSET      0x053c /* PMECC Error Location SIGMA 5 Register */
#  define SAM_HSMC_SIGMA6_OFFSET      0x0540 /* PMECC Error Location SIGMA 6 Register */
#  define SAM_HSMC_SIGMA7_OFFSET      0x0544 /* PMECC Error Location SIGMA 7 Register */
#  define SAM_HSMC_SIGMA8_OFFSET      0x0548 /* PMECC Error Location SIGMA 8 Register */
#  define SAM_HSMC_SIGMA9_OFFSET      0x054c /* PMECC Error Location SIGMA 9 Register */
#  define SAM_HSMC_SIGMA10_OFFSET     0x0550 /* PMECC Error Location SIGMA 10 Register */
#  define SAM_HSMC_SIGMA11_OFFSET     0x0554 /* PMECC Error Location SIGMA 11 Register */
#  define SAM_HSMC_SIGMA12_OFFSET     0x0558 /* PMECC Error Location SIGMA 12 Register */
#  define SAM_HSMC_SIGMA13_OFFSET     0x055c /* PMECC Error Location SIGMA 13 Register */
#  define SAM_HSMC_SIGMA14_OFFSET     0x0560 /* PMECC Error Location SIGMA 14 Register */
#  define SAM_HSMC_SIGMA15_OFFSET     0x0564 /* PMECC Error Location SIGMA 15 Register */
#  define SAM_HSMC_SIGMA16_OFFSET     0x0568 /* PMECC Error Location SIGMA 16 Register */
#  define SAM_HSMC_SIGMA17_OFFSET     0x056c /* PMECC Error Location SIGMA 17 Register */
#  define SAM_HSMC_SIGMA18_OFFSET     0x0570 /* PMECC Error Location SIGMA 18 Register */
#  define SAM_HSMC_SIGMA19_OFFSET     0x0574 /* PMECC Error Location SIGMA 19 Register */
#  define SAM_HSMC_SIGMA20_OFFSET     0x0578 /* PMECC Error Location SIGMA 20 Register */
#  define SAM_HSMC_SIGMA21_OFFSET     0x057c /* PMECC Error Location SIGMA 21 Register */
#  define SAM_HSMC_SIGMA22_OFFSET     0x0580 /* PMECC Error Location SIGMA 22 Register */
#  define SAM_HSMC_SIGMA23_OFFSET     0x0584 /* PMECC Error Location SIGMA 23 Register */
#  define SAM_HSMC_SIGMA24_OFFSET     0x0588 /* PMECC Error Location SIGMA 24 Register */

#define SAM_HSMC_ERRLOC_OFFSET(n)     (0x058c + ((n) << 2)) /* PMECC Error Location n Register */

#  define SAM_HSMC_ERRLOC0_OFFSET     0x058c /* PMECC Error Location 0 Register */
#  define SAM_HSMC_ERRLOC1_OFFSET     0x0590 /* PMECC Error Location 1 Register */
#  define SAM_HSMC_ERRLOC2_OFFSET     0x0594 /* PMECC Error Location 2 Register */
#  define SAM_HSMC_ERRLOC3_OFFSET     0x0598 /* PMECC Error Location 3 Register */
#  define SAM_HSMC_ERRLOC4_OFFSET     0x059c /* PMECC Error Location 4 Register */
#  define SAM_HSMC_ERRLOC5_OFFSET     0x05a0 /* PMECC Error Location 5 Register */
#  define SAM_HSMC_ERRLOC6_OFFSET     0x05a4 /* PMECC Error Location 6 Register */
#  define SAM_HSMC_ERRLOC7_OFFSET     0x05a8 /* PMECC Error Location 7 Register */
#  define SAM_HSMC_ERRLOC8_OFFSET     0x05ac /* PMECC Error Location 8 Register */
#  define SAM_HSMC_ERRLOC9_OFFSET     0x05b0 /* PMECC Error Location 9 Register */
#  define SAM_HSMC_ERRLOC10_OFFSET    0x05b4 /* PMECC Error Location 10 Register */
#  define SAM_HSMC_ERRLOC11_OFFSET    0x05b0 /* PMECC Error Location 11 Register */
#  define SAM_HSMC_ERRLOC12_OFFSET    0x05bc /* PMECC Error Location 12 Register */
#  define SAM_HSMC_ERRLOC13_OFFSET    0x05c0 /* PMECC Error Location 13 Register */
#  define SAM_HSMC_ERRLOC14_OFFSET    0x05c4 /* PMECC Error Location 14 Register */
#  define SAM_HSMC_ERRLOC15_OFFSET    0x05c8 /* PMECC Error Location 15 Register */
#  define SAM_HSMC_ERRLOC16_OFFSET    0x05cc /* PMECC Error Location 16 Register */
#  define SAM_HSMC_ERRLOC17_OFFSET    0x05d0 /* PMECC Error Location 17 Register */
#  define SAM_HSMC_ERRLOC18_OFFSET    0x05d4 /* PMECC Error Location 18 Register */
#  define SAM_HSMC_ERRLOC19_OFFSET    0x05d8 /* PMECC Error Location 19 Register */
#  define SAM_HSMC_ERRLOC20_OFFSET    0x05dc /* PMECC Error Location 20 Register */
#  define SAM_HSMC_ERRLOC21_OFFSET    0x05e0 /* PMECC Error Location 21 Register */
#  define SAM_HSMC_ERRLOC22_OFFSET    0x05e4 /* PMECC Error Location 22 Register */
#  define SAM_HSMC_ERRLOC23_OFFSET    0x05e8 /* PMECC Error Location 23 Register */

                                             /* 0x05ec-0x05fc Reserved */
#define SAM_HSMC_SETUP_OFFSET(n)      (0x0600 + 0x14 * (n)) /* HSMC Setup Register */
#define SAM_HSMC_PULSE_OFFSET(n)      (0x0604 + 0x14 * (n)) /* HSMC Pulse Register */
#define SAM_HSMC_CYCLE_OFFSET(n)      (0x0608 + 0x14 * (n)) /* HSMC Cycle Register */
#define SAM_HSMC_TIMINGS_OFFSET(n)    (0x060c + 0x14 * (n)) /* HSMC Timings Register */
#define SAM_HSMC_MODE_OFFSET(n)       (0x0610 + 0x14 * (n)) /* HSMC Mode Register */

#define SAM_HSMC_OCMS_OFFSET          0x06a0 /* HSMC OCMS Register */
#define SAM_HSMC_KEY1_OFFSET          0x06a4 /* HSMC OCMS KEY1 Register */
#define SAM_HSMC_KEY2_OFFSET          0x06a8 /* HSMC OCMS KEY2 Register */
                                             /* 0x06ac-0x06e0 Reserved */
#define SAM_HSMC_WPMR_OFFSET          0x06e4 /* HSMC Write Protection Mode Register */
#define SAM_HSMC_WPSR_OFFSET          0x06e8 /* HSMC Write Protection Status Register */
                                             /* 0x06fc Reserved */

/* SMC Register Addresses ***************************************************/

#define SAM_HSMC_CFG                  (SAM_HSMC_VBASE+SAM_HSMC_CFG_OFFSET)
#define SAM_HSMC_CTRL                 (SAM_HSMC_VBASE+SAM_HSMC_CTRL_OFFSET)
#define SAM_HSMC_SR                   (SAM_HSMC_VBASE+SAM_HSMC_SR_OFFSET)
#define SAM_HSMC_IER                  (SAM_HSMC_VBASE+SAM_HSMC_IER_OFFSET)
#define SAM_HSMC_IDR                  (SAM_HSMC_VBASE+SAM_HSMC_IDR_OFFSET)
#define SAM_HSMC_IMR                  (SAM_HSMC_VBASE+SAM_HSMC_IMR_OFFSET)
#define SAM_HSMC_ADDR                 (SAM_HSMC_VBASE+SAM_HSMC_ADDR_OFFSET)
#define SAM_HSMC_BANK                 (SAM_HSMC_VBASE+SAM_HSMC_BANK_OFFSET)
#define SAM_HSMC_PMECCFG              (SAM_HSMC_VBASE+SAM_HSMC_PMECCFG_OFFSET)
#define SAM_HSMC_PMECCSAREA           (SAM_HSMC_VBASE+SAM_HSMC_PMECCSAREA_OFFSET)
#define SAM_HSMC_PMECCSADDR           (SAM_HSMC_VBASE+SAM_HSMC_PMECCSADDR_OFFSET)
#define SAM_HSMC_PMECCEADDR           (SAM_HSMC_VBASE+SAM_HSMC_PMECCEADDR_OFFSET)
#define SAM_HSMC_PMECCTRL             (SAM_HSMC_VBASE+SAM_HSMC_PMECCTRL_OFFSET)
#define SAM_HSMC_PMECCSR              (SAM_HSMC_VBASE+SAM_HSMC_PMECCSR_OFFSET)
#define SAM_HSMC_PMECCIER             (SAM_HSMC_VBASE+SAM_HSMC_PMECCIER_OFFSET)
#define SAM_HSMC_PMECCIDR             (SAM_HSMC_VBASE+SAM_HSMC_PMECCIDR_OFFSET)
#define SAM_HSMC_PMECCIMR             (SAM_HSMC_VBASE+SAM_HSMC_PMECCIMR_OFFSET)
#define SAM_HSMC_PMECCISR             (SAM_HSMC_VBASE+SAM_HSMC_PMECCISR_OFFSET)
#define SAM_HSMC_PMECC_BASE(n)        (SAM_HSMC_VBASE+SAM_HSMC_PMECC_OFFSET(n))
#  define SAM_HSMC_PMECC0(n)          (SAM_HSMC_VBASE+SAM_HSMC_PMECC0_OFFSET(n))
#  define SAM_HSMC_PMECC1(n)          (SAM_HSMC_VBASE+SAM_HSMC_PMECC1_OFFSET(n))
#  define SAM_HSMC_PMECC2(n)          (SAM_HSMC_VBASE+SAM_HSMC_PMECC2_OFFSET(n))
#  define SAM_HSMC_PMECC3(n)          (SAM_HSMC_VBASE+SAM_HSMC_PMECC3_OFFSET(n))
#  define SAM_HSMC_PMECC4(n)          (SAM_HSMC_VBASE+SAM_HSMC_PMECC4_OFFSET(n))
#  define SAM_HSMC_PMECC5(n)          (SAM_HSMC_VBASE+SAM_HSMC_PMECC5_OFFSET(n))
#  define SAM_HSMC_PMECC6(n)          (SAM_HSMC_VBASE+SAM_HSMC_PMECC6_OFFSET(n))
#  define SAM_HSMC_PMECC7(n)          (SAM_HSMC_VBASE+SAM_HSMC_PMECC7_OFFSET(n))
#  define SAM_HSMC_PMECC8(n)          (SAM_HSMC_VBASE+SAM_HSMC_PMECC8_OFFSET(n))
#  define SAM_HSMC_PMECC9(n)          (SAM_HSMC_VBASE+SAM_HSMC_PMECC9_OFFSET(n))
#  define SAM_HSMC_PMECC10(n)         (SAM_HSMC_VBASE+SAM_HSMC_PMECC10_OFFSET(n))
#define SAM_HSMC_REM_BASE(n)          (SAM_HSMC_VBASE+SAM_HSMC_REM_OFFSET(n))
#  define SAM_HSMC_REM0(n)            (SAM_HSMC_VBASE+SAM_HSMC_REM0_OFFSET(n))
#  define SAM_HSMC_REM1(n)            (SAM_HSMC_VBASE+SAM_HSMC_REM1_OFFSET(n))
#  define SAM_HSMC_REM2(n)            (SAM_HSMC_VBASE+SAM_HSMC_REM2_OFFSET(n))
#  define SAM_HSMC_REM3(n)            (SAM_HSMC_VBASE+SAM_HSMC_REM3_OFFSET(n))
#  define SAM_HSMC_REM4(n)            (SAM_HSMC_VBASE+SAM_HSMC_REM4_OFFSET(n))
#  define SAM_HSMC_REM5(n)            (SAM_HSMC_VBASE+SAM_HSMC_REM5_OFFSET(n))
#  define SAM_HSMC_REM6(n)            (SAM_HSMC_VBASE+SAM_HSMC_REM6_OFFSET(n))
#  define SAM_HSMC_REM7(n)            (SAM_HSMC_VBASE+SAM_HSMC_REM7_OFFSET(n))
#  define SAM_HSMC_REM8(n)            (SAM_HSMC_VBASE+SAM_HSMC_REM8_OFFSET(n))
#  define SAM_HSMC_REM9(n)            (SAM_HSMC_VBASE+SAM_HSMC_REM9_OFFSET(n))
#  define SAM_HSMC_REM10(n)           (SAM_HSMC_VBASE+SAM_HSMC_REM10_OFFSET(n))
#  define SAM_HSMC_REM11(n)           (SAM_HSMC_VBASE+SAM_HSMC_REM11_OFFSET(n))
#define SAM_HSMC_ELCFG                (SAM_HSMC_VBASE+SAM_HSMC_ELCFG_OFFSET)
#define SAM_HSMC_ELPRIM               (SAM_HSMC_VBASE+SAM_HSMC_ELPRIM_OFFSET)
#define SAM_HSMC_ELEN                 (SAM_HSMC_VBASE+SAM_HSMC_ELEN_OFFSET)
#define SAM_HSMC_ELDIS                (SAM_HSMC_VBASE+SAM_HSMC_ELDIS_OFFSET)
#define SAM_HSMC_ELSR                 (SAM_HSMC_VBASE+SAM_HSMC_ELSR_OFFSET)
#define SAM_HSMC_ELIER                (SAM_HSMC_VBASE+SAM_HSMC_ELIER_OFFSET)
#define SAM_HSMC_ELIDR                (SAM_HSMC_VBASE+SAM_HSMC_ELIDR_OFFSET)
#define SAM_HSMC_ELIMR                (SAM_HSMC_VBASE+SAM_HSMC_ELIMR_OFFSET)
#define SAM_HSMC_ELISR                (SAM_HSMC_VBASE+SAM_HSMC_ELISR_OFFSET)
#define SAM_HSMC_SIGMA_BASE(n)        (SAM_HSMC_VBASE+SAM_HSMC_SIGMA_OFFSET(n))
#  define SAM_HSMC_SIGMA0             (SAM_HSMC_VBASE+SAM_HSMC_SIGMA0_OFFSET)
#  define SAM_HSMC_SIGMA1             (SAM_HSMC_VBASE+SAM_HSMC_SIGMA1_OFFSET)
#  define SAM_HSMC_SIGMA2             (SAM_HSMC_VBASE+SAM_HSMC_SIGMA2_OFFSET)
#  define SAM_HSMC_SIGMA3             (SAM_HSMC_VBASE+SAM_HSMC_SIGMA3_OFFSET)
#  define SAM_HSMC_SIGMA4             (SAM_HSMC_VBASE+SAM_HSMC_SIGMA4_OFFSET)
#  define SAM_HSMC_SIGMA5             (SAM_HSMC_VBASE+SAM_HSMC_SIGMA5_OFFSET)
#  define SAM_HSMC_SIGMA6             (SAM_HSMC_VBASE+SAM_HSMC_SIGMA6_OFFSET)
#  define SAM_HSMC_SIGMA7             (SAM_HSMC_VBASE+SAM_HSMC_SIGMA7_OFFSET)
#  define SAM_HSMC_SIGMA8             (SAM_HSMC_VBASE+SAM_HSMC_SIGMA8_OFFSET)
#  define SAM_HSMC_SIGMA9             (SAM_HSMC_VBASE+SAM_HSMC_SIGMA9_OFFSET)
#  define SAM_HSMC_SIGMA10            (SAM_HSMC_VBASE+SAM_HSMC_SIGMA10_OFFSET)
#  define SAM_HSMC_SIGMA11            (SAM_HSMC_VBASE+SAM_HSMC_SIGMA11_OFFSET)
#  define SAM_HSMC_SIGMA12            (SAM_HSMC_VBASE+SAM_HSMC_SIGMA12_OFFSET)
#  define SAM_HSMC_SIGMA13            (SAM_HSMC_VBASE+SAM_HSMC_SIGMA13_OFFSET)
#  define SAM_HSMC_SIGMA14            (SAM_HSMC_VBASE+SAM_HSMC_SIGMA14_OFFSET)
#  define SAM_HSMC_SIGMA15            (SAM_HSMC_VBASE+SAM_HSMC_SIGMA15_OFFSET)
#  define SAM_HSMC_SIGMA16            (SAM_HSMC_VBASE+SAM_HSMC_SIGMA16_OFFSET)
#  define SAM_HSMC_SIGMA17            (SAM_HSMC_VBASE+SAM_HSMC_SIGMA17_OFFSET)
#  define SAM_HSMC_SIGMA18            (SAM_HSMC_VBASE+SAM_HSMC_SIGMA18_OFFSET)
#  define SAM_HSMC_SIGMA19            (SAM_HSMC_VBASE+SAM_HSMC_SIGMA19_OFFSET)
#  define SAM_HSMC_SIGMA20            (SAM_HSMC_VBASE+SAM_HSMC_SIGMA20_OFFSET)
#  define SAM_HSMC_SIGMA21            (SAM_HSMC_VBASE+SAM_HSMC_SIGMA21_OFFSET)
#  define SAM_HSMC_SIGMA22            (SAM_HSMC_VBASE+SAM_HSMC_SIGMA22_OFFSET)
#  define SAM_HSMC_SIGMA23            (SAM_HSMC_VBASE+SAM_HSMC_SIGMA23_OFFSET)
#  define SAM_HSMC_SIGMA24            (SAM_HSMC_VBASE+SAM_HSMC_SIGMA24_OFFSET)
#define SAM_HSMC_ERRLOC_BASE(n)       (SAM_HSMC_VBASE+SAM_HSMC_ERRLOC_OFFSET(n))
#  define SAM_HSMC_ERRLOC0            (SAM_HSMC_VBASE+SAM_HSMC_ERRLOC0_OFFSET)
#  define SAM_HSMC_ERRLOC1            (SAM_HSMC_VBASE+SAM_HSMC_ERRLOC1_OFFSET)
#  define SAM_HSMC_ERRLOC2            (SAM_HSMC_VBASE+SAM_HSMC_ERRLOC2_OFFSET)
#  define SAM_HSMC_ERRLOC3            (SAM_HSMC_VBASE+SAM_HSMC_ERRLOC3_OFFSET)
#  define SAM_HSMC_ERRLOC4            (SAM_HSMC_VBASE+SAM_HSMC_ERRLOC4_OFFSET)
#  define SAM_HSMC_ERRLOC5            (SAM_HSMC_VBASE+SAM_HSMC_ERRLOC5_OFFSET)
#  define SAM_HSMC_ERRLOC6            (SAM_HSMC_VBASE+SAM_HSMC_ERRLOC6_OFFSET)
#  define SAM_HSMC_ERRLOC7            (SAM_HSMC_VBASE+SAM_HSMC_ERRLOC7_OFFSET)
#  define SAM_HSMC_ERRLOC8            (SAM_HSMC_VBASE+SAM_HSMC_ERRLOC8_OFFSET)
#  define SAM_HSMC_ERRLOC9            (SAM_HSMC_VBASE+SAM_HSMC_ERRLOC9_OFFSET)
#  define SAM_HSMC_ERRLOC10           (SAM_HSMC_VBASE+SAM_HSMC_ERRLOC10_OFFSET)
#  define SAM_HSMC_ERRLOC11           (SAM_HSMC_VBASE+SAM_HSMC_ERRLOC11_OFFSET)
#  define SAM_HSMC_ERRLOC12           (SAM_HSMC_VBASE+SAM_HSMC_ERRLOC12_OFFSET)
#  define SAM_HSMC_ERRLOC13           (SAM_HSMC_VBASE+SAM_HSMC_ERRLOC13_OFFSET)
#  define SAM_HSMC_ERRLOC14           (SAM_HSMC_VBASE+SAM_HSMC_ERRLOC14_OFFSET)
#  define SAM_HSMC_ERRLOC15           (SAM_HSMC_VBASE+SAM_HSMC_ERRLOC15_OFFSET)
#  define SAM_HSMC_ERRLOC16           (SAM_HSMC_VBASE+SAM_HSMC_ERRLOC16_OFFSET)
#  define SAM_HSMC_ERRLOC17           (SAM_HSMC_VBASE+SAM_HSMC_ERRLOC17_OFFSET)
#  define SAM_HSMC_ERRLOC18           (SAM_HSMC_VBASE+SAM_HSMC_ERRLOC18_OFFSET)
#  define SAM_HSMC_ERRLOC19           (SAM_HSMC_VBASE+SAM_HSMC_ERRLOC19_OFFSET)
#  define SAM_HSMC_ERRLOC20           (SAM_HSMC_VBASE+SAM_HSMC_ERRLOC20_OFFSET)
#  define SAM_HSMC_ERRLOC21           (SAM_HSMC_VBASE+SAM_HSMC_ERRLOC21_OFFSET)
#  define SAM_HSMC_ERRLOC22           (SAM_HSMC_VBASE+SAM_HSMC_ERRLOC22_OFFSET)
#  define SAM_HSMC_ERRLOC23           (SAM_HSMC_VBASE+SAM_HSMC_ERRLOC23_OFFSET)
#define SAM_HSMC_SETUP(n)             (SAM_HSMC_VBASE+SAM_HSMC_SETUP_OFFSET(n))
#define SAM_HSMC_PULSE(n)             (SAM_HSMC_VBASE+SAM_HSMC_PULSE_OFFSET(n))
#define SAM_HSMC_CYCLE(n)             (SAM_HSMC_VBASE+SAM_HSMC_CYCLE_OFFSET(n))
#define SAM_HSMC_TIMINGS(n)           (SAM_HSMC_VBASE+SAM_HSMC_TIMINGS_OFFSET(n))
#define SAM_HSMC_MODE(n)              (SAM_HSMC_VBASE+SAM_HSMC_MODE_OFFSET(n))
#define SAM_HSMC_OCMS                 (SAM_HSMC_VBASE+SAM_HSMC_OCMS_OFFSET)
#define SAM_HSMC_KEY1                 (SAM_HSMC_VBASE+SAM_HSMC_KEY1_OFFSET)
#define SAM_HSMC_KEY2                 (SAM_HSMC_VBASE+SAM_HSMC_KEY2_OFFSET)
#define SAM_HSMC_WPMR                 (SAM_HSMC_VBASE+SAM_HSMC_WPMR_OFFSET)
#define SAM_HSMC_WPSR                 (SAM_HSMC_VBASE+SAM_HSMC_WPSR_OFFSET)

/* SMC Register Bit Definitions *********************************************/

/* HSMC NFC Configuration Register */

#define HSMC_CFG_PAGESIZE_SHIFT       (0)       /* Bit 0-2: */
#define HSMC_CFG_PAGESIZE_MASK        (7 << HSMC_CFG_PAGESIZE_SHIFT)
#  define HSMC_CFG_PAGESIZE_512       (0 << HSMC_CFG_PAGESIZE_SHIFT) /* Main area 512 Bytes */
#  define HSMC_CFG_PAGESIZE_1024      (1 << HSMC_CFG_PAGESIZE_SHIFT) /* Main area 1024 Bytes */
#  define HSMC_CFG_PAGESIZE_2048      (2 << HSMC_CFG_PAGESIZE_SHIFT) /* Main area 2048 Bytes */
#  define HSMC_CFG_PAGESIZE_4096      (3 << HSMC_CFG_PAGESIZE_SHIFT) /* Main area 4096 Bytes */
#  define HSMC_CFG_PAGESIZE_8192      (4 << HSMC_CFG_PAGESIZE_SHIFT) /* Main area 8192 Bytes */

#define HSMC_CFG_WSPARE               (1 << 8)  /* Bit 8:  Write Spare Area */
#define HSMC_CFG_RSPARE               (1 << 9)  /* Bit 9:  Read Spare Area */
#define HSMC_CFG_EDGECTRL             (1 << 12) /* Bit 12: Rising/Falling Edge Detection Control */
#define HSMC_CFG_RBEDGE               (1 << 13) /* Bit 13: Ready/Busy Signal Edge Detection */
#define HSMC_CFG_DTOCYC_SHIFT         (16)      /* Bit 16-19: Data Timeout Cycle Number */
#define HSMC_CFG_DTOCYC_MASK          (15 << HSMC_CFG_DTOCYC_SHIFT)
#  define HSMC_CFG_DTOCYC(n)          ((uint32_t)(n) << HSMC_CFG_DTOCYC_SHIFT)
#define HSMC_CFG_DTOMUL_SHIFT         (20)      /* Bit 20-22: Data Timeout Multiplier */
#define HSMC_CFG_DTOMUL_MASK          (7 << HSMC_CFG_DTOMUL_SHIFT)
#  define HSMC_CFG_DTOMUL_1           (0 << HSMC_CFG_DTOMUL_SHIFT) /* DTOCYC */
#  define HSMC_CFG_DTOMUL_16          (1 << HSMC_CFG_DTOMUL_SHIFT) /* DTOCYC x 16 */
#  define HSMC_CFG_DTOMUL_128         (2 << HSMC_CFG_DTOMUL_SHIFT) /* DTOCYC x 128 */
#  define HSMC_CFG_DTOMUL_256         (3 << HSMC_CFG_DTOMUL_SHIFT) /* DTOCYC x 256 */
#  define HSMC_CFG_DTOMUL_1024        (4 << HSMC_CFG_DTOMUL_SHIFT) /* DTOCYC x 1024 */
#  define HSMC_CFG_DTOMUL_4096        (5 << HSMC_CFG_DTOMUL_SHIFT) /* DTOCYC x 4096 */
#  define HSMC_CFG_DTOMUL_65536       (6 << HSMC_CFG_DTOMUL_SHIFT) /* DTOCYC x 65536 */
#  define HSMC_CFG_DTOMUL_1048576     (7 << HSMC_CFG_DTOMUL_SHIFT) /* DTOCYC x 1048576 */

#define HSMC_CFG_NFCSPARESIZE_SHIFT   (24)      /* Bit 24-30: NAND Flash Spare Area Size */
#define HSMC_CFG_NFCSPARESIZE_MASK    (0x7f << HSMC_CFG_NFCSPARESIZE_SHIFT)
#  define HSMC_CFG_NFCSPARESIZE(n)    ((uint32_t)(n) << HSMC_CFG_NFCSPARESIZE_SHIFT)

/* HSMC NFC Control Register */

#define HSMC_CTRL_NFCEN               (1 << 0)  /* Bit 0:  NAND Flash Controller Enable */
#define HSMC_CTRL_NFCDIS              (1 << 1)  /* Bit 1:  NAND Flash Controller Disable */

/* HSMC NFC Status Register */

/* HSMC NFC Interrupt Enable Register */

/* HSMC NFC Interrupt Disable Register */

/* HSMC NFC Interrupt Mask Register */

#define HSMC_SR_SMCSTS                (1 << 0)  /* Bit 0:  NAND Flash Controller Status (SR only) */
#define HSMC_NFCINT_RB_RISE           (1 << 4)  /* Bit 4:  Ready Busy Rising Edge Detection Interrupt */
#define HSMC_NFCINT_RB_FALL           (1 << 5)  /* Bit 5:  Ready Busy Falling Edge Detection Interrupt */
#define HSMC_SR_NFCBUSY               (1 << 8)  /* Bit 8:  NFC Busy (SR only) */
#define HSMC_SR_NFCWR                 (1 << 11) /* Bit 11: NFC Write/Read Operation (SR only) */
#define HSMC_SR_NFCSID_SHIFT          (12)      /* Bits 12-14:  NFC Chip Select ID (SR only) */
#define HSMC_SR_NFCSID_MASK           (7 << HSMC_SR_NFCSID_SHIFT)
#define HSMC_NFCINT_XFRDONE           (1 << 16) /* Bit 16: Transfer Done Interrupt */
#define HSMC_NFCINT_CMDDONE           (1 << 17) /* Bit 17: Command Done Interrupt */
#define HSMC_NFCINT_DTOE              (1 << 20) /* Bit 20: Data Timeout Error Interrupt Enable */
#define HSMC_NFCINT_UNDEF             (1 << 21) /* Bit 21: Undefined Area Access Interrupt */
#define HSMC_NFCINT_AWB               (1 << 22) /* Bit 22: Accessing While Busy Interrupt */
#define HSMC_NFCINT_NFCASE            (1 << 23) /* Bit 23: NFC Access Size Error Interrupt */
#define HSMC_NFCINT_RBEDGE0           (1 << 24) /* Bit 24: Ready/Busy Line 0 Interrupt */

#define HSMC_NFCINT_ALL               (0x01f30030)

/* HSMC NFC Address Cycle Zero Register */

#define HSMC_ADDR_MASK                (0xff)    /* Bits 0-7: NAND Flash Array Address Cycle 0 */

/* HSMC Bank Address Register */

#define HSMC_BANK                     (1 << 0)  /* Bit 0: Bank Identifier */

/* PMECC Configuration Register */

#define HSMC_PMECCFG_BCHERR_SHIFT     (0)       /* Bit 0-2: Error Correcting Capability */
#define HSMC_PMECCFG_BCHERR_MASK      (7 << HSMC_PMECCFG_BCHERR_SHIFT)
#  define HSMC_PMECCFG_BCHERR_2       (0 << HSMC_PMECCFG_BCHERR_SHIFT) /* 2 errors */
#  define HSMC_PMECCFG_BCHERR_4       (1 << HSMC_PMECCFG_BCHERR_SHIFT) /* 4 errors */
#  define HSMC_PMECCFG_BCHERR_8       (2 << HSMC_PMECCFG_BCHERR_SHIFT) /* 8 errors */
#  define HSMC_PMECCFG_BCHERR_12      (3 << HSMC_PMECCFG_BCHERR_SHIFT) /* 12 errors */
#  define HSMC_PMECCFG_BCHERR_24      (4 << HSMC_PMECCFG_BCHERR_SHIFT) /* 24 errors */

#define HSMC_PMECCFG_SECTORSZ_SHIFT   (4)       /* Bit 4:  Sector Size */
#define HSMC_PMECCFG_SECTORSZ_MASK    (1 << HSMC_PMECCFG_SECTORSZ_SHIFT)
#  define HSMC_PMECCFG_SECTORSZ_512   (0 << HSMC_PMECCFG_SECTORSZ_SHIFT)
#  define HSMC_PMECCFG_SECTORSZ_1024  (1 << HSMC_PMECCFG_SECTORSZ_SHIFT)

#define HSMC_PMECCFG_PAGESIZE_SHIFT   (8)       /* Bit 8-9: Number of Sectors in the Page */
#define HSMC_PMECCFG_PAGESIZE_MASK    (3 << HSMC_PMECCFG_PAGESIZE_SHIFT)
#  define HSMC_PMECCFG_PAGESIZE_1SEC  (0 << HSMC_PMECCFG_PAGESIZE_SHIFT) /* 1 sector (5121K) */
#  define HSMC_PMECCFG_PAGESIZE_2SEC  (1 << HSMC_PMECCFG_PAGESIZE_SHIFT) /* 2 sectors (1/2K) */
#  define HSMC_PMECCFG_PAGESIZE_4SEC  (2 << HSMC_PMECCFG_PAGESIZE_SHIFT) /* 4 sectors (2/4K) */
#  define HSMC_PMECCFG_PAGESIZE_8SEC  (3 << HSMC_PMECCFG_PAGESIZE_SHIFT) /* 8 sectors (4/8K) */

#define HSMC_PMECCFG_NANDWR_SHIFT     (12)      /* Bit 12: NAND Write Access */
#define HSMC_PMECCFG_NANDWR_MASK      (1 << HSMC_PMECCFG_NANDWR_SHIFT)
#  define HSMC_PMECCFG_NANDWR_READ    (0 << HSMC_PMECCFG_NANDWR_SHIFT)
#  define HSMC_PMECCFG_NANDWR_WRITE   (1 << HSMC_PMECCFG_NANDWR_SHIFT)
#define HSMC_PMECCFG_SPAREEN_SHIFT    (16)      /* Bit 16: Spare Enable */
#define HSMC_PMECCFG_SPAREEN_MASK     (1 << HSMC_PMECCFG_SPAREEN_SHIFT)
#  define HSMC_PMECCFG_SPARE_DISABLE  (0 << HSMC_PMECCFG_SPAREEN_SHIFT)
#  define HSMC_PMECCFG_SPARE_ENABLE   (1 << HSMC_PMECCFG_SPAREEN_SHIFT)
#define HSMC_PMECCFG_AUTO_SHIFT       (20)      /* Bit 20: Automatic Mode Enable */
#define HSMC_PMECCFG_AUTO_MASK        (1 << HSMC_PMECCFG_AUTO_SHIFT)
#  define HSMC_PMECCFG_AUTO_DISABLE   (0 << HSMC_PMECCFG_AUTO_SHIFT)
#  define HSMC_PMECCFG_AUTO_ENABLE    (1 << HSMC_PMECCFG_AUTO_SHIFT)

/* PMECC Spare Area Size Register */

#define HSMC_PMECCSAREA_MASK          (0x1ff)   /* Bits 0-8: Spare Area Size */

/* PMECC Start Address Register */

#define HSMC_PMECCSADDR_MASK          (0x1ff)   /* Bits 0-8: ECC Area Start Address */

/* PMECC End Address Register */

#define HSMC_PMECCEADDR_MASK          (0x1ff)   /* Bits 0-8: ECC Area End Address */

/* PMECC Control Register */

#define HSMC_PMECCTRL_RST             (1 << 0)  /* Bit 0:  Reset the PMECC Module */
#define HSMC_PMECCTRL_DATA            (1 << 1)  /* Bit 1:  Start a Data Phase */
#define HSMC_PMECCTRL_USER            (1 << 2)  /* Bit 2:  Start a User Mode Phase */
#define HSMC_PMECCTRL_ENABLE          (1 << 4)  /* Bit 4:  PMECC Enable */
#define HSMC_PMECCTRL_DISABLE         (1 << 5)  /* Bit 5:  PMECC Enable */

/* PMECC Status Register */

#define HSMC_PMECCSR_BUSY             (1 << 0)  /* Bit 0:  The kernel of the PMECC is busy */
#define HSMC_PMECCSR_ENABLE           (1 << 4)  /* Bit 4:  PMECC Enable bit */

/* PMECC Interrupt Enable Register */

/* PMECC Interrupt Disable Register */

/* PMECC Interrupt Mask Register */

#define HSMC_PMECCINT_ERRI            (1 << 0)  /* Bit 0:  Error Interrupt */

/* PMECC Interrupt Status Register */

#define HSMC_PMECCISR_ERRIS_SHIFT     (0)      /* Bits 0-7: Error Interrupt Status */
#define HSMC_PMECCISR_ERRIS_MASK      (0xff << HSMC_PMECCISR_ERRIS_SHIFT)
#  define HSMC_PMECCISR_ERRIS(n)      (1 << (n))  /* Bits n: Error on sector n */

/* PMECC Redundancy x Register (32-bit ECC value) */

/* PMECC Remainder x Register */

#define HSMC_REM_REM2NP1_SHIFT        (0)       /* Bit 0-13: BCH Remainder 2 * N + 1 */
#define HSMC_REM_REM2NP1_MASK         (0x3fff << HSMC_REM_REM2NP1_SHIFT)
#define HSMC_REM_REM2NP3_SHIFT        (16)      /* Bit 16-29: BCH Remainder 2 * N + 3 */
#define HSMC_REM_REM2NP3_MASK         (0x3fff << HSMC_REM_REM2NP3_SHIFT)

/* PMECC Error Location Configuration Register */

#define HSMC_ELCFG_SECTORSZ_SHIFT     (0)      /* Bit 0:  Sector Size */
#define HSMC_ELCFG_SECTORSZ_MASK      (1 << HSMC_ELCFG_SECTORSZ_SHIFT)
#  define HSMC_ELCFG_SECTORSZ_512     (0 << HSMC_ELCFG_SECTORSZ_SHIFT)
#  define HSMC_ELCFG_SECTORSZ_1024    (1 << HSMC_ELCFG_SECTORSZ_SHIFT)
#define HSMC_ELCFG_ERRNUM_SHIFT       (16)      /* Bit 16-20: Number of Errors */
#define HSMC_ELCFG_ERRNUM_MASK        (0x1f << HSMC_ELCFG_ERRNUM_SHIFT)
#  define HSMC_ELCFG_ERRNUM(n)        ((uint32_t)(n) << HSMC_ELCFG_ERRNUM_SHIFT)

/* PMECC Error Location Primitive Register */

#define HSMC_ELPRIM_MASK              (0xffff)  /* Bits 0-15: Primitive Polynomial */

/* PMECC Error Location Enable Register */

#define HSMC_ELEN_MASK                (0x3fff)  /* Bits 0-13: Error Location Enable */

/* PMECC Error Location Disable Register */

#define HSMC_ELDIS_DIS                (1 << 0)  /* Bit 0:  Disable Error Location Engine */

/* PMECC Error Location Status Register */

#define HSMC_ELSR_BUSY                (1 << 0)  /* Bit 0:  Error Location Engine Busy */

/* PMECC Error Location Interrupt Enable Register */

/* PMECC Error Location Interrupt Disable Register */

/* PMECC Error Location Interrupt Mask Register */

#define HSMC_ELIINT_DONE              (1 << 0)  /* Bit 0:  Computation Terminated Interrupt */
#define HSMC_ELISR_ERRCNT_SHIFT       (8)       /* Bit 8-12: Error Counter value (SR only) */
#define HSMC_ELISR_ERRCNT_MASK        (0x1f << HSMC_ELISR_ERRCNT_SHIFT)

/* PMECC Error Location SIGMA x Register */

#define HSMC_SIGMA_MASK               (0x3fff)  /* Bits 0-13: Coefficient of degree x in the SIGMA polynomial */

/* PMECC Error Location x Register */

#define HSMC_ERRLOC_MASK              (0x3fff)  /* Bits 0-13: Error Position within the Set */

/* HSMC Setup Register */

#define HSMC_SETUP_NWE_SETUP_SHIFT    (0)       /* Bit 0-5: NWE Setup Length */
#define HSMC_SETUP_NWE_SETUP_MASK     (0x3f << HSMC_SETUP_NWE_SETUP_SHIFT)
#  define HSMC_SETUP_NWE_SETUP(n)     ((n) << HSMC_SETUP_NWE_SETUP_SHIFT)
#define HSMC_SETUP_NCS_WRSETUP_SHIFT  (8)       /* Bit 8-13: NCS Setup Length in Write Access */
#define HSMC_SETUP_NCS_WRSETUP_MASK   (0x3f << HSMC_SETUP_NCS_WRSETUP_SHIFT)
#  define HSMC_SETUP_NCS_WRSETUP(n)   ((n) << HSMC_SETUP_NCS_WRSETUP_SHIFT)
#define HSMC_SETUP_NRD_SETUP_SHIFT    (16)      /* Bit 16-21: NRD Setup Length */
#define HSMC_SETUP_NRD_SETUP_MASK     (0x3f << HSMC_SETUP_NRD_SETUP_SHIFT)
#  define HSMC_SETUP_NRD_SETUP(n)     ((n) << HSMC_SETUP_NRD_SETUP_SHIFT)
#define HSMC_SETUP_NCS_RDSETUP_SHIFT  (24)      /* Bit 24-29: NCS Setup Length in Read Access */
#define HSMC_SETUP_NCS_RDSETUP_MASK   (0x3f << HSMC_SETUP_NCS_RDSETUP_SHIFT)
#  define HSMC_SETUP_NCS_RDSETUP(n)   ((n) << HSMC_SETUP_NCS_RDSETUP_SHIFT)

/* HSMC Pulse Register */

#define HSMC_PULSE_NWE_PULSE_SHIFT    (0)       /* Bit 0-5: NWE Pulse Length */
#define HSMC_PULSE_NWE_PULSE_MASK     (0x3f << HSMC_PULSE_NWE_PULSE_SHIFT)
#  define HSMC_PULSE_NWE_PULSE(n)     ((n) << HSMC_PULSE_NWE_PULSE_SHIFT)
#define HSMC_PULSE_NCS_WRPULSE_SHIFT  (8)       /* Bit 8-13: NCS Pulse Length in WRITE Access */
#define HSMC_PULSE_NCS_WRPULSE_MASK   (0x3f << HSMC_PULSE_NCS_WRPULSE_SHIFT)
#  define HSMC_PULSE_NCS_WRPULSE(n)   ((n) << HSMC_PULSE_NCS_WRPULSE_SHIFT)
#define HSMC_PULSE_NRD_PULSE_SHIFT    (16)      /* Bit 16-21: NRD Pulse Length */
#define HSMC_PULSE_NRD_PULSE_MASK     (0x3f << HSMC_PULSE_NRD_PULSE_SHIFT)
#  define HSMC_PULSE_NRD_PULSE(n)     ((n) << HSMC_PULSE_NRD_PULSE_SHIFT)
#define HSMC_PULSE_NCS_RDPULSE_SHIFT  (24)      /* Bit 24-29: NCS Pulse Length in READ Access */
#define HSMC_PULSE_NCS_RDPULSE_MASK   (0x3f << HSMC_PULSE_NCS_RDPULSE_SHIFT)
#  define HSMC_PULSE_NCS_RDPULSE(n)   ((n) << HSMC_PULSE_NCS_RDPULSE_SHIFT)

/* HSMC Cycle Register */

#define HSMC_CYCLE_NWE_CYCLE_SHIFT     (0)       /* Bit 0-8: Total Write Cycle Length */
#define HSMC_CYCLE_NWE_CYCLE_MASK      (0x1ff << HSMC_CYCLE_NWE_CYCLE_SHIFT)
#  define HSMC_CYCLE_NWE_CYCLE(n)      ((n) << HSMC_CYCLE_NWE_CYCLE_SHIFT)
#define HSMC_CYCLE_NRD_CYCLE_SHIFT     (16)      /* Bit 16-24: Total Read Cycle Length */
#define HSMC_CYCLE_NRD_CYCLE_MASK      (0x1ff << HSMC_CYCLE_NRD_CYCLE_SHIFT)
#  define HSMC_CYCLE_NRD_CYCLE(n)      ((n) << HSMC_CYCLE_NRD_CYCLE_SHIFT)

/* HSMC Timings Register */

#define HSMC_TIMINGS_TCLR_SHIFT       (0)       /* Bit 0-3: CLE to REN Low Delay */
#define HSMC_TIMINGS_TCLR_MASK        (15 << HSMC_TIMINGS_TCLR_SHIFT)
#  define HSMC_TIMINGS_TCLR(n)        ((n) << HSMC_TIMINGS_TCLR_SHIFT)
#define HSMC_TIMINGS_TADL_SHIFT       (4)       /* Bit 4-7: ALE to Data Start */
#define HSMC_TIMINGS_TADL_MASK        (15 << HSMC_TIMINGS_TADL_SHIFT)
#  define HSMC_TIMINGS_TADL(n)        ((n) << HSMC_TIMINGS_TADL_SHIFT)
#define HSMC_TIMINGS_TAR_SHIFT        (8)       /* Bit 8-11: ALE to REN Low Delay */
#define HSMC_TIMINGS_TAR_MASK         (15 << HSMC_TIMINGS_TAR_SHIFT)
#  define HSMC_TIMINGS_TAR(n)         ((n) << HSMC_TIMINGS_TAR_SHIFT)
#define HSMC_TIMINGS_OCMS             (1 << 12) /* Bit 12: Off Chip Memory Scrambling Enable */
#define HSMC_TIMINGS_TRR_SHIFT        (16)      /* Bit 16-19: Ready to REN Low Delay */
#define HSMC_TIMINGS_TRR_MASK         (15 << HSMC_TIMINGS_TRR_SHIFT)
#  define HSMC_TIMINGS_TRR(n)         ((n) << HSMC_TIMINGS_TRR_SHIFT)
#define HSMC_TIMINGS_TWB_SHIFT        (24)      /* Bit 24-27: WEN High to REN to Busy */
#define HSMC_TIMINGS_TWB_MASK         (15 << HSMC_TIMINGS_TWB_SHIFT)
#  define HSMC_TIMINGS_TWB(n)         ((n) << HSMC_TIMINGS_TWB_SHIFT)
#define HSMC_TIMINGS_RBNSEL_SHIFT     (28)      /* Bit 28-30: Ready/Busy Line Selection */
#define HSMC_TIMINGS_RBNSEL_MASK      (7 << HSMC_TIMINGS_RBNSEL_SHIFT)
#  define HSMC_TIMINGS_RBNSEL(n)      ((n) << HSMC_TIMINGS_RBNSEL_SHIFT)
#define HSMC_TIMINGS_NFSEL            (1 << 31) /* Bit 31: NAND Flash Selection */

/* HSMC Mode Register */

#define HSMC_MODE_READMODE            (1 << 0)  /* Bit 0: */
#define HSMC_MODE_WRITEMODE           (1 << 1)  /* Bit 1: */
#define HSMC_MODE_EXNWMODE_SHIFT      (4)       /* Bit 4-5: NWAIT Mode */
#define HSMC_MODE_EXNWMODE_MASK       (3 << HSMC_MODE_EXNWMODE_SHIFT)
#  define HSMC_MODE_EXNWMODE_DISABLED (0 << HSMC_MODE_EXNWMODE_SHIFT) /* Disabled */
#  define HSMC_MODE_EXNWMODE_FROZEN   (2 << HSMC_MODE_EXNWMODE_SHIFT) /* Frozen Mode */
#  define HSMC_MODE_EXNWMODE_READY    (3 << HSMC_MODE_EXNWMODE_SHIFT) /* Ready Mode */

#define HSMC_MODE_BAT                 (1 << 8)  /* Bit 8:  Byte Access Type */
#define HSMC_MODE_DBW                 (1 << 12) /* Bit 12: Data Bus Width */

#  define HSMC_MODE_BIT_8             (0)             /* 0=8-bit bus */
#  define HSMC_MODE_BIT_16             HSMC_MODE_DBW  /* 1=16-bit bus */

#define HSMC_MODE_TDFCYCLES_SHIFT     (16)      /* Bit 16-19: Data Float Time */
#define HSMC_MODE_TDFCYCLES_MASK      (15 << HSMC_MODE_TDFCYCLES_SHIFT)
#  define HSMC_MODE_TDFCYCLES(n)      ((n) << HSMC_MODE_TDFCYCLES_SHIFT)
#define HSMC_MODE_TDFMODE             (1 << 20) /* Bit 20: TDF Optimization */

/* HSMC OCMS Register */

#define HSMC_OCMS_SMSE                (1 << 0)  /* Bit 0: Static Memory Controller Scrambling Enable */
#define HSMC_OCMS_SRSE                (1 << 1)  /* Bit 1: SRAM Scrambling Enable */

/* HSMC OCMS KEY1 Register (32-bits of 64-bit key value) */

/* HSMC OCMS KEY2 Register (32-bits of 64-bit key value) */

/* HSMC Write Protection Mode Register */

#define HSMC_WPMR_WPEN                (1 << 0)  /* Bit 0:  Write Protection Enable */
#define HSMC_WPMR_WPKEY_SHIFT         (8)       /* Bit 8-31: Write Protection KEY password */
#define HSMC_WPMR_WPKEY_MASK          (0x00ffffff << HSMC_WPMR_WPKEY_SHIFT)
#  define HSMC_WPMR_WPKEY             (0x00534d43 << HSMC_WPMR_WPKEY_SHIFT)

/* HSMC Write Protection Status Register */

#define HSMC_WPSR_WPVS_SHIFT          (0)       /* Bit 0-3: Write Protection Violation Status */
#define HSMC_WPSR_WPVS_MASK           (15 << HSMC_WPSR_WPVS_SHIFT)
#define HSMC_WPSR_WPVSRC_SHIFT        (8)       /* Bit 8-23: Write Protection Violation Source */
#define HSMC_WPSR_WPVSRC_MASK         (0xffff << HSMC_WPSR_WPVSRC_SHIFT)

/* NFC Command/Data Registers ***********************************************/

/* NFC Command and Status Registers */

#define NFCADDR_CMD_CMD1_SHIFT        (2)        /* Bits 2-9: Command Register Value for Cycle 1 */
#define NFCADDR_CMD_CMD1_MASK         (0xff <<  NFCADDR_CMD_CMD1_SHIFT)
#  define NFCADDR_CMD_CMD1(n)         ((uint32_t)(n) <<  NFCADDR_CMD_CMD1_SHIFT)
#define NFCADDR_CMD_CMD2_SHIFT        (10)       /* Bits 10-17: Command Register Value for Cycle 1 */
#define NFCADDR_CMD_CMD2_MASK         (0xff <<  NFCADDR_CMD_CMD2_SHIFT)
#  define NFCADDR_CMD_CMD2(n)         ((uint32_t)(n) <<  NFCADDR_CMD_CMD2_SHIFT)
#define NFCADDR_CMD_VCMD2             (1 << 18)  /* Bit 18:Valid Cycle 2 Command */
#define NFCADDR_CMD_ACYCLE_SHIFT      (19)       /* Bits 19-21: Number of Address required for command */
#define NFCADDR_CMD_ACYCLE_MASK       (7 << NFCADDR_CMD_ACYCLE_SHIFT)
#  define NFCADDR_CMD_ACYCLE(n)       ((uint32_t)(n) << NFCADDR_CMD_ACYCLE_SHIFT) /* n address cycles, n=0-5 */

#  define NFCADDR_CMD_ACYCLE_NONE     (0 << NFCADDR_CMD_ACYCLE_SHIFT) /* No address cycle */
#  define NFCADDR_CMD_ACYCLE_ONE      (1 << NFCADDR_CMD_ACYCLE_SHIFT) /* One address cycle */
#  define NFCADDR_CMD_ACYCLE_TWO      (2 << NFCADDR_CMD_ACYCLE_SHIFT) /* Two address cycles */
#  define NFCADDR_CMD_ACYCLE_THREE    (3 << NFCADDR_CMD_ACYCLE_SHIFT) /* Three address cycles */
#  define NFCADDR_CMD_ACYCLE_FOUR     (4 << NFCADDR_CMD_ACYCLE_SHIFT) /* Four address cycles */
#  define NFCADDR_CMD_ACYCLE_FIVE     (5 << NFCADDR_CMD_ACYCLE_SHIFT) /* Five address cycles */

#define NFCADDR_CMD_CSID_SHIFT        (22)       /* Bits 22-24: Chip Select Identifier */
#define NFCADDR_CMD_CSID_MASK         (7 << NFCADDR_CMD_CSID_SHIFT)
#  define NFCADDR_CMD_CSID(n)         ((uint32_t)(n) << NFCADDR_CMD_CSID_SHIFT) /* CSn, n=0-7 */

#  define NFCADDR_CMD_CSID_0          (0 << NFCADDR_CMD_CSID_SHIFT) /* CS0 */
#  define NFCADDR_CMD_CSID_1          (1 << NFCADDR_CMD_CSID_SHIFT) /* CS1 */
#  define NFCADDR_CMD_CSID_2          (2 << NFCADDR_CMD_CSID_SHIFT) /* CS2 */
#  define NFCADDR_CMD_CSID_3          (3 << NFCADDR_CMD_CSID_SHIFT) /* CS3 */
#  define NFCADDR_CMD_CSID_4          (4 << NFCADDR_CMD_CSID_SHIFT) /* CS4 */
#  define NFCADDR_CMD_CSID_5          (5 << NFCADDR_CMD_CSID_SHIFT) /* CS5 */
#  define NFCADDR_CMD_CSID_6          (6 << NFCADDR_CMD_CSID_SHIFT) /* CS6 */
#  define NFCADDR_CMD_CSID_7          (7 << NFCADDR_CMD_CSID_SHIFT) /* CS7 */

#define NFCADDR_CMD_DATAEN            (1 << 25) /* Bit 25: 1=NFC Data Enable */
#define NFCADDR_CMD_DATADIS           (0 << 25) /* Bit 25: 0=NFC Data disable */
#define NFCADDR_CMD_NFCRD             (0 << 26) /* Bit 26: 0=NFC Read Enable */
#define NFCADDR_CMD_NFCWR             (1 << 26) /* Bit 26: 1=NFC Write Enable */
#define NFCADDR_CMD_NFCCMD            (1 << 27) /* Bit 27: 1=NFC Command Enable (status only) */

/* NFC Data Address */

#define NFCDATA_ADDT_CYCLE1_SHIFT     (0)      /* Bits 0-7: NAND Flash Array Address Cycle 1 */
#define NFCDATA_ADDT_CYCLE1_MASK      (0xff << NFCDATA_ADDT_CYCLE1_SHIFT)
#  define NFCDATA_ADDT_CYCLE1(n)      ((uint32_t)(n) << NFCDATA_ADDT_CYCLE1_SHIFT)
#define NFCDATA_ADDT_CYCLE2_SHIFT     (8)      /* Bits 8-15: NAND Flash Array Address Cycle 2 */
#define NFCDATA_ADDT_CYCLE2_MASK      (0xff << NFCDATA_ADDT_CYCLE2_SHIFT)
#  define NFCDATA_ADDT_CYCLE2(n)      ((uint32_t)(n) << NFCDATA_ADDT_CYCLE2_SHIFT)
#define NFCDATA_ADDT_CYCLE3_SHIFT     (16)      /* Bits 16-23: NAND Flash Array Address Cycle 3 */
#define NFCDATA_ADDT_CYCLE3_MASK      (0xff << NFCDATA_ADDT_CYCLE3_SHIFT)
#  define NFCDATA_ADDT_CYCLE3(n)      ((uint32_t)(n) << NFCDATA_ADDT_CYCLE3_SHIFT)
#define NFCDATA_ADDT_CYCLE4_SHIFT     (24)      /* Bits 24-31: NAND Flash Array Address Cycle 4 */
#define NFCDATA_ADDT_CYCLE4_MASK      (0xff << NFCDATA_ADDT_CYCLE4_SHIFT)
#  define NFCDATA_ADDT_CYCLE4(n)      ((uint32_t)(n) << NFCDATA_ADDT_CYCLE4_SHIFT)

#endif /* __ARCH_ARM_SRC_SAMA5_HARDWARE_SAM_HSMC_H */
