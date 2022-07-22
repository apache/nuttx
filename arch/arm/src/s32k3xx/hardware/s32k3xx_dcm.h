/****************************************************************************
 * arch/arm/src/s32k3xx/hardware/s32k3xx_dcm.h
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

#ifndef __ARCH_ARM_SRC_S32K3XX_HARDWARE_S32K3XX_DCM_H
#define __ARCH_ARM_SRC_S32K3XX_HARDWARE_S32K3XX_DCM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <hardware/s32k3xx_memorymap.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* DCM Register Offsets *****************************************************/

#define S32K3XX_DCM_DCMSTAT_OFFSET      (0x0000) /* DCM Status (DCMSTAT) */
#define S32K3XX_DCM_DCMLCC_OFFSET       (0x0004) /* LC and LC Control (DCMLCC) */
#define S32K3XX_DCM_DCMLCS_OFFSET       (0x0008) /* LC Scan Status (DCMLCS) */
#define S32K3XX_DCM_DCMMISC_OFFSET      (0x001c) /* DCM Miscellaneous (DCMMISC) */
#define S32K3XX_DCM_DCMDEB_OFFSET       (0x0020) /* Debug Status and Configuration (DCMDEB) */
#define S32K3XX_DCM_DCMEC_OFFSET        (0x002c) /* DCF Error Count (DCMEC) */
#define S32K3XX_DCM_DCMSRR1_OFFSET      (0x0030) /* DCF Scan Report 1 (DCMSRR1) */
#define S32K3XX_DCM_DCMSRR2_OFFSET      (0x0034) /* DCF Scan Report 2 (DCMSRR2) */
#define S32K3XX_DCM_DCMSRR3_OFFSET      (0x0038) /* DCF Scan Report 3 (DCMSRR3) */
#define S32K3XX_DCM_DCMSRR4_OFFSET      (0x003c) /* DCF Scan Report 4 (DCMSRR4) */
#define S32K3XX_DCM_DCMSRR5_OFFSET      (0x0040) /* DCF Scan Report 5 (DCMSRR5) */
#define S32K3XX_DCM_DCMSRR6_OFFSET      (0x0044) /* DCF Scan Report 6 (DCMSRR6) */
#define S32K3XX_DCM_DCMSRR7_OFFSET      (0x0048) /* DCF Scan Report 7 (DCMSRR7) */
#define S32K3XX_DCM_DCMSRR8_OFFSET      (0x004c) /* DCF Scan Report 8 (DCMSRR8) */
#define S32K3XX_DCM_DCMSRR9_OFFSET      (0x0050) /* DCF Scan Report 9 (DCMSRR9) */
#define S32K3XX_DCM_DCMSRR10_OFFSET     (0x0054) /* DCF Scan Report 10 (DCMSRR10) */
#define S32K3XX_DCM_DCMSRR11_OFFSET     (0x0058) /* DCF Scan Report 11 (DCMSRR11) */
#define S32K3XX_DCM_DCMSRR12_OFFSET     (0x005c) /* DCF Scan Report 12 (DCMSRR12) */
#define S32K3XX_DCM_DCMSRR13_OFFSET     (0x0060) /* DCF Scan Report 13 (DCMSRR13) */
#define S32K3XX_DCM_DCMSRR14_OFFSET     (0x0064) /* DCF Scan Report 14 (DCMSRR14) */
#define S32K3XX_DCM_DCMSRR15_OFFSET     (0x0068) /* DCF Scan Report 15 (DCMSRR15) */
#define S32K3XX_DCM_DCMSRR16_OFFSET     (0x006c) /* DCF Scan Report 16 (DCMSRR16) */
#define S32K3XX_DCM_DCMLCS2_OFFSET      (0x0080) /* LC Scan Status 2 (DCMLCS2) */
#define S32K3XX_DCM_GPR_DCMROD1_OFFSET  (0x0200) /* Read Only GPR On Destructive Reset Register 1 (DCMROD1) */
#define S32K3XX_DCM_GPR_DCMROD3_OFFSET  (0x0208) /* Read Only GPR On Destructive Reset Register 3 (DCMROD3) */
#define S32K3XX_DCM_GPR_DCMROD4_OFFSET  (0x020c) /* Read Only GPR On Destructive Reset Register 4 (DCMROD4) */
#define S32K3XX_DCM_GPR_DCMROD5_OFFSET  (0x0210) /* Read Only GPR On Destructive Reset Register 5 (DCMROD5) */
#define S32K3XX_DCM_GPR_DCMROF1_OFFSET  (0x0300) /* Read Only GPR On Functional Reset Register 1 (DCMROF1) */
#define S32K3XX_DCM_GPR_DCMROF2_OFFSET  (0x0304) /* Read Only GPR On Functional Reset Register 2 (DCMROF2) */
#define S32K3XX_DCM_GPR_DCMROF3_OFFSET  (0x0308) /* Read Only GPR On Functional Reset Register 3 (DCMROF3) */
#define S32K3XX_DCM_GPR_DCMROF4_OFFSET  (0x030c) /* Read Only GPR On Functional Reset Register 4 (DCMROF4) */
#define S32K3XX_DCM_GPR_DCMROF5_OFFSET  (0x0310) /* Read Only GPR On Functional Reset Register 5 (DCMROF5) */
#define S32K3XX_DCM_GPR_DCMROF6_OFFSET  (0x0314) /* Read Only GPR On Functional Reset Register 6 (DCMROF6) */
#define S32K3XX_DCM_GPR_DCMROF7_OFFSET  (0x0318) /* Read Only GPR On Functional Reset Register 7 (DCMROF7) */
#define S32K3XX_DCM_GPR_DCMROF8_OFFSET  (0x031c) /* Read Only GPR On Functional Reset Register 8 (DCMROF8) */
#define S32K3XX_DCM_GPR_DCMROF9_OFFSET  (0x0320) /* Read Only GPR On Functional Reset Register 9 (DCMROF9) */
#define S32K3XX_DCM_GPR_DCMROF10_OFFSET (0x0324) /* Read Only GPR On Functional Reset Register 10 (DCMROF10) */
#define S32K3XX_DCM_GPR_DCMROF11_OFFSET (0x0328) /* Read Only GPR On Functional Reset Register 11 (DCMROF11) */
#define S32K3XX_DCM_GPR_DCMROF12_OFFSET (0x032c) /* Read Only GPR On Functional Reset Register 12 (DCMROF12) */
#define S32K3XX_DCM_GPR_DCMROF13_OFFSET (0x0330) /* Read Only GPR On Functional Reset Register 13 (DCMROF13) */
#define S32K3XX_DCM_GPR_DCMROF14_OFFSET (0x0334) /* Read Only GPR On Functional Reset Register 14 (DCMROF14) */
#define S32K3XX_DCM_GPR_DCMROF15_OFFSET (0x0338) /* Read Only GPR On Functional Reset Register 15 (DCMROF15) */
#define S32K3XX_DCM_GPR_DCMROF16_OFFSET (0x033c) /* Read Only GPR On Functional Reset Register 16 (DCMROF16) */
#define S32K3XX_DCM_GPR_DCMROF17_OFFSET (0x0340) /* Read Only GPR On Functional Reset Register 17 (DCMROF17) */
#define S32K3XX_DCM_GPR_DCMROF19_OFFSET (0x0348) /* Read Only GPR On Functional Reset Register 19 (DCMROF19) */
#define S32K3XX_DCM_GPR_DCMROF20_OFFSET (0x034c) /* Read Only GPR On Functional Reset Register 20 (DCMROF20) */
#define S32K3XX_DCM_GPR_DCMROF21_OFFSET (0x0350) /* Read Only GPR On Functional Reset Register 21 (DCMROF21) */
#define S32K3XX_DCM_GPR_DCMRWP1_OFFSET  (0x0400) /* Read Write GPR On Power On Reset Register 1 (DCMRWP1) */
#define S32K3XX_DCM_GPR_DCMRWP3_OFFSET  (0x0408) /* Read Write GPR On Power On Reset Register 3 (DCMRWP3) */
#define S32K3XX_DCM_GPR_DCMRWD2_OFFSET  (0x0504) /* Read Write GPR On Destructive Reset Register 2 (DCMRWD2) */
#define S32K3XX_DCM_GPR_DCMRWD3_OFFSET  (0x0508) /* Read Write GPR On Destructive Reset Register 3 (DCMRWD3) */
#define S32K3XX_DCM_GPR_DCMRWD4_OFFSET  (0x050c) /* Read Write GPR On Destructive Reset Register 4 (DCMRWD4) */
#define S32K3XX_DCM_GPR_DCMRWD5_OFFSET  (0x0510) /* Read Write GPR On Destructive Reset Register 5 (DCMRWD5) */
#define S32K3XX_DCM_GPR_DCMRWD6_OFFSET  (0x0514) /* Read Write GPR On Destructive Reset Register 6 (DCMRWD6) */
#define S32K3XX_DCM_GPR_DCMRWD7_OFFSET  (0x0518) /* Read Write GPR On Destructive Reset Register 7 (DCMRWD7) */
#define S32K3XX_DCM_GPR_DCMRWD8_OFFSET  (0x051c) /* Read Write GPR On Destructive Reset Register 8 (DCMRWD8) */
#define S32K3XX_DCM_GPR_DCMRWD9_OFFSET  (0x0520) /* Read Write GPR On Destructive Reset Register 9 (DCMRWD9) */
#define S32K3XX_DCM_GPR_DCMRWF1_OFFSET  (0x0600) /* Read Write GPR On Functional Reset Register 1 (DCMRWF1) */
#define S32K3XX_DCM_GPR_DCMRWF2_OFFSET  (0x0604) /* Read Write GPR On Functional Reset Register 2 (DCMRWF2) */
#define S32K3XX_DCM_GPR_DCMRWF4_OFFSET  (0x060c) /* Read Write GPR On Functional Reset Register 4 (DCMRWF4) */
#define S32K3XX_DCM_GPR_DCMRWF5_OFFSET  (0x0610) /* Read Write GPR On Functional Reset Register 5 (DCMRWF5) */
#define S32K3XX_DCM_GPR_DCMROPP1_OFFSET (0x0700) /* Read Only GPR On PMCPOR Reset Register 1 (DCMROPP1) */
#define S32K3XX_DCM_GPR_DCMROPP2_OFFSET (0x0704) /* Read Only GPR On PMCPOR Reset Register 2 (DCMROPP2) */
#define S32K3XX_DCM_GPR_DCMROPP3_OFFSET (0x0708) /* Read Only GPR On PMCPOR Reset Register 3 (DCMROPP3) */
#define S32K3XX_DCM_GPR_DCMROPP4_OFFSET (0x070c) /* Read Only GPR On PMCPOR Reset Register 4 (DCMROPP4) */

/* DCM Register Addresses ***************************************************/

#define S32K3XX_DCM_DCMSTAT            (S32K3XX_DCM_BASE + S32K3XX_DCM_DCMSTAT_OFFSET)
#define S32K3XX_DCM_DCMLCC             (S32K3XX_DCM_BASE + S32K3XX_DCM_DCMLCC_OFFSET)
#define S32K3XX_DCM_DCMLCS             (S32K3XX_DCM_BASE + S32K3XX_DCM_DCMLCS_OFFSET)
#define S32K3XX_DCM_DCMMISC            (S32K3XX_DCM_BASE + S32K3XX_DCM_DCMMISC_OFFSET)
#define S32K3XX_DCM_DCMDEB             (S32K3XX_DCM_BASE + S32K3XX_DCM_DCMDEB_OFFSET)
#define S32K3XX_DCM_DCMEC              (S32K3XX_DCM_BASE + S32K3XX_DCM_DCMEC_OFFSET)
#define S32K3XX_DCM_DCMSRR1            (S32K3XX_DCM_BASE + S32K3XX_DCM_DCMSRR1_OFFSET)
#define S32K3XX_DCM_DCMSRR2            (S32K3XX_DCM_BASE + S32K3XX_DCM_DCMSRR2_OFFSET)
#define S32K3XX_DCM_DCMSRR3            (S32K3XX_DCM_BASE + S32K3XX_DCM_DCMSRR3_OFFSET)
#define S32K3XX_DCM_DCMSRR4            (S32K3XX_DCM_BASE + S32K3XX_DCM_DCMSRR4_OFFSET)
#define S32K3XX_DCM_DCMSRR5            (S32K3XX_DCM_BASE + S32K3XX_DCM_DCMSRR5_OFFSET)
#define S32K3XX_DCM_DCMSRR6            (S32K3XX_DCM_BASE + S32K3XX_DCM_DCMSRR6_OFFSET)
#define S32K3XX_DCM_DCMSRR7            (S32K3XX_DCM_BASE + S32K3XX_DCM_DCMSRR7_OFFSET)
#define S32K3XX_DCM_DCMSRR8            (S32K3XX_DCM_BASE + S32K3XX_DCM_DCMSRR8_OFFSET)
#define S32K3XX_DCM_DCMSRR9            (S32K3XX_DCM_BASE + S32K3XX_DCM_DCMSRR9_OFFSET)
#define S32K3XX_DCM_DCMSRR10           (S32K3XX_DCM_BASE + S32K3XX_DCM_DCMSRR10_OFFSET)
#define S32K3XX_DCM_DCMSRR11           (S32K3XX_DCM_BASE + S32K3XX_DCM_DCMSRR11_OFFSET)
#define S32K3XX_DCM_DCMSRR12           (S32K3XX_DCM_BASE + S32K3XX_DCM_DCMSRR12_OFFSET)
#define S32K3XX_DCM_DCMSRR13           (S32K3XX_DCM_BASE + S32K3XX_DCM_DCMSRR13_OFFSET)
#define S32K3XX_DCM_DCMSRR14           (S32K3XX_DCM_BASE + S32K3XX_DCM_DCMSRR14_OFFSET)
#define S32K3XX_DCM_DCMSRR15           (S32K3XX_DCM_BASE + S32K3XX_DCM_DCMSRR15_OFFSET)
#define S32K3XX_DCM_DCMSRR16           (S32K3XX_DCM_BASE + S32K3XX_DCM_DCMSRR16_OFFSET)
#define S32K3XX_DCM_DCMLCS2            (S32K3XX_DCM_BASE + S32K3XX_DCM_DCMLCS2_OFFSET)
#define S32K3XX_DCM_GPR_DCMROD1        (S32K3XX_DCM_BASE + S32K3XX_DCM_GPR_DCMROD1_OFFSET)
#define S32K3XX_DCM_GPR_DCMROD3        (S32K3XX_DCM_BASE + S32K3XX_DCM_GPR_DCMROD3_OFFSET)
#define S32K3XX_DCM_GPR_DCMROD4        (S32K3XX_DCM_BASE + S32K3XX_DCM_GPR_DCMROD4_OFFSET)
#define S32K3XX_DCM_GPR_DCMROD5        (S32K3XX_DCM_BASE + S32K3XX_DCM_GPR_DCMROD5_OFFSET)
#define S32K3XX_DCM_GPR_DCMROF1        (S32K3XX_DCM_BASE + S32K3XX_DCM_GPR_DCMROF1_OFFSET)
#define S32K3XX_DCM_GPR_DCMROF2        (S32K3XX_DCM_BASE + S32K3XX_DCM_GPR_DCMROF2_OFFSET)
#define S32K3XX_DCM_GPR_DCMROF3        (S32K3XX_DCM_BASE + S32K3XX_DCM_GPR_DCMROF3_OFFSET)
#define S32K3XX_DCM_GPR_DCMROF4        (S32K3XX_DCM_BASE + S32K3XX_DCM_GPR_DCMROF4_OFFSET)
#define S32K3XX_DCM_GPR_DCMROF5        (S32K3XX_DCM_BASE + S32K3XX_DCM_GPR_DCMROF5_OFFSET)
#define S32K3XX_DCM_GPR_DCMROF6        (S32K3XX_DCM_BASE + S32K3XX_DCM_GPR_DCMROF6_OFFSET)
#define S32K3XX_DCM_GPR_DCMROF7        (S32K3XX_DCM_BASE + S32K3XX_DCM_GPR_DCMROF7_OFFSET)
#define S32K3XX_DCM_GPR_DCMROF8        (S32K3XX_DCM_BASE + S32K3XX_DCM_GPR_DCMROF8_OFFSET)
#define S32K3XX_DCM_GPR_DCMROF9        (S32K3XX_DCM_BASE + S32K3XX_DCM_GPR_DCMROF9_OFFSET)
#define S32K3XX_DCM_GPR_DCMROF10       (S32K3XX_DCM_BASE + S32K3XX_DCM_GPR_DCMROF10_OFFSET)
#define S32K3XX_DCM_GPR_DCMROF11       (S32K3XX_DCM_BASE + S32K3XX_DCM_GPR_DCMROF11_OFFSET)
#define S32K3XX_DCM_GPR_DCMROF12       (S32K3XX_DCM_BASE + S32K3XX_DCM_GPR_DCMROF12_OFFSET)
#define S32K3XX_DCM_GPR_DCMROF13       (S32K3XX_DCM_BASE + S32K3XX_DCM_GPR_DCMROF13_OFFSET)
#define S32K3XX_DCM_GPR_DCMROF14       (S32K3XX_DCM_BASE + S32K3XX_DCM_GPR_DCMROF14_OFFSET)
#define S32K3XX_DCM_GPR_DCMROF15       (S32K3XX_DCM_BASE + S32K3XX_DCM_GPR_DCMROF15_OFFSET)
#define S32K3XX_DCM_GPR_DCMROF16       (S32K3XX_DCM_BASE + S32K3XX_DCM_GPR_DCMROF16_OFFSET)
#define S32K3XX_DCM_GPR_DCMROF17       (S32K3XX_DCM_BASE + S32K3XX_DCM_GPR_DCMROF17_OFFSET)
#define S32K3XX_DCM_GPR_DCMROF19       (S32K3XX_DCM_BASE + S32K3XX_DCM_GPR_DCMROF19_OFFSET)
#define S32K3XX_DCM_GPR_DCMROF20       (S32K3XX_DCM_BASE + S32K3XX_DCM_GPR_DCMROF20_OFFSET)
#define S32K3XX_DCM_GPR_DCMROF21       (S32K3XX_DCM_BASE + S32K3XX_DCM_GPR_DCMROF21_OFFSET)
#define S32K3XX_DCM_GPR_DCMRWP1        (S32K3XX_DCM_BASE + S32K3XX_DCM_GPR_DCMRWP1_OFFSET)
#define S32K3XX_DCM_GPR_DCMRWP3        (S32K3XX_DCM_BASE + S32K3XX_DCM_GPR_DCMRWP3_OFFSET)
#define S32K3XX_DCM_GPR_DCMRWD2        (S32K3XX_DCM_BASE + S32K3XX_DCM_GPR_DCMRWD2_OFFSET)
#define S32K3XX_DCM_GPR_DCMRWD3        (S32K3XX_DCM_BASE + S32K3XX_DCM_GPR_DCMRWD3_OFFSET)
#define S32K3XX_DCM_GPR_DCMRWD4        (S32K3XX_DCM_BASE + S32K3XX_DCM_GPR_DCMRWD4_OFFSET)
#define S32K3XX_DCM_GPR_DCMRWD5        (S32K3XX_DCM_BASE + S32K3XX_DCM_GPR_DCMRWD5_OFFSET)
#define S32K3XX_DCM_GPR_DCMRWD6        (S32K3XX_DCM_BASE + S32K3XX_DCM_GPR_DCMRWD6_OFFSET)
#define S32K3XX_DCM_GPR_DCMRWD7        (S32K3XX_DCM_BASE + S32K3XX_DCM_GPR_DCMRWD7_OFFSET)
#define S32K3XX_DCM_GPR_DCMRWD8        (S32K3XX_DCM_BASE + S32K3XX_DCM_GPR_DCMRWD8_OFFSET)
#define S32K3XX_DCM_GPR_DCMRWD9        (S32K3XX_DCM_BASE + S32K3XX_DCM_GPR_DCMRWD9_OFFSET)
#define S32K3XX_DCM_GPR_DCMRWF1        (S32K3XX_DCM_BASE + S32K3XX_DCM_GPR_DCMRWF1_OFFSET)
#define S32K3XX_DCM_GPR_DCMRWF2        (S32K3XX_DCM_BASE + S32K3XX_DCM_GPR_DCMRWF2_OFFSET)
#define S32K3XX_DCM_GPR_DCMRWF4        (S32K3XX_DCM_BASE + S32K3XX_DCM_GPR_DCMRWF4_OFFSET)
#define S32K3XX_DCM_GPR_DCMRWF5        (S32K3XX_DCM_BASE + S32K3XX_DCM_GPR_DCMRWF5_OFFSET)
#define S32K3XX_DCM_GPR_DCMROPP1       (S32K3XX_DCM_BASE + S32K3XX_DCM_GPR_DCMROPP1_OFFSET)
#define S32K3XX_DCM_GPR_DCMROPP2       (S32K3XX_DCM_BASE + S32K3XX_DCM_GPR_DCMROPP2_OFFSET)
#define S32K3XX_DCM_GPR_DCMROPP3       (S32K3XX_DCM_BASE + S32K3XX_DCM_GPR_DCMROPP3_OFFSET)
#define S32K3XX_DCM_GPR_DCMROPP4       (S32K3XX_DCM_BASE + S32K3XX_DCM_GPR_DCMROPP4_OFFSET)

/* DCM Register Bitfield Definitions ****************************************/

/* DCM Status (DCMSTAT) */

#define DCM_DCMSTAT_DCMDONE            (1 << 0)  /* Bit 0: DCM Scanning Status (DCMDONE) */
#define DCM_DCMSTAT_DCMERR             (1 << 1)  /* Bit 1: DCM completion with error status (DCMERR) */
                                                 /* Bits 2-3: Reserved */
#define DCM_DCMSTAT_DCMLCST            (1 << 4)  /* Bit 4: LC Scanning Status (DCMLCST) */
                                                 /* Bits 5-7: Reserved */
#define DCM_DCMSTAT_DCMUTS             (1 << 8)  /* Bit 8: DCM UTEST DCF Scanning Status (DCMUTS) */
#define DCM_DCMSTAT_DCMOTAS            (1 << 9)  /* Bit 9: DCM OTA Scanning Status (DCMOTAS) */
#define DCM_DCMSTAT_DCMDBGPS           (1 << 10) /* Bit 10: Debug Password Scanning Status (DCMDBGPS) */
                                                 /* Bits 11-31: Reserved */

/* LC and LC Control (DCMLCC) */

#define DCM_DCMLCC_DCMCLC_SHIFT        (0)       /* Bits 0-2: Current LC (DCMCLC) */
#define DCM_DCMLCC_DCMCLC_MASK         (0x07 << DCM_DCMLCC_DCMCLC_SHIFT)
#define DCM_DCMLCC_DCMLCFN             (1 << 3)  /* Bit 3: Force LC (DCMLCFN) */
#define DCM_DCMLCC_DCMRLC_SHIFT        (0)       /* Bits 4-6: Real LC (DCMRLC) */
#define DCM_DCMLCC_DCMRLC_MASK         (0x07 << DCM_DCMLCC_DCMRLC_SHIFT)
                                                 /* Bit 7: Reserved */
#define DCM_DCMLCC_DCMFLC_SHIFT        (8)       /* Bits 8-9: Force Next LC (DCMFLC) */
#define DCM_DCMLCC_DCMFLC_MASK         (0x03 << DCM_DCMLCC_DCMFLC_SHIFT)
                                                 /* Bits 10-31: Reserved */

/* LC Scan Status (DCMLCS) */

#define DCM_DCMLCS_DCMLCSS1            (1 << 0)  /* Bit 0: MCU_PROD Scan Status (DCMLCSS1) */
#define DCM_DCMLCS_DCMLCC1_SHIFT       (1)       /* Bits 1-3: MCU_PROD Marking (DCMLCC1) */
#define DCM_DCMLCS_DCMLCC1_MASK        (0x07 << DCM_DCMLCS_DCMLCC1_SHIFT)
#  define DCM_DCMLCS_DCMLCC1_NOSCAN    (0x00 << DCM_DCMLCS_DCMLCC1_SHIFT) /* Not scanned yet */
#  define DCM_DCMLCS_DCMLCC1_ACTIVE    (0x01 << DCM_DCMLCS_DCMLCC1_SHIFT) /* Marked as active */
#  define DCM_DCMLCS_DCMLCC1_INACTIVE  (0x02 << DCM_DCMLCS_DCMLCC1_SHIFT) /* Marked as inactive */
#  define DCM_DCMLCS_DCMLCC1_ERASED    (0x03 << DCM_DCMLCS_DCMLCC1_SHIFT) /* Region is erased/virgin */
#  define DCM_DCMLCS_DCMLCC1_UNKNOWN   (0x05 << DCM_DCMLCS_DCMLCC1_SHIFT) /* Marked as inactive by an unknown pattern */
#  define DCM_DCMLCS_DCMLCC1_TIMEOUT   (0x06 << DCM_DCMLCS_DCMLCC1_SHIFT) /* Scanning timed out */

#define DCM_DCMLCS_DCMLCE1             (1 << 4)  /* Bit 4: MCU_PROD ECC Errors (DCMLCE1) */
#define DCM_DCMLCS_DCMLCFE1            (1 << 5)  /* Bit 5: MCU_PROD Flash Memory Error Check (DCMLCFE1) */
#define DCM_DCMLCS_DCMLCSS2            (1 << 6)  /* Bit 6: CUST_DEL Scan Status (DCMLCSS2) */
#define DCM_DCMLCS_DCMLCC2_SHIFT       (7)       /* Bits 7-9: CUST_DEL Marking (DCMLCC2) */
#define DCM_DCMLCS_DCMLCC2_MASK        (0x07 << DCM_DCMLCS_DCMLCC2_SHIFT)
#  define DCM_DCMLCS_DCMLCC2_NOSCAN    (0x00 << DCM_DCMLCS_DCMLCC2_SHIFT) /* Not scanned yet */
#  define DCM_DCMLCS_DCMLCC2_ACTIVE    (0x01 << DCM_DCMLCS_DCMLCC2_SHIFT) /* Marked as active */
#  define DCM_DCMLCS_DCMLCC2_INACTIVE  (0x02 << DCM_DCMLCS_DCMLCC2_SHIFT) /* Marked as inactive */
#  define DCM_DCMLCS_DCMLCC2_ERASED    (0x03 << DCM_DCMLCS_DCMLCC2_SHIFT) /* Region is erased/virgin */
#  define DCM_DCMLCS_DCMLCC2_UNKNOWN   (0x05 << DCM_DCMLCS_DCMLCC2_SHIFT) /* Marked as inactive by an unknown pattern */
#  define DCM_DCMLCS_DCMLCC2_TIMEOUT   (0x06 << DCM_DCMLCS_DCMLCC2_SHIFT) /* Scanning timed out */

#define DCM_DCMLCS_DCMLCE2             (1 << 10) /* Bit 10: CUST_DEL ECC Errors (DCMLCE2) */
#define DCM_DCMLCS_DCMLCFE2            (1 << 11) /* Bit 11: CUST_DEL Flash Memory Error Check (DCMLCFE2) */
#define DCM_DCMLCS_DCMLCSS3            (1 << 12) /* Bit 12: OEM_PROD Scan Status (DCMLCSS3) */
#define DCM_DCMLCS_DCMLCC3_SHIFT       (13)      /* Bits 13-15: OEM_PROD Marking (DCMLCC3) */
#define DCM_DCMLCS_DCMLCC3_MASK        (0x07 << DCM_DCMLCS_DCMLCC3_SHIFT)
#  define DCM_DCMLCS_DCMLCC3_NOSCAN    (0x00 << DCM_DCMLCS_DCMLCC3_SHIFT) /* Not scanned yet */
#  define DCM_DCMLCS_DCMLCC3_ACTIVE    (0x01 << DCM_DCMLCS_DCMLCC3_SHIFT) /* Marked as active */
#  define DCM_DCMLCS_DCMLCC3_INACTIVE  (0x02 << DCM_DCMLCS_DCMLCC3_SHIFT) /* Marked as inactive */
#  define DCM_DCMLCS_DCMLCC3_ERASED    (0x03 << DCM_DCMLCS_DCMLCC3_SHIFT) /* Region is erased/virgin */
#  define DCM_DCMLCS_DCMLCC3_UNKNOWN   (0x05 << DCM_DCMLCS_DCMLCC3_SHIFT) /* Marked as inactive by an unknown pattern */
#  define DCM_DCMLCS_DCMLCC3_TIMEOUT   (0x06 << DCM_DCMLCS_DCMLCC3_SHIFT) /* Scanning timed out */

#define DCM_DCMLCS_DCMLCE3             (1 << 16) /* Bit 16: OEM_PROD ECC Errors (DCMLCE3) */
#define DCM_DCMLCS_DCMLCFE3            (1 << 17) /* Bit 17: OEM_PROD Flash Memory Error Check (DCMLCFE3) */
#define DCM_DCMLCS_DCMLCSS4            (1 << 18) /* Bit 18: IN_FIELD Scan Status (DCMLCSS4) */
#define DCM_DCMLCS_DCMLCC4_SHIFT       (19)      /* Bits 19-21: IN_FIELD Marking (DCMLCC4) */
#define DCM_DCMLCS_DCMLCC4_MASK        (0x07 << DCM_DCMLCS_DCMLCC4_SHIFT)
#  define DCM_DCMLCS_DCMLCC4_NOSCAN    (0x00 << DCM_DCMLCS_DCMLCC4_SHIFT) /* Not scanned yet */
#  define DCM_DCMLCS_DCMLCC4_ACTIVE    (0x01 << DCM_DCMLCS_DCMLCC4_SHIFT) /* Marked as active */
#  define DCM_DCMLCS_DCMLCC4_INACTIVE  (0x02 << DCM_DCMLCS_DCMLCC4_SHIFT) /* Marked as inactive */
#  define DCM_DCMLCS_DCMLCC4_ERASED    (0x03 << DCM_DCMLCS_DCMLCC4_SHIFT) /* Region is erased/virgin */
#  define DCM_DCMLCS_DCMLCC4_UNKNOWN   (0x05 << DCM_DCMLCS_DCMLCC4_SHIFT) /* Marked as inactive by an unknown pattern */
#  define DCM_DCMLCS_DCMLCC4_TIMEOUT   (0x06 << DCM_DCMLCS_DCMLCC4_SHIFT) /* Scanning timed out */

#define DCM_DCMLCS_DCMLCE4             (1 << 22) /* Bit 22: IN_FIELD ECC Errors (DCMLCE4) */
#define DCM_DCMLCS_DCMLCFE4            (1 << 23) /* Bit 23: IN_FIELD Flash Memory Error Check (DCMLCFE4) */
#define DCM_DCMLCS_DCMLCSS5            (1 << 24) /* Bit 24: Pre-FA Scan Status (DCMLCSS5) */
#define DCM_DCMLCS_DCMLCC5_SHIFT       (25)      /* Bits 25-27: Pre-FA Marking (DCMLCC5) */
#define DCM_DCMLCS_DCMLCC5_MASK        (0x07 << DCM_DCMLCS_DCMLCC5_SHIFT)
#  define DCM_DCMLCS_DCMLCC5_NOSCAN    (0x00 << DCM_DCMLCS_DCMLCC5_SHIFT) /* Not scanned yet */
#  define DCM_DCMLCS_DCMLCC5_ACTIVE    (0x01 << DCM_DCMLCS_DCMLCC5_SHIFT) /* Marked as active */
#  define DCM_DCMLCS_DCMLCC5_INACTIVE  (0x02 << DCM_DCMLCS_DCMLCC5_SHIFT) /* Marked as inactive */
#  define DCM_DCMLCS_DCMLCC5_ERASED    (0x03 << DCM_DCMLCS_DCMLCC5_SHIFT) /* Region is erased/virgin */
#  define DCM_DCMLCS_DCMLCC5_UNKNOWN   (0x05 << DCM_DCMLCS_DCMLCC5_SHIFT) /* Marked as inactive by an unknown pattern */
#  define DCM_DCMLCS_DCMLCC5_TIMEOUT   (0x06 << DCM_DCMLCS_DCMLCC5_SHIFT) /* Scanning timed out */

#define DCM_DCMLCS_DCMLCE5             (1 << 28) /* Bit 28: Pre-FA ECC Errors (DCMLCE5) */
#define DCM_DCMLCS_DCMLCFE5            (1 << 29) /* Bit 29: Pre-FA Flash Memory Error Check (DCMLCFE5) */
                                                 /* Bits 30-31: Reserved */

/* DCM Miscellaneous (DCMMISC) */

                                                 /* Bits 0-9: Reserved */
#define DCM_DCMMISC_DCMDBGT            (1 << 10) /* Bit 10: DBG Section Error (DCMDBGT) */
#define DCM_DCMMISC_DCMDBGE            (1 << 11) /* Bit 11: DCM ECC error on DBG sections (DCMDBGE) */
                                                 /* Bits 12-27: Reserved */
#define DCM_DCMMISC_DCMCERS            (1 << 28) /* Bits 28: DCF Client Errors (DCMCERS) */
                                                 /* Bit 29: Reserved */
#define DCM_DCMMISC_MRKLSTRCHK         (1 << 30) /* Bit 30: MRK Local Storage Check (MRKLSTRCHK) */
                                                 /* Bit 31: Reserved */

/* Debug Status and Configuration (DCMDEB) */

                                                 /* Bit 0: Reserved */
#define DCM_DCMDEB_DCM_APPDBG_STAT     (1 << 1)  /* Bit 1: DCM Authentication Engine Status (DCM_APPDBG_STAT) */
                                                 /* Bits 2-15: Reserved */
#define DCM_DCMDEB_APPDBG_STAT_SOC     (1 << 16) /* Bit 16: Application Debug Status (APPDBG_STAT_SOC) */
                                                 /* Bits 17-31: Reserved */

/* DCF Error Count (DCMEC) */

#define DCM_DCMEC_DCMECT_SHIFT         (0)       /* Bits 0-15: Error Count (DCMECT) */
#define DCM_DCMEC_DCMECT_MASK          (0xffff << DCM_DCMEC_DCMECT_SHIFT)
                                                 /* Bits 16-31: Reserved */

/* DCF Scan Report (DCMSRRn) */

#define DCM_DCMSSR_DCMDCFE_SHIFT       (0)       /* Bits 0-20: Flash Memory Address (DCMDCFE1) */
#define DCM_DCMSSR_DCMDCFE_MASK        (0x1fffff << DCM_DCMSSR1_DCMDCFE1_SHIFT)
                                                 /* Bits 21-23: Reserved */
#define DCM_DCMSSR_DCMDCFF_SHIFT       (24)      /* Bits 24-26: DCF Record Location (DCMDCFF1) */
#define DCM_DCMSSR_DCMDCFF_MASK        (0x1fffff << DCM_DCMSSR1_DCMDCFE1_SHIFT)
#define DCM_DCMSSR_DCMESF              (1 << 27) /* Bit 27: Flash Memory Error (DCMESF1) */
#define DCM_DCMSSR_DCMESD              (1 << 28) /* Bit 28: Chip Side Error (DCMESD1) */
#define DCM_DCMSSR_DCMDCFT             (1 << 29) /* Bit 29: Scanning Timeout On Flash Memory (DCMDCFT1) */
                                                 /* Bit 30-31: Reserved */

/* LC Scan Status 2 (DCMLCS2) */

#define DCM_DCMLCS2_DCMLCSS6           (1 << 0)  /* Bit 0: FA Scan Status (DCMLCSS6) */
#define DCM_DCMLCS2_DCMLCC6_SHIFT      (1)       /* Bits 1-3: FA Marking (DCMLCC6) */
#define DCM_DCMLCS2_DCMLCC6_MASK       (0x07 << DCM_DCMLCS2_DCMLCC6_SHIFT)
#  define DCM_DCMLCS2_DCMLCC6_NOSCAN   (0x00 << DCM_DCMLCS2_DCMLCC6_SHIFT) /* Not scanned yet */
#  define DCM_DCMLCS2_DCMLCC6_ACTIVE   (0x01 << DCM_DCMLCS2_DCMLCC6_SHIFT) /* Marked as active */
#  define DCM_DCMLCS2_DCMLCC6_INACTIVE (0x02 << DCM_DCMLCS2_DCMLCC6_SHIFT) /* Marked as inactive */
#  define DCM_DCMLCS2_DCMLCC6_ERASED   (0x03 << DCM_DCMLCS2_DCMLCC6_SHIFT) /* Region is erased/virgin */
#  define DCM_DCMLCS2_DCMLCC6_UNKNOWN  (0x05 << DCM_DCMLCS2_DCMLCC6_SHIFT) /* Marked as inactive by an unknown pattern */
#  define DCM_DCMLCS2_DCMLCC6_TIMEOUT  (0x06 << DCM_DCMLCS2_DCMLCC6_SHIFT) /* Scanning timed out */

#define DCM_DCMLCS2_DCMLCE6            (1 << 4)  /* Bit 4: FA ECC Errors (DCMLCE6) */
#define DCM_DCMLCS2_DCMLCFE6           (1 << 5)  /* Bit 5: Flash Memory Error Check (DCMLCFE6) */
                                                 /* Bits 6-31: Reserved */

/* Read Only GPR On Destructive Reset Register 1 (DCMROD1) */

#define DCM_GPR_DCMROD1_PCU_ISO_STATUS    (1 << 0)  /* Bit 0: PCU Input Isolation status on previous standb entry (PCU_ISO_STATUS) */
#define DCM_GPR_DCMROD1_HSE_DCF_VIO       (1 << 1)  /* Bit 1: DCF violation from HSE (HSE_DCF_VIO) */
#define DCM_GPR_DCMROD1_KEY_RESP_READY    (1 << 2)  /* Bit 2: Key Response Ready (KEY_RESP_READY) */
                                                    /* Bits 3-31: Reserved */

/* Read Only GPR On Destructive Reset Register 3 (DCMROD3) */

#define DCM_GPR_DCMROD3_CM7_0_LOCKUP      (1 << 0)  /* Bit 0: CM7_0 Core Lockup Status (CM7_0_LOCKUP) */
#define DCM_GPR_DCMROD3_CM7_1_LOCKUP      (1 << 1)  /* Bit 0: CM7_1 Core Lockup Status (CM7_1_LOCKUP) */
#define DCM_GPR_DCMROD3_HSE_LOCKUP        (1 << 2)  /* Bit 2: HSE Core Lockup Status (HSE_LOCKUP) */
#define DCM_GPR_DCMROD3_CM7_RCCU1_ALARM   (1 << 3)  /* Bit 3: Cortex M7 Cores Lockstep Error Status (CM7_RCCU1_ALARM) */
#define DCM_GPR_DCMROD3_CM7_RCCU2_ALARM   (1 << 4)  /* Bit 4: Cortex M7 Cores Redundant Lockstep Error Status (CM7_RCCU2_ALARM) */
#define DCM_GPR_DCMROD3_TCM_GSKT_ALARM    (1 << 5)  /* Bit 5: TCM IAHB Gasket Monitor Alarm Status (TCM_GSKT_ALARM) */

#define DCM_GPR_DCMROD3_DMA_SYS_GSKT_ALARM    (1 << 6) /* Bit 6: Status of IAHB gasket safety alarm from DMA system AXBS IAHB gasket (DMA_SYS_GSKT_ALARM) */
#define DCM_GPR_DCMROD3_DMA_PERIPH_GSKT_ALARM (1 << 7) /* Bit 7: Status of IAHB gasket safety alarm from DMA periph AXBS IAHB gasket (DMA_PERIPH_GSKT_ALARM) */

#define DCM_GPR_DCMROD3_SYS_AXBS_ALARM    (1 << 8)  /* Bit 8: System AXBS Safety Alarm Status (SYS_AXBS_ALARM) */
#define DCM_GPR_DCMROD3_DMA_AXBS_ALARM    (1 << 9)  /* Bit 9: DMA AXBS_Lite Safety Alarm Status (DMA_AXBS_ALARM) */
                                                    /* Bit 10: Reserved */
#define DCM_GPR_DCMROD3_HSE_GSKT_ALARM    (1 << 11) /* Bit 11: HSE IAHB Gasket Alarm Status (HSE_GSKT_ALARM) */
#define DCM_GPR_DCMROD3_QSPI_GSKT_ALARM   (1 << 12) /* Bit 12: QSPI IAHB Gasket Alarm Status (QSPI_GSKT_ALARM) */
#define DCM_GPR_DCMROD3_AIPS1_GSKT_ALARM  (1 << 13) /* Bit 13: AIPS1 IAHB Gasket Alarm Status (AIPS1_GSKT_ALARM) */
#define DCM_GPR_DCMROD3_AIPS2_GSKT_ALARM  (1 << 14) /* Bit 14: AIPS2 IAHB Gasket Alarm Status (AIPS2_GSKT_ALARM) */
#define DCM_GPR_DCMROD3_ADDR_EDC_ERR      (1 << 15) /* Bit 15: Status of integrity error on addresses for safety (ADDR_EDC_ERR) */
#define DCM_GPR_DCMROD3_DATA_EDC_ERR      (1 << 16) /* Bit 16: Status of integrity error on data for safety (DATA_EDC_ERR) */
#define DCM_GPR_DCMROD3_TCM_AXBS_ALARM    (1 << 17) /* Bit 17: TCM AHB Splitter Safety Alarm Status (TCM_AXBS_ALARM) */
#define DCM_GPR_DCMROD3_EMAC_GSKT_ALARM   (1 << 18) /* Bit 18: EMAC IAHB Gasket Alarm Status (EMAC_GSKT_ALARM) */
#define DCM_GPR_DCMROD3_PERIPH_AXBS_ALARM (1 << 19) /* Bit 19: PERIPH AXBS_Lite Safety Alarm Status (PERIPH_AXBS_ALARM) */
                                                    /* Bits 20-21: Reserved */
#define DCM_GPR_DCMROD3_LC_ERR            (1 << 22) /* Bit 22: Error in Lifecycle Scanning (LC_ERR) */
                                                    /* Bit 23: Reserved */
#define DCM_GPR_DCMROD3_PRAM1_ECC_ERR     (1 << 24) /* Bit 24: Multi bit ECC error from SRAM1 (PRAM1_ECC_ERR) */
#define DCM_GPR_DCMROD3_PRAM0_ECC_ERR     (1 << 25) /* Bit 25: Multi bit ECC error from SRAM0 (PRAM0_ECC_ERR) */

#define DCM_GPR_DCMROD3_CM7_0_DCDATA_ECC_ERR (1 << 26) /* Bit 26: Multi bit ECC error from CM7_0 DCache data memory (CM7_0_DCDATA_ECC_ERR) */
#define DCM_GPR_DCMROD3_CM7_1_DCDATA_ECC_ERR (1 << 27) /* Bit 27: Multi bit ECC error from CM7_1 DCache data memory (CM7_1_DCDATA_ECC_ERR) */
#define DCM_GPR_DCMROD3_CM7_0_DCTAG_ECC_ERR  (1 << 28) /* Bit 28: Multi bit ECC error from CM7_0 DCache tag memory (CM7_0_DCTAG_ECC_ERR) */
#define DCM_GPR_DCMROD3_CM7_1_DCTAG_ECC_ERR  (1 << 29) /* Bit 29: Multi bit ECC error from CM7_1 DCache tag memory (CM7_1_DCTAG_ECC_ERR) */
#define DCM_GPR_DCMROD3_CM7_0_ICDATA_ECC_ERR (1 << 30) /* Bit 30: Multi bit ECC error from CM7_0 ICache data memory (CM7_0_ICDATA_ECC_ERR) */
#define DCM_GPR_DCMROD3_CM7_1_ICDATA_ECC_ERR (1 << 31) /* Bit 31: Multi bit ECC error from CM7_1 ICache data memory (CM7_1_ICDATA_ECC_ERR) */

/* Read Only GPR On Destructive Reset Register 4 (DCMROD4) */

#define DCM_GPR_DCMROD4_CM7_0_ICTAG_ECC_ERR (1 << 0) /* Bit 0: Multi bit ECC error from CM7_0 ICache tag memory (CM7_0_ICTAG_ECC_ERR) */
#define DCM_GPR_DCMROD4_CM7_1_ICTAG_ECC_ERR (1 << 1) /* Bit 1: Multi bit ECC error from CM7_1 ICache tag memory (CM7_1_ICTAG_ECC_ERR) */
#define DCM_GPR_DCMROD4_CM7_0_ITCM_ECC_ERR  (1 << 2) /* Bit 2: Uncorrectable ECC error reported from CM7_0 Instruction TCM memory (CM7_0_ITCM_ECC_ERR) */
#define DCM_GPR_DCMROD4_CM7_0_DTCM0_ECC_ERR (1 << 3) /* Bit 3: Uncorrectable ECC error reported from CM7_0 Data TCM memory block 0 (CM7_0_DTCM0_ECC_ERR) */
#define DCM_GPR_DCMROD4_CM7_0_DTCM1_ECC_ERR (1 << 4) /* Bit 4: Uncorrectable ECC error reported from CM7_1 Data TCM memory block 1 (CM7_0_DTCM1_ECC_ERR) */
#define DCM_GPR_DCMROD4_CM7_1_ITCM_ECC_ERR  (1 << 5) /* Bit 5: Uncorrectable ECC error reported from CM7_1 Instruction TCM memory (CM7_1_ITCM_ECC_ERR) */
#define DCM_GPR_DCMROD4_CM7_1_DTCM0_ECC_ERR (1 << 6) /* Bit 6: Uncorrectable ECC error reported from CM7_1 Data TCM memory block 0 (CM7_1_DTCM0_ECC_ERR) */
#define DCM_GPR_DCMROD4_CM7_1_DTCM1_ECC_ERR (1 << 7) /* Bit 7: Uncorrectable ECC error reported from CM7_1 Data TCM memory block 1 (CM7_1_DTCM1_ECC_ERR) */
#define DCM_GPR_DCMROD4_DMA_TCD_RAM_ECC_ERR (1 << 8) /* Bit 8: Uncorrectable ECC error reported from DMA_TCD memory (DMA_TCD_RAM_ECC_ERR) */

#define DCM_GPR_DCMROD4_PRAM0_FCCU_ALARM  (1 << 9)  /* Bit 9: Status of PRAM0 safety alarm (PRAM0_FCCU_ALARM) */
#define DCM_GPR_DCMROD4_PRAM1_FCCU_ALARM  (1 << 10) /* Bit 10: Status of PRAM1 safety alarm (PRAM1_FCCU_ALARM) */
#define DCM_GPR_DCMROD4_HSE_RAM_ECC_ERR   (1 << 11) /* Bit 11: HSE RAM Uncorrectable ECC status (HSE_RAM_ECC_ERR) */
#define DCM_GPR_DCMROD4_PF0_CODE_ECC_ERR  (1 << 12) /* Bit 12: Flash0 Code ECC Uncorrectable Error (PF0_CODE_ECC_ERR) */
#define DCM_GPR_DCMROD4_PF0_DATA_ECC_ERR  (1 << 13) /* Bit 13: Flash0 Data ECC Uncorrectable Error (PF0_DATA_ECC_ERR) */
#define DCM_GPR_DCMROD4_PF1_CODE_ECC_ERR  (1 << 14) /* Bit 14: Flash1 Code ECC Uncorrectable Error (PF1_CODE_ECC_ERR) */
#define DCM_GPR_DCMROD4_PF1_DATA_ECC_ERR  (1 << 15) /* Bit 15: Flash1 Data ECC Uncorrectable Error (PF1_DATA_ECC_ERR) */
#define DCM_GPR_DCMROD4_PF2_CODE_ECC_ERR  (1 << 16) /* Bit 16: Flash2 Code ECC Uncorrectable Error (PF2_CODE_ECC_ERR) */
#define DCM_GPR_DCMROD4_PF2_DATA_ECC_ERR  (1 << 17) /* Bit 17: Flash2 Data ECC Uncorrectable Error (PF2_DATA_ECC_ERR) */
#define DCM_GPR_DCMROD4_FLASH_EDC_ERR     (1 << 18) /* Bit 18: Status of flash ECC correction error through EDC reported by FMU (FLASH_EDC_ERR) */

#define DCM_GPR_DCMROD4_FLASH_ADDR_ENC_ERR (1 << 19) /* Bit 19: Flash Address Encode Error (FLASH_ADDR_ENC_ERR) */

#define DCM_GPR_DCMROD4_FLASH_REF_ERR     (1 << 20) /* Bit 20: Flash reference current loss or read voltage error while previous read(s) (FLASH_REF_ERR) */
#define DCM_GPR_DCMROD4_FLASH_RST_ERR     (1 << 21) /* Bit 21: Flash Reset Error Status (FLASH_RST_ERR) */
#define DCM_GPR_DCMROD4_FLASH_SCAN_ERR    (1 << 22) /* Bit 22: Error while DCM flash scanning process due to invalid data (FLASH_SCAN_ERR) */
                                                    /* Bit 23: Reserved */
#define DCM_GPR_DCMROD4_FLASH_ECC_ERR     (1 << 24) /* Bit 24: ECC Error from Flash Controller (FLASH_ECC_ERR) */
#define DCM_GPR_DCMROD4_FLASH_ACCESS_ERR  (1 << 25) /* Bit 25: Transaction Monitor Mismatch Error from Flash Controller (FLASH_ACCESS_ERR) */
#define DCM_GPR_DCMROD4_VDD1P1_GNG_ERR    (1 << 26) /* Bit 26: Go/no-go indicator for VDD1PD1 (double bond) supply going to PLL (VDD1P1_GNG_ERR) */
#define DCM_GPR_DCMROD4_VDD2P5_GNG_ERR    (1 << 27) /* Bit 27: Go/no-go indicator for VDD_HV_FLA (double bond) supply going to FXOSC and PLL (VDD2P5_GNG_ERR) */
                                                    /* Bit 28: Reserved */

#define DCM_GPR_DCMROD4_TEST_ACTIVATION_0_ERR (1 << 29) /* Bit 29: Accidental Partial Test Activation (TEST_ACTIVATION_0_ERR) */
#define DCM_GPR_DCMROD4_TEST_ACTIVATION_1_ERR (1 << 30) /* Bit 30: Accidental Partial Test Activation (TEST_ACTIVATION_1_ERR) */

                                                    /* Bit 31: Reserved */

/* Read Only GPR On Destructive Reset Register 5 (DCMROD5) */

                                                    /* Bit 0: Reserved */
#define DCM_GPR_DCMROD5_INTM_0_ERR        (1 << 1)  /* Bit 1: Interrupt monitor0 error reported by INTM (INTM_0_ERR) */
#define DCM_GPR_DCMROD5_INTM_1_ERR        (1 << 2)  /* Bit 2: Interrupt monitor1 error reported by INTM (INTM_0_ERR) */
#define DCM_GPR_DCMROD5_INTM_2_ERR        (1 << 3)  /* Bit 3: Interrupt monitor2 error reported by INTM (INTM_0_ERR) */
#define DCM_GPR_DCMROD5_INTM_3_ERR        (1 << 4)  /* Bit 4: Interrupt monitor3 error reported by INTM (INTM_0_ERR) */
#define DCM_GPR_DCMROD5_SW_NCF_0          (1 << 5)  /* Bit 5: Status of DCMRWF1[FCCU_SW_NCF0] (SW_NCF_0) */
#define DCM_GPR_DCMROD5_SW_NCF_1          (1 << 6)  /* Bit 6: Status of DCMRWF1[FCCU_SW_NCF1] (SW_NCF_1) */
#define DCM_GPR_DCMROD5_SW_NCF_2          (1 << 7)  /* Bit 7: Status of DCMRWF1[FCCU_SW_NCF2] (SW_NCF_2) */
#define DCM_GPR_DCMROD5_SW_NCF_3          (1 << 8)  /* Bit 8: Status of DCMRWF1[FCCU_SW_NCF3] (SW_NCF_3) */
#define DCM_GPR_DCMROD5_STCU_NCF          (1 << 9)  /* Bit 9: STCU non-critical fault / BIST result error (STCU_NCF) */

#define DCM_GPR_DCMROD5_MBIST_ACTIVATION_ERR (1 << 10) /* Bit 10: Indicates an accidental backdoor access on memories (MBIST_ACTIVATION_ERR) */

#define DCM_GPR_DCMROD5_STCU_BIST_USER_CF (1 << 11) /* Bit 11: L/M BIST enabled accidentally (STCU_BIST_USER_CF) */
#define DCM_GPR_DCMROD5_MTR_BUS_ERR       (1 << 12) /* Bit 12: Fault reported due to illegal access on MTR (MTR_BUS_ERR) */

#define DCM_GPR_DCMROD5_DEBUG_ACTIVATION_ERR (1 << 13) /* Bit 13: Monitoring of unintended debug activation (DEBUG_ACTIVATION_ERR) */

#define DCM_GPR_DCMROD5_TCM_RDATA_EDC_ERR (1 << 14) /* Bit 14: Integrity (EDC) error on TCM read data for safety (TCM_RDATA_EDC_ERR) */

#define DCM_GPR_DCMROD5_EMAC_RDATA_EDC_ERR (1 << 15) /* Bit 15: Integrity (EDC) error on EMAC read data for safety (EMAC_RDATA_EDC_ERR) */

                                                    /* Bit 16: Reserved */
#define DCM_GPR_DCMROD5_DMA_RDATA_EDC_ERR (1 << 17) /* Bit 17: Integrity (EDC) error on eDMA read data for safety (DMA_RDATA_EDC_ERR) */

#define DCM_GPR_DCMROD5_CM7_1_AHBP_RDATA_EDC_ERR (1 << 18) /* Bit 18: Integrity error on CM7_1 peripheral read data for safety (CM7_1_AHBP_RDATA_EDC_ERR) */
#define DCM_GPR_DCMROD5_CM7_1_AHBM_RDATA_EDC_ERR (1 << 19) /* Bit 19: Integrity error on CM7_1 main read data for safety (CM7_1_AHBM_RDATA_EDC_ERR) */
#define DCM_GPR_DCMROD5_CM7_0_AHBP_RDATA_EDC_ERR (1 << 20) /* Bit 18: Integrity error on CM7_0 peripheral read data for safety (CM7_0_AHBP_RDATA_EDC_ERR) */
#define DCM_GPR_DCMROD5_CM7_0_AHBM_RDATA_EDC_ERR (1 << 21) /* Bit 19: Integrity error on CM7_0 main read data for safety (CM7_0_AHBM_RDATA_EDC_ERR) */

#define DCM_GPR_DCMROD5_HSE_RDATA_EDC_ERR (1 << 22) /* Bit 22: Integrity (EDC) error on HSE read data for safety (HSE_RDATA_EDC_ERR) */
                                                    /* Bits 23-31: Reserved */

/* Read Only GPR On Functional Reset Register 1 (DCMROF1) */

#define DCM_GPR_DCMROF1_EMAC_MDC_CHID_0   (1 << 0)  /* Bit 0: EMAC DMA Channel ID0 Status (EMAC_MDC_CHID_0) */
#define DCM_GPR_DCMROF1_EMAC_MDC_CHID_1   (1 << 1)  /* Bit 1: EMAC DMA Channel ID0 Status (EMAC_MDC_CHID_0) */
                                                    /* Bits 2-31: Reserved */

/* Read Only GPR On Functional Reset Register n (DCMROFn, n=2..17) */

#define DCM_GPR_DCMROF2_17_DCF_SDID_SHIFT (0)       /* Bits 0-31: Configuration bits of DCF client SDID x (DCF_SDIDx) */
#define DCM_GPR_DCMROF2_17_DCF_SDID_MASK  (0xffffffff << DCM_GPR_DCMROF2_17_DCF_SDID_SHIFT)

/* Read Only GPR On Functional Reset Register 19 (DCMROF19) */

                                                    /* Bits 0-28: Reserved */
#define DCM_GPR_DCMROF19_LOCKSTEP_EN      (1 << 29) /* Bit 29: Lockstep Enable (LOCKSTEP_EN) */
#define DCM_GPR_DCMROF19_DCM_DONE         (1 << 30) /* Bit 30: DCM Done (DCM_DONE) */

#define DCM_GPR_DCMROF19_FCCU_EOUT_DEDICATED (1 << 31) /* Bit 31: FCCU EOUT Dedicated (FCCU_EOUT_DEDICATED) */

/* Read Only GPR On Functional Reset Register 20 (DCMROF20) */

#define DCM_GPR_DCMROF20_POR_WDG_EN       (1 << 0)  /* Bit 0: Indicates the status of POR_WDG as configured in DCF record (POR_WDG_EN) */
#define DCM_GPR_DCMROF20_LMAUTO_DIS       (1 << 1)  /* Bit 1: PMC last mile automatic crossover from boot regulation feature support (LMAUTO_DIS) */
#define DCM_GPR_DCMROF20_CM7_TCM_WS_EN    (1 << 2)  /* Bit 2: Status of CM7 DTCM and ITCM waitstates as configured in DCF record (CM7_TCM_WS_EN) */

#define DCM_GPR_DCMROF20_DMA_AXBS_IAHB_BYP (1 << 3) /* Bit 3: Status of DMA AXBS IAHB gasket as configured in DCF record (DMA_AXBS_IAHB_BYP) */
                                                    /* Bit 4: Reserved */
#define DCM_GPR_DCMROF20_QSPI_IAHB_BYP    (1 << 5)  /* Bit 5: Status of QSPI IAHB gasket as configured in DCF record (QSPI_IAHB_BYP) */
#define DCM_GPR_DCMROF20_AIPS_IAHB_BYP    (1 << 6)  /* Bit 6: Status of AIPS1/2 IAHB gasket as configured in DCF record (AIPS_IAHB_BYP) */
                                                    /* Bits 7-17: Reserved */

#define DCM_GPR_DCMROF20_DCF_DEST_RST_ESC_SHIFT (18) /* Bits 18-31: Destructive Reset Escalation (DCF_DEST_RST_ESC) */
#define DCM_GPR_DCMROF20_DCF_DEST_RST_ESC_MASK  (0x3fff << DCM_GPR_DCMROF20_DCF_DEST_RST_ESC_SHIFT)
#  define DCM_GPR_DCMROF20_DCF_DEST_RST_ESC_DIS (0x0000 << DCM_GPR_DCMROF20_DCF_DEST_RST_ESC_SHIFT) /* Destructive Reset Escalation disabled */
#  define DCM_GPR_DCMROF20_DCF_DEST_RST_ESC_EN  (0x0001 << DCM_GPR_DCMROF20_DCF_DEST_RST_ESC_SHIFT) /* Destructive Rest Escalation enabled */

/* Read Only GPR On Functional Reset Register 21 (DCMROF21) */

#define DCM_GPR_DCMROF21_DCF_DEST_RST_ESC_SHIFT (0) /* Bits 0-17: Destructive Reset Escalation (DCF_DEST_RST_ESC) */
#define DCM_GPR_DCMROF21_DCF_DEST_RST_ESC_MASK  (0x03ffff << DCM_GPR_DCMROF21_DCF_DEST_RST_ESC_SHIFT)
#  define DCM_GPR_DCMROF21_DCF_DEST_RST_ESC_DIS (0x000000 << DCM_GPR_DCMROF21_DCF_DEST_RST_ESC_SHIFT) /* Destructive Reset Escalation disabled */
#  define DCM_GPR_DCMROF21_DCF_DEST_RST_ESC_EN  (0x000001 << DCM_GPR_DCMROF21_DCF_DEST_RST_ESC_SHIFT) /* Destructive Rest Escalation enabled */

                                                    /* Bit 18: Reserved */

#define DCM_GPR_DCMROF21_HSE_CLK_MODE_OPTION_SHIFT  (19) /* Bits 19-20: HSE Clock Mode Option (HSE_CLK_MODE_OPTION) */
#define DCM_GPR_DCMROF21_HSE_CLK_MODE_OPTION_MASK   (0x03 << DCM_GPR_DCMROF21_HSE_CLK_MODE_OPTION_SHIFT)
#  define DCM_GPR_DCMROF21_HSE_CLK_MODE_OPTION_A    (0x00 << DCM_GPR_DCMROF21_HSE_CLK_MODE_OPTION_SHIFT) /* Applicable for clocking option A */
#  define DCM_GPR_DCMROF21_HSE_CLK_MODE_OPTION_CDEF (0x01 << DCM_GPR_DCMROF21_HSE_CLK_MODE_OPTION_SHIFT) /* Applicable for clocking options C, D, E, E2 and F */
#  define DCM_GPR_DCMROF21_HSE_CLK_MODE_OPTION_B    (0x02 << DCM_GPR_DCMROF21_HSE_CLK_MODE_OPTION_SHIFT) /* Applicable for clocking option B */

/* Read Write GPR On Power On Reset Register 1 (DCMRWP1) */

                                                    /* Bits 0-2: Reserved */
#define DCM_GPR_DCMRWP1_CLKOUT_STANDBY    (1 << 3)  /* Bit 3: Clockout standby expose over functional and destructive reset (CLKOUT_STANDBY) */
                                                    /* Bits 4-7: Reserved */
#define DCM_GPR_DCMRWP1_STANDBY_PWDOG_DIS (1 << 8)  /* Bit 8: Disables the standby entry and exit monitoring window of the POR WDOG (STANDBY_PWDOG_DIS) */

#define DCM_GPR_DCMRWP1_POR_WDOG_TRIM_SHIFT    (9)  /* Bits 9-10: Trims for POR WDG timeout value (POR_WDOG_TRIM) */
#define DCM_GPR_DCMRWP1_POR_WDOG_TRIM_MASK     (0x03 << DCM_GPR_DCMRWP1_POR_WDOG_TRIM_SHIFT)
#  define DCM_GPR_DCMRWP1_POR_WDOG_TRIM_6_25MS (0x00 << DCM_GPR_DCMRWP1_POR_WDOG_TRIM_SHIFT) /* POR WDOG Timeout = 06.25ms */
#  define DCM_GPR_DCMRWP1_POR_WDOG_TRIM_12_5MS (0x01 << DCM_GPR_DCMRWP1_POR_WDOG_TRIM_SHIFT) /* POR WDOG Timeout = 12.50ms */
#  define DCM_GPR_DCMRWP1_POR_WDOG_TRIM_25MS   (0x02 << DCM_GPR_DCMRWP1_POR_WDOG_TRIM_SHIFT) /* POR WDOG Timeout = 25.00ms */
#  define DCM_GPR_DCMRWP1_POR_WDOG_TRIM_50MS   (0x03 << DCM_GPR_DCMRWP1_POR_WDOG_TRIM_SHIFT) /* POR WDOG Timeout = 50.00ms */

                                                    /* Bits 11-31: Reserved */

/* Read Write GPR On Power On Reset Register 3 (DCMRWP3) */

                                                    /* Bits 0-8: Reserved */
#define DCM_GPR_DCMRWP3_DEST_RST9_AS_IPI  (1 << 9)  /* Bit 9: Configures a destructive reset to interrupt (DEST_RST9_AS_IPI) */
                                                    /* Bits 10-31: Reserved */

/* Read Write GPR On Destructive Reset Register 2 (DCMRWD2) */

                                                    /* Bits 0-6: Reserved */

#define DCM_GPR_DCMRWD2_EOUT_STAT_DUR_STEST (1 << 7) /* Bit 7: Controls the EOUT state during selftest (EOUT_STAT_DUR_STEST) */

                                                    /* Bits 8-31: Reserved */

/* Read Write GPR On Destructive Reset Register 3 (DCMRWD3) */

#define DCM_GPR_DCMRWD3_CM7_0_LOCKUP_EN   (1 << 0)  /* Bit 0: Enable fault monitoring at FCCU NCF 0 for CM7_0 core lockup (CM7_0_LOCKUP_EN) */
#define DCM_GPR_DCMRWD3_CM7_1_LOCKUP_EN   (1 << 1)  /* Bit 1: Enable fault monitoring at FCCU NCF 0 for CM7_1 core lockup (CM7_1_LOCKUP_EN) */
                                                    /* Bit 2: Reserved */
#define DCM_GPR_DCMRWD3_CM7_RCCU1_ALARM_EN (1 << 3) /* Bit 3: Enable fault monitoring at FCCU NCF 0 for Cortex M7 cores lockstep error (CM7_RCCU1_ALARM_EN) */
#define DCM_GPR_DCMRWD3_CM7_RCCU2_ALARM_EN (1 << 4) /* Bit 4: Enable fault monitoring at FCCU NCF 0 for Cortex M7 cores redundant lockstep error (CM7_RCCU2_ALARM_EN) */

#define DCM_GPR_DCMRWD3_TCM_GSKT_ALARM_EN (1 << 5)  /* Bit 5: Enable fault monitoring at FCCU NCF 1 for TCM IAHB Gasket monitor alarm (TCM_GSKT_ALRM_EN) */

#define DCM_GPR_DCMRWD3_DMA_SYS_GSKT_ALARM_EN    (1 << 6) /* Bit 6: Enable fault monitoring at FCCU NCF 1 for IAHB gasket safety alarm from DMA system AXBS IAHB gasket (DMA_SYS_GSKT_ALARM_EN) */
#define DCM_GPR_DCMRWD3_DMA_PERIPH_GSKT_ALARM_EN (1 << 7) /* Bit 7: Enable fault monitoring at FCCU NCF 1 for IAHB gasket safety alarm from DMA periph AXBS IAHB gasket (DMA_PERIPH_GSKT_ALARM_EN) */

#define DCM_GPR_DCMRWD3_SYS_AXBS_ALARM_EN (1 << 8)  /* Bit 8: Enable fault monitoring at FCCU NCF 1 for system AXBS safety alarm (SYS_AXBS_ALARM_EN) */
#define DCM_GPR_DCMRWD3_DMA_AXBS_ALARM_EN (1 << 9)  /* Bit 9: Enable fault monitoring at FCCU NCF 1 for DMA AXBS_Lite safety alarm (DMA_AXBS_ALARM_EN) */
                                                    /* Bit 10: Reserved */
#define DCM_GPR_DCMRWD3_HSE_GSKT_ALARM_EN (1 << 11) /* Bit 11: Enable fault monitoring at FCCU NCF 1 for HSE IAHB gasket alarm (HSE_GSKT_ALARM_EN) */

#define DCM_GPR_DCMRWD3_QSPI_GSKT_ALARM_EN  (1 << 12) /* Bit 12: Enable fault monitoring at FCCU NCF 1 for QSPI IAHB gasket alarm (QSPI_GSKT_ALARM_EN) */
#define DCM_GPR_DCMRWD3_AIPS1_GSKT_ALARM_EN (1 << 13) /* Bit 13: Enable fault monitoring at FCCU NCF 1 for AIPS1 IAHB gasket alarm (AIPS1_GSKT_ALARM_EN) */
#define DCM_GPR_DCMRWD3_AIPS2_GSKT_ALARM_EN (1 << 14) /* Bit 14: Enable fault monitoring at FCCU NCF 1 for AIPS2 IAHB gasket alarm (AIPS2_GSKT_ALARM_EN) */

#define DCM_GPR_DCMRWD3_ADDR_EDC_ERR_EN   (1 << 15) /* Bit 15: Enable fault monitoring at FCCU NCF 1 for integrity error on address for safety (ADDR_EDC_ERR_EN) */
#define DCM_GPR_DCMRWD3_DATA_EDC_ERR_EN   (1 << 16) /* Bit 16: Enable fault monitoring at FCCU NCF 1 for integrity error on data for safety (DATA_EDC_ERR_EN) */
#define DCM_GPR_DCMRWD3_TCM_AXBS_ALARM_EN (1 << 17) /* Bit 17: Enable fault monitoring at FCCU NCF 1 for TCM AHB splitter safety alarm (TCM_AXBS_ALARM_EN) */

#define DCM_GPR_DCMRWD3_EMAC_GSKT_ALARM_EN   (1 << 18) /* Bit 18: Enable fault monitoring at FCCU NCF 1 for EMAC IAHB gasket alarm (EMAC_GSKT_ALARM_EN) */
#define DCM_GPR_DCMRWD3_PERIPH_AXBS_ALARM_EN (1 << 19) /* Bit 19: Enable fault monitoring at FCCU NCF 1 for PERIPH AXBS_Lite safety alarm (PERIPH_AXBS_ALARM_EN) */

                                                    /* Bits 20-21: Reserved */
#define DCM_GPR_DCMRWD3_LC_ERR_EN         (1 << 22) /* Bit 22: Enable fault monitoring at FCCU NCF 3 for error in lifecycle scanning (LC_ERR_EN) */
                                                    /* Bits 23: Reserved */

#define DCM_GPR_DCMRWD3_PRAM1_ECC_ERR_EN  (1 << 24) /* Bit 24: Enable fault monitoring at FCCU NCF 2 for multi bit ECC error from SRAM1 (PRAM1_ECC_ERR_EN) */
#define DCM_GPR_DCMRWD3_PRAM0_ECC_ERR_EN  (1 << 25) /* Bit 25: Enable fault monitoring at FCCU NCF 2 for multi bit ECC error from SRAM0 (PRAM0_ECC_ERR_EN) */

#define DCM_GPR_DCMRWD3_CM7_0_DCDATA_ECC_ERR_EN (1 << 26) /* Bit 26: Enable fault monitoring at FCCU NCF 2 for multi bit ECC error from CM7_0 DCache data memory (CM7_0_DCDATA_ECC_ERR_EN) */ 
#define DCM_GPR_DCMRWD3_CM7_1_DCDATA_ECC_ERR_EN (1 << 27) /* Bit 27: Enable fault monitoring at FCCU NCF 2 for multi bit ECC error from CM7_1 DCache data memory (CM7_1_DCDATA_ECC_ERR_EN) */ 
#define DCM_GPR_DCMRWD3_CM7_0_DCTAG_ECC_ERR_EN  (1 << 28) /* Bit 28: Enable fault monitoring at FCCU NCF 2 for multi bit ECC error from CM7_0 DCache tag memory (CM7_0_DCTAG_ECC_ERR_EN) */ 
#define DCM_GPR_DCMRWD3_CM7_1_DCTAG_ECC_ERR_EN  (1 << 29) /* Bit 29: Enable fault monitoring at FCCU NCF 2 for multi bit ECC error from CM7_1 DCache tag memory (CM7_1_DCTAG_ECC_ERR_EN) */ 
#define DCM_GPR_DCMRWD3_CM7_0_ICDATA_ECC_ERR_EN (1 << 30) /* Bit 30: Enable fault monitoring at FCCU NCF 2 for multi bit ECC error from CM7_0 ICache data memory (CM7_0_DCDATA_ECC_ERR_EN) */ 
#define DCM_GPR_DCMRWD3_CM7_1_ICDATA_ECC_ERR_EN (1 << 31) /* Bit 31: Enable fault monitoring at FCCU NCF 2 for multi bit ECC error from CM7_1 ICache data memory (CM7_1_DCDATA_ECC_ERR_EN) */ 

/* Read Write GPR On Destructive Reset Register 4 (DCMRWD4) */

#define DCM_GPR_DCMRWD4_CM7_0_ICTAG_ECC_ERR_EN (1 << 0)  /* Bit 0: Enable fault monitoring at FCCU NCF 2 for multi bit ECC error from CM7_0 ICache tag memory (CM7_0_ICTAG_ECC_ERR_EN) */
#define DCM_GPR_DCMRWD4_CM7_1_ICTAG_ECC_ERR_EN (1 << 1)  /* Bit 1: Enable fault monitoring at FCCU NCF 2 for multi bit ECC error from CM7_1 ICache tag memory (CM7_1_ICTAG_ECC_ERR_EN) */
#define DCM_GPR_DCMRWD4_CM7_0_ITCM_ECC_ERR_EN  (1 << 2)  /* Bit 2: Enable fault monitoring at FCCU NCF 2 for uncorrectable ECC error from CM7_0 Instruction TCM memory (CM7_0_ITCM_ECC_ERR_EN) */
#define DCM_GPR_DCMRWD4_CM7_0_DTCM0_ECC_ERR_EN (1 << 3)  /* Bit 3: Enable fault monitoring at FCCU NCF 2 for uncorrectable ECC error from CM7_0 Data TCM memory block 0 (CM7_0_DTCM0_ECC_ERR_EN) */
#define DCM_GPR_DCMRWD4_CM7_0_DTCM1_ECC_ERR_EN (1 << 4)  /* Bit 4: Enable fault monitoring at FCCU NCF 2 for uncorrectable ECC error from CM7_0 Data TCM memory block 1 (CM7_0_DTCM1_ECC_ERR_EN) */
#define DCM_GPR_DCMRWD4_CM7_1_ITCM_ECC_ERR_EN  (1 << 5)  /* Bit 5: Enable fault monitoring at FCCU NCF 2 for uncorrectable ECC error from CM7_1 Instruction TCM memory (CM7_1_ITCM_ECC_ERR_EN) */
#define DCM_GPR_DCMRWD4_CM7_1_DTCM0_ECC_ERR_EN (1 << 6)  /* Bit 6: Enable fault monitoring at FCCU NCF 2 for uncorrectable ECC error from CM7_1 Data TCM memory block 0 (CM7_1_DTCM0_ECC_ERR_EN) */
#define DCM_GPR_DCMRWD4_CM7_1_DTCM1_ECC_ERR_EN (1 << 7)  /* Bit 7: Enable fault monitoring at FCCU NCF 2 for uncorrectable ECC error from CM7_1 Data TCM memory block 1 (CM7_1_DTCM1_ECC_ERR_EN) */
#define DCM_GPR_DCMRWD4_DMA_TCD_RAM_ECC_ERR_EN (1 << 8)  /* Bit 8: Enable fault monitoring at FCCU NCF 2 for uncorrectable ECC error reported from DMA_TCD memory (DMA_TCD_RAM_ECC_ERR_EN) */
#define DCM_GPR_DCMRWD4_PRAM0_FCCU_ALARM_EN    (1 << 9)  /* Bit 9: Enable fault monitoring at FCCU NCF 2 for PRAM0 safety alarm (PRAM0_FCCU_ALARM_EN) */
#define DCM_GPR_DCMRWD4_PRAM1_FCCU_ALARM_EN    (1 << 10) /* Bit 10: Enable fault monitoring at FCCU NCF 2 for PRAM1 safety alarm (PRAM1_FCCU_ALARM_EN) */
#define DCM_GPR_DCMRWD4_HSE_RAM_ECC_ERR_EN     (1 << 11) /* Bit 11: Enable fault monitoring at FCCU NCF 2 for HSE RAM Uncorrectable ECC (HSE_RAM_ECC_ERR_EN) */
#define DCM_GPR_DCMRWD4_PF0_CODE_ECC_ERR_EN    (1 << 12) /* Bit 12: Enable fault monitoring at FCCU NCF 3 for Flash0 code ECC uncorrectable error (PF0_CODE_ECC_ERR_EN) */
#define DCM_GPR_DCMRWD4_PF0_DATA_ECC_ERR_EN    (1 << 13) /* Bit 13: Enable fault monitoring at FCCU NCF 3 for Flash0 data ECC uncorrectable error (PF0_DATA_ECC_ERR_EN) */
#define DCM_GPR_DCMRWD4_PF1_CODE_ECC_ERR_EN    (1 << 14) /* Bit 14: Enable fault monitoring at FCCU NCF 3 for Flash1 code ECC uncorrectable error (PF1_CODE_ECC_ERR_EN) */
#define DCM_GPR_DCMRWD4_PF1_DATA_ECC_ERR_EN    (1 << 15) /* Bit 15: Enable fault monitoring at FCCU NCF 3 for Flash1 data ECC uncorrectable error (PF1_DATA_ECC_ERR_EN) */
#define DCM_GPR_DCMRWD4_PF2_CODE_ECC_ERR_EN    (1 << 16) /* Bit 16: Enable fault monitoring at FCCU NCF 3 for Flash2 code ECC uncorrectable error (PF2_CODE_ECC_ERR_EN) */
#define DCM_GPR_DCMRWD4_PF2_DATA_ECC_ERR_EN    (1 << 17) /* Bit 17: Enable fault monitoring at FCCU NCF 3 for Flash2 data ECC uncorrectable error (PF2_DATA_ECC_ERR_EN) */

#define DCM_GPR_DCMRWD4_FLASH_EDC_ERR_EN  (1 << 18) /* Bit 18: Enable fault monitoring at FCCU NCF 3 for Flash ECC correction error through EDC reported by FMU (FLASH_EDC_ERR_EN) */

#define DCM_GPR_DCMRWD4_FLASH_ADDR_ENC_ERR_EN  (1 << 19) /* Bit 19: Enable fault monitoring at FCCU NCF 3 for flash address encode error (FLASH_ADDR_ENC_ERR_EN) */

#define DCM_GPR_DCMRWD4_FLASH_REF_ERR_EN  (1 << 20) /* Bit 20: Enable fault monitoring at FCCU NCF 3 for flash reference current loss or read voltage error while previous read(s) (FLASH_REF_ERR_EN) */
#define DCM_GPR_DCMRWD4_FLASH_RST_ERR_EN  (1 << 21) /* Bit 21: Enable fault monitoring at FCCU NCF 3 for flash reset error (FLASH_RST_ERR_EN) */
#define DCM_GPR_DCMRWD4_FLASH_SCAN_ERR_EN (1 << 22) /* Bit 22: Enable fault monitoring at FCCU NCF 3 for error while DCM flash scanning process due to invalid data (FLASH_SCAN_ERR_EN) */
                                                    /* Bit 23: Reserved */
#define DCM_GPR_DCMRWD4_FLASH_ECC_ERR_EN  (1 << 24) /* Bit 24: Enable fault monitoring at FCCU NCF 3 for ECC error from Flash Controller (FLASH_ECC_ERR_EN) */

#define DCM_GPR_DCMRWD4_FLASH_ACCESS_ERR_EN (1 << 25) /* Bit 25: Enable fault monitoring at FCCU NCF 3 for transaction monitor mismatch (FLASH_ACCESS_ERR_EN) */

#define DCM_GPR_DCMRWD4_VDD1P1_GNG_ERR_EN (1 << 26) /* Bit 26: Enable fault monitoring at FCCU NCF 4 for Go/No-go indicator for VDD1PD1 (double bond) supply going to PLL (VDD1P1_GNG_ERR_EN) */
#define DCM_GPR_DCMRWD4_VDD2P5_GNG_ERR_EN (1 << 27) /* Bit 27: Enable fault monitoring at FCCU NCF 4 for Go/No-go indicator for VDD_HV_FLA (double bond) supply going to FXOSC and PLL (VDD2P5_GNG_ERR_EN) */
                                                    /* Bit 28: Reserved */

#define DCM_GPR_DCMRWD4_TEST_ACTIVATION_0_ERR_EN (1 << 29) /* Bit 29: Enable fault monitoring at FCCU NCF 5 for accidental partial test activation (TEST_ACTIVATION_0_ERR_EN) */
#define DCM_GPR_DCMRWD4_TEST_ACTIVATION_1_ERR_EN (1 << 30) /* Bit 30: Enable fault monitoring at FCCU NCF 5 for accidental partial test activation (TEST_ACTIVATION_1_ERR_EN) */

                                                    /* Bit 31: Reserved */

/* Read Write GPR On Destructive Reset Register 5 (DCMRWD5) */

                                                    /* Bit 0: Reserved */
#define DCM_GPR_DCMRWD5_INTM_0_ERR_EN     (1 << 1)  /* Bit 1: Enable fault monitoring at FCCU NCF 6 for interrupt monitor 0 error reported by INTM (INTM_0_ERR_EN) */
#define DCM_GPR_DCMRWD5_INTM_1_ERR_EN     (1 << 2)  /* Bit 2: Enable fault monitoring at FCCU NCF 6 for interrupt monitor 1 error reported by INTM (INTM_1_ERR_EN) */
#define DCM_GPR_DCMRWD5_INTM_2_ERR_EN     (1 << 3)  /* Bit 3: Enable fault monitoring at FCCU NCF 6 for interrupt monitor 2 error reported by INTM (INTM_2_ERR_EN) */
#define DCM_GPR_DCMRWD5_INTM_3_ERR_EN     (1 << 4)  /* Bit 4: Enable fault monitoring at FCCU NCF 6 for interrupt monitor 3 error reported by INTM (INTM_3_ERR_EN) */
#define DCM_GPR_DCMRWD5_SW_NCF_0_EN       (1 << 5)  /* Bit 5: Enable fault monitoring at FCCU NCF 7 for Software NFC0 (SW_NCF_0_EN) */
#define DCM_GPR_DCMRWD5_SW_NCF_1_EN       (1 << 6)  /* Bit 6: Enable fault monitoring at FCCU NCF 7 for Software NFC1 (SW_NCF_1_EN) */
#define DCM_GPR_DCMRWD5_SW_NCF_2_EN       (1 << 7)  /* Bit 7: Enable fault monitoring at FCCU NCF 7 for Software NFC2 (SW_NCF_2_EN) */
#define DCM_GPR_DCMRWD5_SW_NCF_3_EN       (1 << 8)  /* Bit 8: Enable fault monitoring at FCCU NCF 7 for Software NFC3 (SW_NCF_3_EN) */
#define DCM_GPR_DCMRWD5_STCU_NCF_EN       (1 << 9)  /* Bit 9: Enable fault monitoring at FCCU NCF 5 for STCU non-critical fault / BIST result error (STCU_NCF_EN) */

#define DCM_GPR_DCMRWD5_MBIST_ACTIVATION_ERR_EN (1 << 10) /* Bit 10: Enable fault monitoring at FCCU NCF 5 for accidental backdoor access on memories (MBIST_ACTIVATION_ERR_EN) */
#define DCM_GPR_DCMRWD5_STCU_BIST_USER_CF_EN    (1 << 11) /* Bit 11: Enable fault monitoring at FCCU NCF 5 for L/M BIST enabled accidentally (STCU_BIST_USER_CF_EN) */

#define DCM_GPR_DCMRWD5_MTR_BUS_ERR_EN    (1 << 12) /* Bit 12: Enable fault monitoring at FCCU NFC 5 for fault reported due to illegal access on MTR (MTR_BUS_ERR_EN) */

#define DCM_GPR_DCMRWD5_DEBUG_ACTIVATION_ERR_EN (1 << 13) /* Bit 13: Enable fault monitoring at FCCU NCF 5 for monitoring of unintended debug activation (DEBUG_ACTIVATION_ERR_EN) */
#define DCM_GPR_DCMRWD5_TCM_RDATA_EDC_ERR_EN    (1 << 14) /* Bit 14: Enable fault monitoring at FCCU NCF 1 for integrity (EDC) error on TCM read data for safety (TCM_RDATA_EDC_ERR_EN) */
#define DCM_GPR_DCMRWD5_EMAC_RDATA_EDC_ERR_EN   (1 << 15) /* Bit 15: Enable fault monitoring at FCCU NCF 1 for integrity (EDC) error on EMAC read data for safety (EMAC_RDATA_EDC_ERR_EN) */

                                                    /* Bit 16: Reserved */

#define DCM_GPR_DCMRWD5_DMA_RDATA_EDC_ERR_EN        (1 << 17) /* Bit 17: Enable fault monitoring at FCCU NCF 1 for integrity (EDC) error on eDMA read data for safety (DMA_RDATA_EDC_ERR_EN) */
#define DCM_GPR_DCMRWD5_CM7_1_AHBP_RDATA_EDC_ERR_EN (1 << 18) /* Bit 18: Enable fault monitoring at FCCU NCF 1 for integrity error on CM7_1 peripheral read data for safety (CM7_1_AHBP_RDATA_EDC_ERR_EN) */
#define DCM_GPR_DCMRWD5_CM7_1_AHBM_RDATA_EDC_ERR_EN (1 << 19) /* Bit 19: Enable fault monitoring at FCCU NCF 1 for integrity error on CM7_1 main read data for safety (CM7_1_AHBM_RDATA_EDC_ERR_EN) */
#define DCM_GPR_DCMRWD5_CM7_0_AHBP_RDATA_EDC_ERR_EN (1 << 20) /* Bit 20: Enable fault monitoring at FCCU NCF 1 for integrity error on CM7_0 peripheral read data for safety (CM7_0_AHBP_RDATA_EDC_ERR_EN) */
#define DCM_GPR_DCMRWD5_CM7_0_AHBM_RDATA_EDC_ERR_EN (1 << 21) /* Bit 21: Enable fault monitoring at FCCU NCF 1 for integrity error on CM7_0 main read data for safety (CM7_0_AHBM_RDATA_EDC_ERR_EN) */
#define DCM_GPR_DCMRWD5_HSE_RDATA_EDC_ERR_EN        (1 << 22) /* Bit 22: Enable fault monitoring at FCCU NCF 1 for integrity (EDC) error on HSE read data for safety (HSE_RDATA_EDC_ERR_EN) */

                                                    /* Bits 23-31: Reserved */

/* Read Write GPR On Destructive Reset Register 6 (DCMRWD6) */

#define DCM_GPR_DCMRWD6_EDMA_DBG_DIS_CM7_0 (1 << 0) /* Bit 0: EDMA debug disable bit for CM7_0 (EDMA_DBG_DIS_CM7_0) */
#define DCM_GPR_DCMRWD6_FCCU_DBG_DIS_CM7_0 (1 << 1) /* Bit 1: FCCU debug disable bit for CM7_0 (FCCU_DBG_DIS_CM7_0) */
#define DCM_GPR_DCMRWD6_LCU0_DBG_DIS_CM7_0 (1 << 2) /* Bit 2: LCU0 debug disable bit for CM7_0 (LCU0_DBG_DIS_CM7_0) */
#define DCM_GPR_DCMRWD6_LCU1_DBG_DIS_CM7_0 (1 << 3) /* Bit 3: LCU1 debug disable bit for CM7_0 (LCU1_DBG_DIS_CM7_0) */

#define DCM_GPR_DCMRWD6_EMIOS0_DBG_DIS_CM7_0 (1 << 4) /* Bit 4: EMIOS0 debug disable bit for CM7_0 (EMIOS0_DBG_DIS_CM7_0) */
#define DCM_GPR_DCMRWD6_EMIOS1_DBG_DIS_CM7_0 (1 << 5) /* Bit 5: EMIOS1 debug disable bit for CM7_0 (EMIOS1_DBG_DIS_CM7_0) */
#define DCM_GPR_DCMRWD6_EMIOS2_DBG_DIS_CM7_0 (1 << 6) /* Bit 6: EMIOS2 debug disable bit for CM7_0 (EMIOS2_DBG_DIS_CM7_0) */

#define DCM_GPR_DCMRWD6_RTC_DBG_DIS_CM7_0 (1 << 7)  /* Bit 7: RTC debug disable bit for CM7_0 (RTC_DBG_DIS_CM7_0) */

#define DCM_GPR_DCMRWD6_SWT0_DBG_DIS_CM7_0 (1 << 8) /* Bit 8: SWT0 debug disable bit for CM7_0 (SWT0_DBG_DIS_CM7_0) */
#define DCM_GPR_DCMRWD6_SWT1_DBG_DIS_CM7_0 (1 << 9) /* Bit 9: SWT1 debug disable bit for CM7_0 (SWT1_DBG_DIS_CM7_0) */

#define DCM_GPR_DCMRWD6_STM0_DBG_DIS_CM7_0     (1 << 10) /* Bit 10: STM0 debug disable bit for CM7_0 (STM0_DBG_DIS_CM7_0) */
#define DCM_GPR_DCMRWD6_STM1_DBG_DIS_CM7_0     (1 << 11) /* Bit 11: STM1 debug disable bit for CM7_0 (STM1_DBG_DIS_CM7_0) */
#define DCM_GPR_DCMRWD6_PIT0_DBG_DIS_CM7_0     (1 << 12) /* Bit 12: PIT0 debug disable bit for CM7_0 (PIT0_DBG_DIS_CM7_0) */
#define DCM_GPR_DCMRWD6_PIT1_DBG_DIS_CM7_0     (1 << 13) /* Bit 13: PIT1 debug disable bit for CM7_0 (PIT1_DBG_DIS_CM7_0) */
#define DCM_GPR_DCMRWD6_PIT2_DBG_DIS_CM7_0     (1 << 14) /* Bit 14: PIT2 debug disable bit for CM7_0 (PIT2_DBG_DIS_CM7_0) */
#define DCM_GPR_DCMRWD6_LPSPI0_DBG_DIS_CM7_0   (1 << 15) /* Bit 15: LPSPI0 debug disable bit for CM7_0 (LPSPI0_DBG_DIS_CM7_0) */
#define DCM_GPR_DCMRWD6_LPSPI1_DBG_DIS_CM7_0   (1 << 16) /* Bit 16: LPSPI1 debug disable bit for CM7_0 (LPSPI1_DBG_DIS_CM7_0) */
#define DCM_GPR_DCMRWD6_LPSPI2_DBG_DIS_CM7_0   (1 << 17) /* Bit 17: LPSPI2 debug disable bit for CM7_0 (LPSPI2_DBG_DIS_CM7_0) */
#define DCM_GPR_DCMRWD6_LPSPI3_DBG_DIS_CM7_0   (1 << 18) /* Bit 18: LPSPI3 debug disable bit for CM7_0 (LPSPI3_DBG_DIS_CM7_0) */
#define DCM_GPR_DCMRWD6_LPSPI4_DBG_DIS_CM7_0   (1 << 19) /* Bit 19: LPSPI4 debug disable bit for CM7_0 (LPSPI4_DBG_DIS_CM7_0) */
#define DCM_GPR_DCMRWD6_LPSPI5_DBG_DIS_CM7_0   (1 << 20) /* Bit 20: LPSPI5 debug disable bit for CM7_0 (LPSPI5_DBG_DIS_CM7_0) */
#define DCM_GPR_DCMRWD6_LPI2C0_DBG_DIS_CM7_0   (1 << 21) /* Bit 21: LPI2C0 debug disable bit for CM7_0 (LPI2C0_DBG_DIS_CM7_0) */
#define DCM_GPR_DCMRWD6_LPI2C1_DBG_DIS_CM7_0   (1 << 22) /* Bit 22: LPI2C1 debug disable bit for CM7_0 (LPI2C1_DBG_DIS_CM7_0) */
#define DCM_GPR_DCMRWD6_FLEXIO_DBG_DIS_CM7_0   (1 << 23) /* Bit 23: FLEXIO debug disable bit for CM7_0 (FLEXIO_DBG_DIS_CM7_0) */
#define DCM_GPR_DCMRWD6_FLEXCAN0_DBG_DIS_CM7_0 (1 << 24) /* Bit 24: FLEXCAN0 debug disable bit for CM7_0 (FLEXCAN0_DBG_DIS_CM7_0) */
#define DCM_GPR_DCMRWD6_FLEXCAN1_DBG_DIS_CM7_0 (1 << 25) /* Bit 25: FLEXCAN1 debug disable bit for CM7_0 (FLEXCAN1_DBG_DIS_CM7_0) */
#define DCM_GPR_DCMRWD6_FLEXCAN2_DBG_DIS_CM7_0 (1 << 26) /* Bit 26: FLEXCAN2 debug disable bit for CM7_0 (FLEXCAN2_DBG_DIS_CM7_0) */
#define DCM_GPR_DCMRWD6_FLEXCAN3_DBG_DIS_CM7_0 (1 << 27) /* Bit 27: FLEXCAN3 debug disable bit for CM7_0 (FLEXCAN3_DBG_DIS_CM7_0) */
#define DCM_GPR_DCMRWD6_FLEXCAN4_DBG_DIS_CM7_0 (1 << 28) /* Bit 28: FLEXCAN4 debug disable bit for CM7_0 (FLEXCAN4_DBG_DIS_CM7_0) */
#define DCM_GPR_DCMRWD6_FLEXCAN5_DBG_DIS_CM7_0 (1 << 29) /* Bit 29: FLEXCAN5 debug disable bit for CM7_0 (FLEXCAN5_DBG_DIS_CM7_0) */
#define DCM_GPR_DCMRWD6_SAI0_DBG_DIS_CM7_0     (1 << 30) /* Bit 30: SAI0 debug disable bit for CM7_0 (SAI0_DBG_DIS_CM7_0) */
#define DCM_GPR_DCMRWD6_SAI1_DBG_DIS_CM7_0     (1 << 31) /* Bit 31: SAI1 debug disable bit for CM7_0 (SAI1_DBG_DIS_CM7_0) */

/* Read Write GPR On Destructive Reset Register 7 (DCMRWD7) */

#define DCM_GPR_DCMRWD7_I3C_DBG_DIS_CM7_0 (1 << 0)  /* Bit 0: I3C debug disable bit for CM7_0 (I3C_DBG_DIS_CM7_0) */
                                                    /* Bits 1-31: Reserved */

/* Read Write GPR On Destructive Reset Register 8 (DCMRWD8) */

#define DCM_GPR_DCMRWD8_EDMA_DBG_DIS_CM7_1 (1 << 0) /* Bit 0: EDMA debug disable bit for CM7_1 (EDMA_DBG_DIS_CM7_1) */
#define DCM_GPR_DCMRWD8_FCCU_DBG_DIS_CM7_1 (1 << 1) /* Bit 1: FCCU debug disable bit for CM7_1 (FCCU_DBG_DIS_CM7_1) */
#define DCM_GPR_DCMRWD8_LCU0_DBG_DIS_CM7_1 (1 << 2) /* Bit 2: LCU0 debug disable bit for CM7_1 (LCU0_DBG_DIS_CM7_1) */
#define DCM_GPR_DCMRWD8_LCU1_DBG_DIS_CM7_1 (1 << 3) /* Bit 3: LCU1 debug disable bit for CM7_1 (LCU1_DBG_DIS_CM7_1) */

#define DCM_GPR_DCMRWD8_EMIOS0_DBG_DIS_CM7_1 (1 << 4) /* Bit 4: EMIOS0 debug disable bit for CM7_1 (EMIOS0_DBG_DIS_CM7_1) */
#define DCM_GPR_DCMRWD8_EMIOS1_DBG_DIS_CM7_1 (1 << 5) /* Bit 5: EMIOS1 debug disable bit for CM7_1 (EMIOS1_DBG_DIS_CM7_1) */
#define DCM_GPR_DCMRWD8_EMIOS2_DBG_DIS_CM7_1 (1 << 6) /* Bit 6: EMIOS2 debug disable bit for CM7_1 (EMIOS2_DBG_DIS_CM7_1) */

#define DCM_GPR_DCMRWD8_RTC_DBG_DIS_CM7_1 (1 << 7)  /* Bit 7: RTC debug disable bit for CM7_1 (RTC_DBG_DIS_CM7_1) */

#define DCM_GPR_DCMRWD8_SWT0_DBG_DIS_CM7_1 (1 << 8) /* Bit 8: SWT0 debug disable bit for CM7_1 (SWT0_DBG_DIS_CM7_1) */
#define DCM_GPR_DCMRWD8_SWT1_DBG_DIS_CM7_1 (1 << 9) /* Bit 9: SWT1 debug disable bit for CM7_1 (SWT1_DBG_DIS_CM7_1) */

#define DCM_GPR_DCMRWD8_STM0_DBG_DIS_CM7_1     (1 << 10) /* Bit 10: STM0 debug disable bit for CM7_1 (STM0_DBG_DIS_CM7_1) */
#define DCM_GPR_DCMRWD8_STM1_DBG_DIS_CM7_1     (1 << 11) /* Bit 11: STM1 debug disable bit for CM7_1 (STM1_DBG_DIS_CM7_1) */
#define DCM_GPR_DCMRWD8_PIT0_DBG_DIS_CM7_1     (1 << 12) /* Bit 12: PIT0 debug disable bit for CM7_1 (PIT0_DBG_DIS_CM7_1) */
#define DCM_GPR_DCMRWD8_PIT1_DBG_DIS_CM7_1     (1 << 13) /* Bit 13: PIT1 debug disable bit for CM7_1 (PIT1_DBG_DIS_CM7_1) */
#define DCM_GPR_DCMRWD8_PIT2_DBG_DIS_CM7_1     (1 << 14) /* Bit 14: PIT2 debug disable bit for CM7_1 (PIT2_DBG_DIS_CM7_1) */
#define DCM_GPR_DCMRWD8_LPSPI0_DBG_DIS_CM7_1   (1 << 15) /* Bit 15: LPSPI0 debug disable bit for CM7_1 (LPSPI0_DBG_DIS_CM7_1) */
#define DCM_GPR_DCMRWD8_LPSPI1_DBG_DIS_CM7_1   (1 << 16) /* Bit 16: LPSPI1 debug disable bit for CM7_1 (LPSPI1_DBG_DIS_CM7_1) */
#define DCM_GPR_DCMRWD8_LPSPI2_DBG_DIS_CM7_1   (1 << 17) /* Bit 17: LPSPI2 debug disable bit for CM7_1 (LPSPI2_DBG_DIS_CM7_1) */
#define DCM_GPR_DCMRWD8_LPSPI3_DBG_DIS_CM7_1   (1 << 18) /* Bit 18: LPSPI3 debug disable bit for CM7_1 (LPSPI3_DBG_DIS_CM7_1) */
#define DCM_GPR_DCMRWD8_LPSPI4_DBG_DIS_CM7_1   (1 << 19) /* Bit 19: LPSPI4 debug disable bit for CM7_1 (LPSPI4_DBG_DIS_CM7_1) */
#define DCM_GPR_DCMRWD8_LPSPI5_DBG_DIS_CM7_1   (1 << 20) /* Bit 20: LPSPI5 debug disable bit for CM7_1 (LPSPI5_DBG_DIS_CM7_1) */
#define DCM_GPR_DCMRWD8_LPI2C0_DBG_DIS_CM7_1   (1 << 21) /* Bit 21: LPI2C0 debug disable bit for CM7_1 (LPI2C0_DBG_DIS_CM7_1) */
#define DCM_GPR_DCMRWD8_LPI2C1_DBG_DIS_CM7_1   (1 << 22) /* Bit 22: LPI2C1 debug disable bit for CM7_1 (LPI2C1_DBG_DIS_CM7_1) */
#define DCM_GPR_DCMRWD8_FLEXIO_DBG_DIS_CM7_1   (1 << 23) /* Bit 23: FLEXIO debug disable bit for CM7_1 (FLEXIO_DBG_DIS_CM7_1) */
#define DCM_GPR_DCMRWD8_FLEXCAN0_DBG_DIS_CM7_1 (1 << 24) /* Bit 24: FLEXCAN0 debug disable bit for CM7_1 (FLEXCAN0_DBG_DIS_CM7_1) */
#define DCM_GPR_DCMRWD8_FLEXCAN1_DBG_DIS_CM7_1 (1 << 25) /* Bit 25: FLEXCAN1 debug disable bit for CM7_1 (FLEXCAN1_DBG_DIS_CM7_1) */
#define DCM_GPR_DCMRWD8_FLEXCAN2_DBG_DIS_CM7_1 (1 << 26) /* Bit 26: FLEXCAN2 debug disable bit for CM7_1 (FLEXCAN2_DBG_DIS_CM7_1) */
#define DCM_GPR_DCMRWD8_FLEXCAN3_DBG_DIS_CM7_1 (1 << 27) /* Bit 27: FLEXCAN3 debug disable bit for CM7_1 (FLEXCAN3_DBG_DIS_CM7_1) */
#define DCM_GPR_DCMRWD8_FLEXCAN4_DBG_DIS_CM7_1 (1 << 28) /* Bit 28: FLEXCAN4 debug disable bit for CM7_1 (FLEXCAN4_DBG_DIS_CM7_1) */
#define DCM_GPR_DCMRWD8_FLEXCAN5_DBG_DIS_CM7_1 (1 << 29) /* Bit 29: FLEXCAN5 debug disable bit for CM7_1 (FLEXCAN5_DBG_DIS_CM7_1) */
#define DCM_GPR_DCMRWD8_SAI0_DBG_DIS_CM7_1     (1 << 30) /* Bit 30: SAI0 debug disable bit for CM7_1 (SAI0_DBG_DIS_CM7_1) */
#define DCM_GPR_DCMRWD8_SAI1_DBG_DIS_CM7_1     (1 << 31) /* Bit 31: SAI1 debug disable bit for CM7_1 (SAI1_DBG_DIS_CM7_1) */

/* Read Write GPR On Destructive Reset Register 9 (DCMRWD9) */

#define DCM_GPR_DCMRWD9_I3C_DBG_DIS_CM7_1 (1 << 0)  /* Bit 0: I3C debug disable bit for CM7_1 (I3C_DBG_DIS_CM7_1) */
                                                    /* Bits 1-31: Reserved */

/* Read Write GPR On Functional Reset Register 1 (DCMRWF1) */

#define DCM_GPR_DCMRWF1_CAN_TIMESTAMP_SEL        (1 << 0) /* Bit 0: Select between EMAC and STM for CAN timestamping (CAN_TIMESTAMP_SEL) */
#  define DCM_GPR_DCMRWF1_CAN_TIMESTAMP_SEL_EMAC (1 << 0) /*        EMAC selected for CAN timestamping */
#  define DCM_GPR_DCMRWF1_CAN_TIMESTAMP_SEL_STM0 (1 << 1) /*        STM0 selected for CAN timestamping */

#define DCM_GPR_DCMRWF1_CAN_TIMESTAMP_EN  (1 << 1)  /* Bit 1: Enables CAN timestamping feature for all FlexCANs (CAN_TIMESTAMP_EN) */
#define DCM_GPR_DCMRWF1_FCCU_SW_NCF0      (1 << 2)  /* Bit 2: Control to initiate Software NFC to FCCU (FCCU_SW_NFC0) */
#define DCM_GPR_DCMRWF1_FCCU_SW_NCF1      (1 << 3)  /* Bit 3: Control to initiate Software NFC to FCCU (FCCU_SW_NFC1) */
#define DCM_GPR_DCMRWF1_FCCU_SW_NCF2      (1 << 4)  /* Bit 4: Control to initiate Software NFC to FCCU (FCCU_SW_NFC2) */
#define DCM_GPR_DCMRWF1_FCCU_SW_NCF3      (1 << 5)  /* Bit 5: Control to initiate Software NFC to FCCU (FCCU_SW_NFC3) */
                                                    /* Bit 6: Reserved */

#define DCM_GPR_DCMRWF1_RMII_MII_SEL        (1 << 7) /* Bit 7: Selects between MII and RMII mode of ethernet (RMII_MII_SEL) */
#  define DCM_GPR_DCMRWF1_RMII_MII_SEL_MII  (0 << 7) /*        MII mode */
#  define DCM_GPR_DCMRWF1_RMII_MII_SEL_RMII (1 << 7) /*        RMII mode */

                                                    /* Bits 8-14: Reserved */

#define DCM_GPR_DCMRWF1_VDD_HV_B_IO_CTRL_LATCH (1 << 15) /* Bit 15: Controls the IO controls latching in low frequency RUN mode to reduce power consumption on VDD_HV_B domain pins (VDD_HV_B_IO_CTRL_LATCH) */

#define DCM_GPR_DCMRWF1_STANDBY_IO_CONFIG (1 << 16) /* Bit 16: Controls the IO state in standby mode (STANDBY_IO_CONFIG) */
                                                    /* Bits 17-19: Reserved */
#define DCM_GPR_DCMRWF1_SUPPLY_MON_EN     (1 << 20) /* Bit 20: Enable the supply voltage monitoring by ADC (SUPPLY_MON_SEL) */

#define DCM_GPR_DCMRWF1_SUPPLY_MON_SEL_SHIFT  (21)  /* Bits 21-23: Selects the source of voltage used by ADC for supply monitoring (SUPPLY_MON_SEL) */
#define DCM_GPR_DCMRWF1_SUPPLY_MON_SEL_MASK   (0x07 << DCM_GPR_DCMRWF1_SUPPLY_MON_SEL_SHIFT)
#  define DCM_GPR_DCMRWF1_SUPPLY_MON_SEL_HV_A (0x00 << DCM_GPR_DCMRWF1_SUPPLY_MON_SEL_SHIFT) /* VDD_HV_A_DIV */
#  define DCM_GPR_DCMRWF1_SUPPLY_MON_SEL_HV_B (0x01 << DCM_GPR_DCMRWF1_SUPPLY_MON_SEL_SHIFT) /* VDD_HV_B_DIV */
#  define DCM_GPR_DCMRWF1_SUPPLY_MON_SEL_1_5  (0x02 << DCM_GPR_DCMRWF1_SUPPLY_MON_SEL_SHIFT) /* VDD_1.5_DIV */
#  define DCM_GPR_DCMRWF1_SUPPLY_MON_SEL_2_5  (0x03 << DCM_GPR_DCMRWF1_SUPPLY_MON_SEL_SHIFT) /* VDD_2.5_OSC */
#  define DCM_GPR_DCMRWF1_SUPPLY_MON_SEL_HOT  (0x04 << DCM_GPR_DCMRWF1_SUPPLY_MON_SEL_SHIFT) /* VDD1.1_PD1_HOT_POINT */
#  define DCM_GPR_DCMRWF1_SUPPLY_MON_SEL_COLD (0x05 << DCM_GPR_DCMRWF1_SUPPLY_MON_SEL_SHIFT) /* VDD1.1_PD1_COLD_POINT */
#  define DCM_GPR_DCMRWF1_SUPPLY_MON_SEL_PLL  (0x06 << DCM_GPR_DCMRWF1_SUPPLY_MON_SEL_SHIFT) /* VDD1.1_PLL */
#  define DCM_GPR_DCMRWF1_SUPPLY_MON_SEL_PD0  (0x07 << DCM_GPR_DCMRWF1_SUPPLY_MON_SEL_SHIFT) /* VDD1.1_PD0 */

#define DCM_GPR_DCMRWF1_VSS_LV_ANMUX_EN   (1 << 24) /* Bit 24: Enable VSS_LV monitoring (VSS_LV_ANMUX_EN) */

#define DCM_GPR_DCMRWF1_VDD_HV_A_VLT_DVDR_EN (1 << 25) /* Bit 25: Enable 2:1 divider for VDD_HV_A for supply voltage monitoring by ADC (VDD_HV_A_VLT_DVDR_EN) */
#define DCM_GPR_DCMRWF1_VDD_HV_B_VLT_DVDR_EN (1 << 26) /* Bit 26: Enable 2:1 divider for VDD_HV_B for supply voltage monitoring by ADC (VDD_HV_B_VLT_DVDR_EN) */
#define DCM_GPR_DCMRWF1_VDD_1_5_VLT_DVDR_EN  (1 << 27) /* Bit 27: Enable 2:1 divider for VDD1P5 for supply voltage monitoring by ADC (VDD_1_5_VLT_DVDR_EN) */

                                                    /* Bits 28-31: Reserved */

/* Read Write GPR On Functional Reset Register 2 (DCMRWF2) */

                                                    /* Bits 0-2: Reserved */

#define DCM_GPR_DCMRWF2_DCM_SCAN_BYP_STDBY_EXT         (1 << 3) /* Bit 3: Bypass the DCM scanning on standby exit (DCM_SCAN_BYP_STDBY_EXT) */
#define DCM_GPR_DCMRWF2_FIRC_TRIM_BYP_STDBY_EXT        (1 << 4) /* Bit 4: Bypass the FIRC trimming on standby exit (FIRC_TRIM_BYP_STDBY_EXT) */
#define DCM_GPR_DCMRWF2_PMC_TRIM_RGM_DCF_BYP_STDBY_EXT (1 << 5) /* Bit 5: Bypass the PMC trimming and RGM DCF loading on standby exit (PMC_TRIM_RGM_DCF_BYP_STDBY_EXT) */
#define DCM_GPR_DCMRWF2_SIRC_TRIM_BYP_STDBY_EXT        (1 << 6) /* Bit 6: Bypass the SIRC trimming on standby exit (SIRC_TRIM_BYP_STDBY_EXT) */

                                                    /* Bits 7-15: Reserved */
#define DCM_GPR_DCMRWF2_HSE_GSKT_BYPASS   (1 << 16) /* Enable the HSE IAHB gasket bypass out of standby mode (HSE_GSKT_BYPASS) */
                                                    /* Bits 17-31: Reserved */

/* Read Write GPR On Functional Reset Register 4 (DCMRWF4) */

                                                    /* Bit 0: Reserved */

#define DCM_GPR_DCMRWF4_MUX_MODE_EN_ADC0_S8  (1 << 1) /* Bit 1: Selects GPIO45 to drive ADC0_S8 (MUX_MODE_EN_ADC0_S8) */
#define DCM_GPR_DCMRWF4_MUX_MODE_EN_ADC0_S9  (1 << 2) /* Bit 2: Selects GPIO46 to drive ADC0_S9 (MUX_MODE_EN_ADC0_S9) */
#define DCM_GPR_DCMRWF4_MUX_MODE_EN_ADC1_S14 (1 << 3) /* Bit 3: Selects GPIO32 to drive ADC1_S14 (MUX_MODE_EN_ADC1_S14) */
#define DCM_GPR_DCMRWF4_MUX_MODE_EN_ADC1_S15 (1 << 4) /* Bit 4: Selects GPIO33 to drive ADC1_S15 (MUX_MODE_EN_ADC1_S15) */
#define DCM_GPR_DCMRWF4_MUX_MODE_EN_ADC1_S22 (1 << 5) /* Bit 5: Selects GPIO114 to drive ADC1_S22 (MUX_MODE_EN_ADC1_S22) */
#define DCM_GPR_DCMRWF4_MUX_MODE_EN_ADC1_S23 (1 << 6) /* Bit 6: Selects GPIO115 to drive ADC1_S23 (MUX_MODE_EN_ADC1_S23) */

                                                    /* Bits 7-8: Reserved */

#define DCM_GPR_DCMRWF4_MUX_MODE_EN_ADC2_S8  (1 << 9)  /* Bit 9: Selects GPIO45 to drive ADC2_S8 (MUX_MODE_EN_ADC2_S8) */
#define DCM_GPR_DCMRWF4_MUX_MODE_EN_ADC2_S9  (1 << 10) /* Bit 10: Selects GPIO46 to drive ADC2_S9 (MUX_MODE_EN_ADC2_S9) */

                                                    /* Bits 11-12: Reserved */

#define DCM_GPR_DCMRWF4_GLITCH_FIL_TRG_IN0_BYP (1 << 13) /* Bit 13: Bypass glitch filter on TRGMUX input63 (GLITCH_FIL_TRG_IN0_BYP) */
#define DCM_GPR_DCMRWF4_GLITCH_FIL_TRG_IN1_BYP (1 << 14) /* Bit 14: Bypass glitch filter on TRGMUX input62 (GLITCH_FIL_TRG_IN1_BYP) */
#define DCM_GPR_DCMRWF4_GLITCH_FIL_TRG_IN2_BYP (1 << 15) /* Bit 15: Bypass glitch filter on TRGMUX input61 (GLITCH_FIL_TRG_IN2_BYP) */
#define DCM_GPR_DCMRWF4_GLITCH_FIL_TRG_IN3_BYP (1 << 16) /* Bit 16: Bypass glitch filter on TRGMUX input60 (GLITCH_FIL_TRG_IN3_BYP) */

#define DCM_GPR_DCMRWF4_CM7_0_CPUWAIT     (1 << 17) /* Bit 17: Put CM7_0 core into wait mode (CM7_0_CPUWAIT) */
#define DCM_GPR_DCMRWF4_CM7_1_CPUWAIT     (1 << 18) /* Bit 18: Put CM7_1 core into wait mode (CM7_1_CPUWAIT) */
                                                    /* Bits 19-31: Reserved */

/* Read Write GPR On Functional Reset Register 5 (DCMRWF5) */

#define DCM_GPR_DCMRWF5_BOOT_MODE          (1 << 0) /* Bit 0: Selects the boot mode after exiting standby mode (BOOT_MODE) */
#  define DCM_GPR_DCMRWF5_BOOT_MODE_NORMAL (0 << 0) /*        Normal */
#  define DCM_GPR_DCMRWF5_BOOT_MODE_FAST   (1 << 0) /*        Fast Standby */

#define DCM_GPR_DCMRWF5_BOOT_ADDRESS_SHIFT (1)      /* Bits 1-31: Cortex-M7_0 base address of vector table to be used after exiting (fast) standby mode (BOOT_ADDRESS) */
#define DCM_GPR_DCMRWF5_BOOT_ADDRESS_MASK  (0x7fffffff << DCM_GPR_DCMRWF5_BOOT_ADDRESS_SHIFT) 

/* Read Only GPR On PMCPOR Reset Register 1 (DCMROPP1) */

#define DCM_GPR_DCMROPP1_POR_WDG_STAT0    (1 << 0)  /* Bit 0: Status of functional reset sequence process FUNC0 when POR_WDG overflows (POR_WDG_STAT0) */
#define DCM_GPR_DCMROPP1_POR_WDG_STAT1    (1 << 1)  /* Bit 1: Status of functional reset sequence process FUNC1 when POR_WDG overflows (POR_WDG_STAT1) */
#define DCM_GPR_DCMROPP1_POR_WDG_STAT2    (1 << 2)  /* Bit 2: Status of functional reset sequence process FUNC2 when POR_WDG overflows (POR_WDG_STAT2) */
#define DCM_GPR_DCMROPP1_POR_WDG_STAT3    (1 << 3)  /* Bit 3: Status of functional reset sequence process FUNC3 when POR_WDG overflows (POR_WDG_STAT3) */
#define DCM_GPR_DCMROPP1_POR_WDG_STAT4    (1 << 4)  /* Bit 4: Status of functional reset sequence process FUNC4 when POR_WDG overflows (POR_WDG_STAT4) */
#define DCM_GPR_DCMROPP1_POR_WDG_STAT5    (1 << 5)  /* Bit 5: Status of functional reset sequence process FUNC5 when POR_WDG overflows (POR_WDG_STAT5) */
#define DCM_GPR_DCMROPP1_POR_WDG_STAT6    (1 << 6)  /* Bit 6: Status of functional reset sequence process FUNC6 when POR_WDG overflows (POR_WDG_STAT6) */
                                                    /* Bits 7-9: Reserved */
#define DCM_GPR_DCMROPP1_POR_WDG_STAT10   (1 << 10) /* Bit 10: Status of functional reset sequence process FUNC7 when POR_WDG overflows (POR_WDG_STAT10) */
#define DCM_GPR_DCMROPP1_POR_WDG_STAT11   (1 << 11) /* Bit 11: Status of functional reset sequence process FUNC8 when POR_WDG overflows (POR_WDG_STAT11) */
                                                    /* Bits 12-13: Reserved */
#define DCM_GPR_DCMROPP1_POR_WDG_STAT14   (1 << 14) /* Bit 14: Status of functional reset sequence process FUNC9 when POR_WDG overflows (POR_WDG_STAT14) */
                                                    /* Bits 15-16: Reserved */
#define DCM_GPR_DCMROPP1_POR_WDG_STAT17   (1 << 17) /* Bit 17: Status of functional reset sequence process FUNC10 when POR_WDG overflows (POR_WDG_STAT17) */
                                                    /* Bits 18-19: Reserved */
#define DCM_GPR_DCMROPP1_POR_WDG_STAT20   (1 << 20) /* Bit 20: Status of functional reset sequence process DEST0 when POR_WDG overflows (POR_WDG_STAT20) */
                                                    /* Bits 21-28: Reserved */
#define DCM_GPR_DCMROPP1_POR_WDG_STAT29   (1 << 29) /* Bit 29: Status of standby entry request initiated by MC_ME when POR_WDG overflows (POR_WDG_STAT29) */
#define DCM_GPR_DCMROPP1_POR_WDG_STAT30   (1 << 30) /* Bit 30: Status of standby exit acknowledgement by MC_PCU when POR_WDG overflows (POR_WDG_STAT30) */
#define DCM_GPR_DCMROPP1_POR_WDG_STAT31   (1 << 31) /* Bit 31: MC_RGM reset event (if occurred) while the device is in STANDBY mode (POR_WDG_STAT31) */

/* Read Only GPR On PMCPOR Reset Register 2 (DCMROPP2) */

#define DCM_GPR_DCMROPP2_POR_WDG_STAT32   (1 << 0)  /* Bit 0: MC_RGM functional/external event status register when POR_WDG overflows (POR_WDG_STAT32) */
#define DCM_GPR_DCMROPP2_POR_WDG_STAT33   (1 << 1)  /* Bit 1: MC_RGM functional/external event status register when POR_WDG overflows (POR_WDG_STAT33) */
#define DCM_GPR_DCMROPP2_POR_WDG_STAT34   (1 << 2)  /* Bit 2: MC_RGM functional/external event status register when POR_WDG overflows (POR_WDG_STAT34) */
#define DCM_GPR_DCMROPP2_POR_WDG_STAT35   (1 << 3)  /* Bit 3: MC_RGM functional/external event status register when POR_WDG overflows (POR_WDG_STAT35) */
#define DCM_GPR_DCMROPP2_POR_WDG_STAT36   (1 << 4)  /* Bit 4: MC_RGM functional/external event status register when POR_WDG overflows (POR_WDG_STAT36) */
#define DCM_GPR_DCMROPP2_POR_WDG_STAT37   (1 << 5)  /* Bit 5: MC_RGM functional/external event status register when POR_WDG overflows (POR_WDG_STAT37) */
#define DCM_GPR_DCMROPP2_POR_WDG_STAT38   (1 << 6)  /* Bit 6: MC_RGM functional/external event status register when POR_WDG overflows (POR_WDG_STAT38) */
#define DCM_GPR_DCMROPP2_POR_WDG_STAT39   (1 << 7)  /* Bit 7: MC_RGM functional/external event status register when POR_WDG overflows (POR_WDG_STAT39) */
#define DCM_GPR_DCMROPP2_POR_WDG_STAT40   (1 << 8)  /* Bit 8: MC_RGM functional/external event status register when POR_WDG overflows (POR_WDG_STAT40) */
#define DCM_GPR_DCMROPP2_POR_WDG_STAT41   (1 << 9)  /* Bit 9: MC_RGM functional/external event status register when POR_WDG overflows (POR_WDG_STAT41) */
#define DCM_GPR_DCMROPP2_POR_WDG_STAT42   (1 << 10) /* Bit 10: MC_RGM functional/external event status register when POR_WDG overflows (POR_WDG_STAT42) */
#define DCM_GPR_DCMROPP2_POR_WDG_STAT43   (1 << 11) /* Bit 11: MC_RGM functional/external event status register when POR_WDG overflows (POR_WDG_STAT43) */
#define DCM_GPR_DCMROPP2_POR_WDG_STAT44   (1 << 12) /* Bit 12: MC_RGM functional/external event status register when POR_WDG overflows (POR_WDG_STAT44) */
#define DCM_GPR_DCMROPP2_POR_WDG_STAT45   (1 << 13) /* Bit 13: MC_RGM functional/external event status register when POR_WDG overflows (POR_WDG_STAT45) */
#define DCM_GPR_DCMROPP2_POR_WDG_STAT46   (1 << 14) /* Bit 14: MC_RGM functional/external event status register when POR_WDG overflows (POR_WDG_STAT46) */
#define DCM_GPR_DCMROPP2_POR_WDG_STAT47   (1 << 15) /* Bit 15: MC_RGM functional/external event status register when POR_WDG overflows (POR_WDG_STAT47) */
#define DCM_GPR_DCMROPP2_POR_WDG_STAT48   (1 << 16) /* Bit 16: MC_RGM functional/external event status register when POR_WDG overflows (POR_WDG_STAT48) */
#define DCM_GPR_DCMROPP2_POR_WDG_STAT49   (1 << 17) /* Bit 17: MC_RGM functional/external event status register when POR_WDG overflows (POR_WDG_STAT49) */
#define DCM_GPR_DCMROPP2_POR_WDG_STAT50   (1 << 18) /* Bit 18: MC_RGM functional/external event status register when POR_WDG overflows (POR_WDG_STAT50) */
#define DCM_GPR_DCMROPP2_POR_WDG_STAT51   (1 << 19) /* Bit 19: MC_RGM functional/external event status register when POR_WDG overflows (POR_WDG_STAT51) */
#define DCM_GPR_DCMROPP2_POR_WDG_STAT52   (1 << 20) /* Bit 20: MC_RGM functional/external event status register when POR_WDG overflows (POR_WDG_STAT52) */
#define DCM_GPR_DCMROPP2_POR_WDG_STAT53   (1 << 21) /* Bit 21: MC_RGM functional/external event status register when POR_WDG overflows (POR_WDG_STAT53) */
#define DCM_GPR_DCMROPP2_POR_WDG_STAT54   (1 << 22) /* Bit 22: MC_RGM functional/external event status register when POR_WDG overflows (POR_WDG_STAT54) */
#define DCM_GPR_DCMROPP2_POR_WDG_STAT55   (1 << 23) /* Bit 23: MC_RGM functional/external event status register when POR_WDG overflows (POR_WDG_STAT55) */
#define DCM_GPR_DCMROPP2_POR_WDG_STAT56   (1 << 24) /* Bit 24: MC_RGM functional/external event status register when POR_WDG overflows (POR_WDG_STAT56) */
#define DCM_GPR_DCMROPP2_POR_WDG_STAT57   (1 << 25) /* Bit 25: MC_RGM functional/external event status register when POR_WDG overflows (POR_WDG_STAT57) */
#define DCM_GPR_DCMROPP2_POR_WDG_STAT58   (1 << 26) /* Bit 26: MC_RGM functional/external event status register when POR_WDG overflows (POR_WDG_STAT58) */
#define DCM_GPR_DCMROPP2_POR_WDG_STAT59   (1 << 27) /* Bit 27: MC_RGM functional/external event status register when POR_WDG overflows (POR_WDG_STAT59) */
#define DCM_GPR_DCMROPP2_POR_WDG_STAT60   (1 << 28) /* Bit 28: MC_RGM functional/external event status register when POR_WDG overflows (POR_WDG_STAT60) */
#define DCM_GPR_DCMROPP2_POR_WDG_STAT61   (1 << 29) /* Bit 29: MC_RGM functional/external event status register when POR_WDG overflows (POR_WDG_STAT61) */
#define DCM_GPR_DCMROPP2_POR_WDG_STAT62   (1 << 30) /* Bit 30: MC_RGM functional/external event status register when POR_WDG overflows (POR_WDG_STAT62) */
#define DCM_GPR_DCMROPP2_POR_WDG_STAT63   (1 << 31) /* Bit 31: MC_RGM functional/external event status register when POR_WDG overflows (POR_WDG_STAT63) */

/* Read Only GPR On PMCPOR Reset Register 3 (DCMROPP3) */

#define DCM_GPR_DCMROPP3_POR_WDG_STAT64   (1 << 0)  /* Bit 0: MC_RGM destructive event status register when POR_WDG overflows (POR_WDG_STAT64) */
#define DCM_GPR_DCMROPP3_POR_WDG_STAT65   (1 << 1)  /* Bit 1: MC_RGM destructive event status register when POR_WDG overflows (POR_WDG_STAT65) */
#define DCM_GPR_DCMROPP3_POR_WDG_STAT66   (1 << 2)  /* Bit 2: MC_RGM destructive event status register when POR_WDG overflows (POR_WDG_STAT66) */
#define DCM_GPR_DCMROPP3_POR_WDG_STAT67   (1 << 3)  /* Bit 3: MC_RGM destructive event status register when POR_WDG overflows (POR_WDG_STAT67) */
#define DCM_GPR_DCMROPP3_POR_WDG_STAT68   (1 << 4)  /* Bit 4: MC_RGM destructive event status register when POR_WDG overflows (POR_WDG_STAT68) */
#define DCM_GPR_DCMROPP3_POR_WDG_STAT69   (1 << 5)  /* Bit 5: MC_RGM destructive event status register when POR_WDG overflows (POR_WDG_STAT69) */
#define DCM_GPR_DCMROPP3_POR_WDG_STAT70   (1 << 6)  /* Bit 6: MC_RGM destructive event status register when POR_WDG overflows (POR_WDG_STAT70) */
#define DCM_GPR_DCMROPP3_POR_WDG_STAT71   (1 << 7)  /* Bit 7: MC_RGM destructive event status register when POR_WDG overflows (POR_WDG_STAT71) */
#define DCM_GPR_DCMROPP3_POR_WDG_STAT72   (1 << 8)  /* Bit 8: MC_RGM destructive event status register when POR_WDG overflows (POR_WDG_STAT72) */
#define DCM_GPR_DCMROPP3_POR_WDG_STAT73   (1 << 9)  /* Bit 9: MC_RGM destructive event status register when POR_WDG overflows (POR_WDG_STAT73) */
#define DCM_GPR_DCMROPP3_POR_WDG_STAT74   (1 << 10) /* Bit 10: MC_RGM destructive event status register when POR_WDG overflows (POR_WDG_STAT74) */
#define DCM_GPR_DCMROPP3_POR_WDG_STAT75   (1 << 11) /* Bit 11: MC_RGM destructive event status register when POR_WDG overflows (POR_WDG_STAT75) */
#define DCM_GPR_DCMROPP3_POR_WDG_STAT76   (1 << 12) /* Bit 12: MC_RGM destructive event status register when POR_WDG overflows (POR_WDG_STAT76) */
#define DCM_GPR_DCMROPP3_POR_WDG_STAT77   (1 << 13) /* Bit 13: MC_RGM destructive event status register when POR_WDG overflows (POR_WDG_STAT77) */
#define DCM_GPR_DCMROPP3_POR_WDG_STAT78   (1 << 14) /* Bit 14: MC_RGM destructive event status register when POR_WDG overflows (POR_WDG_STAT78) */
#define DCM_GPR_DCMROPP3_POR_WDG_STAT79   (1 << 15) /* Bit 15: MC_RGM destructive event status register when POR_WDG overflows (POR_WDG_STAT79) */
#define DCM_GPR_DCMROPP3_POR_WDG_STAT80   (1 << 16) /* Bit 16: MC_RGM destructive event status register when POR_WDG overflows (POR_WDG_STAT80) */
#define DCM_GPR_DCMROPP3_POR_WDG_STAT81   (1 << 17) /* Bit 17: MC_RGM destructive event status register when POR_WDG overflows (POR_WDG_STAT81) */
#define DCM_GPR_DCMROPP3_POR_WDG_STAT82   (1 << 18) /* Bit 18: MC_RGM destructive event status register when POR_WDG overflows (POR_WDG_STAT82) */
#define DCM_GPR_DCMROPP3_POR_WDG_STAT83   (1 << 19) /* Bit 19: MC_RGM destructive event status register when POR_WDG overflows (POR_WDG_STAT83) */
#define DCM_GPR_DCMROPP3_POR_WDG_STAT84   (1 << 20) /* Bit 20: MC_RGM destructive event status register when POR_WDG overflows (POR_WDG_STAT84) */
#define DCM_GPR_DCMROPP3_POR_WDG_STAT85   (1 << 21) /* Bit 21: MC_RGM destructive event status register when POR_WDG overflows (POR_WDG_STAT85) */
#define DCM_GPR_DCMROPP3_POR_WDG_STAT86   (1 << 22) /* Bit 22: MC_RGM destructive event status register when POR_WDG overflows (POR_WDG_STAT86) */
#define DCM_GPR_DCMROPP3_POR_WDG_STAT87   (1 << 23) /* Bit 23: MC_RGM destructive event status register when POR_WDG overflows (POR_WDG_STAT87) */
#define DCM_GPR_DCMROPP3_POR_WDG_STAT88   (1 << 24) /* Bit 24: MC_RGM destructive event status register when POR_WDG overflows (POR_WDG_STAT88) */
#define DCM_GPR_DCMROPP3_POR_WDG_STAT89   (1 << 25) /* Bit 25: MC_RGM destructive event status register when POR_WDG overflows (POR_WDG_STAT89) */
#define DCM_GPR_DCMROPP3_POR_WDG_STAT90   (1 << 26) /* Bit 26: MC_RGM destructive event status register when POR_WDG overflows (POR_WDG_STAT90) */
#define DCM_GPR_DCMROPP3_POR_WDG_STAT91   (1 << 27) /* Bit 27: MC_RGM destructive event status register when POR_WDG overflows (POR_WDG_STAT91) */
#define DCM_GPR_DCMROPP3_POR_WDG_STAT92   (1 << 28) /* Bit 28: MC_RGM destructive event status register when POR_WDG overflows (POR_WDG_STAT92) */
#define DCM_GPR_DCMROPP3_POR_WDG_STAT93   (1 << 29) /* Bit 29: MC_RGM destructive event status register when POR_WDG overflows (POR_WDG_STAT93) */
#define DCM_GPR_DCMROPP3_POR_WDG_STAT94   (1 << 30) /* Bit 30: MC_RGM destructive event status register when POR_WDG overflows (POR_WDG_STAT94) */
#define DCM_GPR_DCMROPP3_POR_WDG_STAT95   (1 << 31) /* Bit 31: MC_RGM destructive event status register when POR_WDG overflows (POR_WDG_STAT95) */

/* Read Only GPR On PMCPOR Reset Register 4 (DCMROPP4) */

#define DCM_GPR_DCMROPP4_POR_WDG_STAT96   (1 << 0) /* Bit 0: POR_WDG reset event if POR_WDG initiates a POR sequence (POR_WDG_STAT96) */
                                                   /* Bits 1-31: Reserved */

#endif /* __ARCH_ARM_SRC_S32K3XX_HARDWARE_S32K3XX_DCM_H */
