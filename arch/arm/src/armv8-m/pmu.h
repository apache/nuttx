/****************************************************************************
 * arch/arm/src/armv8-m/pmu.h
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

#ifndef __ARCH_ARM_SRC_ARMV8_M_PMU_H
#define __ARCH_ARM_SRC_ARMV8_M_PMU_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>
#include <stdint.h>

#include "arm_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define PMU_BASE        (0xe0003000ul)                             /* PMU Base Address */

#define PMU_EVCNTR      (PMU_BASE + 0x0000)   /* Offset: 0x0 (R/W)    PMU Event Counter Registers */
#define PMU_CCNTR       (PMU_BASE + 0x007c)   /* Offset: 0x7c (R/W)   PMU Cycle Counter Register */
#define PMU_EVTYPER     (PMU_BASE + 0x0400)   /* Offset: 0x400 (R/W)  PMU Event Type and Filter Registers */
#define PMU_CCFILTR     (PMU_BASE + 0x047c)   /* Offset: 0x47c (R/W)  PMU Cycle Counter Filter Register */
#define PMU_CNTENSET    (PMU_BASE + 0x0c00)   /* Offset: 0xc00 (R/W)  PMU Count Enable Set Register */
#define PMU_CNTENCLR    (PMU_BASE + 0x0c20)   /* Offset: 0xc20 (R/W)  PMU Count Enable Clear Register */
#define PMU_INTENSET    (PMU_BASE + 0x0c40)   /* Offset: 0xc40 (R/W)  PMU Interrupt Enable Set Register */
#define PMU_INTENCLR    (PMU_BASE + 0x0c60)   /* Offset: 0xc60 (R/W)  PMU Interrupt Enable Clear Register */
#define PMU_OVSCLR      (PMU_BASE + 0x0c80)   /* Offset: 0xc80 (R/W)  PMU Overflow Flag Status Clear Register */
#define PMU_SWINC       (PMU_BASE + 0x0ca0)   /* Offset: 0xca0 (R/W)  PMU Software Increment Register */
#define PMU_OVSSET      (PMU_BASE + 0x0cc0)   /* Offset: 0xcc0 (R/W)  PMU Overflow Flag Status Set Register */
#define PMU_TYPE        (PMU_BASE + 0x0e00)   /* Offset: 0xe00 (R/W)  PMU Type Register */
#define PMU_CTRL        (PMU_BASE + 0x0e04)   /* Offset: 0xe04 (R/W)  PMU Control Register */
#define PMU_AUTHSTATUS  (PMU_BASE + 0x0fb8)   /* Offset: 0xfb8 (R/W)  PMU Authentication Status Register */
#define PMU_DEVARCH     (PMU_BASE + 0x0fbc)   /* Offset: 0xfbc (R/W)  PMU Device Architecture Register */
#define PMU_DEVTYPE     (PMU_BASE + 0x0fcc)   /* Offset: 0xfcc (R/W)  PMU Device Type Register */
#define PMU_PIDR4       (PMU_BASE + 0x0fd0)   /* Offset: 0xfd0 (R/W)  PMU Peripheral Identification Register 4 */
#define PMU_PIDR0       (PMU_BASE + 0x0fe0)   /* Offset: 0xfe0 (R/W)  PMU Peripheral Identification Register 0 */
#define PMU_PIDR1       (PMU_BASE + 0x0fe4)   /* Offset: 0xfe4 (R/W)  PMU Peripheral Identification Register 1 */
#define PMU_PIDR2       (PMU_BASE + 0x0fe8)   /* Offset: 0xfe8 (R/W)  PMU Peripheral Identification Register 2 */
#define PMU_PIDR3       (PMU_BASE + 0x0fec)   /* Offset: 0xfec (R/W)  PMU Peripheral Identification Register 3 */
#define PMU_CIDR0       (PMU_BASE + 0x0ff0)   /* Offset: 0xff0 (R/W)  PMU Component Identification Register 0 */
#define PMU_CIDR1       (PMU_BASE + 0x0ff4)   /* Offset: 0xff4 (R/W)  PMU Component Identification Register 1 */
#define PMU_CIDR2       (PMU_BASE + 0x0ff8)   /* Offset: 0xff8 (R/W)  PMU Component Identification Register 2 */
#define PMU_CIDR3       (PMU_BASE + 0x0ffc)   /* Offset: 0xffc (R/W)  PMU Component Identification Register 3 */

#define PMU_EVCNTR_CNT_MASK                   (0xfffful)           /* PMU EVCNTR: Counter Mask */

/* PMU Event Type and Filter Registers (0-30) Definitions  */

#define PMU_EVTYPER_EVENTTOCNT_MASK           (0xfffful)          /* PMU EVTYPER: Event to Count Mask */

/* PMU Count Enable Set Register Definitions */

#define PMU_CNTENSET_CNT0_ENABLE_MASK         (1ul << 0u)         /* PMU CNTENSET: Event Counter 0 Enable Set Mask */
#define PMU_CNTENSET_CNT1_ENABLE_MASK         (1ul << 1u)         /* PMU CNTENSET: Event Counter 1 Enable Set Mask */
#define PMU_CNTENSET_CNT2_ENABLE_MASK         (1ul << 2u)         /* PMU CNTENSET: Event Counter 2 Enable Set Mask */
#define PMU_CNTENSET_CNT3_ENABLE_MASK         (1ul << 3u)         /* PMU CNTENSET: Event Counter 3 Enable Set Mask */
#define PMU_CNTENSET_CNT4_ENABLE_MASK         (1ul << 4u)         /* PMU CNTENSET: Event Counter 4 Enable Set Mask */
#define PMU_CNTENSET_CNT5_ENABLE_MASK         (1ul << 5u)         /* PMU CNTENSET: Event Counter 5 Enable Set Mask */
#define PMU_CNTENSET_CNT6_ENABLE_MASK         (1ul << 6u)         /* PMU CNTENSET: Event Counter 6 Enable Set Mask */
#define PMU_CNTENSET_CNT7_ENABLE_MASK         (1ul << 7u)         /* PMU CNTENSET: Event Counter 7 Enable Set Mask */
#define PMU_CNTENSET_CNT8_ENABLE_MASK         (1ul << 8u)         /* PMU CNTENSET: Event Counter 8 Enable Set Mask */
#define PMU_CNTENSET_CNT9_ENABLE_MASK         (1ul << 9u)         /* PMU CNTENSET: Event Counter 9 Enable Set Mask */
#define PMU_CNTENSET_CNT10_ENABLE_MASK        (1ul << 10u)        /* PMU CNTENSET: Event Counter 10 Enable Set Mask */
#define PMU_CNTENSET_CNT11_ENABLE_MASK        (1ul << 11u)        /* PMU CNTENSET: Event Counter 11 Enable Set Mask */
#define PMU_CNTENSET_CNT12_ENABLE_MASK        (1ul << 12u)        /* PMU CNTENSET: Event Counter 12 Enable Set Mask */
#define PMU_CNTENSET_CNT13_ENABLE_MASK        (1ul << 13u)        /* PMU CNTENSET: Event Counter 13 Enable Set Mask */
#define PMU_CNTENSET_CNT14_ENABLE_MASK        (1ul << 14u)        /* PMU CNTENSET: Event Counter 14 Enable Set Mask */
#define PMU_CNTENSET_CNT15_ENABLE_MASK        (1ul << 15u)        /* PMU CNTENSET: Event Counter 15 Enable Set Mask */
#define PMU_CNTENSET_CNT16_ENABLE_MASK        (1ul << 16u)        /* PMU CNTENSET: Event Counter 16 Enable Set Mask */
#define PMU_CNTENSET_CNT17_ENABLE_MASK        (1ul << 17u)        /* PMU CNTENSET: Event Counter 17 Enable Set Mask */
#define PMU_CNTENSET_CNT18_ENABLE_MASK        (1ul << 18u)        /* PMU CNTENSET: Event Counter 18 Enable Set Mask */
#define PMU_CNTENSET_CNT19_ENABLE_MASK        (1ul << 19u)        /* PMU CNTENSET: Event Counter 19 Enable Set Mask */
#define PMU_CNTENSET_CNT20_ENABLE_MASK        (1ul << 20u)        /* PMU CNTENSET: Event Counter 20 Enable Set Mask */
#define PMU_CNTENSET_CNT21_ENABLE_MASK        (1ul << 21u)        /* PMU CNTENSET: Event Counter 21 Enable Set Mask */
#define PMU_CNTENSET_CNT22_ENABLE_MASK        (1ul << 22u)        /* PMU CNTENSET: Event Counter 22 Enable Set Mask */
#define PMU_CNTENSET_CNT23_ENABLE_MASK        (1ul << 23u)        /* PMU CNTENSET: Event Counter 23 Enable Set Mask */
#define PMU_CNTENSET_CNT24_ENABLE_MASK        (1ul << 24u)        /* PMU CNTENSET: Event Counter 24 Enable Set Mask */
#define PMU_CNTENSET_CNT25_ENABLE_MASK        (1ul << 25u)        /* PMU CNTENSET: Event Counter 25 Enable Set Mask */
#define PMU_CNTENSET_CNT26_ENABLE_MASK        (1ul << 26u)        /* PMU CNTENSET: Event Counter 26 Enable Set Mask */
#define PMU_CNTENSET_CNT27_ENABLE_MASK        (1ul << 27u)        /* PMU CNTENSET: Event Counter 27 Enable Set Mask */
#define PMU_CNTENSET_CNT28_ENABLE_MASK        (1ul << 28u)        /* PMU CNTENSET: Event Counter 28 Enable Set Mask */
#define PMU_CNTENSET_CNT29_ENABLE_MASK        (1ul << 29u)        /* PMU CNTENSET: Event Counter 29 Enable Set Mask */
#define PMU_CNTENSET_CNT30_ENABLE_MASK        (1ul << 30u)        /* PMU CNTENSET: Event Counter 30 Enable Set Mask */
#define PMU_CNTENSET_CCNTR_ENABLE_MASK        (1ul << 31u)        /* PMU CNTENSET: Cycle Counter Enable Set Mask */

/* PMU Count Enable Clear Register Definitions */

#define PMU_CNTENCLR_CNT0_ENABLE_MASK         (1ul << 0u)         /* PMU CNTENCLR: Event Counter 0 Enable Clear Mask */
#define PMU_CNTENCLR_CNT1_ENABLE_MASK         (1ul << 1u)         /* PMU CNTENCLR: Event Counter 1 Enable Clear */
#define PMU_CNTENCLR_CNT2_ENABLE_MASK         (1ul << 2u)         /* PMU CNTENCLR: Event Counter 2 Enable Clear Mask */
#define PMU_CNTENCLR_CNT3_ENABLE_MASK         (1ul << 3u)         /* PMU CNTENCLR: Event Counter 3 Enable Clear Mask */
#define PMU_CNTENCLR_CNT4_ENABLE_MASK         (1ul << 4u)         /* PMU CNTENCLR: Event Counter 4 Enable Clear Mask */
#define PMU_CNTENCLR_CNT5_ENABLE_MASK         (1ul << 5u)         /* PMU CNTENCLR: Event Counter 5 Enable Clear Mask */
#define PMU_CNTENCLR_CNT6_ENABLE_MASK         (1ul << 6u)         /* PMU CNTENCLR: Event Counter 6 Enable Clear Mask */
#define PMU_CNTENCLR_CNT7_ENABLE_MASK         (1ul << 7u)         /* PMU CNTENCLR: Event Counter 7 Enable Clear Mask */
#define PMU_CNTENCLR_CNT8_ENABLE_MASK         (1ul << 8u)         /* PMU CNTENCLR: Event Counter 8 Enable Clear Mask */
#define PMU_CNTENCLR_CNT9_ENABLE_MASK         (1ul << 9u)         /* PMU CNTENCLR: Event Counter 9 Enable Clear Mask */
#define PMU_CNTENCLR_CNT10_ENABLE_MASK        (1ul << 10u)        /* PMU CNTENCLR: Event Counter 10 Enable Clear Mask */
#define PMU_CNTENCLR_CNT11_ENABLE_MASK        (1ul << 11u)        /* PMU CNTENCLR: Event Counter 11 Enable Clear Mask */
#define PMU_CNTENCLR_CNT12_ENABLE_MASK        (1ul << 12u)        /* PMU CNTENCLR: Event Counter 12 Enable Clear Mask */
#define PMU_CNTENCLR_CNT13_ENABLE_MASK        (1ul << 13u)        /* PMU CNTENCLR: Event Counter 13 Enable Clear Mask */
#define PMU_CNTENCLR_CNT14_ENABLE_MASK        (1ul << 14u)        /* PMU CNTENCLR: Event Counter 14 Enable Clear Mask */
#define PMU_CNTENCLR_CNT15_ENABLE_MASK        (1ul << 15u)        /* PMU CNTENCLR: Event Counter 15 Enable Clear Mask */
#define PMU_CNTENCLR_CNT16_ENABLE_MASK        (1ul << 16u)        /* PMU CNTENCLR: Event Counter 16 Enable Clear Mask */
#define PMU_CNTENCLR_CNT17_ENABLE_MASK        (1ul << 17u)        /* PMU CNTENCLR: Event Counter 17 Enable Clear Mask */
#define PMU_CNTENCLR_CNT18_ENABLE_MASK        (1ul << 18u)        /* PMU CNTENCLR: Event Counter 18 Enable Clear Mask */
#define PMU_CNTENCLR_CNT19_ENABLE_MASK        (1ul << 19u)        /* PMU CNTENCLR: Event Counter 19 Enable Clear Mask */
#define PMU_CNTENCLR_CNT20_ENABLE_MASK        (1ul << 20u)        /* PMU CNTENCLR: Event Counter 20 Enable Clear Mask */
#define PMU_CNTENCLR_CNT21_ENABLE_MASK        (1ul << 21u)        /* PMU CNTENCLR: Event Counter 21 Enable Clear Mask */
#define PMU_CNTENCLR_CNT22_ENABLE_MASK        (1ul << 22u)        /* PMU CNTENCLR: Event Counter 22 Enable Clear Mask */
#define PMU_CNTENCLR_CNT23_ENABLE_MASK        (1ul << 23u)        /* PMU CNTENCLR: Event Counter 23 Enable Clear Mask */
#define PMU_CNTENCLR_CNT24_ENABLE_MASK        (1ul << 24u)        /* PMU CNTENCLR: Event Counter 24 Enable Clear Mask */
#define PMU_CNTENCLR_CNT25_ENABLE_MASK        (1ul << 25u)        /* PMU CNTENCLR: Event Counter 25 Enable Clear Mask */
#define PMU_CNTENCLR_CNT26_ENABLE_MASK        (1ul << 26u)        /* PMU CNTENCLR: Event Counter 26 Enable Clear Mask */
#define PMU_CNTENCLR_CNT27_ENABLE_MASK        (1ul << 27u)        /* PMU CNTENCLR: Event Counter 27 Enable Clear Mask */
#define PMU_CNTENCLR_CNT28_ENABLE_MASK        (1ul << 28u)        /* PMU CNTENCLR: Event Counter 28 Enable Clear Mask */
#define PMU_CNTENCLR_CNT29_ENABLE_MASK        (1ul << 29u)        /* PMU CNTENCLR: Event Counter 29 Enable Clear Mask */
#define PMU_CNTENCLR_CNT30_ENABLE_MASK        (1ul << 30u)        /* PMU CNTENCLR: Event Counter 30 Enable Clear Mask */
#define PMU_CNTENCLR_CCNTR_ENABLE_MASK        (1ul << 31u)        /* PMU CNTENCLR: Cycle Counter Enable Clear Mask */

/* PMU Interrupt Enable Set Register Definitions */

#define PMU_INTENSET_CNT0_ENABLE_MASK         (1ul << 0u)        /* PMU INTENSET: Event Counter 0 Interrupt Enable Set Mask */
#define PMU_INTENSET_CNT1_ENABLE_MASK         (1ul << 1u)        /* PMU INTENSET: Event Counter 1 Interrupt Enable Set Mask */
#define PMU_INTENSET_CNT2_ENABLE_MASK         (1ul << 2u)        /* PMU INTENSET: Event Counter 2 Interrupt Enable Set Mask */
#define PMU_INTENSET_CNT3_ENABLE_MASK         (1ul << 3u)        /* PMU INTENSET: Event Counter 3 Interrupt Enable Set Mask */
#define PMU_INTENSET_CNT4_ENABLE_MASK         (1ul << 4u)        /* PMU INTENSET: Event Counter 4 Interrupt Enable Set Mask */
#define PMU_INTENSET_CNT5_ENABLE_MASK         (1ul << 5u)        /* PMU INTENSET: Event Counter 5 Interrupt Enable Set Mask */
#define PMU_INTENSET_CNT6_ENABLE_MASK         (1ul << 6u)        /* PMU INTENSET: Event Counter 6 Interrupt Enable Set Mask */
#define PMU_INTENSET_CNT7_ENABLE_MASK         (1ul << 7u)        /* PMU INTENSET: Event Counter 7 Interrupt Enable Set Mask */
#define PMU_INTENSET_CNT8_ENABLE_MASK         (1ul << 8u)        /* PMU INTENSET: Event Counter 8 Interrupt Enable Set Mask */
#define PMU_INTENSET_CNT9_ENABLE_MASK         (1ul << 9u)        /* PMU INTENSET: Event Counter 9 Interrupt Enable Set Mask */
#define PMU_INTENSET_CNT10_ENABLE_MASK        (1ul << 10u)       /* PMU INTENSET: Event Counter 10 Interrupt Enable Set Mask */
#define PMU_INTENSET_CNT11_ENABLE_MASK        (1ul << 11u)       /* PMU INTENSET: Event Counter 11 Interrupt Enable Set Mask */
#define PMU_INTENSET_CNT12_ENABLE_MASK        (1ul << 12u)       /* PMU INTENSET: Event Counter 12 Interrupt Enable Set Mask */
#define PMU_INTENSET_CNT13_ENABLE_MASK        (1ul << 13u)       /* PMU INTENSET: Event Counter 13 Interrupt Enable Set Mask */
#define PMU_INTENSET_CNT14_ENABLE_MASK        (1ul << 14u)       /* PMU INTENSET: Event Counter 14 Interrupt Enable Set Mask */
#define PMU_INTENSET_CNT15_ENABLE_MASK        (1ul << 15u)       /* PMU INTENSET: Event Counter 15 Interrupt Enable Set Mask */
#define PMU_INTENSET_CNT16_ENABLE_MASK        (1ul << 16u)       /* PMU INTENSET: Event Counter 16 Interrupt Enable Set Mask */
#define PMU_INTENSET_CNT17_ENABLE_MASK        (1ul << 17u)       /* PMU INTENSET: Event Counter 17 Interrupt Enable Set Mask */
#define PMU_INTENSET_CNT18_ENABLE_MASK        (1ul << 18u)       /* PMU INTENSET: Event Counter 18 Interrupt Enable Set Mask */
#define PMU_INTENSET_CNT19_ENABLE_MASK        (1ul << 19u)       /* PMU INTENSET: Event Counter 19 Interrupt Enable Set Mask */
#define PMU_INTENSET_CNT20_ENABLE_MASK        (1ul << 20u)       /* PMU INTENSET: Event Counter 20 Interrupt Enable Set Mask */
#define PMU_INTENSET_CNT21_ENABLE_MASK        (1ul << 21u)       /* PMU INTENSET: Event Counter 21 Interrupt Enable Set Mask */
#define PMU_INTENSET_CNT22_ENABLE_MASK        (1ul << 22u)       /* PMU INTENSET: Event Counter 22 Interrupt Enable Set Mask */
#define PMU_INTENSET_CNT23_ENABLE_MASK        (1ul << 23u)       /* PMU INTENSET: Event Counter 23 Interrupt Enable Set Mask */
#define PMU_INTENSET_CNT24_ENABLE_MASK        (1ul << 24u)       /* PMU INTENSET: Event Counter 24 Interrupt Enable Set Mask */
#define PMU_INTENSET_CNT25_ENABLE_MASK        (1ul << 25u)       /* PMU INTENSET: Event Counter 25 Interrupt Enable Set Mask */
#define PMU_INTENSET_CNT26_ENABLE_MASK        (1ul << 26u)       /* PMU INTENSET: Event Counter 26 Interrupt Enable Set Mask */
#define PMU_INTENSET_CNT27_ENABLE_MASK        (1ul << 27u)       /* PMU INTENSET: Event Counter 27 Interrupt Enable Set Mask */
#define PMU_INTENSET_CNT28_ENABLE_MASK        (1ul << 28u)       /* PMU INTENSET: Event Counter 28 Interrupt Enable Set Mask */
#define PMU_INTENSET_CNT29_ENABLE_MASK        (1ul << 29u)       /* PMU INTENSET: Event Counter 29 Interrupt Enable Set Mask */
#define PMU_INTENSET_CNT30_ENABLE_MASK        (1ul << 30u)       /* PMU INTENSET: Event Counter 30 Interrupt Enable Set Mask */
#define PMU_INTENSET_CCYCNT_ENABLE_MASK       (1ul << 31u)       /* PMU INTENSET: Cycle Counter Interrupt Enable Set Mask */

/* PMU Interrupt Enable Clear Register Definitions */

#define PMU_INTENCLR_CNT0_ENABLE_MASK         (1ul << 0u)        /* PMU INTENCLR: Event Counter 0 Interrupt Enable Clear Mask */
#define PMU_INTENCLR_CNT1_ENABLE_MASK         (1ul << 1u)        /* PMU INTENCLR: Event Counter 1 Interrupt Enable Clear */
#define PMU_INTENCLR_CNT2_ENABLE_MASK         (1ul << 2u)        /* PMU INTENCLR: Event Counter 2 Interrupt Enable Clear Mask */
#define PMU_INTENCLR_CNT3_ENABLE_MASK         (1ul << 3u)        /* PMU INTENCLR: Event Counter 3 Interrupt Enable Clear Mask */
#define PMU_INTENCLR_CNT4_ENABLE_MASK         (1ul << 4u)        /* PMU INTENCLR: Event Counter 4 Interrupt Enable Clear Mask */
#define PMU_INTENCLR_CNT5_ENABLE_MASK         (1ul << 5u)        /* PMU INTENCLR: Event Counter 5 Interrupt Enable Clear Mask */
#define PMU_INTENCLR_CNT6_ENABLE_MASK         (1ul << 6u)        /* PMU INTENCLR: Event Counter 6 Interrupt Enable Clear Mask */
#define PMU_INTENCLR_CNT7_ENABLE_MASK         (1ul << 7u)        /* PMU INTENCLR: Event Counter 7 Interrupt Enable Clear Mask */
#define PMU_INTENCLR_CNT8_ENABLE_MASK         (1ul << 8u)        /* PMU INTENCLR: Event Counter 8 Interrupt Enable Clear Mask */
#define PMU_INTENCLR_CNT9_ENABLE_MASK         (1ul << 9u)        /* PMU INTENCLR: Event Counter 9 Interrupt Enable Clear Mask */
#define PMU_INTENCLR_CNT10_ENABLE_MASK        (1ul << 10u)       /* PMU INTENCLR: Event Counter 10 Interrupt Enable Clear Mask */
#define PMU_INTENCLR_CNT11_ENABLE_MASK        (1ul << 11u)       /* PMU INTENCLR: Event Counter 11 Interrupt Enable Clear Mask */
#define PMU_INTENCLR_CNT12_ENABLE_MASK        (1ul << 12u)       /* PMU INTENCLR: Event Counter 12 Interrupt Enable Clear Mask */
#define PMU_INTENCLR_CNT13_ENABLE_MASK        (1ul << 13u)       /* PMU INTENCLR: Event Counter 13 Interrupt Enable Clear Mask */
#define PMU_INTENCLR_CNT14_ENABLE_MASK        (1ul << 14u)       /* PMU INTENCLR: Event Counter 14 Interrupt Enable Clear Mask */
#define PMU_INTENCLR_CNT15_ENABLE_MASK        (1ul << 15u)       /* PMU INTENCLR: Event Counter 15 Interrupt Enable Clear Mask */
#define PMU_INTENCLR_CNT16_ENABLE_MASK        (1ul << 16u)       /* PMU INTENCLR: Event Counter 16 Interrupt Enable Clear Mask */
#define PMU_INTENCLR_CNT17_ENABLE_MASK        (1ul << 17u)       /* PMU INTENCLR: Event Counter 17 Interrupt Enable Clear Mask */
#define PMU_INTENCLR_CNT18_ENABLE_MASK        (1ul << 18u)       /* PMU INTENCLR: Event Counter 18 Interrupt Enable Clear Mask */
#define PMU_INTENCLR_CNT19_ENABLE_MASK        (1ul << 19u)       /* PMU INTENCLR: Event Counter 19 Interrupt Enable Clear Mask */
#define PMU_INTENCLR_CNT20_ENABLE_MASK        (1ul << 20u)       /* PMU INTENCLR: Event Counter 20 Interrupt Enable Clear Mask */
#define PMU_INTENCLR_CNT21_ENABLE_MASK        (1ul << 21u)       /* PMU INTENCLR: Event Counter 21 Interrupt Enable Clear Mask */
#define PMU_INTENCLR_CNT22_ENABLE_MASK        (1ul << 22u)       /* PMU INTENCLR: Event Counter 22 Interrupt Enable Clear Mask */
#define PMU_INTENCLR_CNT23_ENABLE_MASK        (1ul << 23u)       /* PMU INTENCLR: Event Counter 23 Interrupt Enable Clear Mask */
#define PMU_INTENCLR_CNT24_ENABLE_MASK        (1ul << 24u)       /* PMU INTENCLR: Event Counter 24 Interrupt Enable Clear Mask */
#define PMU_INTENCLR_CNT25_ENABLE_MASK        (1ul << 25u)       /* PMU INTENCLR: Event Counter 25 Interrupt Enable Clear Mask */
#define PMU_INTENCLR_CNT26_ENABLE_MASK        (1ul << 26u)       /* PMU INTENCLR: Event Counter 26 Interrupt Enable Clear Mask */
#define PMU_INTENCLR_CNT27_ENABLE_MASK        (1ul << 27u)       /* PMU INTENCLR: Event Counter 27 Interrupt Enable Clear Mask */
#define PMU_INTENCLR_CNT28_ENABLE_MASK        (1ul << 28u)       /* PMU INTENCLR: Event Counter 28 Interrupt Enable Clear Mask */
#define PMU_INTENCLR_CNT29_ENABLE_MASK        (1ul << 29u)       /* PMU INTENCLR: Event Counter 29 Interrupt Enable Clear Mask */
#define PMU_INTENCLR_CNT30_ENABLE_MASK        (1ul << 30u)       /* PMU INTENCLR: Event Counter 30 Interrupt Enable Clear Mask */
#define PMU_INTENCLR_CYCCNT_ENABLE_MASK       (1ul << 31u)       /* PMU INTENCLR: Cycle Counter Interrupt Enable Clear Mask */

/* PMU Overflow Flag Status Set Register Definitions */

#define PMU_OVSSET_CNT0_STATUS_MASK           (1ul << 0u)        /* PMU OVSSET: Event Counter 0 Overflow Set Mask */
#define PMU_OVSSET_CNT1_STATUS_MASK           (1ul << 1u)        /* PMU OVSSET: Event Counter 1 Overflow Set Mask */
#define PMU_OVSSET_CNT2_STATUS_MASK           (1ul << 2u)        /* PMU OVSSET: Event Counter 2 Overflow Set Mask */
#define PMU_OVSSET_CNT3_STATUS_MASK           (1ul << 3u)        /* PMU OVSSET: Event Counter 3 Overflow Set Mask */
#define PMU_OVSSET_CNT4_STATUS_MASK           (1ul << 4u)        /* PMU OVSSET: Event Counter 4 Overflow Set Mask */
#define PMU_OVSSET_CNT5_STATUS_MASK           (1ul << 5u)        /* PMU OVSSET: Event Counter 5 Overflow Set Mask */
#define PMU_OVSSET_CNT6_STATUS_MASK           (1ul << 6u)        /* PMU OVSSET: Event Counter 6 Overflow Set Mask */
#define PMU_OVSSET_CNT7_STATUS_MASK           (1ul << 7u)        /* PMU OVSSET: Event Counter 7 Overflow Set Mask */
#define PMU_OVSSET_CNT8_STATUS_MASK           (1ul << 8u)        /* PMU OVSSET: Event Counter 8 Overflow Set Mask */
#define PMU_OVSSET_CNT9_STATUS_MASK           (1ul << 9u)        /* PMU OVSSET: Event Counter 9 Overflow Set Mask */
#define PMU_OVSSET_CNT10_STATUS_MASK          (1ul << 10u)       /* PMU OVSSET: Event Counter 10 Overflow Set Mask */
#define PMU_OVSSET_CNT11_STATUS_MASK          (1ul << 11u)       /* PMU OVSSET: Event Counter 11 Overflow Set Mask */
#define PMU_OVSSET_CNT12_STATUS_MASK          (1ul << 12u)       /* PMU OVSSET: Event Counter 12 Overflow Set Mask */
#define PMU_OVSSET_CNT13_STATUS_MASK          (1ul << 13u)       /* PMU OVSSET: Event Counter 13 Overflow Set Mask */
#define PMU_OVSSET_CNT14_STATUS_MASK          (1ul << 14u)       /* PMU OVSSET: Event Counter 14 Overflow Set Mask */
#define PMU_OVSSET_CNT15_STATUS_MASK          (1ul << 15u)       /* PMU OVSSET: Event Counter 15 Overflow Set Mask */
#define PMU_OVSSET_CNT16_STATUS_MASK          (1ul << 16u)       /* PMU OVSSET: Event Counter 16 Overflow Set Mask */
#define PMU_OVSSET_CNT17_STATUS_MASK          (1ul << 17u)       /* PMU OVSSET: Event Counter 17 Overflow Set Mask */
#define PMU_OVSSET_CNT18_STATUS_MASK          (1ul << 18u)       /* PMU OVSSET: Event Counter 18 Overflow Set Mask */
#define PMU_OVSSET_CNT19_STATUS_MASK          (1ul << 19u)       /* PMU OVSSET: Event Counter 19 Overflow Set Mask */
#define PMU_OVSSET_CNT20_STATUS_MASK          (1ul << 20u)       /* PMU OVSSET: Event Counter 20 Overflow Set Mask */
#define PMU_OVSSET_CNT21_STATUS_MASK          (1ul << 21u)       /* PMU OVSSET: Event Counter 21 Overflow Set Mask */
#define PMU_OVSSET_CNT22_STATUS_MASK          (1ul << 22u)       /* PMU OVSSET: Event Counter 22 Overflow Set Mask */
#define PMU_OVSSET_CNT23_STATUS_MASK          (1ul << 23u)       /* PMU OVSSET: Event Counter 23 Overflow Set Mask */
#define PMU_OVSSET_CNT24_STATUS_MASK          (1ul << 24u)       /* PMU OVSSET: Event Counter 24 Overflow Set Mask */
#define PMU_OVSSET_CNT25_STATUS_MASK          (1ul << 25u)       /* PMU OVSSET: Event Counter 25 Overflow Set Mask */
#define PMU_OVSSET_CNT26_STATUS_MASK          (1ul << 26u)       /* PMU OVSSET: Event Counter 26 Overflow Set Mask */
#define PMU_OVSSET_CNT27_STATUS_MASK          (1ul << 27u)       /* PMU OVSSET: Event Counter 27 Overflow Set Mask */
#define PMU_OVSSET_CNT28_STATUS_MASK          (1ul << 28u)       /* PMU OVSSET: Event Counter 28 Overflow Set Mask */
#define PMU_OVSSET_CNT29_STATUS_MASK          (1ul << 29u)       /* PMU OVSSET: Event Counter 29 Overflow Set Mask */
#define PMU_OVSSET_CNT30_STATUS_MASK          (1ul << 30u)       /* PMU OVSSET: Event Counter 30 Overflow Set Mask */
#define PMU_OVSSET_CYCCNT_STATUS_MASK         (1ul << 31u)       /* PMU OVSSET: Cycle Counter Overflow Set Mask */

/* PMU Overflow Flag Status Clear Register Definitions */

#define PMU_OVSCLR_CNT0_STATUS_MASK           (1ul << 0u)        /* PMU OVSCLR: Event Counter 0 Overflow Clear Mask */
#define PMU_OVSCLR_CNT1_STATUS_MASK           (1ul << 1u)        /* PMU OVSCLR: Event Counter 1 Overflow Clear */
#define PMU_OVSCLR_CNT2_STATUS_MASK           (1ul << 2u)        /* PMU OVSCLR: Event Counter 2 Overflow Clear Mask */
#define PMU_OVSCLR_CNT3_STATUS_MASK           (1ul << 3u)        /* PMU OVSCLR: Event Counter 3 Overflow Clear Mask */
#define PMU_OVSCLR_CNT4_STATUS_MASK           (1ul << 4u)        /* PMU OVSCLR: Event Counter 4 Overflow Clear Mask */
#define PMU_OVSCLR_CNT5_STATUS_MASK           (1ul << 5u)        /* PMU OVSCLR: Event Counter 5 Overflow Clear Mask */
#define PMU_OVSCLR_CNT6_STATUS_MASK           (1ul << 6u)        /* PMU OVSCLR: Event Counter 6 Overflow Clear Mask */
#define PMU_OVSCLR_CNT7_STATUS_MASK           (1ul << 7u)        /* PMU OVSCLR: Event Counter 7 Overflow Clear Mask */
#define PMU_OVSCLR_CNT8_STATUS_MASK           (1ul << 8u)        /* PMU OVSCLR: Event Counter 8 Overflow Clear Mask */
#define PMU_OVSCLR_CNT9_STATUS_MASK           (1ul << 9u)        /* PMU OVSCLR: Event Counter 9 Overflow Clear Mask */
#define PMU_OVSCLR_CNT10_STATUS_MASK          (1ul << 10u)       /* PMU OVSCLR: Event Counter 10 Overflow Clear Mask */
#define PMU_OVSCLR_CNT11_STATUS_MASK          (1ul << 11u)       /* PMU OVSCLR: Event Counter 11 Overflow Clear Mask */
#define PMU_OVSCLR_CNT12_STATUS_MASK          (1ul << 12u)       /* PMU OVSCLR: Event Counter 12 Overflow Clear Mask */
#define PMU_OVSCLR_CNT13_STATUS_MASK          (1ul << 13u)       /* PMU OVSCLR: Event Counter 13 Overflow Clear Mask */
#define PMU_OVSCLR_CNT14_STATUS_MASK          (1ul << 14u)       /* PMU OVSCLR: Event Counter 14 Overflow Clear Mask */
#define PMU_OVSCLR_CNT15_STATUS_MASK          (1ul << 15u)       /* PMU OVSCLR: Event Counter 15 Overflow Clear Mask */
#define PMU_OVSCLR_CNT16_STATUS_MASK          (1ul << 16u)       /* PMU OVSCLR: Event Counter 16 Overflow Clear Mask */
#define PMU_OVSCLR_CNT17_STATUS_MASK          (1ul << 17u)       /* PMU OVSCLR: Event Counter 17 Overflow Clear Mask */
#define PMU_OVSCLR_CNT18_STATUS_MASK          (1ul << 18u)       /* PMU OVSCLR: Event Counter 18 Overflow Clear Mask */
#define PMU_OVSCLR_CNT19_STATUS_MASK          (1ul << 19u)       /* PMU OVSCLR: Event Counter 19 Overflow Clear Mask */
#define PMU_OVSCLR_CNT20_STATUS_MASK          (1ul << 20u)       /* PMU OVSCLR: Event Counter 20 Overflow Clear Mask */
#define PMU_OVSCLR_CNT21_STATUS_MASK          (1ul << 21u)       /* PMU OVSCLR: Event Counter 21 Overflow Clear Mask */
#define PMU_OVSCLR_CNT22_STATUS_MASK          (1ul << 22u)       /* PMU OVSCLR: Event Counter 22 Overflow Clear Mask */
#define PMU_OVSCLR_CNT23_STATUS_MASK          (1ul << 23u)       /* PMU OVSCLR: Event Counter 23 Overflow Clear Mask */
#define PMU_OVSCLR_CNT24_STATUS_MASK          (1ul << 24u)       /* PMU OVSCLR: Event Counter 24 Overflow Clear Mask */
#define PMU_OVSCLR_CNT25_STATUS_MASK          (1ul << 25u)       /* PMU OVSCLR: Event Counter 25 Overflow Clear Mask */
#define PMU_OVSCLR_CNT26_STATUS_MASK          (1ul << 26u)       /* PMU OVSCLR: Event Counter 26 Overflow Clear Mask */
#define PMU_OVSCLR_CNT27_STATUS_MASK          (1ul << 27u)       /* PMU OVSCLR: Event Counter 27 Overflow Clear Mask */
#define PMU_OVSCLR_CNT28_STATUS_MASK          (1ul << 28u)       /* PMU OVSCLR: Event Counter 28 Overflow Clear Mask */
#define PMU_OVSCLR_CNT29_STATUS_MASK          (1ul << 29u)       /* PMU OVSCLR: Event Counter 29 Overflow Clear Mask */
#define PMU_OVSCLR_CNT30_STATUS_MASK          (1ul << 30u)       /* PMU OVSCLR: Event Counter 30 Overflow Clear Mask */
#define PMU_OVSCLR_CYCCNT_STATUS_MASK         (1ul << 31u)       /* PMU OVSCLR: Cycle Counter Overflow Clear Mask */

/* PMU Software Increment Counter */

#define PMU_SWINC_CNT0_MASK                   (1ul << 0u)        /* PMU SWINC: Event Counter 0 Software Increment Mask */
#define PMU_SWINC_CNT1_MASK                   (1ul << 1u)        /* PMU SWINC: Event Counter 1 Software Increment Mask */
#define PMU_SWINC_CNT2_MASK                   (1ul << 2u)        /* PMU SWINC: Event Counter 2 Software Increment Mask */
#define PMU_SWINC_CNT3_MASK                   (1ul << 3u)        /* PMU SWINC: Event Counter 3 Software Increment Mask */
#define PMU_SWINC_CNT4_MASK                   (1ul << 4u)        /* PMU SWINC: Event Counter 4 Software Increment Mask */
#define PMU_SWINC_CNT5_MASK                   (1ul << 5u)        /* PMU SWINC: Event Counter 5 Software Increment Mask */
#define PMU_SWINC_CNT6_MASK                   (1ul << 6u)        /* PMU SWINC: Event Counter 6 Software Increment Mask */
#define PMU_SWINC_CNT7_MASK                   (1ul << 7u)        /* PMU SWINC: Event Counter 7 Software Increment Mask */
#define PMU_SWINC_CNT8_MASK                   (1ul << 8u)        /* PMU SWINC: Event Counter 8 Software Increment Mask */
#define PMU_SWINC_CNT9_MASK                   (1ul << 9u)        /* PMU SWINC: Event Counter 9 Software Increment Mask */
#define PMU_SWINC_CNT10_MASK                  (1ul << 10u)       /* PMU SWINC: Event Counter 10 Software Increment Mask */
#define PMU_SWINC_CNT11_MASK                  (1ul << 11u)       /* PMU SWINC: Event Counter 11 Software Increment Mask */
#define PMU_SWINC_CNT12_MASK                  (1ul << 12u)       /* PMU SWINC: Event Counter 12 Software Increment Mask */
#define PMU_SWINC_CNT13_MASK                  (1ul << 13u)       /* PMU SWINC: Event Counter 13 Software Increment Mask */
#define PMU_SWINC_CNT14_MASK                  (1ul << 14u)       /* PMU SWINC: Event Counter 14 Software Increment Mask */
#define PMU_SWINC_CNT15_MASK                  (1ul << 15u)       /* PMU SWINC: Event Counter 15 Software Increment Mask */
#define PMU_SWINC_CNT16_MASK                  (1ul << 16u)       /* PMU SWINC: Event Counter 16 Software Increment Mask */
#define PMU_SWINC_CNT17_MASK                  (1ul << 17u)       /* PMU SWINC: Event Counter 17 Software Increment Mask */
#define PMU_SWINC_CNT18_MASK                  (1ul << 18u)       /* PMU SWINC: Event Counter 18 Software Increment Mask */
#define PMU_SWINC_CNT19_MASK                  (1ul << 19u)       /* PMU SWINC: Event Counter 19 Software Increment Mask */
#define PMU_SWINC_CNT20_MASK                  (1ul << 20u)       /* PMU SWINC: Event Counter 20 Software Increment Mask */
#define PMU_SWINC_CNT21_MASK                  (1ul << 21u)       /* PMU SWINC: Event Counter 21 Software Increment Mask */
#define PMU_SWINC_CNT22_MASK                  (1ul << 22u)       /* PMU SWINC: Event Counter 22 Software Increment Mask */
#define PMU_SWINC_CNT23_MASK                  (1ul << 23u)       /* PMU SWINC: Event Counter 23 Software Increment Mask */
#define PMU_SWINC_CNT24_MASK                  (1ul << 24u)       /* PMU SWINC: Event Counter 24 Software Increment Mask */
#define PMU_SWINC_CNT25_MASK                  (1ul << 25u)       /* PMU SWINC: Event Counter 25 Software Increment Mask */
#define PMU_SWINC_CNT26_MASK                  (1ul << 26u)       /* PMU SWINC: Event Counter 26 Software Increment Mask */
#define PMU_SWINC_CNT27_MASK                  (1ul << 27u)       /* PMU SWINC: Event Counter 27 Software Increment Mask */
#define PMU_SWINC_CNT28_MASK                  (1ul << 28u)       /* PMU SWINC: Event Counter 28 Software Increment Mask */
#define PMU_SWINC_CNT29_MASK                  (1ul << 29u)       /* PMU SWINC: Event Counter 29 Software Increment Mask */
#define PMU_SWINC_CNT30_MASK                  (1ul << 30u)       /* PMU SWINC: Event Counter 30 Software Increment Mask */

/* PMU Control Register Definitions */

#define PMU_CTRL_ENABLE_MASK                  (1ul << 0u)        /* PMU CTRL: ENABLE Mask */
#define PMU_CTRL_EVENTCNT_RESET_MASK          (1ul << 1u)        /* PMU CTRL: Event Counter Reset Mask */
#define PMU_CTRL_CYCCNT_RESET_MASK            (1ul << 2u)        /* PMU CTRL: Cycle Counter Reset Mask */
#define PMU_CTRL_CYCCNT_DISABLE_MASK          (1ul << 5u)        /* PMU CTRL: Disable Cycle Counter Mask */
#define PMU_CTRL_FRZ_ON_OV_MASK               (1ul << 9u)        /* PMU CTRL: Freeze-on-overflow Mask */
#define PMU_CTRL_TRACE_ON_OV_MASK             (1ul << 11u)       /* PMU CTRL: Trace-on-overflow Mask */

/* PMU Type Register Definitions */

#define PMU_TYPE_NUM_CNTS_MASK                (0xfful << 0u)     /* PMU TYPE: Number of Counters Mask */
#define PMU_TYPE_SIZE_CNTS_MASK               (0x3ful << 8u)     /* PMU TYPE: Size of Counters Mask */
#define PMU_TYPE_CYCCNT_PRESENT_MASK          (1ul << 14u)       /* PMU TYPE: Cycle Counter Present Mask */
#define PMU_TYPE_FRZ_OV_SUPPORT_MASK          (1ul << 21u)       /* PMU TYPE: Freeze-on-overflow Support Mask */
#define PMU_TYPE_TRACE_ON_OV_SUPPORT_MASK     (1ul << 23u)       /* PMU TYPE: Trace-on-overflow Support Mask */

/* PMU Authentication Status Register Definitions */

#define PMU_AUTHSTATUS_NSID_MASK              (0x3UL << 0U)      /* PMU AUTHSTATUS: Non-secure Invasive Debug Mask */
#define PMU_AUTHSTATUS_NSNID_MASK             (0x3UL << 2U)      /* PMU AUTHSTATUS: Non-secure Non-invasive Debug Mask */
#define PMU_AUTHSTATUS_SID_MASK               (0x3UL << 4U)      /* PMU AUTHSTATUS: Secure Invasive Debug Mask */
#define PMU_AUTHSTATUS_SNID_MASK              (0x3UL << 6U)      /* PMU AUTHSTATUS: Secure Non-invasive Debug Mask */
#define PMU_AUTHSTATUS_NSUID_MASK             (0x3UL << 16U)     /* PMU AUTHSTATUS: Non-secure Unprivileged Invasive Debug Mask */
#define PMU_AUTHSTATUS_NSUNID_MASK            (0x3UL << 18U)     /* PMU AUTHSTATUS: Non-secure Unprivileged Non-invasive Debug Mask */
#define PMU_AUTHSTATUS_SUID_MASK              (0x3UL << 20U)     /* PMU AUTHSTATUS: Secure Unprivileged Invasive Debug Mask */
#define PMU_AUTHSTATUS_SUNID_MASK             (0x3UL << 22U)     /* PMU AUTHSTATUS: Secure Unprivileged Non-invasive Debug Mask */

/* PMU Events. See the Armv8.1-M Architecture Reference Manual
 * for full details on these PMU events.
 */

#define PMU_SW_INCR                              0x0000          /* Software update to the PMU_SWINC register, architecturally executed and condition code check pass */
#define PMU_L1I_CACHE_REFILL                     0x0001          /* L1 I-Cache refill */
#define PMU_L1D_CACHE_REFILL                     0x0003          /* L1 D-Cache refill */
#define PMU_L1D_CACHE                            0x0004          /* L1 D-Cache access */
#define PMU_LD_RETIRED                           0x0006          /* Memory-reading instruction architecturally executed and condition code check pass */
#define PMU_ST_RETIRED                           0x0007          /* Memory-writing instruction architecturally executed and condition code check pass */
#define PMU_INST_RETIRED                         0x0008          /* Instruction architecturally executed */
#define PMU_EXC_TAKEN                            0x0009          /* Exception entry */
#define PMU_EXC_RETURN                           0x000a          /* Exception return instruction architecturally executed and the condition code check pass */
#define PMU_PC_WRITE_RETIRED                     0x000c          /* Software change to the Program Counter (PC). Instruction is architecturally executed and condition code check pass */
#define PMU_BR_IMMED_RETIRED                     0x000d          /* Immediate branch architecturally executed */
#define PMU_BR_RETURN_RETIRED                    0x000e          /* Function return instruction architecturally executed and the condition code check pass */
#define PMU_UNALIGNED_LDST_RETIRED               0x000f          /* Unaligned memory memory-reading or memory-writing instruction architecturally executed and condition code check pass */
#define PMU_BR_MIS_PRED                          0x0010          /* Mispredicted or not predicted branch speculatively executed */
#define PMU_CPU_CYCLES                           0x0011          /* Cycle */
#define PMU_BR_PRED                              0x0012          /* Predictable branch speculatively executed */
#define PMU_MEM_ACCESS                           0x0013          /* Data memory access */
#define PMU_L1I_CACHE                            0x0014          /* Level 1 instruction cache access */
#define PMU_L1D_CACHE_WB                         0x0015          /* Level 1 data cache write-back */
#define PMU_L2D_CACHE                            0x0016          /* Level 2 data cache access */
#define PMU_L2D_CACHE_REFILL                     0x0017          /* Level 2 data cache refill */
#define PMU_L2D_CACHE_WB                         0x0018          /* Level 2 data cache write-back */
#define PMU_BUS_ACCESS                           0x0019          /* Bus access */
#define PMU_MEMORY_ERROR                         0x001a          /* Local memory error */
#define PMU_INST_SPEC                            0x001b          /* Instruction speculatively executed */
#define PMU_BUS_CYCLES                           0x001d          /* Bus cycles */
#define PMU_CHAIN                                0x001e          /* For an odd numbered counter, increment when an overflow occurs on the preceding even-numbered counter on the same PE */
#define PMU_L1D_CACHE_ALLOCATE                   0x001f          /* Level 1 data cache allocation without refill */
#define PMU_L2D_CACHE_ALLOCATE                   0x0020          /* Level 2 data cache allocation without refill */
#define PMU_BR_RETIRED                           0x0021          /* Branch instruction architecturally executed */
#define PMU_BR_MIS_PRED_RETIRED                  0x0022          /* Mispredicted branch instruction architecturally executed */
#define PMU_STALL_FRONTEND                       0x0023          /* No operation issued because of the frontend */
#define PMU_STALL_BACKEND                        0x0024          /* No operation issued because of the backend */
#define PMU_L2I_CACHE                            0x0027          /* Level 2 instruction cache access */
#define PMU_L2I_CACHE_REFILL                     0x0028          /* Level 2 instruction cache refill */
#define PMU_L3D_CACHE_ALLOCATE                   0x0029          /* Level 3 data cache allocation without refill */
#define PMU_L3D_CACHE_REFILL                     0x002a          /* Level 3 data cache refill */
#define PMU_L3D_CACHE                            0x002b          /* Level 3 data cache access */
#define PMU_L3D_CACHE_WB                         0x002c          /* Level 3 data cache write-back */
#define PMU_LL_CACHE_RD                          0x0036          /* Last level data cache read */
#define PMU_LL_CACHE_MISS_RD                     0x0037          /* Last level data cache read miss */
#define PMU_L1D_CACHE_MISS_RD                    0x0039          /* Level 1 data cache read miss */
#define PMU_OP_COMPLETE                          0x003a          /* Operation retired */
#define PMU_OP_SPEC                              0x003b          /* Operation speculatively executed */
#define PMU_STALL                                0x003c          /* Stall cycle for instruction or operation not sent for execution */
#define PMU_STALL_OP_BACKEND                     0x003d          /* Stall cycle for instruction or operation not sent for execution due to pipeline backend */
#define PMU_STALL_OP_FRONTEND                    0x003e          /* Stall cycle for instruction or operation not sent for execution due to pipeline frontend */
#define PMU_STALL_OP                             0x003f          /* Instruction or operation slots not occupied each cycle */
#define PMU_L1D_CACHE_RD                         0x0040          /* Level 1 data cache read */
#define PMU_LE_RETIRED                           0x0100          /* Loop end instruction executed */
#define PMU_LE_SPEC                              0x0101          /* Loop end instruction speculatively executed */
#define PMU_BF_RETIRED                           0x0104          /* Branch future instruction architecturally executed and condition code check pass */
#define PMU_BF_SPEC                              0x0105          /* Branch future instruction speculatively executed and condition code check pass */
#define PMU_LE_CANCEL                            0x0108          /* Loop end instruction not taken */
#define PMU_BF_CANCEL                            0x0109          /* Branch future instruction not taken */
#define PMU_SE_CALL_S                            0x0114          /* Call to secure function, resulting in Security state change */
#define PMU_SE_CALL_NS                           0x0115          /* Call to non-secure function, resulting in Security state change */
#define PMU_DWT_CMPMATCH0                        0x0118          /* DWT comparator 0 match */
#define PMU_DWT_CMPMATCH1                        0x0119          /* DWT comparator 1 match */
#define PMU_DWT_CMPMATCH2                        0x011a          /* DWT comparator 2 match */
#define PMU_DWT_CMPMATCH3                        0x011b          /* DWT comparator 3 match */
#define PMU_MVE_INST_RETIRED                     0x0200          /* MVE instruction architecturally executed */
#define PMU_MVE_INST_SPEC                        0x0201          /* MVE instruction speculatively executed */
#define PMU_MVE_FP_RETIRED                       0x0204          /* MVE floating-point instruction architecturally executed */
#define PMU_MVE_FP_SPEC                          0x0205          /* MVE floating-point instruction speculatively executed */
#define PMU_MVE_FP_HP_RETIRED                    0x0208          /* MVE half-precision floating-point instruction architecturally executed */
#define PMU_MVE_FP_HP_SPEC                       0x0209          /* MVE half-precision floating-point instruction speculatively executed */
#define PMU_MVE_FP_SP_RETIRED                    0x020c          /* MVE single-precision floating-point instruction architecturally executed */
#define PMU_MVE_FP_SP_SPEC                       0x020d          /* MVE single-precision floating-point instruction speculatively executed */
#define PMU_MVE_FP_MAC_RETIRED                   0x0214          /* MVE floating-point multiply or multiply-accumulate instruction architecturally executed */
#define PMU_MVE_FP_MAC_SPEC                      0x0215          /* MVE floating-point multiply or multiply-accumulate instruction speculatively executed */
#define PMU_MVE_INT_RETIRED                      0x0224          /* MVE integer instruction architecturally executed */
#define PMU_MVE_INT_SPEC                         0x0225          /* MVE integer instruction speculatively executed */
#define PMU_MVE_INT_MAC_RETIRED                  0x0228          /* MVE multiply or multiply-accumulate instruction architecturally executed */
#define PMU_MVE_INT_MAC_SPEC                     0x0229          /* MVE multiply or multiply-accumulate instruction speculatively executed */
#define PMU_MVE_LDST_RETIRED                     0x0238          /* MVE load or store instruction architecturally executed */
#define PMU_MVE_LDST_SPEC                        0x0239          /* MVE load or store instruction speculatively executed */
#define PMU_MVE_LD_RETIRED                       0x023c          /* MVE load instruction architecturally executed */
#define PMU_MVE_LD_SPEC                          0x023d          /* MVE load instruction speculatively executed */
#define PMU_MVE_ST_RETIRED                       0x0240          /* MVE store instruction architecturally executed */
#define PMU_MVE_ST_SPEC                          0x0241          /* MVE store instruction speculatively executed */
#define PMU_MVE_LDST_CONTIG_RETIRED              0x0244          /* MVE contiguous load or store instruction architecturally executed */
#define PMU_MVE_LDST_CONTIG_SPEC                 0x0245          /* MVE contiguous load or store instruction speculatively executed */
#define PMU_MVE_LD_CONTIG_RETIRED                0x0248          /* MVE contiguous load instruction architecturally executed */
#define PMU_MVE_LD_CONTIG_SPEC                   0x0249          /* MVE contiguous load instruction speculatively executed */
#define PMU_MVE_ST_CONTIG_RETIRED                0x024c          /* MVE contiguous store instruction architecturally executed */
#define PMU_MVE_ST_CONTIG_SPEC                   0x024d          /* MVE contiguous store instruction speculatively executed */
#define PMU_MVE_LDST_NONCONTIG_RETIRED           0x0250          /* MVE non-contiguous load or store instruction architecturally executed */
#define PMU_MVE_LDST_NONCONTIG_SPEC              0x0251          /* MVE non-contiguous load or store instruction speculatively executed */
#define PMU_MVE_LD_NONCONTIG_RETIRED             0x0254          /* MVE non-contiguous load instruction architecturally executed */
#define PMU_MVE_LD_NONCONTIG_SPEC                0x0255          /* MVE non-contiguous load instruction speculatively executed */
#define PMU_MVE_ST_NONCONTIG_RETIRED             0x0258          /* MVE non-contiguous store instruction architecturally executed */
#define PMU_MVE_ST_NONCONTIG_SPEC                0x0259          /* MVE non-contiguous store instruction speculatively executed */
#define PMU_MVE_LDST_MULTI_RETIRED               0x025c          /* MVE memory instruction targeting multiple registers architecturally executed */
#define PMU_MVE_LDST_MULTI_SPEC                  0x025d          /* MVE memory instruction targeting multiple registers speculatively executed */
#define PMU_MVE_LD_MULTI_RETIRED                 0x0260          /* MVE memory load instruction targeting multiple registers architecturally executed */
#define PMU_MVE_LD_MULTI_SPEC                    0x0261          /* MVE memory load instruction targeting multiple registers speculatively executed */
#define PMU_MVE_ST_MULTI_RETIRED                 0x0261          /* MVE memory store instruction targeting multiple registers architecturally executed */
#define PMU_MVE_ST_MULTI_SPEC                    0x0265          /* MVE memory store instruction targeting multiple registers speculatively executed */
#define PMU_MVE_LDST_UNALIGNED_RETIRED           0x028c          /* MVE unaligned memory load or store instruction architecturally executed */
#define PMU_MVE_LDST_UNALIGNED_SPEC              0x028d          /* MVE unaligned memory load or store instruction speculatively executed */
#define PMU_MVE_LD_UNALIGNED_RETIRED             0x0290          /* MVE unaligned load instruction architecturally executed */
#define PMU_MVE_LD_UNALIGNED_SPEC                0x0291          /* MVE unaligned load instruction speculatively executed */
#define PMU_MVE_ST_UNALIGNED_RETIRED             0x0294          /* MVE unaligned store instruction architecturally executed */
#define PMU_MVE_ST_UNALIGNED_SPEC                0x0295          /* MVE unaligned store instruction speculatively executed */
#define PMU_MVE_LDST_UNALIGNED_NONCONTIG_RETIRED 0x0298          /* MVE unaligned noncontiguous load or store instruction architecturally executed */
#define PMU_MVE_LDST_UNALIGNED_NONCONTIG_SPEC    0x0299          /* MVE unaligned noncontiguous load or store instruction speculatively executed */
#define PMU_MVE_VREDUCE_RETIRED                  0x02a0          /* MVE vector reduction instruction architecturally executed */
#define PMU_MVE_VREDUCE_SPEC                     0x02a1          /* MVE vector reduction instruction speculatively executed */
#define PMU_MVE_VREDUCE_FP_RETIRED               0x02a4          /* MVE floating-point vector reduction instruction architecturally executed */
#define PMU_MVE_VREDUCE_FP_SPEC                  0x02a5          /* MVE floating-point vector reduction instruction speculatively executed */
#define PMU_MVE_VREDUCE_INT_RETIRED              0x02a8          /* MVE integer vector reduction instruction architecturally executed */
#define PMU_MVE_VREDUCE_INT_SPEC                 0x02a9          /* MVE integer vector reduction instruction speculatively executed */
#define PMU_MVE_PRED                             0x02b8          /* Cycles where one or more predicated beats architecturally executed */
#define PMU_MVE_STALL                            0x02cc          /* Stall cycles caused by an MVE instruction */
#define PMU_MVE_STALL_RESOURCE                   0x02cd          /* Stall cycles caused by an MVE instruction because of resource conflicts */
#define PMU_MVE_STALL_RESOURCE_MEM               0x02ce          /* Stall cycles caused by an MVE instruction because of memory resource conflicts */
#define PMU_MVE_STALL_RESOURCE_FP                0x02cf          /* Stall cycles caused by an MVE instruction because of floating-point resource conflicts */
#define PMU_MVE_STALL_RESOURCE_INT               0x02d0          /* Stall cycles caused by an MVE instruction because of integer resource conflicts */
#define PMU_MVE_STALL_BREAK                      0x02d3          /* Stall cycles caused by an MVE chain break */
#define PMU_MVE_STALL_DEPENDENCY                 0x02d4          /* Stall cycles caused by MVE register dependency */
#define PMU_ITCM_ACCESS                          0x4007          /* Instruction TCM access */
#define PMU_DTCM_ACCESS                          0x4008          /* Data TCM access */
#define PMU_TRCEXTOUT0                           0x4010          /* ETM external output 0 */
#define PMU_TRCEXTOUT1                           0x4011          /* ETM external output 1 */
#define PMU_TRCEXTOUT2                           0x4012          /* ETM external output 2 */
#define PMU_TRCEXTOUT3                           0x4013          /* ETM external output 3 */
#define PMU_CTI_TRIGOUT4                         0x4018          /* Cross-trigger Interface output trigger 4 */
#define PMU_CTI_TRIGOUT5                         0x4019          /* Cross-trigger Interface output trigger 5 */
#define PMU_CTI_TRIGOUT6                         0x401a          /* Cross-trigger Interface output trigger 6 */
#define PMU_CTI_TRIGOUT7                         0x401b          /* Cross-trigger Interface output trigger 7 */

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/* PMU Functions */

/****************************************************************************
 * Name: pmu_enable
 *
 * Description:
 *   Enable the PMU.
 *
 ****************************************************************************/

inline void pmu_enable(void)
{
  modreg32(PMU_CTRL_ENABLE_MASK, PMU_CTRL_ENABLE_MASK, PMU_CTRL);
}

/****************************************************************************
 * Name: pmu_disable
 *
 * Description:
 *   Disable the PMU.
 *
 ****************************************************************************/

inline void pmu_disable(void)
{
  modreg32(PMU_CTRL_ENABLE_MASK, 0, PMU_CTRL);
}

/****************************************************************************
 * Name: pmu_set_evtyper
 *
 * Description:
 *   Set event to count for PMU eventer counter.
 *
 * Parameters:
 *   num - Event counter (0-30) to configure.
 *   type - Event to count.
 *
 ****************************************************************************/

inline void pmu_set_evtyper(uint32_t num, uint32_t type)
{
  putreg32(type, PMU_EVTYPER + 4 * num);
}

/****************************************************************************
 * Name: pmu_cyccnt_reset
 *
 * Description:
 *   Reset cycle counter.
 *
 ****************************************************************************/

inline void pmu_cyccnt_reset(void)
{
  modreg32(PMU_CTRL_CYCCNT_RESET_MASK, PMU_CTRL_CYCCNT_RESET_MASK, PMU_CTRL);
}

/****************************************************************************
 * Name: pmu_evcntr_reset_all
 *
 * Description:
 *   Reset all event counters.
 *
 ****************************************************************************/

inline void pmu_evcntr_reset_all(void)
{
  modreg32(PMU_CTRL_EVENTCNT_RESET_MASK, PMU_CTRL_EVENTCNT_RESET_MASK,
           PMU_CTRL);
}

/****************************************************************************
 * Name: pmu_cntr_enable
 *
 * Description:
 *   Enable counters.
 *
 * Parameters:
 *   mask - Counters to enable.
 *
 * Note:
 *   Enables one or more of the following:
 *     event counters (0-30)
 *     cycle counter
 *
 ****************************************************************************/

inline void pmu_cntr_enable(uint32_t mask)
{
  putreg32(mask, PMU_CNTENSET);
}

/****************************************************************************
 * Name: pmu_cntr_disable
 *
 * Description:
 *   Disable counters.
 *
 * Parameters:
 *   mask - Counters to disable.
 *
 * Note:
 *   Disables one or more of the following:
 *     event counters (0-30)
 *     cycle counter
 *
 ****************************************************************************/

inline void pmu_cntr_disable(uint32_t mask)
{
  putreg32(mask, PMU_CNTENCLR);
}

/****************************************************************************
 * Name: pmu_get_ccntr
 *
 * Description:
 *   Read cycle counter.
 *
 * Return Value:
 *   Cycle count.
 *
 ****************************************************************************/

inline uint32_t pmu_get_ccntr(void)
{
  return getreg32(PMU_CCNTR);
}

/****************************************************************************
 * Name: pmu_get_evcntr
 *
 * Description:
 *   Read event counter.
 *
 * Parameters:
 *   num - Event counter (0-30) to read.
 *
 * Return Value:
 *   Event count.
 *
 ****************************************************************************/

inline uint32_t pmu_get_evcntr(uint32_t num)
{
  return getreg32(PMU_EVCNTR + 4 * num) & PMU_EVCNTR_CNT_MASK;
}

/****************************************************************************
 * Name: pmu_get_cntr_ovs
 *
 * Description:
 *   Read counter overflow status.
 *
 * Return Value:
 *   Counter overflow status bits for the following:
 *     event counters (0-30)
 *     cycle counter
 *
 ****************************************************************************/

inline uint32_t pmu_get_cntr_ovs(void)
{
  return getreg32(PMU_OVSSET);
}

/****************************************************************************
 * Name: pmu_clear_cntr_ovs
 *
 * Description:
 *   Clear counter overflow status.
 *
 * Parameters:
 *   mask - Counter overflow status bits to clear.
 *
 * Note:
 *   Counter overflow status bits for the following:
 *     event counters (0-30)
 *     cycle counter
 *
 ****************************************************************************/

inline void pmu_clear_cntr_ovs(uint32_t mask)
{
  putreg32(mask, PMU_OVSCLR);
}

/****************************************************************************
 * Name: pmu_cntr_irq_enable
 *
 * Description:
 *   Enable counter overflow interrupt request.
 *
 * Parameters:
 *   mask - Counter overflow interrupt request bits to set.
 *
 * Note:
 *   Sets overflow interrupt request bits for one or more of the following:
 *     event counters (0-30)
 *     cycle counter
 *
 ****************************************************************************/

inline void pmu_cntr_irq_enable(uint32_t mask)
{
  putreg32(mask, PMU_INTENSET);
}

/****************************************************************************
 * Name: pmu_cntr_irq_disable
 *
 * Description:
 *   Disable counter overflow interrupt request.
 *
 * Parameters:
 *   mask - Counter overflow interrupt request bits to clear.
 *
 * Note:
 *   Clears overflow interrupt request bits for one or more of the following:
 *     event counters (0-30)
 *     cycle counter
 *
 ****************************************************************************/

inline void pmu_cntr_irq_disable(uint32_t mask)
{
  putreg32(mask, PMU_INTENCLR);
}

/****************************************************************************
 * Name: pmu_cntr_increment
 *
 * Description:
 *   Software increment event counter.
 *
 * Parameters:
 *   mask - Counters to increment.
 *
 * Note:
 *   Software increment bits for one or more event counters (0-30)
 *
 ****************************************************************************/

inline void pmu_cntr_increment(uint32_t mask)
{
  putreg32(mask, PMU_SWINC);
}

#endif
