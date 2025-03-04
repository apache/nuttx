/****************************************************************************
 * arch/arm/src/rp23xx/hardware/rp23xx_trng.h
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

#ifndef __ARCH_ARM_SRC_RP23XX_HARDWARE_RP23XX_TRNG_H
#define __ARCH_ARM_SRC_RP23XX_HARDWARE_RP23XX_TRNG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/rp23xx_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define RP23XX_TRNG_RNG_IMR_OFFSET              0x00000100
#define RP23XX_TRNG_RNG_ISR_OFFSET              0x00000104
#define RP23XX_TRNG_RNG_ICR_OFFSET              0x00000108
#define RP23XX_TRNG_TRNG_CONFIG_OFFSET          0x0000010c
#define RP23XX_TRNG_TRNG_VALID_OFFSET           0x00000110
#define RP23XX_TRNG_EHR_DATA_OFFSET(n)          (0x000114 + (n) * 4)
#define RP23XX_TRNG_RND_SOURCE_ENABLE_OFFSET    0x0000012c
#define RP23XX_TRNG_SAMPLE_CNT1_OFFSET          0x00000130
#define RP23XX_TRNG_AUTOCORR_STATISTIC_OFFSET   0x00000134
#define RP23XX_TRNG_TRNG_DEBUG_CONTROL_OFFSET   0x00000138
#define RP23XX_TRNG_TRNG_SW_RESET_OFFSET        0x00000140
#define RP23XX_TRNG_RNG_DEBUG_EN_INPUT_OFFSET   0x000001b4
#define RP23XX_TRNG_TRNG_BUSY_OFFSET            0x000001b8
#define RP23XX_TRNG_RST_BITS_COUNTER_OFFSET     0x000001bc
#define RP23XX_TRNG_RNG_VERSION_OFFSET          0x000001c0
#define RP23XX_TRNG_RNG_BIST_CNTR_0_OFFSET      0x000001e0
#define RP23XX_TRNG_RNG_BIST_CNTR_1_OFFSET      0x000001e4
#define RP23XX_TRNG_RNG_BIST_CNTR_2_OFFSET      0x000001e8

/* Register definitions *****************************************************/

#define RP23XX_TRNG_RNG_IMR             (RP23XX_TRNG_BASE + RP23XX_TRNG_RNG_IMR_OFFSET)
#define RP23XX_TRNG_RNG_ISR             (RP23XX_TRNG_BASE + RP23XX_TRNG_RNG_ISR_OFFSET)
#define RP23XX_TRNG_RNG_ICR             (RP23XX_TRNG_BASE + RP23XX_TRNG_RNG_ICR_OFFSET)
#define RP23XX_TRNG_TRNG_CONFIG         (RP23XX_TRNG_BASE + RP23XX_TRNG_TRNG_CONFIG_OFFSET)
#define RP23XX_TRNG_TRNG_VALID          (RP23XX_TRNG_BASE + RP23XX_TRNG_TRNG_VALID_OFFSET)
#define RP23XX_TRNG_EHR_DATA(n)         (RP23XX_TRNG_BASE + RP23XX_TRNG_EHR_DATA_OFFSET(n))
#define RP23XX_TRNG_RND_SOURCE_ENABLE   (RP23XX_TRNG_BASE + RP23XX_TRNG_RND_SOURCE_ENABLE_OFFSET)
#define RP23XX_TRNG_SAMPLE_CNT1         (RP23XX_TRNG_BASE + RP23XX_TRNG_SAMPLE_CNT1_OFFSET)
#define RP23XX_TRNG_AUTOCORR_STATISTIC  (RP23XX_TRNG_BASE + RP23XX_TRNG_AUTOCORR_STATISTIC_OFFSET)
#define RP23XX_TRNG_TRNG_DEBUG_CONTROL  (RP23XX_TRNG_BASE + RP23XX_TRNG_TRNG_DEBUG_CONTROL_OFFSET)
#define RP23XX_TRNG_TRNG_SW_RESET       (RP23XX_TRNG_BASE + RP23XX_TRNG_TRNG_SW_RESET_OFFSET)
#define RP23XX_TRNG_RNG_DEBUG_EN_INPUT  (RP23XX_TRNG_BASE + RP23XX_TRNG_RNG_DEBUG_EN_INPUT_OFFSET)
#define RP23XX_TRNG_TRNG_BUSY           (RP23XX_TRNG_BASE + RP23XX_TRNG_TRNG_BUSY_OFFSET)
#define RP23XX_TRNG_RST_BITS_COUNTER    (RP23XX_TRNG_BASE + RP23XX_TRNG_RST_BITS_COUNTER_OFFSET)
#define RP23XX_TRNG_RNG_VERSION         (RP23XX_TRNG_BASE + RP23XX_TRNG_RNG_VERSION_OFFSET)
#define RP23XX_TRNG_RNG_BIST_CNTR_0     (RP23XX_TRNG_BASE + RP23XX_TRNG_RNG_BIST_CNTR_0_OFFSET)
#define RP23XX_TRNG_RNG_BIST_CNTR_1     (RP23XX_TRNG_BASE + RP23XX_TRNG_RNG_BIST_CNTR_1_OFFSET)
#define RP23XX_TRNG_RNG_BIST_CNTR_2     (RP23XX_TRNG_BASE + RP23XX_TRNG_RNG_BIST_CNTR_2_OFFSET)

/* Register bit definitions *************************************************/

#define RP23XX_TRNG_RNG_IMR_VN_ERR_INT_MASK                     (1 << 3)
#define RP23XX_TRNG_RNG_IMR_CRNGT_ERR_INT_MASK                  (1 << 2)
#define RP23XX_TRNG_RNG_IMR_AUTOCORR_ERR_INT_MASK               (1 << 1)
#define RP23XX_TRNG_RNG_IMR_EHR_VALID_INT_MASK                  (1 << 0)
#define RP23XX_TRNG_RNG_ISR_VN_ERR                              (1 << 3)
#define RP23XX_TRNG_RNG_ISR_CRNGT_ERR                           (1 << 2)
#define RP23XX_TRNG_RNG_ISR_AUTOCORR_ERR                        (1 << 1)
#define RP23XX_TRNG_RNG_ISR_EHR_VALID                           (1 << 0)
#define RP23XX_TRNG_RNG_ICR_VN_ERR                              (1 << 3)
#define RP23XX_TRNG_RNG_ICR_CRNGT_ERR                           (1 << 2)
#define RP23XX_TRNG_RNG_ICR_AUTOCORR_ERR                        (1 << 1)
#define RP23XX_TRNG_RNG_ICR_EHR_VALID                           (1 << 0)
#define RP23XX_TRNG_TRNG_CONFIG_RND_SRC_SEL_MASK                (0x00000003)
#define RP23XX_TRNG_TRNG_VALID_EHR_VALID                        (1 << 0)
#define RP23XX_TRNG_RND_SOURCE_ENABLE_RND_SRC_EN                (1 << 0)
#define RP23XX_TRNG_AUTOCORR_STATISTIC_AUTOCORR_FAILS_SHIFT     (14)
#define RP23XX_TRNG_AUTOCORR_STATISTIC_AUTOCORR_FAILS_MASK      (0xff << RP23XX_TRNG_AUTOCORR_STATISTIC_AUTOCORR_FAILS_SHIFT)
#define RP23XX_TRNG_AUTOCORR_STATISTIC_AUTOCORR_TRYS_MASK       (0x00003fff)
#define RP23XX_TRNG_TRNG_DEBUG_CONTROL_AUTO_CORRELATE_BYPASS    (1 << 3)
#define RP23XX_TRNG_TRNG_DEBUG_CONTROL_TRNG_CRNGT_BYPASS        (1 << 2)
#define RP23XX_TRNG_TRNG_DEBUG_CONTROL_VNC_BYPASS               (1 << 1)
#define RP23XX_TRNG_TRNG_DEBUG_CONTROL_RESERVED                 (1 << 0)
#define RP23XX_TRNG_TRNG_SW_RESET_TRNG_SW_RESET                 (1 << 0)
#define RP23XX_TRNG_RNG_DEBUG_EN_INPUT_RNG_DEBUG_EN             (1 << 0)
#define RP23XX_TRNG_TRNG_BUSY_TRNG_BUSY                         (1 << 0)
#define RP23XX_TRNG_RST_BITS_COUNTER_RST_BITS_COUNTER           (1 << 0)
#define RP23XX_TRNG_RNG_VERSION_RNG_USE_5_SBOXES                (1 << 7)
#define RP23XX_TRNG_RNG_VERSION_RESEEDING_EXISTS                (1 << 6)
#define RP23XX_TRNG_RNG_VERSION_KAT_EXISTS                      (1 << 5)
#define RP23XX_TRNG_RNG_VERSION_PRNG_EXISTS                     (1 << 4)
#define RP23XX_TRNG_RNG_VERSION_TRNG_TESTS_BYPASS_EN            (1 << 3)
#define RP23XX_TRNG_RNG_VERSION_AUTOCORR_EXISTS                 (1 << 2)
#define RP23XX_TRNG_RNG_VERSION_CRNGT_EXISTS                    (1 << 1)
#define RP23XX_TRNG_RNG_VERSION_EHR_WIDTH_192                   (1 << 0)
#define RP23XX_TRNG_RNG_BIST_CNTR_0_ROSC_CNTR_VAL_MASK          (0x003fffff)
#define RP23XX_TRNG_RNG_BIST_CNTR_1_ROSC_CNTR_VAL_MASK          (0x003fffff)
#define RP23XX_TRNG_RNG_BIST_CNTR_2_ROSC_CNTR_VAL_MASK          (0x003fffff)

#endif /* __ARCH_ARM_SRC_RP23XX_HARDWARE_RP23XX_TRNG_H */
