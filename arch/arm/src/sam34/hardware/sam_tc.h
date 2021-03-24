/****************************************************************************
 * arch/arm/src/sam34/hardware/sam_tc.h
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

#ifndef __ARCH_ARM_SRC_SAM34_HARDWARE_SAM_TC_H
#define __ARCH_ARM_SRC_SAM34_HARDWARE_SAM_TC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "hardware/sam_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* TC register offsets ******************************************************/

/* Timer channel offsets
 *(with respect to timer base offset at 0x00, 0x40, and 0x80
 */

#define SAM_TC_CCR_OFFSET            0x0000 /* Channel Control Register */
#define SAM_TC_CMR_OFFSET            0x0004 /* Channel Mode Register */

#if defined(CONFIG_ARCH_CHIP_SAM4S) || defined(CONFIG_ARCH_CHIP_SAM4E)
#  define SAM_TC_SMMR_OFFSET         0x0008 /* Stepper Motor Mode Register */
#endif

#if defined(CONFIG_ARCH_CHIP_SAM4E)
#  define SAM_TC_RAB_OFFSET          0x000c /* Register AB */
#endif
                                            /* 0x0c Reserved */
#define SAM_TC_CV_OFFSET             0x0010 /* Counter Value */
#define SAM_TC_RA_OFFSET             0x0014 /* Register A */
#define SAM_TC_RB_OFFSET             0x0018 /* Register B */
#define SAM_TC_RC_OFFSET             0x001c /* Register C */
#define SAM_TC_SR_OFFSET             0x0020 /* Status Register */
#define SAM_TC_IER_OFFSET            0x0024 /* Interrupt Enable Register */
#define SAM_TC_IDR_OFFSET            0x0028 /* Interrupt Disable Register */
#define SAM_TC_IMR_OFFSET            0x002c /* Interrupt Mask Register */

#if defined(CONFIG_ARCH_CHIP_SAM4E)
#  define SAM_TC_EMR_OFFSET          0x0030 /* Extended Mode Register */
#endif

/* Timer common registers */

#define SAM_TC_BCR_OFFSET            0x00c0 /* Block Control Register */
#define SAM_TC_BMR_OFFSET            0x00c4 /* Block Mode Register */
#define SAM_TC_QIER_OFFSET           0x00c8 /* QDEC Interrupt Enable Register */
#define SAM_TC_QIDR_OFFSET           0x00cc /* QDEC Interrupt Disable Register */
#define SAM_TC_QIMR_OFFSET           0x00d0 /* QDEC Interrupt Mask Register */
#define SAM_TC_QISR_OFFSET           0x00d4 /* QDEC Interrupt Status Register */

#if defined(CONFIG_ARCH_CHIP_SAM4S) || defined(CONFIG_ARCH_CHIP_SAM4E)
#  define SAM_TC_FMR_OFFSET          0xd8 /* Fault Mode Register */
#  define SAM_TC_WPMR_OFFSET         0xe4 /* Write Protect Mode Register */
#endif

/* TC register addresses ****************************************************/

#define SAM_TC0_CCR                  (SAM_TC0_BASE+SAM_TC_CCR_OFFSET)
#define SAM_TC0_CMR                  (SAM_TC0_BASE+SAM_TC_CMR_OFFSET)
#if defined(CONFIG_ARCH_CHIP_SAM4S) || defined(CONFIG_ARCH_CHIP_SAM4E)
#  define SAM_TC0_SMMR               (SAM_TC0_BASE+SAM_TC_SMMR_OFFSET)
#endif
#if defined(CONFIG_ARCH_CHIP_SAM4E)
#  define SAM_TC0_RAB                (SAM_TC0_BASE+SAM_TC_RAB_OFFSET)
#endif
#define SAM_TC0_CV                   (SAM_TC0_BASE+SAM_TC_CV_OFFSET)
#define SAM_TC0_RA                   (SAM_TC0_BASE+SAM_TC_RA_OFFSET)
#define SAM_TC0_RB                   (SAM_TC0_BASE+SAM_TC_RB_OFFSET)
#define SAM_TC0_RC                   (SAM_TC0_BASE+SAM_TC_RC_OFFSET)
#define SAM_TC0_SR                   (SAM_TC0_BASE+SAM_TC_SR_OFFSET)
#define SAM_TC0_IER                  (SAM_TC0_BASE+SAM_TC_IER_OFFSET)
#define SAM_TC0_IDR                  (SAM_TC0_BASE+SAM_TC_IDR_OFFSET)
#define SAM_TC0_IMR                  (SAM_TC0_BASE+SAM_TC_IMR_OFFSET)
#if defined(CONFIG_ARCH_CHIP_SAM4E)
#  define SAM_TC0_EMR                (SAM_TC0_BASE+SAM_TC_EMR_OFFSET)
#endif

#define SAM_TC1_CCR                  (SAM_TC1_BASE+SAM_TC_CCR_OFFSET)
#define SAM_TC1_CMR                  (SAM_TC1_BASE+SAM_TC_CMR_OFFSET)
#if defined(CONFIG_ARCH_CHIP_SAM4S) || defined(CONFIG_ARCH_CHIP_SAM4E)
#  define SAM_TC1_SMMR               (SAM_TC1_BASE+SAM_TC_SMMR_OFFSET)
#endif
#if defined(CONFIG_ARCH_CHIP_SAM4E)
#  define SAM_TC1_RAB                (SAM_TC1_BASE+SAM_TC_RAB_OFFSET)
#endif
#define SAM_TC1_CV                   (SAM_TC1_BASE+SAM_TC_CV_OFFSET)
#define SAM_TC1_RA                   (SAM_TC1_BASE+SAM_TC_RA_OFFSET)
#define SAM_TC1_RB                   (SAM_TC1_BASE+SAM_TC_RB_OFFSET)
#define SAM_TC1_RC                   (SAM_TC1_BASE+SAM_TC_RC_OFFSET)
#define SAM_TC1_SR                   (SAM_TC1_BASE+SAM_TC_SR_OFFSET)
#define SAM_TC1_IER                  (SAM_TC1_BASE+SAM_TC_IER_OFFSET)
#define SAM_TC1_IDR                  (SAM_TC1_BASE+SAM_TC_IDR_OFFSET)
#define SAM_TC1_IMR                  (SAM_TC1_BASE+SAM_TC_IMR_OFFSET)
#if defined(CONFIG_ARCH_CHIP_SAM4E)
#  define SAM_TC1_EMR                (SAM_TC1_BASE+SAM_TC_EMR_OFFSET)
#endif

#define SAM_TC2_CCR                  (SAM_TC2_BASE+SAM_TC_CCR_OFFSET)
#define SAM_TC2_CMR                  (SAM_TC2_BASE+SAM_TC_CMR_OFFSET)
#if defined(CONFIG_ARCH_CHIP_SAM4S) || defined(CONFIG_ARCH_CHIP_SAM4E)
#  define SAM_TC2_SMMR               (SAM_TC2_BASE+SAM_TC_SMMR_OFFSET)
#endif
#if defined(CONFIG_ARCH_CHIP_SAM4E)
#  define SAM_TC2_RAB                (SAM_TC2_BASE+SAM_TC_RAB_OFFSET)
#endif
#define SAM_TC2_CV                   (SAM_TC2_BASE+SAM_TC_CV_OFFSET)
#define SAM_TC2_RA                   (SAM_TC2_BASE+SAM_TC_RA_OFFSET)
#define SAM_TC2_RB                   (SAM_TC2_BASE+SAM_TC_RB_OFFSET)
#define SAM_TC2_RC                   (SAM_TC2_BASE+SAM_TC_RC_OFFSET)
#define SAM_TC2_SR                   (SAM_TC2_BASE+SAM_TC_SR_OFFSET)
#define SAM_TC2_IER                  (SAM_TC2_BASE+SAM_TC_IER_OFFSET)
#define SAM_TC2_IDR                  (SAM_TC2_BASE+SAM_TC_IDR_OFFSET)
#define SAM_TC2_IMR                  (SAM_TC2_BASE+SAM_TC_IMR_OFFSET)
#if defined(CONFIG_ARCH_CHIP_SAM4E)
#  define SAM_TC2_EMR                (SAM_TC2_BASE+SAM_TC_EMR_OFFSET)
#endif

#define SAM_TC3_CCR                  (SAM_TC3_BASE+SAM_TC_CCR_OFFSET)
#define SAM_TC3_CMR                  (SAM_TC3_BASE+SAM_TC_CMR_OFFSET)
#if defined(CONFIG_ARCH_CHIP_SAM4S) || defined(CONFIG_ARCH_CHIP_SAM4E)
#  define SAM_TC3_SMMR               (SAM_TC3_BASE+SAM_TC_SMMR_OFFSET)
#endif
#if defined(CONFIG_ARCH_CHIP_SAM4E)
#  define SAM_TC3_RAB                (SAM_TC3_BASE+SAM_TC_RAB_OFFSET)
#endif
#define SAM_TC3_CV                   (SAM_TC3_BASE+SAM_TC_CV_OFFSET)
#define SAM_TC3_RA                   (SAM_TC3_BASE+SAM_TC_RA_OFFSET)
#define SAM_TC3_RB                   (SAM_TC3_BASE+SAM_TC_RB_OFFSET)
#define SAM_TC3_RC                   (SAM_TC3_BASE+SAM_TC_RC_OFFSET)
#define SAM_TC3_SR                   (SAM_TC3_BASE+SAM_TC_SR_OFFSET)
#define SAM_TC3_IER                  (SAM_TC3_BASE+SAM_TC_IER_OFFSET)
#define SAM_TC3_IDR                  (SAM_TC3_BASE+SAM_TC_IDR_OFFSET)
#define SAM_TC3_IMR                  (SAM_TC3_BASE+SAM_TC_IMR_OFFSET)
#if defined(CONFIG_ARCH_CHIP_SAM4E)
#  define SAM_TC3_EMR                (SAM_TC3_BASE+SAM_TC_EMR_OFFSET)
#endif

#define SAM_TC4_CCR                  (SAM_TC4_BASE+SAM_TC_CCR_OFFSET)
#define SAM_TC4_CMR                  (SAM_TC4_BASE+SAM_TC_CMR_OFFSET)
#if defined(CONFIG_ARCH_CHIP_SAM4S) || defined(CONFIG_ARCH_CHIP_SAM4E)
#  define SAM_TC4_SMMR               (SAM_TC4_BASE+SAM_TC_SMMR_OFFSET)
#endif
#if defined(CONFIG_ARCH_CHIP_SAM4E)
#  define SAM_TC4_RAB                (SAM_TC4_BASE+SAM_TC_RAB_OFFSET)
#endif
#define SAM_TC4_CV                   (SAM_TC4_BASE+SAM_TC_CV_OFFSET)
#define SAM_TC4_RA                   (SAM_TC4_BASE+SAM_TC_RA_OFFSET)
#define SAM_TC4_RB                   (SAM_TC4_BASE+SAM_TC_RB_OFFSET)
#define SAM_TC4_RC                   (SAM_TC4_BASE+SAM_TC_RC_OFFSET)
#define SAM_TC4_SR                   (SAM_TC4_BASE+SAM_TC_SR_OFFSET)
#define SAM_TC4_IER                  (SAM_TC4_BASE+SAM_TC_IER_OFFSET)
#define SAM_TC4_IDR                  (SAM_TC4_BASE+SAM_TC_IDR_OFFSET)
#define SAM_TC4_IMR                  (SAM_TC4_BASE+SAM_TC_IMR_OFFSET)
#if defined(CONFIG_ARCH_CHIP_SAM4E)
#  define SAM_TC4_EMR                (SAM_TC4_BASE+SAM_TC_EMR_OFFSET)
#endif

#define SAM_TC5_CCR                  (SAM_TC5_BASE+SAM_TC_CCR_OFFSET)
#define SAM_TC5_CMR                  (SAM_TC5_BASE+SAM_TC_CMR_OFFSET)
#if defined(CONFIG_ARCH_CHIP_SAM4S) || defined(CONFIG_ARCH_CHIP_SAM4E)
#  define SAM_TC5_SMMR               (SAM_TC5_BASE+SAM_TC_SMMR_OFFSET)
#endif
#if defined(CONFIG_ARCH_CHIP_SAM4E)
#  define SAM_TC5_RAB                (SAM_TC5_BASE+SAM_TC_RAB_OFFSET)
#endif
#define SAM_TC5_CV                   (SAM_TC5_BASE+SAM_TC_CV_OFFSET)
#define SAM_TC5_RA                   (SAM_TC5_BASE+SAM_TC_RA_OFFSET)
#define SAM_TC5_RB                   (SAM_TC5_BASE+SAM_TC_RB_OFFSET)
#define SAM_TC5_RC                   (SAM_TC5_BASE+SAM_TC_RC_OFFSET)
#define SAM_TC5_SR                   (SAM_TC5_BASE+SAM_TC_SR_OFFSET)
#define SAM_TC5_IER                  (SAM_TC5_BASE+SAM_TC_IER_OFFSET)
#define SAM_TC5_IDR                  (SAM_TC5_BASE+SAM_TC_IDR_OFFSET)
#define SAM_TC5_IMR                  (SAM_TC5_BASE+SAM_TC_IMR_OFFSET)
#if defined(CONFIG_ARCH_CHIP_SAM4E)
#  define SAM_TC5_EMR                (SAM_TC5_BASE+SAM_TC_EMR_OFFSET)
#endif

#define SAM_TC6_CCR                  (SAM_TC6_BASE+SAM_TC_CCR_OFFSET)
#define SAM_TC6_CMR                  (SAM_TC6_BASE+SAM_TC_CMR_OFFSET)
#if defined(CONFIG_ARCH_CHIP_SAM4S) || defined(CONFIG_ARCH_CHIP_SAM4E)
#  define SAM_TC6_SMMR               (SAM_TC6_BASE+SAM_TC_SMMR_OFFSET)
#endif
#if defined(CONFIG_ARCH_CHIP_SAM4E)
#  define SAM_TC6_RAB                (SAM_TC6_BASE+SAM_TC_RAB_OFFSET)
#endif
#define SAM_TC6_CV                   (SAM_TC6_BASE+SAM_TC_CV_OFFSET)
#define SAM_TC6_RA                   (SAM_TC6_BASE+SAM_TC_RA_OFFSET)
#define SAM_TC6_RB                   (SAM_TC6_BASE+SAM_TC_RB_OFFSET)
#define SAM_TC6_RC                   (SAM_TC6_BASE+SAM_TC_RC_OFFSET)
#define SAM_TC6_SR                   (SAM_TC6_BASE+SAM_TC_SR_OFFSET)
#define SAM_TC6_IER                  (SAM_TC6_BASE+SAM_TC_IER_OFFSET)
#define SAM_TC6_IDR                  (SAM_TC6_BASE+SAM_TC_IDR_OFFSET)
#define SAM_TC6_IMR                  (SAM_TC6_BASE+SAM_TC_IMR_OFFSET)
#if defined(CONFIG_ARCH_CHIP_SAM4E)
#  define SAM_TC6_EMR                (SAM_TC6_BASE+SAM_TC_EMR_OFFSET)
#endif

#define SAM_TC7_CCR                  (SAM_TC7_BASE+SAM_TC_CCR_OFFSET)
#define SAM_TC7_CMR                  (SAM_TC7_BASE+SAM_TC_CMR_OFFSET)
#if defined(CONFIG_ARCH_CHIP_SAM4S) || defined(CONFIG_ARCH_CHIP_SAM4E)
#  define SAM_TC7_SMMR               (SAM_TC7_BASE+SAM_TC_SMMR_OFFSET)
#endif
#if defined(CONFIG_ARCH_CHIP_SAM4E)
#  define SAM_TC7_RAB                (SAM_TC7_BASE+SAM_TC_RAB_OFFSET)
#endif
#define SAM_TC7_CV                   (SAM_TC7_BASE+SAM_TC_CV_OFFSET)
#define SAM_TC7_RA                   (SAM_TC7_BASE+SAM_TC_RA_OFFSET)
#define SAM_TC7_RB                   (SAM_TC7_BASE+SAM_TC_RB_OFFSET)
#define SAM_TC7_RC                   (SAM_TC7_BASE+SAM_TC_RC_OFFSET)
#define SAM_TC7_SR                   (SAM_TC7_BASE+SAM_TC_SR_OFFSET)
#define SAM_TC7_IER                  (SAM_TC7_BASE+SAM_TC_IER_OFFSET)
#define SAM_TC7_IDR                  (SAM_TC7_BASE+SAM_TC_IDR_OFFSET)
#define SAM_TC7_IMR                  (SAM_TC7_BASE+SAM_TC_IMR_OFFSET)
#if defined(CONFIG_ARCH_CHIP_SAM4E)
#  define SAM_TC7_EMR                (SAM_TC7_BASE+SAM_TC_EMR_OFFSET)
#endif

#define SAM_TC8_CCR                  (SAM_TC8_BASE+SAM_TC_CCR_OFFSET)
#define SAM_TC8_CMR                  (SAM_TC8_BASE+SAM_TC_CMR_OFFSET)
#if defined(CONFIG_ARCH_CHIP_SAM4S) || defined(CONFIG_ARCH_CHIP_SAM4E)
#  define SAM_TC8_SMMR               (SAM_TC8_BASE+SAM_TC_SMMR_OFFSET)
#endif
#if defined(CONFIG_ARCH_CHIP_SAM4E)
#  define SAM_TC8_RAB                (SAM_TC8_BASE+SAM_TC_RAB_OFFSET)
#endif
#define SAM_TC8_CV                   (SAM_TC8_BASE+SAM_TC_CV_OFFSET)
#define SAM_TC8_RA                   (SAM_TC8_BASE+SAM_TC_RA_OFFSET)
#define SAM_TC8_RB                   (SAM_TC8_BASE+SAM_TC_RB_OFFSET)
#define SAM_TC8_RC                   (SAM_TC8_BASE+SAM_TC_RC_OFFSET)
#define SAM_TC8_SR                   (SAM_TC8_BASE+SAM_TC_SR_OFFSET)
#define SAM_TC8_IER                  (SAM_TC8_BASE+SAM_TC_IER_OFFSET)
#define SAM_TC8_IDR                  (SAM_TC8_BASE+SAM_TC_IDR_OFFSET)
#define SAM_TC8_IMR                  (SAM_TC8_BASE+SAM_TC_IMR_OFFSET)
#if defined(CONFIG_ARCH_CHIP_SAM4E)
#  define SAM_TC8_EMR                (SAM_TC8_BASE+SAM_TC_EMR_OFFSET)
#endif

/* Timer common registers */

#if defined(CONFIG_ARCH_CHIP_SAM4S) || defined(CONFIG_ARCH_CHIP_SAM4E)
#  define SAM_TC0_BCR                (SAM_TC012_BASE+SAM_TC_BCR_OFFSET)
#  define SAM_TC0_BMR                (SAM_TC012_BASE+SAM_TC_BMR_OFFSET)
#  define SAM_TC0_QIER               (SAM_TC012_BASE+SAM_TC_QIER_OFFSET)
#  define SAM_TC0_QIDR               (SAM_TC012_BASE+SAM_TC_QIDR_OFFSET)
#  define SAM_TC0_QIMR               (SAM_TC012_BASE+SAM_TC_QIMR_OFFSET)
#  define SAM_TC0_QISR               (SAM_TC012_BASE+SAM_TC_QISR_OFFSET)
#  define SAM_TC0_FMR                (SAM_TC012_BASE+SAM_TC_FMR_OFFSET)
#  define SAM_TC0_WPMR               (SAM_TC012_BASE+SAM_TC_WPMR_OFFSET)

#  define SAM_TC1_BCR                (SAM_TC345_BASE+SAM_TC_BCR_OFFSET)
#  define SAM_TC1_BMR                (SAM_TC345_BASE+SAM_TC_BMR_OFFSET)
#  define SAM_TC1_QIER               (SAM_TC345_BASE+SAM_TC_QIER_OFFSET)
#  define SAM_TC1_QIDR               (SAM_TC345_BASE+SAM_TC_QIDR_OFFSET)
#  define SAM_TC1_QIMR               (SAM_TC345_BASE+SAM_TC_QIMR_OFFSET)
#  define SAM_TC1_QISR               (SAM_TC345_BASE+SAM_TC_QISR_OFFSET)
#  define SAM_TC1_FMR                (SAM_TC345_BASE+SAM_TC_FMR_OFFSET)
#  define SAM_TC1_WPMR               (SAM_TC345_BASE+SAM_TC_WPMR_OFFSET)

#  define SAM_TC2_BCR                (SAM_TC678_BASE+SAM_TC_BCR_OFFSET)
#  define SAM_TC2_BMR                (SAM_TC678_BASE+SAM_TC_BMR_OFFSET)
#  define SAM_TC2_QIER               (SAM_TC678_BASE+SAM_TC_QIER_OFFSET)
#  define SAM_TC2_QIDR               (SAM_TC678_BASE+SAM_TC_QIDR_OFFSET)
#  define SAM_TC2_QIMR               (SAM_TC678_BASE+SAM_TC_QIMR_OFFSET)
#  define SAM_TC2_QISR               (SAM_TC678_BASE+SAM_TC_QISR_OFFSET)
#  define SAM_TC2_FMR                (SAM_TC678_BASE+SAM_TC_FMR_OFFSET)
#  define SAM_TC2_WPMR               (SAM_TC678_BASE+SAM_TC_WPMR_OFFSET)
#else
#  define SAM_TC_BCR                 (SAM_TC_BASE+SAM_TC_BCR_OFFSET)
#  define SAM_TC_BMR                 (SAM_TC_BASE+SAM_TC_BMR_OFFSET)
#  define SAM_TC_QIER                (SAM_TC_BASE+SAM_TC_QIER_OFFSET)
#  define SAM_TC_QIDR                (SAM_TC_BASE+SAM_TC_QIDR_OFFSET)
#  define SAM_TC_QIMR                (SAM_TC_BASE+SAM_TC_QIMR_OFFSET)
#  define SAM_TC_QISR                (SAM_TC_BASE+SAM_TC_QISR_OFFSET)
#endif

/* TC register bit definitions **********************************************/

/* Timer channel registers **************************************************/

/* TC Channel Control Register */

#define TC_CCR_CLKEN                 (1 << 0)  /* Bit 0: Counter Clock Enable Command */
#define TC_CCR_CLKDIS                (1 << 1)  /* Bit 1: Counter Clock Disable Command */
#define TC_CCR_SWTRG                 (1 << 2)  /* Bit 2: Software Trigger Command */

/* TC Channel Mode Register -- Common */

#define TC_CMR_TCCLKS_SHIFT          (0)       /* Bits 0-2: Clock Selection */
#define TC_CMR_TCCLKS_MASK           (7 << TC_CMR_TCCLKS_SHIFT)
#  define TC_CMR_TCCLKS(n)           ((uint32_t)(n) << TC_CMR_TCCLKS_SHIFT)
#  define TC_CMR_TCCLKS_TIMERCLOCK1  (0 << TC_CMR_TCCLKS_SHIFT)
#  define TC_CMR_TCCLKS_TIMERCLOCK2  (1 << TC_CMR_TCCLKS_SHIFT)
#  define TC_CMR_TCCLKS_TIMERCLOCK3  (2 << TC_CMR_TCCLKS_SHIFT)
#  define TC_CMR_TCCLKS_TIMERCLOCK4  (3 << TC_CMR_TCCLKS_SHIFT)
#  define TC_CMR_TCCLKS_TIMERCLOCK5  (4 << TC_CMR_TCCLKS_SHIFT)
#  define TC_CMR_TCCLKS_XC0          (5 << TC_CMR_TCCLKS_SHIFT)
#  define TC_CMR_TCCLKS_XC1          (6 << TC_CMR_TCCLKS_SHIFT)
#  define TC_CMR_TCCLKS_XC2          (7 << TC_CMR_TCCLKS_SHIFT)
#define TC_CMR_CLKI                  (1 << 3)  /* Bit 3: Clock Invert */
#define TC_CMR_BURST_SHIFT           (4)       /* Bits 4-5: Burst Signal Selection */
#define TC_CMR_BURST_MASK            (3 << TC_CMR_BURST_SHIFT)
#  define TC_CMR_BURST_NOTGATED      (0 << TC_CMR_BURST_SHIFT) /* Not gated by external signal */
#  define TC_CMR_BURST_XC0           (1 << TC_CMR_BURST_SHIFT) /* XC0 ANDed with selected clock */
#  define TC_CMR_BURST_XC1           (2 << TC_CMR_BURST_SHIFT) /* XC1 ANDed with selected clock */
#  define TC_CMR_BURST_XC2           (3 << TC_CMR_BURST_SHIFT) /* XC2 ANDed with selected clock */

#define TC_CMR_WAVE                  (1 << 15) /* Bit 15: Waveform Mode */

/* TC Channel Mode Register -- Capture mode only */

#define TC_CMR_LDBSTOP               (1 << 6)  /* Bit 6: Counter stopped with RB Loading */
#define TC_CMR_LDBDIS                (1 << 7)  /* Bit 7: Counter disable with RB Loading */
#define TC_CMR_ETRGEDG_SHIFT         (8)       /* Bits 8-9: External Trigger Edge Selection */
#define TC_CMR_ETRGEDG_MASK          (3 << TC_CMR_ETRGEDG_SHIFT)
#  define TC_CMR_ETRGEDG_NONE        (0 << TC_CMR_ETRGEDG_SHIFT) /* None */
#  define TC_CMR_ETRGEDG_REDGE       (1 << TC_CMR_ETRGEDG_SHIFT) /* Rising edge */
#  define TC_CMR_ETRGEDG_FEDGE       (2 << TC_CMR_ETRGEDG_SHIFT) /* Falling edge */
#  define TC_CMR_ETRGEDG_EACH        (3 << TC_CMR_ETRGEDG_SHIFT) /* Each */

#define TC_CMR_ABETRG                (1 << 10) /* Bit 10: TIOA or TIOB External Trigger Selection */
#define TC_CMR_CPCTRG                (1 << 14) /* Bit 14: RC Compare Trigger Enable */
#define TC_CMR_LDRA_SHIFT            (16)      /* Bits 16-17: RA Loading Selection */
#define TC_CMR_LDRA_MASK             (3 << TC_CMR_LDRA_SHIFT)
#  define TC_CMR_LDRA_NONE           (0 << TC_CMR_LDRA_SHIFT) /* None */
#  define TC_CMR_LDRA_REDGE          (1 << TC_CMR_LDRA_SHIFT) /* Rising edge of TIOA */
#  define TC_CMR_LDRA_FEDGE          (2 << TC_CMR_LDRA_SHIFT) /* Falling edge of TIOA */
#  define TC_CMR_LDRA_EACH           (3 << TC_CMR_LDRA_SHIFT) /* Each  edge of TIOA */

#define TC_CMR_LDRB_SHIFT            (18)      /* Bits 18-19: RB Loading Selection */
#define TC_CMR_LDRB_MASK             (3 << TC_CMR_LDRB_SHIFT)
#  define TC_CMR_LDRB_NONE           (0 << TC_CMR_LDRB_SHIFT) /* None */
#  define TC_CMR_LDRB_REDGE          (1 << TC_CMR_LDRB_SHIFT) /* Rising edge of TIOB */
#  define TC_CMR_LDRB_FEDGE          (2 << TC_CMR_LDRB_SHIFT) /* Falling edge of TIOB */
#  define TC_CMR_LDRB_EACH           (3 << TC_CMR_LDRB_SHIFT) /* Each  edge of TIOB */

#if defined(CONFIG_ARCH_CHIP_SAM4E)
#define TC_CMR_SBSMPLR_SHIFT         (20)      /* Bits 20-22: Loading Edge Subsampling Ratio */
#define TC_CMR_SBSMPLR_MASK          (7 << TC_CMR_SBSMPLR_SHIFT)
#  define TC_CMR_SBSMPLR_ONE         (0 << TC_CMR_SBSMPLR_SHIFT) /* Load on each selected edge */
#  define TC_CMR_SBSMPLR_HALF        (1 << TC_CMR_SBSMPLR_SHIFT) /* Load on every 2 selected edges */
#  define TC_CMR_SBSMPLR_4TH         (2 << TC_CMR_SBSMPLR_SHIFT) /* Load on every 4 selected edges */
#  define TC_CMR_SBSMPLR_8TH         (3 << TC_CMR_SBSMPLR_SHIFT) /* Load on every 8 selected edges */
#  define TC_CMR_SBSMPLR_16TH        (4 << TC_CMR_SBSMPLR_SHIFT) /* Load on every 16 selected edges */
#endif

/* TC Channel Mode Register -- Waveform mode only */

#define TC_CMR_CPCSTOP               (1 << 6)  /* Bit 6: Counter Clock Stopped with RC Compare (Waveform mode) */
#define TC_CMR_CPCDIS                (1 << 7)  /* Bit 7: Counter Clock Disable with RC Compare (Waveform mode) */
#define TC_CMR_EEVTEDG_SHIFT         (8)       /* Bits 8-9: External Event Edge Selection (Waveform mode) */
#define TC_CMR_EEVTEDG_MASK          (3 << TC_CMR_EEVTEDG_SHIFT)
#  define TC_CMR_EEVTEDG_NONE        (0 << TC_CMR_EEVTEDG_SHIFT) /* None */
#  define TC_CMR_EEVTEDG_REDGE       (1 << TC_CMR_EEVTEDG_SHIFT) /* Rising edge */
#  define TC_CMR_EEVTEDG_FEDGE       (2 << TC_CMR_EEVTEDG_SHIFT) /* Falling edge */
#  define TC_CMR_EEVTEDG_EACH        (3 << TC_CMR_EEVTEDG_SHIFT) /* Each edge */

#define TC_CMR_EEVT_SHIFT            (10)      /* Bits 10-11: External Event Selection (Waveform mode) */
#define TC_CMR_EEVT_MASK             (3 << TC_CMR_EEVT_SHIFT)
#  define TC_CMR_EEVT_TIOB           (0 << TC_CMR_EEVT_SHIFT) /* TIOB input */
#  define TC_CMR_EEVT_XC0            (1 << TC_CMR_EEVT_SHIFT) /* XC0 output */
#  define TC_CMR_EEVT_XC1            (2 << TC_CMR_EEVT_SHIFT) /* XC1 output */
#  define TC_CMR_EEVT_XC2            (3 << TC_CMR_EEVT_SHIFT) /* XC2 output */

#define TC_CMR_ENETRG                (1 << 12) /* Bit 12: External Event Trigger Enable (Waveform mode) */
#define TC_CMR_WAVSEL_SHIFT          (13)      /* Bits 13-14: Waveform Selection (Waveform mode) */
#define TC_CMR_WAVSEL_MASK           (3 << TC_CMR_WAVSEL_SHIFT)
#  define TC_CMR_WAVSEL_UP           (0 << TC_CMR_WAVSEL_SHIFT) /* UP mode w/o auto trigger (Waveform mode) */
#  define TC_CMR_WAVSEL_UPDWN        (1 << TC_CMR_WAVSEL_SHIFT) /* UPDOWN mode w/o  auto trigger (Waveform mode) */
#  define TC_CMR_WAVSEL_UPAUTO       (2 << TC_CMR_WAVSEL_SHIFT) /* UP mode with auto trigger (Waveform mode) */
#  define TC_CMR_WAVSEL_UPDWNAUTO    (3 << TC_CMR_WAVSEL_SHIFT) /* UPDOWN mode with auto trigger (Waveform mode) */

#define TC_CMR_ACPA_SHIFT            (16)      /* Bits 16-17: RA Compare Effect on TIOA (Waveform mode) */
#define TC_CMR_ACPA_MASK             (3 << TC_CMR_ACPA_SHIFT)
#  define TC_CMR_ACPA_NONE           (0 << TC_CMR_ACPA_SHIFT)
#  define TC_CMR_ACPA_SET            (1 << TC_CMR_ACPA_SHIFT)
#  define TC_CMR_ACPA_CLEAR          (2 << TC_CMR_ACPA_SHIFT)
#  define TC_CMR_ACPA_TOGGLE         (3 << TC_CMR_ACPA_SHIFT)
#define TC_CMR_ACPC_SHIFT            (18)      /* Bits 18-19: RC Compare Effect on TIOA (Waveform mode) */
#define TC_CMR_ACPC_MASK             (3 << TC_CMR_ACPC_SHIFT)
#  define TC_CMR_ACPC_NONE           (0 << TC_CMR_ACPC_SHIFT)
#  define TC_CMR_ACPC_SET            (1 << TC_CMR_ACPC_SHIFT)
#  define TC_CMR_ACPC_CLEAR          (2 << TC_CMR_ACPC_SHIFT)
#  define TC_CMR_ACPC_TOGGLE         (3 << TC_CMR_ACPC_SHIFT)
#define TC_CMR_AEEVT_SHIFT           (20)      /* Bits 20-21: External Event Effect on TIOA (Waveform mode) */
#define TC_CMR_AEEVT_MASK            (3 << TC_CMR_AEEVT_SHIFT)
#  define TC_CMR_AEEVT_NONE          (0 << TC_CMR_AEEVT_SHIFT)
#  define TC_CMR_AEEVT_SET           (1 << TC_CMR_AEEVT_SHIFT)
#  define TC_CMR_AEEVT_CLEAR         (2 << TC_CMR_AEEVT_SHIFT)
#  define TC_CMR_AEEVT_TOGGLE        (3 << TC_CMR_AEEVT_SHIFT)
#define TC_CMR_ASWTRG_SHIFT          (22)      /* Bits 22-23: Software Trigger Effect on TIOA (Waveform mode) */
#define TC_CMR_ASWTRG_MASK           (3 << TC_CMR_ASWTRG_SHIFT)
#  define TC_CMR_ASWTRG_NONE         (0 << TC_CMR_ASWTRG_SHIFT)
#  define TC_CMR_ASWTRG_SET          (1 << TC_CMR_ASWTRG_SHIFT)
#  define TC_CMR_ASWTRG_CLEAR        (2 << TC_CMR_ASWTRG_SHIFT)
#  define TC_CMR_ASWTRG_TOGGLE       (3 << TC_CMR_ASWTRG_SHIFT)
#define TC_CMR_BCPB_SHIFT            (24)      /* Bits 24-25: RB Compare Effect on TIOB (Waveform mode) */
#define TC_CMR_BCPB_MASK             (3 << TC_CMR_BCPB_SHIFT)
#  define TC_CMR_BCPB_NONE           (0 << TC_CMR_BCPB_SHIFT)
#  define TC_CMR_BCPB_SET            (1 << TC_CMR_BCPB_SHIFT)
#  define TC_CMR_BCPB_CLEAR          (2 << TC_CMR_BCPB_SHIFT)
#  define TC_CMR_BCPB_TOGGLE         (3 << TC_CMR_BCPB_SHIFT)
#define TC_CMR_BCPC_SHIFT            (26)      /* Bits 26-27: RC Compare Effect on TIOB (Waveform mode) */
#define TC_CMR_BCPC_MASK             (3 << TC_CMR_BCPC_SHIFT)
#  define TC_CMR_BCPC_NONE           (0 << TC_CMR_BCPC_SHIFT)
#  define TC_CMR_BCPC_SET            (1 << TC_CMR_BCPC_SHIFT)
#  define TC_CMR_BCPC_CLEAR          (2 << TC_CMR_BCPC_SHIFT)
#  define TC_CMR_BCPC_TOGGLE         (3 << TC_CMR_BCPC_SHIFT)
#define TC_CMR_BEEVT_SHIFT           (28)      /* Bits 28-29: External Event Effect on TIOB (Waveform mode) */
#define TC_CMR_BEEVT_MASK            (3 << TC_CMR_BEEVT_SHIFT)
#  define TC_CMR_BEEVT_NONE          (0 << TC_CMR_BEEVT_SHIFT)
#  define TC_CMR_BEEVT_SET           (1 << TC_CMR_BEEVT_SHIFT)
#  define TC_CMR_BEEVT_CLEAR         (2 << TC_CMR_BEEVT_SHIFT)
#  define TC_CMR_BEEVT_TOGGLE        (3 << TC_CMR_BEEVT_SHIFT)
#define TC_CMR_BSWTRG_SHIFT          (30)      /* Bits 30-31: Software Trigger Effect on TIOB (Waveform mode) */
#define TC_CMR_BSWTRG_MASK           (3 << TC_CMR_BSWTRG_SHIFT)
#  define TC_CMR_BSWTRG_NONE         (0 << TC_CMR_BSWTRG_SHIFT)
#  define TC_CMR_BSWTRG_SET          (1 << TC_CMR_BSWTRG_SHIFT)
#  define TC_CMR_BSWTRG_CLEAR        (2 << TC_CMR_BSWTRG_SHIFT)
#  define TC_CMR_BSWTRG_TOGGLE       (3 << TC_CMR_BSWTRG_SHIFT)

/* Stepper Motor Mode Register */

#if defined(CONFIG_ARCH_CHIP_SAM4S) || defined(CONFIG_ARCH_CHIP_SAM4E)
#  define TC_SMMR_GCEN               (1 << 0)  /* Bit 0:  Gray Count Enable */
#  define TC_SMMR_DOWN               (1 << 1)  /* Bit 1:  DOWN Count */
#endif

/* Register AB -- 32-bit value */

#if defined(CONFIG_ARCH_CHIP_SAM4E)
#  define TC_RAB_MASK                (0xffffffff)
#endif

/* TC Counter Value Register */

#if defined(CONFIG_ARCH_CHIP_SAM4S) || defined(CONFIG_ARCH_CHIP_SAM4E)
#  define TC_CV_MASK                 (0xffffffff)
#else
#  define TC_CV_MASK                 (0x0000ffff)
#endif

/* TC Register A, B, C */

#if defined(CONFIG_ARCH_CHIP_SAM4S) || defined(CONFIG_ARCH_CHIP_SAM4E)
#  define TC_RVALUE_MASK             (0xffffffff)
#else
#  define TC_RVALUE_MASK             (0x0000ffff)
#endif

/* TC Status Register, TC Interrupt Enable Register,
 * TC Interrupt Disable Register, and
 * TC Interrupt Mask Register common bit-field definitions
 */

#define TC_INT_COVFS                 (1 << 0)  /* Bit 0:  Counter Overflow */
#define TC_INT_LOVRS                 (1 << 1)  /* Bit 1:  Load Overrun */
#define TC_INT_CPAS                  (1 << 2)  /* Bit 2:  RA Compare */
#define TC_INT_CPBS                  (1 << 3)  /* Bit 3:  RB Compare */
#define TC_INT_CPCS                  (1 << 4)  /* Bit 4:  RC Compare */
#define TC_INT_LDRAS                 (1 << 5)  /* Bit 5:  RA Loading */
#define TC_INT_LDRBS                 (1 << 6)  /* Bit 6:  RB Loading */
#define TC_INT_ETRGS                 (1 << 7)  /* Bit 7:  External Trigger */

#if defined(CONFIG_ARCH_CHIP_SAM4E)
#  define TC_INT_ENDRX               (1 << 8)  /* Bit 8: End of Receiver Transfer */
#  define TC_INT_RXBUFF              (1 << 9)  /* Bit 9: Reception Buffer Full */
#  define TC_INT_ALL                 (TC_INT_COVFS + TC_INT_LOVRS + TC_INT_CPAS + TC_INT_CPBS + TC_INT_CPCS + TC_INT_LDRAS + TC_INT_LDRBS + TC_INT_ETRGS + TC_INT_ENDRX + TC_INT_RXBUFF)
#else
#  define TC_INT_ALL                 (TC_INT_COVFS + TC_INT_LOVRS + TC_INT_CPAS + TC_INT_CPBS + TC_INT_CPCS + TC_INT_LDRAS + TC_INT_LDRBS + TC_INT_ETRGS)
#endif

#define TC_INT_CLKSTA                (1 << 16) /* Bit 16: Clock Enabling (SR only) */
#define TC_SR_MTIOA                  (1 << 17) /* Bit 17: TIOA Mirror (SR only) */
#define TC_SR_MTIOB                  (1 << 18) /* Bit 18: TIOB Mirror (SR only)*/

/* Extended Mode Register */

#if defined(CONFIG_ARCH_CHIP_SAM4E)
#  define TC_EMR_TRIGSRCA_SHIFT      (0)       /* Bits 0-1: Trigger source for input A */
#  define TC_EMR_TRIGSRCA_MASK       (3 << TC_EMR_TRIGSRCA_SHIFT)
#    define TC_EMR_TRIGSRCA_TIOA     (0 << TC_EMR_TRIGSRCA_SHIFT) /* Input A driven by pin TIOAx */
#    define TC_EMR_TRIGSRCA_PWM      (1 << TC_EMR_TRIGSRCA_SHIFT) /* Input A driven by PWMx */

#  define TC_EMR_TRIGSRCB_SHIFT      (5)       /* Bits 4-5: Trigger source for input B */
#  define TC_EMR_TRIGSRCB_MASK       (3 << TC_EMR_TRIGSRCB_SHIFT)
#    define TC_EMR_TRIGSRCB_TIOA     (0 << TC_EMR_TRIGSRCB_SHIFT) /* Input B driven by pin TIOBx */
#    define TC_EMR_TRIGSRCB_PWM      (1 << TC_EMR_TRIGSRCB_SHIFT) /* Input B driven by PWMx */

#  define TC_EMR_NODIVCLK            (1 << 8)  /* Bit 8:  NO DIVided CLocK */
#endif

/* Timer common registers ***************************************************/

/* TC Block Control Register */

#define TC_BCR_SYNC                  (1 << 0)  /* Bit 0: Synchro Command */

/* TC Block Mode Register */

#define TC_BMR_TC0XC0S_SHIFT         (0)       /* Bits 0-1: External Clock Signal 0 Selection */
#define TC_BMR_TC0XC0S_MASK          (3 << TC_BMR_TC0XC0S_SHIFT)
#  define TC_BMR_TC0XC0S_TCLK0       (0 << TC_BMR_TC0XC0S_SHIFT)
#  define TC_BMR_TC0XC0S_NONE        (1 << TC_BMR_TC0XC0S_SHIFT)
#  define TC_BMR_TC0XC0S_TIOA1       (2 << TC_BMR_TC0XC0S_SHIFT)
#  define TC_BMR_TC0XC0S_TIOA2       (3 << TC_BMR_TC0XC0S_SHIFT)
#define TC_BMR_TC1XC1S_SHIFT         (2)       /* Bits 2-3: External Clock Signal 1 Selection */
#define TC_BMR_TC1XC1S_MASK          (3 << TC_BMR_TC1XC1S_MASK)
#  define TC_BMR_TC1XC1S_TCLK1       (0 << TC_BMR_TC1XC1S_SHIFT)
#  define TC_BMR_TC1XC1S_NONE        (1 << TC_BMR_TC1XC1S_SHIFT)
#  define TC_BMR_TC1XC1S_TIOA0       (2 << TC_BMR_TC1XC1S_SHIFT)
#  define TC_BMR_TC1XC1S_TIOA2       (3 << TC_BMR_TC1XC1S_SHIFT)
#define TC_BMR_TC2XC2S_SHIFT         (4)       /* Bits 4-5: External Clock Signal 2 Selection */
#define TC_BMR_TC2XC2S_MASK          (3 << TC_BMR_TC2XC2S_SHIFT)
#  define TC_BMR_TC2XC2S_TCLK2       (0 << TC_BMR_TC2XC2S_SHIFT)
#  define TC_BMR_TC2XC2S_NONE        (1 << TC_BMR_TC2XC2S_SHIFT)
#  define TC_BMR_TC2XC2S_TIOA0       (2 << TC_BMR_TC2XC2S_SHIFT)
#  define TC_BMR_TC2XC2S_TIOA1       (3 << TC_BMR_TC2XC2S_SHIFT)
#define TC_BMR_QDEN                  (1 << 8)  /* Bit 8:  Quadrature Decoder Enabled */
#define TC_BMR_POSEN                 (1 << 9)  /* Bit 9:  Position Enabled */
#define TC_BMR_SPEEDEN               (1 << 10) /* Bit 10: Speed Enabled */
#define TC_BMR_QDTRANS               (1 << 11) /* Bit 11: Quadrature Decoding Transparent */
#define TC_BMR_EDGPHA                (1 << 12) /* Bit 12: Edge on PHA count mode */
#define TC_BMR_INVA                  (1 << 13) /* Bit 13: Inverted PHA */
#define TC_BMR_INVB                  (1 << 14) /* Bit 14: Inverted PHB */
#define TC_BMR_INVIDX                (1 << 15) /* Bit 15: Inverted Index */
#define TC_BMR_SWAP                  (1 << 16) /* Bit 16: Swap PHA and PHB */
#define TC_BMR_IDXPHB                (1 << 17) /* Bit 17: Index pin is PHB pin */
#define TC_BMR_FILTER                (1 << 19) /* Bit 19 */
#define TC_BMR_MAXFILT_SHIFT         (20)      /* Bits 20-25: Maximum Filter */
#define TC_BMR_MAXFILT_MASK          (63 << TC_BMR_MAXFILT_SHIFT)
#  define TC_BMR_MAXFILT(n)          ((uint32_t)(n) << TC_BMR_MAXFILT_SHIFT)

/* TC QDEC Interrupt Enable Register, TC QDEC Interrupt Disable Register,
 * TC QDEC Interrupt Mask Register, TC QDEC Interrupt Status Register common
 * bit field definitions
 */

#define TC_QINT_IDX                  (1 << 0)  /* Bit 0: Index (Common) */
#define TC_QINT_DIRCHG               (1 << 1)  /* Bit 1: Direction Change (Common) */
#define TC_QINT_QERR                 (1 << 2)  /* Bit 2: Quadrature Error (Common) */
#define TC_QISR_DIR                  (1 << 8)  /* Bit 8: Direction (QISR only) */

/* Fault Mode Register */

#if defined(CONFIG_ARCH_CHIP_SAM4S) || defined(CONFIG_ARCH_CHIP_SAM4E)
#  define TC_FMR_ENCF0               (1 << 0)  /* Bit 0:  Enable compare fault channel 0 */
#  define TC_FMR_ENCF1               (1 << 1)  /* Bit 1:  Enable compare fault channel 1 */
#endif

/* Write Protect Mode Register */

#if defined(CONFIG_ARCH_CHIP_SAM4S) || defined(CONFIG_ARCH_CHIP_SAM4E)
#  define TC_WPMR_WPEN               (1 << 0)  /* Bit 0:  Write Protect Enable */
#  define TC_WPMR_WPKEY_SHIFT        (8)       /* Bits 8-31: Write Protect KEY */
#  define TC_WPMR_WPKEY_MASK         (0x00ffffff << TC_WPMR_WPKEY_SHIFT)
#    define TC_WPMR_WPKEY            (0x0054494d << TC_WPMR_WPKEY_SHIFT)
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_SAM34_HARDWARE_SAM_TC_H */
