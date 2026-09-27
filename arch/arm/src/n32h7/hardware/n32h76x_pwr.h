/****************************************************************************
 * arch/arm/src/n32h7/hardware/n32h76x_pwr.h
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

#ifndef __ARCH_ARM_SRC_N32H7_HARDWARE_N32H76X_PWR_H
#define __ARCH_ARM_SRC_N32H7_HARDWARE_N32H76X_PWR_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define N32_PWR_C1_PWR_CR_OFFSET        0x0000  /* Core1 power control register */
#define N32_PWR_C1_PWR_CSR_OFFSET       0x0004  /* Core1 power status register */
#define N32_PWR_C1_PWR_CR2_OFFSET       0x0008  /* Core1 power control register 2 */

#define N32_PWR_C2_PWR_CR_OFFSET        0x0020  /* Core2 power control register */
#define N32_PWR_C2_PWR_CSR_OFFSET       0x0024  /* Core2 power status register */
#define N32_PWR_C2_PWR_CR2_OFFSET       0x0028  /* Core2 power control register 2 */

#define N32_PWR_SYS_PWR_CR_OFFSET       0x0040  /* System power control register */
#define N32_PWR_SYS_PWR_CSR_OFFSET      0x0044  /* System power status register */
#define N32_PWR_SYS_PWR_CR2_OFFSET      0x0048  /* System power control register 2 */
#define N32_PWR_SYS_PWR_CR3_OFFSET      0x004C  /* System power control register 3 */
#define N32_PWR_SYS_PWR_CR4_OFFSET      0x0050  /* System power control register 4 */
#define N32_PWR_SYS_PWR_BOR_CR_OFFSET   0x0054  /* System brown-out reset control register */
#define N32_PWR_IP_MEMPWR_CR_OFFSET     0x0058  /* IP memory power control register */
#define N32_PWR_IP_MEMPWR_CSR_OFFSET    0x005C  /* IP memory power status register */

#define N32_PWR_C1_MEM_LPCR1_OFFSET     0x0060  /* Core1 memory low-power control register 1 */
#define N32_PWR_C1_MEM_LPSR1_OFFSET     0x0064  /* Core1 memory low-power status register 1 */
#define N32_PWR_C1_TCM_PG0_OFFSET       0x0068  /* Core1 TCM power gating register 0 */
#define N32_PWR_C1_TCM_PG1_OFFSET       0x006C  /* Core1 TCM power gating register 1 */
#define N32_PWR_C1_TCM_RET1N0_OFFSET    0x0070  /* Core1 TCM retention register 1N0 */
#define N32_PWR_C1_TCM_RET1N1_OFFSET    0x0074  /* Core1 TCM retention register 1N1 */
#define N32_PWR_C1_TCM_RET2N0_OFFSET    0x0078  /* Core1 TCM retention register 2N0 */
#define N32_PWR_C1_TCM_RET2N1_OFFSET    0x007C  /* Core1 TCM retention register 2N1 */

#define N32_PWR_C2_MEM_LPCR_OFFSET      0x0090  /* Core2 memory low-power control register */

#define N32_PWR_SYS_MEM_LPCR_OFFSET     0x00A0  /* System memory low-power control register */

#define N32_PWR_SYS_PWR_SHRT_CR_OFFSET  0x00B0  /* System power short control register */
#define N32_PWR_SYS_PWR_MDMA_CR_OFFSET  0x00B4  /* System power MDMA control register */
#define N32_PWR_SYS_PWR_ETHCAT_CR_OFFSET 0x00B8 /* System power EtherCAT control register */

#define N32_PWR_PWR_EMC_CR0_OFFSET      0x0100  /* EMC power control register 0 */
#define N32_PWR_PWR_EMC_CR1_OFFSET      0x0104  /* EMC power control register 1 */
#define N32_PWR_PWR_EMC_CR2_OFFSET      0x0108  /* EMC power control register 2 */
#define N32_PWR_PWR_EMC_CR3_OFFSET      0x010C  /* EMC power control register 3 */
#define N32_PWR_PWR_EMC_CR4_OFFSET      0x0110  /* EMC power control register 4 */
#define N32_PWR_PWR_EMC_CR5_OFFSET      0x0114  /* EMC power control register 5 */
#define N32_PWR_PWR_EMC_CR6_OFFSET      0x0118  /* EMC power control register 6 */
#define N32_PWR_PWR_EMC_CR7_OFFSET      0x011C  /* EMC power control register 7 */
#define N32_PWR_PWR_BKP_EMC_CR0_OFFSET  0x0120  /* Backup EMC power control register 0 */
#define N32_PWR_PWR_BKP_EMC_CR1_OFFSET  0x0124  /* Backup EMC power control register 1 */

/* Register Addresses *******************************************************/

/** POR BOR DCDC VSEL Control Register **/
#define PVD_Contrl                   (N32_AFEC_BASE + 0x2CU)
#define AVD_Contrl                   (N32_AFEC_BASE + 0x00U)
#define DCDC_Contrl                  (N32_AFEC_BASE + 0x30U)

#define N32_PWR_C1_PWR_CR            (N32_PWR_BASE + N32_PWR_C1_PWR_CR_OFFSET)
#define N32_PWR_C1_PWR_CSR           (N32_PWR_BASE + N32_PWR_C1_PWR_CSR_OFFSET)
#define N32_PWR_C1_PWR_CR2           (N32_PWR_BASE + N32_PWR_C1_PWR_CR2_OFFSET)

#define N32_PWR_C2_PWR_CR            (N32_PWR_BASE + N32_PWR_C2_PWR_CR_OFFSET)
#define N32_PWR_C2_PWR_CSR           (N32_PWR_BASE + N32_PWR_C2_PWR_CSR_OFFSET)
#define N32_PWR_C2_PWR_CR2           (N32_PWR_BASE + N32_PWR_C2_PWR_CR2_OFFSET)

#define N32_PWR_SYS_PWR_CR           (N32_PWR_BASE + N32_PWR_SYS_PWR_CR_OFFSET)
#define N32_PWR_SYS_PWR_CSR          (N32_PWR_BASE + N32_PWR_SYS_PWR_CSR_OFFSET)
#define N32_PWR_SYS_PWR_CR2          (N32_PWR_BASE + N32_PWR_SYS_PWR_CR2_OFFSET)
#define N32_PWR_SYS_PWR_CR3          (N32_PWR_BASE + N32_PWR_SYS_PWR_CR3_OFFSET)
#define N32_PWR_SYS_PWR_CR4          (N32_PWR_BASE + N32_PWR_SYS_PWR_CR4_OFFSET)
#define N32_PWR_SYS_PWR_BOR_CR       (N32_PWR_BASE + N32_PWR_SYS_PWR_BOR_CR_OFFSET)
#define N32_PWR_IP_MEMPWR_CR         (N32_PWR_BASE + N32_PWR_IP_MEMPWR_CR_OFFSET)
#define N32_PWR_IP_MEMPWR_CSR        (N32_PWR_BASE + N32_PWR_IP_MEMPWR_CSR_OFFSET)

#define N32_PWR_C1_MEM_LPCR1         (N32_PWR_BASE + N32_PWR_C1_MEM_LPCR1_OFFSET)
#define N32_PWR_C1_MEM_LPSR1         (N32_PWR_BASE + N32_PWR_C1_MEM_LPSR1_OFFSET)
#define N32_PWR_C1_TCM_PG0           (N32_PWR_BASE + N32_PWR_C1_TCM_PG0_OFFSET)
#define N32_PWR_C1_TCM_PG1           (N32_PWR_BASE + N32_PWR_C1_TCM_PG1_OFFSET)
#define N32_PWR_C1_TCM_RET1N0        (N32_PWR_BASE + N32_PWR_C1_TCM_RET1N0_OFFSET)
#define N32_PWR_C1_TCM_RET1N1        (N32_PWR_BASE + N32_PWR_C1_TCM_RET1N1_OFFSET)
#define N32_PWR_C1_TCM_RET2N0        (N32_PWR_BASE + N32_PWR_C1_TCM_RET2N0_OFFSET)
#define N32_PWR_C1_TCM_RET2N1        (N32_PWR_BASE + N32_PWR_C1_TCM_RET2N1_OFFSET)

#define N32_PWR_C2_MEM_LPCR          (N32_PWR_BASE + N32_PWR_C2_MEM_LPCR_OFFSET)

#define N32_PWR_SYS_MEM_LPCR         (N32_PWR_BASE + N32_PWR_SYS_MEM_LPCR_OFFSET)

#define N32_PWR_SYS_PWR_SHRT_CR      (N32_PWR_BASE + N32_PWR_SYS_PWR_SHRT_CR_OFFSET)
#define N32_PWR_SYS_PWR_MDMA_CR      (N32_PWR_BASE + N32_PWR_SYS_PWR_MDMA_CR_OFFSET)
#define N32_PWR_SYS_PWR_ETHCAT_CR    (N32_PWR_BASE + N32_PWR_SYS_PWR_ETHCAT_CR_OFFSET)

#define N32_PWR_PWR_EMC_CR0          (N32_PWR_BASE + N32_PWR_PWR_EMC_CR0_OFFSET)
#define N32_PWR_PWR_EMC_CR1          (N32_PWR_BASE + N32_PWR_PWR_EMC_CR1_OFFSET)
#define N32_PWR_PWR_EMC_CR2          (N32_PWR_BASE + N32_PWR_PWR_EMC_CR2_OFFSET)
#define N32_PWR_PWR_EMC_CR3          (N32_PWR_BASE + N32_PWR_PWR_EMC_CR3_OFFSET)
#define N32_PWR_PWR_EMC_CR4          (N32_PWR_BASE + N32_PWR_PWR_EMC_CR4_OFFSET)
#define N32_PWR_PWR_EMC_CR5          (N32_PWR_BASE + N32_PWR_PWR_EMC_CR5_OFFSET)
#define N32_PWR_PWR_EMC_CR6          (N32_PWR_BASE + N32_PWR_PWR_EMC_CR6_OFFSET)
#define N32_PWR_PWR_EMC_CR7          (N32_PWR_BASE + N32_PWR_PWR_EMC_CR7_OFFSET)
#define N32_PWR_PWR_BKP_EMC_CR0      (N32_PWR_BASE + N32_PWR_PWR_BKP_EMC_CR0_OFFSET)
#define N32_PWR_PWR_BKP_EMC_CR1      (N32_PWR_BASE + N32_PWR_PWR_BKP_EMC_CR1_OFFSET)

/* Register Bitfield Definitions ********************************************/

/** PVD level config **/
#define PWR_PVD_LEVEL_MASK                          ((uint32_t)0x001E0000U)
#define PWR_PVD_LEVEL_1V78                          ((uint32_t)0x00000000U)
#define PWR_PVD_LEVEL_1V88                          ((uint32_t)0x00020000U)
#define PWR_PVD_LEVEL_1V98                          ((uint32_t)0x00040000U)
#define PWR_PVD_LEVEL_2V08                          ((uint32_t)0x00060000U)
#define PWR_PVD_LEVEL_2V18                          ((uint32_t)0x00080000U)
#define PWR_PVD_LEVEL_2V28                          ((uint32_t)0x000A0000U)
#define PWR_PVD_LEVEL_2V38                          ((uint32_t)0x000C0000U)
#define PWR_PVD_LEVEL_2V48                          ((uint32_t)0x000E0000U)
#define PWR_PVD_LEVEL_2V58                          ((uint32_t)0x00100000U)
#define PWR_PVD_LEVEL_2V68                          ((uint32_t)0x00120000U)
#define PWR_PVD_LEVEL_2V78                          ((uint32_t)0x00140000U)
#define PWR_PVD_LEVEL_2V88                          ((uint32_t)0x00160000U)
#define PWR_PVD_LEVEL_3V28                          ((uint32_t)0x00180000U)
#define PWR_PVD_LEVEL_3V38                          ((uint32_t)0x001A0000U)
#define PWR_PVD_LEVEL_3V48                          ((uint32_t)0x001C0000U)
#define PWR_PVD_LEVEL_3V58                          ((uint32_t)0x001E0000U)

/** AVD level config **/
#define PWR_AVD_LEVEL_MASK                          ((uint32_t)0x0000F000U)
#define PWR_AVD_LEVEL_1V78                          ((uint32_t)0x00000000U)
#define PWR_AVD_LEVEL_1V88                          ((uint32_t)0x00001000U)
#define PWR_AVD_LEVEL_1V98                          ((uint32_t)0x00002000U)
#define PWR_AVD_LEVEL_2V08                          ((uint32_t)0x00003000U)
#define PWR_AVD_LEVEL_2V18                          ((uint32_t)0x00004000U)
#define PWR_AVD_LEVEL_2V28                          ((uint32_t)0x00005000U)
#define PWR_AVD_LEVEL_2V38                          ((uint32_t)0x00006000U)
#define PWR_AVD_LEVEL_2V48                          ((uint32_t)0x00007000U)
#define PWR_AVD_LEVEL_2V58                          ((uint32_t)0x00008000U)
#define PWR_AVD_LEVEL_2V68                          ((uint32_t)0x00009000U)
#define PWR_AVD_LEVEL_2V78                          ((uint32_t)0x0000A000U)
#define PWR_AVD_LEVEL_2V88                          ((uint32_t)0x0000B000U)
#define PWR_AVD_LEVEL_3V28                          ((uint32_t)0x0000C000U)
#define PWR_AVD_LEVEL_3V38                          ((uint32_t)0x0000D000U)
#define PWR_AVD_LEVEL_3V48                          ((uint32_t)0x0000E000U)
#define PWR_AVD_LEVEL_3V58                          ((uint32_t)0x0000F000U)

/* PWR M7 Control register 1 (PWR_M7CTRL1) */
#define PWR_M7CTRL1_CVBTF          (1 << 16)  /* Bit 16: Clear VBAT flags */
#define PWR_M7CTRL1_CSBF           (1 << 3)   /* Bit 3: Clear STANDBY flags */
#define PWR_M7CTRL1_CWKUPF         (1 << 2)   /* Bit 2: Clear the pin wakeup bit */
#define PWR_M7CTRL1_PDSEN          (1 << 1)   /* Bit 1: Power down deep sleep enable bit */

/* PWR M7 Control Status register (PWR_M7CTRLSTS) */
#define PWR_M7CTRLSTS_WKUP6F       (1 << 26)  /* Bit 26: RTC wakeup flag */
#define PWR_M7CTRLSTS_WKUP5F       (1 << 25)  /* Bit 25: WKUP5 pin PC1 wakeup flag */
#define PWR_M7CTRLSTS_WKUP4F       (1 << 24)  /* Bit 24: WKUP4 pin PI11 wakeup flag */
#define PWR_M7CTRLSTS_WKUP3F       (1 << 23)  /* Bit 23: WKUP3 pin PI8 wakeup flag */
#define PWR_M7CTRLSTS_WKUP2F       (1 << 22)  /* Bit 22: WKUP2 pin PC13 wakeup flag */
#define PWR_M7CTRLSTS_WKUP1F       (1 << 21)  /* Bit 21: WKUP1 pin PA2 wakeup flag */
#define PWR_M7CTRLSTS_WKUP0F       (1 << 20)  /* Bit 20: WKUP0 pin PA0 wakeup flag */

#define PWR_M7CTRLSTS_WKUP5POL     (1 << 19)  /* Bit 19: Wake-up polarity for the WKUP5 pin PC1 */
#define PWR_M7CTRLSTS_WKUP4POL     (1 << 18)  /* Bit 18: Wake-up polarity for the WKUP4 pin PI11 */
#define PWR_M7CTRLSTS_WKUP3POL     (1 << 17)  /* Bit 17: Wake-up polarity for the WKUP3 pin PI8 */
#define PWR_M7CTRLSTS_WKUP2POL     (1 << 16)  /* Bit 16: Wake-up polarity for the WKUP2 pin PC13 */
#define PWR_M7CTRLSTS_WKUP1POL     (1 << 15)  /* Bit 15: Wake-up polarity for the WKUP1 pin PA2 */
#define PWR_M7CTRLSTS_WKUP0POL     (1 << 14)  /* Bit 14: Wake-up polarity for the WKUP0 pin PA0 */

#define PWR_M7CTRLSTS_WKUP5EN      (1 << 13)  /* Bit 13: WKUP5 pin PC1 wakeup enable */
#define PWR_M7CTRLSTS_WKUP4EN      (1 << 12)  /* Bit 12: WKUP4 pin PI11 wakeup enable */
#define PWR_M7CTRLSTS_WKUP3EN      (1 << 11)  /* Bit 11: WKUP3 pin PI8 wakeup enable */
#define PWR_M7CTRLSTS_WKUP2EN      (1 << 10)  /* Bit 10: WKUP2 pin PC13 wakeup enable */
#define PWR_M7CTRLSTS_WKUP1EN      (1 << 9)   /* Bit 9: WKUP1 pin PA2 wakeup enable */
#define PWR_M7CTRLSTS_WKUP0EN      (1 << 8)   /* Bit 8: WKUP0 pin PA0 wakeup enable */

#define PWR_M7CTRLSTS_VBATF        (1 << 2)   /* Bit 2: VBAT flag */
#define PWR_M7CTRLSTS_SBF          (1 << 1)   /* Bit 1: STANDBY flag */

/* PWR M7 Control register 2 (PWR_M7CTRL2) */
#define PWR_M7CTRL2_MEM_CNTVAL_SHIFT (25)     /* Bits 25-30: Counter value for memory power ready count down */
#define PWR_M7CTRL2_MEM_CNTVAL_MASK  (0x3F << PWR_M7CTRL2_MEM_CNTVAL_SHIFT)

#define PWR_M7CTRL2_PWR_CNTVAL_SHIFT (17)     /* Bits 17-24: Counter value for CM7 logic power ready count down */
#define PWR_M7CTRL2_PWR_CNTVAL_MASK  (0xFF << PWR_M7CTRL2_PWR_CNTVAL_SHIFT)

#define PWR_M7CTRL2_TCM_RDYMD_SHIFT (15)      /* Bits 15-16: Selected the way of check TCM power ready for CM7 domain */
#define PWR_M7CTRL2_TCM_RDYMD_MASK  (0x3 << PWR_M7CTRL2_TCM_RDYMD_SHIFT)
#  define PWR_M7CTRL2_TCM_RDYMD_RDY_ONLY (0x0 << PWR_M7CTRL2_TCM_RDYMD_SHIFT) /* 00: Check only RDY signal */
#  define PWR_M7CTRL2_TCM_RDYMD_DELAY (0x1 << PWR_M7CTRL2_TCM_RDYMD_SHIFT)    /* 01: Check delay after enabled */
#  define PWR_M7CTRL2_TCM_RDYMD_RDY_DELAY (0x2 << PWR_M7CTRL2_TCM_RDYMD_SHIFT)/* 10: Check delay and RDY signal */
#  define PWR_M7CTRL2_TCM_RDYMD_DELAY_RDY (0x3 << PWR_M7CTRL2_TCM_RDYMD_SHIFT)/* 11: Check delay after RDY signal */

#define PWR_M7CTRL2_PWR_RDYMD_SHIFT (13)      /* Bits 13-14: Selected the way of check power ready for CM7 domain */
#define PWR_M7CTRL2_PWR_RDYMD_MASK  (0x3 << PWR_M7CTRL2_PWR_RDYMD_SHIFT)
#  define PWR_M7CTRL2_PWR_RDYMD_RDY_ONLY (0x0 << PWR_M7CTRL2_PWR_RDYMD_SHIFT) /* 00: Check only RDY signal */
#  define PWR_M7CTRL2_PWR_RDYMD_DELAY (0x1 << PWR_M7CTRL2_PWR_RDYMD_SHIFT)    /* 01: Check delay after enabled */
#  define PWR_M7CTRL2_PWR_RDYMD_RDY_DELAY (0x2 << PWR_M7CTRL2_PWR_RDYMD_SHIFT)/* 10: Check delay and RDY signal */
#  define PWR_M7CTRL2_PWR_RDYMD_DELAY_RDY (0x3 << PWR_M7CTRL2_PWR_RDYMD_SHIFT)/* 11: Check delay after RDY signal */

#define PWR_M7CTRL2_HCLK_ONINSLP   (1 << 12)  /* Bit 12: HCLK on in sleep mode */
#define PWR_M7CTRL2_NRST_WUPEN     (1 << 11)  /* Bit 11: NRST wakeup event enable in standby mode for M7 core */
#define PWR_M7CTRL2_RTC_ALMWUPEN   (1 << 8)   /* Bit 8: RTC_ALARM wakeup enable in standby mode for M7 core */
#define PWR_M7CTRL2_BSRSTBRET      (1 << 2)   /* Bit 2: Backup SRAM retention enable in STANDBY mode */
#define PWR_M7CTRL2_BSRVBRET       (1 << 1)   /* Bit 1: Backup SRAM retention enable in VBAT mode */
#define PWR_M7CTRL2_STOP2EN        (1 << 0)   /* Bit 0: M7 core STOP2 mode enable */

/* PWR M4 Control register 1 (PWR_M4CTRL1) */
#define PWR_M4CTRL1_CVBTF          (1 << 16)  /* Bit 16: Clear VBAT flags */
#define PWR_M4CTRL1_CSBF           (1 << 3)   /* Bit 3: Clear STANDBY flags */
#define PWR_M4CTRL1_CWKUPF         (1 << 2)   /* Bit 2: Clear the pin wakeup bit */
#define PWR_M4CTRL1_PDS            (1 << 1)   /* Bit 1: Power down deep sleep bit */

/* PWR M4 Control Status register (PWR_M4CTRLSTS) */
#define PWR_M4CTRLSTS_WKUP6F       (1 << 26)  /* Bit 26: RTC wakeup flag */
#define PWR_M4CTRLSTS_WKUP5F       (1 << 25)  /* Bit 25: WKUP5 pin PC1 wakeup flag */
#define PWR_M4CTRLSTS_WKUP4F       (1 << 24)  /* Bit 24: WKUP4 pin PI11 wakeup flag */
#define PWR_M4CTRLSTS_WKUP3F       (1 << 23)  /* Bit 23: WKUP3 pin PI8 wakeup flag */
#define PWR_M4CTRLSTS_WKUP2F       (1 << 22)  /* Bit 22: WKUP2 pin PC13 wakeup flag */
#define PWR_M4CTRLSTS_WKUP1F       (1 << 21)  /* Bit 21: WKUP1 pin PA2 wakeup flag */
#define PWR_M4CTRLSTS_WKUP0F       (1 << 20)  /* Bit 20: WKUP0 pin PA0 wakeup flag */

#define PWR_M4CTRLSTS_WKUP5POL     (1 << 19)  /* Bit 19: Wake-up polarity for the WKUP5 pin PC1 */
#define PWR_M4CTRLSTS_WKUP4POL     (1 << 18)  /* Bit 18: Wake-up polarity for the WKUP4 pin PI11 */
#define PWR_M4CTRLSTS_WKUP3POL     (1 << 17)  /* Bit 17: Wake-up polarity for the WKUP3 pin PI8 */
#define PWR_M4CTRLSTS_WKUP2POL     (1 << 16)  /* Bit 16: Wake-up polarity for the WKUP2 pin PC13 */
#define PWR_M4CTRLSTS_WKUP1POL     (1 << 15)  /* Bit 15: Wake-up polarity for the WKUP1 pin PA2 */
#define PWR_M4CTRLSTS_WKUP0POL     (1 << 14)  /* Bit 14: Wake-up polarity for the WKUP0 pin PA0 */

#define PWR_M4CTRLSTS_WKUP5EN      (1 << 13)  /* Bit 13: WKUP5 pin PC1 wakeup enable */
#define PWR_M4CTRLSTS_WKUP4EN      (1 << 12)  /* Bit 12: WKUP4 pin PI11 wakeup enable */
#define PWR_M4CTRLSTS_WKUP3EN      (1 << 11)  /* Bit 11: WKUP3 pin PI8 wakeup enable */
#define PWR_M4CTRLSTS_WKUP2EN      (1 << 10)  /* Bit 10: WKUP2 pin PC13 wakeup enable */
#define PWR_M4CTRLSTS_WKUP1EN      (1 << 9)   /* Bit 9: WKUP1 pin PA2 wakeup enable */
#define PWR_M4CTRLSTS_WKUP0EN      (1 << 8)   /* Bit 8: WKUP0 pin PA0 wakeup enable */

#define PWR_M4CTRLSTS_VBATF        (1 << 2)   /* Bit 2: VBAT flag */
#define PWR_M4CTRLSTS_SBF          (1 << 1)   /* Bit 1: STANDBY flag */

/* PWR M4 Control register 2 (PWR_M4CTRL2) */
#define PWR_M4CTRL2_MEM_CNTVAL_SHIFT (25)     /* Bits 25-30: Counter value for memory power ready count down */
#define PWR_M4CTRL2_MEM_CNTVAL_MASK  (0x3F << PWR_M4CTRL2_MEM_CNTVAL_SHIFT)

#define PWR_M4CTRL2_PWR_CNTVAL_SHIFT (17)     /* Bits 17-24: Counter value for CM4 logic power ready count down */
#define PWR_M4CTRL2_PWR_CNTVAL_MASK  (0xFF << PWR_M4CTRL2_PWR_CNTVAL_SHIFT)

#define PWR_M4CTRL2_TCM_RDYMD_SHIFT (15)      /* Bits 15-16: Selected the way of check TCM power ready for CM4 domain */
#define PWR_M4CTRL2_TCM_RDYMD_MASK  (0x3 << PWR_M4CTRL2_TCM_RDYMD_SHIFT)
#  define PWR_M4CTRL2_TCM_RDYMD_RDY_ONLY (0x0 << PWR_M4CTRL2_TCM_RDYMD_SHIFT) /* 00: Check only RDY signal */
#  define PWR_M4CTRL2_TCM_RDYMD_DELAY (0x1 << PWR_M4CTRL2_TCM_RDYMD_SHIFT)    /* 01: Check delay after enabled */
#  define PWR_M4CTRL2_TCM_RDYMD_RDY_DELAY (0x2 << PWR_M4CTRL2_TCM_RDYMD_SHIFT)/* 10: Check delay and RDY signal */
#  define PWR_M4CTRL2_TCM_RDYMD_DELAY_RDY (0x3 << PWR_M4CTRL2_TCM_RDYMD_SHIFT)/* 11: Check delay after RDY signal */

#define PWR_M4CTRL2_PWR_RDYMD_SHIFT (13)      /* Bits 13-14: Selected the way of check power ready for CM4 domain */
#define PWR_M4CTRL2_PWR_RDYMD_MASK  (0x3 << PWR_M4CTRL2_PWR_RDYMD_SHIFT)
#  define PWR_M4CTRL2_PWR_RDYMD_RDY_ONLY (0x0 << PWR_M4CTRL2_PWR_RDYMD_SHIFT) /* 00: Check only RDY signal */
#  define PWR_M4CTRL2_PWR_RDYMD_DELAY (0x1 << PWR_M4CTRL2_PWR_RDYMD_SHIFT)    /* 01: Check delay after enabled */
#  define PWR_M4CTRL2_PWR_RDYMD_RDY_DELAY (0x2 << PWR_M4CTRL2_PWR_RDYMD_SHIFT)/* 10: Check delay and RDY signal */
#  define PWR_M4CTRL2_PWR_RDYMD_DELAY_RDY (0x3 << PWR_M4CTRL2_PWR_RDYMD_SHIFT)/* 11: Check delay after RDY signal */

#define PWR_M4CTRL2_HCLK_ONINSLP   (1 << 12)  /* Bit 12: HCLK on in sleep mode */
#define PWR_M4CTRL2_NRST_WUPEN     (1 << 11)  /* Bit 11: NRST wakeup event enable in standby mode for M4 core */
#define PWR_M4CTRL2_RTC_ALMWUPEN   (1 << 8)   /* Bit 8: RTC_ALARM wakeup enable in standby mode for M4 core */
#define PWR_M4CTRL2_BSRSTBRET      (1 << 2)   /* Bit 2: Backup SRAM retention enable in STANDBY mode */
#define PWR_M4CTRL2_BSRVBRET       (1 << 1)   /* Bit 1: Backup SRAM retention enable in VBAT mode */
#define PWR_M4CTRL2_STOP2EN        (1 << 0)   /* Bit 0: M4 core STOP2 mode enable */

/* PWR System Control register 1 (PWR_SYSCTRL1) */
#define PWR_SYSCTRL1_DCDC_VSELKEY_SHIFT (28)  /* Bits 28-31: DCDC_VSELKEY_UNLOCK key */
#define PWR_SYSCTRL1_DCDC_VSELKEY_MASK  (0xF << PWR_SYSCTRL1_DCDC_VSELKEY_SHIFT)
#  define PWR_SYSCTRL1_DCDC_VSELKEY_UNLOCK (0xF << PWR_SYSCTRL1_DCDC_VSELKEY_SHIFT) /* 1111: Unlock */

#define PWR_SYSCTRL1_NRST_DGFCNT_SHIFT (16)   /* Bits 16-27: Digital Glitch Filter on NRST filtered pulse width configuration */
#define PWR_SYSCTRL1_NRST_DGFCNT_MASK  (0xFFF << PWR_SYSCTRL1_NRST_DGFCNT_SHIFT)

#define PWR_SYSCTRL1_NRST_DGFBP       (1 << 15) /* Bit 15: Bypass digital glitch Filter on NRST */
#define PWR_SYSCTRL1_AGF_STBWUPPBP    (1 << 13) /* Bit 13: Bypass analog glitch filter on standby wakeup pads */
#define PWR_SYSCTRL1_AGF_DTASGBP      (1 << 12) /* Bit 12: Bypass analog glitch filter on some digital to analog enable signals */
#define PWR_SYSCTRL1_AGF_ARSTOBP      (1 << 11) /* Bit 11: Bypass analog glitch filter on analog reset outputs */
#define PWR_SYSCTRL1_DBKP             (1 << 8)  /* Bit 8: Disable write protection for the backup domain */
#define PWR_SYSCTRL1_PVDEN            (1 << 4)  /* Bit 4: Power Voltage Detector (PVD) Enable */
#define PWR_SYSCTRL1_AVDEN            (1 << 3)  /* Bit 3: Analog Voltage Detector (AVD) Enable */
#define PWR_SYSCTRL1_BKPLDOEN         (1 << 1)  /* Bit 1: Backup LDO enable */
#define PWR_SYSCTRL1_BKPLDO_CTRLEN    (1 << 0)  /* Bit 0: Backup LDO control enable */

/* PWR System Control Status register (PWR_SYSCTRLSTS) */
#define PWR_SYSCTRLSTS_PVDO          (1 << 4)  /* Bit 4: PVD output */
#define PWR_SYSCTRLSTS_AVDO          (1 << 3)  /* Bit 3: AVD output */
#define PWR_SYSCTRLSTS_OTPPWRRDY     (1 << 1)  /* Bit 1: OTP Power Ready */
#define PWR_SYSCTRLSTS_DCDCBPF       (1 << 0)  /* Bit 0: DCDC Bypass flag */

/* PWR System Control register 2 (PWR_SYSCTRL2) */
#define PWR_SYSCTRL2_OTP_LPCLKDIV_SHIFT (24)   /* Bits 24-26: Clock divisor for OTP low power mode */
#define PWR_SYSCTRL2_OTP_LPCLKDIV_MASK  (0x7 << PWR_SYSCTRL2_OTP_LPCLKDIV_SHIFT)
#  define PWR_SYSCTRL2_OTP_LPCLKDIV_DIV1 (0x0 << PWR_SYSCTRL2_OTP_LPCLKDIV_SHIFT) /* 000: No division */
#  define PWR_SYSCTRL2_OTP_LPCLKDIV_DIV2 (0x1 << PWR_SYSCTRL2_OTP_LPCLKDIV_SHIFT) /* 001: Divide by 2 */
#  define PWR_SYSCTRL2_OTP_LPCLKDIV_DIV3 (0x2 << PWR_SYSCTRL2_OTP_LPCLKDIV_SHIFT) /* 010: Divide by 3 */
#  define PWR_SYSCTRL2_OTP_LPCLKDIV_DIV4 (0x3 << PWR_SYSCTRL2_OTP_LPCLKDIV_SHIFT) /* 011: Divide by 4 */
#  define PWR_SYSCTRL2_OTP_LPCLKDIV_DIV5 (0x4 << PWR_SYSCTRL2_OTP_LPCLKDIV_SHIFT) /* 100: Divide by 5 */
#  define PWR_SYSCTRL2_OTP_LPCLKDIV_DIV6 (0x5 << PWR_SYSCTRL2_OTP_LPCLKDIV_SHIFT) /* 101: Divide by 6 */
#  define PWR_SYSCTRL2_OTP_LPCLKDIV_DIV7 (0x6 << PWR_SYSCTRL2_OTP_LPCLKDIV_SHIFT) /* 110: Divide by 7 */
#  define PWR_SYSCTRL2_OTP_LPCLKDIV_DIV8 (0x7 << PWR_SYSCTRL2_OTP_LPCLKDIV_SHIFT) /* 111: Divide by 8 */

#define PWR_SYSCTRL2_OTP_FRCSTB      (1 << 23) /* Bit 23: OTP force STBY */
#define PWR_SYSCTRL2_OTP_FRCDSTB     (1 << 22) /* Bit 22: OTP force DSTBY */
#define PWR_SYSCTRL2_OTP_STB_INSTP0  (1 << 21) /* Bit 21: OTP STBY in STOP0 mode */
#define PWR_SYSCTRL2_OTP_DSTB_INSTP0 (1 << 20) /* Bit 20: OTP DSTBY in STOP0 mode */
#define PWR_SYSCTRL2_OTP_STB_INSTP2  (1 << 19) /* Bit 19: OTP STBY in STOP2 mode */
#define PWR_SYSCTRL2_OTP_DSTB_INSTP2 (1 << 18) /* Bit 18: OTP DSTBY in STOP2 mode */
#define PWR_SYSCTRL2_EXTI_MASKRSTEN  (1 << 17) /* Bit 17: EXTI mask reset enable */
#define PWR_SYSCTRL2_VDDDM_RDYMD     (1 << 16) /* Bit 16: VDDDM ready mode */
#define PWR_SYSCTRL2_BKPLDO_CTRLEN   (1 << 11) /* Bit 11: Backup LDO control enable */
#define PWR_SYSCTRL2_MR_STBOFFEN     (1 << 1)  /* Bit 1: Main regulator off in standby mode */

/* PWR System Control register 3 (PWR_SYSCTRL3) */
#define PWR_SYSCTRL3_HSC2_PSWACK1    (1 << 22) /* Bit 22: HSC2 power switch ACK1 */
#define PWR_SYSCTRL3_HSC1_PSWACK1    (1 << 21) /* Bit 21: HSC1 power switch ACK1 */
#define PWR_SYSCTRL3_GRC_PSWACK1     (1 << 20) /* Bit 20: GRC power switch ACK1 */

#define PWR_SYSCTRL3_HSC2_PWRRDY     (1 << 18) /* Bit 18: HSC2 power domain ready flag */
#define PWR_SYSCTRL3_HSC1_PWRRDY     (1 << 17) /* Bit 17: HSC1 power domain ready flag */
#define PWR_SYSCTRL3_GRC_PWRRDY      (1 << 16) /* Bit 16: GRC power domain ready flag */

#define PWR_SYSCTRL3_HSC2_ISNEN      (1 << 10) /* Bit 10: HSC2 power domain isolation signal */
#define PWR_SYSCTRL3_HSC1_ISNEN      (1 << 9)  /* Bit 9: HSC1 power domain isolation signal */
#define PWR_SYSCTRL3_GRC_ISNEN       (1 << 8)  /* Bit 8: GRC power domain isolation signal */

#define PWR_SYSCTRL3_HSC2_FUCEN      (1 << 6)  /* Bit 6: HSC2 function mode enable */
#define PWR_SYSCTRL3_HSC1_FUCEN      (1 << 5)  /* Bit 5: HSC1 function mode enable */
#define PWR_SYSCTRL3_GRC_FUCEN       (1 << 4)  /* Bit 4: GRC function mode enable */

#define PWR_SYSCTRL3_HSC2_PGEN       (1 << 2)  /* Bit 2: HSC2 power gate enable */
#define PWR_SYSCTRL3_HSC1_PGEN       (1 << 1)  /* Bit 1: HSC1 power gate enable */
#define PWR_SYSCTRL3_GRC_PGEN        (1 << 0)  /* Bit 0: GRC power gate enable */

/* PWR System Control register 4 (PWR_SYSCTRL4) */
#define PWR_SYSCTRL4_MR_LP_VSELEN    (1 << 29) /* Bit 29: Enable targeting MR voltage output in low power mode */
#define PWR_SYSCTRL4_DCDC_LP_VSEL_SHIFT (24)   /* Bits 24-27: Targeting dcdc voltage in low power mode */
#define PWR_SYSCTRL4_DCDC_LP_VSEL_MASK  (0xF << PWR_SYSCTRL4_DCDC_LP_VSEL_SHIFT)
#  define PWR_SYSCTRL4_DCDC_LP_VSEL_0_80V (0x5 << PWR_SYSCTRL4_DCDC_LP_VSEL_SHIFT) /* 0101: 0.80V */
#  define PWR_SYSCTRL4_DCDC_LP_VSEL_0_85V (0x6 << PWR_SYSCTRL4_DCDC_LP_VSEL_SHIFT) /* 0110: 0.85V */
#  define PWR_SYSCTRL4_DCDC_LP_VSEL_0_90V (0x7 << PWR_SYSCTRL4_DCDC_LP_VSEL_SHIFT) /* 0111: 0.90V */

#define PWR_SYSCTRL4_VDDD_LP_PORSEL_SHIFT (22) /* Bits 22-23: VDDD POR config in low power mode */
#define PWR_SYSCTRL4_VDDD_LP_PORSEL_MASK  (0x3 << PWR_SYSCTRL4_VDDD_LP_PORSEL_SHIFT)
#  define PWR_SYSCTRL4_VDDD_LP_PORSEL_0_7V_0_65V (0x0 << PWR_SYSCTRL4_VDDD_LP_PORSEL_SHIFT)     /* 00: POR 0.7V, PDR 0.65V */
#  define PWR_SYSCTRL4_VDDD_LP_PORSEL_0_7V_0_65V_ALT (0x1 << PWR_SYSCTRL4_VDDD_LP_PORSEL_SHIFT) /* 01: POR 0.7V, PDR 0.65V */
#  define PWR_SYSCTRL4_VDDD_LP_PORSEL_0_75V_0_7V (0x2 << PWR_SYSCTRL4_VDDD_LP_PORSEL_SHIFT)     /* 10: POR 0.75V, PDR 0.7V */
#  define PWR_SYSCTRL4_VDDD_LP_PORSEL_0_8V_0_75V (0x3 << PWR_SYSCTRL4_VDDD_LP_PORSEL_SHIFT)     /* 11: POR 0.8V, PDR 0.75V */

#define PWR_SYSCTRL4_DCDC_LP_PORVSEL_SHIFT (17) /* Bits 17-21: DCDC POR voltage value in low power mode */
#define PWR_SYSCTRL4_DCDC_LP_PORVSEL_MASK  (0x1F << PWR_SYSCTRL4_DCDC_LP_PORVSEL_SHIFT)
#  define PWR_SYSCTRL4_DCDC_LP_PORVSEL_0_8V (0x5 << PWR_SYSCTRL4_DCDC_LP_PORVSEL_SHIFT)  /* 00101: 0.8V */
#  define PWR_SYSCTRL4_DCDC_LP_PORVSEL_0_85V (0x6 << PWR_SYSCTRL4_DCDC_LP_PORVSEL_SHIFT) /* 00110: 0.85V */
#  define PWR_SYSCTRL4_DCDC_LP_PORVSEL_0_9V (0x7 << PWR_SYSCTRL4_DCDC_LP_PORVSEL_SHIFT)  /* 00111: 0.9V */

#define PWR_SYSCTRL4_BG_LPVREF       (1 << 16) /* Bit 16: Targeting BG ref in low power mode */
#define PWR_SYSCTRL4_BG_VREF         (1 << 15) /* Bit 15: BG voltage reference select */
#define PWR_SYSCTRL4_VDDD_PORSEL_SHIFT (12)    /* Bits 12-13: VDDD POR config in low power mode */
#define PWR_SYSCTRL4_VDDD_PORSEL_MASK (0x3 << PWR_SYSCTRL4_VDDD_PORSEL_SHIFT)
#  define PWR_SYSCTRL4_VDDD_PORSEL_0_7V_0_65V (0x0 << PWR_SYSCTRL4_VDDD_PORSEL_SHIFT) /* 00: POR 0.7V, PDR 0.65V */
#  define PWR_SYSCTRL4_VDDD_PORSEL_0_75V_0_7V (0x2 << PWR_SYSCTRL4_VDDD_PORSEL_SHIFT) /* 10: POR 0.75V, PDR 0.7V */
#  define PWR_SYSCTRL4_VDDD_PORSEL_0_8V_0_75V (0x3 << PWR_SYSCTRL4_VDDD_PORSEL_SHIFT) /* 11: POR 0.8V, PDR 0.75V */

#define PWR_SYSCTRL4_MLDO_LPO_VSEL_SHIFT (8)   /* Bits 8-9: Output voltage of Main Regulator LDO option in SYS STOP2 and STANDBY mode */
#define PWR_SYSCTRL4_MLDO_LPO_VSEL_MASK  (0x3 << PWR_SYSCTRL4_MLDO_LPO_VSEL_SHIFT)
#  define PWR_SYSCTRL4_MLDO_LPO_VSEL_0_8V (0x0 << PWR_SYSCTRL4_MLDO_LPO_VSEL_SHIFT)  /* 00: 0.8V */
#  define PWR_SYSCTRL4_MLDO_LPO_VSEL_0_85V (0x1 << PWR_SYSCTRL4_MLDO_LPO_VSEL_SHIFT) /* 01: 0.85V */
#  define PWR_SYSCTRL4_MLDO_LPO_VSEL_0_9V (0x2 << PWR_SYSCTRL4_MLDO_LPO_VSEL_SHIFT)  /* 10: 0.9V */

#define PWR_SYSCTRL4_MLDO_OVSEL_SHIFT (8)      /* Bits 8-9: Output voltage of Main Regulator LDO option in SYS RUN mode */
#define PWR_SYSCTRL4_MLDO_OVSEL_MASK  (0x3 << PWR_SYSCTRL4_MLDO_OVSEL_SHIFT)
#  define PWR_SYSCTRL4_MLDO_OVSEL_0_8V (0x0 << PWR_SYSCTRL4_MLDO_OVSEL_SHIFT)  /* 00: 0.8V */
#  define PWR_SYSCTRL4_MLDO_OVSEL_0_85V (0x1 << PWR_SYSCTRL4_MLDO_OVSEL_SHIFT) /* 01: 0.85V */
#  define PWR_SYSCTRL4_MLDO_OVSEL_0_9V (0x2 << PWR_SYSCTRL4_MLDO_OVSEL_SHIFT)  /* 10: 0.9V */

#define PWR_SYSCTRL4_DCD_FRCEN       (1 << 3)  /* Bit 3: DCDC force enable */
#define PWR_SYSCTRL4_DCDEN           (1 << 2)  /* Bit 2: DCDC enable */
#define PWR_SYSCTRL4_MLDOEN          (1 << 1)  /* Bit 1: Main regulator LDO enable */
#define PWR_SYSCTRL4_VCORESRC        (1 << 0)  /* Bit 0: VCORE source select */

#define PWR_VCORESRC_MASK            (PWR_SYSCTRL4_DCD_FRCEN|PWR_SYSCTRL4_DCDEN|\
                                      PWR_SYSCTRL4_MLDOEN|PWR_SYSCTRL4_VCORESRC)

typedef enum
{
  PWR_VCORESRC_EXT = 0,
  PWR_VCORESRC_LDO,
  PWR_VCORESRC_SMPS,
} pwr_vcoresrc_e;

/* PWR IP Memory Control register (PWR_IPMEMCTRL) */
#define PWR_IPMEMCTRL_ESC_PGEN       (1 << 12) /* Bit 12: ESC memory power gate enable */
#define PWR_IPMEMCTRL_FMAC_PGEN      (1 << 11) /* Bit 11: FMAC memory power gate enable */
#define PWR_IPMEMCTRL_SDMMC1_PGEN    (1 << 10) /* Bit 10: SDMMC1 memory power gate enable */
#define PWR_IPMEMCTRL_USB1_PGEN      (1 << 9)  /* Bit 9: USB1 memory power gate enable */
#define PWR_IPMEMCTRL_ETH1_PGEN      (1 << 8)  /* Bit 8: ETH1 memory power gate enable */
#define PWR_IPMEMCTRL_SDMMC2_PGEN    (1 << 7)  /* Bit 7: SDMMC2 memory power gate enable */
#define PWR_IPMEMCTRL_USB2_PGEN      (1 << 6)  /* Bit 6: USB2 memory power gate enable */
#define PWR_IPMEMCTRL_ETH2_PGEN      (1 << 5)  /* Bit 5: ETH2 memory power gate enable */
#define PWR_IPMEMCTRL_DVP_PGEN       (1 << 4)  /* Bit 4: DVP memory power gate enable */
#define PWR_IPMEMCTRL_DSI_PGEN       (1 << 3)  /* Bit 3: DSI memory power gate enable */
#define PWR_IPMEMCTRL_JPEG_PGEN      (1 << 2)  /* Bit 2: JPEG memory power gate enable */
#define PWR_IPMEMCTRL_LCDC_PGEN      (1 << 1)  /* Bit 1: LCDC memory power gate enable */
#define PWR_IPMEMCTRL_GPU_PGEN       (1 << 0)  /* Bit 0: GPU memory power gate enable */

/* PWR IP Memory Status register (PWR_IPMEMSTS) */
#define PWR_IPMEMSTS_ALLIP_PRDY      (1 << 31) /* Bit 31: ALL IP memory power ready flag */
#define PWR_IPMEMSTS_ESC_PRDY        (1 << 12) /* Bit 12: ESC memory power ready flag */
#define PWR_IPMEMSTS_FMAC_PRDY       (1 << 11) /* Bit 11: FMAC memory power ready flag */
#define PWR_IPMEMSTS_SDMMC1_PRDY     (1 << 10) /* Bit 10: SDMMC1 memory power ready flag */
#define PWR_IPMEMSTS_USB1_PRDY       (1 << 9)  /* Bit 9: USB1 memory power ready flag */
#define PWR_IPMEMSTS_ETH1_PRDY       (1 << 8)  /* Bit 8: ETH1 memory power ready flag */
#define PWR_IPMEMSTS_SDMMC2_PRDY     (1 << 7)  /* Bit 7: SDMMC2 memory power ready flag */
#define PWR_IPMEMSTS_USB2_PRDY       (1 << 6)  /* Bit 6: USB2 memory power ready flag */
#define PWR_IPMEMSTS_ETH2_PRDY       (1 << 5)  /* Bit 5: ETH2 memory power ready flag */
#define PWR_IPMEMSTS_DVP_PRDY        (1 << 4)  /* Bit 4: DVP memory power ready flag */
#define PWR_IPMEMSTS_DSI_PRDY        (1 << 3)  /* Bit 3: DSI memory power ready flag */
#define PWR_IPMEMSTS_JPEG_PRDY       (1 << 2)  /* Bit 2: JPEG memory power ready flag */
#define PWR_IPMEMSTS_LCDC_PRDY       (1 << 1)  /* Bit 1: LCDC memory power ready flag */
#define PWR_IPMEMSTS_GPU_PRDY        (1 << 0)  /* Bit 0: GPU memory power ready flag */

/* PWR CM7 Memory Low Power Control register (PWR_M7MEMLPCTRL) */
#define PWR_M7MEMLPCTRL_MEM_PGCFG_SHIFT (2)    /* Bits 2-3: Memory power gate sequence control */
#define PWR_M7MEMLPCTRL_MEM_PGCFG_MASK  (0x3 << PWR_M7MEMLPCTRL_MEM_PGCFG_SHIFT)
#  define PWR_M7MEMLPCTRL_MEM_PGCFG_CM7ALL (0x0 << PWR_M7MEMLPCTRL_MEM_PGCFG_SHIFT) /* 00: All CM7 memories in one daisy chain */
#  define PWR_M7MEMLPCTRL_MEM_PGCFG_DITCM (0x1 << PWR_M7MEMLPCTRL_MEM_PGCFG_SHIFT)  /* 01: D-cache -> I-cache -> TCM daisy */
#  define PWR_M7MEMLPCTRL_MEM_PGCFG_DIC (0x2 << PWR_M7MEMLPCTRL_MEM_PGCFG_SHIFT)    /* 10: D-cache + I-cache -> every 4 pieces of TCM memory */
#  define PWR_M7MEMLPCTRL_MEM_PGCFG_ALL (0x3 << PWR_M7MEMLPCTRL_MEM_PGCFG_SHIFT)    /* 11: All memory in one daisy chain */

#define PWR_M7MEMLPCTRL_MEM_RETSTP0EN_SHIFT (0) /* Bits 0-1: Memory retention mode select in STOP0 */
#define PWR_M7MEMLPCTRL_MEM_RETSTP0EN_MASK  (0x3 << PWR_M7MEMLPCTRL_MEM_RETSTP0EN_SHIFT)
#  define PWR_M7MEMLPCTRL_MEM_RETSTP0EN_CHIP_DISABLE (0x0 << PWR_M7MEMLPCTRL_MEM_RETSTP0EN_SHIFT) /* 00: Chip disable mode */
#  define PWR_M7MEMLPCTRL_MEM_RETSTP0EN_PRECHARGE (0x1 << PWR_M7MEMLPCTRL_MEM_RETSTP0EN_SHIFT)    /* 01: Precharge mode */
#  define PWR_M7MEMLPCTRL_MEM_RETSTP0EN_RETENTION1 (0x3 << PWR_M7MEMLPCTRL_MEM_RETSTP0EN_SHIFT)   /* 11: Retention 1 mode */

#define PWR_M7MEMLPCTRL_MEM_PGSTP0EN_SHIFT (0) /* Bits 0-1: Memory power gate mode select in STOP0 */
#define PWR_M7MEMLPCTRL_MEM_PGSTP0EN_MASK  (0x3 << PWR_M7MEMLPCTRL_MEM_PGSTP0EN_SHIFT)
#  define PWR_M7MEMLPCTRL_MEM_PGSTP0EN_CHIP_DISABLE (0x0 << PWR_M7MEMLPCTRL_MEM_PGSTP0EN_SHIFT) /* 00: Chip disable mode */
#  define PWR_M7MEMLPCTRL_MEM_PGSTP0EN_PRECHARGE (0x1 << PWR_M7MEMLPCTRL_MEM_PGSTP0EN_SHIFT)    /* 01: Precharge mode */
#  define PWR_M7MEMLPCTRL_MEM_PGSTP0EN_RETENTION1 (0x3 << PWR_M7MEMLPCTRL_MEM_PGSTP0EN_SHIFT)   /* 11: Retention 1 mode */

/* PWR CM7 Memory Low Power Status register (PWR_M7MEMLPSTS) */
#define PWR_M7MEMLPSTS_TCMRDY        (1 << 31) /* Bit 31: All enabled TCM memory power ready flag */

/* PWR CM7 TCM Part0 Program register (PWR_M7TCMPG0) */
#define PWR_M7TCMPG0_TCM_PG0_SHIFT   (0)      /* Bits 0-31: Software control to power gate TCM Part0 memories */
#define PWR_M7TCMPG0_TCM_PG0_MASK    (0xFFFFFFFF << PWR_M7TCMPG0_TCM_PG0_SHIFT)

/* PWR CM7 TCM Part1 Program register (PWR_M7TCMPG1) */
#define PWR_M7TCMPG1_TCM_PG1_SHIFT   (0)      /* Bits 0-31: Software control to power gate TCM Part1 memories */
#define PWR_M7TCMPG1_TCM_PG1_MASK    (0xFFFFFFFF << PWR_M7TCMPG1_TCM_PG1_SHIFT)

/* PWR CM7 TCM Part0 Retain 1 register (PWR_M7TCMRET1N0) */
#define PWR_M7TCMRET1N0_TCM_RENT1N0_SHIFT (0) /* Bits 0-31: TCM Part0 Retain 1 mode select */
#define PWR_M7TCMRET1N0_TCM_RENT1N0_MASK  (0xFFFFFFFF << PWR_M7TCMRET1N0_TCM_RENT1N0_SHIFT)

/* PWR CM7 TCM Part1 Retain 1 register (PWR_M7TCMRET1N1) */
#define PWR_M7TCMRET1N1_TCM_RENT1N1_SHIFT (0) /* Bits 0-31: TCM Part1 Retain 1 mode select */
#define PWR_M7TCMRET1N1_TCM_RENT1N1_MASK  (0xFFFFFFFF << PWR_M7TCMRET1N1_TCM_RENT1N1_SHIFT)

/* PWR CM7 TCM Part0 Retain 2 register (PWR_M7TCMRET2N0) */
#define PWR_M7TCMRET2N0_TCM_RENT2N0_SHIFT (0) /* Bits 0-31: TCM Part0 Retain 2 mode select */
#define PWR_M7TCMRET2N0_TCM_RENT2N0_MASK  (0xFFFFFFFF << PWR_M7TCMRET2N0_TCM_RENT2N0_SHIFT)

/* PWR CM7 TCM Part1 Retain 2 register (PWR_M7TCMRET2N1) */
#define PWR_M7TCMRET2N1_TCM_RENT2N1_SHIFT (0) /* Bits 0-31: TCM Part1 Retain 2 mode select */
#define PWR_M7TCMRET2N1_TCM_RENT2N1_MASK  (0xFFFFFFFF << PWR_M7TCMRET2N1_TCM_RENT2N1_SHIFT)

/* PWR CM7 TCM Part0 Power Ready register (PWR_M7TCMRDY0) */
#define PWR_M7TCMRDY0_TCM_RDY0_SHIFT (0)      /* Bits 0-31: TCM Part0 Power Ready flag */
#define PWR_M7TCMRDY0_TCM_RDY0_MASK  (0xFFFFFFFF << PWR_M7TCMRDY0_TCM_RDY0_SHIFT)

/* PWR CM7 TCM Part1 Power Ready register (PWR_M7TCMRDY1) */
#define PWR_M7TCMRDY1_TCM_RDY1_SHIFT (0)      /* Bits 0-31: TCM Part1 Power Ready flag */
#define PWR_M7TCMRDY1_TCM_RDY1_MASK  (0xFFFFFFFF << PWR_M7TCMRDY1_TCM_RDY1_SHIFT)

/* PWR CM4 Memory Low Power Control register (PWR_M4MEMLPCTRL) */
#define PWR_M4MEMLPCTRL_MEM_PGCFG_SHIFT (2)    /* Bits 2-3: Memory power gate sequence control */
#define PWR_M4MEMLPCTRL_MEM_PGCFG_MASK  (0x3 << PWR_M4MEMLPCTRL_MEM_PGCFG_SHIFT)
#  define PWR_M4MEMLPCTRL_MEM_PGCFG_CM4ALL (0x0 << PWR_M4MEMLPCTRL_MEM_PGCFG_SHIFT)       /* 00: All CM4 memories in one daisy chain */
#  define PWR_M4MEMLPCTRL_MEM_PGCFG_ALL_PARALLEL (0x1 << PWR_M4MEMLPCTRL_MEM_PGCFG_SHIFT) /* 01: All memory parallel */
#  define PWR_M4MEMLPCTRL_MEM_PGCFG_DITCM (0x2 << PWR_M4MEMLPCTRL_MEM_PGCFG_SHIFT)        /* 10: D-cache -> I-cache, parallel within each group */
#  define PWR_M4MEMLPCTRL_MEM_PGCFG_ALL (0x3 << PWR_M4MEMLPCTRL_MEM_PGCFG_SHIFT)          /* 11: All memory in one daisy chain */

#define PWR_M4MEMLPCTRL_MEM_RETSTP0EN_SHIFT (0) /* Bits 0-1: Memory retention mode select in STOP0 */
#define PWR_M4MEMLPCTRL_MEM_RETSTP0EN_MASK  (0x3 << PWR_M4MEMLPCTRL_MEM_RETSTP0EN_SHIFT)
#  define PWR_M4MEMLPCTRL_MEM_RETSTP0EN_CHIP_DISABLE (0x0 << PWR_M4MEMLPCTRL_MEM_RETSTP0EN_SHIFT) /* 00: Chip disable mode */
#  define PWR_M4MEMLPCTRL_MEM_RETSTP0EN_PRECHARGE (0x1 << PWR_M4MEMLPCTRL_MEM_RETSTP0EN_SHIFT)    /* 01: Precharge mode */
#  define PWR_M4MEMLPCTRL_MEM_RETSTP0EN_RETENTION1 (0x3 << PWR_M4MEMLPCTRL_MEM_RETSTP0EN_SHIFT)   /* 11: Retention 1 mode */

#define PWR_M4MEMLPCTRL_MEM_PGSTP0EN_SHIFT (0) /* Bits 0-1: Memory power gate mode select in STOP0 */
#define PWR_M4MEMLPCTRL_MEM_PGSTP0EN_MASK  (0x3 << PWR_M4MEMLPCTRL_MEM_PGSTP0EN_SHIFT)
#  define PWR_M4MEMLPCTRL_MEM_PGSTP0EN_CHIP_DISABLE (0x0 << PWR_M4MEMLPCTRL_MEM_PGSTP0EN_SHIFT) /* 00: Chip disable mode */
#  define PWR_M4MEMLPCTRL_MEM_PGSTP0EN_PRECHARGE (0x1 << PWR_M4MEMLPCTRL_MEM_PGSTP0EN_SHIFT)    /* 01: Precharge mode */
#  define PWR_M4MEMLPCTRL_MEM_PGSTP0EN_RETENTION1 (0x3 << PWR_M4MEMLPCTRL_MEM_PGSTP0EN_SHIFT)   /* 11: Retention 1 mode */

/* PWR System Memory Low Power Control register (PWR_SYSMEMLPCTRL) */
#define PWR_SYSMEMLPCTRL_PRDY            (1 << 31) /* Bit 31: All enabled system memory power ready flag */
#define PWR_SYSMEMLPCTRL_AHBSRM5S2_RET2N (1 << 26) /* Bit 26: AHBSRM5S2 Retention 2 mode select */
#define PWR_SYSMEMLPCTRL_AHBSRM5S2_RET1N (1 << 25) /* Bit 25: AHBSRM5S2 Retention 1 mode select */
#define PWR_SYSMEMLPCTRL_AHBSRM5S1_RET2N (1 << 24) /* Bit 24: AHBSRM5S1 Retention 2 mode select */
#define PWR_SYSMEMLPCTRL_AHBSRM5S1_RET1N (1 << 23) /* Bit 23: AHBSRM5S1 Retention 1 mode select */
#define PWR_SYSMEMLPCTRL_AHBSRM4_RET2N   (1 << 22) /* Bit 22: AHBSRM4 Retention 2 mode select */
#define PWR_SYSMEMLPCTRL_AHBSRM4_RET1N   (1 << 21) /* Bit 21: AHBSRM4 Retention 1 mode select */
#define PWR_SYSMEMLPCTRL_AHBSRM3_RET2N   (1 << 20) /* Bit 20: AHBSRM3 Retention 2 mode select */
#define PWR_SYSMEMLPCTRL_AHBSRM3_RET1N   (1 << 19) /* Bit 19: AHBSRM3 Retention 1 mode select */
#define PWR_SYSMEMLPCTRL_AHBSRM2_RET2N   (1 << 18) /* Bit 18: AHBSRM2 Retention 2 mode select */
#define PWR_SYSMEMLPCTRL_AHBSRM2_RET1N   (1 << 17) /* Bit 17: AHBSRM2 Retention 1 mode select */
#define PWR_SYSMEMLPCTRL_AHBSRM1_RET2N   (1 << 16) /* Bit 16: AHBSRM1 Retention 2 mode select */
#define PWR_SYSMEMLPCTRL_AHBSRM1_RET1N   (1 << 15) /* Bit 15: AHBSRM1 Retention 1 mode select */
#define PWR_SYSMEMLPCTRL_AXISRM_RET2N    (1 << 14) /* Bit 14: AXISRM Retention 2 mode select */
#define PWR_SYSMEMLPCTRL_AXISRM_RET1N    (1 << 13) /* Bit 13: AXISRM Retention 1 mode select */

#define PWR_SYSMEMLPCTRL_MEM_RETSTP0EN_SHIFT (8)   /* Bits 8-9: Memory retention mode select in STOP0 */
#define PWR_SYSMEMLPCTRL_MEM_RETSTP0EN_MASK  (0x3 << PWR_SYSMEMLPCTRL_MEM_RETSTP0EN_SHIFT)
#  define PWR_SYSMEMLPCTRL_MEM_RETSTP0EN_CHIP_DISABLE (0x0 << PWR_SYSMEMLPCTRL_MEM_RETSTP0EN_SHIFT) /* 00: Chip disable mode */
#  define PWR_SYSMEMLPCTRL_MEM_RETSTP0EN_PRECHARGE (0x1 << PWR_SYSMEMLPCTRL_MEM_RETSTP0EN_SHIFT)    /* 01: Precharge mode */
#  define PWR_SYSMEMLPCTRL_MEM_RETSTP0EN_RETENTION1 (0x3 << PWR_SYSMEMLPCTRL_MEM_RETSTP0EN_SHIFT)   /* 11: Retention 1 mode */

#define PWR_SYSMEMLPCTRL_MEM_PGSTP0EN_SHIFT (8)    /* Bits 8-9: Memory power gate mode select in STOP0 */
#define PWR_SYSMEMLPCTRL_MEM_PGSTP0EN_MASK  (0x3 << PWR_SYSMEMLPCTRL_MEM_PGSTP0EN_SHIFT)
#  define PWR_SYSMEMLPCTRL_MEM_PGSTP0EN_CHIP_DISABLE (0x0 << PWR_SYSMEMLPCTRL_MEM_PGSTP0EN_SHIFT) /* 00: Chip disable mode */
#  define PWR_SYSMEMLPCTRL_MEM_PGSTP0EN_PRECHARGE (0x1 << PWR_SYSMEMLPCTRL_MEM_PGSTP0EN_SHIFT)    /* 01: Precharge mode */
#  define PWR_SYSMEMLPCTRL_MEM_PGSTP0EN_RETENTION1 (0x3 << PWR_SYSMEMLPCTRL_MEM_PGSTP0EN_SHIFT)   /* 11: Retention 1 mode */

#define PWR_SYSMEMLPCTRL_AHBSRM5S2_PG    (1 << 6)  /* Bit 6: AHBSRM5S2 power gate enable */
#define PWR_SYSMEMLPCTRL_AHBSRM5S1_PG    (1 << 5)  /* Bit 5: AHBSRM5S1 power gate enable */
#define PWR_SYSMEMLPCTRL_AHBSRM4_PG      (1 << 4)  /* Bit 4: AHBSRM4 power gate enable */
#define PWR_SYSMEMLPCTRL_AHBSRM3_PG      (1 << 3)  /* Bit 3: AHBSRM3 power gate enable */
#define PWR_SYSMEMLPCTRL_AHBSRM2_PG      (1 << 2)  /* Bit 2: AHBSRM2 power gate enable */
#define PWR_SYSMEMLPCTRL_AHBSRM1_PG      (1 << 1)  /* Bit 1: AHBSRM1 power gate enable */
#define PWR_SYSMEMLPCTRL_AXISRM_PG       (1 << 0)  /* Bit 0: AXISRM power gate enable */

/* PWR System SHRTIM Power Control register (PWR_SHRTIMCTRL) */
#define PWR_SHRTIMCTRL_SHRA_PSWACK1      (1 << 20) /* Bit 20: SHRTIM AFE power switch ACK1 */
#define PWR_SHRTIMCTRL_SHRA_PWRRDY       (1 << 19) /* Bit 19: SHRTIM AFE power domain ready flag */
#define PWR_SHRTIMCTRL_SHRA_ISNEN        (1 << 18) /* Bit 18: SHRTIM AFE power domain isolation signal */
#define PWR_SHRTIMCTRL_SHRA_FUCEN        (1 << 17) /* Bit 17: SHRTIM AFE function mode enable */
#define PWR_SHRTIMCTRL_SHRA_PGEN         (1 << 16) /* Bit 16: SHRTIM AFE power gate enable */

#define PWR_SHRTIMCTRL_SHR2_PSWACK1      (1 << 12) /* Bit 12: SHRTIM2 power switch ACK1 */
#define PWR_SHRTIMCTRL_SHR2_PWRRDY       (1 << 11) /* Bit 11: SHRTIM2 power domain ready flag */
#define PWR_SHRTIMCTRL_SHR2_ISNEN        (1 << 10) /* Bit 10: SHRTIM2 power domain isolation signal */
#define PWR_SHRTIMCTRL_SHR2_FUCEN        (1 << 9)  /* Bit 9: SHRTIM2 function mode enable */
#define PWR_SHRTIMCTRL_SHR2_PGEN         (1 << 8)  /* Bit 8: SHRTIM2 power gate enable */

#define PWR_SHRTIMCTRL_SHR1_PSWACK1      (1 << 4)  /* Bit 4: SHRTIM1 power switch ACK1 */
#define PWR_SHRTIMCTRL_SHR1_PWRRDY       (1 << 3)  /* Bit 3: SHRTIM1 power domain ready flag */
#define PWR_SHRTIMCTRL_SHR1_ISNEN        (1 << 2)  /* Bit 2: SHRTIM1 power domain isolation signal */
#define PWR_SHRTIMCTRL_SHR1_FUCEN        (1 << 1)  /* Bit 1: SHRTIM1 function mode enable */
#define PWR_SHRTIMCTRL_SHR1_PGEN         (1 << 0)  /* Bit 0: SHRTIM1 power gate enable */

/* PWR System MDMA Power Control register (PWR_MDMACTRL) */
#define PWR_MDMACTRL_MDMA_PSWACK1       (1 << 4)  /* Bit 4: MDMA power switch ACK1 */
#define PWR_MDMACTRL_MDMA_PWRRDY        (1 << 3)  /* Bit 3: MDMA power domain ready flag */
#define PWR_MDMACTRL_MDMA_ISNEN         (1 << 2)  /* Bit 2: MDMA power domain isolation signal */
#define PWR_MDMACTRL_MDMA_FUCEN         (1 << 1)  /* Bit 1: MDMA function mode enable */
#define PWR_MDMACTRL_MDMA_PGEN          (1 << 0)  /* Bit 0: MDMA power gate enable */

/* PWR System ESC Power Control register (PWR_ESCCTRL) */
#define PWR_ESCCTRL_ESC_PSWACK1         (1 << 4)  /* Bit 4: ESC power switch ACK1 */
#define PWR_ESCCTRL_ESC_PWRRDY          (1 << 3)  /* Bit 3: ESC power domain ready flag */
#define PWR_ESCCTRL_ESC_ISNEN           (1 << 2)  /* Bit 2: ESC power domain isolation signal */
#define PWR_ESCCTRL_ESC_FUCEN           (1 << 1)  /* Bit 1: ESC function mode enable */
#define PWR_ESCCTRL_ESC_PGEN            (1 << 0)  /* Bit 0: ESC power gate enable */

/* PWR EMC RET Control Register 1 (PWR_EMCRETCTRL1) */
#define PWR_EMCRETCTRL1_RETGB3DET        (1 << 11) /* Bit 11: RET domain EMC GB3 detection enable */
#define PWR_EMCRETCTRL1_RETGB2DET        (1 << 10) /* Bit 10: RET domain EMC GB2 detection enable */
#define PWR_EMCRETCTRL1_RETGB1DET        (1 << 9)  /* Bit 9: RET domain EMC GB1 detection enable */
#define PWR_EMCRETCTRL1_RETGB0DET        (1 << 8)  /* Bit 8: RET domain EMC GB0 detection enable */
#define PWR_EMCRETCTRL1_RETGBN3DET       (1 << 7)  /* Bit 7: RET domain EMC GBN3 detection enable */
#define PWR_EMCRETCTRL1_RETGBN2DET       (1 << 6)  /* Bit 6: RET domain EMC GBN2 detection enable */
#define PWR_EMCRETCTRL1_RETGBN1DET       (1 << 5)  /* Bit 5: RET domain EMC GBN1 detection enable */
#define PWR_EMCRETCTRL1_RETGBN0DET       (1 << 4)  /* Bit 4: RET domain EMC GBN0 detection enable */
#define PWR_EMCRETCTRL1_RETCLP3DET       (1 << 3)  /* Bit 3: RET domain EMC Clamp3 detection enable */
#define PWR_EMCRETCTRL1_RETCLP2DET       (1 << 2)  /* Bit 2: RET domain EMC Clamp2 detection enable */
#define PWR_EMCRETCTRL1_RETCLP1DET       (1 << 1)  /* Bit 1: RET domain EMC Clamp1 detection enable */
#define PWR_EMCRETCTRL1_RETCLP0DET       (1 << 0)  /* Bit 0: RET domain EMC Clamp0 detection enable */

/* PWR EMC RET Status Register 1 (PWR_EMCRETSTS1) */
#define PWR_EMCRETSTS1_EMCFCLR          (1 << 24) /* Bit 24: Clear EMC Flag */
#define PWR_EMCRETSTS1_GB3F             (1 << 11) /* Bit 11: RET Domain EMC GB3 Flag */
#define PWR_EMCRETSTS1_GB2F             (1 << 10) /* Bit 10: RET Domain EMC GB2 Flag */
#define PWR_EMCRETSTS1_GB1F             (1 << 9)  /* Bit 9: RET Domain EMC GB1 Flag */
#define PWR_EMCRETSTS1_GB0F             (1 << 8)  /* Bit 8: RET Domain EMC GB0 Flag */
#define PWR_EMCRETSTS1_GBN3F            (1 << 7)  /* Bit 7: RET Domain EMC GBN3 Flag */
#define PWR_EMCRETSTS1_GBN2F            (1 << 6)  /* Bit 6: RET Domain EMC GBN2 Flag */
#define PWR_EMCRETSTS1_GBN1F            (1 << 5)  /* Bit 5: RET Domain EMC GBN1 Flag */
#define PWR_EMCRETSTS1_GBN0F            (1 << 4)  /* Bit 4: RET Domain EMC GBN0 Flag */
#define PWR_EMCRETSTS1_CLP3F            (1 << 3)  /* Bit 3: RET Domain EMC Clamp3 Flag */
#define PWR_EMCRETSTS1_CLP2F            (1 << 2)  /* Bit 2: RET Domain EMC Clamp2 Flag */
#define PWR_EMCRETSTS1_CLP1F            (1 << 1)  /* Bit 1: RET Domain EMC Clamp1 Flag */
#define PWR_EMCRETSTS1_CLP0F            (1 << 0)  /* Bit 0: RET Domain EMC Clamp0 Flag */

/* PWR EMC RET Control Register 2 (PWR_EMCRETCTRL2) */
#define PWR_EMCRETCTRL2_RETGB7DET       (1 << 11) /* Bit 11: RET domain EMC GB7 detection enable */
#define PWR_EMCRETCTRL2_RETGB6DET       (1 << 10) /* Bit 10: RET domain EMC GB6 detection enable */
#define PWR_EMCRETCTRL2_RETGB5DET       (1 << 9)  /* Bit 9: RET domain EMC GB5 detection enable */
#define PWR_EMCRETCTRL2_RETGB4DET       (1 << 8)  /* Bit 8: RET domain EMC GB4 detection enable */
#define PWR_EMCRETCTRL2_RETGBN7DET      (1 << 7)  /* Bit 7: RET domain EMC GBN7 detection enable */
#define PWR_EMCRETCTRL2_RETGBN6DET      (1 << 6)  /* Bit 6: RET domain EMC GBN6 detection enable */
#define PWR_EMCRETCTRL2_RETGBN5DET      (1 << 5)  /* Bit 5: RET domain EMC GBN5 detection enable */
#define PWR_EMCRETCTRL2_RETGBN4DET      (1 << 4)  /* Bit 4: RET domain EMC GBN4 detection enable */
#define PWR_EMCRETCTRL2_RETCLP7DET      (1 << 3)  /* Bit 3: RET domain EMC Clamp7 detection enable */
#define PWR_EMCRETCTRL2_RETCLP6DET      (1 << 2)  /* Bit 2: RET domain EMC Clamp6 detection enable */
#define PWR_EMCRETCTRL2_RETCLP5DET      (1 << 1)  /* Bit 1: RET domain EMC Clamp5 detection enable */
#define PWR_EMCRETCTRL2_RETCLP4DET      (1 << 0)  /* Bit 0: RET domain EMC Clamp4 detection enable */

/* PWR EMC RET Status Register 2 (PWR_EMCRETSTS2) */
#define PWR_EMCRETSTS2_GB7F             (1 << 11) /* Bit 11: RET Domain EMC GB7 Flag */
#define PWR_EMCRETSTS2_GB6F             (1 << 10) /* Bit 10: RET Domain EMC GB6 Flag */
#define PWR_EMCRETSTS2_GB5F             (1 << 9)  /* Bit 9: RET Domain EMC GB5 Flag */
#define PWR_EMCRETSTS2_GB4F             (1 << 8)  /* Bit 8: RET Domain EMC GB4 Flag */
#define PWR_EMCRETSTS2_GBN7F            (1 << 7)  /* Bit 7: RET Domain EMC GBN7 Flag */
#define PWR_EMCRETSTS2_GBN6F            (1 << 6)  /* Bit 6: RET Domain EMC GBN6 Flag */
#define PWR_EMCRETSTS2_GBN5F            (1 << 5)  /* Bit 5: RET Domain EMC GBN5 Flag */
#define PWR_EMCRETSTS2_GBN4F            (1 << 4)  /* Bit 4: RET Domain EMC GBN4 Flag */
#define PWR_EMCRETSTS2_CLP7F            (1 << 3)  /* Bit 3: RET Domain EMC Clamp7 Flag */
#define PWR_EMCRETSTS2_CLP6F            (1 << 2)  /* Bit 2: RET Domain EMC Clamp6 Flag */
#define PWR_EMCRETSTS2_CLP5F            (1 << 1)  /* Bit 1: RET Domain EMC Clamp5 Flag */
#define PWR_EMCRETSTS2_CLP4F            (1 << 0)  /* Bit 0: RET Domain EMC Clamp4 Flag */

/* PWR EMC RET Control Register 3 (PWR_EMCRETCTRL3) */
#define PWR_EMCRETCTRL3_RETGB11DET      (1 << 11) /* Bit 11: RET domain EMC GB11 detection enable */
#define PWR_EMCRETCTRL3_RETGB10DET      (1 << 10) /* Bit 10: RET domain EMC GB10 detection enable */
#define PWR_EMCRETCTRL3_RETGB9DET       (1 << 9)  /* Bit 9: RET domain EMC GB9 detection enable */
#define PWR_EMCRETCTRL3_RETGB8DET       (1 << 8)  /* Bit 8: RET domain EMC GB8 detection enable */
#define PWR_EMCRETCTRL3_RETGBN11DET     (1 << 7)  /* Bit 7: RET domain EMC GBN11 detection enable */
#define PWR_EMCRETCTRL3_RETGBN10DET     (1 << 6)  /* Bit 6: RET domain EMC GBN10 detection enable */
#define PWR_EMCRETCTRL3_RETGBN9DET      (1 << 5)  /* Bit 5: RET domain EMC GBN9 detection enable */
#define PWR_EMCRETCTRL3_RETGBN8DET      (1 << 4)  /* Bit 4: RET domain EMC GBN8 detection enable */
#define PWR_EMCRETCTRL3_RETCLP11DET     (1 << 3)  /* Bit 3: RET domain EMC Clamp11 detection enable */
#define PWR_EMCRETCTRL3_RETCLP10DET     (1 << 2)  /* Bit 2: RET domain EMC Clamp10 detection enable */
#define PWR_EMCRETCTRL3_RETCLP9DET      (1 << 1)  /* Bit 1: RET domain EMC Clamp9 detection enable */
#define PWR_EMCRETCTRL3_RETCLP8DET      (1 << 0)  /* Bit 0: RET domain EMC Clamp8 detection enable */

/* PWR EMC RET Status Register 3 (PWR_EMCRETSTS3) */
#define PWR_EMCRETSTS3_GB11F            (1 << 11) /* Bit 11: RET Domain EMC GB11 Flag */
#define PWR_EMCRETSTS3_GB10F            (1 << 10) /* Bit 10: RET Domain EMC GB10 Flag */
#define PWR_EMCRETSTS3_GB9F             (1 << 9)  /* Bit 9: RET Domain EMC GB9 Flag */
#define PWR_EMCRETSTS3_GB8F             (1 << 8)  /* Bit 8: RET Domain EMC GB8 Flag */
#define PWR_EMCRETSTS3_GBN11F           (1 << 7)  /* Bit 7: RET Domain EMC GBN11 Flag */
#define PWR_EMCRETSTS3_GBN10F           (1 << 6)  /* Bit 6: RET Domain EMC GBN10 Flag */
#define PWR_EMCRETSTS3_GBN9F            (1 << 5)  /* Bit 5: RET Domain EMC GBN9 Flag */
#define PWR_EMCRETSTS3_GBN8F            (1 << 4)  /* Bit 4: RET Domain EMC GBN8 Flag */
#define PWR_EMCRETSTS3_CLP11F           (1 << 3)  /* Bit 3: RET Domain EMC Clamp11 Flag */
#define PWR_EMCRETSTS3_CLP10F           (1 << 2)  /* Bit 2: RET Domain EMC Clamp10 Flag */
#define PWR_EMCRETSTS3_CLP9F            (1 << 1)  /* Bit 1: RET Domain EMC Clamp9 Flag */
#define PWR_EMCRETSTS3_CLP8F            (1 << 0)  /* Bit 0: RET Domain EMC Clamp8 Flag */

/* PWR EMC RET Control Register 4 (PWR_EMCRETCTRL4) */
#define PWR_EMCRETCTRL4_RETGB14DET      (1 << 10) /* Bit 10: RET domain EMC GB14 detection enable */
#define PWR_EMCRETCTRL4_RETGB13DET      (1 << 9)  /* Bit 9: RET domain EMC GB13 detection enable */
#define PWR_EMCRETCTRL4_RETGB12DET      (1 << 8)  /* Bit 8: RET domain EMC GB12 detection enable */
#define PWR_EMCRETCTRL4_RETGBN14DET     (1 << 6)  /* Bit 6: RET domain EMC GBN14 detection enable */
#define PWR_EMCRETCTRL4_RETGBN13DET     (1 << 5)  /* Bit 5: RET domain EMC GBN13 detection enable */
#define PWR_EMCRETCTRL4_RETGBN12DET     (1 << 4)  /* Bit 4: RET domain EMC GBN12 detection enable */
#define PWR_EMCRETCTRL4_RETCLP14DET     (1 << 2)  /* Bit 2: RET domain EMC Clamp14 detection enable */
#define PWR_EMCRETCTRL4_RETCLP13DET     (1 << 1)  /* Bit 1: RET domain EMC Clamp13 detection enable */
#define PWR_EMCRETCTRL4_RETCLP12DET     (1 << 0)  /* Bit 0: RET domain EMC Clamp12 detection enable */

/* PWR EMC RET Status Register 4 (PWR_EMCRETSTS4) */
#define PWR_EMCRETSTS4_GB14F            (1 << 10) /* Bit 10: RET Domain EMC GB14 Flag */
#define PWR_EMCRETSTS4_GB13F            (1 << 9)  /* Bit 9: RET Domain EMC GB13 Flag */
#define PWR_EMCRETSTS4_GB12F            (1 << 8)  /* Bit 8: RET Domain EMC GB12 Flag */
#define PWR_EMCRETSTS4_GBN14F           (1 << 6)  /* Bit 6: RET Domain EMC GBN14 Flag */
#define PWR_EMCRETSTS4_GBN13F           (1 << 5)  /* Bit 5: RET Domain EMC GBN13 Flag */
#define PWR_EMCRETSTS4_GBN12F           (1 << 4)  /* Bit 4: RET Domain EMC GBN12 Flag */
#define PWR_EMCRETSTS4_CLP14F           (1 << 2)  /* Bit 2: RET Domain EMC Clamp14 Flag */
#define PWR_EMCRETSTS4_CLP13F           (1 << 1)  /* Bit 1: RET Domain EMC Clamp13 Flag */
#define PWR_EMCRETSTS4_CLP12F           (1 << 0)  /* Bit 0: RET Domain EMC Clamp12 Flag */

/* PWR EMC BKP Control Register (PWR_EMCBKPCTRL) */
#define PWR_EMCBKPCTRL_BKPGBDET         (1 << 8)  /* Bit 8: Backup domain EMC GB detection enable */
#define PWR_EMCBKPCTRL_BKPGBNDET        (1 << 4)  /* Bit 4: Backup domain EMC GBN detection enable */
#define PWR_EMCBKPCTRL_BKPCLPDET        (1 << 0)  /* Bit 0: Backup domain EMC Clamp detection enable */

/* PWR EMC BKP Status Register (PWR_EMCRETSTS) */
#define PWR_EMCRETSTS_BKPGBF            (1 << 8)  /* Bit 8: Backup Domain EMC GB Flag */
#define PWR_EMCRETSTS_BKPGBNF           (1 << 4)  /* Bit 4: Backup Domain EMC GBN Flag */
#define PWR_EMCRETSTS_BKPCLPF           (1 << 0)  /* Bit 0: Backup Domain EMC Clamp Flag */

#endif /* __ARCH_ARM_SRC_N32H7_HARDWARE_N32H76X_PWR_H */
