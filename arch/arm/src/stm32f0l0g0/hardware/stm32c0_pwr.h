/****************************************************************************
 * arch/arm/src/stm32f0l0g0/hardware/stm32c0_pwr.h
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

#ifndef __ARCH_ARM_SRC_STM32F0L0G0_HARDWARE_STM32C0_PWR_H
#define __ARCH_ARM_SRC_STM32F0L0G0_HARDWARE_STM32C0_PWR_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#undef HAVE_PWR_DBP /* No Disable backup write protection bit */

/* Register Offsets *********************************************************/

#define STM32_PWR_CR1_OFFSET   0x0000  /* Power control register 1 */
#define STM32_PWR_CR2_OFFSET   0x0004  /* Power control register 2 */
#define STM32_PWR_CR3_OFFSET   0x0008  /* Power control register 3 */
#define STM32_PWR_CR4_OFFSET   0x000C  /* Power control register 4 */
#define STM32_PWR_SR1_OFFSET   0x0010  /* Power status register 1 */
#define STM32_PWR_SR2_OFFSET   0x0014  /* Power status register 2 */
#define STM32_PWR_SCR_OFFSET   0x0018  /* Power status clear register */
#define STM32_PWR_PUCRA_OFFSET 0x0020  /* Power Port A pull-up control register */
#define STM32_PWR_PDCRA_OFFSET 0x0024  /* Power Port A pull-down control register */
#define STM32_PWR_PUCRB_OFFSET 0x0028  /* Power Port B pull-up control register */
#define STM32_PWR_PDCRB_OFFSET 0x002C  /* Power Port B pull-down control register */
#define STM32_PWR_PUCRC_OFFSET 0x0030  /* Power Port C pull-up control register */
#define STM32_PWR_PDCRC_OFFSET 0x0034  /* Power Port C pull-down control register */
#define STM32_PWR_PUCRD_OFFSET 0x0038  /* Power Port D pull-up control register */
#define STM32_PWR_PDCRD_OFFSET 0x003C  /* Power Port D pull-down control register */
#define STM32_PWR_PUCRF_OFFSET 0x0048  /* Power Port F pull-up control register */
#define STM32_PWR_PDCRF_OFFSET 0x004C  /* Power Port F pull-down control register */
#define STM32_PWR_PUCRG_OFFSET 0x0050  /* Power Port G pull-up control register */
#define STM32_PWR_PDCRG_OFFSET 0x0054  /* Power Port G pull-down control register */
#define STM32_PWR_PUCRH_OFFSET 0x0058  /* Power Port H pull-up control register */
#define STM32_PWR_PDCRH_OFFSET 0x005C  /* Power Port H pull-down control register */
#define STM32_PWR_PUCRI_OFFSET 0x0060  /* Power Port I pull-up control register */
#define STM32_PWR_PDCRI_OFFSET 0x0064  /* Power Port I pull-down control register */

/* Register Addresses *******************************************************/

#define STM32_PWR_CR1          (STM32_PWR_BASE+STM32_PWR_CR1_OFFSET)
#define STM32_PWR_CR2          (STM32_PWR_BASE+STM32_PWR_CR2_OFFSET)
#define STM32_PWR_CR3          (STM32_PWR_BASE+STM32_PWR_CR3_OFFSET)
#define STM32_PWR_CR4          (STM32_PWR_BASE+STM32_PWR_CR4_OFFSET)
#define STM32_PWR_SR1          (STM32_PWR_BASE+STM32_PWR_SR1_OFFSET)
#define STM32_PWR_SR2          (STM32_PWR_BASE+STM32_PWR_SR2_OFFSET)
#define STM32_PWR_SCR          (STM32_PWR_BASE+STM32_PWR_SCR_OFFSET)
#define STM32_PWR_PUCRA        (STM32_PWR_BASE+STM32_PWR_PUCRA_OFFSET)
#define STM32_PWR_PDCRA        (STM32_PWR_BASE+STM32_PWR_PDCRA_OFFSET)
#define STM32_PWR_PUCRB        (STM32_PWR_BASE+STM32_PWR_PUCRB_OFFSET)
#define STM32_PWR_PDCRB        (STM32_PWR_BASE+STM32_PWR_PDCRB_OFFSET)
#define STM32_PWR_PUCRC        (STM32_PWR_BASE+STM32_PWR_PUCRC_OFFSET)
#define STM32_PWR_PDCRC        (STM32_PWR_BASE+STM32_PWR_PDCRC_OFFSET)
#define STM32_PWR_PUCRD        (STM32_PWR_BASE+STM32_PWR_PUCRD_OFFSET)
#define STM32_PWR_PDCRD        (STM32_PWR_BASE+STM32_PWR_PDCRD_OFFSET)
#define STM32_PWR_PUCRE        (STM32_PWR_BASE+STM32_PWR_PUCRE_OFFSET)
#define STM32_PWR_PDCRE        (STM32_PWR_BASE+STM32_PWR_PDCRE_OFFSET)
#define STM32_PWR_PUCRF        (STM32_PWR_BASE+STM32_PWR_PUCRF_OFFSET)
#define STM32_PWR_PDCRF        (STM32_PWR_BASE+STM32_PWR_PDCRF_OFFSET)
#define STM32_PWR_PUCRG        (STM32_PWR_BASE+STM32_PWR_PUCRG_OFFSET)
#define STM32_PWR_PDCRG        (STM32_PWR_BASE+STM32_PWR_PDCRG_OFFSET)
#define STM32_PWR_PUCRH        (STM32_PWR_BASE+STM32_PWR_PUCRH_OFFSET)
#define STM32_PWR_PDCRH        (STM32_PWR_BASE+STM32_PWR_PDCRH_OFFSET)
#define STM32_PWR_PUCRI        (STM32_PWR_BASE+STM32_PWR_PUCRI_OFFSET)
#define STM32_PWR_PDCRI        (STM32_PWR_BASE+STM32_PWR_PDCRI_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* Power control register 1 */

#define PWR_CR1_LPMS_SHIFT       (0)      /* Bits 0-2: Low-power mode selection */
#define PWR_CR1_LPMS_MASK        (7 << PWR_CR1_LPMS_SHIFT)
#  define PWR_CR1_LPMS_STOP1MR   (0 << PWR_CR1_LPMS_SHIFT) /* Stop 1 mode with main regulator (MR) */
#  define PWR_CR1_LPMS_STOP1LPR  (1 << PWR_CR1_LPMS_SHIFT) /* Stop 1 mode with low-power regulator (LPR) */
#  define PWR_CR1_LPMS_STOP2     (2 << PWR_CR1_LPMS_SHIFT) /* 010: Stop 2 mode */
#  define PWR_CR1_LPMS_STANDBY   (3 << PWR_CR1_LPMS_SHIFT) /* 011: Standby mode */
#  define PWR_CR1_LPMS_SHUTDOWN  (4 << PWR_CR1_LPMS_SHIFT) /* 1xx: Shutdown mode */

                                          /* Bit 4: Reserved */
#define PWR_CR1_FPDLPSLP         (1 << 5) /* Bit 5: Flash memory powered down during Low-power sleep mode */
                                          /* Bits 6-31: Reserved */

/* Power control register 2 */

#define PWR_CR2_VDDIO2_SHIFT      (8)      /* Bits 8-9: VDDIO2 supply voltage monitoring */
#define PWR_CR2_VDDIO2_MASK       (3 << PWR_CR2_PLS_SHIFT)

/* Power control register 3 */

#define PWR_CR3_EWUP1            (1 << 0)  /* Bit 0: Enable Wakeup pin WKUP1 */
#define PWR_CR3_EWUP2            (1 << 1)  /* Bit 1: Enable Wakeup pin WKUP2 */
#define PWR_CR3_EWUP3            (1 << 2)  /* Bit 2: Enable Wakeup pin WKUP3 */
#define PWR_CR3_EWUP4            (1 << 3)  /* Bit 3: Enable Wakeup pin WKUP4 */
#define PWR_CR3_EWUP5            (1 << 4)  /* Bit 4: Enable Wakeup pin WKUP5 */
#define PWR_CR3_EWUP6            (1 << 5)  /* Bit 5: Enable Wakeup pin WKUP6 */
#define PWR_CR3_APC              (1 << 10) /* Bit 10: Apply pull-up and pull-down configuration */
#define PWR_CR3_EIWUL            (1 << 15) /* Bit 15: Enable internal wakeup line */

/* Power control register 4 */

#define PWR_CR4_WP1              (1 << 0) /* Bit 0: Wakeup pin WKUP1 polarity */
#define PWR_CR4_WP2              (1 << 1) /* Bit 1: Wakeup pin WKUP2 polarity */
#define PWR_CR4_WP3              (1 << 2) /* Bit 2: Wakeup pin WKUP3 polarity */
#define PWR_CR4_WP4              (1 << 3) /* Bit 3: Wakeup pin WKUP4 polarity */
#define PWR_CR4_WP5              (1 << 4) /* Bit 4: Wakeup pin WKUP5 polarity */
#define PWR_CR4_WP6              (1 << 5) /* Bit 5: Wakeup pin WKUP6 polarity */

/* Power status register 1 */

#define PWR_SR1_WUF1             (1 << 0)  /* Bit 0: Wakeup flag 1 */
#define PWR_SR1_WUF2             (1 << 1)  /* Bit 1: Wakeup flag 2 */
#define PWR_SR1_WUF3             (1 << 2)  /* Bit 2: Wakeup flag 3 */
#define PWR_SR1_WUF4             (1 << 3)  /* Bit 3: Wakeup flag 4 */
#define PWR_SR1_WUF5             (1 << 4)  /* Bit 4: Wakeup flag 5 */
#define PWR_SR1_WUF6             (1 << 5)  /* Bit 5: Wakeup flag 6 */
#define PWR_SR1_SBF              (1 << 8)  /* Bit 8: Standby flag */
#define PWR_SR1_WUFI             (1 << 15) /* Bit 15: Wakeup internal flag */

/* Power status register 2 */

#define PWR_SR2_FLASHRDY         (1 << 7)  /* Bit 7: Flash ready flag */
#define PWR_SR2_VDDIO2           (1 << 13) /* Bit 13: VDDIO2 supply voltage monitoring output flag */

/* Power status clear register */

#define PWR_SCR_CWUF1            (1 << 0) /* Bit 0: Clear wakeup flag 1 */
#define PWR_SCR_CWUF2            (1 << 1) /* Bit 1: Clear wakeup flag 2 */
#define PWR_SCR_CWUF3            (1 << 2) /* Bit 2: Clear wakeup flag 3 */
#define PWR_SCR_CWUF4            (1 << 3) /* Bit 3: Clear wakeup flag 4 */
#define PWR_SCR_CWUF5            (1 << 4) /* Bit 4: Clear wakeup flag 5 */
#define PWR_SCR_CSBF             (1 << 8) /* Bit 8: Clear standby flag */

#endif /* __ARCH_ARM_SRC_STM32F0L0G0_HARDWARE_STM32C0_PWR_H */
