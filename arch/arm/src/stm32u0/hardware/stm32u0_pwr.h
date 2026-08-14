/****************************************************************************
 * arch/arm/src/stm32u0/hardware/stm32u0_pwr.h
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

#ifndef __ARCH_ARM_SRC_STM32U0_HARDWARE_STM32U0_PWR_H
#define __ARCH_ARM_SRC_STM32U0_HARDWARE_STM32U0_PWR_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define HAVE_PWR_DBP            1

/* Register Offsets *********************************************************/

#define STM32_PWR_CR1_OFFSET    0x0000 /* Power control register 1 */
#define STM32_PWR_CR2_OFFSET    0x0004 /* Power control register 2 */
#define STM32_PWR_CR3_OFFSET    0x0008 /* Power control register 3 */
#define STM32_PWR_CR4_OFFSET    0x000C /* Power control register 4 */
#define STM32_PWR_SR1_OFFSET    0x0010 /* Power status register 1 */
#define STM32_PWR_SR2_OFFSET    0x0014 /* Power status register 2 */
#define STM32_PWR_SCR_OFFSET    0x0018 /* Power status clear register */
#define STM32_PWR_PUCRA_OFFSET  0x0020 /* Port A pull-up control register */
#define STM32_PWR_PDCRA_OFFSET  0x0024 /* Port A pull-down control register */
#define STM32_PWR_PUCRB_OFFSET  0x0028 /* Port B pull-up control register */
#define STM32_PWR_PDCRB_OFFSET  0x002C /* Port B pull-down control register */
#define STM32_PWR_PUCRC_OFFSET  0x0030 /* Port C pull-up control register */
#define STM32_PWR_PDCRC_OFFSET  0x0034 /* Port C pull-down control register */
#define STM32_PWR_PUCRD_OFFSET  0x0038 /* Port D pull-up control register */
#define STM32_PWR_PDCRD_OFFSET  0x003C /* Port D pull-down control register */
#define STM32_PWR_PUCRE_OFFSET  0x0040 /* Port E pull-up control register */
#define STM32_PWR_PDCRE_OFFSET  0x0044 /* Port E pull-down control register */
#define STM32_PWR_PUCRF_OFFSET  0x0048 /* Port F pull-up control register */
#define STM32_PWR_PDCRF_OFFSET  0x004C /* Port F pull-down control register */

/* Register Addresses *******************************************************/

#define STM32_PWR_CR1           (STM32_PWR_BASE+STM32_PWR_CR1_OFFSET)
#define STM32_PWR_CR2           (STM32_PWR_BASE+STM32_PWR_CR2_OFFSET)
#define STM32_PWR_CR3           (STM32_PWR_BASE+STM32_PWR_CR3_OFFSET)
#define STM32_PWR_CR4           (STM32_PWR_BASE+STM32_PWR_CR4_OFFSET)
#define STM32_PWR_SR1           (STM32_PWR_BASE+STM32_PWR_SR1_OFFSET)
#define STM32_PWR_SR2           (STM32_PWR_BASE+STM32_PWR_SR2_OFFSET)
#define STM32_PWR_SCR           (STM32_PWR_BASE+STM32_PWR_SCR_OFFSET)
#define STM32_PWR_PUCRA         (STM32_PWR_BASE+STM32_PWR_PUCRA_OFFSET)
#define STM32_PWR_PDCRA         (STM32_PWR_BASE+STM32_PWR_PDCRA_OFFSET)
#define STM32_PWR_PUCRB         (STM32_PWR_BASE+STM32_PWR_PUCRB_OFFSET)
#define STM32_PWR_PDCRB         (STM32_PWR_BASE+STM32_PWR_PDCRB_OFFSET)
#define STM32_PWR_PUCRC         (STM32_PWR_BASE+STM32_PWR_PUCRC_OFFSET)
#define STM32_PWR_PDCRC         (STM32_PWR_BASE+STM32_PWR_PDCRC_OFFSET)
#define STM32_PWR_PUCRD         (STM32_PWR_BASE+STM32_PWR_PUCRD_OFFSET)
#define STM32_PWR_PDCRD         (STM32_PWR_BASE+STM32_PWR_PDCRD_OFFSET)
#define STM32_PWR_PUCRE         (STM32_PWR_BASE+STM32_PWR_PUCRE_OFFSET)
#define STM32_PWR_PDCRE         (STM32_PWR_BASE+STM32_PWR_PDCRE_OFFSET)
#define STM32_PWR_PUCRF         (STM32_PWR_BASE+STM32_PWR_PUCRF_OFFSET)
#define STM32_PWR_PDCRF         (STM32_PWR_BASE+STM32_PWR_PDCRF_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* Power control register 1 */

#define PWR_CR1_LPMS_SHIFT      (0) /* Bits 0-2: Low-power mode */
#define PWR_CR1_LPMS_MASK       (7 << PWR_CR1_LPMS_SHIFT)
#  define PWR_CR1_LPMS_STOP0    (0 << PWR_CR1_LPMS_SHIFT) /* 000: Stop 0 */
#  define PWR_CR1_LPMS_STOP1    (1 << PWR_CR1_LPMS_SHIFT) /* 001: Stop 1 */
#  define PWR_CR1_LPMS_STOP2    (2 << PWR_CR1_LPMS_SHIFT) /* 010: Stop 2 */
#  define PWR_CR1_LPMS_STANDBY  (3 << PWR_CR1_LPMS_SHIFT) /* 011: Standby */
#  define PWR_CR1_LPMS_SHUTDOWN (4 << PWR_CR1_LPMS_SHIFT) /* 1xx: Shutdown */
#define PWR_CR1_FPDSTOP         (1 << 3)                  /* Bit 3: Flash PD in Stop mode */
#define PWR_CR1_FPDLPRUN        (1 << 4)                  /* Bit 4: Flash PD in LP run */
#define PWR_CR1_FPDLPSLP        (1 << 5)                  /* Bit 5: Flash PD in LP sleep */
#define PWR_CR1_DBP             (1 << 8)                  /* Bit 8: Disable backup domain write protection */
#define PWR_CR1_VOS_SHIFT       (9)                       /* Bits 9-10: Voltage scaling range selection */
#define PWR_CR1_VOS_MASK        (3 << PWR_CR1_VOS_SHIFT)
#  define PWR_CR1_VOS_RANGE1    (1 << PWR_CR1_VOS_SHIFT) /* 01: Range 1 */
#  define PWR_CR1_VOS_RANGE2    (2 << PWR_CR1_VOS_SHIFT) /* 10: Range 2 */
#define PWR_CR1_LPR             (1 << 14)                /* Bit 14: Low-power run */

/* Power control register 2 */

#define PWR_CR2_PVDE            (1 << 0) /* Bit 0: PVD enable */
#define PWR_CR2_PLS_SHIFT       (1)      /* Bits 1-3: PVD level */
#define PWR_CR2_PLS_MASK        (7 << PWR_CR2_PLS_SHIFT)
#  define PWR_CR2_PLS(n)        ((n) << PWR_CR2_PLS_SHIFT)
#define PWR_CR2_PVME1           (1 << 4)  /* Bit 4: PVM VDDUSB enable */
#define PWR_CR2_PVME3           (1 << 5)  /* Bit 5: PVM VDDADC enable */
#define PWR_CR2_PVME4           (1 << 6)  /* Bit 6: PVM VDDDAC enable */
#define PWR_CR2_USV             (1 << 10) /* Bit 10: VDDUSB supply valid */

/* Power control register 3 */

#define PWR_CR3_EWUP1           (1 << 0)  /* Bit 0: Enable WKUP1 pin */
#define PWR_CR3_EWUP2           (1 << 1)  /* Bit 1: Enable WKUP2 pin */
#define PWR_CR3_EWUP3           (1 << 2)  /* Bit 2: Enable WKUP3 pin */
#define PWR_CR3_EWUP4           (1 << 3)  /* Bit 3: Enable WKUP4 pin */
#define PWR_CR3_EWUP5           (1 << 4)  /* Bit 4: Enable WKUP5 pin */
#define PWR_CR3_EWUP7           (1 << 6)  /* Bit 6: Enable WKUP7 pin */
#define PWR_CR3_RRS             (1 << 8)  /* Bit 8: SRAM2 retention in Standby mode */
#define PWR_CR3_ENULP           (1 << 9)  /* Bit 9: Enable ULP sampling */
#define PWR_CR3_APC             (1 << 10) /* Bit 10: Apply pull-up and pull-down configuration */
#define PWR_CR3_EIWUL           (1 << 15) /* Bit 15: Enable internal wakeup line */

/* Power control register 4 */

#define PWR_CR4_WP1             (1 << 0)     /* Bit 0: WKUP1 pin polarity */
#define PWR_CR4_WP2             (1 << 1)     /* Bit 1: WKUP2 pin polarity */
#define PWR_CR4_WP3             (1 << 2)     /* Bit 2: WKUP3 pin polarity */
#define PWR_CR4_WP4             (1 << 3)     /* Bit 3: WKUP4 pin polarity */
#define PWR_CR4_WP5             (1 << 4)     /* Bit 4: WKUP5 pin polarity */
#define PWR_CR4_WP7             (1 << 6)     /* Bit 6: WKUP7 pin polarity */
#define PWR_CR4_VBE             (1 << 8)     /* Bit 8: VBAT charging enable */
#define PWR_CR4_VBRS            (1 << 9)     /* Bit 9: VBAT charging resistor selection */
#  define PWR_CR4_VBRS_5k       0            /* 0: 5k  resistor */
#  define PWR_CR4_VBRS_1k5      PWR_CR4_VBRS /* 1: 1k5 resistor */

/* Power status register 1 */

#define PWR_SR1_WUF1            (1 << 0) /* Bit 0: Wakeup flag 1 */
#define PWR_SR1_WUF2            (1 << 1) /* Bit 1: Wakeup flag 2 */
#define PWR_SR1_WUF3            (1 << 2) /* Bit 2: Wakeup flag 3 */
#define PWR_SR1_WUF4            (1 << 3) /* Bit 3: Wakeup flag 4 */
#define PWR_SR1_WUF5            (1 << 4) /* Bit 4: Wakeup flag 5 */
#define PWR_SR1_WUF7            (1 << 6) /* Bit 6: Wakeup flag 7 */
#define PWR_SR1_SBF             (1 << 8) /* Bit 8: Standby flag */
#define PWR_SR1_STOPF_SHIFT     (9)      /* Bits 9-11: Stop flags */
#define PWR_SR1_STOPF_MASK      (7 << PWR_SR1_STOPF_SHIFT)
#define PWR_SR1_WUFI            (1 << 15) /* Bit 15: Wakeup internal flag */

/* Power status register 2 */

#define PWR_SR2_FLASHRDY        (1 << 7)  /* Bit 7: Flash ready flag */
#define PWR_SR2_REGLPS          (1 << 8)  /* Bit 8: Low-power regulator started */
#define PWR_SR2_REGLPF          (1 << 9)  /* Bit 9: Low-power regulator flag */
#define PWR_SR2_VOSF            (1 << 10) /* Bit 10: Voltage scaling flag */
#define PWR_SR2_PVDO            (1 << 11) /* Bit 11: PVD output */
#define PWR_SR2_PVMO1           (1 << 12) /* Bit 12: PVM VDDUSB output */
#define PWR_SR2_PVMO3           (1 << 14) /* Bit 14: PVM VDDADC output */
#define PWR_SR2_PVMO4           (1 << 15) /* Bit 15: PVM VDDDAC output */

/* Power status clear register */

#define PWR_SCR_CWUF1           (1 << 0) /* Bit 0: Clear wakeup flag 1 */
#define PWR_SCR_CWUF2           (1 << 1) /* Bit 1: Clear wakeup flag 2 */
#define PWR_SCR_CWUF3           (1 << 2) /* Bit 2: Clear wakeup flag 3 */
#define PWR_SCR_CWUF4           (1 << 3) /* Bit 3: Clear wakeup flag 4 */
#define PWR_SCR_CWUF5           (1 << 4) /* Bit 4: Clear wakeup flag 5 */
#define PWR_SCR_CWUF7           (1 << 6) /* Bit 6: Clear wakeup flag 7 */
#define PWR_SCR_CSBF            (1 << 8) /* Bit 8: Clear standby flag */

#endif /* __ARCH_ARM_SRC_STM32U0_HARDWARE_STM32U0_PWR_H */
