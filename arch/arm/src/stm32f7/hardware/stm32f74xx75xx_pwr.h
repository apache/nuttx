/****************************************************************************
 * arch/arm/src/stm32f7/hardware/stm32f74xx75xx_pwr.h
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

#ifndef __ARCH_ARM_SRC_STM32F7_HARDWARE_STM32F74XX75XX_PWR_H
#define __ARCH_ARM_SRC_STM32F7_HARDWARE_STM32F74XX75XX_PWR_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#if defined(CONFIG_STM32F7_STM32F74XX) || defined(CONFIG_STM32F7_STM32F75XX)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define STM32_PWR_CR1_OFFSET    0x0000  /* Power control register 1 */
#define STM32_PWR_CSR1_OFFSET   0x0004  /* Power control/status register 1 */
#define STM32_PWR_CR2_OFFSET    0x0008  /* Power control register 1 */
#define STM32_PWR_CSR2_OFFSET   0x000c  /* Power control/status register 1 */

/* Register Addresses *******************************************************/

#define STM32_PWR_CR1           (STM32_PWR_BASE+STM32_PWR_CR1_OFFSET)
#define STM32_PWR_CSR1          (STM32_PWR_BASE+STM32_PWR_CSR1_OFFSET)
#define STM32_PWR_CR2           (STM32_PWR_BASE+STM32_PWR_CR2_OFFSET)
#define STM32_PWR_CSR2          (STM32_PWR_BASE+STM32_PWR_CSR2_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* Power control register 1 */

#define PWR_CR1_LPDS            (1 << 0)  /* Bit 0: Low-Power Deepsleep/sleep; low power run */
#define PWR_CR1_PDDS            (1 << 1)  /* Bit 1: Power Down Deepsleep */
#define PWR_CR1_CSBF            (1 << 3)  /* Bit 3: Clear Standby Flag */
#define PWR_CR1_PVDE            (1 << 4)  /* Bit 4: Power Voltage Detector Enable */
#define PWR_CR1_PLS_SHIFT       (5)       /* Bits 7-5: PVD Level Selection */
#define PWR_CR1_PLS_MASK        (7 << PWR_CR1_PLS_SHIFT)
#  define PWR_CR1_2p0V          (0 << PWR_CR1_PLS_SHIFT) /* 000: 2.0V */
#  define PWR_CR1_2p1V          (1 << PWR_CR1_PLS_SHIFT) /* 001: 2.1V */
#  define PWR_CR1_2p3V          (2 << PWR_CR1_PLS_SHIFT) /* 010: 2.3V */
#  define PWR_CR1_2p5V          (3 << PWR_CR1_PLS_SHIFT) /* 011: 2.5V */
#  define PWR_CR1_2p6V          (4 << PWR_CR1_PLS_SHIFT) /* 100: 2.6V */
#  define PWR_CR1_2p7V          (5 << PWR_CR1_PLS_SHIFT) /* 101: 2.7V */
#  define PWR_CR1_2p8V          (6 << PWR_CR1_PLS_SHIFT) /* 110: 2.8V */
#  define PWR_CR1_2p9V          (7 << PWR_CR1_PLS_SHIFT) /* 111: 2.9V */

#define PWR_CR1_DBP             (1 << 8)  /* Bit 8: Disable Backup Domain write protection */
#define PWR_CR1_FPDS            (1 << 9)  /* Bit 9: Flash power down in Stop mode */
#define PWR_CR1_LPUDS           (1 << 10) /* Bit 10: Low-power regulator in deepsleep under-drive mode */
#define PWR_CR1_MRUDS           (1 << 11) /* Bit 11: Main regulator in deepsleep under-drive mode */
#define PWR_CR1_ADCDC1          (1 << 13) /* Bit 13: see AN4073 for details */
#define PWR_CR1_VOS_SHIFT       (14)      /* Bits 14-15: Regulator voltage scaling output selection */
#define PWR_CR1_VOS_MASK        (3 << PWR_CR1_VOS_SHIFT)
#  define PWR_CR1_VOS_SCALE_3   (1 << PWR_CR1_VOS_SHIFT) /* Fmax = 144MHz */
#  define PWR_CR1_VOS_SCALE_2   (2 << PWR_CR1_VOS_SHIFT) /* Fmax = 168/180MHz */
#  define PWR_CR1_VOS_SCALE_1   (3 << PWR_CR1_VOS_SHIFT) /* Fmax = 180/216MHz */

#define PWR_CR1_ODEN            (1 << 16) /* Bit 16: Over Drive enable */
#define PWR_CR1_ODSWEN          (1 << 17) /* Bit 17: Over Drive switch enabled */
#define PWR_CR1_UDEN_SHIFT      (18)      /* Bits 18-19: Under-drive enable in stop mode */
#define PWR_CR1_UDEN_MASK       (3 << PWR_CR1_UDEN_SHIFT)
#  define PWR_CR1_UDEN_DISABLE  (0 << PWR_CR1_UDEN_SHIFT) /* Under-drive disable */
#  define PWR_CR1_UDEN_ENABLE   (3 << PWR_CR1_UDEN_SHIFT) /* Under-drive enable */

/* Power control/status register 1 */

#define PWR_CSR1_WUIF           (1 << 0)  /* Bit 0:  Wakeup internal flag */
#define PWR_CSR1_SBF            (1 << 1)  /* Bit 1:  Standby flag */
#define PWR_CSR1_PVDO           (1 << 2)  /* Bit 2:  PVD Output */
#define PWR_CSR1_BRR            (1 << 3)  /* Bit 3:  Backup regulator ready */
#define PWR_CSR1_EIWUP          (1 << 8)  /* Bit 8:  Enable internal wakeup */
#define PWR_CSR1_BRE            (1 << 9)  /* Bit 9:  Backup regulator enable */
#define PWR_CSR1_VOSRDY         (1 << 14) /* Bit 14: Regulator voltage scaling output selection ready bite */
#define PWR_CSR1_ODRDY          (1 << 16) /* Bit 16: Over Drive generator ready */
#define PWR_CSR1_ODSWRDY        (1 << 17) /* Bit 17: Over Drive Switch ready */
#define PWR_CSR1_UDSRDY_SHIFT   (18)      /* Bits 18-19: Under-drive ready flag */
#define PWR_CSR1_UDSRDY_MASK    (3 << PWR_CSR1_UDSRDY_SHIFT)
#  define PWR_CSR1_UDSRDY_DISAB (0 << PWR_CSR1_UDSRDY_SHIFT) /* Under-drive is disabled */
#  define PWR_CSR1_UDSRDY_STOP  (3 << PWR_CSR1_UDSRDY_SHIFT) /* Under-drive mode is activated in Stop mode */

/* Power control register 2 */

#define PWR_CR2_CWUPF1           (1 << 0)  /* Bit 0:  Clear Wakeup Pin flag for PA0 */
#define PWR_CR2_CWUPF2           (1 << 1)  /* Bit 1:  Clear Wakeup Pin flag for PA2 */
#define PWR_CR2_CWUPF3           (1 << 2)  /* Bit 2:  Clear Wakeup Pin flag for PC1 */
#define PWR_CR2_CWUPF4           (1 << 3)  /* Bit 3:  Clear Wakeup Pin flag for PC13 */
#define PWR_CR2_CWUPF5           (1 << 4)  /* Bit 4:  Clear Wakeup Pin flag for PI8 */
#define PWR_CR2_CWUPF6           (1 << 5)  /* Bit 5:  Clear Wakeup Pin flag for PI11 */
#define PWR_CR2_WUPP1            (1 << 8)  /* Bit 8:  Wakeup pin polarity bit for PA0 */
#  define PWR_CR2_WUPP1_RISING   (0 << 8)  /*         0=Detection on rising edge */
#  define PWR_CR2_WUPP1_FALLING  (1 << 8)  /*         1= Detection on falling edge */
#define PWR_CR2_WUPP2            (1 << 9)  /* Bit 9:  Wakeup pin polarity bit for PA2 */
#  define PWR_CR2_WUPP2_RISING   (0 << 9)  /*         0=Detection on rising edge */
#  define PWR_CR2_WUPP2_FALLING  (1 << 9)  /*         1= Detection on falling edge */
#define PWR_CR2_WUPP3            (1 << 10) /* Bit 10: Wakeup pin polarity bit for PC1 */
#  define PWR_CR2_WUPP3_RISING   (0 << 10) /*         0=Detection on rising edge */
#  define PWR_CR2_WUPP3_FALLING  (1 << 10) /*         1= Detection on falling edge */
#define PWR_CR2_WUPP4            (1 << 11) /* Bit 11: Wakeup pin polarity bit for PC13 */
#  define PWR_CR2_WUPP4_RISING   (0 << 11) /*         0=Detection on rising edge */
#  define PWR_CR2_WUPP4_FALLING  (1 << 11) /*         1= Detection on falling edge */
#define PWR_CR2_WUPP5            (1 << 12) /* Bit 12: Wakeup pin polarity bit for PI8 */
#  define PWR_CR2_WUPP5_RISING   (0 << 12) /*         0=Detection on rising edge */
#  define PWR_CR2_WUPP5_FALLING  (1 << 12) /*         1= Detection on falling edge */
#define PWR_CR2_WUPP6            (1 << 13) /* Bits 13: Wakeup pin polarity bit for PI11 */
#  define PWR_CR2_WUPP6_RISING   (0 << 13) /*         0=Detection on rising edge */
#  define PWR_CR2_WUPP6_FALLING  (1 << 13) /*         1= Detection on falling edge */

/* Power control/status register 2 */

#define PWR_CSR2_WUPF1           (1 << 0)  /* Bit 0:  Wakeup Pin flag for PA0 */
#define PWR_CSR2_WUPF2           (1 << 1)  /* Bit 1:  Wakeup Pin flag for PA2 */
#define PWR_CSR2_WUPF3           (1 << 2)  /* Bit 2:  Wakeup Pin flag for PC1 */
#define PWR_CSR2_WUPF4           (1 << 3)  /* Bit 3:  Wakeup Pin flag for PC13 */
#define PWR_CSR2_WUPF5           (1 << 4)  /* Bit 4:  Wakeup Pin flag for PI8 */
#define PWR_CSR2_WUPF6           (1 << 5)  /* Bit 5:  Wakeup Pin flag for PI11 */
#define PWR_CSR2_EWUP1           (1 << 8)  /* Bit 8:  Enable wakeup pin for PA0 */
#define PWR_CSR2_EWUP2           (1 << 9)  /* Bit 9:  Enable wakeup pin for PA2 */
#define PWR_CSR2_EWUP3           (1 << 10) /* Bit 10: Enable wakeup pin for PC1 */
#define PWR_CSR2_EWUP4           (1 << 11) /* Bit 11: Enable wakeup pin for PC13 */
#define PWR_CSR2_EWUP5           (1 << 12) /* Bit 12: Enable wakeup pin for PI8 */
#define PWR_CSR2_EWUP6           (1 << 13) /* Bit 13: Enable wakeup pin for PI11 */

#endif /* CONFIG_STM32F7_STM32F74XX || CONFIG_STM32F7_STM32F75XX */
#endif /* __ARCH_ARM_SRC_STM32F7_HARDWARE_STM32F74XX75XX_PWR_H */
