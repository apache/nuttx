/****************************************************************************
 * arch/arm/src/stm32n6/hardware/stm32n6xxx_pwr.h
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

#ifndef __ARCH_ARM_SRC_STM32N6_HARDWARE_STM32N6XXX_PWR_H
#define __ARCH_ARM_SRC_STM32N6_HARDWARE_STM32N6XXX_PWR_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define STM32_PWR_CR1_OFFSET       0x0000  /* Power control register 1 */
#define STM32_PWR_CR2_OFFSET       0x0004  /* Power control register 2 */
#define STM32_PWR_CR3_OFFSET       0x0008  /* Power control register 3 */
#define STM32_PWR_CR4_OFFSET       0x000c  /* Power control register 4 */
#define STM32_PWR_VOSCR_OFFSET     0x0020  /* Voltage scaling control register */
#define STM32_PWR_BDCR1_OFFSET     0x0024  /* Backup domain control register 1 */
#define STM32_PWR_BDCR2_OFFSET     0x0028  /* Backup domain control register 2 */
#define STM32_PWR_DBPCR_OFFSET     0x002c  /* Debug backup domain protection control register */
#define STM32_PWR_CPUCR_OFFSET     0x0030  /* CPU power control register */
#define STM32_PWR_SVMCR1_OFFSET    0x0034  /* Supply voltage monitoring control register 1 */
#define STM32_PWR_SVMCR2_OFFSET    0x0038  /* Supply voltage monitoring control register 2 */
#define STM32_PWR_SVMCR3_OFFSET    0x003c  /* Supply voltage monitoring control register 3 */
#define STM32_PWR_PRIVCFGR_OFFSET  0x0104  /* Privilege configuration register */
#define STM32_PWR_SECCFGR_OFFSET   0x0100  /* Secure configuration register */

/* Register Addresses *******************************************************/

#define STM32_PWR_CR1          (STM32_PWR_BASE + STM32_PWR_CR1_OFFSET)
#define STM32_PWR_CR2          (STM32_PWR_BASE + STM32_PWR_CR2_OFFSET)
#define STM32_PWR_CR3          (STM32_PWR_BASE + STM32_PWR_CR3_OFFSET)
#define STM32_PWR_CR4          (STM32_PWR_BASE + STM32_PWR_CR4_OFFSET)
#define STM32_PWR_VOSCR        (STM32_PWR_BASE + STM32_PWR_VOSCR_OFFSET)
#define STM32_PWR_BDCR1        (STM32_PWR_BASE + STM32_PWR_BDCR1_OFFSET)
#define STM32_PWR_BDCR2        (STM32_PWR_BASE + STM32_PWR_BDCR2_OFFSET)
#define STM32_PWR_DBPCR        (STM32_PWR_BASE + STM32_PWR_DBPCR_OFFSET)
#define STM32_PWR_CPUCR        (STM32_PWR_BASE + STM32_PWR_CPUCR_OFFSET)
#define STM32_PWR_SVMCR1       (STM32_PWR_BASE + STM32_PWR_SVMCR1_OFFSET)
#define STM32_PWR_SVMCR2       (STM32_PWR_BASE + STM32_PWR_SVMCR2_OFFSET)
#define STM32_PWR_SVMCR3       (STM32_PWR_BASE + STM32_PWR_SVMCR3_OFFSET)
#define STM32_PWR_PRIVCFGR     (STM32_PWR_BASE + STM32_PWR_PRIVCFGR_OFFSET)
#define STM32_PWR_SECCFGR      (STM32_PWR_BASE + STM32_PWR_SECCFGR_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* Debug Backup Domain Protection Control Register (PWR_DBPCR) */

/* Power control register 1 (PWR_CR1) */

#define PWR_CR1_SDEN               (1 << 2)  /* Bit 2:  SMPS step-down converter enable */
#define PWR_CR1_MODE_PDN           (1 << 4)  /* Bit 4:  SMPS power-down in Stop */
#define PWR_CR1_LPDS08V            (1 << 5)  /* Bit 5:  Low-power deepsleep 0.8 V */
#define PWR_CR1_VDD18SMPSVMEN      (1 << 8)  /* Bit 8:  VDD18 SMPS voltage monitor enable */
#define PWR_CR1_VDD18SMPSRDY       (1 << 15) /* Bit 15: VDD18 SMPS ready */
#define PWR_CR1_POPL_SHIFT         (16)      /* Bits 16-20: Power-on pulse length */
#define PWR_CR1_POPL_MASK          (0x1f << PWR_CR1_POPL_SHIFT)

/* Power control register 3 (PWR_CR3) */

#define PWR_CR3_VCOREMONEN         (1 << 0)  /* Bit 0:  VCORE voltage monitor enable */
#define PWR_CR3_VCORELLS           (1 << 4)  /* Bit 4:  VCORE low level select */
#define PWR_CR3_VCOREL             (1 << 8)  /* Bit 8:  VCORE below low threshold */
#define PWR_CR3_VCOREH             (1 << 9)  /* Bit 9:  VCORE above high threshold */

#define PWR_DBPCR_DBP              (1 << 0)  /* Bit 0: Disable backup domain write protection */

/* Voltage Scaling Control Register (PWR_VOSCR)
 *
 * STM32N6 has a single-bit VOS field selecting between low (0) and high (1)
 * voltage scaling.  VOSRDY/ACTVOS status flags live in the same register.
 */

#define PWR_VOSCR_VOS              (1 << 0)  /* Bit 0:  Voltage scaling 0=low, 1=high */
#define PWR_VOSCR_VOSRDY           (1 << 1)  /* Bit 1:  VOS Ready */
#define PWR_VOSCR_ACTVOS           (1 << 16) /* Bit 16: Active VOS (read-back) */
#define PWR_VOSCR_ACTVOSRDY        (1 << 17) /* Bit 17: Active VOS ready */

/* CPU Power Control Register (PWR_CPUCR) */

#define PWR_CPUCR_PDDS             (1 << 0)  /* Bit 0:  Power down deepsleep for CPU */
#define PWR_CPUCR_STOPF            (1 << 8)  /* Bit 8:  STOP flag */
#define PWR_CPUCR_SBF              (1 << 9)  /* Bit 9:  Standby flag */
#define PWR_CPUCR_SVOS_SHIFT       (16)      /* Bits 16-17: System voltage scaling in stop */
#define PWR_CPUCR_SVOS_MASK        (3 << PWR_CPUCR_SVOS_SHIFT)

/* Supply Voltage Monitoring Control Register 1 (PWR_SVMCR1) - VddIO4 */

#define PWR_SVMCR1_VDDIO4VMEN     (1 << 0)  /* Bit 0:  VddIO4 voltage monitor enable */
#define PWR_SVMCR1_VDDIO4SV       (1 << 8)  /* Bit 8:  VddIO4 supply valid */
#define PWR_SVMCR1_VDDIO4RDY      (1 << 16) /* Bit 16: VddIO4 ready */
#define PWR_SVMCR1_VDDIO4VRSEL    (1 << 24) /* Bit 24: VddIO4 voltage range (1=1.8 V) */
#define PWR_SVMCR1_VDDIO4VRSTBY   (1 << 25) /* Bit 25: VddIO4 range in Standby */

/* Supply Voltage Monitoring Control Register 2 (PWR_SVMCR2) - VddIO5 */

#define PWR_SVMCR2_VDDIO5VMEN     (1 << 0)  /* Bit 0:  VddIO5 voltage monitor enable */
#define PWR_SVMCR2_VDDIO5SV       (1 << 8)  /* Bit 8:  VddIO5 supply valid */
#define PWR_SVMCR2_VDDIO5RDY      (1 << 16) /* Bit 16: VddIO5 ready */
#define PWR_SVMCR2_VDDIO5VRSEL    (1 << 24) /* Bit 24: VddIO5 voltage range (1=1.8 V) */
#define PWR_SVMCR2_VDDIO5VRSTBY   (1 << 25) /* Bit 25: VddIO5 range in Standby */

/* Supply Voltage Monitoring Control Register 3 (PWR_SVMCR3) - VddIO2/3,
 * USB33 and analog supplies
 */

#define PWR_SVMCR3_VDDIO2VMEN     (1 << 0)  /* Bit 0:  VddIO2 voltage monitor enable */
#define PWR_SVMCR3_VDDIO3VMEN     (1 << 1)  /* Bit 1:  VddIO3 voltage monitor enable */
#define PWR_SVMCR3_USB33VMEN      (1 << 2)  /* Bit 2:  USB33 voltage monitor enable */
#define PWR_SVMCR3_AVMEN          (1 << 4)  /* Bit 4:  Analog voltage monitor enable */
#define PWR_SVMCR3_VDDIO2SV       (1 << 8)  /* Bit 8:  VddIO2 supply valid */
#define PWR_SVMCR3_VDDIO3SV       (1 << 9)  /* Bit 9:  VddIO3 supply valid */
#define PWR_SVMCR3_USB33SV        (1 << 10) /* Bit 10: USB33 supply valid */
#define PWR_SVMCR3_ASV            (1 << 12) /* Bit 12: Analog supply valid */
#define PWR_SVMCR3_VDDIO2RDY      (1 << 16) /* Bit 16: VddIO2 ready */
#define PWR_SVMCR3_VDDIO3RDY      (1 << 17) /* Bit 17: VddIO3 ready */
#define PWR_SVMCR3_USB33RDY       (1 << 18) /* Bit 18: USB33 ready */
#define PWR_SVMCR3_ARDY           (1 << 20) /* Bit 20: Analog supply ready */
#define PWR_SVMCR3_VDDIOVRSEL     (1 << 24) /* Bit 24: VddIO voltage range (1=1.8 V) */
#define PWR_SVMCR3_VDDIO2VRSEL    (1 << 25) /* Bit 25: VddIO2 voltage range (1=1.8 V) */
#define PWR_SVMCR3_VDDIO3VRSEL    (1 << 26) /* Bit 26: VddIO3 voltage range (1=1.8 V) */

#endif /* __ARCH_ARM_SRC_STM32N6_HARDWARE_STM32N6XXX_PWR_H */
