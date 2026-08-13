/****************************************************************************
 * arch/arm/src/stm32u3/hardware/stm32u3xx_pwr.h
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

#ifndef __ARCH_ARM_SRC_STM32U3_HARDWARE_STM32U3XX_PWR_H
#define __ARCH_ARM_SRC_STM32U3_HARDWARE_STM32U3XX_PWR_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define STM32_PWR_CR1_OFFSET      0x0000
#define STM32_PWR_CR2_OFFSET      0x0004
#define STM32_PWR_CR3_OFFSET      0x0008
#define STM32_PWR_VOSR_OFFSET     0x000c
#define STM32_PWR_SVMCR_OFFSET    0x0010
#define STM32_PWR_WUCR1_OFFSET    0x0014
#define STM32_PWR_WUCR2_OFFSET    0x0018
#define STM32_PWR_WUCR3_OFFSET    0x001c
#define STM32_PWR_BDCR_OFFSET     0x0024
#define STM32_PWR_DBPR_OFFSET     0x0028
#define STM32_PWR_SECCFGR_OFFSET  0x0030
#define STM32_PWR_PRIVCFGR_OFFSET 0x0034
#define STM32_PWR_SR_OFFSET       0x0038
#define STM32_PWR_SVMSR_OFFSET    0x003c
#define STM32_PWR_WUSR_OFFSET     0x0044
#define STM32_PWR_WUSCR_OFFSET    0x0048
#define STM32_PWR_APCR_OFFSET     0x004c
#define STM32_PWR_PUCRA_OFFSET    0x0050
#define STM32_PWR_PDCRA_OFFSET    0x0054
#define STM32_PWR_PUCRB_OFFSET    0x0058
#define STM32_PWR_PDCRB_OFFSET    0x005c
#define STM32_PWR_PUCRC_OFFSET    0x0060
#define STM32_PWR_PDCRC_OFFSET    0x0064
#define STM32_PWR_PUCRD_OFFSET    0x0068
#define STM32_PWR_PDCRD_OFFSET    0x006c
#define STM32_PWR_PUCRE_OFFSET    0x0070
#define STM32_PWR_PDCRE_OFFSET    0x0074
#define STM32_PWR_PUCRF_OFFSET    0x0078
#define STM32_PWR_PDCRF_OFFSET    0x007c
#define STM32_PWR_PUCRG_OFFSET    0x0080
#define STM32_PWR_PDCRG_OFFSET    0x0084
#define STM32_PWR_PUCRH_OFFSET    0x0088
#define STM32_PWR_PDCRH_OFFSET    0x008c
#define STM32_PWR_I3CPUCR1_OFFSET 0x00b0
#define STM32_PWR_I3CPUCR2_OFFSET 0x00b4

/* Register Addresses *******************************************************/

#define STM32_PWR_CR1      (STM32_PWR_BASE + STM32_PWR_CR1_OFFSET)
#define STM32_PWR_CR2      (STM32_PWR_BASE + STM32_PWR_CR2_OFFSET)
#define STM32_PWR_CR3      (STM32_PWR_BASE + STM32_PWR_CR3_OFFSET)
#define STM32_PWR_VOSR     (STM32_PWR_BASE + STM32_PWR_VOSR_OFFSET)
#define STM32_PWR_SVMCR    (STM32_PWR_BASE + STM32_PWR_SVMCR_OFFSET)
#define STM32_PWR_WUCR1    (STM32_PWR_BASE + STM32_PWR_WUCR1_OFFSET)
#define STM32_PWR_WUCR2    (STM32_PWR_BASE + STM32_PWR_WUCR2_OFFSET)
#define STM32_PWR_WUCR3    (STM32_PWR_BASE + STM32_PWR_WUCR3_OFFSET)
#define STM32_PWR_BDCR     (STM32_PWR_BASE + STM32_PWR_BDCR_OFFSET)
#define STM32_PWR_DBPR     (STM32_PWR_BASE + STM32_PWR_DBPR_OFFSET)
#define STM32_PWR_SECCFGR  (STM32_PWR_BASE + STM32_PWR_SECCFGR_OFFSET)
#define STM32_PWR_PRIVCFGR (STM32_PWR_BASE + STM32_PWR_PRIVCFGR_OFFSET)
#define STM32_PWR_SR       (STM32_PWR_BASE + STM32_PWR_SR_OFFSET)
#define STM32_PWR_SVMSR    (STM32_PWR_BASE + STM32_PWR_SVMSR_OFFSET)
#define STM32_PWR_WUSR     (STM32_PWR_BASE + STM32_PWR_WUSR_OFFSET)
#define STM32_PWR_WUSCR    (STM32_PWR_BASE + STM32_PWR_WUSCR_OFFSET)
#define STM32_PWR_APCR     (STM32_PWR_BASE + STM32_PWR_APCR_OFFSET)
#define STM32_PWR_PUCRA    (STM32_PWR_BASE + STM32_PWR_PUCRA_OFFSET)
#define STM32_PWR_PDCRA    (STM32_PWR_BASE + STM32_PWR_PDCRA_OFFSET)
#define STM32_PWR_PUCRB    (STM32_PWR_BASE + STM32_PWR_PUCRB_OFFSET)
#define STM32_PWR_PDCRB    (STM32_PWR_BASE + STM32_PWR_PDCRB_OFFSET)
#define STM32_PWR_PUCRC    (STM32_PWR_BASE + STM32_PWR_PUCRC_OFFSET)
#define STM32_PWR_PDCRC    (STM32_PWR_BASE + STM32_PWR_PDCRC_OFFSET)
#define STM32_PWR_PUCRD    (STM32_PWR_BASE + STM32_PWR_PUCRD_OFFSET)
#define STM32_PWR_PDCRD    (STM32_PWR_BASE + STM32_PWR_PDCRD_OFFSET)
#define STM32_PWR_PUCRE    (STM32_PWR_BASE + STM32_PWR_PUCRE_OFFSET)
#define STM32_PWR_PDCRE    (STM32_PWR_BASE + STM32_PWR_PDCRE_OFFSET)
#define STM32_PWR_PUCRF    (STM32_PWR_BASE + STM32_PWR_PUCRF_OFFSET)
#define STM32_PWR_PDCRF    (STM32_PWR_BASE + STM32_PWR_PDCRF_OFFSET)
#define STM32_PWR_PUCRG    (STM32_PWR_BASE + STM32_PWR_PUCRG_OFFSET)
#define STM32_PWR_PDCRG    (STM32_PWR_BASE + STM32_PWR_PDCRG_OFFSET)
#define STM32_PWR_PUCRH    (STM32_PWR_BASE + STM32_PWR_PUCRH_OFFSET)
#define STM32_PWR_PDCRH    (STM32_PWR_BASE + STM32_PWR_PDCRH_OFFSET)
#define STM32_PWR_I3CPUCR1 (STM32_PWR_BASE + STM32_PWR_I3CPUCR1_OFFSET)
#define STM32_PWR_I3CPUCR2 (STM32_PWR_BASE + STM32_PWR_I3CPUCR2_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* PWR control register 1 */

#define PWR_CR1_LPMS_SHIFT      (0)
#define PWR_CR1_LPMS_MASK       (0x7 << PWR_CR1_LPMS_SHIFT)
#define PWR_CR1_LPMS(n)         ((n) << PWR_CR1_LPMS_SHIFT)
#  define PWR_CR1_LPMS_STOP0    PWR_CR1_LPMS(0)
#  define PWR_CR1_LPMS_STOP1    PWR_CR1_LPMS(1)
#  define PWR_CR1_LPMS_STOP2    PWR_CR1_LPMS(2)
#  define PWR_CR1_LPMS_STOP3    PWR_CR1_LPMS(3)
#  define PWR_CR1_LPMS_STANDBY  PWR_CR1_LPMS(4)
#  define PWR_CR1_LPMS_SHUTDOWN PWR_CR1_LPMS(6)
#define PWR_CR1_RRSB1           (1 << 4)
#define PWR_CR1_RRSB2           (1 << 5)
#define PWR_CR1_RRSB3           (1 << 6)
#define PWR_CR1_ULPMEN          (1 << 7)
#define PWR_CR1_SRAM1PD         (1 << 8)
#define PWR_CR1_SRAM2PD         (1 << 9)
#define PWR_CR1_SRAM3PD         (1 << 10)
#define PWR_CR1_SRAM4PD         (1 << 11)

/* PWR control register 2 */

#define PWR_CR2_RESET           0x00000000
#define PWR_CR2_SRAM1PDS1       (1 << 0)
#define PWR_CR2_SRAM1PDS2       (1 << 1)
#define PWR_CR2_SRAM1PDS3       (1 << 2)
#define PWR_CR2_SRAM1PDS4       (1 << 3)
#define PWR_CR2_SRAM1PDS5       (1 << 4)
#define PWR_CR2_SRAM1PDS6       (1 << 5)
#define PWR_CR2_SRAM1PDS7       (1 << 6)
#define PWR_CR2_SRAM3PDS1       (1 << 8)
#define PWR_CR2_SRAM3PDS2       (1 << 9)
#define PWR_CR2_SRAM3PDS3       (1 << 10)
#define PWR_CR2_SRAM3PDS4       (1 << 11)
#define PWR_CR2_SRAM3PDS5       (1 << 12)
#define PWR_CR2_SRAM2PDS1       (1 << 16)
#define PWR_CR2_SRAM2PDS2       (1 << 17)
#define PWR_CR2_SRAM2PDS3       (1 << 18)
#define PWR_CR2_SRAM4PDS        (1 << 20)
#define PWR_CR2_ICRAMPDS        (1 << 25)
#define PWR_CR2_PRAMPDS         (1 << 26)
#define PWR_CR2_PKARAMPDS       (1 << 27)
#define PWR_CR2_SRAMFWU         (1 << 28)
#define PWR_CR2_FLASHFWU        (1 << 29)

/* PWR control register 3 */

#define PWR_CR3_RESET           0x00000000
#define PWR_CR3_REGSEL          (1 << 1)
#  define PWR_CR3_REGSEL_LDO    (0)
#  define PWR_CR3_REGSEL_SMPS   PWR_CR3_REGSEL
#define PWR_CR3_FSTEN           (1 << 2)

/* PWR voltage scaling register */

#define PWR_VOSR_RESET          0x00020002
#define PWR_VOSR_R1EN           (1 << 0)
#define PWR_VOSR_R2EN           (1 << 1)
#define PWR_VOSR_BOOSTEN        (1 << 8)
#define PWR_VOSR_R1RDY          (1 << 16)
#define PWR_VOSR_R2RDY          (1 << 17)
#define PWR_VOSR_BOOSTRDY       (1 << 24)

/* PWR supply voltage monitoring control register */

#define PWR_SVMCR_RESET         0x00000000
#define PWR_SVMCR_PVDE          (1 << 4)
#define PWR_SVMCR_PVDLS_SHIFT   (5)
#define PWR_SVMCR_PVDLS_MASK    (0x7 << PWR_SVMCR_PVDLS_SHIFT)
#define PWR_SVMCR_PVDLS(n)      ((n) << PWR_SVMCR_PVDLS_SHIFT)
#  define PWR_SVMCR_PVDLS_2000mv PWR_SVMCR_PVDLS(0)
#  define PWR_SVMCR_PVDLS_2200mv PWR_SVMCR_PVDLS(1)
#  define PWR_SVMCR_PVDLS_2400mv PWR_SVMCR_PVDLS(2)
#  define PWR_SVMCR_PVDLS_2500mv PWR_SVMCR_PVDLS(3)
#  define PWR_SVMCR_PVDLS_2600mv PWR_SVMCR_PVDLS(4)
#  define PWR_SVMCR_PVDLS_2800mv PWR_SVMCR_PVDLS(5)
#  define PWR_SVMCR_PVDLS_2900mv PWR_SVMCR_PVDLS(6)
#  define PWR_SVMCR_PVDLS_EXT   PWR_SVMCR_PVDLS(7)
#define PWR_SVMCR_UVMEN         (1 << 24)
#define PWR_SVMCR_IO2VMEN       (1 << 25)
#define PWR_SVMCR_AVM1EN        (1 << 26)
#define PWR_SVMCR_AVM2EN        (1 << 27)
#define PWR_SVMCR_USV           (1 << 28)
#define PWR_SVMCR_IO2SV         (1 << 29)
#define PWR_SVMCR_ASV           (1 << 30)

/* PWR wake-up control register 1 */

#define PWR_WUCR1_RESET         0x00000000
#define PWR_WUCR1_WUPEN1        (1 << 0)
#define PWR_WUCR1_WUPEN2        (1 << 1)
#define PWR_WUCR1_WUPEN3        (1 << 2)
#define PWR_WUCR1_WUPEN4        (1 << 3)
#define PWR_WUCR1_WUPEN5        (1 << 4)
#define PWR_WUCR1_WUPEN6        (1 << 5)
#define PWR_WUCR1_WUPEN7        (1 << 6)
#define PWR_WUCR1_WUPEN8        (1 << 7)
#define PWR_WUCR1_WUPEN9        (1 << 8)
#define PWR_WUCR1_WUPEN10       (1 << 9)

/* PWR wake-up control register 2 */

#define PWR_WUCR2_RESET         0x00000000
#define PWR_WUCR2_WUPP1         (1 << 0)
#define PWR_WUCR2_WUPP2         (1 << 1)
#define PWR_WUCR2_WUPP3         (1 << 2)
#define PWR_WUCR2_WUPP4         (1 << 3)
#define PWR_WUCR2_WUPP5         (1 << 4)
#define PWR_WUCR2_WUPP6         (1 << 5)
#define PWR_WUCR2_WUPP7         (1 << 6)
#define PWR_WUCR2_WUPP8         (1 << 7)

/* PWR wake-up control register 3 */

#define PWR_WUCR3_RESET         0x00000000
#define PWR_WUCR3_WUSEL1_SHIFT  (0)
#define PWR_WUCR3_WUSEL1_MASK   (0x3 << PWR_WUCR3_WUSEL1_SHIFT)
#define PWR_WUCR3_WUSEL1(n)     ((n) << PWR_WUCR3_WUSEL1_SHIFT)
#  define PWR_WUCR3_WUSEL1_0    PWR_WUCR3_WUSEL1(0)
#  define PWR_WUCR3_WUSEL1_1    PWR_WUCR3_WUSEL1(1)
#  define PWR_WUCR3_WUSEL1_2    PWR_WUCR3_WUSEL1(2)
#  define PWR_WUCR3_WUSEL1_3    PWR_WUCR3_WUSEL1(3)
#define PWR_WUCR3_WUSEL2_SHIFT  (2)
#define PWR_WUCR3_WUSEL2_MASK   (0x3 << PWR_WUCR3_WUSEL2_SHIFT)
#define PWR_WUCR3_WUSEL2(n)     ((n) << PWR_WUCR3_WUSEL2_SHIFT)
#  define PWR_WUCR3_WUSEL2_0    PWR_WUCR3_WUSEL2(0)
#  define PWR_WUCR3_WUSEL2_1    PWR_WUCR3_WUSEL2(1)
#  define PWR_WUCR3_WUSEL2_2    PWR_WUCR3_WUSEL2(2)
#  define PWR_WUCR3_WUSEL2_3    PWR_WUCR3_WUSEL2(3)
#define PWR_WUCR3_WUSEL3_SHIFT  (4)
#define PWR_WUCR3_WUSEL3_MASK   (0x3 << PWR_WUCR3_WUSEL3_SHIFT)
#define PWR_WUCR3_WUSEL3(n)     ((n) << PWR_WUCR3_WUSEL3_SHIFT)
#  define PWR_WUCR3_WUSEL3_0    PWR_WUCR3_WUSEL3(0)
#  define PWR_WUCR3_WUSEL3_1    PWR_WUCR3_WUSEL3(1)
#  define PWR_WUCR3_WUSEL3_2    PWR_WUCR3_WUSEL3(2)
#  define PWR_WUCR3_WUSEL3_3    PWR_WUCR3_WUSEL3(3)
#define PWR_WUCR3_WUSEL4_SHIFT  (6)
#define PWR_WUCR3_WUSEL4_MASK   (0x3 << PWR_WUCR3_WUSEL4_SHIFT)
#define PWR_WUCR3_WUSEL4(n)     ((n) << PWR_WUCR3_WUSEL4_SHIFT)
#  define PWR_WUCR3_WUSEL4_0    PWR_WUCR3_WUSEL4(0)
#  define PWR_WUCR3_WUSEL4_1    PWR_WUCR3_WUSEL4(1)
#  define PWR_WUCR3_WUSEL4_2    PWR_WUCR3_WUSEL4(2)
#  define PWR_WUCR3_WUSEL4_3    PWR_WUCR3_WUSEL4(3)
#define PWR_WUCR3_WUSEL5_SHIFT  (8)
#define PWR_WUCR3_WUSEL5_MASK   (0x3 << PWR_WUCR3_WUSEL5_SHIFT)
#define PWR_WUCR3_WUSEL5(n)     ((n) << PWR_WUCR3_WUSEL5_SHIFT)
#  define PWR_WUCR3_WUSEL5_0    PWR_WUCR3_WUSEL5(0)
#  define PWR_WUCR3_WUSEL5_1    PWR_WUCR3_WUSEL5(1)
#  define PWR_WUCR3_WUSEL5_2    PWR_WUCR3_WUSEL5(2)
#  define PWR_WUCR3_WUSEL5_3    PWR_WUCR3_WUSEL5(3)
#define PWR_WUCR3_WUSEL6_SHIFT  (10)
#define PWR_WUCR3_WUSEL6_MASK   (0x3 << PWR_WUCR3_WUSEL6_SHIFT)
#define PWR_WUCR3_WUSEL6(n)     ((n) << PWR_WUCR3_WUSEL6_SHIFT)
#  define PWR_WUCR3_WUSEL6_0    PWR_WUCR3_WUSEL6(0)
#  define PWR_WUCR3_WUSEL6_1    PWR_WUCR3_WUSEL6(1)
#  define PWR_WUCR3_WUSEL6_2    PWR_WUCR3_WUSEL6(2)
#  define PWR_WUCR3_WUSEL6_3    PWR_WUCR3_WUSEL6(3)
#define PWR_WUCR3_WUSEL7_SHIFT  (12)
#define PWR_WUCR3_WUSEL7_MASK   (0x3 << PWR_WUCR3_WUSEL7_SHIFT)
#define PWR_WUCR3_WUSEL7(n)     ((n) << PWR_WUCR3_WUSEL7_SHIFT)
#  define PWR_WUCR3_WUSEL7_0    PWR_WUCR3_WUSEL7(0)
#  define PWR_WUCR3_WUSEL7_1    PWR_WUCR3_WUSEL7(1)
#  define PWR_WUCR3_WUSEL7_2    PWR_WUCR3_WUSEL7(2)
#  define PWR_WUCR3_WUSEL7_3    PWR_WUCR3_WUSEL7(3)
#define PWR_WUCR3_WUSEL8_SHIFT  (14)
#define PWR_WUCR3_WUSEL8_MASK   (0x3 << PWR_WUCR3_WUSEL8_SHIFT)
#define PWR_WUCR3_WUSEL8(n)     ((n) << PWR_WUCR3_WUSEL8_SHIFT)
#  define PWR_WUCR3_WUSEL8_0    PWR_WUCR3_WUSEL8(0)
#  define PWR_WUCR3_WUSEL8_1    PWR_WUCR3_WUSEL8(1)
#  define PWR_WUCR3_WUSEL8_2    PWR_WUCR3_WUSEL8(2)
#  define PWR_WUCR3_WUSEL8_3    PWR_WUCR3_WUSEL8(3)

/* PWR backup domain control register */

#define PWR_BDCR_RESET          0x00000000
#define PWR_BDCR_VBE            (1 << 0)
#define PWR_BDCR_VBRS           (1 << 1)
#  define PWR_BDCR_VBRS_5K      (0)
#  define PWR_BDCR_VBRS_1K5     PWR_BDCR_VBRS

/* PWR disable backup domain register */

#define PWR_DBPR_RESET          0x00000000
#define PWR_DBPR_DBP            (1 << 0)

/* PWR security configuration register */

#define PWR_SECCFGR_RESET       0x00000000
#define PWR_SECCFGR_WUP1SEC     (1 << 0)
#define PWR_SECCFGR_WUP2SEC     (1 << 1)
#define PWR_SECCFGR_WUP3SEC     (1 << 2)
#define PWR_SECCFGR_WUP4SEC     (1 << 3)
#define PWR_SECCFGR_WUP5SEC     (1 << 4)
#define PWR_SECCFGR_WUP6SEC     (1 << 5)
#define PWR_SECCFGR_WUP7SEC     (1 << 6)
#define PWR_SECCFGR_WUP8SEC     (1 << 7)
#define PWR_SECCFGR_WUP9SEC     (1 << 8)
#define PWR_SECCFGR_WUP10SEC    (1 << 9)
#define PWR_SECCFGR_LPMSEC      (1 << 12)
#define PWR_SECCFGR_VDMSEC      (1 << 13)
#define PWR_SECCFGR_VBSEC       (1 << 14)
#define PWR_SECCFGR_APCSEC      (1 << 15)

/* PWR privilege control register */

#define PWR_PRIVCFGR_RESET      0x00000000
#define PWR_PRIVCFGR_SPRIV      (1 << 0)
#define PWR_PRIVCFGR_NSPRIV     (1 << 1)

/* PWR status register */

#define PWR_SR_RESET            0x00000000
#define PWR_SR_CSSF             (1 << 0)
#define PWR_SR_STOPF            (1 << 1)
#define PWR_SR_SBF              (1 << 2)

/* PWR supply voltage monitoring status register */

#define PWR_SVMSR_RESET         0x00000000
#define PWR_SVMSR_REGS          (1 << 1)
#  define PWR_SVMSR_REGS_LDO    (0)
#  define PWR_SVMSR_REGS_SMPS   PWR_SVMSR_REGS
#define PWR_SVMSR_PVDO          (1 << 4)
#define PWR_SVMSR_VDDUSBRDY     (1 << 24)
#define PWR_SVMSR_VDDIO2RDY     (1 << 25)
#define PWR_SVMSR_VDDA1RDY      (1 << 26)
#define PWR_SVMSR_VDDA2RDY      (1 << 27)

/* PWR wake-up status register */

#define PWR_WUSR_RESET          0x00000000
#define PWR_WUSR_WUF1           (1 << 0)
#define PWR_WUSR_WUF2           (1 << 1)
#define PWR_WUSR_WUF3           (1 << 2)
#define PWR_WUSR_WUF4           (1 << 3)
#define PWR_WUSR_WUF5           (1 << 4)
#define PWR_WUSR_WUF6           (1 << 5)
#define PWR_WUSR_WUF7           (1 << 6)
#define PWR_WUSR_WUF8           (1 << 7)
#define PWR_WUSR_WUF9           (1 << 8)
#define PWR_WUSR_WUF10          (1 << 9)

/* PWR wake-up status clear register */

#define PWR_WUSCR_RESET         0x00000000
#define PWR_WUSCR_CWUF1         (1 << 0)
#define PWR_WUSCR_CWUF2         (1 << 1)
#define PWR_WUSCR_CWUF3         (1 << 2)
#define PWR_WUSCR_CWUF4         (1 << 3)
#define PWR_WUSCR_CWUF5         (1 << 4)
#define PWR_WUSCR_CWUF6         (1 << 5)
#define PWR_WUSCR_CWUF7         (1 << 6)
#define PWR_WUSCR_CWUF8         (1 << 7)
#define PWR_WUSCR_CWUF9         (1 << 8)
#define PWR_WUSCR_CWUF10        (1 << 9)

/* PWR apply pull configuration register */

#define PWR_APCR_RESET          0x00000000
#define PWR_APCR_APC            (1 << 0)

/* PWR port A pull-up control register */

#define PWR_PUCRA_RESET         0x00000000
#define PWR_PUCRA_PU0           (1 << 0)
#define PWR_PUCRA_PU1           (1 << 1)
#define PWR_PUCRA_PU2           (1 << 2)
#define PWR_PUCRA_PU3           (1 << 3)
#define PWR_PUCRA_PU4           (1 << 4)
#define PWR_PUCRA_PU5           (1 << 5)
#define PWR_PUCRA_PU6           (1 << 6)
#define PWR_PUCRA_PU7           (1 << 7)
#define PWR_PUCRA_PU8           (1 << 8)
#define PWR_PUCRA_PU9           (1 << 9)
#define PWR_PUCRA_PU10          (1 << 10)
#define PWR_PUCRA_PU11          (1 << 11)
#define PWR_PUCRA_PU12          (1 << 12)
#define PWR_PUCRA_PU13          (1 << 13)
#define PWR_PUCRA_PU15          (1 << 15)

/* PWR port A pull-down control register */

#define PWR_PDCRA_RESET         0x00000000
#define PWR_PDCRA_PD0           (1 << 0)
#define PWR_PDCRA_PD1           (1 << 1)
#define PWR_PDCRA_PD2           (1 << 2)
#define PWR_PDCRA_PD3           (1 << 3)
#define PWR_PDCRA_PD4           (1 << 4)
#define PWR_PDCRA_PD5           (1 << 5)
#define PWR_PDCRA_PD6           (1 << 6)
#define PWR_PDCRA_PD7           (1 << 7)
#define PWR_PDCRA_PD8           (1 << 8)
#define PWR_PDCRA_PD9           (1 << 9)
#define PWR_PDCRA_PD10          (1 << 10)
#define PWR_PDCRA_PD11          (1 << 11)
#define PWR_PDCRA_PD12          (1 << 12)
#define PWR_PDCRA_PD14          (1 << 14)

/* PWR port B pull-up control register */

#define PWR_PUCRB_RESET         0x00000000
#define PWR_PUCRB_PU0           (1 << 0)
#define PWR_PUCRB_PU1           (1 << 1)
#define PWR_PUCRB_PU2           (1 << 2)
#define PWR_PUCRB_PU3           (1 << 3)
#define PWR_PUCRB_PU4           (1 << 4)
#define PWR_PUCRB_PU5           (1 << 5)
#define PWR_PUCRB_PU6           (1 << 6)
#define PWR_PUCRB_PU7           (1 << 7)
#define PWR_PUCRB_PU8           (1 << 8)
#define PWR_PUCRB_PU9           (1 << 9)
#define PWR_PUCRB_PU10          (1 << 10)
#define PWR_PUCRB_PU11          (1 << 11)
#define PWR_PUCRB_PU12          (1 << 12)
#define PWR_PUCRB_PU13          (1 << 13)
#define PWR_PUCRB_PU14          (1 << 14)
#define PWR_PUCRB_PU15          (1 << 15)

/* PWR port B pull-down control register */

#define PWR_PDCRB_RESET         0x00000000
#define PWR_PDCRB_PD0           (1 << 0)
#define PWR_PDCRB_PD1           (1 << 1)
#define PWR_PDCRB_PD2           (1 << 2)
#define PWR_PDCRB_PD3           (1 << 3)
#define PWR_PDCRB_PD5           (1 << 5)
#define PWR_PDCRB_PD6           (1 << 6)
#define PWR_PDCRB_PD7           (1 << 7)
#define PWR_PDCRB_PD8           (1 << 8)
#define PWR_PDCRB_PD9           (1 << 9)
#define PWR_PDCRB_PD10          (1 << 10)
#define PWR_PDCRB_PD11          (1 << 11)
#define PWR_PDCRB_PD12          (1 << 12)
#define PWR_PDCRB_PD13          (1 << 13)
#define PWR_PDCRB_PD14          (1 << 14)
#define PWR_PDCRB_PD15          (1 << 15)

/* PWR port C pull-up control register */

#define PWR_PUCRC_RESET         0x00000000
#define PWR_PUCRC_PU0           (1 << 0)
#define PWR_PUCRC_PU1           (1 << 1)
#define PWR_PUCRC_PU2           (1 << 2)
#define PWR_PUCRC_PU3           (1 << 3)
#define PWR_PUCRC_PU4           (1 << 4)
#define PWR_PUCRC_PU5           (1 << 5)
#define PWR_PUCRC_PU6           (1 << 6)
#define PWR_PUCRC_PU7           (1 << 7)
#define PWR_PUCRC_PU8           (1 << 8)
#define PWR_PUCRC_PU9           (1 << 9)
#define PWR_PUCRC_PU10          (1 << 10)
#define PWR_PUCRC_PU11          (1 << 11)
#define PWR_PUCRC_PU12          (1 << 12)
#define PWR_PUCRC_PU13          (1 << 13)
#define PWR_PUCRC_PU14          (1 << 14)
#define PWR_PUCRC_PU15          (1 << 15)

/* PWR port C pull-down control register */

#define PWR_PDCRC_RESET         0x00000000
#define PWR_PDCRC_PD0           (1 << 0)
#define PWR_PDCRC_PD1           (1 << 1)
#define PWR_PDCRC_PD2           (1 << 2)
#define PWR_PDCRC_PD3           (1 << 3)
#define PWR_PDCRC_PD4           (1 << 4)
#define PWR_PDCRC_PD5           (1 << 5)
#define PWR_PDCRC_PD6           (1 << 6)
#define PWR_PDCRC_PD7           (1 << 7)
#define PWR_PDCRC_PD8           (1 << 8)
#define PWR_PDCRC_PD9           (1 << 9)
#define PWR_PDCRC_PD10          (1 << 10)
#define PWR_PDCRC_PD11          (1 << 11)
#define PWR_PDCRC_PD12          (1 << 12)
#define PWR_PDCRC_PD13          (1 << 13)
#define PWR_PDCRC_PD14          (1 << 14)
#define PWR_PDCRC_PD15          (1 << 15)

/* PWR port D pull-up control register */

#define PWR_PUCRD_RESET         0x00000000
#define PWR_PUCRD_PU0           (1 << 0)
#define PWR_PUCRD_PU1           (1 << 1)
#define PWR_PUCRD_PU2           (1 << 2)
#define PWR_PUCRD_PU3           (1 << 3)
#define PWR_PUCRD_PU4           (1 << 4)
#define PWR_PUCRD_PU5           (1 << 5)
#define PWR_PUCRD_PU6           (1 << 6)
#define PWR_PUCRD_PU7           (1 << 7)
#define PWR_PUCRD_PU8           (1 << 8)
#define PWR_PUCRD_PU9           (1 << 9)
#define PWR_PUCRD_PU10          (1 << 10)
#define PWR_PUCRD_PU11          (1 << 11)
#define PWR_PUCRD_PU12          (1 << 12)
#define PWR_PUCRD_PU13          (1 << 13)
#define PWR_PUCRD_PU14          (1 << 14)
#define PWR_PUCRD_PU15          (1 << 15)

/* PWR port D pull-down control register */

#define PWR_PDCRD_RESET         0x00000000
#define PWR_PDCRD_PD0           (1 << 0)
#define PWR_PDCRD_PD1           (1 << 1)
#define PWR_PDCRD_PD2           (1 << 2)
#define PWR_PDCRD_PD3           (1 << 3)
#define PWR_PDCRD_PD4           (1 << 4)
#define PWR_PDCRD_PD5           (1 << 5)
#define PWR_PDCRD_PD6           (1 << 6)
#define PWR_PDCRD_PD7           (1 << 7)
#define PWR_PDCRD_PD8           (1 << 8)
#define PWR_PDCRD_PD9           (1 << 9)
#define PWR_PDCRD_PD10          (1 << 10)
#define PWR_PDCRD_PD11          (1 << 11)
#define PWR_PDCRD_PD12          (1 << 12)
#define PWR_PDCRD_PD13          (1 << 13)
#define PWR_PDCRD_PD14          (1 << 14)
#define PWR_PDCRD_PD15          (1 << 15)

/* PWR port E pull-up control register */

#define PWR_PUCRE_RESET         0x00000000
#define PWR_PUCRE_PU0           (1 << 0)
#define PWR_PUCRE_PU1           (1 << 1)
#define PWR_PUCRE_PU2           (1 << 2)
#define PWR_PUCRE_PU3           (1 << 3)
#define PWR_PUCRE_PU4           (1 << 4)
#define PWR_PUCRE_PU5           (1 << 5)
#define PWR_PUCRE_PU6           (1 << 6)
#define PWR_PUCRE_PU7           (1 << 7)
#define PWR_PUCRE_PU8           (1 << 8)
#define PWR_PUCRE_PU9           (1 << 9)
#define PWR_PUCRE_PU10          (1 << 10)
#define PWR_PUCRE_PU11          (1 << 11)
#define PWR_PUCRE_PU12          (1 << 12)
#define PWR_PUCRE_PU13          (1 << 13)
#define PWR_PUCRE_PU14          (1 << 14)
#define PWR_PUCRE_PU15          (1 << 15)

/* PWR port E pull-down control register */

#define PWR_PDCRE_RESET         0x00000000
#define PWR_PDCRE_PD0           (1 << 0)
#define PWR_PDCRE_PD1           (1 << 1)
#define PWR_PDCRE_PD2           (1 << 2)
#define PWR_PDCRE_PD3           (1 << 3)
#define PWR_PDCRE_PD4           (1 << 4)
#define PWR_PDCRE_PD5           (1 << 5)
#define PWR_PDCRE_PD6           (1 << 6)
#define PWR_PDCRE_PD7           (1 << 7)
#define PWR_PDCRE_PD8           (1 << 8)
#define PWR_PDCRE_PD9           (1 << 9)
#define PWR_PDCRE_PD10          (1 << 10)
#define PWR_PDCRE_PD11          (1 << 11)
#define PWR_PDCRE_PD12          (1 << 12)
#define PWR_PDCRE_PD13          (1 << 13)
#define PWR_PDCRE_PD14          (1 << 14)
#define PWR_PDCRE_PD15          (1 << 15)

/* PWR port F pull-up control register */

#define PWR_PUCRF_RESET         0x00000000
#define PWR_PUCRF_PU0           (1 << 0)
#define PWR_PUCRF_PU1           (1 << 1)
#define PWR_PUCRF_PU2           (1 << 2)
#define PWR_PUCRF_PU3           (1 << 3)
#define PWR_PUCRF_PU4           (1 << 4)
#define PWR_PUCRF_PU5           (1 << 5)
#define PWR_PUCRF_PU6           (1 << 6)
#define PWR_PUCRF_PU7           (1 << 7)
#define PWR_PUCRF_PU8           (1 << 8)
#define PWR_PUCRF_PU9           (1 << 9)
#define PWR_PUCRF_PU10          (1 << 10)
#define PWR_PUCRF_PU11          (1 << 11)
#define PWR_PUCRF_PU12          (1 << 12)
#define PWR_PUCRF_PU13          (1 << 13)
#define PWR_PUCRF_PU14          (1 << 14)
#define PWR_PUCRF_PU15          (1 << 15)

/* PWR port F pull-down control register */

#define PWR_PDCRF_RESET         0x00000000
#define PWR_PDCRF_PD0           (1 << 0)
#define PWR_PDCRF_PD1           (1 << 1)
#define PWR_PDCRF_PD2           (1 << 2)
#define PWR_PDCRF_PD3           (1 << 3)
#define PWR_PDCRF_PD4           (1 << 4)
#define PWR_PDCRF_PD5           (1 << 5)
#define PWR_PDCRF_PD6           (1 << 6)
#define PWR_PDCRF_PD7           (1 << 7)
#define PWR_PDCRF_PD8           (1 << 8)
#define PWR_PDCRF_PD9           (1 << 9)
#define PWR_PDCRF_PD10          (1 << 10)
#define PWR_PDCRF_PD11          (1 << 11)
#define PWR_PDCRF_PD12          (1 << 12)
#define PWR_PDCRF_PD13          (1 << 13)
#define PWR_PDCRF_PD14          (1 << 14)
#define PWR_PDCRF_PD15          (1 << 15)

/* PWR port G pull-up control register */

#define PWR_PUCRG_RESET         0x00000000
#define PWR_PUCRG_PU0           (1 << 0)
#define PWR_PUCRG_PU1           (1 << 1)
#define PWR_PUCRG_PU2           (1 << 2)
#define PWR_PUCRG_PU3           (1 << 3)
#define PWR_PUCRG_PU4           (1 << 4)
#define PWR_PUCRG_PU5           (1 << 5)
#define PWR_PUCRG_PU6           (1 << 6)
#define PWR_PUCRG_PU7           (1 << 7)
#define PWR_PUCRG_PU8           (1 << 8)
#define PWR_PUCRG_PU9           (1 << 9)
#define PWR_PUCRG_PU10          (1 << 10)
#define PWR_PUCRG_PU11          (1 << 11)
#define PWR_PUCRG_PU12          (1 << 12)
#define PWR_PUCRG_PU13          (1 << 13)
#define PWR_PUCRG_PU14          (1 << 14)
#define PWR_PUCRG_PU15          (1 << 15)

/* PWR port G pull-down control register */

#define PWR_PDCRG_RESET         0x00000000
#define PWR_PDCRG_PD0           (1 << 0)
#define PWR_PDCRG_PD1           (1 << 1)
#define PWR_PDCRG_PD2           (1 << 2)
#define PWR_PDCRG_PD3           (1 << 3)
#define PWR_PDCRG_PD4           (1 << 4)
#define PWR_PDCRG_PD5           (1 << 5)
#define PWR_PDCRG_PD6           (1 << 6)
#define PWR_PDCRG_PD7           (1 << 7)
#define PWR_PDCRG_PD8           (1 << 8)
#define PWR_PDCRG_PD9           (1 << 9)
#define PWR_PDCRG_PD10          (1 << 10)
#define PWR_PDCRG_PD11          (1 << 11)
#define PWR_PDCRG_PD12          (1 << 12)
#define PWR_PDCRG_PD13          (1 << 13)
#define PWR_PDCRG_PD14          (1 << 14)
#define PWR_PDCRG_PD15          (1 << 15)

/* PWR port H pull-up control register */

#define PWR_PUCRH_RESET         0x00000000
#define PWR_PUCRH_PU0           (1 << 0)
#define PWR_PUCRH_PU1           (1 << 1)
#define PWR_PUCRH_PU3           (1 << 3)

/* PWR port H pull-down control register */

#define PWR_PDCRH_RESET         0x00000000
#define PWR_PDCRH_PD0           (1 << 0)
#define PWR_PDCRH_PD1           (1 << 1)
#define PWR_PDCRH_PD3           (1 << 3)

/* PWR I3C pull-up control register 1 */

#define PWR_I3CPUCR1_RESET      0x00000000
#define PWR_I3CPUCR1_PA1_I3CPU  (1 << 0)
#define PWR_I3CPUCR1_PA6_I3CPU  (1 << 1)
#define PWR_I3CPUCR1_PA7_I3CPU  (1 << 2)
#define PWR_I3CPUCR1_PB2_I3CPU  (1 << 4)
#define PWR_I3CPUCR1_PB6_I3CPU  (1 << 5)
#define PWR_I3CPUCR1_PB8_I3CPU  (1 << 6)
#define PWR_I3CPUCR1_PB9_I3CPU  (1 << 7)
#define PWR_I3CPUCR1_PB10_I3CPU (1 << 8)
#define PWR_I3CPUCR1_PB12_I3CPU (1 << 9)
#define PWR_I3CPUCR1_PB13_I3CPU (1 << 10)
#define PWR_I3CPUCR1_PB14_I3CPU (1 << 11)
#define PWR_I3CPUCR1_PB7_I3CPU  (1 << 12)

/* PWR I3C pull-up control register 2 */

#define PWR_I3CPUCR2_RESET      0x00000000
#define PWR_I3CPUCR2_PC0_I3CPU  (1 << 0)
#define PWR_I3CPUCR2_PC1_I3CPU  (1 << 1)
#define PWR_I3CPUCR2_PD12_I3CPU (1 << 3)
#define PWR_I3CPUCR2_PD13_I3CPU (1 << 4)
#define PWR_I3CPUCR2_PG7_I3CPU  (1 << 6)
#define PWR_I3CPUCR2_PG8_I3CPU  (1 << 7)
#define PWR_I3CPUCR2_PG13_I3CPU (1 << 8)
#define PWR_I3CPUCR2_PG14_I3CPU (1 << 9)
#define PWR_I3CPUCR2_PH3_I3CPU  (1 << 11)

#endif /* __ARCH_ARM_SRC_STM32U3_HARDWARE_STM32U3XX_PWR_H */
