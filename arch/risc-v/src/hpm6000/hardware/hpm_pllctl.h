/****************************************************************************
 * arch/risc-v/src/hpm6000/hardware/hpm_pllctl.h
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

#ifndef __ARCH_RISCV_SRC_HPM6000_HARDWARE_HPM_PLLCTL_H
#define __ARCH_RISCV_SRC_HPM6000_HARDWARE_HPM_PLLCTL_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define HPM_PLLCTLV2_XTAL                 (HPM_PLLCTL_BASE + 0x0000)
#define HPM_PLLCTLV2_PLL0_MFI             (HPM_PLLCTL_BASE + 0x0080)
#define HPM_PLLCTLV2_PLL0_MFN             (HPM_PLLCTL_BASE + 0x0084)
#define HPM_PLLCTLV2_PLL0_MFD             (HPM_PLLCTL_BASE + 0x0088)
#define HPM_PLLCTLV2_PLL0_SS_STEP         (HPM_PLLCTL_BASE + 0x008c)
#define HPM_PLLCTLV2_PLL0_SS_STOP         (HPM_PLLCTL_BASE + 0x0090)
#define HPM_PLLCTLV2_PLL0_CONFIG          (HPM_PLLCTL_BASE + 0x0094)
#define HPM_PLLCTLV2_PLL0_LOCKTIME        (HPM_PLLCTL_BASE + 0x0098)
#define HPM_PLLCTLV2_PLL0_STEPTIME        (HPM_PLLCTL_BASE + 0x009c)
#define HPM_PLLCTLV2_PLL0_ADVANCED        (HPM_PLLCTL_BASE + 0x00a0)
#define HPM_PLLCTLV2_PLL0_DIV0            (HPM_PLLCTL_BASE + 0x00c0)
#define HPM_PLLCTLV2_PLL0_DIV1            (HPM_PLLCTL_BASE + 0x00c4)
#define HPM_PLLCTLV2_PLL0_DIV2            (HPM_PLLCTL_BASE + 0x00c8)

#define HPM_PLLCTLV2_PLL1_MFI             (HPM_PLLCTL_BASE + 0x0100)
#define HPM_PLLCTLV2_PLL1_MFN             (HPM_PLLCTL_BASE + 0x0104)
#define HPM_PLLCTLV2_PLL1_MFD             (HPM_PLLCTL_BASE + 0x0108)
#define HPM_PLLCTLV2_PLL1_SS_STEP         (HPM_PLLCTL_BASE + 0x010c)
#define HPM_PLLCTLV2_PLL1_SS_STOP         (HPM_PLLCTL_BASE + 0x0110)
#define HPM_PLLCTLV2_PLL1_CONFIG          (HPM_PLLCTL_BASE + 0x0114)
#define HPM_PLLCTLV2_PLL1_LOCKTIME        (HPM_PLLCTL_BASE + 0x0118)
#define HPM_PLLCTLV2_PLL1_STEPTIME        (HPM_PLLCTL_BASE + 0x011c)
#define HPM_PLLCTLV2_PLL1_ADVANCED        (HPM_PLLCTL_BASE + 0x0120)
#define HPM_PLLCTLV2_PLL1_DIV0            (HPM_PLLCTL_BASE + 0x0140)
#define HPM_PLLCTLV2_PLL1_DIV1            (HPM_PLLCTL_BASE + 0x0144)
#define HPM_PLLCTLV2_PLL1_DIV2            (HPM_PLLCTL_BASE + 0x0148)

#define HPM_PLLCTLV2_PLL2_MFI             (HPM_PLLCTL_BASE + 0x0180)
#define HPM_PLLCTLV2_PLL2_MFN             (HPM_PLLCTL_BASE + 0x0184)
#define HPM_PLLCTLV2_PLL2_MFD             (HPM_PLLCTL_BASE + 0x0188)
#define HPM_PLLCTLV2_PLL2_SS_STEP         (HPM_PLLCTL_BASE + 0x018c)
#define HPM_PLLCTLV2_PLL2_SS_STOP         (HPM_PLLCTL_BASE + 0x0190)
#define HPM_PLLCTLV2_PLL2_CONFIG          (HPM_PLLCTL_BASE + 0x0194)
#define HPM_PLLCTLV2_PLL2_LOCKTIME        (HPM_PLLCTL_BASE + 0x0198)
#define HPM_PLLCTLV2_PLL2_STEPTIME        (HPM_PLLCTL_BASE + 0x019c)
#define HPM_PLLCTLV2_PLL2_ADVANCED        (HPM_PLLCTL_BASE + 0x01a0)
#define HPM_PLLCTLV2_PLL2_DIV(n)          (HPM_PLLCTL_BASE + 0x01c0 + n * 0x04)

#define HPM_PLLCTLV2_XTAL_BUSY                   (0x80000000UL)
#define HPM_PLLCTLV2_XTAL_RESPONSE               (0x20000000UL)
#define HPM_PLLCTLV2_XTAL_ENABLE                 (0x10000000UL)
#define HPM_PLLCTLV2_XTAL_RAMP_TIME_SHIFT        (0U)
#define HPM_PLLCTLV2_XTAL_RAMP_TIME_MASK         (0xFFFFFUL)
#define HPM_PLLCTLV2_XTAL_RAMP_TIME(n) \
                           ((uint32_t)(n) << HPM_PLLCTLV2_XTAL_RAMP_TIME_SHIFT)

#define HPM_PLLCTLV2_PLL_MFI_BUSY                (0x80000000UL)
#define HPM_PLLCTLV2_PLL_MFI_RESPONSE            (0x20000000UL)
#define HPM_PLLCTLV2_PLL_MFI_ENABLE              (0x10000000UL)
#define HPM_PLLCTLV2_PLL_MFI_MFI_MASK            (0x7FU)
#define HPM_PLLCTLV2_PLL_MFI_MFI_SHIFT           (0U)
#define HPM_PLLCTLV2_PLL_MFI_MFI(n) \
                              ((uint32_t)(n) << HPM_PLLCTLV2_PLL_MFI_MFI_SHIFT)

#define HPM_PLLCTLV2_PLL_MFN_MFN_MASK            (0x3FFFFFFFUL)
#define HPM_PLLCTLV2_PLL_MFN_MFN_SHIFT           (0U)
#define HPM_PLLCTLV2_PLL_MFN_MFN(x) \
                                  ((uint32_t)(x) << PLLCTLV2_PLL_MFN_MFN_SHIFT)
#define HPM_PLLCTLV2_PLL_MFD_MFD_MASK            (0x3FFFFFFFUL)
#define HPM_PLLCTLV2_PLL_MFD_MFD_SHIFT           (0U)
#define HPM_PLLCTLV2_PLL_MFD_MFD(n) \
                                  ((uint32_t)(n) << PLLCTLV2_PLL_MFD_MFD_SHIFT)

#define HPM_PLLCTLV2_PLL_SS_STEP_STEP_MASK       (0x3FFFFFFFUL)
#define HPM_PLLCTLV2_PLL_SS_STEP_STEP_SHIFT      (0U)
#define HPM_PLLCTLV2_PLL_SS_STEP_STEP(n) \
                         ((uint32_t)(n) << HPM_PLLCTLV2_PLL_SS_STEP_STEP_SHIFT)
#define HPM_PLLCTLV2_PLL_SS_STOP_STOP_MASK       (0x3FFFFFFFUL)
#define HPM_PLLCTLV2_PLL_SS_STOP_STOP_SHIFT      (0U)
#define HPM_PLLCTLV2_PLL_SS_STOP_STOP(n) \
                             ((uint32_t)(n) << PLLCTLV2_PLL_SS_STOP_STOP_SHIFT)

#define HPM_PLLCTLV2_PLL_CONFIG_SPREAD           (0x100U)
#define HPM_PLLCTLV2_PLL_CONFIG_REFSEL           (0x1U)

#define HPM_PLLCTLV2_PLL_LOCKTIME_LOCKTIME_MASK  (0xFFFFU)
#define HPM_PLLCTLV2_PLL_LOCKTIME_LOCKTIME_SHIFT (0U)
#define HPM_PLLCTLV2_PLL_LOCKTIME_LOCKTIME(x) \
                        ((uint32_t)(x) << PLLCTLV2_PLL_LOCKTIME_LOCKTIME_SHIFT)

#define HPM_PLLCTLV2_PLL_STEPTIME_STEPTIME_MASK  (0xFFFFU)
#define HPM_PLLCTLV2_PLL_STEPTIME_STEPTIME_SHIFT (0U)
#define HPM_PLLCTLV2_PLL_STEPTIME_STEPTIME(x) \
                        ((uint32_t)(x) << PLLCTLV2_PLL_STEPTIME_STEPTIME_SHIFT)

#define HPM_PLLCTLV2_PLL_ADVANCED_SLOW           (0x10000000UL)
#define HPM_PLLCTLV2_PLL_ADVANCED_DITHER         (0x1000000UL)

#define HPM_PLLCTLV2_PLL_DIV_BUSY                (0x80000000UL)
#define HPM_PLLCTLV2_PLL_DIV_RESPONSE            (0x20000000UL)
#define HPM_PLLCTLV2_PLL_DIV_ENABLE              (0x10000000UL)
#define HPM_PLLCTLV2_PLL_DIV_DIV_MASK            (0x3FU)
#define HPM_PLLCTLV2_PLL_DIV_DIV_SHIFT           (0U)
#define HPM_PLLCTLV2_PLL_DIV_DIV(x) \
                              ((uint32_t)(x) << HPM_PLLCTLV2_PLL_DIV_DIV_SHIFT)
#  define HPM_PLLCTLV2_PLL_DIV_DIV0              (0UL)
#  define HPM_PLLCTLV2_PLL_DIV_DIV1              (1UL)
#  define HPM_PLLCTLV2_PLL_DIV_DIV2              (2UL)

#define HPM_PLLCTLV2_PLL_PLL0                    (0UL)
#define HPM_PLLCTLV2_PLL_PLL1                    (1UL)
#define HPM_PLLCTLV2_PLL_PLL2                    (2UL)

#endif /* __ARCH_RISCV_SRC_HPM6000_HARDWARE_HPM_PLLCTL_H */
