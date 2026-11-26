/****************************************************************************
 * arch/arm/src/efr32/hardware/efr32mg24_memorymap.h
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

#ifndef __ARCH_ARM_SRC_EFR32_HARDWARE_EFR32MG24_MEMORYMAP_H
#define __ARCH_ARM_SRC_EFR32_HARDWARE_EFR32MG24_MEMORYMAP_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* NuttX runs in secure state, so the peripherals are reached through their
 * secure aliases.  The non-secure aliases are at +0x10000000.
 */

/* Memories */

#define EFR32_FLASH_BASE         0x08000000  /* Main flash */
#define EFR32_FLASH_PAGESIZE     8192        /* Flash page size */
#define EFR32_USERDATA_BASE      0x0fe00000  /* User data page */
#define EFR32_DEVINFO_BASE       0x0fe08000  /* Device information page */
#define EFR32_SRAM_BASE          0x20000000  /* RAM */

/* Peripherals (secure aliases) */

#define EFR32_EMU_BASE           0x40004000
#define EFR32_CMU_BASE           0x40008000
#define EFR32_HFRCO0_BASE        0x40010000
#define EFR32_FSRCO_BASE         0x40018000
#define EFR32_DPLL0_BASE         0x4001c000
#define EFR32_LFXO_BASE          0x40020000
#define EFR32_LFRCO_BASE         0x40024000
#define EFR32_ULFRCO_BASE        0x40028000
#define EFR32_MSC_BASE           0x40030000
#define EFR32_ICACHE0_BASE       0x40034000
#define EFR32_PRS_BASE           0x40038000
#define EFR32_GPIO_BASE          0x4003c000
#define EFR32_LDMA_BASE          0x40040000
#define EFR32_LDMAXBAR_BASE      0x40044000
#define EFR32_TIMER0_BASE        0x40048000
#define EFR32_TIMER1_BASE        0x4004c000
#define EFR32_TIMER2_BASE        0x40050000
#define EFR32_TIMER3_BASE        0x40054000
#define EFR32_TIMER4_BASE        0x40058000
#define EFR32_USART0_BASE        0x4005c000
#define EFR32_BURTC_BASE         0x40064000
#define EFR32_I2C1_BASE          0x40068000
#define EFR32_SYSCFG_BASE        0x4007c000
#define EFR32_BURAM_BASE         0x40080000
#define EFR32_GPCRC_BASE         0x40088000
#define EFR32_DCDC_BASE          0x40094000
#define EFR32_EUSART1_BASE       0x400a0000
#define EFR32_SYSRTC0_BASE       0x400a8000
#define EFR32_SMU_BASE           0x44008000
#define EFR32_LETIMER0_BASE      0x49000000
#define EFR32_IADC0_BASE         0x49004000
#define EFR32_HFRCOEM23_BASE     0x4a000000
#define EFR32_HFXO0_BASE         0x4a004000
#define EFR32_I2C0_BASE          0x4b000000
#define EFR32_WDOG0_BASE         0x4b004000
#define EFR32_WDOG1_BASE         0x4b008000
#define EFR32_EUSART0_BASE       0x4b010000

#endif /* __ARCH_ARM_SRC_EFR32_HARDWARE_EFR32MG24_MEMORYMAP_H */
