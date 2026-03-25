/****************************************************************************
 * arch/arm/include/ht32f491x3/chip.h
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

#ifndef __ARCH_ARM_INCLUDE_HT32F491X3_CHIP_H
#define __ARCH_ARM_INCLUDE_HT32F491X3_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define HT32M4_SYSH_PRIORITY_MIN     0xe0
#define HT32M4_SYSH_PRIORITY_DEFAULT 0x80
#define HT32M4_SYSH_PRIORITY_MAX     0x00
#define HT32M4_SYSH_PRIORITY_STEP    0x20

#define NVIC_SYSH_PRIORITY_MIN        HT32M4_SYSH_PRIORITY_MIN
#define NVIC_SYSH_PRIORITY_DEFAULT    HT32M4_SYSH_PRIORITY_DEFAULT
#define NVIC_SYSH_PRIORITY_MAX        HT32M4_SYSH_PRIORITY_MAX
#define NVIC_SYSH_PRIORITY_STEP       HT32M4_SYSH_PRIORITY_STEP

#if defined(CONFIG_ARCH_CHIP_HT32F49153)
#  define HT32_FLASH_KB 128
#elif defined(CONFIG_ARCH_CHIP_HT32F49163)
#  define HT32_FLASH_KB 256
#else
#  error "Unsupported HT32F491x3 device"
#endif

#define HT32_SRAM_KB             48
#define HT32_NUSART              8
#define HT32_NGPIO               6

#define HT32_HICK_FREQUENCY       8000000
#define HT32_HICK48_FREQUENCY    48000000
#define HT32_HEXT_MIN_FREQUENCY   4000000
#define HT32_HEXT_MAX_FREQUENCY  25000000
#define HT32_PLL_MAX_FREQUENCY  150000000

#define HT32_SYSCLK_FREQUENCY CONFIG_HT32F491X3_SYSCLK_FREQUENCY
#define HT32_HCLK_FREQUENCY   HT32_SYSCLK_FREQUENCY
#define HT32_PCLK1_FREQUENCY  CONFIG_HT32F491X3_PCLK1_FREQUENCY
#define HT32_PCLK2_FREQUENCY  CONFIG_HT32F491X3_PCLK2_FREQUENCY

#endif /* __ARCH_ARM_INCLUDE_HT32F491X3_CHIP_H */
