/****************************************************************************
 * arch/arm/include/gd32w51x/chip.h
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

#ifndef __ARCH_ARM_INCLUDE_GD32W51X_CHIP_H
#define __ARCH_ARM_INCLUDE_GD32W51X_CHIP_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* GD32W515 family: Cortex-M33 (FPU, DSP, TrustZone) up to 180 MHz with an
 * on-chip 2.4 GHz 802.11 b/g/n radio.  Variants:
 *
 *   GD32W515PI - QFN56, 2048 KB SIP flash, 448 KB SRAM
 *   GD32W515TI - QFN36, 2048 KB SIP flash, 448 KB SRAM
 *   GD32W515TG - QFN36, 1024 KB SIP flash, 384 KB SRAM
 */

#if defined(CONFIG_ARCH_CHIP_GD32W515PI)
#  define GD32W51X_FLASH_SIZE          (2048 * 1024)
#  define GD32W51X_SRAM_SIZE           (448 * 1024)
#  define GD32W51X_QFN56               1
#elif defined(CONFIG_ARCH_CHIP_GD32W515TI)
#  define GD32W51X_FLASH_SIZE          (2048 * 1024)
#  define GD32W51X_SRAM_SIZE           (448 * 1024)
#elif defined(CONFIG_ARCH_CHIP_GD32W515TG)
#  define GD32W51X_FLASH_SIZE          (1024 * 1024)
#  define GD32W51X_SRAM_SIZE           (384 * 1024)
#else
#  error "Unknown GD32W51x chip variant"
#endif

/* NVIC priority levels.  The Cortex-M33 of the GD32W51x implements 4
 * priority bits (msbits [7:4]).
 */

#define NVIC_SYSH_PRIORITY_MIN        0xf0 /* All bits set: lowest priority */
#define NVIC_SYSH_PRIORITY_DEFAULT    0x80 /* Midpoint is the default */
#define NVIC_SYSH_PRIORITY_MAX        0x00 /* Zero is maximum priority */
#define NVIC_SYSH_PRIORITY_STEP       0x10 /* Four priority bits used */

#endif /* __ARCH_ARM_INCLUDE_GD32W51X_CHIP_H */
