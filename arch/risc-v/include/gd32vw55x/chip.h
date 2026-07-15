/****************************************************************************
 * arch/risc-v/include/gd32vw55x/chip.h
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

#ifndef __ARCH_RISCV_INCLUDE_GD32VW55X_CHIP_H
#define __ARCH_RISCV_INCLUDE_GD32VW55X_CHIP_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* GD32VW553 family: Nuclei N307 (rv32imafc + B + P), ECLIC, 160 MHz.
 * All family members share the same 320 KB SRAM and peripheral set.
 * Variants differ only in flash size and package/GPIO count:
 *
 *   GD32VW553KI - QFN32, 22 GPIO, 2048 KB flash
 *   GD32VW553KM - QFN32, 22 GPIO, 4096 KB flash
 *   GD32VW553HI - QFN40, 29 GPIO, 2048 KB flash
 *   GD32VW553HM - QFN40, 29 GPIO, 4096 KB flash
 */

#if defined(CONFIG_ARCH_CHIP_GD32VW553KI)
#  define GD32VW55X_FLASH_SIZE          (2048 * 1024)
#  define GD32VW55X_NGPIO               22
#elif defined(CONFIG_ARCH_CHIP_GD32VW553KM)
#  define GD32VW55X_FLASH_SIZE          (4096 * 1024)
#  define GD32VW55X_NGPIO               22
#elif defined(CONFIG_ARCH_CHIP_GD32VW553HI)
#  define GD32VW55X_FLASH_SIZE          (2048 * 1024)
#  define GD32VW55X_NGPIO               29
#elif defined(CONFIG_ARCH_CHIP_GD32VW553HM)
#  define GD32VW55X_FLASH_SIZE          (4096 * 1024)
#  define GD32VW55X_NGPIO               29
#else
#  error "Unknown GD32VW55x chip variant"
#endif

/* SRAM: 288 KB application + 32 KB shared (top, used by Wi-Fi RX) */

#define GD32VW55X_SRAM_SIZE             (320 * 1024)
#define GD32VW55X_APP_SRAM_SIZE         (288 * 1024)

#endif /* __ARCH_RISCV_INCLUDE_GD32VW55X_CHIP_H */
