/****************************************************************************
 * arch/arm/include/efr32/chip.h
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

#ifndef __ARCH_ARM_INCLUDE_EFR32_CHIP_H
#define __ARCH_ARM_INCLUDE_EFR32_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* EFR32MG24 parts: Cortex-M33 (FPU, DSP, TrustZone) up to 78 MHz with a
 * 2.4 GHz radio.
 *
 *   EFR32MG24B210F1536IM48 - QFN48, 1536 KB flash, 256 KB RAM, +10 dBm
 */

#if defined(CONFIG_ARCH_CHIP_EFR32MG24B210F1536IM48)
#  define EFR32_FLASH_SIZE            (1536 * 1024)
#  define EFR32_SRAM_SIZE             (256 * 1024)
#  define EFR32_NUSART                1  /* USART0 */
#  define EFR32_NEUSART               2  /* EUSART0-1 */
#  define EFR32_NGPIO_PORTS           4  /* Ports A-D */
#else
#  error "Unknown EFR32 chip"
#endif

/* NVIC priority levels.  The Cortex-M33 of the EFR32MG24 implements 4
 * priority bits (msbits [7:4]).
 */

#define NVIC_SYSH_PRIORITY_MIN        0xf0 /* All bits set: lowest priority */
#define NVIC_SYSH_PRIORITY_DEFAULT    0x80 /* Midpoint is the default */
#define NVIC_SYSH_PRIORITY_MAX        0x00 /* Zero is maximum priority */
#define NVIC_SYSH_PRIORITY_STEP       0x10 /* Four priority bits used */

#endif /* __ARCH_ARM_INCLUDE_EFR32_CHIP_H */
