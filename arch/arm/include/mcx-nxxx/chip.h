/****************************************************************************
 * arch/arm/include/mcx-nxxx/chip.h
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

#ifndef __ARCH_ARM_INCLUDE_MCX_NXXX_CHIP_H
#define __ARCH_ARM_INCLUDE_MCX_NXXX_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Cache line sizes (in bytes) for the MCX_NXXX (Cortex-M33). MCX_NXXX has
 * two caches; LPCACHE which is the instruction cache, and CACHE64 which is
 * for FlexSPI (Not present on n23x-series).
 *
 * There is no generic data cache.
 */

#define ARMV8M_ICACHE_LINESIZE 16  /* 16 bytes (4 words, Cortex-M33 only) */
#define ARMV8M_DCACHE_LINESIZE 32  /* 32 bytes (8 words, FlexSPI only) */

/* NVIC priority levels *****************************************************/

/* Each priority field holds an 8-bit priority value, 0-15. The lower the
 * value, the greater the priority of the corresponding interrupt. nx94x uses
 * msbits [7:5] for priority and the rest are reserved.
 */

#define NVIC_SYSH_PRIORITY_MIN        0xe0 /* E0h is minimum priority */
#define NVIC_SYSH_PRIORITY_DEFAULT    0x80 /* Midpoint is the default */
#define NVIC_SYSH_PRIORITY_MAX        0x00 /* Zero is maximum priority */
#define NVIC_SYSH_PRIORITY_STEP       0x20 /* Step is the 5th bit */

#define NXXX_GPIO_NPORTS              6

#endif /* __ARCH_ARM_INCLUDE_MCX_NXXX_CHIP_H */
