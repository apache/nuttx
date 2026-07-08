/****************************************************************************
 * arch/arm/src/rtl8720f/hardware/ameba_memorymap.h
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

#ifndef __ARCH_ARM_SRC_RTL8720F_HARDWARE_AMEBA_MEMORYMAP_H
#define __ARCH_ARM_SRC_RTL8720F_HARDWARE_AMEBA_MEMORYMAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* RTL8720F memory map (from SDK
 * component/soc/RTL8720F/project/ameba_layout.ld and
 * fwlib/include/hal_platform.h).
 *
 * On-chip SRAM is 512 KiB: 0x2000_0000 .. 0x2008_0000 (secure alias at
 * 0x3000_0000).  Partitioned low->high: 8 KiB shared KM4TZ/KM4NS RAM, the
 * KM4TZ IMG2 region (where NuttX runs), then the KM4NS IMG2 region
 * (~184-192 KiB, the prebuilt NP/device image).
 *
 * NuttX runs on km4tz as IMG2.  The heap is taken from the SDK image2 linker
 * symbols __bdram_heap_buffer_start__/_size__ (see ameba_allocateheap.c), so
 * the window below mainly feeds CONFIG_RAM_START/SIZE (kept in sync with the
 * board defconfig).
 */

/* Whole on-chip SRAM (secure alias base; km4tz runs in the secure world). */

#define AMEBA_SRAM_START   0x30000000
#define AMEBA_SRAM_SIZE    0x00080000   /* 512 KiB */

/* km4tz IMG2 RAM window (matches board CONFIG_RAM_START/SIZE). */

#define PRIMARY_RAM_START  0x30008000
#define PRIMARY_RAM_SIZE   0x00040000
#define PRIMARY_RAM_END    (PRIMARY_RAM_START + PRIMARY_RAM_SIZE)

#endif /* __ARCH_ARM_SRC_RTL8720F_HARDWARE_AMEBA_MEMORYMAP_H */
