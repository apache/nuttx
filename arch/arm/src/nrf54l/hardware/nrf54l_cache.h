/****************************************************************************
 * arch/arm/src/nrf54l/hardware/nrf54l_cache.h
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

#ifndef __ARCH_ARM_SRC_NRF54L_HARDWARE_NRF54L_CACHE_H
#define __ARCH_ARM_SRC_NRF54L_HARDWARE_NRF54L_CACHE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/nrf54l_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define NRF54L_CACHE_TASKS_INVALIDATECACHE_OFFSET 0x008 /* Invalidate cache */
#define NRF54L_CACHE_TASKS_INVALIDATELINE_OFFSET  0x014 /* Invalidate line */
#define NRF54L_CACHE_TASKS_ERASE_OFFSET           0x020 /* Erase cache */
#define NRF54L_CACHE_STATUS_OFFSET                0x400 /* Cache activity */
#define NRF54L_CACHE_ENABLE_OFFSET                0x404 /* Enable cache */
#define NRF54L_CACHE_LINEADDR_OFFSET              0x410 /* Line address */
#define NRF54L_CACHE_PROFILING_ENABLE_OFFSET      0x414 /* Profiling enable */
#define NRF54L_CACHE_PROFILING_HIT_OFFSET         0x418 /* Hit count */
#define NRF54L_CACHE_PROFILING_MISS_OFFSET        0x41c /* Miss count */
#define NRF54L_CACHE_PROFILING_LMISS_OFFSET       0x420 /* Line miss count */
#define NRF54L_CACHE_PROFILING_READS_OFFSET       0x424 /* Read count */
#define NRF54L_CACHE_PROFILING_WRITES_OFFSET      0x428 /* Write count */
#define NRF54L_CACHE_DEBUGLOCK_OFFSET             0x430 /* Debug lock */
#define NRF54L_CACHE_WRITELOCK_OFFSET             0x434 /* Update lock */

/* Register addresses *******************************************************/

#define NRF54L_ICACHE_TASKS_INVALIDATECACHE (NRF54L_ICACHE_BASE + NRF54L_CACHE_TASKS_INVALIDATECACHE_OFFSET)
#define NRF54L_ICACHE_TASKS_INVALIDATELINE  (NRF54L_ICACHE_BASE + NRF54L_CACHE_TASKS_INVALIDATELINE_OFFSET)
#define NRF54L_ICACHE_TASKS_ERASE           (NRF54L_ICACHE_BASE + NRF54L_CACHE_TASKS_ERASE_OFFSET)
#define NRF54L_ICACHE_STATUS                (NRF54L_ICACHE_BASE + NRF54L_CACHE_STATUS_OFFSET)
#define NRF54L_ICACHE_ENABLE                (NRF54L_ICACHE_BASE + NRF54L_CACHE_ENABLE_OFFSET)
#define NRF54L_ICACHE_LINEADDR              (NRF54L_ICACHE_BASE + NRF54L_CACHE_LINEADDR_OFFSET)
#define NRF54L_ICACHE_PROFILING_ENABLE      (NRF54L_ICACHE_BASE + NRF54L_CACHE_PROFILING_ENABLE_OFFSET)
#define NRF54L_ICACHE_PROFILING_HIT         (NRF54L_ICACHE_BASE + NRF54L_CACHE_PROFILING_HIT_OFFSET)
#define NRF54L_ICACHE_PROFILING_MISS        (NRF54L_ICACHE_BASE + NRF54L_CACHE_PROFILING_MISS_OFFSET)
#define NRF54L_ICACHE_PROFILING_LMISS       (NRF54L_ICACHE_BASE + NRF54L_CACHE_PROFILING_LMISS_OFFSET)
#define NRF54L_ICACHE_PROFILING_READS       (NRF54L_ICACHE_BASE + NRF54L_CACHE_PROFILING_READS_OFFSET)
#define NRF54L_ICACHE_PROFILING_WRITES      (NRF54L_ICACHE_BASE + NRF54L_CACHE_PROFILING_WRITES_OFFSET)
#define NRF54L_ICACHE_DEBUGLOCK             (NRF54L_ICACHE_BASE + NRF54L_CACHE_DEBUGLOCK_OFFSET)
#define NRF54L_ICACHE_WRITELOCK             (NRF54L_ICACHE_BASE + NRF54L_CACHE_WRITELOCK_OFFSET)

/* ENABLE register */

#define CACHE_ENABLE_ENABLE        (1 << 0)  /* Enable cache */

/* STATUS register */

#define CACHE_STATUS_BUSY          (1 << 0)

/* PROFILING.ENABLE register */

#define CACHE_PROFILING_ENABLE     (1 << 0)

/* DEBUGLOCK and WRITELOCK registers */

#define CACHE_DEBUGLOCK_LOCK       (1 << 0)
#define CACHE_WRITELOCK_LOCK       (1 << 0)

#endif /* __ARCH_ARM_SRC_NRF54L_HARDWARE_NRF54L_CACHE_H */
