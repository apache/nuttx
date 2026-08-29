/****************************************************************************
 * arch/arm/src/nrf53/hardware/nrf53_cache.h
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

#ifndef __ARCH_ARM_SRC_NRF53_HARDWARE_NRF53_CACHE_H
#define __ARCH_ARM_SRC_NRF53_HARDWARE_NRF53_CACHE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/nrf53_memorymap.h"

/* The application core CACHE peripheral (flash instruction/data cache).
 * Distinct from the nRF52-era NVMC ICACHECNF register, which does not
 * exist on this part.
 */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define NRF53_CACHE_ENABLE_OFFSET          0x500  /* Enable the cache */
#define NRF53_CACHE_INVALIDATE_OFFSET      0x504  /* Invalidate the cache */
#define NRF53_CACHE_INFO_OFFSET            0x508  /* Cache info */
#define NRF53_CACHE_PROFILINGENABLE_OFFSET 0x518  /* Profiling enable */
#define NRF53_CACHE_MODE_OFFSET            0x51c  /* Cache mode */

/* Register addresses *******************************************************/

#define NRF53_CACHE_ENABLE         (NRF53_CACHE_BASE + NRF53_CACHE_ENABLE_OFFSET)
#define NRF53_CACHE_INVALIDATE     (NRF53_CACHE_BASE + NRF53_CACHE_INVALIDATE_OFFSET)
#define NRF53_CACHE_INFO           (NRF53_CACHE_BASE + NRF53_CACHE_INFO_OFFSET)
#define NRF53_CACHE_PROFILINGENABLE (NRF53_CACHE_BASE + NRF53_CACHE_PROFILINGENABLE_OFFSET)
#define NRF53_CACHE_MODE           (NRF53_CACHE_BASE + NRF53_CACHE_MODE_OFFSET)

/* ENABLE Register **********************************************************/

#define CACHE_ENABLE_ENABLE        (1 << 0)  /* Enable cache */

/* INVALIDATE Register ******************************************************/

#define CACHE_INVALIDATE_INVALIDATE (1 << 0) /* Invalidate cache */

#endif /* __ARCH_ARM_SRC_NRF53_HARDWARE_NRF53_CACHE_H */
