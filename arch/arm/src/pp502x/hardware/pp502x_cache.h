/****************************************************************************
 * arch/arm/src/pp502x/hardware/pp502x_cache.h
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

#ifndef __ARCH_ARM_SRC_PP502X_HARDWARE_PP502X_CACHE_H
#define __ARCH_ARM_SRC_PP502X_HARDWARE_PP502X_CACHE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/pp502x_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Cache geometry: 8 KiB with 16 byte lines */

#define PP502X_CACHE_SIZE       8192
#define PP502X_CACHE_LINESIZE   16

/* Register Offsets *********************************************************/

/* PP502X_CACHE_CTL is in the cache control block, PP502X_CACHE_PRIORITY in
 * the device control block, and the other two in the remap block.
 */

#define PP502X_CACHE_CTL_OFFSET         0x0000
#define PP502X_CACHE_PRIORITY_OFFSET    0x0044
#define PP502X_CACHE_MASK_OFFSET        0x0040
#define PP502X_CACHE_OPERATION_OFFSET   0x0044

/* Register Addresses *******************************************************/

#define PP502X_CACHE_CTL        (PP502X_CACHECTL_BASE + \
                                 PP502X_CACHE_CTL_OFFSET)
#define PP502X_CACHE_PRIORITY   (PP502X_DEVCTL_BASE + \
                                 PP502X_CACHE_PRIORITY_OFFSET)
#define PP502X_CACHE_MASK       (PP502X_REMAP_BASE + PP502X_CACHE_MASK_OFFSET)
#define PP502X_CACHE_OPERATION  (PP502X_REMAP_BASE + \
                                 PP502X_CACHE_OPERATION_OFFSET)

/* One status word per line, on the CPU side */

#define PP502X_CACHE_STATUS_CPU 0xf0004000

/* Register Bitfield Definitions ********************************************/

/* CACHE_CTL */

#define PP502X_CACHE_CTL_ENABLE     (1 << 0)
#define PP502X_CACHE_CTL_RUN        (1 << 1)
#define PP502X_CACHE_CTL_INIT       (1 << 2)

/* CACHE_PRIORITY */

#define PP502X_CACHE_PRIORITY_CPU   (1 << 4)

/* CACHE_MASK: the windows at 0x0 and 0x20000000 are cached */

#define PP502X_CACHE_MASK_CACHED    0x00001c00

/* CACHE_OPERATION */

#define PP502X_CACHE_OP_INIT        0x00000fc0

#endif /* __ARCH_ARM_SRC_PP502X_HARDWARE_PP502X_CACHE_H */
