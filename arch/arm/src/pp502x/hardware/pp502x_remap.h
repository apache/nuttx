/****************************************************************************
 * arch/arm/src/pp502x/hardware/pp502x_remap.h
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

#ifndef __ARCH_ARM_SRC_PP502X_HARDWARE_PP502X_REMAP_H
#define __ARCH_ARM_SRC_PP502X_HARDWARE_PP502X_REMAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/pp502x_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

/* Eight windows with a logical and a physical register each */

#define PP502X_REMAP_LOGICAL_OFFSET(n)  (8 * (n))
#define PP502X_REMAP_PHYSICAL_OFFSET(n) (8 * (n) + 4)

/* Register Addresses *******************************************************/

#define PP502X_REMAP_LOGICAL(n)  (PP502X_REMAP_BASE + \
                                  PP502X_REMAP_LOGICAL_OFFSET(n))
#define PP502X_REMAP_PHYSICAL(n) (PP502X_REMAP_BASE + \
                                  PP502X_REMAP_PHYSICAL_OFFSET(n))

/* Register Bitfield Definitions ********************************************/

/* REMAP_LOGICAL: compare address bits 29:25 (32 MiB) or 29:26 (64 MiB) */

#define PP502X_REMAP_MASK_32MB      0x3e00
#define PP502X_REMAP_MASK_64MB      0x3c00

/* REMAP_PHYSICAL: access permissions, and the low bits that are always set
 * for SDRAM
 */

#define PP502X_REMAP_PHYS_READ      (1 << 8)
#define PP502X_REMAP_PHYS_WRITE     (1 << 9)
#define PP502X_REMAP_PHYS_DATA      (1 << 10)
#define PP502X_REMAP_PHYS_CODE      (1 << 11)
#define PP502X_REMAP_PHYS_SDRAM     0x84

#define PP502X_REMAP_PHYS_SDRAM_FLAGS \
  (PP502X_REMAP_PHYS_READ | PP502X_REMAP_PHYS_WRITE | \
   PP502X_REMAP_PHYS_DATA | PP502X_REMAP_PHYS_CODE | PP502X_REMAP_PHYS_SDRAM)

#endif /* __ARCH_ARM_SRC_PP502X_HARDWARE_PP502X_REMAP_H */
