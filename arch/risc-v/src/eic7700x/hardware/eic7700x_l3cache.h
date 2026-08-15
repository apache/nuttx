/****************************************************************************
 * arch/risc-v/src/eic7700x/hardware/eic7700x_l3cache.h
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

#ifndef __ARCH_RISCV_SRC_EIC7700X_HARDWARE_EIC7700X_L3CACHE_H
#define __ARCH_RISCV_SRC_EIC7700X_HARDWARE_EIC7700X_L3CACHE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/eic7700x_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The L3 cache controller, also the coherency manager.  Four kilobytes
 * here, TRM part 1 table 3-40.
 */

#define EIC7700X_L3CACHE_BASE 0x02010000ul

/* Configuration, read only.  TRM part 1 section 3.4.1 tables 3-129 and
 * 3-131.  Reads 0x060a1004 on this part: four banks, sixteen ways, 1024
 * sets, sixty four byte blocks.  The one place the block size can be had
 * from the silicon rather than the manual.
 */

#define EIC7700X_L3_CONFIG    (EIC7700X_L3CACHE_BASE + 0x000)

/* Flush64, write only.  TRM part 1 section 3.4.1 table 3-129.
 *
 * The value written is the physical address of a cache block, and the
 * manual is emphatic that all sixty four bits have to be written in a
 * single access or nothing happens at all: two thirty two bit writes are
 * not a substitute.  That is why this is written with putreg64().
 *
 * What the write does, in the manual's words, is "a write-back and
 * invalidate, meaning the contents are written to memory and L3, pL2, and
 * L1 cache lines are then invalidated".  The L3 is inclusive of the L1
 * data cache and back probes it, so one write per block is the whole
 * maintenance operation for the whole hierarchy.  There is nothing to do
 * per hart and nothing to do for the private L2.
 *
 * It is also the only operation there is.  There is no invalidate that
 * does not write back and no write back that does not invalidate, and
 * these cores have no Zicbom to offer an alternative.  eic7700x_cache.c
 * describes what that costs and what it requires of callers.
 */

#define EIC7700X_L3_FLUSH64   (EIC7700X_L3CACHE_BASE + 0x200)

/* Sixty four bytes at every level of the hierarchy: the thirty two
 * kilobyte four way L1 data cache, the two hundred and fifty six kilobyte
 * eight way private L2, and the four megabyte sixteen way L3.
 */

#define EIC7700X_L3_LINESIZE  (64)
#define EIC7700X_L1_DCACHE_SIZE (32 * 1024)

#endif /* __ARCH_RISCV_SRC_EIC7700X_HARDWARE_EIC7700X_L3CACHE_H */
