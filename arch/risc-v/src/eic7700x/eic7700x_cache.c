/****************************************************************************
 * arch/risc-v/src/eic7700x/eic7700x_cache.c
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

/* Data cache maintenance on a SoC that is not cache coherent.
 *
 * No device that moves data on its own snoops the harts' caches or is
 * snooped by them, so software maintains the cache around every transfer.
 * The harts are coherent with each other; it is DMA that is not.
 *
 * These cores have no Zicbom, so maintenance is a store to the L3
 * controller carrying a block's physical address, one per sixty four byte
 * block.  TRM part 1 section 3.4.1; see hardware/eic7700x_l3cache.h.
 *
 * That store both writes back and invalidates, and is the only operation
 * there is.  Two rules follow, both the caller's to keep:
 *
 *   A range being invalidated must own whole blocks, sixty four byte
 *   aligned at both ends, which up_invalidate_dcache() asserts.  A shared
 *   end block is written back as well as dropped, so a neighbour dirtied
 *   during the transfer lands on top of what the device delivered.
 *
 *   Cleaning has no alignment requirement: writing back more than was
 *   asked costs a refill and nothing else.
 *
 * The rules are the chip's, not any one driver's.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <assert.h>
#include <stdint.h>

#include <nuttx/cache.h>
#include <arch/barriers.h>

#include "riscv_internal.h"
#include "hardware/eic7700x_l3cache.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: dcache_flush_pa
 *
 * Description:
 *   Write back and invalidate every cache block covering a range of
 *   physical addresses.
 *
 * Input Parameters:
 *   pstart - physical start address of the region
 *   pend   - physical end address of the region + 1
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void dcache_flush_pa(uintptr_t pstart, uintptr_t pend)
{
  uintptr_t pa;

  /* Round out to whole blocks: the register takes a block address. */

  pa   = pstart & ~(uintptr_t)(EIC7700X_L3_LINESIZE - 1);
  pend = (pend + EIC7700X_L3_LINESIZE - 1) &
         ~(uintptr_t)(EIC7700X_L3_LINESIZE - 1);

  /* Both fences order input and output as well as memory: what follows is
   * typically the store that starts a device, which a memory-only fence
   * would not order.
   */

  UP_DSB();

  for (; pa < pend; pa += EIC7700X_L3_LINESIZE)
    {
      putreg64(pa, EIC7700X_L3_FLUSH64);
    }

  UP_DSB();
}

/****************************************************************************
 * Name: dcache_range
 *
 * Description:
 *   The body of all three range operations.  Kernel memory is mapped
 *   virtual equals physical here, so translation is a range check.
 *
 *   An address outside that window is refused: this interface does not
 *   say which address environment a user address belongs to, so a caller
 *   reaching a user buffer translates it first and passes physical
 *   addresses, which are valid kernel addresses here.
 *
 * Input Parameters:
 *   start - virtual start address of the region
 *   end   - virtual end address of the region + 1
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void dcache_range(uintptr_t start, uintptr_t end)
{
  if (start >= end)
    {
      return;
    }

  DEBUGASSERT(start >= CONFIG_RAM_START && end <= (uintptr_t)CONFIG_RAM_END);

  if (start < CONFIG_RAM_START || end > (uintptr_t)CONFIG_RAM_END)
    {
      return;
    }

  dcache_flush_pa(start, end);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_get_dcache_linesize
 * Returned Value:
 *   The cache block size in bytes.
 *
 ****************************************************************************/

size_t up_get_dcache_linesize(void)
{
  return EIC7700X_L3_LINESIZE;
}

/****************************************************************************
 * Name: up_get_dcache_size
 *
 * Description:
 *   The first level data cache only.  A flush reaches the private L2 and
 *   the L3 as well, so this is not what one covers.
 *
 * Returned Value:
 *   The size of one level one data cache in bytes.
 *
 ****************************************************************************/

size_t up_get_dcache_size(void)
{
  return EIC7700X_L1_DCACHE_SIZE;
}

/****************************************************************************
 * Name: up_enable_dcache / up_disable_dcache
 *
 * Description:
 *   Nothing to do.  The caches are running before this port is entered and
 *   there is no supervisor accessible control that would turn them off.
 *
 ****************************************************************************/

void up_enable_dcache(void)
{
}

void up_disable_dcache(void)
{
}

/****************************************************************************
 * Name: up_clean_dcache
 *
 * Description:
 *   Write the range back so a device about to read it sees what the
 *   processor wrote.  Safe at any alignment.
 *
 * Input Parameters:
 *   start - virtual start address of the region
 *   end   - virtual end address of the region + 1
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void up_clean_dcache(uintptr_t start, uintptr_t end)
{
  dcache_range(start, end);
}

/****************************************************************************
 * Name: up_flush_dcache
 *
 * Description:
 *   Write back and invalidate.  This is the operation the hardware has;
 *   the other two are it under different names.
 *
 * Input Parameters:
 *   start - virtual start address of the region
 *   end   - virtual end address of the region + 1
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void up_flush_dcache(uintptr_t start, uintptr_t end)
{
  dcache_range(start, end);
}

/****************************************************************************
 * Name: up_invalidate_dcache
 *
 * Description:
 *   Discard the range so the next read comes from memory, where a device
 *   has just left something.  Must be block aligned at both ends; the
 *   file's opening comment says why.
 *
 * Input Parameters:
 *   start - virtual start address of the region
 *   end   - virtual end address of the region + 1
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void up_invalidate_dcache(uintptr_t start, uintptr_t end)
{
  DEBUGASSERT((start & (EIC7700X_L3_LINESIZE - 1)) == 0 &&
              (end & (EIC7700X_L3_LINESIZE - 1)) == 0);

  dcache_range(start, end);
}

/****************************************************************************
 * Name: up_invalidate_dcache_all / up_clean_dcache_all / up_flush_dcache_all
 *
 * Description:
 *   Order what has been written, and nothing else.
 *
 *   The hardware has no whole-hierarchy operation, and naming every block
 *   in turn would be fifteen and a half million stores, on a path two of
 *   the three callers reach only after the system has gone wrong.
 *
 *   Nor is one needed between harts: the L3 is a directory based coherency
 *   manager and the L1 data caches are coherent with each other (TRM part
 *   1 section 3.4).  Ordering is what is left.  Only DMA is non coherent,
 *   and a DMA buffer is always given as a range.
 *
 ****************************************************************************/

void up_invalidate_dcache_all(void)
{
  UP_DSB();
}

void up_clean_dcache_all(void)
{
  UP_DSB();
}

void up_flush_dcache_all(void)
{
  UP_DSB();
}
