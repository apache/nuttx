/****************************************************************************
 * arch/mips/src/jz4780/jz4780_cache.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <assert.h>
#include <stdint.h>

#include <nuttx/cache.h>
#include <nuttx/debug.h>
#include <nuttx/spinlock.h>

#include <arch/board/board.h>

#include "jz4780_cache.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static spinlock_t g_cache_lock = SP_UNLOCKED;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef CONFIG_ARCH_ICACHE

/****************************************************************************
 * Name: up_get_icache_linesize
 *
 * Description:
 *   Get icache linesize
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Cache line size
 *
 ****************************************************************************/

size_t up_get_icache_linesize(void)
{
  return mips_icache_linesize;
}

/****************************************************************************
 * Name: up_get_icache_size
 *
 * Description:
 *   Get icache size
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Cache size
 *
 ****************************************************************************/

size_t up_get_icache_size(void)
{
  return mips_icache_size;
}

/****************************************************************************
 * Name: up_invalidate_icache_all
 *
 * Description:
 *   Invalidate all instruction caches to PoU, also flushes branch target
 *   cache
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void up_invalidate_icache_all(void)
{
}

/****************************************************************************
 * Name: up_invalidate_icache
 *
 * Description:
 *   Validate the specified range instruction cache as PoU,
 *   and flush the branch target cache
 *
 * Input Parameters:
 *   start - virtual start address of region
 *   end   - virtual end address of region + 1
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void up_invalidate_icache(uintptr_t start, uintptr_t end)
{
}

/****************************************************************************
 * Name: up_enable_icache
 *
 * Description:
 *   Enable the I-Cache
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void up_enable_icache(void)
{
}

/****************************************************************************
 * Name: up_disable_icache
 *
 * Description:
 *   Disable the I-Cache
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void up_disable_icache(void)
{
}

#endif /* CONFIG_ARCH_ICACHE */

#ifdef CONFIG_ARCH_DCACHE

/****************************************************************************
 * Name: up_get_dcache_linesize
 *
 * Description:
 *   Get dcache linesize
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Cache line size
 *
 ****************************************************************************/

size_t up_get_dcache_linesize(void)
{
  return mips_dcache_linesize;
}

/****************************************************************************
 * Name: up_get_dcache_size
 *
 * Description:
 *   Get dcache size
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Cache size
 *
 ****************************************************************************/

size_t up_get_dcache_size(void)
{
  return mips_dcache_size;
}

/****************************************************************************
 * Name: up_invalidate_dcache
 *
 * Description:
 *   Invalidate the data cache within the specified region; we will be
 *   performing a DMA operation in this region and we want to purge old data
 *   in the cache.
 *
 * Input Parameters:
 *   start - virtual start address of region
 *   end   - virtual end address of region + 1
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   This operation is not atomic.  This function assumes that the caller
 *   has exclusive access to the address range so that no harm is done if
 *   the operation is preempted.
 *
 ****************************************************************************/

void up_invalidate_dcache(uintptr_t start, uintptr_t end)
{
  if (end > start)
    {
      if ((end - start) % mips_dcache_linesize == 0)
        {
          m32_dcache_invalidate(start, end - start);
        }
      else
        {
          m32_dcache_clean_invalidate(start, end - start);
        }
    }
}

/****************************************************************************
 * Name: up_invalidate_dcache_all
 *
 * Description:
 *   Invalidate the entire contents of D cache.
 *
 *   NOTE: This function forces L1 and L2 cache operations to be atomic
 *   by disabling interrupts.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void up_invalidate_dcache_all(void)
{
}

/****************************************************************************
 * Name: up_clean_dcache
 *
 * Description:
 *   Clean the data cache within the specified region by flushing the
 *   contents of the data cache to memory.
 *
 * Input Parameters:
 *   start - virtual start address of region
 *   end   - virtual end address of region + 1
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   This operation is not atomic.  This function assumes that the caller
 *   has exclusive access to the address range so that no harm is done if
 *   the operation is preempted.
 *
 ****************************************************************************/

void up_clean_dcache(uintptr_t start, uintptr_t end)
{
  if (end > start)
    {
      m32_dcache_clean(start, end - start);
    }
}

/****************************************************************************
 * Name: up_clean_dcache_all
 *
 * Description:
 *   Clean the entire data cache within the specified region by flushing the
 *   contents of the data cache to memory.
 *
 *   NOTE: This operation is un-necessary if the DCACHE is configured in
 *   write-through mode.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   This operation is not atomic.  This function assumes that the caller
 *   has exclusive access to the address range so that no harm is done if
 *   the operation is preempted.
 *
 ****************************************************************************/

void up_clean_dcache_all(void)
{
}

/****************************************************************************
 * Name: up_flush_dcache
 *
 * Description:
 *   Flush the data cache within the specified region by cleaning and
 *   invalidating the D cache.
 *
 * Input Parameters:
 *   start - virtual start address of region
 *   end   - virtual end address of region + 1
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   This operation is not atomic.  This function assumes that the caller
 *   has exclusive access to the address range so that no harm is done if
 *   the operation is preempted.
 *
 ****************************************************************************/

void up_flush_dcache(uintptr_t start, uintptr_t end)
{
  if (start % mips_dcache_linesize != 0)
    {
      start = start - start % mips_dcache_linesize;
    }

  if (end > start)
    {
      m32_dcache_clean_invalidate(start, end - start);
    }
}

/****************************************************************************
 * Name: up_flush_dcache_all
 *
 * Description:
 *   Flush the entire data cache by cleaning and invalidating the D cache.
 *
 *   NOTE: If DCACHE write-through is configured, then this operation is the
 *   same as up_invalidate_cache_all().
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   This operation is not atomic.  This function assumes that the caller
 *   has exclusive access to the address range so that no harm is done if
 *   the operation is preempted.
 *
 ****************************************************************************/

void up_flush_dcache_all(void)
{
  irqstate_t flags;
  flags = spin_lock_irqsave(&g_cache_lock);
  m32_flush_dcache();
  spin_unlock_irqrestore(&g_cache_lock, flags);
}

/****************************************************************************
 * Name: up_enable_dcache
 *
 * Description:
 *   Enable the D-Cache
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void up_enable_dcache(void)
{
}

/****************************************************************************
 * Name: up_disable_dcache
 *
 * Description:
 *   Disable the D-Cache
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void up_disable_dcache(void)
{
}

/****************************************************************************
 * Name: up_coherent_dcache
 *
 * Description:
 *   Ensure that the I and D caches are coherent within specified region
 *   by cleaning the D cache (i.e., flushing the D cache contents to memory
 *   and invalidating the I cache. This is typically used when code has been
 *   written to a memory region, and will be executed.
 *
 * Input Parameters:
 *   addr - virtual start address of region
 *   len  - Size of the address region in bytes
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void up_coherent_dcache(uintptr_t addr, size_t len)
{
}

#endif /* CONFIG_ARCH_DCACHE */
