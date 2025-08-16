/****************************************************************************
 * arch/xtensa/src/esp32s3/esp32s3_cache.c
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
#include <nuttx/cache.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * ROM Function Prototypes
 ****************************************************************************/

extern uint32_t cache_get_icache_line_size(void);
extern void cache_resume_icache(uint32_t autoload);
extern uint32_t cache_suspend_icache(void);
extern void cache_invalidate_icache_items(uint32_t addr, uint32_t items);
extern void cache_invalidate_icache_all(void);
extern void cache_lock_icache_items(uint32_t addr, uint32_t items);
extern void cache_unlock_icache_items(uint32_t addr, uint32_t items);

extern uint32_t cache_get_dcache_line_size(void);
extern void cache_resume_dcache(uint32_t autoload);
extern uint32_t cache_suspend_dcache(void);
extern void cache_invalidate_dcache_items(uint32_t addr, uint32_t items);
extern void cache_invalidate_dcache_all(void);
extern int cache_writeback_addr(uint32_t addr, uint32_t size);
extern void cache_writeback_all(void);
extern void cache_lock_dcache_items(uint32_t addr, uint32_t items);
extern void cache_unlock_dcache_items(uint32_t addr, uint32_t items);

/****************************************************************************
 * Public Functions
 ****************************************************************************/

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

#ifdef CONFIG_ARCH_ICACHE
size_t up_get_icache_linesize(void)
{
  return (size_t)cache_get_icache_line_size();
}
#endif

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

#ifdef CONFIG_ARCH_ICACHE
size_t up_get_icache_size(void)
{
  return (size_t)CONFIG_ESP32S3_INSTRUCTION_CACHE_SIZE;
}
#endif

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
 * Caution:
 *   The writable global variables aren't initialized yet.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_ICACHE
void up_enable_icache(void)
{
  cache_resume_icache(0);
}
#endif

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

#ifdef CONFIG_ARCH_ICACHE
void up_disable_icache(void)
{
  cache_suspend_icache();
}
#endif

/****************************************************************************
 * Name: up_invalidate_icache
 *
 * Description:
 *   Invalidate the instruction cache within the specified region.
 *
 * Input Parameters:
 *   start - virtual start address of region
 *   end   - virtual end address of region + 1
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_ICACHE
void up_invalidate_icache(uintptr_t start, uintptr_t end)
{
  uint32_t items = (end - start) / up_get_icache_linesize();

  cache_invalidate_icache_items((uint32_t)start, items);
}
#endif

/****************************************************************************
 * Name: up_invalidate_icache_all
 *
 * Description:
 *   Invalidate the entire contents of I cache.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_ICACHE
void up_invalidate_icache_all(void)
{
  cache_invalidate_icache_all();
}
#endif

/****************************************************************************
 * Name: up_lock_icache
 *
 * Description:
 *   Prefetch and lock the instruction cache within the specified region.
 *   If the specified address if not present in the instruction cache,
 *   some architectures transfer the line from memory, others wait the
 *   address be read from memory, and then lock.
 *
 * Input Parameters:
 *   start - virtual start address of region
 *   end   - virtual end address of region + 1
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_ICACHE_LOCK
void up_lock_icache(uintptr_t start, uintptr_t end)
{
  uint32_t items = (end - start) / up_get_icache_linesize();

  cache_lock_icache_items((uint32_t)start, items);
}
#endif

/****************************************************************************
 * Name: up_unlock_icache
 *
 * Description:
 *   Unlock the instruction cache within the specified region.
 *
 * Input Parameters:
 *   start - virtual start address of region
 *   end   - virtual end address of region + 1
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_ICACHE_LOCK
void up_unlock_icache(uintptr_t start, uintptr_t end)
{
  uint32_t items = (end - start) / up_get_icache_linesize();

  cache_unlock_icache_items((uint32_t)start, items);
}
#endif

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

#ifdef CONFIG_ARCH_DCACHE
size_t up_get_dcache_linesize(void)
{
  return (size_t)cache_get_dcache_line_size();
}
#endif

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

#ifdef CONFIG_ARCH_DCACHE
size_t up_get_dcache_size(void)
{
  return (size_t)CONFIG_ESP32S3_DATA_CACHE_SIZE;
}
#endif

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
 * Caution:
 *   The writable global variables aren't initialized yet.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_DCACHE
void up_enable_dcache(void)
{
  cache_resume_dcache(0);
}
#endif

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

#ifdef CONFIG_ARCH_DCACHE
void up_disable_dcache(void)
{
  cache_suspend_dcache();
}
#endif

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
 ****************************************************************************/

#ifdef CONFIG_ARCH_DCACHE
void up_invalidate_dcache(uintptr_t start, uintptr_t end)
{
  uint32_t items = (end - start) / up_get_dcache_linesize();

  cache_invalidate_dcache_items((uint32_t)start, end);
}
#endif

/****************************************************************************
 * Name: up_invalidate_dcache_all
 *
 * Description:
 *   Invalidate the entire contents of D cache.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_DCACHE
void up_invalidate_dcache_all(void)
{
  cache_invalidate_dcache_all();
}
#endif

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
 ****************************************************************************/

#ifdef CONFIG_ARCH_DCACHE
void up_clean_dcache(uintptr_t start, uintptr_t end)
{
  /* Please note that, according to ESP32-S3 TRM, the clean operation just
   * clears the dirty bits in the dirty block, without updating data to the
   * external memory. After the clean operation finishes, there will still be
   * old data stored in the external memory, while the cache keeps the new
   * one (but the cache does not know about this). That's why - according to
   * this function's description - we need to call the writeback operation.
   * The writeback operation will clear the dirty bits and updates the data
   * to the external memory.
   */

  cache_writeback_addr((uint32_t)start, (uint32_t)(end - start));
}
#endif

/****************************************************************************
 * Name: up_clean_dcache_all
 *
 * Description:
 *   Clean the entire data cache within the specified region by flushing the
 *   contents of the data cache to memory.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_DCACHE
void up_clean_dcache_all(void)
{
  cache_writeback_all();
}
#endif

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
 ****************************************************************************/

#ifdef CONFIG_ARCH_DCACHE
void up_flush_dcache(uintptr_t start, uintptr_t end)
{
  uint32_t items = (end - start) / up_get_dcache_linesize();

  /* Please note that, according to ESP32-S3 TRM, the clean operation just
   * clears the dirty bits in the dirty block, without updating data to the
   * external memory. After the clean operation finishes, there will still be
   * old data stored in the external memory, while the cache keeps the new
   * one (but the cache does not know about this). That's why - according to
   * this function's description - we need to call the writeback operation.
   * The writeback operation will clear the dirty bits and updates the data
   * to the external memory.
   */

  cache_writeback_addr((uint32_t)start, (uint32_t)(end - start));

  cache_invalidate_dcache_items((uint32_t)start, end);
}
#endif

/****************************************************************************
 * Name: up_flush_dcache_all
 *
 * Description:
 *   Flush the entire data cache by cleaning and invalidating the D cache.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_DCACHE
void up_flush_dcache_all(void)
{
  cache_writeback_all();
  cache_invalidate_dcache_all();
}
#endif

/****************************************************************************
 * Name: up_lock_dcache
 *
 * Description:
 *   Prefetch and lock the data cache within the specified region.
 *   If the specified address is not present in the data cache,
 *   some architectures transfer the line from memory, others wait the
 *   address be read from memory, and then lock.
 *
 * Input Parameters:
 *   start - virtual start address of region
 *   end   - virtual end address of region + 1
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_DCACHE_LOCK
void up_lock_dcache(uintptr_t start, uintptr_t end)
{
  uint32_t items = (end - start) / up_get_dcache_linesize();

  cache_lock_dcache_items((uint32_t)start, items);
}
#endif

/****************************************************************************
 * Name: up_unlock_dcache
 *
 * Description:
 *   Unlock the data cache within the specified region.
 *
 * Input Parameters:
 *   start - virtual start address of region
 *   end   - virtual end address of region + 1
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_DCACHE_LOCK
void up_unlock_dcache(uintptr_t start, uintptr_t end)
{
  uint32_t items = (end - start) / up_get_dcache_linesize();

  cache_unlock_dcache_items((uint32_t)start, items);
}
#endif

/****************************************************************************
 * Name: up_coherent_dcache
 *
 * Description:
 *   Ensure that the I and D caches are coherent within specified region
 *   by cleaning the D cache (i.e., flushing the D cache contents to memory)
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

#if defined(CONFIG_ARCH_ICACHE) && defined(CONFIG_ARCH_DCACHE)
void up_coherent_dcache(uintptr_t addr, size_t len)
{
  uint32_t items = (len) / up_get_dcache_linesize();

  cache_writeback_addr((uint32_t)addr, (uint32_t)len);
  cache_invalidate_icache_items((uint32_t)addr, items);
}
#endif
