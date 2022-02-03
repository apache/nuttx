/****************************************************************************
 * arch/xtensa/src/common/xtensa_cache.c
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

#include <arch/chip/core-isa.h>
#include <arch/xtensa/xtensa_corebits.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

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

#ifdef CONFIG_XTENSA_ICACHE
void up_enable_icache(void)
{
  uint32_t memctl = 0;

  __asm__ __volatile__ ("rsr %0, memctl\n" : "=r"(memctl) :);

  memctl &= ~MEMCTL_ICWU_MASK;
  memctl |= (XCHAL_ICACHE_WAYS << MEMCTL_ICWU_SHIFT);
  memctl |= MEMCTL_INV_EN;

  __asm__ __volatile__ ("wsr %0, memctl\n" : : "r"(memctl));
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

#ifdef CONFIG_XTENSA_ICACHE
void up_disable_icache(void)
{
  uint32_t memctl = 0;

  __asm__ __volatile__ ("rsr %0, memctl\n" : "=r"(memctl) :);

  memctl &= ~MEMCTL_ICWU_MASK;
  memctl |= MEMCTL_INV_EN;

  __asm__ __volatile__ ("wsr %0, memctl\n" : : "r"(memctl));
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
 *
 ****************************************************************************/

#ifdef CONFIG_XTENSA_ICACHE
void up_invalidate_icache(uintptr_t start, uintptr_t end)
{
  /* align to XCHAL_ICACHE_LINESIZE */

  start &= ~(XCHAL_ICACHE_LINESIZE - 1);

  for (; start < end; start += XCHAL_ICACHE_LINESIZE)
    {
      __asm__ __volatile__ ("ihi %0, 0\n" : : "r"(start));
    }

  __asm__ __volatile__ ("isync\n");
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

#ifdef CONFIG_XTENSA_ICACHE
void up_invalidate_icache_all(void)
{
  uint32_t index;

  for (index = 0; index < XCHAL_ICACHE_SIZE; index += XCHAL_ICACHE_LINESIZE)
    {
      __asm__ __volatile__ ("iii %0, 0\n": : "r"(index));
    };

  __asm__ __volatile__ ("isync\n");
}
#endif

/****************************************************************************
 * Name: up_lock_icache
 *
 * Description:
 *   Prefetch and lock the instruction cache within the specified region.
 *
 * Input Parameters:
 *   start - virtual start address of region
 *   end   - virtual end address of region + 1
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_XTENSA_ICACHE_LOCK
void up_lock_icache(uintptr_t start, uintptr_t end)
{
  /* align to XCHAL_ICACHE_LINESIZE */

  start &= ~(XCHAL_ICACHE_LINESIZE - 1);

  for (; start < end; start += XCHAL_ICACHE_LINESIZE)
    {
      __asm__ __volatile__ ("ipfl %0, 0\n": : "r"(start));
    };

  __asm__ __volatile__ ("isync\n");
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

#ifdef CONFIG_XTENSA_ICACHE_LOCK
void up_unlock_icache(uintptr_t start, uintptr_t end)
{
  /* align to XCHAL_ICACHE_LINESIZE */

  start &= ~(XCHAL_ICACHE_LINESIZE - 1);

  for (; start < end; start += XCHAL_ICACHE_LINESIZE)
    {
      __asm__ __volatile__ ("ihu %0, 0\n": : "r"(start));
    };

  __asm__ __volatile__ ("isync\n");
}
#endif

/****************************************************************************
 * Name: up_unlock_icache_all
 *
 * Description:
 *   Unlock the entire contents of instruction cache.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_XTENSA_ICACHE_LOCK
void up_unlock_icache_all(void)
{
  uint32_t index;

  for (index = 0; index < XCHAL_ICACHE_SIZE; index += XCHAL_ICACHE_LINESIZE)
    {
      __asm__ __volatile__ ("iiu %0, 0\n": : "r"(index));
    };

  __asm__ __volatile__ ("isync\n");
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
 ****************************************************************************/

#ifdef CONFIG_XTENSA_DCACHE
void up_enable_dcache(void)
{
  uint32_t memctl = 0;

  __asm__ __volatile__ ("rsr %0, memctl\n" : "=r"(memctl) :);

  /* set ways allocatable & ways use */

  memctl = memctl & ~(MEMCTL_DCWA_MASK | MEMCTL_DCWU_MASK);

  memctl |= (XCHAL_DCACHE_WAYS << MEMCTL_DCWA_SHIFT);
  memctl |= (XCHAL_DCACHE_WAYS << MEMCTL_DCWU_SHIFT);
  memctl |= MEMCTL_INV_EN;

  __asm__ __volatile__ ("wsr %0, memctl\n" : : "r"(memctl));
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

#ifdef CONFIG_XTENSA_DCACHE
void up_disable_dcache(void)
{
  uint32_t memctl = 0;

  __asm__ __volatile__ ("rsr %0, memctl\n" : "=r"(memctl) :);

  /* clear ways allocatable & ways use */

  memctl = memctl & ~(MEMCTL_DCWA_MASK | MEMCTL_DCWU_MASK);
  memctl |= MEMCTL_INV_EN;

  __asm__ __volatile__ ("wsr %0, memctl\n" : : "r"(memctl));
}
#endif

/****************************************************************************
 * Name: up_invalidate_dcache
 *
 * Description:
 *   Invalidate the data cache within the specified region; we will be
 *   performing a DMA operation in this region and we want to purge old data
 *   in the cache. Note that this function invalidates all cache ways
 *   in sets that could be associated with the address range, regardless of
 *   whether the address range is contained in the cache or not.
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
 *   the operation is pre-empted.
 *
 ****************************************************************************/

#ifdef CONFIG_XTENSA_DCACHE
void up_invalidate_dcache(uintptr_t start, uintptr_t end)
{
  if (start & (XCHAL_DCACHE_LINESIZE - 1))
    {
      /* Align to XCHAL_DCACHE_LINESIZE */

      start &= ~(XCHAL_DCACHE_LINESIZE - 1);
      __asm__ __volatile__ ("dhwbi %0, 0\n" : : "r"(start));
      start += XCHAL_DCACHE_LINESIZE;
    }

  for (; start + XCHAL_DCACHE_LINESIZE <= end;
       start += XCHAL_DCACHE_LINESIZE)
    {
      __asm__ __volatile__ ("dhi %0, 0\n" : : "r"(start));
    }

  if (start != end)
    {
      __asm__ __volatile__ ("dhwbi %0, 0\n" : : "r"(start));
    }

  __asm__ __volatile__ ("dsync\n");
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

#ifdef CONFIG_XTENSA_DCACHE
void up_invalidate_dcache_all(void)
{
  uint32_t index;

  for (index = 0; index < XCHAL_DCACHE_SIZE; index += XCHAL_DCACHE_LINESIZE)
    {
      __asm__ __volatile__ ("dii %0, 0\n" : : "r"(index));
    };

  __asm__ __volatile__ ("dsync\n");
}
#endif

/****************************************************************************
 * Name: up_clean_dcache
 *
 * Description:
 *   Clean the data cache within the specified region by flushing the
 *   contents of the data cache to memory.
 *
 *   NOTE: This operation is un-necessary if the DCACHE is configured in
 *   write-through mode.
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
 *   the operation is pre-empted.
 *
 ****************************************************************************/

#ifdef CONFIG_XTENSA_DCACHE
void up_clean_dcache(uintptr_t start, uintptr_t end)
{
  /* Align to XCHAL_DCACHE_SIZE */

  start &= ~(XCHAL_DCACHE_LINESIZE - 1);

  if ((end - start) >= XCHAL_DCACHE_SIZE)
    {
      return up_clean_dcache_all();
    }

  for (; start < end; start += XCHAL_DCACHE_LINESIZE)
    {
      __asm__ __volatile__ ("dhwb %0, 0\n" : : "r"(start));
    }

  __asm__ __volatile__ ("dsync\n");
}
#endif

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
 *   the operation is pre-empted.
 *
 ****************************************************************************/

#ifdef CONFIG_XTENSA_DCACHE
void up_clean_dcache_all(void)
{
  uint32_t index;

  for (index = 0; index < XCHAL_DCACHE_SIZE; index += XCHAL_DCACHE_LINESIZE)
    {
      __asm__ __volatile__ ("diwb %0, 0\n" : : "r"(index));
    };

  __asm__ __volatile__ ("dsync\n");
}
#endif

/****************************************************************************
 * Name: up_flush_dcache
 *
 * Description:
 *   Flush the data cache within the specified region by cleaning and
 *   invalidating the D cache.
 *
 *   NOTE: If DCACHE write-through is configured, then this operation is the
 *   same as up_invalidate_cache().
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
 *   the operation is pre-empted.
 *
 ****************************************************************************/

#ifdef CONFIG_XTENSA_DCACHE
void up_flush_dcache(uintptr_t start, uintptr_t end)
{
  /* Align to XCHAL_DCACHE_LINESIZE */

  start &= ~(XCHAL_DCACHE_LINESIZE - 1);

  if ((end - start) >= XCHAL_DCACHE_SIZE)
    {
      return up_clean_dcache_all();
    }

  for (; start < end; start += XCHAL_DCACHE_LINESIZE)
    {
      __asm__ __volatile__ ("dhwbi %0, 0\n" : : "r"(start));
    }

  __asm__ __volatile__ ("dsync\n");
}
#endif

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
 *   the operation is pre-empted.
 *
 ****************************************************************************/

#ifdef CONFIG_XTENSA_DCACHE
void up_flush_dcache_all(void)
{
  uint32_t index;

  for (index = 0; index < XCHAL_DCACHE_SIZE; index += XCHAL_DCACHE_LINESIZE)
    {
      __asm__ __volatile__ ("diwbi %0, 0\n" : : "r"(index));
    };

  __asm__ __volatile__ ("dsync\n");
}
#endif

/****************************************************************************
 * Name: up_lock_dcache
 *
 * Description:
 *   Prefetch and lock the data cache within the specified region.
 *
 * Input Parameters:
 *   start - virtual start address of region
 *   end   - virtual end address of region + 1
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_XTENSA_DCACHE_LOCK
void up_lock_dcache(uintptr_t start, uintptr_t end)
{
  /* align to XCHAL_DCACHE_LINESIZE */

  start &= ~(XCHAL_DCACHE_LINESIZE - 1);

  for (; start < end; start += XCHAL_DCACHE_LINESIZE)
    {
      __asm__ __volatile__ ("dpfl %0, 0\n": : "r"(start));
    };

  __asm__ __volatile__ ("dsync\n");
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

#ifdef CONFIG_XTENSA_DCACHE_LOCK
void up_unlock_dcache(uintptr_t start, uintptr_t end)
{
  /* align to XCHAL_DCACHE_LINESIZE */

  start &= ~(XCHAL_DCACHE_LINESIZE - 1);

  for (; start < end; start += XCHAL_DCACHE_LINESIZE)
    {
      __asm__ __volatile__ ("dhu %0, 0\n": : "r"(start));
    };

  __asm__ __volatile__ ("dsync\n");
}
#endif

/****************************************************************************
 * Name: up_unlock_dcache_all
 *
 * Description:
 *   Unlock the entire contents of data cache.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_XTENSA_DCACHE_LOCK
void up_unlock_dcache_all(void)
{
  uint32_t index;

  for (index = 0; index < XCHAL_DCACHE_SIZE; index += XCHAL_DCACHE_LINESIZE)
    {
      __asm__ __volatile__ ("diu %0, 0\n" : : "r"(index));
    };

  __asm__ __volatile__ ("dsync\n");
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

#if defined(CONFIG_XTENSA_ICACHE) && defined(CONFIG_XTENSA_DCACHE)
void up_coherent_dcache(uintptr_t addr, size_t len)
{
  if (len > 0)
    {
      up_clean_dcache(addr, addr + len);
      up_invalidate_icache(addr, addr + len);
    }
}
#endif
