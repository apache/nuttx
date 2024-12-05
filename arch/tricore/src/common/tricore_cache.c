/****************************************************************************
 * arch/tricore/src/common/tricore_cache.c
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

#include <stdint.h>

#include "tricore_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define tc_invalidate_cache_byaddr(addr) \
  __asm__ __volatile__("cachea.i [%0]0"::"a"(addr))
#define tc_clear_cache_byaddr(addr) \
  __asm__ __volatile__("cachea.w [%0]0"::"a"(addr))
#define tc_flush_cache_byaddr(addr) \
  __asm__ __volatile__("cachea.wi [%0]0"::"a"(addr))
#define tc_invalidate_cache_byline(line) \
  __asm__ __volatile__("cachei.i [%0]0"::"a"(line))
#define tc_clear_cache_byline(line) \
  __asm__ __volatile__("cachei.w [%0]0"::"a"(line))
#define tc_flush_cache_byline(line) \
  __asm__ __volatile__("cachei.wi [%0]0"::"a"(line))

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
  return IFXCPU_PCACHE_LINE_SIZE / 8;
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
  Ifx_CPU_PCON2 pcon2;

  pcon2.U = __mfcr(CPU_PCON2);
  return pcon2.B.PCACHE_SZE * 1000;
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
 * Caution:
 *   The writable global variables aren't initialized yet.
 *
 ****************************************************************************/

void up_enable_icache(void)
{
  Ifx_CPU_PCON0 pcon0;

  pcon0.B.PCBYP = 0;
  __mtcr(CPU_PCON0, pcon0.U);

  __isync();
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
  Ifx_CPU_PCON0 pcon0;

  pcon0.U       = 0;
  pcon0.B.PCBYP = 1;
  __mtcr(CPU_PCON0, pcon0.U);

  __isync();
}

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

void up_invalidate_icache(uintptr_t start, uintptr_t end)
{
  up_invalidate_icache_all();
}

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

void up_invalidate_icache_all(void)
{
  Ifx_CPU_PCON1 pcon1;

  pcon1.U       = 0;
  pcon1.B.PCINV = 1;
  __mtcr(CPU_PCON1, pcon1.U);

  __isync();
}
#endif

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
  return IFXCPU_DCACHE_LINE_SIZE / 8;
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
  Ifx_CPU_DCON2 dcon2;

  dcon2.U = __mfcr(CPU_DCON2);
  return dcon2.B.DCACHE_SZE * 1000;
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
 * Caution:
 *   The writable global variables aren't initialized yet.
 *
 ****************************************************************************/

void up_enable_dcache(void)
{
  Ifx_CPU_DCON0 dcon0;

  /* Check if the D-Cache is enabled */

  dcon0.U = __mfcr(CPU_DCON0);
  if (dcon0.B.DCBYP == 0)
    {
      return;
    }

  up_invalidate_dcache_all();

  dcon0.U       = 0;
  dcon0.B.DCBYP = 0;
  __mtcr(CPU_DCON0, dcon0.U);

  __dsync();
  __isync();
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
  Ifx_CPU_DCON0 dcon0;

  dcon0.B.DCBYP = 1;
  __mtcr(CPU_DCON0, dcon0.U);

  __dsync();
  __isync();
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
 ****************************************************************************/

void up_invalidate_dcache(uintptr_t start, uintptr_t end)
{
  size_t line_size = up_get_dcache_linesize();

  if ((start & (line_size - 1)) != 0)
    {
      start &= ~(line_size - 1);
      tc_flush_cache_byaddr(start);
      start += line_size;
    }

  while (start < end)
    {
      if (start + line_size <= end)
        {
          tc_invalidate_cache_byaddr(start);
        }
      else
        {
          tc_flush_cache_byaddr(start);
        }

      start += line_size;
    }

  __dsync();
  __isync();
}

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

void up_invalidate_dcache_all(void)
{
  Ifx_CPU_DCON1 dcon1;

  dcon1.U       = 0;
  dcon1.B.DCINV = 1;
  __mtcr(CPU_DCON1, dcon1.U);

  __dsync();
  __isync();
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
 ****************************************************************************/

void up_clean_dcache(uintptr_t start, uintptr_t end)
{
  size_t line_size = up_get_dcache_linesize();

  start &= ~(line_size - 1);

  while (start < end)
    {
      tc_clear_cache_byaddr(start);
      start += line_size;
    }

  __dsync();
  __isync();
}

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

void up_clean_dcache_all(void)
{
  uintptr_t cache_addr = CONFIG_ARCH_DCACHE_ADDR;
  size_t line_size = up_get_dcache_linesize();
  size_t line_cnt = up_get_dcache_size() / line_size;
  size_t i;

  for (i = 0; i < line_cnt; i++)
    {
      tc_clear_cache_byline(cache_addr);
      cache_addr += line_size;
    }

  __dsync();
  __isync();
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
 ****************************************************************************/

void up_flush_dcache(uintptr_t start, uintptr_t end)
{
  size_t line_size = up_get_dcache_linesize();

  start &= ~(line_size - 1);

  while (start < end)
    {
      tc_flush_cache_byaddr(start);
      start += line_size;
    }

  __dsync();
  __isync();
}

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

void up_flush_dcache_all(void)
{
  uintptr_t cache_addr = CONFIG_ARCH_DCACHE_ADDR;
  size_t line_size = up_get_dcache_linesize();
  size_t line_cnt = up_get_dcache_size() / line_size;
  size_t i;

  for (i = 0; i < line_cnt; i++)
    {
      tc_flush_cache_byline(cache_addr);
      cache_addr += line_size;
    }

  __dsync();
  __isync();
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
  if (len > 0)
    {
      up_flush_dcache(addr, addr + len);
      up_invalidate_icache_all();
    }
}
#endif
