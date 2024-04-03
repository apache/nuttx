/****************************************************************************
 * arch/x86_64/src/intel64/intel64_cache.c
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

#include <stdint.h>

#include "x86_64_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Name: x86_64_wbindv
 ****************************************************************************/

static inline void x86_64_wbindv(void)
{
  asm volatile("wbinvd" : : : "memory");
}

/****************************************************************************
 * Name: x86_64_wbnoinvd
 ****************************************************************************/

static inline void x86_64_wbnoinvd(void)
{
  asm volatile("wbnoinvd" : : : "memory");
}

/****************************************************************************
 * Name: x86_64_invd
 ****************************************************************************/

static inline void x86_64_invd(void)
{
  asm volatile("invd" : : : "memory");
}

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: x86_64_cache_linesize
 ****************************************************************************/

static size_t x86_64_cache_linesize(void)
{
#if CONFIG_ARCH_INTEL64_CACHE_LINESIZE == 0
  unsigned long eax = 0;
  unsigned long ebx = 0;

  eax = 1;
  asm volatile("cpuid\n\t"
               : "=b" (ebx)
               : "a" (eax));

  return ((ebx >> 8) & 0xff) * 8;
#else
  return CONFIG_ARCH_INTEL64_CACHE_LINESIZE;
#endif
}

/****************************************************************************
 * Name: x86_64_cache_size
 ****************************************************************************/

static size_t x86_64_cache_size(int leaf)
{
  unsigned long eax;
  unsigned long ebx;
  unsigned long ecx;
  unsigned long edx;

  /* The leaf 0 is Data cache */

  eax = 4;
  ecx = leaf;
  asm volatile("cpuid"
               : "=a"(eax), "=b"(ebx), "=c"(ecx), "=d"(edx)
               : "a"(eax), "c"(ecx));

  /* (Ways + 1) * (Partitions + 1) * (Line_Size + 1) * (Sets + 1) */

  return ((((ebx >> 22) & 0x3ff) + 1) *
          (((ebx >> 12) & 0x3ff) + 1) *
          ((ebx & 0x7ff) + 1) *
          (ecx + 1));
}

/****************************************************************************
 * Name: x86_64_cache_enable
 ****************************************************************************/

static void x86_64_cache_enable(void)
{
  /* Clear "Not-write through" (NW) and "Cache disable" (CD) bits */

  asm volatile("\t mov %%cr0, %%rax\n"
               "\t mov $0x9fffffff, %%rbx\n"
               "\t and %%rbx, %%rax\n"
               "\t mov %%rax, %%cr0\n"
               ::: "memory", "rax", "rbx");
}

/****************************************************************************
 * Name: x86_64_cache_disable
 ****************************************************************************/

static void x86_64_cache_disable(void)
{
  /* Set "Not-write through" (NW) and "Cache disable" (CD) bits */

  asm volatile("\t mov %%cr0, %%rax\n"
               "\t mov $0x9fffffff, %%rbx \n"
               "\t and %%rbx, %%rax \n"
               "\t mov $0x60000000, %%rbx\n"
               "\t or %%rbx, %%rax\n"
               "\t mov %%rax, %%cr0\n"
               :::"memory", "rax", "rbx");

  /* And flush all caches */

  up_flush_dcache_all();

  /* Disable MTRR */

  write_msr(MSR_MTRR_DEF_TYPE, 0);

  /* And flush once again */

  up_flush_dcache_all();
}

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
  return x86_64_cache_linesize();
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
#if CONFIG_ARCH_INTEL64_ICACHE_SIZE == 0
  return x86_64_cache_size(1);
#else
  return CONFIG_ARCH_INTEL64_ICACHE_SIZE;
#endif
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
 ****************************************************************************/

#ifdef CONFIG_ARCH_ICACHE
void up_enable_icache(void)
{
  x86_64_cache_enable();
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
  x86_64_cache_disable();
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
  /* NOTE: x86 doesn't have separate instructions for I-cache */

  up_invalidate_dcache(start, end);
}
#endif /* CONFIG_ARCH_ICACHE */

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
  /* NOTE: x86 doesn't have separate instructions for I-cache */

  up_invalidate_dcache_all();
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
  return x86_64_cache_linesize();
}

/****************************************************************************
 * Name: up_get_dcache_size
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

size_t up_get_dcache_size(void)
{
#if CONFIG_ARCH_INTEL64_DCACHE_SIZE == 0
  return x86_64_cache_size(0);
#else
  return CONFIG_ARCH_INTEL64_DCACHE_SIZE;
#endif
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

#ifdef CONFIG_ARCH_DCACHE
void up_enable_dcache(void)
{
  x86_64_cache_enable();
}
#endif /* CONFIG_ARCH_DCACHE */

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
  x86_64_cache_disable();
}
#endif /* CONFIG_ARCH_DCACHE */

/****************************************************************************
 * Name: up_invalidae_dcache
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
  up_flush_dcache(start, end);
}
#endif /* CONFIG_ARCH_DCACHE */

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
  x86_64_invd();
}
#endif /* CONFIG_ARCH_DCACHE */

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
#ifdef CONFIG_ARCH_INTEL64_HAVE_CLWB
  size_t lsize = up_get_dcache_linesize();

  start &= ~(lsize - 1);

  asm volatile("mfence" : : : "memory");

  do
    {
      asm volatile("\tclwb %0;\n" : "+m" (start));

      /* Increment the address by the size of one cache line. */

      start += lsize;
    }
  while (start < end);

  asm volatile("mfence" : : : "memory");
#else
  x86_64_wbnoinvd();
#endif
}
#endif /* CONFIG_ARCH_DCACHE */

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
  x86_64_wbnoinvd();
}
#endif /* CONFIG_ARCH_DCACHE */

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
  size_t lsize = up_get_dcache_linesize();

  start &= ~(lsize - 1);

  asm volatile("mfence" : : : "memory");

  do
    {
      asm volatile("\tclflush %0;\n" : "+m" (start));

      /* Increment the address by the size of one cache line. */

      start += lsize;
    }
  while (start < end);

  asm volatile("mfence" : : : "memory");
}
#endif /* CONFIG_ARCH_DCACHE */

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
  x86_64_wbindv();
}
#endif /* CONFIG_ARCH_DCACHE */

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

#if defined(CONFIG_ARCH_ICACHE) && defined(CONFIG_ARCH_DCACHE)
void up_coherent_dcache(uintptr_t addr, size_t len)
{
  uintptr_t end;

  if (len > 0)
    {
      /* Flush any dirtcy D-Cache lines to memory */

      end = addr + len;
      up_clean_dcache(addr, end);

      /* Invalidate I-Cache lines */

      up_invalidate_icache(addr, end);
    }
}
#endif
