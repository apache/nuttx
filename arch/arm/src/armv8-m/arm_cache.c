/****************************************************************************
 * arch/arm/src/armv8-m/arm_cache.c
 *
 *   Copyright (C) 2015, 2018-2019 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *           Bob Feretich <bob.feretich@rafresearch.com>
 *
 * Some logic in this header file derives from the ARM CMSIS core_cm7.h
 * header file which has a compatible 3-clause BSD license:
 *
 *   Copyright (c) 2009 - 2014 ARM LIMITED.  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name ARM, NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/cache.h>

#include "arm_internal.h"
#include "barriers.h"
#include "nvic.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Cache Size ID (CCSIDR) register macros used by inline functions
 * Given the value of the CCSIDR reginer (n):
 *
 *   CCSIDR_WAYS    - Returns the (number of ways) - 1
 *   CCSIDR_SETS    - Returns the (number of sets) - 1
 *   CCSIDR_LSSHIFT - Returns log2(cache line size in words) - 2
 *                    Eg. 0 -> 4 words
 *                        1 -> 8 words
 *                        ...
 */

#define CCSIDR_WAYS(n) \
  (((n) & NVIC_CCSIDR_ASSOCIATIVITY_MASK) >> NVIC_CCSIDR_ASSOCIATIVITY_SHIFT)
#define CCSIDR_SETS(n) \
  (((n) & NVIC_CCSIDR_NUMSETS_MASK) >> NVIC_CCSIDR_NUMSETS_SHIFT)
#define CCSIDR_LSSHIFT(n) \
  (((n) & NVIC_CCSIDR_LINESIZE_MASK) >> NVIC_CCSIDR_LINESIZE_SHIFT)

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_clz
 *
 * Description:
 *   Access to CLZ instructions
 *
 * Input Parameters:
 *   value - The value to perform the CLZ operation on
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline uint32_t arm_clz(unsigned int value)
{
  uint32_t ret;

  __asm__ __volatile__ ("clz %0, %1" : "=r"(ret) : "r"(value));
  return ret;
}

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

#ifdef CONFIG_ARMV8M_ICACHE
void up_enable_icache(void)
{
  uint32_t regval;

  ARM_DSB();
  ARM_ISB();

  /* Invalidate the entire I-Cache */

  putreg32(0, NVIC_ICIALLU);

  /* Enable the I-Cache */

  regval  = getreg32(NVIC_CFGCON);
  regval |= NVIC_CFGCON_IC;
  putreg32(regval, NVIC_CFGCON);

  ARM_DSB();
  ARM_ISB();
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

#ifdef CONFIG_ARMV8M_ICACHE
void up_disable_icache(void)
{
  uint32_t regval;

  ARM_DSB();
  ARM_ISB();

  /* Disable the I-Cache */

  regval  = getreg32(NVIC_CFGCON);
  regval &= ~NVIC_CFGCON_IC;
  putreg32(regval, NVIC_CFGCON);

  /* Invalidate the entire I-Cache */

  putreg32(0, NVIC_ICIALLU);

  ARM_DSB();
  ARM_ISB();
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

#ifdef CONFIG_ARMV8M_ICACHE
void up_invalidate_icache(uintptr_t start, uintptr_t end)
{
  uint32_t ccsidr;
  uint32_t sshift;
  uint32_t ssize;

  /* Get the characteristics of the I-Cache */

  ccsidr = getreg32(NVIC_CCSIDR);
  sshift = CCSIDR_LSSHIFT(ccsidr) + 4;   /* log2(cache-line-size-in-bytes) */

  /* Invalidate the I-Cache containing this range of addresses */

  ssize  = (1 << sshift);

  /* Round down the start address to the nearest cache line boundary.
   *
   *   sshift = 5      : Offset to the beginning of the set field
   *   (ssize - 1)  = 0x007f : Mask of the set field
   */

  ARM_DSB();

  if ((start & (ssize - 1)) != 0)
    {
      start &= ~(ssize - 1);
      putreg32(start, NVIC_ICIMVAU);
      start += ssize;
    }

  while (start + ssize <= end)
    {
      /* The below store causes the cache to check its directory and
       * determine if this address is contained in the cache. If so, it
       * invalidate that cache line. Only the cache way containing the
       * address is invalidated. If the address is not in the cache, then
       * nothing is invalidated.
       */

      putreg32(start, NVIC_ICIMVAU);

      /* Increment the address by the size of one cache line. */

      start += ssize;
    }

  if (start < end)
    {
      putreg32(start, NVIC_ICIMVAU);
    }

  ARM_DSB();
  ARM_ISB();
}
#endif /* CONFIG_ARMV8M_ICACHE */

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

#ifdef CONFIG_ARMV8M_ICACHE
void up_invalidate_icache_all(void)
{
  ARM_DSB();
  ARM_ISB();

  /* Invalidate the entire I-Cache */

  putreg32(0, NVIC_ICIALLU);

  ARM_DSB();
  ARM_ISB();
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

#ifdef CONFIG_ARMV8M_DCACHE
void up_enable_dcache(void)
{
  uint32_t ccsidr;
  uint32_t ccr;
  uint32_t sshift;
  uint32_t wshift;
  uint32_t sw;
  uint32_t sets;
  uint32_t ways;

  /* Get the characteristics of the D-Cache */

  ccsidr = getreg32(NVIC_CCSIDR);
  sets   = CCSIDR_SETS(ccsidr);          /* (Number of sets) - 1 */
  sshift = CCSIDR_LSSHIFT(ccsidr) + 4;   /* log2(cache-line-size-in-bytes) */
  ways   = CCSIDR_WAYS(ccsidr);          /* (Number of ways) - 1 */

  /* Calculate the bit offset for the way field in the DCISW register by
   * counting the number of leading zeroes.  For example:
   *
   *   Number of  Value of ways  Field
   *   Ways       'ways'         Offset
   *     2         1             31
   *     4         3             30
   *     8         7             29
   *   ...
   */

  wshift = arm_clz(ways) & 0x1f;

  /* Invalidate the entire D-Cache */

  ARM_DSB();
  do
    {
      int32_t tmpways = ways;

      do
        {
          sw = ((tmpways << wshift) | (sets << sshift));
          putreg32(sw, NVIC_DCISW);
        }
      while (tmpways--);
    }
  while (sets--);

  ARM_DSB();

#ifdef CONFIG_ARMV8M_DCACHE_WRITETHROUGH
  ccr = getreg32(NVIC_CACR);
  ccr |= NVIC_CACR_FORCEWT;
  putreg32(ccr, NVIC_CACR);
#endif

  /* Enable the D-Cache */

  ccr  = getreg32(NVIC_CFGCON);
  ccr |= NVIC_CFGCON_DC;
  putreg32(ccr, NVIC_CFGCON);

  ARM_DSB();
  ARM_ISB();
}
#endif /* CONFIG_ARMV8M_DCACHE */

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

#ifdef CONFIG_ARMV8M_DCACHE
void up_disable_dcache(void)
{
  uint32_t ccsidr;
  uint32_t ccr;
  uint32_t sshift;
  uint32_t wshift;
  uint32_t sw;
  uint32_t sets;
  uint32_t ways;

  /* Get the characteristics of the D-Cache */

  ccsidr = getreg32(NVIC_CCSIDR);
  sets   = CCSIDR_SETS(ccsidr);          /* (Number of sets) - 1 */
  sshift = CCSIDR_LSSHIFT(ccsidr) + 4;   /* log2(cache-line-size-in-bytes) */
  ways   = CCSIDR_WAYS(ccsidr);          /* (Number of ways) - 1 */

  /* Calculate the bit offset for the way field in the DCCISW register by
   * counting the number of leading zeroes.  For example:
   *
   *   Number of  Value of ways  Field
   *   Ways       'ways'         Offset
   *     2         1             31
   *     4         3             30
   *     8         7             29
   *   ...
   */

  wshift = arm_clz(ways) & 0x1f;

  ARM_DSB();

  /* Disable the D-Cache */

  ccr = getreg32(NVIC_CFGCON);
  ccr &= ~NVIC_CFGCON_DC;
  putreg32(ccr, NVIC_CFGCON);

  /* Clean and invalidate the entire D-Cache */

  do
    {
      int32_t tmpways = ways;

      do
        {
          sw = ((tmpways << wshift) | (sets << sshift));
          putreg32(sw, NVIC_DCCISW);
        }
      while (tmpways--);
    }
  while (sets--);

  ARM_DSB();
  ARM_ISB();
}
#endif /* CONFIG_ARMV8M_DCACHE */

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

#ifdef CONFIG_ARMV8M_DCACHE
void up_invalidate_dcache(uintptr_t start, uintptr_t end)
{
  uint32_t ccsidr;
  uint32_t sshift;
  uint32_t ssize;

  /* Get the characteristics of the D-Cache */

  ccsidr = getreg32(NVIC_CCSIDR);
  sshift = CCSIDR_LSSHIFT(ccsidr) + 4;   /* log2(cache-line-size-in-bytes) */

  /* Invalidate the D-Cache containing this range of addresses */

  ssize  = (1 << sshift);

  /* Round down the start address to the nearest cache line boundary.
   *
   *   sshift = 5      : Offset to the beginning of the set field
   *   (ssize - 1)  = 0x007f : Mask of the set field
   */

  ARM_DSB();

  if ((start & (ssize - 1)) != 0)
    {
      start &= ~(ssize - 1);
      putreg32(start, NVIC_DCCIMVAC);
      start += ssize;
    }

  while (start + ssize <= end)
    {
      /* The below store causes the cache to check its directory and
       * determine if this address is contained in the cache. If so, it
       * invalidate that cache line. Only the cache way containing the
       * address is invalidated. If the address is not in the cache, then
       * nothing is invalidated.
       */

      putreg32(start, NVIC_DCIMVAC);

      /* Increment the address by the size of one cache line. */

      start += ssize;
    }

  if (start < end)
    {
      putreg32(start, NVIC_DCCIMVAC);
    }

  ARM_DSB();
  ARM_ISB();
}
#endif /* CONFIG_ARMV8M_DCACHE */

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

#ifdef CONFIG_ARMV8M_DCACHE
void up_invalidate_dcache_all(void)
{
  uint32_t ccsidr;
  uint32_t sshift;
  uint32_t wshift;
  uint32_t sw;
  uint32_t sets;
  uint32_t ways;

  /* Get the characteristics of the D-Cache */

  ccsidr = getreg32(NVIC_CCSIDR);
  sets   = CCSIDR_SETS(ccsidr);          /* (Number of sets) - 1 */
  sshift = CCSIDR_LSSHIFT(ccsidr) + 4;   /* log2(cache-line-size-in-bytes) */
  ways   = CCSIDR_WAYS(ccsidr);          /* (Number of ways) - 1 */

  /* Calculate the bit offset for the way field in the DCISW register by
   * counting the number of leading zeroes.  For example:
   *
   *   Number of  Value of ways  Field
   *   Ways       'ways'         Offset
   *     2         1             31
   *     4         3             30
   *     8         7             29
   *   ...
   */

  wshift = arm_clz(ways) & 0x1f;

  ARM_DSB();

  /* Invalidate the entire D-Cache */

  do
    {
      int32_t tmpways = ways;

      do
        {
          sw = ((tmpways << wshift) | (sets << sshift));
          putreg32(sw, NVIC_DCISW);
        }
      while (tmpways--);
    }
  while (sets--);

  ARM_DSB();
  ARM_ISB();
}
#endif /* CONFIG_ARMV8M_DCACHE */

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

#ifdef CONFIG_ARMV8M_DCACHE
void up_clean_dcache(uintptr_t start, uintptr_t end)
{
#ifndef CONFIG_ARMV8M_DCACHE_WRITETHROUGH
  uint32_t ccsidr;
  uint32_t sshift;
  uint32_t ssize;
  uint32_t sets;
  uint32_t ways;

  /* Get the characteristics of the D-Cache */

  ccsidr = getreg32(NVIC_CCSIDR);
  sshift = CCSIDR_LSSHIFT(ccsidr) + 4;   /* log2(cache-line-size-in-bytes) */
  sets   = CCSIDR_SETS(ccsidr);          /* (Number of sets) - 1 */
  ways   = CCSIDR_WAYS(ccsidr);          /* (Number of ways) - 1 */

  /* Clean the D-Cache over the range of addresses */

  ssize  = (1 << sshift);

  if ((end - start) >= ssize * (sets + 1) * (ways + 1))
    {
      return up_clean_dcache_all();
    }

  start &= ~(ssize - 1);
  ARM_DSB();

  do
    {
      /* The below store causes the cache to check its directory and
       * determine if this address is contained in the cache. If so, it
       * clean that cache line. Only the cache way containing the
       * address is invalidated. If the address is not in the cache, then
       * nothing is invalidated.
       */

      putreg32(start, NVIC_DCCMVAC);

      /* Increment the address by the size of one cache line. */

      start += ssize;
    }
  while (start < end);

  ARM_DSB();
  ARM_ISB();
#endif /* !CONFIG_ARMV8M_DCACHE_WRITETHROUGH */
}
#endif /* CONFIG_ARMV8M_DCACHE */

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

#ifdef CONFIG_ARMV8M_DCACHE
void up_clean_dcache_all(void)
{
#ifndef CONFIG_ARMV8M_DCACHE_WRITETHROUGH
  uint32_t ccsidr;
  uint32_t sshift;
  uint32_t wshift;
  uint32_t sw;
  uint32_t sets;
  uint32_t ways;

  /* Get the characteristics of the D-Cache */

  ccsidr = getreg32(NVIC_CCSIDR);
  sets   = CCSIDR_SETS(ccsidr);          /* (Number of sets) - 1 */
  sshift = CCSIDR_LSSHIFT(ccsidr) + 4;   /* log2(cache-line-size-in-bytes) */
  ways   = CCSIDR_WAYS(ccsidr);          /* (Number of ways) - 1 */

  /* Calculate the bit offset for the way field in the DCCSW register by
   * counting the number of leading zeroes.  For example:
   *
   *   Number of  Value of ways  Field
   *   Ways       'ways'         Offset
   *     2         1             31
   *     4         3             30
   *     8         7             29
   *   ...
   */

  wshift = arm_clz(ways) & 0x1f;

  ARM_DSB();

  /* Clean the entire D-Cache */

  do
    {
      int32_t tmpways = ways;

      do
        {
          sw = ((tmpways << wshift) | (sets << sshift));
          putreg32(sw, NVIC_DCCSW);
        }
      while (tmpways--);
    }
  while (sets--);

  ARM_DSB();
  ARM_ISB();
#endif /* !CONFIG_ARMV8M_DCACHE_WRITETHROUGH */
}
#endif /* CONFIG_ARMV8M_DCACHE */

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

#ifdef CONFIG_ARMV8M_DCACHE
void up_flush_dcache(uintptr_t start, uintptr_t end)
{
#ifndef CONFIG_ARMV8M_DCACHE_WRITETHROUGH
  uint32_t ccsidr;
  uint32_t sshift;
  uint32_t ssize;
  uint32_t sets;
  uint32_t ways;

  /* Get the characteristics of the D-Cache */

  ccsidr = getreg32(NVIC_CCSIDR);
  sshift = CCSIDR_LSSHIFT(ccsidr) + 4;   /* log2(cache-line-size-in-bytes) */
  sets   = CCSIDR_SETS(ccsidr);          /* (Number of sets) - 1 */
  ways   = CCSIDR_WAYS(ccsidr);          /* (Number of ways) - 1 */

  /* Clean and invalidate the D-Cache over the range of addresses */

  ssize  = (1 << sshift);

  if ((end - start) >= ssize * (sets + 1) * (ways + 1))
    {
      return up_flush_dcache_all();
    }

  start &= ~(ssize - 1);
  ARM_DSB();

  do
    {
      /* The below store causes the cache to check its directory and
       * determine if this address is contained in the cache. If so, it clean
       * and invalidate that cache line. Only the cache way containing the
       * address is invalidated. If the address is not in the cache, then
       * nothing is invalidated.
       */

      putreg32(start, NVIC_DCCIMVAC);

      /* Increment the address by the size of one cache line. */

      start += ssize;
    }
  while (start < end);

  ARM_DSB();
  ARM_ISB();
#else
  up_invalidate_dcache(start, end);
#endif /* !CONFIG_ARMV8M_DCACHE_WRITETHROUGH */
}
#endif /* CONFIG_ARMV8M_DCACHE */

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

#ifdef CONFIG_ARMV8M_DCACHE
void up_flush_dcache_all(void)
{
#ifndef CONFIG_ARMV8M_DCACHE_WRITETHROUGH
  uint32_t ccsidr;
  uint32_t sshift;
  uint32_t wshift;
  uint32_t sw;
  uint32_t sets;
  uint32_t ways;

  /* Get the characteristics of the D-Cache */

  ccsidr = getreg32(NVIC_CCSIDR);
  sets   = CCSIDR_SETS(ccsidr);          /* (Number of sets) - 1 */
  sshift = CCSIDR_LSSHIFT(ccsidr) + 4;   /* log2(cache-line-size-in-bytes) */
  ways   = CCSIDR_WAYS(ccsidr);          /* (Number of ways) - 1 */

  /* Calculate the bit offset for the way field in the DCCISW register by
   * counting the number of leading zeroes.  For example:
   *
   *   Number of  Value of ways  Field
   *   Ways       'ways'         Offset
   *     2         1             31
   *     4         3             30
   *     8         7             29
   *   ...
   */

  wshift = arm_clz(ways) & 0x1f;

  ARM_DSB();

  /* Clean and invalidate the entire D-Cache */

  do
    {
      int32_t tmpways = ways;

      do
        {
          sw = ((tmpways << wshift) | (sets << sshift));
          putreg32(sw, NVIC_DCCISW);
        }
      while (tmpways--);
    }
  while (sets--);

  ARM_DSB();
  ARM_ISB();
#else
  up_invalidate_dcache_all();
#endif /* !CONFIG_ARMV8M_DCACHE_WRITETHROUGH */
}
#endif /* CONFIG_ARMV8M_DCACHE */

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

#ifdef CONFIG_ARMV8M_ICACHE
void up_coherent_dcache(uintptr_t addr, size_t len)
{
  uintptr_t end;

  if (len > 0)
    {
      /* Flush any dirtcy D-Cache lines to memory */

      end = addr + len;
      up_clean_dcache(addr, end);
      UNUSED(end);

      /* Invalidate the entire I-Cache */

      up_invalidate_icache_all();
    }
}
#endif
