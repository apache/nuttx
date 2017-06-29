/****************************************************************************
 * arch/arm/src/armv7-m/arch_clean_dcache.c
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#include "cache.h"

#if defined(CONFIG_ARMV7M_DCACHE) && !defined(CONFIG_ARMV7M_DCACHE_WRITETHROUGH)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arch_clean_dcache
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
 *   the operation is pre-empted.
 *
 ****************************************************************************/

void arch_clean_dcache(uintptr_t start, uintptr_t end)
{
  uint32_t ccsidr;
  uint32_t smask;
  uint32_t sshift;
  uint32_t ways;
  uint32_t wshift;
  uint32_t ssize;
  uint32_t set;
  uint32_t sw;

  /* Get the characteristics of the D-Cache */

  ccsidr = getreg32(NVIC_CCSIDR);
  smask  = CCSIDR_SETS(ccsidr);          /* (Number of sets) - 1 */
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

  /* Clean the D-Cache over the range of addresses */

  ssize  = (1 << sshift);
  start &= ~(ssize - 1);
  ARM_DSB();

  do
    {
      int32_t tmpways = ways;

      /* Isolate the cache line associated with this address.  For example
       * if the cache line size is 32 bytes and the cache size is 16KB, then
       *
       *   sshift = 5      : Offset to the beginning of the set field
       *   smask  = 0x007f : Mask of the set field
       */

      set = ((uint32_t)start >> sshift) & smask;

      /* Clean and invalidate each way for this cacheline */

      do
        {
          sw = ((tmpways << wshift) | (set << sshift));
          putreg32(sw, NVIC_DCCSW);
        }
      while (tmpways--);

      /* Increment the address by the size of one cache line. */

      start += ssize;
    }
  while (start < end);

  ARM_DSB();
  ARM_ISB();
}

#endif  /* CONFIG_ARMV7M_DCACHE && !CONFIG_ARMV7M_DCACHE_WRITETHROUGH */
