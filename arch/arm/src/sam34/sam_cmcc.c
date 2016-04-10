/****************************************************************************
 * arch/arm/src/sam34/sam_cmcc.c
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 * 3. Neither the name NuttX nor the names of its contributors may be
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

#include <sys/types.h>
#include <stdint.h>
#include <assert.h>

#include "up_arch.h"
#include "chip/sam_cmcc.h"
#include "sam_cmcc.h"

#ifdef CONFIG_SAM34_CMCC

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CMCC_MASK     (CMCC_CACHE_LINE_SIZE-1)

#if CMCC_CACHE_LINE_SIZE == 4
#  define CMCC_SHIFT   2
#elif CMCC_CACHE_LINE_SIZE == 8
#  define CMCC_SHIFT   3
#elif CMCC_CACHE_LINE_SIZE == 16
#  define CMCC_SHIFT   4
#elif CMCC_CACHE_LINE_SIZE == 32
#  define CMCC_SHIFT   5
#else
#  error Unknown cache line size
#endif

#define ALIGN_UP(a)   (((a)+CMCC_MASK) & ~CMCC_MASK)
#define ALIGN_DOWN(a) ((a) & ~CMCC_MASK)

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_cmcc_enable
 *
 * Description:
 *   Enable the Cortex-M Cache Controller
 *
 ****************************************************************************/

void sam_cmcc_enable(void)
{
  /* "On reset, the cache controller data entries are all invalidated and the
   *  cache is disabled. The cache is transparent to processor operations. The
   *  cache controller is activated with its configuration registers. The
   *  configuration interface is memory mapped in the private peripheral bus.
   *
   * "Use the following sequence to enable the cache controller.
   *
   * "1. Verify that the cache controller is disabled, reading the value of the
   *     CSTS (cache status) field of the CMCC_SR register.
   * "2. Enable the cache controller, writing 1 to the CEN (cache enable) field
   *    of the CMCC_CTRL register."
   */

  if ((getreg32(SAM_CMCC_SR) & CMCC_SR_CSTS) == 0)
    {
      putreg32(CMCC_CTRL_CEN, SAM_CMCC_CTRL);
    }
}

/****************************************************************************
 * Name: sam_cmcc_disable
 *
 * Description:
 *   Disable the Cortex-M Cache Controller
 *
 ****************************************************************************/

void sam_cmcc_disable(void)
{
  /* "1. Disable the cache controller, writing 0 to the CEN field of the
   *     CMCC_CTRL register.
   * "2. Check CSTS field of the CMCC_SR to verify that the cache is
   *     successfully disabled.
   */

  putreg32(0, SAM_CMCC_CTRL);
  while ((getreg32(SAM_CMCC_SR) & CMCC_SR_CSTS) != 0);
}

/****************************************************************************
 * Name: sam_cmcc_invalidate
 *
 * Description:
 *   Invalidate a range of addresses.  Note:  These addresses should be
 *   aligned with the beginning and end of cache lines.  Otherwise, values
 *   at the edges of the region will also be invalidated!
 *
 ****************************************************************************/

void sam_cmcc_invalidate(uintptr_t start, uintptr_t end)
{
  uintptr_t addr;
  uint32_t regval;
  ssize_t size;
  int index;
  int way;

  /* Get the aligned addresses and size (in bytes) for the memory region
   * to be invalidated.
   */

  start  = ALIGN_DOWN(start);
  end    = ALIGN_UP(end);
  size   = end - start + 1;

  /* If this is a large region (as big as the cache), then just invalidate
   * the entire cache the easy way.
   *
   *   CacheSize = CacheLineSize * NCacheLines * NWays
   *   CacheAddressRange = CacheLineSize * NCacheLines = CacheSize / NWays
   *
   * Example: CacheSize = 2048, CacheLineSize=16, NWays=4:
   *
   *   CacheAddressRange = 2048 / 4 = 512
   *   NCacheLines       = 32
   */

  if (size >= (CMCC_CACHE_SIZE / CMCC_NWAYS))
    {
      sam_cmcc_invalidateall();
      return;
    }

  /* "When an invalidate by line command is issued the cache controller resets
   *  the valid bit information of the decoded cache line. As the line is no
   *  longer valid the replacement counter points to that line.
   *
   * "Use the following sequence to invalidate one line of cache.
   *
   * "1. Disable the cache controller, writing 0 to the CEN field of the
   *     CMCC_CTRL register.
   * "2. Check CSTS field of the CMCC_SR to verify that the cache is
   *     successfully disabled.
   * "3. Perform an invalidate by line writing the bit set {index, way} in
   *     the CMCC_MAINT1 register.
   * "4. Enable the cache controller, writing 1 to the CEN field of the
   *     CMCC_CTRL register."
   */

  /* Disable the cache controller */

  sam_cmcc_disable();

  /* Invalidate the address region */

  for (addr = start, index  = (int)(start >> CMCC_SHIFT);
       addr <= end;
       addr += CMCC_CACHE_LINE_SIZE, index++)
    {
      regval = CMCC_MAINT1_INDEX(index);
      for (way = 0; way < CMCC_NWAYS; way++)
        {
          putreg32(regval | CMCC_MAINT1_WAY(way), SAM_CMCC_MAINT1);
        }
    }

  /* Re-enable the cache controller */

  sam_cmcc_enable();
}

/****************************************************************************
 * Name: sam_cmcc_invalidateall
 *
 * Description:
 *   Invalidate the entire cache
 *
 ****************************************************************************/

void sam_cmcc_invalidateall(void)
{
  /* "To invalidate all cache entries:
   *
   * " Write 1 to the INVALL field of the CMCC_MAINT0 register."
   */

  putreg32(CMCC_MAINT0_INVALL, SAM_CMCC_MAINT0);
}

#endif /* CONFIG_SAM34_CMCC */
