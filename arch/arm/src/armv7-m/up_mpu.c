/****************************************************************************
 * arch/arm/src/armv7-m/up_mpu.c
 *
 *   Copyright (C) 2011, 2013 Gregory Nutt. All rights reserved.
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

#include <stdint.h>
#include <assert.h>

#include "mpu.h"
#include "up_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/

#ifndef CONFIG_ARMV7M_MPU_NREGIONS
#  define CONFIG_ARMV7M_MPU_NREGIONS 8
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* These sets represent the set of disabled memory sub-regions.  A bit set
 * corresponds to a disabled sub-region; the LS bit corresponds to the first
 * region.
 *
 * The g_ms_regionmask array is indexed by the number of subregions at the
 * end of the region:  0 means no sub-regions are available(0xff) and 8 means
 * all subregions are available (0x00).
 */

static const uint8_t g_ms_regionmask[9] =
{
  0xff, 0xfe, 0xfc, 0xf8, 0xf0, 0xe0, 0xc0, 0x80, 0x00
};

/* The g_ls_regionmask array is indexed by the number of subregions at the
 * beginning of the region:  0 means no sub-regions need be disabled (0x00)
 * and 8 means all subregions must be disabled (0xff).
 */

static const uint8_t g_ls_regionmask[9] =
{
  0x00, 0x01, 0x03, 0x07, 0x0f, 0x1f, 0x3f, 0x7f, 0xff
};

/* The next available region number */

static uint8_t g_region;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mpu_subregion_ms
 *
 * Description:
 *   Given (1) the size of the memory to be mapped and (2) the log2 size
 *   of the mapping to use, determine the minimal sub-region set at the
 *   to be disabled at the higher end of the region.
 *
 * Assumption:
 *   l2size has the same properties as the return value from
 *   mpu_log2regionceil()
 *
 ****************************************************************************/

static inline uint32_t mpu_subregion_ms(size_t size, uint8_t l2size)
{
  unsigned int nsrs;
  uint32_t     asize;
  uint32_t     mask;

  /* Examples with l2size = 12:
   *
   *         Shifted Adjusted        Number      Sub-Region
   * Size    Mask    Size      Shift Sub-Regions Bitset
   * 0x1000  0x01ff  0x1000    9     8           0x00
   * 0x0c00  0x01ff  0x0c00    9     6           0xc0
   * 0x0c40  0x01ff  0x0e00    9     7           0x80
   */

  if (l2size < 32)
    {
      mask  = ((1 << l2size)-1) >> 3; /* Shifted mask */
    }

  /* The 4Gb region size is a special case */

  else
    {
      /* NOTE: There is no way to represent a 4Gb region size in the 32-bit
       * input.
       */

      mask = 0x1fffffff;         /* Shifted mask */
    }

  asize = (size + mask) & ~mask; /* Adjusted size */
  nsrs  = asize >> (l2size-3);   /* Number of subregions */
  return g_ms_regionmask[nsrs];
}

/****************************************************************************
 * Name: mpu_subregion_ls
 *
 * Description:
 *   Given (1) the offset to the beginning of data in the region and (2) the
 *   log2 size of the mapping to use, determine the minimal sub-region set
 *   to span that memory region sub-region set at the to be disabled at the
 *   higher end of the region
 *
 * Assumption:
 *   l2size has the same properties as the return value from
 *   mpu_log2regionceil()
 *
 ****************************************************************************/

static inline uint32_t mpu_subregion_ls(size_t offset, uint8_t l2size)
{
  unsigned int nsrs;
  uint32_t     aoffset;
  uint32_t     mask;

  /* Examples with l2size = 12:
   *
   *         Shifted Adjusted        Number      Sub-Region
   * Offset  Mask    Offset    Shift Sub-Regions Bitset
   * 0x0000  0x01ff  0x0000    9     8           0x00
   * 0x0400  0x01ff  0x0400    9     6           0x03
   * 0x02c0  0x01ff  0x0200    9     7           0x01
   */

  if (l2size < 32)
    {
      mask  = ((1 << l2size)-1) >> 3; /* Shifted mask */
    }

  /* The 4Gb region size is a special case */

  else
    {
      /* NOTE: There is no way to represent a 4Gb region size in the 32-bit
       * input.
       */

      mask = 0x1fffffff;           /* Shifted mask */
    }

  aoffset = offset & ~mask;        /* Adjusted offset */
  nsrs    = aoffset >> (l2size-3); /* Number of subregions */
  return g_ls_regionmask[nsrs];
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mpu_allocregion
 *
 * Description:
 *   Allocate the next region
 *
 * Assumptions:
 *   - Regions are never deallocated
 *   - Regions are only allocated early in initialization, so no special
 *     protection against re-entrancy is required;
 *
 ****************************************************************************/

unsigned int mpu_allocregion(void)
{
  DEBUGASSERT(g_region < CONFIG_ARMV7M_MPU_NREGIONS);
  return (unsigned int)g_region++;
}

/****************************************************************************
 * Name: mpu_log2regionceil
 *
 * Description:
 *   Determine the smallest value of l2size (log base 2 size) such that the
 *   following is true:
 *
 *   size <= (1 << l2size)
 *
 ****************************************************************************/

uint8_t mpu_log2regionceil(size_t size)
{
  uint8_t l2size;

  /* The minimum permitted region size is 32 bytes (log2(32) = 5. */

  for (l2size = 5; l2size < 32 && size > (1 << l2size); l2size++);
  return l2size;
}

/****************************************************************************
 * Name: mpu_log2regionfloor
 *
 * Description:
 *   Determine the largest value of l2size (log base 2 size) such that the
 *   following is true:
 *
 *   size >= (1 << l2size)
 *
 ****************************************************************************/

uint8_t mpu_log2regionfloor(size_t size)
{
  uint8_t l2size = mpu_log2regionceil(size);

  if (l2size > 4 && size < (1 << l2size))
    {
      l2size--;
    }

  return l2size;
}

/****************************************************************************
 * Name: mpu_subregion
 *
 * Description:
 *   Given the size of the (1) memory to be mapped and (2) the log2 size
 *   of the mapping to use, determine the minimal sub-region set to span
 *   that memory region.
 *
 * Assumption:
 *   l2size has the same properties as the return value from
 *   mpu_log2regionceil()
 *
 ****************************************************************************/

uint32_t mpu_subregion(uintptr_t base, size_t size, uint8_t l2size)
{
  uint32_t mask;
  size_t offset;
  uint32_t ret;

  /* Eight subregions are supported.  The representation is as an 8-bit
   * value with the LS bit corresponding to subregion 0.  A bit is set
   * to disable the sub-region.
   *
   * l2size: Log2 of the actual region size is <= (1 << l2size);
   */

  DEBUGASSERT(l2size > 4 && size <= (1 << l2size));

  /* For region sizes of 32, 64, and 128 bytes, the effect of setting
   * one or more bits of the SRD field to 1 is UNPREDICTABLE.
   */

  if (l2size < 8)
    {
      return 0;
    }

  /* Calculate the offset of the base address into the aligned region. */

  mask   = (1 << l2size) - 1;
  offset = base & mask;

  /* Calculate the mask need to handle disabled subregions at the end of the
   * region
   */

  ret = mpu_subregion_ms(size + offset, l2size);

  /* Then OR in the mask need to handle disabled subregions at the beginning
   * of the region.
   */

  ret |= mpu_subregion_ls(offset, l2size);
  return ret;
}
