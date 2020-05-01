/*****************************************************************************
 * arch/arm/src/armv7-m/arm_mpu.c
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
 *****************************************************************************/

/*****************************************************************************
 * Included Files
 *****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <assert.h>

#include "mpu.h"
#include "arm_internal.h"

/*****************************************************************************
 * Pre-processor Definitions
 *****************************************************************************/

/* Configuration *************************************************************/

#ifndef CONFIG_ARM_MPU_NREGIONS
#  define CONFIG_ARM_MPU_NREGIONS 8
#endif

/*****************************************************************************
 * Private Data
 *****************************************************************************/

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

/*****************************************************************************
 * Private Functions
 *****************************************************************************/

/*****************************************************************************
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
 *****************************************************************************/

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
      mask  = ((1 << l2size) - 1) >> 3; /* Shifted mask */
    }

  /* The 4Gb region size is a special case */

  else
    {
      /* NOTE: There is no way to represent a 4Gb region size in the 32-bit
       * input.
       */

      mask = 0x1fffffff;          /* Shifted mask */
    }

  asize = (size + mask) & ~mask;  /* Adjusted size */
  nsrs  = asize >> (l2size - 3);  /* Number of subregions */
  return g_ms_regionmask[nsrs];
}

/*****************************************************************************
 * Name: mpu_subregion_ls
 *
 * Description:
 *   Given (1) the offset to the beginning of data in the region and (2) the
 *   log2 size of the mapping to use, determine the minimal sub-region set
 *   to span that memory region sub-region set at the to be disabled at the
 *   lower end of the region
 *
 * Assumption:
 *   l2size has the same properties as the return value from
 *   mpu_log2regionceil()
 *
 *****************************************************************************/

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

      mask = 0x1fffffff;              /* Shifted mask */
    }

  aoffset = offset & ~mask;           /* Adjusted offset */
  nsrs    = aoffset >> (l2size - 3);  /* Number of subregions */
  return g_ls_regionmask[nsrs];
}

/*****************************************************************************
 * Public Functions
 *****************************************************************************/

/*****************************************************************************
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
 *****************************************************************************/

unsigned int mpu_allocregion(void)
{
  DEBUGASSERT(g_region < CONFIG_ARM_MPU_NREGIONS);
  return (unsigned int)g_region++;
}

/*****************************************************************************
 * Name: mpu_log2regionceil
 *
 * Description:
 *   Determine the smallest value of l2size (log base 2 size) such that the
 *   following is true:
 *
 *   size <= (1 << l2size)
 *
 *****************************************************************************/

uint8_t mpu_log2regionceil(size_t size)
{
  uint8_t l2size;

  /* The minimum permitted region size is 32 bytes (log2(32) = 5. */

  for (l2size = 5; l2size < 32 && size > (1 << l2size); l2size++);
  return l2size;
}

/*****************************************************************************
 * Name: mpu_log2regionfloor
 *
 * Description:
 *   Determine the largest value of l2size (log base 2 size) such that the
 *   following is true:
 *
 *   size >= (1 << l2size)
 *
 *****************************************************************************/

uint8_t mpu_log2regionfloor(size_t size)
{
  uint8_t l2size = mpu_log2regionceil(size);

  if (l2size > 4 && size < (1 << l2size))
    {
      l2size--;
    }

  return l2size;
}

/*****************************************************************************
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
 *****************************************************************************/

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

/*****************************************************************************
 * Name: mpu_control
 *
 * Description:
 *   Configure and enable (or disable) the MPU
 *
 *****************************************************************************/

void mpu_control(bool enable, bool hfnmiena, bool privdefena)
{
  uint32_t regval = 0;

  if (enable)
    {
      regval |= MPU_CTRL_ENABLE; /* Enable the MPU */

      if (hfnmiena)
        {
           regval |= MPU_CTRL_HFNMIENA; /* Enable MPU during hard fault, NMI, and FAULTMAS */
        }

      if (privdefena)
        {
          regval |= MPU_CTRL_PRIVDEFENA; /* Enable privileged access to default memory map */
        }
    }

  putreg32(regval, MPU_CTRL);
}

/*****************************************************************************
 * Name: mpu_configure_region
 *
 * Description:
 *   Configure a region for privileged, strongly ordered memory
 *
 *****************************************************************************/

void mpu_configure_region(uintptr_t base, size_t size,
                                        uint32_t flags)
{
  unsigned int region = mpu_allocregion();
  uint32_t     regval;
  uint8_t      l2size;
  uint8_t      subregions;
  uintptr_t    alignedbase;

  /* Ensure the base address alignment
   *
   * ARMv7-M Architecture Reference Manual
   * B3.5.8 MPU Region Base Address Register, MPU_RBAR
   * "Software must ensure that the value written to the ADDR field
   * aligns with the size of the selected region."
   */

  alignedbase  = base & MPU_RBAR_ADDR_MASK;
  l2size       = mpu_log2regionceil(size + base - alignedbase);
  alignedbase &= ~((1 << l2size) - 1);
  l2size       = mpu_log2regionceil(size + base - alignedbase);

  DEBUGASSERT(alignedbase + (1 << l2size) >= base + size);
  DEBUGASSERT(l2size == 5 || alignedbase + (1 << (l2size - 1)) < base + size);
  DEBUGASSERT((alignedbase & MPU_RBAR_ADDR_MASK) == alignedbase);
  DEBUGASSERT((alignedbase & ((1 << l2size) - 1)) == 0);

  /* Select the region */

  putreg32(region, MPU_RNR);

  /* Select the region base address */

  putreg32(alignedbase | region | MPU_RBAR_VALID, MPU_RBAR);

  /* Select the region size and the sub-region map */

  subregions = mpu_subregion(base, size, l2size);

  /* The configure the region */

  regval = MPU_RASR_ENABLE                              | /* Enable region  */
           MPU_RASR_SIZE_LOG2((uint32_t)l2size)         | /* Region size    */
           ((uint32_t)subregions << MPU_RASR_SRD_SHIFT) | /* Sub-regions    */
           flags;
  putreg32(regval, MPU_RASR);
}
