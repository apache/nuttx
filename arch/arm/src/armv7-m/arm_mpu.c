/****************************************************************************
 * arch/arm/src/armv7-m/arm_mpu.c
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

#include <stdint.h>
#include <assert.h>
#include <debug.h>

#include <arch/barriers.h>

#include "mpu.h"
#include "arm_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_ARM_MPU_NREGIONS
#  define CONFIG_ARM_MPU_NREGIONS 8
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

/* The available region bitmap */

static unsigned int g_mpu_region;

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
 * Input Parameters:
 *   size: The size of the region.
 *   l2size: The L2 size of the region.
 *
 * Returned Value:
 *   The sub-region bitmask.
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

/****************************************************************************
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
 * Input Parameters:
 *   offset: The offset of the region.
 *   l2size: The L2 size of the region.
 *
 * Returned Value:
 *   The sub-region bitmask.
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

      mask = 0x1fffffff;              /* Shifted mask */
    }

  aoffset = offset & ~mask;           /* Adjusted offset */
  nsrs    = aoffset >> (l2size - 3);  /* Number of subregions */
  return g_ls_regionmask[nsrs];
}

/****************************************************************************
 * Name: mpu_reset_internal
 *
 * Description:
 *   Resets the MPU to disabled.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#if defined(CONFIG_ARM_MPU_RESET) || defined(CONFIG_ARM_MPU_EARLY_RESET)
static void mpu_reset_internal(void)
{
  int region;
  int regions;
  regions = (getreg32(MPU_TYPE) & MPU_TYPE_DREGION_MASK)
                                  >> MPU_TYPE_DREGION_SHIFT;

  for (region = 0; region < regions; region++)
    {
      putreg32(region, MPU_RNR);
      putreg32(0, MPU_RASR);
      putreg32(0, MPU_RBAR);
    }

  putreg32(0, MPU_CTRL);

  UP_MB();
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mpu_allocregion
 *
 * Description:
 *   Allocate the next region
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   The index of the allocated region.
 *
 ****************************************************************************/

unsigned int mpu_allocregion(void)
{
  unsigned int i = ffs(~g_mpu_region) - 1;

  /* There are not enough regions to apply */

  DEBUGASSERT(i < CONFIG_ARM_MPU_NREGIONS);
  g_mpu_region |= 1 << i;
  return i;
}

/****************************************************************************
 * Name: mpu_freeregion
 *
 * Description:
 *   Free target region.
 *
 * Input Parameters:
 *  region - The index of the region to be freed.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void mpu_freeregion(unsigned int region)
{
  DEBUGASSERT(region < CONFIG_ARM_MPU_NREGIONS);

  /* Clear and disable the given MPU Region */

  putreg32(region, MPU_RNR);
  putreg32(0, MPU_RASR);
  putreg32(0, MPU_RBAR);
  g_mpu_region &= ~(1 << region);
  UP_MB();
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
 * Input Parameters:
 *   size - The size of the region.
 *
 * Returned Value:
 *   The logarithm base 2 of the ceiling value for the MPU region size.
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
 * Input Parameters:
 *   size - The size of the region.
 *
 * Returned Value:
 *   The logarithm base 2 of the floor value for the MPU region size.
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
 * Input Parameters:
 *   base   - The base address of the region.
 *   size   - The size of the region.
 *   l2size - Log2 of the actual region size is <= (1 << l2size).
 *
 * Returned Value:
 *   The subregion settings as a 32-bit value.
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

/****************************************************************************
 * Name: mpu_control
 *
 * Description:
 *   Configure and enable (or disable) the MPU
 *
 * Input Parameters:
 *   enable     - Flag indicating whether to enable the MPU.
 *   hfnmiena   - Flag indicating whether to enable the MPU during hard
 *                fult, NMI, and FAULTMAS.
 *   privdefena - Flag indicating whether to enable privileged access to
 *                the default memory map.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

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

  /* Ensure MPU setting take effects */

  UP_MB();
}

/****************************************************************************
 * Name: mpu_modify_region
 *
 * Description:
 *   Modify a region for privileged, strongly ordered memory
 *
 * Input Parameters:
 *   region - Region number to modify.
 *   base   - Base address of the region.
 *   size   - Size of the region.
 *   flags  - Flags to configure the region.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void mpu_modify_region(unsigned int region, uintptr_t base, size_t size,
                       uint32_t flags)
{
  uint32_t     regval;
  uint8_t      l2size;
  uint8_t      subregions;
  uintptr_t    alignedbase;

  /* Check that the region is valid */

  DEBUGASSERT(g_mpu_region & (1 << region));

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
  DEBUGASSERT(l2size == 5 ||
              alignedbase + (1 << (l2size - 1)) < base + size);
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

  /* Ensure MPU setting take effects */

  UP_MB();
}

/****************************************************************************
 * Name: mpu_configure_region
 *
 * Description:
 *   Configure a region for privileged, strongly ordered memory
 *
 * Input Parameters:
 *   base  - Base address of the region.
 *   size  - Size of the region.
 *   flags - Flags to configure the region.
 *
 * Returned Value:
 *   The region number allocated for the configured region.
 *
 ****************************************************************************/

unsigned int mpu_configure_region(uintptr_t base, size_t size,
                                  uint32_t flags)
{
  unsigned int region = mpu_allocregion();
  mpu_modify_region(region, base, size, flags);
  return region;
}

/****************************************************************************
 * Name: mpu_initialize
 *
 * Description:
 *   Configure a region for privileged, strongly ordered memory
 *
 * Input Parameters:
 *   table      - MPU initialization table.
 *   count      - Initialize the number of entries in the region table.
 *   hfnmiena   - A boolean indicating whether the MPU should enable the
 *                HFNMIENA bit.
 *   privdefena - A boolean indicating whether the MPU should enable the
 *                PRIVDEFENA bit.
 *
 * Returned Value:
 *   NULL.
 *
 ****************************************************************************/

void mpu_initialize(const struct mpu_region_s *table, size_t count,
                    bool hfnmiena, bool privdefena)
{
  const struct mpu_region_s *conf;
  size_t index;

  mpu_control(false, false, false);
  for (index = 0; index < count; index++)
    {
      conf = &table[index];
      mpu_configure_region(conf->base, conf->size, conf->flags);
    }

  mpu_control(true, hfnmiena, privdefena);
}

/****************************************************************************
 * Name: mpu_dump_region
 *
 * Description:
 *   Dump the region that has been used.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void mpu_dump_region(void)
{
  int i;
  int count = 0;
  uint32_t rasr;
  uint32_t rbar;
  uint32_t ctrl;

  /* Get free region */

  ctrl = getreg32(MPU_CTRL);
  _info("MPU-CTRL Enable:%" PRIu32 ", HFNMIENA:%"PRIu32","
        "PRIVDEFENA:%" PRIu32 "\n", ctrl & MPU_CTRL_ENABLE,
        ctrl & MPU_CTRL_HFNMIENA, ctrl & MPU_CTRL_PRIVDEFENA);
  for (i = 0; i < CONFIG_ARM_MPU_NREGIONS; i++)
    {
      putreg32(i, MPU_RNR);
      rasr = getreg32(MPU_RASR);
      rbar = getreg32(MPU_RBAR);
      _info("MPU-%d, alignedbase=%08"PRIx32" l2size=%"PRIu32" SRD=%"PRIx32""
            "AP=%"PRIx32" XN=%"PRIu32"\n", i, rbar & MPU_RBAR_ADDR_MASK,
            rasr & MPU_RASR_SIZE_MASK, rasr & MPU_RASR_SRD_MASK,
            rasr & MPU_RASR_AP_MASK, rasr & MPU_RASR_XN);
      if (rasr & MPU_RASR_ENABLE)
        {
          count++;
        }
    }

  _info("Total Use Region:%d, Remaining Available:%d\n", count,
        CONFIG_ARM_MPU_NREGIONS - count);
}

/****************************************************************************
 * Name: mpu_reset
 *
 * Description:
 *   Conditional public interface that resets the MPU to disabled during
 *   MPU initialization.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/
#if defined(CONFIG_ARM_MPU_RESET)
void mpu_reset(void)
{
  mpu_reset_internal();
}
#endif

/****************************************************************************
 * Name: mpu_early_reset
 *
 * Description:
 *   Conditional public interface that resets the MPU to disabled immediately
 *   after reset.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/
#if defined(CONFIG_ARM_MPU_EARLY_RESET)
void mpu_early_reset(void)
{
  mpu_reset_internal();
}
#endif
