/****************************************************************************
 * arch/xtensa/src/common/xtensa_mpu.c
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
#include <debug.h>

#include "mpu.h"

#if defined(XCHAL_MPU_ENTRIES) && XCHAL_MPU_ENTRIES > 0

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Cache for mpu base address in entry */

static uint32_t xtensa_mpu_base[XCHAL_MPU_ENTRIES];

/* The next available region number in decreasing way */

static uint8_t g_region = XCHAL_MPU_ENTRIES;

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
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   The index of the allocated region.
 *
 ****************************************************************************/

unsigned int mpu_allocregion(void)
{
  DEBUGASSERT((unsigned int)g_region >= 0);
  return (unsigned int)--g_region;
}

/****************************************************************************
 * Name: mpu_control
 *
 * Description:
 *   Configure and enable (or disable) the MPU
 *
 * Input Parameters:
 *   enable - Flag indicating whether to enable the MPU.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void mpu_control(bool enable)
{
  uint32_t regval = 0;

  if (enable)
    {
      uint32_t i;
      for (i = XCHAL_MPU_ENTRIES - 1; i >= g_region; i--)
        {
          regval |= (1 << i);
        }
    }

  __asm__ __volatile__ ("wsr %0, mpuenb\n" : : "r"(regval));
}

/****************************************************************************
 * Name: mpu_modify_region
 *
 * Description:
 *   Modify a region for privileged, strongly ordered memory
 *
 * Input Parameters:
 *   region  - Region number to modify.
 *   base    - Base address of the region.
 *   size    - Unused.
 *   acc     - A uint32_t value representing the access permissions of
 *             the region.
 *   memtype - A uint32_t value representing the memory type of the region.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void mpu_modify_region(unsigned int region, uintptr_t base, size_t size,
                       uint32_t acc, uint32_t memtype)
{
  uint32_t at;
  uint32_t as;

  /* Ensure address not overlay
   *
   * Xtensa ISA Reference Manual B4.6.5.3:
   * The lowest address of each foregound segment
   * must be no smaller than the lowest address of the
   * preceding foreground segment in numerical order.
   */

  if (region < (XCHAL_MPU_ENTRIES - 1))
    {
      DEBUGASSERT(base < xtensa_mpu_base[region + 1]);
    }

  /* Now only setup MPU entry, enable mpu in mpu_control */

  as = MPU_ENTRY_AS(base, 0);
  at = MPU_ENTRY_AR(acc, memtype);

  /* Set segment bits */

  at |= region;

  /* Update mpu entry .. requires an memw on same cache line */

  __asm__ __volatile__
    (
      "\tj 1f\n"
      "\t.align 16\n"
      "\t1: memw\n"
      "\twptlb %0, %1\n"
      :
      : "a" (at), "a"(as)
    );

  /* Save base information for check overlap */

  xtensa_mpu_base[region] = base;
}

/****************************************************************************
 * Name: mpu_configure_region
 *
 * Description:
 *   Configure a region
 *
 * Input Parameters:
 *   base    - Base address of the region.
 *   size    - Unused.
 *   acc     - A uint32_t value representing the access permissions of
 *             the region.
 *   memtype - A uint32_t value representing the memory type of the region.
 *
 * Returned Value:
 *   The region number allocated for the configured region.
 *
 ****************************************************************************/

unsigned int mpu_configure_region(uintptr_t base, size_t size,
                                  uint32_t acc, uint32_t memtype)
{
  unsigned int region = mpu_allocregion();
  mpu_modify_region(region, base, size, acc, memtype);
  return region;
}

/****************************************************************************
 * Name: mpu_initialize
 *
 * Description:
 *   Configure a region for privileged, strongly ordered memory
 *
 * Input Parameters:
 *   table - MPU initialization table.
 *   count - Initialize the number of entries in the region table.
 *
 * Returned Value:
 *   NULL.
 *
 ****************************************************************************/

void mpu_initialize(const struct mpu_region_s *table, size_t count)
{
  const struct mpu_region_s *conf;
  size_t index;

  mpu_control(false);
  for (index = 0; index < count; index++)
    {
      conf = &table[index];
      mpu_configure_region(conf->base, conf->size, conf->acc, conf->memtype);
    }

  mpu_control(true);
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

  for (i = XCHAL_MPU_ENTRIES - 1; i >= g_region; i--)
    {
      _info("MPU-Index(%d) Enable, Base:%" PRIu32 "\n", i,
            xtensa_mpu_base[i]);
    }

  _info("Total Use Region:%d, Remaining Available:%d\n",
        XCHAL_MPU_ENTRIES - g_region, g_region);
}

#endif /* defined(XCHAL_MPU_ENTRIES) && XCHAL_MPU_ENTRIES > 0 */
