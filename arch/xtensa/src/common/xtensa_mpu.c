/****************************************************************************
 * arch/xtensa/src/common/xtensa_mpu.c
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

#include "mpu.h"

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
 * Name: mpu_configure_region
 *
 * Description:
 *   Configure a region
 *
 ****************************************************************************/

void mpu_configure_region(uintptr_t base, size_t size,
                          uint32_t acc, uint32_t memtype)
{
  unsigned int region = mpu_allocregion();
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

  /* update mpu entry .. requires an memw on same cache line */

  __asm__ __volatile__
    (
      "\tj 1f\n"
      "\t.align 16\n"
      "\t1: memw\n"
      "\twptlb %0, %1\n"
      :
      : "a" (at), "a"(as)
    );

  xtensa_mpu_base[region] = base;
}
