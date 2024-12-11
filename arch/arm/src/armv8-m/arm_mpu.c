/****************************************************************************
 * arch/arm/src/armv8-m/arm_mpu.c
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
#include <sys/param.h>
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

/* The available region bitmap */

static unsigned int g_mpu_region;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

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
      putreg32(0, MPU_RLAR);
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
  putreg32(0, MPU_RLAR);
  putreg32(0, MPU_RBAR);
  g_mpu_region &= ~(1 << region);
  UP_MB();
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

  putreg32((MPU_MAIR_STRONGLY_ORDER <<  0) |
           (MPU_MAIR_DEVICE         <<  8) |
           (MPU_MAIR_NONCACHEABLE   << 16) |
           (MPU_MAIR_WRITE_THROUGH  << 24),
           MPU_MAIR0);

  putreg32((MPU_MAIR_WRITE_BACK     <<  0),
           MPU_MAIR1);

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
 *   region - The index of the MPU region to modify.
 *   base   - The base address of the region.
 *   size   - The size of the region.
 *   flags1 - Additional flags for the region.
 *   flags2 - Additional flags for the region.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void mpu_modify_region(unsigned int region, uintptr_t base, size_t size,
                       uint32_t flags1, uint32_t flags2)
{
  uintptr_t limit;
  uintptr_t rbase;

  /* Check that the region is valid */

  DEBUGASSERT(g_mpu_region & (1 << region));

  /* Ensure the base address alignment
   *
   * ARMv8-M Architecture Reference Manual
   * B3.5.8 MPU Region Base Address Register, MPU_RBAR
   * "Software must ensure that the value written to the ADDR field
   * aligns with the size of the selected region."
   */

  limit = (base + size - 1) & MPU_RLAR_LIMIT_MASK;
  rbase = base & MPU_RBAR_BASE_MASK;

  /* Select the region */

  putreg32(region, MPU_RNR);

  /* Set the region base, limit and attribute */

  putreg32(rbase | flags1, MPU_RBAR);
  putreg32(limit | flags2 | MPU_RLAR_ENABLE, MPU_RLAR);

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
 *   base   - The base address of the region.
 *   size   - The size of the region.
 *   flags1 - Additional flags for the region.
 *   flags2 - Additional flags for the region.
 *
 * Returned Value:
 *   The region number allocated for the configured region.
 *
 ****************************************************************************/

unsigned int mpu_configure_region(uintptr_t base, size_t size,
                                  uint32_t flags1, uint32_t flags2)
{
  unsigned int region = mpu_allocregion();
  mpu_modify_region(region, base, size, flags1, flags2);
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
      mpu_configure_region(conf->base, conf->size, conf->flags1,
                           conf->flags2);
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
  uint32_t rlar;
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
      rlar = getreg32(MPU_RLAR);
      rbar = getreg32(MPU_RBAR);
      _info("MPU-%d, 0x%08"PRIx32"-0x%08"PRIx32" SH=%"PRIx32" AP=%"PRIx32""
            "XN=%"PRIu32"\n", i, rbar & MPU_RBAR_BASE_MASK,
            rlar & MPU_RLAR_LIMIT_MASK, rbar & MPU_RBAR_SH_MASK,
            rbar & MPU_RBAR_AP_MASK, rbar & MPU_RBAR_XN);
      if (rlar & MPU_RLAR_ENABLE)
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
