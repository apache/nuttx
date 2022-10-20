/****************************************************************************
 * arch/arm/src/armv8-m/arm_mpu.c
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

/* The next available region number */

static uint8_t g_region;

/****************************************************************************
 * Private Functions
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
  DEBUGASSERT(g_region < CONFIG_ARM_MPU_NREGIONS);
  return (unsigned int)g_region++;
}

/****************************************************************************
 * Name: mpu_reset_internal
 *
 * Description:
 *   Resets the MPU to disabled.
 *
 ****************************************************************************/

#if defined(CONFIG_MPU_RESET) || defined(CONFIG_ARM_MPU_EARLY_RESET)
static void mpu_reset_internal()
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
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mpu_control
 *
 * Description:
 *   Configure and enable (or disable) the MPU
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
}

/****************************************************************************
 * Name: mpu_configure_region
 *
 * Description:
 *   Configure a region for privileged, strongly ordered memory
 *
 ****************************************************************************/

void mpu_configure_region(uintptr_t base, size_t size,
                          uint32_t flags1, uint32_t flags2)
{
  unsigned int region = mpu_allocregion();
  uintptr_t    limit;

  /* Ensure the base address alignment
   *
   * ARMv8-M Architecture Reference Manual
   * B3.5.8 MPU Region Base Address Register, MPU_RBAR
   * "Software must ensure that the value written to the ADDR field
   * aligns with the size of the selected region."
   */

  limit = (base + size - 1) & MPU_RLAR_LIMIT_MASK;
  base &= MPU_RBAR_BASE_MASK;

  /* Select the region */

  putreg32(region, MPU_RNR);

  /* Set the region base, limit and attribute */

  putreg32(base | flags1, MPU_RBAR);
  putreg32(limit | flags2 | MPU_RLAR_ENABLE, MPU_RLAR);
}

/****************************************************************************
 * Name: mpu_reset
 *
 * Description:
 *   Conditional public interface that resets the MPU to disabled during
 *   MPU initialization.
 *
 ****************************************************************************/
#if defined(CONFIG_MPU_RESET)
void mpu_reset()
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
 ****************************************************************************/
#if defined(CONFIG_ARM_MPU_EARLY_RESET)
void mpu_early_reset()
{
  mpu_reset_internal();
}
#endif
