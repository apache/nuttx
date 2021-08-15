/****************************************************************************
 * arch/arm/src/armv8-m/arm_sau.c
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

#include "sau.h"
#include "arm_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The next available region number */

static uint8_t g_region;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sau_allocregion
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

static unsigned int sau_allocregion(void)
{
  return (unsigned int)g_region++;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sau_control
 *
 * Description:
 *   Configure and enable (or disable) the SAU
 *
 ****************************************************************************/

void sau_control(bool enable, bool allns)
{
  uint32_t regval = 0;

  if (enable)
    {
      regval |= SAU_CTRL_ENABLE; /* Enable the SAU */
    }
  else if (allns)
    {
      regval |= SAU_CTRL_ALLNS; /* All region non-secure */
    }

  putreg32(regval, SAU_CTRL);
}

/****************************************************************************
 * Name: sau_configure_region
 *
 * Description:
 *   Configure a region for secure attribute
 *
 ****************************************************************************/

void sau_configure_region(uintptr_t base, size_t size, uint32_t flags)
{
  unsigned int region = sau_allocregion();
  uintptr_t    limit;

  /* Ensure the base address alignment */

  limit = (base + size) & SAU_RLAR_LIMIT_MASK;
  base &= SAU_RBAR_BASE_MASK;

  /* Select the region */

  putreg32(region, SAU_RNR);

  /* Set the region base, limit and attribute */

  putreg32(base, SAU_RBAR);
  putreg32(limit | flags | SAU_RLAR_ENABLE, SAU_RLAR);
}
