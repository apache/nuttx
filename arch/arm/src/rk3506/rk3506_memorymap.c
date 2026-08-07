/****************************************************************************
 * arch/arm/src/rk3506/rk3506_memorymap.c
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

#include <sys/param.h>

#include "mmu.h"

#include "hardware/rk3506_memorymap.h"
#include "rk3506_memorymap.h"

/****************************************************************************
 * Macro Definitions
 ****************************************************************************/

#define _NSECTIONS(b)                 (((b) + 0x000fffff) >> 20)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct section_mapping_s g_section_mapping[] =
{
  {
    RK3506_DDR_PSECTION, RK3506_DDR_VSECTION,
    MMU_MEMFLAGS, _NSECTIONS(RK3506_DDR_SECSIZE)
  },
  {
    RK3506_DEVICE_PSECTION, RK3506_DEVICE_VSECTION,
    MMU_IOFLAGS, _NSECTIONS(RK3506_DEVICE_SECSIZE)
  },
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rk3506_setupmappings
 *
 * Description:
 *   Initializes the non-code area page table
 *
 ****************************************************************************/

int rk3506_setupmappings(void)
{
  mmu_l1_map_regions(g_section_mapping, nitems(g_section_mapping));

  return 0;
}
