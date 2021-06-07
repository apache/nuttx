/****************************************************************************
 * arch/arm/src/arm/arm_va2pte.c
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

#include <nuttx/sched.h>
#include <nuttx/page.h>

#include "chip.h"
#include "pg_macros.h"
#include "arm_internal.h"

#ifdef CONFIG_PAGING

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

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
 * Name: arm_va2pte()
 *
 * Description:
 *  Convert a virtual address within the paged text region into a pointer to
 *  the corresponding page table entry.
 *
 * Input Parameters:
 *   vaddr - The virtual address within the paged text region.
 *
 * Returned Value:
 *   A pointer to  the corresponding page table entry.
 *
 * Assumptions:
 *   - This function is called from the normal tasking context (but with
 *     interrupts disabled).  The implementation must take whatever actions
 *     are necessary to assure that the operation is safe within this
 *     context.
 *
 ****************************************************************************/

uint32_t *arm_va2pte(uintptr_t vaddr)
{
  uint32_t L1;
  uint32_t *L2;
  unsigned int ndx;

  /* The virtual address is expected to lie in the paged text region */

  DEBUGASSERT(vaddr >= PG_PAGED_VBASE && vaddr < PG_PAGED_VEND);

  /* Get the L1 table entry associated with this virtual address */

  L1 = *(uint32_t *)PG_POOL_VA2L1VADDR(vaddr);

  /* Get the address of the L2 page table from the L1 entry */

  L2 = (uint32_t *)PG_POOL_L12VPTABLE(L1);

  /* Get the index into the L2 page table.  Each L1 entry maps
   * 256 x 4Kb or 1024 x 1Kb pages.
   */

  ndx = (vaddr & 0x000fffff) >> PAGESHIFT;

  /* Return true if this virtual address is mapped. */

  return &L2[ndx];
}

#endif /* CONFIG_PAGING */
