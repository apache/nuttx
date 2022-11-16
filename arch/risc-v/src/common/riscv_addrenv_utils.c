/****************************************************************************
 * arch/risc-v/src/common/riscv_addrenv_utils.c
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

#include <nuttx/arch.h>
#include <nuttx/addrenv.h>
#include <nuttx/irq.h>
#include <nuttx/pgalloc.h>
#include <nuttx/sched.h>

#include <arch/barriers.h>

#include "pgalloc.h"
#include "riscv_mmu.h"

#ifdef CONFIG_BUILD_KERNEL

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: riscv_get_pgtable
 *
 * Description:
 *   Get the physical address of the final level page table corresponding to
 *   'vaddr'. If one does not exist, it will be allocated.
 *
 * Input Parameters:
 *   addrenv - Pointer to a structure describing the address environment
 *   vaddr - Virtual address to query for
 *
 * Returned Value:
 *   The physical address of the corresponding final level page table, or
 *   NULL if one does not exist, and there is no free memory to allocate one
 *
 ****************************************************************************/

uintptr_t riscv_get_pgtable(group_addrenv_t *addrenv, uintptr_t vaddr)
{
  uintptr_t paddr;
  uintptr_t ptprev;
  uint32_t  ptlevel;

  /* Get the current level MAX_LEVELS-1 entry corresponding to this vaddr */

  ptlevel = ARCH_SPGTS;
  ptprev  = riscv_pgvaddr(addrenv->spgtables[ARCH_SPGTS - 1]);
  paddr   = mmu_pte_to_paddr(mmu_ln_getentry(ptlevel, ptprev, vaddr));

  if (!paddr)
    {
      /* No page table has been allocated... allocate one now */

      paddr = mm_pgalloc(1);
      if (paddr)
        {
          /* Wipe the page and assign it */

          riscv_pgwipe(paddr);
          mmu_ln_setentry(ptlevel, ptprev, paddr, vaddr, MMU_UPGT_FLAGS);
        }
    }

  /* Flush the data cache, so the changes are committed to memory */

  __DMB();

  return paddr;
}

#endif /* CONFIG_BUILD_KERNEL */
