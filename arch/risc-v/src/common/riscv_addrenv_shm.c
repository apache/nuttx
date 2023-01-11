/****************************************************************************
 * arch/risc-v/src/common/riscv_addrenv_shm.c
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

#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/addrenv.h>
#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/pgalloc.h>

#include <arch/barriers.h>

#include "addrenv.h"
#include "pgalloc.h"
#include "riscv_mmu.h"

#if defined(CONFIG_BUILD_KERNEL) && defined(CONFIG_ARCH_VMA_MAPPING)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_shmat
 *
 * Description:
 *   Attach, i.e, map, on shared memory region to a user virtual address
 *
 * Input Parameters:
 *   pages - A pointer to the first element in a array of physical address,
 *     each corresponding to one page of memory.
 *   npages - The number of pages in the list of physical pages to be mapped.
 *   vaddr - The virtual address corresponding to the beginning of the
 *     (contiguous) virtual address region.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

int up_shmat(uintptr_t *pages, unsigned int npages, uintptr_t vaddr)
{
  struct tcb_s        *tcb = nxsched_self();
  struct task_group_s *group;
  uintptr_t            ptlast;
  uintptr_t            ptlevel;
  uintptr_t            paddr;

  /* Sanity checks */

  DEBUGASSERT(tcb && tcb->group);
  DEBUGASSERT(pages != NULL && npages > 0);
  DEBUGASSERT(vaddr >= CONFIG_ARCH_SHM_VBASE && vaddr < ARCH_SHM_VEND);
  DEBUGASSERT(MM_ISALIGNED(vaddr));

  group   = tcb->group;
  ptlevel = RV_MMU_PT_LEVELS;

  /* Add the references to pages[] into the caller's address environment */

  for (; npages > 0; npages--)
    {
      /* Get the address of the last level page table */

      ptlast = riscv_pgvaddr(riscv_get_pgtable(&group->tg_addrenv, vaddr));
      if (!ptlast)
        {
          return -ENOMEM;
        }

      /* Then add the reference */

      paddr = *pages++;
      mmu_ln_setentry(ptlevel, ptlast, paddr, vaddr, MMU_UDATA_FLAGS);
      vaddr += MM_PGSIZE;
    }

  /* Flush the data cache, so the changes are committed to memory */

  __DMB();

  return OK;
}

/****************************************************************************
 * Name: up_shmdt
 *
 * Description:
 *   Detach, i.e, unmap, on shared memory region from a user virtual address
 *
 * Input Parameters:
 *   vaddr - The virtual address corresponding to the beginning of the
 *     (contiguous) virtual address region.
 *   npages - The number of pages to be unmapped
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

int up_shmdt(uintptr_t vaddr, unsigned int npages)
{
  struct tcb_s        *tcb = nxsched_self();
  struct task_group_s *group;
  uintptr_t            ptlast;
  uintptr_t            ptprev;
  uintptr_t            ptlevel;
  uintptr_t            paddr;

  /* Sanity checks */

  DEBUGASSERT(tcb && tcb->group);
  DEBUGASSERT(npages > 0);
  DEBUGASSERT(vaddr >= CONFIG_ARCH_SHM_VBASE && vaddr < ARCH_SHM_VEND);
  DEBUGASSERT(MM_ISALIGNED(vaddr));

  group   = tcb->group;
  ptlevel = ARCH_SPGTS;
  ptprev  = riscv_pgvaddr(group->tg_addrenv.spgtables[ARCH_SPGTS - 1]);
  if (!ptprev)
    {
      /* Something is very wrong */

      return -EFAULT;
    }

  /* Remove the references from the caller's address environment */

  for (; npages > 0; npages--)
    {
      /* Get the current final level entry corresponding to this vaddr */

      paddr = mmu_pte_to_paddr(mmu_ln_getentry(ptlevel, ptprev, vaddr));
      ptlast = riscv_pgvaddr(paddr);

      /* Then wipe the reference */

      mmu_ln_clear(ptlevel + 1, ptlast, vaddr);
      vaddr += MM_PGSIZE;
    }

  /* Flush the data cache, so the changes are committed to memory */

  __DMB();

  return OK;
}

#endif /* CONFIG_BUILD_KERNEL */
