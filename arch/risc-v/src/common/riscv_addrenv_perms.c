/****************************************************************************
 * arch/risc-v/src/common/riscv_addrenv_perms.c
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

#include <errno.h>
#include <assert.h>

#include <nuttx/addrenv.h>
#include <nuttx/arch.h>
#include <nuttx/pgalloc.h>
#include <nuttx/sched.h>

#include <arch/barriers.h>

#include <sys/mman.h>

#include "pgalloc.h"
#include "riscv_mmu.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CLR_MASK (PTE_R | PTE_W | PTE_X)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_SMP
static int flush_hart(FAR void *arg)
{
  UNUSED(arg);
  mmu_invalidate_tlbs();
  UP_ISB();
  return OK;
}
#endif

static uintptr_t get_pgtable(uintptr_t l1vaddr, uintptr_t vaddr)
{
  uintptr_t lnvaddr;
  uint32_t  ptlevel;

  for (ptlevel = 1, lnvaddr = l1vaddr;
       ptlevel < RV_MMU_PT_LEVELS && lnvaddr != 0;
       ptlevel++)
    {
      lnvaddr = riscv_pgvaddr(mmu_pte_to_paddr(
                  mmu_ln_getentry(ptlevel, lnvaddr, vaddr)));
    }

  return lnvaddr;
}

/* With on-demand paging the page fault handler maps a page of .text, .data
 * or the heap at its first access.
 */

static bool paged_vaddr(uintptr_t vaddr)
{
#ifdef CONFIG_PAGING
  return (vaddr >= CONFIG_ARCH_TEXT_VBASE && vaddr < ARCH_TEXT_VEND) ||
         (vaddr >= CONFIG_ARCH_DATA_VBASE && vaddr < ARCH_DATA_VEND) ||
         (vaddr >= CONFIG_ARCH_HEAP_VBASE && vaddr < ARCH_HEAP_VEND);
#else
  UNUSED(vaddr);
  return false;
#endif
}

static int modify_region(uintptr_t vstart, uintptr_t vend, uintptr_t setmask)
{
  uintptr_t l1vaddr;
  uintptr_t lnvaddr;
  uintptr_t entry;
  uintptr_t vaddr;

  /* Must perform a reverse table walk */

  l1vaddr = riscv_pgvaddr(mmu_get_satp_pgbase());

  if (!l1vaddr)
    {
      /* Oops, there is no address environment to modify at all ? */

      return -EINVAL;
    }

  /* Change nothing unless each page is mapped user memory */

  for (vaddr = vstart; vaddr < vend; vaddr += MM_PGSIZE)
    {
      lnvaddr = get_pgtable(l1vaddr, vaddr);
      if (!riscv_uservaddr(vaddr) || lnvaddr == 0)
        {
          return -ENOMEM;
        }

      if ((mmu_ln_getentry(RV_MMU_PT_LEVELS, lnvaddr, vaddr) &
           PTE_VALID) == 0 && !paged_vaddr(vaddr))
        {
          return -ENOMEM;
        }
    }

  for (vaddr = vstart; vaddr < vend; vaddr += MM_PGSIZE)
    {
      /* Get entry and modify the flags */

      lnvaddr = get_pgtable(l1vaddr, vaddr);
      entry   = mmu_ln_getentry(RV_MMU_PT_LEVELS, lnvaddr, vaddr);
      entry  &= ~CLR_MASK;
      entry  |= setmask;

      /* Restore the entry */

      mmu_ln_restore(RV_MMU_PT_LEVELS, lnvaddr, vaddr, entry);
    }

  /* Make the new entries and the code in the pages visible to instruction
   * fetch: sfence.vma and fence.i act on one hart only.
   */

  UP_DMB();
#ifdef CONFIG_SMP
  {
    cpu_set_t cpus;
    int cpu;

    CPU_ZERO(&cpus);
    for (cpu = 0; cpu < CONFIG_SMP_NCPUS; cpu++)
      {
        CPU_SET(cpu, &cpus);
      }

    nxsched_smp_call(cpus, flush_hart, NULL);
  }
#else
  UP_ISB();
#endif

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_addrenv_mprot
 *
 * Description:
 *   Modify access rights to an address range.
 *
 * Input Parameters:
 *   addrenv - The address environment to be modified.
 *   addr - Base address of the region.
 *   len - Size of the region.
 *   prot - Access right flags.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int up_addrenv_mprot(arch_addrenv_t *addrenv, uintptr_t addr, size_t len,
                     int prot)
{
  uintptr_t setmask;
  uintptr_t vend;

  /* addrenv not needed by this implementation */

  UNUSED(addrenv);

  setmask = 0;
  vend    = addr + MM_PGALIGNUP(len);

  if (prot & PROT_READ)
    {
      setmask |= PTE_R;
    }

  /* A writable page must be readable: W without R is reserved */

  if (prot & PROT_WRITE)
    {
      setmask |= PTE_R | PTE_W;
    }

  if (prot & PROT_EXEC)
    {
      setmask |= PTE_X;
    }

  return modify_region(addr, vend, setmask);
}
