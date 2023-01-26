/****************************************************************************
 * arch/risc-v/src/common/riscv_addrenv_perms.c
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

static int modify_region(uintptr_t vstart, uintptr_t vend, uintptr_t setmask)
{
  uintptr_t l1vaddr;
  uintptr_t lnvaddr;
  uintptr_t entry;
  uintptr_t paddr;
  uintptr_t vaddr;
  uint32_t  ptlevel;

  /* Must perform a reverse table walk */

  l1vaddr = riscv_pgvaddr(mmu_get_satp_pgbase());

  if (!l1vaddr)
    {
      /* Oops, there is no address environment to modify at all ? */

      return -EINVAL;
    }

  for (vaddr = vstart; vaddr < vend; vaddr += MM_PGSIZE)
    {
      for (ptlevel = 1, lnvaddr = l1vaddr;
           ptlevel < RV_MMU_PT_LEVELS;
           ptlevel++)
        {
          paddr = mmu_pte_to_paddr(mmu_ln_getentry(ptlevel, lnvaddr, vaddr));
          lnvaddr = riscv_pgvaddr(paddr);
          if (!lnvaddr)
            {
              return -EINVAL;
            }
        }

      /* Get entry and modify the flags */

      entry  = mmu_ln_getentry(ptlevel, lnvaddr, vaddr);
      entry &= ~CLR_MASK;
      entry |= setmask;

      /* Restore the entry */

      mmu_ln_restore(ptlevel, lnvaddr, vaddr, entry);
    }

  /* When all is set and done, flush the data caches */

  __ISB();
  __DMB();

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

  if (prot & PROT_WRITE)
    {
      setmask |= PTE_W;
    }

  if (prot & PROT_EXEC)
    {
      setmask |= PTE_X;
    }

  return modify_region(addr, vend, setmask);
}
