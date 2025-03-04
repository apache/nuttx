/****************************************************************************
 * arch/arm64/src/common/arm64_addrenv_perms.c
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

#include <arch/barriers.h>

#include <sys/mman.h>

#include "pgalloc.h"
#include "arm64_arch.h"
#include "arm64_mmu.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CLR_MASK (PTE_BLOCK_DESC_AP_MASK | PTE_BLOCK_DESC_UXN)

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

  l1vaddr = arm64_pgvaddr(mmu_get_ttbr0_pgbase());

  if (!l1vaddr)
    {
      /* Oops, there is no address environment to modify at all ? */

      return -EINVAL;
    }

  for (vaddr = vstart; vaddr < vend; vaddr += MM_PGSIZE)
    {
      for (ptlevel = mmu_get_base_pgt_level(), lnvaddr = l1vaddr;
           ptlevel < MMU_PGT_LEVEL_MAX;
           ptlevel++)
        {
          paddr = mmu_pte_to_paddr(mmu_ln_getentry(ptlevel, lnvaddr, vaddr));
          lnvaddr = arm64_pgvaddr(paddr);
          if (!lnvaddr)
            {
              return -EINVAL;
            }
        }

      /* Get entry and modify the flags */

      entry  = mmu_ln_getentry(ptlevel, lnvaddr, vaddr);
      entry &= ~CLR_MASK;
      entry |= setmask | PTE_BLOCK_DESC_AP_USER;

      /* Restore the entry */

      mmu_ln_restore(ptlevel, lnvaddr, vaddr, entry);
    }

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

  /* addrenv not needed by this implementation */

  UNUSED(addrenv);

  /* The access is either RO or RW, read access cannot be revoked */

  if ((prot & PROT_WRITE) == 0)
    {
      setmask = PTE_BLOCK_DESC_AP_RO;
    }
  else
    {
      setmask = PTE_BLOCK_DESC_AP_RW;
    }

  /* Execute access is explicitly revoked, implicitly granted */

  if ((prot & PROT_EXEC) == 0)
    {
      setmask |= PTE_BLOCK_DESC_UXN;
    }

  return modify_region(addr, addr + MM_PGALIGNUP(len), setmask);
}
