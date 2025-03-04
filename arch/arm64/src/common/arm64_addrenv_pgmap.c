/****************************************************************************
 * arch/arm64/src/common/arm64_addrenv_pgmap.c
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

#include <nuttx/arch.h>
#include <nuttx/addrenv.h>
#include <nuttx/irq.h>
#include <nuttx/pgalloc.h>
#include <nuttx/sched.h>

#include <arch/barriers.h>

#include <sys/mman.h>

#include "pgalloc.h"
#include "arm64_mmu.h"

#ifdef CONFIG_BUILD_KERNEL

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_MM_KMAP
static struct arch_addrenv_s g_kernel_addrenv;
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_addrenv_find_page
 *
 * Description:
 *   Find physical page mapped to user virtual address from the address
 *   environment page directory.
 *
 * Input Parameters:
 *   addrenv - The user address environment.
 *   vaddr   - The user virtual address
 *
 * Returned Value:
 *   Page physical address on success; NULL on failure.
 *
 ****************************************************************************/

uintptr_t up_addrenv_find_page(arch_addrenv_t *addrenv, uintptr_t vaddr)
{
  uintptr_t pgdir;
  uintptr_t lnvaddr;
  uintptr_t paddr;
  uint32_t  ptlevel;

  /* If vaddr is not user space, get out */

  if (!arm64_uservaddr(vaddr))
    {
      return 0;
    }

  /* Get the kernel addressable virtual address of the page directory root */

  pgdir = arm64_pgvaddr(mmu_ttbr_to_paddr(addrenv->ttbr0));
  if (!pgdir)
    {
      return 0;
    }

  /* Make table walk to find the page */

  for (ptlevel = mmu_get_base_pgt_level(), lnvaddr = pgdir;
       ptlevel < MMU_PGT_LEVEL_MAX;
       ptlevel++)
    {
      paddr = mmu_pte_to_paddr(mmu_ln_getentry(ptlevel, lnvaddr, vaddr));
      lnvaddr = arm64_pgvaddr(paddr);
      if (!lnvaddr)
        {
          return 0;
        }
    }

  /* Got it, now convert it to physical page */

  paddr = mmu_ln_getentry(ptlevel, lnvaddr, vaddr);
  paddr = mmu_pte_to_paddr(paddr);

  return paddr;
}

/****************************************************************************
 * Name: up_addrenv_page_vaddr
 *
 * Description:
 *   Find the kernel virtual address associated with physical page.
 *
 * Input Parameters:
 *   page - The page physical address.
 *
 * Returned Value:
 *   Page kernel virtual address on success; NULL on failure.
 *
 ****************************************************************************/

uintptr_t up_addrenv_page_vaddr(uintptr_t page)
{
  return arm64_pgvaddr(page);
}

/****************************************************************************
 * Name: up_addrenv_user_vaddr
 *
 * Description:
 *   Check if a virtual address is in user virtual address space.
 *
 * Input Parameters:
 *   vaddr - The virtual address.
 *
 * Returned Value:
 *   True if it is; false if it's not
 *
 ****************************************************************************/

bool up_addrenv_user_vaddr(uintptr_t vaddr)
{
  return arm64_uservaddr(vaddr);
}

/****************************************************************************
 * Name: up_addrenv_page_wipe
 *
 * Description:
 *   Wipe a page of physical memory, first mapping it into kernel virtual
 *   memory.
 *
 * Input Parameters:
 *   page - The page physical address.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void up_addrenv_page_wipe(uintptr_t page)
{
  arm64_pgwipe(page);
}

#ifdef CONFIG_MM_KMAP

/****************************************************************************
 * Name: up_addrenv_kmap_init
 *
 * Description:
 *   Initialize the architecture specific part of the kernel mapping
 *   interface.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

int up_addrenv_kmap_init(void)
{
  struct arch_addrenv_s *addrenv;
  uintptr_t              next;
  uintptr_t              vaddr;
  uintptr_t              l0;
  int                    i;

  /* Populate the static page tables one by one */

  addrenv = &g_kernel_addrenv;
  next    =  g_kernel_pgt_pbase;
  vaddr   =  CONFIG_ARCH_KMAP_VBASE;
  l0      =  mmu_get_base_pgt_level();

  for (i = l0; i < ARCH_SPGTS; i++)
    {
      /* Connect the static page tables */

      uintptr_t lnvaddr = arm64_pgvaddr(next);
      addrenv->spgtables[i] = next;
      next = mmu_pte_to_paddr(mmu_ln_getentry(i, lnvaddr, vaddr));
    }

  /* Set the page directory root */

  addrenv->ttbr0 = mmu_ttbr_reg(addrenv->spgtables[l0], 0);

  /* When all is set and done, flush the data caches */

  UP_DSB();

  return OK;
}

/****************************************************************************
 * Name: up_addrenv_kmap_pages
 *
 * Description:
 *   Map physical pages into a continuous virtual memory block.
 *
 * Input Parameters:
 *   pages - A pointer to the first element in a array of physical address,
 *     each corresponding to one page of memory.
 *   npages - The number of pages in the list of physical pages to be mapped.
 *   vaddr - The virtual address corresponding to the beginning of the
 *     (continuous) virtual address region.
 *   prot - Access right flags.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

int up_addrenv_kmap_pages(void **pages, unsigned int npages, uintptr_t vaddr,
                          int prot)
{
  struct arch_addrenv_s *addrenv = &g_kernel_addrenv;
  int mask                       = 0;

  /* Sanity checks */

  DEBUGASSERT(pages != NULL && npages > 0);
  DEBUGASSERT(vaddr >= CONFIG_ARCH_KMAP_VBASE && vaddr < ARCH_KMAP_VEND);
  DEBUGASSERT(MM_ISALIGNED(vaddr));

  /* The access is either RO or RW, read access cannot be revoked */

  if ((prot & PROT_WRITE) == 0)
    {
      mask = PTE_BLOCK_DESC_AP_RO;
    }
  else
    {
      mask = PTE_BLOCK_DESC_AP_RW;
    }

  /* Execute access is explicitly revoked, implicitly granted */

  if ((prot & PROT_EXEC) == 0)
    {
      mask |= PTE_BLOCK_DESC_PXN;
    }

  /* This is a kernel (global) mapping */

  mask &= ~PTE_BLOCK_DESC_NG;

  /* Also, revoke user execute access */

  mask |= PTE_BLOCK_DESC_UXN;

  /* Flags for normal memory region */

  mask |= MMU_MT_NORMAL_FLAGS;

  /* Let arm64_map_pages do the work */

  return arm64_map_pages(addrenv, (uintptr_t *)pages, npages, vaddr, mask);
}

/****************************************************************************
 * Name: up_addrenv_kunmap_pages
 *
 * Description:
 *   Unmap a previously mapped virtual memory region.
 *
 * Input Parameters:
 *   vaddr - The virtual address corresponding to the beginning of the
 *     (continuous) virtual address region.
 *   npages - The number of pages to be unmapped
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

int up_addrenv_kunmap_pages(uintptr_t vaddr, unsigned int npages)
{
  struct arch_addrenv_s *addrenv = &g_kernel_addrenv;

  /* Sanity checks */

  DEBUGASSERT(npages > 0);
  DEBUGASSERT(vaddr >= CONFIG_ARCH_KMAP_VBASE && vaddr < ARCH_KMAP_VEND);
  DEBUGASSERT(MM_ISALIGNED(vaddr));

  /* Let arm64_unmap_pages do the work */

  return arm64_unmap_pages(addrenv, vaddr, npages);
}

#endif /* CONFIG_MM_KMAP */
#endif /* CONFIG_BUILD_KERNEL */
