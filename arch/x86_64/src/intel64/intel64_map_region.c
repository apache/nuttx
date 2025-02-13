/****************************************************************************
 * arch/x86_64/src/intel64/intel64_map_region.c
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

#include <debug.h>
#include <nuttx/irq.h>
#include <nuttx/pgalloc.h>

#include "x86_64_internal.h"
#include "x86_64_mmu.h"
#include "pgalloc.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline_function
void map_region_lowmem(void *base, size_t size, int flags)
{
  uint64_t bb;
  uint64_t num_of_pages;
  uint64_t entry;
  uint64_t curr;
  int i;

  /* Round to page boundary */

  bb = (uint64_t)base & ~(PAGE_SIZE - 1);

  /* Increase size if the base address is rounded off */

  size += (uint64_t)base - bb;
  num_of_pages = (size + PAGE_SIZE - 1) / PAGE_SIZE;

  curr = bb;
  for (i = 0; i < num_of_pages; i++)
    {
      entry = (curr >> 12) & 0x7ffffff;

      g_pt[entry] = curr | flags;
      curr += PAGE_SIZE;
    }
}

static inline_function
void map_region_highmem_1g(void *base, size_t size, int flags)
{
  uint64_t curr;
  uint64_t num_of_pages;
  uint64_t entry;
  int i;

  flags |= X86_PAGE_HUGE;
  curr   = (uint64_t)base & HUGE_PAGE_MASK_1G;
  num_of_pages = (size + (HUGE_PAGE_SIZE_1G - 1)) / HUGE_PAGE_SIZE_1G;

  for (i = 0; i < num_of_pages; i++)
    {
      entry = (curr >> 30) & 0x7ffffff;
      g_pdpt[entry] = curr | flags;
      curr += HUGE_PAGE_SIZE_1G;
    }
}

/****************************************************************************
 * Name: up_map_region_higmem
 *
 * Description:
 *   Map a memory region as 1:1 by MMU for high memory region (>4Gb)
 *
 ****************************************************************************/

#ifdef CONFIG_MM_PGALLOC
static int map_region_highmem(void *base, size_t size, int flags)
{
  uintptr_t bb;
  int       ptlevel;
  uintptr_t ptprev;
  uintptr_t paddr;
  uintptr_t vaddr;
  size_t    nmapped;
  int       i;

  /* Round to page boundary */

  bb = (uintptr_t)base & ~(PAGE_SIZE - 1);

  /* Increase size if the base address is rounded off */

  size += (uintptr_t)base - bb;

  /* Map 1:1 */

  vaddr   = bb;
  nmapped = 0;

  while (nmapped < size)
    {
      /* Start from PTL4 */

      ptprev = x86_64_pgvaddr(get_pml4());

      for (ptlevel = 0; ptlevel < X86_MMU_PT_LEVELS - 1; ptlevel++)
        {
          paddr = mmu_pte_to_paddr(mmu_ln_getentry(ptlevel, ptprev, vaddr));
          if (!paddr)
            {
              /* Nothing yet, allocate one page for final level page table */

              paddr = mm_pgalloc(1);
              if (!paddr)
                {
                  return -ENOMEM;
                }

              /* Map the page table to the prior level */

              mmu_ln_setentry(ptlevel, ptprev, paddr, vaddr, X86_PAGE_WR);

              /* This is then used to map the final level */

              x86_64_pgwipe(paddr);
            }

          ptprev = x86_64_pgvaddr(paddr);
        }

      /* Then map the virtual address to the physical address */

      for (i = X86_MMU_VADDR_INDEX(vaddr, ptlevel);
           i < X86_MMU_ENTRIES_PER_PGT && nmapped < size;
           i++)
        {
          mmu_ln_setentry(ptlevel, ptprev, bb + nmapped, vaddr, flags);
          nmapped += MM_PGSIZE;
          vaddr   += MM_PGSIZE;
        }
    }

  return 0;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_map_region
 *
 * Description:
 *   Map a memory region as 1:1 by MMU
 *
 ****************************************************************************/

int up_map_region(void *base, size_t size, int flags)
{
  if ((uintptr_t)base > 0xffffffff)
    {
      /* More than 4GB only the 1G huge pages can be mapped.
       * Others can't be mapped without CONFIG_MM_PGALLOC.
       */

      if (((uintptr_t)base < 512ul * HUGE_PAGE_SIZE_1G) &&
          (size & (HUGE_PAGE_SIZE_1G - 1)) == 0)
        {
          map_region_highmem_1g(base, size, flags);
          return 0;
        }
      else
        {
#ifdef CONFIG_MM_PGALLOC
          return map_region_highmem(base, size, flags);
#else
          PANIC();
#endif
        }
    }

  map_region_lowmem(base, size, flags);

  return 0;
}
