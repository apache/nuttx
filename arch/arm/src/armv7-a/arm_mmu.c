/****************************************************************************
 * arch/arm/src/armv7-a/arm_mmu.c
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

#include "cp15_cacheops.h"
#include "mmu.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mmu_l1_setentry
 *
 * Description:
 *   Set a one level 1 translation table entry.  Only a single L1 page table
 *   is supported.
 *
 * Input Parameters:
 *   paddr - The physical address to be mapped.  Must be aligned to a 1MB
 *     address boundary
 *   vaddr - The virtual address to be mapped.  Must be aligned to a 1MB
 *     address boundary
 *   mmuflags - The MMU flags to use in the mapping.
 *
 ****************************************************************************/

#ifndef CONFIG_ARCH_ROMPGTABLE
void mmu_l1_setentry(uint32_t paddr, uint32_t vaddr, uint32_t mmuflags)
{
  uint32_t *l1table = mmu_l1_pgtable();
  uint32_t  index   = vaddr >> 20;

  /* Save the page table entry */

  l1table[index] = (paddr | mmuflags);

  /* Flush the data cache entry.  Make sure that the modified contents
   * of the page table are flushed into physical memory.
   */

  cp15_clean_dcache_bymva((uint32_t)&l1table[index]);

  /* Invalidate the TLB cache associated with virtual address range */

  mmu_invalidate_region(vaddr, SECTION_SIZE);
}
#endif

/****************************************************************************
 * Name: mmu_l1_restore
 *
 * Description:
 *   Restore one L1 table entry previously returned by mmu_l1_getentry() (or
 *   any other encoded L1 page table value).
 *
 * Input Parameters:
 *   vaddr - A virtual address to be mapped
 *   l1entry - The value to write into the page table entry
 *
 ****************************************************************************/

#if !defined(CONFIG_ARCH_ROMPGTABLE) && defined(CONFIG_ARCH_ADDRENV)
void mmu_l1_restore(uintptr_t vaddr, uint32_t l1entry)
{
  uint32_t *l1table = mmu_l1_pgtable();
  uint32_t  index   = vaddr >> 20;

  /* Set the encoded page table entry */

  l1table[index] = l1entry;

  /* Flush the data cache entry.  Make sure that the modified contents
   * of the page table are flushed into physical memory.
   */

  cp15_clean_dcache_bymva((uint32_t)&l1table[index]);

  /* Invalidate the TLB cache associated with virtual address range */

  mmu_invalidate_region(vaddr & PMD_PTE_PADDR_MASK, SECTION_SIZE);
}
#endif

/****************************************************************************
 * Name: mmu_l2_setentry
 *
 * Description:
 *   Set one small (4096B) entry in a level2 translation table.
 *
 * Input Parameters:
 *   l2vaddr - the virtual address of the beginning of the L2 translation
 *     table.
 *   paddr - The physical address to be mapped.  Must be aligned to a 4KB
 *     address boundary
 *   vaddr - The virtual address to be mapped.  Must be aligned to a 4KB
 *     address boundary
 *   mmuflags - The MMU flags to use in the mapping.
 *
 ****************************************************************************/

#ifndef CONFIG_ARCH_ROMPGTABLE
void mmu_l2_setentry(uint32_t l2vaddr, uint32_t paddr, uint32_t vaddr,
                     uint32_t mmuflags)
{
  uint32_t *l2table  = (uint32_t *)l2vaddr;
  uint32_t  index;

  /* The table divides a 1Mb address space up into 256 entries, each
   * corresponding to 4Kb of address space.  The page table index is
   * related to the offset from the beginning of 1Mb region.
   */

  index = (vaddr & 0x000ff000) >> 12;

  /* Save the table entry */

  l2table[index] = (paddr | mmuflags);

  /* Flush the data cache entry.  Make sure that the modified contents
   * of the page table are flushed into physical memory.
   */

  cp15_clean_dcache_bymva((uint32_t)&l2table[index]);

  /* Invalidate the TLB cache associated with virtual address range */

  cp15_invalidate_tlb_bymva(vaddr);
}
#endif

/****************************************************************************
 * Name: mmu_l1_map_region
 *
 * Description:
 *   Set multiple level 1 translation table entries in order to map a
 *   region of memory.
 *
 * Input Parameters:
 *   mapping - Describes the mapping to be performed.
 *
 ****************************************************************************/

#ifndef CONFIG_ARCH_ROMPGTABLE
void mmu_l1_map_region(const struct section_mapping_s *mapping)
{
  uint32_t paddr    = mapping->physbase;
  uint32_t vaddr    = mapping->virtbase;
  uint32_t mmuflags = mapping->mmuflags;
  int i;

  /* Loop, writing each mapping into the L1 page table */

  for (i = 0; i < mapping->nsections; i++)
    {
      mmu_l1_setentry(paddr, vaddr, mmuflags);
      paddr += SECTION_SIZE;
      vaddr += SECTION_SIZE;
    }
}
#endif

/****************************************************************************
 * Name: mmu_l1_map_regions
 *
 * Description:
 *   Set multiple level 1 translation table entries in order to map a region
 *   array of memory.
 *
 * Input Parameters:
 *   mappings - Describes the array of mappings to be performed.
 *   count    - The number of mappings to be performed.
 *
 ****************************************************************************/

#ifndef CONFIG_ARCH_ROMPGTABLE
void mmu_l1_map_regions(const struct section_mapping_s *mappings,
                        size_t count)
{
  size_t i;

  for (i = 0; i < count; i++)
    {
      mmu_l1_map_region(&mappings[i]);
    }
}
#endif

/****************************************************************************
 * Name: mmu_l1_map_page
 *
 * Description:
 *   Set level 1 page entrie in order to map a region
 *   array of memory.
 *
 * Input Parameters:
 *   mapping - Describes the mapping to be performed.
 *
 ****************************************************************************/

#ifndef CONFIG_ARCH_ROMPGTABLE
void mmu_l1_map_page(const struct section_mapping_s *mapping)
{
  uint32_t virtaddr = mapping->virtbase;
  uint32_t l2table = mapping->physbase;
  uint32_t i;

  for (i = 0; i < mapping->nsections; i++)
    {
      mmu_l1_setentry(l2table, virtaddr, mapping->mmuflags);

      /* Update the L2 page table address for the next L1 table entry. */

      virtaddr += SECTION_SIZE;
      l2table += 1024;
    }
}
#endif

/****************************************************************************
 * Name: mmu_l1_map_pages
 *
 * Description:
 *   Set multiple level 1 page entries in order to map a region
 *   array of memory.
 *
 * Input Parameters:
 *   mappings - Describes the mapping to be performed.
 *   count    - The number of mappings to be performed.
 *
 ****************************************************************************/

#ifndef CONFIG_ARCH_ROMPGTABLE
void mmu_l1_map_pages(const struct section_mapping_s *mappings,
                      size_t count)
{
  size_t i;

  for (i = 0; i < count; i++)
    {
      mmu_l1_map_page(&mappings[i]);
    }
}
#endif

/****************************************************************************
 * Name: mmu_l2_map_page
 *
 * Description:
 *   Set level 2 page entrie in order to map a region
 *   array of memory.
 *
 * Input Parameters:
 *   mapping - Describes the mapping to be performed.
 *
 ****************************************************************************/

#ifndef CONFIG_ARCH_ROMPGTABLE
void mmu_l2_map_page(const struct page_mapping_s *mapping)
{
  uint32_t l2table = mapping->l2table;
  struct page_entry_s *entry;
  uint32_t paddr;
  uint32_t vaddr;
  uint32_t entry_cnt;
  uint32_t page_cnt;

  for (entry_cnt = 0; entry_cnt < mapping->entrynum; entry_cnt++)
    {
      entry = (struct page_entry_s *)&mapping->entry[entry_cnt];
      paddr = entry->physbase;
      vaddr = entry->virtbase;

      for (page_cnt = 0; page_cnt < entry->npages; page_cnt++)
        {
          mmu_l2_setentry(l2table, paddr, vaddr, entry->mmuflags);

          paddr += 4096;
          vaddr += 4096;

          if ((vaddr & 0x000ff000) == 0)
            {
              l2table += 1024;
            }
        }
    }
}
#endif

/****************************************************************************
 * Name: mmu_l2_map_pages
 *
 * Description:
 *   Set multiple level 2 page entries in order to map a region
 *   array of memory.
 *
 * Input Parameters:
 *   mappings - Describes the mapping to be performed.
 *   count    - The number of mappings to be performed.
 *
 ****************************************************************************/

#ifndef CONFIG_ARCH_ROMPGTABLE
void mmu_l2_map_pages(const struct page_mapping_s *mappings,
                      size_t count)
{
  size_t i;

  for (i = 0; i < count; i++)
    {
      mmu_l2_map_page(&mappings[i]);
    }
}
#endif

/****************************************************************************
 * Name: mmu_invalidate_region
 *
 * Description:
 *   Invalidate TLBs for a range of addresses (all 4KB aligned).
 *
 * Input Parameters:
 *   vaddr - The beginning of the region to invalidate.
 *   size  - The size of the region in bytes to be invalidated.
 *
 ****************************************************************************/

#ifndef CONFIG_ARCH_ROMPGTABLE
void mmu_invalidate_region(uint32_t vstart, size_t size)
{
  uint32_t vaddr = vstart & 0xfffff000;
  uint32_t vend  = vstart + size;

  /* Loop, invalidating regions */

  while (vaddr < vend)
    {
      cp15_invalidate_tlb_bymva(vaddr);
      vaddr += 4096;
    }
}
#endif
