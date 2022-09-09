/****************************************************************************
 * arch/risc-v/src/mpfs/mpfs_mm_init.c
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

#include <stdint.h>
#include <assert.h>
#include <debug.h>

#include <arch/board/board_memorymap.h>

#include "mpfs_memorymap.h"

#include "riscv_internal.h"
#include "riscv_mmu.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Map the whole I/O memory with vaddr = paddr mappings */

#define MMU_IO_BASE     (0x00000000)
#define MMU_IO_SIZE     (0x80000000)

/* Physical and virtual addresses to page tables (vaddr = paddr mapping) */

#define PGT_L1_PBASE    (uintptr_t)&m_l1_pgtable
#define PGT_L2_PBASE    (uintptr_t)&m_l2_pgtable
#define PGT_L3_PBASE    (uintptr_t)&m_l3_pgtable
#define PGT_L1_VBASE    PGT_L1_PBASE
#define PGT_L2_VBASE    PGT_L2_PBASE
#define PGT_L3_VBASE    PGT_L3_PBASE

#define PGT_L1_SIZE     (512)  /* Enough to map 512 GiB */
#define PGT_L2_SIZE     (512)  /* Enough to map 1 GiB */
#define PGT_L3_SIZE     (1024) /* Enough to map 4 MiB */

#define SLAB_COUNT      (sizeof(m_l3_pgtable) / RV_MMU_PAGE_SIZE)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct pgalloc_slab_s
{
  sq_entry_t  *next;
  void        *memory;
};
typedef struct pgalloc_slab_s pgalloc_slab_t;

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Kernel mappings simply here, mapping is vaddr=paddr */

static uint64_t         m_l1_pgtable[PGT_L1_SIZE] locate_data(".pgtables");
static uint64_t         m_l2_pgtable[PGT_L2_SIZE] locate_data(".pgtables");
static uint64_t         m_l3_pgtable[PGT_L3_SIZE] locate_data(".pgtables");

/* Kernel mappings (L1 base) */

uintptr_t               g_kernel_mappings  = PGT_L1_VBASE;
uintptr_t               g_kernel_pgt_pbase = PGT_L1_PBASE;

/* L3 page table allocator */

static sq_queue_t       g_free_slabs;
static pgalloc_slab_t   g_slabs[SLAB_COUNT];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: slab_init
 *
 * Description:
 *   Initialize slab allocator for L3 page table entries
 *
 * Input Parameters:
 *   start - Beginning of the L3 page table pool
 *
 ****************************************************************************/

static void slab_init(uintptr_t start)
{
  int i;

  sq_init(&g_free_slabs);

  for (i = 0; i < SLAB_COUNT; i++)
    {
      g_slabs[i].memory = (void *)start;
      sq_addlast((sq_entry_t *)&g_slabs[i], (sq_queue_t *)&g_free_slabs);
      start += RV_MMU_PAGE_SIZE;
    }
}

/****************************************************************************
 * Name: slab_alloc
 *
 * Description:
 *   Allocate single slab for L3 page table entry
 *
 ****************************************************************************/

static uintptr_t slab_alloc(void)
{
  pgalloc_slab_t *slab = (pgalloc_slab_t *)sq_remfirst(&g_free_slabs);
  return slab ? (uintptr_t)slab->memory : (uintptr_t)NULL;
}

/****************************************************************************
 * Name: map_region
 *
 * Description:
 *   Map a region of physical memory to the L3 page table
 *
 * Input Parameters:
 *   paddr - Beginning of the physical address mapping
 *   vaddr - Beginning of the virtual address mapping
 *   size - Size of the region in bytes
 *   mmuflags - The MMU flags to use in the mapping
 *
 ****************************************************************************/

static void map_region(uintptr_t paddr, uintptr_t vaddr, size_t size,
                       uint32_t mmuflags)
{
  uintptr_t endaddr;
  uintptr_t l3pbase;
  int npages;
  int i;
  int j;

  /* How many pages */

  npages = (size + RV_MMU_PAGE_MASK) >> RV_MMU_PAGE_SHIFT;
  endaddr = vaddr + size;

  for (i = 0; i < npages; i += RV_MMU_PAGE_ENTRIES)
    {
      /* See if a L3 mapping exists ? */

      l3pbase = mmu_pte_to_paddr(mmu_ln_getentry(2, PGT_L2_VBASE, vaddr));
      if (!l3pbase)
        {
          /* No, allocate 1 page, this must not fail */

          l3pbase = slab_alloc();
          DEBUGASSERT(l3pbase);

          /* Map it to the L3 table */

          mmu_ln_setentry(2, PGT_L2_VBASE, l3pbase, vaddr, MMU_UPGT_FLAGS);
        }

      /* Then add the L3 mappings */

      for (j = 0; j < RV_MMU_PAGE_ENTRIES && vaddr < endaddr; j++)
        {
          mmu_ln_setentry(3, l3pbase, paddr, vaddr, mmuflags);
          paddr += RV_MMU_L3_PAGE_SIZE;
          vaddr += RV_MMU_L3_PAGE_SIZE;
        }
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mpfs_kernel_mappings
 *
 * Description:
 *  Setup kernel mappings when usinc CONFIG_BUILD_KERNEL. Sets up the kernel
 *  MMU mappings.
 *
 ****************************************************************************/

void mpfs_kernel_mappings(void)
{
  /* Initialize slab allocator for L3 page tables */

  slab_init(PGT_L3_PBASE);

  /* Begin mapping memory to MMU; note that at this point the MMU is not yet
   * active, so the page table virtual addresses are actually physical
   * addresses and so forth. M-mode does not perform translations anyhow, so
   * this mapping is quite simple to do
   */

  /* Map I/O region, use 2 gigapages for this */

  mmu_ln_map_region(1, PGT_L1_VBASE, MMU_IO_BASE, MMU_IO_BASE,
                    MMU_IO_SIZE, MMU_IO_FLAGS);

  /* Map the kernel text and data */

  map_region(KFLASH_START, KFLASH_START, KFLASH_SIZE, MMU_KTEXT_FLAGS);
  map_region(KSRAM_START, KSRAM_START, KSRAM_SIZE, MMU_KDATA_FLAGS);

  /* Connect the L1 and L2 page tables */

  mmu_ln_setentry(1, PGT_L1_VBASE, PGT_L2_PBASE, KFLASH_START, PTE_G);

  /* Map the page pool */

  mmu_ln_map_region(2, PGT_L2_VBASE, PGPOOL_START, PGPOOL_START, PGPOOL_SIZE,
                    MMU_KDATA_FLAGS);
}
