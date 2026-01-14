/****************************************************************************
 * arch/risc-v/src/sg2000/sg2000_mm_init.c
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

#include <stdint.h>
#include <assert.h>
#include <debug.h>

#include <arch/board/board_memorymap.h>

#include "sg2000_memorymap.h"

#include "riscv_internal.h"
#include "riscv_mmu.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* T-Head C906 MMU Extensions */

#define MMU_THEAD_SHAREABLE    (1ul << 60)
#define MMU_THEAD_BUFFERABLE   (1ul << 61)
#define MMU_THEAD_CACHEABLE    (1ul << 62)
#define MMU_THEAD_STRONG_ORDER (1ul << 63)

/* T-Head C906 MMU requires Strong Order and Shareable for I/O Memory */

#define MMU_THEAD_IO_FLAGS     (MMU_IO_FLAGS | MMU_THEAD_SHAREABLE | \
                                MMU_THEAD_STRONG_ORDER)

/* T-Head C906 MMU requires Kernel Memory to be explicitly cached */

#define MMU_THEAD_PMA_FLAGS    (MMU_THEAD_SHAREABLE | \
                                MMU_THEAD_BUFFERABLE | \
                                MMU_THEAD_CACHEABLE)

/* Map the I/O and PLIC Memory with vaddr = paddr mappings */

#define MMU_IO_BASE      (0x00000000ul)
#define MMU_IO_SIZE      (0x40000000ul)

#define MMU_INT_BASE     (0x70000000ul)
#define MMU_INT_SIZE     (0x10000000ul)

/* Physical and virtual addresses to page tables (vaddr = paddr mapping) */

#define PGT_L1_PBASE     (uintptr_t)&m_l1_pgtable
#define PGT_L2_PBASE     (uintptr_t)&m_l2_pgtable
#define PGT_L2_INT_PBASE (uintptr_t)&m_l2_int_pgtable
#define PGT_L3_PBASE     (uintptr_t)&m_l3_pgtable
#define PGT_L1_VBASE     PGT_L1_PBASE
#define PGT_L2_VBASE     PGT_L2_PBASE
#define PGT_L2_INT_VBASE PGT_L2_INT_PBASE
#define PGT_L3_VBASE     PGT_L3_PBASE

#define PGT_L1_SIZE      (512)  /* Enough to map 512 GiB */
#define PGT_L2_SIZE      (512)  /* Enough to map 1 GiB */
#define PGT_L2_INT_SIZE  (512)  /* Enough to map 1 GiB */
#define PGT_L3_SIZE      (1024) /* Enough to map 4 MiB (2MiB x 2) */

#define SLAB_COUNT       (sizeof(m_l3_pgtable) / RV_MMU_PAGE_SIZE)

#define KMM_PAGE_SIZE    RV_MMU_L3_PAGE_SIZE
#define KMM_PBASE        PGT_L3_PBASE
#define KMM_PBASE_IDX    3
#define KMM_SPBASE       PGT_L2_PBASE
#define KMM_SPBASE_IDX   2

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

static size_t m_l1_pgtable[PGT_L1_SIZE] locate_data(".pgtables");
static size_t m_l2_pgtable[PGT_L2_SIZE] locate_data(".pgtables");
static size_t m_l2_int_pgtable[PGT_L2_INT_SIZE] locate_data(".pgtables");
static size_t m_l3_pgtable[PGT_L3_SIZE] locate_data(".pgtables");

/* Kernel mappings (L1 base) */

uintptr_t g_kernel_mappings  = PGT_L1_VBASE;
uintptr_t g_kernel_pgt_pbase = PGT_L1_PBASE;

/* L3 page table allocator */

static sq_queue_t     g_free_slabs;
static pgalloc_slab_t g_slabs[SLAB_COUNT];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: slab_init
 *
 * Description:
 *   Initialize slab allocator for L2 or L3 page table entries.
 *   L2 Page table is used for SV32. L3 is used for SV39.
 *
 * Input Parameters:
 *   start - Beginning of the L2 or L3 page table pool
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
 *   Allocate single slab for L2/L3 page table entry.
 *   L2 Page table is used for SV32. L3 is used for SV39.
 *
 ****************************************************************************/

static uintptr_t slab_alloc(void)
{
  pgalloc_slab_t *slab = (pgalloc_slab_t *)sq_remfirst(&g_free_slabs);
  return slab ? (uintptr_t)slab->memory : 0;
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
                       uint64_t mmuflags)
{
  uintptr_t endaddr;
  uintptr_t pbase;
  int npages;
  int i;
  int j;

  /* How many pages */

  npages = (size + RV_MMU_PAGE_MASK) >> RV_MMU_PAGE_SHIFT;
  endaddr = vaddr + size;

  for (i = 0; i < npages; i += RV_MMU_PAGE_ENTRIES)
    {
      /* See if a mapping exists */

      pbase = mmu_pte_to_paddr(mmu_ln_getentry(KMM_SPBASE_IDX,
                                               KMM_SPBASE, vaddr));
      if (!pbase)
        {
          /* No, allocate 1 page, this must not fail */

          pbase = slab_alloc();
          DEBUGASSERT(pbase);

          /* Map it to the new table */

          mmu_ln_setentry(KMM_SPBASE_IDX, KMM_SPBASE, pbase, vaddr,
                          MMU_UPGT_FLAGS);
        }

      /* Then add the mappings */

      for (j = 0; j < RV_MMU_PAGE_ENTRIES && vaddr < endaddr; j++)
        {
          mmu_ln_setentry(KMM_PBASE_IDX, pbase, paddr, vaddr, mmuflags);
          paddr += KMM_PAGE_SIZE;
          vaddr += KMM_PAGE_SIZE;
        }
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sg2000_kernel_mappings
 *
 * Description:
 *  Setup kernel mappings when using CONFIG_BUILD_KERNEL. Sets up the kernel
 *  MMU mappings.
 *
 ****************************************************************************/

void sg2000_kernel_mappings(void)
{
  /* Initialize slab allocator for the L2/L3 page tables */

  slab_init(KMM_PBASE);

  /* Begin mapping memory to MMU; note that at this point the MMU is not yet
   * active, so the page table virtual addresses are actually physical
   * addresses and so forth.
   */

  /* Map I/O region, use enough large page tables for the IO region. */

  binfo("map I/O regions\n");
  mmu_ln_map_region(1, PGT_L1_VBASE, MMU_IO_BASE, MMU_IO_BASE,
                    MMU_IO_SIZE, MMU_THEAD_IO_FLAGS);

  /* Map the PLIC with L2 page table */

  binfo("map PLIC with L2 page table\n");
  mmu_ln_map_region(2, PGT_L2_INT_PBASE, MMU_INT_BASE, MMU_INT_BASE,
                    MMU_INT_SIZE, MMU_THEAD_IO_FLAGS);

  /* Connect the L1 and PLIC L2 page tables */

  binfo("connect the L1 and PLIC L2 page tables\n");
  mmu_ln_setentry(1, PGT_L1_VBASE, PGT_L2_INT_PBASE, MMU_INT_BASE, PTE_G);

  /* Map the kernel text and data for L2/L3 */

  binfo("map kernel text\n");
  map_region(KFLASH_START, KFLASH_START, KFLASH_SIZE,
             MMU_KTEXT_FLAGS | MMU_THEAD_PMA_FLAGS);

  binfo("map kernel data\n");
  map_region(KSRAM_START, KSRAM_START, KSRAM_SIZE,
             MMU_KDATA_FLAGS | MMU_THEAD_PMA_FLAGS);

  /* Connect the L1 and L2 page tables for the kernel text and data */

  binfo("connect the L1 and L2 page tables\n");
  mmu_ln_setentry(1, PGT_L1_VBASE, PGT_L2_PBASE, KFLASH_START, PTE_G);

  /* Map the page pool */

  binfo("map the page pool\n");
  mmu_ln_map_region(2, PGT_L2_VBASE, PGPOOL_START, PGPOOL_START, PGPOOL_SIZE,
                    MMU_KDATA_FLAGS | MMU_THEAD_PMA_FLAGS);
}

/****************************************************************************
 * Name: sg2000_mm_init
 *
 * Description:
 *  Setup kernel mappings when using CONFIG_BUILD_KERNEL. Sets up kernel MMU
 *  mappings. Function also sets the first address environment (satp value).
 *
 ****************************************************************************/

void sg2000_mm_init(void)
{
  /* Setup the kernel mappings */

  sg2000_kernel_mappings();

  /* Enable MMU */

  binfo("mmu_enable: satp=%" PRIuPTR "\n", g_kernel_pgt_pbase);
  mmu_enable(g_kernel_pgt_pbase, 0);
}

/****************************************************************************
 * Name: mmu_flush_cache
 *
 * Description:
 *   Flush the MMU Cache for T-Head C906.  Called by mmu_write_satp() after
 *   updating the MMU SATP Register, when swapping MMU Page Tables.
 *   This operation executes RISC-V Instructions that are specific to
 *   T-Head C906.
 *
 ****************************************************************************/

void weak_function mmu_flush_cache(uintptr_t reg)
{
  UNUSED(reg);
  __asm__ __volatile__
    (

      /* DCACHE.IALL: Invalidate all Page Table Entries in the D-Cache */

      ".long 0x0020000b\n"

      /* SYNC.S: Ensure that all Cache Operations are completed */

      ".long 0x0190000b\n"
    );
}
