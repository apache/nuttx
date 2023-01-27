/****************************************************************************
 * arch/risc-v/src/mpfs/mpfs_userspace.c
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
#include <assert.h>

#include <nuttx/queue.h>
#include <nuttx/userspace.h>

#include <arch/board/board_memorymap.h>

#include "mpfs_userspace.h"
#include "riscv_internal.h"
#include "riscv_mmu.h"

#ifdef CONFIG_BUILD_PROTECTED

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Physical and virtual addresses to page tables (vaddr = paddr mapping) */

#define PGT_L1_PBASE    (uint64_t)&m_l1_pgtable
#define PGT_L2_PBASE    (uint64_t)&m_l2_pgtable
#define PGT_L3_PBASE    (uint64_t)&m_l3_pgtable
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
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: configure_mpu
 *
 * Description:
 *   This function configures the MPU for for kernel- / userspace separation.
 *   It will also grant access to the page table memory for the supervisor.
 *
 ****************************************************************************/

static void configure_mpu(void);

/****************************************************************************
 * Name: configure_mmu
 *
 * Description:
 *   This function configures the MMU and page tables for kernel- / userspace
 *   separation.
 *
 ****************************************************************************/

static void configure_mmu(void);

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

static void slab_init(uintptr_t start);

/****************************************************************************
 * Name: slab_alloc
 *
 * Description:
 *   Allocate single slab for L3 page table entry
 *
 ****************************************************************************/

static uintptr_t slab_alloc(void);

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
                       uint32_t mmuflags);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* With a 3 level page table setup the total available memory is 512GB.
 * However, this is overkill. A single L3 page table can map 2MB of memory,
 * and for MPFS, this user space is plenty enough. If more memory is needed,
 * simply increase the size of the L3 page table (n * 512), where each 'n'
 * provides 2MB of memory.
 */

/* L1-L3 tables must be in memory always for this to work */

static uint64_t         m_l1_pgtable[PGT_L1_SIZE] locate_data(".pgtables");
static uint64_t         m_l2_pgtable[PGT_L2_SIZE] locate_data(".pgtables");
static uint64_t         m_l3_pgtable[PGT_L3_SIZE] locate_data(".pgtables");

/* L3 page table allocator */

static sq_queue_t       g_free_slabs;
static pgalloc_slab_t   g_slabs[SLAB_COUNT];

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mpfs_userspace
 *
 * Description:
 *   For the case of the separate user-/kernel-space build, perform whatever
 *   platform specific initialization of the user memory is required.
 *   Normally this just means initializing the user space .data and .bss
 *   segments.
 *
 ****************************************************************************/

void mpfs_userspace(void)
{
  uint8_t *src;
  uint8_t *dest;
  uint8_t *end;

  /* Clear all of user-space .bss */

  DEBUGASSERT(USERSPACE->us_bssstart != 0 && USERSPACE->us_bssend != 0 &&
              USERSPACE->us_bssstart <= USERSPACE->us_bssend);

  dest = (uint8_t *)USERSPACE->us_bssstart;
  end  = (uint8_t *)USERSPACE->us_bssend;

  while (dest != end)
    {
      *dest++ = 0;
    }

  /* Initialize all of user-space .data */

  DEBUGASSERT(USERSPACE->us_datasource != 0 &&
              USERSPACE->us_datastart != 0 && USERSPACE->us_dataend != 0 &&
              USERSPACE->us_datastart <= USERSPACE->us_dataend);

  src  = (uint8_t *)USERSPACE->us_datasource;
  dest = (uint8_t *)USERSPACE->us_datastart;
  end  = (uint8_t *)USERSPACE->us_dataend;

  while (dest != end)
    {
      *dest++ = *src++;
    }

  /* Configure MPU / PMP to grant access to the userspace */

  configure_mpu();
  configure_mmu();
}

/****************************************************************************
 * Name: configure_mpu
 *
 * Description:
 *   This function configures the MPU for for kernel- / userspace separation.
 *   It will also grant access to the page table memory for the supervisor.
 *
 ****************************************************************************/

static void configure_mpu(void)
{
  /* Open everything for PMP */

  WRITE_CSR(pmpaddr0, UINT64_C(~0));
  WRITE_CSR(pmpcfg0, (PMPCFG_A_NAPOT | PMPCFG_R | PMPCFG_W | PMPCFG_X));
}

/****************************************************************************
 * Name: configure_mmu
 *
 * Description:
 *   This function configures the MMU and page tables for kernel- / userspace
 *   separation.
 *
 ****************************************************************************/

static void configure_mmu(void)
{
  /* Setup MMU for user. First granule allocator for L3 entries */

  slab_init(PGT_L3_PBASE);

  /* Setup the L3 references for text and data */

  map_region(UFLASH_START, UFLASH_START, UFLASH_SIZE, MMU_UTEXT_FLAGS);
  map_region(USRAM_START, USRAM_START, USRAM_SIZE, MMU_UDATA_FLAGS);

  /* Connect the L1 and L2 page tables */

  mmu_ln_setentry(1, PGT_L1_VBASE, PGT_L2_PBASE, UFLASH_START, PTE_G);

  /* Enable MMU */

  mmu_enable(PGT_L1_PBASE, 0);
}

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

#endif /* CONFIG_BUILD_PROTECTED */
