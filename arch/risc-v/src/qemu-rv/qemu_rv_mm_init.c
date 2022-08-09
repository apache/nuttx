/****************************************************************************
 * arch/risc-v/src/qemu-rv/qemu_rv_mm_init.c
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

#include "qemu_rv_memorymap.h"

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
#define PGT_L3_SIZE     (1024) /* Enough to map 4 MiB (2MiB x 2) */

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

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void map_region(uintptr_t paddr, uintptr_t vaddr, size_t size,
                       uint32_t mmuflags)
{
  uintptr_t offset;
  uintptr_t l3base;
  uintptr_t end_vaddr;

  /* Start offset for the L3 table, kernel flash is always first */

  offset = ((paddr - KFLASH_START) / RV_MMU_L2_PAGE_SIZE) * RV_MMU_PAGE_SIZE;

  /* L3 base address per 2MiB boundary */

  l3base = PGT_L3_PBASE + offset;

  /* Map the region to the L3 table as a whole */

  mmu_ln_map_region(3, l3base, paddr, vaddr, size, mmuflags);

  /* Connect to L2 table */

  end_vaddr = vaddr + size;
  while (vaddr < end_vaddr)
    {
      mmu_ln_setentry(2, PGT_L2_VBASE, l3base, vaddr, PTE_G);
      l3base += RV_MMU_L3_PAGE_SIZE;
      vaddr += RV_MMU_L2_PAGE_SIZE;
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: qemu_rv_kernel_mappings
 *
 * Description:
 *  Setup kernel mappings when usinc CONFIG_BUILD_KERNEL. Sets up the kernel
 *  MMU mappings.
 *
 ****************************************************************************/

void qemu_rv_kernel_mappings(void)
{
  /* Begin mapping memory to MMU; note that at this point the MMU is not yet
   * active, so the page table virtual addresses are actually physical
   * addresses and so forth. M-mode does not perform translations anyhow, so
   * this mapping is quite simple to do
   */

  /* Map I/O region, use 2 L1 entries (i.e. 2 * 1GB address space) */

  binfo("map I/O regions\n");
  mmu_ln_map_region(1, PGT_L1_VBASE, MMU_IO_BASE, MMU_IO_BASE,
                    MMU_IO_SIZE, MMU_IO_FLAGS);

  /* Map the kernel text and data for L2/L3 */

  binfo("map kernel text\n");
  map_region(KFLASH_START, KFLASH_START, KFLASH_SIZE, MMU_KTEXT_FLAGS);

  binfo("map kernel data\n");
  map_region(KSRAM_START, KSRAM_START, KSRAM_SIZE, MMU_KDATA_FLAGS);

  /* Connect the L1 and L2 page tables for the kernel text and data */

  binfo("connect the L1 and L2 page tables\n");
  mmu_ln_setentry(1, PGT_L1_VBASE, PGT_L2_PBASE, KFLASH_START, PTE_G);

  /* Map the page pool */

  binfo("map the page pool\n");
  mmu_ln_map_region(2, PGT_L2_VBASE, PGPOOL_START, PGPOOL_START, PGPOOL_SIZE,
                    MMU_KDATA_FLAGS);
}

/****************************************************************************
 * Name: qemu_rv_mm_init
 *
 * Description:
 *  Setup kernel mappings when using CONFIG_BUILD_KERNEL. Sets up kernel MMU
 *  mappings. Function also sets the first address environment (satp value).
 *
 ****************************************************************************/

void qemu_rv_mm_init(void)
{
  /* Setup the kernel mappings */

  qemu_rv_kernel_mappings();

  /* Enable MMU (note: system is still in M-mode) */

  binfo("mmu_enable: satp=%lx\n", g_kernel_pgt_pbase);
  mmu_enable(g_kernel_pgt_pbase, 0);
}
