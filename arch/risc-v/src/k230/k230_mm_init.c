/****************************************************************************
 * arch/risc-v/src/k230/k230_mm_init.c
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

#include "k230_memorymap.h"

#include "riscv_internal.h"
#include "riscv_mmu.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* T-Head C908 MMU flags for I/O Memory, just in case standard Risc-V
 * PTE format is not working.
 */

#define MMU_THEAD_SHAREABLE    (1ul << 60)
#define MMU_THEAD_STRONG_ORDER (1ul << 63)
#define MMU_THEAD_IO_FLAGS     (MMU_IO_FLAGS | MMU_THEAD_SHAREABLE | \
                                MMU_THEAD_STRONG_ORDER)

/* Map the whole I/O & PLIC memory with vaddr = paddr mappings */

#define MMU_IO_BASE     (0x80000000)      /* KPU-Cache */
#define MMU_IO_SIZE     (0x40000000)      /* 1GB, till XIP start */

#define MMU_INT_BASE    (0xF00000000ul)   /* PLIC base */
#define MMU_INT_SIZE    (0x400000ul)      /* 4MB */ 

#ifndef CONFIG_ARCH_MMU_TYPE_SV39
#error "No valid MMU type defined"
#endif

/* Physical and virtual addresses to page tables (vaddr = paddr mapping) */

#define PGT_L1_PBASE    (uintptr_t)&m_l1_pgtable
#define PGT_L2_PBDDR    (uintptr_t)&m_l2_pgt_ddr
#define PGT_L2_PBINT    (uintptr_t)&m_l2_pgt_int
#define PGT_L1_VBASE    PGT_L1_PBASE
#define PGT_L2_VBDDR    PGT_L2_PBDDR
#define PGT_L2_VBINT    PGT_L2_PBINT

#define PGT_L1_SIZE     (512)  /* Enough to map 512 GiB */
#define PGT_L2_SIZE     (512)  /* Enough to map 1 GiB */
#define PGT_L3_SIZE     (1024) /* Enough to map 4 MiB (2MiB x 2) */

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Kernel mappings simply here, mapping is vaddr=paddr */

static size_t         m_l1_pgtable[PGT_L1_SIZE] locate_data(".pgtables");
static size_t         m_l2_pgt_ddr[PGT_L2_SIZE] locate_data(".pgtables");
static size_t         m_l2_pgt_int[PGT_L2_SIZE] locate_data(".pgtables");

/* Kernel mappings (L1 base) required by riscv_addrenv */

uintptr_t               g_kernel_mappings  = PGT_L1_VBASE;
uintptr_t               g_kernel_pgt_pbase = PGT_L1_PBASE;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: dump_pgtable
 *
 * Description:
 *  Dump page tables to console, mainly for debugging purposes.
 *
 ****************************************************************************/

static void dump_pgtable(const size_t * pgt, uint32_t len, const char * name)
{
  sinfo("%s at %lx\n", name, (size_t)pgt);
  for (uint32_t i = 0; i < len ; i++)
    {
      uintptr_t pte = (uintptr_t)pgt[i];
      if (pte & PTE_VALID)
        {
          sinfo("#%03d paddr:%09lx flags:%02x %s\n", i,
                mmu_pte_to_paddr(pte), (unsigned)(pte & 0xff),
                (pte & PTE_LEAF_MASK)? "" : ">>>");
        }
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: k230_kernel_mappings
 *
 * Description:
 *  Setup kernel mappings when using CONFIG_BUILD_KERNEL. Sets up the kernel
 *  MMU mappings.
 *
 ****************************************************************************/

void k230_kernel_mappings(void)
{
  /* Begin mapping memory to MMU; note that at this point the MMU is not yet
   * active, so the page table virtual addresses are actually physical
   * addresses and so forth. M-mode does not perform translations anyhow, so
   * this mapping is quite simple to do
   */

  /* Map I/O region in L1 page table. */

  binfo("map L1 I/O regions(%dGB)\n", MMU_IO_SIZE >> 30);
  mmu_ln_map_region(1, PGT_L1_VBASE, MMU_IO_BASE, MMU_IO_BASE,
                    MMU_IO_SIZE, MMU_IO_FLAGS);

  /* Map INT region using L2 page table */

  binfo("map L2 INT regions(%ldMB)\n", MMU_INT_SIZE >> 20);
  mmu_ln_map_region(2, PGT_L2_VBINT, MMU_INT_BASE, MMU_INT_BASE,
                    MMU_INT_SIZE, MMU_IO_FLAGS);

  /* Map kernel text and data using L2 pages  */

  binfo("map L2 kernel text(%ldMB)\n", KFLASH_SIZE >> 20);
  mmu_ln_map_region(2, PGT_L2_VBDDR, KFLASH_START, KFLASH_START,
                   KFLASH_SIZE, MMU_KTEXT_FLAGS);

  binfo("map L2 kernel data(%ldMB)\n", KSRAM_SIZE >> 20);
  mmu_ln_map_region(2, PGT_L2_VBDDR, KSRAM_START, KSRAM_START,
                   KSRAM_SIZE, MMU_KDATA_FLAGS);

  /* Map the page pool */

  binfo("map L2 pages pool(%ldMB)\n", PGPOOL_SIZE >> 20);
  mmu_ln_map_region(2, PGT_L2_VBDDR, PGPOOL_START, PGPOOL_START,
                   PGPOOL_SIZE, MMU_KDATA_FLAGS);

  /* Connect the L1 and L2 page tables */

  binfo("connect L1 and L2 DDR pgtables\n");
  mmu_ln_setentry(1, PGT_L1_VBASE, PGT_L2_PBDDR, KFLASH_START, 0);

  /* Connect the L1 and L2 page tables for INT regions */

  binfo("connect L1 and L2 INT pgtables\n");
  mmu_ln_setentry(1, PGT_L1_VBASE, PGT_L2_PBINT, MMU_INT_BASE, 0);

  /* dump page tables */

  dump_pgtable(m_l1_pgtable, PGT_L1_SIZE, "L1");
  dump_pgtable(m_l2_pgt_ddr, PGT_L2_SIZE, "L2_DDR");
  dump_pgtable(m_l2_pgt_int, PGT_L2_SIZE, "L2_INT");
}

/****************************************************************************
 * Name: k230_mm_init
 *
 * Description:
 *  Setup kernel mappings when using CONFIG_BUILD_KERNEL. Sets up kernel MMU
 *  mappings. Function also sets the first address environment (satp value).
 *
 ****************************************************************************/

void k230_mm_init(void)
{
  /* Setup the kernel mappings */

  k230_kernel_mappings();

  /* Enable MMU (note: system in S-mode) */

  binfo("mmu_enable: satp=%lx\n", g_kernel_pgt_pbase);
  mmu_enable(g_kernel_pgt_pbase, 0);
}
