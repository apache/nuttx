/****************************************************************************
 * arch/risc-v/src/k230/k230_mm_init.c
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

#include "k230_memorymap.h"

#include "riscv_internal.h"
#include "riscv_mmu.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_K230_PBMT_THEAD
/* T-Head Memory Type PTE definitions */

#  define _PAGE_SEC   (1UL << 59)   /* Security */
#  define _PAGE_SHARE (1UL << 60)   /* Shareable */
#  define _PAGE_BUF   (1UL << 61)   /* Bufferable */
#  define _PAGE_CACHE (1UL << 62)   /* Cacheable */
#  define _PAGE_SO    (1UL << 63)   /* Strong Order */

#  define _THEAD_PMA  (_PAGE_SHARE | _PAGE_BUF | _PAGE_CACHE)
#  define _THEAD_IO   (_PAGE_BUF   | _PAGE_SO)
#  define _THEAD_NC   (_PAGE_SHARE | _PAGE_BUF)

#  define K230_DEV_FLAGS  (_THEAD_IO  | MMU_IO_FLAGS)
#  define K230_TXT_FLAGS  (_THEAD_PMA | MMU_KTEXT_FLAGS)
#  define K230_DAT_FLAGS  (_THEAD_PMA | MMU_KDATA_FLAGS)
#  define K230_SHM_FLAGS  (_THEAD_NC  | MMU_KDATA_FLAGS)

#else
/* Svpbmt Memory Type PTE definitions */

#  define _PAGE_NC    (1UL << 61)
#  define _PAGE_IO    (1UL << 62)

#  define SVPBMT_PMA  (0)
#  define SVPBMT_IO   (_PAGE_IO)
#  define SVPBMT_NC   (_PAGE_NC)

#  define K230_DEV_FLAGS   (SVPBMT_IO  | MMU_IO_FLAGS)
#  define K230_SHM_FLAGS   (SVPBMT_NC  | MMU_KDATA_FLAGS)
#  define K230_TXT_FLAGS   (SVPBMT_PMA | MMU_KTEXT_FLAGS)
#  define K230_DAT_FLAGS   (SVPBMT_PMA | MMU_KDATA_FLAGS)
#endif

/* for PTE dump purposes, covers Svpbmt or MAEE w/o bit-59 */

#define MMU_PBMT_MASK     (0xful << 60)
#define MMU_PBMT_VAL(x)   (((x) & MMU_PBMT_MASK) >> 60)

#define MMUFD(x)  (uint16_t)(((x) & 0xff) | (MMU_PBMT_VAL(x) << 8))

/* Map the whole I/O & PLIC memory with vaddr = paddr mappings */

#define MMU_DEV_BASE    (0x80400000ul)      /* KPU config */
#define MMU_DEV_SIZE    (0x11200000ul)      /* 274MB till Hi-sys end */
#define MMU_INT_BASE    (0xF00000000ul)     /* PLIC base */
#define MMU_INT_SIZE    (0x400000ul)        /* 4MB for PLIC */

#ifdef CONFIG_RPTUN
#define MMU_SHM_BASE    (uintptr_t)CONFIG_K230_RPTUN_SHM_ADDR
#define MMU_SHM_SIZE    CONFIG_K230_RPTUN_SHM_SIZE
#endif

#ifndef CONFIG_ARCH_MMU_TYPE_SV39
#error "No valid MMU type defined"
#endif

/* Physical and virtual addresses to page tables (vaddr = paddr mapping)
 * Note NUTTSBI kernel can live in small flash+ram regions thus needs L3.
 * We also assume KFLASH and KSRAM can be held by one L3 table (4MB).
 */

#define PGT_L1_PBASE    (uintptr_t)&m_l1_pgtable
#define PGT_L2_PBDDR    (uintptr_t)&m_l2_pgt_ddr
#define PGT_L2_PBINT    (uintptr_t)&m_l2_pgt_int
#define PGT_L2_PBDEV    (uintptr_t)&m_l2_pgt_dev
#define PGT_L3_PBDDR    (uintptr_t)&m_l3_pgt_ddr

#define PGT_L1_VBASE    PGT_L1_PBASE
#define PGT_L2_VBDDR    PGT_L2_PBDDR
#define PGT_L2_VBINT    PGT_L2_PBINT
#define PGT_L2_VBDEV    PGT_L2_PBDEV
#define PGT_L3_VBDDR    PGT_L3_PBDDR

#define PGT_L1_SIZE     (512)  /* Enough for 512 GiB */
#define PGT_L2_SIZE     (512)  /* Enough for 1 GiB */
#define PGT_L3_SIZE     (1024) /* Enough for 4 MiB */

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Kernel mapping use simple identical mapping (i.e. vaddr==paddr). */

static size_t         m_l1_pgtable[PGT_L1_SIZE] locate_data(".pgtables");
static size_t         m_l2_pgt_dev[PGT_L2_SIZE] locate_data(".pgtables");
static size_t         m_l2_pgt_int[PGT_L2_SIZE] locate_data(".pgtables");
static size_t         m_l2_pgt_ddr[PGT_L2_SIZE] locate_data(".pgtables");
static size_t         m_l3_pgt_ddr[PGT_L3_SIZE] locate_data(".pgtables");

/* Kernel mappings (L1 base) required by riscv_addrenv */

uintptr_t             g_kernel_mappings  = PGT_L1_VBASE;
uintptr_t             g_kernel_pgt_pbase = PGT_L1_PBASE;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: dump_pgtable  dump one pagetable
 ****************************************************************************/

static void dump_pgtable(const size_t * pgt, uint32_t len, const char * name)
{
  minfo("%s at %lx\n", name, (size_t)pgt);
  for (uint32_t i = 0; i < len ; i++)
    {
      uintptr_t pte = (uintptr_t)pgt[i];
      if (pte & PTE_VALID)
        {
          minfo("#%03d paddr:%09lx flags:%03x %s\n", i,
                mmu_pte_to_paddr(pte), MMUFD(pte),
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
  /* Map I/O region in L2 page table. */

  minfo("\nflags: dev=%03x shm=%03x dat=%03x txt=%03x\n",
       MMUFD(K230_DEV_FLAGS | PTE_VALID), MMUFD(K230_SHM_FLAGS | PTE_VALID),
       MMUFD(K230_DAT_FLAGS | PTE_VALID), MMUFD(K230_TXT_FLAGS | PTE_VALID));

  minfo("map DEV L2(%ldMB)\n", MMU_DEV_SIZE >> 20);
  mmu_ln_map_region(2, PGT_L2_VBDEV, MMU_DEV_BASE, MMU_DEV_BASE,
                    MMU_DEV_SIZE, K230_DEV_FLAGS);

  /* Map INT region using L2 page table */

  minfo("map INT L2(%ldMB)\n", MMU_INT_SIZE >> 20);
  mmu_ln_map_region(2, PGT_L2_VBINT, MMU_INT_BASE, MMU_INT_BASE,
                    MMU_INT_SIZE, K230_DEV_FLAGS);

  /* Map kernel area use L3 */

  minfo("map kernel L2/L3(%ldKB)\n", (KFLASH_SIZE + KSRAM_SIZE) >> 10);
  mmu_ln_map_region(3, PGT_L3_VBDDR, KFLASH_START, KFLASH_START,
                   KFLASH_SIZE, K230_TXT_FLAGS);
  mmu_ln_map_region(3, PGT_L3_VBDDR, KSRAM_START, KSRAM_START,
                   KSRAM_SIZE, K230_DAT_FLAGS);

  /* Map the page pool */

  minfo("map pgpool L2(%ldMB)\n", PGPOOL_SIZE >> 20);
  mmu_ln_map_region(2, PGT_L2_VBDDR, PGPOOL_START, PGPOOL_START,
                   PGPOOL_SIZE, K230_DAT_FLAGS);

#ifdef CONFIG_RPTUN
  uintptr_t base;

  /* map shared memory, 2MB aligned in either DDR or DEV/SRAM zone */

  minfo("map shmem L2(%dKB)\n", (MMU_SHM_SIZE) >> 10);
  base = (CONFIG_K230_RPTUN_SHM_ADDR >= 0x80000000) ? PGT_L2_VBDEV
                                                    : PGT_L2_VBDDR;
  mmu_ln_map_region(2, base, MMU_SHM_BASE, MMU_SHM_BASE,
                    MMU_SHM_SIZE, K230_SHM_FLAGS);
#endif

  /* Connect page tables, two tables can only connect once. */

  minfo("connect L1, L2 and L3\n");
  mmu_ln_setentry(1, PGT_L1_VBASE, PGT_L2_PBDDR, KFLASH_START, 0);
  mmu_ln_setentry(1, PGT_L1_VBASE, PGT_L2_PBDEV, MMU_DEV_BASE, 0);
  mmu_ln_setentry(1, PGT_L1_VBASE, PGT_L2_PBINT, MMU_INT_BASE, 0);
  mmu_ln_setentry(2, PGT_L2_VBDDR, PGT_L3_PBDDR, KFLASH_START, 0);

  /* dump page tables */

#ifdef DEBUG_MM
  dump_pgtable(m_l1_pgtable, PGT_L1_SIZE, "L1");
  dump_pgtable(m_l2_pgt_dev, PGT_L2_SIZE, "L2_DEV");
  dump_pgtable(m_l2_pgt_int, PGT_L2_SIZE, "L2_INT");
  dump_pgtable(m_l2_pgt_ddr, PGT_L2_SIZE, "L2_DDR");
  dump_pgtable(m_l3_pgt_ddr, PGT_L3_SIZE, "L3_DDR");
#else
  UNUSED(dump_pgtable);
#endif
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

  minfo("mmu_enable: satp=%lx\n", g_kernel_pgt_pbase);
  mmu_enable(g_kernel_pgt_pbase, 0);
}
