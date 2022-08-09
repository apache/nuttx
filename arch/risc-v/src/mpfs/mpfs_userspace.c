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
#define PGT_L3_ROMPBASE (uint64_t)&m_l3_romtbl
#define PGT_L3_RAMPBASE (uint64_t)&m_l3_ramtbl
#define PGT_L1_VBASE    PGT_L1_PBASE
#define PGT_L2_VBASE    PGT_L2_PBASE
#define PGT_L3_ROMVBASE PGT_L3_ROMPBASE
#define PGT_L3_RAMVBASE PGT_L3_RAMPBASE

#define PGT_L1_SIZE     (512)  /* Enough to map 512 GiB */
#define PGT_L2_SIZE     (512)  /* Enough to map 1 GiB */
#define PGT_L3_SIZE     (512)  /* Enough to map 2 MiB */

/****************************************************************************
 * Private Functions
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
 * Name: map_region
 *
 * Description:
 *   Map a region of physical memory to the L3 page table
 *
 * Input Parameters:
 *   l3base - L3 page table physical base address
 *   paddr - Beginning of the physical address mapping
 *   vaddr - Beginning of the virtual address mapping
 *   size - Size of the region in bytes
 *   mmuflags - The MMU flags to use in the mapping
 *
 ****************************************************************************/

static void map_region(uintptr_t l3base, uintptr_t paddr, uintptr_t vaddr,
                       size_t size, uint32_t mmuflags);

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

/* Allocate separate tables for ROM/RAM mappings */

static uint64_t         m_l3_romtbl[PGT_L3_SIZE]  locate_data(".pgtables");
static uint64_t         m_l3_ramtbl[PGT_L3_SIZE]  locate_data(".pgtables");

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
  /* Setup MMU for user */

  /* Setup the L3 references for executable memory */

  map_region(PGT_L3_ROMPBASE, UFLASH_START, UFLASH_START, UFLASH_SIZE,
             MMU_UTEXT_FLAGS);

  /* Setup the L3 references for data memory */

  map_region(PGT_L3_RAMPBASE, USRAM_START, USRAM_START, USRAM_SIZE,
             MMU_UDATA_FLAGS);

  /* Connect the L1 and L2 page tables */

  mmu_ln_setentry(1, PGT_L1_VBASE, PGT_L2_PBASE, UFLASH_START, PTE_G);

  /* Enable MMU */

  mmu_enable(PGT_L1_PBASE, 0);
}

/****************************************************************************
 * Name: map_region
 *
 * Description:
 *   Map a region of physical memory to the L3 page table
 *
 * Input Parameters:
 *   l3base - L3 page table physical base address
 *   paddr - Beginning of the physical address mapping
 *   vaddr - Beginning of the virtual address mapping
 *   size - Size of the region in bytes
 *   mmuflags - The MMU flags to use in the mapping
 *
 ****************************************************************************/

static void map_region(uintptr_t l3base, uintptr_t paddr, uintptr_t vaddr,
                       size_t size, uint32_t mmuflags)
{
  uintptr_t end_vaddr;

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

#endif /* CONFIG_BUILD_PROTECTED */
