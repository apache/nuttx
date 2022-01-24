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

#include "mpfs_userspace.h"
#include "riscv_internal.h"
#include "riscv_mmu.h"

#ifdef CONFIG_BUILD_PROTECTED

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define PMP_UFLASH_FLAGS    (PMPCFG_A_NAPOT | PMPCFG_X | PMPCFG_R)
#define PMP_USRAM_FLAGS     (PMPCFG_A_NAPOT | PMPCFG_W | PMPCFG_R)

#define UFLASH_START        (uintptr_t)&__uflash_start
#define UFLASH_SIZE         (uintptr_t)&__uflash_size
#define USRAM_START         (uintptr_t)&__usram_start
#define USRAM_SIZE          (uintptr_t)&__usram_size

/* Physical and virtual addresses to page tables (vaddr = paddr mapping) */

#define PGT_L1_PBASE        (uint64_t)&m_l1_pgtable
#define PGT_L2_PBASE        (uint64_t)&m_l2_pgtable
#define PGT_L3_PBASE        (uint64_t)&m_l3_pgtable
#define PGT_L1_VBASE        PGT_L1_PBASE
#define PGT_L2_VBASE        PGT_L2_PBASE
#define PGT_L3_VBASE        PGT_L3_PBASE

/* Flags for user FLASH (RX) and user RAM (RW) */

#define MMU_UFLASH_FLAGS    (PTE_R | PTE_X | PTE_U | PTE_G)
#define MMU_USRAM_FLAGS     (PTE_R | PTE_W | PTE_U | PTE_G)

/* Kernel RAM needs to be opened (the page tables) */

#define KSRAM_START         (uintptr_t)&__ksram_start
#define KSRAM_SIZE          (uintptr_t)&__ksram_size

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
 * Private Data
 ****************************************************************************/

/* With a 3 level page table setup the total available memory is 512GB.
 * However, this is overkill. A single L3 page table can map 2MB of memory,
 * and for MPFS, this user space is plenty enough. If more memory is needed,
 * simply increase the size of the L3 page table (n * 512), where each 'n'
 * provides 2MB of memory.
 */

/* L1-L3 tables must be in memory always for this to work */

static uint64_t             m_l1_pgtable[512] locate_data(".pgtables");
static uint64_t             m_l2_pgtable[512] locate_data(".pgtables");
static uint64_t             m_l3_pgtable[512] locate_data(".pgtables");

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern uintptr_t            __uflash_start;
extern uintptr_t            __uflash_size;
extern uintptr_t            __usram_start;
extern uintptr_t            __usram_size;

/* Needed to allow access to the page tables, which reside in kernel RAM */

extern uintptr_t            __ksram_start;
extern uintptr_t            __ksram_size;

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
  /* Configure the PMP to permit user-space access to its ROM and RAM.
   *
   * Note: PMP by default revokes access, thus if different privilege modes
   * are in use, the user space _must_ be granted access here, otherwise
   * an exception will fire when the user space task is started.
   *
   * Note: according to the Polarfire reference manual, address bits [1:0]
   * are not considered (due to 4 octet alignment), so strictly they don't
   * have to be cleared here.
   *
   * Note: do not trust the stext / etc sections to be correctly aligned
   * here, they should be but it is simpler and safer to handle the user
   * region as a whole
   *
   * Access is currently granted by simply adding each userspace memory area
   * to PMP, without further granularity.
   *
   * "RX" for the user progmem
   * "RW" for the user RAM area
   *
   */

  int ret;
  int idx;

  /* First, test access to user flash */

  ret = riscv_check_pmp_access(PMP_UFLASH_FLAGS, UFLASH_START, UFLASH_SIZE);

  /* No access or partial access means we must crash */

  DEBUGASSERT(ret != PMP_ACCESS_DENIED);

  if (ret == PMP_ACCESS_OFF)
    {
      idx = riscv_next_free_pmp_region();
      DEBUGASSERT(idx >= 0);
      riscv_config_pmp_region(idx, PMP_UFLASH_FLAGS, UFLASH_START,
                              UFLASH_SIZE);
    }

  /* Then, test access to user RAM */

  ret = riscv_check_pmp_access(PMP_USRAM_FLAGS, USRAM_START, USRAM_SIZE);
  DEBUGASSERT(ret != PMP_ACCESS_DENIED);
  if (ret == PMP_ACCESS_OFF)
    {
      idx = riscv_next_free_pmp_region();
      DEBUGASSERT(idx >= 0);
      riscv_config_pmp_region(idx, PMP_USRAM_FLAGS, USRAM_START, USRAM_SIZE);
    }

  /* The supervisor must have access to the page tables */

  ret = riscv_check_pmp_access(PMP_USRAM_FLAGS, KSRAM_START, KSRAM_SIZE);
  DEBUGASSERT(ret != PMP_ACCESS_DENIED);
  if (ret == PMP_ACCESS_OFF)
    {
      idx = riscv_next_free_pmp_region();
      DEBUGASSERT(idx >= 0);
      riscv_config_pmp_region(idx, PMP_USRAM_FLAGS, KSRAM_START, KSRAM_SIZE);
    }
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

  mmu_ln_map_region(3, PGT_L3_VBASE, UFLASH_START, UFLASH_START,
                    UFLASH_SIZE, MMU_UFLASH_FLAGS);

  /* Setup the L3 references for data memory */

  mmu_ln_map_region(3, PGT_L3_VBASE, USRAM_START, USRAM_START,
                    USRAM_SIZE, MMU_USRAM_FLAGS);

  /* Setup the L2 and L1 references */

  mmu_ln_setentry(2, PGT_L2_VBASE, PGT_L3_PBASE, PGT_L3_VBASE, PTE_G);
  mmu_ln_setentry(1, PGT_L1_VBASE, PGT_L2_PBASE, PGT_L2_VBASE, PTE_G);

  /* Enable MMU */

  mmu_enable(PGT_L1_PBASE, 0);
}

#endif /* CONFIG_BUILD_PROTECTED */
