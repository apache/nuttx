/****************************************************************************
 * arch/arm/src/armv7-a/pgalloc.h
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

#ifndef __ARCH_ARM_SRC_ARMV7_A_PGALLOC_H
#define __ARCH_ARM_SRC_ARMV7_A_PGALLOC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <assert.h>

#include <nuttx/addrenv.h>

#include "mmu.h"

#ifdef CONFIG_MM_PGALLOC

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_ARCH_PGPOOL_MAPPING
#  error "ARMv7-A needs CONFIG_ARCH_PGPOOL_MAPPING"
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_pgvaddr
 *
 * Description:
 *   If the page memory pool is statically mapped, then we do not have to
 *   go through the temporary mapping.  We simply have to perform a
 *   physical to virtual memory address mapping.
 *
 ****************************************************************************/

static inline uintptr_t arm_pgvaddr(uintptr_t paddr)
{
  DEBUGASSERT(paddr >= CONFIG_ARCH_PGPOOL_PBASE &&
              paddr < CONFIG_ARCH_PGPOOL_PEND);

  return paddr - CONFIG_ARCH_PGPOOL_PBASE + CONFIG_ARCH_PGPOOL_VBASE;
}

/****************************************************************************
 * Name: arm_uservaddr
 *
 * Description:
 *   Return true if the virtual address, vaddr, lies in the user address
 *   space.
 *
 ****************************************************************************/

static inline bool arm_uservaddr(uintptr_t vaddr)
{
  /* Check if this address is within the range of the virtualized .bss/.data,
   * heap, or stack regions.
   */

  return ((vaddr >= CONFIG_ARCH_TEXT_VBASE && vaddr < ARCH_TEXT_VEND) ||
          (vaddr >= CONFIG_ARCH_DATA_VBASE && vaddr < ARCH_DATA_VEND) ||
          (vaddr >= CONFIG_ARCH_HEAP_VBASE && vaddr < ARCH_HEAP_VEND)
#ifdef CONFIG_ARCH_STACK_DYNAMIC
       || (vaddr >= CONFIG_ARCH_STACK_VBASE && vaddr < ARCH_STACK_VEND)
#endif
#ifdef CONFIG_MM_SHM
       || (vaddr >= CONFIG_ARCH_SHM_VBASE && vaddr < ARCH_SHM_VEND)
#endif
      );
}

/****************************************************************************
 * Name: set_l2_entry
 *
 * Description:
 *   Set the L2 table entry as part of the initialization of the L2 Page
 *   table.
 *
 ****************************************************************************/

static inline void set_l2_entry(uint32_t *l2table, uintptr_t paddr,
                                uintptr_t vaddr, uint32_t mmuflags)
{
  uint32_t index;

  /* The table divides a 1Mb address space up into 256 entries, each
   * corresponding to 4Kb of address space.  The page table index is
   * related to the offset from the beginning of 1Mb region.
   */

  index = (vaddr & 0x000ff000) >> 12;

  /* Save the level 2 page table entry */

  l2table[index] = (paddr | mmuflags);
}

/****************************************************************************
 * Name: clr_l2_entry
 *
 * Description:
 *   Claear the L2 table entry.
 *
 ****************************************************************************/

static inline void clr_l2_entry(uint32_t *l2table, uintptr_t vaddr)
{
  uint32_t index;

  /* The table divides a 1Mb address space up into 256 entries, each
   * corresponding to 4Kb of address space.  The page table index is
   * related to the offset from the beginning of 1Mb region.
   */

  index = (vaddr & 0x000ff000) >> 12;

  /* Save the level 2 page table entry */

  l2table[index] = 0;
}

/****************************************************************************
 * Name: get_l2_entry
 *
 * Description:
 *   Set the L2 table entry as part of the initialization of the L2 Page
 *   table.
 *
 ****************************************************************************/

static inline uintptr_t get_l2_entry(uint32_t *l2table, uintptr_t vaddr)
{
  uint32_t index;

  /* The table divides a 1Mb address space up into 256 entries, each
   * corresponding to 4Kb of address space.  The page table index is
   * related to the offset from the beginning of 1Mb region.
   */

  index = (vaddr & 0x000ff000) >> 12;

  /* Return the level 2 page table entry */

  return l2table[index];
}

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: arm_physpgaddr
 *
 * Description:
 *   Check if the virtual address lies in the user data area and, if so
 *   get the mapping to the physical address in the page pool.
 *
 ****************************************************************************/

uintptr_t arm_physpgaddr(uintptr_t vaddr);

/****************************************************************************
 * Name: arm_virtpgaddr
 *
 * Description:
 *   Check if the physical address lies in the page pool and, if so
 *   get the mapping to the virtual address in the user data area.
 *
 ****************************************************************************/

uintptr_t arm_virtpgaddr(uintptr_t paddr);

#endif /* CONFIG_MM_PGALLOC */
#endif /* __ARCH_ARM_SRC_ARMV7_A_PGALLOC_H */
