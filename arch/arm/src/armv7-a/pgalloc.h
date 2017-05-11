/****************************************************************************
 * arch/arm/src/armv7-a/pgalloc.h
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_pgmap
 *
 * Description:
 *   Map one page to a temporary, scratch virtual memory address
 *
 ****************************************************************************/

#if !defined(CONFIG_ARCH_PGPOOL_MAPPING) && defined(CONFIG_ARCH_USE_MMU)
static inline uintptr_t arm_tmpmap(uintptr_t paddr, FAR uint32_t *l1save)
{
  *l1save = mmu_l1_getentry(ARCH_SCRATCH_VBASE);
  mmu_l1_setentry(paddr & ~SECTION_MASK, ARCH_SCRATCH_VBASE, MMU_MEMFLAGS);
  return ((uintptr_t)ARCH_SCRATCH_VBASE | (paddr & SECTION_MASK));
}
#endif

/****************************************************************************
 * Name: arm_pgrestore
 *
 * Description:
 *  Restore any previous L1 page table mapping that was in place when
 *  arm_tmpmap() was called
 *
 ****************************************************************************/

#if !defined(CONFIG_ARCH_PGPOOL_MAPPING) && defined(CONFIG_ARCH_USE_MMU)
static inline void arm_tmprestore(uint32_t l1save)
{
  mmu_l1_restore(ARCH_SCRATCH_VBASE, l1save);
}
#endif

/****************************************************************************
 * Name: arm_pgvaddr
 *
 * Description:
 *   If the page memory pool is statically mapped, then we do not have to
 *   go through the temporary mapping.  We simply have to perform a
 *   physical to virtual memory address mapping.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_PGPOOL_MAPPING
static inline uintptr_t arm_pgvaddr(uintptr_t paddr)
{
  DEBUGASSERT(paddr >= CONFIG_ARCH_PGPOOL_PBASE &&
              paddr < CONFIG_ARCH_PGPOOL_PEND);

  return paddr - CONFIG_ARCH_PGPOOL_PBASE + CONFIG_ARCH_PGPOOL_VBASE;
}
#endif

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

static inline void set_l2_entry(FAR uint32_t *l2table, uintptr_t paddr,
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

static inline void clr_l2_entry(FAR uint32_t *l2table, uintptr_t vaddr)
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

static inline uintptr_t get_l2_entry(FAR uint32_t *l2table, uintptr_t vaddr)
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
 * Public Functions
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

#ifdef CONFIG_ARCH_PGPOOL_MAPPING
uintptr_t arm_virtpgaddr(uintptr_t paddr);
#endif

#endif /* CONFIG_MM_PGALLOC */
#endif /* __ARCH_ARM_SRC_ARMV7_A_PGALLOC_H */
