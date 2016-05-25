/****************************************************************************
 * arch/arm/src/armv7-a/arm_phypgaddr.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/addrenv.h>
#include <nuttx/pgalloc.h>

#include "chip.h"
#include "mmu.h"
#include "cache.h"

#include "pgalloc.h"

#ifdef CONFIG_MM_PGALLOC

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

uintptr_t arm_physpgaddr(uintptr_t vaddr)
{
  FAR uint32_t *l2table;
  uintptr_t paddr;
  uint32_t l1entry;
#ifndef CONFIG_ARCH_PGPOOL_MAPPING
  uint32_t l1save;
#endif
  int index;

  /* Check if this address is within the range of one of the virtualized user
   * address regions.
   */

  if (arm_uservaddr(vaddr))
    {
      /* Yes.. Get Level 1 page table entry corresponding to this virtual
       * address.
       */

      l1entry = mmu_l1_getentry(vaddr);
      if ((l1entry & PMD_TYPE_MASK) == PMD_TYPE_PTE)
        {
          /* Get the physical address of the level 2 page table from the
           * level 1 page table entry.
           */

          paddr = ((uintptr_t)l1entry & PMD_PTE_PADDR_MASK);

#ifdef CONFIG_ARCH_PGPOOL_MAPPING
          /* Get the virtual address of the base of level 2 page table */

          l2table = (FAR uint32_t *)arm_pgvaddr(paddr);
#else
          /* Temporarily map the page into the virtual address space */

          l1save = mmu_l1_getentry(ARCH_SCRATCH_VBASE);
          mmu_l1_setentry(paddr & ~SECTION_MASK, ARCH_SCRATCH_VBASE, MMU_MEMFLAGS);
          l2table = (FAR uint32_t *)(ARCH_SCRATCH_VBASE | (paddr & SECTION_MASK));
#endif
          if (l2table)
            {
              /* Invalidate D-Cache line containing this virtual address so that
               * we re-read from physical memory
               */

              index = (vaddr & SECTION_MASK) >> MM_PGSHIFT;
              arch_invalidate_dcache((uintptr_t)&l2table[index],
                                     (uintptr_t)&l2table[index] + sizeof(uint32_t));

              /* Get the Level 2 page table entry corresponding to this virtual
               * address.  Extract the physical address of the page containing
               * the mapping of the virtual address.
               */

              paddr = ((uintptr_t)l2table[index] & PTE_SMALL_PADDR_MASK);

#ifndef CONFIG_ARCH_PGPOOL_MAPPING
              /* Restore the scratch section L1 page table entry */

              mmu_l1_restore(ARCH_SCRATCH_VBASE, l1save);
#endif

              /* Add the correct offset and return the physical address
               * corresponding to the virtual address.
               */

              return paddr + (vaddr & MM_PGMASK);
            }
        }
    }

  /* No mapping available */

  return 0;
}

#endif /* CONFIG_MM_PGALLOC */
