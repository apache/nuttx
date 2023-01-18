/****************************************************************************
 * arch/risc-v/src/common/riscv_mmu.c
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

#include <assert.h>

#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include "riscv_internal.h"
#include "riscv_mmu.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_ARCH_MMU_TYPE_SV39
static const size_t g_pgt_sizes[] =
{
    RV_MMU_L1_PAGE_SIZE, RV_MMU_L2_PAGE_SIZE, RV_MMU_L3_PAGE_SIZE
};
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void mmu_ln_setentry(uint32_t ptlevel, uintptr_t lnvaddr, uintptr_t paddr,
                     uintptr_t vaddr, uint32_t mmuflags)
{
  uintptr_t *lntable = (uintptr_t *)lnvaddr;
  uint32_t   index;

  DEBUGASSERT(ptlevel > 0 && ptlevel <= RV_MMU_PT_LEVELS);

  /* Test if this is a leaf PTE, if it is, set A+D even if they are not used
   * by the implementation.
   *
   * If not, clear A+D+U because the spec. says:
   * For non-leaf PTEs, the D, A, and U bits are reserved for future use and
   * must be cleared by software for forward compatibility.
   */

  if (mmuflags & PTE_LEAF_MASK)
    {
      mmuflags |= (PTE_A | PTE_D);
    }
  else
    {
      mmuflags &= ~(PTE_A | PTE_D | PTE_U);
    }

  /* Make sure the entry is valid */

  mmuflags |= PTE_VALID;

  /* Calculate index for lntable */

  index = (vaddr >> RV_MMU_VADDR_SHIFT(ptlevel)) & RV_MMU_VPN_MASK;

  /* Move PPN to correct position */

  paddr >>= RV_MMU_PTE_PPN_SHIFT;

  /* Save it */

  lntable[index] = (paddr | mmuflags);

  /* Update with memory by flushing the cache(s) */

  mmu_invalidate_tlb_by_vaddr(vaddr);
}

uintptr_t mmu_ln_getentry(uint32_t ptlevel, uintptr_t lnvaddr,
                          uintptr_t vaddr)
{
  uintptr_t *lntable = (uintptr_t *)lnvaddr;
  uint32_t  index;

  DEBUGASSERT(ptlevel > 0 && ptlevel <= RV_MMU_PT_LEVELS);

  index = (vaddr >> RV_MMU_VADDR_SHIFT(ptlevel)) & RV_MMU_VPN_MASK;

  return lntable[index];
}

void mmu_ln_restore(uint32_t ptlevel, uintptr_t lnvaddr, uintptr_t vaddr,
                    uintptr_t entry)
{
  uintptr_t *lntable = (uintptr_t *)lnvaddr;
  uint32_t  index;

  DEBUGASSERT(ptlevel > 0 && ptlevel <= RV_MMU_PT_LEVELS);

  index = (vaddr >> RV_MMU_VADDR_SHIFT(ptlevel)) & RV_MMU_VPN_MASK;

  lntable[index] = entry;

  /* Update with memory by flushing the cache(s) */

  mmu_invalidate_tlb_by_vaddr(vaddr);
}

void mmu_ln_map_region(uint32_t ptlevel, uintptr_t lnvaddr, uintptr_t paddr,
                       uintptr_t vaddr, size_t size, uint32_t mmuflags)
{
  uintptr_t end_paddr = paddr + size;
  size_t    page_size = g_pgt_sizes[ptlevel - 1];

  DEBUGASSERT(ptlevel > 0 && ptlevel <= RV_MMU_PT_LEVELS);

  while (paddr < end_paddr)
    {
      mmu_ln_setentry(ptlevel, lnvaddr, paddr, vaddr, mmuflags);
      paddr += page_size;
      vaddr += page_size;
    }
}

size_t mmu_get_region_size(uint32_t ptlevel)
{
  DEBUGASSERT(ptlevel > 0 && ptlevel <= RV_MMU_PT_LEVELS);

  return g_pgt_sizes[ptlevel - 1];
}
