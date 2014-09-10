/****************************************************************************
 * arch/arm/src/armv7/pginline.h
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

#ifndef __ARCH_ARM_SRC_ARMV7_A_PGINLINE_H
#define __ARCH_ARM_SRC_ARMV7_A_PGINLINE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <nuttx/addrenv.h>

#include "mmu.h"

#if defined(CONFIG_MM_PGALLOC) && defined(CONFIG_ARCH_USE_MMU)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_pgmap
 *
 * Description:
 *   Map one page to a temporary, scratch virtual memory address
 *
 ****************************************************************************/

#ifndef CONFIG_ARCH_PGPOOL_MAPPING
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

#ifndef CONFIG_ARCH_PGPOOL_MAPPING
static inline void arm_tmprestore(uint32_t l1save)
{
  mmu_l1_restore(ARCH_SCRATCH_VBASE, l1save);
}
#endif

/****************************************************************************
 * Name: arm_pgvaddr
 *
 * Description:
 *   If the page memory pool is staticly mapped, then we do not have to
 *   go through the the temporary mapping.  We simply have to perform a
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

#endif /* CONFIG_MM_PGALLOC && CONFIG_ARCH_USE_MMU */
#endif /* __ARCH_ARM_SRC_ARMV7_A_PGINLINE_H */
