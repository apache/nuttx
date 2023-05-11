/****************************************************************************
 * arch/risc-v/src/common/riscv_addrenv_shm.c
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

#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/addrenv.h>
#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/pgalloc.h>

#include <arch/barriers.h>

#include "addrenv.h"
#include "pgalloc.h"
#include "riscv_mmu.h"

#if defined(CONFIG_BUILD_KERNEL) && defined(CONFIG_ARCH_VMA_MAPPING)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_shmat
 *
 * Description:
 *   Attach, i.e, map, on shared memory region to a user virtual address
 *
 * Input Parameters:
 *   pages - A pointer to the first element in a array of physical address,
 *     each corresponding to one page of memory.
 *   npages - The number of pages in the list of physical pages to be mapped.
 *   vaddr - The virtual address corresponding to the beginning of the
 *     (contiguous) virtual address region.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

int up_shmat(uintptr_t *pages, unsigned int npages, uintptr_t vaddr)
{
  struct tcb_s          *tcb     = nxsched_self();
  struct arch_addrenv_s *addrenv = &tcb->addrenv_own->addrenv;

  /* Sanity checks */

  DEBUGASSERT(tcb && tcb->addrenv_own);
  DEBUGASSERT(pages != NULL && npages > 0);
  DEBUGASSERT(vaddr >= CONFIG_ARCH_SHM_VBASE && vaddr < ARCH_SHM_VEND);
  DEBUGASSERT(MM_ISALIGNED(vaddr));

  /* Let riscv_map_pages do the work */

  return riscv_map_pages(addrenv, pages, npages, vaddr, MMU_UDATA_FLAGS);
}

/****************************************************************************
 * Name: up_shmdt
 *
 * Description:
 *   Detach, i.e, unmap, on shared memory region from a user virtual address
 *
 * Input Parameters:
 *   vaddr - The virtual address corresponding to the beginning of the
 *     (contiguous) virtual address region.
 *   npages - The number of pages to be unmapped
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

int up_shmdt(uintptr_t vaddr, unsigned int npages)
{
  struct tcb_s          *tcb     = nxsched_self();
  struct arch_addrenv_s *addrenv = &tcb->addrenv_own->addrenv;

  /* Sanity checks */

  DEBUGASSERT(tcb && tcb->addrenv_own);
  DEBUGASSERT(npages > 0);
  DEBUGASSERT(vaddr >= CONFIG_ARCH_SHM_VBASE && vaddr < ARCH_SHM_VEND);
  DEBUGASSERT(MM_ISALIGNED(vaddr));

  /* Let riscv_unmap_pages do the work */

  return riscv_unmap_pages(addrenv, vaddr, npages);
}

#endif /* CONFIG_BUILD_KERNEL */
