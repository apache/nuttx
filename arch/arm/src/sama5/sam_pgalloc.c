/****************************************************************************
 * arch/arm/src/sama5/sam_pgalloc.c
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

#include <sys/types.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/addrenv.h>
#include <nuttx/pgalloc.h>

#include "chip.h"
#include "mmu.h"

#include "sam_pgalloc.h"

#ifdef CONFIG_MM_PGALLOC

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Currently, page cache memory must be allocated in DRAM.  There are other
 * possibilities, but the logic in this file will have to extended in order
 * handle any other possibility.
 */

#ifndef CONFIG_SAMA5_DDRCS_PGHEAP
#  error CONFIG_SAMA5_DDRCS_PGHEAP must be selected
#endif

#ifndef CONFIG_SAMA5_DDRCS_PGHEAP_OFFSET
#  error CONFIG_SAMA5_DDRCS_PGHEAP_OFFSET must be specified
#endif

#if (CONFIG_SAMA5_DDRCS_PGHEAP_OFFSET & MM_PGMASK) != 0
#  warning CONFIG_SAMA5_DDRCS_PGHEAP_OFFSET is not aligned to a page boundary
#endif

#ifndef CONFIG_SAMA5_DDRCS_PGHEAP_SIZE
#  error CONFIG_SAMA5_DDRCS_PGHEAP_SIZE must be specified
#endif

#if (CONFIG_SAMA5_DDRCS_PGHEAP_SIZE & MM_PGMASK) != 0
#  warning CONFIG_SAMA5_DDRCS_PGHEAP_SIZE is not aligned to a page boundary
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_allocate_pgheap
 *
 * Description:
 *   If there is a page allocator in the configuration, then this function
 *   must be provided by the platform-specific code.  The OS initialization
 *   logic will call this function early in the initialization sequence to
 *   get the page heap information needed to configure the page allocator.
 *
 ****************************************************************************/

void up_allocate_pgheap(void **heap_start, size_t *heap_size)
{
  DEBUGASSERT(heap_start && heap_size);

  *heap_start = (void *)((uintptr_t)SAM_DDRCS_PSECTION +
                             CONFIG_SAMA5_DDRCS_PGHEAP_OFFSET);
  *heap_size  = CONFIG_SAMA5_DDRCS_PGHEAP_SIZE;
}

/****************************************************************************
 * Name: sam_virtpgaddr
 *
 * Description:
 *   Check if the physical address lies in the page pool and, if so
 *   get the mapping to the virtual address in the user data area.
 *
 ****************************************************************************/

#ifndef CONFIG_ARCH_PGPOOL_MAPPING
uintptr_t sam_virtpgaddr(uintptr_t paddr)
{
  uintptr_t poolstart;
  uintptr_t poolend;

  /* REVISIT: Not implemented correctly.  The reverse lookup from physical
   * to virtual.  This will return a kernel accessible virtual address, but
   * not an address usable by the user code.
   *
   * The correct solutions is complex and, perhaps, will never be needed.
   */

  poolstart = ((uintptr_t)SAM_DDRCS_PSECTION +
                CONFIG_SAMA5_DDRCS_PGHEAP_OFFSET);
  poolend   = poolstart + CONFIG_SAMA5_DDRCS_PGHEAP_SIZE;

  if (paddr >= poolstart && paddr < poolend)
    {
      return paddr - SAM_DDRCS_PSECTION + SAM_DDRCS_VSECTION;
    }

  return 0;
}
#endif /* !CONFIG_ARCH_PGPOOL_MAPPING */

#endif /* CONFIG_MM_PGALLOC */
