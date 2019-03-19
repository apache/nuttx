/****************************************************************************
 * arch/arm/src/sama5/sam_pgalloc.c
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

void up_allocate_pgheap(FAR void **heap_start, size_t *heap_size)
{
  DEBUGASSERT(heap_start && heap_size);

  *heap_start = (FAR void *)((uintptr_t)SAM_DDRCS_PSECTION +
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

  poolstart = ((uintptr_t)SAM_DDRCS_PSECTION + CONFIG_SAMA5_DDRCS_PGHEAP_OFFSET);
  poolend   = poolstart + CONFIG_SAMA5_DDRCS_PGHEAP_SIZE;

  if (paddr >= poolstart && paddr < poolend)
    {
      return paddr - SAM_DDRCS_PSECTION + SAM_DDRCS_VSECTION;
    }

  return 0;
}
#endif /* !CONFIG_ARCH_PGPOOL_MAPPING */

#endif /* CONFIG_MM_PGALLOC */
