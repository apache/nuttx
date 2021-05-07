/****************************************************************************
 * mm/mm_gran/mm_pgalloc.c
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

#include <assert.h>

#include <nuttx/mm/gran.h>
#include <nuttx/pgalloc.h>

#include "mm_gran/mm_gran.h"

#ifdef CONFIG_MM_PGALLOC

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* CONFIG_MM_PGALLOC - Enable page allocator support
 * CONFIG_MM_PGSIZE - The page size.  Must be one of {1024, 2048,
 *   4096, 8192, or 16384}.  This is easily extensible, but only those
 *   values are currently support.
 * CONFIG_DEBUG_PGALLOC - Just like CONFIG_DEBUG_MM, but only generates
 *   output from the page allocation logic.
 *
 * Dependencies:  CONFIG_ARCH_USE_MMU and CONFIG_GRAN
 */

/* Debug */

#ifdef CONFIG_DEBUG_PGALLOC
#  define pgaerr                    _err
#  define pgawarn                   _warn
#  define pgainfo                   _info
#else
#  define pgaerr                    merr
#  define pgawarn                   mwarn
#  define pgainfo                   minfo
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The state of the page allocator */

static GRAN_HANDLE g_pgalloc;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mm_pginitialize
 *
 * Description:
 *   Initialize the page allocator.
 *
 * Input Parameters:
 *   heap_start - The physical address of the start of memory region that
 *                will be used for the page allocator heap
 *   heap_size  - The size (in bytes) of the memory region that will be used
 *                for the page allocator heap.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void mm_pginitialize(FAR void *heap_start, size_t heap_size)
{
  g_pgalloc = gran_initialize(heap_start, heap_size, MM_PGSHIFT, MM_PGSHIFT);
  DEBUGASSERT(g_pgalloc != NULL);
}

/****************************************************************************
 * Name: mm_pgreserve
 *
 * Description:
 *   Reserve memory in the page memory pool.  This will reserve the pages
 *   that contain the start and end addresses plus all of the pages
 *   in between.  This should be done early in the initialization sequence
 *   before any other allocations are made.
 *
 *   Reserved memory can never be allocated (it can be freed however which
 *   essentially unreserves the memory).
 *
 * Input Parameters:
 *   start  - The address of the beginning of the region to be reserved.
 *   size   - The size of the region to be reserved
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void mm_pgreserve(uintptr_t start, size_t size)
{
  gran_reserve(g_pgalloc, start, size);
}

/****************************************************************************
 * Name: mm_pgalloc
 *
 * Description:
 *   Allocate page memory from the page memory pool.
 *
 * Input Parameters:
 *   npages - The number of pages to allocate, each of size CONFIG_MM_PGSIZE.
 *
 * Returned Value:
 *   On success, a non-zero, physical address of the allocated page memory
 *   is returned.  Zero is returned on failure.  NOTE:  This is an unmapped
 *   physical address and cannot be used until it is appropriately mapped.
 *
 ****************************************************************************/

uintptr_t mm_pgalloc(unsigned int npages)
{
  return (uintptr_t)gran_alloc(g_pgalloc, (size_t)npages << MM_PGSHIFT);
}

/****************************************************************************
 * Name: mm_pgfree
 *
 * Description:
 *   Return page memory to the page memory pool.
 *
 * Input Parameters:
 *   paddr  - A physical address to a page in the page memory pool previously
 *            allocated by mm_pgalloc.
 *   npages - The number of contiguous pages to be return to the page memory
 *            pool, beginning with the page at paddr;
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void mm_pgfree(uintptr_t paddr, unsigned int npages)
{
  gran_free(g_pgalloc, (FAR void *)paddr, (size_t)npages << MM_PGSHIFT);
}

/****************************************************************************
 * Name: mm_pginfo
 *
 * Description:
 *   Return information about the page allocator.
 *
 * Input Parameters:
 *   handle - The handle previously returned by gran_initialize
 *   info   - Memory location to return the gran allocator info.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is return on
 *   any failure.
 *
 ****************************************************************************/

void mm_pginfo(FAR struct pginfo_s *info)
{
  struct graninfo_s graninfo;

  DEBUGASSERT(info != NULL);
  gran_info(g_pgalloc, &graninfo);

  info->ntotal = graninfo.ngranules;
  info->nfree  = graninfo.nfree;
  info->mxfree = graninfo.mxfree;
}

#endif /* CONFIG_MM_PGALLOC */
