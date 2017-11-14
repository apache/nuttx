/****************************************************************************
 * mm/mm_gran/mm_pgalloc.c
 *
 *   Copyright (C) 2014, 2017 Gregory Nutt. All rights reserved.
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

#ifdef CONFIG_CPP_HAVE_VARARGS
#  ifdef CONFIG_DEBUG_PGALLOC
#    define pgaerr(format, ...)       _err(format, ##__VA_ARGS__)
#    define pgawarn(format, ...)      _warn(format, ##__VA_ARGS__)
#    define pgainfo(format, ...)      _info(format, ##__VA_ARGS__)
#  else
#    define pgaerr(format, ...)       merr(format, ##__VA_ARGS__)
#    define pgawarn(format, ...)      mwarn(format, ##__VA_ARGS__)
#    define pgainfo(format, ...)      minfo(format, ##__VA_ARGS__)
#  endif
#else
#  ifdef CONFIG_DEBUG_PGALLOC
#    define pgaerr                    _err
#    define pgawarn                   _warn
#    define pgainfo                   _info
#  else
#    define pgaerr                    merr
#    define pgawarn                   mwarn
#    define pgainfo                   minfo
#  endif
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
  DEBUGASSERT(pg_alloc != NULL);
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
  return (uintptr_t)gran_alloc(g_pgalloc, (size_t)1 << MM_PGSHIFT);
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
