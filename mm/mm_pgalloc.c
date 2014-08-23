/****************************************************************************
 * mm/mm_pgalloc.c
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

#include <assert.h>

#include <nuttx/gran.h>
#include <nuttx/pgalloc.h>

#include "mm_gran.h"

#ifdef CONFIG_MM_PGALLOC

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/
/* CONFIG_MM_PGALLOC - Enable page allocator support
 * CONFIG_MM_PGSIZE - The page size.  Must be one of {1024, 2048,
 *   4096, 8192, or 16384}.  This is easily extensible, but only those
 *   values are currently support.
 * CONFIG_MM_PGPOOL_PADDR - Physical address of the start of the page
 *   memory pool.  This will be aligned to the page size if it is not
 *   already aligned.
 * CONFIG_MM_PGPOOL_SIZE - The size of the page memory pool in bytes.  This
 *   will be aligned if it is not already aligned.
 * CONFIG_DEBUG_PGALLOC - Just like CONFIG_DEBUG_MM, but only generates
 *   output from the page allocation logic.
 *
 * Dependencies:  CONFIG_ARCH_HAVE_MMU and CONFIG_GRAN
 */
 
/* Debug */

#ifdef CONFIG_CPP_HAVE_VARARGS
#  ifdef CONFIG_DEBUG_PGALLOC
#    define pgadbg(format, ...)       dbg(format, ##__VA_ARGS__)
#    define pgavdbg(format, ...)      vdbg(format, ##__VA_ARGS__)
#  else
#    define pgadbg(format, ...)       mdbg(format, ##__VA_ARGS__)
#    define pgavdbg(format, ...)      mvdbg(format, ##__VA_ARGS__)
#  endif
#else
#  ifdef CONFIG_DEBUG_PGALLOC
#    define pgadbg                    dbg
#    define pgavdbg                   vdbg
#  else
#    define pgadbg                    (void)
#    define pgavdbg                   (void)
#  endif
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifndef CONFIG_GRAN_SINGLE
/* The state of the page allocator */

static GRAN_HANDLE g_pgalloc;
#endif

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
 *   None
 *
 * Returned Value:
 *   Mpme
 *
 ****************************************************************************/

void mm_pginitialize(void)
{
#ifdef CONFIG_GRAN_SINGLE
  int ret;

  ret = gran_initialize((FAR void *)CONFIG_MM_PGPOOL_PADDR,
                        CONFIG_MM_PGPOOL_SIZE,
                        MM_PGSHIFT, MM_PGSHIFT);
  DEBUGASSERT(ret == OK);
  UNUSED(ret);

#else
  g_pgalloc = gran_initialize((FAR void *)CONFIG_MM_PGPOOL_PADDR,
                              CONFIG_MM_PGPOOL_SIZE,
                              MM_PGSHIFT, MM_PGSHIFT);
  DEBUGASSERT(pg_alloc != NULL);

#endif
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
#ifdef CONFIG_GRAN_SINGLE
  gran_reserve(start, size);
#else
  gran_reserve(g_pgalloc, start, size);
#endif
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
#ifdef CONFIG_GRAN_SINGLE
  return (uintptr_t)gran_alloc((size_t)1 << MM_PGSHIFT);
#else
  return (uintptr_t)gran_alloc(g_pgalloc, (size_t)1 << MM_PGSHIFT);
#endif
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
#ifdef CONFIG_GRAN_SINGLE
  gran_free((FAR void*)paddr, (size_t)npages << MM_PGSHIFT);
#else
  gran_free(g_pgalloc, (FAR void*)paddr, (size_t)npages << MM_PGSHIFT);
#endif
}

#endif /* CONFIG_MM_PGALLOC */
