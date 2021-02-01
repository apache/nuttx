/****************************************************************************
 * include/nuttx/pgalloc.h
 * Page memory allocator.
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

#ifndef __INCLUDE_NUTTX_PGALLOC_H
#define __INCLUDE_NUTTX_PGALLOC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>

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

#ifndef CONFIG_MM_PGALLOC_PGSIZE
#  define CONFIG_MM_PGALLOC_PGSIZE 4096
#endif

#if CONFIG_MM_PGSIZE == 1024
#  define MM_PGSIZE       1024
#  define MM_PGSHIFT      10
#elif CONFIG_MM_PGSIZE == 2048
#  define MM_PGSIZE       2048
#  define MM_PGSHIFT      11
#elif CONFIG_MM_PGSIZE == 4096
#  define MM_PGSIZE       4096
#  define MM_PGSHIFT      12
#elif CONFIG_MM_PGSIZE == 8192
#  define MM_PGSIZE       8192
#  define MM_PGSHIFT      13
#elif CONFIG_MM_PGSIZE == 16384
#  define MM_PGSIZE       16384
#  define MM_PGSHIFT      14
#else
#  error CONFIG_MM_PGSIZE not supported
#endif

#define MM_PGMASK         (MM_PGSIZE - 1)
#define MM_PGALIGNDOWN(a) ((uintptr_t)(a) & ~MM_PGMASK)
#define MM_PGALIGNUP(a)   (((uintptr_t)(a) + MM_PGMASK) & ~MM_PGMASK)
#define MM_NPAGES(s)      (((uintptr_t)(s) + MM_PGMASK) >> MM_PGSHIFT)
#define MM_ISALIGNED(a)   (((uintptr_t)(a) & MM_PGMASK) == 0)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Form in which the state of the granule allocator is returned, The size of
 * each page is MM_PGSIZE/MM_PGSHIFT
 */

struct pginfo_s
{
  uint16_t  ntotal;  /* The total number of pages */
  uint16_t  nfree;   /* The number of free pages */
  uint16_t  mxfree;  /* The longest sequence of free pages */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

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

void mm_pginitialize(FAR void *heap_start, size_t heap_size);

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

void mm_pgreserve(uintptr_t start, size_t size);

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

uintptr_t mm_pgalloc(unsigned int npages);

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

void mm_pgfree(uintptr_t paddr, unsigned int npages);

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

void mm_pginfo(FAR struct pginfo_s *info);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_MM_PGALLOC */
#endif /* __INCLUDE_NUTTX_PGALLOC_H */
