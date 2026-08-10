/****************************************************************************
 * arch/xtensa/src/esp32s3/esp32s3_pgalloc.c
 *
 * SPDX-License-Identifier: Apache-2.0
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
#include <debug.h>
#include <inttypes.h>

#include <nuttx/addrenv.h>
#include <nuttx/arch.h>
#include <nuttx/pgalloc.h>
#include <nuttx/sched.h>

#include "sched/sched.h"

#include "esp32s3_addrenv.h"

#ifdef CONFIG_ESP32S3_SPIRAM
#  include "esp32s3_spiram.h"
#endif

#ifdef CONFIG_MM_PGALLOC

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
 *   On the ESP32-S3 the page pool is a slice of the external octal PSRAM.
 *   The pool is described by its *physical* base -- for the cache MMU that
 *   is the zero-based offset into the PSRAM device -- because mm_pgalloc()
 *   hands out physical page addresses.  The whole pool is also permanently
 *   mapped into the kernel (WORLD0) data-bus window at
 *   CONFIG_ARCH_PGPOOL_VBASE so the kernel can reach any page it allocates
 *   (see esp32s3_pgvaddr()).
 *
 * Input Parameters:
 *   heap_start - Receives the physical base address of the page pool.
 *   heap_size  - Receives the size of the page pool in bytes.
 *
 ****************************************************************************/

void up_allocate_pgheap(void **heap_start, size_t *heap_size)
{
  DEBUGASSERT(heap_start && heap_size);

#ifdef CONFIG_ESP32S3_SPIRAM
  /* Where the kernel's PSRAM window lands is decided at run time:
   * esp32s3_spiram.c maps PSRAM immediately after the last cache-MMU entry
   * the flash mappings occupy, so it moves as the kernel image grows.  The
   * page pool is described to the OS by compile-time constants, so the two
   * have to be checked against each other -- and loudly, because getting it
   * wrong is otherwise silent: an unmapped cache window swallows writes and
   * reads back as zero without faulting, so a misplaced pool would simply
   * lose every page handed out of it.
   */

    {
      uintptr_t ramstart = (uintptr_t)esp_spiram_allocable_vaddr_start();
      uintptr_t ramend   = (uintptr_t)esp_spiram_allocable_vaddr_end();

      _info("PSRAM window %08" PRIxPTR "-%08" PRIxPTR ", "
            "page pool %08x-%08x\n",
            ramstart, ramend,
            CONFIG_ARCH_PGPOOL_VBASE, CONFIG_ARCH_PGPOOL_VEND);

      if ((uintptr_t)CONFIG_ARCH_PGPOOL_VBASE < ramstart ||
          (uintptr_t)CONFIG_ARCH_PGPOOL_VEND > ramend)
        {
          _err("ERROR: page pool %08x-%08x is outside the mapped PSRAM "
               "window %08" PRIxPTR "-%08" PRIxPTR "\n",
               CONFIG_ARCH_PGPOOL_VBASE, CONFIG_ARCH_PGPOOL_VEND,
               ramstart, ramend);
          PANIC();
        }
    }
#endif

  *heap_start = (void *)CONFIG_ARCH_PGPOOL_PBASE;
  *heap_size  = (size_t)CONFIG_ARCH_PGPOOL_SIZE;
}

#ifdef CONFIG_BUILD_KERNEL

/****************************************************************************
 * Name: pgalloc
 *
 * Description:
 *   If there is a page allocator in the configuration and if and MMU is
 *   available to map physical addresses to virtual address, then function
 *   must be provided by the platform-specific code.  This is part of the
 *   implementation of sbrk().  This function will allocate the requested
 *   number of pages using the page allocator and map them into consecutive
 *   virtual addresses beginning with 'brkaddr'
 *
 *   NOTE:  This function does not use the up_ naming standard because it
 *   is indirectly callable from user-space code via a system trap.
 *   Therefore, it is a system interface and follows a different naming
 *   convention.
 *
 *   The ESP32-S3 has no per-process page table: a page is "mapped" by
 *   recording it in the address environment's heap page array, and the
 *   shared data-bus cache-MMU window is programmed from that array by
 *   up_addrenv_select().  Since sbrk() runs on behalf of the calling task,
 *   whose environment is by definition the resident one, the new pages are
 *   also programmed into the window right away so the caller can use them
 *   without waiting for a context switch.
 *
 * Input Parameters:
 *   brkaddr - The heap break address.  The next page will be allocated and
 *     mapped to this address.  Must be page aligned.  If the memory manager
 *     has not yet been initialized and this is the first block requested for
 *     the heap, then brkaddr should be zero.  pgalloc will then assigned the
 *     well-known virtual address of the beginning of the heap.
 *   npages - The number of pages to allocate and map.  Mapping of pages
 *     will be contiguous beginning beginning at 'brkaddr'
 *
 * Returned Value:
 *   The (virtual) base address of the mapped page will returned on success.
 *   Normally this will be the same as the 'brkaddr' input. However, if
 *   the 'brkaddr' input was zero, this will be the virtual address of the
 *   beginning of the heap.  Zero is returned on any failure.
 *
 ****************************************************************************/

uintptr_t pgalloc(uintptr_t brkaddr, unsigned int npages)
{
  struct tcb_s   *tcb = this_task();
  arch_addrenv_t *addrenv;
  uintptr_t       vaddr;
  unsigned int    index;

  DEBUGASSERT(tcb && tcb->addrenv_own);
  addrenv = &tcb->addrenv_own->addrenv;

  /* brkaddr = 0 means that no heap has yet been allocated */

  if (brkaddr == 0)
    {
      brkaddr = addrenv->heapvbase;
    }

  DEBUGASSERT(brkaddr >= addrenv->heapvbase);
  DEBUGASSERT(MM_ISALIGNED(brkaddr));

  /* Start mapping from the old heap break address */

  vaddr = brkaddr;
  index = (brkaddr - addrenv->heapvbase) >> MM_PGSHIFT;

  for (; npages > 0; npages--, index++, vaddr += MM_PGSIZE)
    {
      uintptr_t paddr;

      if (index >= CONFIG_ARCH_HEAP_NPAGES)
        {
          berr("ERROR: heap window is full (%d pages)\n",
               CONFIG_ARCH_HEAP_NPAGES);
          return 0;
        }

      /* up_addrenv_create() already backs the initial heap allocation, so a
       * page may well be present at this index.  Reuse it rather than
       * leaking it behind a fresh allocation.
       */

      paddr = addrenv->heappages[index];
      if (paddr == 0)
        {
          paddr = mm_pgalloc(1);
          if (paddr == 0)
            {
              berr("ERROR: page pool exhausted\n");
              return 0;
            }

          esp32s3_pgwipe(paddr);

          addrenv->heappages[index] = paddr;
          if (index >= addrenv->nheap)
            {
              addrenv->nheap = index + 1;
            }
        }

      /* Make the page reachable by the running task */

      esp32s3_addrenv_mapnew(addrenv, vaddr, paddr);
    }

  return brkaddr;
}

#endif /* CONFIG_BUILD_KERNEL */
#endif /* CONFIG_MM_PGALLOC */
