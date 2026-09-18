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
#include <sys/param.h>

#include <nuttx/addrenv.h>
#include <nuttx/arch.h>
#include <nuttx/nuttx.h>
#include <nuttx/pgalloc.h>
#include <nuttx/sched.h>

#include "sched/sched.h"

#include "esp32s3_addrenv.h"
#include "esp32s3_mmu.h"

#ifdef CONFIG_ESP32S3_SPIRAM
#  include "esp32s3_spiram.h"
#endif

#ifdef CONFIG_MM_PGALLOC

#if defined(CONFIG_ESP32S3_SPIRAM) && defined(CONFIG_ARCH_ADDRENV)

/* The page pool and a PSRAM kernel heap cannot coexist.  xtensa_add_region()
 * hands the *whole* allocable PSRAM window to the kernel heap, which
 * necessarily includes the pool -- and a kernel heap that contains pool
 * pages is a permanently mapped view of every process's memory, arriving
 * from the other side of the same problem esp32s3_pgmap() exists to solve.
 *
 * This is not hypothetical: up_textheap_memalign() falls back to the kernel
 * heap and derives an instruction-bus alias for anything it finds outside
 * internal RAM, which is the path a dlopen()ed shared library would take.
 * Kernel-side PSRAM has to come from outside the pool.
 */

#ifdef CONFIG_ESP32S3_SPIRAM_COMMON_HEAP
#  error "the page pool cannot be shared with a PSRAM kernel heap"
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct window_s
{
  FAR const char *name;
  uintptr_t       vbase;
  uint32_t        npages;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Every cache-MMU window this configuration uses, checked against each other
 * and against the kernel's PSRAM window at boot.  The scratch region is one
 * of them: it is a real window now, and the heap window growing into it
 * would be as silent as every other mapping mistake in this port.
 */

static const struct window_s g_windows[] =
{
  {
    .name   = "text",
    .vbase  = ESP32S3_TEXT_VBASE,
    .npages = CONFIG_ARCH_TEXT_NPAGES
  },
  {
    .name   = "data",
    .vbase  = ESP32S3_DATA_VBASE,
    .npages = CONFIG_ARCH_DATA_NPAGES
  },
  {
    .name   = "heap",
    .vbase  = ESP32S3_HEAP_VBASE,
    .npages = CONFIG_ARCH_HEAP_NPAGES
  },
  {
    .name   = "scratch",
    .vbase  = ESP32S3_KMAP_VBASE,
    .npages = ESP32S3_KMAP_NPAGES
  }
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: check_windows_clear
 *
 * Description:
 *   Panic if any two cache-MMU windows overlap, or if one of them overlaps
 *   the kernel's PSRAM window.  They would otherwise do it silently: the
 *   instruction and the data bus share one entry per 64 KB, so two windows
 *   64 MB apart are the same entries, and mapping one over another simply
 *   takes the first one's pages away.
 *
 *   The comparison is by entry, since that is what actually collides, and it
 *   belongs at run time because the kernel PSRAM window starts wherever the
 *   flash mappings end and so moves as the image grows.
 *
 * Input Parameters:
 *   kfirst - First cache-MMU entry of the kernel's PSRAM window.
 *   klast  - Last cache-MMU entry of the kernel's PSRAM window.
 *
 ****************************************************************************/

static void check_windows_clear(uint32_t kfirst, uint32_t klast)
{
  size_t i;
  size_t j;

  for (i = 0; i < nitems(g_windows); i++)
    {
      uint32_t first = MMU_ENTRY_OF(g_windows[i].vbase);
      uint32_t last  = first + g_windows[i].npages - 1;

      if (first <= klast && last >= kfirst)
        {
          _err("ERROR: %s window (MMU entries %" PRIu32 "-%" PRIu32 ") "
               "overlaps the kernel PSRAM window (entries %" PRIu32 "-%"
               PRIu32 ")\n", g_windows[i].name, first, last, kfirst, klast);
          PANIC();
        }

      for (j = 0; j < i; j++)
        {
          uint32_t ofirst = MMU_ENTRY_OF(g_windows[j].vbase);
          uint32_t olast  = ofirst + g_windows[j].npages - 1;

          if (first <= olast && last >= ofirst)
            {
              _err("ERROR: %s window (MMU entries %" PRIu32 "-%" PRIu32 ") "
                   "overlaps the %s window (entries %" PRIu32 "-%" PRIu32
                   ")\n", g_windows[i].name, first, last, g_windows[j].name,
                   ofirst, olast);
              PANIC();
            }
        }
    }
}
#endif

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
 *   hands out physical page addresses.
 *
 *   It is deliberately not mapped into the kernel.  esp32s3_spiram.c maps
 *   the whole PSRAM device at boot, and this function is where the pool's
 *   share of that mapping is withdrawn again, after the checks that need it.
 *   The kernel reaches a pool page one at a time through esp32s3_pgmap();
 *   see esp32s3_addrenv.h for why a permanent window cannot be allowed to
 *   stand.
 *
 * Input Parameters:
 *   heap_start - Receives the physical base address of the page pool.
 *   heap_size  - Receives the size of the page pool in bytes.
 *
 ****************************************************************************/

void up_allocate_pgheap(void **heap_start, size_t *heap_size)
{
  DEBUGASSERT(heap_start && heap_size);

#if defined(CONFIG_ESP32S3_SPIRAM) && defined(CONFIG_ARCH_ADDRENV)
  /* Where the kernel's PSRAM window lands is decided at run time:
   * esp32s3_spiram.c maps PSRAM immediately after the last cache-MMU entry
   * the flash mappings occupy, so it moves as the kernel image grows.  The
   * pool is described by a compile-time *physical* base, so ask the cache
   * MMU where that physical page currently is rather than deriving it from
   * a constant -- which is how CONFIG_ARCH_PGPOOL_PBASE went stale twice,
   * and the second time by exactly one page, which left one unwiped page in
   * every region of every new process.
   */

  {
    uintptr_t ramstart = (uintptr_t)esp_spiram_allocable_vaddr_start();
    uintptr_t ramend   = (uintptr_t)esp_spiram_allocable_vaddr_end();
    uintptr_t poolvbase;
    uint32_t  rampbase;

    if (!esp32s3_mmu_paddr(ramstart, &rampbase))
      {
        _err("ERROR: PSRAM window base %08" PRIxPTR " maps nothing\n",
             ramstart);
        PANIC();
      }

    if (ESP32S3_PGPOOL_PBASE < rampbase ||
        ESP32S3_PGPOOL_PEND > rampbase + (ramend - ramstart))
      {
        _err("ERROR: page pool %08x-%08x is outside the mapped PSRAM "
             "%08" PRIx32 "-%08" PRIxPTR "\n",
             ESP32S3_PGPOOL_PBASE, ESP32S3_PGPOOL_PEND,
             rampbase, rampbase + (ramend - ramstart));
        PANIC();
      }

    poolvbase = ramstart + (ESP32S3_PGPOOL_PBASE - rampbase);

    _info("PSRAM window %08" PRIxPTR "-%08" PRIxPTR " (phys %08" PRIx32
          "), page pool phys %08x-%08x at %08" PRIxPTR "\n",
          ramstart, ramend, rampbase,
          ESP32S3_PGPOOL_PBASE, ESP32S3_PGPOOL_PEND, poolvbase);

    check_windows_clear(MMU_ENTRY_OF(ramstart), MMU_ENTRY_OF(ramend - 1));

    /* Everything above has been established while the pool was still
     * mapped, which is the only time it can be.  Now take that mapping
     * away: from here the kernel reaches a pool page only through
     * esp32s3_pgmap(), and an unprivileged task reaches one only if it is
     * its own.
     */

    esp32s3_pgpool_unmap(poolvbase, ESP32S3_PGPOOL_SIZE);
  }
#endif

  *heap_start = (void *)ESP32S3_PGPOOL_PBASE;
  *heap_size  = (size_t)ESP32S3_PGPOOL_SIZE;
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
