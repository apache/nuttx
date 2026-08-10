/****************************************************************************
 * arch/xtensa/src/esp32s3/esp32s3_addrenv.h
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

#ifndef __ARCH_XTENSA_SRC_ESP32S3_ESP32S3_ADDRENV_H
#define __ARCH_XTENSA_SRC_ESP32S3_ESP32S3_ADDRENV_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <nuttx/addrenv.h>
#include <nuttx/pgalloc.h>

#ifdef CONFIG_ARCH_ADDRENV

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The page pool must NOT be statically mapped into the kernel.  It is carved
 * out of the same PSRAM the user processes run from, and the external memory
 * permissions (APB_CTRL_SRAM_ACEn_*) are indexed by physical address, so a
 * permanent kernel window onto the pool is a window onto every process's
 * memory that no permission setting can close: a process's own pages are
 * pool pages.  This was measured, not assumed -- a user task read another
 * process's .bss through it.  The kernel uses the scratch region below
 * instead.
 */

#ifdef CONFIG_ARCH_PGPOOL_MAPPING
#  error "ESP32-S3 must not map the page pool; see esp32s3_pgmap()"
#endif

#ifndef CONFIG_ARCH_KMAP_VBASE
#  error "ESP32-S3 address environments need CONFIG_ARCH_KMAP_VBASE"
#endif

#if CONFIG_ARCH_KMAP_NPAGES < 2
#  error "CONFIG_ARCH_KMAP_NPAGES must be at least 2 (fork() copies "        \
         "a source and a destination page at once)"
#endif

/* CONFIG_MM_KMAP cannot work here.  kmm_map()'s single-page path is
 * up_addrenv_page_vaddr(), which asks for a kernel address that stays valid
 * after the call returns -- exactly what a scratch mapping cannot promise.
 */

#ifdef CONFIG_MM_KMAP
#  error "CONFIG_MM_KMAP needs a permanently mapped page pool"
#endif

/* The kernel's scratch region.  ARCH_KMAP_VEND is only defined by
 * <nuttx/addrenv.h> when CONFIG_MM_KMAP is set, which it is not.
 */

#define ESP32S3_KMAP_VBASE  (CONFIG_ARCH_KMAP_VBASE)
#define ESP32S3_KMAP_NPAGES (CONFIG_ARCH_KMAP_NPAGES)
#define ESP32S3_KMAP_VEND   (CONFIG_ARCH_KMAP_VBASE + \
                             CONFIG_ARCH_KMAP_NPAGES * MM_PGSIZE)

/* The page pool, described physically.  mm_pgalloc() hands out physical page
 * addresses; only the virtual mapping went away, not the pool.
 */

#define ESP32S3_PGPOOL_PBASE (CONFIG_ESP32S3_PGPOOL_PBASE)
#define ESP32S3_PGPOOL_SIZE  (CONFIG_ESP32S3_PGPOOL_SIZE)
#define ESP32S3_PGPOOL_PEND  (CONFIG_ESP32S3_PGPOOL_PBASE + \
                              CONFIG_ESP32S3_PGPOOL_SIZE)

/* The user address space is split across two disjoint cache-MMU windows:
 * .text lives in the instruction-bus window, .data/.bss and the heap in the
 * data-bus window.  Each window is described by its base and page count.
 */

#define ESP32S3_TEXT_VBASE  (CONFIG_ARCH_TEXT_VBASE)
#define ESP32S3_TEXT_VEND   (CONFIG_ARCH_TEXT_VBASE + \
                             CONFIG_ARCH_TEXT_NPAGES * MM_PGSIZE)
#define ESP32S3_DATA_VBASE  (CONFIG_ARCH_DATA_VBASE)
#define ESP32S3_DATA_VEND   (CONFIG_ARCH_DATA_VBASE + \
                             CONFIG_ARCH_DATA_NPAGES * MM_PGSIZE)
#define ESP32S3_HEAP_VBASE  (CONFIG_ARCH_HEAP_VBASE)
#define ESP32S3_HEAP_VEND   (CONFIG_ARCH_HEAP_VBASE + \
                             CONFIG_ARCH_HEAP_NPAGES * MM_PGSIZE)

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32s3_pgpool_page
 *
 * Description:
 *   Return true if paddr is a page-pool physical address.
 *
 ****************************************************************************/

static inline bool esp32s3_pgpool_page(uintptr_t paddr)
{
  return (paddr >= ESP32S3_PGPOOL_PBASE && paddr < ESP32S3_PGPOOL_PEND);
}

/****************************************************************************
 * Name: esp32s3_uservaddr
 *
 * Description:
 *   Return true if vaddr lies inside one of the user (.text/.data/heap)
 *   cache-MMU windows.
 *
 ****************************************************************************/

static inline bool esp32s3_uservaddr(uintptr_t vaddr)
{
  return ((vaddr >= ESP32S3_TEXT_VBASE && vaddr < ESP32S3_TEXT_VEND) ||
          (vaddr >= ESP32S3_DATA_VBASE && vaddr < ESP32S3_DATA_VEND) ||
          (vaddr >= ESP32S3_HEAP_VBASE && vaddr < ESP32S3_HEAP_VEND));
}

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: esp32s3_pgmap
 *
 * Description:
 *   Map a page-pool physical page into the kernel's scratch region and
 *   return the virtual address it can be reached at.
 *
 *   This replaces the arithmetic esp32s3_pgvaddr() the port used while the
 *   whole pool was mapped, and unlike it the result has a *lifetime*: it is
 *   valid until the matching esp32s3_pgunmap(), and not one instruction
 *   longer.  It must not be stored anywhere that outlives the operation.
 *
 *   The caller must hold sched_lock() across the whole map/use/unmap
 *   sequence.  A scratch address is ordinary external memory as far as the
 *   permission control is concerned, so an unprivileged task that ran while
 *   the mapping was live could read the page through it -- which is the leak
 *   this whole arrangement exists to close.  Interrupts do not need to be
 *   disabled: no interrupt handler reaches these paths.
 *
 * Input Parameters:
 *   paddr - Physical (page pool) address of the page, page-aligned.
 *
 * Returned Value:
 *   The kernel virtual address of the page, or 0 if 'paddr' is not a pool
 *   page or no scratch slot is free.
 *
 ****************************************************************************/

uintptr_t esp32s3_pgmap(uintptr_t paddr);

/****************************************************************************
 * Name: esp32s3_pgunmap
 *
 * Description:
 *   Release a mapping made by esp32s3_pgmap(), writing back anything written
 *   through it and leaving the scratch entry invalid.
 *
 * Input Parameters:
 *   vaddr - The address esp32s3_pgmap() returned.
 *
 ****************************************************************************/

void esp32s3_pgunmap(uintptr_t vaddr);

/****************************************************************************
 * Name: esp32s3_pgwipe
 *
 * Description:
 *   Zero a page-pool physical page, mapping it into the kernel's scratch
 *   region for as long as that takes.
 *
 * Input Parameters:
 *   paddr - Physical (page pool) address of the page.
 *
 ****************************************************************************/

void esp32s3_pgwipe(uintptr_t paddr);

/****************************************************************************
 * Name: esp32s3_pgpool_unmap
 *
 * Description:
 *   Tear down the boot-time cache-MMU mapping of the page pool, leaving the
 *   pool reachable only a page at a time through esp32s3_pgmap().  Called
 *   once, from up_allocate_pgheap(), after its consistency checks -- which
 *   have to run first, since they ask the cache MMU where the pool actually
 *   is rather than deriving it.
 *
 * Input Parameters:
 *   vbase - Virtual base the boot-time mapping put the pool at.
 *   size  - Size of the pool in bytes.
 *
 ****************************************************************************/

void esp32s3_pgpool_unmap(uintptr_t vbase, size_t size);

/****************************************************************************
 * Name: esp32s3_addrenv_mapnew
 *
 * Description:
 *   Make a page that was added to an address environment after that
 *   environment was created -- heap growth through sbrk()/pgalloc() --
 *   visible to the running task.  The cache-MMU windows only ever reflect
 *   the resident environment, so this is a no-op unless 'addrenv' is the one
 *   currently selected.  For any other environment the page is picked up
 *   from the page array by the next up_addrenv_select().
 *
 * Input Parameters:
 *   addrenv - The address environment the page was added to.
 *   vaddr   - The user virtual address the page is mapped at.
 *   paddr   - The physical (page pool) address of the page.
 *
 ****************************************************************************/

void esp32s3_addrenv_mapnew(const arch_addrenv_t *addrenv, uintptr_t vaddr,
                            uintptr_t paddr);

#endif /* CONFIG_ARCH_ADDRENV */
#endif /* __ARCH_XTENSA_SRC_ESP32S3_ESP32S3_ADDRENV_H */
