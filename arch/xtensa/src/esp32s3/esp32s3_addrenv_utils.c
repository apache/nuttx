/****************************************************************************
 * arch/xtensa/src/esp32s3/esp32s3_addrenv_utils.c
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
#include <sched.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <nuttx/addrenv.h>
#include <nuttx/arch.h>
#include <nuttx/pgalloc.h>

#include "esp32s3_addrenv.h"
#include "esp32s3_mmu.h"

#ifdef CONFIG_ARCH_ADDRENV

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The kernel's scratch region: the only way it can reach a page-pool page,
 * one page at a time and only while it is working on it.  A slot holds the
 * physical page it currently maps, or 0 when it is free.
 *
 * There is no lock here on purpose.  Callers hold sched_lock() for the whole
 * map/use/unmap sequence -- which they must anyway, so that a live mapping
 * cannot be observed by an unprivileged task -- and that also makes this
 * table single-threaded.  No interrupt handler reaches these paths.
 */

static uintptr_t g_scratch[ESP32S3_KMAP_NPAGES];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: scratch_slot
 *
 * Description:
 *   Return the scratch slot index a scratch virtual address belongs to, or
 *   -1 if the address is not in the scratch region.
 *
 ****************************************************************************/

static int scratch_slot(uintptr_t vaddr)
{
  if (vaddr < ESP32S3_KMAP_VBASE || vaddr >= ESP32S3_KMAP_VEND)
    {
      return -1;
    }

  return (int)((vaddr - ESP32S3_KMAP_VBASE) >> MM_PGSHIFT);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32s3_pgmap
 ****************************************************************************/

uintptr_t esp32s3_pgmap(uintptr_t paddr)
{
  uintptr_t vaddr;
  int       i;

  DEBUGASSERT(!up_interrupt_context());
  DEBUGASSERT(MM_ISALIGNED(paddr));

  if (!esp32s3_pgpool_page(paddr))
    {
      return 0;
    }

  for (i = 0; i < ESP32S3_KMAP_NPAGES; i++)
    {
      if (g_scratch[i] == 0)
        {
          break;
        }
    }

  if (i >= ESP32S3_KMAP_NPAGES)
    {
      return 0;
    }

  g_scratch[i] = paddr;
  vaddr = ESP32S3_KMAP_VBASE + i * MM_PGSIZE;

  esp32s3_mmu_scratch_map(vaddr, paddr);

  return vaddr;
}

/****************************************************************************
 * Name: esp32s3_pgunmap
 ****************************************************************************/

void esp32s3_pgunmap(uintptr_t vaddr)
{
  int i = scratch_slot(vaddr);

  DEBUGASSERT(i >= 0 && g_scratch[i] != 0);

  esp32s3_mmu_scratch_unmap(ESP32S3_KMAP_VBASE + i * MM_PGSIZE);

  g_scratch[i] = 0;
}

/****************************************************************************
 * Name: esp32s3_pgwipe
 ****************************************************************************/

void esp32s3_pgwipe(uintptr_t paddr)
{
  uintptr_t vaddr;

  /* Hold off the scheduler for the whole sequence.  Not for mutual exclusion
   * against another wipe -- there is none to worry about here -- but because
   * a scratch address is reachable by the unprivileged world like any other
   * external memory address, so no user task may run while a pool page is
   * parked at one.
   */

  sched_lock();

  vaddr = esp32s3_pgmap(paddr);
  if (vaddr == 0)
    {
      /* Not recoverable, and much too dangerous to let pass: an unwiped page
       * carries its previous tenant's data into a new process.  This is
       * exactly how the CONFIG_ARCH_PGPOOL_PBASE drift stayed hidden.
       */

      _err("ERROR: no scratch mapping for page %08" PRIxPTR "\n", paddr);
      PANIC();
    }

  memset((void *)vaddr, 0, MM_PGSIZE);
  esp32s3_pgunmap(vaddr);

  sched_unlock();
}

/****************************************************************************
 * Name: esp32s3_pgpool_unmap
 ****************************************************************************/

void esp32s3_pgpool_unmap(uintptr_t vbase, size_t size)
{
  uint32_t cache_state;

  /* Take away the boot-time mapping of the pool in one go.  esp32s3_spiram.c
   * mapped the whole PSRAM device; only the pool's share of that window is
   * withdrawn, because the rest maps no page a process will ever own and is
   * the aperture a kernel-side PSRAM allocation (a loaded library's text
   * heap, say) would have to come from.
   *
   * Write back before invalidating the entries: this runs after the PSRAM
   * memory test, which leaves the window's lines dirty.
   */

  cache_state = esp32s3_dcache_suspend(true);
  esp32s3_mmu_unmap(vbase, size / MM_PGSIZE);
  esp32s3_icache_invalidate_all();
  esp32s3_dcache_resume(cache_state);
}

/****************************************************************************
 * Name: up_addrenv_find_page
 *
 * Description:
 *   Find the physical address of the page that backs a user virtual address
 *   in the given address environment.  On the ESP32-S3 there is no per-task
 *   page table to walk; the physical pages are recorded per region in the
 *   address environment, so the lookup reduces to selecting the window that
 *   contains 'vaddr' and indexing that region's page array.
 *
 * Returned Value:
 *   Physical address of the backing page (page-aligned) on success, or 0 if
 *   'vaddr' is not a mapped user address.
 *
 ****************************************************************************/

uintptr_t up_addrenv_find_page(arch_addrenv_t *addrenv, uintptr_t vaddr)
{
  const uintptr_t *pages;
  uintptr_t        base;
  uint16_t         count;
  uint16_t         index;

  DEBUGASSERT(addrenv);

  if (vaddr >= ESP32S3_TEXT_VBASE && vaddr < ESP32S3_TEXT_VEND)
    {
      base  = ESP32S3_TEXT_VBASE;
      pages = addrenv->textpages;
      count = addrenv->ntext;
    }
  else if (vaddr >= ESP32S3_DATA_VBASE && vaddr < ESP32S3_DATA_VEND)
    {
      base  = ESP32S3_DATA_VBASE;
      pages = addrenv->datapages;
      count = addrenv->ndata;
    }
  else if (vaddr >= ESP32S3_HEAP_VBASE && vaddr < ESP32S3_HEAP_VEND)
    {
      base  = ESP32S3_HEAP_VBASE;
      pages = addrenv->heappages;
      count = addrenv->nheap;
    }
  else
    {
      return 0;
    }

  index = (uint16_t)((vaddr - base) >> MM_PGSHIFT);
  if (index >= count)
    {
      return 0;
    }

  return pages[index];
}

/****************************************************************************
 * Name: up_addrenv_page_vaddr
 *
 * Description:
 *   Get the kernel virtual address of a physical page allocated for an
 *   address environment.
 *
 *   The ESP32-S3 cannot answer this.  The contract is a kernel address that
 *   stays valid after the call returns, and the page pool is deliberately
 *   not mapped: a pool page is reachable only for the duration of an
 *   esp32s3_pgmap()/esp32s3_pgunmap() pair.  Returning an address that is
 *   about to stop meaning anything would be worse than refusing, so this
 *   fails, and CONFIG_MM_KMAP -- whose single-page path is built on this
 *   function -- is rejected at compile time in esp32s3_addrenv.h.
 *
 ****************************************************************************/

uintptr_t up_addrenv_page_vaddr(uintptr_t page)
{
  UNUSED(page);
  return 0;
}

/****************************************************************************
 * Name: up_addrenv_user_vaddr
 *
 * Description:
 *   Check if a virtual address is a user virtual address.
 *
 ****************************************************************************/

bool up_addrenv_user_vaddr(uintptr_t vaddr)
{
  return esp32s3_uservaddr(vaddr);
}

/****************************************************************************
 * Name: up_addrenv_page_wipe
 *
 * Description:
 *   Wipe a page of physical memory, first mapping it into kernel virtual
 *   memory.
 *
 ****************************************************************************/

void up_addrenv_page_wipe(uintptr_t page)
{
  esp32s3_pgwipe(page);
}

/****************************************************************************
 * Name: up_addrenv_pa_to_va
 *
 * Description:
 *   Map a physical page-pool address to the kernel virtual address that
 *   currently maps it.
 *
 *   Which is now literally what this does: only a page held in a scratch
 *   slot is mapped at all, so the answer comes from the slot table rather
 *   than from arithmetic over a window that no longer exists.  A page nobody
 *   is working on has no kernel virtual address, and 0 says so.
 *
 ****************************************************************************/

void *up_addrenv_pa_to_va(uintptr_t pa)
{
  uintptr_t page = MM_PGALIGNDOWN(pa);
  int       i;

  /* A free slot holds 0, so page 0 could otherwise match one of them */

  if (!esp32s3_pgpool_page(page))
    {
      return NULL;
    }

  for (i = 0; i < ESP32S3_KMAP_NPAGES; i++)
    {
      if (g_scratch[i] == page)
        {
          return (void *)(ESP32S3_KMAP_VBASE + i * MM_PGSIZE +
                          (pa & MM_PGMASK));
        }
    }

  return NULL;
}

/****************************************************************************
 * Name: up_addrenv_va_to_pa
 *
 * Description:
 *   Map a kernel page-pool virtual address back to its physical address.
 *   The inverse of the above, and just as narrow: scratch addresses are the
 *   only kernel virtual addresses that name a pool page.
 *
 ****************************************************************************/

uintptr_t up_addrenv_va_to_pa(void *va)
{
  uintptr_t vaddr = (uintptr_t)va;
  int       i     = scratch_slot(vaddr);

  if (i < 0 || g_scratch[i] == 0)
    {
      return 0;
    }

  return g_scratch[i] + (vaddr & MM_PGMASK);
}

#endif /* CONFIG_ARCH_ADDRENV */
