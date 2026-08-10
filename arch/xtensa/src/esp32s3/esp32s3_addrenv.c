/****************************************************************************
 * arch/xtensa/src/esp32s3/esp32s3_addrenv.c
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
#include <errno.h>
#include <sched.h>
#include <string.h>

#include <nuttx/addrenv.h>
#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/pgalloc.h>
#include <nuttx/sched.h>

#include "soc/ext_mem_defs.h"

#include "esp32s3_addrenv.h"
#include "esp32s3_mmu.h"
#include "esp32s3_spiram.h"

#ifdef CONFIG_ARCH_ADDRENV

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The ESP32-S3 cache MMU is a single global remap table: only one user
 * address environment can be resident in the shared .text/.data/.heap
 * windows at a time.  Track which one it is so up_addrenv_select() can skip
 * the (expensive) remap when the incoming environment is already active --
 * the common thread<->thread, ISR and syscall case.
 */

static const arch_addrenv_t *g_current_addrenv;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: alloc_region
 *
 * Description:
 *   Allocate and wipe the physical page-pool (PSRAM) pages that back a
 *   single user region, recording them in the caller's page array.  On the
 *   ESP32-S3 the physical pages are only recorded here; the global cache-MMU
 *   table is (re)programmed lazily in up_addrenv_select().
 *
 * Input Parameters:
 *   pages    - Destination page array (physical addresses)
 *   maxpages - Capacity of the page array
 *   size     - Region size in bytes
 *   count    - Receives the number of pages actually allocated
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.  On failure
 *   *count reflects the pages allocated so far so the caller can free them.
 *
 ****************************************************************************/

static int alloc_region(uintptr_t *pages, unsigned int maxpages, size_t size,
                        uint16_t *count)
{
  unsigned int npages = MM_NPAGES(size);
  unsigned int i;

  *count = 0;

  if (npages > maxpages)
    {
      berr("ERROR: region needs %u pages, only %u available\n",
           npages, maxpages);
      return -E2BIG;
    }

  for (i = 0; i < npages; i++)
    {
      uintptr_t paddr = mm_pgalloc(1);
      if (paddr == 0)
        {
          berr("ERROR: page pool exhausted at page %u of %u\n", i, npages);
          *count = i;
          return -ENOMEM;
        }

      esp32s3_pgwipe(paddr);
      pages[i] = paddr;
    }

  *count = npages;
  return OK;
}

/****************************************************************************
 * Name: free_region
 *
 * Description:
 *   Return every page recorded in a region's page array to the page pool.
 *
 ****************************************************************************/

static void free_region(uintptr_t *pages, uint16_t *count)
{
  uint16_t i;

  for (i = 0; i < *count; i++)
    {
      if (pages[i] != 0)
        {
          mm_pgfree(pages[i], 1);
          pages[i] = 0;
        }
    }

  *count = 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_addrenv_create
 *
 * Description:
 *   This function is called when a new task is created in order to
 *   instantiate an address environment for the new task group.  Physical
 *   pages for .text, .data/.bss and the heap are allocated from the PSRAM
 *   page pool and recorded in 'addrenv'.  The reserved OS region (heap MM
 *   bookkeeping and signal delivery support) occupies the first page of the
 *   data window.
 *
 ****************************************************************************/

int up_addrenv_create(size_t textsize, size_t datasize, size_t heapsize,
                      arch_addrenv_t *addrenv)
{
  size_t datatotal;
  int    ret;

  DEBUGASSERT(addrenv);
  DEBUGASSERT(MM_ISALIGNED(CONFIG_ARCH_TEXT_VBASE));
  DEBUGASSERT(MM_ISALIGNED(CONFIG_ARCH_DATA_VBASE));
  DEBUGASSERT(MM_ISALIGNED(CONFIG_ARCH_HEAP_VBASE));

  /* Start from a clean slate */

  memset(addrenv, 0, sizeof(arch_addrenv_t));

  /* The data window carries the OS reserve at its base, followed by the
   * task's .data/.bss.  vdata is therefore reported past the reserve.
   */

  datatotal = MM_PGALIGNUP(ARCH_DATA_RESERVE_SIZE) + datasize;

  addrenv->textvbase = CONFIG_ARCH_TEXT_VBASE;
  addrenv->datavbase = CONFIG_ARCH_DATA_VBASE +
                       MM_PGALIGNUP(ARCH_DATA_RESERVE_SIZE);
  addrenv->heapvbase = CONFIG_ARCH_HEAP_VBASE;
  addrenv->heapsize  = heapsize;

  /* Allocate the backing pages for each region */

  ret = alloc_region(addrenv->textpages, CONFIG_ARCH_TEXT_NPAGES, textsize,
                     &addrenv->ntext);
  if (ret < 0)
    {
      goto errout;
    }

  ret = alloc_region(addrenv->datapages, CONFIG_ARCH_DATA_NPAGES, datatotal,
                     &addrenv->ndata);
  if (ret < 0)
    {
      goto errout;
    }

  ret = alloc_region(addrenv->heappages, CONFIG_ARCH_HEAP_NPAGES, heapsize,
                     &addrenv->nheap);
  if (ret < 0)
    {
      goto errout;
    }

  return OK;

errout:
  up_addrenv_destroy(addrenv);
  return ret;
}

/****************************************************************************
 * Name: up_addrenv_destroy
 *
 * Description:
 *   This function is called when a task group is finally deleted.  Return
 *   all of the group's physical pages to the page pool.
 *
 ****************************************************************************/

int up_addrenv_destroy(arch_addrenv_t *addrenv)
{
  DEBUGASSERT(addrenv);

  /* If this environment is the resident one, forget it so a later select of
   * a different environment that happens to reuse this address does not take
   * the fast path by mistake.
   */

  if (addrenv == g_current_addrenv)
    {
      g_current_addrenv = NULL;
    }

  free_region(addrenv->textpages, &addrenv->ntext);
  free_region(addrenv->datapages, &addrenv->ndata);
  free_region(addrenv->heappages, &addrenv->nheap);

  memset(addrenv, 0, sizeof(arch_addrenv_t));
  return OK;
}

/****************************************************************************
 * Name: up_addrenv_vtext
 *
 * Description:
 *   Return the virtual address associated with the newly created .text
 *   address environment.
 *
 ****************************************************************************/

int up_addrenv_vtext(arch_addrenv_t *addrenv, void **vtext)
{
  DEBUGASSERT(addrenv && vtext);
  *vtext = (void *)addrenv->textvbase;
  return OK;
}

/****************************************************************************
 * Name: up_addrenv_vdata
 *
 * Description:
 *   Return the virtual address associated with the newly created .bss/.data
 *   address environment.
 *
 ****************************************************************************/

int up_addrenv_vdata(arch_addrenv_t *addrenv, uintptr_t textsize,
                     void **vdata)
{
  DEBUGASSERT(addrenv && vdata);
  *vdata = (void *)addrenv->datavbase;
  return OK;
}

/****************************************************************************
 * Name: up_addrenv_vheap
 *
 * Description:
 *   Return the heap virtual address associated with the newly created
 *   address environment.
 *
 ****************************************************************************/

int up_addrenv_vheap(const arch_addrenv_t *addrenv, void **vheap)
{
  DEBUGASSERT(addrenv && vheap);
  *vheap = (void *)addrenv->heapvbase;
  return OK;
}

/****************************************************************************
 * Name: up_addrenv_heapsize
 *
 * Description:
 *   Return the size of the initial heap allocation.
 *
 ****************************************************************************/

ssize_t up_addrenv_heapsize(const arch_addrenv_t *addrenv)
{
  DEBUGASSERT(addrenv);
  return (ssize_t)addrenv->heapsize;
}

/****************************************************************************
 * Name: up_addrenv_select
 *
 * Description:
 *   After an address environment has been established for a task group (via
 *   up_addrenv_create()), this function may be called to instantiate that
 *   address environment in the virtual address space.  On the ESP32-S3 there
 *   is no page-table-base register to load; instead the shared user
 *   cache-MMU windows (.text on the instruction bus, .data/.bss and heap on
 *   the data bus) are reprogrammed to point at this environment's PSRAM
 *   pages.  Only one user environment can be resident at a time, so while a
 *   group runs only its own pages are visible *in the windows*: the entries
 *   it uses are pointed at its pages and the rest are invalidated.
 *
 *   That is not by itself isolation between groups.  Every user page comes
 *   from the one page pool, which the kernel keeps permanently mapped at
 *   CONFIG_ARCH_PGPOOL_VBASE, and the external-memory permissions are
 *   indexed by physical address, so they cannot deny the unprivileged world
 *   that window without also denying it its own pages.  Closing that is a
 *   separate piece of work on the kernel's side of the map.
 *
 *   The remap is skipped when 'addrenv' is already the resident environment
 *   (thread<->thread within a group, ISRs, syscalls), which pays nothing.
 *
 * Input Parameters:
 *   addrenv - Describes the address environment to instantiate.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int up_addrenv_select(const arch_addrenv_t *addrenv)
{
  irqstate_t flags;
  uint32_t   cache_state;
  uint16_t   i;

  DEBUGASSERT(addrenv);

  /* Fast path: this environment is already resident */

  if (addrenv == g_current_addrenv)
    {
      return OK;
    }

  flags = enter_critical_section();

  /* Suspend the data cache while the windows are rewritten.  Instruction
   * fetches keep running from the (unchanged) flash mapping through the
   * instruction cache, so this function may execute from flash.
   */

  cache_state = esp32s3_dcache_suspend(false);

  /* Point the instruction-bus (.text) window at this group's pages */

  for (i = 0; i < addrenv->ntext; i++)
    {
      esp32s3_mmu_map_ibus(SOC_MMU_ACCESS_SPIRAM,
                           ESP32S3_TEXT_VBASE + i * MM_PGSIZE,
                           addrenv->textpages[i], 1);
    }

  /* Point the data-bus (.data/.bss, then heap) windows at this group's
   * pages
   */

  for (i = 0; i < addrenv->ndata; i++)
    {
      esp32s3_mmu_map_dbus(SOC_MMU_ACCESS_SPIRAM,
                           ESP32S3_DATA_VBASE + i * MM_PGSIZE,
                           addrenv->datapages[i], 1);
    }

  for (i = 0; i < addrenv->nheap; i++)
    {
      esp32s3_mmu_map_dbus(SOC_MMU_ACCESS_SPIRAM,
                           ESP32S3_HEAP_VBASE + i * MM_PGSIZE,
                           addrenv->heappages[i], 1);
    }

  /* Take away what this group does not use.  Only as many entries as the
   * incoming group has pages were rewritten above; the rest of each window
   * would otherwise still point at the pages of whoever was resident before,
   * which a task that ran off the end of its own allocation could read.
   *
   * A group's page count only ever grows (the heap window, through
   * pgalloc()), so this cannot invalidate an entry that is about to be
   * needed again without a select in between.
   */

  esp32s3_mmu_unmap(ESP32S3_TEXT_VBASE + addrenv->ntext * MM_PGSIZE,
                    CONFIG_ARCH_TEXT_NPAGES - addrenv->ntext);
  esp32s3_mmu_unmap(ESP32S3_DATA_VBASE + addrenv->ndata * MM_PGSIZE,
                    CONFIG_ARCH_DATA_NPAGES - addrenv->ndata);
  esp32s3_mmu_unmap(ESP32S3_HEAP_VBASE + addrenv->nheap * MM_PGSIZE,
                    CONFIG_ARCH_HEAP_NPAGES - addrenv->nheap);

  /* Drop stale instruction lines from the previous mapping and resume */

  esp32s3_icache_invalidate_all();
  esp32s3_dcache_resume(cache_state);

  g_current_addrenv = addrenv;

  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: esp32s3_addrenv_coherent
 *
 * Description:
 *   Make everything written through the data bus visible to instruction
 *   fetch: push the data cache out to PSRAM, then drop the instruction cache
 *   so the next fetch reloads from it.
 *
 *   Both halves are needed.  The write-back is needed because text arrives
 *   through the data side; the invalidate is needed because the cache-MMU is
 *   one global table, so an entry now pointing at a process's page may still
 *   have instruction-cache lines belonging to whatever it mapped before.
 *
 ****************************************************************************/

static void esp32s3_addrenv_coherent(void)
{
  irqstate_t flags = enter_critical_section();

  esp_spiram_writeback_cache();
  esp32s3_icache_invalidate_all();

  leave_critical_section(flags);
}

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
                            uintptr_t paddr)
{
  irqstate_t flags;
  uint32_t   cache_state;

  DEBUGASSERT(addrenv);

  if (addrenv != g_current_addrenv)
    {
      return;
    }

  flags = enter_critical_section();

  cache_state = esp32s3_dcache_suspend(false);
  esp32s3_mmu_map_dbus(SOC_MMU_ACCESS_SPIRAM, vaddr, paddr, 1);
  esp32s3_dcache_resume(cache_state);

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: up_addrenv_coherent
 *
 * Description:
 *   Flush D-Cache and invalidate I-Cache in preparation for a change in
 *   address environments.
 *
 *   This matters more here than the name suggests, because it is the point
 *   at which a freshly loaded program becomes executable.  The ELF loader
 *   writes .text as *data*, so it lands in the data cache; instructions are
 *   fetched through the separate instruction cache.  Without a writeback the
 *   PSRAM pages still hold whatever was there before, and the new process
 *   runs a mixture of its own code and stale bytes -- which shows up as an
 *   illegal instruction somewhere in the middle of a function, not at its
 *   entry point.  up_addrenv_select() cannot cover this: it skips all cache
 *   maintenance when the environment it is handed is already resident, which
 *   is exactly the case on the way out of the loader.
 *
 ****************************************************************************/

int up_addrenv_coherent(const arch_addrenv_t *addrenv)
{
  DEBUGASSERT(addrenv);

  esp32s3_addrenv_coherent();
  return OK;
}

/****************************************************************************
 * Name: up_addrenv_mprot
 *
 * Description:
 *   Modify access rights to an address range.  The ELF loader uses this to
 *   make .text writable while it copies the program in, and read-only again
 *   afterwards.
 *
 *   The ESP32-S3 cannot honour that.  A cache-MMU entry carries only a valid
 *   bit, a memory type and a physical page number -- there are no per-page
 *   permission bits -- and the coarse alternatives cannot express it either:
 *   the PMS areas gate a whole window per world rather than a page, and they
 *   report violations asynchronously.  A mapped user page is therefore
 *   always readable and writable by its owner, so a process can write to its
 *   own .text.  This does not weaken isolation between groups, which comes
 *   from the window remap in up_addrenv_select(), and the request is
 *   accepted so that the loader can proceed.
 *
 ****************************************************************************/

int up_addrenv_mprot(arch_addrenv_t *addrenv, uintptr_t addr, size_t len,
                     int prot)
{
  UNUSED(addrenv);
  UNUSED(addr);
  UNUSED(len);
  UNUSED(prot);

  /* The permission change cannot be honoured, but the call still marks the
   * two moments that matter for cache state: the loader asks for write
   * access before it copies a program in, and takes it away again once the
   * program is complete.  Make both a synchronisation point.
   *
   * This is what makes a freshly loaded program executable.  Text is written
   * as data through the data-bus alias of the same cache-MMU entry, so it
   * sits in the data cache, while instructions are fetched through the
   * separate instruction cache -- which may still hold lines from whatever
   * that entry mapped before, since one global table is shared by every
   * process and by the kernel's own PSRAM window.  Those stale lines read
   * back as zeroes, so without this the process runs some of its own code
   * and then executes a hole.
   */

  esp32s3_addrenv_coherent();
  return OK;
}

/****************************************************************************
 * Name: up_addrenv_clone
 *
 * Description:
 *   Duplicate an address environment.  The threads of a task group share one
 *   address environment, so cloning it is simply copying the descriptor: the
 *   copy references the same PSRAM pages.  (A true fork() that gives the
 *   child its own pages is a separate, higher-level operation built on
 *   up_addrenv_create() + a content copy.)
 *
 ****************************************************************************/

int up_addrenv_clone(const arch_addrenv_t *src, arch_addrenv_t *dest)
{
  DEBUGASSERT(src && dest);
  memcpy(dest, src, sizeof(arch_addrenv_t));
  return OK;
}

/****************************************************************************
 * Name: copy_region
 *
 * Description:
 *   Copy one region's pages from a parent environment into a child's.  Both
 *   sides are reached through the kernel's scratch region, which is why it
 *   has two slots: source and destination are mapped at the same time so
 *   this is one memcpy rather than a bounce through kernel memory.
 *
 *   sched_lock() is held across each page pair for the reason every scratch
 *   mapping is: those addresses are ordinary external memory to the
 *   permission control, so no unprivileged task may run while a page of
 *   somebody's memory is parked at one.
 *
 ****************************************************************************/

static int copy_region(const uintptr_t *src, uintptr_t *dest, uint16_t count)
{
  uint16_t i;

  for (i = 0; i < count; i++)
    {
      uintptr_t svaddr;
      uintptr_t dvaddr;

      sched_lock();

      svaddr = esp32s3_pgmap(src[i]);
      dvaddr = esp32s3_pgmap(dest[i]);

      if (svaddr == 0 || dvaddr == 0)
        {
          if (svaddr != 0)
            {
              esp32s3_pgunmap(svaddr);
            }

          if (dvaddr != 0)
            {
              esp32s3_pgunmap(dvaddr);
            }

          sched_unlock();
          berr("ERROR: no scratch mapping for page %u\n", i);
          return -EFAULT;
        }

      memcpy((void *)dvaddr, (const void *)svaddr, MM_PGSIZE);

      esp32s3_pgunmap(dvaddr);
      esp32s3_pgunmap(svaddr);

      sched_unlock();
    }

  return OK;
}

/****************************************************************************
 * Name: up_addrenv_fork
 *
 * Description:
 *   Duplicate an address environment for fork():  allocate the child pages
 *   to match the parent's regions and copy the parent's contents into them.
 *
 *   The copy is eager and complete.  There is no copy-on-write and no demand
 *   fill, because this chip provides no synchronous restartable write fault
 *   to build them on -- proven, not assumed.  So a fork costs a full copy of
 *   the process image.
 *
 *   The child's pages land at the same *virtual* addresses as the parent's,
 *   which is the property the whole thing rests on: a copied stack is full
 *   of pointers into itself, and they are only still correct because the
 *   copy is addressed identically.
 *
 ****************************************************************************/

int up_addrenv_fork(const arch_addrenv_t *src, arch_addrenv_t *dest)
{
  int ret;

  DEBUGASSERT(src && dest);

  memset(dest, 0, sizeof(arch_addrenv_t));

  dest->textvbase = src->textvbase;
  dest->datavbase = src->datavbase;
  dest->heapvbase = src->heapvbase;
  dest->heapsize  = src->heapsize;

  /* Allocate the child's pages.  alloc_region() takes a size, and the
   * parent's page counts are the exact sizes wanted.
   */

  ret = alloc_region(dest->textpages, CONFIG_ARCH_TEXT_NPAGES,
                     (size_t)src->ntext * MM_PGSIZE, &dest->ntext);
  if (ret < 0)
    {
      goto errout;
    }

  ret = alloc_region(dest->datapages, CONFIG_ARCH_DATA_NPAGES,
                     (size_t)src->ndata * MM_PGSIZE, &dest->ndata);
  if (ret < 0)
    {
      goto errout;
    }

  ret = alloc_region(dest->heappages, CONFIG_ARCH_HEAP_NPAGES,
                     (size_t)src->nheap * MM_PGSIZE, &dest->nheap);
  if (ret < 0)
    {
      goto errout;
    }

  /* Then fill them from the parent */

  ret = copy_region(src->textpages, dest->textpages, dest->ntext);
  if (ret < 0)
    {
      goto errout;
    }

  ret = copy_region(src->datapages, dest->datapages, dest->ndata);
  if (ret < 0)
    {
      goto errout;
    }

  ret = copy_region(src->heappages, dest->heappages, dest->nheap);
  if (ret < 0)
    {
      goto errout;
    }

  /* The text pages were written through the data bus.  Make them visible to
   * instruction fetch before anything runs from them.
   */

  esp32s3_addrenv_coherent();
  return OK;

errout:
  up_addrenv_destroy(dest);
  return ret;
}

/****************************************************************************
 * Name: up_addrenv_attach
 *
 * Description:
 *   Called when a task or thread is created in order to instantiate an
 *   address environment.  On the ESP32-S3 the group's environment is made
 *   resident lazily by up_addrenv_select() at context-switch time, so there
 *   is nothing to do here.
 *
 ****************************************************************************/

int up_addrenv_attach(struct tcb_s *ptcb, struct tcb_s *tcb)
{
  return OK;
}

/****************************************************************************
 * Name: up_addrenv_detach
 *
 * Description:
 *   Called when a task or thread exits.  The group's pages are released by
 *   up_addrenv_destroy() when the final member leaves, so there is nothing
 *   per-thread to undo here.
 *
 ****************************************************************************/

int up_addrenv_detach(struct tcb_s *tcb)
{
  return OK;
}

#endif /* CONFIG_ARCH_ADDRENV */
