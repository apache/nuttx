/****************************************************************************
 * mm/kmap/kmm_map.c
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

#include <stdint.h>
#include <stddef.h>

#include <nuttx/addrenv.h>
#include <nuttx/arch.h>
#include <nuttx/compiler.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mm/gran.h>
#include <nuttx/mm/map.h>
#include <nuttx/pgalloc.h>
#include <nuttx/sched.h>

#include <sys/mman.h>

#if defined(CONFIG_BUILD_KERNEL)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static GRAN_HANDLE     g_kmm_map_vpages;
static struct mm_map_s g_kmm_map;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: get_user_pages
 *
 * Description:
 *   Get the physical pages mapped to a user virtual memory region.
 *
 * Input Parameters:
 *   pages  - Pointer to buffer where the page addresses are recorded.
 *   npages - Amount of pages.
 *   vaddr  - Start address of the user virtual memory region.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

static int get_user_pages(FAR void **pages, size_t npages, uintptr_t vaddr)
{
  FAR struct tcb_s *tcb = nxsched_self();
  uintptr_t         page;
  int               i;

  /* Find the pages associated with the user virtual address space */

  for (i = 0; i < npages; i++, vaddr += MM_PGSIZE)
    {
      page = up_addrenv_find_page(&tcb->addrenv_own->addrenv, vaddr);
      if (!page)
        {
          /* Something went wrong, get out */

          return -EINVAL;
        }

      pages[i] = (FAR void *)page;
    }

  return OK;
}

/****************************************************************************
 * Name: map_pages
 *
 * Description:
 *   Map pages into kernel virtual memory.
 *
 * Input Parameters:
 *   pages  - Pointer to buffer that contains the physical page addresses.
 *   npages - Amount of pages.
 *   prot   - Access right flags.
 *
 * Returned Value:
 *   Pointer to the mapped virtual memory on success; NULL on failure
 *
 ****************************************************************************/

static FAR void *map_pages(FAR void **pages, size_t npages, int prot)
{
  struct mm_map_entry_s entry;
  FAR void             *vaddr;
  size_t                size;
  int                   ret;

  /* The region size is full pages */

  size = npages << MM_PGSHIFT;

  /* Find a virtual memory area that fits */

  vaddr = gran_alloc(&g_kmm_map_vpages, size);
  if (!vaddr)
    {
      return NULL;
    }

  /* Map the pages into the kernel page directory */

  ret = up_addrenv_kmap_pages(pages, npages, (uintptr_t)vaddr, prot);
  if (ret < 0)
    {
      goto errout_with_vaddr;
    }

  entry.vaddr = vaddr;
  entry.length = size;
  entry.offset = 0;
  entry.munmap = NULL;

  ret = mm_map_add(&g_kmm_map, &entry);
  if (ret < 0)
    {
      goto errout_with_pgmap;
    }

  return vaddr;

errout_with_pgmap:
  up_addrenv_kunmap_pages((uintptr_t)vaddr, npages);
errout_with_vaddr:
  gran_free(&g_kmm_map_vpages, vaddr, size);
  return NULL;
}

/****************************************************************************
 * Name: map_single_user_page
 *
 * Description:
 *   Map a single user page into kernel memory.
 *
 * Input Parameters:
 *   pages  - Pointer to buffer that contains the physical page addresses.
 *   npages - Amount of pages.
 *   prot   - Access right flags.
 *
 * Returned Value:
 *   Pointer to the mapped virtual memory on success; NULL on failure
 *
 ****************************************************************************/

static FAR void *map_single_user_page(uintptr_t vaddr)
{
  FAR struct tcb_s *tcb = nxsched_self();
  uintptr_t         page;

  /* Find the page associated with this virtual address */

  page = up_addrenv_find_page(&tcb->addrenv_own->addrenv, vaddr);
  if (!page)
    {
      return NULL;
    }

  vaddr = up_addrenv_page_vaddr(page);
  return (FAR void *)vaddr;
}

/****************************************************************************
 * Name: kmm_map_lock
 *
 * Description:
 *   Get exclusive access to the kernel mm_map
 *
 ****************************************************************************/

static int kmm_map_lock(void)
{
  return nxrmutex_lock(&g_kmm_map.mm_map_mutex);
}

/****************************************************************************
 * Name: kmm_map_unlock
 *
 * Description:
 *   Relinquish exclusive access to the kernel mm_map
 *
 ****************************************************************************/

static void kmm_map_unlock(void)
{
  DEBUGVERIFY(nxrmutex_unlock(&g_kmm_map.mm_map_mutex));
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: kmm_map_initialize
 *
 * Description:
 *   Initialize the kernel dynamic mapping module.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void kmm_map_initialize(void)
{
  g_kmm_map_vpages = gran_initialize((FAR void *)CONFIG_ARCH_KMAP_VBASE,
                                     CONFIG_ARCH_KMAP_NPAGES << MM_PGSHIFT,
                                     MM_PGSHIFT, MM_PGSHIFT);
  DEBUGVERIFY(g_kmm_map_vpages);
  mm_map_initialize(&g_kmm_map, true);
}

/****************************************************************************
 * Name: kmm_map_pages
 *
 * Description:
 *   Map pages into kernel virtual memory.
 *
 * Input Parameters:
 *   pages  - Pointer to buffer that contains the physical page addresses.
 *   npages - Amount of pages.
 *   prot   - Access right flags.
 *
 * Returned Value:
 *   Pointer to the mapped virtual memory on success; NULL on failure
 *
 ****************************************************************************/

FAR void *kmm_map(FAR void **pages, size_t npages, int prot)
{
  uintptr_t vaddr;

  if (!pages || !npages || npages > CONFIG_ARCH_KMAP_NPAGES)
    {
      return NULL;
    }

  /* Attempt to map the pages */

  vaddr = (uintptr_t)map_pages(pages, npages, prot);
  if (!vaddr)
    {
      return NULL;
    }

  return (FAR void *)vaddr;
}

/****************************************************************************
 * Name: kmm_unmap
 *
 * Description:
 *   Unmap a previously allocated kernel virtual memory area.
 *
 * Input Parameters:
 *   kaddr - The kernel virtual address where the mapping begins.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void kmm_unmap(FAR void *kaddr)
{
  FAR struct mm_map_entry_s *entry;
  unsigned int               npages;
  int                        ret;

  /* Lock the mapping list when we fiddle around with it */

  ret = kmm_map_lock();
  if (ret == OK)
    {
      /* Find the entry, it is OK if none found */

      entry = mm_map_find(get_current_mm(), kaddr, 1);
      if (entry)
        {
          npages = MM_NPAGES(entry->length);

          /* Remove the mappings from the page directory */

          up_addrenv_kunmap_pages((uintptr_t)entry->vaddr, npages);

          /* Release the virtual memory area for use */

          gran_free(&g_kmm_map_vpages, entry->vaddr, entry->length);

          /* Remove the mapping from the kernel mapping list */

          mm_map_remove(&g_kmm_map, entry);
        }

      kmm_map_unlock();
    }
}

/****************************************************************************
 * Name: kmm_user_map
 *
 * Description:
 *   Map a region of user memory (physical pages) for kernel use through
 *   a continuous virtual memory area.
 *
 * Input Parameters:
 *   uaddr - The user virtual address where mapping begins.
 *   size  - Size of the region.
 *
 * Returned Value:
 *   Pointer to the virtual memory area, or NULL if out of memory.
 *
 ****************************************************************************/

FAR void *kmm_user_map(FAR void *uaddr, size_t size)
{
  FAR void *pages;
  uintptr_t vaddr;
  uintptr_t offset;
  size_t    npages;
  int       ret;

  /* Find the kernel addressable virtual address, or map the user memory */

  vaddr = (uintptr_t)uaddr;

  /* If the memory is not user memory, get out */

  if (!up_addrenv_user_vaddr(vaddr))
    {
      /* Not user memory, get out */

      return uaddr;
    }

  /* How many pages (including partial) does the space encompass ? */

  offset = vaddr & MM_PGMASK;
  vaddr  = MM_PGALIGNDOWN(vaddr);
  npages = MM_NPAGES(offset + size);

  /* Does the area fit in 1 page, including page boundary crossings ? */

  if (npages == 1)
    {
      /* Yes, can simply return the kernel addressable virtual address */

      vaddr = (uintptr_t)map_single_user_page(vaddr);
      return (FAR void *)(vaddr + offset);
    }

  /* No, the area must be mapped into kernel virtual address space */

  pages = kmm_zalloc(npages * sizeof(FAR void *));
  if (!pages)
    {
      return NULL;
    }

  /* Fetch the physical pages for the user virtual address range */

  ret = get_user_pages(&pages, npages, vaddr);
  if (ret < 0)
    {
      goto errout_with_pages;
    }

  /* Map the physical pages to kernel memory */

  vaddr = (uintptr_t)map_pages(&pages, npages, PROT_READ | PROT_WRITE);
  if (!vaddr)
    {
      goto errout_with_pages;
    }

  /* Ok, we have a virtual memory area, add the offset back */

  return (FAR void *)(vaddr + offset);

errout_with_pages:
  kmm_free(pages);
  return NULL;
}

/****************************************************************************
 * Name: kmm_map_user_page
 *
 * Description:
 *   Map a single physical page into kernel virtual memory. Typically just
 *   returns the kernel addressable page pool virtual address.
 *
 * Input Parameters:
 *   uaddr - The virtual address of the user page.
 *
 * Returned Value:
 *   Pointer to the new address environment, or NULL if out of memory.
 *
 ****************************************************************************/

FAR void *kmm_map_user_page(FAR void *uaddr)
{
  uintptr_t vaddr;
  uintptr_t offset;

  /* Find the kernel addressable virtual address, or map the user memory */

  vaddr = (uintptr_t)uaddr;

  /* If the memory is not user memory, get out */

  if (!up_addrenv_user_vaddr(vaddr))
    {
      /* Not user memory, get out */

      return uaddr;
    }

  /* Record the offset and add it back later */

  offset = vaddr & MM_PGMASK;
  vaddr = MM_PGALIGNDOWN(vaddr);

  vaddr = (uintptr_t)map_single_user_page(vaddr);
  if (!vaddr)
    {
      return NULL;
    }

  return (FAR void *)(vaddr + offset);
}

#endif
