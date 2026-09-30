/****************************************************************************
 * fs/mmap/fs_anonmap.c
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
#include <nuttx/fs/fs.h>
#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mm/map.h>
#include <nuttx/pgalloc.h>
#include <nuttx/sched.h>
#include <assert.h>
#include <nuttx/debug.h>

#include "sched/sched.h"
#include "fs_heap.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: unmap_anonymous
 ****************************************************************************/

static int unmap_anonymous(FAR struct task_group_s *group,
                           FAR struct mm_map_entry_s *entry,
                           FAR void *start,
                           size_t length)
{
  FAR void *newaddr = NULL;
  off_t offset;
  bool kernel = entry->priv.i;
  int ret = OK;

  /* Get the offset from the beginning of the region and the actual number
   * of bytes to "unmap".  All mappings must extend to the end of the region.
   * There is no support for freeing a block of memory but leaving a block of
   * memory at the end.  This is a consequence of using kumm_realloc() to
   * simulate the unmapping.
   */

  offset = (uintptr_t)start - (uintptr_t)entry->vaddr;
  if (offset + length < entry->length)
    {
      ferr("ERROR: Cannot umap without unmapping to the end\n");
      return -ENOSYS;
    }

  /* Okay.. the region is being unmapped to the end.  Make sure the length
   * indicates that.
   */

  length = entry->length - offset;

  /* Are we unmapping the entire region (offset == 0)? */

  if (length >= entry->length)
    {
      /* Free the region */

      if (kernel)
        {
          fs_heap_free(entry->vaddr);
        }
      else
        {
          kumm_free(entry->vaddr);
        }

      /* Then remove the mapping from the list */

      ret = mm_map_remove(get_group_mm(group), entry);
    }

  /* No.. We have been asked to "unmap' only a portion of the memory
   * (offset > 0).
   */

  else
    {
      if (kernel)
        {
          newaddr = fs_heap_realloc(entry->vaddr, length);
        }
      else
        {
          newaddr = kumm_realloc(entry->vaddr, length);
        }

      DEBUGASSERT(newaddr == entry->vaddr);
      entry->vaddr = newaddr;
      entry->length = length;
    }

  return ret;
}

#if defined(CONFIG_BUILD_KERNEL) && defined(CONFIG_ARCH_VMA_MAPPING)
static int unmap_anonymous_pages(FAR struct task_group_s *group,
                                 FAR struct mm_map_entry_s *entry,
                                 FAR void *start,
                                 size_t length)
{
  FAR uintptr_t *pages = entry->priv.p;
  size_t npages = MM_NPAGES(entry->length);
  size_t first;
  size_t i;

  if (!MM_ISALIGNED(start) || start < entry->vaddr)
    {
      return -EINVAL;
    }

  first = ((uintptr_t)start - (uintptr_t)entry->vaddr) >> MM_PGSHIFT;
  if (first + MM_NPAGES(length) < npages)
    {
      ferr("ERROR: Cannot umap without unmapping to the end\n");
      return -ENOSYS;
    }

  if (group != NULL)
    {
      up_shmdt((uintptr_t)start, npages - first);
      vm_release_region(get_group_mm(group), start,
                        (npages - first) << MM_PGSHIFT);
    }

  for (i = first; i < npages; i++)
    {
      mm_pgfree(pages[i], 1);
    }

  if (first > 0)
    {
      entry->length = first << MM_PGSHIFT;
      return OK;
    }

  kmm_free(pages);
  return group != NULL ? mm_map_remove(get_group_mm(group), entry) : OK;
}

static int map_anonymous_pages(FAR struct mm_map_entry_s *entry)
{
  FAR struct mm_map_s *mm = get_current_mm();
  size_t npages = MM_NPAGES(entry->length);
  FAR uintptr_t *pages;
  size_t i;
  int ret = -ENOMEM;

  pages = kmm_zalloc(npages * sizeof(*pages));
  if (pages == NULL)
    {
      return -ENOMEM;
    }

  for (i = 0; i < npages; i++)
    {
      pages[i] = mm_pgalloc(1);
      if (pages[i] == 0)
        {
          goto errout;
        }

      up_addrenv_page_wipe(pages[i]);
    }

  entry->length = npages << MM_PGSHIFT;
  entry->vaddr  = vm_alloc_region(mm, NULL, entry->length);
  if (entry->vaddr == NULL)
    {
      goto errout;
    }

  ret = up_shmat(pages, npages, (uintptr_t)entry->vaddr);
  if (ret >= 0)
    {
      entry->munmap = unmap_anonymous_pages;
      entry->priv.p = pages;
      ret = mm_map_add(mm, entry);
      if (ret >= 0)
        {
          return ret;
        }

      up_shmdt((uintptr_t)entry->vaddr, npages);
    }

  vm_release_region(mm, entry->vaddr, entry->length);
  entry->vaddr = NULL;

errout:
  while (i-- > 0)
    {
      mm_pgfree(pages[i], 1);
    }

  kmm_free(pages);
  return ret;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int map_anonymous(FAR struct mm_map_entry_s *entry, bool kernel)
{
  int ret;

#if defined(CONFIG_BUILD_KERNEL) && defined(CONFIG_ARCH_VMA_MAPPING)
  if (!kernel)
    {
      return map_anonymous_pages(entry);
    }
#endif

  entry->vaddr = kernel ?
    fs_heap_zalloc(entry->length) : kumm_zalloc(entry->length);
  if (entry->vaddr == NULL)
    {
      ferr("ERROR: kumm_alloc() failed, enable DEBUG_MM for info!\n");
      return -ENOMEM;
    }

  entry->munmap = unmap_anonymous;
  entry->priv.i = kernel;

  ret = mm_map_add(get_current_mm(), entry);
  if (ret < 0)
    {
      if (kernel)
        {
          fs_heap_free(entry->vaddr);
        }
      else
        {
          kumm_free(entry->vaddr);
        }

      entry->vaddr = NULL;
    }

  return ret;
}
