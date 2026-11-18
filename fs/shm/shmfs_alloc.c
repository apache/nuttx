/****************************************************************************
 * fs/shm/shmfs_alloc.c
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

#include <stdbool.h>
#include <string.h>
#include <unistd.h>

#include <nuttx/arch.h>
#include <nuttx/cache.h>
#include <nuttx/nuttx.h>
#include <nuttx/kmalloc.h>
#include <nuttx/pgalloc.h>

#include "shm/shmfs.h"
#include "fs_heap.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

FAR struct shmfs_object_s *shmfs_alloc_object(size_t length)
{
  FAR struct shmfs_object_s *object;

  object = fs_heap_zalloc(sizeof(struct shmfs_object_s));
  if (object == NULL)
    {
      return NULL;
    }

  nxmutex_init(&object->lock);

  if (length > 0)
    {
      object->paddr = shmfs_alloc_buffer(length);
      if (object->paddr == NULL)
        {
          nxmutex_destroy(&object->lock);
          fs_heap_free(object);
          return NULL;
        }
    }

  object->length = length;
  return object;
}

FAR void *shmfs_alloc_buffer(size_t length)
{
  FAR void *paddr = NULL;

#if defined(CONFIG_BUILD_KERNEL)
  /* In KERNEL build, allocate the shared memory from the page pool and
   * keep the page addresses in a separate vector.
   */

  size_t n_pages = MM_NPAGES(length);
  FAR void **pages;
  size_t i;

  pages = fs_heap_zalloc(n_pages * sizeof(FAR void *));
  if (pages == NULL)
    {
      return NULL;
    }

  for (i = 0; i < n_pages; i++)
    {
      pages[i] = (FAR void *)mm_pgalloc(1);
      if (pages[i] == NULL)
        {
          break;
        }

      /* Clear the page memory (requirement for truncate) */

      up_addrenv_page_wipe((uintptr_t)pages[i]);
    }

  if (i < n_pages)
    {
      /* Delete any partial allocation */

      while (i-- > 0)
        {
          mm_pgfree((uintptr_t)pages[i], 1);
        }

      fs_heap_free(pages);
      return NULL;
    }

  paddr = pages;
#else
  /* In FLAT build the shared memory comes from the kernel heap, in
   * PROTECTED build from the user heap.  mmap() hands out this buffer
   * directly: round it up to a page so that a page-sized mapping of a
   * smaller object stays inside it.
   */

  size_t alloc_size = ALIGN_UP(length, sysconf(_SC_PAGESIZE));
#  ifdef CONFIG_FS_SHMFS_NO_ALIGN
  size_t cachesize = 0;
#  else
  size_t cachesize = up_get_dcache_linesize();
#  endif

  if (cachesize > 0)
    {
      alloc_size = ALIGN_UP(alloc_size, cachesize);
#  ifdef CONFIG_BUILD_FLAT
      paddr = fs_heap_memalign(cachesize, alloc_size);
#  else
      paddr = kumm_memalign(cachesize, alloc_size);
#  endif
    }
  else
    {
#  ifdef CONFIG_BUILD_FLAT
      paddr = fs_heap_malloc(alloc_size);
#  else
      paddr = kumm_malloc(alloc_size);
#  endif
    }

  if (paddr != NULL)
    {
      memset(paddr, 0, alloc_size);
    }
#endif

  return paddr;
}

void shmfs_free_object(FAR struct shmfs_object_s *object)
{
  if (object != NULL)
    {
      shmfs_free_buffer(object);
      nxmutex_destroy(&object->lock);
      fs_heap_free(object);
    }
}

void shmfs_free_buffer(FAR struct shmfs_object_s *object)
{
  if (object->paddr != NULL)
    {
#if defined(CONFIG_BUILD_KERNEL)
      FAR void **pages = object->paddr;
      size_t n_pages = MM_NPAGES(object->length);
      size_t i;

      for (i = 0; i < n_pages; i++)
        {
          if (pages[i] != NULL)
            {
              mm_pgfree((uintptr_t)pages[i], 1);
            }
        }

      fs_heap_free(pages);
#elif defined(CONFIG_BUILD_PROTECTED)
      kumm_free(object->paddr);
#else
      fs_heap_free(object->paddr);
#endif

      object->paddr = NULL;
    }

  object->length = 0;
}
