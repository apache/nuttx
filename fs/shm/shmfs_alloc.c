/****************************************************************************
 * fs/shm/shmfs_alloc.c
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

#include <stdbool.h>
#include <nuttx/kmalloc.h>
#include <nuttx/pgalloc.h>

#include "shm/shmfs.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

FAR struct shmfs_object_s *shmfs_alloc_object(size_t length)
{
  FAR struct shmfs_object_s *object;
  bool allocated = false;

#if defined(CONFIG_BUILD_FLAT)
  /* in FLAT build, allocate the object metadata and the data in the same
   * chunk in kernel heap
   */

  object = kmm_zalloc(sizeof(struct shmfs_object_s) + length);
  if (object)
    {
      object->paddr = (FAR char *)(object + 1);
      allocated = true;
    }

#elif defined(CONFIG_BUILD_PROTECTED)
  /* in PROTECTED build, allocate the shm object in kernel heap, and shared
   * memory in user heap
   */

  object = kmm_zalloc(sizeof(struct shmfs_object_s));
  if (object)
    {
      object->paddr = kumm_zalloc(length);

      if (object->paddr)
        {
           allocated = true;
        }
    }

#elif defined(CONFIG_BUILD_KERNEL)
  /* in KERNEL build, allocate the shared memory from page pool and store the
   * physical address
   */

  size_t i = 0;
  FAR void **pages;
  size_t n_pages = MM_NPAGES(length);

  object = kmm_zalloc(sizeof(struct shmfs_object_s) +
                      (n_pages - 1) * sizeof(object->paddr));

  if (object)
    {
      pages = &object->paddr;
      for (; i < n_pages; i++)
        {
          pages[i] = (FAR void *)mm_pgalloc(1);
          if (!pages[i])
            {
              break;
            }
        }
    }

  if (i == n_pages)
    {
      allocated = true;
    }
#endif

  if (allocated)
    {
      object->length = length;
    }
  else
    {
      /* delete any partial allocation */

      shmfs_free_object(object);
      object = NULL;
    }

  return object;
}

void shmfs_free_object(FAR struct shmfs_object_s *object)
{
#if defined(CONFIG_BUILD_KERNEL)
  size_t i;
  size_t n_pages = MM_NPAGES(object->length);
  FAR void **pages;
#endif

  if (object)
    {
#if defined (CONFIG_BUILD_PROTECTED)
      kumm_free(object->paddr);
#elif defined(CONFIG_BUILD_KERNEL)
      pages = &object->paddr;
      for (i = 0; i < n_pages; i++)
        {
          if (pages[i])
            {
              mm_pgfree((uintptr_t)pages[i], 1);
            }
        }
#endif

      /* Delete the object metadata
       * (and the shared memory in case of FLAT build)
       */

      kmm_free(object);
    }
}
