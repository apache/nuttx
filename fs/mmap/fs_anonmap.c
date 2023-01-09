/****************************************************************************
 * fs/mmap/fs_anonmap.c
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
#include <nuttx/kmalloc.h>
#include <debug.h>

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
  int ret;

  /* De-allocate memory.
   * NB: This is incomplete anounymous mapping implementation
   * see file_mmap_ below
   */

  if (start == entry->vaddr && length == entry->length)
    {
      /* entry->priv marks allocation from kernel heap */

      if (entry->priv.i)
        {
          kmm_free(start);
        }
      else
        {
          kumm_free(start);
        }

      ret = mm_map_remove(get_group_mm(group), entry);
    }
  else
    {
      ret = -EINVAL;
      ferr("ERROR: Unknown map type\n");
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int map_anonymous(FAR struct mm_map_entry_s *entry, bool kernel)
{
  int ret;

  /* REVISIT:  Should reside outside of the heap.  That is really the
   * only purpose of MAP_ANONYMOUS:  To get non-heap memory.  In KERNEL
   * build, this could be accomplished using pgalloc(), provided that
   * you had logic in place to assign a virtual address to the mapping.
   */

  entry->vaddr = kernel ?
    kmm_zalloc(entry->length) : kumm_zalloc(entry->length);
  if (entry->vaddr == NULL)
    {
      ferr("ERROR: kumm_alloc() failed, enable DEBUG_MM for info!\n");
      return -ENOMEM;
    }

  entry->munmap = unmap_anonymous;
  entry->priv.i = kernel;

  ret = mm_map_add(entry);
  if (ret < 0)
    {
      if (kernel)
        {
          kmm_free(entry->vaddr);
        }
      else
        {
          kumm_free(entry->vaddr);
        }

      entry->vaddr = NULL;
    }

  return ret;
}
