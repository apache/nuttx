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
#include <nuttx/sched.h>
#include <assert.h>
#include <debug.h>

#include "fs_anonmap.h"

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
  FAR void *newaddr;
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
          kmm_free(entry->vaddr);
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
          newaddr = kmm_realloc(entry->vaddr, length);
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

  ret = mm_map_add(get_current_mm(), entry);
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
