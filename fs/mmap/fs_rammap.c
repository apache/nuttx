/****************************************************************************
 * fs/mmap/fs_rammap.c
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
#include <sys/types.h>

#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/fs/fs.h>
#include <nuttx/kmalloc.h>

#include "fs_rammap.h"

#ifdef CONFIG_FS_RAMMAP

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int unmap_rammap(FAR struct task_group_s *group,
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

  offset = start - entry->vaddr;
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

/****************************************************************************
 * Name: rammmap
 *
 * Description:
 *   Support simulation of memory mapped files by copying files into RAM.
 *
 * Input Parameters:
 *   filep   file descriptor of the backing file -- required.
 *   length  The length of the mapping.  For exception #1 above, this length
 *           ignored:  The entire underlying media is always accessible.
 *   offset  The offset into the file to map
 *   kernel  kmm_zalloc or kumm_zalloc
 *   mapped  The pointer to the mapped area
 *
 * Returned Value:
 *  On success, rammmap returns 0. Otherwise errno is returned appropriately.
 *
 *     EBADF
 *      'fd' is not a valid file descriptor.
 *     EINVAL
 *       'length' or 'offset' are invalid
 *     ENOMEM
 *       Insufficient memory is available to map the file.
 *
 ****************************************************************************/

int rammap(FAR struct file *filep, FAR struct mm_map_entry_s *entry,
           bool kernel)
{
  FAR uint8_t *rdbuffer;
  ssize_t nread;
  off_t fpos;
  int ret;
  size_t length = entry->length;

  /* There is a major design flaw that I have not yet thought of fix for:
   * The goal is to have a single region of memory that represents a single
   * file and can be shared by many threads.  That is, given a filename a
   * thread should be able to open the file, get a file descriptor, and
   * call mmap() to get a memory region.  Different file descriptors opened
   * with the same file path should get the same memory region when mapped.
   *
   * The design flaw is that I don't have sufficient knowledge to know that
   * these different file descriptors map to the same file.  So, for the time
   * being, a new memory region is created each time that rammap() is called.
   * Not very useful!
   */

  /* Allocate a region of memory of the specified size */

  rdbuffer = kernel ? kmm_malloc(length) : kumm_malloc(length);
  if (!rdbuffer)
    {
      ferr("ERROR: Region allocation failed, length: %zu\n", length);
      return -ENOMEM;
    }

  /* Seek to the specified file offset */

  fpos = file_seek(filep, entry->offset, SEEK_SET);
  if (fpos < 0)
    {
      /* Seek failed... errno has already been set, but EINVAL is probably
       * the correct response.
       */

      ferr("ERROR: Seek to position %zu failed\n", (size_t)entry->offset);
      ret = fpos;
      goto errout_with_region;
    }

  /* Read the file data into the memory region */

  while (length > 0)
    {
      nread = file_read(filep, rdbuffer, length);
      if (nread < 0)
        {
          /* Handle the special case where the read was interrupted by a
           * signal.
           */

          if (nread != -EINTR)
            {
              /* All other read errors are bad. */

              ferr("ERROR: Read failed: offset=%zu ret=%zd\n",
                   (size_t)entry->offset, nread);

              ret = nread;
              goto errout_with_region;
            }
        }

      /* Check for end of file. */

      if (nread == 0)
        {
          break;
        }

      /* Increment number of bytes read */

      rdbuffer += nread;
      length   -= nread;
    }

  /* Zero any memory beyond the amount read from the file */

  memset(rdbuffer, 0, length);

  /* Add the buffer to the list of regions */

  entry->vaddr = rdbuffer;
  entry->priv.i = kernel;
  entry->munmap = unmap_rammap;

  ret = mm_map_add(entry);
  if (ret < 0)
    {
      goto errout_with_region;
    }

  return OK;

errout_with_region:
  if (kernel)
    {
      kmm_free(rdbuffer);
    }
  else
    {
      kumm_free(rdbuffer);
    }

  return ret;
}

#endif /* CONFIG_FS_RAMMAP */
