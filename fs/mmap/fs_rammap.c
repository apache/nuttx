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
#include <sys/mman.h>

#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/fs/fs.h>
#include <nuttx/kmalloc.h>

#include "inode/inode.h"
#include "fs_rammap.h"

#ifdef CONFIG_FS_RAMMAP

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* This is the list of all mapped files */

struct fs_allmaps_s g_rammaps =
{
  NXMUTEX_INITIALIZER
};

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

int rammap(FAR struct file *filep, size_t length,
           off_t offset, bool kernel, FAR void **mapped)
{
  FAR struct fs_rammap_s *map;
  FAR uint8_t *alloc;
  FAR uint8_t *rdbuffer;
  ssize_t nread;
  off_t fpos;
  int ret;

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

  alloc = kernel ?
    kmm_malloc(sizeof(struct fs_rammap_s) + length) :
    kumm_malloc(sizeof(struct fs_rammap_s) + length);
  if (!alloc)
    {
      ferr("ERROR: Region allocation failed, length: %d\n", (int)length);
      return -ENOMEM;
    }

  /* Initialize the region */

  map         = (FAR struct fs_rammap_s *)alloc;
  memset(map, 0, sizeof(struct fs_rammap_s));
  map->addr   = alloc + sizeof(struct fs_rammap_s);
  map->length = length;
  map->offset = offset;

  /* Seek to the specified file offset */

  fpos = file_seek(filep, offset, SEEK_SET);
  if (fpos < 0)
    {
      /* Seek failed... errno has already been set, but EINVAL is probably
       * the correct response.
       */

      ferr("ERROR: Seek to position %d failed\n", (int)offset);
      ret = fpos;
      goto errout_with_region;
    }

  /* Read the file data into the memory region */

  rdbuffer = map->addr;
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

              ferr("ERROR: Read failed: offset=%d ret=%d\n",
                   (int)offset, (int)nread);

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

  ret = nxmutex_lock(&g_rammaps.lock);
  if (ret < 0)
    {
      goto errout_with_region;
    }

  map->flink = g_rammaps.head;
  g_rammaps.head = map;

  nxmutex_unlock(&g_rammaps.lock);
  *mapped = map->addr;
  return OK;

errout_with_region:
  if (kernel)
    {
      kmm_free(alloc);
    }
  else
    {
      kumm_free(alloc);
    }

  return ret;
}

#endif /* CONFIG_FS_RAMMAP */
