/****************************************************************************
 * fs/mmap/fs_msync.c
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

#include <sys/types.h>
#include <sys/mman.h>

#include <unistd.h>
#include <stdint.h>
#include <errno.h>
#include <debug.h>

#include "fs_rammap.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: msync
 *
 * Description:
 *
 *   msync() flushes changes made to the in-core copy of a file that was
 *   mapped into memory using mmap(2) back to the filesystem. Without use
 *   of this call, there is no guarantee that changes are written back
 *   before munmap(2) is called. To be more precise, the part of the file
 *   that corresponds to the memory area starting at addr and having length
 *   length is updated.
 *
 * Input Parameters:
 *   start   The start address of the mapping to sync.
 *   length  The length region to be updated.
 *   flags   The flags specify, include  MS_ASYNC, MS_SYNC, MS_INVALIDATE.
 *
 * Returned Value:
 *   On success, msync() returns 0, on failure -1, and errno is set.
 *
 ****************************************************************************/

int msync(FAR void *start, size_t length, int flags)
{
#ifdef CONFIG_FS_RAMMAP
  FAR struct fs_rammap_s *curr;
  FAR uint8_t *wrbuffer = NULL;
  unsigned int offset;
  ssize_t nwrite;
  off_t fpos;
  int ret;

  /* Find a region containing this start and length in the list of regions */

  ret = nxmutex_lock(&g_rammaps.lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Search the list of regions */

  for (curr = g_rammaps.head; curr; curr = curr->flink)
    {
      if ((uintptr_t)start >= (uintptr_t)curr->addr + curr->length ||
          (uintptr_t)start + length <= (uintptr_t)curr->addr)
        {
          continue;
        }

      /* Ignore flags MS_ASYNC is same as MS_SYNC */

      /* Get the offset from the beginning of the region and the actual number
       * of bytes to "msync".
       */

      offset = start - curr->addr;
      if (length > curr->length - offset)
        {
          length = curr->length - offset;
        }

      fpos = file_seek(curr->file, offset, SEEK_SET);
      if (fpos < 0)
        {
          /* Seek failed... errno has already been set, but EINVAL is probably
           * the correct response.
           */

          ret = (int)fpos;
          ferr("ERROR: Seek to position %d failed\n", ret);
          goto errout_with_semaphore;
        }

      wrbuffer = start;
      while (length > 0)
        {
          nwrite = file_write(curr->file, wrbuffer, length);
          if (nwrite < 0)
            {
              /* Handle the special case where the read was interrupted by a
               * signal.
               */

              if (nwrite != -EINTR)
                {
                  /* All other read errors are bad. */

                  ferr("ERROR: write failed: offset=%d ret=%d\n",
                       (int)offset, (int)nwrite);

                  ret = nwrite;
                  goto errout_with_semaphore;
                }
              else
                {
                  continue;
                }
            }

          /* Increment number of bytes read */

          wrbuffer += nwrite;
          length   -= nwrite;
        }
    }

  /* Did we find the region */

  if (length)
    {
      ferr("ERROR: Region not found\n");
      ret = -ENOMEM;
    }

errout_with_semaphore:
  nxmutex_unlock(&g_rammaps.lock);
  if (ret < 0)
    {
      set_errno(-ret);
      ret = ERROR;
    }

  return ret;
#endif /* CONFIG_FS_RAMMAP */

  return OK;
}
