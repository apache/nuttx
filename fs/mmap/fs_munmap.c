/****************************************************************************
 * fs/mmap/fs_munmap.c
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
#include <nuttx/mm/map.h>

#include <sys/types.h>
#include <sys/mman.h>

#include <stdint.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/kmalloc.h>

#include "inode/inode.h"
#include "fs_rammap.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int file_munmap_(FAR void *start, size_t length, bool kernel)
{
  FAR struct tcb_s *tcb = nxsched_self();
  FAR struct task_group_s *group = tcb->group;
  FAR struct mm_map_entry_s *entry = NULL;
  int ret = OK;

  /* Iterate through all the mappings and call the underlying
   * unmap for every mapping where "start" lies
   * break loop on any errors.
   *
   * Get exclusive access to mm_map for this
   */

  ret = mm_map_lock();
  if (ret == OK)
    {
      while (ret == OK && (entry = mm_map_find(start, length)))
        {
          DEBUGASSERT(entry->munmap);
          ret = entry->munmap(group, entry, start, length);
        }

      mm_map_unlock();
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: file_mummap
 *
 * Description:
 *   Equivalent to the standard file_mummap() function except it does not set
 *   the errno variable.
 *
 ****************************************************************************/

int file_munmap(FAR void *start, size_t length)
{
  return file_munmap_(start, length, true);
}

/****************************************************************************
 * Name: munmap
 *
 * Description:
 *
 *   munmap() system call deletes mappings for the specified address range.
 *   All memory starting with 'start' and continuing for a length of 'length'
 *   bytes are removed.
 *
 *   NuttX operates in a flat open address space.  Therefore, it generally
 *   does not require mmap() and, hence, munmap functionality.  There are
 *   two exceptions where mmap() is available:
 *
 *   1. mmap() is the API that is used to support direct access to random
 *     access media under the following very restrictive conditions:
 *
 *     a. The filesystem impelements the mmap file operation.  Any file
 *        system that maps files contiguously on the media should support
 *        this ioctl. (vs. file system that scatter files over the media
 *        in non-contiguous sectors).  As of this writing, ROMFS is the
 *        only file system that meets this requirement.
 *     b. The underlying block driver supports the BIOC_XIPBASE ioctl
 *        command that maps the underlying media to a randomly accessible
 *        address. At  present, only the RAM/ROM disk driver does this.
 *
 *     munmap() is still not required in this first case.  In this first
 *     The mapped address is a static address in the MCUs address space
 *     does not need to be munmapped.  Support for munmap() in this case
 *     provided by the simple definition in sys/mman.h:
 *
 *        #define munmap(start, length)
 *
 *   2. If CONFIG_FS_RAMMAP is defined in the configuration, then mmap() will
 *      support simulation of memory mapped files by copying files whole
 *      into RAM.  munmap() is required in this case to free the allocated
 *      memory holding the shared copy of the file.
 *
 * Input Parameters:
 *   start   The start address of the mapping to delete.  For this
 *           simplified munmap() implementation, the *must* be the start
 *           address of the memory region (the same address returned by
 *           mmap()).
 *   length  The length region to be umapped.
 *
 * Returned Value:
 *   On success, munmap() returns 0, on failure -1, and errno is set
 *   (probably to EINVAL).
 *
 ****************************************************************************/

int munmap(FAR void *start, size_t length)
{
  int ret;

  ret = file_munmap_(start, length, false);
  if (ret < 0)
    {
      set_errno(-ret);
      ret = ERROR;
    }

  return ret;
}
