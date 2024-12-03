/****************************************************************************
 * fs/mmap/fs_msync.c
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

#include <sys/mman.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/fs/fs.h>
#include <nuttx/sched.h>

#include "inode/inode.h"
#include "fs_rammap.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: msync
 *
 * Description:
 *   Equivalent to the standard msync() function except that it accepts
 *   a struct file instance instead of a memory address and it does not set
 *   the errno variable.
 *
 * Input Parameters:
 *   file   - A pointer to the struct file instance representing the
 *            mappedfile
 *   start  - The starting address of the memory region to be synchronized
 *   length - The length of the memory region to be synchronized
 *   flags  - Flags that determine the type of synchronization to be
 *            performed
 *
 * Returned Value:
 *   On success, returns 0 (OK); On failure, returns a negated errno value.
 *
 ****************************************************************************/

int msync(FAR void *start, size_t length, int flags)
{
  FAR struct mm_map_entry_s *entry;
  int ret;

  ret = mm_map_lock();
  if (ret < 0)
    {
      return ret;
    }

  entry = mm_map_find(get_current_mm(), start, length);
  if (!entry)
    {
      ferr("ERROR: Region not found\n");
      ret = -ENOMEM;
      goto out;
    }

  if (entry->msync == NULL)
    {
      ret = OK;
      goto out;
    }

  ret = entry->msync(entry, start, length, flags);
out:
  mm_map_unlock();
  if (ret < 0)
    {
      set_errno(-ret);
      ret = ERROR;
    }

  return ret;
}
