/****************************************************************************
 * fs/inode/fs_filedetach.c
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

#include <assert.h>
#include <errno.h>

#include <nuttx/sched.h>
#include <nuttx/fs/fs.h>
#include <nuttx/semaphore.h>

#include "inode/inode.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: _files_semtake
 ****************************************************************************/

static inline int _files_semtake(FAR struct filelist *list)
{
  return nxsem_wait_uninterruptible(&list->fl_sem);
}

/****************************************************************************
 * Name: _files_semgive
 ****************************************************************************/

#define _files_semgive(list) nxsem_post(&list->fl_sem)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: file_detach
 *
 * Description:
 *   This function is used in device drivers to create a task-independent
 *   handle to an entity in the file system.  file_detach() duplicates the
 *   'struct file' that underlies the file descriptor, then closes the file
 *   descriptor.
 *
 *   This function will fail if fd is not a valid file descriptor.  In
 *   particular, it will fail if fd is a socket descriptor.
 *
 * Input Parameters:
 *   fd    - The file descriptor to be detached.  This descriptor will be
 *           closed and invalid if the file was successfully detached.
 *   filep - A pointer to a user provided memory location in which to
 *           received the duplicated, detached file structure.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned on
 *   any failure to indicate the nature of the failure.
 *
 ****************************************************************************/

int file_detach(int fd, FAR struct file *filep)
{
  FAR struct filelist *list;
  FAR struct file *parent;
  int ret;

  DEBUGASSERT(filep != NULL);

  /* Verify the file descriptor range */

  if (fd < 0 || fd >= CONFIG_NFILE_DESCRIPTORS)
    {
      /* Not a file descriptor (might be a socket descriptor) */

      return -EBADF;
    }

  /* Get the thread-specific file list.  It should never be NULL in this
   * context.
   */

  list = nxsched_get_files();
  DEBUGASSERT(list != NULL);

  /* If the file was properly opened, there should be an inode assigned */

  ret = _files_semtake(list);
  if (ret < 0)
    {
      /* Probably canceled */

      return ret;
    }

  parent = &list->fl_files[fd];
  if (parent->f_inode == NULL)
    {
      /* File is not open */

      _files_semgive(list);
      return -EBADF;
    }

  /* Duplicate the 'struct file' content into the user-provided file
   * structure.
   */

  filep->f_oflags  = parent->f_oflags;
  filep->f_pos     = parent->f_pos;
  filep->f_inode   = parent->f_inode;
  filep->f_priv    = parent->f_priv;

  /* Release the file descriptor *without* calling the driver close method
   * and without decrementing the inode reference count.  That will be done
   * in file_close().
   */

  parent->f_oflags = 0;
  parent->f_pos    = 0;
  parent->f_inode  = NULL;
  parent->f_priv   = NULL;

  _files_semgive(list);
  return OK;
}
