/****************************************************************************
 * fs/inode/fs_files.c
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
#include <assert.h>
#include <sched.h>
#include <errno.h>

#include <nuttx/fs/fs.h>
#include <nuttx/kmalloc.h>
#include <nuttx/cancelpt.h>
#include <nuttx/semaphore.h>

#include "inode/inode.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: _files_semtake
 ****************************************************************************/

static int _files_semtake(FAR struct filelist *list)
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
 * Name: files_initialize
 *
 * Description:
 *   This is called from the FS initialization logic to configure the files.
 *
 ****************************************************************************/

void files_initialize(void)
{
}

/****************************************************************************
 * Name: files_initlist
 *
 * Description: Initializes the list of files for a new task
 *
 ****************************************************************************/

void files_initlist(FAR struct filelist *list)
{
  DEBUGASSERT(list);

  /* Initialize the list access mutex */

  nxsem_init(&list->fl_sem, 0, 1);
}

/****************************************************************************
 * Name: files_releaselist
 *
 * Description:
 *   Release a reference to the file list
 *
 ****************************************************************************/

void files_releaselist(FAR struct filelist *list)
{
  int i;

  DEBUGASSERT(list);

  /* Close each file descriptor .. Normally, you would need take the list
   * semaphore, but it is safe to ignore the semaphore in this context
   * because there should not be any references in this context.
   */

  for (i = CONFIG_NFILE_DESCRIPTORS; i > 0; i--)
    {
      file_close(&list->fl_files[i - 1]);
    }

  /* Destroy the semaphore */

  nxsem_destroy(&list->fl_sem);
}

/****************************************************************************
 * Name: files_allocate
 *
 * Description:
 *   Allocate a struct files instance and associate it with an inode
 *   instance.  Returns the file descriptor == index into the files array.
 *
 ****************************************************************************/

int files_allocate(FAR struct inode *inode, int oflags, off_t pos,
                   FAR void *priv, int minfd)
{
  FAR struct filelist *list;
  int ret;
  int i;

  /* Get the file descriptor list.  It should not be NULL in this context. */

  list = nxsched_get_files();
  DEBUGASSERT(list != NULL);

  ret = _files_semtake(list);
  if (ret < 0)
    {
      /* Probably canceled */

      return ret;
    }

  for (i = minfd; i < CONFIG_NFILE_DESCRIPTORS; i++)
    {
      if (!list->fl_files[i].f_inode)
        {
          list->fl_files[i].f_oflags = oflags;
          list->fl_files[i].f_pos    = pos;
          list->fl_files[i].f_inode  = inode;
          list->fl_files[i].f_priv   = priv;
          _files_semgive(list);
          return i;
        }
    }

  _files_semgive(list);
  return -EMFILE;
}

/****************************************************************************
 * Name: nx_dup2
 *
 * Description:
 *   nx_dup2() is similar to the standard 'dup2' interface except that is
 *   not a cancellation point and it does not modify the errno variable.
 *
 *   nx_dup2() is an internal NuttX interface and should not be called from
 *   applications.
 *
 *   Clone a file descriptor to a specific descriptor number.
 *
 * Returned Value:
 *   fd2 is returned on success; a negated errno value is return on
 *   any failure.
 *
 ****************************************************************************/

int nx_dup2(int fd1, int fd2)
{
  FAR struct filelist *list;
  int ret;

  if (fd1 < 0 || fd1 >= CONFIG_NFILE_DESCRIPTORS ||
      fd2 < 0 || fd2 >= CONFIG_NFILE_DESCRIPTORS)
    {
      return -EBADF;
    }

  /* Get the file descriptor list.  It should not be NULL in this context. */

  list = nxsched_get_files();
  DEBUGASSERT(list != NULL);

  ret = _files_semtake(list);
  if (ret < 0)
    {
      /* Probably canceled */

      return ret;
    }

  /* Perform the dup2 operation */

  ret = file_dup2(&list->fl_files[fd1], &list->fl_files[fd2]);
  _files_semgive(list);
  if (ret < 0)
    {
      return ret;
    }

  return fd2;
}

/****************************************************************************
 * Name: dup2
 *
 * Description:
 *   Clone a file descriptor or socket descriptor to a specific descriptor
 *   number
 *
 ****************************************************************************/

int dup2(int fd1, int fd2)
{
  int ret;

  ret = nx_dup2(fd1, fd2);
  if (ret < 0)
    {
      set_errno(-ret);
      ret = ERROR;
    }

  return ret;
}

/****************************************************************************
 * Name: nx_close
 *
 * Description:
 *   nx_close() is similar to the standard 'close' interface except that is
 *   not a cancellation point and it does not modify the errno variable.
 *
 *   nx_close() is an internal NuttX interface and should not be called from
 *   applications.
 *
 *   Close an inode (if open)
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned on
 *   on any failure.
 *
 * Assumptions:
 *   Caller holds the list semaphore because the file descriptor will be
 *   freed.
 *
 ****************************************************************************/

int nx_close(int fd)
{
  FAR struct filelist *list;
  int                  ret;

  /* Get the thread-specific file list.  It should never be NULL in this
   * context.
   */

  list = nxsched_get_files();
  DEBUGASSERT(list != NULL);

  /* If the file was properly opened, there should be an inode assigned */

  if (fd < 0 || fd >= CONFIG_NFILE_DESCRIPTORS ||
      !list->fl_files[fd].f_inode)
    {
      return -EBADF;
    }

  /* Perform the protected close operation */

  ret = _files_semtake(list);
  if (ret >= 0)
    {
      ret = file_close(&list->fl_files[fd]);
      _files_semgive(list);
    }

  return ret;
}

/****************************************************************************
 * Name: close
 *
 * Description:
 *   close() closes a file descriptor, so that it no longer refers to any
 *   file and may be reused. Any record locks (see fcntl(2)) held on the file
 *   it was associated with, and owned by the process, are removed
 *   (regardless of the file descriptor that was used to obtain the lock).
 *
 *   If fd is the last copy of a particular file descriptor the resources
 *   associated with it are freed; if the descriptor was the last reference
 *   to a file which has been removed using unlink(2) the file is deleted.
 *
 * Input Parameters:
 *   fd   file descriptor to close
 *
 * Returned Value:
 *   0 on success; -1 on error with errno set appropriately.
 *
 * Assumptions:
 *
 ****************************************************************************/

int close(int fd)
{
  int ret;

  /* close() is a cancellation point */

  enter_cancellation_point();

  ret = nx_close(fd);
  if (ret < 0)
    {
      set_errno(-ret);
      ret = ERROR;
    }

  leave_cancellation_point();
  return ret;
}
