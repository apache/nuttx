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
 * Name: files_dup2
 *
 * Description:
 *   Clone a file descriptor to a specific descriptor number.
 *
 * Returned Value:
 *   fd2 is returned on success; a negated errno value is return on
 *   any failure.
 *
 ****************************************************************************/

int files_dup2(int fd1, int fd2)
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
 * Name: files_close
 *
 * Description:
 *   Close an inode (if open)
 *
 * Assumptions:
 *   Caller holds the list semaphore because the file descriptor will be
 *   freed.
 *
 ****************************************************************************/

int files_close(int fd)
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
 * Name: files_release
 *
 * Assumptions:
 *   Similar to files_close().  Called only from open() logic on error
 *   conditions.
 *
 ****************************************************************************/

void files_release(int fd)
{
  FAR struct filelist *list;
  int ret;

  list = nxsched_get_files();
  DEBUGASSERT(list != NULL);

  if (fd >= 0 && fd < CONFIG_NFILE_DESCRIPTORS)
    {
      ret = _files_semtake(list);
      if (ret >= 0)
        {
          list->fl_files[fd].f_oflags  = 0;
          list->fl_files[fd].f_pos     = 0;
          list->fl_files[fd].f_inode = NULL;
          _files_semgive(list);
        }
    }
}
