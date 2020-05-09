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
 * Name: _files_close
 *
 * Description:
 *   Close an inode (if open)
 *
 * Assumptions:
 *   Caller holds the list semaphore because the file descriptor will be
 *   freed.
 *
 ****************************************************************************/

static int _files_close(FAR struct file *filep)
{
  struct inode *inode = filep->f_inode;
  int ret = OK;

  /* Check if the struct file is open (i.e., assigned an inode) */

  if (inode)
    {
      /* Close the file, driver, or mountpoint. */

      if (inode->u.i_ops && inode->u.i_ops->close)
        {
          /* Perform the close operation */

          ret = inode->u.i_ops->close(filep);
        }

      /* And release the inode */

      inode_release(inode);

      /* Release the file descriptor */

      filep->f_oflags  = 0;
      filep->f_pos     = 0;
      filep->f_inode = NULL;
    }

  return ret;
}

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

  for (i = 0; i < CONFIG_NFILE_DESCRIPTORS; i++)
    {
      _files_close(&list->fl_files[i]);
    }

  /* Destroy the semaphore */

  nxsem_destroy(&list->fl_sem);
}

/****************************************************************************
 * Name: file_dup2
 *
 * Description:
 *   Assign an inode to a specific files structure.  This is the heart of
 *   dup2.
 *
 *   Equivalent to the non-standard fs_dupfd2() function except that it
 *   accepts struct file instances instead of file descriptors and it does
 *   not set the errno variable.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is return on
 *   any failure.
 *
 ****************************************************************************/

int file_dup2(FAR struct file *filep1, FAR struct file *filep2)
{
  FAR struct filelist *list;
  FAR struct inode *inode;
  int ret;

  if (filep1 == NULL || filep1->f_inode == NULL || filep2 == NULL)
    {
      return -EBADF;
    }

  list = nxsched_get_files();

  /* The file list can be NULL under two cases:  (1) One is an obscure
   * cornercase:  When memory management debug output is enabled.  Then
   * there may be attempts to write to stdout from malloc before the group
   * data has been allocated.  The other other is (2) if this is a kernel
   * thread.  Kernel threads have no allocated file descriptors.
   */

  if (list != NULL)
    {
      ret = _files_semtake(list);
      if (ret < 0)
        {
          /* Probably canceled */

          return ret;
        }
    }

  /* If there is already an inode contained in the new file structure,
   * close the file and release the inode.
   */

  ret = _files_close(filep2);
  if (ret < 0)
    {
      /* An error occurred while closing the driver */

      goto errout_with_sem;
    }

  /* Increment the reference count on the contained inode */

  inode = filep1->f_inode;
  ret   = inode_addref(inode);
  if (ret < 0)
    {
      goto errout_with_sem;
    }

  /* Then clone the file structure */

  filep2->f_oflags = filep1->f_oflags;
  filep2->f_pos    = filep1->f_pos;
  filep2->f_inode  = inode;

  /* Call the open method on the file, driver, mountpoint so that it
   * can maintain the correct open counts.
   */

  if (inode->u.i_ops && inode->u.i_ops->open)
    {
#ifndef CONFIG_DISABLE_MOUNTPOINT
      if (INODE_IS_MOUNTPT(inode))
        {
          /* Dup the open file on the in the new file structure */

          ret = inode->u.i_mops->dup(filep1, filep2);
        }
      else
#endif
        {
          /* (Re-)open the pseudo file or device driver */

          ret = inode->u.i_ops->open(filep2);
        }

      /* Handle open failures */

      if (ret < 0)
        {
          goto errout_with_inode;
        }
    }

  if (list != NULL)
    {
      _files_semgive(list);
    }

  return OK;

  /* Handle various error conditions */

errout_with_inode:

  inode_release(filep2->f_inode);
  filep2->f_oflags = 0;
  filep2->f_pos    = 0;
  filep2->f_inode  = NULL;

errout_with_sem:
  if (list != NULL)
    {
      _files_semgive(list);
    }

  return ret;
}

/****************************************************************************
 * Name: files_allocate
 *
 * Description:
 *   Allocate a struct files instance and associate it with an inode
 *   instance.  Returns the file descriptor == index into the files array.
 *
 ****************************************************************************/

int files_allocate(FAR struct inode *inode, int oflags, off_t pos, int minfd)
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
          list->fl_files[i].f_priv   = NULL;
          _files_semgive(list);
          return i;
        }
    }

  _files_semgive(list);
  return ERROR;
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
      ret = _files_close(&list->fl_files[fd]);
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
