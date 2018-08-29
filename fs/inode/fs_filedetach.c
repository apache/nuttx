/****************************************************************************
 * fs/inode/fs_filedetach.c
 *
 *   Copyright (C) 2016-2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <semaphore.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/sched.h>
#include <nuttx/fs/fs.h>

#include "inode/inode.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: _files_semtake
 ****************************************************************************/

static inline void _files_semtake(FAR struct filelist *list)
{
  int ret;

  do
    {
      /* Take the semaphore (perhaps waiting) */

      ret = nxsem_wait(&list->fl_sem);

      /* The only case that an error should occur here is if the wait was
       * awakened by a signal.
       */

      DEBUGASSERT(ret == OK || ret == -EINTR);
    }
  while (ret == -EINTR);
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

#if CONFIG_NFILE_DESCRIPTORS > 0
int file_detach(int fd, FAR struct file *filep)
{
  FAR struct filelist *list;
  FAR struct file *parent;

  /* Verify the file descriptor range */

  if (fd < 0 || fd >= CONFIG_NFILE_DESCRIPTORS)
    {
      /* Not a file descriptor (might be a socket descriptor) */

      return -EBADF;
    }

  /* Get the thread-specific file list.  It should never be NULL in this
   * context.
   */

  list = sched_getfiles();
  DEBUGASSERT(list != NULL);

  /* If the file was properly opened, there should be an inode assigned */

  _files_semtake(list);
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
   * in file_close_detached().
   */

  parent->f_oflags = 0;
  parent->f_pos    = 0;
  parent->f_inode  = NULL;
  parent->f_priv   = NULL;

  _files_semgive(list);
  return OK;
}
#endif

/****************************************************************************
 * Name: file_close_detached
 *
 * Description:
 *   Close a file that was previously detached with file_detach().
 *
 *   REVISIT:  This is essentially the same as _files_close()
 *
 * Input Parameters:
 *   filep - A pointer to a user provided memory location containing the
 *           open file data returned by file_detach().
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned on
 *   any failure to indicate the nature of the failure.
 *
 ****************************************************************************/

int file_close_detached(FAR struct file *filep)
{
  struct inode *inode;
  int ret = OK;

  DEBUGASSERT(filep != NULL);
  inode = filep->f_inode;

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

      /* Reset the user file struct instance so that it cannot be reused. */

      filep->f_oflags = 0;
      filep->f_pos    = 0;
      filep->f_inode  = NULL;
      filep->f_priv   = NULL;
    }

  return ret;
}
