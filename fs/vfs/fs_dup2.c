/****************************************************************************
 * fs/vfs/fs_dup2.c
 *
 *   Copyright (C) 2007-2009, 2011, 2013, 2017 Gregory Nutt. All rights
 *     reserved.
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

#include <unistd.h>
#include <sched.h>
#include <errno.h>

#include "inode/inode.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: file_dup2
 *
 * Description:
 *   Assign an inode to a specific files structure.  This is the heart of
 *   dup2.
 *
 *   Equivalent to the non-standard dup2() function except that it
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
  FAR struct inode *inode;
  struct file temp;
  int ret;

  if (filep1 == NULL || filep1->f_inode == NULL || filep2 == NULL)
    {
      return -EBADF;
    }

  if (filep1 == filep2)
    {
      return OK;
    }

  /* Increment the reference count on the contained inode */

  inode = filep1->f_inode;
  ret   = inode_addref(inode);
  if (ret < 0)
    {
      return ret;
    }

  /* Then clone the file structure */

  temp.f_oflags = filep1->f_oflags;
  temp.f_pos    = filep1->f_pos;
  temp.f_inode  = inode;
  temp.f_priv   = NULL;

  /* Call the open method on the file, driver, mountpoint so that it
   * can maintain the correct open counts.
   */

  if (inode->u.i_ops && inode->u.i_ops->open)
    {
#ifndef CONFIG_DISABLE_MOUNTPOINT
      if (INODE_IS_MOUNTPT(inode))
        {
          /* Dup the open file on the in the new file structure */

          ret = inode->u.i_mops->dup(filep1, &temp);
        }
      else
#endif
        {
          /* (Re-)open the pseudo file or device driver */

          ret = inode->u.i_ops->open(&temp);
        }

      /* Handle open failures */

      if (ret < 0)
        {
          inode_release(inode);
          return ret;
        }
    }

  /* If there is already an inode contained in the new file structure,
   * close the file and release the inode.
   */

  ret = file_close(filep2);
  DEBUGASSERT(ret == 0);

  /* Return the file structure */

  memcpy(filep2, &temp, sizeof(temp));
  return OK;
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
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is return on
 *   any failure.
 *
 ****************************************************************************/

int nx_dup2(int fd1, int fd2)
{
  /* Check the range of the descriptor to see if we got a file or a socket
   * descriptor.
   */

  if (fd1 >= CONFIG_NFILE_DESCRIPTORS)
    {
      /* Not a valid file descriptor.
       * Did we get a valid socket descriptor?
       */

#ifdef CONFIG_NET
      if (fd1 < (CONFIG_NFILE_DESCRIPTORS + CONFIG_NSOCKET_DESCRIPTORS))
        {
          /* Yes.. dup the socket descriptor. */

          return net_dup2(fd1, fd2);
        }
      else
#endif
        {
          /* No.. then it is a bad descriptor number */

          return -EBADF;
        }
    }
  else
    {
      /* Its a valid file descriptor.. dup the file descriptor.
       */

      return files_dup2(fd1, fd2);
    }
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
