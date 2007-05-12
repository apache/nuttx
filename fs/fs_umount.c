/****************************************************************************
 * fs_umount.c
 *
 *   Copyright (C) 2007 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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
 * 3. Neither the name Gregory Nutt nor the names of its contributors may be
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
#include <sys/types.h>
#include <sys/mount.h>
#include <errno.h>
#include <nuttx/fs.h>
#include "fs_internal.h"

#if CONFIG_NFILE_DESCRIPTORS > 0

/****************************************************************************
 * Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Variables
 ****************************************************************************/

/****************************************************************************
 * Public Variables
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: umount
 *
 * Description:
 *   umount() detaches the filesystem mounted at the path specified by
 *  'target.'
 *
 * Return:
 *   Zero is returned on success; -1 is returned on an error and errno is
 *   set appropriately:
 *
 *   EACCES A component of a path was not searchable or mounting a read-only
 *      filesystem was attempted without giving the MS_RDONLY flag.
 *   EBUSY The target could not be unmounted because it is busy.
 *   EFAULT The pointer argument points outside the user address space.
 *
 ****************************************************************************/

int umount(const char *target)
{
  FAR struct inode *mountpt_inode;
  int errcode;
  int status;

  /* Verify required pointer arguments */

  if (!target)
    {
      errcode = EFAULT;
      goto errout;
    }

  /* Find the mountpt */

  inode_semtake();
  mountpt_inode = inode_find(target, NULL);
  if (!mountpt_inode)
    {
      errcode = ENOENT;
      goto errout_with_semaphore;
    }

  /* Verify that the inode is a mountpoint */

  if (!INODE_IS_MOUNTPT(mountpt_inode))
    { 
      errcode = EINVAL;
      goto errout_with_mountpt;
   }

  /* Unbind the block driver from the file system (destroying any fs
   * private data.
   */

  if (!mountpt_inode->u.i_mops->unbind)
    {
      /* The filesystem does not support the unbind operation ??? */

      errcode = EINVAL;
      goto errout_with_mountpt;
    }

  /* The unbind method returns the number of references to the
   * filesystem (i.e., open files), zero if the unbind was
   * performed, or a negated error code on a failure.
   */

  status = mountpt_inode->u.i_mops->unbind( mountpt_inode->i_private );
  if (status < 0)
    {
      /* The inode is unhappy with the blkdrvr for some reason */

      errcode = -status;
      goto errout_with_mountpt;
    }
  else if (status > 0)
    {
      errcode = EBUSY;
      goto errout_with_mountpt;
    }

  /* Successfully unbound */

  mountpt_inode->i_private = NULL;

  /* Remove the inode */

  inode_release(mountpt_inode);
  status = inode_remove(target);
  inode_semgive();
  return status;

  /* A lot of goto's!  But they make the error handling much simpler */

 errout_with_mountpt:
  inode_release(mountpt_inode);
 errout_with_semaphore:
  inode_semgive();
 errout:
  *get_errno_ptr() = errcode;
  return ERROR;
}

#endif /* CONFIG_NFILE_DESCRIPTORS */
