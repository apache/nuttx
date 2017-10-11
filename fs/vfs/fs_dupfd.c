/****************************************************************************
 * fs/vfs/fs_dupfd.c
 *
 *   Copyright (C) 2007-2009, 2011-2014, 2017 Gregory Nutt. All rights reserved.
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

#include <sched.h>
#include <errno.h>
#include <assert.h>

#include <nuttx/fs/fs.h>

#include "inode/inode.h"

#if CONFIG_NFILE_DESCRIPTORS > 0

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DUP_ISOPEN(filep) (filep->f_inode != NULL)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: file_dup
 *
 * Description:
 *   Equivalent to the non-standard fs_dupfd() function except that it
 *   accepts a struct file instance instead of a file descriptor and does
 *   not set the errno variable.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

int file_dup(FAR struct file *filep, int minfd)
{
  int fd2;

  /* Verify that fd is a valid, open file descriptor */

  if (!DUP_ISOPEN(filep))
    {
      return -EBADF;
    }

  /* Increment the reference count on the contained inode */

  inode_addref(filep->f_inode);

  /* Then allocate a new file descriptor for the inode */

  fd2 = files_allocate(filep->f_inode, filep->f_oflags, filep->f_pos, minfd);
  if (fd2 < 0)
    {
      inode_release(filep->f_inode);
      return -EMFILE;
    }

  return fd2;
}

/****************************************************************************
 * Name: fs_dupfd OR dup
 *
 * Description:
 *   Clone a file descriptor 'fd' to an arbitray descriptor number (any value
 *   greater than or equal to 'minfd'). If socket descriptors are
 *   implemented, then this is called by dup() for the case of file
 *   descriptors.  If socket descriptors are not implemented, then this
 *   function IS dup().
 *
 * Returned Value:
 *   fs_dupfd is sometimes an OS internal function and sometimes is a direct
 *   substitute for dup().  So it must return an errno value as though it
 *   were dup().
 *
 ****************************************************************************/

int fs_dupfd(int fd, int minfd)
{
  FAR struct file *filep;
  int ret;

  /* Get the file structure corresponding to the file descriptor. */

  ret = fs_getfilep(fd, &filep);
  if (ret < 0)
    {
      goto errout;
    }

  DEBUGASSERT(filep != NULL);

  /* Let file_dup() do the real work */

  ret = file_dup(filep, minfd);
  if (ret < 0)
    {
      goto errout;
    }

  return OK;

errout:
  set_errno(-ret);
  return ERROR;
}

#endif /* CONFIG_NFILE_DESCRIPTORS > 0 */

