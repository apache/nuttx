/****************************************************************************
 * fs/vfs/fs_dupfd2.c
 *
 *   Copyright (C) 2007-2009, 2011-2014, 2017 Gregory Nutt. All rights
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

#if CONFIG_NFILE_DESCRIPTORS > 0

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DUP_ISOPEN(filep) (filep->f_inode != NULL)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fs_dupfd2 OR dup2
 *
 * Description:
 *   Clone a file descriptor to a specific descriptor number. If socket
 *   descriptors are implemented, then this is called by dup2() for the
 *   case of file descriptors.  If socket descriptors are not implemented,
 *   then this function IS dup2().
 *
 * Returned Value:
 *   fs_dupfd is sometimes an OS internal function and sometimes is a direct
 *   substitute for dup2().  So it must return an errno value as though it
 *   were dup2().
 *
 ****************************************************************************/

#if defined(CONFIG_NET) && CONFIG_NSOCKET_DESCRIPTORS > 0
int fs_dupfd2(int fd1, int fd2)
#else
int dup2(int fd1, int fd2)
#endif
{
  FAR struct file *filep1;
  FAR struct file *filep2 = NULL;
  int ret;

  /* Get the file structures corresponding to the file descriptors. */

  ret = fs_getfilep(fd1, &filep1);
  if (ret >= 0)
    {
      ret = fs_getfilep(fd2, &filep2);
    }

  if (ret < 0)
    {
      goto errout;
    }

  DEBUGASSERT(filep1 != NULL && filep2 != NULL);

  /* Verify that fd1 is a valid, open file descriptor */

  if (!DUP_ISOPEN(filep1))
    {
      ret = -EBADF;
      goto errout;
    }

  /* Handle a special case */

  if (fd1 == fd2)
    {
      return fd1;
    }

  /* Perform the dup2 operation */

  ret = file_dup2(filep1, filep2);
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

