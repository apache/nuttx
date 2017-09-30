/****************************************************************************
 * fs/vfs/fs_dup2.c
 *
 *   Copyright (C) 2007-2009, 2011, 2013, 2017 Gregory Nutt. All rights reserved.
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

/* This logic in this applies only when both socket and file descriptors are
 * in that case, this function descriminates which type of dup2 is being
 * performed.
 */

#if CONFIG_NFILE_DESCRIPTORS > 0 && defined(CONFIG_NET) && CONFIG_NSOCKET_DESCRIPTORS > 0

/****************************************************************************
 * Public Functions
 ****************************************************************************/

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
  /* Check the range of the descriptor to see if we got a file or a socket
   * descriptor.
   */

  if ((unsigned int)fd1 >= CONFIG_NFILE_DESCRIPTORS)
    {
      int ret;

      /* Not a valid file descriptor.  Did we get a valid socket descriptor? */

      if ((unsigned int)fd1 < (CONFIG_NFILE_DESCRIPTORS+CONFIG_NSOCKET_DESCRIPTORS))
        {
          /* Yes.. dup the socket descriptor. The errno value is not set. */

          ret = net_dupsd2(fd1, fd2);
        }
      else
        {
          /* No.. then it is a bad descriptor number */

          ret = -EBADF;
        }

      /* Set the errno value on failures */

      if (ret < 0)
        {
          set_errno(-ret);
          ret = ERROR;
        }

      return ret;
    }
  else
    {
      /* Its a valid file descriptor.. dup the file descriptor.  fd_dupfd()
       * sets the errno value in the event of any failures.
       */

      return fs_dupfd2(fd1, fd2);
    }
}

#endif /* CONFIG_NFILE_DESCRIPTORS > 0 ... */

