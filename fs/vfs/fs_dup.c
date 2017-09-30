/****************************************************************************
 * fs/vfs/fs_dup.c
 *
 *   Copyright (C) 2007-2009, 2017 Gregory Nutt. All rights reserved.
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

#include <nuttx/fs/fs.h>
#include "inode/inode.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: dup
 *
 * Description:
 *   Clone a file or socket descriptor to an arbitray descriptor number
 *
 ****************************************************************************/

int dup(int fd)
{
  int ret = OK;

  /* Check the range of the descriptor to see if we got a file or a socket
   * descriptor. */

#if CONFIG_NFILE_DESCRIPTORS > 0
  if ((unsigned int)fd < CONFIG_NFILE_DESCRIPTORS)
    {
      /* Its a valid file descriptor.. dup the file descriptor using any
       * other file descriptor.  fd_dupfd() sets the errno value in the
       * event of any failures.
       */

      ret = fs_dupfd(fd, 0);
    }
  else
#endif
    {
      /* Not a valid file descriptor.  Did we get a valid socket descriptor? */

#if defined(CONFIG_NET) && CONFIG_NSOCKET_DESCRIPTORS > 0
      if ((unsigned int)fd < (CONFIG_NFILE_DESCRIPTORS+CONFIG_NSOCKET_DESCRIPTORS))
        {
          /* Yes.. dup the socket descriptor.  The errno value is not set. */

          ret = net_dupsd(fd, CONFIG_NFILE_DESCRIPTORS);
        }
      else
#endif
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
    }

  return ret;
}
