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
 * Name: file_dup
 *
 * Description:
 *   Equivalent to the standard dup() function except that it
 *   accepts a struct file instance instead of a file descriptor.
 *
 * Returned Value:
 *   The new file descriptor is returned on success; a negated errno value
 *   is returned on any failure.
 *
 ****************************************************************************/

int file_dup(FAR struct file *filep, int minfd)
{
  struct file filep2;
  int fd2;
  int ret;

  /* Let file_dup2() do the real work */

  memset(&filep2, 0, sizeof(filep2));
  ret = file_dup2(filep, &filep2);
  if (ret < 0)
    {
      return ret;
    }

  /* Then allocate a new file descriptor for the inode */

  fd2 = files_allocate(filep2.f_inode, filep2.f_oflags,
                       filep2.f_pos, filep2.f_priv, minfd);
  if (fd2 < 0)
    {
      file_close(&filep2);
      return fd2;
    }

  return fd2;
}

/****************************************************************************
 * Name: nx_dup
 *
 * Description:
 *   nx_dup() is similar to the standard 'dup' interface except that is
 *   not a cancellation point and it does not modify the errno variable.
 *
 *   nx_dup() is an internal NuttX interface and should not be called from
 *   applications.
 *
 * Returned Value:
 *   The new file descriptor is returned on success; a negated errno value is
 *   returned on any failure.
 *
 ****************************************************************************/

int nx_dup(int fd)
{
  /* Check the range of the descriptor to see if we got a file or a socket
   * descriptor.
   */

  if (fd < CONFIG_NFILE_DESCRIPTORS)
    {
      FAR struct file *filep;
      int ret;

      /* Get the file structure corresponding to the file descriptor. */

      ret = fs_getfilep(fd, &filep);
      if (ret < 0)
        {
          return ret;
        }

      DEBUGASSERT(filep != NULL);

      /* Let file_dup() do the real work */

      return file_dup(filep, 0);
    }
  else
    {
      /* Not a valid file descriptor.
       * Did we get a valid socket descriptor?
       */

#ifdef CONFIG_NET
      if (fd < (CONFIG_NFILE_DESCRIPTORS + CONFIG_NSOCKET_DESCRIPTORS))
        {
          /* Yes.. dup the socket descriptor. */

          return net_dup(fd, CONFIG_NFILE_DESCRIPTORS);
        }
      else
#endif
        {
          /* No.. then it is a bad descriptor number */

          return -EBADF;
        }
    }
}

/****************************************************************************
 * Name: dup
 *
 * Description:
 *   Clone a file or socket descriptor to an arbitrary descriptor number
 *
 ****************************************************************************/

int dup(int fd)
{
  int ret;

  ret = nx_dup(fd);
  if (ret < 0)
    {
      set_errno(-ret);
      ret = ERROR;
    }

  return ret;
}
