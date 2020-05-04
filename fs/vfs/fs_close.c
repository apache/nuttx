/****************************************************************************
 * fs/vfs/fs_close.c
 *
 *   Copyright (C) 2007-2009, 2012, 2016 Gregory Nutt. All rights reserved.
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

#include <nuttx/cancelpt.h>
#include <nuttx/fs/fs.h>

#ifdef CONFIG_NET
# include <nuttx/net/net.h>
#endif

#include "inode/inode.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nx_close
 *
 * Description:
 *   nx_close() is similar to the standard 'close' interface except that is
 *   not a cancellation point and it does not modify the errno variable.
 *
 *   nx_close() is an internal NuttX interface and should not be called from
 *   applications.
 *
 * Returned Value:
 *   The new file descriptor is returned on success; a negated errno value is
 *   returned on any failure.
 *
 ****************************************************************************/

int nx_close(int fd)
{
  /* Did we get a valid file descriptor? */

  if (fd >= CONFIG_NFILE_DESCRIPTORS)
    {
      /* Close a socket descriptor */

#ifdef CONFIG_NET
      if (fd < (CONFIG_NFILE_DESCRIPTORS + CONFIG_NSOCKET_DESCRIPTORS))
        {
          return net_close(fd);
        }
      else
#endif
        {
          return -EBADF;
        }
    }

  /* Close the driver or mountpoint.  NOTES: (1) there is no
   * exclusion mechanism here, the driver or mountpoint must be
   * able to handle concurrent operations internally, (2) The driver
   * may have been opened numerous times (for different file
   * descriptors) and must also handle being closed numerous times.
   * (3) for the case of the mountpoint, we depend on the close
   * methods bing identical in signature and position in the operations
   * vtable.
   */

  return files_close(fd);
}

/****************************************************************************
 * Name: close
 *
 * Description:
 *   close() closes a file descriptor, so that it no longer refers to any
 *   file and may be reused. Any record locks (see fcntl(2)) held on the file
 *   it was associated with, and owned by the process, are removed
 *   (regardless of the file descriptor that was used to obtain the lock).
 *
 *   If fd is the last copy of a particular file descriptor the resources
 *   associated with it are freed; if the descriptor was the last reference
 *   to a file which has been removed using unlink(2) the file is deleted.
 *
 * Input Parameters:
 *   fd   file descriptor to close
 *
 * Returned Value:
 *   0 on success; -1 on error with errno set appropriately.
 *
 * Assumptions:
 *
 ****************************************************************************/

int close(int fd)
{
  int ret;

  /* close() is a cancellation point */

  enter_cancellation_point();

  ret = nx_close(fd);
  if (ret < 0)
    {
      set_errno(-ret);
      ret = ERROR;
    }

  leave_cancellation_point();
  return ret;
}
