/****************************************************************************
 * fs/vfs/fs_ioctl.c
 *
 *   Copyright (C) 2007-2010, 2012-2014, 2016-2017 Gregory Nutt. All rights
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

#include <sys/ioctl.h>
#include <sched.h>
#include <errno.h>
#include <assert.h>

#include <net/if.h>

#if defined(CONFIG_NET) && CONFIG_NSOCKET_DESCRIPTORS > 0
# include <nuttx/net/net.h>
#endif

#include "inode/inode.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: file_ioctl
 *
 * Description:
 *   Perform device specific operations.
 *
 * Input Parameters:
 *   file     File structure instance
 *   req      The ioctl command
 *   arg      The argument of the ioctl cmd
 *
 * Returned Value:
 *   Returns a non-negative number on success;  A negated errno value is
 *   returned on any failure (see comments ioctl() for a list of appropriate
 *   errno values).
 *
 ****************************************************************************/

#if CONFIG_NFILE_DESCRIPTORS > 0
int file_ioctl(FAR struct file *filep, int req, unsigned long arg)
{
  FAR struct inode *inode;

  DEBUGASSERT(filep != NULL);

  /* Is a driver opened? */

  inode = filep->f_inode;
  if (!inode)
    {
      return -EBADF;
    }

  /* Does the driver support the ioctl method? */

  if (inode->u.i_ops == NULL || inode->u.i_ops->ioctl == NULL)
    {
      return -ENOTTY;
    }

  /* Yes on both accounts.  Let the driver perform the ioctl command */

  return (int)inode->u.i_ops->ioctl(filep, req, arg);
}
#endif /* CONFIG_NFILE_DESCRIPTORS > 0 */

/****************************************************************************
 * Name: ioctl/fs_ioctl
 *
 * Description:
 *   Perform device specific operations.
 *
 * Input Parameters:
 *   fd       File/socket descriptor of device
 *   req      The ioctl command
 *   arg      The argument of the ioctl cmd
 *
 * Returned Value:
 *   >=0 on success (positive non-zero values are cmd-specific)
 *   -1 on failure with errno set properly:
 *
 *   EBADF
 *     'fd' is not a valid descriptor.
 *   EFAULT
 *     'arg' references an inaccessible memory area.
 *   EINVAL
 *     'cmd' or 'arg' is not valid.
 *   ENOTTY
 *     'fd' is not associated with a character special device.
 *   ENOTTY
 *      The specified request does not apply to the kind of object that the
 *      descriptor 'fd' references.
 *
 ****************************************************************************/

#ifdef CONFIG_LIBC_IOCTL_VARIADIC
int fs_ioctl(int fd, int req, unsigned long arg)
#else
int ioctl(int fd, int req, unsigned long arg)
#endif
{
  int errcode;
#if CONFIG_NFILE_DESCRIPTORS > 0
  FAR struct file *filep;
  int ret;

  /* Did we get a valid file descriptor? */

  if ((unsigned int)fd >= CONFIG_NFILE_DESCRIPTORS)
#endif
    {
      /* Perform the socket ioctl */

#if defined(CONFIG_NET) && CONFIG_NSOCKET_DESCRIPTORS > 0
      if ((unsigned int)fd < (CONFIG_NFILE_DESCRIPTORS+CONFIG_NSOCKET_DESCRIPTORS))
        {
          ret = netdev_ioctl(fd, req, arg);
          if (ret < 0)
            {
              errcode = -ret;
              goto errout;
            }

          return ret;
        }
      else
#endif
        {
          errcode = EBADF;
          goto errout;
        }
    }

#if CONFIG_NFILE_DESCRIPTORS > 0
  /* Get the file structure corresponding to the file descriptor. */

  ret = fs_getfilep(fd, &filep);
  if (ret < 0)
    {
      errcode = -ret;
      goto errout;
    }

  DEBUGASSERT(filep != NULL);

  /* Perform the file ioctl.  If file_ioctl() fails, it will set the errno
   * value appropriately.
   */

  ret = file_ioctl(filep, req, arg);
  if (ret < 0)
    {
      errcode = -ret;
      goto errout;
    }

  return ret;
#else
  errcode = ENOTTY;
#endif

errout:
  set_errno(errcode);
  return ERROR;
}
