/****************************************************************************
 * fs/vfs/fs_ioctl.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/ioctl.h>
#include <sched.h>
#include <errno.h>
#include <fcntl.h>
#include <assert.h>

#include "inode/inode.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: file_vioctl
 ****************************************************************************/

int file_vioctl(FAR struct file *filep, int req, va_list ap)
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

  return inode->u.i_ops->ioctl(filep, req, va_arg(ap, unsigned long));
}

/****************************************************************************
 * Name: nx_vioctl
 ****************************************************************************/

static int nx_vioctl(int fd, int req, va_list ap)
{
  FAR struct file *filep;
  FAR int *arg;
  int ret;

  /* Get the file structure corresponding to the file descriptor. */

  ret = fs_getfilep(fd, &filep);
  if (ret < 0)
    {
      return ret;
    }

  DEBUGASSERT(filep != NULL);

  /* Perform the file ioctl. */

  ret = file_vioctl(filep, req, ap);

  /* Check for File system IOCTL commands that can be implemented via
   * fcntl()
   */

  if (ret == -ENOTTY)
    {
      switch (req)
        {
          case FIONBIO:
            arg = va_arg(ap, FAR int *);
            if (arg && *arg)
              {
                ret = nx_fcntl(fd, F_SETFL,
                               nx_fcntl(fd, F_GETFL) | O_NONBLOCK);
              }
            else
              {
                ret = nx_fcntl(fd, F_SETFL,
                               nx_fcntl(fd, F_GETFL) & ~O_NONBLOCK);
              }
            break;
          case FIOCLEX:
            ret = nx_fcntl(fd, F_SETFD, nx_fcntl(fd, F_GETFD) | FD_CLOEXEC);
            break;
          case FIONCLEX:
            ret = nx_fcntl(fd, F_SETFD, nx_fcntl(fd, F_GETFD) & ~FD_CLOEXEC);
            break;
        }
    }

  return ret;
}

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
 *   ap       The argument of the ioctl cmd
 *
 * Returned Value:
 *   Returns a non-negative number on success;  A negated errno value is
 *   returned on any failure (see comments ioctl() for a list of appropriate
 *   errno values).
 *
 ****************************************************************************/

int file_ioctl(FAR struct file *filep, int req, ...)
{
  va_list ap;
  int ret;

  /* Let file_vioctl() do the real work. */

  va_start(ap, req);
  ret = file_vioctl(filep, req, ap);
  va_end(ap);

  return ret;
}

/****************************************************************************
 * Name: nx_ioctl
 *
 * Description:
 *   nx_ioctl() is similar to the standard 'ioctl' interface except that is
 *   not a cancellation point and it does not modify the errno variable.
 *
 *   nx_ioctl() is an internal NuttX interface and should not be called from
 *   applications.
 *
 * Returned Value:
 *   Returns a non-negative number on success;  A negated errno value is
 *   returned on any failure (see comments ioctl() for a list of appropriate
 *   errno values).
 *
 ****************************************************************************/

int nx_ioctl(int fd, int req, ...)
{
  va_list ap;
  int ret;

  /* Let nx_vioctl() do the real work. */

  va_start(ap, req);
  ret = nx_vioctl(fd, req, ap);
  va_end(ap);

  return ret;
}

/****************************************************************************
 * Name: ioctl
 *
 * Description:
 *   Perform device specific operations.
 *
 * Input Parameters:
 *   fd       File/socket descriptor of device
 *   req      The ioctl command
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

int ioctl(int fd, int req, ...)
{
  va_list ap;
  int ret;

  /* Let nx_vioctl() do the real work. */

  va_start(ap, req);
  ret = nx_vioctl(fd, req, ap);
  va_end(ap);

  if (ret < 0)
    {
      set_errno(-ret);
      ret = ERROR;
    }

  return ret;
}
