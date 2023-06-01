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

static int file_vioctl(FAR struct file *filep, int req, va_list ap)
{
  FAR struct inode *inode;
#ifdef CONFIG_FDSAN
  FAR uint64_t *tag;
#endif
  unsigned long arg;
  int ret = -ENOTTY;

  DEBUGASSERT(filep != NULL);

  arg = va_arg(ap, unsigned long);

  /* Is a driver opened? */

  inode = filep->f_inode;
  if (!inode)
    {
      return -EBADF;
    }

  /* Does the driver support the ioctl method? */

  if (inode->u.i_ops != NULL && inode->u.i_ops->ioctl != NULL)
    {
      /* Yes on both accounts.  Let the driver perform the ioctl command */

      ret = inode->u.i_ops->ioctl(filep, req, arg);
    }

  switch (req)
    {
      case FIONBIO:
        if (ret == OK || ret == -ENOTTY)
          {
            FAR int *nonblock = (FAR int *)(uintptr_t)arg;
            if (nonblock && *nonblock)
              {
                filep->f_oflags |= O_NONBLOCK;
              }
            else
              {
                filep->f_oflags &= ~O_NONBLOCK;
              }

            ret = OK;
          }
        break;

      case FIOCLEX:
        if (ret == OK || ret == -ENOTTY)
          {
            filep->f_oflags |= O_CLOEXEC;
            ret = OK;
          }
        break;

      case FIONCLEX:
        if (ret == OK || ret == -ENOTTY)
          {
            filep->f_oflags &= ~O_CLOEXEC;
            ret = OK;
          }
        break;

      case FIOC_FILEPATH:
        if (ret == -ENOTTY && !INODE_IS_MOUNTPT(inode))
          {
            ret = inode_getpath(inode, (FAR char *)(uintptr_t)arg, PATH_MAX);
          }
        break;

#ifdef CONFIG_FDSAN
      case FIOC_SETTAG:
        tag = (FAR uint64_t *)arg;
        filep->f_tag = *tag;
        ret = OK;
        break;

      case FIOC_GETTAG:
        tag = (FAR uint64_t *)arg;
        *tag = filep->f_tag;
        ret = OK;
        break;
#endif

#ifndef CONFIG_DISABLE_MOUNTPOINT
      case BIOC_BLKSSZGET:
        if (ret == -ENOTTY && inode->u.i_ops != NULL &&
            inode->u.i_ops->ioctl != NULL)
          {
            struct geometry geo;
            ret = inode->u.i_ops->ioctl(filep, BIOC_GEOMETRY,
                                        (unsigned long)(uintptr_t)&geo);
            if (ret >= 0)
              {
                *(FAR blksize_t *)(uintptr_t)arg = geo.geo_sectorsize;
              }
          }
#endif
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
  FAR struct file *filep;
  va_list ap;
  int ret;

  /* Get the file structure corresponding to the file descriptor. */

  ret = fs_getfilep(fd, &filep);
  if (ret < 0)
    {
      goto err;
    }

  /* Let file_vioctl() do the real work. */

  va_start(ap, req);
  ret = file_vioctl(filep, req, ap);
  va_end(ap);

  if (ret < 0)
    {
      goto err;
    }

  return ret;

err:
  set_errno(-ret);
  return ERROR;
}
