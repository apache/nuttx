/****************************************************************************
 * fs/vfs/fs_ioctl.c
 *
 * SPDX-License-Identifier: Apache-2.0
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
#include <limits.h>
#include <net/if.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <termios.h>

#include <nuttx/audio/audio.h>
#include <nuttx/drivers/rpmsgdev.h>
#include <nuttx/input/mouse.h>
#include <nuttx/mtd/mtd.h>
#include <nuttx/net/ioctl.h>
#include <nuttx/power/battery_ioctl.h>
#include <nuttx/video/fb.h>

#include "inode/inode.h"
#include "vfs.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ioctl_arglen_add
 ****************************************************************************/

static ssize_t ioctl_arglen_add(size_t header, size_t data)
{
  if (data > SSIZE_MAX - header)
    {
      return -EOVERFLOW;
    }

  return header + data;
}

/****************************************************************************
 * Name: file_vioctl
 ****************************************************************************/

static int file_vioctl(FAR struct file *filep, int req, va_list ap)
{
  FAR struct inode *inode;
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

#ifdef CONFIG_FS_IOCTL_FILEPATH
  /* A driver may return OK without filling the path buffer */

  if (req == FIOC_FILEPATH && arg != 0)
    {
      *(FAR char *)(uintptr_t)arg = '\0';
    }
#else
  if (req == FIOC_FILEPATH)
    {
      return -ENOTTY;
    }
#endif

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

#ifdef CONFIG_FS_IOCTL_FILEPATH
      case FIOC_FILEPATH:
        if (ret == -ENOTTY && !INODE_IS_MOUNTPT(inode))
          {
            ret = inode_getpath(inode, (FAR char *)(uintptr_t)arg, PATH_MAX);
          }
        break;
#endif

      case FIOC_GETLK:
        if (ret == -ENOTTY)
          {
            ret = file_getlk(filep, (FAR struct flock *)(uintptr_t)arg);
          }
        break;

      case FIOC_SETLK:
        if (ret == -ENOTTY)
          {
            ret = file_setlk(filep, (FAR struct flock *)(uintptr_t)arg,
                             true);
          }
        break;

      case FIOC_SETLKW:
        if (ret == -ENOTTY)
          {
            ret = file_setlk(filep, (FAR struct flock *)(uintptr_t)arg,
                             false);
          }
        break;

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
        break;

      case BIOC_BLKGETSIZE:
        if (ret == -ENOTTY && inode->u.i_ops != NULL &&
            inode->u.i_ops->ioctl != NULL)
          {
            struct geometry geo;

            ret = inode->u.i_ops->ioctl(filep, BIOC_GEOMETRY,
                                        (unsigned long)(uintptr_t)&geo);
            if (ret >= 0)
              {
                *(FAR blkcnt_t *)(uintptr_t)arg = geo.geo_nsectors;
              }
          }
        break;

#endif
    }

  return ret;
}

/****************************************************************************
 * Name: nx_vioctl
 ****************************************************************************/

static int nx_vioctl(int fd, int req, va_list ap)
{
  FAR struct file *filep;
  FAR struct fd *fdp;
  FAR int *nonblockp = NULL;
  int nonblock;
  int ret;

  /* FIONBIO passes a pointer to an int.  Read the int before file_get2(),
   * so that a bad pointer faults with no file reference held, and give the
   * driver a pointer to the copy.
   */

  if (req == FIONBIO)
    {
      nonblockp = va_arg(ap, FAR int *);
      if (nonblockp != NULL)
        {
          nonblock  = *nonblockp;
          nonblockp = &nonblock;
        }
    }

  ret = file_get2(fd, &filep, &fdp);
  if (ret < 0)
    {
      return ret;
    }

  switch (req)
    {
      case FIOCLEX:
        fdp->f_cloexec = true;
        break;

      case FIONCLEX:
        fdp->f_cloexec = false;
        break;

      case FIOGCLEX:
        *va_arg(ap, FAR int *) = fdp->f_cloexec ? FD_CLOEXEC : 0;
        break;

#ifdef CONFIG_FDSAN
      case FIOC_SETTAG_FDSAN:
        fdp->f_tag_fdsan = *va_arg(ap, FAR uint64_t *);
        break;

      case FIOC_GETTAG_FDSAN:
        *va_arg(ap, FAR uint64_t *) = fdp->f_tag_fdsan;
        break;
#endif

#ifdef CONFIG_FDCHECK
      case FIOC_SETTAG_FDCHECK:
        fdp->f_tag_fdcheck = *va_arg(ap, FAR uint8_t *);
        break;

      case FIOC_GETTAG_FDCHECK:
        *va_arg(ap, FAR uint8_t *) = fdp->f_tag_fdcheck;
        break;
#endif

      case FIONBIO:
        ret = file_ioctl(filep, req, (unsigned long)(uintptr_t)nonblockp);
        break;

      default:
        ret = file_vioctl(filep, req, ap);
        break;
    }

  file_put(filep);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ioctl_arglen
 *
 * Description:
 *   Calculate the size of a contiguous device ioctl argument buffer.
 *
 * Input Parameters:
 *   cmd - The ioctl command
 *   arg - The argument at the device ioctl entry point
 *
 * Returned Value:
 *   Positive buffer size, zero for a value argument, or a negated errno.
 *
 ****************************************************************************/

ssize_t ioctl_arglen(int cmd, unsigned long arg)
{
  switch (cmd)
    {
      case FIONBIO:
      case FIONWRITE:
      case FIONREAD:
      case FIONSPACE:
      case FBIOGET_POWER:
      case BATIOC_CURRENT:
      case BATIOC_VOLTAGE:
      case BATIOC_TEMPERATURE:
      case BATIOC_INPUT_CURRENT:
      case BATIOC_STATE:
      case BATIOC_CAPACITY:
      case BATIOC_CHIPID:
      case BATIOC_GET_PROTOCOL:
        return sizeof(int);

      case BATIOC_ONLINE:
        return sizeof(bool);

      case FIOC_FILEPATH:
        return PATH_MAX;

      case TUNSETIFF:
      case TUNGETIFF:
        return sizeof(struct ifreq);

      case FBIOSET_POWER:
      case TCDRN:
      case TCFLSH:
      case MSIOC_GRAB:
      case BIOC_PROBE:
      case BIOC_EJECT:
      case BIOC_LLFORMAT:
      case BIOC_ALLOCSECT:
      case BIOC_FREESECT:
      case BIOC_FLUSH:
      case MTDIOC_BULKERASE:
        return 0;

      case TCGETS:
      case TCSETS:
        return sizeof(struct termios);

      case FIOC_SETLK:
      case FIOC_GETLK:
      case FIOC_SETLKW:
        return sizeof(struct flock);

      case BIOC_XIPBASE:
        return sizeof(uintptr_t);

      case BIOC_PARTINFO:
        return sizeof(struct partition_info_s);

      case BIOC_BLKSSZGET:
        return sizeof(blksize_t);

      case MTDIOC_ERASESTATE:
        return sizeof(uint8_t);

      case MTDIOC_PROTECT:
      case MTDIOC_UNPROTECT:
        return sizeof(struct mtd_protect_s);

      case BATIOC_OPERATE:
        return sizeof(struct batio_operate_msg_s);

      case MSIOC_VENDOR:
        {
          FAR const struct mouse_vendor_cmd_s *req =
            (FAR const struct mouse_vendor_cmd_s *)(uintptr_t)arg;

          if (req == NULL)
            {
              return -EINVAL;
            }

          return ioctl_arglen_add(sizeof(*req), req->len);
        }

#ifdef CONFIG_DEV_RPMSG
      case RPMSGDEV_PRIVATE_IOCTL:
        {
          FAR const struct rpmsgdev_private_ioctl *req =
            (FAR const struct rpmsgdev_private_ioctl *)(uintptr_t)arg;

          if (req == NULL)
            {
              return -EINVAL;
            }

          return ioctl_arglen_add(sizeof(*req), req->len);
        }
#endif

#ifdef CONFIG_AUDIO
      case AUDIOIOC_HWRESET:
        return 0;

      case AUDIOIOC_GETBUFFERINFO:
      case AUDIOIOC_SETBUFFERINFO:
        return sizeof(struct ap_buffer_info_s);

      case AUDIOIOC_GETAUDIOINFO:
        return sizeof(struct audio_info_s);

      case AUDIOIOC_SETPARAMTER:
        {
          size_t len;

          if (arg == 0)
            {
              return -EINVAL;
            }

          len = strnlen((FAR const char *)(uintptr_t)arg, SSIZE_MAX);
          return ioctl_arglen_add(1, len);
        }

#endif

      default:
        return -ENOTTY;
    }
}

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
  va_list ap;
  int ret;

  va_start(ap, req);

  /* Let nx_vioctl() do the real work. */

  ret = nx_vioctl(fd, req, ap);
  if (ret < 0)
    {
      set_errno(-ret);
      ret = ERROR;
    }

  va_end(ap);

  return ret;
}
