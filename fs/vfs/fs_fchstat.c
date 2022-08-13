/****************************************************************************
 * fs/vfs/fs_fchstat.c
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

#include <sys/stat.h>
#include <unistd.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/fs/fs.h>

#include "inode/inode.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fchstat
 ****************************************************************************/

static int fchstat(int fd, FAR struct stat *buf, int flags)
{
  FAR struct file *filep;
  int ret;

  /* First, get the file structure.  Note that on failure,
   * fs_getfilep() will return the errno.
   */

  ret = fs_getfilep(fd, &filep);
  if (ret < 0)
    {
      goto errout;
    }

  /* Perform the fchstat operation */

  ret = file_fchstat(filep, buf, flags);
  if (ret >= 0)
    {
      /* Successfully fchstat'ed the file */

      return OK;
    }

errout:
  set_errno(-ret);
  return ERROR;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: file_fchstat
 *
 * Description:
 *   file_fchstat() is an internal OS interface. It is functionally similar
 *   to the combination of fchmod/fchown/futimens standard interface except:
 *
 *    - It does not modify the errno variable,
 *    - It is not a cancellation point,
 *    - It does not handle socket descriptors, and
 *    - It accepts a file structure instance instead of file descriptor.
 *
 * Input Parameters:
 *   filep  - File structure instance
 *   buf    - The stat to be modified
 *   flags  - The valid field in buf
 *
 * Returned Value:
 *   Upon successful completion, 0 shall be returned. Otherwise, the
 *   negative errno shall be returned to indicate the error.
 *
 ****************************************************************************/

int file_fchstat(FAR struct file *filep, FAR struct stat *buf, int flags)
{
  FAR struct inode *inode;
  int ret;

  DEBUGASSERT(filep != NULL);

  /* Get the inode from the file structure */

  inode = filep->f_inode;
  DEBUGASSERT(inode != NULL);

  /* Adjust and check buf and flags */

  if ((flags & CH_STAT_MODE) && (buf->st_mode & ~0177777))
    {
      return -EINVAL;
    }

  if ((flags & CH_STAT_UID) && buf->st_uid == -1)
    {
      flags &= ~CH_STAT_UID;
    }

  if ((flags & CH_STAT_GID) && buf->st_gid == -1)
    {
      flags &= ~CH_STAT_GID;
    }

  clock_gettime(CLOCK_REALTIME, &buf->st_ctim);

  if (flags & CH_STAT_ATIME)
    {
      if (buf->st_atim.tv_nsec == UTIME_OMIT)
        {
          flags &= ~CH_STAT_ATIME;
        }
      else if (buf->st_atim.tv_nsec == UTIME_NOW)
        {
          buf->st_atim = buf->st_ctim;
        }
      else if (buf->st_atim.tv_nsec >= 1000000000)
        {
          return -EINVAL;
        }
    }

  if (flags & CH_STAT_MTIME)
    {
      if (buf->st_mtim.tv_nsec == UTIME_OMIT)
        {
          flags &= ~CH_STAT_MTIME;
        }
      else if (buf->st_mtim.tv_nsec == UTIME_NOW)
        {
          buf->st_mtim = buf->st_ctim;
        }
      else if (buf->st_mtim.tv_nsec >= 1000000000)
        {
          return -EINVAL;
        }
    }

  /* The way we handle the chstat depends on the type of inode that we
   * are dealing with.
   */

#ifndef CONFIG_DISABLE_MOUNTPOINT
  if (INODE_IS_MOUNTPT(inode))
    {
      /* The inode is a file system mountpoint. Verify that the mountpoint
       * supports the fchstat() method
       */

      if (inode->u.i_mops && inode->u.i_mops->fchstat)
        {
          /* Perform the fchstat() operation */

          ret = inode->u.i_mops->fchstat(filep, buf, flags);
        }
      else
        {
          ret = -ENOSYS;
        }
    }
  else
#endif
    {
      /* The inode is part of the root pseudo file system. */

      ret = inode_chstat(inode, buf, flags, 0);
    }

  return ret;
}

/****************************************************************************
 * Name: fchmod
 *
 * Description:
 *   The fchmod() function shall be equivalent to chmod() except that the
 *   file whose permissions are changed is specified by the file descriptor.
 *
 * Input Parameters:
 *   fd   - Specifies the fd to be modified
 *   mode - Specifies the permission to set
 *
 * Returned Value:
 *   Upon successful completion, fchmod() shall return 0.
 *   Otherwise, it shall return -1 and set errno to indicate the error.
 *
 ****************************************************************************/

int fchmod(int fd, mode_t mode)
{
  struct stat buf;

  buf.st_mode = mode;

  return fchstat(fd, &buf, CH_STAT_MODE);
}

/****************************************************************************
 * Name: fchown
 *
 * Description:
 *   The fchown() function shall be equivalent to chown() except that the
 *   file whose owner and group are changed is specified by the file
 *   descriptor.
 *
 * Input Parameters:
 *   fd    - Specifies the fd to be modified
 *   owner - Specifies the owner to set
 *   group - Specifies the group to set
 *
 * Returned Value:
 *   Upon successful completion, fchown() shall return 0.
 *   Otherwise, it shall return -1 and set errno to indicate the error.
 *
 ****************************************************************************/

int fchown(int fd, uid_t owner, gid_t group)
{
  struct stat buf;

  buf.st_uid = owner;
  buf.st_gid = group;

  return fchstat(fd, &buf, CH_STAT_UID | CH_STAT_GID);
}

/****************************************************************************
 * Name: futimens
 *
 * Description:
 *   futimens() update the timestamps of a file with nanosecond precision.
 *   This contrasts with the historical utime(2) and utimes(2), which permit
 *   only second and microsecond precision, respectively, when setting file
 *   timestamps.
 *
 * Input Parameters:
 *   fd    - Specifies the fd to be modified
 *   times - Specifies the time value to set
 *
 * Returned Value:
 *   On success, futimens() return 0.
 *   On error, -1 is returned and errno is set to indicate the error.
 *
 ****************************************************************************/

int futimens(int fd, FAR const struct timespec times[2])
{
  struct stat buf;

  if (times != NULL)
    {
      buf.st_atim = times[0];
      buf.st_mtim = times[1];
    }
  else
    {
      buf.st_atim.tv_nsec = UTIME_NOW;
      buf.st_mtim.tv_nsec = UTIME_NOW;
    }

  return fchstat(fd, &buf, CH_STAT_ATIME | CH_STAT_MTIME);
}
