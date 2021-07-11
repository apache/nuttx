/****************************************************************************
 * fs/vfs/fs_fstat.c
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
#include <sched.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/fs/fs.h>
#include <nuttx/mtd/mtd.h>
#include "inode/inode.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: proxy_fstat
 *
 * Description:
 *   Check for special cases where the character driver is really just a
 *   proxy for the real, underlying MTD or block driver.
 *
 *   NOTE:  This must be done here rather than in the the common
 *   inode_stat() function because the filep reference must be available
 *   in order to call the character driver ioctl method.
 *
 * Input Parameters:
 *   filep  - File structure instance
 *   inode  - The inode associated with the file descriptor
 *   buf    - The caller provide location in which to return information
 *            about the open file.
 *
 * Returned Value:
 *   Upon successful completion, 0 is returned. Otherwise, a negated errno
 *   value is returned.
 *
 ****************************************************************************/

static int proxy_fstat(FAR struct file *filep, FAR struct inode *inode,
                       FAR struct stat *buf)
{
#ifdef CONFIG_MTD
  struct mtd_geometry_s mtdgeo;
#endif
#ifndef CONFIG_DISABLE_MOUNTPOINT
  struct geometry blkgeo;
#endif
  int ret = -ENOENT;

  /* Check if this is a valid character driver */

  if (INODE_IS_DRIVER(inode) &&
      inode->u.i_ops != NULL &&
      inode->u.i_ops->ioctl != NULL)
    {
#ifdef CONFIG_MTD
      /* Check if this is a proxy for an MTD driver.  In this case, both the
       * MTDIOC_GEOMETRY ioctl and the BIOC_GEOMTRY will be supported by
       * character driver.
       */

      if (inode->u.i_ops->ioctl(filep, MTDIOC_GEOMETRY,
                                (unsigned long)((uintptr_t)&mtdgeo)) >= 0)
        {
          memset(buf, 0, sizeof(struct stat));
          buf->st_mode  = S_IFMTD;
          buf->st_mode |= S_IROTH | S_IRGRP | S_IRUSR;
          buf->st_mode |= S_IWOTH | S_IWGRP | S_IWUSR;
          buf->st_size  = mtdgeo.neraseblocks * mtdgeo.erasesize;
          ret           = OK;
        }
#ifndef CONFIG_DISABLE_MOUNTPOINT
      else
#endif
#endif

#ifndef CONFIG_DISABLE_MOUNTPOINT
      /* Check if this is a proxy for a block driver.  In this case, only
       * the BIOC_GEOMETRY ioctl will be supported.
       */

      if (inode->u.i_ops->ioctl(filep, BIOC_GEOMETRY,
                                (unsigned long)((uintptr_t)&blkgeo)) >= 0)
        {
          memset(buf, 0, sizeof(struct stat));
          buf->st_mode = S_IFBLK;
          if (inode->u.i_ops->read)
            {
              buf->st_mode |= S_IROTH | S_IRGRP | S_IRUSR;
            }

          if (inode->u.i_ops->write)
            {
              buf->st_mode |= S_IWOTH | S_IWGRP | S_IWUSR;
            }

          if (blkgeo.geo_available)
            {
              buf->st_size = blkgeo.geo_nsectors * blkgeo.geo_sectorsize;
            }

          ret = OK;
        }
#endif
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: file_fstat
 *
 * Description:
 *   file_fstat() is an internal OS interface.  It is functionally similar
 *   to the standard fstat() interface except:
 *
 *    - It does not modify the errno variable,
 *    - It is not a cancellation point,
 *    - It does not handle socket descriptors, and
 *    - It accepts a file structure instance instead of file descriptor.
 *
 * Input Parameters:
 *   filep  - File structure instance
 *   buf    - The caller provide location in which to return information
 *            about the open file.
 *
 * Returned Value:
 *   Upon successful completion, 0 shall be returned. Otherwise, -1 shall be
 *   returned and errno set to indicate the error.
 *
 ****************************************************************************/

int file_fstat(FAR struct file *filep, FAR struct stat *buf)
{
  FAR struct inode *inode;
  int ret;

  DEBUGASSERT(filep != NULL);

  /* Get the inode from the file structure */

  inode = filep->f_inode;
  DEBUGASSERT(inode != NULL);

  /* The way we handle the stat depends on the type of inode that we
   * are dealing with.
   */

#ifndef CONFIG_DISABLE_MOUNTPOINT
  if (INODE_IS_MOUNTPT(inode))
    {
      /* The inode is a file system mountpoint. Verify that the mountpoint
       * supports the fstat() method
       */

      ret = -ENOSYS;
      if (inode->u.i_mops && inode->u.i_mops->fstat)
        {
          /* Perform the fstat() operation */

          ret = inode->u.i_mops->fstat(filep, buf);
        }
    }
  else
#endif
    {
      /* Check if the inode is a proxy for a block or MTD driver */

      ret = proxy_fstat(filep, inode, buf);
      if (ret < 0)
        {
          /* The inode is part of the root pseudo file system. */

          ret = inode_stat(inode, buf, 0);
        }
    }

  return ret;
}

/****************************************************************************
 * Name: fstat
 *
 * Description:
 *   The fstat() function will obtain information about an open file
 *   associated with the file descriptor 'fd', and will write it to the area
 *   pointed to by 'buf'.
 *
 *   The 'buf' argument is a pointer to a stat structure, as defined in
 *   <sys/stat.h>, into which information is placed concerning the file.
 *
 * Input Parameters:
 *   fd  - The file descriptor associated with the open file of interest
 *   buf - The caller provide location in which to return information about
 *         the open file.
 *
 * Returned Value:
 *   Upon successful completion, 0 shall be returned. Otherwise, -1 shall be
 *   returned and errno set to indicate the error.
 *
 ****************************************************************************/

int fstat(int fd, FAR struct stat *buf)
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

#ifdef CONFIG_NET
  if (INODE_IS_SOCKET(filep->f_inode))
    {
      /* Let the networking logic handle the fstat() */

      ret = psock_fstat(sockfd_socket(fd), buf);
      if (ret < 0)
        {
          goto errout;
        }

      return OK;
    }
#endif

  /* Perform the fstat operation */

  ret = file_fstat(filep, buf);

  /* Check if the fstat operation was successful */

  if (ret >= 0)
    {
      /* Successfully fstat'ed the file */

      return OK;
    }

errout:
  set_errno(-ret);
  return ERROR;
}
