/****************************************************************************
 * fs/vfs/fs_stat.c
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
#include <stdbool.h>
#include <string.h>
#include <sched.h>
#include <assert.h>
#include <errno.h>

#include "inode/inode.h"
#include <nuttx/mtd/mtd.h>
#include <nuttx/fs/ioctl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RESET_BUF(b) memset((b), 0, sizeof(struct stat));

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int stat_recursive(FAR const char *path,
                          FAR struct stat *buf, int resolve);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stat_recursive
 *
 * Input Parameters:
 *   path    - The inode of interest
 *   buf     - The caller-provided location in which to return information
 *             about the inode.
 *   resolve - Whether to resolve the symbolic link:
 *               0: Don't resolve the symbolic line
 *               1: Resolve the symbolic link
 *             >=2: The recursive count in the resolving process
 *
 * Returned Value:
 *   Zero on success; < 0 on failure:
 *
 *   EACCES  Search permission is denied for one of the directories in the
 *           path prefix of path.
 *   EFAULT  Bad address.
 *   ENOENT  A component of the path path does not exist, or the path is an
 *           empty string.
 *   ENOMEM  Out of memory
 *   ENOTDIR A component of the path is not a directory.
 *
 ****************************************************************************/

static int stat_recursive(FAR const char *path,
                          FAR struct stat *buf, int resolve)
{
  struct inode_search_s desc;
  FAR struct inode *inode;
  int ret;

  /* Get an inode for this path */

  SETUP_SEARCH(&desc, path, true);

  ret = inode_find(&desc);
  if (ret < 0)
    {
      /* This name does not refer to an inode in the pseudo file system and
       * there is no mountpoint that includes in this path.
       */

      goto errout_with_search;
    }

  /* Get the search results */

  inode = desc.node;
  DEBUGASSERT(inode != NULL);

  /* The way we handle the stat depends on the type of inode that we
   * are dealing with.
   */

#ifndef CONFIG_DISABLE_MOUNTPOINT
  if (INODE_IS_MOUNTPT(inode))
    {
      /* The node is a file system mointpoint. Verify that the mountpoint
       * supports the stat() method
       */

      if (inode->u.i_mops && inode->u.i_mops->stat)
        {
          /* Perform the stat() operation */

          ret = inode->u.i_mops->stat(inode, desc.relpath, buf);
        }
      else
        {
          ret = -ENOSYS;
        }
    }
  else
#endif
    {
      /* The node is part of the root pseudo file system.  This path may
       * recurse if soft links are supported in the pseudo file system.
       */

      ret = inode_stat(inode, buf, resolve);
    }

  inode_release(inode);
errout_with_search:
  RELEASE_SEARCH(&desc);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nx_stat
 *
 * Description:
 *   nx_stat() is similar to the standard 'stat' interface except that is
 *   not a cancellation point and it does not modify the errno variable.
 *
 *   nx_stat() is an internal NuttX interface and should not be called from
 *   applications.
 *
 * Returned Value:
 *   Zero is returned on success; a negated value is returned on any failure.
 *
 ****************************************************************************/

int nx_stat(FAR const char *path, FAR struct stat *buf, int resolve)
{
  /* Sanity checks */

  if (path == NULL  || buf == NULL)
    {
      return -EFAULT;
    }

  if (*path == '\0')
    {
      return -ENOENT;
    }

  /* The perform the stat() operation on the path.  This is potentially
   * recursive if soft link support is enabled.
   */

  return stat_recursive(path, buf, resolve);
}

/****************************************************************************
 * Name: stat
 *
 * Returned Value:
 *   Zero on success; -1 on failure with errno set:
 *
 *   EACCES  Search permission is denied for one of the directories in the
 *           path prefix of path.
 *   EFAULT  Bad address.
 *   ENOENT  A component of the path path does not exist, or the path is an
 *           empty string.
 *   ENOMEM  Out of memory
 *   ENOTDIR A component of the path is not a directory.
 *
 ****************************************************************************/

int stat(FAR const char *path, FAR struct stat *buf)
{
  int ret;

  ret = nx_stat(path, buf, 1);
  if (ret < 0)
    {
      set_errno(-ret);
      ret = ERROR;
    }

  return ret;
}

int lstat(FAR const char *path, FAR struct stat *buf)
{
  int ret;

  ret = nx_stat(path, buf, 0);
  if (ret < 0)
    {
      set_errno(-ret);
      ret = ERROR;
    }

  return ret;
}

/****************************************************************************
 * Name: inode_stat
 *
 * Description:
 *   The inode_stat() function will obtain information about an 'inode' in
 *   the pseudo file system and write it to the area pointed to by 'buf'.
 *
 *   The 'buf' argument is a pointer to a stat structure, as defined in
 *   <sys/stat.h>, into which information is placed concerning the file.
 *
 * Input Parameters:
 *   inode   - The inode of interest
 *   buf     - The caller-provided location in which to return information
 *             about the inode.
 *   resolve - Whether to resolve the symbolic link:
 *               0: Don't resolve the symbolic line
 *               1: Resolve the symbolic link
 *             >=2: The recursive count in the resolving process
 *
 * Returned Value:
 *   Zero (OK) returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int inode_stat(FAR struct inode *inode, FAR struct stat *buf, int resolve)
{
  DEBUGASSERT(inode != NULL && buf != NULL);

  /* Most of the stat entries just do not apply */

  RESET_BUF(buf);

  /* Handle "special" nodes */

#if defined(CONFIG_FS_NAMED_SEMAPHORES)
  /* Check for a named semaphore */

  if (INODE_IS_NAMEDSEM(inode))
    {
      buf->st_mode = S_IFSEM;
    }
  else
#endif
#if !defined(CONFIG_DISABLE_MQUEUE) || !defined(CONFIG_DISABLE_MQUEUE_SYSV)
  /* Check for a message queue */

  if (INODE_IS_MQUEUE(inode))
    {
      buf->st_mode = S_IFMQ;
    }
  else
#endif
#if defined(CONFIG_FS_SHM)
  /* Check for shared memory */

  if (INODE_IS_SHM(inode))
    {
      buf->st_mode = S_IFSHM;
    }
  else
#endif
#if defined(CONFIG_MTD)
  /* Check for an MTD driver */

  if (INODE_IS_MTD(inode))
    {
      struct mtd_geometry_s mtdgeo;

      buf->st_mode  = S_IFMTD;
      buf->st_mode |= S_IROTH | S_IRGRP | S_IRUSR;
      buf->st_mode |= S_IWOTH | S_IWGRP | S_IWUSR;

      if (inode->u.i_mtd != NULL &&
          MTD_IOCTL(inode->u.i_mtd, MTDIOC_GEOMETRY,
                    (unsigned long)((uintptr_t)&mtdgeo)) >= 0)
        {
          buf->st_size = mtdgeo.neraseblocks * mtdgeo.erasesize;
        }
    }
  else
#endif
#ifdef CONFIG_PSEUDOFS_SOFTLINKS
  /* Handle softlinks differently.  Just call stat() recursively on the
   * target of the softlink.
   *
   * REVISIT: This has the possibility of an infinite loop!
   */

  if (INODE_IS_SOFTLINK(inode))
    {
      if (resolve)
        {
          int ret;

          /* Increment the link counter.  This is necessary to avoid
           * infinite recursion if loops are encountered in the
           * traversal. If we encounter more SYMLOOP_MAX symbolic links
           * at any time during the traversal, error out.
           *
           * NOTE: That inode_search() will automatically skip over
           * consecutive, intermediate symbolic links.  Those numbers
           * will not be included in the total.
           */

          if (resolve > SYMLOOP_MAX)
            {
              return -ELOOP;
            }

          /* stat() the target of the soft link. */

          ret = stat_recursive(inode->u.i_link, buf, ++resolve);

          /* If stat() fails, then there is a problem with the target of
           * the symbolic link, but not with the symbolic link itself.
           * We should still report success, just with less information.
           */

          if (ret < 0)
            {
              RESET_BUF(buf);
            }

          return ret;
        }
      else
        {
          /* Make sure the caller knows that this is a symbolic link. */

          buf->st_mode = S_IRWXO | S_IRWXG | S_IRWXU | S_IFLNK;
        }
    }
  else
#endif

  /* Handle "normal inodes */

  if (inode->u.i_ops != NULL)
    {
      /* Determine read/write privileges based on the existence of read
       * and write methods.
       */

      if (inode->u.i_ops->read)
        {
          buf->st_mode = S_IROTH | S_IRGRP | S_IRUSR;
        }

      if (inode->u.i_ops->write)
        {
          buf->st_mode |= S_IWOTH | S_IWGRP | S_IWUSR;
        }

      /* Determine the type of the inode */

      /* Check for a mountpoint and a pseudo dir */

      if (INODE_IS_MOUNTPT(inode) || INODE_IS_PSEUDODIR(inode))
        {
          buf->st_mode |= S_IFDIR;
        }

      /* Check for a block driver */

      else if (INODE_IS_BLOCK(inode))
        {
          /* What is if also has child inodes? */

          buf->st_mode |= S_IFBLK;

#ifndef CONFIG_DISABLE_MOUNTPOINT
          if ((inode->u.i_bops != NULL) &&
              (inode->u.i_bops->geometry != NULL))
            {
              struct geometry geo;
              if (inode->u.i_bops->geometry(inode, &geo) >= 0 &&
                  geo.geo_available)
                {
                  buf->st_size = geo.geo_nsectors * geo.geo_sectorsize;
                }
            }
#endif
        }

      /* Otherwise, the node must refer to a character driver */

      else /* if (INODE_IS_DRIVER(inode)) */
        {
          /* What is it if it also has child inodes? */

          buf->st_mode |= S_IFCHR;
        }
    }
  else
    {
      /* If it has no operations, then it must just be a intermediate
       * node in the inode tree.  It is something like a directory.
       * We'll say that all pseudo-directories are read-able but not
       * write-able.
       */

      buf->st_mode |= S_IFDIR | S_IROTH | S_IRGRP | S_IRUSR;
    }

#ifdef CONFIG_PSEUDOFS_ATTRIBUTES
  buf->st_mode |= inode->i_mode;
  buf->st_uid   = inode->i_owner;
  buf->st_gid   = inode->i_group;
  buf->st_atim  = inode->i_atime;
  buf->st_mtim  = inode->i_mtime;
  buf->st_ctim  = inode->i_ctime;
#endif
  buf->st_ino   = inode->i_ino;

  return OK;
}
