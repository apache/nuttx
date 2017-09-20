/****************************************************************************
 * fs/vfs/fs_stat.c
 *
 *   Copyright (C) 2007-2009, 2012, 2017 Gregory Nutt. All rights reserved.
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

#include <sys/stat.h>
#include <stdbool.h>
#include <string.h>
#include <sched.h>
#include <errno.h>

#include "inode/inode.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_PSEUDOFS_SOFTLINKS
/* Reset, preserving the number of symbolic links encountered so far */

#  define RESET_BUF(b) \
  { \
    uint16_t save = (b)->st_count; \
    memset((b), 0, sizeof(struct stat)); \
    (b)->st_count = save; \
  }
#else
/* Reset everything */

#  define RESET_BUF(b) memset((b), 0, sizeof(struct stat));
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static inline int statroot(FAR struct stat *buf);
int stat_recursive(FAR const char *path, FAR struct stat *buf);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: statroot
 ****************************************************************************/

static inline int statroot(FAR struct stat *buf)
{
  /* There is no inode associated with the fake root directory */

  RESET_BUF(buf);
  buf->st_mode = S_IFDIR | S_IROTH | S_IRGRP | S_IRUSR;
  return OK;
}

/****************************************************************************
 * Name: stat_recursive
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

int stat_recursive(FAR const char *path, FAR struct stat *buf)
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

      ret = -ret;
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
    }
  else
#endif
    {
      /* The node is part of the root pseudo file system.  This path may
       * recurse if soft links are supported in the pseudo file system.
       */

      ret = inode_stat(inode, buf);
    }

  /* Check if the stat operation was successful */

  if (ret < 0)
    {
      ret = -ret;
      goto errout_with_inode;
    }

  /* Successfully stat'ed the file */

  inode_release(inode);
  RELEASE_SEARCH(&desc);
  return OK;

/* Failure conditions always set the errno appropriately */

errout_with_inode:
  inode_release(inode);

errout_with_search:
  RELEASE_SEARCH(&desc);
  set_errno(ret);
  return ERROR;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

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

  /* Sanity checks */

  if (path == NULL  || buf == NULL)
    {
      ret = EFAULT;
      goto errout;
    }

  if (*path == '\0')
    {
      ret = ENOENT;
      goto errout;
    }

  /* Check for the fake root directory (which has no inode) */

  if (strcmp(path, "/") == 0)
    {
      return statroot(buf);
    }

  /* The perform the stat() operation on the path.  This is potentially
   * recursive if soft link support is enabled.
   */

#ifdef CONFIG_PSEUDOFS_SOFTLINKS
  buf->st_count = 0;
#endif
  return stat_recursive(path, buf);

errout:
  set_errno(ret);
  return ERROR;
}

/****************************************************************************
 * Name: inode_stat
 *
 * Description:
 *   The inode_stat() function will obtain information about an 'inode' in
 *   the pseudo file system and will write it to the area pointed to by 'buf'.
 *
 *   The 'buf' argument is a pointer to a stat structure, as defined in
 *   <sys/stat.h>, into which information is placed concerning the file.
 *
 * Input Parameters:
 *   inode - The indoe of interest
 *   buf   - The caller provide location in which to return information about
 *           the inode.
 *
 * Returned Value:
 *   Zero (OK) returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int inode_stat(FAR struct inode *inode, FAR struct stat *buf)
{
  DEBUGASSERT(inode != NULL && buf != NULL);

  /* Most of the stat entries just do not apply */

  RESET_BUF(buf);

  if (INODE_IS_SPECIAL(inode))
    {
#if defined(CONFIG_FS_NAMED_SEMAPHORES)
      if (INODE_IS_NAMEDSEM(inode))
        {
          buf->st_mode = S_IFSEM;
        }
      else
#endif
#if !defined(CONFIG_DISABLE_MQUEUE)
      if (INODE_IS_MQUEUE(inode))
        {
          buf->st_mode = S_IFMQ;
        }
      else
#endif
#if defined(CONFIG_FS_SHM)
       if (INODE_IS_SHM(inode))
        {
          buf->st_mode = S_IFSHM;
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
          int ret;

          /* Increment the link counter.  This is necesssary to avoid
           * infinite recursion if loops are encountered in the traversal.
           * If we encounter more SYMLOOP_MAX symbolic links at any time
           * during the travrsal, error out.
           *
           * NOTE: That inode_search() will automatically skip over
           * consecutive, intermediate symbolic links.  Those numbers will
           * not be included in the total.
           */

          if (++buf->st_count > SYMLOOP_MAX)
            {
              return -ELOOP;
            }

          DEBUGASSERT(buf->st_count > 0);  /* Check for unsigned integer overflow */

          /* stat() the target of the soft link. */

          ret = stat_recursive((FAR const char *)inode->u.i_link, buf);

          /* If stat() fails, then there is a problem with the target of the
           * symbolic link, but not with the symbolic link itself.  We should
           * still report success, just with less information.
           */

          if (ret < 0)
            {
              RESET_BUF(buf);
            }

          /* Make sure that the caller knows that this really a symbolic link. */

          buf->st_mode |= S_IFLNK;
        }
      else
#endif
       {
       }
    }
  else if (inode->u.i_ops != NULL)
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

      if (INODE_IS_MOUNTPT(inode))
        {
          buf->st_mode |= S_IFDIR;
        }
      else if (INODE_IS_BLOCK(inode))
        {
          /* What is if also has child inodes? */

          buf->st_mode |= S_IFBLK;
        }
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

  return OK;
}
