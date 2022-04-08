/****************************************************************************
 * fs/vfs/fs_chstat.c
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
 * Name: chstat_recursive
 ****************************************************************************/

static int chstat_recursive(FAR const char *path,
                            FAR const struct stat *buf,
                            int flags, int resolve)
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

  /* The way we handle the chstat depends on the type of inode that we
   * are dealing with.
   */

#ifndef CONFIG_DISABLE_MOUNTPOINT
  if (INODE_IS_MOUNTPT(inode))
    {
      /* The node is a file system mointpoint. Verify that the mountpoint
       * supports the chstat() method
       */

      if (inode->u.i_mops && inode->u.i_mops->chstat)
        {
          /* Perform the chstat() operation */

          ret = inode->u.i_mops->chstat(inode, desc.relpath, buf, flags);
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

      ret = inode_chstat(inode, buf, flags, resolve);
    }

  inode_release(inode);

errout_with_search:
  RELEASE_SEARCH(&desc);
  return ret;
}

/****************************************************************************
 * Name: fchstat
 ****************************************************************************/

static int chstat(FAR const char *path,
                  FAR struct stat *buf, int flags, int resolve)
{
  int ret = -EINVAL;

  /* Adjust and check buf and flags */

  if ((flags & CH_STAT_MODE) && (buf->st_mode & ~0177777))
    {
      goto errout;
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
          goto errout;
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
          goto errout;
        }
    }

  /* Perform the chstat operation */

  ret = chstat_recursive(path, buf, flags, resolve);
  if (ret >= 0)
    {
      /* Successfully chstat'ed the file */

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
 * Name: chmod
 *
 * Description:
 *   The chmod() function changes S_ISUID, S_ISGID, S_ISVTX and the file
 *   permission bits of the file named by the pathname pointed to by the
 *   path argument to the corresponding bits in the mode argument. The
 *   effective user ID of the process must match the owner of the file or
 *   the process must have appropriate privileges in order to do this.
 *
 * Input Parameters:
 *   path - Specifies the file to be modified
 *   mode - Specifies the permission to set
 *
 * Returned Value:
 *   Upon successful completion, chmod() shall return 0.
 *   Otherwise, it shall return -1 and set errno to indicate the error.
 *
 ****************************************************************************/

int chmod(FAR const char *path, mode_t mode)
{
  struct stat buf;

  buf.st_mode = mode;

  return chstat(path, &buf, CH_STAT_MODE, 1);
}

/****************************************************************************
 * Name: lchmod
 *
 * Description:
 *   The lchmod() system call is similar to chmod() but does not follow
 *   the symbolic links.
 *
 * Input Parameters:
 *   path - Specifies the file to be modified
 *   mode - Specifies the permission to set
 *
 * Returned Value:
 *   Upon successful completion, lchmod() shall return 0.
 *   Otherwise, it shall return -1 and set errno to indicate the error.
 *
 ****************************************************************************/

int lchmod(FAR const char *path, mode_t mode)
{
  struct stat buf;

  buf.st_mode = mode;

  return chstat(path, &buf, CH_STAT_MODE, 0);
}

/****************************************************************************
 * Name: chown
 *
 * Description:
 *   The chown() function shall change the user and group ownership of a
 *   file. Only processes with an effective user ID equal to the user ID
 *   of the file or with appropriate privileges may change the ownership
 *   of a file.
 *
 * Input Parameters:
 *   path  - Specifies the file to be modified
 *   owner - Specifies the owner to set
 *   group - Specifies the group to set
 *
 * Returned Value:
 *   Upon successful completion, chown() shall return 0.
 *   Otherwise, it shall return -1 and set errno to indicate the error.
 *
 ****************************************************************************/

int chown(FAR const char *path, uid_t owner, gid_t group)
{
  struct stat buf;

  buf.st_uid = owner;
  buf.st_gid = group;

  return chstat(path, &buf, CH_STAT_UID | CH_STAT_GID, 1);
}

/****************************************************************************
 * Name: lchown
 *
 * Description:
 *   The lchown() system call is similar to chown() but does not follow
 *   the symbolic links.
 *
 * Input Parameters:
 *   path  - Specifies the file to be modified
 *   owner - Specifies the owner to set
 *   group - Specifies the group to set
 *
 * Returned Value:
 *   Upon successful completion, lchown() shall return 0.
 *   Otherwise, it shall return -1 and set errno to indicate the error.
 *
 ****************************************************************************/

int lchown(FAR const char *path, uid_t owner, gid_t group)
{
  struct stat buf;

  buf.st_uid = owner;
  buf.st_gid = group;

  return chstat(path, &buf, CH_STAT_UID | CH_STAT_GID, 0);
}

/****************************************************************************
 * Name: utimens
 *
 * Description:
 *   The utimens() function shall set the access and modification times of
 *   the file pointed to by the path argument to the value of the times
 *   argument. utimens() function allows time specifications accurate to
 *   the microsecond.
 *
 *   For utimens(), the times argument is an array of timeval structures.
 *   The first array member represents the date and time of last access,
 *   and the second member represents the date and time of last
 *   modification. The times in the timeval structure are measured in
 *   seconds and microseconds since the Epoch, although rounding toward
 *   the nearest second may occur.
 *
 *   If the times argument is a null pointer, the access and modification
 *   times of the file shall be set to the current time. The effective
 *   user ID of the process shall match the owner of the file, has write
 *   access to the file or appropriate privileges to use this call in this
 *   manner. Upon completion, utimens() shall mark the time of the last
 *   file status change, st_ctime, for update.
 *
 * Input Parameters:
 *   path  - Specifies the file to be modified
 *   times - Specifies the time value to set
 *
 * Returned Value:
 *   Upon successful completion, 0 shall be returned. Otherwise, -1 shall
 *   be returned and errno shall be set to indicate the error, and the file
 *   times shall not be affected.
 *
 ****************************************************************************/

int utimens(FAR const char *path, const struct timespec times[2])
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

  return chstat(path, &buf, CH_STAT_ATIME | CH_STAT_MTIME, 1);
}

/****************************************************************************
 * Name: lutimens
 *
 * Description:
 *   The lutimens() system call is similar to utimens() but does not follow
 *   the symbolic links.
 *
 * Input Parameters:
 *   path  - Specifies the file to be modified
 *   times - Specifies the time value to set
 *
 * Returned Value:
 *   Upon successful completion, 0 shall be returned. Otherwise, -1 shall
 *   be returned and errno shall be set to indicate the error, and the file
 *   times shall not be affected.
 *
 ****************************************************************************/

int lutimens(FAR const char *path, const struct timespec times[2])
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

  return chstat(path, &buf, CH_STAT_ATIME | CH_STAT_MTIME, 0);
}

/****************************************************************************
 * Name: inode_chstat
 *
 * Description:
 *   The inode_chstat() function will change information about an 'inode'
 *   in the pseudo file system according the area pointed to by 'buf'.
 *
 *   The 'buf' argument is a pointer to a stat structure, as defined in
 *   <sys/stat.h>, which information is placed concerning the file.
 *
 * Input Parameters:
 *   inode   - The inode of interest
 *   buf     - The caller provide location in which to apply information
 *             about the inode.
 *   flags   - The vaild field in buf
 *   resolve - Whether to resolve the symbolic link
 *
 * Returned Value:
 *   Zero (OK) returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int inode_chstat(FAR struct inode *inode,
                 FAR const struct stat *buf, int flags, int resolve)
{
  DEBUGASSERT(inode != NULL && buf != NULL);

#ifdef CONFIG_PSEUDOFS_SOFTLINKS
  /* Handle softlinks differently.  Just call chstat() recursively on the
   * target of the softlink.
   */

  if (INODE_IS_SOFTLINK(inode))
    {
      if (resolve)
        {
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

          /* chstat() the target of the soft link. */

          return chstat_recursive(inode->u.i_link, buf, flags, ++resolve);
        }
    }
#endif

#ifdef CONFIG_PSEUDOFS_ATTRIBUTES
  if (flags & CH_STAT_MODE)
    {
      inode->i_mode  = buf->st_mode;
    }

  if (flags & CH_STAT_UID)
    {
      inode->i_owner = buf->st_uid;
    }

  if (flags & CH_STAT_GID)
    {
      inode->i_group = buf->st_gid;
    }

  if (flags & CH_STAT_ATIME)
    {
      inode->i_atime = buf->st_atim;
    }

  if (flags & CH_STAT_MTIME)
    {
      inode->i_mtime = buf->st_mtim;
    }

  inode->i_ctime = buf->st_ctim;
#endif

  return OK;
}
