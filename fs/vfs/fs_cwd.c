/****************************************************************************
 * fs/vfs/fs_cwd.c
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

#include <sys/stat.h>
#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <nuttx/fs/fs.h>
#include <nuttx/kmalloc.h>
#include <nuttx/lib/lib.h>
#include <nuttx/mutex.h>
#include <nuttx/sched.h>

#include "inode/inode.h"
#include "fs_heap.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* tg_cwd of each task group is the canonical absolute host path of its
 * current directory, in the kernel heap.  NULL is the home directory and
 * "" is a removed directory.  g_cwd_lock protects tg_cwd of all groups.
 */

static mutex_t g_cwd_lock = NXMUTEX_INITIALIZER;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cwd_root
 *
 * Description:
 *   Return the chroot() root of the calling task group, or NULL.
 *
 ****************************************************************************/

static FAR const char *cwd_root(void)
{
#ifdef CONFIG_FS_CHROOT
  FAR struct tcb_s *tcb = nxsched_self();

  if (tcb != NULL && tcb->group != NULL)
    {
      return tcb->group->tg_root;
    }
#endif

  return NULL;
}

/****************************************************************************
 * Name: cwd_userpath
 *
 * Description:
 *   Return the part of a host path that the caller uses: the path relative
 *   to the root of the caller.  NULL if the path is outside the root.
 *
 ****************************************************************************/

#ifdef CONFIG_FS_CHROOT
static FAR const char *cwd_userpath(FAR const char *path)
{
  FAR const char *root = cwd_root();
  size_t len;

  if (root == NULL)
    {
      return path;
    }

  len = strlen(root);
  if (strncmp(path, root, len) != 0 ||
      (path[len] != '/' && path[len] != '\0'))
    {
      return NULL;
    }

  return path[len] == '\0' ? "/" : path + len;
}
#else
#  define cwd_userpath(path) (path)
#endif

/****************************************************************************
 * Name: cwd_under
 *
 * Description:
 *   Return true if 'path' is 'dir' or a path below 'dir'.
 *
 ****************************************************************************/

static bool cwd_under(FAR const char *path, FAR const char *dir)
{
  size_t len = strlen(dir);

  if (len == 1 && dir[0] == '/')
    {
      return true;
    }

  return strncmp(path, dir, len) == 0 &&
         (path[len] == '\0' || path[len] == '/');
}

/****************************************************************************
 * Name: cwd_searchable
 *
 * Description:
 *   Return true if the effective IDs of the caller have search permission
 *   on the directory.  Root needs any execute bit, as in access() and
 *   exec().
 *
 ****************************************************************************/

#ifdef CONFIG_FS_PERMISSION
static bool cwd_searchable(FAR const struct stat *buf)
{
  mode_t xbit = S_IXOTH;
  uid_t uid = geteuid();

  if (uid == 0)
    {
      return (buf->st_mode & (S_IXUSR | S_IXGRP | S_IXOTH)) != 0;
    }

  if (buf->st_uid == uid)
    {
      xbit = S_IXUSR;
    }
  else if (buf->st_gid == getegid())
    {
      xbit = S_IXGRP;
    }
#if NGROUPS_MAX > 0
  else
    {
      gid_t groups[NGROUPS_MAX];
      int ngroups = getgroups(NGROUPS_MAX, groups);
      int i;

      for (i = 0; i < ngroups; i++)
        {
          if (groups[i] == buf->st_gid)
            {
              xbit = S_IXGRP;
              break;
            }
        }
    }
#endif

  return (buf->st_mode & xbit) != 0;
}
#endif

/****************************************************************************
 * Name: cwd_tohost
 *
 * Description:
 *   Put the root of the caller in front of an absolute path in 'buf', in
 *   place.  'buf' holds PATH_MAX bytes.
 *
 ****************************************************************************/

static int cwd_tohost(FAR char *buf)
{
  FAR const char *root = cwd_root();
  size_t rootlen;
  size_t len;

  if (root == NULL)
    {
      return OK;
    }

  rootlen = strlen(root);
  len     = strlen(buf);
  if (len == 1)
    {
      /* The root itself: no "/" after it */

      buf[0] = '\0';
      len    = 0;
    }

  if (rootlen + len >= PATH_MAX)
    {
      return -ENAMETOOLONG;
    }

  memmove(buf + rootlen, buf, len + 1);
  memcpy(buf, root, rootlen);
  return OK;
}

/****************************************************************************
 * Name: cwd_set
 *
 * Description:
 *   Make 'path' the current directory of the task group of the caller.
 *   'path' is an absolute path inside the root of the caller.  A buffer
 *   'buf' of PATH_MAX bytes is given for the work.
 *
 ****************************************************************************/

static int cwd_set(FAR const char *path, FAR char *buf)
{
  FAR struct task_group_s *group = nxsched_self()->group;
  FAR char *cwd;
  FAR char *old;
  struct stat sb;
  int ret;

  /* The canonical path: absolute, no ".", ".." or symbolic links.  The
   * search permission of each component is checked on the way.
   */

  if (lib_realpath(path, buf, false) == NULL)
    {
      return -get_errno();
    }

  ret = nx_stat(buf, &sb, 1);
  if (ret < 0)
    {
      return ret;
    }

  if (!S_ISDIR(sb.st_mode))
    {
      return -ENOTDIR;
    }

#ifdef CONFIG_FS_PERMISSION
  /* stat() needs no search permission on the last component */

  if (!cwd_searchable(&sb))
    {
      return -EACCES;
    }
#endif

  ret = cwd_tohost(buf);
  if (ret < 0)
    {
      return ret;
    }

  cwd = fs_heap_strdup(buf);
  if (cwd == NULL)
    {
      return -ENOMEM;
    }

  nxmutex_lock(&g_cwd_lock);
  old = group->tg_cwd;
  group->tg_cwd = cwd;
  nxmutex_unlock(&g_cwd_lock);

  fs_heap_free(old);
  return OK;
}

/****************************************************************************
 * Name: cwd_get
 *
 * Description:
 *   Copy the current directory of the caller to 'buf' of 'size' bytes as
 *   the path that the caller uses: inside its root.  Return the length
 *   with the NUL.
 *
 ****************************************************************************/

static int cwd_get(FAR char *buf, size_t size)
{
  FAR const char *cwd;
  FAR const char *path = CONFIG_LIBC_HOMEDIR;
  size_t len;
  int ret;

  nxmutex_lock(&g_cwd_lock);
  cwd = nxsched_self()->group->tg_cwd;
  if (cwd != NULL)
    {
      /* A directory outside the root has no name for the caller.  The
       * default, the home directory, is inside the root.
       */

      path = cwd[0] == '\0' ? NULL : cwd_userpath(cwd);
    }

  if (path == NULL)
    {
      ret = -ENOENT;
    }
  else
    {
      len = strlen(path) + 1;
      if (len > size)
        {
          ret = -ERANGE;
        }
      else
        {
          memcpy(buf, path, len);
          ret = len;
        }
    }

  nxmutex_unlock(&g_cwd_lock);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fs_cwd_inherit
 *
 * Description:
 *   See include/nuttx/fs/fs.h.
 *
 ****************************************************************************/

int fs_cwd_inherit(FAR struct task_group_s *group)
{
  FAR struct tcb_s *rtcb = nxsched_self();
  FAR const char *parent;
  int ret = OK;

  if (rtcb == NULL || rtcb->group == NULL || rtcb->group == group)
    {
      return OK;
    }

  nxmutex_lock(&g_cwd_lock);
  parent = rtcb->group->tg_cwd;
  if (parent != NULL)
    {
      group->tg_cwd = fs_heap_strdup(parent);
      if (group->tg_cwd == NULL)
        {
          ret = -ENOMEM;
        }
    }

  nxmutex_unlock(&g_cwd_lock);
  return ret;
}

/****************************************************************************
 * Name: fs_cwd_release
 *
 * Description:
 *   See include/nuttx/fs/fs.h.
 *
 ****************************************************************************/

void fs_cwd_release(FAR struct task_group_s *group)
{
  FAR char *cwd;

  nxmutex_lock(&g_cwd_lock);
  cwd = group->tg_cwd;
  group->tg_cwd = NULL;
  nxmutex_unlock(&g_cwd_lock);

  fs_heap_free(cwd);
}

/****************************************************************************
 * Name: fs_cwd_join
 *
 * Description:
 *   See fs/inode/inode.h.
 *
 ****************************************************************************/

int fs_cwd_join(FAR const char *path, FAR char *buffer, size_t buflen)
{
  FAR const char *cwd;
  FAR const char *root = cwd_root();
  int ret = -ENOENT;

  nxmutex_lock(&g_cwd_lock);
  cwd = nxsched_self()->group->tg_cwd;
  if (cwd == NULL)
    {
      /* The default: the home directory, inside the root of the caller */

      cwd = CONFIG_LIBC_HOMEDIR;
    }
  else if (cwd[0] == '\0' || (root != NULL && !cwd_under(cwd, root)))
    {
      /* A removed directory, or a directory outside the root (chroot()
       * without chdir()): relative names cannot leave the root.
       */

      cwd = NULL;
    }
  else
    {
      root = NULL;  /* tg_cwd is a host path: the root is in it */
    }

  if (cwd != NULL)
    {
      /* The size that the joined path needs, with its NUL */

      ret = snprintf(buffer, buflen, "%s%s/%s", root != NULL ? root : "",
                     cwd, path) + 1;
    }

  nxmutex_unlock(&g_cwd_lock);
  return ret;
}

/****************************************************************************
 * Name: chdir
 *
 * Description:
 *   The chdir() function causes the directory named by the pathname pointed
 *   to by the 'path' argument to become the current working directory; that
 *   is, the starting point for path searches for pathnames not beginning
 *   with '/'.  The current directory is an attribute of the task group.
 *   The kernel threads share one: chdir() in one changes it for all.
 *   chdir() does not change the environment variable PWD.
 *
 * Input Parameters:
 *   path - A pointer to a directory to use as the new current working
 *     directory
 *
 * Returned Value:
 *   0(OK) on success; -1(ERROR) on failure with errno set appropriately:
 *
 *   EACCES
 *     Search permission is denied for any component of the pathname.
 *   ELOOP
 *     A loop exists in symbolic links encountered during resolution of the
 *     'path' argument.
 *   ENAMETOOLONG
 *     The length of the path argument exceeds PATH_MAX or a pathname
 *     component is longer than NAME_MAX.
 *   ENOENT
 *     A component of 'path' does not name an existing directory or path is
 *     an empty string.
 *   ENOTDIR
 *     A component of the pathname is not a directory.
 *
 ****************************************************************************/

int chdir(FAR const char *path)
{
  FAR char *buf;
  int ret;

  if (path == NULL)
    {
      ret = -EFAULT;
      goto errout;
    }

  buf = lib_get_tempbuffer(PATH_MAX);
  if (buf == NULL)
    {
      ret = -ENOMEM;
      goto errout;
    }

  ret = cwd_set(path, buf);
  lib_put_tempbuffer(buf);
  if (ret >= 0)
    {
      return OK;
    }

errout:
  set_errno(-ret);
  return ERROR;
}

/****************************************************************************
 * Name: fchdir
 *
 * Description:
 *   The fchdir() function is the same as chdir(), with the directory given
 *   as an open file descriptor.  The directory is found by the path that it
 *   had when it was opened.
 *
 * Input Parameters:
 *   fd - An open file descriptor of a directory.
 *
 * Returned Value:
 *   0(OK) on success; -1(ERROR) on failure with errno set appropriately:
 *
 *   EACCES
 *     Search permission was denied on the directory open on fd.
 *   EBADF
 *     fd is not a valid file descriptor.
 *   ENOTDIR
 *     fd does not refer to a directory.
 *   ENOSYS
 *     CONFIG_FS_IOCTL_FILEPATH is not enabled.
 *
 ****************************************************************************/

int fchdir(int fd)
{
  FAR struct file *filep;
  FAR const char *user;
  FAR char *path;
  FAR char *buf;
  int ret;

  ret = file_get(fd, &filep);
  if (ret < 0)
    {
      goto errout;
    }

  path = lib_get_tempbuffer(PATH_MAX);
  buf  = lib_get_tempbuffer(PATH_MAX);
  if (path == NULL || buf == NULL)
    {
      ret = -ENOMEM;
    }
  else
    {
      /* A directory always has a path.  A file without one (ENOTTY, or a
       * name that is not a path, as for a socket) is not a directory.
       * cwd_set() gives ENOTDIR for a file that has a path.
       */

      ret = file_fcntl(filep, F_GETPATH, path);
      if (ret == -ENOTTY || (ret >= 0 && path[0] != '/'))
        {
#ifdef CONFIG_FS_IOCTL_FILEPATH
          ret = -ENOTDIR;
#else
          ret = -ENOSYS;  /* No file has a path */
#endif
        }
    }

  file_put(filep);

  if (ret >= 0)
    {
      /* The path that the caller uses, inside its root */

      user = cwd_userpath(path);
      ret  = user != NULL ? cwd_set(user, buf) : -ENOENT;
    }

  if (buf != NULL)
    {
      lib_put_tempbuffer(buf);
    }

  if (path != NULL)
    {
      lib_put_tempbuffer(path);
    }

  if (ret >= 0)
    {
      return OK;
    }

errout:
  set_errno(-ret);
  return ERROR;
}

/****************************************************************************
 * Name: getcwd
 *
 * Description:
 *   getcwd() function places the absolute pathname of the current working
 *   directory in the array pointed to by 'buf', and returns 'buf.' The
 *   pathname copied to the array shall contain no components that are
 *   symbolic links. The 'size' argument is the size in bytes of the
 *   character array pointed to by the 'buf' argument.  The current working
 *   directory is kernel state of the task group (chdir(), fchdir()); the
 *   environment variable PWD has no part in it.
 *
 *   As an extension to the POSIX.1-2001 standard, getcwd() allocates
 *   the buffer in the heap of the caller if buf is NULL. In this case,
 *   the allocated buffer has the length size unless size is zero, when buf
 *   is allocated with PATH_MAX bytes. The caller should free the returned
 *   buffer.
 *
 * Input Parameters:
 *   buf - a pointer to the location in which the current working directory
 *     pathname is returned.
 *   size - The size in bytes available at 'buf'
 *
 * Returned Value:
 *   Upon successful completion, getcwd() returns the 'buf' argument.
 *   Otherwise, getcwd() returns a null pointer and sets errno to indicate
 *   the error:
 *
 *   EINVAL
 *     The 'size' argument is 0 and the 'buf' argument is not NULL.
 *   ERANGE
 *     The size argument is greater than 0, but is smaller than the length
 *     of the current working directory pathname +1.
 *   ENOENT
 *     The current working directory was removed, or it is outside the
 *     chroot() root of the caller.
 *   ENOMEM
 *     Insufficient storage space is available.
 *
 ****************************************************************************/

FAR char *getcwd(FAR char *buf, size_t size)
{
  FAR char *alloc = NULL;
  int ret = -EINVAL;

  if (buf == NULL)
    {
      if (size == 0)
        {
          size = PATH_MAX;
        }

      alloc = kumm_malloc(size);
      buf   = alloc;
      ret   = -ENOMEM;
    }

  if (buf != NULL && size > 0)
    {
      ret = cwd_get(buf, size);
      if (ret >= 0)
        {
          return buf;
        }
    }

  kumm_free(alloc);
  set_errno(-ret);
  return NULL;
}
