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
#include <nuttx/lib/lib.h>
#include <nuttx/list.h>
#include <nuttx/mutex.h>
#include <nuttx/sched.h>

#include "inode/inode.h"
#include "fs_heap.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* The current directory of one or more task groups.  The groups that a
 * group creates share its object until one of them calls chdir().  All
 * fields are protected by g_cwd_lock.
 */

struct fs_cwd_s
{
  struct list_node node;      /* In g_cwd_list */
  unsigned int     crefs;     /* Number of task groups that use it */
  bool             removed;   /* The directory was removed */
  FAR char        *path;      /* Absolute host path, no symbolic links */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static mutex_t g_cwd_lock = NXMUTEX_INITIALIZER;
static struct list_node g_cwd_list = LIST_INITIAL_VALUE(g_cwd_list);

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
 *   Make a host path relative to the root of the caller, in place: the
 *   path that the caller uses.  -ENOENT if the path is outside the root.
 *
 ****************************************************************************/

#ifdef CONFIG_FS_CHROOT
static int cwd_userpath(FAR char *path)
{
  FAR const char *root = cwd_root();
  size_t len;

  if (root == NULL)
    {
      return OK;
    }

  len = strlen(root);
  if (strncmp(path, root, len) != 0 ||
      (path[len] != '/' && path[len] != '\0'))
    {
      return -ENOENT;
    }

  if (path[len] == '\0')
    {
      path[0] = '/';
      path[1] = '\0';
    }
  else
    {
      memmove(path, path + len, strlen(path + len) + 1);
    }

  return OK;
}
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
 * Name: cwd_put
 *
 * Description:
 *   Drop one reference to a current directory object.  The caller holds
 *   g_cwd_lock.  Return the object if it must be freed after the lock is
 *   released, else NULL.
 *
 ****************************************************************************/

static FAR struct fs_cwd_s *cwd_put(FAR struct fs_cwd_s *cwd)
{
  if (cwd != NULL && --cwd->crefs == 0)
    {
      list_delete(&cwd->node);
      return cwd;
    }

  return NULL;
}

/****************************************************************************
 * Name: cwd_free
 ****************************************************************************/

static void cwd_free(FAR struct fs_cwd_s *cwd)
{
  if (cwd != NULL)
    {
      fs_heap_free(cwd->path);
      fs_heap_free(cwd);
    }
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
  FAR struct fs_cwd_s *cwd;
  FAR struct fs_cwd_s *old;
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

  cwd = fs_heap_malloc(sizeof(*cwd));
  if (cwd == NULL)
    {
      return -ENOMEM;
    }

  cwd->path = fs_heap_strdup(buf);
  if (cwd->path == NULL)
    {
      fs_heap_free(cwd);
      return -ENOMEM;
    }

  cwd->crefs   = 1;
  cwd->removed = false;

  nxmutex_lock(&g_cwd_lock);
  old = group->tg_cwd;
  group->tg_cwd = cwd;
  list_add_tail(&g_cwd_list, &cwd->node);
  old = cwd_put(old);
  nxmutex_unlock(&g_cwd_lock);

  cwd_free(old);
  return OK;
}

/****************************************************************************
 * Name: cwd_get
 *
 * Description:
 *   Copy the current directory of the caller to 'buf' (PATH_MAX bytes) as
 *   the path that the caller uses: inside its root.
 *
 ****************************************************************************/

static int cwd_get(FAR char *buf)
{
  FAR struct fs_cwd_s *cwd;
  bool host = false;
  int ret = OK;

  nxmutex_lock(&g_cwd_lock);
  cwd = nxsched_self()->group->tg_cwd;
  if (cwd == NULL)
    {
      /* The default: the home directory, inside the root of the caller */

      strlcpy(buf, CONFIG_LIBC_HOMEDIR, PATH_MAX);
    }
  else if (cwd->removed)
    {
      ret = -ENOENT;
    }
  else if (strlcpy(buf, cwd->path, PATH_MAX) >= PATH_MAX)
    {
      ret = -ENAMETOOLONG;
    }
  else
    {
      host = true;
    }

  nxmutex_unlock(&g_cwd_lock);

#ifdef CONFIG_FS_CHROOT
  /* A directory outside the root has no name for the caller */

  if (ret >= 0 && host)
    {
      ret = cwd_userpath(buf);
    }
#else
  UNUSED(host);
#endif

  return ret;
}

/****************************************************************************
 * Name: cwd_hostpath
 *
 * Description:
 *   Write the canonical host path of 'path' to 'buf' (PATH_MAX bytes): the
 *   form of the paths that chdir() keeps.  The last component is not
 *   followed: only its directory is resolved, so this works after the
 *   last component was removed or renamed.
 *
 ****************************************************************************/

static int cwd_hostpath(FAR const char *path, FAR char *buf)
{
  FAR const char *base;
  FAR char *dir = NULL;
  size_t baselen = 0;
  size_t len;
  int ret;

  len = strlen(path);
  while (len > 1 && path[len - 1] == '/')
    {
      len--;
    }

  /* The last component, and its directory */

  base = path + len;
  while (base > path && *(base - 1) != '/')
    {
      base--;
    }

  baselen = path + len - base;
  if (baselen == 0 || baselen > NAME_MAX ||
      (base[0] == '.' && (baselen == 1 || (baselen == 2 && base[1] == '.'))))
    {
      base = NULL;     /* "/", "." or "..": resolve the whole path */
    }

  if (base != NULL)
    {
      if (base - path > PATH_MAX)
        {
          return -ENAMETOOLONG;
        }

      dir = lib_get_pathbuffer();
      if (dir == NULL)
        {
          return -ENOMEM;
        }

      if (base == path)
        {
          strlcpy(dir, ".", PATH_MAX);
        }
      else if (base == path + 1)
        {
          strlcpy(dir, "/", PATH_MAX);
        }
      else
        {
          strlcpy(dir, path, base - path);
        }

      path = dir;
    }

  if (lib_realpath(path, buf, false) == NULL)
    {
      ret = -get_errno();
    }
  else
    {
      ret = OK;
      if (base != NULL)
        {
          len = strlen(buf);
          if (len > 1)
            {
              buf[len++] = '/';
            }

          if (len + baselen >= PATH_MAX)
            {
              ret = -ENAMETOOLONG;
            }
          else
            {
              memcpy(buf + len, base, baselen);
              buf[len + baselen] = '\0';
            }
        }

      if (ret >= 0)
        {
          ret = cwd_tohost(buf);
        }
    }

  if (dir != NULL)
    {
      lib_put_pathbuffer(dir);
    }

  return ret;
}

/****************************************************************************
 * Name: cwd_rename
 *
 * Description:
 *   Each current directory at or below the host path 'oldpath' gets the
 *   new host path, and a current directory at 'newpath' (an empty
 *   directory that the rename replaced) is removed.
 *
 ****************************************************************************/

static void cwd_rename(FAR const char *oldpath, FAR const char *newpath)
{
  FAR struct fs_cwd_s *cwd;
  size_t oldlen = strlen(oldpath);
  size_t newlen = strlen(newpath);
  FAR char *path;

  if (strcmp(oldpath, newpath) == 0)
    {
      return;
    }

  nxmutex_lock(&g_cwd_lock);
  list_for_every_entry(&g_cwd_list, cwd, struct fs_cwd_s, node)
    {
      if (cwd->removed)
        {
          continue;
        }

      /* An empty directory that the rename replaced */

      if (strcmp(cwd->path, newpath) == 0)
        {
          cwd->removed = true;
          continue;
        }

      if (!cwd_under(cwd->path, oldpath))
        {
          continue;
        }

      path = fs_heap_malloc(newlen + strlen(cwd->path + oldlen) + 1);
      if (path == NULL)
        {
          continue;   /* Keep the old name */
        }

      memcpy(path, newpath, newlen);
      strcpy(path + newlen, cwd->path + oldlen);
      fs_heap_free(cwd->path);
      cwd->path = path;
    }

  nxmutex_unlock(&g_cwd_lock);
}

/****************************************************************************
 * Name: cwd_remove
 *
 * Description:
 *   A current directory at the host path 'path' is removed.
 *
 ****************************************************************************/

static void cwd_remove(FAR const char *path)
{
  FAR struct fs_cwd_s *cwd;

  nxmutex_lock(&g_cwd_lock);
  list_for_every_entry(&g_cwd_list, cwd, struct fs_cwd_s, node)
    {
      if (strcmp(cwd->path, path) == 0)
        {
          cwd->removed = true;
        }
    }

  nxmutex_unlock(&g_cwd_lock);
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

void fs_cwd_inherit(FAR struct task_group_s *group)
{
  FAR struct tcb_s *rtcb = nxsched_self();
  FAR struct fs_cwd_s *cwd;

  if (rtcb == NULL || rtcb->group == NULL || rtcb->group == group)
    {
      return;
    }

  nxmutex_lock(&g_cwd_lock);
  cwd = rtcb->group->tg_cwd;
  if (cwd != NULL)
    {
      cwd->crefs++;
    }

  group->tg_cwd = cwd;
  nxmutex_unlock(&g_cwd_lock);
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
  FAR struct fs_cwd_s *cwd;

  if (group->tg_cwd == NULL)
    {
      return;
    }

  nxmutex_lock(&g_cwd_lock);
  cwd = cwd_put(group->tg_cwd);
  group->tg_cwd = NULL;
  nxmutex_unlock(&g_cwd_lock);

  cwd_free(cwd);
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
  FAR struct fs_cwd_s *cwd;
  FAR const char *root = cwd_root();
  int ret = OK;
  int len = 0;

  nxmutex_lock(&g_cwd_lock);
  cwd = nxsched_self()->group->tg_cwd;
  if (cwd == NULL)
    {
      /* The default: the home directory, inside the root of the caller */

      len = snprintf(buffer, buflen, "%s%s/%s", root != NULL ? root : "",
                     CONFIG_LIBC_HOMEDIR, path);
    }
  else if (cwd->removed)
    {
      ret = -ENOENT;
    }
  else if (root != NULL && !cwd_under(cwd->path, root))
    {
      /* A directory outside the root (chroot() without chdir()) is not the
       * start of a lookup: relative names cannot leave the root.
       */

      ret = -ENOENT;
    }
  else
    {
      len = snprintf(buffer, buflen, "%s/%s", cwd->path, path);
    }

  nxmutex_unlock(&g_cwd_lock);

  /* The size that the joined path needs, with its NUL */

  return ret < 0 ? ret : len + 1;
}

/****************************************************************************
 * Name: fs_cwd_renamed
 *
 * Description:
 *   See fs/inode/inode.h.
 *
 ****************************************************************************/

void fs_cwd_renamed(FAR const char *oldpath, FAR const char *newpath)
{
  FAR char *oldhost;
  FAR char *newhost;

  if (list_is_empty(&g_cwd_list))
    {
      return;
    }

  oldhost = lib_get_pathbuffer();
  newhost = lib_get_pathbuffer();
  if (oldhost != NULL && newhost != NULL &&
      cwd_hostpath(oldpath, oldhost) >= 0 &&
      cwd_hostpath(newpath, newhost) >= 0)
    {
      cwd_rename(oldhost, newhost);
    }

  if (newhost != NULL)
    {
      lib_put_pathbuffer(newhost);
    }

  if (oldhost != NULL)
    {
      lib_put_pathbuffer(oldhost);
    }
}

/****************************************************************************
 * Name: fs_cwd_removed
 *
 * Description:
 *   See fs/inode/inode.h.
 *
 ****************************************************************************/

void fs_cwd_removed(FAR const char *path)
{
  FAR char *host;

  if (list_is_empty(&g_cwd_list))
    {
      return;
    }

  host = lib_get_pathbuffer();
  if (host != NULL)
    {
      if (cwd_hostpath(path, host) >= 0)
        {
          cwd_remove(host);
        }

      lib_put_pathbuffer(host);
    }
}

/****************************************************************************
 * Name: fs_cwd_busy
 *
 * Description:
 *   See fs/inode/inode.h.
 *
 ****************************************************************************/

bool fs_cwd_busy(FAR const char *path)
{
  FAR struct fs_cwd_s *cwd;
  bool busy = false;

  nxmutex_lock(&g_cwd_lock);
  list_for_every_entry(&g_cwd_list, cwd, struct fs_cwd_s, node)
    {
      if (!cwd->removed && cwd_under(cwd->path, path))
        {
          busy = true;
          break;
        }
    }

  nxmutex_unlock(&g_cwd_lock);
  return busy;
}

/****************************************************************************
 * Name: chdir
 *
 * Description:
 *   The chdir() function causes the directory named by the pathname pointed
 *   to by the 'path' argument to become the current working directory; that
 *   is, the starting point for path searches for pathnames not beginning
 *   with '/'.  The current directory is an attribute of the task group.
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

  buf = lib_get_pathbuffer();
  if (buf == NULL)
    {
      ret = -ENOMEM;
      goto errout;
    }

  ret = cwd_set(path, buf);
  lib_put_pathbuffer(buf);
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
 *
 ****************************************************************************/

int fchdir(int fd)
{
  FAR struct file *filep;
  FAR char *path;
  FAR char *buf;
  int ret;

  ret = file_get(fd, &filep);
  if (ret < 0)
    {
      goto errout;
    }

  path = lib_get_pathbuffer();
  buf  = lib_get_pathbuffer();
  if (path == NULL || buf == NULL)
    {
      ret = -ENOMEM;
    }
  else
    {
      /* A file without a path (a pipe, a socket) is not a directory.
       * cwd_set() gives ENOTDIR for a file that has one.
       */

      ret = file_fcntl(filep, F_GETPATH, path);
      if (ret < 0)
        {
          ret = -ENOTDIR;
        }
    }

  file_put(filep);

#ifdef CONFIG_FS_CHROOT
  if (ret >= 0)
    {
      ret = cwd_userpath(path);
    }
#endif

  if (ret >= 0)
    {
      ret = cwd_set(path, buf);
    }

  if (buf != NULL)
    {
      lib_put_pathbuffer(buf);
    }

  if (path != NULL)
    {
      lib_put_pathbuffer(path);
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
 * Name: nx_getcwd
 *
 * Description:
 *   See include/nuttx/fs/fs.h.
 *
 ****************************************************************************/

int nx_getcwd(FAR char *buf, size_t size)
{
  FAR char *path;
  size_t len;
  int ret;

  if (buf == NULL || size == 0)
    {
      return -EINVAL;
    }

  path = lib_get_pathbuffer();
  if (path == NULL)
    {
      return -ENOMEM;
    }

  ret = cwd_get(path);
  if (ret >= 0)
    {
      len = strlen(path) + 1;
      if (len > size)
        {
          ret = -ERANGE;
        }
      else
        {
          /* Copy to the caller with no lock held */

          memcpy(buf, path, len);
          ret = len;
        }
    }

  lib_put_pathbuffer(path);
  return ret;
}
