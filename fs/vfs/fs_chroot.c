/****************************************************************************
 * fs/vfs/fs_chroot.c
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
#include <assert.h>
#include <errno.h>
#include <limits.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <nuttx/fs/fs.h>
#include <nuttx/kmalloc.h>
#include <nuttx/sched.h>

#include "inode/inode.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: chroot_strip_slash
 ****************************************************************************/

static void chroot_strip_slash(FAR char *path)
{
  size_t len;

  if (path == NULL)
    {
      return;
    }

  len = strlen(path);
  while (len > 1 && path[len - 1] == '/')
    {
      path[--len] = '\0';
    }
}

/****************************************************************************
 * Name: chroot_set_pwd
 *
 * Description:
 *   Rewrite PWD so relative lookups stay inside the jail.  If the current
 *   directory is not under the new root, PWD becomes "/".
 *
 ****************************************************************************/

#ifndef CONFIG_DISABLE_ENVIRON
static void chroot_set_pwd(FAR struct inode *root, FAR const char *relpath)
{
  char rootpath[PATH_MAX];
  FAR const char *pwd;
  size_t rootlen;

  rootpath[0] = '\0';
  if (root != NULL)
    {
      if (inode_getpath(root, rootpath, sizeof(rootpath)) < 0)
        {
          setenv("PWD", "/", TRUE);
          return;
        }
    }
  else
    {
      strlcpy(rootpath, "/", sizeof(rootpath));
    }

  if (relpath != NULL && relpath[0] != '\0')
    {
      chroot_strip_slash(rootpath);
      if (rootpath[0] != '\0' && strcmp(rootpath, "/") != 0)
        {
          strlcat(rootpath, "/", sizeof(rootpath));
        }
      else if (strcmp(rootpath, "/") != 0)
        {
          strlcpy(rootpath, "/", sizeof(rootpath));
        }

      strlcat(rootpath, relpath, sizeof(rootpath));
    }

  chroot_strip_slash(rootpath);
  if (rootpath[0] == '\0')
    {
      strlcpy(rootpath, "/", sizeof(rootpath));
    }

  pwd = getenv("PWD");
  if (pwd == NULL)
    {
      pwd = CONFIG_LIBC_HOMEDIR;
    }

  rootlen = strlen(rootpath);
  if (strcmp(pwd, rootpath) == 0)
    {
      setenv("PWD", "/", TRUE);
    }
  else if (rootlen > 1 && strncmp(pwd, rootpath, rootlen) == 0 &&
           pwd[rootlen] == '/')
    {
      setenv("PWD", pwd + rootlen, TRUE);
    }
  else
    {
      setenv("PWD", "/", TRUE);
    }
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: chroot
 *
 * Description:
 *   Cause the named directory to become the root directory, that is, the
 *   starting point for path names beginning with '/'.
 *
 * Input Parameters:
 *   path - Directory to use as the new root
 *
 * Returned Value:
 *   0(OK) on success; -1(ERROR) on failure with errno set appropriately.
 *
 ****************************************************************************/

int chroot(FAR const char *path)
{
  struct inode_search_s desc;
  FAR struct tcb_s *rtcb;
  FAR struct task_group_s *group;
  FAR struct inode *oldroot;
  FAR char *oldrel;
  FAR char *newrel = NULL;
  struct stat buf;
  int errcode;
  int ret;

  if (path == NULL || path[0] == '\0')
    {
      set_errno(ENOENT);
      return ERROR;
    }

  rtcb = nxsched_self();
  DEBUGASSERT(rtcb != NULL && rtcb->group != NULL);
  group = rtcb->group;

#ifdef CONFIG_SCHED_USER_IDENTITY
  if (group->tg_euid != 0)
    {
      set_errno(EPERM);
      return ERROR;
    }
#endif

  ret = nx_stat(path, &buf, 1);
  if (ret < 0)
    {
      set_errno(-ret);
      return ERROR;
    }

  if (!S_ISDIR(buf.st_mode))
    {
      set_errno(ENOTDIR);
      return ERROR;
    }

  SETUP_SEARCH(&desc, path, false);

  ret = inode_find(&desc);
  if (ret < 0)
    {
      errcode = -ret;
      goto errout_with_search;
    }

  /* chroot("/") from the global root is a no-op jail (tg_root NULL). */

  if (desc.node == g_root_inode &&
      (desc.relpath == NULL || desc.relpath[0] == '\0'))
    {
      inode_release(desc.node);
      RELEASE_SEARCH(&desc);
      return OK;
    }
  else if (desc.relpath != NULL && desc.relpath[0] != '\0')
    {
      size_t len = strlen(desc.relpath) + 1;

      newrel = kmm_malloc(len);
      if (newrel == NULL)
        {
          errcode = ENOMEM;
          inode_release(desc.node);
          goto errout_with_search;
        }

      memcpy(newrel, desc.relpath, len);
    }

  oldroot = group->tg_root;
  oldrel  = group->tg_rootrel;

  group->tg_root    = desc.node;
  group->tg_rootrel = newrel;

  if (oldroot != NULL)
    {
      inode_release(oldroot);
    }

  if (oldrel != NULL)
    {
      kmm_free(oldrel);
    }

#ifndef CONFIG_DISABLE_ENVIRON
  chroot_set_pwd(group->tg_root, group->tg_rootrel);
#endif

  RELEASE_SEARCH(&desc);
  return OK;

errout_with_search:
  RELEASE_SEARCH(&desc);
  set_errno(errcode);
  return ERROR;
}
