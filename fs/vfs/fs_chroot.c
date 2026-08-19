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
#include <string.h>

#include <nuttx/fs/fs.h>
#include <nuttx/sched.h>

#include "inode/inode.h"
#include "fs_heap.h"

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
  FAR struct tcb_s *rtcb;
  FAR struct task_group_s *group;
  FAR char *newroot;
  struct inode_search_s desc;
  struct stat buf;
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

  /* Resolve to a host absolute path the same way lookups do: make
   * absolute, prepend the current jail, and canonicalize.  No second
   * inode walk.
   */

  ret = inode_search_setup(&desc, path, true);
  if (ret < 0)
    {
      set_errno(-ret);
      return ERROR;
    }

  /* Host "/" means no jail.  Clear any previous root. */

  if (strcmp(desc.path, "/") == 0)
    {
      fs_heap_free(group->tg_root);
      group->tg_root = NULL;
      inode_search_release(&desc);
      return OK;
    }

  newroot = fs_heap_strdup(desc.path);
  inode_search_release(&desc);
  if (newroot == NULL)
    {
      set_errno(ENOMEM);
      return ERROR;
    }

  fs_heap_free(group->tg_root);
  group->tg_root = newroot;
  return OK;
}
