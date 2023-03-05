/****************************************************************************
 * fs/mount/fs_foreachmountpoint.c
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

#include <sys/statfs.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/fs/fs.h>

#include "inode/inode.h"
#include "mount/mount.h"

#ifndef CONFIG_DISABLE_MOUNTPOINT

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure just remembers the final consumer of the mountpoint
 * information (and its argument).
 */

struct enum_mountpoint_s
{
  foreach_mountpoint_t handler;
  FAR void            *arg;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int mountpoint_filter(FAR struct inode *node,
                             FAR char dirpath[PATH_MAX], FAR void *arg)
{
  FAR struct enum_mountpoint_s *info = (FAR struct enum_mountpoint_s *)arg;
  struct statfs statbuf;
  int pathlen;
  int namlen;
  int ret = OK;

  DEBUGASSERT(node && info && info->handler);

  /* Check if the inode is a mountpoint.  Mountpoints must support statfs.
   * If this one does not for some reason, then it will be ignored.
   *
   * The root node is a special case: It has no operations (u.i_mops == NULL)
   */

  if (INODE_IS_MOUNTPT(node) && node->u.i_mops && node->u.i_mops->statfs)
    {
      /* Yes... get the full path to the inode by concatenating the inode
       * name and the path to the directory containing the inode.
       */

      pathlen = strnlen(dirpath, PATH_MAX);
      namlen  = strnlen(node->i_name, PATH_MAX) + 1;

      /* Make sure that this would not exceed the maximum path length */

      if (pathlen + namlen > PATH_MAX)
        {
          return OK;
        }

      /* Append the inode name to the directory path */

      snprintf(&dirpath[pathlen], PATH_MAX - pathlen, "/%s", node->i_name);

      /* Get the status of the file system */

      if (node->u.i_mops->statfs(node, &statbuf) == OK)
        {
          /* And pass the full path and file system status to the handler */

          ret = info->handler(dirpath, &statbuf, info->arg);
        }

      /* Truncate the path name back to the correct length */

      dirpath[pathlen] = '\0';
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: foreach_mountpoint
 *
 * Description:
 *   Visit each mountpoint in the pseudo-file system.  The traversal is
 *   terminated when the callback 'handler' returns a non-zero value, or when
 *   all of the mountpoints have been visited.
 *
 *   This is just a front end "filter" to foreach_inode() that forwards only
 *   mountpoint inodes.  It is intended to support the mount() command to
 *   when the mount command is used to enumerate mounts.
 *
 *   NOTE 1: Use with caution... The pseudo-file system is locked throughout
 *   the traversal.
 *   NOTE 2: The search algorithm is recursive and could, in principle, use
 *   an indeterminant amount of stack space.  This will not usually be a
 *   real work issue.
 *
 ****************************************************************************/

int foreach_mountpoint(foreach_mountpoint_t handler, FAR void *arg)
{
  struct enum_mountpoint_s info;

  /* Let foreach_inode do the real work */

  info.handler = handler;
  info.arg     = arg;

  return foreach_inode(mountpoint_filter, (FAR void *)&info);
}

#endif
