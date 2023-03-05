/****************************************************************************
 * fs/inode/fs_foreachinode.c
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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>

#include "inode/inode.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Is it better to allocate the struct inode_path_s from the heap? or
 * from the stack?  This decision depends on how often this is down and
 * how much stack space you can afford.
 */

#define ENUM_INODE_ALLOC 1

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure manages the full path to the inode. */

struct inode_path_s
{
  foreach_inode_t handler;
  FAR void       *arg;
  char            path[PATH_MAX];
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: foreach_inodelevel
 *
 * Description:
 *   This is the recursive 'heart' of foreach_inode.  It will visit each
 *   inode at this level in the hierarchy and recurse handle each inode
 *   at the next level down.
 *
 * Assumptions:
 *   The caller holds the inode semaphore.
 *
 ****************************************************************************/

static int foreach_inodelevel(FAR struct inode *node,
                              FAR struct inode_path_s *info)
{
  int ret = OK;

  /* Visit each node at this level */

  for (; node; node = node->i_peer)
    {
      /* Give the next inode to the callback */

      ret = info->handler(node, info->path, info->arg);

      /* Break out of the loop early if the handler returns a non-zero
       * value.
       */

      if (ret != 0)
        {
          break;
        }

      /* If there is a level 'beneath' this one, then recurse to visit all
       * of the inodes at that level.
       */

      if (node->i_child)
        {
          /* Construct the path to the next level */

          int pathlen = strlen(info->path);
          int namlen  = strlen(node->i_name) + 1;

          /* Make sure that this would not exceed the maximum path length */

          if (pathlen + namlen >= PATH_MAX)
            {
              ret = -ENAMETOOLONG;
              break;
            }

          /* Append the path segment to this inode and recurse */

          snprintf(&info->path[pathlen], sizeof(info->path) - pathlen,
                   "/%s", node->i_name);
          ret = foreach_inodelevel(node->i_child, info);

          /* Truncate the path name back to the correct length */

          info->path[pathlen] = '\0';

          /* Return early if the handler at the lower level returned a non-
           * zero value
           */

          if (ret != 0)
            {
              break;
            }
        }
    }

  /* Return the result of the traversal. */

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: foreach_inode
 *
 * Description:
 *   Visit each inode in the pseudo-file system.  The traversal is terminated
 *   when the callback 'handler' returns a non-zero value, or when all of
 *   the inodes have been visited.
 *
 *   NOTE 1: Use with caution... The pseudo-file system is locked throughout
 *   the traversal.
 *   NOTE 2: The search algorithm is recursive and could, in principle, use
 *   an indeterminant amount of stack space.  This will not usually be a
 *   real work issue.
 *
 ****************************************************************************/

int foreach_inode(foreach_inode_t handler, FAR void *arg)
{
#ifdef ENUM_INODE_ALLOC
  FAR struct inode_path_s *info;
  int ret;

  /* Allocate the mountpoint info structure */

  info = (FAR struct inode_path_s *)kmm_malloc(sizeof(struct inode_path_s));
  if (!info)
    {
      return -ENOMEM;
    }

  /* Initialize the info structure */

  info->handler = handler;
  info->arg     = arg;
  info->path[0] = '\0';

  /* Start the recursion at the root inode */

  ret = inode_lock();
  if (ret >= 0)
    {
      ret = foreach_inodelevel(g_root_inode->i_child, info);
      inode_unlock();
    }

  /* Free the info structure and return the result */

  kmm_free(info);
  return ret;

#else
  struct inode_path_s info;
  int ret;

  /* Initialize the info structure */

  info.handler = handler;
  info.arg     = arg;
  info.path[0] = '\0';

  /* Start the recursion at the root inode */

  ret = inode_lock();
  if (ret >= 0)
    {
      ret = foreach_inodelevel(g_root_inode->i_child, &info);
      inode_unlock();
    }

  return ret;

#endif
}
