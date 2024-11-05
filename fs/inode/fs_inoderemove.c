/****************************************************************************
 * fs/inode/fs_inoderemove.c
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

#include <assert.h>
#include <errno.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>

#include "inode/inode.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: inode_unlink
 *
 * Description:
 *   Given a path, remove a the node from the in-memory, inode tree that the
 *   path refers to.  This is normally done in preparation to removing or
 *   moving an inode.
 *
 *   In symbolic links in the pseduo file system are enabled, then this
 *   logic will follow the symbolic links up until the terminal node.  Then
 *   that link in removed. So if this the terminal node is a symbolic link,
 *   the symbolic link node will be removed, not the target of the link.
 *
 * Assumptions/Limitations:
 *   The caller must hold the inode semaphore
 *
 ****************************************************************************/

static FAR struct inode *inode_unlink(FAR const char *path)
{
  struct inode_search_s desc;
  FAR struct inode *inode = NULL;
  int ret;

  /* Verify parameters.  Ignore null paths */

  if (path == NULL)
    {
      return NULL;
    }

  /* Find the node to unlink */

  SETUP_SEARCH(&desc, path, true);

  ret = inode_search(&desc);
  if (ret >= 0)
    {
      inode = desc.node;
      DEBUGASSERT(inode != NULL);

      /* If peer is non-null, then remove the node from the right of
       * of that peer node.
       */

      if (desc.peer != NULL)
        {
          desc.peer->i_peer = inode->i_peer;
        }

      /* Then remove the node from head of the list of children. */

      else
        {
          DEBUGASSERT(desc.parent != NULL);
          desc.parent->i_child = inode->i_peer;
        }

      inode->i_peer   = NULL;
      inode->i_parent = NULL;
      atomic_fetch_sub(&inode->i_crefs, 1);
    }

  RELEASE_SEARCH(&desc);
  return inode;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: inode_remove
 *
 * Description:
 *   Given a path, remove a the node from the in-memory, inode tree that the
 *   path refers to and free all resources related to the inode.  If the
 *   inode is in-use, then it will be unlinked, but will not be freed until
 *   the last reference to the inode is released.
 *
 * Assumptions/Limitations:
 *   The caller must hold the inode semaphore
 *
 ****************************************************************************/

int inode_remove(FAR const char *path)
{
  FAR struct inode *inode;

  /* Find the inode and unlink it from the in-memory inode tree */

  inode = inode_unlink(path);
  if (inode)
    {
      /* Found it! But we cannot delete the inode if there are references
       * to it
       */

      if (atomic_load(&inode->i_crefs))
        {
          return -EBUSY;
        }
      else
        {
          /* And delete it now -- recursively to delete all of its children.
           * Since it has been unlinked, then the peer pointer should be
           * NULL.
           */

          DEBUGASSERT(inode->i_peer == NULL);
          inode_free(inode);
          return OK;
        }
    }

  /* The node does not exist */

  return -ENOENT;
}
