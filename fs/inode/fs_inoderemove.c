/****************************************************************************
 * fs/inode/fs_inoderemove.c
 *
 *   Copyright (C) 2007-2009, 2017 Gregory Nutt. All rights reserved.
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

#include <errno.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>

#include "inode/inode.h"

/****************************************************************************
 * Public Functions
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

FAR struct inode *inode_unlink(FAR const char *path)
{
  struct inode_search_s desc;
  FAR struct inode *node = NULL;
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
      node = desc.node;
      DEBUGASSERT(node != NULL);

      /* If peer is non-null, then remove the node from the right of
       * of that peer node.
       */

      if (desc.peer != NULL)
        {
          desc.peer->i_peer = node->i_peer;
        }

      /* Then remove the node from head of the list of children. */

      else
        {
          DEBUGASSERT(desc.parent != NULL);
          desc.parent->i_child = node->i_peer;
        }

      node->i_peer = NULL;
    }

  RELEASE_SEARCH(&desc);
  return node;
}

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
  FAR struct inode *node;

  /* Find the inode and unlink it from the in-memory inode tree */

  node = inode_unlink(path);
  if (node)
    {
      /* Found it! But we cannot delete the inode if there are references
       * to it
       */

      if (node->i_crefs)
        {
          /* In that case, we will mark it deleted, when the filesystem
           * releases the inode, we will then, finally delete the subtree
           */

          node->i_flags |= FSNODEFLAG_DELETED;
          return -EBUSY;
        }
      else
        {
          /* And delete it now -- recursively to delete all of its children.
           * Since it has been unlinked, then the peer pointer should be
           * NULL.
           */

          DEBUGASSERT(node->i_peer == NULL);
          inode_free(node);
          return OK;
        }
    }

  /* The node does not exist */

  return -ENOENT;
}
