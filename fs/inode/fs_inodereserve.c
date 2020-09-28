/****************************************************************************
 * fs/inode/fs_inodereserve.c
 *
 *   Copyright (C) 2007-2009, 2011-2012, 2015, 2017 Gregory Nutt. All
 *     rights reserved.
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

#include <assert.h>
#include <errno.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>

#include "inode/inode.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: inode_namelen
 ****************************************************************************/

static int inode_namelen(FAR const char *name)
{
  const char *tmp = name;
  while (*tmp && *tmp != '/')
    {
      tmp++;
    }

  return tmp - name;
}

/****************************************************************************
 * Name: inode_namecpy
 ****************************************************************************/

static void inode_namecpy(char *dest, const char *src)
{
  while (*src && *src != '/')
    {
      *dest++ = *src++;
    }

  *dest = '\0';
}

/****************************************************************************
 * Name: inode_alloc
 ****************************************************************************/

static FAR struct inode *inode_alloc(FAR const char *name)
{
  FAR struct inode *node;
  int namelen;

  namelen = inode_namelen(name);
  node    = (FAR struct inode *)kmm_zalloc(FSNODE_SIZE(namelen));
  if (node)
    {
      inode_namecpy(node->i_name, name);
    }

  return node;
}

/****************************************************************************
 * Name: inode_insert
 ****************************************************************************/

static void inode_insert(FAR struct inode *node,
                         FAR struct inode *peer,
                         FAR struct inode *parent)
{
  /* If peer is non-null, then new node simply goes to the right
   * of that peer node.
   */

  if (peer)
    {
      node->i_peer = peer->i_peer;
      peer->i_peer = node;
    }

  /* Then it must go at the head of parent's list of children. */

  else
    {
      DEBUGASSERT(parent != NULL);
      node->i_peer    = parent->i_child;
      parent->i_child = node;
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: inode_root_reserve
 *
 * Description:
 *   Reserve the root inode for the pseudo file system.
 *
 ****************************************************************************/

void inode_root_reserve(void)
{
  g_root_inode = inode_alloc("");
}

/****************************************************************************
 * Name: inode_reserve
 *
 * Description:
 *   Reserve an (initialized) inode the pseudo file system.  The initial
 *   reference count on the new inode is zero.
 *
 * Input Parameters:
 *   path - The path to the inode to create
 *   inode - The location to return the inode pointer
 *
 * Returned Value:
 *   Zero on success (with the inode point in 'inode'); A negated errno
 *   value is returned on failure:
 *
 *   EINVAL - 'path' is invalid for this operation
 *   EEXIST - An inode already exists at 'path'
 *   ENOMEM - Failed to allocate in-memory resources for the operation
 *
 * Assumptions:
 *   Caller must hold the inode semaphore
 *
 ****************************************************************************/

int inode_reserve(FAR const char *path, FAR struct inode **inode)
{
  struct inode_search_s desc;
  FAR struct inode *left;
  FAR struct inode *parent;
  FAR const char *name;
  int ret;

  /* Assume failure */

  DEBUGASSERT(path != NULL && inode != NULL);
  *inode = NULL;

  if (path[0] == '\0')
    {
      return -EINVAL;
    }

  /* Find the location to insert the new subtree */

  SETUP_SEARCH(&desc, path, false);

  ret = inode_search(&desc);
  if (ret >= 0)
    {
      /* It is an error if the node already exists in the tree (or if it
       * lies within a mountpoint, we don't distinguish here).
       */

      ret = -EEXIST;
      goto errout_with_search;
    }

  /* Now we now where to insert the subtree */

  name   = desc.path;
  left   = desc.peer;
  parent = desc.parent;

  for (; ; )
    {
      FAR struct inode *node;

      /* Create a new node -- we need to know if this is the
       * the leaf node or some intermediary.  We can find this
       * by looking at the next name.
       */

      FAR const char *nextname = inode_nextname(name);
      if (*nextname != '\0')
        {
          /* Insert an operationless node */

          node = inode_alloc(name);
          if (node != NULL)
            {
              inode_insert(node, left, parent);

              /* Set up for the next time through the loop */

              name   = nextname;
              left   = NULL;
              parent = node;
              continue;
            }
        }
      else
        {
          node = inode_alloc(name);
          if (node != NULL)
            {
              inode_insert(node, left, parent);
              *inode = node;
              ret = OK;
              break;
            }
        }

      /* We get here on failures to allocate node memory */

      ret = -ENOMEM;
      break;
    }

errout_with_search:
  RELEASE_SEARCH(&desc);
  return ret;
}
