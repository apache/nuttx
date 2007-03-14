/************************************************************
 * fs_unregisterinode.c
 *
 *   Copyright (C) 2007 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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
 * 3. Neither the name Gregory Nutt nor the names of its contributors may be
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
 ************************************************************/

/************************************************************
 * Included Files
 ************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>
#include <stdlib.h>
#include <errno.h>
#include <nuttx/fs.h>
#include "fs_internal.h"

/************************************************************
 * Definitions
 ************************************************************/

/************************************************************
 * Private Variables
 ************************************************************/

/************************************************************
 * Public Variables
 ************************************************************/

/************************************************************
 * Private Functions
 ************************************************************/

/************************************************************
 * Name: inode_remove
 ************************************************************/

static void inode_remove(struct inode *node,
                         struct inode *peer,
                         struct inode *parent)
{
  /* If peer is non-null, then remove the node from the right of
   * of that peer node.
   */

   if (peer)
     {
       peer->i_peer = node->i_peer;
     }

   /* If parent is non-null, then remove the node from head of
    * of the list of children.
    */

   else if (parent)
     {
       parent->i_child = node->i_peer;
     }

   /* Otherwise, we must be removing the root inode. */

   else
     {
        root_inode = node->i_peer;
     }
   node->i_peer    = NULL;
}

/************************************************************
 * Public Functions
 ************************************************************/

/************************************************************
 * Name: unregister_inode
 ************************************************************/

STATUS unregister_inode(const char *path)
{
  const char       *name = path;
  FAR struct inode *node;
  FAR struct inode *left;
  FAR struct inode *parent;

  if (*path && path[0] == '/')
    {
      return ERROR;
    }

  /* Find the node to delete */

  inode_semtake();
  node = inode_search(&name, &left, &parent);
  if (node)
    {
      /* Found it, now remove it from the tree */

      inode_remove(node, left, parent);

      /* We cannot delete it if there reference to the inode */

      if (node->i_crefs)
        {
           /* In that case, we will mark it deleted, when the FS
            * releases the inode, we will then, finally delete
            * the subtree.
            */

           node->i_flags |= FSNODEFLAG_DELETED;
           inode_semgive();
         }
       else
         {
          /* And delete it now -- recursively to delete all of its children */

          inode_semgive();
          inode_free(node->i_child);
          free(node);
          return OK;
        }
    }

  /* The node does not exist or it has references */

  inode_semgive();
  return ERROR;
}
