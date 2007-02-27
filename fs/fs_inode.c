/************************************************************
 * fs_inode.c
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
 * Compilation Switches
 ************************************************************/

/************************************************************
 * Included Files
 ************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>

#include <stdlib.h>
#include <assert.h>
#include <semaphore.h>
#include <errno.h>

#include <nuttx/fs.h>
#include "fs_internal.h"

/************************************************************
 * Definitions
 ************************************************************/

#define INODE_SEMGIVE() \
  sem_post(&tree_sem)

/************************************************************
 * Private Variables
 ************************************************************/

static sem_t tree_sem;

/************************************************************
 * Public Variables
 ************************************************************/

FAR struct inode *root_inode = NULL;

/************************************************************
 * Private Functions
 ************************************************************/

static void _inode_semtake(void)
{
  /* Take the semaphore (perhaps waiting) */

  while (sem_wait(&tree_sem) != 0)
    {
      /* The only case that an error should occr here is if
       * the wait was awakened by a signal.
       */

      ASSERT(*get_errno_ptr() == EINTR);
    }
}

#define _inode_semgive(void) sem_post(&tree_sem)

static int _inode_compare(const char *fname,
                          FAR struct inode *node)
{
  char *nname = node->i_name;

  if (!nname)
    {
      return 1;
    }

  if (!fname)
    {
      return -1;
    }

  for (;;)
    {
      /* At end of node name? */
      if (!*nname)
        {
           /* Yes.. also end of find name? */
           if (!*fname || *fname == '/')
             {
               /* Yes.. return match */
               return 0;
             }
           else
             {
               /* No... return find name > node name */
               return 1;
             }
        }

      /* At end of find name?*/
      else if (!*fname || *fname == '/')
        {
           /* Yes... return find name < node name */
           return -1;
        }

      /* check for non-matching characters */
      else if (*fname > *nname)
        {
          return 1;
        }
      else if (*fname < *nname)
        {
          return -1;
        }

      /* Not at the end of either string and all of the
       * characters still match.  keep looking.
       */
      else
        {
          fname++;
          nname++;
        }
    }
}

static int _inode_namelen(const char *name)
{
  const char *tmp = name;
  while(*tmp && *tmp != '/') tmp++;
  return tmp - name;
}

static void _inode_namecpy(char *dest, const char *src)
{
  while(*src && *src != '/') *dest++ = *src++;
  *dest='\0';
}

static const char *_inode_nextname(const char *name)
{
   while (*name && *name != '/') name++;
   if (*name) name++;
   return name;
}

static FAR struct inode *_inode_alloc(const char *name,
                                      struct file_operations *fops,
                                      mode_t mode, void *private)
{
  int namelen = _inode_namelen(name);
  FAR struct inode *node = (FAR struct inode*)malloc(FSNODE_SIZE(namelen));
  if (node)
    {
      node->i_peer    = NULL;
      node->i_child   = NULL;
      node->i_ops     = fops;
#ifdef CONFIG_FILE_MODE
      node->i_mode    = mode;
#endif
      node->i_private = private;
      _inode_namecpy(node->i_name, name);
    }
  return node;
}

static FAR struct inode *_inode_find(const char **path,
                                     FAR struct inode **peer,
                                     FAR struct inode **parent)
{
  const char       *name  = *path + 1; /* Skip over leading '/' */
  FAR struct inode *node  = root_inode;
  FAR struct inode *left  = NULL;
  FAR struct inode *above = NULL;

  while (node)
    {
      int result = _inode_compare(name, node);

      /* Case 1:  The name is less than the name of the node.
       * Since the names are ordered, these means that there
       * is no peer node with this name and that there can be
       * no match in the fileystem.
       */

      if (result < 0)
        {
          node = NULL;
          break;
        }

      /* Case 2: the name is greater than the name of the node.
       * In this case, the name may still be in the list to the
       * "right"
       */

      else if (result > 0)
        {
          left = node;
          node = node->i_peer;
        }

      /* The names match */

      else
        {
          /* Now there are two more possibilities:
           *   (1) This is the node that we are looking for or,
           *   (2) the node we are looking for is "blow" this one.
           */

          name = _inode_nextname(name);
          if (!*name)
            {
              /* We are at the end of the path, so this must be
               * the node we are looking for.
               */

              break;
            }
          else
            {
              /* More to go, keep looking at the next level "down" */

              above = node;
              left  = NULL;
              node = node->i_child;
            }
        }
    }

  /* node is null.  This can happen in one of four cases:
   * With node = NULL
   *   (1) We went left past the final peer:  The new node
   *       name is larger than any existing node name at
   *       that level.
   *   (2) We broke out in the middle of the list of peers
   *       because the name was not found in the ordered
   *       list.
   *   (3) We went down past the final parent:  The new node
   *       name is "deeper" than anything that we currently
   *       have in the tree.
   * with node != NULL
   *   (4) When the node matching the full path is found
   */

  if (peer)   *peer   = left;
  if (parent) *parent = above;
  *path = name;
  return node;
}

static void _inode_insert(FAR struct inode *node,
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

   /* If parent is non-null, then it must go at the head of its
    * list of children.
    */

   else if (parent)
     {
       node->i_peer    = parent->i_child;
       parent->i_child = node;
     }

   /* Otherwise, this must be the new root_inode */

   else
     {
       node->i_peer = root_inode;
       root_inode   = node;
     }
}

static void _inode_remove(struct inode *node,
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

static void _inode_free(FAR struct inode *node)
{
  if (node)
    {
      _inode_free(node->i_peer);
      _inode_free(node->i_child);
      free(node);
    }
}

/************************************************************
 * Public Functions
 ************************************************************/

/* This is called from the OS initialization logic to configure
 * the file system.
 */

void fs_initialize(void)
{
  /* Initialize the semaphore to one (to support one-at-
   * a-time access to the inode tree).
   */

  (void)sem_init(&tree_sem, 0, 1);

  /* Initialize files array (if it is used) */

#ifdef CONFIG_HAVE_WEAKFUNCTIONS
  if (files_initialize != NULL)
#endif
    {
      files_initialize();
    }
}

/* This is called from the open() logic to get a reference
 * to the inode associatged with a path.
 */

FAR struct inode *inode_find(const char *path)
{
  FAR struct inode *node;

  if (!*path || path[0] != '/')
    {
      return NULL;
    }

  /* Find the node matching the path.  If found,
   * increment the count of references on the node.
   */

  _inode_semtake();
  node = _inode_find(&path, (FAR void*)NULL, (FAR void*)NULL);
  if (node) node->i_crefs++;
  _inode_semgive();
  return node;
}

/* Increment the reference count on an inode (as when a file
 * descriptor is dup'ed.
 */

void inode_addref(FAR struct inode *inode)
{
  if (inode)
    {
      _inode_semtake();
      inode->i_crefs++;
      _inode_semgive();
    }
}

/* This is called from close() logic when it no longer refers
 * to the inode.
 */

void inode_release(FAR struct inode *node)
{
  if (node)
    {
      /* Decrement the references of the inode */

      _inode_semtake();
      if (node->i_crefs)
        {
          node->i_crefs--;
        }

      /* If the subtree was previously deleted and the reference
       * count has decrement to zero,  then delete the inode
       * now.
       */

      if (node->i_crefs <= 0 && (node->i_flags & FSNODEFLAG_DELETED) != 0)
        {
           _inode_semgive();
           _inode_free(node->i_child);
           free(node);
        }
      else
        {
           _inode_semgive();
        }
    }
}


STATUS register_inode(const char *path,
                      struct file_operations *fops,
                      mode_t mode, void *private)
{
  const char       *name = path;
  FAR struct inode *left;
  FAR struct inode *parent;

  if (!*path || path[0] != '/')
    {
      return ERROR;
    }

  /* Find the location to insert the new subtree */

  _inode_semtake();
  if (_inode_find(&name, &left, &parent) != NULL)
    {
      /* Is is an error if the node already exists in the tree */

      _inode_semgive();
      return ERROR;
    }

  /* Now we now where to insert the subtree */

  for (;;)
    {
      FAR struct inode *node;

      /* Create a new node -- we need to know if this is the
       * the leaf node or some intermediary.  We can find this
       * by looking at the next name.
       */

      const char *next_name = _inode_nextname(name);
      if (*next_name)
        {
          /* Insert an operationless node */

          node = _inode_alloc(name, NULL, mode, NULL);
          if (node)
            {
              _inode_insert(node, left, parent);

              /* Set up for the next time through the loop */

              name   = next_name;
              left   = NULL;
              parent = node;
              continue;
            }
        }
      else
        {
          node = _inode_alloc(name, fops, mode, private);
          if (node)
            {
              _inode_insert(node, left, parent);
              _inode_semgive();
              return 0;
            }
        }

      /* We get here on failures to allocate node memory */

      _inode_semgive();
      return ERROR;
    }
}

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

  _inode_semtake();
  node = _inode_find(&name, &left, &parent);
  if (node)
    {
      /* Found it, now remove it from the tree */

      _inode_remove(node, left, parent);

      /* We cannot delete it if there reference to the inode */

      if (node->i_crefs)
        {
           /* In that case, we will mark it deleted, when the FS
            * releases the inode, we will then, finally delete
            * the subtree.
            */

           node->i_flags |= FSNODEFLAG_DELETED;
           _inode_semgive();
         }
       else
         {
          /* And delete it now -- recursively to delete all of its children */

          _inode_semgive();
          _inode_free(node->i_child);
          free(node);
          return OK;
        }
    }

  /* The node does not exist or it has references */
  _inode_semgive();
  return ERROR;
}
