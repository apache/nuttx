/****************************************************************************
 * fs/inode/fs_inode.c
 *
 *   Copyright (C) 2007-2009, 2011-2012 Gregory Nutt. All rights reserved.
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
#include <semaphore.h>
#include <errno.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>

#include "inode/inode.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define NO_HOLDER (pid_t)-1;

/****************************************************************************
 * Private Types
 ****************************************************************************/
/* Implements a re-entrant mutex for inode access.  This must be re-entrant
 * because there can be cycles.  For example, it may be necessary to destroy
 * a block driver inode on umount() after a removable block device has been
 * removed.  In that case umount() hold the inode semaphore, but the block
 * driver may callback to unregister_blockdriver() after the un-mount,
 * requiring the seamphore again.
 */

struct inode_sem_s
{
  sem_t   sem;     /* The semaphore */
  pid_t   holder;  /* The current holder of the semaphore */
  int16_t count;   /* Number of counts held */
};

/****************************************************************************
 * Private Variables
 ****************************************************************************/

static struct inode_sem_s g_inode_sem;

/****************************************************************************
 * Public Variables
 ****************************************************************************/

FAR struct inode *g_root_inode = NULL;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: _inode_compare
 *
 * Description:
 *   Compare two inode names
 *
 ****************************************************************************/

static int _inode_compare(FAR const char *fname,
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

      /* Check for non-matching characters */

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

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: inode_initialize
 *
 * Description:
 *   This is called from the OS initialization logic to configure the file
 *   system.
 *
 ****************************************************************************/

void inode_initialize(void)
{
  /* Initialize the semaphore to one (to support one-at-a-time access to the
   * inode tree).
   */

  (void)sem_init(&g_inode_sem.sem, 0, 1);
  g_inode_sem.holder = NO_HOLDER;
  g_inode_sem.count  = 0;

  /* Initialize files array (if it is used) */

#ifdef CONFIG_HAVE_WEAKFUNCTIONS
  if (files_initialize != NULL)
#endif
    {
      files_initialize();
    }
}

/****************************************************************************
 * Name: inode_semtake
 *
 * Description:
 *   Get exclusive access to the in-memory inode tree (g_inode_sem).
 *
 ****************************************************************************/

void inode_semtake(void)
{
  pid_t me;

  /* Do we already hold the semaphore? */

  me = getpid();
  if (me == g_inode_sem.holder)
    {
      /* Yes... just increment the count */

      g_inode_sem.count++;
      DEBUGASSERT(g_inode_sem.count > 0);
    }

  /* Take the semaphore (perhaps waiting) */

  else
    {
      while (sem_wait(&g_inode_sem.sem) != 0)
        {
          /* The only case that an error should occr here is if
           * the wait was awakened by a signal.
           */

          ASSERT(get_errno() == EINTR);
        }

      /* No we hold the semaphore */

      g_inode_sem.holder = me;
      g_inode_sem.count  = 1;
    }
}

/****************************************************************************
 * Name: inode_semgive
 *
 * Description:
 *   Relinquish exclusive access to the in-memory inode tree (g_inode_sem).
 *
 ****************************************************************************/

void inode_semgive(void)
{
  DEBUGASSERT(g_inode_sem.holder == getpid());

  /* Is this our last count on the semaphore? */

  if (g_inode_sem.count > 1)
    {
      /* No.. just decrement the count */

      g_inode_sem.count--;
    }

  /* Yes.. then we can really release the semaphore */

  else
    {
      g_inode_sem.holder = NO_HOLDER;
      g_inode_sem.count  = 0;
      sem_post(&g_inode_sem.sem);
    }
}

/****************************************************************************
 * Name: inode_search
 *
 * Description:
 *   Find the inode associated with 'path' returning the inode references
 *   and references to its companion nodes.
 *
 * Assumptions:
 *   The caller holds the g_inode_sem semaphore
 *
 ****************************************************************************/

FAR struct inode *inode_search(FAR const char **path,
                               FAR struct inode **peer,
                               FAR struct inode **parent,
                               FAR const char **relpath)
{
  FAR const char   *name  = *path + 1; /* Skip over leading '/' */
  FAR struct inode *node  = g_root_inode;
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
          /* Now there are three more possibilities:
           *   (1) This is the node that we are looking for or,
           *   (2) The node we are looking for is "below" this one.
           *   (3) This node is a mountpoint and will absorb all request
           *       below this one
           */

          name = inode_nextname(name);
          if (!*name || INODE_IS_MOUNTPT(node))
            {
              /* Either (1) we are at the end of the path, so this must be the
               * node we are looking for or else (2) this node is a mountpoint
               * and will handle the remaining part of the pathname
               */

              if (relpath)
                {
                  *relpath = name;
                }
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

  if (peer)
    {
      *peer = left;
    }

  if (parent)
    {
      *parent = above;
    }

  *path = name;
  return node;
}

/****************************************************************************
 * Name: inode_free
 *
 * Description:
 *   Free resources used by an inode
 *
 ****************************************************************************/

void inode_free(FAR struct inode *node)
{
  /* Verify that we were passed valid pointer to an inode */

  if (node != NULL)
    {
      inode_free(node->i_peer);
      inode_free(node->i_child);
      kmm_free(node);
    }
}

/****************************************************************************
 * Name: inode_nextname
 *
 * Description:
 *   Given a path with node names separated by '/', return the next path
 *   segment name.
 *
 ****************************************************************************/

FAR const char *inode_nextname(FAR const char *name)
{
  /* Search for the '/' delimiter or the NUL terminator at the end of the
   * path segment.
   */

   while (*name && *name != '/')
     {
       name++;
     }

   /* If we found the '/' delimiter, then the path segment we want begins at
    * the next character (which might also be the NUL terminator).
    */

   if (*name)
     {
       name++;
     }

   return name;
}
