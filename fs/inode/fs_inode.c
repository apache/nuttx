/****************************************************************************
 * fs/inode/fs_inode.c
 *
 *   Copyright (C) 2007-2009, 2011-2012, 2016-2017 Gregory Nutt. All rights reserved.
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
#include <limits.h>
#include <semaphore.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>

#include "inode/inode.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define NO_HOLDER ((pid_t)-1)

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
 * Private Data
 ****************************************************************************/

static struct inode_sem_s g_inode_sem;

/****************************************************************************
 * Public Data
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

  for (; ; )
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

      /* At end of find name? */

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
 * Name: _inode_dereference
 *
 * Description:
 *   If the inode is a soft link, then (1) get the name of the full path of
 *   the soft link, (2) recursively look-up the inode referenced by the soft
 *   link, and (3) return the inode referenced by the soft link.
 *
 * Assumptions:
 *   The caller holds the g_inode_sem semaphore
 *
 ****************************************************************************/

#ifdef CONFIG_PSEUDOFS_SOFTLINKS
static inline FAR struct inode *
_inode_dereference(FAR struct inode *node, FAR struct inode **peer,
                   FAR struct inode **parent, FAR const char **relpath)
{
  FAR const char *copy;
  unsigned int count = 0;

  /* An infinite loop is avoided only by the loop count.
  *
   * REVISIT:  The ELOOP error should be reported to the application in that
   * case but there is no simple mechanism to do that.
   */

  while (node != NULL && INODE_IS_SOFTLINK(node))
    {
      /* Careful: inode_search_nofollow overwrites the input string pointer */

      copy = (FAR const char *)node->u.i_link;

      /* Now, look-up the inode associated with the target path */

      node = inode_search_nofollow(&copy, peer, parent, relpath);
      if (node == NULL && ++count > SYMLOOP_MAX)
        {
          return NULL;
        }
    }

  return node;
}
#endif

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
 * Name: inode_search and inode_search_nofollow
 *
 * Description:
 *   Find the inode associated with 'path' returning the inode references
 *   and references to its companion nodes.
 *
 *   Both versions will follow soft links in path leading up to the terminal
 *   node.  inode_search() will deference that terminal node,
 *    inode_search_nofollow will not.
 *
 * Assumptions:
 *   The caller holds the g_inode_sem semaphore
 *
 ****************************************************************************/

#ifdef CONFIG_PSEUDOFS_SOFTLINKS

FAR struct inode *inode_search_nofollow(FAR const char **path,
                                        FAR struct inode **peer,
                                        FAR struct inode **parent,
                                        FAR const char **relpath)
#else
FAR struct inode *inode_search(FAR const char **path,
                               FAR struct inode **peer,
                               FAR struct inode **parent,
                               FAR const char **relpath)
#endif
{
  FAR const char   *name  = *path + 1; /* Skip over leading '/' */
  FAR struct inode *node  = g_root_inode;
  FAR struct inode *left  = NULL;
  FAR struct inode *above = NULL;
#ifdef CONFIG_PSEUDOFS_SOFTLINKS
  FAR struct inode *newnode;
#endif

#ifdef CONFIG_PSEUDOFS_SOFTLINKS
  /* Handle the case were the root node is a symbolic link */

#warning Missing logic
#endif

  /* Traverse the pseudo file system node tree until either (1) all nodes
   * have been examined without finding the matching node, or (2) the
   * matching node is found.
   */

  while (node != NULL)
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
#ifdef CONFIG_PSEUDOFS_SOFTLINKS
         /* If the inode in the is a soft link and this is the inode at
          * at the head of the peer list and not the final node in the
          * path), then (1) get the name of the full path of the soft
          * link, (2) recursively look-up the inode referenced by the
          * soft link, and (3) use the peer of that inode instead.
          */

          FAR const char *nextname = inode_nextname(name);
          if (*nextname != '\0')
            {
              newnode = _inode_dereference(node, NULL, &above, relpath);
              if (newnode == NULL)
                {
                  /* Probably means that the node is a symbolic link, but
                   * that the target of the symbolic link does not exist.
                   */

                  break;
                }
              else if (newnode != node)
                {
                  /* The node was a valid symbolic link and we have jumped to a
                   * different, spot in the the pseudo file system tree.  Reset
                   * everything and continue looking at the next level "down"
                   * from that new spot in the tree.
                   */

                  above = newnode;
                  left  = NULL;
                  node  = newnode->i_child;
                  continue;
                }
            }
#endif
          /* Continue looking to the left */

          left = node;
          node = node->i_peer;
        }

      /* The names match */

      else
        {
          /* Now there are three remaining possibilities:
           *   (1) This is the node that we are looking for.
           *   (2) The node we are looking for is "below" this one.
           *   (3) This node is a mountpoint and will absorb all request
           *       below this one
           */

          name = inode_nextname(name);
          if (*name == '\0' || INODE_IS_MOUNTPT(node))
            {
              /* Either (1) we are at the end of the path, so this must be the
               * node we are looking for or else (2) this node is a mountpoint
               * and will handle the remaining part of the pathname
               */

              if (relpath)
                {
                  *relpath = name;
                }

#ifdef CONFIG_PSEUDOFS_SOFTLINKS
              /* NOTE that if the terminal inode is a soft link, it is not
               * deferenced in this case. The raw inode is returned.
               *
               * In that case a wrapper function will perform that operation.
               */
#endif
              break;
            }
          else
            {
              /* More nodes to be examined in the path... */

#ifdef CONFIG_PSEUDOFS_SOFTLINKS
              /* If this intermediate inode in the is a soft link, then (1)
               * get the name of the full path of the soft link, (2) recursively
               * look-up the inode referenced by the sof link, and (3)
               * continue searching with that inode instead.
               */

              newnode = _inode_dereference(node,  NULL, NULL, relpath);
              if (newnode == NULL)
                {
                  /* Probably means that the node is a symbolic link, but
                   * that the target of the symbolic link does not exist.
                   */

                  break;
                }
              else if (newnode != node)
                {
                  /* The node was a valid symbolic link and we have jumped to a
                   * different, spot in the the pseudo file system tree.  Reset
                   * everything and continue looking to the right (if possible)
                   * otherwise at the next level "down" from that new spot in
                   * the tree.
                   */

                  if (newnode->i_peer != NULL)
                    {
                      above = NULL;  /* REVISIT: This can't be right */
                      left  = newnode;
                      node  = newnode->i_peer;

                      /* Did the symbolic link take us to a mountpoint? */

                      if (INODE_IS_MOUNTPT(newnode))
                        {
                          /* Yes.. return the mountpoint information */

                          if (relpath)
                           {
                             *relpath = name;
                           }

                          break;
                        }
                    }
                }
#endif
              /* Keep looking at the next level "down" */

              above = node;
              left  = NULL;
              node  = node->i_child;
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

#ifdef CONFIG_PSEUDOFS_SOFTLINKS
FAR struct inode *inode_search(FAR const char **path,
                               FAR struct inode **peer,
                               FAR struct inode **parent,
                               FAR const char **relpath)
{
  /* Lookup the terminal inode */

  FAR struct inode *node = inode_search_nofollow(path, peer, parent, relpath);

  /* Did we find it? */

  if (node != NULL)
    {
      /* Yes.. If the terminal inode in the is a soft link, then (1) get
       * the name of the full path of the soft link, (2) recursively
       * look-up the inode referenced by the soft link, and (3)
       * return that inode instead.
       */

       return _inode_dereference(node, peer, parent, relpath);
    }

  return node;
}
#endif

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
#ifdef CONFIG_PSEUDOFS_SOFTLINKS
      /* Symbol links should never have peers or children */

      DEBUGASSERT(!INODE_IS_SOFTLINK(node) ||
                  (node->i_peer == NULL && node->i_child == NULL));
#endif

      /* Free all peers and children of this i_node */

      inode_free(node->i_peer);
      inode_free(node->i_child);

#ifdef CONFIG_PSEUDOFS_SOFTLINKS
      /* If the inode is a symbolic link, the free the path to the linked
       * entity.
       */

      if (INODE_IS_SOFTLINK(node) && node->u.i_link != NULL)
        {
          kmm_free(node->u.i_link);
        }
#endif

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
