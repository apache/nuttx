/****************************************************************************
 * fs/inode/fs_inodesearch.c
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

#include <stdio.h>
#include <string.h>
#include <limits.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/fs/fs.h>

#include "inode/inode.h"

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int _inode_compare(FAR const char *fname, FAR struct inode *node);
#ifdef CONFIG_PSEUDOFS_SOFTLINKS
static int _inode_linktarget(FAR struct inode *node,
                             FAR struct inode_search_s *desc);
#endif
static int _inode_search(FAR struct inode_search_s *desc);

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

static int _inode_compare(FAR const char *fname, FAR struct inode *node)
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
 * Name: _inode_linktarget
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
static int _inode_linktarget(FAR struct inode *node,
                             FAR struct inode_search_s *desc)
{
  unsigned int count = 0;
  bool save;
  int ret = -ENOENT;

  DEBUGASSERT(desc != NULL && node != NULL);

  /* An infinite loop is avoided only by the loop count.
   *
   * REVISIT:  The ELOOP error should be reported to the application in that
   * case but there is no simple mechanism to do that.
   */

  save = desc->nofollow;
  while (INODE_IS_SOFTLINK(node))
    {
      FAR const char *link = (FAR const char *)node->u.i_link;

      /* Reset and reinitialize the search descriptor.  */

      RELEASE_SEARCH(desc);
      SETUP_SEARCH(desc, link, true);

      /* Look up inode associated with the target of the symbolic link */

      ret = _inode_search(desc);
      if (ret < 0)
        {
          break;
        }

      /* Limit the number of symbolic links that we pass through */

      if (++count > SYMLOOP_MAX)
        {
          ret = -ELOOP;
          break;
        }

      /* Set up for the next time through the loop */

      node = desc->node;
      DEBUGASSERT(node != NULL);
      desc->linktgt = link;
    }

  desc->nofollow = save;
  return ret;
}
#endif

/****************************************************************************
 * Name: _inode_search
 *
 * Description:
 *   Find the inode associated with 'path' returning the inode references
 *   and references to its companion nodes.  This is the internal, common
 *   implementation of inode_search().
 *
 *   If a mountpoint is encountered in the search prior to encountering the
 *   terminal node, the search will terminate at the mountpoint inode.  That
 *   inode and the relative path from the mountpoint, 'relpath' will be
 *   returned.
 *
 *   If a soft link is encountered that is not the terminal node in the path,
 *   that link WILL be deferenced unconditionally.
 *
 * Assumptions:
 *   The caller holds the g_inode_sem semaphore
 *
 ****************************************************************************/

static int _inode_search(FAR struct inode_search_s *desc)
{
  FAR const char   *name;
  FAR struct inode *node    = g_root_inode;
  FAR struct inode *left    = NULL;
  FAR struct inode *above   = NULL;
  FAR const char   *relpath = NULL;
  int ret = -ENOENT;

  /* Get the search path, skipping over the leading '/'.  The leading '/' is
   * mandatory because only absolte paths are expected in this context.
   */

  DEBUGASSERT(desc != NULL && desc->path != NULL);
  name  = desc->path;

  if (*name != '/')
    {
      return -EINVAL;
    }

  /* Skip over the leading '/' */

  while (*name == '/')
    {
      name++;
    }

  /* Special case the root directory.  There is no root inode and there is
   * no name for the root.
   */

  if (*name == '\0')
    {
      /* This is a bug.  I don't know how to handle this case yet. */

      return -ENOSYS;
    }

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
          /* Continue looking to the "right" of this inode. */

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

              relpath = name;
              ret = OK;
              break;
            }
          else
            {
              /* More nodes to be examined in the path "below" this one. */

#ifdef CONFIG_PSEUDOFS_SOFTLINKS
              /* Was the node a soft link?  If so, then we need need to
               * continue below the target of the link, not the link itself.
               */

              if (INODE_IS_SOFTLINK(node))
                {
                  int status;

                  /* If this intermediate inode in the is a soft link, then
                   * (1) get the name of the full path of the soft link, (2)
                   * recursively look-up the inode referenced by the soft
                   * link, and (3) continue searching with that inode instead.
                   */

                  status = _inode_linktarget(node, desc);
                  if (status < 0)
                    {
                      /* Probably means that the target of the symbolic link
                       * does not exist.
                       */

                      ret = status;
                      break;
                    }
                  else
                    {
                      FAR struct inode *newnode = desc->node;

                      if (newnode != node)
                        {
                          /* The node was a valid symbolic link and we have
                           * jumped to a different, spot in the pseudo file
                           * system tree.
                           */

                          /* Check if this took us to a mountpoint. */

                          if (INODE_IS_MOUNTPT(newnode))
                            {
                              /* Return the mountpoint information.
                               * NOTE that the last path to the link target
                               * was already set by _inode_linktarget().
                               */

                              node    = newnode;
                              above   = NULL;
                              left    = NULL;
                              relpath = name;

                              ret     = OK;
                              break;
                            }

                          /* Continue from this new inode. */

                          node = newnode;
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

  /* The node may or may not be null as per one of the following four cases
   * cases:
   *
   * With node = NULL
   *
   *   (1) We went left past the final peer:  The new node name is larger
   *       than any existing node name at that level.
   *   (2) We broke out in the middle of the list of peers because the name
   *       was not found in the ordered list.
   *   (3) We went down past the final parent:  The new node name is
   *       "deeper" than anything that we currently have in the tree.
   *
   * With node != NULL
   *
   *   (4) When the node matching the full path is found
   */

  desc->path    = name;
  desc->node    = node;
  desc->peer    = left;
  desc->parent  = above;
  desc->relpath = relpath;
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: inode_search
 *
 * Description:
 *   Find the inode associated with 'path' returning the inode references
 *   and references to its companion nodes.
 *
 *   If a mountpoint is encountered in the search prior to encountering the
 *   terminal node, the search will terminate at the mountpoint inode.  That
 *   inode and the relative path from the mountpoint, 'relpath' will be
 *   returned.
 *
 *   inode_search will follow soft links in path leading up to the terminal
 *   node.  Whether or no inode_search() will deference that terminal node
 *   depends on the 'nofollow' input.
 *
 *   If a soft link is encountered that is not the terminal node in the path,
 *   that link WILL be deferenced unconditionally.
 *
 * Assumptions:
 *   The caller holds the g_inode_sem semaphore
 *
 ****************************************************************************/

int inode_search(FAR struct inode_search_s *desc)
{
  int ret;

  /* Perform the common _inode_search() logic.  This does everything except
   * operations special operations that must be performed on the terminal
   * node if node is a symbolic link.
   */

  DEBUGASSERT(desc != NULL);
#ifdef CONFIG_PSEUDOFS_SOFTLINKS
  desc->linktgt = NULL;
#endif

  ret = _inode_search(desc);

#ifdef CONFIG_PSEUDOFS_SOFTLINKS
  if (ret >= 0)
    {
      FAR struct inode *node;

      /* Search completed successfully */

      node    = desc->node;
      DEBUGASSERT(node != NULL);

      /* Is the terminal node a softlink? Should we follow it? */

      if (!desc->nofollow && INODE_IS_SOFTLINK(node))
        {
          /* Save some things we need that will be clobbered by the call to
           * _inode_linktgt().
           */

          FAR const char *relpath = desc->relpath; /* Will always be "" here */

          /* The terminating inode is a valid soft link:  Return the inode,
           * corresponding to link target.  _inode_linktarget() will follow
           * a link (or a series of links to links) and will return the
           * link target of the final symbolic link in the series.
           */

          ret = _inode_linktarget(node, desc);
          if (ret < 0)
            {
              /* The most likely cause for failure is that the target of the
               * symbolic link does not exist.
               */

              return ret;
            }

          /* The dereferenced node might be a mountpoint */

          node = desc->node;
          DEBUGASSERT(node != NULL && desc->linktgt != NULL);

          if (INODE_IS_MOUNTPT(node))
            {
               /* Yes... set up for the MOUNTPOINT logic below. */

              desc->relpath = relpath;
            }
        }

      /* Handle a special case.  This special occurs with either (1)
       * inode_search() terminates early because it encountered a MOUNTPOINT
       * at an intermediate node in the path, or (2) inode_search()
       * terminates because it reached the terminal node and 'nofollow' is
       * false and the above logic converted the symbolic link to a
       * MOUNTPOINT.
       *
       * We can detect the special cases because desc->linktgt will be
       * non-NULL.
       */

      if (desc->linktgt != NULL && INODE_IS_MOUNTPT(node))
        {
          FAR char *buffer;

          /* There would be no problem in this case if the link was to
           * either to the root directory of the MOUNTPOINT or to a
           * regular file within the mounted volume.  However, there
           * is a problem if the symbolic link is to a directory within
           * the mounted volume.  In that case, the 'relpath' will be
           * relative to the symbolic link and not to the MOUNTPOINT.
           *
           * We will handle the worst case by creating the full path
           * excluding the symbolic link and performing the look-up
           * again.
           */

          if (desc->relpath != NULL && *desc->relpath != '\0')
            {
              (void)asprintf(&buffer, "%s/%s",
                             desc->linktgt, desc->relpath);
            }
          else
            {
              buffer = strdup(desc->linktgt);
            }

          if (buffer == NULL)
            {
              ret = -ENOMEM;
            }
          else
            {
              /* Reset the search description and perform the search again. */

              RELEASE_SEARCH(desc);
              SETUP_SEARCH(desc, buffer, false);
              desc->buffer = buffer;

              ret = _inode_search(desc);
            }
        }
    }
#endif

  return ret;
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

  while (*name != '\0' && *name != '/')
    {
      name++;
    }

  /* If we found the '/' delimiter, then the path segment we want begins at
   * the next character (which might also be the NUL terminator).
   */

  while (*name == '/')
    {
      name++;
    }

  return name;
}
