/****************************************************************************
 * fs/inode/fs_inodesearch.c
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
#include <string.h>
#include <stdlib.h>
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
static FAR const char *_inode_getcwd(void);

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
      /* At the end of the node name? */

      if (!*nname)
        {
          /* Yes.. also at the end of find name? */

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

      /* At end of the find name? */

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
 *   If the inode is a soft link, then (1) recursively look-up the inode
 *   referenced by the soft link, and (2) return the inode referenced by
 *   the soft link.
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

  /* An infinite loop is avoided only by the loop count. */

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
   * mandatory because only absolute paths are expected in this context.
   */

  DEBUGASSERT(desc != NULL && desc->path != NULL);
  name  = desc->path;

  if (*name != '/')
    {
      return -EINVAL;
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
       * no match in the filesystem.
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
           *   (3) This node is a mountpoint and will absorb all requests
           *       below this one
           */

          name = inode_nextname(name);
          if (*name == '\0' || INODE_IS_MOUNTPT(node))
            {
              /* Either (1) we are at the end of the path, so this must be
               * the node we are looking for or else (2) this node is a
               * mountpoint and will handle the remaining part of the
               * pathname
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
                   * (1) recursively look-up the inode referenced by the
                   * soft link, and (2) continue searching with that inode
                   * instead.
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
                              above   = desc->parent;
                              left    = desc->peer;
                              ret     = OK;

                              if (*desc->relpath != '\0')
                                {
                                  char *buffer = NULL;

                                  ret = asprintf(&buffer,
                                                 "%s/%s", desc->relpath,
                                                 name);
                                  if (ret > 0)
                                    {
                                      kmm_free(desc->buffer);
                                      desc->buffer = buffer;
                                      relpath = buffer;
                                      ret = OK;
                                    }
                                  else
                                    {
                                      ret = -ENOMEM;
                                    }
                                }
                              else
                                {
                                  relpath = name;
                                }

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

  /* The node may or may not be null as per one of the following four cases:
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
 * Name: _inode_getcwd
 *
 * Description:
 *   Return the current working directory
 *
 ****************************************************************************/

static FAR const char *_inode_getcwd(void)
{
  FAR const char *pwd = "";

#ifndef CONFIG_DISABLE_ENVIRON
  pwd = getenv("PWD");
  if (pwd == NULL)
    {
      pwd = CONFIG_LIBC_HOMEDIR;
    }
#endif

  return pwd;
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

  DEBUGASSERT(desc != NULL && desc->path != NULL);

  /* Convert the relative path to the absolute path */

  if (*desc->path != '/')
    {
      ret = asprintf(&desc->buffer, "%s/%s", _inode_getcwd(), desc->path);
      if (ret < 0)
        {
          return -ENOMEM;
        }

      desc->path = desc->buffer;
    }

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

  /* Skip single '.' path segment, but not '..' */

  if (*name == '.' && *(name + 1) == '/')
    {
      /* If there is a '/' after '.',
       * continue searching from the next character
       */

      name = inode_nextname(name);
    }

  return name;
}
