/****************************************************************************
 * fs/inode/fs_inodesearch.c
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

static int _inode_compare(FAR const char *fname, FAR struct inode *inode);
#ifdef CONFIG_FS_LINKS
static int _inode_linktarget(FAR struct inode *inode,
                             FAR struct inode_search_s *desc);
#endif
static int _inode_search(FAR struct inode_search_s *desc);
static FAR const char *_inode_getcwd(void);
static int _inode_canonicalize(FAR char *path);

/****************************************************************************
 * Public Data
 ****************************************************************************/

FAR struct inode *g_root_inode = NULL;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: _inode_isdot
 ****************************************************************************/

static inline bool _inode_isdot(FAR const char *name)
{
  return name[0] == '.' && (name[1] == '\0' || name[1] == '/');
}

/****************************************************************************
 * Name: _inode_compare
 *
 * Description:
 *   Compare two inode names
 *
 ****************************************************************************/

static int _inode_compare(FAR const char *fname, FAR struct inode *inode)
{
  FAR char *nname = inode->i_name;

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

#ifdef CONFIG_FS_LINKS
static int _inode_linktarget(FAR struct inode *inode,
                             FAR struct inode_search_s *desc)
{
  unsigned int count = 0;
  bool save;
  int ret = -ENOENT;

  DEBUGASSERT(desc != NULL && inode != NULL);

  /* An infinite loop is avoided only by the loop count. */

  save = desc->nofollow;
  while (INODE_IS_SOFTLINK(inode))
    {
      FAR const char *link = (FAR const char *)inode->u.i_link;

      /* Reset and reinitialize the search descriptor.  */

      RELEASE_SEARCH(desc);
      SETUP_SEARCH(desc, link, true);

      /* Look up inode associated with the target of the symbolic link */

      ret = inode_search(desc);
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

      inode = desc->node;
      DEBUGASSERT(inode != NULL);
    }

  desc->nofollow = save;
  return ret;
}
#endif

/****************************************************************************
 * Name: _compute_path_depth
 ****************************************************************************/

static int _compute_path_depth(FAR const char *path)
{
  FAR const char *name = path;
  int depth = 0;

  /* After _inode_canonicalize(), path never contains ".." segments,
   * so we only need to count path components.
   */

  while (*name != '\0')
    {
      depth++;
      name = inode_nextname(name);
    }

  return depth;
}

/****************************************************************************
 * Name: _inode_canonicalize
 *
 * Description:
 *   Remove "." and ".." segments from an absolute path in-place.
 *   The path MUST start with '/'.  Returns -EINVAL if ".." attempts
 *   to ascend beyond the root directory, or -ENAMETOOLONG if the
 *   canonicalized result is >= PATH_MAX bytes.
 *
 ****************************************************************************/

static int _inode_canonicalize(FAR char *path)
{
  /* Skip the initial '/' -- caller guarantees absolute path */

  FAR char *src = path + 1;
  FAR char *dst = path + 1;

  while (*src != '\0')
    {
      /* Skip duplicate slashes */

      if (*src == '/')
        {
          src++;
          continue;
        }

      /* Check for "." (current directory) */

      if (src[0] == '.' && (src[1] == '/' || src[1] == '\0'))
        {
          src += (src[1] == '/') ? 2 : 1;
          continue;
        }

      /* Check for ".." (parent directory) */

      if (src[0] == '.' && src[1] == '.' &&
          (src[2] == '/' || src[2] == '\0'))
        {
          /* Cannot go above root */

          if (dst <= path + 1)
            {
              return -EINVAL;
            }

          /* Remove trailing slash first */

          dst--;

          /* Scan backward to find the previous '/' */

          while (dst > path + 1 && *(dst - 1) != '/')
            {
              dst--;
            }

          src += (src[2] == '/') ? 3 : 2;
          continue;
        }

      /* Regular path component: copy until end of segment (including '/') */

      do
        {
          if (dst != src)
            {
              *dst = *src;
            }

          dst++;
          src++;
        }
      while (*src != '\0' && *(src - 1) != '/');
    }

  /* Remove trailing slash (unless root "/") */

  if (dst > path + 1 && *(dst - 1) == '/')
    {
      dst--;
    }

  *dst = '\0';

  /* After canonicalization, check if the resolved path exceeds PATH_MAX */

  if ((dst - path) >= PATH_MAX)
    {
      return -ENAMETOOLONG;
    }

  return 0;
}

/****************************************************************************
 * Name: _inode_checkpath
 ****************************************************************************/

static int _inode_checkpath(const char *path)
{
  int namelen = 0;
  int pathlen = 0;

  if (*path == '\0')
    {
      return -ENOENT;
    }

  /* Check each segment of the path */

  while (*path != '\0' && pathlen < PATH_MAX)
    {
      if (*path == '/')
        {
          namelen = 0;
        }
      else
        {
          if (++namelen > NAME_MAX)
            {
              return -ENAMETOOLONG;
            }
        }

      path++;
      pathlen++;
    }

  return pathlen >= PATH_MAX ? -ENAMETOOLONG : OK;
}

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
  FAR struct inode *inode   = g_root_inode;
  FAR struct inode *left    = NULL;
  FAR struct inode *above   = NULL;
  FAR const char   *relpath = NULL;
  int ret;

  ret = _inode_checkpath(desc->path);
  if (ret < 0)
    {
      return ret;
    }

  /* Ensure we have a writable buffer for path manipulation */

  if (desc->buffer == NULL)
    {
      FAR const char *cwd = NULL;
      size_t buflen;

      /* For a relative path the absolute form is "<cwd>/<path>".  That
       * concatenation can exceed PATH_MAX even when the relative path
       * itself is within PATH_MAX: a relative path of PATH_MAX-1 bytes
       * is legal per pathconf(_PC_PATH_MAX), but the prefix added by the
       * cwd pushes the uncanonicalized form past the limit.  Size the
       * buffer to hold the full absolute form so that ".." segments are
       * collapsed against the correct suffix; truncating first could
       * drop the trailing component and let ".." collapse the path onto
       * a directory (yielding the wrong errno, e.g. EISDIR, instead of
       * resolving the file).  _inode_canonicalize() still rejects any
       * result whose canonicalized length reaches PATH_MAX.
       */

      if (*desc->path != '/')
        {
          cwd = _inode_getcwd();
          buflen = strlen(cwd) + 1 + strlen(desc->path) + 1;
        }
      else
        {
          buflen = strlen(desc->path) + 1;
        }

      if (buflen < PATH_MAX)
        {
          buflen = PATH_MAX;
        }

      desc->buffer = lib_get_tempbuffer(buflen);
      if (desc->buffer == NULL)
        {
          return -ENOMEM;
        }

      if (cwd != NULL)
        {
          snprintf(desc->buffer, buflen, "%s/%s", cwd, desc->path);
        }
      else
        {
          strlcpy(desc->buffer, desc->path, buflen);
        }

      desc->path = desc->buffer;
    }

  /* Canonicalize the path to remove "." and ".." segments.  This ensures
   * that mountpoint relpath never contains ".." which most filesystems
   * (tmpfs, romfs, etc.) cannot resolve.
   */

  ret = _inode_canonicalize(desc->buffer);
  if (ret < 0)
    {
      return ret;
    }

  name = desc->path;
  ret = -ENOENT;

  /* Traverse the pseudo file system node tree until either (1) all nodes
   * have been examined without finding the matching node, or (2) the
   * matching node is found.
   */

  while (inode != NULL)
    {
      int result = _inode_compare(name, inode);

      /* Case 1:  The name is less than the name of the node.
       * Since the names are ordered, these means that there
       * is no peer node with this name and that there can be
       * no match in the filesystem.
       */

      if (result < 0)
        {
          inode = NULL;
          break;
        }

      /* Case 2: the name is greater than the name of the node.
       * In this case, the name may still be in the list to the
       * "right"
       */

      else if (result > 0)
        {
          /* Continue looking to the "right" of this inode. */

          left  = inode;
          inode = inode->i_peer;
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
          if (*name == '\0' ||
              (INODE_IS_MOUNTPT(inode) && _compute_path_depth(name) > 0))
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

#ifdef CONFIG_FS_LINKS
              /* Was the node a soft link?  If so, then we need need to
               * continue below the target of the link, not the link itself.
               */

              if (INODE_IS_SOFTLINK(inode))
                {
                  int status;

                  /* If this intermediate inode in the is a soft link, then
                   * (1) recursively look-up the inode referenced by the
                   * soft link, and (2) continue searching with that inode
                   * instead.
                   */

                  status = _inode_linktarget(inode, desc);
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

                      if (newnode != inode)
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

                              inode   = newnode;
                              above   = desc->parent;
                              left    = desc->peer;
                              ret     = OK;

                              if (*desc->relpath != '\0')
                                {
                                  FAR char *buffer = NULL;

                                  buffer = lib_get_tempbuffer(PATH_MAX);
                                  if (buffer == NULL)
                                    {
                                      ret = -ENOMEM;
                                    }
                                  else
                                    {
                                      snprintf(buffer, PATH_MAX, "%s/%s",
                                               desc->relpath, name);
                                      lib_put_tempbuffer(desc->buffer);
                                      desc->buffer = buffer;
                                      relpath = buffer;
                                      ret = OK;
                                    }
                                }
                              else
                                {
                                  relpath = name;
                                }

                              break;
                            }

                          /* Continue from this new inode. */

                          inode = newnode;
                        }
                    }
                }
#endif

              /* Keep looking at the next level "down" */

              above = inode;
              left  = NULL;
              inode = inode->i_child;
              if (!INODE_IS_PSEUDODIR(above))
                {
                  /* The prefix of the path is not a directory */

                  ret = -ENOTDIR;
                  break;
                }
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
  desc->node    = inode;
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

  ret = _inode_search(desc);

#ifdef CONFIG_FS_LINKS
  if (ret >= 0)
    {
      FAR struct inode *inode;

      /* Search completed successfully */

      inode = desc->node;
      DEBUGASSERT(inode != NULL);

      /* Is the terminal node a softlink? Should we follow it? */

      if (!desc->nofollow && INODE_IS_SOFTLINK(inode))
        {
          /* The terminating inode is a valid soft link:  Return the inode,
           * corresponding to link target.  _inode_linktarget() will follow
           * a link (or a series of links to links) and will return the
           * link target of the final symbolic link in the series.
           */

          ret = _inode_linktarget(inode, desc);
          if (ret < 0)
            {
              /* The most likely cause for failure is that the target of the
               * symbolic link does not exist.
               */

              return ret;
            }
        }
      else if (!desc->nofollow && INODE_IS_HARDLINK(inode))
        {
          /* The terminating inode is a valid hard link */

          inode = inode->i_private;
          DEBUGASSERT(inode != NULL);

          desc->node = inode;
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

  /* Skip single '.' path segment, but not '..'. This includes a lone
   * trailing '.' as the final path component (e.g. "/foo/."), which
   * refers to "foo" itself the same way "/foo/./" would -- without this,
   * a trailing '.' is instead treated as a literal child name to look up
   * under "foo" and fails to resolve, since no real node is ever named
   * ".", rather than resolving to the node the search already reached.
   */

  if (_inode_isdot(name))
    {
      if (*(name + 1) == '/')
        {
          /* If there is a '/' after '.',
           * continue searching from the next character
           */

          name = inode_nextname(name);
        }
      else
        {
          /* Lone trailing '.': point past it, at the terminating NUL,
           * the same as if the path had ended one character earlier.
           */

          name++;
        }
    }

  return name;
}
