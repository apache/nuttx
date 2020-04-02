/****************************************************************************
 * fs/dirent/fs_opendir.c
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

#include <stdbool.h>
#include <string.h>
#include <ctype.h>
#include <dirent.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/dirent.h>

#include "inode/inode.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: open_mountpoint
 *
 * Description:
 *   Handle the case where the inode to be opened is within a mountpoint.
 *
 * Input Parameters:
 *   inode -- the inode of the mountpoint to open
 *   relpath -- the relative path within the mountpoint to open
 *   dir -- the dirent structure to be initialized
 *
 * Returned Value:
 *   On success, OK is returned; Otherwise, a positive errno is returned.
 *
 ****************************************************************************/

#ifndef CONFIG_DISABLE_MOUNTPOINT
static inline int open_mountpoint(FAR struct inode *inode,
                                  FAR const char *relpath,
                                  FAR struct fs_dirent_s *dir)
{
  int ret;

  /* The inode itself as the 'root' of mounted volume.  The actually
   * directory is at relpath into the* mounted filesystem.
   *
   * Verify that the mountpoint inode  supports the opendir() method
   */

  if (!inode->u.i_mops || !inode->u.i_mops->opendir)
    {
      return ENOSYS;
    }

  /* Take reference to the mountpoint inode.  Note that we do not use
   * inode_addref() because we already hold the tree semaphore.
   */

  inode->i_crefs++;

  /* Perform the opendir() operation */

  ret = inode->u.i_mops->opendir(inode, relpath, dir);
  if (ret < 0)
    {
      /* We now need to back off our reference to the inode.  We can't
       * call inode_release() to do that unless we release the tree
       * semaphore.  The following should be safe because:  (1) after the
       * reference count was incremented above it should be >=1 so it should
       * not decrement below zero, and (2) we hold the tree semaphore so no
       * other thread should be able to change the reference count.
       */

      inode->i_crefs--;
      DEBUGASSERT(inode->i_crefs >= 0);

      /* Negate the error value so that it can be used to set errno */

      return -ret;
    }

  return OK;
}
#endif

/****************************************************************************
 * Name: open_pseudodir
 *
 * Description:
 *   Handle the case where the inode to be opened is within the top-level
 *   pseudo-file system.
 *
 * Input Parameters:
 *   inode -- the inode of the mountpoint to open
 *   dir -- the dirent structure to be initialized
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void open_pseudodir(FAR struct inode *inode,
                           FAR struct fs_dirent_s *dir)
{
  /* We have a valid pseudo-filesystem node.  Take two references on the
   * inode -- one for the parent (fd_root) and one for the child (fd_next).
   * Note that we do not call inode_addref because we are holding the tree
   * semaphore and that would result in deadlock.
   */

  inode->i_crefs       += 2;
  dir->fd_root          = inode; /* Save the inode where we start */
  dir->u.pseudo.fd_next = inode; /* This is the next node to use for readdir() */

  /* Flag the inode as belonging to the pseudo-filesystem */

#ifndef CONFIG_DISABLE_MOUNTPOINT
  DIRENT_SETPSEUDONODE(dir->fd_flags);
#endif
}

/****************************************************************************
 * Name: open_emptydir
 *
 * Description:
 *   Handle the case where the inode to be opened is an empty, directory node
 *   within the top-level pseudo-file system.  That is, it has no operations
 *   and, therefore, it must be a directory node.  But is has no children
 *   to be enumerated either.
 *
 * Input Parameters:
 *   dir -- the dirent structure to be initialized
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void open_emptydir(FAR struct fs_dirent_s *dir)
{
  /* We have a valid, but empty pseudo-filesystem node.  fd_next is NULL
   * meaning that we are already at the end of the list of its children.
   * fd_root is NULL so that if the directory is rewound, it will still be
   * at the end of the list.
   */

#if 0 /* Already nullified by kumm_zalloc */
  dir->fd_root          = NULL; /* Save the inode where we start */
  dir->u.pseudo.fd_next = NULL; /* We are at the end of the list */
#endif

  /* Flag the inode as belonging to the pseudo-filesystem */

#ifndef CONFIG_DISABLE_MOUNTPOINT
  DIRENT_SETPSEUDONODE(dir->fd_flags);
#endif
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: opendir
 *
 * Description:
 *   The  opendir() function opens a directory stream corresponding to the
 *   directory name, and returns a pointer to the directory stream. The
 *   stream is positioned at the first entry in the directory.
 *
 * Input Parameters:
 *   path -- the directory to open
 *
 * Returned Value:
 *   The opendir() function returns a pointer to the directory stream.  On
 *   error, NULL is returned, and errno is set appropriately.
 *
 *   EACCES  - Permission denied.
 *   EMFILE  - Too many file descriptors in use by process.
 *   ENFILE  - Too many files are currently open in the
 *             system.
 *   ENOENT  - Directory does not exist, or name is an empty
 *             string.
 *   ENOMEM  - Insufficient memory to complete the operation.
 *   ENOTDIR - 'path' is not a directory.
 *
 ****************************************************************************/

FAR DIR *opendir(FAR const char *path)
{
  FAR struct inode *inode = NULL;
  FAR struct fs_dirent_s *dir;
  struct inode_search_s desc;
#ifndef CONFIG_DISABLE_MOUNTPOINT
  FAR const char *relpath = NULL;
#endif
  FAR char *alloc = NULL;
  bool isroot = false;
  int len;
  int ret;

  /* Strip off any trailing whitespace or '/' characters.  In this case we
   * must make a copy of the user string so we can chop off bytes on the
   * 'right' without modifying the user's const string.
   */

  if (path != NULL)
    {
      /* Length of the string excludes NUL terminator */

      len = strlen(path);

      /* Check for whitespace or a dangling '/' at the end of the string.
       * But don't muck with the string any further if it has been reduced
       * to "/"
       */

      while (len > 0 && strcmp(path, "/") != 0 &&
             (isspace(path[len - 1]) || path[len - 1] == '/'))
        {
          /* Have we already allocated memory for the modified string? */

          if (alloc == NULL)
            {
              alloc = strdup(path); /* Allocates one too many bytes */
              if (alloc == NULL)
                {
                  ret = ENOMEM;
                  goto errout;
                }

              /* Use the cloned, writable string instead of the user string */

              path = alloc;
            }

          /* Chop off the final character */

          len--;
          alloc[len] = '\0';
        }
    }

  /* If we are given 'nothing' then we will interpret this as
   * request for the root inode.
   */

  SETUP_SEARCH(&desc, path, false);

  ret = inode_semtake();
  if (ret < 0)
    {
      ret = -ret;
      goto errout_with_alloc;
    }

  if (path == NULL || *path == '\0' || strcmp(path, "/") == 0)
    {
      inode   = g_root_inode;
      isroot  = true;
    }
  else
    {
      /* We don't know what to do with relative paths */

      if (*path != '/')
        {
          ret = ENOTDIR;
          goto errout_with_semaphore;
        }

      /* Find the node matching the path. */

      ret = inode_search(&desc);
      if (ret >= 0)
        {
          inode   = desc.node;
          DEBUGASSERT(inode != NULL);
#ifndef CONFIG_DISABLE_MOUNTPOINT
          relpath = desc.relpath;
#endif
        }
    }

  /* Did we get an inode? */

  if (inode == NULL)
    {
      /* Inode for 'path' does not exist. */

      ret = ENOTDIR;
      goto errout_with_semaphore;
    }

  /* Allocate a type DIR -- which is little more than an inode
   * container.
   */

  dir = (FAR struct fs_dirent_s *)kumm_zalloc(sizeof(struct fs_dirent_s));
  if (!dir)
    {
      /* Insufficient memory to complete the operation. */

      ret = ENOMEM;
      goto errout_with_semaphore;
    }

  /* Populate the DIR structure and return it to the caller.  The way that
   * we do this depends on whenever this is a "normal" pseudo-file-system
   * inode or a file system mountpoint.
   */

  dir->fd_position = 0;      /* This is the position in the read stream */

  /* First, handle the special case of the root inode.  This must be
   * special-cased here because the root inode might ALSO be a mountpoint.
   */

  if (isroot)
    {
      /* Whatever payload the root inode carries, the root inode is always
       * a directory inode in the pseudo-file system
       */

      open_pseudodir(inode, dir);
    }

  /* Is this a node in the pseudo filesystem? Or a mountpoint?  If the node
   * is the root (isroot == TRUE), then this is a special case.
   */

#ifndef CONFIG_DISABLE_MOUNTPOINT
  else if (INODE_IS_MOUNTPT(inode))
    {
      /* Yes, the node is a file system mountpoint */

      dir->fd_root = inode;  /* Save the inode where we start */

      /* Open the directory at the relative path */

      ret = open_mountpoint(inode, relpath, dir);
      if (ret != OK)
        {
          goto errout_with_direntry;
        }
    }
#endif
  else
    {
      /* The node is part of the root pseudo file system.  Does the inode
       * have a child? If so that the child would be the 'root' of a list
       * of nodes under the directory.
       */

      FAR struct inode *child = inode->i_child;
      if (child != NULL)
        {
          /* It looks we have a valid pseudo-filesystem directory node. */

          open_pseudodir(child, dir);
        }
      else if (!inode->u.i_ops)
        {
          /* This is a dangling node with no children and no operations. Set
           * up to enumerate an empty directory.
           */

          open_emptydir(dir);
        }
      else
        {
          ret = ENOTDIR;
          goto errout_with_direntry;
        }
    }

  RELEASE_SEARCH(&desc);
  inode_semgive();

  /* Free any allocated string memory */

  if (alloc != NULL)
    {
      kmm_free(alloc);
    }

  return ((FAR DIR *)dir);

  /* Nasty goto's make error handling simpler */

errout_with_direntry:
  kumm_free(dir);

errout_with_semaphore:
  RELEASE_SEARCH(&desc);
  inode_semgive();

errout_with_alloc:

  /* Free any allocated string memory */

  if (alloc != NULL)
    {
      kmm_free(alloc);
    }

errout:
  set_errno(ret);
  return NULL;
}
