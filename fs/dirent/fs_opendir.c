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
      return -ENOSYS;
    }

  /* Perform the opendir() operation */

  ret = inode->u.i_mops->opendir(inode, relpath, dir);
  if (ret < 0)
    {
      /* Negate the error value so that it can be used to set errno */

      return ret;
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
   */

  inode_addref(inode);

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
  int ret;

  /* If we are given 'nothing' then we will interpret this as
   * request for the root inode.
   */

  SETUP_SEARCH(&desc, path, false);

  ret = inode_find(&desc);
  if (ret < 0)
    {
      goto errout_with_search;
    }

  /* Get the search results */

#ifndef CONFIG_DISABLE_MOUNTPOINT
  relpath = desc.relpath;
#endif

  inode = desc.node;
  DEBUGASSERT(inode != NULL);

  /* Allocate a type DIR -- which is little more than an inode
   * container.
   */

  dir = (FAR struct fs_dirent_s *)kumm_zalloc(sizeof(struct fs_dirent_s));
  if (!dir)
    {
      /* Insufficient memory to complete the operation. */

      ret = -ENOMEM;
      goto errout_with_inode;
    }

  /* Populate the DIR structure and return it to the caller.  The way that
   * we do this depends on whenever this is a "normal" pseudo-file-system
   * inode or a file system mountpoint.
   */

  dir->fd_position = 0;      /* This is the position in the read stream */

  /* Is this a node in the pseudo filesystem? Or a mountpoint? */

#ifndef CONFIG_DISABLE_MOUNTPOINT
  if (INODE_IS_MOUNTPT(inode))
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
  else
#endif
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
          ret = -ENOTDIR;
          goto errout_with_direntry;
        }
    }

  RELEASE_SEARCH(&desc);
  return ((FAR DIR *)dir);

  /* Nasty goto's make error handling simpler */

errout_with_direntry:
  kumm_free(dir);

errout_with_inode:
  inode_release(inode);

errout_with_search:
  RELEASE_SEARCH(&desc);
  set_errno(-ret);
  return NULL;
}
