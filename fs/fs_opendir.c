/****************************************************************************
 * fs/fs_opendir.c
 *
 *   Copyright (C) 2007-2009, 2011 Gregory Nutt. All rights reserved.
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

#include <stdlib.h>
#include <stdbool.h>
#include <dirent.h>
#include <string.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/fs.h>
#include <nuttx/dirent.h>

#include "fs_internal.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: opendir
 *
 * Description:
 *   The  opendir() function opens a directory stream
 *   corresponding to the directory name, and returns a
 *   pointer to the directory stream. The stream is
 *   positioned at the first entry in the directory.
 *
 * Inputs:
 *   path -- the directory to open
 *
 * Return:
 *   The opendir() function returns a pointer to the
 *   directory stream.  On error, NULL is returned, and
 *   errno is set appropriately.
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
  FAR const char *relpath;
  bool isroot = false;
  int ret;

  /* If we are given 'nothing' then we will interpret this as
   * request for the root inode.
   */

  inode_semtake();
  if (!path || *path == 0 || strcmp(path, "/") == 0)
    {
       inode = root_inode;
       isroot = true;
    }
  else
    {
      /* We don't know what to do with relative pathes */

       if (*path != '/')
        {
          ret = -ENOTDIR;
          goto errout_with_semaphore;
        }

      /* Find the node matching the path. */

      inode = inode_search(&path, (FAR struct inode**)NULL, (FAR struct inode**)NULL, &relpath);
    }

  /* Did we get an inode? */

  if (!inode)
    {
      /* 'path' is not a does not exist.*/

      ret = ENOTDIR;
      goto errout_with_semaphore;
    }

  /* Allocate a type DIR -- which is little more than an inode
   * container.
   */

  dir = (FAR struct fs_dirent_s *)zalloc(sizeof(struct fs_dirent_s));
  if (!dir)
    {
      /* Insufficient memory to complete the operation.*/

      ret = ENOMEM;
      goto errout_with_semaphore;
    }

  /* Populate the DIR structure and return it to the caller.  The way that
   * we do this depends on whenever this is a "normal" psuedo-file-system
   * inode or a file system mountpoint.
   */

  dir->fd_root     = inode;  /* Save the inode where we start */
  dir->fd_position = 0;      /* This is the position in the read stream */

  /* Is this a node in the psuedo filesystem? Or a mountpoint? */

#ifndef CONFIG_DISABLE_MOUNTPOINT
   if (INODE_IS_MOUNTPT(inode))
     {
       /* Yes, then return the inode itself as the 'root' of
        * the directory.  The actually directory is at relpath into the
        * mounted filesystem.
        */

      /* The node is a file system mointpoint. Verify that the mountpoint
       * supports the opendir() method
       */

      if (!inode->u.i_mops || !inode->u.i_mops->opendir)
        {
           ret = ENOSYS;
           goto errout_with_direntry;
        }

      /* Take reference to the mountpoint inode (fd_root).  Note that we do
       * not use inode_addref() because we already hold the tree semaphore.
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

          ret = -ret;
          goto errout_with_direntry;
        }
    }
  else
#endif
    {
      /* The node is part of the root psuedo file system.  Does the inode have a child?
       * If so that the child would be the 'root' of a list of nodes under
       * the directory.
       */

      if (!isroot)
        {
          inode = inode->i_child;
          if (!inode)
            {
              ret = ENOTDIR;
              goto errout_with_direntry;
            }
        }

      /* It looks we have a valid psuedo-filesystem node.  Take two references
       * on the inode -- one for the parent (fd_root) and one for the child (fd_next).
       * Note that we do not call inode_addref because we are holding
       * the tree semaphore and that would result in deadlock.
       */

      inode->i_crefs += 2;
      dir->u.psuedo.fd_next = inode; /* This is the next node to use for readdir() */

      /* Flag the inode as belonging to the psuedo-filesystem */
#ifndef CONFIG_DISABLE_MOUNTPOINT
      DIRENT_SETPSUEDONODE(dir->fd_flags);
#endif
    }

  inode_semgive();
  return ((DIR*)dir);

  /* Nasty goto's make error handling simpler */

errout_with_direntry:
  free(dir);

errout_with_semaphore:
  inode_semgive();
  *get_errno_ptr() = ret;
  return NULL;
}

