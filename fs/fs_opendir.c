/************************************************************
 * fs_opendir.c
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
#include <dirent.h>
#include <string.h>
#include <errno.h>
#include <nuttx/fs.h>
#include "fs_internal.h"

/************************************************************
 * Private Functions
 ************************************************************/

/************************************************************
 * Name: fs_finddirnode
 *
 * Description:
 *   This is called from the opendir() logic to get a reference
 *   to the inode associated with a directory.  There are no
 *   real directories in this design; For our purposes, a
 *   directory inode is simply one that has children or one
 *   that is a mountpoint for a "real" file system
 *
 ************************************************************/

static inline FAR struct inode *fs_finddirnode(const char *path, const char **relpath)
{
  FAR struct inode *node;
  FAR struct inode *root = NULL;

  /* If we are given 'nothing' then we will interpret this as
   * request for the root inode.
   */

  if (!path || *path == 0 || strcmp(path, "/") == 0)
    {
       return root_inode;
    }

  /* We don't know what to do with relative pathes */

  if (*path != '/')
    {
      return NULL;
    }

  /* Find the node matching the path. */

  inode_semtake();
  node = inode_search(&path, (FAR void*)NULL, (FAR void*)NULL, relpath);
  if (node)
    {

      /* Is this a not in the psuedo filesystem? */

      if (INODE_IS_MOUNTPT(node))
        {
           /* Yes, then return the inode itself as the 'root' of
           * the directory.  The actually directory is at relpath into the
           * mounted filesystem.  Increment the count of references
           * on the inode.
           */

           root = node;
           root->i_crefs++;
        }
      else
        {
          /* It is a node in the psuedo filesystem.  Does the inode have a child?
           * If so that the child would be the 'root' of a list of nodes under
           * the directory.
           */

          root = node->i_child;
          if (root)
            {
              /* If found, then  increment the count of
               * references on the child node.
               */

              root->i_crefs++;
            }
        }
    }
  inode_semgive();
  return root;
}

/************************************************************
 * Name: fs_openmountptdir
 ************************************************************/

static inline int fs_openmountptdir(struct inode *inode, const char *relpath,
                                     struct internal_dir_s *dir)
{
#warning "Mountpoint support not implemented"
  return -ENOSYS;
}

/************************************************************
 * Public Functions
 ************************************************************/

/************************************************************
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
 ************************************************************/

#if CONFIG_NFILE_DESCRIPTORS > 0

FAR DIR *opendir(const char *path)
{
  FAR struct inode *inode;
  FAR struct internal_dir_s *dir;
  const char *relpath;
  int ret;

  /* Get an inode corresponding to the path.  On successful
   * return, we will hold on reference count on the inode.
   */

  inode = fs_finddirnode(path, &relpath);
  if (!inode)
    {
      /* 'path' is not a directory.*/

      ret = ENOTDIR;
      goto errout;
    }

  /* Allocate a type DIR -- which is little more than an inode
   * container.
   */

  dir = (FAR struct internal_dir_s *)zalloc(sizeof(struct internal_dir_s));
  if (!dir)
    {
      /* Insufficient memory to complete the operation.*/

      ret = ENOMEM;
      goto errout_with_inode;
    }

  /* Populate the DIR structure and return it to the caller.  The way that
   * we do this depends on whenever this is a "normal" psuedo-file-system
   * inode or a file system mountpoint.
   */

  dir->root     = inode;  /* Save the inode where we start */
  dir->position = 0;      /* This is the position in the read stream */

  /* Is this a not in the psuedo filesystem? */

  if (INODE_IS_MOUNTPT(inode))
    {
      /* The node is a file system mointpoint */

      ret = fs_openmountptdir(inode, relpath, dir);
      if (ret < 0)
        {
          ret = -ret;
          goto errout_with_direntry;
        }
    }
  else
    {
      /* The node is part of the root psuedo file system */

      inode_addref(inode);         /* Now we have two references on inode */
      dir->u.psuedo.next = inode;  /* This is the next node to use for readdir() */
    }

  return ((DIR*)dir);

  /* Nasty goto's make error handling simpler */

errout_with_direntry:
  free(dir);
errout_with_inode:
  inode_release(inode);
errout:
  *get_errno_ptr() = ret;
  return NULL;
}

#endif /* CONFIG_NFILE_DESCRIPTORS */
