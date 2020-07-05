/****************************************************************************
 * fs/dirent/fs_readdir.c
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

#include <string.h>
#include <dirent.h>
#include <errno.h>

#include <nuttx/fs/fs.h>
#include <nuttx/fs/dirent.h>

#include "inode/inode.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: readpseudodir
 ****************************************************************************/

static inline int readpseudodir(struct fs_dirent_s *idir)
{
  FAR struct inode *prev;
  int ret;

  /* Check if we are at the end of the list */

  if (!idir->u.pseudo.fd_next)
    {
      /* End of file and error conditions are not distinguishable with
       * readdir.  Here we return -ENOENT to signal the end of the
       * directory.
       */

      return -ENOENT;
    }

  /* Copy the inode name into the dirent structure */

  strncpy(idir->fd_dir.d_name, idir->u.pseudo.fd_next->i_name,
          NAME_MAX + 1);

  /* If the node has file operations, we will say that it is a file. */

  idir->fd_dir.d_type = DTYPE_UNKNOWN;
  if (idir->u.pseudo.fd_next->u.i_ops)
    {
#ifndef CONFIG_DISABLE_MOUNTPOINT
      if (INODE_IS_BLOCK(idir->u.pseudo.fd_next))
        {
           idir->fd_dir.d_type = DTYPE_BLK;
        }
      else if (INODE_IS_MTD(idir->u.pseudo.fd_next))
        {
           idir->fd_dir.d_type = DTYPE_MTD;
        }
      else if (INODE_IS_MOUNTPT(idir->u.pseudo.fd_next))
        {
           idir->fd_dir.d_type = DTYPE_DIRECTORY;
        }
      else
#endif
#ifdef CONFIG_PSEUDOFS_SOFTLINKS
      if (INODE_IS_SOFTLINK(idir->u.pseudo.fd_next))
        {
           idir->fd_dir.d_type = DTYPE_LINK;
        }
      else
#endif
      if (INODE_IS_DRIVER(idir->u.pseudo.fd_next))
        {
           idir->fd_dir.d_type = DTYPE_CHR;
        }
      else if (INODE_IS_NAMEDSEM(idir->u.pseudo.fd_next))
        {
           idir->fd_dir.d_type = DTYPE_SEM;
        }
      else if (INODE_IS_MQUEUE(idir->u.pseudo.fd_next))
        {
           idir->fd_dir.d_type = DTYPE_MQ;
        }
      else if (INODE_IS_SHM(idir->u.pseudo.fd_next))
        {
           idir->fd_dir.d_type = DTYPE_SHM;
        }
    }

  /* If the node has child node(s) or no operations, then we will say that
   * it is a directory rather than a special file.  NOTE: that the node can
   * be both!
   */

  if (idir->u.pseudo.fd_next->i_child || !idir->u.pseudo.fd_next->u.i_ops)
    {
      idir->fd_dir.d_type = DTYPE_DIRECTORY;
    }

  /* Now get the inode to visit next time that readdir() is called */

  ret = inode_semtake();
  if (ret < 0)
    {
      return ret;
    }

  prev                   = idir->u.pseudo.fd_next;
  idir->u.pseudo.fd_next = prev->i_peer; /* The next node to visit */

  if (idir->u.pseudo.fd_next)
    {
      /* Increment the reference count on this next node */

      idir->u.pseudo.fd_next->i_crefs++;
    }

  inode_semgive();

  if (prev)
    {
      inode_release(prev);
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: readdir
 *
 * Description:
 *   The readdir() function returns a pointer to a dirent structure
 *   representing the next directory entry in the directory stream pointed
 *   to by dir.  It returns NULL on reaching the end-of-file or if an error
 *   occurred.
 *
 * Input Parameters:
 *   dirp -- An instance of type DIR created by a previous call to opendir();
 *
 * Returned Value:
 *   The readdir() function returns a pointer to a dirent structure, or NULL
 *   if an error occurs or end-of-file is reached.  On error, errno is set
 *   appropriately.
 *
 *   EBADF   - Invalid directory stream descriptor dir
 *
 ****************************************************************************/

FAR struct dirent *readdir(DIR *dirp)
{
  FAR struct fs_dirent_s *idir = (struct fs_dirent_s *)dirp;
  struct inode *inode;
  int ret;

  /* Verify that we were provided with a valid directory structure */

  if (!idir)
    {
      ret = EBADF;
      goto errout;
    }

  /* A special case is when we enumerate an "empty", unused inode.  That is
   * an inode in the pseudo-filesystem that has no operations and no
   * children.  This is a "dangling" directory entry that has lost its
   * children.
   */

  inode = idir->fd_root;
  if (!inode)
    {
      /* End of file and error conditions are not distinguishable
       * with readdir.  We return NULL to signal either case.
       */

      ret = OK;
      goto errout;
    }

  /* The way we handle the readdir depends on the type of inode
   * that we are dealing with.
   */

#ifndef CONFIG_DISABLE_MOUNTPOINT
  if (INODE_IS_MOUNTPT(inode) && !DIRENT_ISPSEUDONODE(idir->fd_flags))
    {
      /* The node is a file system mointpoint. Verify that the mountpoint
       * supports the readdir() method
       */

      if (!inode->u.i_mops || !inode->u.i_mops->readdir)
        {
          ret = EACCES;
          goto errout;
        }

      /* Perform the readdir() operation */

      ret = inode->u.i_mops->readdir(inode, idir);
    }
  else
#endif
    {
      /* The node is part of the root pseudo file system */

      ret = readpseudodir(idir);
    }

  /* ret < 0 is an error.  Special case: ret = -ENOENT is end of file */

  if (ret < 0)
    {
      if (ret == -ENOENT)
        {
          ret = OK;
        }
      else
        {
          ret = -ret;
        }

      goto errout;
    }

  /* Success */

  idir->fd_position++;
  return &idir->fd_dir;

errout:
  set_errno(ret);
  return NULL;
}
