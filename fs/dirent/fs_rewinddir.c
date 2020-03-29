/****************************************************************************
 * fs/dirent/fs_rewinddir.c
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

#include <dirent.h>
#include <errno.h>

#include <nuttx/fs/fs.h>
#include <nuttx/fs/dirent.h>

#include "inode/inode.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rewindpseudodir
 ****************************************************************************/

static inline void rewindpseudodir(struct fs_dirent_s *idir)
{
  struct inode *prev;
  int ret;

  ret = inode_semtake();
  if (ret < 0)
    {
      return;
    }

  /* Reset the position to the beginning */

  prev                   = idir->u.pseudo.fd_next; /* (Save to delete later) */
  idir->u.pseudo.fd_next = idir->fd_root;          /* The next node to visit */
  idir->fd_position      = 0;                      /* Reset position */

  /* Increment the reference count on the root=next node.  We
   * should now have two references on the inode.
   */

  idir->fd_root->i_crefs++;
  inode_semgive();

  /* Then release the reference to the old next inode */

  if (prev)
    {
      inode_release(prev);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rewinddir
 *
 * Description:
 *   The  rewinddir() function resets the position of the
 *   directory stream dir to the beginning of the directory.
 *
 * Input Parameters:
 *   dirp -- An instance of type DIR created by a previous
 *     call to opendir();
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void rewinddir(FAR DIR *dirp)
{
  struct fs_dirent_s *idir = (struct fs_dirent_s *)dirp;
#ifndef CONFIG_DISABLE_MOUNTPOINT
  struct inode *inode;
#endif

  /* Verify that we were provided with a valid directory structure,
   * A special case is when we enumerate an "empty", unused inode (fd_root
   * == 0).  That is an inode in the pseudo-filesystem that has no
   * operations and no children.  This is a "dangling" directory entry that
   * has lost its children.
   */

  if (!idir || !idir->fd_root)
    {
      return;
    }

  /* The way we handle the readdir depends on the type of inode
   * that we are dealing with.
   */

#ifndef CONFIG_DISABLE_MOUNTPOINT
  inode = idir->fd_root;
  if (INODE_IS_MOUNTPT(inode))
    {
      /* The node is a file system mointpoint. Verify that the mountpoint
       * supports the rewinddir() method
       */

      if (inode->u.i_mops && inode->u.i_mops->rewinddir)
        {
          /* Perform the rewinddir() operation */

          inode->u.i_mops->rewinddir(inode, idir);
        }
    }
  else
#endif
    {
      /* The node is part of the root pseudo file system */

      rewindpseudodir(idir);
    }
}
