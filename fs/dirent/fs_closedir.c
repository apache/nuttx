/****************************************************************************
 * fs/dirent/fs_closedir.c
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

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/dirent.h>

#include "inode/inode.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: closedir
 *
 * Description:
 *    The closedir() function closes the directory stream associated with
 *    'dirp'.  The directory stream descriptor 'dirp' is not available after
 *    this call.
 *
 * Input Parameters:
 *   dirp -- An instance of type DIR created by a previous call to opendir();
 *
 * Returned Value:
 *   The closedir() function returns 0 on success.  On error, -1 is
 *   returned, and errno is set appropriately.
 *
 ****************************************************************************/

int closedir(FAR DIR *dirp)
{
  struct fs_dirent_s *idir = (struct fs_dirent_s *)dirp;
#ifndef CONFIG_DISABLE_MOUNTPOINT
  struct inode *inode;
#endif
  int ret;

  /* Verify that we were provided with a valid directory structure */

  if (!idir)
    {
      ret = EBADF;
      goto errout;
    }

  /* A special case is when we enumerate an "empty", unused inode.
   * That is an inode in the pseudo-filesystem that has no operations
   * and no children.
   * This is a "dangling" directory entry that has lost its childre.
   */

  if (idir->fd_root)
    {
      /* This is the 'root' inode of the directory.  This means different
       * things with different filesystems.
       */

#ifndef CONFIG_DISABLE_MOUNTPOINT
      inode = idir->fd_root;

      /* The way that we handle the close operation depends on what kind of
       * root inode we have open.
       */

      if (INODE_IS_MOUNTPT(inode) && !DIRENT_ISPSEUDONODE(idir->fd_flags))
        {
          /* The node is a file system mointpoint. Verify that the
           * mountpoint supports the closedir() method (not an error if it
           * does not)
           */

          if (inode->u.i_mops && inode->u.i_mops->closedir)
            {
              /* Perform the closedir() operation */

              ret = inode->u.i_mops->closedir(inode, idir);
              if (ret < 0)
                {
                  ret = -ret;
                  goto errout_with_inode;
                }
            }
        }
      else
#endif
        {
          /* The node is part of the root pseudo file system, release
           * our contained reference to the 'next' inode.
           */

          if (idir->u.pseudo.fd_next)
            {
              inode_release(idir->u.pseudo.fd_next);
            }
        }

      /* Release our references on the contained 'root' inode */

      inode_release(idir->fd_root);
    }

  /* Then release the container */

  kumm_free(idir);
  return OK;

#ifndef CONFIG_DISABLE_MOUNTPOINT
errout_with_inode:
  inode_release(inode);
  kumm_free(idir);
#endif

errout:
  set_errno(ret);
  return ERROR;
}
