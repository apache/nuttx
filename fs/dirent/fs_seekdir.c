/****************************************************************************
 * fs/dirent/fs_seekdir.c
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

#include <sys/types.h>
#include <dirent.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/fs/fs.h>
#include <nuttx/fs/dirent.h>

#include "inode/inode.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: seekpseudodir
 ****************************************************************************/

static inline void seekpseudodir(struct fs_dirent_s *idir, off_t offset)
{
  struct inode *curr;
  struct inode *prev;
  off_t pos;
  int ret;

  /* Determine a starting point for the seek.  If the seek
   * is "forward" from the current position, then we will
   * start at the current position.  Otherwise, we will
   * "rewind" to the root dir.
   */

  if (offset < idir->fd_position)
    {
      pos  = 0;
      curr = idir->fd_root;
    }
  else
    {
      pos  = idir->fd_position;
      curr = idir->u.pseudo.fd_next;
    }

  /* Traverse the peer list starting at the 'root' of the
   * the list until we find the node at 'offset".  If devices
   * are being registered and unregistered, then this can
   * be a very unpredictable operation.
   */

  ret = inode_semtake();
  if (ret < 0)
    {
      ferr("ERROR:  inode_semtake failed: %d\n", ret);
      return;
    }

  for (; curr && pos != offset; pos++, curr = curr->i_peer);

  /* Now get the inode to vist next time that readdir() is called */

  prev                   = idir->u.pseudo.fd_next;
  idir->u.pseudo.fd_next = curr; /* The next node to visit (might be null) */
  idir->fd_position      = pos;  /* Might be beyond the last dirent */

  if (curr)
    {
      /* Increment the reference count on this next node */

      curr->i_crefs++;
    }

  inode_semgive();

  if (prev)
    {
      inode_release(prev);
    }
}

/****************************************************************************
 * Name: seekmountptdir
 ****************************************************************************/

#ifndef CONFIG_DISABLE_MOUNTPOINT
static inline void seekmountptdir(struct fs_dirent_s *idir, off_t offset)
{
  struct inode *inode;
  off_t pos;

  /* Determine a starting point for the seek.  If the seek
   * is "forward" from the current position, then we will
   * start at the current position.  Otherwise, we will
   * "rewind" to the root dir.
   */

  inode = idir->fd_root;
  if (offset < idir->fd_position)
    {
      if (inode->u.i_mops && inode->u.i_mops->rewinddir)
        {
          /* Perform the rewinddir() operation */

          inode->u.i_mops->rewinddir(inode, idir);
          pos = 0;
        }
      else
        {
          /* We can't do the seek and there is no way to return
           * an error indication.
           */

          return;
        }
    }
  else
    {
      pos = idir->fd_position;
    }

  /* This is a brute force approach... we will just read
   * directory entries until we are at the desired position.
   */

  while (pos < offset)
    {
      if (!inode->u.i_mops || !inode->u.i_mops->readdir ||
           inode->u.i_mops->readdir(inode, idir) < 0)
        {
          /* We can't read the next entry and there is no way to return
           * an error indication.
           */

           return;
        }

      /* Increment the position on each successful read */

      pos++;
    }

  /* If we get here the directory position has been successfully set */

  idir->fd_position = pos;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: seekdir
 *
 * Description:
 *   The seekdir() function sets the location in the directory stream from
 *   which the next readdir() call will start.  seekdir() should be used with
 *   an offset returned by telldir().
 *
 * Input Parameters:
 *   dirp -- An instance of type DIR created by a previous
 *     call to opendir();
 *   offset -- offset to seek to
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void seekdir(FAR DIR *dirp, off_t offset)
{
  struct fs_dirent_s *idir = (struct fs_dirent_s *)dirp;

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
  if (INODE_IS_MOUNTPT(idir->fd_root))
    {
      /* The node is a file system mointpoint */

      seekmountptdir(idir, offset);
    }
  else
#endif
    {
      /* The node is part of the root pseudo file system */

      seekpseudodir(idir, offset);
    }
}
