/****************************************************************************
 * fs/vfs/fs_dir.c
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
#include <errno.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/dirent.h>

#include "inode/inode.h"

/****************************************************************************
 * Private Functions Prototypes
 ****************************************************************************/

static int     dir_close(FAR struct file *filep);
static ssize_t dir_read(FAR struct file *filep, FAR char *buffer,
                        size_t buflen);
static off_t   dir_seek(FAR struct file *filep, off_t offset, int whence);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_dir_fileops =
{
  NULL,       /* open */
  dir_close,  /* close */
  dir_read,   /* read */
  NULL,       /* write */
  dir_seek,   /* seek */
  NULL,       /* ioctl */
  NULL,       /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  NULL,       /* unlink */
#endif
};

static struct inode g_dir_inode =
{
  NULL,
  NULL,
  NULL,
  1,
  0,
  { &g_dir_fileops },
};

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
 *   On success, 0 is returned; Otherwise, a negative errno is returned.
 *
 ****************************************************************************/

#ifndef CONFIG_DISABLE_MOUNTPOINT
static int open_mountpoint(FAR struct inode *inode, FAR const char *relpath,
                           FAR struct fs_dirent_s *dir)
{
  /* The inode itself as the 'root' of mounted volume. The actually
   * directory is at relpath into the mounted filesystem.
   *
   * Verify that the mountpoint inode supports the opendir() method.
   */

  if (inode->u.i_mops == NULL || inode->u.i_mops->opendir == NULL ||
      inode->u.i_mops->readdir == NULL)
    {
      return -ENOSYS;
    }

  dir->fd_root = inode;

  return inode->u.i_mops->opendir(inode, relpath, dir);
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
  dir->fd_root          = inode;          /* Save the inode where we start */
  dir->u.pseudo.fd_next = inode->i_child; /* The next node for readdir */
  inode_addref(inode->i_child);
}

/****************************************************************************
 * Name: seek_pseudodir
 ****************************************************************************/

static off_t seek_pseudodir(FAR struct file *filep, off_t offset)
{
  FAR struct fs_dirent_s *dir = filep->f_priv;
  FAR struct inode *curr;
  FAR struct inode *prev;
  off_t pos;

  /* Determine a starting point for the seek. If the seek
   * is "forward" from the current position, then we will
   * start at the current position. Otherwise, we will
   * "rewind" to the root dir.
   */

  if (offset < filep->f_pos)
    {
      pos  = 0;
      curr = dir->fd_root->i_child;
    }
  else
    {
      pos  = filep->f_pos;
      curr = dir->u.pseudo.fd_next;
    }

  /* Traverse the peer list starting at the 'root' of the
   * the list until we find the node at 'offset". If devices
   * are being registered and unregistered, then this can
   * be a very unpredictable operation.
   */

  inode_semtake();

  for (; curr != NULL && pos != offset; pos++, curr = curr->i_peer);

  /* Now get the inode to vist next time that readdir() is called */

  prev = dir->u.pseudo.fd_next;

  /* The next node to visit (might be null) */

  dir->u.pseudo.fd_next = curr;
  if (curr != NULL)
    {
      /* Increment the reference count on this next node */

      curr->i_crefs++;
    }

  inode_semgive();

  if (prev != NULL)
    {
      inode_release(prev);
    }

  return pos;
}

/****************************************************************************
 * Name: seek_mountptdir
 ****************************************************************************/

#ifndef CONFIG_DISABLE_MOUNTPOINT
static off_t seek_mountptdir(FAR struct file *filep, off_t offset)
{
  FAR struct fs_dirent_s *dir = filep->f_priv;
  FAR struct inode *inode = dir->fd_root;
  off_t pos;

  /* Determine a starting point for the seek. If the seek
   * is "forward" from the current position, then we will
   * start at the current position. Otherwise, we will
   * "rewind" to the root dir.
   */

  if (offset < filep->f_pos)
    {
      if (inode->u.i_mops->rewinddir != NULL)
        {
          inode->u.i_mops->rewinddir(inode, dir);
          pos = 0;
        }
      else
        {
          return -ENOTSUP;
        }
    }
  else
    {
      pos = filep->f_pos;
    }

  /* This is a brute force approach... we will just read
   * directory entries until we are at the desired position.
   */

  while (pos < offset)
    {
      int ret;

      ret = inode->u.i_mops->readdir(inode, dir);
      if (ret < 0)
        {
          return ret;
        }

      /* Increment the position on each successful read */

      pos++;
    }

  return pos;
}
#endif

/****************************************************************************
 * Name: read_pseudodir
 ****************************************************************************/

static int read_pseudodir(FAR struct fs_dirent_s *dir)
{
  FAR struct inode *prev;

  /* Check if we are at the end of the list */

  if (dir->u.pseudo.fd_next == NULL)
    {
      /* End of file and error conditions are not distinguishable with
       * readdir. Here we return -ENOENT to signal the end of the directory.
       */

      return -ENOENT;
    }

  /* Copy the inode name into the dirent structure */

  strlcpy(dir->fd_dir.d_name, dir->u.pseudo.fd_next->i_name,
          sizeof(dir->fd_dir.d_name));

  /* If the node has file operations, we will say that it is a file. */

  dir->fd_dir.d_type = DTYPE_UNKNOWN;
  if (dir->u.pseudo.fd_next->u.i_ops != NULL)
    {
#ifndef CONFIG_DISABLE_MOUNTPOINT
      if (INODE_IS_BLOCK(dir->u.pseudo.fd_next))
        {
           dir->fd_dir.d_type = DTYPE_BLK;
        }
      else if (INODE_IS_MTD(dir->u.pseudo.fd_next))
        {
           dir->fd_dir.d_type = DTYPE_MTD;
        }
      else if (INODE_IS_MOUNTPT(dir->u.pseudo.fd_next))
        {
           dir->fd_dir.d_type = DTYPE_DIRECTORY;
        }
      else
#endif
#ifdef CONFIG_PSEUDOFS_SOFTLINKS
      if (INODE_IS_SOFTLINK(dir->u.pseudo.fd_next))
        {
           dir->fd_dir.d_type = DTYPE_LINK;
        }
      else
#endif
      if (INODE_IS_DRIVER(dir->u.pseudo.fd_next))
        {
           dir->fd_dir.d_type = DTYPE_CHR;
        }
      else if (INODE_IS_NAMEDSEM(dir->u.pseudo.fd_next))
        {
           dir->fd_dir.d_type = DTYPE_SEM;
        }
      else if (INODE_IS_MQUEUE(dir->u.pseudo.fd_next))
        {
           dir->fd_dir.d_type = DTYPE_MQ;
        }
      else if (INODE_IS_SHM(dir->u.pseudo.fd_next))
        {
           dir->fd_dir.d_type = DTYPE_SHM;
        }
    }

  /* If the node has child node(s) or no operations, then we will say that
   * it is a directory rather than a special file. NOTE: that the node can
   * be both!
   */

  if (dir->u.pseudo.fd_next->i_child != NULL ||
      dir->u.pseudo.fd_next->u.i_ops == NULL)
    {
      dir->fd_dir.d_type = DTYPE_DIRECTORY;
    }

  /* Now get the inode to visit next time that readdir() is called */

  inode_semtake();

  prev                  = dir->u.pseudo.fd_next;
  dir->u.pseudo.fd_next = prev->i_peer; /* The next node to visit */

  if (dir->u.pseudo.fd_next != NULL)
    {
      /* Increment the reference count on this next node */

      dir->u.pseudo.fd_next->i_crefs++;
    }

  inode_semgive();

  if (prev != NULL)
    {
      inode_release(prev);
    }

  return 0;
}

static int dir_close(FAR struct file *filep)
{
  FAR struct fs_dirent_s *dir = filep->f_priv;
  FAR struct inode *inode = dir->fd_root;
  int ret = 0;

  /* This is the 'root' inode of the directory. This means different
   * things with different filesystems.
   */

#ifndef CONFIG_DISABLE_MOUNTPOINT
  /* The way that we handle the close operation depends on what kind of
   * root inode we have open.
   */

  if (INODE_IS_MOUNTPT(inode))
    {
      /* The node is a file system mointpoint. Verify that the
       * mountpoint supports the closedir() method (not an error if it
       * does not)
       */

      if (inode->u.i_mops->closedir != NULL)
        {
          ret = inode->u.i_mops->closedir(inode, dir);
        }
    }
  else
#endif
    {
      /* The node is part of the root pseudo file system, release
       * our contained reference to the 'next' inode.
       */

      if (dir->u.pseudo.fd_next != NULL)
        {
          inode_release(dir->u.pseudo.fd_next);
        }

      /* Then release the container */

      kmm_free(dir);
    }

  /* Release our references on the contained 'root' inode */

  inode_release(inode);
  return ret;
}

static ssize_t dir_read(FAR struct file *filep, FAR char *buffer,
                        size_t buflen)
{
  FAR struct fs_dirent_s *dir = filep->f_priv;
#ifndef CONFIG_DISABLE_MOUNTPOINT
  FAR struct inode *inode = dir->fd_root;
#endif
  int ret;

  /* Verify that we were provided with a valid directory structure */

  if (buffer == NULL || buflen < sizeof(struct dirent))
    {
      return -EINVAL;
    }

  /* The way we handle the readdir depends on the type of inode
   * that we are dealing with.
   */

#ifndef CONFIG_DISABLE_MOUNTPOINT
  if (INODE_IS_MOUNTPT(inode))
    {
      ret = inode->u.i_mops->readdir(inode, dir);
    }
  else
#endif
    {
      /* The node is part of the root pseudo file system */

      ret = read_pseudodir(dir);
    }

  /* ret < 0 is an error. Special case: ret = -ENOENT is end of file */

  if (ret < 0)
    {
      if (ret == -ENOENT)
        {
          ret = 0;
        }

      return ret;
    }

  filep->f_pos++;
  memcpy(buffer, &dir->fd_dir, sizeof(dir->fd_dir));
  return sizeof(dir->fd_dir);
}

static off_t dir_seek(FAR struct file *filep, off_t offset, int whence)
{
  off_t pos = 0;

  /* The way we handle the readdir depends on the type of inode
   * that we are dealing with.
   */

  if (whence == SEEK_SET)
    {
#ifndef CONFIG_DISABLE_MOUNTPOINT
      FAR struct fs_dirent_s *dir = filep->f_priv;

      if (INODE_IS_MOUNTPT(dir->fd_root))
        {
          pos = seek_mountptdir(filep, offset);
        }
      else
#endif
        {
          pos = seek_pseudodir(filep, offset);
        }
    }
  else if (whence == SEEK_CUR)
    {
      return filep->f_pos;
    }
  else
    {
      return -EINVAL;
    }

  if (pos >= 0)
    {
      filep->f_pos = pos;
    }

  return pos;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: dir_allocate
 *
 * Description:
 *   Allocate a directory instance and bind it to f_priv of filep.
 *
 ****************************************************************************/

int dir_allocate(FAR struct file *filep, FAR const char *relpath)
{
  FAR struct fs_dirent_s *dir;
  FAR struct inode *inode = filep->f_inode;
  int ret = 0;

  dir = kmm_zalloc(sizeof(struct fs_dirent_s));
  if (dir == NULL)
    {
      return -ENOMEM;
    }

  /* Is this a node in the pseudo filesystem? Or a mountpoint? */

#ifndef CONFIG_DISABLE_MOUNTPOINT
  if (INODE_IS_MOUNTPT(inode))
    {
      /* Open the directory at the relative path */

      ret = open_mountpoint(inode, relpath, dir);
      if (ret < 0)
        {
          goto errout_with_direntry;
        }
    }
  else
#endif
    {
      open_pseudodir(inode, dir);
    }

  filep->f_inode  = &g_dir_inode;
  filep->f_priv   = dir;
  inode_addref(&g_dir_inode);

  return ret;

errout_with_direntry:
  kmm_free(dir);
  return ret;
}
