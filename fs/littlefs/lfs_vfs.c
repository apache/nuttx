/****************************************************************************
 * fs/littlefs/lfs_vfs.c
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

#include <errno.h>
#include <fcntl.h>
#include <string.h>

#include <nuttx/fs/dirent.h>
#include <nuttx/fs/fs.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mtd/mtd.h>
#include <nuttx/semaphore.h>

#include <sys/stat.h>
#include <sys/statfs.h>

#include "littlefs/lfs.h"
#include "littlefs/lfs_util.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct littlefs_file_s
{
  struct lfs_file       file;
  int                   refs;
};

/* This structure represents the overall mountpoint state. An instance of
 * this structure is retained as inode private data on each mountpoint that
 * is mounted with a littlefs filesystem.
 */

struct littlefs_mountpt_s
{
  sem_t                 sem;
  FAR struct inode     *drv;
  struct mtd_geometry_s geo;
  struct lfs_config     cfg;
  struct lfs            lfs;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void    littlefs_semgive(FAR struct littlefs_mountpt_s *fs);
static int     littlefs_semtake(FAR struct littlefs_mountpt_s *fs);

static int     littlefs_open(FAR struct file *filep, FAR const char *relpath,
                             int oflags, mode_t mode);
static int     littlefs_close(FAR struct file *filep);
static ssize_t littlefs_read(FAR struct file *filep, FAR char *buffer,
                             size_t buflen);
static ssize_t littlefs_write(FAR struct file *filep, FAR const char *buffer,
                              size_t buflen);
static off_t   littlefs_seek(FAR struct file *filep, off_t offset,
                             int whence);
static int     littlefs_ioctl(FAR struct file *filep, int cmd,
                              unsigned long arg);

static int     littlefs_sync(FAR struct file *filep);
static int     littlefs_dup(FAR const struct file *oldp,
                            FAR struct file *newp);
static int     littlefs_fstat(FAR const struct file *filep,
                              FAR struct stat *buf);
static int     littlefs_truncate(FAR struct file *filep,
                                 off_t length);

static int     littlefs_opendir(FAR struct inode *mountpt,
                                FAR const char *relpath,
                                FAR struct fs_dirent_s *dir);
static int     littlefs_closedir(FAR struct inode *mountpt,
                                 FAR struct fs_dirent_s *dir);
static int     littlefs_readdir(FAR struct inode *mountpt,
                                FAR struct fs_dirent_s *dir);
static int     littlefs_rewinddir(FAR struct inode *mountpt,
                                  FAR struct fs_dirent_s *dir);

static int     littlefs_bind(FAR struct inode *driver,
                             FAR const void *data, FAR void **handle);
static int     littlefs_unbind(FAR void *handle, FAR struct inode **driver,
                               unsigned int flags);
static int     littlefs_statfs(FAR struct inode *mountpt,
                               FAR struct statfs *buf);

static int     littlefs_unlink(FAR struct inode *mountpt,
                               FAR const char *relpath);
static int     littlefs_mkdir(FAR struct inode *mountpt,
                              FAR const char *relpath, mode_t mode);
static int     littlefs_rmdir(FAR struct inode *mountpt,
                              FAR const char *relpath);
static int     littlefs_rename(FAR struct inode *mountpt,
                               FAR const char *oldrelpath,
                               FAR const char *newrelpath);
static int     littlefs_stat(FAR struct inode *mountpt,
                             FAR const char *relpath, FAR struct stat *buf);

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* See fs_mount.c -- this structure is explicitly extern'ed there.
 * We use the old-fashioned kind of initializers so that this will compile
 * with any compiler.
 */

const struct mountpt_operations littlefs_operations =
{
  littlefs_open,          /* open */
  littlefs_close,         /* close */
  littlefs_read,          /* read */
  littlefs_write,         /* write */
  littlefs_seek,          /* seek */
  littlefs_ioctl,         /* ioctl */

  littlefs_sync,          /* sync */
  littlefs_dup,           /* dup */
  littlefs_fstat,         /* fstat */
  littlefs_truncate,      /* truncate */

  littlefs_opendir,       /* opendir */
  littlefs_closedir,      /* closedir */
  littlefs_readdir,       /* readdir */
  littlefs_rewinddir,     /* rewinddir */

  littlefs_bind,          /* bind */
  littlefs_unbind,        /* unbind */
  littlefs_statfs,        /* statfs */

  littlefs_unlink,        /* unlink */
  littlefs_mkdir,         /* mkdir */
  littlefs_rmdir,         /* rmdir */
  littlefs_rename,        /* rename */
  littlefs_stat           /* stat */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: littlefs_semtake
 ****************************************************************************/

static int littlefs_semtake(FAR struct littlefs_mountpt_s *fs)
{
  return nxsem_wait_uninterruptible(&fs->sem);
}

/****************************************************************************
 * Name: littlefs_semgive
 ****************************************************************************/

static void littlefs_semgive(FAR struct littlefs_mountpt_s *fs)
{
  nxsem_post(&fs->sem);
}

/****************************************************************************
 * Name: littlefs_convert_oflags
 ****************************************************************************/

static int littlefs_convert_oflags(int oflags)
{
  int ret = 0;

  if ((oflags & O_RDONLY) != 0)
    {
      ret |= LFS_O_RDONLY;
    }

  if ((oflags & O_WRONLY) != 0)
    {
      ret |= LFS_O_WRONLY;
    }

  if ((oflags & O_CREAT) != 0)
    {
      ret |= LFS_O_CREAT;
    }

  if ((oflags & O_EXCL) != 0)
    {
      ret |= LFS_O_EXCL;
    }

  if ((oflags & O_APPEND) != 0)
    {
      ret |= LFS_O_APPEND;
    }

  if ((oflags & O_TRUNC) != 0)
    {
      ret |= LFS_O_TRUNC;
    }

  return ret;
}

/****************************************************************************
 * Name: littlefs_open
 ****************************************************************************/

static int littlefs_open(FAR struct file *filep, FAR const char *relpath,
                         int oflags, mode_t mode)
{
  FAR struct littlefs_mountpt_s *fs;
  FAR struct littlefs_file_s *priv;
  FAR struct inode *inode;
  int ret;

  /* Get the mountpoint inode reference from the file structure and the
   * mountpoint private data from the inode structure
   */

  inode = filep->f_inode;
  fs    = inode->i_private;

  /* Allocate memory for the open file */

  priv = kmm_malloc(sizeof(*priv));
  if (priv == NULL)
    {
      return -ENOMEM;
    }

  priv->refs = 1;

  /* Take the semaphore */

  ret = littlefs_semtake(fs);
  if (ret < 0)
    {
      goto errsem;
    }

  /* Try to open the file */

  oflags = littlefs_convert_oflags(oflags);
  ret = lfs_file_open(&fs->lfs, &priv->file, relpath, oflags);
  if (ret < 0)
    {
      /* Error opening file */

      goto errout;
    }

  /* In append mode, we need to set the file pointer to the end of the
   * file.
   */

  if (oflags & LFS_O_APPEND)
    {
      ret = lfs_file_seek(&fs->lfs, &priv->file, 0, LFS_SEEK_END);
      if (ret >= 0)
        {
          filep->f_pos = ret;
        }
      else
        {
          goto errout_with_file;
        }
    }

  /* Sync here in case of O_TRUNC haven't actually done immediately,
   * e.g. total 8M, fileA 6M, O_TRUNC re-wrting fileA 6M, meet error.
   */

  lfs_file_sync(&fs->lfs, &priv->file);
  littlefs_semgive(fs);

  /* Attach the private date to the struct file instance */

  filep->f_priv = priv;
  return OK;

errout_with_file:
  lfs_file_close(&fs->lfs, &priv->file);
errout:
  littlefs_semgive(fs);
errsem:
  kmm_free(priv);
  return ret;
}

/****************************************************************************
 * Name: littlefs_close
 ****************************************************************************/

static int littlefs_close(FAR struct file *filep)
{
  FAR struct littlefs_mountpt_s *fs;
  FAR struct littlefs_file_s *priv;
  FAR struct inode *inode;
  int ret;

  /* Recover our private data from the struct file instance */

  priv  = filep->f_priv;
  inode = filep->f_inode;
  fs    = inode->i_private;

  /* Close the file */

  ret = littlefs_semtake(fs);
  if (ret < 0)
    {
      return ret;
    }

  if (--priv->refs <= 0)
    {
      ret = lfs_file_close(&fs->lfs, &priv->file);
    }

  littlefs_semgive(fs);
  if (priv->refs <= 0)
    {
      kmm_free(priv);
    }

  return ret;
}

/****************************************************************************
 * Name: littlefs_read
 ****************************************************************************/

static ssize_t littlefs_read(FAR struct file *filep, FAR char *buffer,
                             size_t buflen)
{
  FAR struct littlefs_mountpt_s *fs;
  FAR struct littlefs_file_s *priv;
  FAR struct inode *inode;
  ssize_t ret;

  /* Recover our private data from the struct file instance */

  priv  = filep->f_priv;
  inode = filep->f_inode;
  fs    = inode->i_private;

  /* Call LFS to perform the read */

  ret = littlefs_semtake(fs);
  if (ret < 0)
    {
      return ret;
    }

  if (filep->f_pos != priv->file.pos)
    {
      ret = lfs_file_seek(&fs->lfs, &priv->file, filep->f_pos, LFS_SEEK_SET);
      if (ret < 0)
        {
          goto out;
        }
    }

  ret = lfs_file_read(&fs->lfs, &priv->file, buffer, buflen);
  if (ret > 0)
    {
      filep->f_pos += ret;
    }

out:
  littlefs_semgive(fs);
  return ret;
}

/****************************************************************************
 * Name: littlefs_write
 ****************************************************************************/

static ssize_t littlefs_write(FAR struct file *filep, const char *buffer,
                              size_t buflen)
{
  FAR struct littlefs_mountpt_s *fs;
  FAR struct littlefs_file_s *priv;
  FAR struct inode *inode;
  ssize_t ret;

  /* Recover our private data from the struct file instance */

  priv  = filep->f_priv;
  inode = filep->f_inode;
  fs    = inode->i_private;

  /* Call LFS to perform the write */

  ret = littlefs_semtake(fs);
  if (ret < 0)
    {
      return ret;
    }

  if (filep->f_pos != priv->file.pos)
    {
      ret = lfs_file_seek(&fs->lfs, &priv->file, filep->f_pos, LFS_SEEK_SET);
      if (ret < 0)
        {
          goto out;
        }
    }

  ret = lfs_file_write(&fs->lfs, &priv->file, buffer, buflen);
  if (ret > 0)
    {
      filep->f_pos += ret;
    }

out:
  littlefs_semgive(fs);
  return ret;
}

/****************************************************************************
 * Name: littlefs_seek
 ****************************************************************************/

static off_t littlefs_seek(FAR struct file *filep, off_t offset, int whence)
{
  FAR struct littlefs_mountpt_s *fs;
  FAR struct littlefs_file_s *priv;
  FAR struct inode *inode;
  off_t ret;

  /* Recover our private data from the struct file instance */

  priv  = filep->f_priv;
  inode = filep->f_inode;
  fs    = inode->i_private;

  /* Call LFS to perform the seek */

  ret = littlefs_semtake(fs);
  if (ret < 0)
    {
      return ret;
    }

  ret = lfs_file_seek(&fs->lfs, &priv->file, offset, whence);
  if (ret >= 0)
    {
      filep->f_pos = ret;
    }

  littlefs_semgive(fs);
  return ret;
}

/****************************************************************************
 * Name: littlefs_ioctl
 ****************************************************************************/

static int littlefs_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct littlefs_mountpt_s *fs;
  FAR struct inode *inode;
  FAR struct inode *drv;

  /* Recover our private data from the struct file instance */

  inode = filep->f_inode;
  fs    = inode->i_private;
  drv   = fs->drv;

  if (INODE_IS_MTD(drv))
    {
      return MTD_IOCTL(drv->u.i_mtd, cmd, arg);
    }
  else
    {
      return drv->u.i_bops->ioctl(drv, cmd, arg);
    }
}

/****************************************************************************
 * Name: littlefs_sync
 *
 * Description: Synchronize the file state on disk to match internal, in-
 *   memory state.
 *
 ****************************************************************************/

static int littlefs_sync(FAR struct file *filep)
{
  FAR struct littlefs_mountpt_s *fs;
  FAR struct littlefs_file_s *priv;
  FAR struct inode *inode;
  int ret;

  /* Recover our private data from the struct file instance */

  priv  = filep->f_priv;
  inode = filep->f_inode;
  fs    = inode->i_private;

  ret = littlefs_semtake(fs);
  if (ret < 0)
    {
      return ret;
    }

  ret = lfs_file_sync(&fs->lfs, &priv->file);
  littlefs_semgive(fs);

  return ret;
}

/****************************************************************************
 * Name: littlefs_dup
 *
 * Description: Duplicate open file data in the new file structure.
 *
 ****************************************************************************/

static int littlefs_dup(FAR const struct file *oldp, FAR struct file *newp)
{
  FAR struct littlefs_mountpt_s *fs;
  FAR struct littlefs_file_s *priv;
  FAR struct inode *inode;
  int ret;

  /* Recover our private data from the struct file instance */

  priv  = oldp->f_priv;
  inode = oldp->f_inode;
  fs    = inode->i_private;

  ret = littlefs_semtake(fs);
  if (ret < 0)
    {
      return ret;
    }

  priv->refs++;
  newp->f_priv = priv;
  littlefs_semgive(fs);

  return ret;
}

/****************************************************************************
 * Name: littlefs_fstat
 *
 * Description:
 *   Obtain information about an open file associated with the file
 *   descriptor 'fd', and will write it to the area pointed to by 'buf'.
 *
 ****************************************************************************/

static int littlefs_fstat(FAR const struct file *filep, FAR struct stat *buf)
{
  FAR struct littlefs_mountpt_s *fs;
  FAR struct littlefs_file_s *priv;
  FAR struct inode *inode;
  int ret;

  memset(buf, 0, sizeof(*buf));

  /* Recover our private data from the struct file instance */

  priv  = filep->f_priv;
  inode = filep->f_inode;
  fs    = inode->i_private;

  /* Call LFS to get file size */

  ret = littlefs_semtake(fs);
  if (ret < 0)
    {
      return ret;
    }

  buf->st_size = lfs_file_size(&fs->lfs, &priv->file);
  littlefs_semgive(fs);

  if (buf->st_size < 0)
    {
      return buf->st_size;
    }

  buf->st_mode    = S_IRWXO | S_IRWXG | S_IRWXU | S_IFREG;
  buf->st_blksize = fs->cfg.block_size;
  buf->st_blocks  = (buf->st_size + buf->st_blksize - 1) / buf->st_blksize;

  return OK;
}

/****************************************************************************
 * Name: littlefs_truncate
 *
 * Description:
 *   Set the length of the open, regular file associated with the file
 *   structure 'filep' to 'length'.
 *
 ****************************************************************************/

static int littlefs_truncate(FAR struct file *filep, off_t length)
{
  FAR struct littlefs_mountpt_s *fs;
  FAR struct littlefs_file_s *priv;
  FAR struct inode *inode;
  int ret;

  /* Recover our private data from the struct file instance */

  priv  = filep->f_priv;
  inode = filep->f_inode;
  fs    = inode->i_private;

  /* Call LFS to perform the truncate */

  ret = littlefs_semtake(fs);
  if (ret < 0)
    {
      return ret;
    }

  ret = lfs_file_truncate(&fs->lfs, &priv->file, length);
  littlefs_semgive(fs);

  return ret;
}

/****************************************************************************
 * Name: littlefs_opendir
 *
 * Description: Open a directory for read access
 *
 ****************************************************************************/

static int littlefs_opendir(FAR struct inode *mountpt,
                            FAR const char *relpath,
                            FAR struct fs_dirent_s *dir)
{
  FAR struct littlefs_mountpt_s *fs;
  FAR struct lfs_dir *priv;
  int ret;

  /* Recover our private data from the inode instance */

  fs = mountpt->i_private;

  /* Allocate memory for the open directory */

  priv = kmm_malloc(sizeof(*priv));
  if (priv == NULL)
    {
      return -ENOMEM;
    }

  /* Take the semaphore */

  ret = littlefs_semtake(fs);
  if (ret < 0)
    {
      goto errsem;
    }

  /* Call the LFS's opendir function */

  ret = lfs_dir_open(&fs->lfs, priv, relpath);
  if (ret < 0)
    {
      goto errout;
    }

  dir->fd_position = lfs_dir_tell(&fs->lfs, priv);

  littlefs_semgive(fs);
  dir->u.littlefs = priv;
  return OK;

errout:
  littlefs_semgive(fs);
errsem:
  kmm_free(priv);
  return ret;
}

/****************************************************************************
 * Name: littlefs_closedir
 *
 * Description: Close a directory
 *
 ****************************************************************************/

static int littlefs_closedir(FAR struct inode *mountpt,
                             FAR struct fs_dirent_s *dir)
{
  struct littlefs_mountpt_s *fs;
  FAR struct lfs_dir *priv;
  int ret;

  /* Recover our private data from the inode instance */

  priv = dir->u.littlefs;
  fs   = mountpt->i_private;

  /* Call the LFS's closedir function */

  ret = littlefs_semtake(fs);
  if (ret < 0)
    {
      return ret;
    }

  lfs_dir_close(&fs->lfs, priv);
  littlefs_semgive(fs);

  kmm_free(priv);
  return OK;
}

/****************************************************************************
 * Name: littlefs_readdir
 *
 * Description: Read the next directory entry
 *
 ****************************************************************************/

static int littlefs_readdir(FAR struct inode *mountpt,
                            FAR struct fs_dirent_s *dir)
{
  FAR struct littlefs_mountpt_s *fs;
  FAR struct lfs_dir *priv;
  struct lfs_info info;
  int ret;

  /* Recover our private data from the inode instance */

  priv = dir->u.littlefs;
  fs   = mountpt->i_private;

  /* Call the LFS's readdir function */

  ret = littlefs_semtake(fs);
  if (ret < 0)
    {
      return ret;
    }

  ret = lfs_dir_read(&fs->lfs, priv, &info);
  if (ret > 0)
    {
      dir->fd_position = lfs_dir_tell(&fs->lfs, priv);
      if (info.type == LFS_TYPE_REG)
        {
          dir->fd_dir.d_type = DTYPE_FILE;
        }
      else
        {
          dir->fd_dir.d_type = DTYPE_DIRECTORY;
        }

      strcpy(dir->fd_dir.d_name, info.name);
    }
  else if (ret == 0)
    {
      ret = -ENOENT;
    }

  littlefs_semgive(fs);
  return ret;
}

/****************************************************************************
 * Name: littlefs_rewindir
 *
 * Description: Reset directory read to the first entry
 *
 ****************************************************************************/

static int littlefs_rewinddir(FAR struct inode *mountpt,
                              FAR struct fs_dirent_s *dir)
{
  struct littlefs_mountpt_s *fs;
  FAR struct lfs_dir *priv;
  int ret;

  /* Recover our private data from the inode instance */

  priv = dir->u.littlefs;
  fs   = mountpt->i_private;

  /* Call the LFS's rewinddir function */

  ret = littlefs_semtake(fs);
  if (ret < 0)
    {
      return ret;
    }

  ret = lfs_dir_rewind(&fs->lfs, priv);
  if (ret >= 0)
    {
      dir->fd_position = lfs_dir_tell(&fs->lfs, priv);
    }

  littlefs_semgive(fs);
  return ret;
}

/****************************************************************************
 * Name: littlefs_bind
 *
 * Description: This implements a portion of the mount operation. This
 *  function allocates and initializes the mountpoint private data and
 *  binds the driver inode to the filesystem private data. The final
 *  binding of the private data (containing the driver) to the
 *  mountpoint is performed by mount().
 *
 ****************************************************************************/

static int littlefs_read_block(FAR const struct lfs_config *c,
                               lfs_block_t block, lfs_off_t off,
                               FAR void *buffer, lfs_size_t size)
{
  FAR struct littlefs_mountpt_s *fs = c->context;
  FAR struct mtd_geometry_s *geo = &fs->geo;
  FAR struct inode *drv = fs->drv;
  int ret;

  block = (block * c->block_size + off) / geo->blocksize;
  size  = size / geo->blocksize;

  if (INODE_IS_MTD(drv))
    {
      ret = MTD_BREAD(drv->u.i_mtd, block, size, buffer);
    }
  else
    {
      ret = drv->u.i_bops->read(drv, buffer, block, size);
    }

  return ret >= 0 ? OK : ret;
}

/****************************************************************************
 * Name: littlefs_write_block
 ****************************************************************************/

static int littlefs_write_block(FAR const struct lfs_config *c,
                                lfs_block_t block, lfs_off_t off,
                                FAR const void *buffer, lfs_size_t size)
{
  FAR struct littlefs_mountpt_s *fs = c->context;
  FAR struct mtd_geometry_s *geo = &fs->geo;
  FAR struct inode *drv = fs->drv;
  int ret;

  block = (block * c->block_size + off) / geo->blocksize;
  size  = size / geo->blocksize;

  if (INODE_IS_MTD(drv))
    {
      ret = MTD_BWRITE(drv->u.i_mtd, block, size, buffer);
    }
  else
    {
      ret = drv->u.i_bops->write(drv, buffer, block, size);
    }

  return ret >= 0 ? OK : ret;
}

/****************************************************************************
 * Name: littlefs_erase_block
 ****************************************************************************/

static int littlefs_erase_block(FAR const struct lfs_config *c,
                                lfs_block_t block)
{
  FAR struct littlefs_mountpt_s *fs = c->context;
  FAR struct inode *drv = fs->drv;
  int ret = OK;

  if (INODE_IS_MTD(drv))
    {
      FAR struct mtd_geometry_s *geo = &fs->geo;
      size_t size = c->block_size / geo->erasesize;

      block = block * c->block_size / geo->erasesize;
      ret = MTD_ERASE(drv->u.i_mtd, block, size);
    }

  return ret >= 0 ? OK : ret;
}

/****************************************************************************
 * Name: littlefs_sync_block
 ****************************************************************************/

static int littlefs_sync_block(FAR const struct lfs_config *c)
{
  FAR struct littlefs_mountpt_s *fs = c->context;
  FAR struct inode *drv = fs->drv;
  int ret;

  if (INODE_IS_MTD(drv))
    {
      ret = MTD_IOCTL(drv->u.i_mtd, MTDIOC_FLUSH, 0);
    }
  else
    {
      ret = drv->u.i_bops->ioctl(drv, BIOC_FLUSH, 0);
    }

  return ret == -ENOTTY ? OK : ret;
}

/****************************************************************************
 * Name: littlefs_bind
 ****************************************************************************/

static int littlefs_bind(FAR struct inode *driver, FAR const void *data,
                         FAR void **handle)
{
  FAR struct littlefs_mountpt_s *fs;
  int ret;

  /* Open the block driver */

  if (INODE_IS_BLOCK(driver) && driver->u.i_bops->open)
    {
      ret = driver->u.i_bops->open(driver);
      if (ret < 0)
        {
          return ret;
        }
    }

  /* Create an instance of the mountpt state structure */

  fs = kmm_zalloc(sizeof(*fs));
  if (!fs)
    {
      ret = -ENOMEM;
      goto errout_with_block;
    }

  /* Initialize the allocated mountpt state structure. The filesystem is
   * responsible for one reference on the driver inode and does not
   * have to addref() here (but does have to release in unbind().
   */

  fs->drv = driver;           /* Save the driver reference */
  nxsem_init(&fs->sem, 0, 0); /* Initialize the access control semaphore */

  if (INODE_IS_MTD(driver))
    {
      /* Get MTD geometry directly */

      ret = MTD_IOCTL(driver->u.i_mtd, MTDIOC_GEOMETRY,
                      (unsigned long)&fs->geo);
    }
  else
    {
      /* Try to get FLT MTD geometry first */

      ret = driver->u.i_bops->ioctl(driver, MTDIOC_GEOMETRY,
                                    (unsigned long)&fs->geo);
      if (ret < 0)
        {
          struct geometry geometry;

          /* Not FLT MTD device, get normal block geometry */

          ret = driver->u.i_bops->geometry(driver, &geometry);
          if (ret >= 0)
            {
              /* And convert to MTD geometry */

              fs->geo.blocksize    = geometry.geo_sectorsize;
              fs->geo.erasesize    = geometry.geo_sectorsize;
              fs->geo.neraseblocks = geometry.geo_nsectors;
            }
        }
    }

  if (ret < 0)
    {
      goto errout_with_fs;
    }

  /* Initialize lfs_config structure */

  fs->cfg.context        = fs;
  fs->cfg.read           = littlefs_read_block;
  fs->cfg.prog           = littlefs_write_block;
  fs->cfg.erase          = littlefs_erase_block;
  fs->cfg.sync           = littlefs_sync_block;
  fs->cfg.read_size      = fs->geo.blocksize;
  fs->cfg.prog_size      = fs->geo.blocksize;
  fs->cfg.block_size     = fs->geo.erasesize;
  fs->cfg.block_count    = fs->geo.neraseblocks;
  fs->cfg.block_cycles   = 500;
  fs->cfg.cache_size     = fs->geo.blocksize;
  fs->cfg.lookahead_size = lfs_min(lfs_alignup(fs->cfg.block_count, 64) / 8,
                                   fs->cfg.read_size);

  /* Then get information about the littlefs filesystem on the devices
   * managed by this driver.
   */

  /* Force format the device if -o forceformat */

  if (data && strcmp(data, "forceformat") == 0)
    {
      ret = lfs_format(&fs->lfs, &fs->cfg);
      if (ret < 0)
        {
          goto errout_with_fs;
        }
    }

  ret = lfs_mount(&fs->lfs, &fs->cfg);
  if (ret < 0)
    {
      /* Auto format the device if -o autoformat */

      if (ret != LFS_ERR_CORRUPT ||
          !data || strcmp(data, "autoformat"))
        {
          goto errout_with_fs;
        }

      ret = lfs_format(&fs->lfs, &fs->cfg);
      if (ret < 0)
        {
          goto errout_with_fs;
        }

      /* Try to mount the device again */

      ret = lfs_mount(&fs->lfs, &fs->cfg);
      if (ret < 0)
        {
          goto errout_with_fs;
        }
    }

  *handle = fs;
  littlefs_semgive(fs);
  return OK;

errout_with_fs:
  nxsem_destroy(&fs->sem);
  kmm_free(fs);
errout_with_block:
  if (INODE_IS_BLOCK(driver) && driver->u.i_bops->close)
    {
      driver->u.i_bops->close(driver);
    }

  return ret;
}

/****************************************************************************
 * Name: littlefs_unbind
 *
 * Description: This implements the filesystem portion of the umount
 *  operation.
 *
 ****************************************************************************/

static int littlefs_unbind(FAR void *handle, FAR struct inode **driver,
                           unsigned int flags)
{
  FAR struct littlefs_mountpt_s *fs = handle;
  FAR struct inode *drv = fs->drv;
  int ret;

  /* Unmount */

  ret = littlefs_semtake(fs);
  if (ret < 0)
    {
      return ret;
    }

  ret = lfs_unmount(&fs->lfs);
  littlefs_semgive(fs);

  if (ret >= 0)
    {
      /* Close the block driver */

      if (INODE_IS_BLOCK(drv) && drv->u.i_bops->close)
        {
          drv->u.i_bops->close(drv);
        }

      /* We hold a reference to the driver but should not but
       * mucking with inodes in this context. So, we will just return
       * our contained reference to the driver inode and let the
       * umount logic dispose of it.
       */

      if (driver)
        {
          *driver = drv;
        }

      /* Release the mountpoint private data */

      nxsem_destroy(&fs->sem);
      kmm_free(fs);
    }

  return ret;
}

/****************************************************************************
 * Name: littlefs_statfs
 *
 * Description: Return filesystem statistics
 *
 ****************************************************************************/

static int littlefs_statfs(FAR struct inode *mountpt, FAR struct statfs *buf)
{
  FAR struct littlefs_mountpt_s *fs;
  int ret;

  /* Get the mountpoint private data from the inode structure */

  fs = mountpt->i_private;

  /* Return something for the file system description */

  memset(buf, 0, sizeof(*buf));
  buf->f_type    = LITTLEFS_SUPER_MAGIC;
  buf->f_namelen = LFS_NAME_MAX;
  buf->f_bsize   = fs->cfg.block_size;
  buf->f_blocks  = fs->cfg.block_count;
  buf->f_bfree   = fs->cfg.block_count;
  buf->f_bavail  = fs->cfg.block_count;

  ret = littlefs_semtake(fs);
  if (ret < 0)
    {
      return ret;
    }

  ret = lfs_fs_size(&fs->lfs);
  if (ret > 0)
    {
      buf->f_bfree -= ret;
      buf->f_bavail -= ret;

      ret = 0;
    }

  littlefs_semgive(fs);
  return ret;
}

/****************************************************************************
 * Name: littlefs_unlink
 *
 * Description: Remove a file
 *
 ****************************************************************************/

static int littlefs_unlink(FAR struct inode *mountpt,
                           FAR const char *relpath)
{
  FAR struct littlefs_mountpt_s *fs;
  int ret;

  /* Get the mountpoint private data from the inode structure */

  fs = mountpt->i_private;

  /* Call the LFS to perform the unlink */

  ret = littlefs_semtake(fs);
  if (ret < 0)
    {
      return ret;
    }

  ret = lfs_remove(&fs->lfs, relpath);
  littlefs_semgive(fs);

  return ret;
}

/****************************************************************************
 * Name: littlefs_mkdir
 *
 * Description: Create a directory
 *
 ****************************************************************************/

static int littlefs_mkdir(FAR struct inode *mountpt, FAR const char *relpath,
                          mode_t mode)
{
  FAR struct littlefs_mountpt_s *fs;
  int ret;

  /* Get the mountpoint private data from the inode structure */

  fs = mountpt->i_private;

  /* Call LFS to do the mkdir */

  ret = littlefs_semtake(fs);
  if (ret < 0)
    {
      return ret;
    }

  ret = lfs_mkdir(&fs->lfs, relpath);
  littlefs_semgive(fs);

  return ret;
}

/****************************************************************************
 * Name: littlefs_rmdir
 *
 * Description: Remove a directory
 *
 ****************************************************************************/

static int littlefs_rmdir(FAR struct inode *mountpt, FAR const char *relpath)
{
  return littlefs_unlink(mountpt, relpath);
}

/****************************************************************************
 * Name: littlefs_rename
 *
 * Description: Rename a file or directory
 *
 ****************************************************************************/

static int littlefs_rename(FAR struct inode *mountpt,
                           FAR const char *oldrelpath,
                           FAR const char *newrelpath)
{
  FAR struct littlefs_mountpt_s *fs;
  int ret;

  /* Get the mountpoint private data from the inode structure */

  fs = mountpt->i_private;

  /* Call LFS to do the rename */

  ret = littlefs_semtake(fs);
  if (ret < 0)
    {
      return ret;
    }

  ret = lfs_rename(&fs->lfs, oldrelpath, newrelpath);
  littlefs_semgive(fs);

  return ret;
}

/****************************************************************************
 * Name: littlefs_stat
 *
 * Description: Return information about a file or directory
 *
 ****************************************************************************/

static int littlefs_stat(FAR struct inode *mountpt, FAR const char *relpath,
                         FAR struct stat *buf)
{
  FAR struct littlefs_mountpt_s *fs;
  struct lfs_info info;
  int ret;

  memset(buf, 0, sizeof(*buf));

  /* Get the mountpoint private data from the inode structure */

  fs = mountpt->i_private;

  /* Call the LFS to do the stat operation */

  ret = littlefs_semtake(fs);
  if (ret < 0)
    {
      return ret;
    }

  ret = lfs_stat(&fs->lfs, relpath, &info);
  littlefs_semgive(fs);

  if (ret >= 0)
    {
      /* Convert info to stat */

      buf->st_mode = S_IRWXO | S_IRWXG | S_IRWXU;
      if (info.type == LFS_TYPE_REG)
        {
          buf->st_mode |= S_IFREG;
        }
      else
        {
          buf->st_mode |= S_IFDIR;
        }

      buf->st_size    = info.size;
      buf->st_blksize = fs->cfg.block_size;
      buf->st_blocks  = (buf->st_size + buf->st_blksize - 1) /
                        buf->st_blksize;
    }

  return ret;
}
