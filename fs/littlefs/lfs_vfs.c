/****************************************************************************
 * fs/littlefs/lfs.c
 *
 * This file is a part of NuttX:
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *
 * Ported by:
 *
 *   Copyright (C) 2019 Pinecone Inc. All rights reserved.
 *   Author: lihaichen <li8303@163.com>
 *
 * This port derives from ARM mbed logic which has a compatible 3-clause
 * BSD license:
 *
 *   Copyright (c) 2017, Arm Limited. All rights reserved.
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
 * 3. Neither the names ARM, NuttX nor the names of its contributors may be
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

#include <sys/stat.h>
#include <sys/statfs.h>
#include <assert.h>
#include <debug.h>
#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <queue.h>
#include <semaphore.h>
#include <stdint.h>
#include <string.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/dirent.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/mtd/mtd.h>

#include "lfs.h"
#include "lfs_util.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct littefs_s
{
  struct lfs_s lfs;
  struct lfs_config_s cfg;
  FAR struct mtd_dev_s *mtd;
  struct mtd_geometry_s geo;
  sem_t fs_sem;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void    littlefs_semgive(FAR struct littefs_s *fs);
static void    littlefs_semtake(FAR struct littefs_s *fs);

static void    littlefs_tostat(FAR struct stat *st,
                               FAR struct lfs_info_s *info);

static int     littlefs_open(FAR struct file *filep,
                             FAR const char *relpath, int oflags,
                             mode_t mode);
static int     littlefs_close(FAR struct file *filep);
static ssize_t littlefs_read(FAR struct file *filep, FAR char *buffer,
                             size_t buflen);
static ssize_t littlefs_write(FAR struct file *filep,
                              FAR const char *buffer, size_t buflen);
static off_t   littlefs_seek(FAR struct file *filep, off_t offset,
                             int whence);

static int     littlefs_bind(FAR struct inode *mtdinode,
                             FAR const void *data, FAR void **handle);
static int     littlefs_unbind(FAR void *handle,
                               FAR struct inode **mtdinode,
                               unsigned int flags);

static int     littlefs_statfs(FAR struct inode *mountpt,
                               FAR struct statfs *buf);

static int     littlefs_opendir(FAR struct inode *mountpt,
                                FAR const char *relpath,
                                FAR struct fs_dirent_s *dir);
static int     littlefs_closedir(FAR struct inode *mountpt,
                                 FAR struct fs_dirent_s *dir);
static int     littlefs_readdir(FAR struct inode *mountpt,
                                FAR struct fs_dirent_s *dir);

static int     littlefs_rewinddir(FAR struct inode *mountpt,
                                  FAR struct fs_dirent_s *dir);

static int     littlefs_fstat(FAR const struct file *filep,
                              FAR struct stat *buf);
static int     littlefs_truncate(FAR struct file *filep, off_t length);

static int     littlefs_mkdir(FAR struct inode *mountpt,
                              FAR const char *relpath, mode_t mode);
static int     littlefs_rmdir(FAR struct inode *mountpt,
                              FAR const char *relpath);

static int     littlefs_unlink(FAR struct inode *mountpt,
                               FAR const char *relpath);

static int     littlefs_rename(FAR struct inode *mountpt,
                               FAR const char *oldrelpath,
                               FAR const char *newrelpath);
static int     littlefs_stat(FAR struct inode *mountpt,
                             FAR const char *relpath,
                             FAR struct stat *buf);

/****************************************************************************
 * Public Data
 ****************************************************************************/

const struct mountpt_operations littlefs_operations =
{
  littlefs_open,      /* open */
  littlefs_close,     /* close */
  littlefs_read,      /* read */
  littlefs_write,     /* write */
  littlefs_seek,      /* seek */
  NULL,               /* ioctl */

  NULL,               /* sync */
  NULL,               /* dup */
  littlefs_fstat,     /* fstat */
  littlefs_truncate,  /* truncate */

  littlefs_opendir,   /* opendir */
  littlefs_closedir,  /* closedir */
  littlefs_readdir,   /* readdir */
  littlefs_rewinddir, /* rewinddir */

  littlefs_bind,      /* bind */
  littlefs_unbind,    /* unbind */
  littlefs_statfs,    /* statfs */

  littlefs_unlink,    /* unlink */
  littlefs_mkdir,     /* mkdir */
  littlefs_rmdir,     /* rmdir */
  littlefs_rename,    /* rename */
  littlefs_stat,      /* stat */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

static void littlefs_semtake(FAR struct littefs_s *fs)
{
  int ret;
  do
    {
      ret = nxsem_wait(&fs->fs_sem);
      DEBUGASSERT(ret == OK || ret == -EINTR);
    }
  while (ret == -EINTR);
}

static void littlefs_semgive(FAR struct littefs_s *fs)
{
  nxsem_post(&fs->fs_sem);
}

static int vfs_flag_to_lfs(int flags)
{
  int res = 0;

  if ((flags & 3) == O_RDONLY)
    {
      res |= LFS_O_RDONLY;
    }

  if ((flags & 3) == O_WRONLY)
    {
      res |= LFS_O_WRONLY;
    }

  if ((flags & 3) == O_RDWR)
    {
      res |= LFS_O_RDWR;
    }

  if (flags & O_CREAT)
    {
      res |= LFS_O_CREAT;
    }

  if (flags & O_EXCL)
    {
      res |= LFS_O_EXCL;
    }

  if (flags & O_TRUNC)
    {
      res |= LFS_O_TRUNC;
    }

  if (flags & O_APPEND)
    {
      res |= LFS_O_APPEND;
    }

  return res;
}

/* Conversion error code */

static int lfs_result_to_vfs(int result)
{
  int status = 0;

  switch (result)
    {
      case LFS_ERR_OK:
        status = OK;
        break;

      case LFS_ERR_IO:
        status = -EIO;
        break; /* Error during device operation */

      case LFS_ERR_NOENT:
        status = -ENOENT;
        break; /* No directory entry */

      case LFS_ERR_EXIST:
        status = -EEXIST;
        break; /* Entry already exists */

      case LFS_ERR_NOTDIR:
        status = -ENOTDIR;
        break; /* Entry is not a dir */

      case LFS_ERR_ISDIR:
        status = -EISDIR;
        break; /* Entry is a dir */

      case LFS_ERR_NOTEMPTY:
        status = -ENOTEMPTY;
        break; /* Dir is not empty */

      case LFS_ERR_BADF:
        status = -EBADF;
        break; /* Bad file number */

      case LFS_ERR_INVAL:
        status = -EINVAL;
        break; /* Invalid parameter */

      case LFS_ERR_NOSPC:
        status = -ENOSPC;
        break; /* No space left on device */

      case LFS_ERR_NOMEM:
        status = -ENOMEM;
        break; /* No more memory available */

      case LFS_ERR_CORRUPT:
        status = LFS_ERR_CORRUPT;
        break; /* Corrupted */

      default:
        status = -EIO;
        break;
    }

  return status;
}

static int _lfs_flash_read(FAR const struct lfs_config_s *cfg,
                           lfs_block_t block, lfs_off_t off,
                           FAR void *buffer, lfs_size_t size)
{
  FAR struct mtd_dev_s *mtd;

  DEBUGASSERT(cfg != NULL);
  DEBUGASSERT(cfg->context != NULL);

  mtd = (FAR struct mtd_dev_s *)cfg->context;
  if (mtd->read(mtd, block * cfg->block_size + off, size, buffer) != size)
    {
      return LFS_ERR_IO;
    }

  return LFS_ERR_OK;
}

static int _lfs_flash_prog(FAR const struct lfs_config_s *cfg,
                           lfs_block_t block, lfs_off_t off,
                           FAR const void *buffer, lfs_size_t size)
{
  FAR struct mtd_dev_s *mtd;

  DEBUGASSERT(cfg != NULL);
  DEBUGASSERT(cfg->context != NULL);

  mtd = (FAR struct mtd_dev_s *)cfg->context;
  if (mtd->write(mtd, block * cfg->block_size + off, size, buffer) != size)
    {
      return LFS_ERR_IO;
    }

  return LFS_ERR_OK;
}

static int _lfs_flash_erase(FAR const struct lfs_config_s *cfg,
                            lfs_block_t block)
{
  FAR struct mtd_dev_s *mtd;

  DEBUGASSERT(cfg != NULL);
  DEBUGASSERT(cfg->context != NULL);

  mtd = (struct mtd_dev_s *)cfg->context;
  if (mtd->erase(mtd, block, 1) != 1)
    {
      return LFS_ERR_IO;
    }

  return LFS_ERR_OK;
}

static int _lfs_flash_sync(FAR const struct lfs_config_s *cfg)
{
    return LFS_ERR_OK;
}

static int littlefs_bind(FAR struct inode *mtdinode,
                         FAR const void *data, FAR void **handle)
{
  FAR struct mtd_dev_s *mtd;
  FAR struct littefs_s *fs;
  int ret;

  finfo("mtdinode=%p data=%p handle=%p\n", mtdinode, data, handle);
  DEBUGASSERT(mtdinode != NULL && handle != NULL);

  /* Extract the MTD interface reference */

  DEBUGASSERT(INODE_IS_MTD(mtdinode) && mtdinode->u.i_mtd != NULL);
  mtd = mtdinode->u.i_mtd;
  fs = (FAR struct littefs_s *)kmm_zalloc(sizeof(struct littefs_s));
  if (fs == NULL)
    {
      ferr("ERROR: Failed to allocate volume structure\n");
      return -ENOMEM;
    }

  (void)nxsem_init(&fs->fs_sem, 0, 0);

  fs->mtd = mtd;

  ret = MTD_IOCTL(mtd, MTDIOC_GEOMETRY,
                  (unsigned long)((uintptr_t)&fs->geo));
  if (ret < 0)
    {
      ret = -ENODEV;
      ferr("ERROR: MTD_IOCTL(MTDIOC_GEOMETRY) failed: %d\n", ret);
      goto errout_with_volume;
    }

  fs->cfg.context = mtd;
  fs->cfg.read_size = fs->geo.blocksize;
  fs->cfg.prog_size = fs->geo.blocksize;
  fs->cfg.block_size = fs->geo.erasesize;
  fs->cfg.block_count = fs->geo.neraseblocks;

  fs->cfg.lookahead = 32 * ((fs->cfg.block_count + 31) / 32);
  if (fs->cfg.lookahead > 512)
    {
      fs->cfg.lookahead = 512;
    }

  fs->cfg.read = _lfs_flash_read;
  fs->cfg.prog = _lfs_flash_prog;
  fs->cfg.erase = _lfs_flash_erase;
  fs->cfg.sync = _lfs_flash_sync;

  ret = lfs_mount(&fs->lfs, &fs->cfg);
  if (ret != LFS_ERR_OK)
    {
      finfo("mount error\n");
      ret = lfs_format(&fs->lfs, &fs->cfg);
      if (ret != LFS_ERR_OK)
        {
          ret = -EINVAL;
          ferr("ERROR: Invalid lfs format failed: %d\n", ret);
          goto errout_with_volume;
        }

      ret = lfs_mount(&fs->lfs, &fs->cfg);
      if (ret != LFS_ERR_OK)
        {
          ret = -EINVAL;
          ferr("ERROR: Invalid lfs mount failed: %d\n", ret);
          goto errout_with_volume;
        }
    }

  *handle = (FAR void *)fs;
  littlefs_semgive(fs);
  finfo("mount ok\n");
  return OK;

errout_with_volume:
  nxsem_destroy(&fs->fs_sem);
  kmm_free(fs);
  return ret;
}

static int littlefs_unbind(FAR void *handle, FAR struct inode **mtdinode,
                           unsigned int flags)
{
  FAR struct littefs_s *fs = (FAR struct littefs_s *)handle;
  int ret = OK;

  if (!fs)
    {
      return -EINVAL;
    }

  littlefs_semtake(fs);
  ret = lfs_unmount(&fs->lfs);
  nxsem_destroy(&fs->fs_sem);
  kmm_free(fs);
  return lfs_result_to_vfs(ret);
}

static int littlefs_opendir(FAR struct inode *mountpt,
                            FAR const char *relpath,
                            FAR struct fs_dirent_s *dir)
{
  FAR struct littefs_s *fs = mountpt->i_private;
  int ret = OK;

  DEBUGASSERT(mountpt != NULL && mountpt->i_private != NULL);

  littlefs_semtake(fs);
  ret = lfs_dir_open(&fs->lfs, (lfs_dir_t *)&dir->u.littlefs, relpath);
  littlefs_semgive(fs);

  finfo("open dir\n");
  return lfs_result_to_vfs(ret);
}

static int littlefs_closedir(FAR struct inode *mountpt,
                             FAR struct fs_dirent_s *dir)
{
  FAR struct littefs_s *fs = mountpt->i_private;
  int ret = OK;

  finfo("close dir\n");
  DEBUGASSERT(mountpt != NULL && mountpt->i_private != NULL);

  littlefs_semtake(fs);
  ret = lfs_dir_close(&fs->lfs, (lfs_dir_t *)&dir->u.littlefs);
  littlefs_semgive(fs);

  return lfs_result_to_vfs(ret);
}

static int littlefs_readdir(FAR struct inode *mountpt,
                            FAR struct fs_dirent_s *dir)
{
  FAR struct littefs_s *fs = mountpt->i_private;
  struct lfs_info_s info;
  int ret = OK;

  DEBUGASSERT(mountpt != NULL && mountpt->i_private != NULL);

  littlefs_semtake(fs);
  ret = lfs_dir_read(&fs->lfs, (lfs_dir_t *)&dir->u.littlefs, &info);
  if (ret == 1)
    {
      memset(dir->fd_dir.d_name, 0, NAME_MAX);
      strncpy(dir->fd_dir.d_name, info.name, NAME_MAX);
      dir->fd_dir.d_type = 0;
      switch (info.type)
        {
          case LFS_TYPE_DIR:
            dir->fd_dir.d_type |= DTYPE_DIRECTORY;
            break;

          case LFS_TYPE_REG:
            dir->fd_dir.d_type |= DTYPE_FILE;
            break;
        }
    }
  else
    {
      goto breakout;
    }

  littlefs_semgive(fs);
  return OK;

breakout:
  ret = -ENOENT;
  littlefs_semgive(fs);
  return ret;
}

static int littlefs_rewinddir(FAR struct inode *mountpt,
                              FAR struct fs_dirent_s *dir)
{
  FAR struct littefs_s *fs = mountpt->i_private;
  int ret = OK;

  finfo("littlefs_rewinddir\n");
  DEBUGASSERT(mountpt != NULL && mountpt->i_private != NULL);

  littlefs_semtake(fs);
  ret = lfs_dir_rewind(&fs->lfs, (lfs_dir_t *)&dir->u.littlefs);
  if (ret != LFS_ERR_OK)
    {
      goto breakout;
    }

  littlefs_semgive(fs);
  return lfs_result_to_vfs(ret);

breakout:
  littlefs_semgive(fs);
  return lfs_result_to_vfs(ret);
}

static int littlefs_mkdir(FAR struct inode *mountpt,
                          FAR const char *relpath, mode_t mode)
{
  FAR struct littefs_s *fs = mountpt->i_private;
  int ret = OK;

  finfo("littlefs_mkdir\n");
  DEBUGASSERT(mountpt != NULL && mountpt->i_private != NULL);

  littlefs_semtake(fs);
  ret = lfs_mkdir(&fs->lfs, relpath);
  if (ret != LFS_ERR_OK)
    {
      goto breakout;
    }

  littlefs_semgive(fs);
  return lfs_result_to_vfs(ret);

breakout:
  littlefs_semgive(fs);
  return lfs_result_to_vfs(ret);
}

static int littlefs_rmdir(FAR struct inode *mountpt,
                          FAR const char *relpath)
{
  FAR struct littefs_s *fs = mountpt->i_private;
  int ret = OK;

  finfo("littlefs_rmdir\n");
  DEBUGASSERT(mountpt != NULL && mountpt->i_private != NULL);

  littlefs_semtake(fs);
  ret = lfs_remove(&fs->lfs, relpath);
  if (ret != LFS_ERR_OK)
    {
      goto breakout;
    }

  littlefs_semgive(fs);
  return lfs_result_to_vfs(ret);

breakout:
  littlefs_semgive(fs);
  return lfs_result_to_vfs(ret);
}

static int littlefs_open(FAR struct file *filep, FAR const char *relpath,
                         int oflags, mode_t mode)
{
  FAR struct inode *inode;
  FAR struct littefs_s *fs;
  FAR lfs_file_t *lf = NULL;
  int ret = OK;

  finfo("littlefs_open [%s]\n", relpath);
  DEBUGASSERT(filep != NULL && filep->f_inode != NULL &&
              filep->f_inode->i_private);

  inode = filep->f_inode;
  fs = inode->i_private;

  littlefs_semtake(fs);

  lf = (lfs_file_t *)kmm_zalloc(sizeof(lfs_file_t));
  if (!lf)
    {
      ret = LFS_ERR_NOMEM;
      goto breakout;
    }

  ret = lfs_file_open(&fs->lfs, lf, relpath, vfs_flag_to_lfs(oflags));
  if (ret != LFS_ERR_OK)
    {
      ret = lfs_result_to_vfs(ret);
      goto breakout;
    }

  filep->f_priv = lf;
  littlefs_semgive(fs);
  return lfs_result_to_vfs(ret);

breakout:
  if (lf != NULL)
    {
      kmm_free(lf);
    }

  littlefs_semgive(fs);
  return lfs_result_to_vfs(ret);
}

static int littlefs_close(FAR struct file *filep)
{
  FAR struct inode *inode;
  FAR struct littefs_s *fs;
  FAR lfs_file_t *lf = NULL;
  int ret = OK;

  finfo("littlefs_close\n");
  DEBUGASSERT(filep != NULL && filep->f_inode != NULL &&
              filep->f_inode->i_private);

  inode = filep->f_inode;
  fs = inode->i_private;
  lf = filep->f_priv;

  littlefs_semtake(fs);
  ret = lfs_file_close(&fs->lfs, lf);
  kmm_free(lf);
  littlefs_semgive(fs);

  return lfs_result_to_vfs(ret);
}

static ssize_t littlefs_read(FAR struct file *filep, FAR char *buffer,
                             size_t buflen)
{
  FAR struct inode *inode;
  FAR struct littefs_s *fs;
  FAR lfs_file_t *lf = NULL;
  ssize_t ret = 0;

  finfo("littlefs_read\n");
  DEBUGASSERT(filep != NULL && filep->f_inode != NULL &&
              filep->f_inode->i_private);

  inode = filep->f_inode;
  fs = inode->i_private;
  lf = filep->f_priv;

  littlefs_semtake(fs);
  ret = lfs_file_read(&fs->lfs, lf, buffer, buflen);
  littlefs_semgive(fs);
  if (ret < 0)
    {
      ret = lfs_result_to_vfs(ret);
    }

  return ret;
}

static ssize_t littlefs_write(FAR struct file *filep, FAR const char *buffer,
                              size_t buflen)
{
  FAR struct inode *inode;
  FAR struct littefs_s *fs;
  FAR lfs_file_t *lf = NULL;
  ssize_t ret = 0;

  finfo("littlefs_write %d\n", buflen);
  DEBUGASSERT(filep != NULL && filep->f_inode != NULL &&
              filep->f_inode->i_private);

  inode = filep->f_inode;
  fs = inode->i_private;
  lf = filep->f_priv;

  littlefs_semtake(fs);
  ret = lfs_file_write(&fs->lfs, lf, buffer, buflen);
  littlefs_semgive(fs);
  if (ret < 0)
    {
      ret = lfs_result_to_vfs(ret);
    }

  return ret;
}

static off_t littlefs_seek(FAR struct file *filep, off_t offset, int whence)
{
  FAR struct inode *inode;
  FAR struct littefs_s *fs;
  FAR lfs_file_t *lf = NULL;
  off_t ret = 0;

  finfo("littlefs_seek\n");
  DEBUGASSERT(filep != NULL && filep->f_inode != NULL &&
              filep->f_inode->i_private);

  inode = filep->f_inode;
  fs = inode->i_private;
  lf = filep->f_priv;

  littlefs_semtake(fs);
  ret = lfs_file_seek(&fs->lfs, lf, offset, whence);
  littlefs_semgive(fs);
  if (ret < 0)
    {
      ret = lfs_result_to_vfs(ret);
    }

  return ret;
}

static int littlefs_fstat(FAR const struct file *filep, FAR struct stat *buf)
{
  finfo("littlefs_fstat\n");
  return OK;
}

static int littlefs_truncate(FAR struct file *filep, off_t length)
{
  FAR struct inode *inode;
  FAR struct littefs_s *fs;
  FAR lfs_file_t *lf = NULL;
  int ret = 0;

  finfo("littlefs_truncate\n");
  DEBUGASSERT(filep != NULL && filep->f_inode != NULL &&
              filep->f_inode->i_private);

  inode = filep->f_inode;
  fs = inode->i_private;
  lf = filep->f_priv;

  littlefs_semtake(fs);
  ret = lfs_file_truncate(&fs->lfs, lf, length);
  littlefs_semgive(fs);
  if (ret < 0)
    {
      ret = lfs_result_to_vfs(ret);
    }

  return ret;
}

static int littlefs_statfs_count(FAR void *p, lfs_block_t b)
{
  *(FAR lfs_size_t *)p += 1;
  return 0;
}

static int littlefs_statfs(FAR struct inode *mountpt, FAR struct statfs *buf)
{
  FAR struct littefs_s *fs = mountpt->i_private;
  lfs_size_t in_use = 0;
  int ret = OK;

  finfo("littlefs_statfs\n");
  DEBUGASSERT(mountpt != NULL && mountpt->i_private != NULL);

  littlefs_semtake(fs);
  ret = lfs_traverse(&fs->lfs, littlefs_statfs_count, &in_use);
  if (ret != LFS_ERR_OK)
    {
      goto breakout;
    }

  memset(buf, 0, sizeof(struct statfs));
  buf->f_type = LITTLEFS_SUPER_MAGIC;
  buf->f_bsize = fs->cfg.block_size;
  buf->f_blocks = fs->cfg.block_count;
  buf->f_bfree = fs->cfg.block_count - in_use;
  buf->f_bavail = buf->f_bfree;

  littlefs_semgive(fs);
  return lfs_result_to_vfs(ret);

breakout:
  littlefs_semgive(fs);
  return lfs_result_to_vfs(ret);
}

static int littlefs_unlink(FAR struct inode *mountpt, FAR const char *relpath)
{
  FAR struct littefs_s *fs = mountpt->i_private;
  int ret = OK;

  finfo("littlefs_unlink\n");
  DEBUGASSERT(mountpt != NULL && mountpt->i_private != NULL);

  littlefs_semtake(fs);
  ret = lfs_remove(&fs->lfs, relpath);
  littlefs_semgive(fs);

  return lfs_result_to_vfs(ret);
}

static int littlefs_rename(FAR struct inode *mountpt,
                           FAR const char *oldrelpath,
                           FAR const char *newrelpath)
{
  FAR struct littefs_s *fs = mountpt->i_private;
  int ret = OK;

  finfo("littlefs_rename\n");
  DEBUGASSERT(mountpt != NULL && mountpt->i_private != NULL);

  littlefs_semtake(fs);
  ret = lfs_rename(&fs->lfs, oldrelpath, newrelpath);
  littlefs_semgive(fs);

  return lfs_result_to_vfs(ret);
}

static int littlefs_stat(FAR struct inode *mountpt, FAR const char *relpath,
                         FAR struct stat *buf)
{
  FAR struct littefs_s *fs = mountpt->i_private;
  struct lfs_info_s info;
  int ret = OK;

  finfo("littlefs_stat\n");
  DEBUGASSERT(mountpt != NULL && mountpt->i_private != NULL);

  memset(buf, 0, sizeof(struct stat));

  littlefs_semtake(fs);
  ret = lfs_stat(&fs->lfs, relpath, &info);
  if (ret != LFS_ERR_OK)
    {
      goto breakout;
    }

  littlefs_tostat(buf, &info);
  littlefs_semgive(fs);
  return lfs_result_to_vfs(ret);

breakout:
  littlefs_semgive(fs);
  return lfs_result_to_vfs(ret);
}

static void littlefs_tostat(struct stat *st, struct lfs_info_s *info)
{
  memset(st, 0, sizeof(struct stat));
  st->st_size = info->size;
  st->st_mode = S_IRWXU | S_IRWXG | S_IRWXO;
  switch (info->type)
    {
      case LFS_TYPE_DIR:
        st->st_mode |= S_IFDIR;
        break;

      case LFS_TYPE_REG:
        st->st_mode |= S_IFREG;
        break;
    }
}
