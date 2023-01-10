/****************************************************************************
 * fs/yaffs/yaffs_vfs.c
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

#include <debug.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>

#include <nuttx/fs/fs.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mtd/mtd.h>
#include <nuttx/mutex.h>

#include <sys/stat.h>
#include <sys/statfs.h>

#include "inode/inode.h"

#include "yaffs_guts.h"
#include "direct/yaffsfs.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define mountpt_to_yaffs_dev(mountpt) \
            (((FAR struct yaffs_mountpt_s *)((mountpt)->i_private))->dev)
#define filep_to_yaffs_handle(filep) \
            (((FAR struct yaffs_file_s *)((filep)->f_priv))->handle)
#define dir_to_yaffs_dir(dir) \
            (((FAR struct yaffs_dir_s *)dir)->dir)
#define yaffs_dev_to_mtd(dev) \
            ((FAR struct mtd_dev_s *)((dev)->driver_context))

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure represents the overall mountpoint state. An instance of
 * this structure is retained as inode private data on each mountpoint that
 * is mounted with a yaffs filesystem.
 */

struct yaffs_mountpt_s
{
  FAR struct inode      *drv;
  FAR struct mtd_dev_s  *mtd;
  FAR struct yaffs_dev  *dev;
  struct mtd_geometry_s geo;
};

struct yaffs_file_s
{
  char path[PATH_MAX + 1];
  int  handle;
};

struct yaffs_dir_s
{
  struct fs_dirent_s base;
  FAR yaffs_DIR      *dir;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int     yaffs_vfs_open(FAR struct file *filep,
                              FAR const char *relpath,
                              int oflags, mode_t mode);
static int     yaffs_vfs_close(FAR struct file *filep);
static ssize_t yaffs_vfs_read(FAR struct file *filep,
                              FAR char *buffer, size_t buflen);
static ssize_t yaffs_vfs_write(FAR struct file *filep,
                               FAR const char *buffer, size_t buflen);
static off_t   yaffs_vfs_seek(FAR struct file *filep, off_t offset,
                              int whence);
static int     yaffs_vfs_ioctl(FAR struct file *filep, int cmd,
                               unsigned long arg);

static int     yaffs_vfs_sync(FAR struct file *filep);
static int     yaffs_vfs_dup(FAR const struct file *oldp,
                             FAR struct file *newp);
static int     yaffs_vfs_fstat(FAR const struct file *filep,
                               FAR struct stat *buf);
static int     yaffs_vfs_fchstat(FAR const struct file *filep,
                                 FAR const struct stat *buf, int flags);
static int     yaffs_vfs_truncate(FAR struct file *filep,
                                  off_t length);

static int     yaffs_vfs_opendir(FAR struct inode *mountpt,
                                 FAR const char *relpath,
                                 FAR struct fs_dirent_s **dir);
static int     yaffs_vfs_closedir(FAR struct inode *mountpt,
                                  FAR struct fs_dirent_s *dir);
static int     yaffs_vfs_readdir(FAR struct inode *mountpt,
                                 FAR struct fs_dirent_s *dir,
                                 FAR struct dirent *entry);
static int     yaffs_vfs_rewinddir(FAR struct inode *mountpt,
                                   FAR struct fs_dirent_s *dir);

static int     yaffs_vfs_bind(FAR struct inode *driver,
                              FAR const void *data, FAR void **handle);
static int     yaffs_vfs_unbind(FAR void *handle, FAR struct inode **driver,
                                unsigned int flags);
static int     yaffs_vfs_statfs(FAR struct inode *mountpt,
                                FAR struct statfs *buf);

static int     yaffs_vfs_unlink(FAR struct inode *mountpt,
                                FAR const char *relpath);
static int     yaffs_vfs_mkdir(FAR struct inode *mountpt,
                               FAR const char *relpath, mode_t mode);
static int     yaffs_vfs_rmdir(FAR struct inode *mountpt,
                               FAR const char *relpath);
static int     yaffs_vfs_rename(FAR struct inode *mountpt,
                                FAR const char *oldrelpath,
                                FAR const char *newrelpath);
static int     yaffs_vfs_stat(FAR struct inode *mountpt,
                              FAR const char *relpath, FAR struct stat *buf);
static int     yaffs_vfs_chstat(FAR struct inode *mountpt,
                                FAR const char *relpath,
                                FAR const struct stat *buf, int flags);

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* See fs_mount.c -- this structure is explicitly extern'ed there.
 * We use the old-fashioned kind of initializers so that this will compile
 * with any compiler.
 */

const struct mountpt_operations yaffs_operations =
{
  yaffs_vfs_open,          /* open */
  yaffs_vfs_close,         /* close */
  yaffs_vfs_read,          /* read */
  yaffs_vfs_write,         /* write */
  yaffs_vfs_seek,          /* seek */
  yaffs_vfs_ioctl,         /* ioctl */
  NULL,                    /* mmap */
  yaffs_vfs_truncate,      /* truncate */

  yaffs_vfs_sync,          /* sync */
  yaffs_vfs_dup,           /* dup */
  yaffs_vfs_fstat,         /* fstat */
  yaffs_vfs_fchstat,       /* fchstat */

  yaffs_vfs_opendir,       /* opendir */
  yaffs_vfs_closedir,      /* closedir */
  yaffs_vfs_readdir,       /* readdir */
  yaffs_vfs_rewinddir,     /* rewinddir */

  yaffs_vfs_bind,          /* bind */
  yaffs_vfs_unbind,        /* unbind */
  yaffs_vfs_statfs,        /* statfs */

  yaffs_vfs_unlink,        /* unlink */
  yaffs_vfs_mkdir,         /* mkdir */
  yaffs_vfs_rmdir,         /* rmdir */
  yaffs_vfs_rename,        /* rename */
  yaffs_vfs_stat,          /* stat */
  yaffs_vfs_chstat         /* chstat */
};

/****************************************************************************
 * External Functions
 ****************************************************************************/

extern int yaffs_format_reldev(FAR struct yaffs_dev *dev, int unmount_flag,
                               int force_unmount_flag, int remount_flag);

extern void yaffs_remove_device(FAR struct yaffs_dev *dev);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: yaffs_vfs_open
 ****************************************************************************/

static int yaffs_vfs_open(FAR struct file *filep,
                          FAR const char *relpath, int oflags, mode_t mode)
{
  FAR struct yaffs_dev *dev;
  FAR struct yaffs_file_s *priv;
  int handle;
  off_t pos;

  dev = mountpt_to_yaffs_dev(filep->f_inode);

  /* Allocate memory for the open file */

  priv = kmm_zalloc(sizeof(*priv));
  if (priv == NULL)
    {
      return -ENOMEM;
    }

  handle = yaffs_open_reldev(dev, relpath, oflags, mode);
  if (handle < 0)
    {
      goto errout;
    }

  strlcpy(priv->path, relpath, sizeof(priv->path));

  /* In append mode, need to set the file pointer to end of the file */

  if (filep->f_oflags & O_APPEND)
    {
      pos = yaffs_lseek(handle, 0, SEEK_END);
      if (pos < 0)
        {
          goto errout_with_file;
        }

      filep->f_pos = pos;
    }

  priv->handle = handle;

  /* Attach the private date to the struct file instance */

  filep->f_priv = priv;
  return 0;

errout_with_file:
  yaffs_close(handle);
errout:
  kmm_free(priv);

  return yaffsfs_GetLastError();
}

/****************************************************************************
 * Name: yaffs_vfs_close
 ****************************************************************************/

static int yaffs_vfs_close(FAR struct file *filep)
{
  FAR struct yaffs_file_s *priv;
  int ret;

  priv = filep->f_priv;
  ret = yaffs_close(priv->handle);
  if (ret >= 0)
    {
      kmm_free(priv);
      return 0;
    }

  return yaffsfs_GetLastError();
}

/****************************************************************************
 * Name: yaffs_vfs_read
 ****************************************************************************/

static ssize_t yaffs_vfs_read(FAR struct file *filep,
                              FAR char *buffer, size_t buflen)
{
  int handle;
  int totalread;

  handle = filep_to_yaffs_handle(filep);
  totalread = yaffs_read(handle, buffer, buflen);
  if (totalread >= 0)
    {
      filep->f_pos += totalread;
      return totalread;
    }

  return yaffsfs_GetLastError();
}

/****************************************************************************
 * Name: yaffs_vfs_write
 ****************************************************************************/

static ssize_t yaffs_vfs_write(FAR struct file *filep,
                               const char *buffer, size_t buflen)
{
  int handle;
  int totalwritten;

  handle = filep_to_yaffs_handle(filep);
  totalwritten = yaffs_write(handle, buffer, buflen);
  if (totalwritten >= 0)
    {
      filep->f_pos += totalwritten;
      return totalwritten;
    }

  return yaffsfs_GetLastError();
}

/****************************************************************************
 * Name: yaffs_vfs_seek
 ****************************************************************************/

static off_t yaffs_vfs_seek(FAR struct file *filep, off_t offset, int whence)
{
  int handle;
  off_t pos;

  handle = filep_to_yaffs_handle(filep);
  pos = yaffs_lseek(handle, offset, whence);
  if (pos >= 0)
    {
      filep->f_pos = pos;
      return pos;
    }

  return yaffsfs_GetLastError();
}

/****************************************************************************
 * Name: yaffs_vfs_ioctl
 ****************************************************************************/

static int yaffs_vfs_ioctl(FAR struct file *filep, int cmd,
                           unsigned long arg)
{
  FAR struct yaffs_mountpt_s *fs;
  FAR struct yaffs_file_s *priv;
  FAR struct inode *inode;
  FAR struct inode *drv;
  int ret;

  /* Recover our private data from the struct file instance */

  priv  = filep->f_priv;
  inode = filep->f_inode;
  fs    = inode->i_private;
  drv   = fs->drv;

  switch (cmd)
    {
      case FIOC_FILEPATH:
        {
          FAR char *path = (FAR char *)(uintptr_t)arg;
          ret = inode_getpath(inode, path);
          if (ret >= 0)
            {
              strcat(path, priv->path);
            }
        }
        break;

      default:
        {
          ret = MTD_IOCTL(drv->u.i_mtd, cmd, arg);
        }
    }

  return ret;
}

/****************************************************************************
 * Name: yaffs_vfs_sync
 *
 * Description:
 *  Synchronize the file state on disk to match internal, in-memory state.
 *
 ****************************************************************************/

static int yaffs_vfs_sync(FAR struct file *filep)
{
  int handle;
  int ret;

  handle = filep_to_yaffs_handle(filep);
  ret = yaffs_fsync(handle);

  return ret < 0 ? yaffsfs_GetLastError() : 0;
}

/****************************************************************************
 * Name: yaffs_vfs_dup
 *
 * Description: Duplicate open file data in the new file structure.
 *
 ****************************************************************************/

static int yaffs_vfs_dup(FAR const struct file *oldp, FAR struct file *newp)
{
  FAR struct yaffs_file_s *oldpriv;
  FAR struct yaffs_file_s *newpriv;
  int newhandle;

  /* Allocate memory for the new file */

  newpriv = kmm_zalloc(sizeof(*newpriv));
  if (newpriv == NULL)
    {
      return -ENOMEM;
    }

  oldpriv = oldp->f_priv;
  newhandle = yaffs_dup(oldpriv->handle);
  if (newhandle < 0)
    {
      goto errout;
    }

  newpriv->handle = newhandle;
  strlcpy(newpriv->path, oldpriv->path, sizeof(newpriv->path));

  /* Attach the private date to the new struct file instance */

  newp->f_priv = newpriv;
  return 0;

errout:
  kmm_free(newpriv);
  return yaffsfs_GetLastError();
}

/****************************************************************************
 * Name: yaffs_vfs_fstat
 *
 * Description:
 *   Obtain information about an open file associated with the file
 *   descriptor 'fd', and will write it to the area pointed to by 'buf'.
 *
 ****************************************************************************/

static int yaffs_vfs_fstat(FAR const struct file *filep,
                           FAR struct stat *buf)
{
  struct yaffs_stat buffer;
  int handle;
  int ret;

  handle = filep_to_yaffs_handle(filep);
  ret = yaffs_fstat(handle, &buffer);
  if (ret < 0)
    {
      goto errout;
    }

  memset(buf, 0, sizeof(*buf));
  buf->st_dev          = buffer.st_dev;
  buf->st_ino          = buffer.st_ino;
  buf->st_mode         = buffer.st_mode;
  buf->st_nlink        = buffer.st_nlink;
  buf->st_uid          = buffer.st_uid;
  buf->st_gid          = buffer.st_gid;
  buf->st_rdev         = buffer.st_rdev;
  buf->st_size         = buffer.st_size;
  buf->st_atim.tv_sec  = buffer.yst_atime / 1000000000ull;
  buf->st_atim.tv_nsec = buffer.yst_atime % 1000000000ull;
  buf->st_mtim.tv_sec  = buffer.yst_mtime / 1000000000ull;
  buf->st_mtim.tv_nsec = buffer.yst_mtime % 1000000000ull;
  buf->st_ctim.tv_sec  = buffer.yst_ctime / 1000000000ull;
  buf->st_ctim.tv_nsec = buffer.yst_ctime % 1000000000ull;

  return 0;

errout:
  return yaffsfs_GetLastError();
}

/****************************************************************************
 * Name: yaffs_vfs_fchstat
 ****************************************************************************/

static int yaffs_vfs_fchstat(FAR const struct file *filep,
                             FAR const struct stat *buf, int flags)
{
  struct yaffs_stat buffer;
  struct yaffs_utimbuf timbuf;
  int handle;
  int ret = 0;

  handle = filep_to_yaffs_handle(filep);

  if ((flags & CH_STAT_GID) || (flags & CH_STAT_UID))
    {
      return -EPERM;
    }

  if (flags & CH_STAT_MODE)
    {
      ret = yaffs_fchmod(handle, buf->st_mode);
      if (ret < 0)
        {
          goto out;
        }
    }

  if (flags & (CH_STAT_ATIME | CH_STAT_MTIME))
    {
      if ((flags & CH_STAT_ATIME) != (flags & CH_STAT_MTIME))
        {
          ret = yaffs_fstat(handle, &buffer);
          if (ret < 0)
            {
              goto out;
            }

          if (flags & CH_STAT_ATIME)
            {
              timbuf.actime = buf->st_atim.tv_sec * 1000000000ull
                        + buf->st_atim.tv_nsec;
              timbuf.modtime = buffer.yst_mtime;
            }
          else
            {
              timbuf.actime = buffer.yst_atime;
              timbuf.modtime = buf->st_mtim.tv_sec * 1000000000ull
                        + buf->st_mtim.tv_nsec;
            }
        }
      else
        {
          timbuf.actime = buf->st_atim.tv_sec * 1000000000ull
                    + buf->st_atim.tv_nsec;
          timbuf.modtime = buf->st_mtim.tv_sec * 1000000000ull
                    + buf->st_mtim.tv_nsec;
        }

      ret = yaffs_futime(handle, &timbuf);
    }

out:
  return ret < 0 ? yaffsfs_GetLastError() : 0;
}

/****************************************************************************
 * Name: yaffs_vfs_truncate
 *
 * Description:
 *   Set the length of the open, regular file associated with the file
 *   structure 'filep' to 'length'.
 *
 ****************************************************************************/

static int yaffs_vfs_truncate(FAR struct file *filep, off_t length)
{
  int handle;
  int ret;

  handle = filep_to_yaffs_handle(filep);
  ret = yaffs_ftruncate(handle, length);

  return ret < 0 ? yaffsfs_GetLastError() : 0;
}

/****************************************************************************
 * Name: yaffs_vfs_opendir
 *
 * Description: Open a directory for read access
 *
 ****************************************************************************/

static int yaffs_vfs_opendir(FAR struct inode *mountpt,
                             FAR const char *relpath,
                             FAR struct fs_dirent_s **dir)
{
  FAR struct yaffs_dev *dev;
  FAR struct yaffs_dir_s *ydir;
  FAR yaffs_DIR *dirp;

  dev = mountpt_to_yaffs_dev(mountpt);

  /* Allocate memory for the open directory */

  ydir = kmm_zalloc(sizeof(*ydir));
  if (ydir == NULL)
    {
      return -ENOMEM;
    }

  dirp = yaffs_opendir_reldev(dev, relpath);
  if (dirp == NULL)
    {
      goto errout;
    }

  ydir->dir = dirp;
  *dir = &ydir->base;

  return 0;

errout:
  kmm_free(ydir);
  return yaffsfs_GetLastError();
}

/****************************************************************************
 * Name: yaffs_vfs_closedir
 *
 * Description: Close a directory
 *
 ****************************************************************************/

static int yaffs_vfs_closedir(FAR struct inode *mountpt,
                              FAR struct fs_dirent_s *dir)
{
  FAR struct yaffs_dir_s *ydir;
  int ret;

  ydir = (FAR struct yaffs_dir_s *)dir;
  ret = yaffs_closedir(ydir->dir);
  if (ret >= 0)
    {
      kmm_free(ydir);
      return 0;
    }

  return yaffsfs_GetLastError();
}

/****************************************************************************
 * Name: yaffs_vfs_readdir
 *
 * Description: Read the next directory entry
 *
 ****************************************************************************/

static int yaffs_vfs_readdir(FAR struct inode *mountpt,
                             FAR struct fs_dirent_s *dir,
                             FAR struct dirent *entry)
{
  FAR yaffs_DIR *dirp;
  FAR struct yaffs_dirent *dirent;
  int ret = 0;

  dirp = dir_to_yaffs_dir(dir);
  dirent = yaffs_readdir(dirp);
  if (dirent == NULL)
    {
      ret = yaffsfs_GetLastError();
      if (ret == 0)
        {
          ret = -ENOENT;
        }

      goto out;
    }

  entry->d_type = dirent->d_type;
  strlcpy(entry->d_name, dirent->d_name, sizeof(entry->d_name));

out:
  return ret;
}

/****************************************************************************
 * Name: yaffs_vfs_rewinddir
 *
 * Description: Reset directory read to the first entry
 *
 ****************************************************************************/

static int yaffs_vfs_rewinddir(FAR struct inode *mountpt,
                               FAR struct fs_dirent_s *dir)
{
  yaffs_rewinddir(dir_to_yaffs_dir(dir));
  return 0;
}

/****************************************************************************
 * Name: yaffs_vfs_statfs
 *
 * Description: Return filesystem statistics
 *
 ****************************************************************************/

static int yaffs_vfs_statfs(FAR struct inode *mountpt,
                            FAR struct statfs *buf)
{
  FAR struct yaffs_mountpt_s *fs;
  off_t size;

  fs = (FAR struct yaffs_mountpt_s *)(mountpt->i_private);
  size = yaffs_freespace_reldev(fs->dev);
  if (size < 0)
    {
      goto errout;
    }

  /* Return something for the file system description */

  memset(buf, 0, sizeof(*buf));
  buf->f_type    = YAFFS_SUPER_MAGIC;
  buf->f_namelen = YAFFS_MAX_NAME_LENGTH;
  buf->f_bsize   = fs->geo.erasesize;
  buf->f_blocks  = fs->geo.neraseblocks - CONFIG_FS_YAFFS_RESERVED_BLOCKS;
  buf->f_bfree   = size / fs->geo.erasesize;
  buf->f_bavail  = buf->f_bfree;

  return OK;

errout:
  return yaffsfs_GetLastError();
}

/****************************************************************************
 * Name: yaffs_vfs_unlink
 *
 * Description: Remove a file
 *
 ****************************************************************************/

static int yaffs_vfs_unlink(FAR struct inode *mountpt,
                            FAR const char *relpath)
{
  FAR struct yaffs_dev *dev;
  int ret;

  dev = mountpt_to_yaffs_dev(mountpt);
  ret = yaffs_unlink_reldev(dev, relpath);

  return ret < 0 ? yaffsfs_GetLastError() : 0;
}

/****************************************************************************
 * Name: yaffs_vfs_mkdir
 *
 * Description: Create a directory
 *
 ****************************************************************************/

static int yaffs_vfs_mkdir(FAR struct inode *mountpt,
                           FAR const char *relpath, mode_t mode)
{
  FAR struct yaffs_dev *dev;
  int ret;

  dev = mountpt_to_yaffs_dev(mountpt);
  ret = yaffs_mkdir_reldev(dev, relpath, mode);

  return ret < 0 ? yaffsfs_GetLastError() : 0;
}

/****************************************************************************
 * Name: yaffs_vfs_rmdir
 *
 * Description: Remove a directory
 *
 ****************************************************************************/

static int yaffs_vfs_rmdir(FAR struct inode *mountpt,
                           FAR const char *relpath)
{
  FAR struct yaffs_dev *dev;
  int ret;

  dev = mountpt_to_yaffs_dev(mountpt);
  ret = yaffs_rmdir_reldev(dev, relpath);

  return ret < 0 ? yaffsfs_GetLastError() : 0;
}

/****************************************************************************
 * Name: yaffs_vfs_rename
 *
 * Description: Rename a file or directory
 *
 ****************************************************************************/

static int yaffs_vfs_rename(FAR struct inode *mountpt,
                            FAR const char *oldrelpath,
                            FAR const char *newrelpath)
{
  FAR struct yaffs_dev *dev;
  int ret;

  dev = mountpt_to_yaffs_dev(mountpt);
  ret = yaffs_rename_reldev(dev, oldrelpath, newrelpath);

  return ret < 0 ? yaffsfs_GetLastError() : 0;
}

/****************************************************************************
 * Name: yaffs_vfs_stat
 *
 * Description: Return information about a file or directory
 *
 ****************************************************************************/

static int yaffs_vfs_stat(FAR struct inode *mountpt, FAR const char *relpath,
                          FAR struct stat *buf)
{
  FAR struct yaffs_dev *dev;
  struct yaffs_stat buffer;
  int ret;

  dev = mountpt_to_yaffs_dev(mountpt);
  ret = yaffs_stat_reldev(dev, relpath, &buffer);
  if (ret < 0)
    {
      goto errout;
    }

  memset(buf, 0, sizeof(*buf));
  buf->st_dev          = buffer.st_dev;
  buf->st_ino          = buffer.st_ino;
  buf->st_mode         = buffer.st_mode;
  buf->st_nlink        = buffer.st_nlink;
  buf->st_uid          = buffer.st_uid;
  buf->st_gid          = buffer.st_gid;
  buf->st_rdev         = buffer.st_rdev;
  buf->st_size         = buffer.st_size;
  buf->st_atim.tv_sec  = buffer.yst_atime / 1000000000ull;
  buf->st_atim.tv_nsec = buffer.yst_atime % 1000000000ull;
  buf->st_mtim.tv_sec  = buffer.yst_mtime / 1000000000ull;
  buf->st_mtim.tv_nsec = buffer.yst_mtime % 1000000000ull;
  buf->st_ctim.tv_sec  = buffer.yst_ctime / 1000000000ull;
  buf->st_ctim.tv_nsec = buffer.yst_ctime % 1000000000ull;

  return 0;

errout:
  return yaffsfs_GetLastError();
}

/****************************************************************************
 * Name: yaffs_vfs_chstat
 ****************************************************************************/

static int yaffs_vfs_chstat(FAR struct inode *mountpt,
                            FAR const char *relpath,
                            FAR const struct stat *buf, int flags)
{
  FAR struct yaffs_dev *dev;
  struct yaffs_stat buffer;
  struct yaffs_utimbuf timbuf;
  int ret = 0;

  dev = mountpt_to_yaffs_dev(mountpt);

  if ((flags & CH_STAT_GID) || (flags & CH_STAT_UID))
    {
      return -EPERM;
    }

  if (flags & CH_STAT_MODE)
    {
      ret = yaffs_chmod_reldev(dev, relpath, buf->st_mode);
      if (ret < 0)
        {
          goto out;
        }
    }

  if (flags & (CH_STAT_ATIME | CH_STAT_MTIME))
    {
      if ((flags & CH_STAT_ATIME) != (flags & CH_STAT_MTIME))
        {
          ret = yaffs_stat_reldev(dev, relpath, &buffer);
          if (ret < 0)
            {
              goto out;
            }

          if (flags & CH_STAT_ATIME)
            {
              timbuf.actime = buf->st_atim.tv_sec * 1000000000ull
                        + buf->st_atim.tv_nsec;
              timbuf.modtime = buffer.yst_mtime;
            }
          else
            {
              timbuf.actime = buffer.yst_atime;
              timbuf.modtime = buf->st_mtim.tv_sec * 1000000000ull
                        + buf->st_mtim.tv_nsec;
            }
        }
      else
        {
          timbuf.actime = buf->st_atim.tv_sec * 1000000000ull
                    + buf->st_atim.tv_nsec;
          timbuf.modtime = buf->st_mtim.tv_sec * 1000000000ull
                    + buf->st_mtim.tv_nsec;
        }

      ret = yaffs_utime_reldev(dev, relpath, &timbuf);
    }

out:
  return ret < 0 ? yaffsfs_GetLastError() : 0;
}

/****************************************************************************
 * Name: yaffs_initialise_fn
 ****************************************************************************/

static int yaffs_initialise_fn(FAR struct yaffs_dev *dev)
{
  return YAFFS_OK;
}

/****************************************************************************
 * Name: yaffs_deinitialise_fn
 ****************************************************************************/

static int yaffs_deinitialise_fn(FAR struct yaffs_dev *dev)
{
  return YAFFS_OK;
}

/****************************************************************************
 * Name: yaffs_read_chunk_fn
 ****************************************************************************/

static int yaffs_read_chunk_fn(FAR struct yaffs_dev *dev, int nand_chunk,
                               FAR u8 *data, int data_len,
                               FAR u8 *oob, int oob_len,
                               FAR enum yaffs_ecc_result *ecc_result)
{
  FAR struct mtd_dev_s *mtd;
  int nchunk;
  ssize_t retval;
  enum yaffs_ecc_result ecc_status;
  int ret = YAFFS_OK;

  mtd = yaffs_dev_to_mtd(dev);
  nchunk = data_len / dev->param.total_bytes_per_chunk;

  if (data && data_len > 0)
    {
      retval = MTD_BREAD(mtd, nand_chunk, nchunk, data);
      if (retval == nchunk)
        {
          ecc_status = YAFFS_ECC_RESULT_NO_ERROR;
        }
      else
        {
          ret = YAFFS_FAIL;

          if (retval == -EUCLEAN)
            {
              ecc_status = YAFFS_ECC_RESULT_FIXED;
            }
          else if (retval == -EBADMSG)
            {
              ecc_status = YAFFS_ECC_RESULT_UNFIXED;
            }
          else
            {
              ecc_status = YAFFS_ECC_RESULT_UNKNOWN;
            }
        }
    }
  else
    {
      ecc_status = YAFFS_ECC_RESULT_UNKNOWN;
    }

  if (ecc_result)
    {
      *ecc_result = ecc_status;
    }

  if (oob && oob_len > 0)
    {
      ferr("### read oob? inband tag should not read oob!\n");
    }

  return ret;
}

/****************************************************************************
 * Name: yaffs_write_chunk_fn
 ****************************************************************************/

static int yaffs_write_chunk_fn(FAR struct yaffs_dev *dev, int nand_chunk,
                                FAR const u8 *data, int data_len,
                                FAR const u8 *oob, int oob_len)
{
  FAR struct mtd_dev_s *mtd;
  int nchunk;
  ssize_t retval;
  int ret = YAFFS_OK;

  mtd = yaffs_dev_to_mtd(dev);
  nchunk = data_len / dev->param.total_bytes_per_chunk;

  if (data && data_len > 0)
    {
      retval = MTD_BWRITE(mtd, nand_chunk, nchunk, data);
      if (retval != nchunk)
        {
          ret = YAFFS_FAIL;
        }
    }

  if (oob && oob_len > 0)
    {
      ferr("### write oob? inband tag should not read oob!\n");
    }

  return ret;
}

/****************************************************************************
 * Name: yaffs_erase_block_fn
 ****************************************************************************/

static int yaffs_erase_block_fn(FAR struct yaffs_dev *dev, int block_no)
{
  FAR struct mtd_dev_s *mtd = yaffs_dev_to_mtd(dev);
  return MTD_ERASE(mtd, block_no, 1) == 1 ? YAFFS_OK : YAFFS_FAIL;
}

/****************************************************************************
 * Name: yaffs_mark_block_bad_fn
 ****************************************************************************/

static int yaffs_mark_block_bad_fn(FAR struct yaffs_dev *dev, int block_no)
{
  FAR struct mtd_dev_s *mtd = yaffs_dev_to_mtd(dev);
  return MTD_MARKBAD(mtd, block_no) == 0 ? YAFFS_OK : YAFFS_FAIL;
}

/****************************************************************************
 * Name: yaffs_check_block_bad_fn
 ****************************************************************************/

static int yaffs_check_block_bad_fn(FAR struct yaffs_dev *dev, int block_no)
{
  FAR struct mtd_dev_s *mtd = yaffs_dev_to_mtd(dev);
  return MTD_ISBAD(mtd, block_no) == 0 ? YAFFS_OK : YAFFS_FAIL;
}

/****************************************************************************
 * Name: yaffs_vfs_bind
 *
 * Description: This implements a portion of the mount operation. This
 *  function allocates and initializes the mountpoint private data and
 *  binds the driver inode to the filesystem private data. The final
 *  binding of the private data (containing the driver) to the
 *  mountpoint is performed by mount().
 *
 ****************************************************************************/

static int yaffs_vfs_bind(FAR struct inode *driver, FAR const void *data,
                          FAR void **handle)
{
  FAR struct yaffs_mountpt_s *fs;
  FAR struct yaffs_dev *dev;
  FAR struct yaffs_param *p;
  FAR struct yaffs_driver *d;
  FAR struct mtd_dev_s *mtd;
  int ret;

  DEBUGASSERT(INODE_IS_MTD(driver) && driver->u.i_mtd != NULL);
  mtd = driver->u.i_mtd;

  /* Create an instance of the mountpt state structure */

  fs = kmm_zalloc(sizeof(*fs));
  if (!fs)
    {
      return -ENOMEM;
    }

  /* Initialize the allocated mountpt state structure. The filesystem is
   * responsible for one reference on the driver inode and does not
   * have to addref() here (but does have to release in unbind().
   */

  fs->drv = driver; /* Save the driver reference */
  fs->mtd = mtd;

  ret = MTD_IOCTL(mtd, MTDIOC_GEOMETRY, (unsigned long)&fs->geo);
  if (ret < 0)
    {
      goto errout_with_fs;
    }

  /* Initialize yaffs_dev structure */

  dev = kmm_zalloc(sizeof(*dev));
  if (!dev)
    {
      ret = -ENOMEM;
      goto errout_with_fs;
    }

  dev->driver_context = mtd;

  p = &dev->param;
  p->name = strdup(mtd->name);
  if (p->name == NULL)
    {
      ret = -ENOMEM;
      goto errout_with_dev;
    }

  p->is_yaffs2 = 1;
  p->use_nand_ecc = 1;
  p->inband_tags = 1;
  p->n_reserved_blocks = CONFIG_FS_YAFFS_RESERVED_BLOCKS;
  p->n_caches = CONFIG_FS_YAFFS_CHUNK_CACHE;
  p->refresh_period = CONFIG_FS_YAFFS_REFRESH_PERIOD;
#ifdef CONFIG_FS_YAFFS_CACHE_BYPASS_ALIGNED
  p->cache_bypass_aligned = 1;
#endif
#ifdef CONFIG_FS_YAFFS_SKIP_READ_CHECKPOINT
  p->skip_checkpt_rd = 1;
#endif
#ifdef CONFIG_FS_YAFFS_SKIP_WRITE_CHECKPOINT
  p->skip_checkpt_wr = 1;
#endif
#ifdef CONFIG_FS_YAFFS_EMPTY_LOST_FOUND
  p->empty_lost_n_found = 1;
#endif
  p->start_block = 0;
  p->end_block = fs->geo.neraseblocks - 1;
  p->total_bytes_per_chunk = fs->geo.blocksize;
  p->chunks_per_block = fs->geo.erasesize / fs->geo.blocksize;

  d = &dev->drv;
  d->drv_initialise_fn = yaffs_initialise_fn;
  d->drv_deinitialise_fn = yaffs_deinitialise_fn;
  d->drv_read_chunk_fn = yaffs_read_chunk_fn;
  d->drv_write_chunk_fn = yaffs_write_chunk_fn;
  d->drv_erase_fn = yaffs_erase_block_fn;
  d->drv_check_bad_fn = yaffs_check_block_bad_fn;
  d->drv_mark_bad_fn = yaffs_mark_block_bad_fn;

  fs->dev = dev;

  yaffs_add_device(dev);

  /* Then get information about the yaffs filesystem on the devices
   * managed by this driver.
   */

  /* Force format the device if -o forceformat */

  if (data && strcmp(data, "forceformat") == 0)
    {
      ret = yaffs_format_reldev(dev, 1, 1, 0);
      if (ret < 0)
        {
          goto errout_with_name;
        }
    }

  ret = yaffs_mount_reldev(dev);
  if (ret < 0)
    {
      /* Auto format the device if -o autoformat */

      if (ret != -EFAULT || !data || strcmp(data, "autoformat"))
        {
          goto errout_with_name;
        }

      ret = yaffs_format_reldev(dev, 1, 1, 1);
      if (ret < 0)
        {
          goto errout_with_name;
        }
    }

  *handle = fs;
  return OK;

errout_with_name:
  kmm_free((FAR void *)(p->name));
  yaffs_remove_device(dev);
errout_with_dev:
  kmm_free(dev);
errout_with_fs:
  kmm_free(fs);
  return ret;
}

/****************************************************************************
 * Name: yaffs_vfs_unbind
 *
 * Description: This implements the filesystem portion of the umount
 *  operation.
 *
 ****************************************************************************/

static int yaffs_vfs_unbind(FAR void *handle, FAR struct inode **driver,
                            unsigned int flags)
{
  FAR struct yaffs_mountpt_s *fs;
  FAR struct yaffs_dev *dev;
  FAR struct inode *drv;
  int ret;

  fs = handle;
  dev = fs->dev;
  drv = fs->drv;

  /* Unmount */

  ret = yaffs_unmount(dev->param.name);
  if (ret >= 0)
    {
      /* Remove and release dev */

      yaffs_remove_device(dev);
      kmm_free((FAR void *)(dev->param.name));
      kmm_free(dev);

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

      kmm_free(fs);
    }

  return ret;
}
