/****************************************************************************
 * fs/rpmsgfs/rpmsgfs_server.c
 *
 * SPDX-License-Identifier: Apache-2.0
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
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <assert.h>
#include <nuttx/debug.h>
#include <errno.h>

#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/fs/fs.h>
#include <nuttx/rpmsg/rpmsg.h>

#include "rpmsgfs.h"
#include "fs_heap.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct rpmsgfs_server_s
{
  struct rpmsg_endpoint ept;
  FAR struct file     **files;
  FAR void            **dirs;
  int                   file_rows;
  int                   dir_nums;
  mutex_t               lock;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int rpmsgfs_open_handler(FAR struct rpmsg_endpoint *ept,
                                FAR void *data, size_t len,
                                uint32_t src, FAR void *priv);
static int rpmsgfs_close_handler(FAR struct rpmsg_endpoint *ept,
                                 FAR void *data, size_t len,
                                 uint32_t src, FAR void *priv);
static int rpmsgfs_read_handler(FAR struct rpmsg_endpoint *ept,
                                FAR void *data, size_t len,
                                uint32_t src, FAR void *priv);
static int rpmsgfs_write_handler(FAR struct rpmsg_endpoint *ept,
                                 FAR void *data, size_t len,
                                 uint32_t src, FAR void *priv);
static int rpmsgfs_lseek_handler(FAR struct rpmsg_endpoint *ept,
                                 FAR void *data, size_t len,
                                 uint32_t src, FAR void *priv);
static int rpmsgfs_ioctl_handler(FAR struct rpmsg_endpoint *ept,
                                 FAR void *data, size_t len,
                                 uint32_t src, FAR void *priv);
static int rpmsgfs_sync_handler(FAR struct rpmsg_endpoint *ept,
                                FAR void *data, size_t len,
                                uint32_t src, FAR void *priv);
static int rpmsgfs_dup_handler(FAR struct rpmsg_endpoint *ept,
                               FAR void *data, size_t len,
                               uint32_t src, FAR void *priv);
static int rpmsgfs_fstat_handler(FAR struct rpmsg_endpoint *ept,
                                 FAR void *data, size_t len,
                                 uint32_t src, FAR void *priv);
static int rpmsgfs_ftruncate_handler(FAR struct rpmsg_endpoint *ept,
                                     FAR void *data, size_t len,
                                     uint32_t src, FAR void *priv);
static int rpmsgfs_opendir_handler(FAR struct rpmsg_endpoint *ept,
                                   FAR void *data, size_t len,
                                   uint32_t src, FAR void *priv);
static int rpmsgfs_readdir_handler(FAR struct rpmsg_endpoint *ept,
                                   FAR void *data, size_t len,
                                   uint32_t src, FAR void *priv);
static int rpmsgfs_rewinddir_handler(FAR struct rpmsg_endpoint *ept,
                                     FAR void *data, size_t len,
                                     uint32_t src, FAR void *priv);
static int rpmsgfs_closedir_handler(FAR struct rpmsg_endpoint *ept,
                                    FAR void *data, size_t len,
                                    uint32_t src, FAR void *priv);
static int rpmsgfs_statfs_handler(FAR struct rpmsg_endpoint *ept,
                                  FAR void *data, size_t len,
                                  uint32_t src, FAR void *priv);
static int rpmsgfs_unlink_handler(FAR struct rpmsg_endpoint *ept,
                                  FAR void *data, size_t len,
                                  uint32_t src, FAR void *priv);
static int rpmsgfs_mkdir_handler(FAR struct rpmsg_endpoint *ept,
                                 FAR void *data, size_t len,
                                 uint32_t src, FAR void *priv);
static int rpmsgfs_rmdir_handler(FAR struct rpmsg_endpoint *ept,
                                 FAR void *data, size_t len,
                                 uint32_t src, FAR void *priv);
static int rpmsgfs_rename_handler(FAR struct rpmsg_endpoint *ept,
                                  FAR void *data, size_t len,
                                  uint32_t src, FAR void *priv);
static int rpmsgfs_stat_handler(FAR struct rpmsg_endpoint *ept,
                                FAR void *data, size_t len,
                                uint32_t src, FAR void *priv);
static int rpmsgfs_fchstat_handler(FAR struct rpmsg_endpoint *ept,
                                   FAR void *data, size_t len,
                                   uint32_t src, FAR void *priv);
static int rpmsgfs_chstat_handler(FAR struct rpmsg_endpoint *ept,
                                  FAR void *data, size_t len,
                                  uint32_t src, FAR void *priv);
#ifdef CONFIG_FS_LINKS
static int rpmsgfs_symlink_handler(FAR struct rpmsg_endpoint *ept,
                                   FAR void *data, size_t len,
                                   uint32_t src, FAR void *priv);
static int rpmsgfs_readlink_handler(FAR struct rpmsg_endpoint *ept,
                                    FAR void *data, size_t len,
                                    uint32_t src, FAR void *priv);
static int rpmsgfs_link_handler(FAR struct rpmsg_endpoint *ept,
                                FAR void *data, size_t len,
                                uint32_t src, FAR void *priv);
static int rpmsgfs_lstat_handler(FAR struct rpmsg_endpoint *ept,
                                 FAR void *data, size_t len,
                                 uint32_t src, FAR void *priv);
#endif
static bool rpmsgfs_ns_match(FAR struct rpmsg_device *rdev,
                             FAR void *priv_, FAR const char *name,
                             uint32_t dest);
static void rpmsgfs_ns_bind(FAR struct rpmsg_device *rdev,
                            FAR void *priv_, FAR const char *name,
                            uint32_t dest);
static int  rpmsgfs_ept_cb(FAR struct rpmsg_endpoint *ept,
                           FAR void *data, size_t len, uint32_t src,
                           FAR void *priv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const rpmsg_ept_cb g_rpmsgfs_handler[] =
{
  [RPMSGFS_OPEN]      = rpmsgfs_open_handler,
  [RPMSGFS_CLOSE]     = rpmsgfs_close_handler,
  [RPMSGFS_READ]      = rpmsgfs_read_handler,
  [RPMSGFS_WRITE]     = rpmsgfs_write_handler,
  [RPMSGFS_LSEEK]     = rpmsgfs_lseek_handler,
  [RPMSGFS_IOCTL]     = rpmsgfs_ioctl_handler,
  [RPMSGFS_SYNC]      = rpmsgfs_sync_handler,
  [RPMSGFS_DUP]       = rpmsgfs_dup_handler,
  [RPMSGFS_FSTAT]     = rpmsgfs_fstat_handler,
  [RPMSGFS_FTRUNCATE] = rpmsgfs_ftruncate_handler,
  [RPMSGFS_OPENDIR]   = rpmsgfs_opendir_handler,
  [RPMSGFS_READDIR]   = rpmsgfs_readdir_handler,
  [RPMSGFS_REWINDDIR] = rpmsgfs_rewinddir_handler,
  [RPMSGFS_CLOSEDIR]  = rpmsgfs_closedir_handler,
  [RPMSGFS_STATFS]    = rpmsgfs_statfs_handler,
  [RPMSGFS_UNLINK]    = rpmsgfs_unlink_handler,
  [RPMSGFS_MKDIR]     = rpmsgfs_mkdir_handler,
  [RPMSGFS_RMDIR]     = rpmsgfs_rmdir_handler,
  [RPMSGFS_RENAME]    = rpmsgfs_rename_handler,
  [RPMSGFS_STAT]      = rpmsgfs_stat_handler,
  [RPMSGFS_FCHSTAT]   = rpmsgfs_fchstat_handler,
  [RPMSGFS_CHSTAT]    = rpmsgfs_chstat_handler,
#ifdef CONFIG_FS_LINKS
  [RPMSGFS_SYMLINK]   = rpmsgfs_symlink_handler,
  [RPMSGFS_READLINK]  = rpmsgfs_readlink_handler,
  [RPMSGFS_LINK]      = rpmsgfs_link_handler,
  [RPMSGFS_LSTAT]     = rpmsgfs_lstat_handler,
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int rpmsgfs_alloc_file(FAR struct rpmsgfs_server_s *priv,
                              FAR struct file **filep)
{
  FAR struct file **tmp;
  int ret;
  int i;
  int j;

  nxmutex_lock(&priv->lock);

  for (i = 0; i < priv->file_rows; i++)
    {
      for (j = 0; j < CONFIG_NFILE_DESCRIPTORS_PER_BLOCK; j++)
        {
          if (priv->files[i][j].f_inode == NULL)
            {
              goto found;
            }
        }
    }

  tmp = fs_heap_realloc(priv->files, sizeof(FAR struct file *) * (i + 1));
  DEBUGASSERT(tmp);
  if (tmp == NULL)
    {
      ret = -ENFILE;
      goto out;
    }

  tmp[i] = fs_heap_zalloc(sizeof(struct file) *
                      CONFIG_NFILE_DESCRIPTORS_PER_BLOCK);
  DEBUGASSERT(tmp[i]);
  if (tmp[i] == NULL)
    {
      fs_heap_free(tmp);
      ret = -ENFILE;
      goto out;
    }

  priv->files = tmp;
  priv->file_rows++;

  j = 0;

found:
  priv->files[i][j].f_inode = (FAR struct inode *)-1;
  *filep = &priv->files[i][j];
  ret = i * CONFIG_NFILE_DESCRIPTORS_PER_BLOCK + j;

out:
  nxmutex_unlock(&priv->lock);
  return ret;
}

static FAR struct file *rpmsgfs_get_file(
                              FAR struct rpmsgfs_server_s *priv,
                              int fd)
{
  FAR struct file *filep;

  if (fd < 0 || fd >= priv->file_rows * CONFIG_NFILE_DESCRIPTORS_PER_BLOCK)
    {
      return NULL;
    }

  nxmutex_lock(&priv->lock);
  filep = &priv->files[fd / CONFIG_NFILE_DESCRIPTORS_PER_BLOCK]
                      [fd % CONFIG_NFILE_DESCRIPTORS_PER_BLOCK];
  nxmutex_unlock(&priv->lock);

  return filep;
}

static int rpmsgfs_attach_dir(FAR struct rpmsgfs_server_s *priv,
                              FAR void *dir)
{
  FAR void **tmp;
  int i;

  nxmutex_lock(&priv->lock);
  for (i = 1; i < priv->dir_nums; i++)
    {
      if (priv->dirs[i] == NULL)
        {
          priv->dirs[i] = dir;
          nxmutex_unlock(&priv->lock);
          return i;
        }
    }

  tmp = fs_heap_realloc(priv->dirs, sizeof(FAR void *) *
                    (priv->dir_nums + CONFIG_NFILE_DESCRIPTORS_PER_BLOCK));
  DEBUGASSERT(tmp);
  if (tmp == NULL)
    {
      nxmutex_unlock(&priv->lock);
      return -ENOMEM;
    }

  memset(&tmp[priv->dir_nums], 0, sizeof(FAR void *) *
         CONFIG_NFILE_DESCRIPTORS_PER_BLOCK);

  priv->dirs = tmp;
  priv->dir_nums += CONFIG_NFILE_DESCRIPTORS_PER_BLOCK;

  priv->dirs[i] = dir;
  nxmutex_unlock(&priv->lock);
  return i;
}

static void *rpmsgfs_detach_dir(FAR struct rpmsgfs_server_s *priv,
                                    int fd)
{
  FAR void *dir = NULL;

  if (fd >= 1 && fd < priv->dir_nums)
    {
      nxmutex_lock(&priv->lock);
      dir = priv->dirs[fd];
      priv->dirs[fd] = NULL;
      nxmutex_unlock(&priv->lock);
    }

  return dir;
}

static FAR void *rpmsgfs_get_dir(
                              FAR struct rpmsgfs_server_s *priv,
                              int fd)
{
  FAR void *dir = NULL;

  if (fd >= 1 && fd < priv->dir_nums)
    {
      nxmutex_lock(&priv->lock);
      dir = priv->dirs[fd];
      nxmutex_unlock(&priv->lock);
    }

  return dir;
}

static void rpmsgfs_get_statbuf(FAR struct stat *st,
                                FAR struct rpmsgfs_stat_priv_s *rst)
{
  st->st_dev           = rst->dev;
  st->st_ino           = rst->ino;
  st->st_mode          = rst->mode;
  st->st_nlink         = rst->nlink;
  st->st_uid           = rst->uid;
  st->st_gid           = rst->gid;
  st->st_rdev          = rst->rdev;
  st->st_size          = rst->size;
  st->st_atim.tv_sec   = rst->atim_sec;
  st->st_atim.tv_nsec  = rst->atim_nsec;
  st->st_mtim.tv_sec   = rst->mtim_sec;
  st->st_mtim.tv_nsec  = rst->mtim_nsec;
  st->st_ctim.tv_sec   = rst->ctim_sec;
  st->st_ctim.tv_nsec  = rst->ctim_nsec;
  st->st_blksize       = rst->blksize;
  st->st_blocks        = rst->blocks;
}

static void rpmsgfs_set_statbuf(FAR struct rpmsgfs_stat_priv_s *rst,
                                FAR const struct stat *st)
{
  rst->dev        = st->st_dev;
  rst->ino        = st->st_ino;
  rst->mode       = st->st_mode;
  rst->nlink      = st->st_nlink;
  rst->uid        = st->st_uid;
  rst->gid        = st->st_gid;
  rst->rdev       = st->st_rdev;
  rst->size       = st->st_size;
  rst->atim_sec   = st->st_atim.tv_sec;
  rst->atim_nsec  = st->st_atim.tv_nsec;
  rst->mtim_sec   = st->st_mtim.tv_sec;
  rst->mtim_nsec  = st->st_mtim.tv_nsec;
  rst->ctim_sec   = st->st_ctim.tv_sec;
  rst->ctim_nsec  = st->st_ctim.tv_nsec;
  rst->blksize    = st->st_blksize;
  rst->blocks     = st->st_blocks;
}

static int rpmsgfs_open_handler(FAR struct rpmsg_endpoint *ept,
                                FAR void *data, size_t len,
                                uint32_t src, FAR void *priv)
{
  FAR struct rpmsgfs_open_s *msg = data;
  FAR struct file *filep;
  int ret;
  int fd;

  ret = fd = rpmsgfs_alloc_file(priv, &filep);
  if (ret < 0)
    {
      goto out;
    }

  ret = file_open(filep, msg->pathname, msg->flags, msg->mode);
  if (ret < 0)
    {
      filep->f_inode = NULL;
    }

out:
  msg->header.result = ret < 0 ? ret : fd;
  return rpmsg_send(ept, msg, sizeof(*msg));
}

static int rpmsgfs_close_handler(FAR struct rpmsg_endpoint *ept,
                                 FAR void *data, size_t len,
                                 uint32_t src, FAR void *priv)
{
  FAR struct rpmsgfs_close_s *msg = data;
  FAR struct file *filep;
  int ret = -ENOENT;

  filep = rpmsgfs_get_file(priv, msg->fd);
  if (filep)
    {
      ret = file_close(filep);
    }

  msg->header.result = ret;
  return rpmsg_send(ept, msg, sizeof(*msg));
}

static int rpmsgfs_read_handler(FAR struct rpmsg_endpoint *ept,
                                FAR void *data, size_t len,
                                uint32_t src, FAR void *priv)
{
  FAR struct rpmsgfs_read_s *msg = data;
  FAR struct rpmsgfs_read_s *rsp;
  FAR struct file *filep;
  int ret = -ENOENT;
  size_t read = 0;
  uint32_t space;

  filep = rpmsgfs_get_file(priv, msg->fd);

  while (read < msg->count)
    {
      rsp = rpmsg_get_tx_payload_buffer(ept, &space, true);
      if (rsp == NULL)
        {
          return -ENOMEM;
        }

      *rsp = *msg;

      space -= sizeof(*msg);
      if (space > msg->count - read)
        {
          space = msg->count - read;
        }

      if (filep != NULL)
        {
          ret = file_read(filep, rsp->buf, space);
        }

      rsp->header.result = ret;
      if (rpmsg_send_nocopy(ept, rsp, (ret < 0 ? 0 : ret) + sizeof(*rsp))
          < 0)
        {
          rpmsg_release_tx_buffer(ept, rsp);
        }

      if (ret <= 0)
        {
          break;
        }

      read += ret;
    }

  return 0;
}

static int rpmsgfs_write_handler(FAR struct rpmsg_endpoint *ept,
                                 FAR void *data, size_t len,
                                 uint32_t src, FAR void *priv)
{
  FAR struct rpmsgfs_write_s *msg = data;
  FAR struct file *filep;
  int ret = -ENOENT;

  filep = rpmsgfs_get_file(priv, msg->fd);
  if (filep != NULL)
    {
      size_t written = 0;

      while (written < msg->count)
        {
          ret = file_write(filep, msg->buf + written, msg->count - written);
          if (ret < 0)
            {
              break;
            }

          written += ret;
        }
    }

  if (msg->header.cookie != 0)
    {
      msg->header.result = ret;
      rpmsg_send(ept, msg, sizeof(*msg));
    }

  return 0;
}

static int rpmsgfs_lseek_handler(FAR struct rpmsg_endpoint *ept,
                                 FAR void *data, size_t len,
                                 uint32_t src, FAR void *priv)
{
  FAR struct rpmsgfs_lseek_s *msg = data;
  FAR struct file *filep;
  int ret = -ENOENT;

  filep = rpmsgfs_get_file(priv, msg->fd);
  if (filep != NULL)
    {
      ret = file_seek(filep, msg->offset, msg->whence);
    }

  msg->header.result = ret;
  return rpmsg_send(ept, msg, sizeof(*msg));
}

static int rpmsgfs_ioctl_handler(FAR struct rpmsg_endpoint *ept,
                                 FAR void *data, size_t len,
                                 uint32_t src, FAR void *priv)
{
  FAR struct rpmsgfs_ioctl_s *msg = data;
  FAR struct file *filep;
  int ret = -ENOENT;

  filep = rpmsgfs_get_file(priv, msg->fd);
  if (filep != NULL)
    {
      ret = file_ioctl(filep, msg->request, msg->arglen > 0 ?
                       (unsigned long)msg->buf : msg->arg);
    }

  msg->header.result = ret;
  return rpmsg_send(ept, msg, len);
}

static int rpmsgfs_sync_handler(FAR struct rpmsg_endpoint *ept,
                                FAR void *data, size_t len,
                                uint32_t src, FAR void *priv)
{
  FAR struct rpmsgfs_sync_s *msg = data;
  FAR struct file *filep;
  int ret = -ENOENT;

  filep = rpmsgfs_get_file(priv, msg->fd);
  if (filep != NULL)
    {
      ret = file_fsync(filep);
    }

  msg->header.result = ret;
  return rpmsg_send(ept, msg, sizeof(*msg));
}

static int rpmsgfs_dup_handler(FAR struct rpmsg_endpoint *ept,
                               FAR void *data, size_t len,
                               uint32_t src, FAR void *priv)
{
  FAR struct rpmsgfs_dup_s *msg = data;
  FAR struct file *newfilep = NULL;
  FAR struct file *filep;
  int ret;
  int fd;

  filep = rpmsgfs_get_file(priv, msg->fd);
  ret = fd = rpmsgfs_alloc_file(priv, &newfilep);
  if (filep != NULL && ret >= 0)
    {
      ret = file_dup2(filep, newfilep);
      if (ret < 0)
        {
          file_close(newfilep);
        }
    }

  msg->header.result = ret < 0 ? ret : fd;
  return rpmsg_send(ept, msg, sizeof(*msg));
}

static int rpmsgfs_fstat_handler(FAR struct rpmsg_endpoint *ept,
                                 FAR void *data, size_t len,
                                 uint32_t src, FAR void *priv)
{
  FAR struct rpmsgfs_fstat_s *msg = data;
  FAR struct file *filep;
  int ret = -ENOENT;
  struct stat buf;

  filep = rpmsgfs_get_file(priv, msg->fd);
  if (filep != NULL)
    {
      ret = file_fstat(filep, &buf);
      if (ret >= 0)
        {
          rpmsgfs_set_statbuf(&msg->buf, &buf);
        }
    }

  msg->header.result = ret;
  return rpmsg_send(ept, msg, sizeof(*msg));
}

static int rpmsgfs_ftruncate_handler(FAR struct rpmsg_endpoint *ept,
                                     FAR void *data, size_t len,
                                     uint32_t src, FAR void *priv)
{
  FAR struct rpmsgfs_ftruncate_s *msg = data;
  FAR struct file *filep;
  int ret = -ENOENT;

  filep = rpmsgfs_get_file(priv, msg->fd);
  if (filep != NULL)
    {
      ret = file_truncate(filep, msg->length);
    }

  msg->header.result = ret;
  return rpmsg_send(ept, msg, sizeof(*msg));
}

static int rpmsgfs_opendir_handler(FAR struct rpmsg_endpoint *ept,
                                   FAR void *data, size_t len,
                                   uint32_t src, FAR void *priv)
{
  FAR struct rpmsgfs_opendir_s *msg = data;
  FAR void *dir;
  int ret = -ENOENT;

  dir = opendir(msg->pathname);
  if (dir)
    {
      ret = rpmsgfs_attach_dir(priv, dir);
      if (ret < 0)
        {
          closedir(dir);
        }
    }

  msg->header.result = ret;
  return rpmsg_send(ept, msg, sizeof(*msg));
}

static int rpmsgfs_readdir_handler(FAR struct rpmsg_endpoint *ept,
                                   FAR void *data, size_t len,
                                   uint32_t src, FAR void *priv)
{
  FAR struct rpmsgfs_readdir_s *msg = data;
  FAR struct dirent *entry;
  int ret = -ENOENT;
  FAR void *dir;
  size_t size;

  dir = rpmsgfs_get_dir(priv, msg->fd);
  if (dir)
    {
      entry = readdir(dir);
      if (entry)
        {
          size = MIN(rpmsg_get_tx_buffer_size(ept),
                     rpmsg_get_rx_buffer_size(ept));
          size = MIN(size - len, strlen(entry->d_name) + 1);
          msg->type = entry->d_type;
          strlcpy(msg->name, entry->d_name, size);
          msg->ino = entry->d_ino;
          len += size;
          ret = 0;
        }
    }

  msg->header.result = ret;
  return rpmsg_send(ept, msg, len);
}

static int rpmsgfs_rewinddir_handler(FAR struct rpmsg_endpoint *ept,
                                     FAR void *data, size_t len,
                                     uint32_t src, FAR void *priv)
{
  FAR struct rpmsgfs_rewinddir_s *msg = data;
  int ret = -ENOENT;
  FAR void *dir;

  dir = rpmsgfs_get_dir(priv, msg->fd);
  if (dir)
    {
      rewinddir(dir);
      ret = 0;
    }

  msg->header.result = ret;
  return rpmsg_send(ept, msg, sizeof(*msg));
}

static int rpmsgfs_closedir_handler(FAR struct rpmsg_endpoint *ept,
                                    FAR void *data, size_t len,
                                    uint32_t src, FAR void *priv)
{
  FAR struct rpmsgfs_closedir_s *msg = data;
  int ret = -ENOENT;
  FAR void *dir;

  dir = rpmsgfs_detach_dir(priv, msg->fd);
  if (dir)
    {
      ret = closedir(dir) ? -get_errno() : 0;
    }

  msg->header.result = ret;
  return rpmsg_send(ept, msg, sizeof(*msg));
}

static int rpmsgfs_statfs_handler(FAR struct rpmsg_endpoint *ept,
                                  FAR void *data, size_t len,
                                  uint32_t src, FAR void *priv)
{
  FAR struct rpmsgfs_statfs_s *msg = data;
  struct statfs buf;
  int ret;

  ret = statfs(msg->pathname, &buf);
  if (ret)
    {
      ret = -get_errno();
    }
  else
    {
      msg->type    = buf.f_type;
      msg->namelen = buf.f_namelen;
      msg->bsize   = buf.f_bsize;
      msg->blocks  = buf.f_blocks;
      msg->bfree   = buf.f_bfree;
      msg->bavail  = buf.f_bavail;
      msg->files   = buf.f_files;
      msg->ffree   = buf.f_ffree;
    }

  msg->header.result = ret;
  return rpmsg_send(ept, msg, sizeof(*msg));
}

static int rpmsgfs_unlink_handler(FAR struct rpmsg_endpoint *ept,
                                  FAR void *data, size_t len,
                                  uint32_t src, FAR void *priv)
{
  FAR struct rpmsgfs_unlink_s *msg = data;

  msg->header.result = nx_unlink(msg->pathname);
  return rpmsg_send(ept, msg, sizeof(*msg));
}

static int rpmsgfs_mkdir_handler(FAR struct rpmsg_endpoint *ept,
                                 FAR void *data, size_t len,
                                 uint32_t src, FAR void *priv)
{
  FAR struct rpmsgfs_mkdir_s *msg = data;
  int ret;

  ret = mkdir(msg->pathname, msg->mode);
  msg->header.result = ret ? -get_errno() : 0;
  return rpmsg_send(ept, msg, sizeof(*msg));
}

static int rpmsgfs_rmdir_handler(FAR struct rpmsg_endpoint *ept,
                                 FAR void *data, size_t len,
                                 uint32_t src, FAR void *priv)
{
  FAR struct rpmsgfs_rmdir_s *msg = data;
  int ret;

  ret = rmdir(msg->pathname);
  msg->header.result = ret ? -get_errno() : 0;
  return rpmsg_send(ept, msg, sizeof(*msg));
}

static int rpmsgfs_rename_handler(FAR struct rpmsg_endpoint *ept,
                                  FAR void *data, size_t len,
                                  uint32_t src, FAR void *priv)
{
  FAR struct rpmsgfs_rename_s *msg = data;
  FAR char *newpath;
  size_t oldlen;
  int ret;

  oldlen = (strlen(msg->pathname) + 1 + 0x7) & ~0x7;
  newpath = msg->pathname + oldlen;

  ret = rename(msg->pathname, newpath);
  msg->header.result = ret ? -get_errno() : 0;
  return rpmsg_send(ept, msg, sizeof(*msg));
}

static int rpmsgfs_stat_handler(FAR struct rpmsg_endpoint *ept,
                                FAR void *data, size_t len,
                                uint32_t src, FAR void *priv)
{
  FAR struct rpmsgfs_stat_s *msg = data;
  struct stat buf;
  int ret;

  ret = nx_stat(msg->pathname, &buf, 1);
  if (ret >= 0)
    {
      rpmsgfs_set_statbuf(&msg->buf, &buf);
    }

  msg->header.result = ret;
  return rpmsg_send(ept, msg, sizeof(*msg));
}

static int rpmsgfs_fchstat_handler(FAR struct rpmsg_endpoint *ept,
                                   FAR void *data, size_t len,
                                   uint32_t src, FAR void *priv)
{
  FAR struct rpmsgfs_fchstat_s *msg = data;
  FAR struct file *filep;
  int ret = -ENOENT;
  struct stat buf;

  filep = rpmsgfs_get_file(priv, msg->fd);
  if (filep != NULL)
    {
      rpmsgfs_get_statbuf(&buf, &msg->buf);

      ret = file_fchstat(filep, &buf, msg->flags);
    }

  msg->header.result = ret;
  return rpmsg_send(ept, msg, sizeof(*msg));
}

static int rpmsgfs_chstat_handler(FAR struct rpmsg_endpoint *ept,
                                  FAR void *data, size_t len,
                                  uint32_t src, FAR void *priv)
{
  FAR struct rpmsgfs_chstat_s *msg = data;
  struct timespec times[2];
  int ret = 0;

  if (msg->flags & CH_STAT_MODE)
    {
      ret = chmod(msg->pathname, msg->buf.mode);
      if (ret < 0)
        {
          ret = -get_errno();
          goto out;
        }
    }

  if (msg->flags & (CH_STAT_UID | CH_STAT_GID))
    {
      ret = chown(msg->pathname, msg->buf.uid, msg->buf.gid);
      if (ret < 0)
        {
          ret = -get_errno();
          goto out;
        }
    }

  if (msg->flags & (CH_STAT_ATIME | CH_STAT_MTIME))
    {
      if (msg->flags & CH_STAT_ATIME)
        {
          times[0].tv_sec = msg->buf.atim_sec;
          times[0].tv_nsec = msg->buf.atim_nsec;
        }
      else
        {
          times[0].tv_sec = 0;
          times[0].tv_nsec = UTIME_OMIT;
        }

      if (msg->flags & CH_STAT_MTIME)
        {
          times[1].tv_sec = msg->buf.mtim_sec;
          times[1].tv_nsec = msg->buf.mtim_nsec;
        }
      else
        {
          times[1].tv_sec = 0;
          times[1].tv_nsec = UTIME_OMIT;
        }

      ret = utimens(msg->pathname, times);
      if (ret < 0)
        {
          ret = -get_errno();
          goto out;
        }
    }

out:
  msg->header.result = ret;
  return rpmsg_send(ept, msg, sizeof(*msg));
}

#ifdef CONFIG_FS_LINKS
static int rpmsgfs_symlink_handler(FAR struct rpmsg_endpoint *ept,
                                   FAR void *data, size_t len,
                                   uint32_t src, FAR void *priv)
{
  FAR struct rpmsgfs_symlink_s *msg = data;
  FAR const char *target = msg->pathname;
  FAR const char *linkpath = target + strlen(target) + 1;
  int ret;

  ret = symlink(target, linkpath);
  msg->header.result = ret ? -get_errno() : 0;
  return rpmsg_send(ept, msg, sizeof(*msg));
}

static int rpmsgfs_readlink_handler(FAR struct rpmsg_endpoint *ept,
                                    FAR void *data, size_t len,
                                    uint32_t src, FAR void *priv)
{
  FAR struct rpmsgfs_readlink_s *msg = data;
  ssize_t ret;
  uint32_t space;

  space = rpmsg_get_tx_buffer_size(ept);
  ret = readlink(msg->pathname, msg->pathname, space - sizeof(*msg));
  msg->header.result = ret < 0 ? -get_errno() : ret;
  return rpmsg_send(ept, msg, sizeof(*msg) + (ret > 0 ? ret : 0));
}

static int rpmsgfs_link_handler(FAR struct rpmsg_endpoint *ept,
                                FAR void *data, size_t len,
                                uint32_t src, FAR void *priv)
{
  FAR struct rpmsgfs_link_s *msg = data;
  FAR const char *path1 = msg->pathname;
  FAR const char *path2 = path1 + strlen(path1) + 1;
  int ret;

  ret = link(path1, path2);
  msg->header.result = ret ? -get_errno() : 0;
  return rpmsg_send(ept, msg, sizeof(*msg));
}

static int rpmsgfs_lstat_handler(FAR struct rpmsg_endpoint *ept,
                                 FAR void *data, size_t len,
                                 uint32_t src, FAR void *priv)
{
  FAR struct rpmsgfs_lstat_s *msg = data;
  struct stat buf;
  int ret;

  ret = nx_stat(msg->pathname, &buf, 0);
  if (ret >= 0)
    {
      rpmsgfs_set_statbuf(&msg->buf, &buf);
    }

  msg->header.result = ret;
  return rpmsg_send(ept, msg, sizeof(*msg));
}
#endif

static bool rpmsgfs_ns_match(FAR struct rpmsg_device *rdev,
                             FAR void *priv_, FAR const char *name,
                             uint32_t dest)
{
  return !strncmp(name, RPMSGFS_NAME_PREFIX, strlen(RPMSGFS_NAME_PREFIX));
}

static void rpmsgfs_ept_release(FAR struct rpmsg_endpoint *ept)
{
  FAR struct rpmsgfs_server_s *priv = ept->priv;
  int i;
  int j;

  for (i = 0; i < priv->file_rows; i++)
    {
      for (j = 0; j < CONFIG_NFILE_DESCRIPTORS_PER_BLOCK; j++)
        {
          if (priv->files[i][j].f_inode)
            {
              file_close(&priv->files[i][j]);
            }
        }

      fs_heap_free(priv->files[i]);
    }

  for (i = 0; i < priv->dir_nums; i++)
    {
      if (priv->dirs[i])
        {
          closedir(priv->dirs[i]);
        }
    }

  nxmutex_destroy(&priv->lock);

  fs_heap_free(priv->files);
  fs_heap_free(priv->dirs);
  fs_heap_free(priv);
}

static void rpmsgfs_ns_bind(FAR struct rpmsg_device *rdev,
                            FAR void *priv_, FAR const char *name,
                            uint32_t dest)
{
  FAR struct rpmsgfs_server_s *priv;
  int ret;

  priv = fs_heap_zalloc(sizeof(*priv));
  if (!priv)
    {
      return;
    }

  priv->ept.priv = priv;
  priv->ept.release_cb = rpmsgfs_ept_release;
  nxmutex_init(&priv->lock);

  ret = rpmsg_create_ept(&priv->ept, rdev, name,
                         RPMSG_ADDR_ANY, dest,
                         rpmsgfs_ept_cb, rpmsg_destroy_ept);
  if (ret)
    {
      nxmutex_destroy(&priv->lock);
      fs_heap_free(priv);
    }
}

static int rpmsgfs_ept_cb(FAR struct rpmsg_endpoint *ept,
                          FAR void *data, size_t len, uint32_t src,
                          FAR void *priv)
{
  struct rpmsgfs_header_s *header = data;
  uint32_t command = header->command;
  int ret;

  if (command >= nitems(g_rpmsgfs_handler))
    {
      return -EINVAL;
    }

  ret = g_rpmsgfs_handler[command](ept, data, len, src, priv);
  if (ret < 0)
    {
      ferr("ERROR: handle failed, ept=%p cmd=%" PRIu32 " ret=%d\n",
           ept, command, ret);
    }

  return ret;
}

int rpmsgfs_server_init(void)
{
  return rpmsg_register_callback(NULL,
                                 NULL,
                                 NULL,
                                 rpmsgfs_ns_match,
                                 rpmsgfs_ns_bind);
}
