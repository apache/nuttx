/****************************************************************************
 * fs/rpmsgfs/rpmsgfs_server.c
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
#include <errno.h>

#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/fs/fs.h>
#include <nuttx/rptun/openamp.h>

#include "rpmsgfs.h"

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

static bool rpmsgfs_ns_match(FAR struct rpmsg_device *rdev,
                             FAR void *priv_, FAR const char *name,
                             uint32_t dest);
static void rpmsgfs_ns_bind(FAR struct rpmsg_device *rdev,
                            FAR void *priv_, FAR const char *name,
                            uint32_t dest);
static void rpmsgfs_ns_unbind(FAR struct rpmsg_endpoint *ept);
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
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int rpmsgfs_attach_file(FAR struct rpmsgfs_server_s *priv,
                               FAR struct file *filep)
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
              memcpy(&priv->files[i][j], filep, sizeof(*filep));
              ret = i * CONFIG_NFILE_DESCRIPTORS_PER_BLOCK + j;
              goto out;
            }
        }
    }

  tmp = kmm_realloc(priv->files, sizeof(FAR struct file *) * (i + 1));
  DEBUGASSERT(tmp);
  if (tmp == NULL)
    {
      ret = -ENFILE;
      goto out;
    }

  tmp[i] = kmm_zalloc(sizeof(struct file) *
                      CONFIG_NFILE_DESCRIPTORS_PER_BLOCK);
  DEBUGASSERT(tmp[i]);
  if (tmp[i] == NULL)
    {
      kmm_free(tmp);
      ret = -ENFILE;
      goto out;
    }

  priv->files = tmp;
  priv->file_rows++;

  memcpy(&priv->files[i][0], filep, sizeof(*filep));
  ret = i * CONFIG_NFILE_DESCRIPTORS_PER_BLOCK;

out:
  nxmutex_unlock(&priv->lock);
  return ret;
}

static int rpmsgfs_detach_file(FAR struct rpmsgfs_server_s *priv,
                               int fd, FAR struct file *filep)
{
  struct file *tfilep;

  if (fd < 0 || fd >= priv->file_rows * CONFIG_NFILE_DESCRIPTORS_PER_BLOCK)
    {
      return -EBADF;
    }

  nxmutex_lock(&priv->lock);
  tfilep = &priv->files[fd / CONFIG_NFILE_DESCRIPTORS_PER_BLOCK]
                       [fd % CONFIG_NFILE_DESCRIPTORS_PER_BLOCK];
  memcpy(filep, tfilep, sizeof(*filep));
  memset(tfilep, 0, sizeof(*tfilep));
  nxmutex_unlock(&priv->lock);

  return 0;
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

  tmp = kmm_realloc(priv->dirs, sizeof(FAR void *) *
                    (priv->dir_nums + CONFIG_NFILE_DESCRIPTORS_PER_BLOCK));
  DEBUGASSERT(tmp);
  if (tmp == NULL)
    {
      nxmutex_unlock(&priv->lock);
      return -ENOMEM;
    }

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

static int rpmsgfs_open_handler(FAR struct rpmsg_endpoint *ept,
                                FAR void *data, size_t len,
                                uint32_t src, FAR void *priv)
{
  FAR struct rpmsgfs_open_s *msg = data;
  struct file file;
  int ret;

  ret = file_open(&file, msg->pathname, msg->flags, msg->mode);
  if (ret >= 0)
    {
      ret = rpmsgfs_attach_file(priv, &file);
      if (ret < 0)
        {
          file_close(&file);
        }
    }

  msg->header.result = ret;
  return rpmsg_send(ept, msg, sizeof(*msg));
}

static int rpmsgfs_close_handler(FAR struct rpmsg_endpoint *ept,
                                 FAR void *data, size_t len,
                                 uint32_t src, FAR void *priv)
{
  FAR struct rpmsgfs_close_s *msg = data;
  struct file file;
  int ret;

  ret = rpmsgfs_detach_file(priv, msg->fd, &file);
  if (ret >= 0)
    {
      ret = file_close(&file);
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
      rpmsg_send_nocopy(ept, rsp, (ret < 0 ? 0 : ret) + sizeof(*rsp));

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
  FAR struct file *filep;
  struct file newfile;
  int ret = -ENOENT;

  filep = rpmsgfs_get_file(priv, msg->fd);
  if (filep != NULL)
    {
      ret = file_dup2(filep, &newfile);
      if (ret >= 0)
        {
          ret = rpmsgfs_attach_file(priv, &newfile);
          if (ret < 0)
            {
              file_close(&newfile);
            }
        }
    }

  msg->header.result = ret;
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
          msg->buf = buf;
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

  dir = rpmsgfs_get_dir(priv, msg->fd);
  if (dir)
    {
      entry = readdir(dir);
      if (entry)
        {
          msg->type = entry->d_type;
          strcpy(msg->name, entry->d_name);
          len += strlen(entry->d_name) + 1;
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
      msg->buf = buf;
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
      msg->buf = buf;
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
      buf = msg->buf;
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
      ret = chmod(msg->pathname, msg->buf.st_mode);
      if (ret < 0)
        {
          ret = -get_errno();
          goto out;
        }
    }

  if (msg->flags & (CH_STAT_UID | CH_STAT_GID))
    {
      ret = chown(msg->pathname, msg->buf.st_uid, msg->buf.st_gid);
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
          times[0] = msg->buf.st_atim;
        }
      else
        {
          times[0].tv_sec = 0;
          times[0].tv_nsec = UTIME_OMIT;
        }

      if (msg->flags & CH_STAT_MTIME)
        {
          times[1] = msg->buf.st_mtim;
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

static bool rpmsgfs_ns_match(FAR struct rpmsg_device *rdev,
                             FAR void *priv_, FAR const char *name,
                             uint32_t dest)
{
  return !strncmp(name, RPMSGFS_NAME_PREFIX, strlen(RPMSGFS_NAME_PREFIX));
}

static void rpmsgfs_ns_bind(FAR struct rpmsg_device *rdev,
                            FAR void *priv_, FAR const char *name,
                            uint32_t dest)
{
  FAR struct rpmsgfs_server_s *priv;
  int ret;

  priv = kmm_zalloc(sizeof(*priv));
  if (!priv)
    {
      return;
    }

  priv->ept.priv = priv;
  nxmutex_init(&priv->lock);

  ret = rpmsg_create_ept(&priv->ept, rdev, name,
                         RPMSG_ADDR_ANY, dest,
                         rpmsgfs_ept_cb, rpmsgfs_ns_unbind);
  if (ret)
    {
      nxmutex_destroy(&priv->lock);
      kmm_free(priv);
    }
}

static void rpmsgfs_ns_unbind(FAR struct rpmsg_endpoint *ept)
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

      kmm_free(priv->files[i]);
    }

  for (i = 0; i < priv->dir_nums; i++)
    {
      if (priv->dirs[i])
        {
          closedir(priv->dirs[i]);
        }
    }

  rpmsg_destroy_ept(&priv->ept);
  nxmutex_destroy(&priv->lock);

  kmm_free(priv->files);
  kmm_free(priv->dirs);
  kmm_free(priv);
}

static int rpmsgfs_ept_cb(FAR struct rpmsg_endpoint *ept,
                          FAR void *data, size_t len, uint32_t src,
                          FAR void *priv)
{
  struct rpmsgfs_header_s *header = data;
  uint32_t command = header->command;

  if (command < nitems(g_rpmsgfs_handler))
    {
      return g_rpmsgfs_handler[command](ept, data, len, src, priv);
    }

  return -EINVAL;
}

int rpmsgfs_server_init(void)
{
  return rpmsg_register_callback(NULL,
                                 NULL,
                                 NULL,
                                 rpmsgfs_ns_match,
                                 rpmsgfs_ns_bind);
}
