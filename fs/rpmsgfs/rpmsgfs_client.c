/****************************************************************************
 * fs/rpmsgfs/rpmsgfs_client.c
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
#include <stdio.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/rpmsgfs.h>
#include <nuttx/rptun/openamp.h>
#include <nuttx/semaphore.h>

#include "rpmsgfs.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct rpmsgfs_s
{
  struct rpmsg_endpoint ept;
  char                  cpuname[RPMSG_NAME_SIZE];
};

struct rpmsgfs_cookie_s
{
  sem_t     sem;
  int       result;
  FAR void  *data;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int rpmsgfs_default_handler(FAR struct rpmsg_endpoint *ept,
                                   FAR void *data, size_t len,
                                   uint32_t src, FAR void *priv);
static int rpmsgfs_read_handler(FAR struct rpmsg_endpoint *ept,
                                FAR void *data, size_t len,
                                uint32_t src, FAR void *priv);
static int rpmsgfs_readdir_handler(FAR struct rpmsg_endpoint *ept,
                                  FAR void *data, size_t len,
                                  uint32_t src, FAR void *priv);
static int rpmsgfs_statfs_handler(FAR struct rpmsg_endpoint *ept,
                                  FAR void *data, size_t len,
                                  uint32_t src, FAR void *priv);
static int rpmsgfs_stat_handler(FAR struct rpmsg_endpoint *ept,
                                 FAR void *data, size_t len,
                                 uint32_t src, FAR void *priv);
static void rpmsgfs_device_created(struct rpmsg_device *rdev,
                                   FAR void *priv_);
static void rpmsgfs_device_destroy(struct rpmsg_device *rdev,
                                   FAR void *priv_);
static int  rpmsgfs_ept_cb(FAR struct rpmsg_endpoint *ept,
                           FAR void *data, size_t len, uint32_t src,
                           FAR void *priv);
static int rpmsgfs_send_recv(FAR struct rpmsgfs_s *priv,
                             uint32_t command, bool copy,
                             FAR struct rpmsgfs_header_s *msg,
                             int len, FAR void *data);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const rpmsg_ept_cb g_rpmsgfs_handler[] =
{
  [RPMSGFS_OPEN]      = rpmsgfs_default_handler,
  [RPMSGFS_CLOSE]     = rpmsgfs_default_handler,
  [RPMSGFS_READ]      = rpmsgfs_read_handler,
  [RPMSGFS_WRITE]     = rpmsgfs_default_handler,
  [RPMSGFS_LSEEK]     = rpmsgfs_default_handler,
  [RPMSGFS_IOCTL]     = rpmsgfs_default_handler,
  [RPMSGFS_SYNC]      = rpmsgfs_default_handler,
  [RPMSGFS_DUP]       = rpmsgfs_default_handler,
  [RPMSGFS_FSTAT]     = rpmsgfs_stat_handler,
  [RPMSGFS_FTRUNCATE] = rpmsgfs_default_handler,
  [RPMSGFS_OPENDIR]   = rpmsgfs_default_handler,
  [RPMSGFS_READDIR]   = rpmsgfs_readdir_handler,
  [RPMSGFS_REWINDDIR] = rpmsgfs_default_handler,
  [RPMSGFS_CLOSEDIR]  = rpmsgfs_default_handler,
  [RPMSGFS_STATFS]    = rpmsgfs_statfs_handler,
  [RPMSGFS_UNLINK]    = rpmsgfs_default_handler,
  [RPMSGFS_MKDIR]     = rpmsgfs_default_handler,
  [RPMSGFS_RMDIR]     = rpmsgfs_default_handler,
  [RPMSGFS_RENAME]    = rpmsgfs_default_handler,
  [RPMSGFS_STAT]      = rpmsgfs_stat_handler,
  [RPMSGFS_FCHSTAT]   = rpmsgfs_default_handler,
  [RPMSGFS_CHSTAT]    = rpmsgfs_default_handler,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int rpmsgfs_default_handler(FAR struct rpmsg_endpoint *ept,
                                   FAR void *data, size_t len,
                                   uint32_t src, FAR void *priv)
{
  FAR struct rpmsgfs_header_s *header = data;
  FAR struct rpmsgfs_cookie_s *cookie =
      (struct rpmsgfs_cookie_s *)(uintptr_t)header->cookie;

  cookie->result = header->result;
  if (cookie->result >= 0 && cookie->data)
    {
      memcpy(cookie->data, data, len);
    }

  nxsem_post(&cookie->sem);

  return 0;
}

static int rpmsgfs_read_handler(FAR struct rpmsg_endpoint *ept,
                                FAR void *data, size_t len,
                                uint32_t src, FAR void *priv)
{
  FAR struct rpmsgfs_header_s *header = data;
  FAR struct rpmsgfs_cookie_s *cookie =
      (struct rpmsgfs_cookie_s *)(uintptr_t)header->cookie;
  FAR struct rpmsgfs_read_s *rsp = data;

  cookie->result = header->result;
  if (cookie->result > 0)
    {
      memcpy(cookie->data, rsp->buf, cookie->result);
    }

  nxsem_post(&cookie->sem);

  return 0;
}

static int rpmsgfs_readdir_handler(FAR struct rpmsg_endpoint *ept,
                                   FAR void *data, size_t len,
                                   uint32_t src, FAR void *priv)
{
  FAR struct rpmsgfs_header_s *header = data;
  FAR struct rpmsgfs_cookie_s *cookie =
      (struct rpmsgfs_cookie_s *)(uintptr_t)header->cookie;
  FAR struct rpmsgfs_readdir_s *rsp = data;
  FAR struct dirent *entry = cookie->data;

  cookie->result = header->result;
  if (cookie->result >= 0)
    {
      strlcpy(entry->d_name, rsp->name, sizeof(entry->d_name));
      entry->d_type = rsp->type;
    }

  nxsem_post(&cookie->sem);

  return 0;
}

static int rpmsgfs_statfs_handler(FAR struct rpmsg_endpoint *ept,
                                  FAR void *data, size_t len,
                                  uint32_t src, FAR void *priv)
{
  FAR struct rpmsgfs_header_s *header = data;
  FAR struct rpmsgfs_cookie_s *cookie =
      (struct rpmsgfs_cookie_s *)(uintptr_t)header->cookie;
  FAR struct rpmsgfs_statfs_s *rsp = data;
  FAR struct statfs *buf = cookie->data;

  cookie->result = header->result;
  if (cookie->result >= 0)
    {
      buf->f_type    = rsp->buf.f_type;
      buf->f_namelen = rsp->buf.f_namelen;
      buf->f_bsize   = rsp->buf.f_bsize;
      buf->f_blocks  = rsp->buf.f_blocks;
      buf->f_bfree   = rsp->buf.f_bfree;
      buf->f_bavail  = rsp->buf.f_bavail;
      buf->f_files   = rsp->buf.f_files;
      buf->f_ffree   = rsp->buf.f_ffree;
    }

  nxsem_post(&cookie->sem);

  return 0;
}

static int rpmsgfs_stat_handler(FAR struct rpmsg_endpoint *ept,
                                FAR void *data, size_t len,
                                uint32_t src, FAR void *priv)
{
  FAR struct rpmsgfs_header_s *header = data;
  FAR struct rpmsgfs_cookie_s *cookie =
      (struct rpmsgfs_cookie_s *)(uintptr_t)header->cookie;
  FAR struct rpmsgfs_stat_s *rsp = data;
  FAR struct stat *buf = cookie->data;

  cookie->result = header->result;
  if (cookie->result >= 0)
    {
      buf->st_dev     = rsp->buf.st_dev;
      buf->st_ino     = rsp->buf.st_ino;
      buf->st_mode    = rsp->buf.st_mode;
      buf->st_nlink   = rsp->buf.st_nlink;
      buf->st_uid     = rsp->buf.st_uid;
      buf->st_gid     = rsp->buf.st_gid;
      buf->st_rdev    = rsp->buf.st_rdev;
      buf->st_size    = rsp->buf.st_size;
      buf->st_atime   = rsp->buf.st_atime;
      buf->st_mtime   = rsp->buf.st_mtime;
      buf->st_ctime   = rsp->buf.st_ctime;
      buf->st_blksize = rsp->buf.st_blksize;
      buf->st_blocks  = rsp->buf.st_blocks;
    }

  nxsem_post(&cookie->sem);

  return 0;
}

static void rpmsgfs_device_created(FAR struct rpmsg_device *rdev,
                                   FAR void *priv_)
{
  FAR struct rpmsgfs_s *priv = priv_;
  char buf[RPMSG_NAME_SIZE];

  if (strcmp(priv->cpuname, rpmsg_get_cpuname(rdev)) == 0)
    {
      priv->ept.priv = priv;
      snprintf(buf, sizeof(buf), "%s%p", RPMSGFS_NAME_PREFIX, priv);
      rpmsg_create_ept(&priv->ept, rdev, buf,
                       RPMSG_ADDR_ANY, RPMSG_ADDR_ANY,
                       rpmsgfs_ept_cb, NULL);
    }
}

static void rpmsgfs_device_destroy(FAR struct rpmsg_device *rdev,
                                   FAR void *priv_)
{
  struct rpmsgfs_s *priv = priv_;

  if (strcmp(priv->cpuname, rpmsg_get_cpuname(rdev)) == 0)
    {
      rpmsg_destroy_ept(&priv->ept);
    }
}

static int rpmsgfs_ept_cb(FAR struct rpmsg_endpoint *ept,
                          FAR void *data, size_t len, uint32_t src,
                          FAR void *priv)
{
  FAR struct rpmsgfs_header_s *header = data;
  uint32_t command = header->command;

  if (command < ARRAY_SIZE(g_rpmsgfs_handler))
    {
      return g_rpmsgfs_handler[command](ept, data, len, src, priv);
    }

  return -EINVAL;
}

static int rpmsgfs_send_recv(FAR struct rpmsgfs_s *priv,
                             uint32_t command, bool copy,
                             FAR struct rpmsgfs_header_s *msg,
                             int len, FAR void *data)
{
  FAR struct rpmsgfs_cookie_s cookie;
  int ret;

  memset(&cookie, 0, sizeof(cookie));
  nxsem_init(&cookie.sem, 0, 0);
  nxsem_set_protocol(&cookie.sem, SEM_PRIO_NONE);

  if (data)
    {
      cookie.data = data;
    }
  else if (copy)
    {
      cookie.data = msg;
    }

  msg->command = command;
  msg->result  = -ENXIO;
  msg->cookie  = (uintptr_t)&cookie;

  if (copy)
    {
      ret = rpmsg_send(&priv->ept, msg, len);
    }
  else
    {
      ret = rpmsg_send_nocopy(&priv->ept, msg, len);
    }

  if (ret < 0)
    {
      goto fail;
    }

  ret = nxsem_wait_uninterruptible(&cookie.sem);
  if (ret == 0)
    {
      ret = cookie.result;
    }

fail:
  nxsem_destroy(&cookie.sem);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int rpmsgfs_client_open(FAR void *handle, FAR const char *pathname,
                        int flags, int mode)
{
  FAR struct rpmsgfs_s *priv = handle;
  FAR struct rpmsgfs_open_s *msg;
  uint32_t space;
  size_t len;

  len  = sizeof(*msg);
  len += strlen(pathname) + 1;

  msg = rpmsg_get_tx_payload_buffer(&priv->ept, &space, true);
  if (!msg)
    {
      return -ENOMEM;
    }

  DEBUGASSERT(len <= space);

  msg->flags = flags;
  msg->mode  = mode;
  strcpy(msg->pathname, pathname);

  return rpmsgfs_send_recv(priv, RPMSGFS_OPEN, false,
          (struct rpmsgfs_header_s *)msg, len, NULL);
}

int rpmsgfs_client_close(FAR void *handle, int fd)
{
  struct rpmsgfs_close_s msg =
  {
    .fd = fd,
  };

  return rpmsgfs_send_recv(handle, RPMSGFS_CLOSE, true,
          (struct rpmsgfs_header_s *)&msg, sizeof(msg), NULL);
}

ssize_t rpmsgfs_client_read(FAR void *handle, int fd,
                            FAR void *buf, size_t count)
{
  size_t read = 0;
  int ret = 0;

  while (read < count)
    {
      struct rpmsgfs_read_s msg =
      {
        .fd    = fd,
        .count = count - read,
      };

      ret = rpmsgfs_send_recv(handle, RPMSGFS_READ, true,
              (FAR struct rpmsgfs_header_s *)&msg, sizeof(msg), buf);
      if (ret <= 0)
        {
          break;
        }

      read += ret;
      buf  += ret;
    }

  return read ? read : ret;
}

ssize_t rpmsgfs_client_write(FAR void *handle, int fd,
                             FAR const void *buf, size_t count)
{
  FAR struct rpmsgfs_s *priv = handle;
  size_t written = 0;
  int ret = 0;

  while (written < count)
    {
      FAR struct rpmsgfs_write_s *msg;
      uint32_t space;

      msg = rpmsg_get_tx_payload_buffer(&priv->ept, &space, true);
      if (!msg)
        {
          ret = -ENOMEM;
          break;
        }

      space -= sizeof(*msg);
      if (space > count - written)
        {
          space = count - written;
        }

      msg->fd    = fd;
      msg->count = space;
      memcpy(msg->buf, buf + written, space);

      ret = rpmsgfs_send_recv(priv, RPMSGFS_WRITE, false,
                                   (FAR struct rpmsgfs_header_s *)msg,
                                   sizeof(*msg) + space, NULL);
      if (ret <= 0)
        {
          break;
        }

      written += ret;
    }

  return written ? written : ret;
}

off_t rpmsgfs_client_lseek(FAR void *handle, int fd,
                           off_t offset, int whence)
{
  struct rpmsgfs_lseek_s msg =
  {
    .fd     = fd,
    .offset = offset,
    .whence = whence,
  };

  int ret;

  ret = rpmsgfs_send_recv(handle, RPMSGFS_LSEEK, true,
          (struct rpmsgfs_header_s *)&msg, sizeof(msg), NULL);

  return ret < 0 ? ret : ret;
}

int rpmsgfs_client_ioctl(FAR void *handle, int fd,
                         int request, unsigned long arg)
{
  struct rpmsgfs_ioctl_s msg =
  {
    .fd      = fd,
    .request = request,
    .arg     = arg,
  };

  return rpmsgfs_send_recv(handle, RPMSGFS_IOCTL, true,
          (struct rpmsgfs_header_s *)&msg, sizeof(msg), NULL);
}

void rpmsgfs_client_sync(FAR void *handle, int fd)
{
  struct rpmsgfs_sync_s msg =
  {
    .fd = fd,
  };

  rpmsgfs_send_recv(handle, RPMSGFS_SYNC, true,
          (struct rpmsgfs_header_s *)&msg, sizeof(msg), NULL);
}

int rpmsgfs_client_dup(FAR void *handle, int fd)
{
  struct rpmsgfs_dup_s msg =
  {
    .fd = fd,
  };

  return rpmsgfs_send_recv(handle, RPMSGFS_DUP, true,
          (struct rpmsgfs_header_s *)&msg, sizeof(msg), NULL);
}

int rpmsgfs_client_fstat(FAR void *handle, int fd, struct stat *buf)
{
  struct rpmsgfs_fstat_s msg =
  {
    .fd = fd,
  };

  return rpmsgfs_send_recv(handle, RPMSGFS_FSTAT, true,
          (struct rpmsgfs_header_s *)&msg, sizeof(msg), buf);
}

int rpmsgfs_client_ftruncate(FAR void *handle, int fd, off_t length)
{
  struct rpmsgfs_ftruncate_s msg =
  {
    .fd     = fd,
    .length = length,
  };

  return rpmsgfs_send_recv(handle, RPMSGFS_FTRUNCATE, true,
          (struct rpmsgfs_header_s *)&msg, sizeof(msg), NULL);
}

FAR void *rpmsgfs_client_opendir(FAR void *handle, FAR const char *name)
{
  FAR struct rpmsgfs_s *priv = handle;
  FAR struct rpmsgfs_opendir_s *msg;
  uint32_t space;
  size_t len;
  int ret;

  len  = sizeof(*msg);
  len += strlen(name) + 1;

  msg = rpmsg_get_tx_payload_buffer(&priv->ept, &space, true);
  if (!msg)
    {
      return NULL;
    }

  DEBUGASSERT(len <= space);

  strcpy(msg->pathname, name);

  ret = rpmsgfs_send_recv(priv, RPMSGFS_OPENDIR, false,
          (struct rpmsgfs_header_s *)msg, len, NULL);

  return ret < 0 ? NULL : (FAR void *)((uintptr_t)ret);
}

int rpmsgfs_client_readdir(FAR void *handle, FAR void *dirp,
                           FAR struct dirent *entry)
{
  struct rpmsgfs_readdir_s msg =
  {
    .fd = (uintptr_t)dirp,
  };

  return rpmsgfs_send_recv(handle, RPMSGFS_READDIR, true,
          (struct rpmsgfs_header_s *)&msg, sizeof(msg), entry);
}

void rpmsgfs_client_rewinddir(FAR void *handle, FAR void *dirp)
{
  struct rpmsgfs_rewinddir_s msg =
  {
    .fd = (uintptr_t)dirp,
  };

  rpmsgfs_send_recv(handle, RPMSGFS_REWINDDIR, true,
          (struct rpmsgfs_header_s *)&msg, sizeof(msg), NULL);
}

int rpmsgfs_client_bind(FAR void **handle, FAR const char *cpuname)
{
  struct rpmsgfs_s *priv;
  int ret;

  if (!cpuname)
    {
      return -EINVAL;
    }

  priv = kmm_zalloc(sizeof(struct rpmsgfs_s));
  if (!priv)
    {
      return -ENOMEM;
    }

  strlcpy(priv->cpuname, cpuname, sizeof(priv->cpuname));
  ret = rpmsg_register_callback(priv,
                                rpmsgfs_device_created,
                                rpmsgfs_device_destroy,
                                NULL);
  if (ret < 0)
    {
      kmm_free(priv);
      return ret;
    }

  *handle = priv;

  return 0;
}

int rpmsgfs_client_unbind(FAR void *handle)
{
  struct rpmsgfs_s *priv = handle;

  rpmsg_unregister_callback(priv,
                            rpmsgfs_device_created,
                            rpmsgfs_device_destroy,
                            NULL);

  kmm_free(priv);
  return 0;
}

int rpmsgfs_client_closedir(FAR void *handle, FAR void *dirp)
{
  struct rpmsgfs_closedir_s msg =
  {
    .fd = (uintptr_t)dirp,
  };

  return rpmsgfs_send_recv(handle, RPMSGFS_CLOSEDIR, true,
          (struct rpmsgfs_header_s *)&msg, sizeof(msg), NULL);
}

int rpmsgfs_client_statfs(FAR void *handle, FAR const char *path,
                          FAR struct statfs *buf)
{
  struct rpmsgfs_s *priv = handle;
  struct rpmsgfs_statfs_s *msg;
  uint32_t space;
  size_t len;

  len  = sizeof(*msg);
  len += strlen(path) + 1;

  msg = rpmsg_get_tx_payload_buffer(&priv->ept, &space, true);
  if (!msg)
    {
      return -ENOMEM;
    }

  DEBUGASSERT(len <= space);

  strcpy(msg->pathname, path);

  return rpmsgfs_send_recv(priv, RPMSGFS_STATFS, false,
          (struct rpmsgfs_header_s *)msg, len, buf);
}

int rpmsgfs_client_unlink(FAR void *handle, FAR const char *pathname)
{
  struct rpmsgfs_s *priv = handle;
  struct rpmsgfs_unlink_s *msg;
  uint32_t space;
  size_t len;

  len  = sizeof(*msg);
  len += strlen(pathname) + 1;

  msg = rpmsg_get_tx_payload_buffer(&priv->ept, &space, true);
  if (!msg)
    {
      return -ENOMEM;
    }

  DEBUGASSERT(len <= space);

  strcpy(msg->pathname, pathname);

  return rpmsgfs_send_recv(priv, RPMSGFS_UNLINK, false,
          (struct rpmsgfs_header_s *)msg, len, NULL);
}

int rpmsgfs_client_mkdir(FAR void *handle, FAR const char *pathname,
                         mode_t mode)
{
  struct rpmsgfs_s *priv = handle;
  struct rpmsgfs_mkdir_s *msg;
  uint32_t space;
  size_t len;

  len  = sizeof(*msg);
  len += strlen(pathname) + 1;

  msg = rpmsg_get_tx_payload_buffer(&priv->ept, &space, true);
  if (!msg)
    {
      return -ENOMEM;
    }

  msg->mode = mode;
  strcpy(msg->pathname, pathname);

  return rpmsgfs_send_recv(priv, RPMSGFS_MKDIR, false,
          (struct rpmsgfs_header_s *)msg, len, NULL);
}

int rpmsgfs_client_rmdir(FAR void *handle, FAR const char *pathname)
{
  struct rpmsgfs_s *priv = handle;
  struct rpmsgfs_rmdir_s *msg;
  uint32_t space;
  size_t len;

  len  = sizeof(*msg);
  len += strlen(pathname) + 1;

  msg = rpmsg_get_tx_payload_buffer(&priv->ept, &space, true);
  if (!msg)
    {
      return -ENOMEM;
    }

  DEBUGASSERT(len <= space);

  strcpy(msg->pathname, pathname);

  return rpmsgfs_send_recv(priv, RPMSGFS_RMDIR, false,
          (struct rpmsgfs_header_s *)msg, len, NULL);
}

int rpmsgfs_client_rename(FAR void *handle, FAR const char *oldpath,
                          FAR const char *newpath)
{
  struct rpmsgfs_s *priv = handle;
  struct rpmsgfs_rename_s *msg;
  size_t len;
  size_t oldlen;
  uint32_t space;

  len     = sizeof(*msg);
  oldlen  = (strlen(oldpath) + 1 + 0x7) & ~0x7;
  len    += oldlen + strlen(newpath) + 1;

  msg = rpmsg_get_tx_payload_buffer(&priv->ept, &space, true);
  if (!msg)
    {
      return -ENOMEM;
    }

  DEBUGASSERT(len <= space);

  strcpy(msg->pathname, oldpath);
  strcpy(msg->pathname + oldlen, newpath);

  return rpmsgfs_send_recv(priv, RPMSGFS_RENAME, false,
          (struct rpmsgfs_header_s *)msg, len, NULL);
}

int rpmsgfs_client_stat(FAR void *handle, FAR const char *path,
                        FAR struct stat *buf)
{
  FAR struct rpmsgfs_s *priv = handle;
  FAR struct rpmsgfs_stat_s *msg;
  uint32_t space;
  size_t len;

  len  = sizeof(*msg);
  len += strlen(path) + 1;

  msg = rpmsg_get_tx_payload_buffer(&priv->ept, &space, true);
  if (!msg)
    {
      return -ENOMEM;
    }

  DEBUGASSERT(len <= space);

  strcpy(msg->pathname, path);

  return rpmsgfs_send_recv(priv, RPMSGFS_STAT, false,
          (struct rpmsgfs_header_s *)msg, len, buf);
}

int rpmsgfs_client_fchstat(FAR void *handle, int fd,
                           const struct stat *buf, int flags)
{
  struct rpmsgfs_fchstat_s msg =
  {
    .flags = flags,
    .buf = *buf,
    .fd = fd,
  };

  return rpmsgfs_send_recv(handle, RPMSGFS_FCHSTAT, true,
          (struct rpmsgfs_header_s *)&msg, sizeof(msg), NULL);
}

int rpmsgfs_client_chstat(FAR void *handle, FAR const char *path,
                          const FAR struct stat *buf, int flags)
{
  FAR struct rpmsgfs_s *priv = handle;
  FAR struct rpmsgfs_chstat_s *msg;
  uint32_t space;
  size_t len;

  len  = sizeof(*msg);
  len += strlen(path) + 1;

  msg = rpmsg_get_tx_payload_buffer(&priv->ept, &space, true);
  if (!msg)
    {
      return -ENOMEM;
    }

  DEBUGASSERT(len <= space);

  msg->flags = flags;
  memcpy(&msg->buf, buf, sizeof(*buf));
  strcpy(msg->pathname, path);

  return rpmsgfs_send_recv(priv, RPMSGFS_CHSTAT, false,
          (struct rpmsgfs_header_s *)msg, len, NULL);
}
