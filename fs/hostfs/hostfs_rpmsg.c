/****************************************************************************
 * fs/hostfs/hostfs_rpmsg.c
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
#include <string.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/hostfs.h>
#include <nuttx/fs/hostfs_rpmsg.h>
#include <nuttx/rptun/openamp.h>
#include <nuttx/semaphore.h>

#include "hostfs_rpmsg.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct hostfs_rpmsg_s
{
  struct rpmsg_endpoint ept;
  FAR const char        *cpuname;
};

struct hostfs_rpmsg_cookie_s
{
  sem_t     sem;
  int       result;
  FAR void  *data;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int hostfs_rpmsg_default_handler(FAR struct rpmsg_endpoint *ept,
                                        FAR void *data, size_t len,
                                        uint32_t src, FAR void *priv);
static int hostfs_rpmsg_read_handler(FAR struct rpmsg_endpoint *ept,
                                     FAR void *data, size_t len,
                                     uint32_t src, FAR void *priv);
static int hostfs_rpmsg_readdir_handler(FAR struct rpmsg_endpoint *ept,
                                        FAR void *data, size_t len,
                                        uint32_t src, FAR void *priv);
static int hostfs_rpmsg_statfs_handler(FAR struct rpmsg_endpoint *ept,
                                       FAR void *data, size_t len,
                                       uint32_t src, FAR void *priv);
static int hostfs_rpmsg_stat_handler(FAR struct rpmsg_endpoint *ept,
                                     FAR void *data, size_t len,
                                     uint32_t src, FAR void *priv);
static void hostfs_rpmsg_device_created(struct rpmsg_device *rdev,
                                        FAR void *priv_);
static void hostfs_rpmsg_device_destroy(struct rpmsg_device *rdev,
                                        FAR void *priv_);
static int  hostfs_rpmsg_ept_cb(FAR struct rpmsg_endpoint *ept,
                                FAR void *data, size_t len, uint32_t src,
                                FAR void *priv);
static int  hostfs_rpmsg_send_recv(uint32_t command, bool copy,
                                   FAR struct hostfs_rpmsg_header_s *msg,
                                   int len, FAR void *data);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct hostfs_rpmsg_s g_hostfs_rpmsg;

static const rpmsg_ept_cb g_hostfs_rpmsg_handler[] =
{
  [HOSTFS_RPMSG_OPEN]      = hostfs_rpmsg_default_handler,
  [HOSTFS_RPMSG_CLOSE]     = hostfs_rpmsg_default_handler,
  [HOSTFS_RPMSG_READ]      = hostfs_rpmsg_read_handler,
  [HOSTFS_RPMSG_WRITE]     = hostfs_rpmsg_default_handler,
  [HOSTFS_RPMSG_LSEEK]     = hostfs_rpmsg_default_handler,
  [HOSTFS_RPMSG_IOCTL]     = hostfs_rpmsg_default_handler,
  [HOSTFS_RPMSG_SYNC]      = hostfs_rpmsg_default_handler,
  [HOSTFS_RPMSG_DUP]       = hostfs_rpmsg_default_handler,
  [HOSTFS_RPMSG_FSTAT]     = hostfs_rpmsg_stat_handler,
  [HOSTFS_RPMSG_FTRUNCATE] = hostfs_rpmsg_default_handler,
  [HOSTFS_RPMSG_OPENDIR]   = hostfs_rpmsg_default_handler,
  [HOSTFS_RPMSG_READDIR]   = hostfs_rpmsg_readdir_handler,
  [HOSTFS_RPMSG_REWINDDIR] = hostfs_rpmsg_default_handler,
  [HOSTFS_RPMSG_CLOSEDIR]  = hostfs_rpmsg_default_handler,
  [HOSTFS_RPMSG_STATFS]    = hostfs_rpmsg_statfs_handler,
  [HOSTFS_RPMSG_UNLINK]    = hostfs_rpmsg_default_handler,
  [HOSTFS_RPMSG_MKDIR]     = hostfs_rpmsg_default_handler,
  [HOSTFS_RPMSG_RMDIR]     = hostfs_rpmsg_default_handler,
  [HOSTFS_RPMSG_RENAME]    = hostfs_rpmsg_default_handler,
  [HOSTFS_RPMSG_STAT]      = hostfs_rpmsg_stat_handler,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int hostfs_rpmsg_default_handler(FAR struct rpmsg_endpoint *ept,
                                        FAR void *data, size_t len,
                                        uint32_t src, FAR void *priv)
{
  FAR struct hostfs_rpmsg_header_s *header = data;
  FAR struct hostfs_rpmsg_cookie_s *cookie =
      (struct hostfs_rpmsg_cookie_s *)(uintptr_t)header->cookie;

  cookie->result = header->result;
  if (cookie->result >= 0 && cookie->data)
    {
      memcpy(cookie->data, data, len);
    }

  nxsem_post(&cookie->sem);

  return 0;
}

static int hostfs_rpmsg_read_handler(FAR struct rpmsg_endpoint *ept,
                                     FAR void *data, size_t len,
                                     uint32_t src, FAR void *priv)
{
  FAR struct hostfs_rpmsg_header_s *header = data;
  FAR struct hostfs_rpmsg_cookie_s *cookie =
      (struct hostfs_rpmsg_cookie_s *)(uintptr_t)header->cookie;
  FAR struct hostfs_rpmsg_read_s *rsp = data;

  cookie->result = header->result;
  if (cookie->result > 0)
    {
      memcpy(cookie->data, rsp->buf, B2C(cookie->result));
    }

  nxsem_post(&cookie->sem);

  return 0;
}

static int hostfs_rpmsg_readdir_handler(FAR struct rpmsg_endpoint *ept,
                                        FAR void *data, size_t len,
                                        uint32_t src, FAR void *priv)
{
  FAR struct hostfs_rpmsg_header_s *header = data;
  FAR struct hostfs_rpmsg_cookie_s *cookie =
      (struct hostfs_rpmsg_cookie_s *)(uintptr_t)header->cookie;
  FAR struct hostfs_rpmsg_readdir_s *rsp = data;
  FAR struct dirent *entry = cookie->data;

  cookie->result = header->result;
  if (cookie->result >= 0)
    {
      nbstr2cstr(entry->d_name, rsp->name, NAME_MAX);
      entry->d_name[NAME_MAX] = '\0';
      entry->d_type = rsp->type;
    }

  nxsem_post(&cookie->sem);

  return 0;
}

static int hostfs_rpmsg_statfs_handler(FAR struct rpmsg_endpoint *ept,
                                       FAR void *data, size_t len,
                                       uint32_t src, FAR void *priv)
{
  FAR struct hostfs_rpmsg_header_s *header = data;
  FAR struct hostfs_rpmsg_cookie_s *cookie =
      (struct hostfs_rpmsg_cookie_s *)(uintptr_t)header->cookie;
  FAR struct hostfs_rpmsg_statfs_s *rsp = data;
  FAR struct statfs *buf = cookie->data;

  cookie->result = header->result;
  if (cookie->result >= 0)
    {
      buf->f_type    = rsp->buf.f_type;
      buf->f_namelen = rsp->buf.f_namelen;
      buf->f_bsize   = B2C(rsp->buf.f_bsize);
      buf->f_blocks  = rsp->buf.f_blocks;
      buf->f_bfree   = rsp->buf.f_bfree;
      buf->f_bavail  = rsp->buf.f_bavail;
      buf->f_files   = rsp->buf.f_files;
      buf->f_ffree   = rsp->buf.f_ffree;
    }

  nxsem_post(&cookie->sem);

  return 0;
}

static int hostfs_rpmsg_stat_handler(FAR struct rpmsg_endpoint *ept,
                                     FAR void *data, size_t len,
                                     uint32_t src, FAR void *priv)
{
  FAR struct hostfs_rpmsg_header_s *header = data;
  FAR struct hostfs_rpmsg_cookie_s *cookie =
      (struct hostfs_rpmsg_cookie_s *)(uintptr_t)header->cookie;
  FAR struct hostfs_rpmsg_stat_s *rsp = data;
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
      buf->st_size    = B2C(rsp->buf.st_size);
      buf->st_atime   = rsp->buf.st_atime;
      buf->st_mtime   = rsp->buf.st_mtime;
      buf->st_ctime   = rsp->buf.st_ctime;
      buf->st_blksize = B2C(rsp->buf.st_blksize);
      buf->st_blocks  = rsp->buf.st_blocks;
    }

  nxsem_post(&cookie->sem);

  return 0;
}

static void hostfs_rpmsg_device_created(FAR struct rpmsg_device *rdev,
                                        FAR void *priv_)
{
  FAR struct hostfs_rpmsg_s *priv = priv_;

  if (strcmp(priv->cpuname, rpmsg_get_cpuname(rdev)) == 0)
    {
      priv->ept.priv = priv;
      rpmsg_create_ept(&priv->ept, rdev, HOSTFS_RPMSG_EPT_NAME,
                       RPMSG_ADDR_ANY, RPMSG_ADDR_ANY,
                       hostfs_rpmsg_ept_cb, NULL);
    }
}

static void hostfs_rpmsg_device_destroy(FAR struct rpmsg_device *rdev,
                                        FAR void *priv_)
{
  struct hostfs_rpmsg_s *priv = priv_;

  if (strcmp(priv->cpuname, rpmsg_get_cpuname(rdev)) == 0)
    {
      rpmsg_destroy_ept(&priv->ept);
    }
}

static int hostfs_rpmsg_ept_cb(FAR struct rpmsg_endpoint *ept,
                               FAR void *data, size_t len, uint32_t src,
                               FAR void *priv)
{
  FAR struct hostfs_rpmsg_header_s *header = data;
  uint32_t command = header->command;

  if (command < ARRAY_SIZE(g_hostfs_rpmsg_handler))
    {
      return g_hostfs_rpmsg_handler[command](ept, data, len, src, priv);
    }

  return -EINVAL;
}

static int hostfs_rpmsg_send_recv(uint32_t command, bool copy,
                                  FAR struct hostfs_rpmsg_header_s *msg,
                                  int len, FAR void *data)
{
  FAR struct hostfs_rpmsg_s *priv = &g_hostfs_rpmsg;
  FAR struct hostfs_rpmsg_cookie_s cookie;
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

int host_open(FAR const char *pathname, int flags, int mode)
{
  FAR struct hostfs_rpmsg_s *priv = &g_hostfs_rpmsg;
  FAR struct hostfs_rpmsg_open_s *msg;
  uint32_t space;
  size_t len;

  len  = sizeof(*msg);
  len += B2C(strlen(pathname) + 1);

  msg = rpmsg_get_tx_payload_buffer(&priv->ept, &space, true);
  if (!msg)
    {
      return -ENOMEM;
    }

  DEBUGASSERT(len <= space);

  msg->flags = flags;
  msg->mode  = mode;
  cstr2bstr(msg->pathname, pathname);

  return hostfs_rpmsg_send_recv(HOSTFS_RPMSG_OPEN, false,
          (struct hostfs_rpmsg_header_s *)msg, len, NULL);
}

int host_close(int fd)
{
  struct hostfs_rpmsg_close_s msg =
  {
    .fd = fd,
  };

  return hostfs_rpmsg_send_recv(HOSTFS_RPMSG_CLOSE, true,
          (struct hostfs_rpmsg_header_s *)&msg, sizeof(msg), NULL);
}

ssize_t host_read(int fd, FAR void *buf, size_t count)
{
  size_t read = 0;
  int ret = 0;

  while (read < count)
    {
      struct hostfs_rpmsg_read_s msg =
      {
        .fd    = fd,
        .count = C2B(count - read),
      };

      ret = hostfs_rpmsg_send_recv(HOSTFS_RPMSG_READ, true,
              (FAR struct hostfs_rpmsg_header_s *)&msg, sizeof(msg), buf);
      if (ret <= 0)
        {
          break;
        }

      read += B2C(ret);
      buf  += B2C(ret);
    }

  return read ? read : ret;
}

ssize_t host_write(int fd, FAR const void *buf, size_t count)
{
  FAR struct hostfs_rpmsg_s *priv = &g_hostfs_rpmsg;
  size_t written = 0;
  int ret = 0;

  while (written < count)
    {
      FAR struct hostfs_rpmsg_write_s *msg;
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
      msg->count = C2B(space);
      memcpy(msg->buf, buf + written, space);

      ret = hostfs_rpmsg_send_recv(HOSTFS_RPMSG_WRITE, false,
                                   (FAR struct hostfs_rpmsg_header_s *)msg,
                                   sizeof(*msg) + space, NULL);
      if (ret <= 0)
        {
          break;
        }

      written += B2C(ret);
    }

  return written ? written : ret;
}

off_t host_lseek(int fd, off_t offset, int whence)
{
  struct hostfs_rpmsg_lseek_s msg =
  {
    .fd     = fd,
    .offset = C2B(offset),
    .whence = whence,
  };

  int ret;

  ret = hostfs_rpmsg_send_recv(HOSTFS_RPMSG_LSEEK, true,
          (struct hostfs_rpmsg_header_s *)&msg, sizeof(msg), NULL);

  return ret < 0 ? ret : B2C(ret);
}

int host_ioctl(int fd, int request, unsigned long arg)
{
  struct hostfs_rpmsg_ioctl_s msg =
  {
    .fd      = fd,
    .request = request,
    .arg     = arg,
  };

  return hostfs_rpmsg_send_recv(HOSTFS_RPMSG_IOCTL, true,
          (struct hostfs_rpmsg_header_s *)&msg, sizeof(msg), NULL);
}

void host_sync(int fd)
{
  struct hostfs_rpmsg_sync_s msg =
  {
    .fd = fd,
  };

  hostfs_rpmsg_send_recv(HOSTFS_RPMSG_SYNC, true,
          (struct hostfs_rpmsg_header_s *)&msg, sizeof(msg), NULL);
}

int host_dup(int fd)
{
  struct hostfs_rpmsg_dup_s msg =
  {
    .fd = fd,
  };

  return hostfs_rpmsg_send_recv(HOSTFS_RPMSG_DUP, true,
          (struct hostfs_rpmsg_header_s *)&msg, sizeof(msg), NULL);
}

int host_fstat(int fd, struct stat *buf)
{
  struct hostfs_rpmsg_fstat_s msg =
  {
    .fd = fd,
  };

  return hostfs_rpmsg_send_recv(HOSTFS_RPMSG_FSTAT, true,
          (struct hostfs_rpmsg_header_s *)&msg, sizeof(msg), buf);
}

int host_ftruncate(int fd, off_t length)
{
  struct hostfs_rpmsg_ftruncate_s msg =
  {
    .fd     = fd,
    .length = length,
  };

  return hostfs_rpmsg_send_recv(HOSTFS_RPMSG_FTRUNCATE, true,
          (struct hostfs_rpmsg_header_s *)&msg, sizeof(msg), NULL);
}

FAR void *host_opendir(FAR const char *name)
{
  FAR struct hostfs_rpmsg_s *priv = &g_hostfs_rpmsg;
  FAR struct hostfs_rpmsg_opendir_s *msg;
  uint32_t space;
  size_t len;
  int ret;

  len  = sizeof(*msg);
  len += B2C(strlen(name) + 1);

  msg = rpmsg_get_tx_payload_buffer(&priv->ept, &space, true);
  if (!msg)
    {
      return NULL;
    }

  DEBUGASSERT(len <= space);

  cstr2bstr(msg->pathname, name);

  ret = hostfs_rpmsg_send_recv(HOSTFS_RPMSG_OPENDIR, false,
          (struct hostfs_rpmsg_header_s *)msg, len, NULL);

  return ret < 0 ? NULL : (FAR void *)((uintptr_t)ret);
}

int host_readdir(FAR void *dirp, FAR struct dirent *entry)
{
  struct hostfs_rpmsg_readdir_s msg =
  {
    .fd = (uintptr_t)dirp,
  };

  return hostfs_rpmsg_send_recv(HOSTFS_RPMSG_READDIR, true,
          (struct hostfs_rpmsg_header_s *)&msg, sizeof(msg), entry);
}

void host_rewinddir(FAR void *dirp)
{
  struct hostfs_rpmsg_rewinddir_s msg =
  {
    .fd = (uintptr_t)dirp,
  };

  hostfs_rpmsg_send_recv(HOSTFS_RPMSG_REWINDDIR, true,
          (struct hostfs_rpmsg_header_s *)&msg, sizeof(msg), NULL);
}

int host_closedir(FAR void *dirp)
{
  struct hostfs_rpmsg_closedir_s msg =
  {
    .fd = (uintptr_t)dirp,
  };

  return hostfs_rpmsg_send_recv(HOSTFS_RPMSG_CLOSEDIR, true,
          (struct hostfs_rpmsg_header_s *)&msg, sizeof(msg), NULL);
}

int host_statfs(FAR const char *path, FAR struct statfs *buf)
{
  struct hostfs_rpmsg_s *priv = &g_hostfs_rpmsg;
  struct hostfs_rpmsg_statfs_s *msg;
  uint32_t space;
  size_t len;

  len  = sizeof(*msg);
  len += B2C(strlen(path) + 1);

  msg = rpmsg_get_tx_payload_buffer(&priv->ept, &space, true);
  if (!msg)
    {
      return -ENOMEM;
    }

  DEBUGASSERT(len <= space);

  cstr2bstr(msg->pathname, path);

  return hostfs_rpmsg_send_recv(HOSTFS_RPMSG_STATFS, false,
          (struct hostfs_rpmsg_header_s *)msg, len, buf);
}

int host_unlink(FAR const char *pathname)
{
  struct hostfs_rpmsg_s *priv = &g_hostfs_rpmsg;
  struct hostfs_rpmsg_unlink_s *msg;
  uint32_t space;
  size_t len;

  len  = sizeof(*msg);
  len += B2C(strlen(pathname) + 1);

  msg = rpmsg_get_tx_payload_buffer(&priv->ept, &space, true);
  if (!msg)
    {
      return -ENOMEM;
    }

  DEBUGASSERT(len <= space);

  cstr2bstr(msg->pathname, pathname);

  return hostfs_rpmsg_send_recv(HOSTFS_RPMSG_UNLINK, false,
          (struct hostfs_rpmsg_header_s *)msg, len, NULL);
}

int host_mkdir(FAR const char *pathname, mode_t mode)
{
  struct hostfs_rpmsg_s *priv = &g_hostfs_rpmsg;
  struct hostfs_rpmsg_mkdir_s *msg;
  uint32_t space;
  size_t len;

  len  = sizeof(*msg);
  len += B2C(strlen(pathname) + 1);

  msg = rpmsg_get_tx_payload_buffer(&priv->ept, &space, true);
  if (!msg)
    {
      return -ENOMEM;
    }

  msg->mode = mode;
  cstr2bstr(msg->pathname, pathname);

  return hostfs_rpmsg_send_recv(HOSTFS_RPMSG_MKDIR, false,
          (struct hostfs_rpmsg_header_s *)msg, len, NULL);
}

int host_rmdir(FAR const char *pathname)
{
  struct hostfs_rpmsg_s *priv = &g_hostfs_rpmsg;
  struct hostfs_rpmsg_rmdir_s *msg;
  uint32_t space;
  size_t len;

  len  = sizeof(*msg);
  len += B2C(strlen(pathname) + 1);

  msg = rpmsg_get_tx_payload_buffer(&priv->ept, &space, true);
  if (!msg)
    {
      return -ENOMEM;
    }

  DEBUGASSERT(len <= space);

  cstr2bstr(msg->pathname, pathname);

  return hostfs_rpmsg_send_recv(HOSTFS_RPMSG_RMDIR, false,
          (struct hostfs_rpmsg_header_s *)msg, len, NULL);
}

int host_rename(FAR const char *oldpath, FAR const char *newpath)
{
  struct hostfs_rpmsg_s *priv = &g_hostfs_rpmsg;
  struct hostfs_rpmsg_rename_s *msg;
  size_t len;
  size_t oldlen;
  uint32_t space;

  len     = sizeof(*msg);
  oldlen  = B2C((strlen(oldpath) + 1 + 0x7) & ~0x7);
  len    += oldlen + B2C(strlen(newpath) + 1);

  msg = rpmsg_get_tx_payload_buffer(&priv->ept, &space, true);
  if (!msg)
    {
      return -ENOMEM;
    }

  DEBUGASSERT(len <= space);

  cstr2bstr(msg->pathname, oldpath);
  cstr2bstr(msg->pathname + oldlen, newpath);

  return hostfs_rpmsg_send_recv(HOSTFS_RPMSG_RENAME, false,
          (struct hostfs_rpmsg_header_s *)msg, len, NULL);
}

int host_stat(FAR const char *path, FAR struct stat *buf)
{
  FAR struct hostfs_rpmsg_s *priv = &g_hostfs_rpmsg;
  FAR struct hostfs_rpmsg_stat_s *msg;
  uint32_t space;
  size_t len;

  len  = sizeof(*msg);
  len += B2C(strlen(path) + 1);

  msg = rpmsg_get_tx_payload_buffer(&priv->ept, &space, true);
  if (!msg)
    {
      return -ENOMEM;
    }

  DEBUGASSERT(len <= space);

  cstr2bstr(msg->pathname, path);

  return hostfs_rpmsg_send_recv(HOSTFS_RPMSG_STAT, false,
          (struct hostfs_rpmsg_header_s *)msg, len, buf);
}

int hostfs_rpmsg_init(FAR const char *cpuname)
{
  struct hostfs_rpmsg_s *priv = &g_hostfs_rpmsg;

  priv->cpuname = cpuname;

  return rpmsg_register_callback(priv,
                                 hostfs_rpmsg_device_created,
                                 hostfs_rpmsg_device_destroy,
                                 NULL);
}
