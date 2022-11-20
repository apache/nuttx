/****************************************************************************
 * drivers/misc/rpmsgdev_server.c
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

#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/list.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/fs/fs.h>
#include <nuttx/drivers/rpmsgdev.h>
#include <nuttx/rptun/openamp.h>

#include "rpmsgdev.h"

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct rpmsgdev_device_s
{
  struct file      file;  /* The open file */
  struct pollfd    fd;    /* The poll fd */
  uint64_t         cfd;   /* The client poll fd pointer */
  struct list_node node;  /* The double-linked list node */
};

struct rpmsgdev_server_s
{
  struct rpmsg_endpoint ept;   /* Rpmsg end point */
  struct list_node      head;  /* The double-linked list head of opened
                                * devices
                                */
  mutex_t               lock;  /* The mutex used to protect the list
                                * operation
                                */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Functions handle the messages from the client cpu */

static int  rpmsgdev_open_handler(FAR struct rpmsg_endpoint *ept,
                                  FAR void *data, size_t len,
                                  uint32_t src, FAR void *priv);
static int  rpmsgdev_close_handler(FAR struct rpmsg_endpoint *ept,
                                   FAR void *data, size_t len,
                                   uint32_t src, FAR void *priv);
static int  rpmsgdev_read_handler(FAR struct rpmsg_endpoint *ept,
                                  FAR void *data, size_t len,
                                  uint32_t src, FAR void *priv);
static int  rpmsgdev_write_handler(FAR struct rpmsg_endpoint *ept,
                                   FAR void *data, size_t len,
                                   uint32_t src, FAR void *priv);
static int  rpmsgdev_lseek_handler(FAR struct rpmsg_endpoint *ept,
                                   FAR void *data, size_t len,
                                   uint32_t src, FAR void *priv);
static int  rpmsgdev_ioctl_handler(FAR struct rpmsg_endpoint *ept,
                                   FAR void *data, size_t len,
                                   uint32_t src, FAR void *priv);
static void rpmsgdev_poll_cb(FAR struct pollfd *fds);
static int  rpmsgdev_poll_handler(FAR struct rpmsg_endpoint *ept,
                                  FAR void *data, size_t len,
                                  uint32_t src, FAR void *priv);

/* Functions for creating communication with client cpu */

static bool rpmsgdev_ns_match(FAR struct rpmsg_device *rdev,
                              FAR void *priv, FAR const char *name,
                              uint32_t dest);
static void rpmsgdev_ns_bind(FAR struct rpmsg_device *rdev,
                             FAR void *priv, FAR const char *name,
                             uint32_t dest);
static void rpmsgdev_ns_unbind(FAR struct rpmsg_endpoint *ept);
static int  rpmsgdev_ept_cb(FAR struct rpmsg_endpoint *ept,
                            FAR void *data, size_t len, uint32_t src,
                            FAR void *priv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const rpmsg_ept_cb g_rpmsgdev_handler[] =
{
  [RPMSGDEV_OPEN]  = rpmsgdev_open_handler,
  [RPMSGDEV_CLOSE] = rpmsgdev_close_handler,
  [RPMSGDEV_READ]  = rpmsgdev_read_handler,
  [RPMSGDEV_WRITE] = rpmsgdev_write_handler,
  [RPMSGDEV_LSEEK] = rpmsgdev_lseek_handler,
  [RPMSGDEV_IOCTL] = rpmsgdev_ioctl_handler,
  [RPMSGDEV_POLL]  = rpmsgdev_poll_handler,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rpmsgdev_open_handler
 ****************************************************************************/

static int rpmsgdev_open_handler(FAR struct rpmsg_endpoint *ept,
                                 FAR void *data, size_t len,
                                 uint32_t src, FAR void *priv)
{
  FAR struct rpmsgdev_server_s *server = ept->priv;
  FAR struct rpmsgdev_open_s *msg = data;
  FAR struct rpmsgdev_device_s *dev;

  dev = kmm_zalloc(sizeof(*dev));
  if (dev == NULL)
    {
      msg->header.result = -ENOMEM;
      goto out;
    }

  msg->header.result = file_open(&dev->file,
                                 &ept->name[RPMSGDEV_NAME_PREFIX_LEN],
                                 msg->flags, 0);
  if (msg->header.result < 0)
    {
      kmm_free(dev);
      goto out;
    }

  msg->filep = (uint64_t)(uintptr_t)&dev->file;

  nxmutex_lock(&server->lock);
  list_add_tail(&server->head, &dev->node);
  nxmutex_unlock(&server->lock);

out:
  return rpmsg_send(ept, msg, sizeof(*msg));
}

/****************************************************************************
 * Name: rpmsgdev_close_handler
 ****************************************************************************/

static int rpmsgdev_close_handler(FAR struct rpmsg_endpoint *ept,
                                  FAR void *data, size_t len,
                                  uint32_t src, FAR void *priv)
{
  FAR struct rpmsgdev_server_s *server = ept->priv;
  FAR struct rpmsgdev_close_s *msg = data;
  FAR struct rpmsgdev_device_s *dev =
    (FAR struct rpmsgdev_device_s *)(uintptr_t)msg->filep;

  msg->header.result = file_close(&dev->file);
  if (msg->header.result == 0)
    {
      nxmutex_lock(&server->lock);
      list_delete(&dev->node);
      nxmutex_unlock(&server->lock);

      kmm_free(dev);
    }

  return rpmsg_send(ept, msg, sizeof(*msg));
}

/****************************************************************************
 * Name: rpmsgdev_read_handler
 ****************************************************************************/

static int rpmsgdev_read_handler(FAR struct rpmsg_endpoint *ept,
                                 FAR void *data, size_t len,
                                 uint32_t src, FAR void *priv)
{
  FAR struct rpmsgdev_read_s *msg = data;
  FAR struct rpmsgdev_read_s *rsp;
  FAR struct file *filep = (FAR struct file *)(uintptr_t)msg->filep;
  int ret = -ENOENT;
  size_t read = 0;
  uint32_t space;

  while (read < msg->count)
    {
      rsp = rpmsg_get_tx_payload_buffer(ept, &space, true);
      if (rsp == NULL)
        {
          return -ENOMEM;
        }

      *rsp = *msg;

      space -= sizeof(*msg) - 1;
      if (space > msg->count - read)
        {
          space = msg->count - read;
        }

      ret = file_read(filep, rsp->buf, space);

      rsp->header.result = ret;
      rpmsg_send_nocopy(ept, rsp, (ret < 0 ? 0 : ret) + sizeof(*rsp) - 1);
      if (ret <= 0)
        {
          break;
        }

      read += ret;
    }

  return 0;
}

/****************************************************************************
 * Name: rpmsgdev_write_handler
 ****************************************************************************/

static int rpmsgdev_write_handler(FAR struct rpmsg_endpoint *ept,
                                  FAR void *data, size_t len,
                                  uint32_t src, FAR void *priv)
{
  FAR struct rpmsgdev_write_s *msg = data;
  FAR struct file *filep = (FAR struct file *)(uintptr_t)msg->filep;
  size_t written = 0;
  int ret = -ENOENT;

  while (written < msg->count)
    {
      ret = file_write(filep, msg->buf + written,
                       msg->count - written);
      if (ret <= 0)
        {
          break;
        }

      written += ret;
    }

  if (msg->header.cookie != 0)
    {
      msg->header.result = ret < 0 ? ret : written;
      rpmsg_send(ept, msg, sizeof(*msg) - 1);
    }

  return 0;
}

/****************************************************************************
 * Name: rpmsgdev_lseek_handler
 ****************************************************************************/

static int rpmsgdev_lseek_handler(FAR struct rpmsg_endpoint *ept,
                                  FAR void *data, size_t len,
                                  uint32_t src, FAR void *priv)
{
  FAR struct rpmsgdev_lseek_s *msg = data;
  FAR struct file *filep = (FAR struct file *)(uintptr_t)msg->filep;

  msg->header.result = file_seek(filep, msg->offset, msg->whence);

  return rpmsg_send(ept, msg, len);
}

/****************************************************************************
 * Name: rpmsgdev_ioctl_handler
 ****************************************************************************/

static int rpmsgdev_ioctl_handler(FAR struct rpmsg_endpoint *ept,
                                  FAR void *data, size_t len,
                                  uint32_t src, FAR void *priv)
{
  FAR struct rpmsgdev_ioctl_s *msg = data;
  FAR struct file *filep = (FAR struct file *)(uintptr_t)msg->filep;

  msg->header.result = file_ioctl(filep, msg->request,
                                  msg->arglen > 0 ? (unsigned long)msg->buf :
                                  msg->arg);

  return rpmsg_send(ept, msg, len);
}

/****************************************************************************
 * Name: rpmsgdev_poll_cb
 ****************************************************************************/

static void rpmsgdev_poll_cb(FAR struct pollfd *fds)
{
  FAR struct rpmsgdev_server_s *server = fds->arg;
  FAR struct rpmsgdev_device_s *dev =
    container_of(fds, FAR struct rpmsgdev_device_s, fd);
  FAR struct rpmsgdev_notify_s msg;

  DEBUGASSERT(fds != NULL && dev->cfd != 0);

  msg.header.command = RPMSGDEV_NOTIFY;
  msg.revents = fds->revents;
  msg.fds     = dev->cfd;

  fds->revents = 0;

  rpmsg_send(&server->ept, &msg, sizeof(msg));
}

/****************************************************************************
 * Name: rpmsgdev_poll_handler
 ****************************************************************************/

static int rpmsgdev_poll_handler(FAR struct rpmsg_endpoint *ept,
                                 FAR void *data, size_t len,
                                 uint32_t src, FAR void *priv)
{
  FAR struct rpmsgdev_server_s *server = ept->priv;
  FAR struct rpmsgdev_poll_s *msg = data;
  FAR struct rpmsgdev_device_s *dev =
    (FAR struct rpmsgdev_device_s *)(uintptr_t)msg->filep;

  DEBUGASSERT(msg->fds != 0);

  if (msg->setup)
    {
      /* Do not allow double setup */

      DEBUGASSERT(dev->cfd == 0);

      dev->cfd        = msg->fds;
      dev->fd.events  = msg->events;
      dev->fd.revents = 0;
      dev->fd.cb      = rpmsgdev_poll_cb;
      dev->fd.arg     = server;

      msg->header.result = file_poll(&dev->file, &dev->fd, true);
    }
  else
    {
      msg->header.result = file_poll(&dev->file, &dev->fd, false);
      if (msg->header.result == OK)
        {
          dev->cfd = 0;
        }
    }

  return rpmsg_send(ept, msg, len);
}

/****************************************************************************
 * Name: rpmsgdev_ns_match
 ****************************************************************************/

static bool rpmsgdev_ns_match(FAR struct rpmsg_device *rdev,
                              FAR void *priv, FAR const char *name,
                              uint32_t dest)
{
  return !strncmp(name, RPMSGDEV_NAME_PREFIX, RPMSGDEV_NAME_PREFIX_LEN);
}

/****************************************************************************
 * Name: rpmsgdev_ns_bind
 ****************************************************************************/

static void rpmsgdev_ns_bind(FAR struct rpmsg_device *rdev,
                             FAR void *priv, FAR const char *name,
                             uint32_t dest)
{
  FAR struct rpmsgdev_server_s *server;
  int ret;

  server = kmm_zalloc(sizeof(*server));
  if (server == NULL)
    {
      return;
    }

  list_initialize(&server->head);
  nxmutex_init(&server->lock);
  server->ept.priv = server;

  ret = rpmsg_create_ept(&server->ept, rdev, name,
                         RPMSG_ADDR_ANY, dest,
                         rpmsgdev_ept_cb, rpmsgdev_ns_unbind);
  if (ret < 0)
    {
      nxmutex_destroy(&server->lock);
      kmm_free(server);
    }
}

/****************************************************************************
 * Name: rpmsgdev_ns_unbind
 ****************************************************************************/

static void rpmsgdev_ns_unbind(FAR struct rpmsg_endpoint *ept)
{
  FAR struct rpmsgdev_server_s *server = ept->priv;
  FAR struct rpmsgdev_device_s *dev;
  FAR struct rpmsgdev_device_s *tmpdev;

  nxmutex_lock(&server->lock);

  list_for_every_entry_safe(&server->head, dev, tmpdev,
                            struct rpmsgdev_device_s, node)
    {
      if (dev->cfd != 0)
        {
          file_poll(&dev->file, &dev->fd, false);
        }

      if (dev->file.f_inode != NULL)
        {
          file_close(&dev->file);
        }

      list_delete(&dev->node);
      kmm_free(dev);
    }

  nxmutex_unlock(&server->lock);

  rpmsg_destroy_ept(&server->ept);
  kmm_free(server);
}

/****************************************************************************
 * Name: rpmsgdev_ept_cb
 ****************************************************************************/

static int rpmsgdev_ept_cb(FAR struct rpmsg_endpoint *ept,
                           FAR void *data, size_t len, uint32_t src,
                           FAR void *priv)
{
  FAR struct rpmsgdev_header_s *header = data;
  uint32_t command = header->command;

  if (command < ARRAY_SIZE(g_rpmsgdev_handler))
    {
      return g_rpmsgdev_handler[command](ept, data, len, src, priv);
    }

  return -EINVAL;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rpmsgdev_server_init
 *
 * Description:
 *   Rpmsg-device server initialize function, the server cpu should call
 *   this function.
 *
 * Parameters:
 *   None
 *
 * Returned Values:
 *   OK on success; A negated errno value is returned on any failure.
 *
 ****************************************************************************/

int rpmsgdev_server_init(void)
{
  return rpmsg_register_callback(NULL,
                                 NULL,
                                 NULL,
                                 rpmsgdev_ns_match,
                                 rpmsgdev_ns_bind);
}
