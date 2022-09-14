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

#include <nuttx/kmalloc.h>
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

struct rpmsgdev_server_s
{
  struct rpmsg_endpoint ept;
  struct file           file;
  struct pollfd         fds[CONFIG_DEV_RPMSG_NPOLLWAITERS];
  uint64_t              cfds[CONFIG_DEV_RPMSG_NPOLLWAITERS];
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

  msg->header.result = file_open(&server->file,
                                 &ept->name[RPMSGDEV_NAME_PREFIX_LEN],
                                 msg->flags, 0);

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

  msg->header.result = file_close(&server->file);

  return rpmsg_send(ept, msg, sizeof(*msg));
}

/****************************************************************************
 * Name: rpmsgdev_read_handler
 ****************************************************************************/

static int rpmsgdev_read_handler(FAR struct rpmsg_endpoint *ept,
                                 FAR void *data, size_t len,
                                 uint32_t src, FAR void *priv)
{
  FAR struct rpmsgdev_server_s *server = ept->priv;
  FAR struct rpmsgdev_read_s *msg = data;
  FAR struct rpmsgdev_read_s *rsp;
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

      ret = file_read(&server->file, rsp->buf, space);

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
  FAR struct rpmsgdev_server_s *server = ept->priv;
  FAR struct rpmsgdev_write_s *msg = data;
  size_t written = 0;
  int ret = -ENOENT;

  while (written < msg->count)
    {
      ret = file_write(&server->file, msg->buf + written,
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
  FAR struct rpmsgdev_server_s *server = ept->priv;
  FAR struct rpmsgdev_lseek_s *msg = data;

  msg->header.result = file_seek(&server->file, msg->offset, msg->whence);

  return rpmsg_send(ept, msg, len);
}

/****************************************************************************
 * Name: rpmsgdev_ioctl_handler
 ****************************************************************************/

static int rpmsgdev_ioctl_handler(FAR struct rpmsg_endpoint *ept,
                                  FAR void *data, size_t len,
                                  uint32_t src, FAR void *priv)
{
  FAR struct rpmsgdev_server_s *server = ept->priv;
  FAR struct rpmsgdev_ioctl_s *msg = data;

  msg->header.result = file_ioctl(&server->file, msg->request,
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
  FAR struct rpmsgdev_notify_s msg;
  int tmp;

  DEBUGASSERT(fds != NULL && (uintptr_t)fds >= (uintptr_t)server->fds);

  tmp = fds - server->fds;

  msg.header.command = RPMSGDEV_NOTIFY;
  msg.revents = fds->revents;
  msg.fds     = server->cfds[tmp];

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
  FAR struct pollfd *fds = NULL;
  int i;

  DEBUGASSERT(msg->fds != 0);

  if (msg->setup)
    {
      for (i = 0; i < CONFIG_DEV_RPMSG_NPOLLWAITERS; i++)
        {
          if (server->cfds[i] == 0)
            {
              server->cfds[i] = msg->fds;
              break;
            }
        }

      if (i >= CONFIG_DEV_RPMSG_NPOLLWAITERS)
        {
          msg->header.result = -EBUSY;
          goto out;
        }

      fds          = &server->fds[i];
      fds->events  = msg->events;
      fds->revents = 0;
      fds->cb      = rpmsgdev_poll_cb;
      fds->arg     = server;

      msg->header.result = file_poll(&server->file, fds, true);
    }
  else
    {
      for (i = 0; i < CONFIG_DEV_RPMSG_NPOLLWAITERS; i++)
        {
          if (server->cfds[i] == msg->fds)
            {
              break;
            }
        }

      if (i >= CONFIG_DEV_RPMSG_NPOLLWAITERS)
        {
          msg->header.result = -EINVAL;
          goto out;
        }

      fds = &server->fds[i];
      msg->header.result = file_poll(&server->file, fds, false);
      if (msg->header.result == OK)
        {
          server->cfds[i] = 0;
        }
    }

out:
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

  server->ept.priv = server;

  ret = rpmsg_create_ept(&server->ept, rdev, name,
                         RPMSG_ADDR_ANY, dest,
                         rpmsgdev_ept_cb, rpmsgdev_ns_unbind);
  if (ret < 0)
    {
      kmm_free(server);
    }
}

/****************************************************************************
 * Name: rpmsgdev_ns_unbind
 ****************************************************************************/

static void rpmsgdev_ns_unbind(FAR struct rpmsg_endpoint *ept)
{
  FAR struct rpmsgdev_server_s *server = ept->priv;

  file_close(&server->file);
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
