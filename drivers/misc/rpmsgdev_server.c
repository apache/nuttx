/****************************************************************************
 * drivers/misc/rpmsgdev_server.c
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

#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <nuttx/debug.h>

#include <nuttx/nuttx.h>
#include <nuttx/list.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/fs/fs.h>
#include <nuttx/percpu.h>
#include <nuttx/wqueue.h>
#include <nuttx/drivers/rpmsgdev.h>
#include <nuttx/rpmsg/rpmsg.h>
#include <nuttx/mtd/configdata.h>

#include "rpmsgdev.h"

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct rpmsgdev_read_pend_s
{
  struct pollfd_s        fd;
  struct rpmsgdev_read_s rsp;
  struct work_s          work;
  bool                   cancel;
};

/* The device slots are preallocated in the endpoint instance and managed by
 * two lists: 'free' holds the unused slots and 'head' the opened ones.  A
 * slot is claimed by removing it from the free list under the lock, which is
 * atomic with respect to any other open, so no in-use flag is needed.
 */

struct rpmsgdev_device_s
{
  struct file                 file;      /* The open file */
  struct pollfd_s             fd;        /* The poll fd */
  uint64_t                    cfd;       /* The client poll fd pointer */
  struct list_node            node;      /* Free or opened list node */
  struct work_s               work;      /* Poll notify work */
  struct rpmsgdev_read_pend_s readpend;
};

struct rpmsgdev_export_s
{
  FAR const char *remotecpu;  /* The client cpu name */
  FAR const char *prefix;     /* The device prefix */
  FAR const char *localpath;  /* The device path in the server cpu */
};

struct rpmsgdev_server_s
{
  struct rpmsg_endpoint        ept;   /* Rpmsg end point */
  struct list_node             head;  /* List head of the opened devices */
  struct list_node             free;  /* List head of the free device slots */
  mutex_t                      lock;  /* Protect the device lists below */
  FAR struct rpmsgdev_export_s *export;
  struct rpmsgdev_device_s     dev[CONFIG_DEV_RPMSG_OPEN_MAX];

                               /* Preallocated per-open device slots */
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
static void rpmsgdev_read_cancel(FAR struct rpmsgdev_device_s *dev);
static int  rpmsgdev_read_nonblock_handler(FAR struct rpmsg_endpoint *ept,
                                           FAR void *data, size_t len,
                                           uint32_t src, FAR void *priv);
static void rpmsgdev_read_work(FAR void *arg);
static void rpmsgdev_read_cb(FAR struct pollfd_s *fds);
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
static void rpmsgdev_poll_worker(FAR void *arg);
static void rpmsgdev_poll_cb(FAR struct pollfd_s *fds);
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
static int  rpmsgdev_server_ept_cb(FAR struct rpmsg_endpoint *ept,
                                   FAR void *data, size_t len, uint32_t src,
                                   FAR void *priv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const rpmsg_ept_cb g_rpmsgdev_handler[] =
{
  [RPMSGDEV_OPEN]          = rpmsgdev_open_handler,
  [RPMSGDEV_CLOSE]         = rpmsgdev_close_handler,
  [RPMSGDEV_READ_NONBLOCK] = rpmsgdev_read_nonblock_handler,
  [RPMSGDEV_READ]          = rpmsgdev_read_handler,
  [RPMSGDEV_WRITE]         = rpmsgdev_write_handler,
  [RPMSGDEV_LSEEK]         = rpmsgdev_lseek_handler,
  [RPMSGDEV_IOCTL]         = rpmsgdev_ioctl_handler,
  [RPMSGDEV_POLL]          = rpmsgdev_poll_handler,
};

static DEFINE_PER_CPU_BSS_BMP(FAR struct kwork_wqueue_s *,
                              g_rpmsgdev_wqueue);
#define g_rpmsgdev_wqueue this_cpu_var_bmp(g_rpmsgdev_wqueue)

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
  FAR struct rpmsgdev_export_s *export = server->export;
  FAR struct rpmsgdev_open_s *msg = data;
  FAR struct rpmsgdev_device_s *dev;

  /* Claim a free device slot by removing it from the free list */

  nxmutex_lock(&server->lock);
  dev = list_remove_head_type(&server->free, struct rpmsgdev_device_s, node);
  nxmutex_unlock(&server->lock);

  if (dev == NULL)
    {
      msg->header.result = -ENOMEM;
      goto out;
    }

  /* Clear the stale poll/work state left by a previous user of this slot.
   * The node is unlinked while the slot is claimed, so clearing it is safe.
   */

  memset(dev, 0, sizeof(*dev));

  msg->header.result = file_open(&dev->file,
                                 export ? export->localpath :
                                 strchr(ept->name, '/'),
                                 msg->flags, 0);

  nxmutex_lock(&server->lock);
  if (msg->header.result < 0)
    {
      list_add_tail(&server->free, &dev->node);
    }
  else
    {
      list_add_tail(&server->head, &dev->node);
      msg->filep = (uint64_t)(uintptr_t)&dev->file;
    }

  nxmutex_unlock(&server->lock);

out:
  return rpmsg_send(ept, msg, sizeof(*msg));
}

/****************************************************************************
 * Name: rpmsgdev_read_cancel
 *
 * Description:
 *   Drop a blocking read that is still waiting for data, so that the
 *   device no longer calls back into the slot.
 *
 ****************************************************************************/

static void rpmsgdev_read_cancel(FAR struct rpmsgdev_device_s *dev)
{
  /* A running read work may poll the device again: stop it first, then
   * tear down the poll, then drop a work the poll queued meanwhile.
   */

  dev->readpend.cancel = true;
  work_cancel_sync_wq(g_rpmsgdev_wqueue, &dev->readpend.work);

  if (dev->readpend.fd.priv != NULL)
    {
      file_poll(&dev->file, &dev->readpend.fd, false);
    }

  work_cancel_sync_wq(g_rpmsgdev_wqueue, &dev->readpend.work);
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

  rpmsgdev_read_cancel(dev);
  msg->header.result = file_close(&dev->file);
  if (msg->header.result == 0)
    {
      nxmutex_lock(&server->lock);
      list_delete(&dev->node);
      work_cancel_sync_wq(g_rpmsgdev_wqueue, &dev->work);
      work_cancel_sync_wq(g_rpmsgdev_wqueue, &dev->readpend.work);
      list_add_tail(&server->free, &dev->node);
      nxmutex_unlock(&server->lock);
    }

  return rpmsg_send(ept, msg, sizeof(*msg));
}

/****************************************************************************
 * Name: rpmsgdev_read_nonblock_handler
 ****************************************************************************/

static int rpmsgdev_read_nonblock_handler(FAR struct rpmsg_endpoint *ept,
                                          FAR void *data, size_t len,
                                          uint32_t src, FAR void *priv)
{
  FAR struct rpmsgdev_read_s *msg = data;
  FAR struct rpmsgdev_read_s *rsp;
  FAR struct file *filep = (FAR struct file *)(uintptr_t)msg->filep;
  bool loop = true;
  size_t read = 0;
  uint32_t space;
  ssize_t ret;

  while (loop && read < msg->count)
    {
      rsp = rpmsg_get_tx_payload_buffer(ept, &space, true);
      if (rsp == NULL)
        {
          return -ENOMEM;
        }

      *rsp = *msg;

      space -= sizeof(*msg) - 1;
      if (space >= msg->count)
        {
          loop = false;
        }

      space = MIN(space, msg->count - read);

      ret = file_read(filep, rsp->buf, space);
      if (ret == -EAGAIN && read == 0 &&
          msg->header.command == RPMSGDEV_READ)
        {
          rpmsg_release_tx_buffer(ept, rsp);
          return ret;
        }

      if (ret > 0)
        {
          read += ret;
          rsp->count = loop ? msg->count - read : 0;
        }
      else
        {
          rsp->count = 0;
          loop = false;
        }

      rsp->header.result = ret;
      if (rpmsg_send_nocopy(ept, rsp,
                            (ret < 0 ? 0 : ret) + sizeof(*rsp) - 1) < 0)
        {
          rpmsg_release_tx_buffer(ept, rsp);
        }
    }

  return 0;
}

/****************************************************************************
 * Name: rpmsgdev_read_work
 ****************************************************************************/

static void rpmsgdev_read_work(FAR void *arg)
{
  FAR struct pollfd_s *fds = arg;
  FAR struct rpmsgdev_server_s *server = fds->arg;
  FAR struct rpmsgdev_read_pend_s *pend =
            container_of(fds, FAR struct rpmsgdev_read_pend_s, fd);
  FAR struct rpmsgdev_device_s *dev =
    container_of(pend, FAR struct rpmsgdev_device_s, readpend);
  FAR struct rpmsgdev_read_s *rsp = &pend->rsp;

  rsp->header.result = file_poll(&dev->file, &pend->fd, false);
  if (rsp->header.result < 0)
    {
      rpmsg_send(&server->ept, rsp, sizeof(*rsp) - 1);
    }
  else if (rpmsgdev_read_nonblock_handler(&server->ept, rsp, sizeof(*rsp),
                                          0, NULL) == -EAGAIN &&
           !pend->cancel)
    {
      /* Another reader took the data: wait for the next POLLIN */

      pend->fd.revents = 0;
      pend->fd.cb      = rpmsgdev_read_cb;
      rsp->header.result = file_poll(&dev->file, &pend->fd, true);
      if (rsp->header.result < 0)
        {
          rpmsg_send(&server->ept, rsp, sizeof(*rsp) - 1);
        }
    }
}

/****************************************************************************
 * Name: rpmsgdev_read_cb
 ****************************************************************************/

static void rpmsgdev_read_cb(FAR struct pollfd_s *fds)
{
  FAR struct rpmsgdev_read_pend_s *pend =
              container_of(fds, FAR struct rpmsgdev_read_pend_s, fd);

  DEBUGASSERT(fds != NULL);

  /* Prevent subsequent poll_notify from re-triggering this callback.
   * Without this guard, if the device IRQ fires again before
   * rpmsgdev_read_work executes file_poll(teardown), the work will be
   * queued a second time (NuttX work_queue allows re-queue when
   * worker==NULL during execution), causing the same READ response
   * cookie to be sent twice and leading to a use-after-free crash
   * on the remote side.
   */

  fds->cb = NULL;

  work_queue_wq(g_rpmsgdev_wqueue, &pend->work,
                rpmsgdev_read_work, fds, 0);
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
  FAR struct rpmsgdev_device_s *dev =
    (FAR struct rpmsgdev_device_s *)(uintptr_t)msg->filep;
  FAR struct rpmsgdev_read_pend_s *pend = &dev->readpend;
  FAR struct rpmsgdev_read_s *rsp = &pend->rsp;

  pend->fd.events  = POLLIN;
  pend->fd.revents = 0;
  pend->fd.cb      = rpmsgdev_read_cb;
  pend->fd.arg     = server;
  pend->cancel     = false;

  memcpy(rsp, msg, sizeof(*msg));

  rsp->header.result = file_poll(&dev->file, &pend->fd, true);
  if (rsp->header.result < 0)
    {
      rpmsg_send(ept, rsp, sizeof(*rsp) - 1);
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

  if (written != 0)
    {
      msg->header.result = written;
    }
  else
    {
      msg->header.result = ret;
    }

  return rpmsg_send(ept, msg, sizeof(*msg) - 1);
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

  msg->header.result = 0;
  msg->offset = file_seek(filep, msg->offset, msg->whence);
  if (msg->offset < 0)
    {
      msg->header.result = (int32_t)msg->offset;
    }

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
  bool   is_cfg = (msg->request >= CFGDIOC_GETCONFIG &&
                   msg->request <= CFGDIOC_NEXTCONFIG);
  size_t cfglen = 0;

  if (is_cfg)
    {
      /* msg->buf is a char[] inside a packed wire struct and may be
       * unaligned; use memcpy to read config_data_s::len safely instead
       * of casting to (struct config_data_s *).
       */

      memcpy(&cfglen,
             msg->buf + offsetof(struct config_data_s, len),
             sizeof(cfglen));
    }

  if (is_cfg && msg->arglen != sizeof(struct config_data_s) + cfglen)
    {
      msg->header.result = -EMSGSIZE;
    }
  else
    {
      msg->header.result = file_ioctl(filep, msg->request,
                                      msg->arglen == 0 ? msg->arg :
                                      (unsigned long)(FAR void *)msg->buf);
    }

  return rpmsg_send(ept, msg, len);
}

/****************************************************************************
 * Name: rpmsgdev_poll_worker
 ****************************************************************************/

static void rpmsgdev_poll_worker(FAR void *arg)
{
  FAR struct pollfd_s *fds = arg;
  FAR struct rpmsgdev_server_s *server = fds->arg;
  FAR struct rpmsgdev_device_s *dev =
    container_of(fds, FAR struct rpmsgdev_device_s, fd);
  FAR struct rpmsgdev_notify_s msg;

  if (dev->cfd != 0)
    {
      msg.header.command = RPMSGDEV_NOTIFY;
      msg.revents = fds->revents;
      msg.fds     = dev->cfd;

      fds->revents = 0;

      rpmsg_send(&server->ept, &msg, sizeof(msg));
    }
}

/****************************************************************************
 * Name: rpmsgdev_poll_cb
 ****************************************************************************/

static void rpmsgdev_poll_cb(FAR struct pollfd_s *fds)
{
  FAR struct rpmsgdev_device_s *dev =
    container_of(fds, FAR struct rpmsgdev_device_s, fd);

  DEBUGASSERT(fds != NULL);

  work_queue_wq(g_rpmsgdev_wqueue, &dev->work, rpmsgdev_poll_worker,
                fds, 0);
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

      if (dev->cfd != 0)
        {
          msg->header.result = file_poll(&dev->file, &dev->fd, false);
          if (msg->header.result < 0)
            {
              return rpmsg_send(ept, msg, len);
            }
        }

      dev->cfd        = msg->fds;
      dev->fd.events  = msg->events;
      dev->fd.revents = 0;
      dev->fd.cb      = rpmsgdev_poll_cb;
      dev->fd.arg     = server;

      msg->header.result = file_poll(&dev->file, &dev->fd, true);
    }
  else
    {
      if (dev->cfd != 0)
        {
          msg->header.result = file_poll(&dev->file, &dev->fd, false);
          if (msg->header.result >= 0)
            {
              dev->cfd = 0;
            }
        }
    }

  return msg->header.cookie ? rpmsg_send(ept, msg, len) : 0;
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
 * Name: rpmsgdev_ept_release
 ****************************************************************************/

static void rpmsgdev_ept_release(FAR struct rpmsg_endpoint *ept)
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

      /* The slots live in server, freed below: nothing may still point
       * at them from the device poll list or the work queue.
       */

      rpmsgdev_read_cancel(dev);
      work_cancel_sync_wq(g_rpmsgdev_wqueue, &dev->work);

      if (dev->file.f_inode != NULL)
        {
          file_close(&dev->file);
        }

      list_delete(&dev->node);
      list_add_tail(&server->free, &dev->node);
    }

  nxmutex_unlock(&server->lock);
  nxmutex_destroy(&server->lock);
  kmm_free(server);
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
  int i;

  server = kmm_zalloc(sizeof(*server));
  if (server == NULL)
    {
      return;
    }

  list_initialize(&server->head);
  list_initialize(&server->free);
  for (i = 0; i < CONFIG_DEV_RPMSG_OPEN_MAX; i++)
    {
      list_add_tail(&server->free, &server->dev[i].node);
    }

  nxmutex_init(&server->lock);
  server->ept.priv = server;
  server->export = priv;
  server->ept.release_cb = rpmsgdev_ept_release;

  ret = rpmsg_create_ept(&server->ept, rdev, name,
                         RPMSG_ADDR_ANY, dest,
                         rpmsgdev_server_ept_cb, rpmsg_destroy_ept);
  if (ret < 0)
    {
      nxmutex_destroy(&server->lock);
      kmm_free(server);
    }
}

/****************************************************************************
 * Name: rpmsgdev_server_ept_cb
 ****************************************************************************/

static int rpmsgdev_server_ept_cb(FAR struct rpmsg_endpoint *ept,
                                  FAR void *data, size_t len, uint32_t src,
                                  FAR void *priv)
{
  FAR struct rpmsgdev_header_s *header = data;
  uint32_t command = header->command;

  if (command < nitems(g_rpmsgdev_handler))
    {
      return g_rpmsgdev_handler[command](ept, data, len, src, priv);
    }

  return -EINVAL;
}

/****************************************************************************
 * Name: rpmsgdev_server_created
 ****************************************************************************/

static void rpmsgdev_server_created(FAR struct rpmsg_device *rdev,
                                    FAR void *priv)
{
  struct rpmsgdev_export_s *export = priv;
  char buf[RPMSG_NAME_SIZE];

  if (strcmp(export->remotecpu, rpmsg_get_cpuname(rdev)) == 0)
    {
      snprintf(buf, sizeof(buf), "%s%s", export->prefix,
               strrchr(export->localpath, '/'));
      rpmsgdev_ns_bind(rdev, export, buf, RPMSG_ADDR_ANY);
    }
}

/****************************************************************************
 * Name: rpmsgdev_init_wqueue
 ****************************************************************************/

static int rpmsgdev_init_wqueue(void)
{
  /* Not thread safe: only called from board bringup. */

  if (g_rpmsgdev_wqueue == NULL)
    {
      g_rpmsgdev_wqueue = work_queue_create("rpmsgdev_server",
        CONFIG_DEV_RPMSG_SERVER_WORK_PRIORITY, NULL,
        CONFIG_DEV_RPMSG_SERVER_WORK_STACKSIZE, 1);
    }

  return g_rpmsgdev_wqueue ? 0 : -ENOMEM;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int rpmsgdev_export_with_prefix(FAR const char *remotecpu,
                                FAR const char *prefix,
                                FAR const char *localpath)
{
  FAR struct rpmsgdev_export_s *export;
  int ret;

  ret = rpmsgdev_init_wqueue();
  if (ret < 0)
    {
      return ret;
    }

  export = kmm_zalloc(sizeof(*export));
  if (export == NULL)
    {
      return -ENOMEM;
    }

  export->remotecpu = remotecpu;
  export->prefix    = prefix;
  export->localpath = localpath;

  return rpmsg_register_callback(export,
                                 rpmsgdev_server_created,
                                 NULL,
                                 NULL,
                                 NULL);
}

int rpmsgdev_export(FAR const char *remotecpu, FAR const char *localpath)
{
  return rpmsgdev_export_with_prefix(remotecpu,
                                     RPMSGDEV_NAME_PREFIX,
                                     localpath);
}

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
  int ret;

  ret = rpmsgdev_init_wqueue();
  if (ret < 0)
    {
      return ret;
    }

  return rpmsg_register_callback(NULL,
                                 NULL,
                                 NULL,
                                 rpmsgdev_ns_match,
                                 rpmsgdev_ns_bind);
}
