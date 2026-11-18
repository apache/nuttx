/****************************************************************************
 * drivers/rpmsg/rpmsg_char.c
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

#include <assert.h>
#include <nuttx/debug.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/param.h>

#include <nuttx/compiler.h>
#include <nuttx/kmalloc.h>
#include <nuttx/list.h>
#include <nuttx/mutex.h>
#include <nuttx/nuttx.h>
#include <nuttx/rpmsg/rpmsg.h>
#include <nuttx/semaphore.h>

#include "rpmsg.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RPMSG_CHAR_NAME_PREFIX     "rpmsg-raw-"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct rpmsg_eptdev_s
{
  struct list_node           node;
  FAR struct rpmsg_char_s   *chr;
  struct rpmsg_endpoint      ept;
  struct rpmsg_endpoint_info info;
  struct list_node           rxbuf;
  spinlock_t                 lock;
  sem_t                      wait;
  sem_t                      rxsem;
  FAR struct pollfd_s       *fds[CONFIG_RPMSG_CHAR_NPOLLWAITERS];
  bool                       created;
  FAR char                  *rdbuf;
  size_t                     rdlen;
};

struct rpmsg_eptdev_buf_s
{
  struct list_node node;
  size_t           len;
  FAR uint8_t      data[1];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int rpmsg_eptdev_open(FAR struct file *filep);
static int rpmsg_eptdev_close(FAR struct file *filep);
static ssize_t rpmsg_eptdev_read(FAR struct file *filep, FAR char *buffer,
                                 size_t buflen);
static ssize_t rpmsg_eptdev_write(FAR struct file *filep,
                                  FAR const char *buffer, size_t buflen);
static int rpmsg_eptdev_ioctl(FAR struct file *filep, int cmd,
                              unsigned long arg);
static int rpmsg_eptdev_poll(FAR struct file *filep,
                             FAR struct pollfd_s *fds, bool setup);

static void rpmsg_eptdev_device_created(FAR struct rpmsg_device *rdev,
                                        FAR void *priv);
static void rpmsg_eptdev_device_destroy(FAR struct rpmsg_device *rdev,
                                        FAR void *priv);

/****************************************************************************
 * Private data
 ****************************************************************************/

static const struct file_operations g_rpmsg_eptdev_fops =
{
  rpmsg_eptdev_open,  /* open */
  rpmsg_eptdev_close, /* close */
  rpmsg_eptdev_read,  /* read */
  rpmsg_eptdev_write, /* write */
  NULL,               /* seek */
  rpmsg_eptdev_ioctl, /* ioctl */
  NULL,               /* mmap */
  NULL,               /* truncate */
  rpmsg_eptdev_poll,  /* poll */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rpmsg_char_get_rdev
 ****************************************************************************/

static inline_function FAR struct rpmsg_device *
rpmsg_char_get_rdev(FAR struct rpmsg_char_s *chr)
{
  return rpmsg_get_rdev_by_rpmsg(container_of(chr, struct rpmsg_s, chr));
}

/****************************************************************************
 * Name: rpmsg_char_add_eptdev
 ****************************************************************************/

static inline_function void
rpmsg_char_add_eptdev(FAR struct rpmsg_char_s *chr,
                      FAR struct rpmsg_eptdev_s *eptdev)
{
  nxrmutex_lock(&chr->lock);
  list_add_tail(&chr->eptdevs, &eptdev->node);
  nxrmutex_unlock(&chr->lock);
}

/****************************************************************************
 * Name: rpmsg_char_del_eptdev
 ****************************************************************************/

static inline_function void
rpmsg_char_del_eptdev(FAR struct rpmsg_char_s *chr,
                      FAR struct rpmsg_eptdev_s *eptdev)
{
  nxrmutex_lock(&chr->lock);
  if (list_in_list(&eptdev->node))
    {
      list_delete(&eptdev->node);
    }

  nxrmutex_unlock(&chr->lock);
}

/****************************************************************************
 * Name: rpmsg_char_in_eptdev_list
 ****************************************************************************/

static inline_function bool
rpmsg_char_in_eptdev_list(FAR struct rpmsg_char_s *chr,
                          FAR struct rpmsg_eptdev_s *eptdev)
{
  bool inlist;

  nxrmutex_lock(&chr->lock);
  inlist = list_in_list(&eptdev->node);
  nxrmutex_unlock(&chr->lock);

  return inlist;
}

/****************************************************************************
 * Name: rpmsg_char_find_eptdev
 ****************************************************************************/

static FAR struct rpmsg_eptdev_s *
rpmsg_char_find_eptdev(FAR struct rpmsg_char_s *chr,
                       FAR const struct rpmsg_endpoint_info *info)
{
  FAR struct rpmsg_eptdev_s *eptdev;
  bool found = false;

  nxrmutex_lock(&chr->lock);
  list_for_every_entry(&chr->eptdevs, eptdev, struct rpmsg_eptdev_s, node)
    {
      if (strcmp(eptdev->info.name, info->name) == 0 &&
          eptdev->info.src == info->src && eptdev->info.dst == info->dst)
        {
          found = true;
          break;
        }
    }

  nxrmutex_unlock(&chr->lock);
  return found ? eptdev : NULL;
}

/****************************************************************************
 * Name: rpmsg_eptdev_wakeup_rx
 ****************************************************************************/

static inline_function void
rpmsg_eptdev_wakeup_rx(FAR struct rpmsg_eptdev_s *eptdev)
{
  int val;

  nxsem_get_value(&eptdev->rxsem, &val);
  while (val++ < 1)
    {
      nxsem_post(&eptdev->rxsem);
    }
}

/****************************************************************************
 * Name: rpmsg_eptdev_destroy
 ****************************************************************************/

static inline_function void
rpmsg_eptdev_destroy(FAR struct rpmsg_eptdev_s *eptdev)
{
  if (rpmsg_char_in_eptdev_list(eptdev->chr, eptdev))
    {
      rpmsg_unregister_callback(eptdev, rpmsg_eptdev_device_created,
                                rpmsg_eptdev_device_destroy,
                                NULL, NULL);
    }
}

/****************************************************************************
 * Name: rpmsg_eptdev_open
 ****************************************************************************/

static int rpmsg_eptdev_open(FAR struct file *filep)
{
  FAR struct rpmsg_eptdev_s *eptdev;

  DEBUGASSERT(filep != NULL && filep->f_inode != NULL);
  eptdev = filep->f_inode->i_private;

  filep->f_priv = eptdev;
  rpmsg_ept_incref(&eptdev->ept);

  return OK;
}

/****************************************************************************
 * Name: rpmsg_eptdev_close
 ****************************************************************************/

static int rpmsg_eptdev_close(FAR struct file *filep)
{
  FAR struct rpmsg_eptdev_s *eptdev = filep->f_priv;

  rpmsg_ept_decref(&eptdev->ept);
  return OK;
}

/****************************************************************************
 * Name: rpmsg_eptdev_read
 ****************************************************************************/

static ssize_t rpmsg_eptdev_read(FAR struct file *filep, FAR char *buffer,
                                 size_t buflen)
{
  FAR struct rpmsg_eptdev_buf_s *buf;
  FAR struct rpmsg_eptdev_s *eptdev = filep->f_priv;
  irqstate_t flags;
  ssize_t ret = 0;
  int val;

  flags = spin_lock_irqsave(&eptdev->lock);
  while (list_is_empty(&eptdev->rxbuf))
    {
      if (filep->f_oflags & O_NONBLOCK)
        {
          ret = -EAGAIN;
          break;
        }

      eptdev->rdbuf = buffer;
      eptdev->rdlen = buflen;
      spin_unlock_irqrestore(&eptdev->lock, flags);
      ret = nxsem_wait(&eptdev->rxsem);
      flags = spin_lock_irqsave(&eptdev->lock);

      nxsem_get_value(&eptdev->wait, &val);
      if (val <= 0)
        {
          ret = -ECONNRESET;
        }

      if (ret < 0)
        {
          break;
        }

      if (eptdev->rdbuf == NULL)
        {
          ret = eptdev->rdlen;
          break;
        }
    }

  eptdev->rdbuf = NULL;
  eptdev->rdlen = 0;

  if (ret == 0)
    {
      buf = list_remove_head_type(&eptdev->rxbuf, struct rpmsg_eptdev_buf_s,
                                  node);
      spin_unlock_irqrestore(&eptdev->lock, flags);
      if (buf != NULL)
        {
          ret = MIN(buflen, buf->len);
          memcpy(buffer, buf->data, ret);
          kmm_free(buf);
        }
    }
  else
    {
      spin_unlock_irqrestore(&eptdev->lock, flags);
    }

  return ret;
}

/****************************************************************************
 * Name: rpmsg_eptdev_write
 ****************************************************************************/

static ssize_t rpmsg_eptdev_write(FAR struct file *filep,
                                  FAR const char *buffer, size_t buflen)
{
  FAR struct rpmsg_eptdev_s *eptdev = filep->f_priv;
  int ret = OK;

  if (filep->f_oflags & O_NONBLOCK)
    {
      ret = rpmsg_trysend(&eptdev->ept, buffer, buflen);
      if (ret < 0)
        {
          ret = -EAGAIN;
        }
    }
  else
    {
      int semcount;

      /* Wait the service bound */

      nxsem_get_value(&eptdev->wait, &semcount);
      if (semcount <= 0)
        {
          rpmsg_wait(&eptdev->ept, &eptdev->wait);
          rpmsg_post(&eptdev->ept, &eptdev->wait);
        }

      ret = rpmsg_send(&eptdev->ept, buffer, buflen);
    }

  return ret;
}

/****************************************************************************
 * Name: rpmsg_eptdev_ioctl
 ****************************************************************************/

static int rpmsg_eptdev_ioctl(FAR struct file *filep, int cmd,
                              unsigned long arg)
{
  FAR struct rpmsg_eptdev_s *eptdev = filep->f_priv;
  int ret = OK;

  switch (cmd)
    {
      case RPMSG_DESTROY_EPT_IOCTL:
        rpmsg_eptdev_destroy(eptdev);
        break;
      default:
        ret = -ENOTTY;
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: rpmsg_eptdev_poll
 ****************************************************************************/

static int rpmsg_eptdev_poll(FAR struct file *filep,
                             FAR struct pollfd_s *fds, bool setup)
{
  FAR struct rpmsg_eptdev_s *eptdev = filep->f_priv;
  pollevent_t eventset = 0;
  irqstate_t flags;
  int ret = OK;
  int i;

  flags = spin_lock_irqsave_nopreempt(&eptdev->lock);
  if (setup)
    {
      for (i = 0; i < CONFIG_RPMSG_CHAR_NPOLLWAITERS; i++)
        {
          if (eptdev->fds[i] == NULL)
            {
              /* Bind the poll structure and this slot */

              eptdev->fds[i] = fds;
              fds->priv      = &eptdev->fds[i];
              break;
            }
        }

      if (i < CONFIG_RPMSG_CHAR_NPOLLWAITERS)
        {
          if (!list_is_empty(&eptdev->rxbuf))
            {
              eventset |= POLLIN;
            }

          /* Need add a API to check tx available */

          eventset |= POLLOUT;
          poll_notify(&fds, 1, eventset);
        }
      else
        {
          ret = -EBUSY;
        }
    }
  else if (fds->priv != NULL)
    {
      /* This is a request to tear down the poll. */

      FAR struct pollfd_s **slot = (FAR struct pollfd_s **)fds->priv;

      if (slot != NULL)
        {
          /* Remove all memory of the poll setup */

          *slot     = NULL;
          fds->priv = NULL;
        }
      else
        {
          ret = -EIO;
        }
    }

  spin_unlock_irqrestore_nopreempt(&eptdev->lock, flags);
  return ret;
}

/****************************************************************************
 * Name: rpmsg_eptdev_release_cb
 ****************************************************************************/

static void rpmsg_eptdev_release_cb(FAR struct rpmsg_endpoint *ept)
{
  FAR struct rpmsg_eptdev_s *eptdev =
    container_of(ept, struct rpmsg_eptdev_s, ept);

  rpmsg_char_del_eptdev(eptdev->chr, eptdev);
  kmm_free(eptdev);
}

/****************************************************************************
 * Name: rpmsg_eptdev_ns_bound_cb
 ****************************************************************************/

static void rpmsg_eptdev_ns_bound_cb(FAR struct rpmsg_endpoint *ept)
{
  FAR struct rpmsg_eptdev_s *eptdev =
    container_of(ept, struct rpmsg_eptdev_s, ept);

  nxsem_post(&eptdev->wait);
}

/****************************************************************************
 * Name: rpmsg_eptdev_ns_unbind_cb
 ****************************************************************************/

static void rpmsg_eptdev_ns_unbind_cb(FAR struct rpmsg_endpoint *ept)
{
  FAR struct rpmsg_eptdev_s *eptdev =
    container_of(ept, struct rpmsg_eptdev_s, ept);
  irqstate_t flags;

  nxsem_wait(&eptdev->wait);
  flags = spin_lock_irqsave_nopreempt(&eptdev->lock);
  rpmsg_eptdev_wakeup_rx(eptdev);
  poll_notify(eptdev->fds, CONFIG_RPMSG_CHAR_NPOLLWAITERS, POLLIN | POLLHUP);
  spin_unlock_irqrestore_nopreempt(&eptdev->lock, flags);
}

/****************************************************************************
 * Name: rpmsg_eptdev_ept_cb
 ****************************************************************************/

static int rpmsg_eptdev_ept_cb(FAR struct rpmsg_endpoint *ept,
                               FAR void *data, size_t len, uint32_t src,
                               FAR void *priv)
{
  FAR struct rpmsg_eptdev_s *eptdev = (FAR struct rpmsg_eptdev_s *)priv;
  FAR struct rpmsg_eptdev_buf_s *buf;
  irqstate_t flags;
  int ret = OK;

  flags = spin_lock_irqsave_nopreempt(&eptdev->lock);
  if (eptdev->rdbuf != NULL)
    {
      eptdev->rdlen = MIN(eptdev->rdlen, len);
      memcpy(eptdev->rdbuf, data, eptdev->rdlen);
      eptdev->rdbuf = NULL;
    }
  else
    {
      spin_unlock_irqrestore_nopreempt(&eptdev->lock, flags);
      buf = kmm_malloc(sizeof(*buf) + len - 1);
      DEBUGASSERT(buf);

      buf->len = len;
      memcpy(buf->data, data, len);
      flags = spin_lock_irqsave_nopreempt(&eptdev->lock);
      list_add_tail(&eptdev->rxbuf, &buf->node);
    }

  rpmsg_eptdev_wakeup_rx(eptdev);
  poll_notify(eptdev->fds, CONFIG_RPMSG_CHAR_NPOLLWAITERS, POLLIN);
  spin_unlock_irqrestore_nopreempt(&eptdev->lock, flags);

  return ret;
}

/****************************************************************************
 * Name: rpmsg_eptdev_device_created
 ****************************************************************************/

static void rpmsg_eptdev_device_created(FAR struct rpmsg_device *rdev,
                                        FAR void *priv)
{
  FAR struct rpmsg_eptdev_s *eptdev = (FAR struct rpmsg_eptdev_s *)priv;
  FAR struct rpmsg_endpoint_info *info = &eptdev->info;
  FAR struct rpmsg_char_s *chr = eptdev->chr;
  char name[RPMSG_NAME_SIZE];
  char path[PATH_MAX];
  int ret;

  nxrmutex_lock(&chr->lock);
  if (rpmsg_char_get_rdev(chr) == rdev)
    {
      eptdev->ept.priv = eptdev;
      eptdev->ept.release_cb = rpmsg_eptdev_release_cb;
      eptdev->ept.ns_bound_cb = rpmsg_eptdev_ns_bound_cb;
      strlcpy(name, RPMSG_CHAR_NAME_PREFIX, RPMSG_NAME_SIZE);
      strlcat(name, info->name, RPMSG_NAME_SIZE);

      ret = rpmsg_create_ept(&eptdev->ept, rdev, name, info->src,
                             info->dst, rpmsg_eptdev_ept_cb,
                             rpmsg_eptdev_ns_unbind_cb);
      if (ret >= 0)
        {
          snprintf(path, PATH_MAX, "/dev/rpmsg-%s", info->name);
          ret = register_driver(path, &g_rpmsg_eptdev_fops, 0666, eptdev);
          if (ret >= 0)
            {
              eptdev->created = true;
            }
          else
            {
              rpmsgerr("Failed to register driver %s %d\n", path, ret);
              rpmsg_destroy_ept(&eptdev->ept);
            }
        }
      else
        {
          rpmsgerr("Failed to create endpoint %s %d\n", info->name, ret);
        }
    }

  nxrmutex_unlock(&chr->lock);
}

/****************************************************************************
 * Name: rpmsg_eptdev_device_destroy
 ****************************************************************************/

static void rpmsg_eptdev_device_destroy(FAR struct rpmsg_device *rdev,
                                        FAR void *priv)
{
  FAR struct rpmsg_eptdev_s *eptdev = (FAR struct rpmsg_eptdev_s *)priv;
  FAR struct rpmsg_char_s *chr = eptdev->chr;
  char path[PATH_MAX];

  nxrmutex_lock(&chr->lock);
  if (rpmsg_char_get_rdev(chr) == rdev && eptdev->created)
    {
      eptdev->created = false;
      snprintf(path, PATH_MAX, "/dev/rpmsg-%s", eptdev->info.name);
      unregister_driver(path);
      rpmsg_destroy_ept(&eptdev->ept);
    }

  nxrmutex_unlock(&chr->lock);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rpmsg_char_create_eptdev
 ****************************************************************************/

int rpmsg_char_create_eptdev(FAR struct rpmsg_char_s *chr,
                             FAR const struct rpmsg_endpoint_info *info)
{
  FAR struct rpmsg_eptdev_s *eptdev;
  int ret = -EEXIST;

  if (rpmsg_char_find_eptdev(chr, info) == NULL)
    {
      eptdev = kmm_zalloc(sizeof(*eptdev));
      if (eptdev != NULL)
        {
          eptdev->chr = chr;
          if (info->src == RPMSG_ADDR_ANY && info->dst == RPMSG_ADDR_ANY)
            {
              nxsem_init(&eptdev->wait, 0, 0);
            }
          else
            {
              nxsem_init(&eptdev->wait, 0, 1);
            }

          nxsem_init(&eptdev->rxsem, 0, 0);
          spin_lock_init(&eptdev->lock);
          list_initialize(&eptdev->rxbuf);
          memcpy(&eptdev->info, info, sizeof(*info));
          rpmsg_char_add_eptdev(chr, eptdev);
          ret = rpmsg_register_callback(eptdev, rpmsg_eptdev_device_created,
                                        rpmsg_eptdev_device_destroy,
                                        NULL, NULL);
          if (ret < 0)
            {
              rpmsg_char_del_eptdev(chr, eptdev);
              nxsem_destroy(&eptdev->rxsem);
              nxsem_destroy(&eptdev->wait);
              kmm_free(eptdev);
            }
        }
      else
        {
          ret = -ENOMEM;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: rpmsg_char_destroy_eptdev
 ****************************************************************************/

void rpmsg_char_destroy_eptdev(FAR struct rpmsg_char_s *chr,
                               FAR const struct rpmsg_endpoint_info *info)
{
  FAR struct rpmsg_eptdev_s *eptdev;

  eptdev = rpmsg_char_find_eptdev(chr, info);
  if (eptdev != NULL)
    {
      rpmsg_eptdev_destroy(eptdev);
    }
  else
    {
      rpmsgwarn("No such endpoint %s\n", info->name);
    }
}

/****************************************************************************
 * Name: rpmsg_char_init
 ****************************************************************************/

void rpmsg_char_init(FAR struct rpmsg_char_s *chr)
{
  nxrmutex_init(&chr->lock);
  list_initialize(&chr->eptdevs);
}

/****************************************************************************
 * Name: rpmsg_char_deinit
 ****************************************************************************/

void rpmsg_char_deinit(FAR struct rpmsg_char_s *chr)
{
  FAR struct rpmsg_eptdev_s *eptdev;
  FAR struct rpmsg_eptdev_s *tmp;

  nxrmutex_lock(&chr->lock);
  list_for_every_entry_safe(&chr->eptdevs, eptdev, tmp,
                            struct rpmsg_eptdev_s, node)
    {
      rpmsg_eptdev_destroy(eptdev);
    }

  nxrmutex_unlock(&chr->lock);
  nxrmutex_destroy(&chr->lock);
}
