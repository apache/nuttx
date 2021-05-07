/****************************************************************************
 * arch/sim/src/sim/up_btuart.c
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

#include <nuttx/fs/fs.h>
#include <nuttx/kmalloc.h>
#include <nuttx/semaphore.h>
#include <nuttx/nuttx.h>

#include <string.h>
#include <poll.h>
#include <queue.h>

#include <nuttx/wireless/bluetooth/bt_uart.h>
#include <nuttx/wireless/bluetooth/bt_hci.h>

#include "up_internal.h"
#include "up_hcisocket_host.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CONFIG_HCI_RECVBUF_SIZE    1024
#define CONFIG_HCI_SENDBUF_SIZE    1024
#define CONFIG_HCI_NPOLLWAITERS    2

/****************************************************************************
 * Private Types
 ****************************************************************************/

union bt_hdr_u
{
  struct bt_hci_cmd_hdr_s cmd;
  struct bt_hci_acl_hdr_s acl;
  struct bt_hci_evt_hdr_s evt;
  struct bt_hci_iso_hdr_s iso;
};

struct sim_btuart_s
{
  sq_entry_t              link;
  uint8_t                 recvbuf[CONFIG_HCI_RECVBUF_SIZE];
  size_t                  recvpos;
  size_t                  recvlen;
  sem_t                   recvsem;
  sem_t                   recvlock;

  uint8_t                 sendbuf[CONFIG_HCI_SENDBUF_SIZE];
  size_t                  sendlen;
  sem_t                   sendlock;

  sem_t                   fdslock;
  FAR struct pollfd       *fds[CONFIG_HCI_NPOLLWAITERS];

  unsigned short          id;
  int                     fd;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int     sim_btuart_open (FAR struct file *filep);
static int     sim_btuart_close(FAR struct file *filep);
static ssize_t sim_btuart_read (FAR struct file *filep,
                                FAR char *buffer, size_t buflen);
static ssize_t sim_btuart_write(FAR struct file *filep,
                                FAR const char *buffer, size_t buflen);
static int     sim_btuart_ioctl(FAR struct file *filep,
                                int cmd, unsigned long arg);
static int     sim_btuart_poll (FAR struct file *filep,
                                FAR struct pollfd *fds, bool setup);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_sim_btuart_ops =
{
  .open  = sim_btuart_open,
  .close = sim_btuart_close,
  .read  = sim_btuart_read,
  .write = sim_btuart_write,
  .ioctl = sim_btuart_ioctl,
  .poll  = sim_btuart_poll
};

static sq_queue_t g_sim_btuart_list;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline void sim_btuart_post(FAR sem_t *sem)
{
  int semcount;

  nxsem_get_value(sem, &semcount);
  if (semcount < 1)
    {
      nxsem_post(sem);
    }
}

static void sim_btuart_pollnotify(FAR struct sim_btuart_s *dev,
                                  pollevent_t eventset)
{
  int i;

  for (i = 0; i < CONFIG_HCI_NPOLLWAITERS; i++)
    {
      FAR struct pollfd *fds = dev->fds[i];

      if (fds)
        {
          fds->revents |= (fds->events & eventset);

          if (fds->revents != 0)
            {
              sim_btuart_post(fds->sem);
            }
        }
    }

  sim_btuart_post(&dev->recvsem);
}

static int sim_btuart_open(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct sim_btuart_s *dev = inode->i_private;
  int fd;

  fd = bthcisock_host_open(dev->id);
  if (fd < 0)
    {
      return fd;
    }

  dev->sendlen = 0;
  dev->recvpos = 0;
  dev->recvlen = 0;
  dev->fd = fd;

  return OK;
}

static int sim_btuart_close(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct sim_btuart_s *dev = inode->i_private;

  bthcisock_host_close(dev->fd);

  dev->fd = -1;

  sim_btuart_pollnotify(dev, POLLIN | POLLOUT);
  return OK;
}

static ssize_t sim_btuart_read(FAR struct file *filep,
                               FAR char *buffer, size_t buflen)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct sim_btuart_s *dev = inode->i_private;
  size_t len = dev->recvlen;
  int ret;

  ret = nxsem_wait_uninterruptible(&dev->recvlock);
  if (ret < 0)
    {
      return ret;
    }

  if (dev->recvpos >= dev->recvlen)
    {
      while (!bthcisock_host_avail(dev->fd))
        {
          nxsem_wait_uninterruptible(&dev->recvsem);
        }

      len = bthcisock_host_read(dev->fd, dev->recvbuf,
                                CONFIG_HCI_RECVBUF_SIZE);
      if (len <= 0)
        {
          nxsem_post(&dev->recvlock);
          return len;
        }

      dev->recvpos = 0;
      dev->recvlen = len;
    }

  if (buflen > dev->recvlen - dev->recvpos)
    {
      buflen = dev->recvlen - dev->recvpos;
    }

  memcpy(buffer, dev->recvbuf + dev->recvpos, buflen);
  dev->recvpos += buflen;

  nxsem_post(&dev->recvlock);
  return buflen;
}

static ssize_t sim_btuart_write(FAR struct file *filep,
                                FAR const char *buffer, size_t buflen)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct sim_btuart_s *dev = inode->i_private;
  FAR union bt_hdr_u *hdr;
  size_t pktlen;
  size_t hdrlen;
  int ret;

  ret = nxsem_wait_uninterruptible(&dev->sendlock);
  if (ret < 0)
    {
      return ret;
    }

  if (dev->sendlen + buflen > CONFIG_HCI_SENDBUF_SIZE)
    {
      ret = -EINVAL;
      goto err;
    }

  memcpy(dev->sendbuf + dev->sendlen, buffer, buflen);
  dev->sendlen += buflen;

  hdr = (FAR union bt_hdr_u *)(dev->sendbuf + 1);

  while (1)
    {
      switch (dev->sendbuf[0])
        {
          case H4_CMD:
            hdrlen = sizeof(struct bt_hci_cmd_hdr_s);
            pktlen = hdr->cmd.param_len;
            break;
          case H4_ACL:
            hdrlen = sizeof(struct bt_hci_acl_hdr_s);
            pktlen = hdr->acl.len;
            break;
          case H4_ISO:
            hdrlen = sizeof(struct bt_hci_iso_hdr_s);
            pktlen = hdr->iso.len;
            break;
          default:
            ret = -EINVAL;
            goto err;
        }

      /* Reassembly is incomplete ? */

      hdrlen += H4_HEADER_SIZE;

      if (dev->sendlen < hdrlen)
        {
          goto out;
        }

      pktlen += hdrlen;
      if (dev->sendlen < pktlen)
        {
          goto out;
        }

      /* Got the full packet, send out */

      ret = bthcisock_host_send(dev->fd, dev->sendbuf, pktlen);
      if (ret < 0)
        {
          goto err;
        }

      dev->sendlen -= pktlen;
      if (dev->sendlen == 0)
        {
          goto out;
        }

      memmove(dev->sendbuf, dev->sendbuf + pktlen, dev->sendlen);
    }

err:
  dev->sendlen = 0;
out:
  nxsem_post(&dev->sendlock);
  return ret < 0 ? ret : buflen;
}

static int sim_btuart_ioctl(FAR struct file *filep,
                            int cmd, unsigned long arg)
{
  return OK;
}

static int sim_btuart_poll(FAR struct file *filep,
                           FAR struct pollfd *fds, bool setup)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct sim_btuart_s *dev = inode->i_private;
  pollevent_t eventset = 0;
  int ret;
  int i;

  ret = nxsem_wait_uninterruptible(&dev->fdslock);
  if (ret < 0)
    {
      return ret;
    }

  if (setup)
    {
      for (i = 0; i < CONFIG_HCI_NPOLLWAITERS; i++)
        {
          /* Find an available slot */

          if (!dev->fds[i])
            {
              /* Bind the poll structure and this slot */

              dev->fds[i]  = fds;
              fds->priv    = &dev->fds[i];
              break;
            }
        }

      if (i >= CONFIG_HCI_NPOLLWAITERS)
        {
          fds->priv = NULL;
          ret = -EBUSY;
        }

      if (bthcisock_host_avail(dev->fd))
        {
          eventset |= (fds->events & POLLIN);
        }

      eventset |= (fds->events & POLLOUT);

      if (eventset)
        {
          sim_btuart_pollnotify(dev, eventset);
        }
    }
  else if (fds->priv != NULL)
    {
      for (i = 0; i < CONFIG_HCI_NPOLLWAITERS; i++)
        {
          if (fds == dev->fds[i])
            {
              dev->fds[i] = NULL;
              fds->priv = NULL;
              break;
            }
        }
    }

  nxsem_post(&dev->fdslock);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void sim_btuart_loop(void)
{
  FAR struct sim_btuart_s *dev;
  FAR sq_entry_t *entry;

  for (entry = sq_peek(&g_sim_btuart_list); entry; entry = sq_next(entry))
    {
      dev = container_of(entry, struct sim_btuart_s, link);
      if (bthcisock_host_avail(dev->fd))
        {
          sim_btuart_pollnotify(dev, POLLIN);
        }
    }
}

int sim_btuart_register(const char *name, int id)
{
  FAR struct sim_btuart_s *dev;
  int ret;

  dev = (FAR struct sim_btuart_s *)kmm_zalloc(sizeof(struct sim_btuart_s));
  if (dev == NULL)
    {
      return -ENOMEM;
    }

  dev->fd = -1;
  dev->id = id;

  nxsem_init(&dev->recvlock, 0, 1);
  nxsem_init(&dev->sendlock, 0, 1);
  nxsem_init(&dev->recvsem, 0, 0);
  nxsem_init(&dev->fdslock, 0, 1);

  nxsem_set_protocol(&dev->recvsem, SEM_PRIO_NONE);

  ret = register_driver(name, &g_sim_btuart_ops, 0666, dev);
  if (ret < 0)
    {
      nxsem_destroy(&dev->recvlock);
      nxsem_destroy(&dev->sendlock);
      nxsem_destroy(&dev->recvsem);
      nxsem_destroy(&dev->fdslock);
      kmm_free(dev);
      return ret;
    }

  sq_addlast(&dev->link, &g_sim_btuart_list);
  return OK;
}
