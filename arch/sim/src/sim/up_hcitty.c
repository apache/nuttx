/****************************************************************************
 * arch/sim/src/sim/up_hcitty.c
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
#include <nuttx/semaphore.h>

#include <string.h>
#include <stdio.h>
#include <poll.h>

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

struct bthcitty_s
{
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

static int      bthcitty_open  (FAR struct file *filep);
static int      bthcitty_close (FAR struct file *filep);
static ssize_t  bthcitty_read  (FAR struct file *filep,
                               FAR char *buffer, size_t buflen);
static ssize_t  bthcitty_write (FAR struct file *filep,
                               FAR const char *buffer, size_t buflen);
static int      bthcitty_ioctl (FAR struct file *filep,
                               int cmd, unsigned long arg);
static int      bthcitty_poll  (FAR struct file *filep,
                               FAR struct pollfd *fds, bool setup);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_hcitty_ops =
{
  .open   = bthcitty_open,
  .close  = bthcitty_close,
  .read   = bthcitty_read,
  .write  = bthcitty_write,
  .ioctl  = bthcitty_ioctl,
  .poll   = bthcitty_poll
};

static struct bthcitty_s g_hcitty =
{
  .fd = -1,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline void bthcitty_post(FAR sem_t *sem)
{
  int semcount;

  nxsem_get_value(sem, &semcount);
  if (semcount < 1)
    {
      nxsem_post(sem);
    }
}

static void bthcitty_pollnotify(FAR struct bthcitty_s *dev,
                                pollevent_t eventset)
{
  int ret;
  int i;

  ret = nxsem_wait_uninterruptible(&dev->fdslock);
  if (ret < 0)
    {
      return;
    }

  for (i = 0; i < CONFIG_HCI_NPOLLWAITERS; i++)
    {
      FAR struct pollfd *fds = dev->fds[i];

      if (fds)
        {
          fds->revents |= (fds->events & eventset);

          if (fds->revents != 0)
            {
              bthcitty_post(fds->sem);
            }
        }
    }

  bthcitty_post(&dev->recvsem);

  nxsem_post(&dev->fdslock);
}

static int bthcitty_open(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct bthcitty_s *dev = inode->i_private;
  int ret;
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

static int bthcitty_close(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct bthcitty_s *dev = inode->i_private;

  bthcisock_host_close(dev->fd);

  dev->fd = -1;

  bthcitty_pollnotify(dev, POLLIN | POLLOUT);

  return 0;
}

static ssize_t bthcitty_read(FAR struct file *filep,
                             FAR char *buffer, size_t buflen)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct bthcitty_s *dev = inode->i_private;
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

static ssize_t bthcitty_write(FAR struct file *filep,
                              FAR const char *buffer, size_t buflen)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct bthcitty_s *dev = inode->i_private;
  size_t minlen = 1 + 2 + 1; /* header + opcode + len */
  size_t len = buflen;
  int ret;

  ret = nxsem_wait_uninterruptible(&dev->sendlock);
  if (ret < 0)
    {
      return ret;
    }

  if (dev->sendlen > 0 || buflen < minlen)
    {
      memcpy(dev->sendbuf + dev->sendlen, buffer, buflen);
      dev->sendlen += buflen;

      buffer = dev->sendbuf;
      len = dev->sendlen;
    }

  if (len >= minlen)
    {
      ret = bthcisock_host_send(dev->fd, buffer, len);
      if (ret >= 0)
        {
          dev->sendlen = 0;
        }
    }

  nxsem_post(&dev->sendlock);

  return ret < 0 ? ret : buflen;
}

static int bthcitty_ioctl(FAR struct file *filep,
                          int cmd, unsigned long arg)
{
  return OK;
}

static int bthcitty_poll(FAR struct file *filep,
                         FAR struct pollfd *fds, bool setup)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct bthcitty_s *dev = inode->i_private;
  pollevent_t eventset;
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
          bthcitty_pollnotify(dev, eventset);
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

void bthcitty_loop(void)
{
  if (bthcisock_host_avail(g_hcitty.fd))
    {
      bthcitty_pollnotify(&g_hcitty, POLLIN);
    }
}

int bthcitty_register(int dev_id)
{
  unsigned char name[16];

  snprintf(name, sizeof(name), "/dev/ttyBT%d", dev_id);

  g_hcitty.id = dev_id;

  nxsem_init(&g_hcitty.recvlock, 0, 1);
  nxsem_init(&g_hcitty.sendlock, 0, 1);
  nxsem_init(&g_hcitty.recvsem, 0, 0);
  nxsem_set_protocol(&g_hcitty.recvsem, SEM_PRIO_NONE);
  nxsem_init(&g_hcitty.fdslock, 0, 1);

  return register_driver(name, &g_hcitty_ops, 0666, &g_hcitty);
}
