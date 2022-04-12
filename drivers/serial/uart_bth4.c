/****************************************************************************
 * drivers/serial/uart_bth4.c
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
#include <nuttx/mm/circbuf.h>

#include <fcntl.h>
#include <string.h>
#include <poll.h>

#include <nuttx/wireless/bluetooth/bt_hci.h>
#include <nuttx/wireless/bluetooth/bt_uart.h>
#include <nuttx/wireless/bluetooth/bt_driver.h>

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

struct uart_bth4_s
{
  FAR struct bt_driver_s  *drv;

  FAR struct circbuf_s    circbuf;

  sem_t                   recvsem;

  uint8_t                 sendbuf[CONFIG_UART_BTH4_TXBUFSIZE];
  size_t                  sendlen;
  sem_t                   sendlock;

  FAR struct pollfd       *fds[CONFIG_UART_BTH4_NPOLLWAITERS];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int     uart_bth4_open(FAR struct file *filep);
static int     uart_bth4_close(FAR struct file *filep);
static ssize_t uart_bth4_read(FAR struct file *filep,
                              FAR char *buffer, size_t buflen);
static ssize_t uart_bth4_write(FAR struct file *filep,
                               FAR const char *buffer, size_t buflen);
static int     uart_bth4_ioctl(FAR struct file *filep,
                               int cmd, unsigned long arg);
static int     uart_bth4_poll(FAR struct file *filep,
                              FAR struct pollfd *fds, bool setup);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_uart_bth4_ops =
{
  uart_bth4_open,   /* open */
  uart_bth4_close,  /* close */
  uart_bth4_read,   /* read */
  uart_bth4_write,  /* write */
  NULL,             /* seek */
  uart_bth4_ioctl,  /* ioctl */
  uart_bth4_poll    /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL            /* unlink */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline void uart_bth4_post(FAR sem_t *sem)
{
  int semcount;

  nxsem_get_value(sem, &semcount);
  if (semcount < 1)
    {
      nxsem_post(sem);
    }
}

static void uart_bth4_pollnotify(FAR struct uart_bth4_s *dev,
                                 pollevent_t eventset)
{
  int i;

  for (i = 0; i < CONFIG_UART_BTH4_NPOLLWAITERS; i++)
    {
      FAR struct pollfd *fds = dev->fds[i];

      if (fds)
        {
          fds->revents |= (fds->events & eventset);

          if (fds->revents != 0)
            {
              uart_bth4_post(fds->sem);
            }
        }
    }

  if ((eventset & POLLIN) != 0)
    {
      uart_bth4_post(&dev->recvsem);
    }
}

static int uart_bth4_receive(FAR struct bt_driver_s *drv,
                             enum bt_buf_type_e type,
                             FAR void *buffer, size_t buflen)
{
  FAR struct uart_bth4_s *dev = drv->priv;
  int ret = buflen;
  irqstate_t flags;
  uint8_t htype;

  flags = enter_critical_section();

  if (circbuf_space(&dev->circbuf) >=
      buflen + H4_HEADER_SIZE)
    {
      if (type == BT_EVT)
        {
          htype = H4_EVT;
        }
      else if (type == BT_ACL_IN)
        {
          htype = H4_ACL;
        }
      else if (type == BT_ISO_IN)
        {
          htype = H4_ISO;
        }
      else
        {
          ret = -EINVAL;
        }

      if (ret >= 0)
        {
          circbuf_write(&dev->circbuf, &htype, H4_HEADER_SIZE);
          circbuf_write(&dev->circbuf, buffer, buflen);
          uart_bth4_pollnotify(dev, POLLIN);
        }
    }

  leave_critical_section(flags);
  return ret;
}

static int uart_bth4_open(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct uart_bth4_s *dev = inode->i_private;
  int ret;

  ret = dev->drv->open(dev->drv);
  if (ret < 0)
    {
      return ret;
    }

  dev->sendlen = 0;

  return OK;
}

static int uart_bth4_close(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct uart_bth4_s *dev = inode->i_private;

  dev->drv->close(dev->drv);

  uart_bth4_pollnotify(dev, POLLIN | POLLOUT);
  return OK;
}

static ssize_t uart_bth4_read(FAR struct file *filep,
                              FAR char *buffer,
                              size_t buflen)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct uart_bth4_s *dev = inode->i_private;
  irqstate_t flags;
  ssize_t nread;

  flags = enter_critical_section();

  for (; ; )
    {
      nread = circbuf_read(&dev->circbuf, buffer, buflen);
      if (nread != 0 || (filep->f_oflags & O_NONBLOCK))
        {
          break;
        }

      while (circbuf_is_empty(&dev->circbuf))
        {
          nxsem_wait_uninterruptible(&dev->recvsem);
        }
    }

  leave_critical_section(flags);
  return nread;
}

static ssize_t uart_bth4_write(FAR struct file *filep,
                               FAR const char *buffer,
                               size_t buflen)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct uart_bth4_s *dev = inode->i_private;
  FAR union bt_hdr_u *hdr;
  enum bt_buf_type_e type;
  size_t reserved;
  uint8_t *data;
  size_t pktlen;
  size_t hdrlen;
  int ret;

  ret = nxsem_wait_uninterruptible(&dev->sendlock);
  if (ret < 0)
    {
      return ret;
    }

  if (dev->drv->head_reserve < H4_HEADER_SIZE)
    {
      reserved = H4_HEADER_SIZE;
    }
  else
    {
      reserved = dev->drv->head_reserve;
    }

  data = dev->sendbuf + reserved;

  if (dev->sendlen + buflen > CONFIG_UART_BTH4_TXBUFSIZE - reserved)
    {
      ret = -E2BIG;
      goto err;
    }

  memcpy(data - H4_HEADER_SIZE + dev->sendlen,
         buffer, buflen);
  dev->sendlen += buflen;

  hdr = (FAR union bt_hdr_u *)data;

  for (; ; )
    {
      switch (*(data - H4_HEADER_SIZE))
        {
          case H4_CMD:
            hdrlen = sizeof(struct bt_hci_cmd_hdr_s);
            pktlen = hdr->cmd.param_len;
            type = BT_CMD;
            break;
          case H4_ACL:
            hdrlen = sizeof(struct bt_hci_acl_hdr_s);
            pktlen = hdr->acl.len;
            type = BT_ACL_OUT;
            break;
          case H4_ISO:
            hdrlen = sizeof(struct bt_hci_iso_hdr_s);
            pktlen = hdr->iso.len;
            type = BT_ISO_OUT;
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

      ret = dev->drv->send(dev->drv, type,
                           data, pktlen - H4_HEADER_SIZE);
      if (ret < 0)
        {
          goto err;
        }

      dev->sendlen -= pktlen;
      if (dev->sendlen == 0)
        {
          goto out;
        }

      memmove(data - H4_HEADER_SIZE,
              dev->sendbuf + pktlen, dev->sendlen);
    }

err:
  dev->sendlen = 0;
out:
  nxsem_post(&dev->sendlock);
  return ret < 0 ? ret : buflen;
}

static int uart_bth4_ioctl(FAR struct file *filep, int cmd,
                           unsigned long arg)
{
  return OK;
}

static int uart_bth4_poll(FAR struct file *filep, FAR struct pollfd *fds,
                          bool setup)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct uart_bth4_s *dev = inode->i_private;
  pollevent_t eventset = 0;
  irqstate_t flags;
  int ret = 0;
  int i;

  flags = enter_critical_section();

  if (setup)
    {
      for (i = 0; i < CONFIG_UART_BTH4_NPOLLWAITERS; i++)
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

      if (i >= CONFIG_UART_BTH4_NPOLLWAITERS)
        {
          fds->priv = NULL;
          ret = -EBUSY;
        }

      if (!circbuf_is_empty(&dev->circbuf))
        {
          eventset |= (fds->events & POLLIN);
        }

      eventset |= (fds->events & POLLOUT);

      if (eventset)
        {
          uart_bth4_pollnotify(dev, eventset);
        }
    }
  else if (fds->priv != NULL)
    {
      for (i = 0; i < CONFIG_UART_BTH4_NPOLLWAITERS; i++)
        {
          if (fds == dev->fds[i])
            {
              dev->fds[i] = NULL;
              fds->priv = NULL;
              break;
            }
        }
    }

  leave_critical_section(flags);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int uart_bth4_register(FAR const char *path, FAR struct bt_driver_s *drv)
{
  FAR struct uart_bth4_s *dev;
  int ret;

  dev = (FAR struct uart_bth4_s *)
        kmm_zalloc(sizeof(struct uart_bth4_s));
  if (dev == NULL)
    {
      return -ENOMEM;
    }

  ret = circbuf_init(&dev->circbuf, NULL,
                     CONFIG_UART_BTH4_RXBUFSIZE);
  if (ret < 0)
    {
      kmm_free(dev);
      return -ENOMEM;
    }

  dev->drv     = drv;
  drv->receive = uart_bth4_receive;
  drv->priv    = dev;

  nxsem_init(&dev->sendlock, 0, 1);
  nxsem_init(&dev->recvsem,  0, 0);

  nxsem_set_protocol(&dev->recvsem, SEM_PRIO_NONE);

  ret = register_driver(path, &g_uart_bth4_ops, 0666, dev);
  if (ret < 0)
    {
      nxsem_destroy(&dev->sendlock);
      nxsem_destroy(&dev->recvsem);
      circbuf_uninit(&dev->circbuf);
      kmm_free(dev);
    }

  return ret;
}
