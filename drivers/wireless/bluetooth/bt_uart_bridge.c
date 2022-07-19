/****************************************************************************
 * drivers/wireless/bluetooth/bt_uart_bridge.c
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

#include <fcntl.h>
#include <poll.h>
#include <stdio.h>
#include <sys/time.h>

#include <nuttx/fs/fs.h>
#include <nuttx/kmalloc.h>
#include <nuttx/semaphore.h>
#include <nuttx/mm/circbuf.h>
#include <nuttx/net/snoop.h>

#include <nuttx/wireless/bluetooth/bt_hci.h>
#include <nuttx/wireless/bluetooth/bt_uart.h>

#include "bt_uart_filter.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define HCI_RECVBUF_SIZE     8192
#define HCI_SENDBUF_SIZE     1024
#define HCI_NPOLLWAITERS     2

/****************************************************************************
 * Private Types
 ****************************************************************************/

union bt_hci_hdr_u
{
  struct bt_hci_cmd_hdr_s        cmd;
  struct bt_hci_acl_hdr_s        acl;
  struct bt_hci_iso_hdr_s        iso;
  struct bt_hci_evt_hdr_s        evt;
};

struct bt_uart_bridge_device_s
{
  FAR struct bt_uart_bridge_s   *bridge;

  struct bt_uart_filter_s        filter;

  struct circbuf_s               recvbuf;
  sem_t                          recvlock;
  sem_t                          recvsig;
  char                           sendbuf[HCI_SENDBUF_SIZE];
  size_t                         sendlen;
};

struct bt_uart_bridge_s
{
  struct bt_uart_bridge_device_s device[BT_UART_FILTER_TYPE_COUNT];

  sem_t                          recvlock;
  sem_t                          sendlock;

  struct file                    filep;
#ifdef CONFIG_BLUETOOTH_UART_BRIDGE_BTSNOOP
  struct snoop_s                 snoop;
#endif /* CONFIG_BLUETOOTH_UART_BRIDGE_BTSNOOP */

  char                           tmpbuf[1024];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int      bt_uart_bridge_open  (FAR struct file *filep);
static int      bt_uart_bridge_close (FAR struct file *filep);
static ssize_t  bt_uart_bridge_read  (FAR struct file *filep,
                                      FAR char *buffer, size_t buflen);
static ssize_t  bt_uart_bridge_write (FAR struct file *filep,
                                      FAR const char *buffer, size_t buflen);
static int      bt_uart_bridge_ioctl (FAR struct file *filep,
                                      int cmd, unsigned long arg);
static int      bt_uart_bridge_poll  (FAR struct file *filep,
                                      FAR struct pollfd *fds, bool setup);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_hcibridge_ops =
{
  .open   = bt_uart_bridge_open,
  .close  = bt_uart_bridge_close,
  .read   = bt_uart_bridge_read,
  .write  = bt_uart_bridge_write,
  .ioctl  = bt_uart_bridge_ioctl,
  .poll   = bt_uart_bridge_poll
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static ssize_t
bt_uart_circbuf_read(FAR struct bt_uart_bridge_device_s *device,
                     FAR char *buffer, size_t buflen)
{
  int ret;

  ret = nxsem_wait_uninterruptible(&device->recvlock);
  if (ret >= 0)
    {
      ret = circbuf_read(&device->recvbuf, buffer, buflen);
    }

  nxsem_post(&device->recvlock);

  return ret;
}

static ssize_t
bt_uart_circbuf_write(FAR struct bt_uart_bridge_device_s *device,
                      FAR char *buffer, size_t buflen)
{
  int ret;

  ret = nxsem_wait_uninterruptible(&device->recvlock);
  if (ret >= 0)
    {
      if (circbuf_space(&device->recvbuf) >= buflen)
        {
          ret = circbuf_write(&device->recvbuf, buffer, buflen);
        }
    }

  nxsem_post(&device->recvlock);

  return ret;
}

static ssize_t bt_uart_file_read(FAR struct file *filep,
                                 FAR char *buffer, size_t buflen)
{
  size_t nread = 0;
  ssize_t ret;

  while (buflen != nread)
    {
      ret = file_read(filep, buffer + nread, buflen - nread);
      if (ret < 0)
        {
          return ret;
        }

      nread += ret;
    }

  return nread;
}

static int bt_uart_bridge_open(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct bt_uart_bridge_device_s *device = inode->i_private;

  if (inode->i_crefs == 1)
    {
      device->sendlen = 0;
      circbuf_reset(&device->recvbuf);
    }

  return OK;
}

static int bt_uart_bridge_close(FAR struct file *filep)
{
  return OK;
}

static ssize_t bt_uart_bridge_read(FAR struct file *filep,
                                   FAR char *buffer, size_t buflen)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct bt_uart_bridge_device_s *device = inode->i_private;
  FAR struct bt_uart_bridge_s *bridge = device->bridge;
  FAR struct bt_uart_bridge_device_s *iterator;
  FAR union bt_hci_hdr_u *hdr;
  size_t pktlen;
  size_t hdrlen;
  int ret;
  int i;

  while (1)
    {
      if (!circbuf_is_empty(&device->recvbuf))
        {
          break;
        }

      ret = nxsem_trywait(&bridge->recvlock);
      if (ret < 0)
        {
          if (ret == -EAGAIN)
            {
              break;
            }

          return ret;
        }

      ret = bt_uart_file_read(&bridge->filep,
                              bridge->tmpbuf, H4_HEADER_SIZE);
      if (ret < 0)
        {
          goto err;
        }

      switch (bridge->tmpbuf[0])
        {
          case H4_EVT:
            hdrlen = sizeof(struct bt_hci_evt_hdr_s);
            break;
          case H4_ACL:
            hdrlen = sizeof(struct bt_hci_acl_hdr_s);
            break;
          case H4_ISO:
            hdrlen = sizeof(struct bt_hci_iso_hdr_s);
            break;
          default:
            ret = -EINVAL;
            goto err;
        }

      ret = bt_uart_file_read(&bridge->filep,
                              bridge->tmpbuf +
                              H4_HEADER_SIZE, hdrlen);
      if (ret < 0)
        {
          goto err;
        }

      hdr = (FAR union bt_hci_hdr_u *)(bridge->tmpbuf + H4_HEADER_SIZE);
      switch (bridge->tmpbuf[0])
        {
          case H4_EVT:
            pktlen = hdr->evt.len;
            break;
          case H4_ACL:
            pktlen = hdr->acl.len;
            break;
          case H4_ISO:
            pktlen = hdr->iso.len;
            break;
          default:
            ret = -EINVAL;
            goto err;
        }

      ret = bt_uart_file_read(&bridge->filep,
                              bridge->tmpbuf +
                              H4_HEADER_SIZE + hdrlen, pktlen);
      if (ret < 0)
        {
          goto err;
        }

#ifdef CONFIG_BLUETOOTH_UART_BRIDGE_BTSNOOP
      snoop_dump(&bridge->snoop, bridge->tmpbuf,
                 H4_HEADER_SIZE + hdrlen + pktlen,
                 0, SNOOP_DIRECTION_FLAG_RECV);
#endif /* CONFIG_BLUETOOTH_UART_BRIDGE_BTSNOOP */

      ret += H4_HEADER_SIZE + hdrlen;
      for (i = 0; i < BT_UART_FILTER_TYPE_COUNT; i++)
        {
          iterator = &bridge->device[i];
          if (bt_uart_filter_forward_recv(&iterator->filter,
                                          bridge->tmpbuf, ret))
            {
              bt_uart_circbuf_write(iterator, bridge->tmpbuf, ret);
              nxsem_post(&iterator->recvsig);
            }
        }

      nxsem_post(&bridge->recvlock);
    }

  while (circbuf_is_empty(&device->recvbuf))
    {
      nxsem_wait_uninterruptible(&device->recvsig);
    }

  return bt_uart_circbuf_read(device, buffer, buflen);

err:
  nxsem_post(&bridge->recvlock);
  return ret;
}

static ssize_t bt_uart_bridge_write(FAR struct file *filep,
                                    FAR const char *buffer, size_t buflen)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct bt_uart_bridge_device_s *device = inode->i_private;
  FAR struct bt_uart_bridge_s *bridge = device->bridge;
  FAR union bt_hci_hdr_u *hdr;
  size_t pktlen;
  size_t hdrlen;
  int ret;

  ret = nxsem_wait_uninterruptible(&bridge->sendlock);
  if (ret < 0)
    {
      return ret;
    }

  if (device->sendlen + buflen > HCI_SENDBUF_SIZE)
    {
      ret = -EINVAL;
      goto err;
    }

  memcpy(device->sendbuf + device->sendlen, buffer, buflen);
  device->sendlen += buflen;

  hdr = (FAR union bt_hci_hdr_u *)(device->sendbuf + 1);

  while (1)
    {
      switch (device->sendbuf[0])
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

      if (device->sendlen < hdrlen)
        {
          goto out;
        }

      pktlen += hdrlen;
      if (device->sendlen < pktlen)
        {
          goto out;
        }

#ifdef CONFIG_BLUETOOTH_UART_BRIDGE_BTSNOOP
      snoop_dump(&bridge->snoop, device->sendbuf,
                 pktlen, 0, SNOOP_DIRECTION_FLAG_SENT);
#endif /* CONFIG_BLUETOOTH_UART_BRIDGE_BTSNOOP */

      /* Got the full packet, check and send out */

      if (bt_uart_filter_forward_send(&device->filter,
                                      device->sendbuf, pktlen))
        {
          ret = file_write(&bridge->filep, device->sendbuf, pktlen);
          if (ret < 0)
            {
              goto err;
            }
        }

      device->sendlen -= pktlen;
      if (device->sendlen == 0)
        {
          goto out;
        }

      memmove(device->sendbuf, device->sendbuf + pktlen, device->sendlen);
    }

err:
  device->sendlen = 0;
out:
  nxsem_post(&bridge->sendlock);
  return ret < 0 ? ret : buflen;
}

static int bt_uart_bridge_ioctl(FAR struct file *filep,
                                int cmd, unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct bt_uart_bridge_device_s *device = inode->i_private;
  FAR struct bt_uart_bridge_s *bridge = device->bridge;

  return file_ioctl(&bridge->filep, cmd, arg);
}

static int bt_uart_bridge_poll(FAR struct file *filep,
                         FAR struct pollfd *fds, bool setup)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct bt_uart_bridge_device_s *device = inode->i_private;
  FAR struct bt_uart_bridge_s *bridge = device->bridge;
  int ret;

  ret = file_poll(&bridge->filep, fds, setup);
  if (setup && ret >= 0 && !circbuf_is_empty(&device->recvbuf))
    {
      fds->revents |= POLLIN;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int bt_uart_bridge_register(const char *hciname,
                            const char *btname, const char *blename)
{
  FAR struct bt_uart_bridge_device_s *device;
  FAR struct bt_uart_bridge_s *bridge;
  int i = 0;
  int ret;
  FAR const char *bridges[] =
    {
      btname,
      blename
    };

  if (!hciname || (!btname && !blename))
    {
      return -EINVAL;
    }

  bridge = (FAR struct bt_uart_bridge_s *)
          kmm_zalloc(sizeof(struct bt_uart_bridge_s));
  if (bridge == NULL)
    {
      return -ENOMEM;
    }

  nxsem_init(&bridge->recvlock, 0, 1);
  nxsem_init(&bridge->sendlock, 0, 1);

  ret = file_open(&bridge->filep, hciname, O_RDWR);
  if (ret < 0)
    {
      goto err_file;
    }

  for (i = 0; i < BT_UART_FILTER_TYPE_COUNT; i++)
    {
      if (!bridges[i])
        {
          continue;
        }

      device = &bridge->device[i];

      device->bridge = bridge;
      bt_uart_filter_init(&device->filter, i);
      nxsem_init(&device->recvlock, 0, 1);
      nxsem_init(&device->recvsig, 0, 0);

      ret = circbuf_init(&device->recvbuf, NULL, HCI_RECVBUF_SIZE);
      if (ret < 0)
        {
          goto err_circbuf;
        }

      ret = register_driver(bridges[i], &g_hcibridge_ops, 0666, device);
      if (ret < 0)
        {
          goto err_device;
        }
    }

#ifdef CONFIG_BLUETOOTH_UART_BRIDGE_BTSNOOP
  char snoop_file[128];
  snprintf(snoop_file, sizeof(snoop_file),
           CONFIG_BLUETOOTH_UART_BRIDGE_BTSNOOP_PATH "btsnoop-%d.log",
           time (NULL));

  snoop_open(&bridge->snoop, snoop_file, SNOOP_DATALINK_HCI_UART, true);
#endif /* CONFIG_BLUETOOTH_UART_BRIDGE_BTSNOOP */

  return OK;

  for (; i >= 0; i--)
    {
      if (!bridges[i])
        {
          continue;
        }

      device = &bridge->device[i];
      unregister_driver(bridges[i]);
err_device:
      circbuf_uninit(&device->recvbuf);
err_circbuf:
      nxsem_destroy(&device->recvsig);
      nxsem_destroy(&device->recvlock);
    }

  file_close(&bridge->filep);

err_file:
  nxsem_destroy(&bridge->recvlock);
  nxsem_destroy(&bridge->sendlock);

  kmm_free(bridge);

  return ret;
}
