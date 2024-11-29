/****************************************************************************
 * arch/sim/src/sim/sim_hcisocket.c
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

#include <sys/types.h>

#include <sys/socket.h>

#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/nuttx.h>
#include <nuttx/kmalloc.h>
#include <nuttx/wqueue.h>
#include <nuttx/net/bluetooth.h>
#include <nuttx/wireless/bluetooth/bt_driver.h>
#include <nuttx/wireless/bluetooth/bt_uart.h>

#include "sim_internal.h"
#include "sim_hosthcisocket.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SIM_BTHCI_RX_FRAMELEN 2048
#define SIM_BTHCI_WORK_DELAY  USEC2TICK(1000)

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
struct bthcisock_s
{
  struct bt_driver_s drv;
  int                id;
  int                fd;

  uint16_t           rxlen;
  uint8_t            rxbuf[SIM_BTHCI_RX_FRAMELEN];

  /* Work queue for transmit */

  struct work_s      worker;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  bthcisock_send(struct bt_driver_s *drv,
                           enum bt_buf_type_e type,
                           void *data, size_t len);
static int  bthcisock_open(struct bt_driver_s *drv);
static void bthcisock_close(struct bt_driver_s *drv);
static int  bthcisock_receive(struct bt_driver_s *drv);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int bthcisock_send(struct bt_driver_s *drv,
                          enum bt_buf_type_e type,
                          void *data, size_t len)
{
  struct bthcisock_s *dev = (struct bthcisock_s *)drv;
  char *hdr = (char *)data - drv->head_reserve;
  int ret;

  if (type == BT_CMD)
    {
      *hdr = H4_CMD;
    }
  else if (type == BT_ACL_OUT)
    {
      *hdr = H4_ACL;
    }
  else if (type == BT_ISO_OUT)
    {
      *hdr = H4_ISO;
    }
  else
    {
      return -EINVAL;
    }

  ret = host_bthcisock_send(dev->fd, hdr, len + H4_HEADER_SIZE);

  return ret < 0 ? ret : len;
}

static void bthcisock_close(struct bt_driver_s *drv)
{
  struct bthcisock_s *dev = (struct bthcisock_s *)drv;

  host_bthcisock_close(dev->fd);
  dev->fd = -1;
}

static int bthcisock_receive(struct bt_driver_s *drv)
{
  struct bthcisock_s *dev = (struct bthcisock_s *)drv;
  enum bt_buf_type_e type;
  union bt_hdr_u *hdr;
  uint16_t pktlen;
  int ret;

  ret = host_bthcisock_receive(dev->fd, &dev->rxbuf[dev->rxlen],
                               sizeof(dev->rxbuf) - dev->rxlen);
  if (ret <= 0)
    {
      return ret;
    }

  dev->rxlen += (uint16_t)ret;

  while (dev->rxlen)
    {
      hdr = (union bt_hdr_u *)&dev->rxbuf[H4_HEADER_SIZE];
      switch (dev->rxbuf[0])
        {
        case H4_EVT:
          {
            if (dev->rxlen < H4_HEADER_SIZE
                + sizeof (struct bt_hci_evt_hdr_s))
              {
                return ret;
              }

            type = BT_EVT;
            pktlen = H4_HEADER_SIZE + sizeof(struct bt_hci_evt_hdr_s)
                     + hdr->evt.len;
          }
          break;
        case H4_ACL:
          {
            if (dev->rxlen < H4_HEADER_SIZE
                + sizeof(struct bt_hci_acl_hdr_s))
              {
                return ret;
              }

            type = BT_ACL_IN;
            pktlen = H4_HEADER_SIZE + sizeof(struct bt_hci_acl_hdr_s)
                     + hdr->acl.len;
          }
          break;
        case H4_ISO:
          {
            if (dev->rxlen < H4_HEADER_SIZE
                + sizeof(struct bt_hci_iso_hdr_s))
              {
                return ret;
              }

            type = BT_ISO_IN;
            pktlen = H4_HEADER_SIZE + sizeof(struct bt_hci_iso_hdr_s)
                     + hdr->iso.len;
          }
          break;
        default:
          return -EINVAL;
        }

      if (dev->rxlen < pktlen)
        {
          return ret;
        }

      bt_netdev_receive(&dev->drv, type, dev->rxbuf + H4_HEADER_SIZE,
                        pktlen - H4_HEADER_SIZE);
      dev->rxlen -= pktlen;
      memmove(dev->rxbuf, dev->rxbuf + pktlen, dev->rxlen);
    }

  return ret;
}

static int bthcisock_open(struct bt_driver_s *drv)
{
  struct bthcisock_s *dev = (struct bthcisock_s *)drv;
  int fd;

  fd = host_bthcisock_open(dev->id);

  if (fd < 0)
    {
      return fd;
    }

  dev->fd = fd;

  return OK;
}

static struct bthcisock_s *bthcisock_alloc(int dev_id)
{
  /* Register the driver with the Bluetooth stack */

  struct bthcisock_s *dev;
  struct bt_driver_s *drv;

  dev = kmm_zalloc(sizeof(*dev));
  if (dev == NULL)
    {
      return NULL;
    }

  dev->id           = dev_id;
  dev->fd           = -1;
  drv               = &dev->drv;
  drv->head_reserve = H4_HEADER_SIZE;
  drv->open         = bthcisock_open;
  drv->send         = bthcisock_send;
  drv->close        = bthcisock_close;

  return dev;
}

static void bthcisock_free(struct bthcisock_s *dev)
{
  kmm_free(dev);
}

/****************************************************************************
 * Name: sim_bthcisock_work
 *
 * Description:
 *   Feed pending packets on the host sockets into the Bluetooth stack.
 *
 ****************************************************************************/

static void sim_bthcisock_work(void *arg)
{
  struct bthcisock_s *dev = arg;

  if (host_bthcisock_avail(dev->fd))
    {
      bthcisock_receive(&dev->drv);
    }

  work_queue(HPWORK, &dev->worker,
            sim_bthcisock_work, dev, SIM_BTHCI_WORK_DELAY);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sim_bthcisock_register
 *
 * Description:
 *   Register the Linux HCI interface with the Bluetooth stack
 *
 * Input Parameters:
 *   dev_id: This is the interface number known to the Linux Kernel
 *
 * Returned Value:
 *   Zero is returned on success; a negated errno value is returned on any
 *   failure.
 *
 ****************************************************************************/

int sim_bthcisock_register(int dev_id)
{
  struct bthcisock_s *dev;
  int ret;

  dev = bthcisock_alloc(dev_id);
  if (dev == NULL)
    {
      return -ENOMEM;
    }

  ret = bt_driver_register_with_id(&dev->drv, dev_id);
  if (ret < 0)
    {
      wlerr("bt_driver_register error: %d\n", ret);
      bthcisock_free(dev);
      return ret;
    }

  return work_queue(HPWORK, &dev->worker, sim_bthcisock_work, dev, 0);
}
