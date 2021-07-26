/****************************************************************************
 * arch/sim/src/sim/up_hcisocket.c
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
#include <queue.h>

#include <nuttx/nuttx.h>
#include <nuttx/kmalloc.h>
#include <nuttx/net/bluetooth.h>
#include <nuttx/wireless/bluetooth/bt_driver.h>
#include <nuttx/wireless/bluetooth/bt_uart.h>
#include <nuttx/serial/uart_bth4.h>

#include "up_internal.h"
#include "up_hcisocket_host.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BLUETOOTH_RX_FRAMELEN 1024

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct bthcisock_s
{
  FAR struct bt_driver_s drv;
  int                    id;
  int                    fd;
  sq_entry_t             link;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  bthcisock_send(FAR struct bt_driver_s *drv,
                           enum bt_buf_type_e type,
                           FAR void *data, size_t len);
static int  bthcisock_open(FAR struct bt_driver_s *drv);
static void bthcisock_close(FAR struct bt_driver_s *drv);
static int  bthcisock_receive(FAR struct bt_driver_s *drv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static sq_queue_t        g_bthcisock_list;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int bthcisock_send(FAR struct bt_driver_s *drv,
                          enum bt_buf_type_e type,
                          FAR void *data, size_t len)
{
  FAR struct bthcisock_s *dev = (FAR struct bthcisock_s *)drv;
  FAR char *hdr = (FAR char *)data - drv->head_reserve;
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

  ret = bthcisock_host_send(dev->fd, hdr, len + H4_HEADER_SIZE);

  return ret < 0 ? ret : len;
}

static void bthcisock_close(FAR struct bt_driver_s *drv)
{
  FAR struct bthcisock_s *dev = (FAR struct bthcisock_s *)drv;

  bthcisock_host_close(dev->fd);
  dev->fd = -1;
}

static int bthcisock_receive(FAR struct bt_driver_s *drv)
{
  FAR struct bthcisock_s *dev = (FAR struct bthcisock_s *)drv;
  char data[BLUETOOTH_RX_FRAMELEN];
  enum bt_buf_type_e type;
  int ret;

  ret = bthcisock_host_read(dev->fd, data, sizeof(data));
  if (ret <= 0)
    {
      return ret;
    }

  if (data[0] == H4_EVT)
    {
      type = BT_EVT;
    }
  else if (data[0] == H4_ACL)
    {
      type = BT_ACL_IN;
    }
  else if (data[0] == H4_ISO)
    {
      type = BT_ISO_IN;
    }
  else
    {
      return -EINVAL;
    }

  return bt_netdev_receive(&dev->drv, type,
                           data + H4_HEADER_SIZE,
                           ret - H4_HEADER_SIZE);
}

static int bthcisock_open(FAR struct bt_driver_s *drv)
{
  FAR struct bthcisock_s *dev = (FAR struct bthcisock_s *)drv;
  int fd;

  fd = bthcisock_host_open(dev->id);

  if (fd < 0)
    {
      return fd;
    }

  dev->fd = fd;

  return OK;
}

static FAR struct bthcisock_s *bthcisock_alloc(int dev_id)
{
  /* Register the driver with the Bluetooth stack */

  FAR struct bthcisock_s *dev;
  FAR struct bt_driver_s *drv;

  dev = (FAR struct bthcisock_s *)kmm_zalloc(sizeof(*dev));
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

  sq_addlast(&dev->link, &g_bthcisock_list);

  return dev;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bthcisock_register
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

int bthcisock_register(int dev_id)
{
  FAR struct bthcisock_s *dev;
#if defined(CONFIG_UART_BTH4)
  char name[32];
#endif
  int ret;

  dev = bthcisock_alloc(dev_id);
  if (dev == NULL)
    {
      return -ENOMEM;
    }

#if defined(CONFIG_UART_BTH4)
  snprintf(name, sizeof(name), "/dev/ttyHCI%d", dev_id);
  ret = uart_bth4_register(name, &dev->drv);
#else
  ret = bt_netdev_register(&dev->drv);
#endif
  if (ret < 0)
    {
      kmm_free(dev);
    }

  return ret;
}

/****************************************************************************
 * Name: bthcisock_loop
 *
 * Description:
 *   Feed pending packets on the host sockets into the Bluetooth stack.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero is returned on success; a negated errno value is returned on any
 *   failure.
 *
 ****************************************************************************/

int bthcisock_loop(void)
{
  FAR struct bthcisock_s *dev;
  FAR sq_entry_t *entry;

  for (entry = sq_peek(&g_bthcisock_list); entry; entry = sq_next(entry))
    {
      dev = container_of(entry, struct bthcisock_s, link);
      if (bthcisock_host_avail(dev->fd))
        {
          bthcisock_receive(&dev->drv);
        }
    }

  return 0;
}
