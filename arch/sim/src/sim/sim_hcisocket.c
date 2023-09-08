/****************************************************************************
 * arch/sim/src/sim/sim_hcisocket.c
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
#include <nuttx/wireless/bluetooth/bt_bridge.h>
#include <nuttx/serial/uart_bth4.h>

#include "sim_internal.h"
#include "sim_hosthcisocket.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SIM_BTHCI_RX_FRAMELEN 1024
#define SIM_BTHCI_WORK_DELAY  USEC2TICK(1000)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct bthcisock_s
{
  struct bt_driver_s drv;
  int                id;
  int                fd;

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
  char data[SIM_BTHCI_RX_FRAMELEN];
  enum bt_buf_type_e type;
  int ret;

  ret = host_bthcisock_receive(dev->fd, data, sizeof(data));
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

static int bthcisock_driver_register(struct bt_driver_s *drv, int id,
                                     bool bt)
{
#ifdef CONFIG_UART_BTH4
  char name[32];

  if (bt)
    {
      snprintf(name, sizeof(name), "/dev/ttyBT%d", id);
    }
  else
    {
      snprintf(name, sizeof(name), "/dev/ttyBLE%d", id);
    }

  return uart_bth4_register(name, drv);
#else
  return bt_netdev_register(drv);
#endif
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
#if defined(CONFIG_BLUETOOTH_BRIDGE)
  struct bt_driver_s *btdrv;
  struct bt_driver_s *bledrv;
#endif
  int ret;

  dev = bthcisock_alloc(dev_id);
  if (dev == NULL)
    {
      return -ENOMEM;
    }

#if defined(CONFIG_BLUETOOTH_BRIDGE)
  ret = bt_bridge_register(&dev->drv, &btdrv, &bledrv);
  if (ret < 0)
    {
      goto end;
    }

  ret = bthcisock_driver_register(btdrv, dev_id, true);
  if (ret < 0)
    {
      goto end;
    }

  ret = bthcisock_driver_register(bledrv, dev_id, false);
  if (ret < 0)
    {
      goto end;
    }

end:
#else
  ret = bthcisock_driver_register(&dev->drv, dev_id, true);
#endif

  if (ret < 0)
    {
      bthcisock_free(dev);
      return ret;
    }

  return work_queue(HPWORK, &dev->worker, sim_bthcisock_work, dev, 0);
}
