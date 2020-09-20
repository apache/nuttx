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
#include <errno.h>

#include <nuttx/wireless/bluetooth/bt_driver.h>
#include <nuttx/net/bluetooth.h>

#include "up_internal.h"
#include "up_hcisocket_host.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* HCI data types as defined by Linux Kernel */

#define HCI_COMMAND_PKT   0x01
#define HCI_ACLDATA_PKT   0x02
#define HCI_SCODATA_PKT   0x03
#define HCI_EVENT_PKT     0x04
#define HCI_ISODATA_PKT   0x05
#define HCI_DIAG_PKT      0xf0
#define HCI_VENDOR_PKT    0xff

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int bthcisock_send(FAR const struct bt_driver_s *dev,
                          FAR struct bt_buf_s *buf);
static int bthcisock_open(FAR const struct bt_driver_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct bt_driver_s g_bt_hcisock =
{
  0,               /* head_reserve */
  bthcisock_open,  /* open */
  bthcisock_send   /* send */
};

static int bt_fd = -1;       /* Host HCI socket fd */
static int host_dev_id = 0;  /* Host HCI interface number */

/* Hold a receive frame buffer here. This allows us to not allocate and free
 * on every socket read since most of the time there will be no data to
 * actually process.
 */

struct bt_buf_s *read_buf = NULL;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int bthcisock_send(FAR const struct bt_driver_s *dev,
                          FAR struct bt_buf_s *buf)
{
  uint8_t pkt_type;

  switch (buf->type)
    {
      case BT_CMD:
        {
          pkt_type = HCI_COMMAND_PKT;
          break;
        }

      case BT_ACL_OUT:
        {
          pkt_type = HCI_ACLDATA_PKT;
          break;
        }

      default:
        {
          wlerr("Unexpected HCI packet type %d", buf->type);
          return buf->len;
        }
    }

  if (bthcisock_host_send(bt_fd, pkt_type, buf->data, buf->len) < 0)
    {
      return -1;
    }

  bt_buf_release(buf);

  return buf->len;
}

static int bthcisock_open(FAR const struct bt_driver_s *dev)
{
  int fd = bthcisock_host_open(host_dev_id);
  if (fd < 0)
    {
      return -1;
    }

  bt_fd = fd;
  return OK;
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
  /* Register the driver with the Bluetooth stack */

  host_dev_id = dev_id;
  return bt_netdev_register(&g_bt_hcisock);
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

int bthcisock_loop()
{
  uint8_t type;
  int len;

  if (bt_fd < 0)
    {
      /* Internal socket has not yet been created */

      return -EBADF;
    }

  if (read_buf == NULL)
    {
      /* NOTE: This shared allocation only works because the allocation size
       * is currently the same for all frame types.  If this changes we will
       * need to allocate for the largest frame or use an intermediate buffer
       * to copy from
       */

      read_buf = bt_buf_alloc(BT_DUMMY, NULL, BLUETOOTH_H4_HDRLEN);
      if (read_buf == NULL)
        {
          wlerr("ERROR: Failed to allocate buffer\n");
          return -ENOMEM;
        }
    }

  len = bthcisock_host_read(bt_fd, &type, read_buf->data,
                            BLUETOOTH_MAX_FRAMELEN);
  if (len < 0)
    {
      return OK;
    }

  read_buf->len = len;

  switch (type)
    {
      case HCI_EVENT_PKT:
        {
          read_buf->type = BT_EVT;
          break;
        }

      case HCI_ACLDATA_PKT:
        {
          read_buf->type = BT_ACL_IN;
          break;
        }

      default:
        {
          wlerr("Unknown packet type %d\n", type);
          return OK;
        }
    }

    bt_hci_receive(read_buf);

    /* Make sure to allocate a new buffer for the next read.  Bluetooth
     * stack will clean up the allocation of this buffer when it has been
     * handled.
     */

    read_buf = NULL;
    return OK;
}
