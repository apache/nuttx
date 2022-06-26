/****************************************************************************
 * drivers/wireless/bluetooth/bt_null.c
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

/* NULL (do-nothing) Bluetooth driver */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/wireless/bluetooth/bt_driver.h>

#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/mm/iob.h>
#include <nuttx/net/bluetooth.h>
#include <nuttx/wireless/bluetooth/bt_hci.h>
#include <nuttx/wireless/bluetooth/bt_null.h>
#include <nuttx/wireless/bluetooth/bt_driver.h>

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void btnull_format_cmdcomplete(FAR struct bt_buf_s *buf,
                                      uint16_t opcode);
static void btnull_format_bdaddr_rsp(FAR struct bt_buf_s *buf,
                                     uint16_t opcode);

static int  btnull_open(FAR struct bt_driver_s *dev);
static int  btnull_send(FAR struct bt_driver_s *dev,
                        enum bt_buf_type_e type,
                        FAR void *data, size_t len);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct bt_driver_s g_bt_null =
{
  0,            /* head_reserve */
  btnull_open,  /* open */
  btnull_send   /* send */
};

static const bt_addr_t g_bt_addr =
{
  {0xfe, 0xed, 0xb0, 0x0b, 0xfa, 0xce}
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void btnull_format_cmdcomplete(FAR struct bt_buf_s *buf,
                                      uint16_t opcode)
{
  struct bt_hci_evt_hdr_s evt;
  struct hci_evt_cmd_complete_s cmd;
  FAR uint8_t *data = buf->frame->io_data;
  int ndx;
  int len;

  /* Minimal setup for the command complete event */

  len        = sizeof(struct bt_hci_evt_hdr_s) +
               sizeof(struct hci_evt_cmd_complete_s);
  ndx        = 0;

  evt.evt    = BT_HCI_EVT_CMD_COMPLETE;
  evt.len    = len;
  memcpy(&data[ndx], &evt, sizeof(struct bt_hci_evt_hdr_s));
  ndx       += sizeof(struct bt_hci_evt_hdr_s);

  cmd.ncmd   = 1;
  cmd.opcode = opcode;
  memcpy(&data[ndx], &cmd, sizeof(struct hci_evt_cmd_complete_s));
  ndx       += sizeof(struct hci_evt_cmd_complete_s);

  buf->frame->io_len = len;
  buf->len           = len;
}

static void btnull_format_local_features_rsp(FAR struct bt_buf_s *buf,
                                             uint16_t opcode)
{
  struct bt_hci_rp_le_read_local_features_s features;
  FAR uint8_t *data = buf->frame->io_data;
  int ndx;
  int len;

  /* Return local features */

  btnull_format_cmdcomplete(buf, opcode);

  len        = buf->len +
               sizeof(struct bt_hci_rp_le_read_local_features_s);
  ndx        = buf->len;

  memset(&features, 0, sizeof(struct bt_hci_rp_le_read_local_features_s));
  if (opcode == BT_HCI_OP_READ_LOCAL_FEATURES)
    {
      features.features[4] = BT_LMP_LE;
    }
  else /* if opcode == BT_HCI_OP_LE_READ_LOCAL_FEATURES */
    {
      features.features[0] = BT_HCI_LE_ENCRYPTION;
    }

  memcpy(&data[ndx], &features,
         sizeof(struct bt_hci_rp_le_read_local_features_s));
  ndx       += sizeof(struct bt_hci_rp_le_read_local_features_s);

  buf->frame->io_len = len;
  buf->len           = len;
}

static void btnull_format_bdaddr_rsp(FAR struct bt_buf_s *buf,
                                     uint16_t opcode)
{
  struct bt_hci_rp_read_bd_addr_s bdaddr;
  FAR uint8_t *data = buf->frame->io_data;
  int ndx;
  int len;

  btnull_format_cmdcomplete(buf, opcode);

  len        = buf->len +
               sizeof(struct bt_hci_rp_read_bd_addr_s);
  ndx        = buf->len;

  BLUETOOTH_ADDRCOPY(bdaddr.bdaddr.val, g_bt_addr.val);
  bdaddr.status = 0;
  memcpy(&data[ndx], &bdaddr, sizeof(struct bt_hci_rp_read_bd_addr_s));
  ndx       += sizeof(struct bt_hci_rp_read_bd_addr_s);

  buf->frame->io_len = len;
  buf->len           = len;
}

static void btnull_format_buffersize_rsp(FAR struct bt_buf_s *buf,
                                         uint16_t opcode)
{
  struct bt_hci_rp_le_read_buffer_size_s bufsize;
  FAR uint8_t *data = buf->frame->io_data;
  int ndx;
  int len;

  btnull_format_cmdcomplete(buf, opcode);

  len        = buf->len +
               sizeof(struct bt_hci_rp_le_read_buffer_size_s);
  ndx        = buf->len;

  bufsize.status     = 0;
  bufsize.le_max_len = BLUETOOTH_MAX_FRAMELEN;
  bufsize.le_max_num = 1;
  memcpy(&data[ndx], &bufsize,
          sizeof(struct bt_hci_rp_le_read_buffer_size_s));
  ndx       += sizeof(struct bt_hci_rp_le_read_buffer_size_s);

  buf->frame->io_len = len;
  buf->len           = len;
}

static int btnull_send(FAR struct bt_driver_s *dev,
                       enum bt_buf_type_e type,
                       FAR void *data, size_t len)
{
  wlinfo("Bit bucket: length %zu\n", len);

  /* Is the Bluetooth stack waiting for an event? */

  if (type == BT_CMD)
    {
      FAR struct bt_hci_cmd_hdr_s *hdr = data;
      FAR struct bt_buf_s *outbuf;
      uint16_t opcode = hdr->opcode;

      wlinfo("CMD: %04x\n", opcode);

      outbuf = bt_buf_alloc(BT_EVT, NULL, 0);
      if (outbuf == NULL)
        {
          wlerr("ERROR: Failed to allocate buffer\n");
          return -ENOMEM;
        }

      switch (opcode)
        {
          case BT_HCI_OP_READ_LOCAL_FEATURES:
          case BT_HCI_OP_LE_READ_LOCAL_FEATURES:
            btnull_format_local_features_rsp(outbuf, opcode);
            break;

          case BT_HCI_OP_READ_BD_ADDR:
            btnull_format_bdaddr_rsp(outbuf, opcode);
            break;

          case BT_HCI_OP_LE_READ_BUFFER_SIZE:
            btnull_format_buffersize_rsp(outbuf, opcode);
            break;

          default:
            btnull_format_cmdcomplete(outbuf, opcode);
            break;
        }

      wlinfo("Send CMD complete event\n");

      bt_netdev_receive(dev, outbuf->type,
                        outbuf->data, outbuf->len);
      bt_buf_release(outbuf);
    }

  return len;
}

static int btnull_open(FAR struct bt_driver_s *dev)
{
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: btnull_register
 *
 * Description:
 *   Register the NULL Bluetooth stack with the Bluetooth stack
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero is returned on success; a negated errno value is returned on any
 *   failure.
 *
 ****************************************************************************/

int btnull_register(void)
{
  /* Register the driver with the Bluetooth stack */

  return bt_netdev_register(&g_bt_null);
}
