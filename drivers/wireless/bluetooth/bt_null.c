/****************************************************************************
 * drivers/wireless/bluetooth/bt_null.c
 * UART based Bluetooth driver
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

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

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void btnull_format_cmdcomplete(FAR struct bt_buf_s *buf,
                                      uint16_t opcode);
static void btnull_format_bdaddr_rsp(FAR struct bt_buf_s *buf,
                                     uint16_t opcode);

static int  btnull_open(FAR const struct bt_driver_s *dev);
static int  btnull_send(FAR const struct bt_driver_s *dev,
                        FAR struct bt_buf_s *buf);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct bt_driver_s g_bt_null =
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
  memcpy(&data[ndx], &bufsize, sizeof(struct bt_hci_rp_le_read_buffer_size_s));
  ndx       += sizeof(struct bt_hci_rp_le_read_buffer_size_s);

  buf->frame->io_len = len;
  buf->len           = len;
}

static int btnull_send(FAR const struct bt_driver_s *dev,
                       FAR struct bt_buf_s *buf)
{
  wlinfo("Bit bucket: length %d\n", (int)buf->len);

  /* Is the Bluetooth stack waiting for an event? */

  if (buf->type == BT_CMD)
    {
      FAR struct bt_hci_cmd_hdr_s *hdr = (FAR void *)buf->data;
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

      bt_hci_receive(outbuf);
    }

  return buf->len;
}

static int btnull_open(FAR const struct bt_driver_s *dev)
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
