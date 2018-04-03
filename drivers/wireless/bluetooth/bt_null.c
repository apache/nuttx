/****************************************************************************
 * drivers/wireless/bluetooth/bt_null.c
 * UART based Bluetooth driver
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/wireless/bt_driver.h>

#include <string.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/wireless/bt_hci.h>
#include <nuttx/wireless/bt_null.h>

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void btnull_send_cmdcomplete(FAR const struct bt_driver_s *dev,
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

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void btnull_send_cmdcomplete(FAR const struct bt_driver_s *dev,
                                    uint16_t opcode)
{
  FAR struct bt_buf_s *buf;

  buf = bt_buf_alloc(BT_EVT, NULL, 0);
  if (buf != NULL)
    {
      struct bt_hci_evt_hdr_s evt;
      struct hci_evt_cmd_complete_s cmd;

      /* Minimal setup for the command complete event */

      evt.evt    = BT_HCI_EVT_CMD_COMPLETE;
      evt.len    = sizeof(struct bt_hci_evt_hdr_s) +
                   sizeof(struct hci_evt_cmd_complete_s);
      memcpy(bt_buf_extend(buf, sizeof(struct bt_hci_evt_hdr_s)), &evt,
                           sizeof(struct bt_hci_evt_hdr_s));

      cmd.ncmd   = 1;
      cmd.opcode = opcode;
      memcpy(bt_buf_extend(buf, sizeof(struct hci_evt_cmd_complete_s)),
                           &cmd, sizeof(struct hci_evt_cmd_complete_s));

      wlinfo("Send CMD complete event\n");

      bt_hci_receive(buf);
    }
}

static int btnull_send(FAR const struct bt_driver_s *dev,
                       FAR struct bt_buf_s *buf)
{
  wlinfo("Bit buffer: length %d\n", (int)buf->len);

  /* Is the Bluetooth stack waiting for an event? */

  if (buf->type == BT_CMD)
    {
      FAR struct bt_hci_cmd_hdr_s *hdr = (FAR void *)buf->data;

      wlinfo("CMD: %04x\n", hdr->opcode);
      btnull_send_cmdcomplete(dev, hdr->opcode);
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

  return bt_driver_register(&g_bt_null);
}
