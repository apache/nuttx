/****************************************************************************
 * libs/zblue/host/hci_common.c
 *
 * Ported to Apache NuttX by chao an <magicdcps@hotmail.com>.
 *
 ****************************************************************************/

/* Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdbool.h>
#include <stdint.h>

#include <nuttx/net/bluetooth/buf.h>
#include <nuttx/net/bluetooth/hci_types.h>
#include <nuttx/net/bluetooth/compat/drivers/bluetooth.h>
#include <nuttx/net/bluetooth/compat/kernel.h>
#include <nuttx/net/bluetooth/compat/net_buf.h>
#include <nuttx/net/bluetooth/compat/sys/byteorder.h>

#include "common/assert.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

struct net_buf *bt_hci_evt_create(uint8_t evt, uint8_t len)
{
  struct bt_hci_evt_hdr *hdr;
  struct net_buf *buf;

  buf = bt_buf_get_evt(evt, false, ((clock_t)-1));

  BT_ASSERT(buf);

  hdr = net_buf_add(buf, sizeof(*hdr));
  hdr->evt = evt;
  hdr->len = len;

  return buf;
}

struct net_buf *bt_hci_cmd_complete_create(uint16_t op, uint8_t plen)
{
  struct net_buf *buf;
  struct bt_hci_evt_cmd_complete *cc;

  buf = bt_hci_evt_create(BT_HCI_EVT_CMD_COMPLETE, sizeof(*cc) + plen);

  cc = net_buf_add(buf, sizeof(*cc));

  /* The Num_HCI_Command_Packets parameter allows the Controller to
   * indicate the number of HCI command packets the Host can send to the
   * Controller. If the Controller requires the Host to stop sending
   * commands, Num_HCI_Command_Packets will be set to zero.
   *
   * NOTE: Zephyr Controller (and may be other Controllers) do not support
   * higher Number of HCI Command packets than 1.
   */

  cc->ncmd = 1U;

  cc->opcode = sys_cpu_to_le16(op);

  return buf;
}

struct net_buf *bt_hci_cmd_status_create(uint16_t op, uint8_t status)
{
  struct net_buf *buf;
  struct bt_hci_evt_cmd_status *cs;

  buf = bt_hci_evt_create(BT_HCI_EVT_CMD_STATUS, sizeof(*cs));

  cs = net_buf_add(buf, sizeof(*cs));
  cs->status = status;

  /* The Num_HCI_Command_Packets parameter allows the Controller to
   * indicate the number of HCI command packets the Host can send to the
   * Controller. If the Controller requires the Host to stop sending
   * commands, Num_HCI_Command_Packets will be set to zero.
   *
   * NOTE: Zephyr Controller (and may be other Controllers) do not support
   * higher Number of HCI Command packets than 1.
   */

  cs->ncmd = 1U;

  cs->opcode = sys_cpu_to_le16(op);

  return buf;
}
