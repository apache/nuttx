/****************************************************************************
 * include/nuttx/net/bluetooth/compat/drivers/bluetooth/hci_lockstep.h
 *
 * Ported to Apache NuttX by chao an <anchao.archer@bytedance.com>.
 *
 ****************************************************************************/

/** @file
 *  @brief Bluetooth HCI lockstep command helper.
 */

/* Copyright (c) 2026 Silicon Laboratories Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __INCLUDE_NUTTX_NET_BLUETOOTH_COMPAT_DRIVERS_BLUETOOTH_HCI_LOCKSTEP_H
#define __INCLUDE_NUTTX_NET_BLUETOOTH_COMPAT_DRIVERS_BLUETOOTH_HCI_LOCKSTEP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <nuttx/spinlock.h>

#include <nuttx/net/bluetooth/hci_pkt.h>
#include <nuttx/net/bluetooth/compat/device.h>
#include <nuttx/net/bluetooth/compat/kernel.h>
#include <nuttx/mm/net_buf.h>

#ifdef __cplusplus
extern "C"
{
#endif

typedef int (*bt_hci_lockstep_send_t)(
  const struct device *dev, const uint8_t *pkt, size_t len);

struct bt_hci_lockstep
{
  const struct device *dev;
  bt_hci_lockstep_send_t send;
  sem_t changed;
  spinlock_t lock;
  struct net_buf_simple *rsp;
  uint16_t opcode;
  uint8_t state;
  uint8_t status;
  clock_t timeout;
};

void bt_hci_lockstep_init(
  struct bt_hci_lockstep *ls, const struct device *dev,
  bt_hci_lockstep_send_t send);
void bt_hci_lockstep_reset(struct bt_hci_lockstep *ls);
bool bt_hci_lockstep_feed(struct bt_hci_lockstep *ls,
                          const uint8_t *pkt, size_t len);
int bt_hci_lockstep_cmd_send_sync(
  struct bt_hci_lockstep *ls, uint16_t opcode, struct net_buf_simple *cmd,
  struct net_buf_simple *rsp);

#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_NET_BLUETOOTH_COMPAT_DRIVERS_BLUETOOTH_HCI_LOCKSTEP_H */
