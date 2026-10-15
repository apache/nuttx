/****************************************************************************
 * libs/zblue/common/hci_public_addr.c
 *
 * Ported to Apache NuttX by chao an <magicdcps@hotmail.com>.
 *
 ****************************************************************************/

/* Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/net/bluetooth/addr.h>
#include <nuttx/net/bluetooth/compat/device.h>
#include <nuttx/net/bluetooth/compat/drivers/bluetooth.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* "No address" is BT_ADDR_ANY, as in the setup() op's parameters and, as the
 * address part of BT_ADDR_LE_ANY, in the Host's identity table, and not
 * BT_ADDR_NONE: zero-initialized driver data then needs no flag to say that
 * nothing has been set.
 */

void bt_hci_set_public_addr(const struct device *dev, const bt_addr_t *addr)
{
  struct bt_hci_driver_data *data = dev->data;

  bt_addr_copy(&data->public_addr, addr);
}

const bt_addr_t *bt_hci_get_public_addr(const struct device *dev)
{
  const struct bt_hci_driver_data *data = dev->data;

  return &data->public_addr;
}
