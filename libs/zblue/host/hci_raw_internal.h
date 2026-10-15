/****************************************************************************
 * libs/zblue/host/hci_raw_internal.h
 *
 * Ported to Apache NuttX by chao an <anchao.archer@bytedance.com>.
 *
 ****************************************************************************/

/* Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef __BT_HCI_RAW_INTERNAL_H
#define __BT_HCI_RAW_INTERNAL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/net/bluetooth/compat/device.h>
#include <nuttx/mm/net_buf.h>

#ifdef __cplusplus
extern "C"
{
#endif

struct bt_dev_raw
{
  const struct device *hci;
};

extern struct bt_dev_raw bt_dev;

#ifdef __cplusplus
}
#endif

#endif /* __BT_HCI_RAW_INTERNAL_H */
