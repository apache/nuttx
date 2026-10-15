/****************************************************************************
 * libs/zblue/mesh/pb_gatt_srv.h
 *
 * Ported to Apache NuttX by chao an <anchao.archer@bytedance.com>.
 *
 ****************************************************************************/

/* Copyright (c) 2017 Intel Corporation Copyright (c) 2021 Lingao Meng
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_SUBSYS_BLUETOOTH_MESH_PB_GATT_SRV_H_
#define ZEPHYR_SUBSYS_BLUETOOTH_MESH_PB_GATT_SRV_H_

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/net/bluetooth/gatt.h>

int bt_mesh_pb_gatt_srv_enable(void);
int bt_mesh_pb_gatt_srv_disable(void);

int bt_mesh_pb_gatt_srv_adv_start(void);

#endif /* ZEPHYR_SUBSYS_BLUETOOTH_MESH_PB_GATT_SRV_H_ */
