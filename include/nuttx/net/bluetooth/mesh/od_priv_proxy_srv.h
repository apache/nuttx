/****************************************************************************
 * include/nuttx/net/bluetooth/mesh/od_priv_proxy_srv.h
 *
 * Ported to Apache NuttX by chao an <magicdcps@hotmail.com>.
 *
 ****************************************************************************/

/* Copyright (c) 2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Header file for the Bluetooth Mesh On-Demand Private GATT Proxy
 * Server model API.
 * @ingroup bt_mesh_od_priv_proxy_srv
 */

#ifndef __INCLUDE_NUTTX_NET_BLUETOOTH_MESH_OD_PRIV_PROXY_SRV_H
#define __INCLUDE_NUTTX_NET_BLUETOOTH_MESH_OD_PRIV_PROXY_SRV_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/net/bluetooth/mesh.h>

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @defgroup bt_mesh_od_priv_proxy_srv Bluetooth Mesh On-Demand Private GATT
 * Proxy Server
 * @ingroup bt_mesh
 * @{
 */

/**
 *  @brief On-Demand Private Proxy Server model composition data entry.
 */
#define BT_MESH_MODEL_OD_PRIV_PROXY_SRV                                                            \
BT_MESH_MODEL_SOL_PDU_RPL_SRV,                                                                   \
        BT_MESH_MODEL_CB(BT_MESH_MODEL_ID_ON_DEMAND_PROXY_SRV, _bt_mesh_od_priv_proxy_srv_op,    \
                         NULL, NULL, &_bt_mesh_od_priv_proxy_srv_cb)

/** @cond INTERNAL_HIDDEN */

extern const struct bt_mesh_model_op _bt_mesh_od_priv_proxy_srv_op[];
extern const struct bt_mesh_model_cb _bt_mesh_od_priv_proxy_srv_cb;

/** @endcond */

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_NET_BLUETOOTH_MESH_OD_PRIV_PROXY_SRV_H */
