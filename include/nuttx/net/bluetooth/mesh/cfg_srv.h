/****************************************************************************
 * include/nuttx/net/bluetooth/mesh/cfg_srv.h
 *
 * Ported to Apache NuttX by chao an <magicdcps@hotmail.com>.
 *
 ****************************************************************************/

/** @file
 *  @brief Configuration Server Model APIs.
 */

/* Copyright (c) 2017 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef __INCLUDE_NUTTX_NET_BLUETOOTH_MESH_CFG_SRV_H
#define __INCLUDE_NUTTX_NET_BLUETOOTH_MESH_CFG_SRV_H

/**
 * @brief Configuration Server Model
 * @defgroup bt_mesh_cfg_srv Configuration Server Model
 * @ingroup bt_mesh
 * @{
 */

#ifdef __cplusplus
extern "C"
{
#endif

/**
 *  @brief Generic Configuration Server model composition data entry.
 */
#define BT_MESH_MODEL_CFG_SRV                                                                      \
BT_MESH_MODEL_CNT_CB(BT_MESH_MODEL_ID_CFG_SRV, bt_mesh_cfg_srv_op, NULL, NULL, 1, 0,             \
                     &bt_mesh_cfg_srv_cb)

/** @cond INTERNAL_HIDDEN */

extern const struct bt_mesh_model_op bt_mesh_cfg_srv_op[];
extern const struct bt_mesh_model_cb bt_mesh_cfg_srv_cb;

/** @endcond */

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif /* __INCLUDE_NUTTX_NET_BLUETOOTH_MESH_CFG_SRV_H */
