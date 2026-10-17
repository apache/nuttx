/****************************************************************************
 * include/nuttx/net/bluetooth/mesh/sar_cfg_srv.h
 *
 * Copyright (c) ByteDance Ltd. and/or its affiliates.
 *
 * Author: chao an <anchao.archer@bytedance.com>
 *
 * SPDX-License-Identifier: Apache-2.0
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

/* Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/** @file
 *  @brief Bluetooth Mesh SAR Configuration Server Model APIs.
 */
#ifndef __INCLUDE_NUTTX_NET_BLUETOOTH_MESH_SAR_CFG_SRV_H
#define __INCLUDE_NUTTX_NET_BLUETOOTH_MESH_SAR_CFG_SRV_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/net/bluetooth/mesh.h>
#include <nuttx/net/bluetooth/mesh/sar_cfg.h>

/**
 * @brief Bluetooth Mesh
 * @defgroup bt_mesh_sar_cfg_srv Bluetooth Mesh SAR Configuration Server
 * Model
 * @ingroup bt_mesh
 * @{
 */

#ifdef __cplusplus
extern "C"
{
#endif

/**
 *
 *  @brief Transport SAR Configuration Server model composition data entry.
 */
#define BT_MESH_MODEL_SAR_CFG_SRV                                                                  \
BT_MESH_MODEL_CB(BT_MESH_MODEL_ID_SAR_CFG_SRV, bt_mesh_sar_cfg_srv_op, NULL, NULL,               \
                 &bt_mesh_sar_cfg_srv_cb)

/** @cond INTERNAL_HIDDEN */

extern const struct bt_mesh_model_op bt_mesh_sar_cfg_srv_op[];
extern const struct bt_mesh_model_cb bt_mesh_sar_cfg_srv_cb;

/** @endcond */

#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_NET_BLUETOOTH_MESH_SAR_CFG_SRV_H */

/**
 * @}
 */
