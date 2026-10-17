/****************************************************************************
 * include/nuttx/net/bluetooth/mesh/op_agg_srv.h
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

/* Copyright (c) 2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Header file for the Bluetooth Mesh Opcodes Aggregator Server model
 * API.
 * @ingroup bt_mesh_op_agg_srv
 */

#ifndef __INCLUDE_NUTTX_NET_BLUETOOTH_MESH_OP_AGG_SRV_H
#define __INCLUDE_NUTTX_NET_BLUETOOTH_MESH_OP_AGG_SRV_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/net/bluetooth/mesh.h>

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @defgroup bt_mesh_op_agg_srv Opcodes Aggregator Server model
 * @ingroup bt_mesh
 * @{
 */

/**
 *
 *  @brief Opcodes Aggretator Server model composition data entry.
 *
 *  @note The Opcodes Aggregator Server handles aggregated messages
 *  and dispatches them to the respective models and their message handlers.
 *  Current implementation assumes that responses are sent from the same
 *  execution context as the received message and doesn't allow to send a
 *  postponed response, e.g. from workqueue.
 */
#define BT_MESH_MODEL_OP_AGG_SRV                                                                   \
BT_MESH_MODEL_CB(BT_MESH_MODEL_ID_OP_AGG_SRV, _bt_mesh_op_agg_srv_op, NULL, NULL,                \
                 &_bt_mesh_op_agg_srv_cb)

/** @cond INTERNAL_HIDDEN */

extern const struct bt_mesh_model_op _bt_mesh_op_agg_srv_op[];
extern const struct bt_mesh_model_cb _bt_mesh_op_agg_srv_cb;

/** @endcond */

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_NET_BLUETOOTH_MESH_OP_AGG_SRV_H */
