/****************************************************************************
 * include/nuttx/net/bluetooth/mesh/dfd.h
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

/* Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Header file for the Bluetooth Mesh Firmware Distribution models
 * API.
 * @ingroup bt_mesh_dfd
 */

#ifndef __INCLUDE_NUTTX_NET_BLUETOOTH_MESH_DFD_H
#define __INCLUDE_NUTTX_NET_BLUETOOTH_MESH_DFD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/net/bluetooth/mesh.h>

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @defgroup bt_mesh_dfd Firmware Distribution models
 * @ingroup bt_mesh
 * @{
 */

/** Firmware distribution status. */

enum bt_mesh_dfd_status
{
  /** The message was processed successfully. */

  BT_MESH_DFD_SUCCESS,

  /** Insufficient resources on the node. */

  BT_MESH_DFD_ERR_INSUFFICIENT_RESOURCES,

  /** The operation cannot be performed while the Server is in the current
   *  phase.
   */

  BT_MESH_DFD_ERR_WRONG_PHASE,

  /** An internal error occurred on the node. */

  BT_MESH_DFD_ERR_INTERNAL,

  /** The requested firmware image is not stored on the Distributor. */

  BT_MESH_DFD_ERR_FW_NOT_FOUND,

  /** The AppKey identified by the AppKey Index is not known to the node.
   */

  BT_MESH_DFD_ERR_INVALID_APPKEY_INDEX,

  /** There are no Target nodes in the Distribution Receivers List state. */

  BT_MESH_DFD_ERR_RECEIVERS_LIST_EMPTY,

  /** Another firmware image distribution is in progress. */

  BT_MESH_DFD_ERR_BUSY_WITH_DISTRIBUTION,

  /** Another upload is in progress. */

  BT_MESH_DFD_ERR_BUSY_WITH_UPLOAD,

  /** The URI scheme name indicated by the Update URI is not supported. */

  BT_MESH_DFD_ERR_URI_NOT_SUPPORTED,

  /** The format of the Update URI is invalid. */

  BT_MESH_DFD_ERR_URI_MALFORMED,

  /** The URI is currently unreachable. */

  BT_MESH_DFD_ERR_URI_UNREACHABLE,

  /** The Check Firmware OOB procedure did not find any new firmware. */

  BT_MESH_DFD_ERR_NEW_FW_NOT_AVAILABLE,

  /** The suspension of the Distribute Firmware procedure failed. */

  BT_MESH_DFD_ERR_SUSPEND_FAILED,
};

/** Firmware distribution phases. */

enum bt_mesh_dfd_phase
{
  /** No firmware distribution is in progress. */

  BT_MESH_DFD_PHASE_IDLE,

  /** Firmware distribution is in progress. */

  BT_MESH_DFD_PHASE_TRANSFER_ACTIVE,

  /** The Transfer BLOB procedure has completed successfully. */

  BT_MESH_DFD_PHASE_TRANSFER_SUCCESS,

  /** The Apply Firmware on Target Nodes procedure is being executed. */

  BT_MESH_DFD_PHASE_APPLYING_UPDATE,

  /** The Distribute Firmware procedure has completed successfully. */

  BT_MESH_DFD_PHASE_COMPLETED,

  /** The Distribute Firmware procedure has failed. */

  BT_MESH_DFD_PHASE_FAILED,

  /** The Cancel Firmware Update procedure is being executed. */

  BT_MESH_DFD_PHASE_CANCELING_UPDATE,

  /** The Transfer BLOB procedure is suspended. */

  BT_MESH_DFD_PHASE_TRANSFER_SUSPENDED,
};

/** Firmware upload phases. */

enum bt_mesh_dfd_upload_phase
{
  /** No firmware upload is in progress. */

  BT_MESH_DFD_UPLOAD_PHASE_IDLE,

  /** The Store Firmware procedure is being executed. */

  BT_MESH_DFD_UPLOAD_PHASE_TRANSFER_ACTIVE,

  /** The Store Firmware procedure or Store Firmware OOB procedure failed.
   */

  BT_MESH_DFD_UPLOAD_PHASE_TRANSFER_ERROR,

  /** The Store Firmware procedure or the Store Firmware OOB procedure
   *  completed successfully.
   */

  BT_MESH_DFD_UPLOAD_PHASE_TRANSFER_SUCCESS,
};

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_NET_BLUETOOTH_MESH_DFD_H */
