/****************************************************************************
 * net/bt/mesh/dfd_srv_internal.h
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

#ifndef ZEPHYR_INCLUDE_BLUETOOTH_MESH_DFD_SRV_INTERNAL_H__
#define ZEPHYR_INCLUDE_BLUETOOTH_MESH_DFD_SRV_INTERNAL_H__

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/net/bluetooth/mesh.h>

#ifdef __cplusplus
extern "C"
{
#endif

struct bt_mesh_dfd_start_params
{
  uint16_t app_idx;
  uint16_t timeout_base;
  uint16_t slot_idx;
  uint16_t group;
  enum bt_mesh_blob_xfer_mode xfer_mode;
  uint8_t ttl;
  bool apply;
};

enum bt_mesh_dfd_status bt_mesh_dfd_srv_receiver_add(
  struct bt_mesh_dfd_srv *srv, uint16_t addr, uint8_t img_idx);

enum bt_mesh_dfd_status bt_mesh_dfd_srv_receivers_delete_all(
  struct bt_mesh_dfd_srv *srv);

enum bt_mesh_dfd_status bt_mesh_dfd_srv_start(struct bt_mesh_dfd_srv *srv,
  struct bt_mesh_dfd_start_params *params);

enum bt_mesh_dfd_status bt_mesh_dfd_srv_suspend(struct bt_mesh_dfd_srv *srv);

enum bt_mesh_dfd_status bt_mesh_dfd_srv_cancel(struct bt_mesh_dfd_srv *srv,
                                               struct bt_mesh_msg_ctx *ctx);

enum bt_mesh_dfd_status bt_mesh_dfd_srv_apply(struct bt_mesh_dfd_srv *srv);

enum bt_mesh_dfd_status bt_mesh_dfd_srv_fw_delete(
  struct bt_mesh_dfd_srv *srv, size_t *fwid_len, const uint8_t **fwid);

enum bt_mesh_dfd_status bt_mesh_dfd_srv_fw_delete_all(
  struct bt_mesh_dfd_srv *srv);

/** Notify a co-located Firmware Distribution Server that the local Firmware
 *  Update Server completed a deferred self-apply.
 */

void bt_mesh_dfd_srv_self_applied(void);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_BLUETOOTH_MESH_DFD_SRV_INTERNAL_H__ */
