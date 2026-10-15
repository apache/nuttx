/****************************************************************************
 * include/nuttx/net/bluetooth/mesh/blob_io_flash.h
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
 * @brief Header file for the Bluetooth Mesh BLOB flash stream API.
 * @ingroup bt_mesh_blob_io_flash
 */

#ifndef __INCLUDE_NUTTX_NET_BLUETOOTH_MESH_BLOB_IO_FLASH_H
#define __INCLUDE_NUTTX_NET_BLUETOOTH_MESH_BLOB_IO_FLASH_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @defgroup bt_mesh_blob_io_flash Bluetooth Mesh BLOB flash stream
 * @ingroup bt_mesh
 * @{
 */

/** BLOB flash stream. */

struct bt_mesh_blob_io_flash
{
  /** Flash stream area ID. */

  uint8_t area_id;

  /** Active stream mode. */

  enum bt_mesh_blob_io_mode mode;

  /** Offset into the stream area, in bytes. */

  off_t offset;

  /* BLOB stream. */

  struct bt_mesh_blob_io io;
};

/** @brief Initialize a flash stream.
 *
 *  @param flash   Flash stream.
 *  @param area_id Stream area identifier.
 *  @param offset  Offset into the stream area, in bytes.
 *
 *  @return 0 on success or (negative) error code otherwise.
 */

int bt_mesh_blob_io_flash_init(struct bt_mesh_blob_io_flash *flash,
                               uint8_t area_id,
                               off_t offset);

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_NET_BLUETOOTH_MESH_BLOB_IO_FLASH_H */
