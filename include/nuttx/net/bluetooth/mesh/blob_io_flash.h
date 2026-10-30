/****************************************************************************
 * include/nuttx/net/bluetooth/mesh/blob_io_flash.h
 *
 * Ported to Apache NuttX by chao an <magicdcps@hotmail.com>.
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
