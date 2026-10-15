/****************************************************************************
 * libs/zblue/mesh/blob_io_flash.c
 *
 * Ported to Apache NuttX by chao an <magicdcps@hotmail.com>.
 *
 ****************************************************************************/

/* Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <errno.h>

#include <nuttx/net/bluetooth/mesh.h>

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int io_open(
  const struct bt_mesh_blob_io *io, const struct bt_mesh_blob_xfer *xfer,
  enum bt_mesh_blob_io_mode mode)
{
  (void)io;
  (void)xfer;
  (void)mode;

  return -ENOTSUP;
}

static void io_close(const struct bt_mesh_blob_io *io,
                     const struct bt_mesh_blob_xfer *xfer)
{
  (void)io;
  (void)xfer;
}

static int block_start(
  const struct bt_mesh_blob_io *io, const struct bt_mesh_blob_xfer *xfer,
  const struct bt_mesh_blob_block *block)
{
  (void)io;
  (void)xfer;
  (void)block;

  return -ENOTSUP;
}

static int rd_chunk(
  const struct bt_mesh_blob_io *io, const struct bt_mesh_blob_xfer *xfer,
  const struct bt_mesh_blob_block *block,
  const struct bt_mesh_blob_chunk *chunk)
{
  (void)io;
  (void)xfer;
  (void)block;
  (void)chunk;

  return -ENOTSUP;
}

static int wr_chunk(
  const struct bt_mesh_blob_io *io, const struct bt_mesh_blob_xfer *xfer,
  const struct bt_mesh_blob_block *block,
  const struct bt_mesh_blob_chunk *chunk)
{
  (void)io;
  (void)xfer;
  (void)block;
  (void)chunk;

  return -ENOTSUP;
}

int bt_mesh_blob_io_flash_init(struct bt_mesh_blob_io_flash *flash,
                               uint8_t area_id, off_t offset)
{
  flash->area_id = area_id;
  flash->offset = offset;
  flash->io.open = io_open;
  flash->io.close = io_close;
  flash->io.block_start = block_start;
  flash->io.block_end = NULL;
  flash->io.rd = rd_chunk;
  flash->io.wr = wr_chunk;

  return 0;
}
