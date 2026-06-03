/****************************************************************************
 * include/nuttx/virtio/virtio-i2c.h
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

#ifndef __INCLUDE_NUTTX_VIRTIO_VIRTIO_I2C_H
#define __INCLUDE_NUTTX_VIRTIO_VIRTIO_I2C_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/compiler.h>

#include <errno.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define VIRTIO_I2C_FLAGS_FAIL_NEXT       (1 << 0)
#define VIRTIO_I2C_FLAGS_M_RD            (1 << 1)

/* Feature bits */

#define VIRTIO_I2C_F_ZERO_LENGTH_REQUEST 0

#define VIRTIO_I2C_MSG_OK                0
#define VIRTIO_I2C_MSG_ERR               1

/* Max length of the local I2C device node path in the device config space */

#define VHOST_I2C_PATH_MAX               64

#define VIRTIO_I2C_REQ_SIZE(n) \
  (sizeof(struct virtio_i2c_out_hdr_s) + (n) + \
   sizeof(struct virtio_i2c_in_hdr_s))

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* 5.16.6.1 Device Operation: Request Queue */

/* Device-side private config space (the I2C spec defines none): the path of
 * the local I2C device node (e.g. "/dev/i2c0") this device forwards to,
 * filled by the resource table author and read back by the vhost I2C
 * backend, which opens it via file_open().
 */

begin_packed_struct struct vhost_i2c_config_s
{
  char path[VHOST_I2C_PATH_MAX];
} end_packed_struct;

begin_packed_struct struct virtio_i2c_out_hdr_s
{
  uint16_t addr;
  uint16_t padding;
  uint32_t flags;
} end_packed_struct;

begin_packed_struct struct virtio_i2c_in_hdr_s
{
  uint8_t status;
} end_packed_struct;

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/* Shared request-buffer layout helpers and status conversion used by both
 * the VirtIO I2C frontend (drivers/virtio) and the vhost I2C backend
 * (drivers/vhost).  The encode (frontend) and decode (backend) routines are
 * private to each driver and built on these primitives.
 */

/* Return the data buffer inside a VirtIO I2C request buffer. */

static inline FAR uint8_t *virtio_i2c_buf(FAR void *req)
{
  return (FAR uint8_t *)req + sizeof(struct virtio_i2c_out_hdr_s);
}

/* Return the VirtIO I2C input status header after buflen data bytes. */

static inline FAR struct virtio_i2c_in_hdr_s *
virtio_i2c_in_hdr(FAR void *req, size_t buflen)
{
  return (FAR struct virtio_i2c_in_hdr_s *)(virtio_i2c_buf(req) + buflen);
}

/* Convert a VirtIO I2C request status into a NuttX errno. */

static inline int virtio_i2c_result(int status)
{
  return status == VIRTIO_I2C_MSG_OK ? OK : -EIO;
}

#endif /* __INCLUDE_NUTTX_VIRTIO_VIRTIO_I2C_H */
