/****************************************************************************
 * include/nuttx/virtio/virtio-spi.h
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

#ifndef __INCLUDE_NUTTX_VIRTIO_VIRTIO_SPI_H
#define __INCLUDE_NUTTX_VIRTIO_VIRTIO_SPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/compiler.h>
#include <nuttx/spi/spi.h>

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define VIRTIO_ID_SPI                   45

#define VIRTIO_SPI_CPHA                 (1 << 0)
#define VIRTIO_SPI_CPOL                 (1 << 1)
#define VIRTIO_SPI_CS_HIGH              (1 << 2)
#define VIRTIO_SPI_MODE_LSB_FIRST       (1 << 3)
#define VIRTIO_SPI_MODE_LOOP            (1 << 4)

#define VIRTIO_SPI_TRANS_OK             0
#define VIRTIO_SPI_PARAM_ERR            1
#define VIRTIO_SPI_TRANS_ERR            2

#define VIRTIO_SPI_RX_TX_SUPPORT_DUAL   (1 << 0)
#define VIRTIO_SPI_RX_TX_SUPPORT_QUAD   (1 << 1)
#define VIRTIO_SPI_RX_TX_SUPPORT_OCTAL  (1 << 2)

#define VIRTIO_SPI_MF_SUPPORT_CPHA_0    (1 << 0)
#define VIRTIO_SPI_MF_SUPPORT_CPHA_1    (1 << 1)
#define VIRTIO_SPI_MF_SUPPORT_CPOL_0    (1 << 2)
#define VIRTIO_SPI_MF_SUPPORT_CPOL_1    (1 << 3)
#define VIRTIO_SPI_MF_SUPPORT_CS_HIGH   (1 << 4)
#define VIRTIO_SPI_MF_SUPPORT_LSB_FIRST (1 << 5)
#define VIRTIO_SPI_MF_SUPPORT_LOOPBACK  (1 << 6)

#define VIRTIO_SPI_SINGLE               1
#define VIRTIO_SPI_DUAL                 2
#define VIRTIO_SPI_QUAD                 4
#define VIRTIO_SPI_OCTAL                8

#define VIRTIO_SPI_REQ_SIZE(txlen, rxlen) \
  (sizeof(struct spi_transfer_head_s) + (txlen) + (rxlen) + \
   sizeof(struct spi_transfer_result_s))

/* Max length of the local SPI device node path in the device config space */

#define VHOST_SPI_PATH_MAX              64

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* 5.21.4 Device configuration layout */

begin_packed_struct struct virtio_spi_config_s
{
  uint8_t cs_max_number;
  uint8_t cs_change_supported;
  uint8_t tx_nbits_supported;
  uint8_t rx_nbits_supported;
  uint32_t bits_per_word_mask;
  uint32_t mode_func_supported;
  uint32_t max_freq_hz;
  uint32_t max_word_delay_ns;
  uint32_t max_cs_setup_ns;
  uint32_t max_cs_hold_ns;
  uint32_t max_cs_inactive_ns;
} end_packed_struct;

/* Device-side private config space extension: the spec config followed by
 * the path of the local SPI device node (e.g. "/dev/spi0") this device
 * forwards to, filled by the resource table author and read back by the
 * vhost SPI backend, which opens it via file_open().
 */

begin_packed_struct struct vhost_spi_config_s
{
  struct virtio_spi_config_s config;
  char path[VHOST_SPI_PATH_MAX];
} end_packed_struct;

/* 5.21.6.1 Device Operation: Request Queue */

begin_packed_struct struct spi_transfer_head_s
{
  uint8_t chip_select_id;
  uint8_t bits_per_word;
  uint8_t cs_change;
  uint8_t tx_nbits;
  uint8_t rx_nbits;
  uint8_t reserved[3];
  uint32_t mode;
  uint32_t freq;
  uint32_t word_delay_ns;
  uint32_t cs_setup_ns;
  uint32_t cs_delay_hold_ns;
  uint32_t cs_change_delay_inactive_ns;
} end_packed_struct;

begin_packed_struct struct spi_transfer_result_s
{
  uint8_t result;
} end_packed_struct;

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/* Shared request-buffer layout helpers and status/mode conversions used by
 * both the VirtIO SPI frontend (drivers/virtio) and the vhost SPI backend
 * (drivers/vhost).  The encode (frontend) and decode (backend) routines are
 * private to each driver and built on these primitives.
 */

/* Container bytes per word for a bits-per-word value. */

static inline size_t virtio_spi_wordsize(uint8_t nbits)
{
  if (nbits > 16)
    {
      return sizeof(uint32_t);
    }

  return nbits > 8 ? sizeof(uint16_t) : sizeof(uint8_t);
}

/* Return true if nbits is a supported VirtIO SPI bus width value. */

static inline bool virtio_spi_bus_width_valid(uint8_t nbits)
{
  return nbits == 0 || nbits == VIRTIO_SPI_SINGLE ||
    nbits == VIRTIO_SPI_DUAL || nbits == VIRTIO_SPI_QUAD ||
    nbits == VIRTIO_SPI_OCTAL;
}

/* Return the transmit buffer inside a VirtIO SPI request buffer. */

static inline FAR uint8_t *virtio_spi_txbuf(FAR void *req)
{
  return (FAR uint8_t *)req + sizeof(struct spi_transfer_head_s);
}

/* Return the receive buffer inside a VirtIO SPI request buffer. */

static inline FAR uint8_t *virtio_spi_rxbuf(FAR void *req, size_t txlen)
{
  return virtio_spi_txbuf(req) + txlen;
}

/* Return the VirtIO SPI result byte inside a request buffer. */

static inline FAR struct spi_transfer_result_s *
virtio_spi_result(FAR void *req, size_t txlen, size_t rxlen)
{
  return (FAR struct spi_transfer_result_s *)
    (virtio_spi_rxbuf(req, txlen) + rxlen);
}

/* Convert a NuttX SPI mode to VirtIO SPI mode flags. */

static inline uint32_t virtio_spi_mode(enum spi_mode_e mode)
{
  uint32_t virtio_mode = 0;

  if (mode == SPIDEV_MODE1 || mode == SPIDEV_MODE3)
    {
      virtio_mode |= VIRTIO_SPI_CPHA;
    }

  if (mode == SPIDEV_MODE2 || mode == SPIDEV_MODE3)
    {
      virtio_mode |= VIRTIO_SPI_CPOL;
    }

  return virtio_mode;
}

/* Convert VirtIO SPI mode flags to a NuttX SPI mode. */

static inline enum spi_mode_e virtio_spi_nuttx_mode(uint32_t mode)
{
  switch (mode & (VIRTIO_SPI_CPHA | VIRTIO_SPI_CPOL))
    {
      case VIRTIO_SPI_CPHA:
        return SPIDEV_MODE1;

      case VIRTIO_SPI_CPOL:
        return SPIDEV_MODE2;

      case VIRTIO_SPI_CPHA | VIRTIO_SPI_CPOL:
        return SPIDEV_MODE3;

      default:
        return SPIDEV_MODE0;
    }
}

/* Convert a VirtIO SPI transfer status to a NuttX errno value. */

static inline int virtio_spi_status_to_errno(uint8_t status)
{
  if (status == VIRTIO_SPI_TRANS_OK)
    {
      return OK;
    }

  if (status == VIRTIO_SPI_PARAM_ERR)
    {
      return -EINVAL;
    }

  return -EIO;
}

/* Convert a NuttX transfer result to a VirtIO SPI status value. */

static inline uint8_t virtio_spi_errno_to_status(int ret)
{
  if (ret >= 0)
    {
      return VIRTIO_SPI_TRANS_OK;
    }

  if (ret == -EINVAL || ret == -ENOTSUP || ret == -E2BIG)
    {
      return VIRTIO_SPI_PARAM_ERR;
    }

  return VIRTIO_SPI_TRANS_ERR;
}

#endif /* __INCLUDE_NUTTX_VIRTIO_VIRTIO_SPI_H */
