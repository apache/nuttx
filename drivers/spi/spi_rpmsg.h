/****************************************************************************
 * drivers/spi/spi_rpmsg.h
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

#ifndef __DRIVERS_SPI_SPI_RPMSG_H
#define __DRIVERS_SPI_SPI_RPMSG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stddef.h>
#include <stdint.h>

#include <nuttx/compiler.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Private client/server wire protocol, host byte order.  EXCHANGE
 * carries TX bytes, its reply RX bytes.  EXCHANGESEQS carries transfer
 * records plus TX bytes, its reply RX bytes; the server runs the batch
 * as one spi_transfer().  No bus lock command: every message is atomic
 * on the server.
 */

#define SPI_RPMSG_EPT_FORMAT    "rpmsg-spi-%s"

#define SPI_RPMSG_STATUS        0
#define SPI_RPMSG_EXCHANGE      1
#define SPI_RPMSG_EXCHANGESEQS  2
#define SPI_RPMSG_TRIGGER       3

/* Records per EXCHANGESEQS message */

#define SPI_RPMSG_MAXTRANS      16

#define SPI_RPMSG_MSG_SIZE(n) (sizeof(struct spi_rpmsg_msg_s) - 1 + (n))

/* A control command carries only the header up to and including value; an
 * EXCHANGE additionally carries the transfer configuration and data.
 */

#define SPI_RPMSG_HDR_SIZE    offsetof(struct spi_rpmsg_msg_s, frequency)

/****************************************************************************
 * Public Types
 ****************************************************************************/

begin_packed_struct struct spi_rpmsg_msg_s
{
  uint32_t command;
  int32_t result;
  uint64_t cookie;
  uint32_t devid;

  /* STATUS: bits, EXCHANGE: hwfeatures, EXCHANGESEQS: record count */

  uint32_t value;

  /* SPI transfer configuration, used by SPI_RPMSG_EXCHANGE[SEQS] */

  uint32_t frequency;
  uint32_t mode;
#ifdef CONFIG_SPI_DELAY_CONTROL
  uint32_t word_delay_ns;
  uint32_t cs_setup_ns;
  uint32_t cs_delay_hold_ns;
  uint32_t cs_change_delay_inactive_ns;
#endif
  uint16_t txlen;
  uint16_t rxlen;
  uint8_t nbits;
  uint8_t pad[3];      /* Keep data 32-bit aligned for word access */

  /* EXCHANGE: TX bytes, reply RX bytes.  EXCHANGESEQS: records plus TX
   * bytes, reply RX bytes.
   */

  uint8_t data[1];
} end_packed_struct;

/* One EXCHANGESEQS transfer; both lengths zero = de-select/delay only */

begin_packed_struct struct spi_rpmsg_trans_s
{
  uint32_t delay;      /* Microsecond delay after this transfer */
  uint32_t hwfeat;     /* Hardware features for this transfer */
  uint16_t txlen;      /* TX bytes (0 = no TX for this transfer) */
  uint16_t rxlen;      /* RX bytes (0 = no RX for this transfer) */
  uint8_t deselect;    /* De-select after this transfer */
  uint8_t cmd;         /* CMDDATA: true=command; false=data */
  uint8_t pad[2];      /* Keep the record size a multiple of four */
} end_packed_struct;

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

static inline size_t spi_rpmsg_wordsize(uint8_t nbits)
{
  if (nbits > 16)
    {
      return sizeof(uint32_t);
    }

  return nbits > 8 ? sizeof(uint16_t) : sizeof(uint8_t);
}

#endif /* __DRIVERS_SPI_SPI_RPMSG_H */
