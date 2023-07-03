/****************************************************************************
 * drivers/modem/alt1250/altmdm_spi.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>

#include <nuttx/modem/alt1250.h>

#include "altmdm_spi.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void altmdm_spipkt_init(FAR altmdm_spipkt_t *pkt)
{
  pkt->header = 0;
  pkt->buffer = NULL;
  pkt->buff_size = 0;
}

void altmdm_set_spipkt_rxbuffer(FAR altmdm_spipkt_t *pkt, FAR void *buf,
  uint16_t sz)
{
  pkt->buffer = buf;
  pkt->buff_size = sz;
}

void altmdm_set_spipkt_txbuffer(FAR altmdm_spipkt_t *pkt, FAR void *buf,
  uint16_t sz)
{
  uint16_t total_sz;

  if (sz <= ALTSPI_MAX_PKTSIZE)
    {
      sz &= ALTSPI_PKT_SIZEMASK;
      total_sz = (sz + (ALTSPI_PKT_WORDSIZE - 1)) &
        ~(ALTSPI_PKT_WORDSIZE - 1);
      pkt->header = (total_sz << ALTSPI_PKT_TOTALSIZE_POS) + sz;
      pkt->buffer = buf;
      pkt->buff_size = sz;
    }
}

void altmdm_overwrite_body_size(FAR altmdm_spipkt_t *pkt, uint16_t sz)
{
  uint16_t total_sz;

  if (sz <= ALTSPI_MAX_PKTSIZE)
    {
      sz &= ALTSPI_PKT_SIZEMASK;
      total_sz = (sz + (ALTSPI_PKT_WORDSIZE - 1)) &
        ~(ALTSPI_PKT_WORDSIZE - 1);
      pkt->header = (pkt->header & ALTSPI_STATUSMASK) |
        ((total_sz << ALTSPI_PKT_TOTALSIZE_POS) + sz);
    }
}

void altmdm_set_sleeppkt(FAR altmdm_spipkt_t *pkt)
{
  altmdm_set_spipkt_txbuffer(pkt, NULL, 0);
  pkt->header |= ALTSPI_SLEEP_BIT;
}

void altmdm_set_retrypkt(FAR altmdm_spipkt_t *pkt)
{
  pkt->header |= ALTSPI_BUFFFULL_BIT;
}

bool altmdm_is_valid_spipkt_header(FAR altmdm_spipkt_t *pkt)
{
  uint16_t actual_size;
  uint16_t total_size;

  actual_size = pkt_actual_size(pkt);
  total_size = pkt_total_size(pkt);

  return ((actual_size <= ALTSPI_MAX_PKTSIZE) &&
    (actual_size <= pkt->buff_size) &&
    (((actual_size + (ALTSPI_PKT_WORDSIZE - 1)) & ~(ALTSPI_PKT_WORDSIZE - 1))
     == total_size));
}

bool altmdm_is_sleeppkt_ok(FAR altmdm_spipkt_t *pkt)
{
  return (strncmp(pkt->buffer, "OKOK", 4) == 0);
}

void altmdm_do_hdr_transaction(FAR struct spi_dev_s *spidev,
  FAR const struct alt1250_lower_s *lower, FAR altmdm_spipkt_t *tx_pkt,
  FAR altmdm_spipkt_t *rx_pkt)
{
  /* Alt1250 is network endian, so swap tx header value. */

  tx_pkt->header = htonl(tx_pkt->header);

  SPI_EXCHANGE(spidev, &tx_pkt->header, &rx_pkt->header, ALTSPI_PKT_HDRSIZE);

  /* Get correct endian values after exchanged */

  rx_pkt->header = ntohl(rx_pkt->header);
  tx_pkt->header = ntohl(tx_pkt->header);

  m_info("Header TRX done. rx: 0x%08lx, tx: 0x%08lx\n",
    rx_pkt->header, tx_pkt->header);
}

void altmdm_do_body_transaction(FAR struct spi_dev_s *spidev,
  FAR const struct alt1250_lower_s *lower, FAR altmdm_spipkt_t *tx_pkt,
  FAR altmdm_spipkt_t *rx_pkt)
{
  size_t exchange_size;

  /* Larger size is exchange size */

  exchange_size = pkt_total_size(tx_pkt) > pkt_total_size(rx_pkt)
    ? pkt_total_size(tx_pkt) : pkt_total_size(rx_pkt);

  SPI_EXCHANGE(spidev, tx_pkt->buffer, rx_pkt->buffer, exchange_size);

  m_info("Body TRX done. ex size: %d, rx size: %d/%d, tx size: %d/%d\n",
    exchange_size, pkt_actual_size(rx_pkt), pkt_total_size(rx_pkt),
    pkt_actual_size(tx_pkt), pkt_total_size(tx_pkt));
}
