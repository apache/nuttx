/****************************************************************************
 * drivers/spi/spi_rpmsg_server.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <sys/param.h>

#include <nuttx/kmalloc.h>
#include <nuttx/rpmsg/rpmsg.h>
#include <nuttx/spi/spi_rpmsg.h>
#include <nuttx/spi/spi_transfer.h>

#include "spi_rpmsg.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct spi_rpmsg_server_s
{
  FAR const char *name;
  FAR struct spi_dev_s *spi;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: spi_rpmsg_lens_ok
 ****************************************************************************/

static bool spi_rpmsg_lens_ok(uint16_t txlen, uint16_t rxlen,
                              size_t wordsize)
{
  return txlen % wordsize == 0 && rxlen % wordsize == 0 &&
         (txlen == 0 || rxlen == 0 || txlen == rxlen);
}

/****************************************************************************
 * Name: spi_rpmsg_exchange_handler
 ****************************************************************************/

static int spi_rpmsg_exchange_handler(FAR struct rpmsg_endpoint *ept,
                                      FAR struct spi_rpmsg_msg_s *msg,
                                      size_t len)
{
  FAR struct spi_rpmsg_server_s *server = ept->priv;
  FAR struct spi_rpmsg_msg_s *rsp;
  struct spi_sequence_s seq;
  struct spi_trans_s trans;
  uint32_t space;
  size_t wordsize;
  size_t nwords;
  int ret;

  /* The request carries txlen TX bytes after the header. */

  if (len != SPI_RPMSG_MSG_SIZE(msg->txlen))
    {
      ret = -EINVAL;
      goto reply_err;
    }

  wordsize = spi_rpmsg_wordsize(msg->nbits);
  if (msg->nbits == 0 || msg->mode > SPIDEV_MODE3 ||
      !spi_rpmsg_lens_ok(msg->txlen, msg->rxlen, wordsize))
    {
      ret = -EINVAL;
      goto reply_err;
    }

  /* Build the reply in a fresh buffer so the RX sink never aliases the TX
   * source and the reply carries only the RX bytes (no echo of TX).
   */

  rsp = rpmsg_get_tx_payload_buffer(ept, &space, true);
  if (rsp == NULL)
    {
      ret = -ENOMEM;
      goto reply_err;
    }

  if (SPI_RPMSG_MSG_SIZE(msg->rxlen) > space)
    {
      rpmsg_release_tx_buffer(ept, rsp);
      ret = -E2BIG;
      goto reply_err;
    }

  /* The word count is the length of whichever buffer is present (both are
   * equal for full-duplex).
   */

  nwords = MAX(msg->txlen, msg->rxlen) / wordsize;

  memset(&trans, 0, sizeof(trans));
  trans.nwords   = nwords;
  trans.txbuffer = msg->txlen != 0 ? msg->data : NULL;
  trans.rxbuffer = msg->rxlen != 0 ? rsp->data : NULL;
#ifdef CONFIG_SPI_HWFEATURES
  trans.hwfeat   = msg->value;
#endif

  memset(&seq, 0, sizeof(seq));
  seq.dev       = msg->devid;
  seq.mode      = msg->mode;
  seq.nbits     = msg->nbits;
  seq.ntrans    = 1;
  seq.frequency = msg->frequency;
#ifdef CONFIG_SPI_DELAY_CONTROL
  seq.a = msg->cs_setup_ns;
  seq.b = msg->cs_delay_hold_ns;
  seq.c = msg->cs_change_delay_inactive_ns;
  seq.i = msg->word_delay_ns;
#endif
  seq.trans = &trans;

  ret = spi_transfer(server->spi, &seq);

  rsp->command = SPI_RPMSG_EXCHANGE;
  rsp->cookie = msg->cookie;
  rsp->rxlen = msg->rxlen;
  rsp->result = ret;
  return rpmsg_send_nocopy(ept, rsp, SPI_RPMSG_MSG_SIZE(msg->rxlen));

reply_err:
  msg->rxlen = 0;
  msg->result = ret;
  return rpmsg_send(ept, msg, SPI_RPMSG_MSG_SIZE(0));
}

/****************************************************************************
 * Name: spi_rpmsg_exchangeseqs_handler
 ****************************************************************************/

static int spi_rpmsg_exchangeseqs_handler(FAR struct rpmsg_endpoint *ept,
                                          FAR struct spi_rpmsg_msg_s *msg,
                                          size_t len)
{
  FAR struct spi_rpmsg_server_s *server = ept->priv;
  struct spi_trans_s trans[SPI_RPMSG_MAXTRANS];
  FAR struct spi_rpmsg_trans_s *rec;
  FAR struct spi_rpmsg_msg_s *rsp;
  FAR const uint8_t *txp;
  FAR uint8_t *rxp;
  struct spi_sequence_s seq;
  uint32_t ntrans = msg->value;
  uint32_t space;
  size_t wordsize;
  size_t txtotal = 0;
  size_t rxtotal = 0;
  uint32_t i;
  int ret = -EINVAL;

  /* ntrans records followed by the concatenated TX bytes */

  wordsize = spi_rpmsg_wordsize(msg->nbits);
  if (msg->nbits == 0 || msg->mode > SPIDEV_MODE3 ||
      ntrans == 0 || ntrans > SPI_RPMSG_MAXTRANS ||
      len < SPI_RPMSG_MSG_SIZE(ntrans * sizeof(*rec)))
    {
      goto reply_err;
    }

  rec = (FAR struct spi_rpmsg_trans_s *)msg->data;
  for (i = 0; i < ntrans; i++)
    {
      if (!spi_rpmsg_lens_ok(rec[i].txlen, rec[i].rxlen, wordsize))
        {
          goto reply_err;
        }

      txtotal += rec[i].txlen;
      rxtotal += rec[i].rxlen;
    }

  if (len != SPI_RPMSG_MSG_SIZE(ntrans * sizeof(*rec) + txtotal))
    {
      goto reply_err;
    }

  /* Fresh reply buffer: RX sinks never alias TX sources, no TX echo */

  rsp = rpmsg_get_tx_payload_buffer(ept, &space, true);
  if (rsp == NULL)
    {
      ret = -ENOMEM;
      goto reply_err;
    }

  if (SPI_RPMSG_MSG_SIZE(rxtotal) > space)
    {
      rpmsg_release_tx_buffer(ept, rsp);
      ret = -E2BIG;
      goto reply_err;
    }

  memset(trans, 0, ntrans * sizeof(trans[0]));

  /* TX consumed from the request, RX produced into the reply, in order */

  txp = msg->data + ntrans * sizeof(*rec);
  rxp = rsp->data;
  for (i = 0; i < ntrans; i++)
    {
      trans[i].deselect = rec[i].deselect != 0;
#ifdef CONFIG_SPI_CMDDATA
      trans[i].cmd      = rec[i].cmd != 0;
#endif
#ifdef CONFIG_SPI_HWFEATURES
      trans[i].hwfeat   = rec[i].hwfeat;
#endif
      trans[i].delay    = rec[i].delay;
      trans[i].nwords   = MAX(rec[i].txlen, rec[i].rxlen) / wordsize;

      if (rec[i].txlen != 0)
        {
          trans[i].txbuffer = txp;
          txp += rec[i].txlen;
        }

      if (rec[i].rxlen != 0)
        {
          trans[i].rxbuffer = rxp;
          rxp += rec[i].rxlen;
        }
    }

  memset(&seq, 0, sizeof(seq));
  seq.dev       = msg->devid;
  seq.mode      = msg->mode;
  seq.nbits     = msg->nbits;
  seq.ntrans    = ntrans;
  seq.frequency = msg->frequency;
#ifdef CONFIG_SPI_DELAY_CONTROL
  seq.a = msg->cs_setup_ns;
  seq.b = msg->cs_delay_hold_ns;
  seq.c = msg->cs_change_delay_inactive_ns;
  seq.i = msg->word_delay_ns;
#endif
  seq.trans = trans;

  ret = spi_transfer(server->spi, &seq);

  rsp->command = SPI_RPMSG_EXCHANGESEQS;
  rsp->cookie = msg->cookie;
  rsp->rxlen = rxtotal;
  rsp->result = ret;
  return rpmsg_send_nocopy(ept, rsp, SPI_RPMSG_MSG_SIZE(rxtotal));

reply_err:
  msg->rxlen = 0;
  msg->result = ret;
  return rpmsg_send(ept, msg, SPI_RPMSG_MSG_SIZE(0));
}

/****************************************************************************
 * Name: spi_rpmsg_server_ept_cb
 ****************************************************************************/

static int spi_rpmsg_server_ept_cb(FAR struct rpmsg_endpoint *ept,
                                   FAR void *data, size_t len,
                                   uint32_t src, FAR void *priv)
{
  FAR struct spi_rpmsg_server_s *server = ept->priv;
  FAR struct spi_rpmsg_msg_s *msg = data;

  if (len < SPI_RPMSG_HDR_SIZE)
    {
      return -EINVAL;
    }

  switch (msg->command)
    {
      case SPI_RPMSG_STATUS:
        msg->value = SPI_STATUS(server->spi, msg->devid);
        msg->result = OK;
        break;

      case SPI_RPMSG_EXCHANGE:
        return spi_rpmsg_exchange_handler(ept, msg, len);

      case SPI_RPMSG_EXCHANGESEQS:
        return spi_rpmsg_exchangeseqs_handler(ept, msg, len);

#ifdef CONFIG_SPI_TRIGGER
      case SPI_RPMSG_TRIGGER:
        msg->result = SPI_TRIGGER(server->spi);
        break;
#endif

      default:
        msg->result = -ENOTTY;
        break;
    }

  return rpmsg_send(ept, msg, len);
}

/****************************************************************************
 * Name: spi_rpmsg_server_match
 ****************************************************************************/

static bool spi_rpmsg_server_match(FAR struct rpmsg_device *rdev,
                                   FAR void *priv, FAR const char *name,
                                   uint32_t dest)
{
  FAR struct spi_rpmsg_server_s *server = priv;
  char eptname[RPMSG_NAME_SIZE];

  snprintf(eptname, sizeof(eptname), SPI_RPMSG_EPT_FORMAT, server->name);
  return strcmp(name, eptname) == 0;
}

/****************************************************************************
 * Name: spi_rpmsg_server_ept_release
 ****************************************************************************/

static void spi_rpmsg_server_ept_release(FAR struct rpmsg_endpoint *ept)
{
  kmm_free(ept);
}

/****************************************************************************
 * Name: spi_rpmsg_server_bind
 ****************************************************************************/

static void spi_rpmsg_server_bind(FAR struct rpmsg_device *rdev,
                                  FAR void *priv, FAR const char *name,
                                  uint32_t dest)
{
  FAR struct rpmsg_endpoint *ept;

  ept = kmm_zalloc(sizeof(*ept));
  if (ept == NULL)
    {
      return;
    }

  ept->priv = priv;
  ept->release_cb = spi_rpmsg_server_ept_release;

  rpmsg_create_ept(ept, rdev, name, RPMSG_ADDR_ANY, RPMSG_ADDR_ANY,
                   spi_rpmsg_server_ept_cb, rpmsg_destroy_ept);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: spi_rpmsg_server_initialize
 *
 * Description:
 *   Export a local SPI lower-half to remote RPMsg SPI clients.
 *
 * Input Parameters:
 *   name - RPMsg SPI service name.
 *   spi  - Local SPI lower-half to export.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

int spi_rpmsg_server_initialize(FAR const char *name,
                                FAR struct spi_dev_s *spi)
{
  FAR struct spi_rpmsg_server_s *server;
  int ret;

  if (name == NULL || spi == NULL)
    {
      return -EINVAL;
    }

  server = kmm_zalloc(sizeof(*server));
  if (server == NULL)
    {
      return -ENOMEM;
    }

  server->name = name;
  server->spi = spi;

  ret = rpmsg_register_callback(server, NULL, NULL,
                                spi_rpmsg_server_match,
                                spi_rpmsg_server_bind);
  if (ret < 0)
    {
      kmm_free(server);
    }

  return ret;
}
