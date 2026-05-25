/****************************************************************************
 * drivers/spi/spi_rpmsg.c
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
#include <stdio.h>
#include <string.h>
#include <sys/param.h>

#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/nuttx.h>
#include <nuttx/rpmsg/rpmsg.h>
#include <nuttx/semaphore.h>
#include <nuttx/spi/spi_rpmsg.h>
#include <nuttx/spi/spi_transfer.h>

#include "spi_rpmsg.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct spi_rpmsg_cookie_s
{
  FAR void *response;
  size_t response_len;
  int result;
  sem_t sem;
};

struct spi_rpmsg_client_s
{
  /* Externally visible lower-half; must be first for container_of() */

  struct spi_dev_s spi;

  /* RPMSG transport to the remote SPI server */

  FAR const char *cpuname;
  FAR const char *name;
  struct rpmsg_endpoint ept;
  sem_t sem;

  /* Serializes local users */

  mutex_t lock;

  /* SPI configuration cached from the ops calls, applied on transfer */

  uint32_t frequency;
  uint32_t devid;
#ifdef CONFIG_SPI_DELAY_CONTROL
  uint32_t word_delay_ns;
  uint32_t cs_setup_ns;
  uint32_t cs_delay_hold_ns;
  uint32_t cs_change_delay_inactive_ns;
#endif
  enum spi_mode_e mode;
  uint8_t nbits;
  bool selected;
#ifdef CONFIG_SPI_HWFEATURES
  spi_hwfeatures_t hwfeatures;
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int spi_rpmsg_lock(FAR struct spi_dev_s *dev, bool lock);
static void spi_rpmsg_select(FAR struct spi_dev_s *dev, uint32_t devid,
                             bool selected);
static uint32_t spi_rpmsg_setfrequency(FAR struct spi_dev_s *dev,
                                       uint32_t frequency);
#ifdef CONFIG_SPI_DELAY_CONTROL
static int spi_rpmsg_setdelay(FAR struct spi_dev_s *dev, uint32_t startdelay,
                              uint32_t stopdelay, uint32_t csdelay,
                              uint32_t ifdelay);
#endif
static void spi_rpmsg_setmode(FAR struct spi_dev_s *dev,
                              enum spi_mode_e mode);
static void spi_rpmsg_setbits(FAR struct spi_dev_s *dev, int nbits);
#ifdef CONFIG_SPI_HWFEATURES
static int spi_rpmsg_hwfeatures(FAR struct spi_dev_s *dev,
                                spi_hwfeatures_t features);
#endif
static uint8_t spi_rpmsg_status(FAR struct spi_dev_s *dev, uint32_t devid);
static uint32_t spi_rpmsg_send(FAR struct spi_dev_s *dev, uint32_t wd);
static int spi_rpmsg_exchange(FAR struct spi_dev_s *dev,
                              FAR const void *txbuffer,
                              FAR void *rxbuffer, size_t nwords);
static int spi_rpmsg_exchangeseqs(FAR struct spi_dev_s *dev,
                                  FAR struct spi_sequence_s *seqs,
                                  size_t nseqs, FAR void **cookie);
static int spi_rpmsg_cancelseqs(FAR struct spi_dev_s *dev,
                                FAR void *cookie);
#ifdef CONFIG_SPI_TRIGGER
static int spi_rpmsg_trigger(FAR struct spi_dev_s *dev);
#endif
static int spi_rpmsg_registercallback(FAR struct spi_dev_s *dev,
                                      spi_mediachange_t callback,
                                      void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct spi_ops_s g_spi_rpmsg_ops =
{
  spi_rpmsg_lock,
  spi_rpmsg_select,
  spi_rpmsg_setfrequency,
#ifdef CONFIG_SPI_DELAY_CONTROL
  spi_rpmsg_setdelay,
#endif
  spi_rpmsg_setmode,
  spi_rpmsg_setbits,
#ifdef CONFIG_SPI_HWFEATURES
  spi_rpmsg_hwfeatures,
#endif
  spi_rpmsg_status,
#ifdef CONFIG_SPI_CMDDATA
  NULL,
#endif
  spi_rpmsg_send,
  spi_rpmsg_exchange,
  spi_rpmsg_exchangeseqs,
  spi_rpmsg_cancelseqs,
#ifdef CONFIG_SPI_TRIGGER
  spi_rpmsg_trigger,
#endif
  spi_rpmsg_registercallback
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: spi_rpmsg_wait_ready
 ****************************************************************************/

static int spi_rpmsg_wait_ready(FAR struct rpmsg_endpoint *ept)
{
  if (!is_rpmsg_ept_ready(ept))
    {
      FAR struct spi_rpmsg_client_s *priv =
        container_of(ept, struct spi_rpmsg_client_s, ept);
      int ret;

      ret = rpmsg_wait(ept, &priv->sem);
      if (ret < 0)
        {
          return ret;
        }

      rpmsg_post(ept, &priv->sem);
    }

  return OK;
}

/****************************************************************************
 * Name: spi_rpmsg_sendrecv
 ****************************************************************************/

static int spi_rpmsg_sendrecv(FAR struct rpmsg_endpoint *ept,
                              FAR struct spi_rpmsg_msg_s *msg,
                              size_t len)
{
  struct spi_rpmsg_cookie_s cookie;
  int ret;

  cookie.response     = msg;
  cookie.response_len = len;
  cookie.result       = -ENXIO;

  ret = spi_rpmsg_wait_ready(ept);
  if (ret < 0)
    {
      return ret;
    }

  nxsem_init(&cookie.sem, 0, 0);

  msg->result = -ENXIO;
  msg->cookie = (uintptr_t)&cookie;

  ret = rpmsg_send(ept, msg, len);
  if (ret >= 0)
    {
      ret = rpmsg_wait(ept, &cookie.sem);
      if (ret >= 0)
        {
          ret = cookie.result;
        }
    }

  nxsem_destroy(&cookie.sem);
  return ret;
}

/****************************************************************************
 * Name: spi_rpmsg_sendrecv_nocopy
 ****************************************************************************/

static int spi_rpmsg_sendrecv_nocopy(FAR struct rpmsg_endpoint *ept,
                                     FAR struct spi_rpmsg_msg_s *msg,
                                     size_t len, FAR void *response,
                                     size_t response_len)
{
  struct spi_rpmsg_cookie_s cookie;
  int ret;

  cookie.response     = response;
  cookie.response_len = response_len;
  cookie.result       = -ENXIO;

  nxsem_init(&cookie.sem, 0, 0);

  msg->result = -ENXIO;
  msg->cookie = (uintptr_t)&cookie;

  ret = rpmsg_send_nocopy(ept, msg, len);
  if (ret >= 0)
    {
      ret = rpmsg_wait(ept, &cookie.sem);
      if (ret >= 0)
        {
          ret = cookie.result;
        }
    }
  else
    {
      rpmsg_release_tx_buffer(ept, msg);
    }

  nxsem_destroy(&cookie.sem);
  return ret;
}

/****************************************************************************
 * Name: spi_rpmsg_transfer
 ****************************************************************************/

static int spi_rpmsg_transfer(FAR struct spi_rpmsg_client_s *priv,
                              FAR const void *txbuffer,
                              FAR void *rxbuffer, size_t txlen,
                              size_t rxlen)
{
  FAR const uint8_t *tx = txbuffer;
  FAR uint8_t *rx = rxbuffer;
  size_t wordsize;
  size_t total;
  size_t offset;
  int ret;

  if (!priv->selected)
    {
      return -ENODEV;
    }

  /* Bytes to move: the larger of the two lengths. */

  wordsize = spi_rpmsg_wordsize(priv->nbits);
  total = MAX(txlen, rxlen);

  /* Split into chunks; CS drops between chunks on the server */

  for (offset = 0; offset < total; )
    {
      FAR struct spi_rpmsg_msg_s *msg;
      uint32_t space;
      size_t maxdata;
      size_t chunk;
      size_t txchunk;
      size_t rxchunk;

      ret = spi_rpmsg_wait_ready(&priv->ept);
      if (ret < 0)
        {
          return ret;
        }

      msg = rpmsg_get_tx_payload_buffer(&priv->ept, &space, true);
      if (msg == NULL)
        {
          return -ENOMEM;
        }

      /* Fit as many whole words as the buffer holds (at least one). */

      if (space <= SPI_RPMSG_MSG_SIZE(0))
        {
          rpmsg_release_tx_buffer(&priv->ept, msg);
          return -E2BIG;
        }

      maxdata = (space - SPI_RPMSG_MSG_SIZE(0)) / wordsize * wordsize;
      if (maxdata == 0)
        {
          rpmsg_release_tx_buffer(&priv->ept, msg);
          return -E2BIG;
        }

      chunk = total - offset;
      if (chunk > maxdata)
        {
          chunk = maxdata;
        }

      txchunk = txlen > 0 ? chunk : 0;
      rxchunk = rxlen > 0 ? chunk : 0;

      msg->command = SPI_RPMSG_EXCHANGE;
      msg->devid = priv->devid;
      msg->frequency = priv->frequency;
      msg->mode = priv->mode;
#ifdef CONFIG_SPI_DELAY_CONTROL
      msg->word_delay_ns = priv->word_delay_ns;
      msg->cs_setup_ns = priv->cs_setup_ns;
      msg->cs_delay_hold_ns = priv->cs_delay_hold_ns;
      msg->cs_change_delay_inactive_ns = priv->cs_change_delay_inactive_ns;
#endif
      msg->txlen = txchunk;
      msg->rxlen = rxchunk;
      msg->nbits = priv->nbits;
      memset(msg->pad, 0, sizeof(msg->pad));
#ifdef CONFIG_SPI_HWFEATURES
      msg->value = priv->hwfeatures;
#endif

      if (txchunk > 0)
        {
          memcpy(msg->data, tx + offset, txchunk);
        }

      ret = spi_rpmsg_sendrecv_nocopy(&priv->ept, msg,
                                      SPI_RPMSG_MSG_SIZE(txchunk),
                                      rxchunk > 0 ? rx + offset : NULL,
                                      rxchunk);
      if (ret < 0)
        {
          return ret;
        }

      offset += chunk;
    }

  return OK;
}

/****************************************************************************
 * Name: spi_rpmsg_lock
 ****************************************************************************/

static int spi_rpmsg_lock(FAR struct spi_dev_s *dev, bool lock)
{
  FAR struct spi_rpmsg_client_s *priv =
    (FAR struct spi_rpmsg_client_s *)dev;

  /* Local only, every message is atomic on the server */

  return lock ? nxmutex_lock(&priv->lock) : nxmutex_unlock(&priv->lock);
}

/****************************************************************************
 * Name: spi_rpmsg_select
 ****************************************************************************/

static void spi_rpmsg_select(FAR struct spi_dev_s *dev, uint32_t devid,
                             bool selected)
{
  FAR struct spi_rpmsg_client_s *priv =
    (FAR struct spi_rpmsg_client_s *)dev;

  priv->devid = devid;
  priv->selected = selected;
}

/****************************************************************************
 * Name: spi_rpmsg_setfrequency
 ****************************************************************************/

static uint32_t spi_rpmsg_setfrequency(FAR struct spi_dev_s *dev,
                                       uint32_t frequency)
{
  FAR struct spi_rpmsg_client_s *priv =
    (FAR struct spi_rpmsg_client_s *)dev;

  priv->frequency = frequency;
  return frequency;
}

/****************************************************************************
 * Name: spi_rpmsg_setdelay
 ****************************************************************************/

#ifdef CONFIG_SPI_DELAY_CONTROL
static int spi_rpmsg_setdelay(FAR struct spi_dev_s *dev, uint32_t startdelay,
                              uint32_t stopdelay, uint32_t csdelay,
                              uint32_t ifdelay)
{
  FAR struct spi_rpmsg_client_s *priv =
    (FAR struct spi_rpmsg_client_s *)dev;

  priv->cs_setup_ns = startdelay;
  priv->cs_delay_hold_ns = stopdelay;
  priv->cs_change_delay_inactive_ns = csdelay;
  priv->word_delay_ns = ifdelay;
  return OK;
}
#endif

/****************************************************************************
 * Name: spi_rpmsg_setmode
 ****************************************************************************/

static void spi_rpmsg_setmode(FAR struct spi_dev_s *dev,
                              enum spi_mode_e mode)
{
  FAR struct spi_rpmsg_client_s *priv =
    (FAR struct spi_rpmsg_client_s *)dev;

  if (mode <= SPIDEV_MODE3)
    {
      priv->mode = mode;
    }
}

/****************************************************************************
 * Name: spi_rpmsg_setbits
 ****************************************************************************/

static void spi_rpmsg_setbits(FAR struct spi_dev_s *dev, int nbits)
{
  FAR struct spi_rpmsg_client_s *priv =
    (FAR struct spi_rpmsg_client_s *)dev;

  if (nbits > 0 && nbits <= 32)
    {
      priv->nbits = nbits;
    }
}

/****************************************************************************
 * Name: spi_rpmsg_hwfeatures
 ****************************************************************************/

#ifdef CONFIG_SPI_HWFEATURES
static int spi_rpmsg_hwfeatures(FAR struct spi_dev_s *dev,
                                spi_hwfeatures_t features)
{
  FAR struct spi_rpmsg_client_s *priv =
    (FAR struct spi_rpmsg_client_s *)dev;

  priv->hwfeatures = features;
  return OK;
}
#endif

/****************************************************************************
 * Name: spi_rpmsg_status
 ****************************************************************************/

static uint8_t spi_rpmsg_status(FAR struct spi_dev_s *dev, uint32_t devid)
{
  FAR struct spi_rpmsg_client_s *priv =
    (FAR struct spi_rpmsg_client_s *)dev;
  struct spi_rpmsg_msg_s msg;

  msg.command = SPI_RPMSG_STATUS;
  msg.devid = devid;
  msg.value = 0;

  if (spi_rpmsg_sendrecv(&priv->ept, &msg, SPI_RPMSG_HDR_SIZE) < 0)
    {
      return 0;
    }

  return msg.value;
}

/****************************************************************************
 * Name: spi_rpmsg_send
 ****************************************************************************/

static uint32_t spi_rpmsg_send(FAR struct spi_dev_s *dev, uint32_t wd)
{
  FAR struct spi_rpmsg_client_s *priv =
    (FAR struct spi_rpmsg_client_s *)dev;
  uint8_t tx8;
  uint8_t rx8 = 0;
  uint16_t tx16;
  uint16_t rx16 = 0;
  uint32_t tx32;
  uint32_t rx32 = 0;
  size_t wordsize;
  int ret;

  wordsize = spi_rpmsg_wordsize(priv->nbits);
  if (wordsize == sizeof(uint32_t))
    {
      tx32 = wd;
      ret = spi_rpmsg_transfer(priv, &tx32, &rx32, wordsize, wordsize);
      return ret < 0 ? 0 : rx32;
    }
  else if (wordsize == sizeof(uint16_t))
    {
      tx16 = wd;
      ret = spi_rpmsg_transfer(priv, &tx16, &rx16, wordsize, wordsize);
      return ret < 0 ? 0 : rx16;
    }

  tx8 = wd;
  ret = spi_rpmsg_transfer(priv, &tx8, &rx8, wordsize, wordsize);
  return ret < 0 ? 0 : rx8;
}

/****************************************************************************
 * Name: spi_rpmsg_exchange
 ****************************************************************************/

static int spi_rpmsg_exchange(FAR struct spi_dev_s *dev,
                              FAR const void *txbuffer,
                              FAR void *rxbuffer, size_t nwords)
{
  FAR struct spi_rpmsg_client_s *priv =
    (FAR struct spi_rpmsg_client_s *)dev;
  size_t nbytes;
  size_t wordsize;

  /* No length ceiling: spi_rpmsg_transfer chunks the payload. */

  wordsize = spi_rpmsg_wordsize(priv->nbits);
  nbytes = nwords * wordsize;
  return spi_rpmsg_transfer(priv, txbuffer, rxbuffer,
                            txbuffer != NULL ? nbytes : 0,
                            rxbuffer != NULL ? nbytes : 0);
}

/****************************************************************************
 * Name: spi_rpmsg_sendseq
 ****************************************************************************/

static int spi_rpmsg_sendseq(FAR struct spi_rpmsg_client_s *priv,
                             FAR struct spi_sequence_s *seq)
{
  FAR struct spi_rpmsg_trans_s *rec;
  FAR struct spi_rpmsg_msg_s *msg;
  FAR uint8_t *payload;
  size_t wordsize;
  size_t txtotal = 0;
  size_t rxtotal = 0;
  uint32_t space;
  int ret;
  int i;

  if (seq->ntrans == 0)
    {
      return -EINVAL;
    }

  wordsize = spi_rpmsg_wordsize(seq->nbits);
  for (i = 0; i < seq->ntrans; i++)
    {
      size_t nbytes = seq->trans[i].nwords * wordsize;

      if (seq->trans[i].txbuffer != NULL)
        {
          txtotal += nbytes;
        }

      if (seq->trans[i].rxbuffer != NULL)
        {
          rxtotal += nbytes;
        }
    }

  /* Request and reply must each fit into one message */

  if (seq->ntrans > SPI_RPMSG_MAXTRANS ||
      txtotal > UINT16_MAX || rxtotal > UINT16_MAX)
    {
      return -ENOTSUP;
    }

  ret = spi_rpmsg_wait_ready(&priv->ept);
  if (ret < 0)
    {
      return ret;
    }

  msg = rpmsg_get_tx_payload_buffer(&priv->ept, &space, true);
  if (msg == NULL)
    {
      return -ENOMEM;
    }

  if (SPI_RPMSG_MSG_SIZE(seq->ntrans * sizeof(*rec) + txtotal) > space)
    {
      rpmsg_release_tx_buffer(&priv->ept, msg);
      return -ENOTSUP;
    }

  /* One record per transfer, then the TX bytes in record order */

  rec = (FAR struct spi_rpmsg_trans_s *)msg->data;
  payload = msg->data + seq->ntrans * sizeof(*rec);
  memset(rec, 0, seq->ntrans * sizeof(*rec));

  for (i = 0; i < seq->ntrans; i++)
    {
      FAR struct spi_trans_s *trans = &seq->trans[i];
      size_t nbytes = trans->nwords * wordsize;

      rec[i].delay    = trans->delay;
#ifdef CONFIG_SPI_HWFEATURES
      rec[i].hwfeat   = trans->hwfeat;
#endif
      rec[i].deselect = trans->deselect;
#ifdef CONFIG_SPI_CMDDATA
      rec[i].cmd      = trans->cmd;
#endif

      if (trans->txbuffer != NULL)
        {
          rec[i].txlen = nbytes;
          memcpy(payload, trans->txbuffer, nbytes);
          payload += nbytes;
        }

      if (trans->rxbuffer != NULL)
        {
          rec[i].rxlen = nbytes;
        }
    }

  msg->command   = SPI_RPMSG_EXCHANGESEQS;
  msg->devid     = seq->dev;
  msg->value     = seq->ntrans;
  msg->frequency = seq->frequency;
  msg->mode      = seq->mode;
#ifdef CONFIG_SPI_DELAY_CONTROL
  msg->cs_setup_ns = seq->a;
  msg->cs_delay_hold_ns = seq->b;
  msg->cs_change_delay_inactive_ns = seq->c;
  msg->word_delay_ns = seq->i;
#endif
  msg->txlen     = txtotal;
  msg->rxlen     = rxtotal;
  msg->nbits     = seq->nbits;
  memset(msg->pad, 0, sizeof(msg->pad));

  return spi_rpmsg_sendrecv_nocopy(&priv->ept, msg,
            SPI_RPMSG_MSG_SIZE(seq->ntrans * sizeof(*rec) + txtotal),
                                   seq, rxtotal);
}

/****************************************************************************
 * Name: spi_rpmsg_exchangeseqs
 ****************************************************************************/

static int spi_rpmsg_exchangeseqs(FAR struct spi_dev_s *dev,
                                  FAR struct spi_sequence_s *seqs,
                                  size_t nseqs, FAR void **cookie)
{
  FAR struct spi_rpmsg_client_s *priv =
    (FAR struct spi_rpmsg_client_s *)dev;
  int ret;

  if (seqs == NULL || nseqs == 0)
    {
      return -EINVAL;
    }

  /* -ENOTSUP: the caller's generic loop handles it, one sequence at
   * a time
   */

  if (nseqs > 1)
    {
      return -ENOTSUP;
    }

  ret = spi_rpmsg_sendseq(priv, seqs);
  if (ret < 0)
    {
      return ret;
    }

  if (cookie != NULL)
    {
      *cookie = NULL;
    }

  return OK;
}

/****************************************************************************
 * Name: spi_rpmsg_cancelseqs
 ****************************************************************************/

static int spi_rpmsg_cancelseqs(FAR struct spi_dev_s *dev, FAR void *cookie)
{
  /* Synchronous: nothing is ever pending */

  return cookie == NULL ? OK : -ENOENT;
}

#ifdef CONFIG_SPI_TRIGGER

/****************************************************************************
 * Name: spi_rpmsg_trigger
 ****************************************************************************/

static int spi_rpmsg_trigger(FAR struct spi_dev_s *dev)
{
  FAR struct spi_rpmsg_client_s *priv =
    (FAR struct spi_rpmsg_client_s *)dev;
  struct spi_rpmsg_msg_s msg;

  msg.command = SPI_RPMSG_TRIGGER;
  msg.devid = 0;
  msg.value = 0;

  return spi_rpmsg_sendrecv(&priv->ept, &msg, SPI_RPMSG_HDR_SIZE);
}
#endif

/****************************************************************************
 * Name: spi_rpmsg_registercallback
 ****************************************************************************/

static int spi_rpmsg_registercallback(FAR struct spi_dev_s *dev,
                                      spi_mediachange_t callback,
                                      void *arg)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: spi_rpmsg_scatter_rx
 ****************************************************************************/

static void spi_rpmsg_scatter_rx(FAR struct spi_sequence_s *seq,
                                 FAR const uint8_t *rx)
{
  size_t wordsize = spi_rpmsg_wordsize(seq->nbits);
  int i;

  for (i = 0; i < seq->ntrans; i++)
    {
      if (seq->trans[i].rxbuffer != NULL)
        {
          size_t nbytes = seq->trans[i].nwords * wordsize;

          memcpy(seq->trans[i].rxbuffer, rx, nbytes);
          rx += nbytes;
        }
    }
}

/****************************************************************************
 * Name: spi_rpmsg_client_ept_cb
 ****************************************************************************/

static int spi_rpmsg_client_ept_cb(FAR struct rpmsg_endpoint *ept,
                                   FAR void *data, size_t len,
                                   uint32_t src, FAR void *priv)
{
  FAR struct spi_rpmsg_msg_s *msg = data;
  FAR struct spi_rpmsg_cookie_s *cookie =
    (FAR struct spi_rpmsg_cookie_s *)(uintptr_t)msg->cookie;

  if (cookie != NULL)
    {
      if (msg->command == SPI_RPMSG_EXCHANGE)
        {
          /* The reply carries rxlen RX bytes after the header and the
           * transfer result in msg->result.
           */

          if (len != SPI_RPMSG_MSG_SIZE(msg->rxlen))
            {
              msg->result = -EINVAL;
            }
          else if (cookie->response != NULL)
            {
              memcpy(cookie->response, msg->data,
                     MIN(msg->rxlen, cookie->response_len));
            }
        }
      else if (msg->command == SPI_RPMSG_EXCHANGESEQS)
        {
          if (msg->result >= 0)
            {
              if (len != SPI_RPMSG_MSG_SIZE(cookie->response_len))
                {
                  msg->result = -EINVAL;
                }
              else
                {
                  spi_rpmsg_scatter_rx(cookie->response, msg->data);
                }
            }
        }
      else if (cookie->response != NULL)
        {
          memcpy(cookie->response, data, MIN(len, cookie->response_len));
        }

      cookie->result = msg->result;
      rpmsg_post(ept, &cookie->sem);
    }

  return OK;
}

/****************************************************************************
 * Name: spi_rpmsg_client_created
 ****************************************************************************/

static void spi_rpmsg_client_created(FAR struct rpmsg_device *rdev,
                                     FAR void *priv)
{
  FAR struct spi_rpmsg_client_s *client = priv;

  if (strcmp(client->cpuname, rpmsg_get_cpuname(rdev)) == 0)
    {
      char eptname[RPMSG_NAME_SIZE];

      snprintf(eptname, sizeof(eptname), SPI_RPMSG_EPT_FORMAT,
               client->name);

      client->ept.priv = client;
      rpmsg_create_ept(&client->ept, rdev, eptname, RPMSG_ADDR_ANY,
                       RPMSG_ADDR_ANY, spi_rpmsg_client_ept_cb, NULL);
      rpmsg_post(&client->ept, &client->sem);
    }
}

/****************************************************************************
 * Name: spi_rpmsg_client_destroy
 ****************************************************************************/

static void spi_rpmsg_client_destroy(FAR struct rpmsg_device *rdev,
                                     FAR void *priv)
{
  FAR struct spi_rpmsg_client_s *client = priv;

  if (strcmp(client->cpuname, rpmsg_get_cpuname(rdev)) == 0)
    {
      rpmsg_wait(&client->ept, &client->sem);
      rpmsg_destroy_ept(&client->ept);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: spi_rpmsg_initialize
 *
 * Description:
 *   Create an SPI client lower-half that forwards transfers to a remote
 *   RPMsg SPI server.
 *
 * Input Parameters:
 *   cpuname - Name of the remote CPU to connect to.
 *   name    - RPMsg SPI service name.
 *
 * Returned Value:
 *   A pointer to the SPI lower-half on success; NULL on failure.
 *
 ****************************************************************************/

FAR struct spi_dev_s *spi_rpmsg_initialize(FAR const char *cpuname,
                                           FAR const char *name)
{
  FAR struct spi_rpmsg_client_s *client;
  int ret;

  if (cpuname == NULL || name == NULL)
    {
      return NULL;
    }

  client = kmm_zalloc(sizeof(*client));
  if (client == NULL)
    {
      return NULL;
    }

  client->spi.ops = &g_spi_rpmsg_ops;
  client->cpuname = cpuname;
  client->name = name;
  client->nbits = 8;
  client->mode = SPIDEV_MODE0;

  nxsem_init(&client->sem, 0, 0);
  nxmutex_init(&client->lock);

  ret = rpmsg_register_callback(client, spi_rpmsg_client_created,
                                spi_rpmsg_client_destroy, NULL, NULL);
  if (ret < 0)
    {
      nxmutex_destroy(&client->lock);
      nxsem_destroy(&client->sem);
      kmm_free(client);
      return NULL;
    }

  return &client->spi;
}
