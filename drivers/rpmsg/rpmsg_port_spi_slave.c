/****************************************************************************
 * drivers/rpmsg/rpmsg_port_spi_slave.c
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

#include <debug.h>
#include <errno.h>
#include <stdatomic.h>
#include <stdio.h>

#include <nuttx/crc16.h>
#include <nuttx/kmalloc.h>
#include <nuttx/kthread.h>
#include <nuttx/irq.h>
#include <nuttx/mutex.h>

#include "rpmsg_port.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_RPMSG_PORT_SPI_CRC
#  define rpmsg_port_spi_crc16(hdr) crc16((FAR uint8_t *)&(hdr)->cmd, \
                                          (hdr)->len - sizeof((hdr)->crc))
#else
#  define rpmsg_port_spi_crc16(hdr) 0
#endif

#define BYTES2WORDS(s,b)            ((b) / ((s)->nbits >> 3))

/****************************************************************************
 * Private Types
 ****************************************************************************/

enum rpmsg_port_spi_cmd_e
{
  RPMSG_PORT_SPI_CMD_CONNECT = 0x01,
  RPMSG_PORT_SPI_CMD_AVAIL,
  RPMSG_PORT_SPI_CMD_DATA,
};

struct rpmsg_port_spi_s
{
  struct rpmsg_port_s            port;
  FAR struct spi_slave_ctrlr_s   *spictrlr;
  struct spi_slave_dev_s         spislv;
  FAR struct ioexpander_dev_s    *ioe;

  /* GPIOs used for handshake */

  uint8_t                        mreq;
  uint8_t                        sreq;

  /* SPI devices' configuration */

  int                            nbits;

  /* Reserved for cmd send */

  FAR struct rpmsg_port_header_s *cmdhdr;

  /* Used for sync data state between mreq_handler and
   * rpmsg_port_spi_slave_notify
   */

  FAR struct rpmsg_port_header_s *txhdr;
  FAR struct rpmsg_port_header_s *rxhdr;

  rpmsg_port_rx_cb_t             rxcb;
  bool                           connected;

  /* Used for flow control */

  uint16_t                       txavail;
  uint16_t                       rxavail;
  uint16_t                       rxthres;

  atomic_int                     transferring;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void rpmsg_port_spi_notify_tx_ready(FAR struct rpmsg_port_s *port);
static void rpmsg_port_spi_notify_rx_free(FAR struct rpmsg_port_s *port);
static void rpmsg_port_spi_register_cb(FAR struct rpmsg_port_s *port,
                                       rpmsg_port_rx_cb_t callback);
static void rpmsg_port_spi_slave_select(FAR struct spi_slave_dev_s *dev,
                                        bool selected);
static void rpmsg_port_spi_slave_cmddata(FAR struct spi_slave_dev_s *dev,
                                         bool data);
static size_t rpmsg_port_spi_slave_getdata(FAR struct spi_slave_dev_s *dev,
                                           FAR const void **data);
static size_t rpmsg_port_spi_slave_receive(FAR struct spi_slave_dev_s *dev,
                                           FAR const void *data,
                                           size_t nwords);
static void rpmsg_port_spi_slave_notify(FAR struct spi_slave_dev_s *dev,
                                        spi_slave_state_t state);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct rpmsg_port_ops_s g_rpmsg_port_spi_ops =
{
  rpmsg_port_spi_notify_tx_ready,
  rpmsg_port_spi_notify_rx_free,
  rpmsg_port_spi_register_cb,
};

static const struct spi_slave_devops_s g_rpmsg_port_spi_slave_ops =
{
  rpmsg_port_spi_slave_select,             /* select */
  rpmsg_port_spi_slave_cmddata,            /* cmddata */
  rpmsg_port_spi_slave_getdata,            /* getdata */
  rpmsg_port_spi_slave_receive,            /* receive */
  rpmsg_port_spi_slave_notify,             /* notify */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rpmsg_port_spi_drop_packets
 ****************************************************************************/

static void rpmsg_port_spi_drop_packets(FAR struct rpmsg_port_spi_s *rpspi)
{
  FAR struct rpmsg_port_header_s *hdr;

  while (!!(hdr = rpmsg_port_queue_get_buffer(&rpspi->port.txq, false)))
    {
      rpmsg_port_queue_return_buffer(&rpspi->port.txq, hdr);
    }

  while (!!(hdr = rpmsg_port_queue_get_buffer(&rpspi->port.rxq, false)))
    {
      rpmsg_port_queue_return_buffer(&rpspi->port.rxq, hdr);
    }
}

/****************************************************************************
 * Name: rpmsg_port_spi_exchange
 ****************************************************************************/

static void rpmsg_port_spi_exchange(FAR struct rpmsg_port_spi_s *rpspi)
{
  FAR struct rpmsg_port_header_s *txhdr;

  if (atomic_fetch_add(&rpspi->transferring, 1))
    {
      return;
    }

  if (!rpspi->connected)
    {
      txhdr = rpspi->cmdhdr;
      txhdr->cmd = RPMSG_PORT_SPI_CMD_CONNECT;
      strlcpy((FAR char *)(txhdr + 1), rpspi->port.cpuname, RPMSG_NAME_SIZE);
    }
  else if (rpspi->txavail > 0 &&
           rpmsg_port_queue_nused(&rpspi->port.txq) > 0)
    {
      txhdr = rpmsg_port_queue_get_buffer(&rpspi->port.txq, false);
      DEBUGASSERT(txhdr != NULL);

      txhdr->cmd = RPMSG_PORT_SPI_CMD_DATA;
      rpspi->txhdr = txhdr;
    }
  else
    {
      txhdr = rpspi->cmdhdr;
      txhdr->cmd = RPMSG_PORT_SPI_CMD_AVAIL;
    }

  txhdr->avail = rpmsg_port_queue_navail(&rpspi->port.rxq);
  txhdr->avail = txhdr->avail > 1 ? txhdr->avail - 1 : 0;
  txhdr->crc = rpmsg_port_spi_crc16(txhdr);

  rpmsginfo("send cmd:%u avail:%u\n", txhdr->cmd, txhdr->avail);

  SPIS_CTRLR_ENQUEUE(rpspi->spictrlr, txhdr,
                     BYTES2WORDS(rpspi, rpspi->cmdhdr->len));
  IOEXP_WRITEPIN(rpspi->ioe, rpspi->sreq, 1);

  rpspi->rxavail = txhdr->avail;
}

/****************************************************************************
 * Name: rpmsg_port_spi_notify_tx_ready
 ****************************************************************************/

static void rpmsg_port_spi_notify_tx_ready(FAR struct rpmsg_port_s *port)
{
  FAR struct rpmsg_port_spi_s *rpspi =
    container_of(port, struct rpmsg_port_spi_s, port);

  if (rpspi->connected)
    {
      rpmsg_port_spi_exchange(rpspi);
    }
  else
    {
      rpmsg_port_spi_drop_packets(rpspi);
    }
}

/****************************************************************************
 * Name: rpmsg_port_spi_notify_rx_free
 ****************************************************************************/

static void rpmsg_port_spi_notify_rx_free(FAR struct rpmsg_port_s *port)
{
  FAR struct rpmsg_port_spi_s *rpspi =
    container_of(port, struct rpmsg_port_spi_s, port);

  if (rpmsg_port_queue_navail(&port->rxq) - rpspi->rxavail >= rpspi->rxthres)
    {
      rpmsg_port_spi_exchange(rpspi);
    }
}

/****************************************************************************
 * Name: rpmsg_port_spi_register_cb
 ****************************************************************************/

static void rpmsg_port_spi_register_cb(FAR struct rpmsg_port_s *port,
                                       rpmsg_port_rx_cb_t callback)
{
  FAR struct rpmsg_port_spi_s *rpspi =
    container_of(port, struct rpmsg_port_spi_s, port);

  rpspi->rxcb = callback;
}

/****************************************************************************
 * Name: rpmsg_port_spi_slave_select
 ****************************************************************************/

static void rpmsg_port_spi_slave_select(FAR struct spi_slave_dev_s *dev,
                                        bool selected)
{
  rpmsginfo("sdev: %p CS: %s\n", dev, selected ? "select" : "free");
}

/****************************************************************************
 * Name: rpmsg_port_spi_slave_cmddata
 ****************************************************************************/

static void rpmsg_port_spi_slave_cmddata(FAR struct spi_slave_dev_s *dev,
                                         bool data)
{
  rpmsginfo("sdev: %p CMD: %s\n", dev, data ? "data" : "command");
}

/****************************************************************************
 * Name: rpmsg_port_spi_slave_getdata
 ****************************************************************************/

static size_t rpmsg_port_spi_slave_getdata(FAR struct spi_slave_dev_s *dev,
                                           FAR const void **data)
{
  FAR struct rpmsg_port_spi_s *rpspi =
    container_of(dev, struct rpmsg_port_spi_s, spislv);

  *data = rpspi->rxhdr;
  return BYTES2WORDS(rpspi, rpspi->cmdhdr->len);
}

/****************************************************************************
 * Name: rpmsg_port_spi_slave_receive
 ****************************************************************************/

static size_t rpmsg_port_spi_slave_receive(FAR struct spi_slave_dev_s *dev,
                                           FAR const void *data,
                                           size_t nwords)
{
  return nwords;
}

/****************************************************************************
 * Name: rpmsg_port_spi_slave_notify
 ****************************************************************************/

static void rpmsg_port_spi_slave_notify(FAR struct spi_slave_dev_s *dev,
                                        spi_slave_state_t state)
{
  FAR struct rpmsg_port_spi_s *rpspi =
    container_of(dev, struct rpmsg_port_spi_s, spislv);

  IOEXP_WRITEPIN(rpspi->ioe, rpspi->sreq, 0);
  SPIS_CTRLR_QPOLL(rpspi->spictrlr);

  rpmsginfo("received cmd:%u avail:%u\n",
            rpspi->rxhdr->cmd, rpspi->rxhdr->avail);

  if (rpspi->txhdr != NULL)
    {
      rpmsg_port_queue_return_buffer(&rpspi->port.txq, rpspi->txhdr);
      rpspi->txhdr = NULL;
    }

  if (rpspi->rxhdr->crc != 0)
    {
      uint16_t crc = rpmsg_port_spi_crc16(rpspi->rxhdr);

      if (crc != 0 && rpspi->rxhdr->crc != crc)
        {
          rpmsgerr("crc check fail received: %u calculated: %u\n",
                   rpspi->rxhdr->crc, crc);
          goto out;
        }
    }

  /* Skip any data received when connection is not established until a
   * connect req data packet has been received.
   */

  if (!rpspi->connected)
    {
      if (rpspi->rxhdr->cmd != RPMSG_PORT_SPI_CMD_CONNECT)
        {
          goto out;
        }

      rpspi->txavail = rpspi->rxhdr->avail;
      rpspi->connected = true;
    }
  else
    {
      rpspi->txavail = rpspi->rxhdr->avail;
      if (rpspi->rxhdr->cmd == RPMSG_PORT_SPI_CMD_CONNECT)
        {
          rpspi->connected = false;
          rpmsg_port_spi_drop_packets(rpspi);
        }
    }

  if (rpspi->rxhdr->cmd != RPMSG_PORT_SPI_CMD_AVAIL)
    {
      rpmsg_port_queue_add_buffer(&rpspi->port.rxq, rpspi->rxhdr);
      rpspi->rxhdr = rpmsg_port_queue_get_available_buffer(
        &rpspi->port.rxq, false);
      DEBUGASSERT(rpspi->rxhdr != NULL);
    }

out:
  if (atomic_exchange(&rpspi->transferring, 0) > 1 ||
      (rpspi->txavail > 0 && rpmsg_port_queue_nused(&rpspi->port.txq) > 0))
    {
      rpmsg_port_spi_exchange(rpspi);
    }
}

/****************************************************************************
 * Name: rpmsg_port_spi_mreq_handler
 ****************************************************************************/

static int rpmsg_port_spi_mreq_handler(FAR struct ioexpander_dev_s *dev,
                                       ioe_pinset_t pinset, FAR void *arg)
{
  FAR struct rpmsg_port_spi_s *rpspi = arg;

  rpmsginfo("received a mreq\n");
  rpmsg_port_spi_exchange(rpspi);
  return 0;
}

/****************************************************************************
 * Name: rpmsg_port_spi_connect
 ****************************************************************************/

static inline void rpmsg_port_spi_connect(FAR struct rpmsg_port_spi_s *rpspi)
{
  rpmsg_port_spi_exchange(rpspi);
}

/****************************************************************************
 * Name: rpmsg_port_spi_process_packet
 ****************************************************************************/

static void
rpmsg_port_spi_process_packet(FAR struct rpmsg_port_spi_s *rpspi,
                              FAR struct rpmsg_port_header_s *rxhdr)
{
  rpmsginfo("received cmd: %u avail: %u", rxhdr->cmd, rxhdr->avail);

  switch (rxhdr->cmd)
    {
      case RPMSG_PORT_SPI_CMD_CONNECT:
        if (!rpspi->connected)
          {
            rpmsg_port_unregister(&rpspi->port);
            rpmsg_port_spi_connect(rpspi);
          }
        else
          {
            rpmsg_port_register(&rpspi->port, (FAR const char *)(rxhdr + 1));
          }

        rpmsg_port_queue_return_buffer(&rpspi->port.rxq, rxhdr);
        break;

      case RPMSG_PORT_SPI_CMD_DATA:
        rpspi->rxcb(&rpspi->port, rxhdr);
        break;

      default:
        rpmsgerr("received a unexpected frame, dropped\n");
        rpmsg_port_queue_return_buffer(&rpspi->port.rxq, rxhdr);
        break;
    }
}

/****************************************************************************
 * Name: rpmsg_port_spi_thread
 ****************************************************************************/

static int rpmsg_port_spi_thread(int argc, FAR char *argv[])
{
  FAR struct rpmsg_port_spi_s *rpspi =
    (FAR struct rpmsg_port_spi_s *)((uintptr_t)strtoul(argv[2], NULL, 16));
  FAR struct rpmsg_port_queue_s *queue = &rpspi->port.rxq;
  FAR struct rpmsg_port_header_s *rxhdr;

  rpmsg_port_spi_connect(rpspi);
  for (; ; )
    {
      while ((rxhdr = rpmsg_port_queue_get_buffer(queue, true)) != NULL)
        {
          rpmsg_port_spi_process_packet(rpspi, rxhdr);
        }
    }

  return 0;
}

/****************************************************************************
 * Name: rpmsg_port_spi_gpio_init
 ****************************************************************************/

static int
rpmsg_port_spi_init_gpio(FAR struct ioexpander_dev_s *ioe,
                         FAR uint8_t *gpio, uint8_t pin, int invert,
                         ioe_callback_t callback, FAR void *args)
{
  int direction = callback ?
    IOEXPANDER_DIRECTION_IN_PULLDOWN : IOEXPANDER_DIRECTION_OUT;
  int ret;

  ret = IOEXP_SETOPTION(ioe, pin, IOEXPANDER_OPTION_INVERT,
                        (FAR void *)invert);
  if (ret < 0)
    {
      rpmsgerr("gpio set invert option error: %d\n", ret);
      return ret;
    }

  ret = IOEXP_SETDIRECTION(ioe, pin, direction);
  if (ret < 0)
    {
      rpmsgerr("gpio set direction %d error: %d\n", direction, ret);
      return ret;
    }

  if (direction == IOEXPANDER_DIRECTION_IN_PULLDOWN)
    {
      int intcfg = invert == IOEXPANDER_VAL_INVERT ?
        IOEXPANDER_VAL_FALLING : IOEXPANDER_VAL_RISING;
      FAR void *ptr;

      ret = IOEXP_SETOPTION(ioe, pin, IOEXPANDER_OPTION_INTCFG,
                            (FAR void *)intcfg);
      if (ret < 0)
        {
          rpmsgerr("gpio set int option %d error: %d\n", intcfg, ret);
          return ret;
        }

      ptr = IOEP_ATTACH(ioe, pin, callback, args);
      if (ptr == NULL)
        {
          rpmsgerr("gpio attach error: %d\n", ret);
          return -EINVAL;
        }
    }

  *gpio = pin;
  return ret;
}

/****************************************************************************
 * Name: rpmsg_port_spi_init_hardware
 ****************************************************************************/

static int
rpmsg_port_spi_init_hardware(FAR struct rpmsg_port_spi_s *rpspi,
  FAR const struct rpmsg_port_spi_config_s *spicfg,
  FAR struct spi_slave_ctrlr_s *spictrlr, FAR struct ioexpander_dev_s *ioe)
{
  int ret;

  if (spictrlr == NULL || ioe == NULL || spicfg == NULL)
    {
      rpmsgerr("invalid params\n");
      return -EINVAL;
    }

  /* Init mreq gpio */

  ret = rpmsg_port_spi_init_gpio(ioe, &rpspi->mreq, spicfg->mreq_pin,
                                 spicfg->mreq_invert,
                                 rpmsg_port_spi_mreq_handler, rpspi);
  if (ret < 0)
    {
      rpmsgerr("mreq init failed\n");
      return ret;
    }

  /* Init sreq gpio */

  ret = rpmsg_port_spi_init_gpio(ioe, &rpspi->sreq, spicfg->sreq_pin,
                                 spicfg->sreq_invert, NULL, NULL);
  if (ret < 0)
    {
      rpmsgerr("sreq init failed\n");
      return ret;
    }

  rpspi->ioe = ioe;
  rpspi->spictrlr = spictrlr;
  rpspi->spislv.ops = &g_rpmsg_port_spi_slave_ops;
  rpspi->nbits = spicfg->nbits;
  SPIS_CTRLR_BIND(spictrlr, &rpspi->spislv, spicfg->mode, spicfg->nbits);

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rpmsg_port_spi_slave_initialize
 ****************************************************************************/

int
rpmsg_port_spi_slave_initialize(FAR const struct rpmsg_port_config_s *cfg,
  FAR const struct rpmsg_port_spi_config_s *spicfg,
  FAR struct spi_slave_ctrlr_s *spictrlr, FAR struct ioexpander_dev_s *ioe)
{
  FAR struct rpmsg_port_spi_s *rpspi;
  FAR char *argv[3];
  char arg1[32];
  int ret;

  rpspi = kmm_zalloc(sizeof(*rpspi));
  if (rpspi == NULL)
    {
      rpmsgerr("malloc rpmsg spi failed\n");
      return -ENOMEM;
    }

  DEBUGASSERT(cfg->txlen == cfg->rxlen);
  ret = rpmsg_port_initialize(&rpspi->port, cfg, &g_rpmsg_port_spi_ops);
  if (ret < 0)
    {
      rpmsgerr("rpmsg port initialize failed\n");
      goto rpmsg_err;
    }

  /* Always reserve one buffer for sending/receiving cmd packet */

  rpspi->cmdhdr = rpmsg_port_queue_get_available_buffer(
    &rpspi->port.txq, true);
  rpspi->rxhdr = rpmsg_port_queue_get_available_buffer(
    &rpspi->port.rxq, true);
  DEBUGASSERT(rpspi->cmdhdr != NULL && rpspi->rxhdr != NULL);

  rpspi->rxthres = rpmsg_port_queue_navail(&rpspi->port.rxq) *
                   CONFIG_RPMSG_PORT_SPI_RX_THRESHOLD / 100;

  ret = rpmsg_port_spi_init_hardware(rpspi, spicfg, spictrlr, ioe);
  if (ret < 0)
    {
      rpmsgerr("rpmsg port spi hardware init failed\n");
      goto out;
    }

  snprintf(arg1, sizeof(arg1), "%p", rpspi);
  argv[0] = (FAR char *)cfg->remotecpu;
  argv[1] = arg1;
  argv[2] = NULL;
  ret = kthread_create("rpmsg-spi-slv",
                       CONFIG_RPMSG_PORT_SPI_THREAD_PRIORITY,
                       CONFIG_RPMSG_PORT_SPI_THREAD_STACKSIZE,
                       rpmsg_port_spi_thread, argv);
  if (ret < 0)
    {
      rpmsgerr("rpmsg port spi create thread failed\n");
      goto out;
    }

  return 0;

out:
  rpmsg_port_uninitialize(&rpspi->port);
rpmsg_err:
  kmm_free(rpspi);
  return ret;
}
