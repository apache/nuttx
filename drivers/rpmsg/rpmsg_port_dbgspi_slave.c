/****************************************************************************
 * drivers/rpmsg/rpmsg_port_dbgspi_slave.c
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
#include <stdio.h>

#include <nuttx/atomic.h>
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
#define WORDS2BYTES(s,w)            ((w) * ((s)->nbits >> 3))

/****************************************************************************
 * Private Types
 ****************************************************************************/

enum rpmsg_port_spi_cmd_e
{
  RPMSG_PORT_SPI_CMD_CONNECT = 0x01,
  RPMSG_PORT_SPI_CMD_AVAIL,
  RPMSG_PORT_SPI_CMD_DATA,
};

enum rpmsg_port_spi_status_e
{
  RPMSG_PORT_SPI_STATUS_UNCONNECTED,
  RPMSG_PORT_SPI_STATUS_PRECONNECT,
  RPMSG_PORT_SPI_STATUS_CONNECTED,
};

struct rpmsg_port_spi_s
{
  struct rpmsg_port_s            port;
  FAR struct spi_slave_ctrlr_s   *spictrlr;
  struct spi_slave_dev_s         spislv;
  FAR struct ioexpander_dev_s    *rioe;
  FAR struct ioexpander_dev_s    *pioe;

  /* GPIOs used for handshake */

  uint8_t                        mrdy;
  uint8_t                        sreq;

  uint8_t                        mpwr;
  uint8_t                        spwr;

  /* SPI devices' configuration */

  int                            nbits;

  /* Reserved for cmd send */

  FAR struct rpmsg_port_header_s *cmdhdr;

  /* Used for sync data state between mrdy_handler and
   * rpmsg_port_spi_slave_notify
   */

  FAR struct rpmsg_port_header_s *txhdr;
  FAR struct rpmsg_port_header_s *rxhdr;
  FAR struct rpmsg_port_header_s *sndhdr;

  rpmsg_port_rx_cb_t             rxcb;
  uint16_t                       status;

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
 * Name: rpmsg_port_spi_send
 ****************************************************************************/

static void rpmsg_port_spi_send(FAR struct rpmsg_port_spi_s *rpspi)
{
  FAR struct rpmsg_port_header_s *txhdr;

  /* Send and Recv process are separated, transferring is only set at the
   * beginning of the send process, and is cleared at the end of it.
   */

  if (atomic_fetch_add(&rpspi->transferring, 1))
    {
      return;
    }

  if (rpspi->status != RPMSG_PORT_SPI_STATUS_CONNECTED)
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

  rpspi->rxavail = rpmsg_port_queue_navail(&rpspi->port.rxq);
  txhdr->avail = rpspi->rxavail > 1 ? rpspi->rxavail - 1 : 0;
  txhdr->crc = rpmsg_port_spi_crc16(txhdr);
  rpspi->sndhdr = txhdr;

  rpmsginfo("send cmd:%u avail:%u\n", txhdr->cmd, txhdr->avail);

  SPIS_CTRLR_ENQUEUE(rpspi->spictrlr, txhdr,
                     BYTES2WORDS(rpspi, rpspi->cmdhdr->len));
  IOEXP_WRITEPIN(rpspi->rioe, rpspi->sreq, 1);
}

/****************************************************************************
 * Name: rpmsg_port_spi_notify_tx_ready
 ****************************************************************************/

static void rpmsg_port_spi_notify_tx_ready(FAR struct rpmsg_port_s *port)
{
  FAR struct rpmsg_port_spi_s *rpspi =
    container_of(port, struct rpmsg_port_spi_s, port);

  if (rpspi->status == RPMSG_PORT_SPI_STATUS_CONNECTED)
    {
      rpmsg_port_spi_send(rpspi);
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
      rpmsg_port_spi_send(rpspi);
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
  FAR struct rpmsg_port_spi_s *rpspi =
    container_of(dev, struct rpmsg_port_spi_s, spislv);

  memcpy(rpspi->rxhdr, data, rpspi->cmdhdr->len);
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

  if (state == SPISLAVE_TX_COMPLETE)
    {
      IOEXP_WRITEPIN(rpspi->rioe, rpspi->sreq, 0);
      if (rpspi->status == RPMSG_PORT_SPI_STATUS_PRECONNECT)
        {
          rpspi->status = RPMSG_PORT_SPI_STATUS_CONNECTED;
          rpmsg_port_queue_add_buffer(&rpspi->port.rxq, rpspi->rxhdr);
          rpspi->rxhdr = rpmsg_port_queue_get_available_buffer(
            &rpspi->port.rxq, false);
          DEBUGASSERT(rpspi->rxhdr != NULL);
        }
      else if (rpspi->txhdr != NULL)
        {
          rpmsg_port_queue_return_buffer(&rpspi->port.txq, rpspi->txhdr);
          rpspi->txhdr = NULL;
          rpspi->txavail -= 1;
        }

      atomic_exchange(&rpspi->transferring, 0);
      goto out;
    }

  /* Get the received data from driver and notify peer current frame has
   * been received by triggerring mrdy gpio.
   */

  SPIS_CTRLR_QPOLL(rpspi->spictrlr);
  IOEXP_WRITEPIN(rpspi->rioe, rpspi->mrdy, 1);

  rpmsginfo("received cmd:%u avail:%u\n",
    rpspi->rxhdr->cmd, rpspi->rxhdr->avail);

  if (rpspi->rxhdr->crc != 0)
    {
      uint16_t crc = rpmsg_port_spi_crc16(rpspi->rxhdr);

      if (crc != 0 && rpspi->rxhdr->crc != crc)
        {
          rpmsgerr("crc check fail received: %u calculated: %u\n",
                   rpspi->rxhdr->crc, crc);
          goto out_recv;
        }
    }

  /* Skip any data received when connection is not established until a
   * connect req data packet has been received.
   */

  if (rpspi->status != RPMSG_PORT_SPI_STATUS_CONNECTED)
    {
      if (rpspi->rxhdr->cmd != RPMSG_PORT_SPI_CMD_CONNECT)
        {
          goto out_recv;
        }

      rpspi->txavail = rpspi->rxhdr->avail;
      rpspi->status = RPMSG_PORT_SPI_STATUS_PRECONNECT;
      goto out_recv;
    }
  else
    {
      rpspi->txavail = rpspi->rxhdr->avail;
      if (rpspi->rxhdr->cmd == RPMSG_PORT_SPI_CMD_CONNECT)
        {
          /* Use spwr interrupt handler to reinit all the state, it
           * should not receive a connect req during connected state.
           */

          DEBUGASSERT(0);
        }
    }

  if (rpspi->rxhdr->cmd != RPMSG_PORT_SPI_CMD_AVAIL)
    {
      if (rpspi->rxhdr->cmd == RPMSG_PORT_SPI_CMD_DATA)
        {
          rpspi->rxavail -= 1;
        }

      rpmsg_port_queue_add_buffer(&rpspi->port.rxq, rpspi->rxhdr);
      rpspi->rxhdr = rpmsg_port_queue_get_available_buffer(
        &rpspi->port.rxq, false);
      DEBUGASSERT(rpspi->rxhdr != NULL);

      if (atomic_load(&rpspi->transferring))
        {
          rpspi->rxavail = rpmsg_port_queue_navail(&rpspi->port.rxq);
          rpspi->sndhdr->avail =
            rpspi->rxavail > 1 ? rpspi->rxavail - 1 : 0;
          SPIS_CTRLR_ENQUEUE(rpspi->spictrlr, rpspi->sndhdr,
                             BYTES2WORDS(rpspi, rpspi->cmdhdr->len));
        }
    }

out_recv:
  IOEXP_WRITEPIN(rpspi->rioe, rpspi->mrdy, 0);
out:
  if (rpspi->status == RPMSG_PORT_SPI_STATUS_PRECONNECT ||
      (rpspi->status == RPMSG_PORT_SPI_STATUS_CONNECTED &&
       ((rpspi->txavail > 0 &&
         rpmsg_port_queue_nused(&rpspi->port.txq) > 0) ||
        (rpmsg_port_queue_navail(&rpspi->port.rxq) -
         rpspi->rxavail >= rpspi->rxthres))))
    {
      rpmsg_port_spi_send(rpspi);
    }
}

/****************************************************************************
 * Name: rpmsg_port_spi_mpwr_handler
 ****************************************************************************/

static int rpmsg_port_spi_mpwr_handler(FAR struct ioexpander_dev_s *dev,
                                       ioe_pinset_t pinset, FAR void *arg)
{
  FAR struct rpmsg_port_spi_s *rpspi = arg;

  rpmsginfo("received a mpwr\n");

  /* Master reboots, and we need to restart the handshake */

  rpspi->status = RPMSG_PORT_SPI_STATUS_UNCONNECTED;
  rpmsg_port_spi_drop_packets(rpspi);

  rpspi->rxhdr->cmd = RPMSG_PORT_SPI_CMD_CONNECT;
  rpmsg_port_queue_add_buffer(&rpspi->port.rxq, rpspi->rxhdr);
  rpspi->rxhdr = rpmsg_port_queue_get_available_buffer(
    &rpspi->port.rxq, false);
  DEBUGASSERT(rpspi->rxhdr != NULL);

  IOEXP_WRITEPIN(rpspi->pioe, rpspi->spwr, 1);
  return 0;
}

/****************************************************************************
 * Name: rpmsg_port_spi_connect
 ****************************************************************************/

static inline void rpmsg_port_spi_connect(FAR struct rpmsg_port_spi_s *rpspi)
{
  IOEXP_WRITEPIN(rpspi->pioe, rpspi->spwr, 1);
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
        if (rpspi->status == RPMSG_PORT_SPI_STATUS_UNCONNECTED)
          {
            rpmsg_port_unregister(&rpspi->port);
          }
        else if (rpspi->status == RPMSG_PORT_SPI_STATUS_CONNECTED)
          {
            rpmsg_port_register(&rpspi->port, (FAR const char *)(rxhdr + 1));
            IOEXP_WRITEPIN(rpspi->pioe, rpspi->spwr, 0);
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
  FAR const struct rpmsg_port_dbgspi_config_s *spicfg,
  FAR struct spi_slave_ctrlr_s *spictrlr, FAR struct ioexpander_dev_s *rioe,
  FAR struct ioexpander_dev_s *pioe)
{
  int ret;

  if (spictrlr == NULL || rioe == NULL || pioe == NULL || spicfg == NULL)
    {
      rpmsgerr("invalid params\n");
      return -EINVAL;
    }

  /* Init mrdy gpio */

  ret = rpmsg_port_spi_init_gpio(rioe, &rpspi->mrdy, spicfg->mrdy_pin,
                                 spicfg->mrdy_invert, NULL, NULL);
  if (ret < 0)
    {
      rpmsgerr("mrdy init failed\n");
      return ret;
    }

  /* Init sreq gpio */

  ret = rpmsg_port_spi_init_gpio(rioe, &rpspi->sreq, spicfg->sreq_pin,
                                 spicfg->sreq_invert, NULL, NULL);
  if (ret < 0)
    {
      rpmsgerr("sreq init failed\n");
      return ret;
    }

  /* Init mpwr gpio */

  ret = rpmsg_port_spi_init_gpio(pioe, &rpspi->mpwr, spicfg->mpwr_pin,
                                 spicfg->mpwr_invert,
                                 rpmsg_port_spi_mpwr_handler, rpspi);
  if (ret < 0)
    {
      rpmsgerr("mpwr init failed\n");
      return ret;
    }

  /* Init spwr gpio */

  ret = rpmsg_port_spi_init_gpio(pioe, &rpspi->spwr, spicfg->spwr_pin,
                                 spicfg->spwr_invert, NULL, NULL);
  if (ret < 0)
    {
      rpmsgerr("spwr init failed\n");
      return ret;
    }

  rpspi->rioe = rioe;
  rpspi->pioe = pioe;
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
rpmsg_port_dbgspi_slave_initialize(FAR const struct rpmsg_port_config_s *cfg,
  FAR const struct rpmsg_port_dbgspi_config_s *spicfg,
  FAR struct spi_slave_ctrlr_s *spictrlr, FAR struct ioexpander_dev_s *rioe,
  FAR struct ioexpander_dev_s *pioe)
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

  ret = rpmsg_port_spi_init_hardware(rpspi, spicfg, spictrlr, rioe, pioe);
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
