/****************************************************************************
 * drivers/rpmsg/rpmsg_port_dbgspi.c
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
#  define rpmsg_port_spi_crc16(hdr)  crc16((FAR uint8_t *)&(hdr)->cmd, \
                                          (hdr)->len - sizeof((hdr)->crc))
#else
#  define rpmsg_port_spi_crc16(hdr)  0
#endif

#define BYTES2WORDS(s,b)             ((b) / ((s)->nbits >> 3))

#define RPMSG_PORT_SPI_CTNS_READ     0xFFFC0004

#define RPMSG_PORT_SPI_DUMMY_DATA    0xDEADBEEF
#define RPMSG_PORT_SPI_WR_ADDR(a)    (((a)>>2)|(0<<30))
#define RPMSG_PORT_SPI_RD_ADDR(a)    (((a)>>2)|(1<<30))

#define RPMSG_PORT_SPI_EXCHANGE_ADDR 0x20000000

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

enum rpmsg_port_spi_state_e
{
  RPMSG_PORT_SPI_STATE_IDLE,
  RPMSG_PORT_SPI_STATE_SEND,
  RPMSG_PORT_SPI_STATE_RECV,
};

enum rpmsg_port_spi_slave_state_e
{
  RPMSG_PORT_SPI_SLAVE_UNINITED,
  RPMSG_PORT_SPI_SLAVE_ENABLE_CTNS,
  RPMSG_PORT_SPI_SLAVE_CHECK_CTNS,
  RPMSG_PORT_SPI_SLAVE_READ_PARAMS,
  RPMSG_PORT_SPI_SLAVE_READY,
};

struct rpmsg_port_spi_s
{
  struct rpmsg_port_s            port;
  FAR struct spi_dev_s           *spi;
  FAR struct ioexpander_dev_s    *ioe;

  /* GPIOs used for handshake */

  uint8_t                        mrdy;
  uint8_t                        sreq;

  uint8_t                        mpwr;
  uint8_t                        spwr;

  /* SPI devices' configuration */

  uint32_t                       devid;
  int                            nbits;

  /* Reserved for cmd send */

  FAR struct rpmsg_port_header_s *cmdhdr;

  /* Used for sync data state between sreq_handler and complete_handler */

  FAR struct rpmsg_port_header_s *txhdr;
  FAR struct rpmsg_port_header_s *rxhdr;

  rpmsg_port_rx_cb_t             rxcb;

  /* Used for flow control */

  uint16_t                       txavail;
  uint16_t                       rxavail;
  uint16_t                       rxthres;

  atomic_int                     transferring;

  FAR uint8_t                    *txbuf;
  FAR uint8_t                    *rxbuf;
  uint32_t                       txaddr;
  uint32_t                       rxaddr;

  uint16_t                       status;
  uint16_t                       state;
  atomic_int                     pendingreq;
  atomic_int                     sending;

  /* Used for shake hands with slave */

  uint16_t                       slave_state;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void rpmsg_port_spi_notify_tx_ready(FAR struct rpmsg_port_s *port);
static void rpmsg_port_spi_notify_rx_free(FAR struct rpmsg_port_s *port);
static void rpmsg_port_spi_register_cb(FAR struct rpmsg_port_s *port,
                                       rpmsg_port_rx_cb_t callback);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct rpmsg_port_ops_s g_rpmsg_port_spi_ops =
{
  rpmsg_port_spi_notify_tx_ready,
  rpmsg_port_spi_notify_rx_free,
  rpmsg_port_spi_register_cb,
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

  if (atomic_fetch_add(&rpspi->transferring, 1))
    {
      return;
    }

  /* Wait until last frame has been received by peer side */

  if (atomic_exchange(&rpspi->sending, 1))
    {
      atomic_exchange(&rpspi->transferring, 0);
      return;
    }

  rpspi->state = RPMSG_PORT_SPI_STATE_SEND;
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

  txhdr->avail = rpmsg_port_queue_navail(&rpspi->port.rxq);
  txhdr->crc = rpmsg_port_spi_crc16(txhdr);

  rpmsginfo("irq send cmd:%u avail:%u\n", txhdr->cmd, txhdr->avail);

  SPI_SELECT(rpspi->spi, rpspi->devid, true);
  *(uint32_t *)rpspi->txbuf = RPMSG_PORT_SPI_WR_ADDR(rpspi->txaddr);
  memcpy(rpspi->txbuf + sizeof(uint32_t), txhdr, rpspi->cmdhdr->len);

  SPI_EXCHANGE(rpspi->spi, rpspi->txbuf, NULL,
               BYTES2WORDS(rpspi, rpspi->cmdhdr->len + sizeof(uint32_t)));

  rpspi->rxavail = txhdr->avail;
}

/****************************************************************************
 * Name: rpmsg_port_spi_receive
 ****************************************************************************/

static void rpmsg_port_spi_receive(FAR struct rpmsg_port_spi_s *rpspi)
{
  if (atomic_fetch_add(&rpspi->transferring, 1))
    {
      atomic_exchange(&rpspi->sending, 1);
      return;
    }

  atomic_exchange(&rpspi->pendingreq, 0);
  rpspi->state = RPMSG_PORT_SPI_STATE_RECV;
  SPI_SELECT(rpspi->spi, rpspi->devid, true);
  *(uint32_t *)rpspi->txbuf = RPMSG_PORT_SPI_RD_ADDR(rpspi->rxaddr);
  *((uint32_t *)rpspi->txbuf + 1) = RPMSG_PORT_SPI_DUMMY_DATA;
  SPI_EXCHANGE(rpspi->spi, rpspi->txbuf, rpspi->rxbuf,
    BYTES2WORDS(rpspi, rpspi->cmdhdr->len + 2 * sizeof(uint32_t)));
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

  if (rpspi->status == RPMSG_PORT_SPI_STATUS_CONNECTED &&
      (rpmsg_port_queue_navail(&port->rxq) -
       rpspi->rxavail >= rpspi->rxthres))
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
 * Name: rpmsg_port_spi_handshake_handler
 ****************************************************************************/

static void
rpmsg_port_spi_handshake_handler(FAR struct rpmsg_port_spi_s *rpspi)
{
  uint32_t *txbuf = (uint32_t *)rpspi->txbuf;
  uint32_t *rxbuf = (uint32_t *)rpspi->rxbuf;

  txbuf[1] = RPMSG_PORT_SPI_DUMMY_DATA;
  switch (rpspi->slave_state)
    {
      case RPMSG_PORT_SPI_SLAVE_UNINITED:
        rpspi->slave_state = RPMSG_PORT_SPI_SLAVE_ENABLE_CTNS;
        txbuf[0] = RPMSG_PORT_SPI_RD_ADDR(RPMSG_PORT_SPI_CTNS_READ);
        SPI_SELECT(rpspi->spi, rpspi->devid, true);
        SPI_EXCHANGE(rpspi->spi, txbuf, rxbuf,
                     BYTES2WORDS(rpspi, sizeof(uint32_t)) * 3);
        break;

      case RPMSG_PORT_SPI_SLAVE_ENABLE_CTNS:
        rpspi->slave_state = RPMSG_PORT_SPI_SLAVE_CHECK_CTNS;
        DEBUGASSERT(rxbuf[1] == 0x01);
        txbuf[0] = RPMSG_PORT_SPI_RD_ADDR(RPMSG_PORT_SPI_EXCHANGE_ADDR);
        SPI_SELECT(rpspi->spi, rpspi->devid, true);
        SPI_EXCHANGE(rpspi->spi, txbuf, rxbuf,
                     BYTES2WORDS(rpspi, sizeof(uint32_t)) * 3);
        break;

      case RPMSG_PORT_SPI_SLAVE_CHECK_CTNS:
        rpspi->slave_state = RPMSG_PORT_SPI_SLAVE_READ_PARAMS;
        txbuf[0] = RPMSG_PORT_SPI_RD_ADDR(rxbuf[2]);
        SPI_SELECT(rpspi->spi, rpspi->devid, true);
        SPI_EXCHANGE(rpspi->spi, txbuf, rxbuf,
                     BYTES2WORDS(rpspi, sizeof(uint32_t)) * 6);
        break;

      case RPMSG_PORT_SPI_SLAVE_READ_PARAMS:
        rpspi->slave_state = RPMSG_PORT_SPI_SLAVE_READY;

        /* Master's rpspi->txaddr is rx buffer address of slave side */

        rpspi->rxaddr = rxbuf[4];
        rpspi->txaddr = rxbuf[2];
        IOEXP_WRITEPIN(rpspi->ioe, rpspi->mpwr, 0);
        rpmsg_port_spi_send(rpspi);
        break;

      default:
        break;
    }
}

/****************************************************************************
 * Name: rpmsg_port_spi_complete_handler
 ****************************************************************************/

static void rpmsg_port_spi_complete_handler(FAR void *arg)
{
  FAR struct rpmsg_port_spi_s *rpspi = arg;

  SPI_SELECT(rpspi->spi, rpspi->devid, false);

  if (rpspi->slave_state < RPMSG_PORT_SPI_SLAVE_READY)
    {
      return rpmsg_port_spi_handshake_handler(rpspi);
    }

  if (rpspi->state == RPMSG_PORT_SPI_STATE_SEND)
    {
      rpmsginfo("send done\n");
      if (rpspi->txhdr != NULL)
        {
          rpmsg_port_queue_return_buffer(&rpspi->port.txq, rpspi->txhdr);
          rpspi->txhdr = NULL;
          rpspi->txavail -= 1;
        }

      goto out;
    }

  memcpy(rpspi->rxhdr, rpspi->rxbuf + 2 * sizeof(uint32_t),
         rpspi->cmdhdr->len);
  rpmsginfo("received cmd:%u avail:%u\n",
    rpspi->rxhdr->cmd, rpspi->rxhdr->avail);

  if (rpspi->rxhdr->crc != 0)
    {
      uint16_t crc = rpmsg_port_spi_crc16(rpspi->rxhdr);

      if (rpspi->rxhdr->crc != crc)
        {
          rpmsgerr("crc check fail received: %u calculated: %u\n",
                   rpspi->rxhdr->crc, crc);
          goto out;
        }
    }

  /* Skip any data received when connection is not established until a
   * connect req data packet has been received.
   */

  if (rpspi->status != RPMSG_PORT_SPI_STATUS_CONNECTED)
    {
      if (rpspi->rxhdr->cmd != RPMSG_PORT_SPI_CMD_CONNECT)
        {
          goto out;
        }

      rpspi->txavail = rpspi->rxhdr->avail;
      rpspi->status = RPMSG_PORT_SPI_STATUS_CONNECTED;
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
    }

out:
  rpspi->state = RPMSG_PORT_SPI_STATE_IDLE;
  atomic_exchange(&rpspi->transferring, 0);
  if (atomic_load(rpspi->pendingreq))
    {
      rpmsg_port_spi_receive(rpspi);
    }
  else if (rpspi->status == RPMSG_PORT_SPI_STATUS_CONNECTED &&
      ((rpspi->txavail > 0 &&
        rpmsg_port_queue_nused(&rpspi->port.txq) > 0) ||
       (rpmsg_port_queue_navail(&rpspi->port.rxq) -
        rpspi->rxavail >= rpspi->rxthres)))
    {
      rpmsg_port_spi_send(rpspi);
    }
}

/****************************************************************************
 * Name: rpmsg_port_spi_mrdy_handler
 ****************************************************************************/

static int rpmsg_port_spi_mrdy_handler(FAR struct ioexpander_dev_s *dev,
                                       ioe_pinset_t pinset, FAR void *arg)
{
  FAR struct rpmsg_port_spi_s *rpspi = arg;

  rpmsginfo("send complete\n");

  atomic_exchange(&rpspi->sending, 0);
  if (rpspi->status == RPMSG_PORT_SPI_STATUS_CONNECTED &&
      ((rpspi->txavail > 0 &&
        rpmsg_port_queue_nused(&rpspi->port.txq) > 0) ||
       (rpmsg_port_queue_navail(&rpspi->port.rxq) -
        rpspi->rxavail >= rpspi->rxthres)))
    {
      rpmsg_port_spi_send(rpspi);
    }

  return 0;
}

/****************************************************************************
 * Name: rpmsg_port_spi_sreq_handler
 ****************************************************************************/

static int rpmsg_port_spi_sreq_handler(FAR struct ioexpander_dev_s *dev,
                                       ioe_pinset_t pinset, FAR void *arg)
{
  FAR struct rpmsg_port_spi_s *rpspi = arg;

  rpmsginfo("sreq enter\n");

  rpmsg_port_spi_receive(rpspi);
  return 0;
}

/****************************************************************************
 * Name: rpmsg_port_spi_handshake
 ****************************************************************************/

static void rpmsg_port_spi_handshake(FAR struct rpmsg_port_spi_s *rpspi)
{
  uint32_t *txbuf = (uint32_t *)rpspi->txbuf;

  rpspi->slave_state = RPMSG_PORT_SPI_SLAVE_UNINITED;

  txbuf[0] = RPMSG_PORT_SPI_WR_ADDR(RPMSG_PORT_SPI_CTNS_READ);
  txbuf[1] = 0x1;

  SPI_SELECT(rpspi->spi, rpspi->devid, true);
  SPI_EXCHANGE(rpspi->spi, txbuf, NULL,
               BYTES2WORDS(rpspi, 2 * sizeof(uint32_t)));
}

/****************************************************************************
 * Name: rpmsg_port_spi_spwr_handler
 ****************************************************************************/

static int rpmsg_port_spi_spwr_handler(FAR struct ioexpander_dev_s *dev,
                                       ioe_pinset_t pinset, FAR void *arg)
{
  FAR struct rpmsg_port_spi_s *rpspi = arg;

  rpmsginfo("spwr enter\n");

  rpspi->status = RPMSG_PORT_SPI_STATUS_UNCONNECTED;
  rpmsg_port_spi_drop_packets(rpspi);

  rpspi->rxhdr->cmd = RPMSG_PORT_SPI_CMD_CONNECT;
  rpmsg_port_queue_add_buffer(&rpspi->port.rxq, rpspi->rxhdr);
  rpspi->rxhdr = rpmsg_port_queue_get_available_buffer(
    &rpspi->port.rxq, false);
  DEBUGASSERT(rpspi->rxhdr != NULL);

  rpmsg_port_spi_handshake(rpspi);
  return 0;
}

/****************************************************************************
 * Name: rpmsg_port_spi_connect
 ****************************************************************************/

static void rpmsg_port_spi_connect(FAR struct rpmsg_port_spi_s *rpspi)
{
  bool val;

  IOEXP_READPIN(rpspi->ioe, rpspi->spwr, &val);
  rpmsginfo("spwr level is %d\n", val);

  /* If spwr gpio is high, that means peer is powered up. */

  if (val)
    {
      rpmsg_port_spi_handshake(rpspi);
    }
  else
    {
      IOEXP_WRITEPIN(rpspi->ioe, rpspi->mpwr, 1);
    }
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
  FAR struct spi_dev_s *spi, FAR struct ioexpander_dev_s *ioe)
{
  int ret;

  /* Init mrdy gpio */

  ret = rpmsg_port_spi_init_gpio(ioe, &rpspi->mrdy, spicfg->mrdy_pin,
                                 spicfg->mrdy_invert,
                                 rpmsg_port_spi_mrdy_handler, rpspi);
  if (ret < 0)
    {
      rpmsgerr("mrdy init failed\n");
      return ret;
    }

  /* Init sreq gpio */

  ret = rpmsg_port_spi_init_gpio(ioe, &rpspi->sreq, spicfg->sreq_pin,
                                 spicfg->sreq_invert,
                                 rpmsg_port_spi_sreq_handler, rpspi);
  if (ret < 0)
    {
      rpmsgerr("sreq init failed\n");
      return ret;
    }

  /* Init mpwr gpio */

  ret = rpmsg_port_spi_init_gpio(ioe, &rpspi->mpwr, spicfg->mpwr_pin,
                                 spicfg->mpwr_invert, NULL, NULL);
  if (ret < 0)
    {
      rpmsgerr("mpwr init failed\n");
      return ret;
    }

  /* Init spwr gpio */

  ret = rpmsg_port_spi_init_gpio(ioe, &rpspi->spwr, spicfg->spwr_pin,
                                 spicfg->spwr_invert,
                                 rpmsg_port_spi_spwr_handler, NULL);
  if (ret < 0)
    {
      rpmsgerr("spwr init failed\n");
      return ret;
    }

  SPI_SETBITS(spi, spicfg->nbits);
  SPI_SETMODE(spi, spicfg->mode);
  SPI_SETFREQUENCY(spi, spicfg->freq);
  SPI_REGISTERCALLBACK(spi, rpmsg_port_spi_complete_handler, rpspi);
  SPI_SELECT(spi, spicfg->devid, false);

  rpspi->spi = spi;
  rpspi->ioe = ioe;
  rpspi->devid = spicfg->devid;
  rpspi->nbits = spicfg->nbits;

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rpmsg_port_dbgspi_initialize
 ****************************************************************************/

int rpmsg_port_dbgspi_initialize(FAR const struct rpmsg_port_config_s *cfg,
  FAR const struct rpmsg_port_dbgspi_config_s *spicfg,
  FAR struct spi_dev_s *spi, FAR struct ioexpander_dev_s *ioe)
{
  FAR struct rpmsg_port_spi_s *rpspi;
  FAR char *argv[3];
  char arg1[32];
  int ret;

  if (spi == NULL || ioe == NULL || spicfg == NULL)
    {
      rpmsgerr("invalid params\n");
      return -EINVAL;
    }

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

  rpspi->txbuf = kmm_memalign(32, rpspi->cmdhdr->len + 2 * sizeof(uint32_t));
  if (rpspi->txbuf == NULL)
    {
      rpmsgerr("malloc rpmsg spi tx buffer failed\n");
      goto rpmsg_err;
    }

  rpspi->rxbuf = kmm_memalign(32, rpspi->rxhdr->len + 2 * sizeof(uint32_t));
  if (rpspi->rxbuf == NULL)
    {
      rpmsgerr("malloc rpmsg spi rx buffer failed\n");
      kmm_free(rpspi->txbuf);
      goto rpmsg_err;
    }

  rpspi->rxthres = rpmsg_port_queue_navail(&rpspi->port.rxq) *
                   CONFIG_RPMSG_PORT_SPI_RX_THRESHOLD / 100;

  ret = rpmsg_port_spi_init_hardware(rpspi, spicfg, spi, ioe);
  if (ret < 0)
    {
      rpmsgerr("rpmsg port spi hardware init failed\n");
      goto out;
    }

  snprintf(arg1, sizeof(arg1), "%p", rpspi);
  argv[0] = (FAR void *)cfg->remotecpu;
  argv[1] = arg1;
  argv[2] = NULL;
  ret = kthread_create("rpmsg-spi",
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
  kmm_free(rpspi->rxbuf);
  kmm_free(rpspi->txbuf);
rpmsg_err:
  kmm_free(rpspi);
  return ret;
}
