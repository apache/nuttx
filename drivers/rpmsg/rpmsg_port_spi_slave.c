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
#include <stdio.h>

#include <nuttx/atomic.h>
#include <nuttx/crc16.h>
#include <nuttx/kmalloc.h>
#include <nuttx/kthread.h>
#include <nuttx/nuttx.h>
#include <nuttx/power/pm.h>
#include <nuttx/irq.h>
#include <nuttx/mutex.h>
#include <nuttx/reboot_notifier.h>
#include <nuttx/spinlock.h>

#include "rpmsg_port.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_RPMSG_PORT_SPI_CRC
#  define rpmsg_port_spi_crc16(hdr)     crc16ibm((FAR uint8_t *)&(hdr)->cmd, \
                                                 (hdr)->len - sizeof((hdr)->crc))
#else
#  define rpmsg_port_spi_crc16(hdr)     0
#endif

#define RPMSG_PORT_SPI_CMD_TIMEOUT      1000
#define RPMSG_PORT_SPI_BYTES2WORDS(s,b) ((b) / ((s)->nbits >> 3))

/****************************************************************************
 * Private Types
 ****************************************************************************/

enum rpmsg_port_spi_cmd_e
{
  RPMSG_PORT_SPI_CMD_CONNECT = 0x01,
  RPMSG_PORT_SPI_CMD_AVAIL,
  RPMSG_PORT_SPI_CMD_DATA,
  RPMSG_PORT_SPI_CMD_SUSPEND,
  RPMSG_PORT_SPI_CMD_RESUME,
  RPMSG_PORT_SPI_CMD_SHUTDOWN,
};

enum rpmsg_port_spi_state_e
{
  RPMSG_PORT_SPI_STATE_UNCONNECTED  = 0x01,
  RPMSG_PORT_SPI_STATE_CONNECTING,
  RPMSG_PORT_SPI_STATE_RECONNECTING,
  RPMSG_PORT_SPI_STATE_DISCONNECTING,
  RPMSG_PORT_SPI_STATE_CONNECTED,
};

struct rpmsg_port_spi_s
{
  struct rpmsg_port_s            port;
  FAR struct spi_slave_ctrlr_s   *spictrlr;
  struct spi_slave_dev_s         spislv;
  FAR struct ioexpander_dev_s    *ioe;
  bool                           bound;

  /* GPIOs used for handshake */

  uint8_t                        mreq;
  uint8_t                        sreq;

  /* SPI devices' configuration */

  int                            nbits;
  int                            mode;

  /* Reserved for cmd send */

  FAR struct rpmsg_port_header_s *cmdhdr;

  /* Used for sync data state between mreq_handler and
   * rpmsg_port_spi_slave_notify
   */

  FAR struct rpmsg_port_header_s *txhdr;
  FAR struct rpmsg_port_header_s *rxhdr;

  rpmsg_port_rx_cb_t             rxcb;
  volatile uint8_t               state;
  spinlock_t                     lock;
#ifdef CONFIG_PM
  spinlock_t                     pmlock;
  struct wdog_s                  wdog;
  struct pm_wakelock_s           wakelock;
#endif
  volatile bool                  shutdown;
  struct notifier_block          nb;

  /* Used for flow control */

  uint16_t                       txavail;
  uint16_t                       rxavail;
  uint16_t                       rxthres;

  atomic_t                       transferring;
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
static size_t
rpmsg_port_spi_slave_getrecvbuf(FAR struct spi_slave_dev_s *dev,
                                FAR void **buffer);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct rpmsg_port_ops_s g_rpmsg_port_spi_ops =
{
  rpmsg_port_spi_notify_tx_ready,
  rpmsg_port_spi_notify_rx_free,
  NULL,
  rpmsg_port_spi_register_cb,
};

static const struct spi_slave_devops_s g_rpmsg_port_spi_slave_ops =
{
  rpmsg_port_spi_slave_select,             /* select */
  rpmsg_port_spi_slave_cmddata,            /* cmddata */
  rpmsg_port_spi_slave_getdata,            /* getdata */
  rpmsg_port_spi_slave_receive,            /* receive */
  rpmsg_port_spi_slave_notify,             /* notify */
  rpmsg_port_spi_slave_getrecvbuf,         /* getrecvbuf */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_PM

/****************************************************************************
 * Name: rpmsg_port_spi_pm_callback
 ****************************************************************************/

static void rpmsg_port_spi_pm_callback(wdparm_t arg)
{
  FAR struct rpmsg_port_spi_s *rpspi = (FAR struct rpmsg_port_spi_s *)arg;
  irqstate_t flags;
  int count;

  flags = spin_lock_irqsave(&rpspi->pmlock);
  count = pm_wakelock_staycount(&rpspi->wakelock);
  if (count > 0 && atomic_load(&rpspi->transferring) == 0 &&
      rpmsg_port_queue_nused(&rpspi->port.txq) == 0 &&
      rpmsg_port_queue_nused(&rpspi->port.rxq) == 0)
    {
      pm_wakelock_relax(&rpspi->wakelock);
    }
  else
    {
      wd_start(&rpspi->wdog, MSEC2TICK(CONFIG_RPMSG_PORT_SPI_PM_TIMEOUT),
               rpmsg_port_spi_pm_callback, (wdparm_t)rpspi);
    }

  spin_unlock_irqrestore(&rpspi->pmlock, flags);
}

/****************************************************************************
 * Name: rpmsg_port_spi_pm_action
 ****************************************************************************/

static inline void
rpmsg_port_spi_pm_action(FAR struct rpmsg_port_spi_s *rpspi, bool stay)
{
  irqstate_t flags;
  int count;

  flags = spin_lock_irqsave(&rpspi->pmlock);
  count = pm_wakelock_staycount(&rpspi->wakelock);
  if (stay && count == 0)
    {
      pm_wakelock_stay(&rpspi->wakelock);
    }
  else if (!stay && count > 0 &&
           rpmsg_port_queue_nused(&rpspi->port.txq) == 0)
    {
      wd_start(&rpspi->wdog, MSEC2TICK(CONFIG_RPMSG_PORT_SPI_PM_TIMEOUT),
               rpmsg_port_spi_pm_callback, (wdparm_t)rpspi);
    }

  spin_unlock_irqrestore(&rpspi->pmlock, flags);
}

#else
#  define rpmsg_port_spi_pm_action(rpspi, stay)
#endif

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

  txhdr = rpspi->cmdhdr;
  if (rpspi->state == RPMSG_PORT_SPI_STATE_UNCONNECTED)
    {
      txhdr->cmd = RPMSG_PORT_SPI_CMD_CONNECT;
      strlcpy((FAR char *)(txhdr + 1), rpspi->port.rpmsg.cpuname,
              RPMSG_NAME_SIZE);
    }
  else if (rpspi->shutdown)
    {
      txhdr->cmd = RPMSG_PORT_SPI_CMD_SHUTDOWN;
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
      txhdr->cmd = RPMSG_PORT_SPI_CMD_AVAIL;
    }

  txhdr->avail = rpmsg_port_queue_navail(&rpspi->port.rxq);
  txhdr->avail = txhdr->avail > 1 ? txhdr->avail - 1 : 0;
  txhdr->crc = rpmsg_port_spi_crc16(txhdr);

  rpmsginfo("send cmd:%u avail:%u\n", txhdr->cmd, txhdr->avail);

  rpmsg_port_spi_pm_action(rpspi, true);
  rpmsg_port_update_timestamp(&rpspi->port.txq, txhdr, true);
  SPIS_CTRLR_ENQUEUE(rpspi->spictrlr, txhdr,
                     RPMSG_PORT_SPI_BYTES2WORDS(rpspi, rpspi->port.txq.len));
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

  if (rpspi->state == RPMSG_PORT_SPI_STATE_CONNECTED)
    {
      rpmsg_port_spi_exchange(rpspi);
    }
  else
    {
      /* Drop txq buffers when a reconnection happens to make connected
       * status false.
       */

      rpmsg_port_drop_packets(&rpspi->port, RPMSG_PORT_DROP_TXQ);
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
  return 0;
}

/****************************************************************************
 * Name: rpmsg_port_spi_slave_getrecvbuf
 ****************************************************************************/

static size_t
rpmsg_port_spi_slave_getrecvbuf(FAR struct spi_slave_dev_s *dev,
                                FAR void **buffer)
{
  FAR struct rpmsg_port_spi_s *rpspi =
    container_of(dev, struct rpmsg_port_spi_s, spislv);

  *buffer = rpspi->rxhdr;
  return RPMSG_PORT_SPI_BYTES2WORDS(rpspi, rpspi->port.rxq.len);
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
  irqstate_t flags;

  IOEXP_WRITEPIN(rpspi->ioe, rpspi->sreq, 0);
  SPIS_CTRLR_QPOLL(rpspi->spictrlr);

  rpmsginfo("received cmd:%u avail:%u\n",
            rpspi->rxhdr->cmd, rpspi->rxhdr->avail);

  rpmsg_port_update_timestamp(&rpspi->port.rxq, rpspi->rxhdr, false);
  if (rpspi->txhdr != NULL)
    {
      rpmsg_port_queue_return_buffer(&rpspi->port.txq, rpspi->txhdr);
      rpspi->txhdr = NULL;
    }
  else if (rpspi->cmdhdr->cmd == RPMSG_PORT_SPI_CMD_SHUTDOWN)
    {
      rpspi->shutdown = false;
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

  flags = spin_lock_irqsave(&rpspi->lock);
  if (rpspi->state == RPMSG_PORT_SPI_STATE_UNCONNECTED)
    {
      if (rpspi->rxhdr->cmd != RPMSG_PORT_SPI_CMD_CONNECT)
        {
          goto unlock;
        }

      rpspi->txavail = rpspi->rxhdr->avail;
      rpspi->state = RPMSG_PORT_SPI_STATE_CONNECTING;
    }
  else if (rpspi->state == RPMSG_PORT_SPI_STATE_CONNECTED)
    {
      rpspi->txavail = rpspi->rxhdr->avail;
      if (rpspi->rxhdr->cmd == RPMSG_PORT_SPI_CMD_CONNECT)
        {
          rpspi->state = RPMSG_PORT_SPI_STATE_RECONNECTING;

          /* Drop all the unprocessed rxq buffer and pre-send txq buffer
           * when a reconnect request to be received.
           */

          rpmsg_port_drop_packets(&rpspi->port, RPMSG_PORT_DROP_ALL);
        }
    }
  else if (rpspi->rxhdr->cmd == RPMSG_PORT_SPI_CMD_CONNECT)
    {
      /* Drop connect packets received during connecting status change
       * and change to RPMSG_PORT_SPI_STATE_CONNECTING to indicate there
       * is a reconnection happened during the shutting down process.
       */

      if (rpspi->state == RPMSG_PORT_SPI_STATE_DISCONNECTING)
        {
          rpspi->state = RPMSG_PORT_SPI_STATE_CONNECTING;
        }

      goto unlock;
    }

  if (rpspi->rxhdr->cmd == RPMSG_PORT_SPI_CMD_SUSPEND)
    {
      rpmsg_modify_signals(&rpspi->port.rpmsg, 0, RPMSG_SIGNAL_RUNNING);
    }
  else if (rpspi->rxhdr->cmd == RPMSG_PORT_SPI_CMD_RESUME)
    {
      rpmsg_modify_signals(&rpspi->port.rpmsg, RPMSG_SIGNAL_RUNNING, 0);
    }
  else if (rpspi->rxhdr->cmd != RPMSG_PORT_SPI_CMD_AVAIL)
    {
      if (rpspi->rxhdr->cmd == RPMSG_PORT_SPI_CMD_SHUTDOWN)
        {
          rpspi->state = RPMSG_PORT_SPI_STATE_DISCONNECTING;
          rpmsg_port_drop_packets(&rpspi->port, RPMSG_PORT_DROP_ALL);
        }

      rpmsg_port_queue_add_buffer(&rpspi->port.rxq, rpspi->rxhdr);
      rpspi->rxhdr = rpmsg_port_queue_get_available_buffer(
        &rpspi->port.rxq, false);
      DEBUGASSERT(rpspi->rxhdr != NULL);
    }

unlock:
  spin_unlock_irqrestore(&rpspi->lock, flags);
out:
  if (atomic_xchg(&rpspi->transferring, 0) > 1 ||
      (rpspi->txavail > 0 && rpmsg_port_queue_nused(&rpspi->port.txq) > 0))
    {
      rpmsg_port_spi_exchange(rpspi);
    }
  else
    {
      rpmsg_port_spi_pm_action(rpspi, false);
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

  if (!rpspi->bound)
    {
      SPIS_CTRLR_BIND(rpspi->spictrlr, &rpspi->spislv,
                      rpspi->mode, rpspi->nbits);
      rpspi->bound = true;
    }

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
  irqstate_t flags;

  rpmsginfo("received cmd: %u avail: %u", rxhdr->cmd, rxhdr->avail);
  switch (rxhdr->cmd)
    {
      case RPMSG_PORT_SPI_CMD_CONNECT:
        flags = spin_lock_irqsave(&rpspi->lock);
        if (rpspi->state == RPMSG_PORT_SPI_STATE_RECONNECTING)
          {
            spin_unlock_irqrestore(&rpspi->lock, flags);
            rpmsg_port_unregister(&rpspi->port);

            /* Do not trigger the reconnect if a shut down cmd has been
             * received during the unregister process
             */

            flags = spin_lock_irqsave(&rpspi->lock);
            if (rpspi->state == RPMSG_PORT_SPI_STATE_RECONNECTING)
              {
                rpspi->state = RPMSG_PORT_SPI_STATE_UNCONNECTED;
                spin_unlock_irqrestore(&rpspi->lock, flags);
                rpmsg_port_spi_connect(rpspi);
              }
            else
              {
                rpspi->state = RPMSG_PORT_SPI_STATE_UNCONNECTED;
                spin_unlock_irqrestore(&rpspi->lock, flags);
                IOEXP_WRITEPIN(rpspi->ioe, rpspi->sreq, 0);
              }
          }
        else if (rpspi->state == RPMSG_PORT_SPI_STATE_CONNECTING)
          {
            rpspi->state = RPMSG_PORT_SPI_STATE_CONNECTED;
            spin_unlock_irqrestore(&rpspi->lock, flags);
            rpmsg_port_register(&rpspi->port, (FAR const char *)(rxhdr + 1));
          }

        rpmsg_port_queue_return_buffer(&rpspi->port.rxq, rxhdr);
        break;

      case RPMSG_PORT_SPI_CMD_DATA:
        rpspi->rxcb(&rpspi->port, rxhdr);
        break;

      case RPMSG_PORT_SPI_CMD_SHUTDOWN:
        rpmsg_port_unregister(&rpspi->port);
        flags = spin_lock_irqsave(&rpspi->lock);
        if (rpspi->state == RPMSG_PORT_SPI_STATE_DISCONNECTING)
          {
            SPIS_CTRLR_UNBIND(rpspi->spictrlr);
            rpspi->bound = false;
            rpspi->state = RPMSG_PORT_SPI_STATE_UNCONNECTED;
            spin_unlock_irqrestore(&rpspi->lock, flags);
            IOEXP_WRITEPIN(rpspi->ioe, rpspi->sreq, 0);
          }
        else if (rpspi->state == RPMSG_PORT_SPI_STATE_CONNECTING)
          {
            rpspi->state = RPMSG_PORT_SPI_STATE_UNCONNECTED;
            spin_unlock_irqrestore(&rpspi->lock, flags);
            rpmsg_port_spi_connect(rpspi);
          }

        rpmsg_port_queue_return_buffer(&rpspi->port.rxq, rxhdr);
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

#if CONFIG_IOEXPANDER_NPINS <= 64
      ptr = IOEP_ATTACH(ioe, 1 << pin, callback, args);
#else
      ptr = IOEP_ATTACH(ioe, pin, callback, args);
#endif
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
  rpspi->mode = spicfg->mode;
  SPIS_CTRLR_BIND(spictrlr, &rpspi->spislv, spicfg->mode, spicfg->nbits);
  rpspi->bound = true;

  return 0;
}

/****************************************************************************
 * Name: rpmsg_port_spi_reboot_handler
 ****************************************************************************/

static int
rpmsg_port_spi_reboot_handler(FAR struct notifier_block *nb,
                              unsigned long action, FAR void *data)
{
  FAR struct rpmsg_port_spi_s *rpspi =
    container_of(nb, struct rpmsg_port_spi_s, nb);
  int timeout = RPMSG_PORT_SPI_CMD_TIMEOUT / 10;

  if ((action == SYS_POWER_OFF || action == SYS_RESTART) &&
      rpspi->state == RPMSG_PORT_SPI_STATE_CONNECTED)
    {
      rpspi->shutdown = true;
      rpmsg_port_spi_exchange(rpspi);

      /* Wait until shutdown cmd has been sent done. */

      while (timeout >= 0 && rpspi->shutdown)
        {
          usleep(10000);
          timeout--;
        }

      if (timeout < 0)
        {
          rpmsgerr("send cmd shutdown cmd timedout\n");
        }
    }

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
  rpspi->cmdhdr->len = sizeof(struct rpmsg_port_header_s);

  rpspi->rxthres = rpmsg_port_queue_navail(&rpspi->port.rxq) *
                   CONFIG_RPMSG_PORT_SPI_RX_THRESHOLD / 100;
  rpspi->state = RPMSG_PORT_SPI_STATE_UNCONNECTED;
  spin_lock_init(&rpspi->lock);

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

#ifdef CONFIG_PM
  spin_lock_init(&rpspi->pmlock);
  pm_wakelock_init(&rpspi->wakelock, cfg->remotecpu,
                   PM_IDLE_DOMAIN, PM_NORMAL);
#endif

  rpspi->nb.notifier_call = rpmsg_port_spi_reboot_handler;
  register_reboot_notifier(&rpspi->nb);

  return 0;

out:
  rpmsg_port_uninitialize(&rpspi->port);
rpmsg_err:
  kmm_free(rpspi);
  return ret;
}
