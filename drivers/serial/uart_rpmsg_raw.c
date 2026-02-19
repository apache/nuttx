/****************************************************************************
 * drivers/serial/uart_rpmsg_raw.c
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

#include <nuttx/fs/ioctl.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/rpmsg/rpmsg.h>
#include <nuttx/serial/serial.h>
#include <nuttx/serial/uart_rpmsg_raw.h>

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/

#define UART_RPMSG_DEV_CONSOLE          "/dev/console"
#define UART_RPMSG_DEV_PREFIX           "/dev/tty"
#define UART_RPMSG_EPT                  "rpmsg-tty"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct uart_rpmsg_priv_s
{
  struct rpmsg_endpoint ept;
  mutex_t               lock;
  FAR const char        *devname;
  FAR const char        *cpuname;
  FAR void              *recv_data;
  size_t                recv_len;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  uart_rpmsg_raw_setup(FAR struct uart_dev_s *dev);
static void uart_rpmsg_raw_shutdown(FAR struct uart_dev_s *dev);
static int  uart_rpmsg_raw_attach(FAR struct uart_dev_s *dev);
static void uart_rpmsg_raw_detach(FAR struct uart_dev_s *dev);
static void uart_rpmsg_raw_rxint(FAR struct uart_dev_s *dev, bool enable);
static void uart_rpmsg_raw_dmasend(FAR struct uart_dev_s *dev);
static void uart_rpmsg_raw_dmareceive(FAR struct uart_dev_s *dev);
static void uart_rpmsg_raw_dmarxfree(FAR struct uart_dev_s *dev);
static void uart_rpmsg_raw_dmatxavail(FAR struct uart_dev_s *dev);
static void uart_rpmsg_raw_send(FAR struct uart_dev_s *dev, int ch);
static void uart_rpmsg_raw_txint(FAR struct uart_dev_s *dev, bool enable);
static bool uart_rpmsg_raw_txready(FAR struct uart_dev_s *dev);
static bool uart_rpmsg_raw_txempty(FAR struct uart_dev_s *dev);
static void uart_rpmsg_raw_device_created(FAR struct rpmsg_device *rdev,
                                      FAR void *priv_);
static void uart_rpmsg_raw_device_destroy(FAR struct rpmsg_device *rdev,
                                      FAR void *priv_);
static int  uart_rpmsg_raw_ept_cb(struct rpmsg_endpoint *ept, FAR void *data,
                              size_t len, uint32_t src, FAR void *priv_);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct uart_ops_s g_uart_rpmsg_ops =
{
  .setup         = uart_rpmsg_raw_setup,
  .shutdown      = uart_rpmsg_raw_shutdown,
  .attach        = uart_rpmsg_raw_attach,
  .detach        = uart_rpmsg_raw_detach,
  .rxint         = uart_rpmsg_raw_rxint,
  .dmasend       = uart_rpmsg_raw_dmasend,
  .dmareceive    = uart_rpmsg_raw_dmareceive,
  .dmarxfree     = uart_rpmsg_raw_dmarxfree,
  .dmatxavail    = uart_rpmsg_raw_dmatxavail,
  .send          = uart_rpmsg_raw_send,
  .txint         = uart_rpmsg_raw_txint,
  .txready       = uart_rpmsg_raw_txready,
  .txempty       = uart_rpmsg_raw_txempty,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int uart_rpmsg_raw_setup(FAR struct uart_dev_s *dev)
{
  return OK;
}

static void uart_rpmsg_raw_shutdown(FAR struct uart_dev_s *dev)
{
}

static int uart_rpmsg_raw_attach(FAR struct uart_dev_s *dev)
{
  return OK;
}

static void uart_rpmsg_raw_detach(FAR struct uart_dev_s *dev)
{
}

static void uart_rpmsg_raw_rxint(FAR struct uart_dev_s *dev, bool enable)
{
}

static void uart_rpmsg_raw_dmasend(FAR struct uart_dev_s *dev)
{
  FAR struct uart_rpmsg_priv_s *priv = dev->priv;
  FAR struct uart_dmaxfer_s *xfer = &dev->dmatx;
  size_t len = xfer->length + xfer->nlength;
  FAR uint8_t *buf;
  uint32_t space;
  int ret;

  buf = rpmsg_get_tx_payload_buffer(&priv->ept, &space, true);
  if (!buf)
    {
      return;
    }

  if (len > space)
    {
      len = space;
    }

  if (len > xfer->length)
    {
      memcpy(buf, xfer->buffer, xfer->length);
      memcpy(buf + xfer->length, xfer->nbuffer, len - xfer->length);
    }
  else
    {
      memcpy(buf, xfer->buffer, len);
    }

  ret = rpmsg_send_nocopy(&priv->ept, buf, len);

  if (ret < 0)
    {
      rpmsg_release_tx_buffer(&priv->ept, buf);
      dev->dmatx.nbytes = 0;
    }
  else
    {
      dev->dmatx.nbytes = len;
    }

  uart_xmitchars_done(dev);
}

static void uart_rpmsg_raw_dmareceive(FAR struct uart_dev_s *dev)
{
  FAR struct uart_rpmsg_priv_s *priv = dev->priv;
  FAR struct uart_dmaxfer_s *xfer = &dev->dmarx;
  FAR char *data = priv->recv_data;
  size_t len = priv->recv_len;
  size_t space = xfer->length + xfer->nlength;

  if (len > space)
    {
      len = space;
    }

  if (len > xfer->length)
    {
      memcpy(xfer->buffer, data, xfer->length);
      memcpy(xfer->nbuffer, data + xfer->length, len - xfer->length);
    }
  else
    {
      memcpy(xfer->buffer, data, len);
    }

  xfer->nbytes = len;
  uart_recvchars_done(dev);
}

static void uart_rpmsg_raw_dmarxfree(FAR struct uart_dev_s *dev)
{
}

static void uart_rpmsg_raw_dmatxavail(FAR struct uart_dev_s *dev)
{
  FAR struct uart_rpmsg_priv_s *priv = dev->priv;

  nxmutex_lock(&priv->lock);

  if (is_rpmsg_ept_ready(&priv->ept) && dev->dmatx.length == 0)
    {
      uart_xmitchars_dma(dev);
    }

  nxmutex_unlock(&priv->lock);
}

static void uart_rpmsg_raw_send(FAR struct uart_dev_s *dev, int ch)
{
  int nexthead;

  nexthead = dev->xmit.head + 1;
  if (nexthead >= dev->xmit.size)
    {
      nexthead = 0;
    }

  if (nexthead != dev->xmit.tail)
    {
      /* No.. not full.  Add the character to the TX buffer and return. */

      dev->xmit.buffer[dev->xmit.head] = ch;
      dev->xmit.head = nexthead;
    }

  uart_rpmsg_raw_dmatxavail(dev);
}

static void uart_rpmsg_raw_txint(FAR struct uart_dev_s *dev, bool enable)
{
}

static bool uart_rpmsg_raw_txready(FAR struct uart_dev_s *dev)
{
  int nexthead;

  nexthead = dev->xmit.head + 1;
  if (nexthead >= dev->xmit.size)
    {
      nexthead = 0;
    }

  return nexthead != dev->xmit.tail;
}

static bool uart_rpmsg_raw_txempty(FAR struct uart_dev_s *dev)
{
  return true;
}

static void uart_rpmsg_raw_device_created(FAR struct rpmsg_device *rdev,
                                          FAR void *priv_)
{
  FAR struct uart_dev_s *dev = priv_;
  FAR struct uart_rpmsg_priv_s *priv = dev->priv;

  if (strcmp(priv->cpuname, rpmsg_get_cpuname(rdev)) == 0)
    {
      priv->ept.priv = dev;
      rpmsg_create_ept(&priv->ept, rdev, UART_RPMSG_EPT,
                       RPMSG_ADDR_ANY, RPMSG_ADDR_ANY,
                       uart_rpmsg_raw_ept_cb, NULL);
    }
}

static void uart_rpmsg_raw_device_destroy(FAR struct rpmsg_device *rdev,
                                          FAR void *priv_)
{
  FAR struct uart_dev_s *dev = priv_;
  FAR struct uart_rpmsg_priv_s *priv = dev->priv;

  if (priv->ept.priv != NULL &&
      strcmp(priv->cpuname, rpmsg_get_cpuname(rdev)) == 0)
    {
      rpmsg_destroy_ept(&priv->ept);
    }

  dev->dmatx.nbytes = dev->dmatx.length + dev->dmatx.nlength;
  uart_xmitchars_done(dev);
}

static int uart_rpmsg_raw_ept_cb(FAR struct rpmsg_endpoint *ept,
                                 FAR void *data, size_t len, uint32_t src,
                                 FAR void *priv_)
{
  FAR struct uart_dev_s *dev = priv_;
  FAR struct uart_rpmsg_priv_s *priv = dev->priv;

  if (len > 0)
    {
      nxmutex_lock(&dev->recv.lock);

      priv->recv_data = data;
      priv->recv_len = len;

      uart_recvchars_dma(dev);

      priv->recv_data = NULL;
      priv->recv_len = 0;

      nxmutex_unlock(&dev->recv.lock);
    }

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int uart_rpmsg_raw_init(FAR const char *cpuname, FAR const char *devname,
                        int buf_size, bool isconsole)
{
  FAR struct uart_rpmsg_priv_s *priv;
  FAR struct uart_dev_s *dev;
  char name[32];
  int ret;

  dev = kmm_zalloc(sizeof(struct uart_dev_s) +
                   sizeof(struct uart_rpmsg_priv_s) +
                   buf_size * 2);
  if (dev == NULL)
    {
      return -ENOMEM;
    }

  dev->ops         = &g_uart_rpmsg_ops;
  dev->isconsole   = isconsole;
  dev->recv.size   = buf_size;
  dev->xmit.size   = buf_size;
  dev->priv        = dev + 1;
  dev->recv.buffer = (FAR char *)dev->priv +
                     sizeof(struct uart_rpmsg_priv_s);
  dev->xmit.buffer = dev->recv.buffer + buf_size;
  priv             = dev->priv;
  priv->cpuname    = cpuname;
  priv->devname    = devname;

  nxmutex_init(&priv->lock);

  ret = rpmsg_register_callback(dev,
                                uart_rpmsg_raw_device_created,
                                uart_rpmsg_raw_device_destroy,
                                NULL,
                                NULL);
  if (ret < 0)
    {
      nxmutex_destroy(&priv->lock);
      kmm_free(dev);
      return ret;
    }

  snprintf(name, sizeof(name), "%s%s",
           UART_RPMSG_DEV_PREFIX, devname);
  uart_register(name, dev);

#ifdef CONFIG_RPMSG_UART_RAW_CONSOLE
  if (dev->isconsole)
    {
      uart_register(UART_RPMSG_DEV_CONSOLE, dev);
    }
#endif

  return 0;
}
