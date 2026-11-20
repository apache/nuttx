/****************************************************************************
 * drivers/serial/uart_rpmsg.c
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

#include <nuttx/debug.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/clock.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/rpmsg/rpmsg.h>
#include <nuttx/serial/serial.h>
#include <nuttx/serial/uart_rpmsg.h>
#include <nuttx/wqueue.h>

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/

#define UART_RPMSG_DEV_CONSOLE          "/dev/console"
#define UART_RPMSG_DEV_PREFIX           "/dev/tty"
#define UART_RPMSG_EPT_PREFIX           "rpmsg-tty"
#define UART_RPMSG_NAME_SIZE            (RPMSG_NAME_SIZE - 10)

#define UART_RPMSG_TTY_WRITE            0
#define UART_RPMSG_TTY_WAKEUP           1

/* Retry delay when no TX buffer is free */

#define UART_RPMSG_TXRETRY              MSEC2TICK(10)

/****************************************************************************
 * Private Types
 ****************************************************************************/

begin_packed_struct struct uart_rpmsg_header_s
{
  uint32_t command : 31;
  uint32_t response : 1;
  int32_t  result;
  uint64_t cookie;
} end_packed_struct;

begin_packed_struct struct uart_rpmsg_write_s
{
  struct uart_rpmsg_header_s header;
  uint32_t                   count;
  uint32_t                   resolved;
  char                       data[0];
} end_packed_struct;

begin_packed_struct struct uart_rpmsg_wakeup_s
{
  struct uart_rpmsg_header_s header;
} end_packed_struct;

struct uart_rpmsg_priv_s
{
  struct rpmsg_endpoint ept;
  struct work_s         txwork;
  struct work_s         wakework;
  mutex_t               lock;
  char                  devname[UART_RPMSG_NAME_SIZE];
  char                  cpuname[UART_RPMSG_NAME_SIZE];
  FAR void              *recv_data;
  bool                  last_upper;
  bool                  wakeup;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  uart_rpmsg_setup(FAR struct uart_dev_s *dev);
static void uart_rpmsg_shutdown(FAR struct uart_dev_s *dev);
static int  uart_rpmsg_attach(FAR struct uart_dev_s *dev);
static void uart_rpmsg_detach(FAR struct uart_dev_s *dev);
static void uart_rpmsg_rxint(FAR struct uart_dev_s *dev, bool enable);
static bool uart_rpmsg_rxflowcontrol(FAR struct uart_dev_s *dev,
                                     unsigned int nbuffered, bool upper);
static void uart_rpmsg_dmasend(FAR struct uart_dev_s *dev);
static void uart_rpmsg_dmareceive(FAR struct uart_dev_s *dev);
static void uart_rpmsg_dmarxfree(FAR struct uart_dev_s *dev);
static void uart_rpmsg_dmatxavail(FAR struct uart_dev_s *dev);
static void uart_rpmsg_send(FAR struct uart_dev_s *dev, int ch);
static void uart_rpmsg_txint(FAR struct uart_dev_s *dev, bool enable);
static bool uart_rpmsg_txready(FAR struct uart_dev_s *dev);
static bool uart_rpmsg_txempty(FAR struct uart_dev_s *dev);
static void uart_rpmsg_device_created(FAR struct rpmsg_device *rdev,
                                      FAR void *priv_);
static void uart_rpmsg_device_destroy(FAR struct rpmsg_device *rdev,
                                      FAR void *priv_);
static int  uart_rpmsg_ept_cb(struct rpmsg_endpoint *ept, FAR void *data,
                              size_t len, uint32_t src, FAR void *priv_);
static void uart_rpmsg_txwork(FAR void *arg);
static void uart_rpmsg_wakework(FAR void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Work queue that sends to the remote, shared by all devices */

static FAR struct kwork_wqueue_s *g_uart_rpmsg_wqueue;

static const struct uart_ops_s g_uart_rpmsg_ops =
{
  .setup         = uart_rpmsg_setup,
  .shutdown      = uart_rpmsg_shutdown,
  .attach        = uart_rpmsg_attach,
  .detach        = uart_rpmsg_detach,
  .rxint         = uart_rpmsg_rxint,
  .rxflowcontrol = uart_rpmsg_rxflowcontrol,
  .dmasend       = uart_rpmsg_dmasend,
  .dmareceive    = uart_rpmsg_dmareceive,
  .dmarxfree     = uart_rpmsg_dmarxfree,
  .dmatxavail    = uart_rpmsg_dmatxavail,
  .send          = uart_rpmsg_send,
  .txint         = uart_rpmsg_txint,
  .txready       = uart_rpmsg_txready,
  .txempty       = uart_rpmsg_txempty,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int uart_rpmsg_setup(FAR struct uart_dev_s *dev)
{
  return OK;
}

static void uart_rpmsg_shutdown(FAR struct uart_dev_s *dev)
{
}

static int uart_rpmsg_attach(FAR struct uart_dev_s *dev)
{
  return OK;
}

static void uart_rpmsg_detach(FAR struct uart_dev_s *dev)
{
}

static void uart_rpmsg_rxint(FAR struct uart_dev_s *dev, bool enable)
{
}

static bool uart_rpmsg_rxflowcontrol(FAR struct uart_dev_s *dev,
                                     unsigned int nbuffered, bool upper)
{
  FAR struct uart_rpmsg_priv_s *priv = dev->priv;

  /* This may be called with the uart spinlock held, so the wakeup message,
   * whose send may block, is sent by uart_rpmsg_wakework().
   */

  if (!upper && upper != priv->last_upper)
    {
      priv->wakeup = true;
      work_queue_wq(g_uart_rpmsg_wqueue, &priv->wakework,
                    uart_rpmsg_wakework, dev, 0);
    }

  priv->last_upper = upper;
  return false;
}

static void uart_rpmsg_dmasend(FAR struct uart_dev_s *dev)
{
  FAR struct uart_rpmsg_priv_s *priv = dev->priv;

  /* Called with the uart spinlock held, so the send, which may block, is
   * done by uart_rpmsg_txwork().
   */

  work_queue_wq(g_uart_rpmsg_wqueue, &priv->txwork, uart_rpmsg_txwork,
                dev, 0);
}

static void uart_rpmsg_dmareceive(FAR struct uart_dev_s *dev)
{
  FAR struct uart_rpmsg_priv_s *priv = dev->priv;
  FAR struct uart_dmaxfer_s *xfer = &dev->dmarx;
  FAR struct uart_rpmsg_write_s *msg = priv->recv_data;
  uint32_t len = msg->count;
  size_t space = xfer->length + xfer->nlength;

  if (len > space)
    {
      len = space;
    }

  if (len > xfer->length)
    {
      memcpy(xfer->buffer, msg->data, xfer->length);
      memcpy(xfer->nbuffer, msg->data + xfer->length, len - xfer->length);
    }
  else
    {
      memcpy(xfer->buffer, msg->data, len);
    }

  xfer->nbytes = len;
  uart_recvchars_done(dev);

  msg->header.result = len;

  if (len != msg->count)
    {
      uart_rpmsg_rxflowcontrol(dev, 0, true);
    }
}

static void uart_rpmsg_dmarxfree(FAR struct uart_dev_s *dev)
{
}

static void uart_rpmsg_dmatxavail(FAR struct uart_dev_s *dev)
{
  FAR struct uart_rpmsg_priv_s *priv = dev->priv;

  /* Called with the uart spinlock held.  Start a new transfer only when
   * the previous one is done.
   */

  if (is_rpmsg_ept_ready(&priv->ept) && dev->dmatx.length == 0)
    {
      uart_xmitchars_dma(dev);
    }
}

/****************************************************************************
 * Name: uart_rpmsg_txavail
 *
 * Description:
 *   Start a transfer from a context that does not hold the uart spinlock.
 *
 ****************************************************************************/

static void uart_rpmsg_txavail(FAR struct uart_dev_s *dev)
{
  irqstate_t flags = uart_spinlock(dev, true);

  uart_rpmsg_dmatxavail(dev);
  uart_spinunlock(dev, true, flags);
}

static void uart_rpmsg_send(FAR struct uart_dev_s *dev, int ch)
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

  uart_rpmsg_dmatxavail(dev);
}

static void uart_rpmsg_txint(FAR struct uart_dev_s *dev, bool enable)
{
}

static bool uart_rpmsg_txready(FAR struct uart_dev_s *dev)
{
  int nexthead;

  nexthead = dev->xmit.head + 1;
  if (nexthead >= dev->xmit.size)
    {
      nexthead = 0;
    }

  return nexthead != dev->xmit.tail;
}

static bool uart_rpmsg_txempty(FAR struct uart_dev_s *dev)
{
  return true;
}

static void uart_rpmsg_ns_bound(struct rpmsg_endpoint *ept)
{
  FAR struct uart_dev_s *dev = ept->priv;

  uart_rpmsg_txavail(dev);
}

static void uart_rpmsg_device_created(FAR struct rpmsg_device *rdev,
                                      FAR void *priv_)
{
  FAR struct uart_dev_s *dev = priv_;
  FAR struct uart_rpmsg_priv_s *priv = dev->priv;
  char eptname[RPMSG_NAME_SIZE];

  if (strcmp(priv->cpuname, rpmsg_get_cpuname(rdev)) == 0)
    {
      priv->ept.priv = dev;
      priv->ept.ns_bound_cb = uart_rpmsg_ns_bound;
      snprintf(eptname, sizeof(eptname), "%s%s",
               UART_RPMSG_EPT_PREFIX, priv->devname);
      rpmsg_create_ept(&priv->ept, rdev, eptname,
                       RPMSG_ADDR_ANY, RPMSG_ADDR_ANY,
                       uart_rpmsg_ept_cb, NULL);
    }
}

static void uart_rpmsg_device_destroy(FAR struct rpmsg_device *rdev,
                                      FAR void *priv_)
{
  FAR struct uart_dev_s *dev = priv_;
  FAR struct uart_rpmsg_priv_s *priv = dev->priv;

  irqstate_t flags;

  /* Wait for the works to finish with the endpoint */

  nxmutex_lock(&priv->lock);

  if (priv->ept.priv != NULL &&
      strcmp(priv->cpuname, rpmsg_get_cpuname(rdev)) == 0)
    {
      rpmsg_destroy_ept(&priv->ept);
    }

  nxmutex_unlock(&priv->lock);

  /* Drop the transfer in progress */

  flags = uart_spinlock(dev, true);
  dev->dmatx.nbytes = dev->dmatx.length + dev->dmatx.nlength;
  uart_xmitchars_done(dev);
  uart_spinunlock(dev, true, flags);
}

static int uart_rpmsg_ept_cb(FAR struct rpmsg_endpoint *ept, FAR void *data,
                             size_t len, uint32_t src, FAR void *priv_)
{
  FAR struct uart_dev_s *dev = priv_;
  FAR struct uart_rpmsg_priv_s *priv = dev->priv;
  FAR struct uart_rpmsg_header_s *header = data;
  FAR struct uart_rpmsg_write_s *msg = data;

  if (header->response)
    {
      /* Get write-cmd response, this tell how many data have sent */

      irqstate_t flags = uart_spinlock(dev, true);

      dev->dmatx.nbytes = header->result;
      if (header->result < 0)
        {
          dev->dmatx.nbytes = 0;
        }

      uart_xmitchars_done(dev);

      /* If have sent some data succeed, then continue send */

      if (msg->count == header->result)
        {
          uart_rpmsg_dmatxavail(dev);
        }

      uart_spinunlock(dev, true, flags);
    }
  else if (header->command == UART_RPMSG_TTY_WRITE)
    {
      /* Get write-cmd, there are some data, we need receive them */

      nxmutex_lock(&dev->recv.lock);
      priv->recv_data = data;
      uart_recvchars_dma(dev);
      priv->recv_data = NULL;
      nxmutex_unlock(&dev->recv.lock);

      header->response = 1;
      rpmsg_send(ept, msg, sizeof(*msg));
    }
  else if (header->command == UART_RPMSG_TTY_WAKEUP)
    {
      /* Remote core have space, then wakeup current core, continue send */

      uart_rpmsg_txavail(dev);
    }

  return 0;
}

/****************************************************************************
 * Name: uart_rpmsg_txwork
 *
 * Description:
 *   Send the buffer prepared by uart_xmitchars_dma().  This runs without
 *   the uart spinlock, so getting a TX buffer and sending may block.  The
 *   transfer completes when the remote answers, in uart_rpmsg_ept_cb().
 *
 ****************************************************************************/

static void uart_rpmsg_txwork(FAR void *arg)
{
  FAR struct uart_dev_s *dev = arg;
  FAR struct uart_rpmsg_priv_s *priv = dev->priv;
  FAR struct uart_dmaxfer_s *xfer = &dev->dmatx;
  FAR struct uart_rpmsg_write_s *msg;
  irqstate_t flags;
  uint32_t space;
  size_t len;
  int ret = -ENOTCONN;

  nxmutex_lock(&priv->lock);

  /* Nothing to send when the transfer was dropped while queued */

  flags = uart_spinlock(dev, true);
  len = xfer->length + xfer->nlength;
  uart_spinunlock(dev, true, flags);

  if (len == 0)
    {
      nxmutex_unlock(&priv->lock);
      return;
    }

  if (is_rpmsg_ept_ready(&priv->ept))
    {
      /* Do not wait for a TX buffer here: that would hold up the other
       * works.  Try again later instead.
       */

      msg = rpmsg_get_tx_payload_buffer(&priv->ept, &space, false);
      if (msg == NULL)
        {
          nxmutex_unlock(&priv->lock);
          work_queue_wq(g_uart_rpmsg_wqueue, &priv->txwork,
                        uart_rpmsg_txwork, dev, UART_RPMSG_TXRETRY);
          return;
        }

      memset(msg, 0, sizeof(*msg));

      space = space - sizeof(*msg);

      if (len > space)
        {
          len = space;
        }

      if (len > xfer->length)
        {
          memcpy(msg->data, xfer->buffer, xfer->length);
          memcpy(msg->data + xfer->length, xfer->nbuffer,
                 len - xfer->length);
        }
      else
        {
          memcpy(msg->data, xfer->buffer, len);
        }

      msg->count          = len;
      msg->header.command = UART_RPMSG_TTY_WRITE;
      msg->header.result  = -ENXIO;
      msg->header.cookie  = (uintptr_t)dev;

      ret = rpmsg_send_nocopy(&priv->ept, msg, sizeof(*msg) + len);
      if (ret < 0)
        {
          rpmsg_release_tx_buffer(&priv->ept, msg);
        }
    }

  nxmutex_unlock(&priv->lock);

  /* No answer will come: end the transfer and keep the data in the xmit
   * buffer for the next write.
   */

  if (ret < 0)
    {
      flags = uart_spinlock(dev, true);
      xfer->nbytes = 0;
      uart_xmitchars_done(dev);
      uart_spinunlock(dev, true, flags);
    }
}

/****************************************************************************
 * Name: uart_rpmsg_wakework
 *
 * Description:
 *   Tell the remote that there is free space in the receive buffer.
 *
 ****************************************************************************/

static void uart_rpmsg_wakework(FAR void *arg)
{
  FAR struct uart_dev_s *dev = arg;
  FAR struct uart_rpmsg_priv_s *priv = dev->priv;
  struct uart_rpmsg_wakeup_s msg;

  nxmutex_lock(&priv->lock);

  if (priv->wakeup)
    {
      priv->wakeup = false;
      memset(&msg, 0, sizeof(msg));
      msg.header.command = UART_RPMSG_TTY_WAKEUP;
      if (is_rpmsg_ept_ready(&priv->ept))
        {
          rpmsg_send(&priv->ept, &msg, sizeof(msg));
        }
    }

  nxmutex_unlock(&priv->lock);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int uart_rpmsg_init(FAR const char *cpuname, FAR const char *devname,
                    int buf_size, bool isconsole)
{
  FAR struct uart_rpmsg_priv_s *priv;
  FAR struct uart_dev_s *dev;
  char name[32];
  int ret;

  if (g_uart_rpmsg_wqueue == NULL)
    {
      g_uart_rpmsg_wqueue =
        work_queue_create("uart_rpmsg", CONFIG_RPMSG_WQUEUE_PRIORITY, NULL,
                          CONFIG_RPMSG_WQUEUE_STACKSIZE, 1);
      if (g_uart_rpmsg_wqueue == NULL)
        {
          return -ENOMEM;
        }
    }

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
  strlcpy(priv->cpuname, cpuname, sizeof(priv->cpuname));
  strlcpy(priv->devname, devname, sizeof(priv->devname));

  nxmutex_init(&priv->lock);

  ret = rpmsg_register_callback(dev,
                                uart_rpmsg_device_created,
                                uart_rpmsg_device_destroy,
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
  ret = uart_register(name, dev);
  if (ret < 0)
    {
      rpmsgerr("uart register failed, ret=%d\n", ret);
      goto fail_with_rpmsg;
    }

  if (dev->isconsole)
    {
      ret = uart_register(UART_RPMSG_DEV_CONSOLE, dev);
      if (ret < 0)
        {
          rpmsgerr("uart register console failed, ret=%d\n", ret);
          unregister_driver(name);
          goto fail_with_rpmsg;
        }
    }

  return OK;

fail_with_rpmsg:
  rpmsg_unregister_callback(dev,
                            uart_rpmsg_device_created,
                            uart_rpmsg_device_destroy,
                            NULL,
                            NULL);
  nxmutex_destroy(&priv->lock);
  kmm_free(dev);
  return ret;
}
