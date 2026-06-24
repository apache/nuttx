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
#include <limits.h>
#include <stdio.h>
#include <string.h>
#include <sys/param.h>

#include <nuttx/debug.h>
#include <nuttx/irq.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/kmalloc.h>
#include <nuttx/semaphore.h>
#include <nuttx/spinlock.h>
#include <nuttx/rpmsg/rpmsg.h>
#include <nuttx/serial/serial.h>
#include <nuttx/serial/uart_rpmsg.h>

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/

#define UART_RPMSG_DEV_CONSOLE          "/dev/console"
#define UART_RPMSG_DEV_PREFIX           "/dev/tty"
#define UART_RPMSG_EPT_PREFIX           "rpmsg-tty"
#define UART_RPMSG_NAME_SIZE            (RPMSG_NAME_SIZE - 10)

#define UART_RPMSG_SYNC                 0  /* Handshake: pos == recv cap */
#define UART_RPMSG_DATA                 1  /* Payload and/or position sync */

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Counter-based flow control protocol.  Each frame carries an explicit
 * command:
 *
 *   - SYNC : sent once after ns_bound, len == 0, pos == local recv
 *            capacity (recv.size - 1).  Establishes the peer send window.
 *   - DATA : pos always piggybacks the sender's consumed position
 *            (recvpos) to acknowledge the reverse stream.  len > 0 carries
 *            payload; len == 0 is a pure position sync (an ack with no
 *            payload).
 */

begin_packed_struct struct uart_rpmsg_msg_s
{
  uint32_t cmd;                  /* UART_RPMSG_SYNC / DATA */
  uint32_t pos;                  /* recvpos (piggyback ack); for SYNC:
                                  * local recv buffer capacity */
  uint32_t len;                  /* Payload length; 0 means position-only */
  char     data[0];
} end_packed_struct;

struct uart_rpmsg_priv_s
{
  struct rpmsg_endpoint ept;
  char                  devname[UART_RPMSG_NAME_SIZE];
  char                  cpuname[UART_RPMSG_NAME_SIZE];
  FAR void              *recv_data;     /* Bridge ept_cb -> dmareceive */

  uint32_t              sendsize;       /* Peer recv capacity; 0 means not
                                         * connected */
  uint32_t              sendpos;        /* Cumulative bytes we have sent */
  uint32_t              ackpos;         /* Cumulative bytes the peer consumed */
  uint32_t              received;       /* Cumulative bytes written to recv */
  uint32_t              lastpos;        /* recvpos at the last ack we sent */
  sem_t                 sendsem;        /* dmasend waits, ept_cb posts */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  uart_rpmsg_setup(FAR struct uart_dev_s *dev);
static void uart_rpmsg_shutdown(FAR struct uart_dev_s *dev);
static int  uart_rpmsg_attach(FAR struct uart_dev_s *dev);
static void uart_rpmsg_detach(FAR struct uart_dev_s *dev);
static void uart_rpmsg_rxint(FAR struct uart_dev_s *dev, bool enable);
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
static int  uart_rpmsg_ept_cb(FAR struct rpmsg_endpoint *ept, FAR void *data,
                              size_t len, uint32_t src, FAR void *priv_);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct uart_ops_s g_uart_rpmsg_ops =
{
  .setup         = uart_rpmsg_setup,
  .shutdown      = uart_rpmsg_shutdown,
  .attach        = uart_rpmsg_attach,
  .detach        = uart_rpmsg_detach,
  .rxint         = uart_rpmsg_rxint,
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

/* Post the send semaphore without letting its count grow unbounded. */

static void uart_rpmsg_post(FAR sem_t *sem)
{
  int semcount;

  nxsem_get_value(sem, &semcount);
  if (semcount < 1)
    {
      nxsem_post(sem);
    }
}

/* Cumulative number of bytes the application has consumed from the recv
 * ring.  recvpos = bytes received - bytes still buffered.  Must be called
 * with dev->lock held.
 */

static inline_function
uint32_t uart_rpmsg_get_recvpos(FAR struct uart_dev_s *dev)
{
  FAR struct uart_rpmsg_priv_s *priv = dev->priv;
  sbuf_size_t head = dev->recv.head;
  sbuf_size_t tail = dev->recv.tail;
  sbuf_size_t nbuffered;

  nbuffered = head >= tail ? head - tail : dev->recv.size - tail + head;
  return priv->received - nbuffered;
}

/* Bytes we may still send before the peer recv ring is full.  Returns 0 when
 * not connected: sendsize, sendpos and ackpos are all reset to 0 on
 * disconnect, so the subtraction naturally yields 0.  Must be called with
 * dev->lock held.
 */

static inline_function
uint32_t uart_rpmsg_get_space(FAR struct uart_rpmsg_priv_s *priv)
{
  return priv->sendsize - (priv->sendpos - priv->ackpos);
}

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

static void uart_rpmsg_dmasend(FAR struct uart_dev_s *dev)
{
  FAR struct uart_rpmsg_priv_s *priv = dev->priv;
  FAR struct uart_dmaxfer_s *xfer = &dev->dmatx;
  FAR struct uart_rpmsg_msg_s *msg;
  size_t total = xfer->length + xfer->nlength;
  size_t sent = 0;

  /* Send the whole transfer, blocking on the send semaphore whenever the
   * peer window is exhausted.  ept_cb posts the semaphore when the peer
   * advances its consumed position (ackpos) or the handshake completes.
   * Runs in the writer thread, so blocking here is allowed.
   */

  while (sent < total)
    {
      uint32_t space;
      uint16_t count;
      size_t len;

      while (uart_rpmsg_get_space(priv) == 0)
        {
          /* Window full or not connected yet.  Release the recursive lock
           * and wait until ept_cb opens the window.  Bail out if the
           * endpoint is torn down while we wait.
           */

          count = rspin_breaklock(&dev->lock);
          rpmsg_wait(&priv->ept, &priv->sendsem);
          rspin_restorelock(&dev->lock, count);

          if (!is_rpmsg_ept_ready(&priv->ept))
            {
              goto out;
            }
        }

      /* Getting a TX buffer may block, so release the lock first */

      count = rspin_breaklock(&dev->lock);
      msg = rpmsg_get_tx_payload_buffer(&priv->ept, &space, true);
      rspin_restorelock(&dev->lock, count);

      if (msg == NULL)
        {
          break;
        }

      space -= sizeof(*msg);
      len = MIN3(total - sent, uart_rpmsg_get_space(priv), space);

      /* Copy from the (possibly wrapped) xmit ring at logical offset sent */

      if (sent < xfer->length)
        {
          size_t first = xfer->length - sent;

          if (first >= len)
            {
              memcpy(msg->data, xfer->buffer + sent, len);
            }
          else
            {
              memcpy(msg->data, xfer->buffer + sent, first);
              memcpy(msg->data + first, xfer->nbuffer, len - first);
            }
        }
      else
        {
          memcpy(msg->data, xfer->nbuffer + (sent - xfer->length), len);
        }

      /* Piggyback our consumed position so the peer can free its window */

      msg->cmd = UART_RPMSG_DATA;
      msg->pos = uart_rpmsg_get_recvpos(dev);
      msg->len = len;

      if (rpmsg_send_nocopy(&priv->ept, msg, sizeof(*msg) + len) < 0)
        {
          rpmsg_release_tx_buffer(&priv->ept, msg);
          break;
        }

      /* Advance the position counters only after the frame is on the wire */

      priv->lastpos  = msg->pos;
      priv->sendpos += len;
      sent          += len;
    }

out:
  dev->dmatx.nbytes = sent;
  uart_xmitchars_done(dev);
}

static void uart_rpmsg_dmareceive(FAR struct uart_dev_s *dev)
{
  FAR struct uart_rpmsg_priv_s *priv = dev->priv;
  FAR struct uart_dmaxfer_s *xfer = &dev->dmarx;
  FAR struct uart_rpmsg_msg_s *msg = priv->recv_data;
  uint32_t len = msg->len;
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
}

static void uart_rpmsg_dmarxfree(FAR struct uart_dev_s *dev)
{
  FAR struct uart_rpmsg_priv_s *priv = dev->priv;
  FAR struct uart_rpmsg_msg_s msg;
  uint32_t recvpos;
  uint16_t count;
  int ret;

  /* Return credit to the peer based on what the reader has consumed.
   * Only emit an ACK once at least half of the recv buffer has been freed,
   * to limit the ACK rate.  Runs in thread (uart_read) context.
   */

  recvpos = uart_rpmsg_get_recvpos(dev);
  if (is_rpmsg_ept_ready(&priv->ept) &&
      recvpos - priv->lastpos > dev->recv.size / 2)
    {
      msg.cmd = UART_RPMSG_DATA;
      msg.pos = recvpos;
      msg.len = 0;

      /* rpmsg_send may block waiting for a TX buffer, so release the
       * recursive lock first (msg is a stack copy and stays valid).
       */

      count = rspin_breaklock(&dev->lock);
      ret = rpmsg_send(&priv->ept, &msg, sizeof(msg));
      rspin_restorelock(&dev->lock, count);

      /* Advance lastpos only after the ack is on the wire; otherwise a
       * failed send would suppress the next ack and stall the peer window.
       * Never let it regress: dmasend may have advanced lastpos
       * (piggybacking a newer recvpos) while the lock was released for
       * rpmsg_send.
       */

      if (ret >= 0 && (int32_t)(recvpos - priv->lastpos) > 0)
        {
          priv->lastpos = recvpos;
        }
    }
}

static void uart_rpmsg_dmatxavail(FAR struct uart_dev_s *dev)
{
  FAR struct uart_rpmsg_priv_s *priv = dev->priv;

  /* Caller must hold dev->lock.  Kick a transfer once the endpoint is ready
   * and no DMA is already in flight.  dmasend itself blocks on sendsem if
   * the peer window is not yet open.
   */

  if (is_rpmsg_ept_ready(&priv->ept) && dev->dmatx.length == 0)
    {
      uart_xmitchars_dma(dev);
    }
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

static void uart_rpmsg_ns_bound(FAR struct rpmsg_endpoint *ept)
{
  FAR struct uart_dev_s *dev = ept->priv;
  FAR struct uart_rpmsg_msg_s msg;

  /* SYNC: announce our recv ring capacity so the peer can size its send
   * window.  The serial recv ring can hold at most (size - 1) bytes.
   */

  msg.cmd = UART_RPMSG_SYNC;
  msg.pos = dev->recv.size - 1;
  msg.len = 0;

  rpmsg_send(ept, &msg, sizeof(msg));
}

static void uart_rpmsg_device_created(FAR struct rpmsg_device *rdev,
                                      FAR void *priv_)
{
  FAR struct uart_dev_s *dev = priv_;
  FAR struct uart_rpmsg_priv_s *priv = dev->priv;
  char eptname[RPMSG_NAME_SIZE];

  if (strcmp(priv->cpuname, rpmsg_get_cpuname(rdev)) == 0)
    {
      /* Create the ept at default priority so the first callback (the peer
       * SYNC) runs in the rpmsg work-queue thread, where dmasend may block
       * to flush data queued before the link came up.  ept_cb raises it to
       * RT once the initial backlog is drained.
       */

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

  if (priv->ept.priv != NULL &&
      strcmp(priv->cpuname, rpmsg_get_cpuname(rdev)) == 0)
    {
      /* Mark not connected and wake a possibly blocked dmasend, which will
       * see the ept is no longer ready and abort, before tearing it down.
       * Reset all window counters so uart_rpmsg_get_space() returns 0 while
       * disconnected.
       */

      priv->sendsize = 0;
      priv->sendpos  = 0;
      priv->ackpos   = 0;
      rpmsg_destroy_ept(&priv->ept);
      uart_rpmsg_post(&priv->sendsem);
    }

  dev->dmatx.nbytes = dev->dmatx.length + dev->dmatx.nlength;
  uart_xmitchars_done(dev);
}

static int uart_rpmsg_ept_cb(FAR struct rpmsg_endpoint *ept, FAR void *data,
                             size_t len, uint32_t src, FAR void *priv_)
{
  FAR struct uart_dev_s *dev = ept->priv;
  FAR struct uart_rpmsg_priv_s *priv = dev->priv;
  FAR struct uart_rpmsg_msg_s *msg = data;
  irqstate_t flags;

  flags = uart_spinlock(dev, true);

  if (msg->cmd == UART_RPMSG_SYNC)
    {
      /* Handshake: record the peer recv capacity as our send window and
       * reset the position counters.
       *
       * This first callback runs in the rpmsg work-queue thread (the ept is
       * created at default priority), so we can flush inline any data that
       * was queued into the xmit ring before the link came up: dmasend may
       * block on rpmsg_wait, whose recursive dispatch keeps servicing acks
       * on the same work queue.  Once the backlog is drained, raise the ept
       * to RT priority so steady-state tty traffic is dispatched
       * synchronously with low latency.
       */

      priv->sendsize = msg->pos;
      priv->sendpos  = 0;
      priv->ackpos   = 0;

      uart_rpmsg_dmatxavail(dev);
      rpmsg_set_priority(&priv->ept, RPMSG_PRIO_RT);
    }
  else
    {
      /* DATA: the peer always piggybacks its consumed position (our
       * ackpos).  len > 0 also carries payload for the recv ring; len == 0
       * is a pure position sync.
       */

      priv->ackpos = msg->pos;

      if (msg->len > 0)
        {
          priv->recv_data  = data;
          uart_recvchars_dma(dev);
          priv->recv_data  = NULL;
          priv->received  += msg->len;
        }
    }

  /* Wake a blocked dmasend only when the send window is actually open.
   * dmasend blocks solely because get_space() == 0, and the only way the
   * window opens is here (SYNC sets sendsize, DATA advances ackpos), so this
   * avoids spurious wakeups that would just re-check and sleep again.  The
   * shared dev->lock serializes this evaluation with dmasend's own check, so
   * no wakeup is lost.
   */

  if (uart_rpmsg_get_space(priv) > 0)
    {
      uart_rpmsg_post(&priv->sendsem);
    }

  uart_spinunlock(dev, true, flags);
  return 0;
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

  nxsem_init(&priv->sendsem, 0, 0);

  strlcpy(priv->cpuname, cpuname, sizeof(priv->cpuname));
  strlcpy(priv->devname, devname, sizeof(priv->devname));

  ret = rpmsg_register_callback(dev,
                                uart_rpmsg_device_created,
                                uart_rpmsg_device_destroy,
                                NULL,
                                NULL);
  if (ret < 0)
    {
      nxsem_destroy(&priv->sendsem);
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
  nxsem_destroy(&priv->sendsem);
  kmm_free(dev);
  return ret;
}
