/****************************************************************************
 * drivers/mbox/mbox.c
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

#include <debug.h>
#include <errno.h>
#include <stdbool.h>
#include <string.h>
#include <sys/types.h>

#include <nuttx/clock.h>
#include <nuttx/irq.h>
#include <nuttx/mbox/mbox.h>
#include <nuttx/semaphore.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: mbox_enqueue_data
 *
 * Description:
 *   Enqueue the message data waiting to be sent to circbuf
 *   This is a buffering operation for non-blocking send.
 ****************************************************************************/

static int mbox_enqueue_data(FAR struct mbox_chan_s *chan,
                             FAR const void *data, size_t size);

/****************************************************************************
 * Name: mbox_dequeue_data
 *
 * Description:
 *   Dequeue the message data from circbuf to do actual send
 *   This is a buffering operation for non-blocking send.
 ****************************************************************************/

static int mbox_dequeue_data(FAR struct mbox_chan_s *chan, FAR void *data,
                             FAR size_t *size);

/****************************************************************************
 * Name: mbox_send_data
 *
 * Description:
 *   Submit data to the lower half and mark this channel as active.
 ****************************************************************************/

static int mbox_send_data(FAR struct mbox_chan_s *chan,
                          FAR const void *data, size_t size,
                          clock_t timeout);

/****************************************************************************
 * Name: submit_msg
 *
 * Description:
 *   Drain a message from circbuf of specific mbox channel, then call
 *   lower half ops->send() to perform actual message transmission.
 ****************************************************************************/

static int submit_msg(FAR struct mbox_chan_s *chan);

/****************************************************************************
 * Name: tx_expiry
 *
 * Description:
 *   Called when tx timer expires
 ****************************************************************************/

static void tx_expiry(wdparm_t arg);

/****************************************************************************
 * Name: notify_txdone
 *
 * Description:
 *   Notify that the mbox channel has completed TX.
 ****************************************************************************/

static void notify_txdone(FAR struct mbox_chan_s *chan);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int mbox_enqueue_data(FAR struct mbox_chan_s *chan,
                             FAR const void *data, size_t size)
{
  if (size > MBOX_MAX_MSG_SIZE)
    {
      return -EMSGSIZE;
    }

  if (circbuf_space(&chan->txbuf) < size + sizeof(size))
    {
      return -ENOBUFS;
    }

  circbuf_write(&chan->txbuf, (FAR void *)&size, sizeof(size_t));
  circbuf_write(&chan->txbuf, (FAR void *)data, size);

#ifdef CONFIG_MBOX_TRACE
  ++chan->stats.buffered;
#endif
  return 0;
}

static int mbox_dequeue_data(FAR struct mbox_chan_s *chan, FAR void *data,
                             FAR size_t *size)
{
  if (circbuf_is_empty(&chan->txbuf))
    {
      return -ENODATA;
    }

  circbuf_read(&chan->txbuf, (FAR void *)size, sizeof(size_t));
  circbuf_read(&chan->txbuf, (FAR void *)data, *size);

#ifdef CONFIG_MBOX_TRACE
  --chan->stats.buffered;
#endif
  return 0;
}

static int mbox_send_data(FAR struct mbox_chan_s *chan,
                          FAR const void *data, size_t size,
                          clock_t timeout)
{
  int ret;

  ret = chan->dev->ops->send(chan, data, size);
  if (ret < 0)
    {
      return ret;
    }

  chan->txstate = MBOX_TX_ACTIVE;
  chan->timeout = timeout;

  if (timeout != 0 && timeout != MBOX_WAIT_FOREVER)
    {
      wd_start(&chan->timer, timeout, tx_expiry, (wdparm_t)chan);
    }

#ifdef CONFIG_MBOX_TRACE
  ++chan->stats.sent;
#endif
  return OK;
}

static int submit_msg(FAR struct mbox_chan_s *chan)
{
  uint8_t data[MBOX_MAX_MSG_SIZE];
  size_t size = 0;
  int ret;

  if (chan->txstate != MBOX_TX_IDLE)
    {
      return -EBUSY;
    }

  ret = mbox_dequeue_data(chan, data, &size);
  if (ret < 0)
    {
      return ret;
    }

  return mbox_send_data(chan, data, size, MBOX_WAIT_FOREVER);
}

static void notify_txdone(FAR struct mbox_chan_s *chan)
{
  if (chan->txwaiting)
    {
      chan->txwaiting = false;
      nxsem_post(&chan->txsem);
    }
}

static void tx_expiry(wdparm_t arg)
{
  FAR struct mbox_chan_s *chan = (FAR struct mbox_chan_s *)arg;
  irqstate_t flags;

  flags = enter_critical_section();

  if (chan->txstate != MBOX_TX_ACTIVE)
    {
      leave_critical_section(flags);
      return;
    }

#ifdef CONFIG_MBOX_TRACE
  chan->stats.timeout++;
#endif

  /* txfinish is expected to clear any pending hardware completion state
   * before the upper half reuses this channel.
   */

  if (chan->dev->ops->txfinish)
    {
      chan->dev->ops->txfinish(chan);
    }

  chan->txstate = MBOX_TX_IDLE;
  chan->txresult = -ETIMEDOUT;
  notify_txdone(chan);

  if (!circbuf_is_empty(&chan->txbuf))
    {
      submit_msg(chan);
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mbox_chan_tx_done
 *
 * Description:
 *   This function should be called upon receiving an ACK/completion (when
 *   triggered by an interrupt or when polling detects the txacked status)
 *   This function will first cancel tx timer because TX has completed. Then
 *   lower-half ops->txfinish() will be called to finish current
 *   transmission, clean related state and prepare for next transmission.
 *   If there are any remaining message in circbuf, it will be submitted to
 *   mailbox controller to send to remote.
 *
 * Input Parameters:
 *   chan    - A pointer to the mbox channel which received ack
 *
 ****************************************************************************/

void mbox_chan_tx_done(FAR struct mbox_chan_s *chan)
{
  irqstate_t flags;

  flags = enter_critical_section();

  if (!chan || chan->dir != MBOX_TX)
    {
      leave_critical_section(flags);
      return;
    }

  /* Cancel timer because ACK has reached */

  if (WDOG_ISACTIVE(&chan->timer))
    {
      wd_cancel(&chan->timer);
    }

  /* Finish this transmission, clean related state for next transmit */

  if (chan->dev->ops->txfinish)
    {
      chan->dev->ops->txfinish(chan);
    }

#ifdef CONFIG_MBOX_TRACE
  ++chan->stats.acked;
#endif

  if (chan->txstate != MBOX_TX_ACTIVE)
    {
      leave_critical_section(flags);
      return;
    }

  chan->txstate = MBOX_TX_IDLE;
  chan->txresult = OK;

  /* notify txdone to blocking API */

  notify_txdone(chan);

  /* Send next data if ringbuf not empty */

  if (!circbuf_is_empty(&chan->txbuf))
    {
      submit_msg(chan);
    }

  leave_critical_section(flags);

  return;
}

/****************************************************************************
 * Name: mbox_chan_rx_data
 *
 * Description:
 *   This function should be called when RX happened (when triggered by an
 *   interrupt or when polling detects the rxavailable status)
 *   This function will call lower-half ops->recv to receive the data from
 *   remote and send acknowledge to remote sender. Finally, the user-defined
 *   callback function will be called to push data to the upper layer.
 *
 * Input Parameters:
 *   chan    - A pointer to the mbox channel which rx happened
 *
 ****************************************************************************/

int mbox_chan_rx_data(FAR struct mbox_chan_s *chan)
{
  uint8_t data[MBOX_MAX_MSG_SIZE];
  int ret;

  if (!chan || chan->dir != MBOX_RX)
    {
      return -EINVAL;
    }

  /* Receive data from lower-half ops */

  ret = chan->dev->ops->recv(chan, data);
  if (ret < 0)
    {
      if (chan->callback)
        {
          chan->callback(ret, chan, chan->priv, NULL, 0);
        }

      return ret;
    }

#ifdef CONFIG_MBOX_TRACE
  ++chan->stats.received;
#endif

  /* Send acknowledge to remote sender */

  if (chan->dev->ops->acknowledge)
    {
      chan->dev->ops->acknowledge(chan, NULL, 0);
    }

#ifdef CONFIG_MBOX_TRACE
  ++chan->stats.acked;
#endif

  /* Call user-defined callback */

  if (chan->callback)
    {
      chan->callback(OK, chan, chan->priv, data, ret);
    }

  return ret;
}

/****************************************************************************
 * Name: mbox_get_chan
 *
 * Description:
 *   Get a mbox channel handle by platform-specific argument.
 *
 * Input Parameters:
 *   dev  - A pointer to mbox dev
 *   arg  - Platform-specific argument to index a channel
 *
 * Returned Value:
 *   Pointer to mbox channel on success; NULL on failure
 *
 ****************************************************************************/

FAR struct mbox_chan_s *mbox_get_chan(FAR struct mbox_dev_s *dev,
                                      FAR void *arg)
{
  if (!dev || !dev->ops || !dev->ops->getchan)
    {
      return NULL;
    }

  return MBOX_GET_CHAN(dev, arg);
}

/****************************************************************************
 * Name: mbox_chan_init
 *
 * Description:
 *   Initialize the members of the specific mbox channel structure.
 *   This function could be called to initialize the base part of
 *   a platform-specific channel in lower-half initialization.
 *
 * Input Parameters:
 *   chan    - A pointer to mbox channel to be initialized
 *   dev     - A pointer to mbox dev
 *   dir     - Mbox xfer direction TX or RX
 *   buffer  - The internal buffer allocated for the txbuf of mbox_chan_s
 *   size    - The size of the txbuf in mbox_chan_s
 *
 * Returned Value:
 *   0: success, <0: A negated errno
 *
 ****************************************************************************/

int mbox_chan_init(FAR struct mbox_chan_s *chan, FAR struct mbox_dev_s *dev,
                   enum mbox_direction_e dir, FAR void *buffer, size_t size)
{
  FAR const struct mbox_ops_s *ops;
  int ret = 0;
  bool sem_inited = false;

  if (!chan || !dev || !dev->ops)
    {
      return -EINVAL;
    }

  ops = dev->ops;

  if (dir != MBOX_TX && dir != MBOX_RX)
    {
      return -EINVAL;
    }

  if (dir == MBOX_TX &&
      (!ops->send || !ops->txready || !ops->txfinish ||
       !buffer || size == 0))
    {
      return -EINVAL;
    }

  if (dir == MBOX_RX && !ops->recv)
    {
      return -EINVAL;
    }

  chan->dev = dev;
  chan->dir = dir;
  chan->timeout = MBOX_WAIT_FOREVER;
  chan->txstate = MBOX_TX_IDLE;
  chan->txwaiting = false;
  chan->txresult = OK;

  if (dir == MBOX_TX)
    {
      ret = nxsem_init(&chan->txsem, 0, 0);
      if (ret < 0)
        {
          goto fail;
        }

      sem_inited = true;

      memset(buffer, 0, size);
      ret = circbuf_init(&chan->txbuf, buffer, size);
      if (ret < 0)
        {
          goto fail;
        }
    }

#ifdef CONFIG_MBOX_TRACE
  memset(&chan->stats, 0, sizeof(chan->stats));
#endif
  return 0;

fail:
  if (sem_inited)
    {
      nxsem_destroy(&chan->txsem);
    }

  return ret;
}

/****************************************************************************
 * Name: mbox_ticksend
 *
 * Description:
 *   Send a message through a specific send channel.
 *
 * Input Parameters:
 *   sender  - A Pointer to a mbox sender
 *   data    - Message Buffer to send
 *   size    - Size of data in bytes
 *   timeout - The ticks to wait until the message is acknowledged or
 *             completed. If timeout is zero, the message is submitted or
 *             buffered without waiting.
 *
 * Returned Value:
 *   0: success, <0: A negated errno
 *   -EINVAL:    Invalid parameter
 *   -EIO:       IO Error happened when send data
 *   -EAGAIN:    Not txready now, please try again
 *   -ETIMEDOUT: Waiting for TX timeout
 *
 ****************************************************************************/

int mbox_ticksend(FAR struct mbox_sender_s *sender, FAR const void *data,
                  size_t size, clock_t timeout)
{
  FAR struct mbox_chan_s *chan;
  irqstate_t flags;
  int ret = 0;

  if (!sender || !sender->chan || !data)
    {
      return -EINVAL;
    }

  if (size > MBOX_MAX_MSG_SIZE)
    {
      return -EMSGSIZE;
    }

  chan = sender->chan;

  if (chan->dir != MBOX_TX)
    {
      return -EINVAL;
    }

  flags = enter_critical_section();

  if (timeout == 0)
    {
      if (chan->txstate != MBOX_TX_IDLE)
        {
          ret = mbox_enqueue_data(chan, data, size);
          goto out;
        }

      if (!chan->dev->ops->txready(chan))
        {
          ret = -EAGAIN;
          goto out;
        }

      ret = mbox_send_data(chan, data, size, MBOX_WAIT_FOREVER);
      goto out;
    }

  /* Blocking send until timeout. If not ready, return -EAGAIN */

  if (chan->txstate != MBOX_TX_IDLE ||
      !chan->dev->ops->txready(chan))
    {
      ret = -EAGAIN;
      goto out;
    }

  chan->txwaiting = true;
  chan->txresult = -EINPROGRESS;

  ret = mbox_send_data(chan, data, size, timeout);
  if (ret < 0)
    {
      chan->txwaiting = false;
      goto out;
    }

  leave_critical_section(flags);

  /* Wait until tx_expiry() or mbox_chan_tx_done() completes this transfer. */

  ret = nxsem_wait_uninterruptible(&chan->txsem);
  if (ret < 0)
    {
      flags = enter_critical_section();
      chan->txwaiting = false;
      leave_critical_section(flags);
      return ret;
    }

  flags = enter_critical_section();
  ret = chan->txresult;
  leave_critical_section(flags);
  return ret;

out:
  leave_critical_section(flags);
  return ret;
}

/****************************************************************************
 * Name: mbox_send
 *
 * Description:
 *   Send a message through a specific send channel. It will block until
 *   the message is acknowledged or completed.
 *
 * Input Parameters:
 *   sender  - A Pointer to a mbox sender
 *   data    - Message to send
 *   size    - Bytes of data to send
 *
 * Returned Value:
 *   0: success, <0: A negated errno
 *   -EINVAL:    Invalid parameter
 *   -EIO:       IO Error happened when send data
 *   -EAGAIN:    Not txready now, please try again
 *   -ETIMEDOUT: Waiting for TX timeout
 *
 ****************************************************************************/

int mbox_send(FAR struct mbox_sender_s *sender, FAR const void *data,
              size_t size)
{
  return mbox_ticksend(sender, data, size, MBOX_WAIT_FOREVER);
}

/****************************************************************************
 * Name: mbox_register_rxcallback
 *
 * Description:
 *   Register a user-defined callback function to receiver channel. This
 *   callback function will be called when rxavailable.
 *
 * Input Parameters:
 *   receiver - Mbox receiver which binding to a rx channel
 *   callback - User-defined callback
 *
 * Returned Value:
 *   0: success, <0: A negated errno
 *
 ****************************************************************************/

int mbox_register_rxcallback(FAR struct mbox_receiver_s *receiver,
                             mbox_callback_t callback, FAR void *arg)
{
  if (!receiver || !receiver->chan || receiver->chan->dir != MBOX_RX)
    {
      return -EINVAL;
    }

  return MBOX_REGISTER_CALLBACK(receiver->chan, callback, arg);
}

/****************************************************************************
 * Name: mbox_unregister_rxcallback
 *
 * Description:
 *   Unregister a previously registered callback function from receiver
 *   channel.
 *
 * Input Parameters:
 *   receiver - Mbox receiver which binding to a rx channel
 *
 * Returned Value:
 *   0: success, <0: A negated errno
 *
 ****************************************************************************/

int mbox_unregister_rxcallback(FAR struct mbox_receiver_s *receiver)
{
  if (!receiver || !receiver->chan || receiver->chan->dir != MBOX_RX)
    {
      return -EINVAL;
    }

  return MBOX_REGISTER_CALLBACK(receiver->chan, NULL, NULL);
}

/****************************************************************************
 * Name: mbox_chan_deinit
 *
 * Description:
 *   Deinitialize a mbox channel, releasing the semaphore and circbuf
 *   resources that were allocated during mbox_chan_init.
 *
 * Input Parameters:
 *   chan    - A pointer to mbox channel to be deinitialized
 *
 * Returned Value:
 *   0: success, <0: A negated errno
 *
 ****************************************************************************/

int mbox_chan_deinit(FAR struct mbox_chan_s *chan)
{
  if (!chan)
    {
      return -EINVAL;
    }

  /* Only TX channels have txsem and txbuf initialized */

  if (chan->dir == MBOX_TX)
    {
      if (WDOG_ISACTIVE(&chan->timer))
        {
          wd_cancel(&chan->timer);
        }

      nxsem_destroy(&chan->txsem);
      circbuf_uninit(&chan->txbuf);
    }

  chan->callback = NULL;
  chan->dev = NULL;

  return 0;
}

#ifdef CONFIG_MBOX_TRACE
/****************************************************************************
 * Name: mbox_chan_get_stats
 *
 * Description:
 *   This function is only visible when configure CONFIG_MBOX_TRACE.
 *   This function will retrieves the message sending and receiving
 *   statistics data of specific mbox channel and returns it to
 *   the caller.
 *
 * Input Parameters:
 *   chan    - A pointer to the mbox channel which received ack
 *   stats   - A pointer to the mbox_stats_s passed by caller
 *
 ****************************************************************************/

void mbox_chan_get_stats(FAR struct mbox_chan_s *chan,
                         FAR struct mbox_stats_s *stats)
{
  if (!chan || !stats)
    {
      return;
    }

  *stats = chan->stats;
}
#endif
