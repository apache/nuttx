/****************************************************************************
 * net/can/can_sendmsg_buffered.c
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
#if defined(CONFIG_NET) && defined(CONFIG_NET_CAN) && \
    defined(CONFIG_NET_CAN_WRITE_BUFFERS)

#include <sys/socket.h>
#include <sys/types.h>

#include <debug.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <arch/irq.h>

#include <nuttx/net/can.h>
#include <nuttx/net/ip.h>
#include <nuttx/net/net.h>
#include <nuttx/net/netdev.h>
#include <nuttx/semaphore.h>

#include "can/can.h"
#include "devif/devif.h"
#include "netdev/netdev.h"
#include "socket/socket.h"
#include "utils/utils.h"

#include <sys/time.h>

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: psock_send_eventhandler
 ****************************************************************************/

static uint16_t psock_send_eventhandler(FAR struct net_driver_s *dev,
                                        FAR void *pvpriv, uint16_t flags)
{
  FAR struct can_conn_s *conn = pvpriv;

  DEBUGASSERT(dev != NULL && conn != NULL);

  /* Check if the outgoing packet is available. It may have been claimed
   * by a send event handler serving a different thread -OR- if the
   * output buffer currently contains unprocessed incoming data. In
   * these cases we will just have to wait for the next polling cycle.
   */

  if (dev->d_sndlen > 0 || (flags & CAN_NEWDATA) != 0)
    {
      /* Another thread has beat us sending data or the buffer is busy,
       * Check for a timeout. If not timed out, wait for the next
       * polling cycle and check again.
       */

      /* No timeout. Just wait for the next polling cycle */

      return flags;
    }

  /* It looks like we are good to send the data */

  else
    {
      uint32_t write_q_len;
      FAR struct iob_s *wrb_iob;

      /* Copy the packet data into the device packet buffer and send it */

      write_q_len = iob_get_queue_entry_count(&conn->write_q);

      if (write_q_len == 0)
        {
          return flags;
        }

      /* Peek at the head of the write queue (but don't remove anything
       * from the write queue yet).  We know from the above test that
       * the write_q is not empty.
       */

      wrb_iob = (FAR struct iob_s *)iob_remove_queue(&conn->write_q);
      DEBUGASSERT(wrb_iob != NULL);

      /* Then set-up to send that amount of data with the offset
       * corresponding to the size of the IP-dependent address structure.
       */

      netdev_iob_replace(dev, wrb_iob);

      /* Get the amount of data that we can send in the next packet.
       * We will send either the remaining data in the buffer I/O
       * buffer chain, or as much as will fit given the MSS and current
       * window size.
       */

      dev->d_sndlen = wrb_iob->io_pktlen;
      ninfo("wrb=%p sndlen=%d\n", wrb_iob, dev->d_sndlen);

      if (write_q_len > 1)
        {
          /* Set up for the next packet transfer
           * next packet now at the header of the write buffer queue.
           */

          /* Notify the device driver that new TX data is available. */

          netdev_txnotify_dev(dev);
        }
      else
        {
          /* stifle any further callbacks until more write data is
           * enqueued.
           */

          conn->sndcb->flags = 0;
          conn->sndcb->priv = NULL;
          conn->sndcb->event = NULL;
        }

      /* Only one data can be sent by low level driver at once,
       * tell the caller stop polling the other connections.
       */

      flags &= ~CAN_POLL;

#if CONFIG_NET_SEND_BUFSIZE > 0
      can_sendbuffer_notify(conn);
#endif
    }

  return flags;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: can_sendmsg
 *
 * Description:
 *   The can_sendmsg() call may be used only when the packet socket is
 *   in a connected state (so that the intended recipient is known).
 *
 * Input Parameters:
 *   psock    An instance of the internal socket structure.
 *   msg      CAN frame and optional CMSG
 *   flags    Send flags (ignored)
 *
 * Returned Value:
 *   On success, returns the number of characters sent.  On  error,
 *   a negated errno value is retruend.  See sendmsg() for the complete list
 *   of return values.
 *
 ****************************************************************************/

ssize_t can_sendmsg(FAR struct socket *psock, FAR struct msghdr *msg,
                    int flags)
{
  FAR struct net_driver_s *dev;
  FAR struct can_conn_s *conn;
  FAR struct iob_s *wb_iob;
  bool empty;
  bool nonblock;
  int ret = OK;

  /* Verify that the sockfd corresponds to valid, allocated socket */

  if (psock == NULL || psock->s_conn == NULL)
    {
      return -EBADF;
    }

  /* Only SOCK_RAW is supported */

  if (psock->s_type != SOCK_RAW)
    {
      /* EDESTADDRREQ.  Signifies that the socket is not connection-mode and
       * no peer address is set.
       */

      return -EDESTADDRREQ;
    }

  conn = psock->s_conn;

  /* Get the device driver that will service this transfer */

  dev = conn->dev;
  if (dev == NULL)
    {
      return -ENODEV;
    }

#if defined(CONFIG_NET_CANPROTO_OPTIONS) && defined(CONFIG_NET_CAN_CANFD)
  if (_SO_GETOPT(conn->sconn.s_options, CAN_RAW_FD_FRAMES))
    {
      if (msg->msg_iov->iov_len != CANFD_MTU &&
          msg->msg_iov->iov_len != CAN_MTU)
        {
          return -EINVAL;
        }
    }
  else
#endif
    {
      if (msg->msg_iov->iov_len != CAN_MTU)
        {
          return -EINVAL;
        }
    }

  nonblock =
      _SS_ISNONBLOCK(conn->sconn.s_flags) || (flags & MSG_DONTWAIT) != 0;

  /* Perform the send operation */

  /* Initialize the state structure. This is done with the network locked
   * because we don't want anything to happen until we are ready.
   */

  conn_dev_lock(&conn->sconn, dev);

#if CONFIG_NET_SEND_BUFSIZE > 0
  if ((iob_get_queue_size(&conn->write_q) + msg->msg_iov->iov_len) >
      conn->sndbufs)
    {
      /* send buffer size exceeds the send limit */

      if (nonblock)
        {
          nerr("ERROR: Buffer overflow\n");
          ret = -EAGAIN;
          goto errout_with_lock;
        }

      ret = net_sem_timedwait_uninterruptible(&conn->sndsem,
        _SO_TIMEOUT(conn->sconn.s_sndtimeo));
      if (ret < 0)
        {
          goto errout_with_lock;
        }
    }
#endif

  if (nonblock)
    {
      wb_iob = can_iob_timedalloc(0);
    }
  else
    {
      wb_iob = can_iob_timedalloc(_SO_TIMEOUT(conn->sconn.s_sndtimeo));
    }

  if (wb_iob == NULL)
    {
      /* A buffer allocation error occurred */

      nerr("ERROR: Failed to allocate write buffer\n");

      if (nonblock)
        {
          ret = -EAGAIN;
        }
      else
        {
          ret = -ENOMEM;
        }

      goto errout_with_lock;
    }

  iob_reserve(wb_iob, CONFIG_NET_LL_GUARDSIZE);
  iob_update_pktlen(wb_iob, 0, false);

  /* Copy the user data into the write buffer.  We cannot wait for
   * buffer space if the socket was opened non-blocking.
   */

  if (nonblock)
    {
      ret = iob_trycopyin(wb_iob, (FAR uint8_t *)msg->msg_iov->iov_base,
                          msg->msg_iov->iov_len, 0, false);
    }
  else
    {
      /* iob_copyin might wait for buffers to be freed, but if
       * network is locked this might never happen, since network
       * driver is also locked, therefore we need to break the lock
       */

      conn_dev_unlock(&conn->sconn, dev);
      ret = iob_copyin(wb_iob, (FAR uint8_t *)msg->msg_iov->iov_base,
                       msg->msg_iov->iov_len, 0, false);
      conn_dev_lock(&conn->sconn, dev);
    }

  if (ret < 0)
    {
      nerr("ERROR: Failed to copy data into write buffer\n");
      goto errout_with_wrb;
    }

  /* Check if the write queue is empty */

  empty = (iob_get_queue_entry_count(&conn->write_q) == 0);

  /* Add the write buffer to the write queue */

  if (nonblock)
    {
      ret = iob_tryadd_queue(wb_iob, &conn->write_q);
    }
  else
    {
      ret = iob_add_queue(wb_iob, &conn->write_q);
    }

  if (ret < 0)
    {
      nerr("ERROR: Failed to add buffer to w_queue\n");
      goto errout_with_wrb;
    }

  /* The new write buffer lies at the head of the write queue.
   * Set up for the next packet transfer.
   */

  if (empty)
    {
      /* Allocate resource to receive a callback */

      if (conn->sndcb == NULL)
        {
          conn->sndcb = can_callback_alloc(dev, conn);
        }

      /* Test if the callback has been allocated */

      if (conn->sndcb == NULL)
        {
          /* A buffer allocation error occurred */

          nerr("ERROR: Failed to allocate callback\n");
          ret = -ENOMEM;
          goto errout_with_wq;
        }

      /* Set up the callback in the connection */

      conn->sndcb->flags = CAN_POLL;
      conn->sndcb->priv = (FAR void *)conn;
      conn->sndcb->event = psock_send_eventhandler;

      /* unlock */

      conn_dev_unlock(&conn->sconn, dev);

      /* Notify the device driver that new TX data is available. */

      netdev_txnotify_dev(dev);
    }
    else
    {
      /* unlock */

      conn_dev_unlock(&conn->sconn, dev);
    }

  return msg->msg_iov->iov_len;

errout_with_wrb:
  iob_free_chain(wb_iob);

errout_with_wq:
  iob_free_queue(&conn->write_q);

errout_with_lock:
  conn_dev_unlock(&conn->sconn, dev);
  return ret;
}

/****************************************************************************
 * Name: psock_can_cansend
 *
 * Description:
 *   psock_can_cansend() returns a value indicating if a write to the socket
 *   would block.  No space in the buffer is actually reserved, so it is
 *   possible that the write may still block if the buffer is filled by
 *   another means.
 *
 * Input Parameters:
 *   psock    An instance of the internal socket structure.
 *
 * Returned Value:
 *   OK
 *     At least one byte of data could be successfully written.
 *   -EWOULDBLOCK
 *     There is no room in the output buffer.
 *   -EBADF
 *     An invalid descriptor was specified.
 *
 ****************************************************************************/

int psock_can_cansend(FAR struct socket *psock)
{
#if CONFIG_NET_SEND_BUFSIZE > 0
  FAR struct can_conn_s *conn;
#endif

  /* Verify that we received a valid socket */

  if (psock == NULL || psock->s_conn == NULL)
    {
      nerr("ERROR: Invalid socket\n");
      return -EBADF;
    }

  /* In order to setup the send, we need to have at least
   * one free IOB to initialize the write buffer head.
   */

#if CONFIG_NET_SEND_BUFSIZE > 0
  conn = psock->s_conn;
#endif

  if (can_iob_navail() <= 0
#if CONFIG_NET_SEND_BUFSIZE > 0
      || iob_get_queue_size(&conn->write_q) >= conn->sndbufs
#endif
     )
    {
      return -EWOULDBLOCK;
    }

  return OK;
}

/****************************************************************************
 * Name: can_sendbuffer_notify
 *
 * Description:
 *   Notify the send buffer semaphore
 *
 * Input Parameters:
 *   conn - The CAN connection of interest
 *
 * Assumptions:
 *   Called from user logic with the network locked.
 *
 ****************************************************************************/

#if CONFIG_NET_SEND_BUFSIZE > 0
void can_sendbuffer_notify(FAR struct can_conn_s *conn)
{
  int val = 0;

  nxsem_get_value(&conn->sndsem, &val);
  if (val < 0)
    {
      nxsem_post(&conn->sndsem);
    }
}

#endif /* CONFIG_NET_SEND_BUFSIZE */

#endif /* CONFIG_NET && CONFIG_NET_CAN */
