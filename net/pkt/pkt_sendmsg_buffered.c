/****************************************************************************
 * net/pkt/pkt_sendmsg_buffered.c
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

#include <sys/types.h>
#include <sys/socket.h>

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <arch/irq.h>
#include <netpacket/packet.h>

#include <nuttx/semaphore.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/net.h>
#include <nuttx/net/ip.h>

#include "devif/devif.h"
#include "netdev/netdev.h"
#include "socket/socket.h"
#include "utils/utils.h"
#include "pkt/pkt.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pkt_sendbuffer_notify
 *
 * Description:
 *   Notify the send buffer semaphore
 *
 * Input Parameters:
 *   conn - The PKT connection of interest
 *
 * Assumptions:
 *   Called from user logic with the network locked.
 *
 ****************************************************************************/

static void pkt_sendbuffer_notify(FAR struct pkt_conn_s *conn)
{
  int val = 0;

  nxsem_get_value(&conn->sndsem, &val);
  if (val < 0)
    {
      nxsem_post(&conn->sndsem);
    }
}

/****************************************************************************
 * Name: psock_send_eventhandler
 ****************************************************************************/

static uint32_t psock_send_eventhandler(FAR struct net_driver_s *dev,
                                        FAR void *pvpriv, uint32_t flags)
{
  FAR struct pkt_conn_s *conn = pvpriv;

  DEBUGASSERT(dev != NULL && conn != NULL);

  /* Check if the outgoing packet is available. It may have been claimed
   * by a send event handler serving a different thread -OR- if the
   * output buffer currently contains unprocessed incoming data. In
   * these cases we will just have to wait for the next polling cycle.
   */

  if (dev->d_sndlen > 0 || (flags & PKT_NEWDATA) != 0)
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
      FAR struct iob_s *iob;

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

      iob = iob_remove_queue(&conn->write_q);
      DEBUGASSERT(iob != NULL);

      /* Then set-up to send that amount of data with the offset
       * corresponding to the size of the IP-dependent address structure.
       */

      netdev_iob_replace(dev, iob);
      conn->pendiob = iob;

      /* Get the amount of data that we can send in the next packet.
       * We will send either the remaining data in the buffer I/O
       * buffer chain, or as much as will fit given the MSS and current
       * window size.
       */

      dev->d_sndlen = iob->io_pktlen + NET_LL_HDRLEN(dev);
      dev->d_len = dev->d_sndlen;
      ninfo("wrb=%p sndlen=%d\n", iob, dev->d_sndlen);

      if (write_q_len > 1)
        {
          /* Set up for the next packet transfer
           * next packet now at the header of the write buffer queue.
           */

          /* Notify the device driver that new TX data is available. */

          netdev_txnotify_dev(dev, PKT_POLL);
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

      flags &= ~PKT_POLL;

      pkt_sendbuffer_notify(conn);
    }

  return flags;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pkt_sendmsg
 *
 * Description:
 *   The pkt_sendmsg() call may be used only when the packet socket is in
 *   a connected state (so that the intended recipient is known).
 *
 * Input Parameters:
 *   psock    An instance of the internal socket structure.
 *   msg      Message to send
 *   flags    Send flags
 *
 * Returned Value:
 *   On success, returns the number of characters sent. On error, a negated
 *   errno value is returned (see sendmsg() for the complete list of return
 *   values.
 *
 ****************************************************************************/

ssize_t pkt_sendmsg(FAR struct socket *psock, FAR const struct msghdr *msg,
                    int flags)
{
  FAR const void *buf = msg->msg_iov->iov_base;
  size_t len = msg->msg_iov->iov_len;
  FAR struct sockaddr_ll *addr = msg->msg_name;
  FAR struct net_driver_s *dev;
  FAR struct pkt_conn_s *conn;
  FAR struct iob_s *iob;
  bool nonblock;
  int offset = 0;
  int ret = OK;

  /* Validity check */

  ret = pkt_sendmsg_is_valid(psock, msg, &dev);
  if (ret != OK)
    {
      return ret;
    }

  if (len <= 0)
    {
      return 0;
    }

  conn = psock->s_conn;
  if (psock->s_type == SOCK_DGRAM)
    {
      /* Set the interface index for devif_poll can match the conn */

      conn->ifindex = addr->sll_ifindex;
    }

  conn_dev_lock(&conn->sconn, dev);
  nonblock = _SS_ISNONBLOCK(conn->sconn.s_flags) ||
             (flags & MSG_DONTWAIT) != 0;

#if CONFIG_NET_SEND_BUFSIZE > 0
  if ((iob_get_queue_size(&conn->write_q) + len) >= conn->sndbufs)
    {
      /* send buffer size exceeds the send limit */

      if (nonblock)
        {
          nerr("ERROR: Buffer overflow\n");
          ret = -EAGAIN;
          goto errout_with_lock;
        }

      ret = conn_dev_sem_timedwait(&conn->sndsem, false,
                                   _SO_TIMEOUT(conn->sconn.s_sndtimeo),
                                   &conn->sconn, dev);
      if (ret < 0)
        {
          goto errout_with_lock;
        }
    }
#endif

  if (nonblock)
    {
      iob = iob_tryalloc(true);
    }
  else
    {
      iob = net_iobtimedalloc(true, _SO_TIMEOUT(conn->sconn.s_sndtimeo));
    }

  if (iob == NULL)
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

  iob_reserve(iob, CONFIG_NET_LL_GUARDSIZE);
  iob_update_pktlen(iob, 0, true);

  /* Copy the user data into the write buffer.  We cannot wait for
   * buffer space if the socket was opened non-blocking.
   */

  if (psock->s_type == SOCK_RAW)
    {
      offset = -NET_LL_HDRLEN(dev);
    }

  if (nonblock)
    {
      ret = iob_trycopyin(iob, buf, len, offset, true);
    }
  else
    {
      /* iob_copyin might wait for buffers to be freed, but if
       * network is locked this might never happen, since network
       * driver is also locked, therefore we need to break the lock
       */

      conn_dev_unlock(&conn->sconn, dev);
      ret = iob_copyin(iob, buf, len, offset, true);
      conn_dev_lock(&conn->sconn, dev);
    }

  if (ret < 0)
    {
      nerr("ERROR: Failed to copy data into write buffer\n");
      goto errout_with_iob;
    }

  if (psock->s_type == SOCK_DGRAM)
    {
      FAR struct eth_hdr_s *ethhdr =
          (FAR struct eth_hdr_s *)(IOB_DATA(iob) - NET_LL_HDRLEN(dev));
      memcpy(ethhdr->dest, addr->sll_addr, ETHER_ADDR_LEN);
      memcpy(ethhdr->src, &dev->d_mac.ether, ETHER_ADDR_LEN);
      ethhdr->type = addr->sll_protocol;
    }

  if (nonblock)
    {
      ret = iob_tryadd_queue(iob, &conn->write_q);
    }
  else
    {
      ret = iob_add_queue(iob, &conn->write_q);
    }

  if (ret < 0)
    {
      nerr("ERROR: Failed to add buffer to w_queue\n");
      goto errout_with_iob;
    }

  /* Allocate resource to receive a callback */

  if (conn->sndcb == NULL)
    {
      conn->sndcb = pkt_callback_alloc(dev, conn);
    }

  /* Test if the callback has been allocated */

  if (conn->sndcb == NULL)
    {
      /* A buffer allocation error occurred */

      nerr("ERROR: Failed to allocate callback\n");
      ret = -ENOMEM;
      goto errout_with_iob;
    }
  else
    {
      /* Set up the callback in the connection */

      conn->sndcb->flags = PKT_POLL;
      conn->sndcb->priv  = conn;
      conn->sndcb->event = psock_send_eventhandler;

      /* Notify the device driver that new TX data is available. */

      netdev_txnotify_dev(dev, PKT_POLL);
      conn_dev_unlock(&conn->sconn, dev);
    }

  return len;

errout_with_iob:
  iob_free_chain(iob);

errout_with_lock:
  conn_dev_unlock(&conn->sconn, dev);

  return ret;
}
