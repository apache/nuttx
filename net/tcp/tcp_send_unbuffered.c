/****************************************************************************
 * net/tcp/tcp_send_unbuffered.c
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
#if defined(CONFIG_NET) && defined(CONFIG_NET_TCP) && \
   !defined(CONFIG_NET_TCP_WRITE_BUFFERS)

#include <sys/types.h>
#include <sys/socket.h>

#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <arch/irq.h>

#include <nuttx/semaphore.h>
#include <nuttx/net/net.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/arp.h>
#include <nuttx/net/tcp.h>

#include "netdev/netdev.h"
#include "devif/devif.h"
#include "socket/socket.h"
#include "inet/inet.h"
#include "arp/arp.h"
#include "icmpv6/icmpv6.h"
#include "neighbor/neighbor.h"
#include "route/route.h"
#include "tcp/tcp.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* If both IPv4 and IPv6 support are both enabled, then we will need to build
 * in some additional domain selection support.
 */

#if defined(CONFIG_NET_IPv4) && defined(CONFIG_NET_IPv6)
#  define NEED_IPDOMAIN_SUPPORT 1
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure holds the state of the send operation until it can be
 * operated upon when the TX poll event occurs.
 */

struct send_s
{
  FAR struct tcp_conn_s  *snd_conn;     /* The TCP connection of interest */
  FAR struct devif_callback_s *snd_cb;  /* Reference to callback instance */
  sem_t                   snd_sem;      /* Used to wake up the waiting thread */
  FAR const uint8_t      *snd_buffer;   /* Points to the buffer of data to send */
  size_t                  snd_buflen;   /* Number of bytes in the buffer to send */
  ssize_t                 snd_sent;     /* The number of bytes sent */
  uint32_t                snd_isn;      /* Initial sequence number */
  uint32_t                snd_acked;    /* The number of bytes acked */
#ifdef CONFIG_NET_TCP_FAST_RETRANSMIT
  uint32_t                snd_prev_ack; /* The previous ACKed seq number */
#ifdef CONFIG_NET_TCP_WINDOW_SCALE
  uint32_t                snd_prev_wnd; /* The advertised window in the last
                                         * incoming acknowledgment
                                         */
#else
  uint16_t                snd_prev_wnd;
#endif
  int                     snd_dup_acks; /* Duplicate ACK counter */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tcpsend_ipselect
 *
 * Description:
 *   If both IPv4 and IPv6 support are enabled, then we will need to select
 *   which one to use when generating the outgoing packet.  If only one
 *   domain is selected, then the setup is already in place and we need do
 *   nothing.
 *
 * Input Parameters:
 *   dev    - The structure of the network driver that caused the event
 *   pstate - sendto state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

#ifdef NEED_IPDOMAIN_SUPPORT
static inline void tcpsend_ipselect(FAR struct net_driver_s *dev,
                                    FAR struct tcp_conn_s *conn)
{
  /* Which domain does the socket support */

  if (conn->domain == PF_INET)
    {
      /* Select the IPv4 domain */

      tcp_ipv4_select(dev);
    }
  else /* if (conn->domain == PF_INET6) */
    {
      /* Select the IPv6 domain */

      tcp_ipv6_select(dev);
    }
}
#endif

/****************************************************************************
 * Name: tcpsend_eventhandler
 *
 * Description:
 *   This function is called to perform the actual send operation when
 *   polled by the lower, device interfacing layer.
 *
 * Input Parameters:
 *   dev      The structure of the network driver that caused the event
 *   pvpriv   An instance of struct send_s cast to void*
 *   flags    Set of events describing why the callback was invoked
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

static uint16_t tcpsend_eventhandler(FAR struct net_driver_s *dev,
                                     FAR void *pvpriv, uint16_t flags)
{
  FAR struct send_s *pstate = pvpriv;
  FAR struct tcp_conn_s *conn;

  DEBUGASSERT(pstate != NULL);

  /* Get the TCP connection pointer reliably from
   * the corresponding TCP socket.
   */

  conn = pstate->snd_conn;
  DEBUGASSERT(conn != NULL);

  /* The TCP socket is connected and, hence, should be bound to a device.
   * Make sure that the polling device is the one that we are bound to.
   */

  DEBUGASSERT(conn->dev != NULL);
  if (dev != conn->dev)
    {
      return flags;
    }

  ninfo("flags: %04x acked: %" PRId32 " sent: %zd\n",
        flags, pstate->snd_acked, pstate->snd_sent);

  /* The TCP_ACKDATA, TCP_REXMIT and TCP_DISCONN_EVENTS flags are expected to
   * appear here strictly one at a time, except for the FIN + ACK case.
   */

  DEBUGASSERT((flags & TCP_ACKDATA) == 0 ||
              (flags & TCP_REXMIT) == 0);
  DEBUGASSERT((flags & TCP_DISCONN_EVENTS) == 0 ||
              (flags & TCP_REXMIT) == 0);

  /* If this packet contains an acknowledgement, then update the count of
   * acknowledged bytes.
   * This condition is located here for performance reasons
   * (TCP_ACKDATA is the most frequent event).
   */

  if ((flags & TCP_ACKDATA) != 0)
    {
      uint32_t ackno;
      FAR struct tcp_hdr_s *tcp;

      /* Get the offset address of the TCP header */

#ifdef CONFIG_NET_IPv4
#ifdef CONFIG_NET_IPv6
      if (conn->domain == PF_INET)
#endif
        {
          tcp = TCPIPv4BUF;
        }
#endif /* CONFIG_NET_IPv4 */

#ifdef CONFIG_NET_IPv6
#ifdef CONFIG_NET_IPv4
      else
#endif
        {
          tcp = TCPIPv6BUF;
        }
#endif /* CONFIG_NET_IPv6 */

      /* The current acknowledgement number is the (relative) offset of the
       * next byte needed by the receiver.  The snd_isn is the offset of the
       * first byte to send to the receiver.  The difference is the number
       * of bytes to be acknowledged.
       */

      ackno = tcp_getsequence(tcp->ackno);
      pstate->snd_acked = TCP_SEQ_SUB(ackno,
                                      pstate->snd_isn);
      ninfo("ACK: acked=%" PRId32 " sent=%zd buflen=%zu\n",
            pstate->snd_acked, pstate->snd_sent, pstate->snd_buflen);

      /* Have all of the bytes in the buffer been sent and acknowledged? */

      if (pstate->snd_acked >= pstate->snd_buflen)
        {
          /* Yes.  Then pstate->snd_buflen should hold the number of bytes
           * actually sent.
           */

          goto end_wait;
        }

#ifdef CONFIG_NET_TCP_FAST_RETRANSMIT
      /* Fast Retransmit (RFC 5681): an acknowledgment is considered a
       * "duplicate" when (a) the receiver of the ACK has outstanding data,
       * (b) the incoming acknowledgment carries no data, (c) the SYN and
       * FIN bits are both off, (d) the acknowledgment number is equal to
       * the greatest acknowledgment received on the given connection
       * and (e) the advertised window in the incoming acknowledgment equals
       * the advertised window in the last incoming acknowledgment.
       */

      if (pstate->snd_acked < pstate->snd_sent &&
          (flags & TCP_NEWDATA) == 0 &&
          (tcp->flags & (TCP_SYN | TCP_FIN)) == 0 &&
          ackno == pstate->snd_prev_ack &&
          conn->snd_wnd == pstate->snd_prev_wnd)
        {
          if (++pstate->snd_dup_acks >= TCP_FAST_RETRANSMISSION_THRESH)
            {
              flags |= TCP_REXMIT;
              pstate->snd_dup_acks = 0;
            }
        }
      else
        {
          pstate->snd_dup_acks = 0;
        }

      pstate->snd_prev_ack = ackno;
      pstate->snd_prev_wnd = conn->snd_wnd;
#endif
    }

  /* Check if we are being asked to retransmit data.
   * This condition is located here after TCP_ACKDATA for performance reasons
   * (TCP_REXMIT is less frequent than TCP_ACKDATA).
   */

  if ((flags & TCP_REXMIT) != 0)
    {
      uint32_t sndlen;

      nwarn("WARNING: TCP_REXMIT\n");

      /* According to RFC 6298 (5.4), retransmit the earliest segment
       * that has not been acknowledged by the TCP receiver.
       */

      /* Reconstruct the length of the earliest segment to be retransmitted */

      sndlen = pstate->snd_buflen - pstate->snd_acked;

      if (sndlen > conn->mss)
        {
          sndlen = conn->mss;
        }

      conn->rexmit_seq = pstate->snd_isn + pstate->snd_acked;

#ifdef NEED_IPDOMAIN_SUPPORT
      /* If both IPv4 and IPv6 support are enabled, then we will need to
       * select which one to use when generating the outgoing packet.
       * If only one domain is selected, then the setup is already in
       * place and we need do nothing.
       */

      tcpsend_ipselect(dev, conn);
#endif
      /* Then set-up to send that amount of data. (this won't actually
       * happen until the polling cycle completes).
       */

      devif_send(dev,
                 &pstate->snd_buffer[pstate->snd_acked],
                 sndlen);

      /* Continue waiting */

      return flags;
    }

  /* Check for a loss of connection.
   * This condition is located here after both the TCP_ACKDATA and TCP_REXMIT
   * because TCP_DISCONN_EVENTS is the least frequent event.
   */

  else if ((flags & TCP_DISCONN_EVENTS) != 0)
    {
      ninfo("Lost connection\n");

      /* We could get here recursively through the callback actions of
       * tcp_lost_connection().  So don't repeat that action if we have
       * already been disconnected.
       */

      if (_SS_ISCONNECTED(conn->sconn.s_flags))
        {
          /* Report not connected */

          tcp_lost_connection(conn, pstate->snd_cb, flags);
        }

      pstate->snd_sent = -ENOTCONN;
      goto end_wait;
    }

  /* Check if the outgoing packet is available (it may have been claimed
   * by a sendto event serving a different thread).
   */

#if 0 /* We can't really support multiple senders on the same TCP socket */
  else if (dev->d_sndlen > 0)
    {
      /* Another thread has beat us sending data, wait for the next poll */

      return flags;
    }
#endif

  /* We get here if (1) not all of the data has been ACKed, (2) we have been
   * asked to retransmit data, (3) the connection is still healthy, and (4)
   * the outgoing packet is available for our use.  In this case, we are
   * now free to send more data to receiver -- UNLESS the buffer contains
   * unprocessed incoming data.  In that event, we will have to wait for the
   * next polling cycle.
   */

  if ((flags & TCP_NEWDATA) == 0 && pstate->snd_sent < pstate->snd_buflen)
    {
      /* Get the amount of data that we can send in the next packet */

      uint32_t sndlen = pstate->snd_buflen - pstate->snd_sent;

      if (sndlen > conn->mss)
        {
          sndlen = conn->mss;
        }

      /* Check if we have "space" in the window */

      if ((pstate->snd_sent - pstate->snd_acked + sndlen) < conn->snd_wnd)
        {
#ifdef NEED_IPDOMAIN_SUPPORT
          /* If both IPv4 and IPv6 support are enabled, then we will need to
           * select which one to use when generating the outgoing packet.
           * If only one domain is selected, then the setup is already in
           * place and we need do nothing.
           */

          tcpsend_ipselect(dev, conn);
#endif
          /* Then set-up to send that amount of data. (this won't actually
           * happen until the polling cycle completes).
           */

          devif_send(dev, &pstate->snd_buffer[pstate->snd_sent], sndlen);

          /* Update the amount of data sent (but not necessarily ACKed) */

          pstate->snd_sent += sndlen;
          ninfo("SEND: acked=%" PRId32 " sent=%zd buflen=%zu\n",
                pstate->snd_acked, pstate->snd_sent, pstate->snd_buflen);
        }
      else
        {
          nwarn("WARNING: Window full, wait for ack\n");
        }
    }

  /* Continue waiting */

  return flags;

end_wait:

  /* Do not allow any further callbacks */

  DEBUGASSERT(pstate->snd_cb != NULL);

  pstate->snd_cb->flags   = 0;
  pstate->snd_cb->priv    = NULL;
  pstate->snd_cb->event   = NULL;

  /* There are no outstanding, unacknowledged bytes */

  conn->tx_unacked        = 0;

  /* Wake up the waiting thread */

  nxsem_post(&pstate->snd_sem);
  return flags;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: psock_tcp_send
 *
 * Description:
 *   psock_tcp_send() call may be used only when the TCP socket is in a
 *   connected state (so that the intended recipient is known).
 *
 * Input Parameters:
 *   psock    An instance of the internal socket structure.
 *   buf      Data to send
 *   len      Length of data to send
 *   flags    Send flags
 *
 * Returned Value:
 *   On success, returns the number of characters sent.  On  error,
 *   a negated errno value is returned.
 *
 *   EAGAIN or EWOULDBLOCK
 *     The socket is marked non-blocking and the requested operation
 *     would block.
 *   EBADF
 *     An invalid descriptor was specified.
 *   ECONNRESET
 *     Connection reset by peer.
 *   EDESTADDRREQ
 *     The socket is not connection-mode, and no peer address is set.
 *   EFAULT
 *      An invalid user space address was specified for a parameter.
 *   EINTR
 *      A signal occurred before any data was transmitted.
 *   EINVAL
 *      Invalid argument passed.
 *   EISCONN
 *     The connection-mode socket was connected already but a recipient
 *     was specified. (Now either this error is returned, or the recipient
 *     specification is ignored.)
 *   EMSGSIZE
 *     The socket type requires that message be sent atomically, and the
 *     size of the message to be sent made this impossible.
 *   ENOBUFS
 *     The output queue for a network interface was full. This generally
 *     indicates that the interface has stopped sending, but may be
 *     caused by transient congestion.
 *   ENOMEM
 *     No memory available.
 *   ENOTCONN
 *     The socket is not connected, and no target has been given.
 *   ENOTSOCK
 *     The argument s is not a socket.
 *   EPIPE
 *     The local end has been shut down on a connection oriented socket.
 *     In this case the process will also receive a SIGPIPE unless
 *     MSG_NOSIGNAL is set.
 *
 ****************************************************************************/

ssize_t psock_tcp_send(FAR struct socket *psock,
                       FAR const void *buf, size_t len, int flags)
{
  FAR struct tcp_conn_s *conn;
  struct send_s state;
  int ret = OK;

  /* Verify that the sockfd corresponds to valid, allocated socket */

  if (psock == NULL || psock->s_type != SOCK_STREAM ||
      psock->s_conn == NULL)
    {
      nerr("ERROR: Invalid socket\n");
      ret = -EBADF;
      goto errout;
    }

  conn = (FAR struct tcp_conn_s *)psock->s_conn;

  /* Check early if this is an un-connected socket, if so, then
   * return -ENOTCONN. Note, we will have to check this again, as we can't
   * guarantee the state won't change until we have the network locked.
   */

  if (!_SS_ISCONNECTED(conn->sconn.s_flags))
    {
      nerr("ERROR: Not connected\n");
      ret = -ENOTCONN;
      goto errout;
    }

  /* Make sure that we have the IP address mapping */

#if defined(CONFIG_NET_ARP_SEND) || defined(CONFIG_NET_ICMPv6_NEIGHBOR)
#ifdef CONFIG_NET_ARP_SEND
#ifdef CONFIG_NET_ICMPv6_NEIGHBOR
  if (psock->s_domain == PF_INET)
#endif
    {
      /* Make sure that the IP address mapping is in the ARP table */

      ret = arp_send(conn->u.ipv4.raddr);
    }
#endif /* CONFIG_NET_ARP_SEND */

#ifdef CONFIG_NET_ICMPv6_NEIGHBOR
#ifdef CONFIG_NET_ARP_SEND
  else
#endif
    {
      /* Make sure that the IP address mapping is in the Neighbor Table */

      ret = icmpv6_neighbor(conn->u.ipv6.raddr);
    }
#endif /* CONFIG_NET_ICMPv6_NEIGHBOR */

  /* Did we successfully get the address mapping? */

  if (ret < 0)
    {
      nerr("ERROR: Not reachable\n");
      ret = -ENETUNREACH;
      goto errout;
    }
#endif /* CONFIG_NET_ARP_SEND || CONFIG_NET_ICMPv6_NEIGHBOR */

  /* Perform the TCP send operation */

  /* Initialize the state structure.  This is done with the network
   * locked because we don't want anything to happen until we are
   * ready.
   */

  net_lock();

  /* Now that we have the network locked, we need to check the connection
   * state again to ensure the connection is still valid.
   */

  if (!_SS_ISCONNECTED(conn->sconn.s_flags))
    {
      nerr("ERROR: No longer connected\n");
      net_unlock();
      ret = -ENOTCONN;
      goto errout;
    }

  memset(&state, 0, sizeof(struct send_s));
  nxsem_init(&state.snd_sem, 0, 0);    /* Doesn't really fail */

  state.snd_conn   = conn; /* Socket descriptor to use */
  state.snd_buflen = len;  /* Number of bytes to send */
  state.snd_buffer = buf;  /* Buffer to send from */

  if (len > 0)
    {
      /* Allocate resources to receive a callback */

      ret = -ENOMEM; /* Assume allocation failure */
      state.snd_cb = tcp_callback_alloc(conn);
      if (state.snd_cb)
        {
          /* Get the initial sequence number that will be used */

          state.snd_isn         = tcp_getsequence(conn->sndseq);

          /* There is no outstanding, unacknowledged data after this
           * initial sequence number.
           */

          conn->tx_unacked      = 0;

          /* Set up the callback in the connection */

          state.snd_cb->flags   = (TCP_ACKDATA | TCP_REXMIT | TCP_POLL |
                                   TCP_DISCONN_EVENTS);
          state.snd_cb->priv    = (FAR void *)&state;
          state.snd_cb->event   = tcpsend_eventhandler;

          /* Notify the device driver of the availability of TX data */

          tcp_send_txnotify(psock, conn);

          /* Wait for the send to complete or an error to occur:  NOTES:
           * net_lockedwait will also terminate if a signal is received.
           */

          for (; ; )
            {
              uint32_t acked = state.snd_acked;

              ret = net_timedwait(&state.snd_sem,
                                  _SO_TIMEOUT(conn->sconn.s_sndtimeo));
              if (ret != -ETIMEDOUT || acked == state.snd_acked)
                {
                  if (ret == -ETIMEDOUT)
                    {
                      ret = -EAGAIN;
                    }

                  break; /* Timeout without any progress */
                }
            }

          /* Make sure that no further events are processed */

          tcp_callback_free(conn, state.snd_cb);
        }
    }

  nxsem_destroy(&state.snd_sem);
  net_unlock();

  /* Check for a errors.  Errors are signalled by negative errno values
   * for the send length
   */

  if (state.snd_sent < 0)
    {
      ret = state.snd_sent;
      goto errout;
    }

  /* If net_timedwait failed, then we were probably reawakened by a signal.
   * In this case, net_timedwait will have returned negated errno
   * appropriately.
   */

  if (ret < 0)
    {
      goto errout;
    }

  /* Return the number of bytes actually sent */

  ret = state.snd_sent;

errout:
  return ret;
}

/****************************************************************************
 * Name: psock_tcp_cansend
 *
 * Description:
 *   psock_tcp_cansend() returns a value indicating if a write to the socket
 *   would block.  It is still possible that the write may block if another
 *   write occurs first.
 *
 * Input Parameters:
 *   conn     The TCP connection of interest
 *
 * Returned Value:
 *   OK (Always can send).
 *
 * Assumptions:
 *   None
 *
 ****************************************************************************/

int psock_tcp_cansend(FAR struct tcp_conn_s *conn)
{
  return OK;
}

#endif /* CONFIG_NET && CONFIG_NET_TCP && !CONFIG_NET_TCP_WRITE_BUFFERS */
