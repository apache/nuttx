/****************************************************************************
 * net/tcp/tcp_sendfile.c
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

#include <sys/stat.h>
#include <sys/types.h>
#include <sys/socket.h>

#include <fcntl.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <arch/irq.h>
#include <nuttx/semaphore.h>
#include <nuttx/fs/fs.h>
#include <nuttx/net/net.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/arp.h>
#include <nuttx/net/tcp.h>

#include "netdev/netdev.h"
#include "devif/devif.h"
#include "arp/arp.h"
#include "icmpv6/icmpv6.h"
#include "neighbor/neighbor.h"
#include "socket/socket.h"
#include "tcp/tcp.h"

#if defined(CONFIG_NET_SENDFILE) && defined(CONFIG_NET_TCP) && \
    defined(NET_TCP_HAVE_STACK)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TCPIPv4BUF ((FAR struct tcp_hdr_s *)&dev->d_buf[NET_LL_HDRLEN(dev) + IPv4_HDRLEN])
#define TCPIPv6BUF ((FAR struct tcp_hdr_s *)&dev->d_buf[NET_LL_HDRLEN(dev) + IPv6_HDRLEN])

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure holds the state of the send operation until it can be
 * operated upon from the driver poll event.
 */

struct sendfile_s
{
  FAR struct socket *snd_sock;             /* Points to the parent socket structure */
  FAR struct devif_callback_s *snd_cb;     /* Reference to callback instance */
  FAR struct file   *snd_file;             /* File structure of the input file */
  sem_t              snd_sem;              /* Used to wake up the waiting thread */
  off_t              snd_foffset;          /* Input file offset */
  size_t             snd_flen;             /* File length */
  ssize_t            snd_sent;             /* The number of bytes sent */
  uint32_t           snd_isn;              /* Initial sequence number */
  uint32_t           snd_acked;            /* The number of bytes acked */
#ifdef CONFIG_NET_TCP_FAST_RETRANSMIT
  uint32_t           snd_prev_ack;         /* The previous ACKed seq number */
#ifdef CONFIG_NET_TCP_WINDOW_SCALE
  uint32_t           snd_prev_wnd;         /* The advertised window in the last
                                            * incoming acknowledgment
                                            */
#else
  uint16_t           snd_prev_wnd;
#endif
  int                snd_dup_acks;         /* Duplicate ACK counter */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sendfile_eventhandler
 *
 * Description:
 *   This function is called to perform the actual send operation when
 *   polled by the lower, device interfacing layer.
 *
 * Input Parameters:
 *   dev      The structure of the network driver that caused the event
 *   conn     The connection structure associated with the socket
 *   flags    Set of events describing why the callback was invoked
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The network is locked
 *
 ****************************************************************************/

static uint16_t sendfile_eventhandler(FAR struct net_driver_s *dev,
                                      FAR void *pvconn, FAR void *pvpriv,
                                      uint16_t flags)
{
  /* FAR struct tcp_conn_s *conn = (FAR struct tcp_conn_s *)pvconn;
   *
   * Do not use pvconn argument to get the TCP connection pointer (the above
   * commented line) because pvconn is normally NULL for some events like
   * NETDEV_DOWN. Instead, the TCP connection pointer can be reliably
   * obtained from the corresponding TCP socket.
   */

  FAR struct sendfile_s *pstate = (FAR struct sendfile_s *)pvpriv;
  FAR struct socket *psock;
  FAR struct tcp_conn_s *conn;
  int ret;

  DEBUGASSERT(pstate != NULL);

  psock = pstate->snd_sock;
  DEBUGASSERT(psock != NULL);

  /* Get the TCP connection pointer reliably from
   * the corresponding TCP socket.
   */

  conn = psock->s_conn;
  DEBUGASSERT(conn != NULL);

#ifdef CONFIG_DEBUG_NET_ERROR
  if (conn->dev == NULL || (pvconn != conn && pvconn != NULL))
    {
      tcp_event_handler_dump(dev, pvconn, pvpriv, flags, conn);
    }
#endif

  /* If pvconn is not NULL, make sure that pvconn refers to the same
   * connection as the socket is bound to.
   */

  DEBUGASSERT(pvconn == conn || pvconn == NULL);

  /* The TCP socket is connected and, hence, should be bound to a device.
   * Make sure that the polling device is the own that we are bound to.
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

#ifdef CONFIG_NET_IPv6
#ifdef CONFIG_NET_IPv4
      if (IFF_IS_IPv6(dev->d_flags))
#endif
        {
          DEBUGASSERT(pstate->snd_sock->s_domain == PF_INET6);
          tcp = TCPIPv6BUF;
        }
#endif /* CONFIG_NET_IPv6 */

#ifdef CONFIG_NET_IPv4
#ifdef CONFIG_NET_IPv6
      else
#endif
        {
          DEBUGASSERT(pstate->snd_sock->s_domain == PF_INET);
          tcp = TCPIPv4BUF;
        }
#endif /* CONFIG_NET_IPv4 */

      /* The current acknowledgement number is the (relative) offset
       * of the of the next byte needed by the receiver.  The snd_isn is the
       * offset of the first byte to send to the receiver.  The difference
       * is the number of bytes to be acknowledged.
       */

      ackno = tcp_getsequence(tcp->ackno);
      pstate->snd_acked = TCP_SEQ_SUB(ackno, pstate->snd_isn);
      ninfo("ACK: acked=%" PRId32 " sent=%zd flen=%zu\n",
            pstate->snd_acked, pstate->snd_sent, pstate->snd_flen);

      /* Have all of the bytes in the buffer been sent and acknowledged? */

      if (pstate->snd_acked >= pstate->snd_flen)
        {
          /* Yes. Then pstate->snd_flen should hold the number of bytes
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
   * This condition is located here (after TCP_ACKDATA and before
   * TCP_DISCONN_EVENTS) for performance reasons.
   */

  if ((flags & TCP_REXMIT) != 0)
    {
      uint32_t sndlen;

      nwarn("WARNING: TCP_REXMIT\n");

      /* According to RFC 6298 (5.4), retransmit the earliest segment
       * that has not been acknowledged by the TCP receiver.
       */

      /* Reconstruct the length of the earliest segment to be retransmitted */

      sndlen = pstate->snd_flen - pstate->snd_acked;

      if (sndlen > conn->mss)
        {
          sndlen = conn->mss;
        }

      conn->rexmit_seq = pstate->snd_isn + pstate->snd_acked;

      /* Then set-up to send that amount of data. (this won't actually
       * happen until the polling cycle completes).
       */

      ret = file_seek(pstate->snd_file,
                      pstate->snd_foffset + pstate->snd_acked, SEEK_SET);
      if (ret < 0)
        {
          nerr("ERROR: Failed to lseek: %d\n", ret);
          pstate->snd_sent = ret;
          goto end_wait;
        }

      ret = file_read(pstate->snd_file, dev->d_appdata, sndlen);
      if (ret < 0)
        {
          nerr("ERROR: Failed to read from input file: %d\n", (int)ret);
          pstate->snd_sent = ret;
          goto end_wait;
        }

      dev->d_sndlen = sndlen;

      /* Continue waiting */

      return flags;
    }

  /* Check for a loss of connection.
   * This condition is located here after both the TCP_ACKDATA and TCP_REXMIT
   * because TCP_DISCONN_EVENTS is the least frequent event.
   */

  else if ((flags & TCP_DISCONN_EVENTS) != 0)
    {
      nwarn("WARNING: Lost connection\n");

      /* We could get here recursively through the callback actions of
       * tcp_lost_connection().  So don't repeat that action if we have
       * already been disconnected.
       */

      if (_SS_ISCONNECTED(conn->sconn.s_flags))
        {
          /* Report not connected */

          tcp_lost_connection(conn, pstate->snd_cb, flags);
        }

      /* Report not connected */

      pstate->snd_sent = -ENOTCONN;
      goto end_wait;
    }

  /* We get here if (1) not all of the data has been ACKed, (2) we have been
   * asked to retransmit data, (3) the connection is still healthy, and (4)
   * the outgoing packet is available for our use.  In this case, we are
   * now free to send more data to receiver -- UNLESS the buffer contains
   * unprocessing incoming data.  In that event, we will have to wait for the
   * next polling cycle.
   */

  if ((flags & TCP_NEWDATA) == 0 && pstate->snd_sent < pstate->snd_flen)
    {
      /* Get the amount of data that we can send in the next packet */

      uint32_t sndlen = pstate->snd_flen - pstate->snd_sent;

      if (sndlen > conn->mss)
        {
          sndlen = conn->mss;
        }

      /* Check if we have "space" in the window */

      if ((pstate->snd_sent - pstate->snd_acked + sndlen) < conn->snd_wnd)
        {
          /* Then set-up to send that amount of data. (this won't actually
           * happen until the polling cycle completes).
           */

          ret = file_seek(pstate->snd_file,
                          pstate->snd_foffset + pstate->snd_sent, SEEK_SET);
          if (ret < 0)
            {
              nerr("ERROR: Failed to lseek: %d\n", ret);
              pstate->snd_sent = ret;
              goto end_wait;
            }

          ret = file_read(pstate->snd_file, dev->d_appdata, sndlen);
          if (ret < 0)
            {
              nerr("ERROR: Failed to read from input file: %d\n", (int)ret);
              pstate->snd_sent = ret;
              goto end_wait;
            }

          dev->d_sndlen = sndlen;

          /* Update the amount of data sent (but not necessarily ACKed) */

          pstate->snd_sent += sndlen;
          ninfo("pid: %d SEND: acked=%" PRId32 " sent=%zd flen=%zu\n",
                getpid(),
                pstate->snd_acked, pstate->snd_sent, pstate->snd_flen);
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
 * Name: tcp_sendfile
 *
 * Description:
 *   The tcp_sendfile() call may be used only when the INET socket is in a
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
 *   a negated errno value is returned.  See sendfile() for a list
 *   appropriate error return values.
 *
 ****************************************************************************/

ssize_t tcp_sendfile(FAR struct socket *psock, FAR struct file *infile,
                      FAR off_t *offset, size_t count)
{
  FAR struct tcp_conn_s *conn;
  struct sendfile_s state;
  off_t startpos;
  int ret;

  conn = psock->s_conn;
  DEBUGASSERT(conn != NULL);

  /* If this is an un-connected socket, then return ENOTCONN */

  if (psock->s_type != SOCK_STREAM || !_SS_ISCONNECTED(conn->sconn.s_flags))
    {
      nerr("ERROR: Not connected\n");
      return -ENOTCONN;
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
      return -ENETUNREACH;
    }
#endif /* CONFIG_NET_ARP_SEND || CONFIG_NET_ICMPv6_NEIGHBOR */

  /* Get the current file position. */

  startpos = file_seek(infile, 0, SEEK_CUR);
  if (startpos < 0)
    {
      return startpos;
    }

  /* Initialize the state structure.  This is done with the network
   * locked because we don't want anything to happen until we are
   * ready.
   */

  net_lock();
#ifdef CONFIG_NET_TCP_WRITE_BUFFERS
  conn->sendfile = true;
#endif
  memset(&state, 0, sizeof(struct sendfile_s));

  /* This semaphore is used for signaling and, hence, should not have
   * priority inheritance enabled.
   */

  nxsem_init(&state.snd_sem, 0, 0);                /* Doesn't really fail */
  nxsem_set_protocol(&state.snd_sem, SEM_PRIO_NONE);

  state.snd_sock    = psock;                       /* Socket descriptor to use */
  state.snd_foffset = offset ? *offset : startpos; /* Input file offset */
  state.snd_flen    = count;                       /* Number of bytes to send */
  state.snd_file    = infile;                      /* File to read from */

  /* Allocate resources to receive a callback */

  state.snd_cb = tcp_callback_alloc(conn);

  if (state.snd_cb == NULL)
    {
      nerr("ERROR: Failed to allocate callback\n");
      ret = -ENOMEM;
      goto errout_locked;
    }

  /* Get the initial sequence number that will be used */

  state.snd_isn          = tcp_getsequence(conn->sndseq);

  /* There is no outstanding, unacknowledged data after this
   * initial sequence number.
   */

  conn->tx_unacked       = 0;

  /* Set up the callback in the connection */

  state.snd_cb->flags    = (TCP_ACKDATA | TCP_REXMIT | TCP_POLL |
                            TCP_DISCONN_EVENTS);
  state.snd_cb->priv     = (FAR void *)&state;
  state.snd_cb->event    = sendfile_eventhandler;

  /* Notify the device driver of the availability of TX data */

  tcp_send_txnotify(psock, conn);

  for (; ; )
    {
      uint32_t acked = state.snd_acked;

      ret = net_timedwait_uninterruptible(
              &state.snd_sem, _SO_TIMEOUT(conn->sconn.s_sndtimeo));
      if (ret != -ETIMEDOUT || acked == state.snd_acked)
        {
          if (ret == -ETIMEDOUT)
            {
              ret = -EAGAIN;
            }

          break; /* Successful completion or timeout without any progress */
        }
    }

  tcp_callback_free(conn, state.snd_cb);

errout_locked:
  nxsem_destroy(&state.snd_sem);
#ifdef CONFIG_NET_TCP_WRITE_BUFFERS
  conn->sendfile = false;
#endif
  net_unlock();

  /* Return the current file position */

  if (offset)
    {
      /* Use lseek to get the current file position */

      off_t curpos = file_seek(infile, 0, SEEK_CUR);
      if (curpos < 0)
        {
          return curpos;
        }

      /* Return the current file position */

      *offset = curpos;

      /* Use lseek again to restore the original file position */

      startpos = file_seek(infile, startpos, SEEK_SET);
      if (startpos < 0)
        {
          return startpos;
        }
    }

  if (ret < 0)
    {
      return ret;
    }
  else
    {
      return state.snd_sent;
    }
}

#endif /* CONFIG_NET_SENDFILE && CONFIG_NET_TCP && NET_TCP_HAVE_STACK */
