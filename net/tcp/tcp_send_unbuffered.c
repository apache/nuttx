/****************************************************************************
 * net/tcp/tcp_send_unbuffered.c
 *
 *   Copyright (C) 2007-2014, 2016-2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <arch/irq.h>

#include <nuttx/clock.h>
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

#if defined(CONFIG_NET_TCP_SPLIT) && !defined(CONFIG_NET_TCP_SPLIT_SIZE)
#  define CONFIG_NET_TCP_SPLIT_SIZE 40
#endif

#define TCPIPv4BUF ((struct tcp_hdr_s *)&dev->d_buf[NET_LL_HDRLEN(dev) + IPv4_HDRLEN])
#define TCPIPv6BUF ((struct tcp_hdr_s *)&dev->d_buf[NET_LL_HDRLEN(dev) + IPv6_HDRLEN])

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure holds the state of the send operation until it can be
 * operated upon when the TX poll event occurs.
 */

struct send_s
{
  FAR struct socket      *snd_sock;    /* Points to the parent socket structure */
  FAR struct devif_callback_s *snd_cb; /* Reference to callback instance */
  sem_t                   snd_sem;     /* Used to wake up the waiting thread */
  FAR const uint8_t      *snd_buffer;  /* Points to the buffer of data to send */
  size_t                  snd_buflen;  /* Number of bytes in the buffer to send */
  ssize_t                 snd_sent;    /* The number of bytes sent */
  uint32_t                snd_isn;     /* Initial sequence number */
  uint32_t                snd_acked;   /* The number of bytes acked */
#ifdef CONFIG_NET_SOCKOPTS
  clock_t                 snd_time;    /* Last send time for determining timeout */
#endif
#if defined(CONFIG_NET_TCP_SPLIT)
  bool                    snd_odd;     /* True: Odd packet in pair transaction */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: send_timeout
 *
 * Description:
 *   Check for send timeout.
 *
 * Input Parameters:
 *   pstate   send state structure
 *
 * Returned Value:
 *   TRUE:timeout FALSE:no timeout
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_SOCKOPTS
static inline int send_timeout(FAR struct send_s *pstate)
{
  FAR struct socket *psock;

  /* Check for a timeout configured via setsockopts(SO_SNDTIMEO).
   * If none... we will let the send wait forever.
   */

  psock = pstate->snd_sock;
  if (psock && psock->s_sndtimeo != 0)
    {
      /* Check if the configured timeout has elapsed */

      return net_timeo(pstate->snd_time, psock->s_sndtimeo);
    }

  /* No timeout */

  return FALSE;
}
#endif /* CONFIG_NET_SOCKOPTS */

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

      DEBUGASSERT(conn->domain == PF_INET6);
      tcp_ipv6_select(dev);
    }
}
#endif

/****************************************************************************
 * Name: psock_send_addrchck
 *
 * Description:
 *   Check if the destination IP address is in the IPv4 ARP or IPv6 Neighbor
 *   tables.  If not, then the send won't actually make it out... it will be
 *   replaced with an ARP request (IPv4) or a Neighbor Solicitation (IPv6).
 *
 *   NOTE 1: This could be an expensive check if there are a lot of
 *   entries in the ARP or Neighbor tables.
 *
 *   NOTE 2: If we are actually harvesting IP addresses on incoming IP
 *   packets, then this check should not be necessary; the MAC mapping
 *   should already be in the ARP table in many cases (IPv4 only).
 *
 *   NOTE 3: If CONFIG_NET_ARP_SEND then we can be assured that the IP
 *   address mapping is already in the ARP table.
 *
 * Input Parameters:
 *   conn  - The TCP connection structure
 *
 * Returned Value:
 *   true - The Ethernet MAC address is in the ARP or Neighbor table (OR
 *          the network device is not Ethernet).
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_ETHERNET
static inline bool psock_send_addrchck(FAR struct tcp_conn_s *conn)
{
  /* Only Ethernet drivers are supported by this function */
  /* REVISIT: Could the MAC address not also be in a routing table? */

  if (conn->dev->d_lltype != NET_LL_ETHERNET)
    {
      /* Return true for non-Ethernet devices. */

      return true;
    }

#ifdef CONFIG_NET_IPv4
#ifdef CONFIG_NET_IPv6
  if (conn->domain == PF_INET)
#endif
    {
      /* For historical reasons, we will return true if both the ARP and the
       * routing table are disabled.
       */

      bool ret = true;
#ifdef CONFIG_NET_ROUTE
      in_addr_t router;
#endif
#if !defined(CONFIG_NET_ARP_IPIN) && !defined(CONFIG_NET_ARP_SEND)
      if (arp_find(conn->u.ipv4.raddr, NULL) >= 0)
        {
          /* Return true if the address was found in the ARP table */

          return true;
        }

      /* Otherwise, return false */

      ret = false;
#endif
#ifdef CONFIG_NET_ROUTE
      if (net_ipv4_router(conn->u.ipv4.raddr, &router) == OK)
        {
          /* Return true if the address was found in the routing table */

          return true;
        }

      /* Otherwise, return false */

      ret = false;
#endif

      return ret;
    }
#endif /* CONFIG_NET_IPv4 */

#ifdef CONFIG_NET_IPv6
#ifdef CONFIG_NET_IPv4
  else
#endif
    {
      /* For historical reasons, we will return true if both the ICMPv6
       * neighbor support and the routing table are disabled.
       */

      bool ret = true;
#ifdef CONFIG_NET_ROUTE
      net_ipv6addr_t router;
#endif

#if !defined(CONFIG_NET_ICMPv6_NEIGHBOR)
      if (neighbor_findentry(conn->u.ipv6.raddr) != NULL)
        {
          /* Return true if the address was found in the ARP table */

          return true;
        }

      /* Otherwise, return false */

      ret = false;
#endif
#ifdef CONFIG_NET_ROUTE
      if (net_ipv6_router(conn->u.ipv6.raddr, router) == OK)
        {
          /* Return true if the address was found in the routing table */

          return true;
        }

      /* Otherwise, return false */

      ret = false;
#endif

      return ret;
    }
#endif /* CONFIG_NET_IPv6 */
}

#else /* CONFIG_NET_ETHERNET */
#  define  psock_send_addrchck(r) (true)
#endif /* CONFIG_NET_ETHERNET */

/****************************************************************************
 * Name: tcpsend_eventhandler
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
 *   The network is locked.
 *
 ****************************************************************************/

static uint16_t tcpsend_eventhandler(FAR struct net_driver_s *dev,
                                     FAR void *pvconn,
                                     FAR void *pvpriv, uint16_t flags)
{
  FAR struct tcp_conn_s *conn = (FAR struct tcp_conn_s *)pvconn;
  FAR struct send_s *pstate = (FAR struct send_s *)pvpriv;

  /* The TCP socket is connected and, hence, should be bound to a device.
   * Make sure that the polling device is the one that we are bound to.
   */

  DEBUGASSERT(conn->dev != NULL);
  if (dev != conn->dev)
    {
      return flags;
    }

  ninfo("flags: %04x acked: %d sent: %d\n",
        flags, pstate->snd_acked, pstate->snd_sent);

  /* If this packet contains an acknowledgement, then update the count of
   * acknowledged bytes.
   */

  if ((flags & TCP_ACKDATA) != 0)
    {
      FAR struct tcp_hdr_s *tcp;

      /* Update the timeout */

#ifdef CONFIG_NET_SOCKOPTS
      pstate->snd_time = clock_systimer();
#endif

      /* Get the offset address of the TCP header */

#ifdef CONFIG_NET_IPv4
#ifdef CONFIG_NET_IPv6
      if (conn->domain == PF_INET)
#endif
        {
          DEBUGASSERT(IFF_IS_IPv4(dev->d_flags));
          tcp = TCPIPv4BUF;
        }
#endif /* CONFIG_NET_IPv4 */

#ifdef CONFIG_NET_IPv6
#ifdef CONFIG_NET_IPv4
      else
#endif
        {
          DEBUGASSERT(IFF_IS_IPv6(dev->d_flags));
          tcp = TCPIPv6BUF;
        }
#endif /* CONFIG_NET_IPv6 */

      /* The current acknowledgement number is the (relative) offset of the
       * next byte needed by the receiver.  The snd_isn is the offset of the
       * first byte to send to the receiver.  The difference is the number
       * of bytes to be acknowledged.
       */

      pstate->snd_acked = tcp_getsequence(tcp->ackno) - pstate->snd_isn;
      ninfo("ACK: acked=%d sent=%d buflen=%d\n",
            pstate->snd_acked, pstate->snd_sent, pstate->snd_buflen);

      /* Have all of the bytes in the buffer been sent and acknowledged? */

      if (pstate->snd_acked >= pstate->snd_buflen)
        {
          /* Yes.  Then pstate->snd_buflen should hold the number of bytes
           * actually sent.
           */

          goto end_wait;
        }

      /* No.. fall through to send more data if necessary */
    }

  /* Check if we are being asked to retransmit data */

  else if ((flags & TCP_REXMIT) != 0)
    {
      /* Yes.. in this case, reset the number of bytes that have been sent
       * to the number of bytes that have been ACKed.
       */

      pstate->snd_sent = pstate->snd_acked;

#if defined(CONFIG_NET_TCP_SPLIT)
      /* Reset the even/odd indicator to even since we need to
       * retransmit.
       */

      pstate->snd_odd = false;
#endif

      /* Fall through to re-send data from the last that was ACKed */
    }

  /* Check for a loss of connection */

  else if ((flags & TCP_DISCONN_EVENTS) != 0)
    {
      FAR struct socket *psock = pstate->snd_sock;

      ninfo("Lost connection\n");

      /* We could get here recursively through the callback actions of
       * tcp_lost_connection().  So don't repeat that action if we have
       * already been disconnected.
       */

      DEBUGASSERT(psock != NULL);
      if (_SS_ISCONNECTED(psock->s_flags))
         {
           /* Report not connected */

           tcp_lost_connection(psock, pstate->snd_cb, flags);
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
      uint32_t seqno;

      /* Get the amount of data that we can send in the next packet */

      uint32_t sndlen = pstate->snd_buflen - pstate->snd_sent;

#if defined(CONFIG_NET_TCP_SPLIT)

      /* RFC 1122 states that a host may delay ACKing for up to 500ms but
       * must respond to every second  segment).  This logic here will trick
       * the RFC 1122 recipient into responding sooner.  This logic will be
       * activated if:
       *
       *   1. An even number of packets has been send (where zero is an even
       *      number),
       *   2. There is more data be sent (more than or equal to
       *      CONFIG_NET_TCP_SPLIT_SIZE), but
       *   3. Not enough data for two packets.
       *
       * Then we will split the remaining, single packet into two partial
       * packets.  This will stimulate the RFC 1122 peer to ACK sooner.
       *
       * Don't try to split very small packets (less than CONFIG_NET_TCP_SPLIT_SIZE).
       * Only the first even packet and the last odd packets could have
       * sndlen less than CONFIG_NET_TCP_SPLIT_SIZE.  The value of sndlen on
       * the last even packet is guaranteed to be at least MSS/2 by the
       * logic below.
       */

      if (sndlen >= CONFIG_NET_TCP_SPLIT_SIZE)
        {
          /* sndlen is the number of bytes remaining to be sent.
           * conn->mss will provide the number of bytes that can sent
           * in one packet.  The difference, then, is the number of bytes
           * that would be sent in the next packet after this one.
           */

          int32_t next_sndlen = sndlen - conn->mss;

          /*  Is this the even packet in the packet pair transaction? */

          if (!pstate->snd_odd)
            {
              /* next_sndlen <= 0 means that the entire remaining data
               * could fit into this single packet.  This is condition
               * in which we must do the split.
               */

              if (next_sndlen <= 0)
                {
                  /* Split so that there will be an odd packet.  Here
                   * we know that 0 < sndlen <= MSS
                   */

                  sndlen = (sndlen / 2) + 1;
                }
            }

          /* No... this is the odd packet in the packet pair transaction */

          else
            {
              /* Will there be another (even) packet afer this one?
               * (next_sndlen > 0)  Will the split condition occur on that
               * next, even packet? ((next_sndlen - conn->mss) < 0) If
               * so, then perform the split now to avoid the case where the
               * byte count is less than CONFIG_NET_TCP_SPLIT_SIZE on the
               * next pair.
               */

              if (next_sndlen > 0 && (next_sndlen - conn->mss) < 0)
                {
                  /* Here, we know that sndlen must be MSS < sndlen <= 2*MSS
                   * and so (sndlen / 2) is <= MSS.
                   */

                  sndlen /= 2;
                }
            }
        }

      /* Toggle the even/odd indicator */

      pstate->snd_odd ^= true;

#endif /* CONFIG_NET_TCP_SPLIT */

      if (sndlen > conn->mss)
        {
          sndlen = conn->mss;
        }

      /* Check if we have "space" in the window */

      if ((pstate->snd_sent - pstate->snd_acked + sndlen) < conn->winsize)
        {
          /* Set the sequence number for this packet.  NOTE:  The network updates
           * sndseq on receipt of ACK *before* this function is called.  In that
           * case sndseq will point to the next unacknowledged byte (which might
           * have already been sent).  We will overwrite the value of sndseq
           * here before the packet is sent.
           */

          seqno = pstate->snd_sent + pstate->snd_isn;
          ninfo("SEND: sndseq %08x->%08x\n", conn->sndseq, seqno);
          tcp_setsequence(conn->sndseq, seqno);

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

          /* Check if the destination IP address is in the ARP  or Neighbor
           * table.  If not, then the send won't actually make it out... it
           * will be replaced with an ARP request or Neighbor Solicitation.
           */

          if (pstate->snd_sent != 0 || psock_send_addrchck(conn))
            {
              /* Update the amount of data sent (but not necessarily ACKed) */

              pstate->snd_sent += sndlen;
              ninfo("SEND: acked=%d sent=%d buflen=%d\n",
                    pstate->snd_acked, pstate->snd_sent, pstate->snd_buflen);

            }
        }
    }

#ifdef CONFIG_NET_SOCKOPTS
  /* All data has been sent and we are just waiting for ACK or re-transmit
   * indications to complete the send.  Check for a timeout.
   */

  if (send_timeout(pstate))
    {
      /* Yes.. report the timeout */

      nwarn("WARNING: SEND timeout\n");
      pstate->snd_sent = -ETIMEDOUT;
      goto end_wait;
    }
#endif /* CONFIG_NET_SOCKOPTS */

  /* Continue waiting */

  return flags;

end_wait:
  /* Do not allow any further callbacks */

  pstate->snd_cb->flags   = 0;
  pstate->snd_cb->priv    = NULL;
  pstate->snd_cb->event   = NULL;

  /* There are no outstanding, unacknowledged bytes */

  conn->unacked           = 0;

  /* Wake up the waiting thread */

  nxsem_post(&pstate->snd_sem);
  return flags;
}

/****************************************************************************
 * Name: send_txnotify
 *
 * Description:
 *   Notify the appropriate device driver that we are have data ready to
 *   be send (TCP)
 *
 * Input Parameters:
 *   psock - Socket state structure
 *   conn  - The TCP connection structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void send_txnotify(FAR struct socket *psock,
                                 FAR struct tcp_conn_s *conn)
{
#ifdef CONFIG_NET_IPv4
#ifdef CONFIG_NET_IPv6
  /* If both IPv4 and IPv6 support are enabled, then we will need to select
   * the device driver using the appropriate IP domain.
   */

  if (psock->s_domain == PF_INET)
#endif
    {
      /* Notify the device driver that send data is available */

      netdev_ipv4_txnotify(conn->u.ipv4.laddr, conn->u.ipv4.raddr);
    }
#endif /* CONFIG_NET_IPv4 */

#ifdef CONFIG_NET_IPv6
#ifdef CONFIG_NET_IPv4
  else /* if (psock->s_domain == PF_INET6) */
#endif /* CONFIG_NET_IPv4 */
    {
      /* Notify the device driver that send data is available */

      DEBUGASSERT(psock->s_domain == PF_INET6);
      netdev_ipv6_txnotify(conn->u.ipv6.laddr, conn->u.ipv6.raddr);
    }
#endif /* CONFIG_NET_IPv6 */
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
                       FAR const void *buf, size_t len)
{
  FAR struct tcp_conn_s *conn;
  struct send_s state;
  int ret = OK;

  /* Verify that the sockfd corresponds to valid, allocated socket */

  if (psock == NULL || psock->s_crefs <= 0)
    {
      nerr("ERROR: Invalid socket\n");
      ret = -EBADF;
      goto errout;
    }

  /* If this is an un-connected socket, then return ENOTCONN */

  if (psock->s_type != SOCK_STREAM || !_SS_ISCONNECTED(psock->s_flags))
    {
      nerr("ERROR: Not connected\n");
      ret = -ENOTCONN;
      goto errout;
    }

  /* Make sure that we have the IP address mapping */

  conn = (FAR struct tcp_conn_s *)psock->s_conn;
  DEBUGASSERT(conn);

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

  /* Set the socket state to sending */

  psock->s_flags = _SS_SETSTATE(psock->s_flags, _SF_SEND);

  /* Perform the TCP send operation */

  /* Initialize the state structure.  This is done with the network
   * locked because we don't want anything to happen until we are
   * ready.
   */

  net_lock();
  memset(&state, 0, sizeof(struct send_s));

  /* This semaphore is used for signaling and, hence, should not have
   * priority inheritance enabled.
   */

  (void)nxsem_init(&state.snd_sem, 0, 0);    /* Doesn't really fail */
  (void)nxsem_setprotocol(&state.snd_sem, SEM_PRIO_NONE);

  state.snd_sock      = psock;             /* Socket descriptor to use */
  state.snd_buflen    = len;               /* Number of bytes to send */
  state.snd_buffer    = buf;               /* Buffer to send from */

  if (len > 0)
    {
      /* Allocate resources to receive a callback */

      state.snd_cb = tcp_callback_alloc(conn);
      if (state.snd_cb)
        {
          /* Get the initial sequence number that will be used */

          state.snd_isn         = tcp_getsequence(conn->sndseq);

          /* There is no outstanding, unacknowledged data after this
           * initial sequence number.
           */

          conn->unacked         = 0;

          /* Set the initial time for calculating timeouts */

#ifdef CONFIG_NET_SOCKOPTS
          state.snd_time        = clock_systimer();
#endif
          /* Set up the callback in the connection */

          state.snd_cb->flags   = (TCP_ACKDATA | TCP_REXMIT | TCP_POLL |
                                   TCP_DISCONN_EVENTS);
          state.snd_cb->priv    = (FAR void *)&state;
          state.snd_cb->event   = tcpsend_eventhandler;

          /* Notify the device driver of the availability of TX data */

          send_txnotify(psock, conn);

          /* Wait for the send to complete or an error to occur:  NOTES:
           * net_lockedwait will also terminate if a signal is received.
           */

          ret = net_lockedwait(&state.snd_sem);

          /* Make sure that no further events are processed */

          tcp_callback_free(conn, state.snd_cb);
        }
    }

  nxsem_destroy(&state.snd_sem);
  net_unlock();

  /* Set the socket state to idle */

  psock->s_flags = _SS_SETSTATE(psock->s_flags, _SF_IDLE);

  /* Check for a errors.  Errors are signalled by negative errno values
   * for the send length
   */

  if (state.snd_sent < 0)
    {
      ret = state.snd_sent;
      goto errout;
    }

  /* If net_lockedwait failed, then we were probably reawakened by a signal. In
   * this case, net_lockedwait will have returned negated errno appropriately.
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
 *   psock    An instance of the internal socket structure.
 *
 * Returned Value:
 *   OK (Function not implemented).
 *
 * Assumptions:
 *   None
 *
 ****************************************************************************/

int psock_tcp_cansend(FAR struct socket *psock)
{
  /* TODO: return OK unless someone is waiting for a packet to send */

  return OK;
}

#endif /* CONFIG_NET && CONFIG_NET_TCP && !CONFIG_NET_TCP_WRITE_BUFFERS */
