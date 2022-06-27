/****************************************************************************
 * net/sixlowpan/sixlowpan_tcpsend.c
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

#include <inttypes.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/semaphore.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/radiodev.h>
#include <nuttx/net/netstats.h>

#include "netdev/netdev.h"
#include "devif/devif.h"
#include "socket/socket.h"
#include "inet/inet.h"
#include "icmpv6/icmpv6.h"
#include "tcp/tcp.h"
#include "utils/utils.h"
#include "sixlowpan/sixlowpan_internal.h"

#if defined(CONFIG_NET_6LOWPAN) && defined(CONFIG_NET_TCP)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Buffer access helpers */

#define TCPBUF(dev) ((FAR struct tcp_hdr_s *)(&(dev)->d_buf[IPv6_HDRLEN]))

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* These are temporary stubs.  Something like this would be needed to
 * monitor the health of a IPv6 neighbor.
 */

#define neighbor_reachable(dev)
#define neighbor_notreachable(dev)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This is the state data provided to the send event handler.  No actions
 * can be taken until the until we receive the TX poll, then we can call
 * sixlowpan_queue_frames() with this data strurcture.
 */

struct sixlowpan_send_s
{
  FAR struct socket           *s_sock;          /* Internal socket reference */
  FAR struct devif_callback_s *s_cb;            /* Reference to callback
                                                 * instance */
  sem_t                        s_waitsem;       /* Supports waiting for
                                                 * driver events */
  int                          s_result;        /* The result of the transfer */
  FAR const struct netdev_varaddr_s *s_destmac; /* Destination MAC address */
  FAR const uint8_t           *s_buf;           /* Data to send */
  size_t                       s_buflen;        /* Length of data in buf */
  ssize_t                      s_sent;          /* The number of bytes sent */
  uint32_t                     s_isn;           /* Initial sequence number */
  uint32_t                     s_acked;         /* The number of bytes acked */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sixlowpan_tcp_chksum
 *
 * Description:
 *   Perform the checksum calcaultion over the IPv6, protocol headers, and
 *   data payload as necessary.
 *
 * Input Parameters:
 *   ipv6tcp - A reference to a structure containing the IPv6 and TCP
 *             headers.
 *   buf     - The beginning of the payload data
 *   buflen  - The length of the payload data.
 *
 * Returned Value:
 *   The calculated checksum
 *
 ****************************************************************************/

static uint16_t sixlowpan_tcp_chksum(FAR const struct ipv6tcp_hdr_s *ipv6tcp,
                                     FAR const uint8_t *buf, uint16_t buflen)
{
  uint16_t upperlen;
  uint16_t tcplen;
  uint16_t sum;

  /* The length reported in the IPv6 header is the length of the payload
   * that follows the header.
   */

  upperlen = ((uint16_t)ipv6tcp->ipv6.len[0] << 8) + ipv6tcp->ipv6.len[1];

  /* Verify some minimal assumptions */

  if (upperlen > CONFIG_NET_6LOWPAN_PKTSIZE)
    {
      return 0;
    }

  /* The checksum is calculated starting with a pseudo-header of IPv6 header
   * fields according to the IPv6 standard, which consists of the source
   * and destination addresses, the packet length and the next header field.
   */

  sum = upperlen + ipv6tcp->ipv6.proto;

  /* Sum IP source and destination addresses. */

  sum = chksum(sum, (FAR uint8_t *)ipv6tcp->ipv6.srcipaddr,
               2 * sizeof(net_ipv6addr_t));

  /* Sum the TCP header
   *
   * The TCP header length is encoded in the top 4 bits of the tcpoffset
   * field (in units of 32-bit words).
   */

  tcplen = ((uint16_t)ipv6tcp->tcp.tcpoffset >> 4) << 2;
  sum = chksum(sum, (FAR uint8_t *)&ipv6tcp->tcp, tcplen);

  /* Sum payload data. */

  sum = chksum(sum, buf, buflen);
  return (sum == 0) ? 0xffff : HTONS(sum);
}

/****************************************************************************
 * Name: sixlowpan_tcp_header
 *
 * Description:
 *   sixlowpan_tcp_header() will construct the IPv6 and TCP headers
 *
 * Input Parameters:
 *   conn    - An instance of the TCP connection structure.
 *   dev     - The network device that will route the packet
 *   buf     - Data to send
 *   bulen   - Length of data to send
 *   ipv6tcp - The location to save the IPv6 + TCP header
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   failure.
 *
 * Assumptions:
 *   Called with the network locked.
 *
 ****************************************************************************/

static int sixlowpan_tcp_header(FAR struct tcp_conn_s *conn,
                                FAR struct net_driver_s *dev,
                                FAR const void *buf, size_t buflen,
                                FAR struct ipv6tcp_hdr_s *ipv6tcp)
{
  uint16_t iplen;

  /* Initialize the IPv6/TCP headers */

  ipv6tcp->ipv6.vtc    = 0x60;
  ipv6tcp->ipv6.tcf    = 0x00;
  ipv6tcp->ipv6.flow   = 0x00;
  ipv6tcp->ipv6.proto  = IP_PROTO_TCP;
  ipv6tcp->ipv6.ttl    = IP_TTL_DEFAULT;

  /* The IPv6 header length field does not include the size of IPv6 IP
   * header.
   */

  iplen                = buflen + TCP_HDRLEN;
  ipv6tcp->ipv6.len[0] = (iplen >> 8);
  ipv6tcp->ipv6.len[1] = (iplen & 0xff);

  /* Copy the source and destination addresses */

  net_ipv6addr_hdrcopy(ipv6tcp->ipv6.destipaddr, conn->u.ipv6.raddr);
  if (!net_ipv6addr_cmp(conn->u.ipv6.laddr, g_ipv6_unspecaddr))
    {
      net_ipv6addr_hdrcopy(ipv6tcp->ipv6.srcipaddr, conn->u.ipv6.laddr);
    }
  else
    {
      net_ipv6addr_hdrcopy(ipv6tcp->ipv6.srcipaddr, dev->d_ipv6addr);
    }

  ninfo("IPv6 length: %d\n",
        ((int)ipv6tcp->ipv6.len[0] << 8) + ipv6tcp->ipv6.len[1]);

#ifdef CONFIG_NET_STATISTICS
  g_netstats.ipv6.sent++;
#endif

  /* Initialize the TCP header */

  ipv6tcp->tcp.srcport   = conn->lport;           /* Local port */
  ipv6tcp->tcp.destport  = conn->rport;           /* Connected remote port */

  ipv6tcp->tcp.tcpoffset = (TCP_HDRLEN / 4) << 4; /* No optdata */
  ipv6tcp->tcp.flags     = TCP_ACK | TCP_PSH;     /* No urgent data */
  ipv6tcp->tcp.urgp[0]   = 0;                     /* No urgent data */
  ipv6tcp->tcp.urgp[1]   = 0;

  /* Set the sequency number information.
   *
   * REVISIT:  There is currently no wait for the data to be ACKed and,
   * hence, no mechanism to retransmit the packet.
   */

  memcpy(ipv6tcp->tcp.ackno, conn->rcvseq, 4);    /* ACK number */
  memcpy(ipv6tcp->tcp.seqno, conn->sndseq, 4);    /* Sequence number */

  /* Set the TCP window */

  if (conn->tcpstateflags & TCP_STOPPED)
    {
      /* If the connection has issued TCP_STOPPED, we advertise a zero
       * window so that the remote host will stop sending data.
       */

      ipv6tcp->tcp.wnd[0] = 0;
      ipv6tcp->tcp.wnd[1] = 0;
    }
  else
    {
      /* Update the TCP received window based on I/O buffer availability */

      uint32_t rcvseq = tcp_getsequence(conn->rcvseq);
      uint32_t recvwndo = tcp_get_recvwindow(dev, conn);

      /* Set the TCP Window */

      ipv6tcp->tcp.wnd[0] = recvwndo >> 8;
      ipv6tcp->tcp.wnd[1] = recvwndo & 0xff;

      /* Update the Receiver Window */

      conn->rcv_adv = rcvseq + recvwndo;
    }

  /* Calculate TCP checksum. */

  ipv6tcp->tcp.tcpchksum   = 0;
  ipv6tcp->tcp.tcpchksum   = ~sixlowpan_tcp_chksum(ipv6tcp, buf, buflen);

  ninfo("Outgoing TCP packet length: %d bytes\n", iplen + IPv6_HDRLEN);
  return OK;
}

/****************************************************************************
 * Name: tcp_send_eventhandler
 *
 * Description:
 *   This function is called with the network locked to perform the actual
 *   TCP send operation when polled by the lower, device interfacing layer.
 *
 * Input Parameters:
 *   dev    - The structure of the network driver that generated the event.
 *   pvconn - The connection structure associated with the socket
 *   pvpriv - The event handler's private data argument
 *   flags  - Set of events describing why the callback was invoked
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

static uint16_t tcp_send_eventhandler(FAR struct net_driver_s *dev,
                                      FAR void *pvconn,
                                      FAR void *pvpriv, uint16_t flags)
{
  FAR struct sixlowpan_send_s *sinfo = (FAR struct sixlowpan_send_s *)pvpriv;
  FAR struct tcp_conn_s *conn = (FAR struct tcp_conn_s *)pvconn;
  struct ipv6tcp_hdr_s ipv6tcp;
  int ret;

  /* Verify that this is an IEEE802.15.4 network driver. */

  if (dev->d_lltype != NET_LL_IEEE802154 &&
      dev->d_lltype != NET_LL_PKTRADIO)
    {
      ninfo("Not a compatible network device\n");
      return flags;
    }

  /* Check if the IEEE802.15.4 network driver went down */

  if ((flags & NETDEV_DOWN) != 0)
    {
      nwarn("WARNING: Device is down\n");
      sinfo->s_result = -ENOTCONN;
      goto end_wait;
    }

  /* The TCP socket is connected and, hence, should be bound to a device.
   * Make sure that the polling device is the one that we are bound to.
   */

  DEBUGASSERT(conn->dev != NULL);
  if (dev != conn->dev)
    {
      ninfo("Not the connected device\n");
      return flags;
    }

  ninfo("flags: %04x acked: %" PRIu32 " sent: %zu\n",
        flags, sinfo->s_acked, sinfo->s_sent);

  /* If this packet contains an acknowledgement, then update the count of
   * acknowledged bytes.
   */

  if ((flags & TCP_ACKDATA) != 0)
    {
      FAR struct tcp_hdr_s *tcp = TCPBUF(dev);

      /* The current acknowledgement number number is the (relative) offset
       * of the of the next byte needed by the receiver.  The s_isn is the
       * offset of the first byte to send to the receiver.  The difference
       * is the number of bytes to be acknowledged.
       */

      sinfo->s_acked = tcp_getsequence(tcp->ackno) - sinfo->s_isn;
      ninfo("ACK: acked=%" PRId32 " sent=%zd buflen=%zd\n",
            sinfo->s_acked, sinfo->s_sent, sinfo->s_buflen);

      /* Have all of the bytes in the buffer been sent and acknowledged? */

      if (sinfo->s_acked >= sinfo->s_buflen)
        {
          /* Yes.  Then sinfo->s_buflen should hold the number of bytes
           * actually sent.
           */

          sinfo->s_result = sinfo->s_sent;
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

      sinfo->s_sent = sinfo->s_acked;

      /* Fall through to re-send data from the last that was ACKed */
    }

  /* Check for a loss of connection */

  else if ((flags & TCP_DISCONN_EVENTS) != 0)
    {
      FAR struct socket *psock = sinfo->s_sock;

      nwarn("WARNING: Lost connection\n");

      /* We could get here recursively through the callback actions of
       * tcp_lost_connection().  So don't repeat that action if we have
       * already been disconnected.
       */

      DEBUGASSERT(psock != NULL);
      if (_SS_ISCONNECTED(conn->sconn.s_flags))
        {
          /* Report the disconnection event to all socket clones */

          tcp_lost_connection(conn, sinfo->s_cb, flags);
        }

      /* Report not connected to the sender */

      sinfo->s_result = -ENOTCONN;
      goto end_wait;
    }

  /* Check if the outgoing packet is available (it may have been claimed
   * by a sendto event handler serving a different thread).
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

  if ((flags & WPAN_NEWDATA) == 0 && sinfo->s_sent < sinfo->s_buflen)
    {
      uint32_t seqno;
      uint16_t winleft;
      uint16_t sndlen;

      /* Get the amount of TCP payload data that we can send in the next
       * packet.
       */

      sndlen = sinfo->s_buflen - sinfo->s_sent;
      if (sndlen > conn->mss)
        {
          sndlen = conn->mss;
        }

      winleft = conn->snd_wnd - sinfo->s_sent + sinfo->s_acked;
      if (sndlen > winleft)
        {
          sndlen = winleft;
        }

      ninfo("s_buflen=%zu s_sent=%zu mss=%u snd_wnd=%u sndlen=%d\n",
            sinfo->s_buflen, sinfo->s_sent, conn->mss, conn->snd_wnd,
            sndlen);

      if (sndlen > 0)
        {
          /* Set the sequence number for this packet.  NOTE:  The network
           * updates sndseq on receipt of ACK *before* this function is
           * called.  In that case sndseq will point to the next
           * unacknowledged byte (which might have already been sent).  We
           * will overwrite the value of sndseq here before the packet is
           * sent.
           */

          seqno = sinfo->s_sent + sinfo->s_isn;
          ninfo("Sending: sndseq %08" PRIx32 "->%08" PRIx32 "\n",
                tcp_getsequence(conn->sndseq), seqno);

          tcp_setsequence(conn->sndseq, seqno);

          /* Create the IPv6 + TCP header */

          ret = sixlowpan_tcp_header(conn, dev, &sinfo->s_buf[sinfo->s_sent],
                                     sndlen, &ipv6tcp);
          if (ret < 0)
            {
              nerr("ERROR: sixlowpan_tcp_header failed: %d\n", ret);
              sinfo->s_result = ret;
              goto end_wait;
            }

          /* Transfer the frame list to the IEEE802.15.4 MAC device */

          ret = sixlowpan_queue_frames((FAR struct radio_driver_s *)dev,
                                       &ipv6tcp.ipv6,
                                       &sinfo->s_buf[sinfo->s_sent], sndlen,
                                       sinfo->s_destmac);
          if (ret < 0)
            {
              nerr("ERROR: sixlowpan_queue_frames failed: %d\n", ret);
              sinfo->s_result = ret;
              goto end_wait;
            }

          /* Increment the count of bytes sent, the number of unacked bytes,
           * and the total count of TCP packets sent.
           *
           * NOTE: tcp_appsend() normally increments conn->tx_unacked based
           * on the value of dev->d_sndlen.  However, dev->d_len is always
           * zero for 6LoWPAN since it does not send via the dev->d_buf
           * but, rather, uses a backdoor frame interface with the IEEE
           * 802.15.4 MAC.
           */

          sinfo->s_sent    += sndlen;
          conn->tx_unacked += sndlen;

#ifdef CONFIG_NET_TCP_WRITE_BUFFERS
          /* For compatibility with buffered send logic */

          conn->sndseq_max = tcp_addsequence(conn->sndseq, conn->tx_unacked);
#endif

#ifdef CONFIG_NET_STATISTICS
          g_netstats.tcp.sent++;
#endif

          ninfo("Sent: acked=%" PRId32 " sent=%zd "
                "buflen=%zd tx_unacked=%" PRId32 "\n",
                sinfo->s_acked, sinfo->s_sent, sinfo->s_buflen,
                (uint32_t)conn->tx_unacked);
        }
    }

  /* Continue waiting */

  return flags;

end_wait:

  /* Do not allow any further callbacks */

  sinfo->s_cb->flags   = 0;
  sinfo->s_cb->priv    = NULL;
  sinfo->s_cb->event   = NULL;

  /* There are no outstanding, unacknowledged bytes */

  conn->tx_unacked     = 0;

  /* Wake up the waiting thread */

  nxsem_post(&sinfo->s_waitsem);
  return flags;
}

/****************************************************************************
 * Name: sixlowpan_send_packet
 *
 * Description:
 *   Process an outgoing TCP packet.  Takes an IP packet and formats it to
 *   be sent on an 802.15.4 network using 6lowpan.  Called from common TCP
 *   send logic.
 *
 *   The payload data is in the caller 'buf' and is of length 'buflen'.
 *   Compressed headers will be added and if necessary the packet is
 *   fragmented. The resulting packet/fragments are submitted to the MAC
 *   via the network driver r_req_data method.
 *
 * Input Parameters:
 *   psock   - An instance of the internal socket structure.
 *   dev     - The IEEE802.15.4 MAC network driver interface.
 *   conn    - TCP connection structure
 *   buf     - Data to send
 *   len     - Length of data to send
 *   destmac - The IEEE802.15.4 MAC address of the destination
 *   timeout - Send timeout in milliseconds
 *
 * Returned Value:
 *   Ok is returned on success; Otherwise a negated errno value is returned.
 *   This function is expected to fail if the driver is not an IEEE802.15.4
 *   MAC network driver.  In that case, the logic will fall back to normal
 *   IPv4/IPv6 formatting.
 *
 * Assumptions:
 *   Called with the network locked.
 *
 ****************************************************************************/

static int sixlowpan_send_packet(FAR struct socket *psock,
                                 FAR struct net_driver_s *dev,
                                 FAR struct tcp_conn_s *conn,
                                 FAR const uint8_t *buf, size_t len,
                                 FAR const struct netdev_varaddr_s *destmac,
                                 unsigned int timeout)
{
  struct sixlowpan_send_s sinfo;

  ninfo("len=%lu timeout=%u\n", (unsigned long)len, timeout);
  DEBUGASSERT(psock != NULL && dev != NULL && conn != NULL && buf != NULL &&
              destmac != NULL);

  memset(&sinfo, 0, sizeof(struct sixlowpan_send_s));

  net_lock();
  if (len > 0)
    {
      /* Allocate resources to receive a callback.
       *
       * The second parameter is NULL meaning that we can get only
       * device related events, no connect-related events.
       */

      sinfo.s_cb = tcp_callback_alloc(conn);
      if (sinfo.s_cb != NULL)
        {
          int ret;

          /* Initialize the send state structure */

          nxsem_init(&sinfo.s_waitsem, 0, 0);
          nxsem_set_protocol(&sinfo.s_waitsem, SEM_PRIO_NONE);

          sinfo.s_sock      = psock;
          sinfo.s_result    = -EBUSY;
          sinfo.s_destmac   = destmac;
          sinfo.s_buf       = buf;
          sinfo.s_buflen    = len;

          /* Set up the initial sequence number */

          sinfo.s_isn       = tcp_getsequence(conn->sndseq);

          /* Set up the callback in the connection */

          sinfo.s_cb->flags = (NETDEV_DOWN | TCP_ACKDATA | TCP_REXMIT |
                               TCP_DISCONN_EVENTS | WPAN_POLL);
          sinfo.s_cb->priv  = (FAR void *)&sinfo;
          sinfo.s_cb->event = tcp_send_eventhandler;

          /* There is no outstanding, unacknowledged data after this
           * initial sequence number.
           */

          conn->tx_unacked  = 0;

          /* Notify the IEEE802.15.4 MAC that we have data to send. */

          netdev_txnotify_dev(dev);

          /* Wait for the send to complete or an error to occur.
           * net_timedwait will also terminate if a signal is received.
           */

          ninfo("Wait for send complete\n");

          for (; ; )
            {
              uint32_t acked = sinfo.s_acked;

              ret = net_timedwait(&sinfo.s_waitsem, timeout);
              if (ret != -ETIMEDOUT || acked == sinfo.s_acked)
                {
                  if (ret == -ETIMEDOUT)
                    {
                      ret = -EAGAIN;
                    }

                  break; /* Timeout without any progress */
                }
            }

          if (ret < 0)
            {
              sinfo.s_result = ret;
            }

          /* Make sure that no further events are processed */

          tcp_callback_free(conn, sinfo.s_cb);
        }
    }

  nxsem_destroy(&sinfo.s_waitsem);
  net_unlock();

  return (sinfo.s_result < 0 ? sinfo.s_result : len);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: psock_6lowpan_tcp_send
 *
 * Description:
 *   psock_6lowpan_tcp_send() call may be used only when the TCP socket is
 *   in a connected state (so that the intended recipient is known).
 *
 * Input Parameters:
 *   psock - An instance of the internal socket structure.
 *   buf   - Data to send
 *   bulen - Length of data to send
 *
 * Returned Value:
 *   On success, returns the number of characters sent.  On  error,
 *   -1 is returned, and errno is set appropriately.  Returned error numbers
 *   must be consistent with definition of errors reported by send() or
 *   sendto().
 *
 * Assumptions:
 *   Called with the network locked.
 *
 ****************************************************************************/

ssize_t psock_6lowpan_tcp_send(FAR struct socket *psock, FAR const void *buf,
                               size_t buflen)
{
  FAR struct tcp_conn_s *conn;
  FAR struct net_driver_s *dev;
  struct netdev_varaddr_s destmac;
  int ret;

  ninfo("buflen %lu\n", (unsigned long)buflen);
  sixlowpan_dumpbuffer("Outgoing TCP payload", buf, buflen);

  DEBUGASSERT(psock != NULL && psock->s_conn != NULL);
  DEBUGASSERT(psock->s_type == SOCK_STREAM);

  /* Make sure that this is a valid socket */

  if (psock == NULL || psock->s_conn == NULL)
    {
      nerr("ERROR: Invalid socket\n");
      return (ssize_t)-EBADF;
    }

  /* Get the underlying TCP connection structure */

  conn = (FAR struct tcp_conn_s *)psock->s_conn;

  /* Make sure that this is a connected TCP socket */

  if (psock->s_type != SOCK_STREAM || !_SS_ISCONNECTED(conn->sconn.s_flags))
    {
      nerr("ERROR: Not connected\n");
      return (ssize_t)-ENOTCONN;
    }

#ifdef CONFIG_NET_IPv4
  /* Ignore if not IPv6 domain */

  if (conn->domain != PF_INET6)
    {
      nwarn("WARNING: Not IPv6\n");
      return (ssize_t)-EPROTOTYPE;
    }
#endif

  /* Route outgoing message to the correct device */

  dev = netdev_findby_ripv6addr(conn->u.ipv6.laddr, conn->u.ipv6.raddr);
  if (dev == NULL)
    {
      nwarn("WARNING: Not routable\n");
      return (ssize_t)-ENETUNREACH;
    }

  /* Some network devices support different link layer protocols.
   * Check if this device has the hooks to support 6LoWPAN.
   */

  if (dev->d_lltype != NET_LL_IEEE802154 &&
      dev->d_lltype != NET_LL_PKTRADIO)
    {
      nwarn("WARNING: Not a compatible network device\n");
      return (ssize_t)-ENONET;
    }

#ifdef CONFIG_NET_ICMPv6_NEIGHBOR
  /* Make sure that the IP address mapping is in the Neighbor Table */

  ret = icmpv6_neighbor(conn->u.ipv6.raddr);
  if (ret < 0)
    {
      nerr("ERROR: Not reachable\n");
      return (ssize_t)-ENETUNREACH;
    }
#endif

  /* Get the IEEE 802.15.4 MAC address of the next hop. */

  ret = sixlowpan_nexthopaddr((FAR struct radio_driver_s *)dev,
                              conn->u.ipv6.raddr, &destmac);
  if (ret < 0)
    {
      nerr("ERROR: Failed to get dest MAC address: %d\n", ret);
      return (ssize_t)ret;
    }

  /* Send the TCP packets, breaking down the potential large user buffer
   * into smaller packets that can be reassembled in the allocated MTU
   * packet buffer.
   */

  ret = sixlowpan_send_packet(psock, dev, conn, buf, buflen, &destmac,
                              _SO_TIMEOUT(conn->sconn.s_sndtimeo));
  if (ret < 0)
    {
      nerr("ERROR: sixlowpan_send_packet() failed: %d\n", ret);
      return (ssize_t)ret;
    }

  return (ssize_t)buflen;
}

/****************************************************************************
 * Name: sixlowpan_tcp_send
 *
 * Description:
 *   TCP output comes through three different mechansims.  Either from:
 *
 *   1. TCP socket output.  For the case of TCP output to an
 *      IEEE802.15.4, the TCP output is caught in the socket
 *      send()/sendto() logic and and redirected to psock_6lowpan_tcp_send().
 *   2. TCP output from the TCP state machine.  That will occur
 *      during TCP packet processing by the TCP state meachine.
 *   3. TCP output resulting from TX or timer polling
 *
 *   Cases 2 and 3 will be handled here.  Logic in ipv6_tcp_input(),
 *   and devif_poll() detect if (1) an attempt to return with
 *   d_len > 0 and (2) that the device is an IEEE802.15.4 MAC network
 *   driver. Under those conditions, this function will be called to create
 *   the IEEE80215.4 frames.
 *
 * Input Parameters:
 *   dev    - The network device containing the packet to be sent.
 *   fwddev - The network device used to send the data.  This will be the
 *            same device except for the IP forwarding case where packets
 *            are sent across devices.
 *   ipv6   - A pointer to the IPv6 header in dev->d_buf which lies AFTER
 *            the L1 header.  NOTE: dev->d_len must have been decremented
 *            by the size of any preceding MAC header.
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called with the network locked.
 *
 ****************************************************************************/

void sixlowpan_tcp_send(FAR struct net_driver_s *dev,
                        FAR struct net_driver_s *fwddev,
                        FAR struct ipv6_hdr_s *ipv6)
{
  DEBUGASSERT(dev != NULL && dev->d_len > 0 && fwddev != NULL);

  /* Double check */

  ninfo("d_len %u\n", dev->d_len);
  sixlowpan_dumpbuffer("Outgoing TCP packet",
                       (FAR const uint8_t *)ipv6, dev->d_len);

  if (dev != NULL && dev->d_len > 0 && fwddev != NULL)
    {
      FAR struct ipv6tcp_hdr_s *ipv6hdr;

      /* The IPv6 header followed by a TCP headers should lie at the
       * beginning of d_buf since there is no link layer protocol header
       * and the TCP state machine should only response with TCP packets.
       */

      ipv6hdr = (FAR struct ipv6tcp_hdr_s *)ipv6;

      /* The TCP data payload should follow the IPv6 header plus the
       * protocol header.
       */

      if (ipv6hdr->ipv6.proto != IP_PROTO_TCP)
        {
          nwarn("WARNING: Expected TCP prototype: %u vs %u\n",
                ipv6hdr->ipv6.proto, IP_PROTO_TCP);
        }
      else
        {
          struct netdev_varaddr_s destmac;
          FAR uint8_t *buf;
          uint16_t hdrlen;
          uint16_t buflen;
          int ret;

          /* Get the IEEE 802.15.4 MAC address of the next hop. */

          ret = sixlowpan_nexthopaddr((FAR struct radio_driver_s *)fwddev,
                                      ipv6hdr->ipv6.destipaddr, &destmac);
          if (ret < 0)
            {
              nerr("ERROR: Failed to get dest MAC address: %d\n", ret);
              goto drop;
            }

          /* Get the IPv6 + TCP combined header length.  The size of the TCP
           * header is encoded in the top 4 bits of the tcpoffset field (in
           * units of 32-bit words).
           */

          hdrlen = IPv6_HDRLEN +
                   (((uint16_t)ipv6hdr->tcp.tcpoffset >> 4) << 2);

          /* Drop the packet if the buffer length is less than this. */

          if (hdrlen > dev->d_len)
            {
              nwarn("WARNING:  Dropping small TCP packet: %u < %u\n",
                    buflen, hdrlen);
            }
          else
            {
              /* Convert the outgoing packet into a frame list. */

              buf    = (FAR uint8_t *)ipv6 + hdrlen;
              buflen = dev->d_len - hdrlen;

              sixlowpan_queue_frames(
                      (FAR struct radio_driver_s *)fwddev,
                      &ipv6hdr->ipv6, buf, buflen, &destmac);
            }
        }
    }

drop:
  dev->d_len = 0;
}

#endif /* CONFIG_NET_6LOWPAN && CONFIG_NET_TCP */
