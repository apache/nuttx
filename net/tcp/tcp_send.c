/****************************************************************************
 * net/tcp/tcp_send.c
 *
 *   Copyright (C) 2007-2010, 2012, 2015, 2018-2019 Gregory Nutt. All rights
 *     reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Adapted for NuttX from logic in uIP which also has a BSD-like license:
 *
 *   Original author Adam Dunkels <adam@dunkels.com>
 *   Copyright () 2001-2003, Adam Dunkels.
 *   All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote
 *    products derived from this software without specific prior
 *    written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#if defined(CONFIG_NET) && defined(CONFIG_NET_TCP)

#include <assert.h>
#include <stdint.h>
#include <string.h>
#include <debug.h>

#include <nuttx/net/netconfig.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/netstats.h>
#include <nuttx/net/ip.h>
#include <nuttx/net/tcp.h>

#include "netdev/netdev.h"
#include "devif/devif.h"
#include "inet/inet.h"
#include "tcp/tcp.h"
#include "utils/utils.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tcp_header
 *
 * Description:
 *   Get the length of the IP header
 *
 * Input Parameters:
 *   dev - The device driver structure to use in the send operation
 *
 * Returned Value:
 *   The length of the IP header (IPv4_HDRLEN or IPv6_HDRLEN)
 *
 ****************************************************************************/

static inline FAR struct tcp_hdr_s *tcp_header(FAR struct net_driver_s *dev)
{
#ifdef CONFIG_NET_IPv6
#ifdef CONFIG_NET_IPv4
  if (IFF_IS_IPv6(dev->d_flags))
#endif
    {
      return TCPIPv6BUF;
    }
#endif /* CONFIG_NET_IPv6 */

#ifdef CONFIG_NET_IPv4
#ifdef CONFIG_NET_IPv6
  else
#endif
    {
      return TCPIPv4BUF;
    }
#endif /* CONFIG_NET_IPv4 */
}

/****************************************************************************
 * Name: tcp_sendcommon
 *
 * Description:
 *   We're done with the input processing. We are now ready to send a reply
 *   Our job is to fill in all the fields of the TCP and IP headers before
 *   calculating the checksum and finally send the packet.
 *
 * Input Parameters:
 *   dev  - The device driver structure to use in the send operation
 *   conn - The TCP connection structure holding connection information
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called with the network locked.
 *
 ****************************************************************************/

static void tcp_sendcommon(FAR struct net_driver_s *dev,
                           FAR struct tcp_conn_s *conn,
                           FAR struct tcp_hdr_s *tcp)
{
  /* Set TCP sequence numbers and port numbers */

  memcpy(tcp->ackno, conn->rcvseq, 4);
  memcpy(tcp->seqno, conn->sndseq, 4);

  tcp->srcport  = conn->lport;
  tcp->destport = conn->rport;

  /* Set the TCP window */

  if (conn->tcpstateflags & TCP_STOPPED)
    {
      /* If the connection has issued TCP_STOPPED, we advertise a zero
       * window so that the remote host will stop sending data.
       */

      tcp->wnd[0] = 0;
      tcp->wnd[1] = 0;
    }
  else
    {
      /* Update the TCP received window based on I/O buffer availability */

      uint32_t rcvseq = tcp_getsequence(conn->rcvseq);
      uint32_t recvwndo = tcp_get_recvwindow(dev, conn);

      /* Update the Receiver Window */

      conn->rcv_adv = rcvseq + recvwndo;

#ifdef CONFIG_NET_TCP_WINDOW_SCALE
      recvwndo >>= conn->rcv_scale;
#endif

      /* Set the TCP Window */

      tcp->wnd[0] = recvwndo >> 8;
      tcp->wnd[1] = recvwndo & 0xff;
    }

  tcp->urgp[0] = 0;
  tcp->urgp[1] = 0;

  /* Update device buffer length before setup the IP header */

  iob_update_pktlen(dev->d_iob, dev->d_len);

  /* Calculate chk & build L3 header */

#ifdef CONFIG_NET_IPv6
#ifdef CONFIG_NET_IPv4
  if (IFF_IS_IPv6(dev->d_flags))
#endif
    {
      ninfo("do IPv6 IP header build!\n");
      ipv6_build_header(IPv6BUF, dev->d_len - IPv6_HDRLEN,
                        IP_PROTO_TCP, dev->d_ipv6addr, conn->u.ipv6.raddr,
                        conn->sconn.ttl, conn->sconn.s_tclass);

      /* Calculate TCP checksum. */

      tcp->tcpchksum = 0;
      tcp->tcpchksum = ~tcp_ipv6_chksum(dev);
#ifdef CONFIG_NET_STATISTICS
      g_netstats.ipv6.sent++;
#endif
    }
#endif /* CONFIG_NET_IPv6 */

#ifdef CONFIG_NET_IPv4
#ifdef CONFIG_NET_IPv6
  else
#endif
    {
      ninfo("do IPv4 IP header build!\n");
      ipv4_build_header(IPv4BUF, dev->d_len, IP_PROTO_TCP,
                        &dev->d_ipaddr, &conn->u.ipv4.raddr,
                        conn->sconn.ttl, conn->sconn.s_tos, NULL);

      /* Calculate TCP checksum. */

      tcp->tcpchksum = 0;
      tcp->tcpchksum = ~tcp_ipv4_chksum(dev);
#ifdef CONFIG_NET_STATISTICS
      g_netstats.ipv4.sent++;
#endif
    }
#endif /* CONFIG_NET_IPv4 */

  ninfo("Outgoing TCP packet length: %d bytes\n", dev->d_len);
#ifdef CONFIG_NET_STATISTICS
  g_netstats.tcp.sent++;
#endif

#if !defined(CONFIG_NET_TCP_WRITE_BUFFERS)
  if ((tcp->flags & (TCP_SYN | TCP_FIN)) != 0)
    {
      /* Remember sndseq that will be used in case of a possible
       * SYN or FIN retransmission
       */

      conn->rexmit_seq = tcp_getsequence(conn->sndseq);

      /* Advance sndseq by +1 because SYN and FIN occupy
       * one sequence number (RFC 793)
       */

      net_incr32(conn->sndseq, 1);
    }
#else
  /* REVISIT for the buffered mode */
#endif
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tcp_send
 *
 * Description:
 *   Setup to send a TCP packet
 *
 * Input Parameters:
 *   dev    - The device driver structure to use in the send operation
 *   conn   - The TCP connection structure holding connection information
 *   flags  - flags to apply to the TCP header
 *   len    - length of the message (includes the length of the IP and TCP
 *            headers)
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called with the network locked.
 *
 ****************************************************************************/

void tcp_send(FAR struct net_driver_s *dev, FAR struct tcp_conn_s *conn,
              uint16_t flags, uint16_t len)
{
  FAR struct tcp_hdr_s *tcp;

  if (dev->d_iob == NULL)
    {
      return;
    }

  tcp        = tcp_header(dev);
  tcp->flags = flags;
  dev->d_len = len;

#ifdef CONFIG_NET_TCP_SELECTIVE_ACK
  if ((conn->flags & TCP_SACK) && (flags == TCP_ACK) && conn->nofosegs > 0)
    {
      int optlen = conn->nofosegs * sizeof(struct tcp_sack_s);
      int i;

      tcp->optdata[0] = TCP_OPT_NOOP;
      tcp->optdata[1] = TCP_OPT_NOOP;
      tcp->optdata[2] = TCP_OPT_SACK;
      tcp->optdata[3] = TCP_OPT_SACK_PERM_LEN + optlen;

      optlen += 4;

      for (i = 0; i < conn->nofosegs; i++)
        {
          ninfo("TCP SACK [%d]"
                "[%" PRIu32 " : %" PRIu32 " : %" PRIu32 "]\n", i,
                conn->ofosegs[i].left, conn->ofosegs[i].right,
                TCP_SEQ_SUB(conn->ofosegs[i].right, conn->ofosegs[i].left));
          tcp_setsequence(&tcp->optdata[4 + i * 2 * sizeof(uint32_t)],
                          conn->ofosegs[i].left);
          tcp_setsequence(&tcp->optdata[4 + (i * 2 + 1) * sizeof(uint32_t)],
                          conn->ofosegs[i].right);
        }

      dev->d_len += optlen;
      tcp->tcpoffset = ((TCP_HDRLEN + optlen) / 4) << 4;
    }
  else
#endif /* CONFIG_NET_TCP_SELECTIVE_ACK */
    {
      tcp->tcpoffset = (TCP_HDRLEN / 4) << 4;
    }

  tcp_sendcommon(dev, conn, tcp);

#if defined(CONFIG_NET_STATISTICS) && \
    defined(CONFIG_NET_TCP_DEBUG_DROP_SEND)

#pragma message \
  "CONFIG_NET_TCP_DEBUG_DROP_SEND is selected, this is debug " \
  "feature to drop the tcp send packet on the floor, " \
  "please confirm the configuration again if you do not want " \
  "debug the TCP stack."

  /* Debug feature to drop the tcp received packet on the floor */

  if ((flags & TCP_PSH) != 0)
    {
      if ((g_netstats.tcp.sent %
          CONFIG_NET_TCP_DEBUG_DROP_SEND_PROBABILITY) == 0)
        {
          uint32_t seq = tcp_getsequence(tcp->seqno);

          ninfo("TCP DROP SNDPKT: "
                "[%d][%" PRIu32 " : %" PRIu32 " : %d]\n",
                g_netstats.tcp.sent, seq, TCP_SEQ_ADD(seq, dev->d_sndlen),
                dev->d_sndlen);

          dev->d_len = 0;
        }
    }
#endif
}

/****************************************************************************
 * Name: tcp_reset
 *
 * Description:
 *   Send a TCP reset (no-data) message
 *
 * Input Parameters:
 *   dev    - The device driver structure to use in the send operation
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called with the network locked.
 *
 ****************************************************************************/

void tcp_reset(FAR struct net_driver_s *dev, FAR struct tcp_conn_s *conn)
{
  FAR struct tcp_hdr_s *tcp;
  uint32_t ackno;
  uint16_t tmp16;
  uint16_t acklen = 0;
  uint8_t seqbyte;

  if (dev->d_iob == NULL)
    {
      return;
    }

#ifdef CONFIG_NET_STATISTICS
  g_netstats.tcp.rst++;
#endif

  /* TCP setup */

  tcp = tcp_header(dev);

  if ((tcp->flags & TCP_SYN) != 0 || (tcp->flags & TCP_FIN) != 0)
    {
      acklen++;
    }

#ifdef CONFIG_NET_IPv6
#ifdef CONFIG_NET_IPv4
  if (IFF_IS_IPv6(dev->d_flags))
#endif
    {
      FAR struct ipv6_hdr_s *ip = IPv6BUF;

      acklen += (ip->len[0] << 8 | ip->len[1]);

      /* Set the packet length to the size of the IPv6 + TCP headers */

      dev->d_len = IPv6TCP_HDRLEN;
    }
#endif /* CONFIG_NET_IPv6 */

#ifdef CONFIG_NET_IPv4
#ifdef CONFIG_NET_IPv6
  else
#endif
    {
      FAR struct ipv4_hdr_s *ip = IPv4BUF;

      acklen += (ip->len[0] << 8) + ip->len[1] - (ip->vhl & 0x0f) * 4;

      /* Set the packet length to the size of the IPv4 + TCP headers */

      dev->d_len = IPv4TCP_HDRLEN;
    }
#endif /* CONFIG_NET_IPv4 */

  acklen        -= (tcp->tcpoffset >> 4) << 2;

  tcp->flags     = TCP_RST | TCP_ACK;
  tcp->tcpoffset = 5 << 4;

  /* Flip the seqno and ackno fields in the TCP header. */

  seqbyte        = tcp->seqno[3];
  tcp->seqno[3]  = tcp->ackno[3];
  tcp->ackno[3]  = seqbyte;

  seqbyte        = tcp->seqno[2];
  tcp->seqno[2]  = tcp->ackno[2];
  tcp->ackno[2]  = seqbyte;

  seqbyte        = tcp->seqno[1];
  tcp->seqno[1]  = tcp->ackno[1];
  tcp->ackno[1]  = seqbyte;

  seqbyte        = tcp->seqno[0];
  tcp->seqno[0]  = tcp->ackno[0];
  tcp->ackno[0]  = seqbyte;

  /* We also have to increase the sequence number we are
   * acknowledging. If the least significant byte overflowed, we need
   * to propagate the carry to the other bytes as well.
   */

  ackno = tcp_addsequence(tcp->ackno, acklen);

  tcp_setsequence(tcp->ackno, ackno);

  /* Swap port numbers. */

  tmp16         = tcp->srcport;
  tcp->srcport  = tcp->destport;
  tcp->destport = tmp16;

  /* Initialize the rest of the tcp header to sane values.
   */

  tcp->wnd[0] = 0;
  tcp->wnd[1] = 0;
  tcp->urgp[0] = 0;
  tcp->urgp[0] = 0;

  /* Update device buffer length before setup the IP header */

  iob_update_pktlen(dev->d_iob, dev->d_len);

  /* Calculate chk & build L3 header */

#ifdef CONFIG_NET_IPv6
#ifdef CONFIG_NET_IPv4
  if (IFF_IS_IPv6(dev->d_flags))
#endif
    {
      FAR struct ipv6_hdr_s *ipv6 = IPv6BUF;

      ipv6_build_header(ipv6, dev->d_len - IPv6_HDRLEN,
                        IP_PROTO_TCP, dev->d_ipv6addr, ipv6->srcipaddr,
                        conn ? conn->sconn.ttl : IP_TTL_DEFAULT,
                        conn ? conn->sconn.s_tos : 0);
      tcp->tcpchksum = 0;
      tcp->tcpchksum = ~tcp_ipv6_chksum(dev);
    }
#endif /* CONFIG_NET_IPv6 */

#ifdef CONFIG_NET_IPv4
#ifdef CONFIG_NET_IPv6
  else
#endif
    {
      FAR struct ipv4_hdr_s *ipv4 = IPv4BUF;

      ipv4_build_header(IPv4BUF, dev->d_len, IP_PROTO_TCP,
                        &dev->d_ipaddr, (FAR in_addr_t *)ipv4->srcipaddr,
                        conn ? conn->sconn.ttl : IP_TTL_DEFAULT,
                        conn ? conn->sconn.s_tos : 0, NULL);

      tcp->tcpchksum = 0;
      tcp->tcpchksum = ~tcp_ipv4_chksum(dev);
    }
#endif /* CONFIG_NET_IPv4 */
}

/****************************************************************************
 * Name: tcp_rx_mss
 *
 * Description:
 *   Return the MSS to advertize to the peer.
 *
 * Input Parameters:
 *   dev  - The device driver structure
 *
 * Returned Value:
 *   The MSS value.
 *
 ****************************************************************************/

uint16_t tcp_rx_mss(FAR struct net_driver_s *dev)
{
  uint16_t tcp_mss;

#ifdef CONFIG_NET_IPv6
#ifdef CONFIG_NET_IPv4
  if (IFF_IS_IPv6(dev->d_flags))
#endif
    {
      tcp_mss = TCP_IPv6_MSS(dev);
    }
#endif /* CONFIG_NET_IPv6 */

#ifdef CONFIG_NET_IPv4
#ifdef CONFIG_NET_IPv6
  else
#endif
    {
      tcp_mss = TCP_IPv4_MSS(dev);
    }
#endif /* CONFIG_NET_IPv4 */

  return tcp_mss;
}

/****************************************************************************
 * Name: tcp_synack
 *
 * Description:
 *   Send the SYN, ACK, or SYNACK response.
 *
 *   - SYN and SYNACK are sent only from the TCP state machine.
 *   - ACK may be sent alone only if delayed ACKs are enabled and the ACK
 *     delay timeout occurs.
 *
 * Input Parameters:
 *   dev  - The device driver structure to use in the send operation
 *   conn - The TCP connection structure holding connection information
 *   ack  - The ACK response to send
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called with the network locked.
 *
 ****************************************************************************/

void tcp_synack(FAR struct net_driver_s *dev, FAR struct tcp_conn_s *conn,
                uint8_t ack)
{
  FAR struct tcp_hdr_s *tcp;
  uint16_t tcp_mss;
  int16_t optlen = 0;

  if (dev->d_iob == NULL)
    {
      return;
    }

  /* Get the offset TCP header address for this packet */

  tcp = tcp_header(dev);

  /* Set the packet length for the TCP Maximum Segment Size */

  dev->d_len = tcpip_hdrsize(conn);

  /* Set the packet length for the TCP Maximum Segment Size */

#ifdef CONFIG_NET_TCPPROTO_OPTIONS
  if (conn->user_mss != 0 && conn->user_mss < tcp_rx_mss(dev))
    {
      tcp_mss = conn->user_mss;
    }
  else
#endif
    {
      tcp_mss = tcp_rx_mss(dev);
    }

  /* Save the ACK bits */

  tcp->flags = ack;

  /* We send out the TCP Maximum Segment Size option with our ACK. */

  tcp->optdata[optlen++] = TCP_OPT_MSS;
  tcp->optdata[optlen++] = TCP_OPT_MSS_LEN;
  tcp->optdata[optlen++] = tcp_mss >> 8;
  tcp->optdata[optlen++] = tcp_mss & 0xff;

#ifdef CONFIG_NET_TCP_WINDOW_SCALE
  if (tcp->flags == TCP_SYN ||
      ((tcp->flags == (TCP_ACK | TCP_SYN)) && (conn->flags & TCP_WSCALE)))
    {
      tcp->optdata[optlen++] = TCP_OPT_NOOP;
      tcp->optdata[optlen++] = TCP_OPT_WS;
      tcp->optdata[optlen++] = TCP_OPT_WS_LEN;
      tcp->optdata[optlen++] = CONFIG_NET_TCP_WINDOW_SCALE_FACTOR;
    }
#endif

#ifdef CONFIG_NET_TCP_SELECTIVE_ACK
  if (tcp->flags == TCP_SYN ||
      ((tcp->flags == (TCP_ACK | TCP_SYN)) && (conn->flags & TCP_SACK)))
    {
      tcp->optdata[optlen++] = TCP_OPT_NOOP;
      tcp->optdata[optlen++] = TCP_OPT_NOOP;
      tcp->optdata[optlen++] = TCP_OPT_SACK_PERM;
      tcp->optdata[optlen++] = TCP_OPT_SACK_PERM_LEN;
    }
#endif

  tcp->tcpoffset         = ((TCP_HDRLEN + optlen) / 4) << 4;
  dev->d_len            += optlen;

  /* Complete the common portions of the TCP message */

  tcp_sendcommon(dev, conn, tcp);
}

/****************************************************************************
 * Name: tcp_send_txnotify
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

void tcp_send_txnotify(FAR struct socket *psock,
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
 * Name: tcpip_hdrsize
 *
 * Description:
 *   Get the total size of L3 and L4 TCP header
 *
 * Input Parameters:
 *   conn     The connection structure associated with the socket
 *
 * Returned Value:
 *   the total size of L3 and L4 TCP header
 *
 ****************************************************************************/

uint16_t tcpip_hdrsize(FAR struct tcp_conn_s *conn)
{
  uint16_t hdrsize = sizeof(struct tcp_hdr_s);

#if defined(CONFIG_NET_IPv4) && defined(CONFIG_NET_IPv6)
  if (conn->domain == PF_INET)
    {
      /* Select the IPv4 domain */

      return sizeof(struct ipv4_hdr_s) + hdrsize;
    }
  else /* if (domain == PF_INET6) */
    {
      /* Select the IPv6 domain */

      return sizeof(struct ipv6_hdr_s) + hdrsize;
    }
#elif defined(CONFIG_NET_IPv4)
  ((void)conn);
  return sizeof(struct ipv4_hdr_s) + hdrsize;
#elif defined(CONFIG_NET_IPv6)
  ((void)conn);
  return sizeof(struct ipv6_hdr_s) + hdrsize;
#endif
}

#endif /* CONFIG_NET && CONFIG_NET_TCP */
