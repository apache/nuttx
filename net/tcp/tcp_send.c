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

#include <stdint.h>
#include <string.h>
#include <debug.h>

#include <nuttx/net/netconfig.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/netstats.h>
#include <nuttx/net/ip.h>
#include <nuttx/net/tcp.h>

#include "devif/devif.h"
#include "inet/inet.h"
#include "tcp/tcp.h"
#include "utils/utils.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define IPv4BUF    ((struct ipv4_hdr_s *)&dev->d_buf[NET_LL_HDRLEN(dev)])
#define IPv6BUF    ((struct ipv6_hdr_s *)&dev->d_buf[NET_LL_HDRLEN(dev)])

#define TCPIPv4BUF ((struct tcp_hdr_s *)&dev->d_buf[NET_LL_HDRLEN(dev) + IPv4_HDRLEN])
#define TCPIPv6BUF ((struct tcp_hdr_s *)&dev->d_buf[NET_LL_HDRLEN(dev) + IPv6_HDRLEN])

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
 * Name: tcp_sendcomplete, tcp_ipv4_sendcomplete, and tcp_ipv6_sendcomplete
 *
 * Description:
 *   Complete the final portions of the send operation.  This function sets
 *   up IP header and computes the TCP checksum
 *
 * Input Parameters:
 *   dev - The device driver structure to use in the send operation
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called with the network locked.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv4
static inline void tcp_ipv4_sendcomplete(FAR struct net_driver_s *dev,
                                         FAR struct tcp_hdr_s *tcp,
                                         FAR struct ipv4_hdr_s *ipv4)
{
  /* Set up some IP header fields that are needed for TCP checksum
   * calculation.
   */

  ipv4->proto       = IP_PROTO_TCP;
  ipv4->ttl         = IP_TTL;
  ipv4->vhl         = 0x45;

  /* At this point the TCP header holds the size of the payload, the
   * TCP header, and the IP header.
   */

  ipv4->len[0]      = (dev->d_len >> 8);
  ipv4->len[1]      = (dev->d_len & 0xff);

  /* Calculate TCP checksum. */

  tcp->urgp[0]      = 0;
  tcp->urgp[1]      = 0;

  tcp->tcpchksum    = 0;
  tcp->tcpchksum    = ~tcp_ipv4_chksum(dev);

  /* Finish initializing the IP header and calculate the IP checksum */

  ipv4->vhl         = 0x45;
  ipv4->tos         = 0;
  ipv4->ipoffset[0] = 0;
  ipv4->ipoffset[1] = 0;
  ++g_ipid;
  ipv4->ipid[0]     = g_ipid >> 8;
  ipv4->ipid[1]     = g_ipid & 0xff;

  /* Calculate IP checksum. */

  ipv4->ipchksum    = 0;
  ipv4->ipchksum    = ~ipv4_chksum(dev);

  ninfo("IPv4 length: %d\n", ((int)ipv4->len[0] << 8) + ipv4->len[1]);

#ifdef CONFIG_NET_STATISTICS
  g_netstats.ipv4.sent++;
#endif
}
#endif /* CONFIG_NET_IPv4 */

/****************************************************************************
 * Name: tcp_ipv6_sendcomplete
 *
 * Description:
 *   Complete the final portions of the send operation.  This function sets
 *   up IP header and computes the TCP checksum
 *
 * Input Parameters:
 *   dev - The device driver structure to use in the send operation
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called with the network locked.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv6
static inline void tcp_ipv6_sendcomplete(FAR struct net_driver_s *dev,
                                         FAR struct tcp_hdr_s *tcp,
                                         FAR struct ipv6_hdr_s *ipv6)
{
  uint16_t iplen;

  /* Set up some IP header fields that are needed for TCP checksum
   * calculation.
   */

  ipv6->proto     = IP_PROTO_TCP;
  ipv6->ttl       = IP_TTL;

  /* At this point the TCP header holds the size of the payload, the
   * TCP header, and the IP header.  For IPv6, the IP length field does
   * not include the size of IPv6 IP header length.
   */

  iplen           = dev->d_len - IPv6_HDRLEN;
  ipv6->len[0]    = (iplen >> 8);
  ipv6->len[1]    = (iplen & 0xff);

  /* Calculate TCP checksum. */

  tcp->urgp[0]     = 0;
  tcp->urgp[1]     = 0;

  tcp->tcpchksum   = 0;
  tcp->tcpchksum   = ~tcp_ipv6_chksum(dev);

  /* Finish initializing the IP header (no IPv6 checksum) */

  ipv6->vtc    = 0x60;
  ipv6->tcf    = 0x00;
  ipv6->flow   = 0x00;

  ninfo("IPv6 length: %d\n", ((int)ipv6->len[0] << 8) + ipv6->len[1]);

#ifdef CONFIG_NET_STATISTICS
  g_netstats.ipv6.sent++;
#endif
}
#endif /* CONFIG_NET_IPv6 */

/****************************************************************************
 * Name: tcp_sendcomplete
 *
 * Description:
 *   Complete the final portions of the send operation.  This function sets
 *   up IP header and computes the TCP checksum
 *
 * Input Parameters:
 *   dev - The device driver structure to use in the send operation
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called with the network locked.
 *
 ****************************************************************************/

static void tcp_sendcomplete(FAR struct net_driver_s *dev,
                             FAR struct tcp_hdr_s *tcp)
{
#ifdef CONFIG_NET_IPv6
#ifdef CONFIG_NET_IPv4
  if (IFF_IS_IPv6(dev->d_flags))
#endif
    {
      tcp_ipv6_sendcomplete(dev, tcp, IPv6BUF);
    }
#endif /* CONFIG_NET_IPv6 */

#ifdef CONFIG_NET_IPv4
#ifdef CONFIG_NET_IPv6
  else
#endif
    {
      tcp_ipv4_sendcomplete(dev, tcp, IPv4BUF);
    }
#endif /* CONFIG_NET_IPv4 */

  ninfo("Outgoing TCP packet length: %d bytes\n", dev->d_len);

#ifdef CONFIG_NET_STATISTICS
  g_netstats.tcp.sent++;
#endif
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
  /* Copy the IP address into the IPv6 header */

#ifdef CONFIG_NET_IPv6
#ifdef CONFIG_NET_IPv4
  if (IFF_IS_IPv6(dev->d_flags))
#endif
    {
      FAR struct ipv6_hdr_s *ipv6 = IPv6BUF;

      net_ipv6addr_hdrcopy(ipv6->srcipaddr, dev->d_ipv6addr);
      net_ipv6addr_hdrcopy(ipv6->destipaddr, conn->u.ipv6.raddr);
    }
#endif /* CONFIG_NET_IPv6 */

#ifdef CONFIG_NET_IPv4
#ifdef CONFIG_NET_IPv6
  else
#endif
    {
      FAR struct ipv4_hdr_s *ipv4 = IPv4BUF;

      net_ipv4addr_hdrcopy(ipv4->srcipaddr, &dev->d_ipaddr);
      net_ipv4addr_hdrcopy(ipv4->destipaddr, &conn->u.ipv4.raddr);
    }
#endif /* CONFIG_NET_IPv4 */

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

      uint16_t recvwndo = tcp_get_recvwindow(dev, conn);

      /* Set the TCP Window */

      tcp->wnd[0] = recvwndo >> 8;
      tcp->wnd[1] = recvwndo & 0xff;

      /* Update the Receiver Window */

      conn->rcv_wnd = recvwndo;
    }

  /* Finish the IP portion of the message and calculate checksums */

  tcp_sendcomplete(dev, tcp);
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
  FAR struct tcp_hdr_s *tcp = tcp_header(dev);

  tcp->flags     = flags;
  dev->d_len     = len;
  tcp->tcpoffset = (TCP_HDRLEN / 4) << 4;
  tcp_sendcommon(dev, conn, tcp);
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

void tcp_reset(FAR struct net_driver_s *dev)
{
  FAR struct tcp_hdr_s *tcp = tcp_header(dev);
  uint32_t ackno;
  uint16_t tmp16;
  uint16_t acklen = 0;
  uint8_t seqbyte;

#ifdef CONFIG_NET_STATISTICS
  g_netstats.tcp.rst++;
#endif

  /* TCP setup */

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
    }
#endif /* CONFIG_NET_IPv6 */

#ifdef CONFIG_NET_IPv4
#ifdef CONFIG_NET_IPv6
  else
#endif
    {
      FAR struct ipv4_hdr_s *ip = IPv4BUF;
      acklen += (ip->len[0] << 8) + ip->len[1] - (ip->vhl & 0x0f) * 4;
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

  /* Set the packet length and swap IP addresses. */

#ifdef CONFIG_NET_IPv6
#ifdef CONFIG_NET_IPv4
  if (IFF_IS_IPv6(dev->d_flags))
#endif
    {
      FAR struct ipv6_hdr_s *ipv6 = IPv6BUF;

      /* Set the packet length to the size of the IPv6 + TCP headers */

      dev->d_len = IPv6TCP_HDRLEN;

      /* Swap IPv6 addresses */

      net_ipv6addr_hdrcopy(ipv6->destipaddr, ipv6->srcipaddr);
      net_ipv6addr_hdrcopy(ipv6->srcipaddr, dev->d_ipv6addr);
    }
#endif /* CONFIG_NET_IPv6 */

#ifdef CONFIG_NET_IPv4
#ifdef CONFIG_NET_IPv6
  else
#endif
    {
      FAR struct ipv4_hdr_s *ipv4 = IPv4BUF;

      /* Set the packet length to the size of the IPv4 + TCP headers */

      dev->d_len = IPv4TCP_HDRLEN;

      /* Swap IPv4 addresses */

      net_ipv4addr_hdrcopy(ipv4->destipaddr, ipv4->srcipaddr);
      net_ipv4addr_hdrcopy(ipv4->srcipaddr, &dev->d_ipaddr);
    }
#endif /* CONFIG_NET_IPv4 */

  /* And send out the RST packet */

  tcp_sendcomplete(dev, tcp);
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
  struct tcp_hdr_s *tcp;
  uint16_t tcp_mss;

  /* Get values that vary with the underlying IP domain */

#ifdef CONFIG_NET_IPv6
#ifdef CONFIG_NET_IPv4
  if (IFF_IS_IPv6(dev->d_flags))
#endif
    {
      /* Get the MSS value and offset TCP header address for this packet */

      tcp     = TCPIPv6BUF;
      tcp_mss = TCP_IPv6_MSS(dev);

      /* Set the packet length for the TCP Maximum Segment Size */

      dev->d_len  = IPv6TCP_HDRLEN + TCP_OPT_MSS_LEN;
    }
#endif /* CONFIG_NET_IPv6 */

#ifdef CONFIG_NET_IPv4
#ifdef CONFIG_NET_IPv6
  else
#endif
    {
      /* Get the MSS value and offset TCP header address for this packet */

      tcp     = TCPIPv4BUF;
      tcp_mss = TCP_IPv4_MSS(dev);

      /* Set the packet length for the TCP Maximum Segment Size */

      dev->d_len  = IPv4TCP_HDRLEN + TCP_OPT_MSS_LEN;
    }
#endif /* CONFIG_NET_IPv4 */

  /* Save the ACK bits */

  tcp->flags      = ack;

  /* We send out the TCP Maximum Segment Size option with our ACK. */

  tcp->optdata[0] = TCP_OPT_MSS;
  tcp->optdata[1] = TCP_OPT_MSS_LEN;
  tcp->optdata[2] = tcp_mss >> 8;
  tcp->optdata[3] = tcp_mss & 0xff;
  tcp->tcpoffset  = ((TCP_HDRLEN + TCP_OPT_MSS_LEN) / 4) << 4;

  /* Complete the common portions of the TCP message */

  tcp_sendcommon(dev, conn, tcp);
}

#endif /* CONFIG_NET && CONFIG_NET_TCP */
