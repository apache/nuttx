/****************************************************************************
 * net/sixlowpan/sixlowpan_tcpsend.c
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
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

#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include "nuttx/net/netdev.h"
#include "nuttx/net/netstats.h"

#include "netdev/netdev.h"
#include "socket/socket.h"
#include "tcp/tcp.h"
#include "utils/utils.h"
#include "sixlowpan/sixlowpan_internal.h"

#if defined(CONFIG_NET_6LOWPAN) && defined(CONFIG_NET_TCP)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Buffer access helpers */

#define IPv6BUF(dev)  ((FAR struct ipv6_hdr_s *)((dev)->d_buf))

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
 *   ipv6tcp - A reference to a structure containing the IPv6 and TCP headers.
 *   buf     - The beginning of the payload data
 *   buflen  - The length of the payload data.
 *
 * Returned Value:
 *   The calculated checksum
 *
 ****************************************************************************/

static uint16_t sixlowpan_tcp_chksum(FAR struct ipv6tcp_hdr_s *ipv6tcp,
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

  if (upperlen > CONFIG_NET_6LOWPAN_MTU)
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
  return (sum == 0) ? 0xffff : htons(sum);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: psock_6lowpan_tcp_send
 *
 * Description:
 *   psock_6lowpan_tcp_send() call may be used only when the TCP socket is in a
 *   connected state (so that the intended recipient is known).
 *
 * Parameters:
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
  struct ipv6tcp_hdr_s ipv6tcp;
  struct sixlowpan_tagaddr_s destmac;
  uint16_t timeout;
  uint16_t iplen;
  int ret;

  ninfo("buflen %lu\n", (unsigned long)buflen);
  sixlowpan_dumpbuffer("Outgoing TCP payload", buf, buflen);

  DEBUGASSERT(psock != NULL && psock->s_crefs > 0);
  DEBUGASSERT(psock->s_type == SOCK_STREAM);

  /* Make sure that this is a valid socket */

  if (psock != NULL || psock->s_crefs <= 0)
    {
      nerr("ERROR: Invalid socket\n");
      return (ssize_t)-EBADF;
    }

  /* Make sure that this is a connected TCP socket */

  if (psock->s_type != SOCK_STREAM || !_SS_ISCONNECTED(psock->s_flags))
    {
      nerr("ERROR: Not connected\n");
      return (ssize_t)-ENOTCONN;
    }

  /* Get the underlying TCP connection structure */

  conn = (FAR struct tcp_conn_s *)psock->s_conn;
  DEBUGASSERT(conn != NULL);

#ifdef CONFIG_NET_IPv4
  /* Ignore if not IPv6 domain */

  if (conn->domain != PF_INET6)
    {
      nwarn("WARNING: Not IPv6\n");
      return (ssize_t)-EPROTOTYPE;
    }
#endif

  /* Route outgoing message to the correct device */

#ifdef CONFIG_NETDEV_MULTINIC
  dev = netdev_findby_ipv6addr(conn->u.ipv6.laddr, conn->u.ipv6.raddr);
#ifdef CONFIG_NETDEV_MULTILINK
  if (dev == NULL || dev->d_lltype != NET_LL_IEEE802154)
#else
  if (dev == NULL)
#endif
    {
      nwarn("WARNING: Not routable or not IEEE802.15.4 MAC\n");
      return (ssize_t)-ENETUNREACH;
    }
#else
  dev = netdev_findby_ipv6addr(conn->u.ipv6.raddr);
#ifdef CONFIG_NETDEV_MULTILINK
  if (dev == NULL || dev->d_lltype != NET_LL_IEEE802154)
#else
  if (dev == NULL)
#endif
    {
      nwarn("WARNING: Not routable\n");
      return (ssize_t)-ENETUNREACH;
    }
#endif

#ifdef CONFIG_NET_ICMPv6_NEIGHBOR
  /* Make sure that the IP address mapping is in the Neighbor Table */

  ret = icmpv6_neighbor(conn->u.ipv6.raddr);
  if (ret < 0)
    {
      nerr("ERROR: Not reachable\n");
      return (ssize_t)-ENETUNREACH;
    }
#endif

  /* Initialize the IPv6/TCP headers */

  /* Initialize the IPv6/UDP headers */

  ipv6tcp.ipv6.vtc    = 0x60;
  ipv6tcp.ipv6.tcf    = 0x00;
  ipv6tcp.ipv6.flow   = 0x00;
  ipv6tcp.ipv6.proto  = IP_PROTO_TCP;
  ipv6tcp.ipv6.ttl    = IP_TTL;

  /* The IPv6 header length field does not include the size of IPv6 IP
   * header.
   */

  iplen               = buflen + TCP_HDRLEN;
  ipv6tcp.ipv6.len[0] = (iplen >> 8);
  ipv6tcp.ipv6.len[1] = (iplen & 0xff);

  /* Copy the source and destination addresses */

  net_ipv6addr_hdrcopy(ipv6tcp.ipv6.srcipaddr,  conn->u.ipv6.laddr);
  net_ipv6addr_hdrcopy(ipv6tcp.ipv6.destipaddr, conn->u.ipv6.raddr);

  ninfo("IPv6 length: %d\n",
        ((int)ipv6tcp.ipv6.len[0] << 8) + ipv6tcp.ipv6.len[1]);

#ifdef CONFIG_NET_STATISTICS
  g_netstats.ipv6.sent++;
#endif

  /* Initialize the TCP header */

  ipv6tcp.tcp.srcport   = conn->lport;           /* Local port */
  ipv6tcp.tcp.destport  = conn->rport;           /* Connected remote port */

  memcpy(ipv6tcp.tcp.ackno, conn->rcvseq, 4);    /* ACK number */
  memcpy(ipv6tcp.tcp.seqno, conn->sndseq, 4);    /* Sequence number */

  ipv6tcp.tcp.tcpoffset = (TCP_HDRLEN / 4) << 4; /* No optdata */
  ipv6tcp.tcp.urgp[0]   = 0;                     /* No urgent data */
  ipv6tcp.tcp.urgp[1]   = 0;

    /* Set the TCP window */

  if (conn->tcpstateflags & TCP_STOPPED)
    {
      /* If the connection has issued TCP_STOPPED, we advertise a zero
       * window so that the remote host will stop sending data.
       */

      ipv6tcp.tcp.wnd[0] = 0;
      ipv6tcp.tcp.wnd[1] = 0;
    }
  else
    {
      ipv6tcp.tcp.wnd[0] = ((NET_DEV_RCVWNDO(dev)) >> 8);
      ipv6tcp.tcp.wnd[1] = ((NET_DEV_RCVWNDO(dev)) & 0xff);
    }

  /* Calculate TCP checksum. */

  ipv6tcp.tcp.tcpchksum   = 0;
  ipv6tcp.tcp.tcpchksum   = ~sixlowpan_tcp_chksum(&ipv6tcp, buf, buflen);

  ninfo("Outgoing TCP packet length: %d bytes\n", iplen + IPv6_HDRLEN);

#ifdef CONFIG_NET_STATISTICS
  g_netstats.tcp.sent++;
#endif

  /* Set the socket state to sending */

  psock->s_flags = _SS_SETSTATE(psock->s_flags, _SF_SEND);

  /* Get the IEEE 802.15.4 MAC address of the destination.  This assumes
   * an encoding of the MAC address in the IPv6 address.
   */

  sixlowpan_addrfromip(conn->u.ipv6.raddr, &destmac);

  /* If routable, then call sixlowpan_send() to format and send the 6loWPAN
   * packet.
   */

#ifdef CONFIG_NET_SOCKOPTS
  timeout = psock->s_sndtimeo;
#else
  timeout = 0;
#endif

  ret = sixlowpan_send(dev, &conn->list,
                       (FAR const struct ipv6_hdr_s *)&ipv6tcp,
                       buf, buflen, &destmac, timeout);
  if (ret < 0)
    {
      nerr("ERROR: sixlowpan_send() failed: %d\n", ret);
    }

  /* Set the socket state to idle */

  psock->s_flags = _SS_SETSTATE(psock->s_flags, _SF_IDLE);
  return ret;
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
 *   devif_poll(), and devif_timer() detect if (1) an attempt to return with
 *   d_len > 0 and (2) that the device is an IEEE802.15.4 MAC network
 *   driver. Under those conditions, this function will be called to create
 *   the IEEE80215.4 frames.
 *
 * Parameters:
 *   dev - An instance of nework device state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called with the network locked.
 *
 ****************************************************************************/

void sixlowpan_tcp_send(FAR struct net_driver_s *dev)
{
  DEBUGASSERT(dev != NULL && dev->d_len > 0);

  /* Double check */

  ninfo("d_len %u\n", dev->d_len);
  sixlowpan_dumpbuffer("Outgoing TCP packet",
                       (FAR const uint8_t *)IPv6BUF(dev), dev->d_len);

  if (dev != NULL && dev->d_len > 0)
    {
      FAR struct ipv6tcp_hdr_s *ipv6hdr;

      /* The IPv6 header followed by a TCP headers should lie at the
       * beginning of d_buf since there is no link layer protocol header
       * and the TCP state machine should only response with TCP packets.
       */

      ipv6hdr = (FAR struct ipv6tcp_hdr_s *)(dev->d_buf);

      /* The TCP data payload should follow the IPv6 header plus the
       * protocol header.
       */

      if (ipv6hdr->ipv6.proto != IP_PROTO_TCP)
        {
          nwarn("WARNING: Expected TCP protoype: %u vs %s\n",
                ipv6hdr->ipv6.proto, IP_PROTO_TCP);
        }
      else
        {
          struct sixlowpan_tagaddr_s destmac;
          FAR uint8_t *buf;
          uint16_t hdrlen;
          uint16_t buflen;

          /* Get the IEEE 802.15.4 MAC address of the destination.  This
           * assumes an encoding of the MAC address in the IPv6 address.
           */

          sixlowpan_addrfromip(ipv6hdr->ipv6.destipaddr, &destmac);

          /* Get the IPv6 + TCP combined header length.  The size of the TCP
           * header is encoded in the top 4 bits of the tcpoffset field (in
           * units of 32-bit words).
           */

          hdrlen = IPv6_HDRLEN + (((uint16_t)ipv6hdr->tcp.tcpoffset >> 4) << 2);

          /* Drop the packet if the buffer length is less than this. */

          if (hdrlen > dev->d_len)
            {
              nwarn("WARNING:  Dropping small TCP packet: %u < %u\n",
                    buflen, hdrlen);
            }
          else
            {
              /* Convert the outgoing packet into a frame list. */

              buf    = dev->d_buf + hdrlen;
              buflen = dev->d_len - hdrlen;

              (void)sixlowpan_queue_frames(
                      (FAR struct ieee802154_driver_s *)dev, &ipv6hdr->ipv6,
                      buf, buflen, &destmac);
            }
        }
    }

  dev->d_len = 0;
}

#endif /* CONFIG_NET_6LOWPAN && CONFIG_NET_TCP */
