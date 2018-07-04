/****************************************************************************
 * net/sixlowpan/sixlowpan_udpsend.c
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

#include <sys/socket.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include "nuttx/net/netdev.h"
#include "nuttx/net/radiodev.h"
#include "nuttx/net/netstats.h"

#include "netdev/netdev.h"
#include "socket/socket.h"
#include "inet/inet.h"
#include "icmpv6/icmpv6.h"
#include "udp/udp.h"
#include "utils/utils.h"
#include "sixlowpan/sixlowpan_internal.h"

#if defined(CONFIG_NET_6LOWPAN) && defined(CONFIG_NET_UDP)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sixlowpan_udp_chksum
 *
 * Description:
 *   Perform the checksum calcaultion over the IPv6, protocol headers, and
 *   data payload as necessary.
 *
 * Input Parameters:
 *   ipv6udp - A reference to a structure containing the IPv6 and UDP headers.
 *   buf     - The beginning of the payload data
 *   buflen  - The length of the payload data.
 *
 * Returned Value:
 *   The calculated checksum
 *
 ****************************************************************************/

#ifdef CONFIG_NET_UDP_CHECKSUMS
static uint16_t sixlowpan_udp_chksum(FAR const struct ipv6udp_hdr_s *ipv6udp,
                                     FAR const uint8_t *buf, uint16_t buflen)
{
  uint16_t upperlen;
  uint16_t sum;

  /* The length reported in the IPv6 header is the length of the payload
   * that follows the header.
   */

  upperlen = ((uint16_t)ipv6udp->ipv6.len[0] << 8) + ipv6udp->ipv6.len[1];

  /* Verify some minimal assumptions */

  if (upperlen > CONFIG_NET_6LOWPAN_PKTSIZE)
    {
      return 0;
    }

  /* The checksum is calculated starting with a pseudo-header of IPv6 header
   * fields according to the IPv6 standard, which consists of the source
   * and destination addresses, the packet length and the next header field.
   */

  sum = upperlen + ipv6udp->ipv6.proto;

  /* Sum IP source and destination addresses. */

  sum = chksum(sum, (FAR uint8_t *)ipv6udp->ipv6.srcipaddr,
               2 * sizeof(net_ipv6addr_t));

  /* Sum the UDP header */

  sum = chksum(sum, (FAR uint8_t *)&ipv6udp->udp, UDP_HDRLEN);

  /* Sum payload data. */

  sum = chksum(sum, buf, buflen);
  return (sum == 0) ? 0xffff : htons(sum);
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: psock_6lowpan_udp_sendto
 *
 * Description:
 *   If sendto() is used on a connection-mode (SOCK_STREAM, SOCK_SEQPACKET)
 *   socket, the parameters to and 'tolen' are ignored (and the error EISCONN
 *   may be returned when they are not NULL and 0), and the error ENOTCONN is
 *   returned when the socket was not actually connected.
 *
 * Input Parameters:
 *   psock    A pointer to a NuttX-specific, internal socket structure
 *   buf      Data to send
 *   buflen   Length of data to send
 *   flags    Send flags
 *   to       Address of recipient
 *   tolen    The length of the address structure
 *
 * Returned Value:
 *   On success, returns the number of characters sent.  On  error,
 *   -1 is returned, and errno is set appropriately.    Returned error
 *   number must be consistent with definition of errors reported by
 *   sendto().
 *
 * Assumptions:
 *   Called with the network locked.
 *
 ****************************************************************************/

ssize_t psock_6lowpan_udp_sendto(FAR struct socket *psock,
                                 FAR const void *buf,
                                 size_t buflen, int flags,
                                 FAR const struct sockaddr *to,
                                 socklen_t tolen)
{
  FAR struct sockaddr_in6 *to6 = (FAR struct sockaddr_in6 *)to;
  FAR struct udp_conn_s *conn;
  FAR struct net_driver_s *dev;
  struct ipv6udp_hdr_s ipv6udp;
  struct netdev_varaddr_s destmac;
  uint16_t iplen;
  uint16_t timeout;
  int ret;

  ninfo("buflen %lu\n", (unsigned long)buflen);

  DEBUGASSERT(psock != NULL && psock->s_crefs > 0 && to != NULL);
  DEBUGASSERT(psock->s_type == SOCK_DGRAM);

  sixlowpan_dumpbuffer("Outgoing UDP payload", buf, buflen);

  if (psock == NULL || to == NULL)
    {
      return (ssize_t)-EINVAL;
    }

  /* Make sure that this is a datagram valid socket */

  if (psock->s_crefs <= 0 || psock->s_type != SOCK_DGRAM)
    {
      nerr("ERROR: Invalid socket\n");
      return (ssize_t)-EBADF;
    }

  /* Make sure that the destination address is valid */

  if (to6->sin6_family != AF_INET6 || tolen < sizeof(struct sockaddr_in6))
    {
      nerr("ERROR: Invalid destination address: sin6_family=%u tolen = %u\n",
           to6->sin6_family, tolen);
      return (ssize_t)-EPROTOTYPE;
    }

  /* Get the underlying UDP "connection" structure */

  conn = (FAR struct udp_conn_s *)psock->s_conn;
  DEBUGASSERT(conn != NULL);

  /* Route outgoing message to the correct device */

  dev = netdev_findby_ipv6addr(conn->u.ipv6.laddr,
                               to6->sin6_addr.in6_u.u6_addr16);
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

  ret = icmpv6_neighbor(to6->sin6_addr.in6_u.u6_addr16);
  if (ret < 0)
    {
      nerr("ERROR: Not reachable\n");
      return (ssize_t)-ENETUNREACH;
    }
#endif

  /* Initialize the IPv6/UDP headers */

  ipv6udp.ipv6.vtc    = 0x60;
  ipv6udp.ipv6.tcf    = 0x00;
  ipv6udp.ipv6.flow   = 0x00;
  ipv6udp.ipv6.proto  = IP_PROTO_UDP;
  ipv6udp.ipv6.ttl    = conn->ttl;

  /* The IPv6 header length field does not include the size of IPv6 IP
   * header.
   */

  iplen               = buflen + UDP_HDRLEN;
  ipv6udp.ipv6.len[0] = (iplen >> 8);
  ipv6udp.ipv6.len[1] = (iplen & 0xff);

  /* Copy the source and destination addresses */

  net_ipv6addr_hdrcopy(ipv6udp.ipv6.destipaddr, to6->sin6_addr.in6_u.u6_addr16);
  if (!net_ipv6addr_cmp(conn->u.ipv6.laddr, g_ipv6_unspecaddr))
    {
      net_ipv6addr_hdrcopy(ipv6udp.ipv6.srcipaddr, conn->u.ipv6.laddr);
    }
  else
    {
      net_ipv6addr_hdrcopy(ipv6udp.ipv6.srcipaddr, dev->d_ipv6addr);
    }

  ninfo("IPv6 length: %d\n",
        ((int)ipv6udp.ipv6.len[0] << 8) + ipv6udp.ipv6.len[1]);

#ifdef CONFIG_NET_STATISTICS
  g_netstats.ipv6.sent++;
#endif

  /* Initialize the UDP header */

  ipv6udp.udp.srcport     = conn->lport;
  ipv6udp.udp.destport    = to6->sin6_port;
  ipv6udp.udp.udplen      = htons(iplen);
  ipv6udp.udp.udpchksum   = 0;

#ifdef CONFIG_NET_UDP_CHECKSUMS
  ipv6udp.udp.udpchksum   = ~sixlowpan_udp_chksum(&ipv6udp, buf, buflen);
  if (ipv6udp.udp.udpchksum == 0)
    {
      ipv6udp.udp.udpchksum = 0xffff;
    }
#endif /* CONFIG_NET_UDP_CHECKSUMS */

  ninfo("Outgoing UDP packet length: %d\n", iplen + IPv6_HDRLEN);

#ifdef CONFIG_NET_STATISTICS
  g_netstats.udp.sent++;
#endif

  /* Get the IEEE 802.15.4 MAC address of the next hop. */

  ret = sixlowpan_nexthopaddr((FAR struct radio_driver_s *)dev,
                              to6->sin6_addr.in6_u.u6_addr16, &destmac);
  if (ret < 0)
    {
      nerr("ERROR: Failed to get dest MAC address: %d\n", ret);
      return (ssize_t)ret;
    }

  /* Set the socket state to sending */

  psock->s_flags = _SS_SETSTATE(psock->s_flags, _SF_SEND);

  /* If routable, then call sixlowpan_send() to format and send the 6LoWPAN
   * packet.
   */

#ifdef CONFIG_NET_SOCKOPTS
  timeout = psock->s_sndtimeo;
#else
  timeout = 0;
#endif

  ret = sixlowpan_send(dev, &conn->list,
                       (FAR const struct ipv6_hdr_s *)&ipv6udp,
                       buf, buflen, &destmac, timeout);
  if (ret < 0)
    {
      nerr("ERROR: sixlowpan_send() failed: %d\n", ret);
    }

  /* Set the socket state to idle */

  psock->s_flags = _SS_SETSTATE(psock->s_flags, _SF_IDLE);
  return (ssize_t)ret;
}

/****************************************************************************
 * Name: psock_6lowpan_udp_send
 *
 * Description:
 *   psock_6lowpan_udp_send() call may be used with connectionlesss UDP
 *   sockets.
 *
 * Input Parameters:
 *   psock  - An instance of the internal socket structure.
 *   buf    - Data to send
 *   buflen - Length of data to send
 *
 * Returned Value:
 *   On success, returns the number of characters sent.  On  error,
 *   -1 is returned, and errno is set appropriately.  Returned error numbers
 *   must be consistent with definition of errors reported by send().
 *
 * Assumptions:
 *   Called with the network locked.
 *
 ****************************************************************************/

ssize_t psock_6lowpan_udp_send(FAR struct socket *psock, FAR const void *buf,
                               size_t buflen)
{
  FAR struct udp_conn_s *conn;
  struct sockaddr_in6 to;

  ninfo("buflen %lu\n", (unsigned long)buflen);

  DEBUGASSERT(psock != NULL && psock->s_crefs > 0);
  DEBUGASSERT(psock->s_type == SOCK_DGRAM);

  sixlowpan_dumpbuffer("Outgoing UDP payload", buf, buflen);

  /* Make sure that this is a valid socket */

  if (psock != NULL || psock->s_crefs <= 0)
    {
      nerr("ERROR: Invalid socket\n");
      return (ssize_t)-EBADF;
    }

  /* Was the UDP socket connected via connect()? */

  if (psock->s_type != SOCK_DGRAM || !_SS_ISCONNECTED(psock->s_flags))
    {
      /* No, then it is not legal to call send() with this socket. */

      return -ENOTCONN;
    }

  /* Get the underlying UDP "connection" structure */

  conn = (FAR struct udp_conn_s *)psock->s_conn;
  DEBUGASSERT(conn != NULL);

  /* Ignore if not IPv6 domain */

  if (conn->domain != PF_INET6)
    {
      nwarn("WARNING: Not IPv6\n");
      return (ssize_t)-EPROTOTYPE;
    }

  /* Create the 'to' address */

  to.sin6_family = AF_INET6;
  to.sin6_port   = conn->rport;  /* Already network order */
  memcpy(to.sin6_addr.in6_u.u6_addr16, conn->u.ipv6.raddr, 16);

  return psock_6lowpan_udp_sendto(psock, buf, buflen, 0,
                                  (FAR const struct sockaddr *)&to,
                                  sizeof(struct sockaddr_in6));
}

/****************************************************************************
 * Name: sixlowpan_udp_send
 *
 * Description:
 *   Handles forwarding a UDP packet via 6LoWPAN.  This is currently only
 *   used by the IPv6 forwarding logic.
 *
 * Input Parameters:
 *   dev    - An instance of nework device state structure
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

#ifdef CONFIG_NET_IPFORWARD
void sixlowpan_udp_send(FAR struct net_driver_s *dev,
                        FAR struct net_driver_s *fwddev,
                        FAR struct ipv6_hdr_s *ipv6)
{
  FAR struct ipv6udp_hdr_s *ipv6udp = (FAR struct ipv6udp_hdr_s *)ipv6;

  /* Double check */

  DEBUGASSERT(dev != NULL && dev->d_len > 0 && fwddev != NULL);

  ninfo("d_len %u\n", dev->d_len);

  if (dev != NULL && dev->d_len > 0 && fwddev != NULL)
    {

      sixlowpan_dumpbuffer("Outgoing UDP packet",
                           (FAR const uint8_t *)ipv6udp, dev->d_len);

      /* The UDP data payload should follow the IPv6 header plus the
       * protocol header.
       */

      if (ipv6udp->ipv6.proto != IP_PROTO_UDP)
        {
          nwarn("WARNING: Expected UDP protoype: %u vs %s\n",
                ipv6udp->ipv6.proto, IP_PROTO_UDP);
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
                                      ipv6udp->ipv6.destipaddr, &destmac);
          if (ret < 0)
            {
              nerr("ERROR: Failed to get dest MAC address: %d\n", ret);
              goto drop;
            }

          /* Get the IPv6 + UDP combined header length. */

          hdrlen = IPv6_HDRLEN + UDP_HDRLEN;

          /* Drop the packet if the buffer length is less than this. */

          if (hdrlen > dev->d_len)
            {
              nwarn("WARNING:  Dropping small UDP packet: %u < %u\n",
                    buflen, hdrlen);
            }
          else
            {
              /* Convert the outgoing packet into a frame list. */

              buf    = (FAR uint8_t *)ipv6 + hdrlen;
              buflen = dev->d_len - hdrlen;

              (void)sixlowpan_queue_frames(
                      (FAR struct radio_driver_s *)fwddev,
                      &ipv6udp->ipv6, buf, buflen, &destmac);
            }
        }
    }

drop:
  dev->d_len = 0;
}
#endif

#endif /* CONFIG_NET_6LOWPAN && CONFIG_NET_UDP */
