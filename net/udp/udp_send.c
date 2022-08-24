/****************************************************************************
 * net/udp/udp_send.c
 *
 *   Copyright (C) 2007-2009, 2011, 2015 Gregory Nutt. All rights reserved.
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
#if defined(CONFIG_NET) && defined(CONFIG_NET_UDP)

#include <string.h>
#include <debug.h>
#include <assert.h>

#include <arpa/inet.h>

#include <nuttx/net/netconfig.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/netstats.h>
#include <nuttx/net/ip.h>
#include <nuttx/net/udp.h>

#include "devif/devif.h"
#include "inet/inet.h"
#include "utils/utils.h"
#include "udp/udp.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: udp_send
 *
 * Description:
 *   Set-up to send a UDP packet
 *
 * Input Parameters:
 *   dev  - The device driver structure to use in the send operation
 *   conn - The UDP "connection" structure holding port information
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

void udp_send(FAR struct net_driver_s *dev, FAR struct udp_conn_s *conn)
{
  FAR struct udp_hdr_s *udp;
#ifdef CONFIG_NET_IPv4
  in_addr_t raddr;
#endif

  ninfo("UDP payload: %d (%d) bytes\n", dev->d_sndlen, dev->d_len);

  if (dev->d_sndlen > 0)
    {
#ifdef CONFIG_NET_IPv4
#ifdef CONFIG_NET_IPv6
      if (conn->domain == PF_INET ||
          (conn->domain == PF_INET6 &&
           ip6_is_ipv4addr((FAR struct in6_addr *)conn->u.ipv6.raddr)))
#endif
        {
          udp = UDPIPv4BUF;
#ifdef CONFIG_NET_IPv6
          if (conn->domain == PF_INET6 &&
              ip6_is_ipv4addr((FAR struct in6_addr *)conn->u.ipv6.raddr))
            {
              raddr =
                ip6_get_ipv4addr((FAR struct in6_addr *)conn->u.ipv6.raddr);
            }
          else
#endif
            {
              raddr = conn->u.ipv4.raddr;
            }

          /* The total length to send is the size of the application data
           * plus the IPv4 and UDP headers (and, eventually, the link layer
           * header)
           */

          dev->d_len        = dev->d_sndlen + IPv4UDP_HDRLEN;

          ipv4_build_header(IPv4BUF, dev->d_len, IP_PROTO_UDP,
                            &dev->d_ipaddr, &raddr, IP_TTL_DEFAULT,
                            NULL);

#ifdef CONFIG_NET_STATISTICS
          g_netstats.ipv4.sent++;
#endif
        }
#endif /* CONFIG_NET_IPv4 */

#ifdef CONFIG_NET_IPv6
#ifdef CONFIG_NET_IPv4
      else
#endif
        {
          /* Get pointers to the IPv6 header and the offset UDP header */

          DEBUGASSERT(IFF_IS_IPv6(dev->d_flags));
          udp = UDPIPv6BUF;

          /* The IPv6 length, Includes the UDP header size but not the IPv6
           * header size
           */

          dev->d_len        = dev->d_sndlen + UDP_HDRLEN;

          ipv6_build_header(IPv6BUF, dev->d_len, IP_PROTO_UDP,
                            dev->d_ipv6addr, conn->u.ipv6.raddr, conn->ttl);

          /* The total length to send is the size of the application data
           * plus the IPv6 and UDP headers (and, eventually, the link layer
           * header)
           */

          dev->d_len       += IPv6_HDRLEN;

#ifdef CONFIG_NET_STATISTICS
          g_netstats.ipv6.sent++;
#endif
        }
#endif /* CONFIG_NET_IPv6 */

      /* Initialize the UDP header */

      udp->srcport     = conn->lport;
      udp->destport    = conn->rport;
      udp->udplen      = HTONS(dev->d_sndlen + UDP_HDRLEN);
      udp->udpchksum   = 0;

      /* Update the device buffer length */

      iob_update_pktlen(dev->d_iob, dev->d_len);

#ifdef CONFIG_NET_UDP_CHECKSUMS
      /* Calculate UDP checksum. */

#ifdef CONFIG_NET_IPv4
#ifdef CONFIG_NET_IPv6
      if (conn->domain == PF_INET ||
          (conn->domain == PF_INET6 &&
           ip6_is_ipv4addr((FAR struct in6_addr *)conn->u.ipv6.raddr)))
#endif
        {
          udp->udpchksum = ~udp_ipv4_chksum(dev);
        }
#endif /* CONFIG_NET_IPv4 */

#ifdef CONFIG_NET_IPv6
#ifdef CONFIG_NET_IPv4
      else
#endif
        {
          udp->udpchksum = ~udp_ipv6_chksum(dev);
        }
#endif /* CONFIG_NET_IPv6 */

      if (udp->udpchksum == 0)
        {
          udp->udpchksum = 0xffff;
        }
#endif /* CONFIG_NET_UDP_CHECKSUMS */

      ninfo("Outgoing UDP packet length: %d\n", dev->d_len);

#ifdef CONFIG_NET_STATISTICS
      g_netstats.udp.sent++;
#endif
    }
}

/****************************************************************************
 * Name: udpip_hdrsize
 *
 * Description:
 *   Get the total size of L3 and L4 UDP header
 *
 * Input Parameters:
 *   conn     The connection structure associated with the socket
 *
 * Returned Value:
 *   the total size of L3 and L4 TCP header
 *
 ****************************************************************************/

uint16_t udpip_hdrsize(FAR struct udp_conn_s *conn)
{
  uint16_t hdrsize = sizeof(struct udp_hdr_s);

#if defined(CONFIG_NET_IPv4) && defined(CONFIG_NET_IPv6)
  /* Which domain the socket used */

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
#endif /* CONFIG_NET && CONFIG_NET_UDP */
