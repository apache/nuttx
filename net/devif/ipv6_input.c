/****************************************************************************
 * net/devif/ipv6_input.c
 * Device driver IPv6 packet receipt interface
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Adapted for NuttX from logic in uIP which also has a BSD-like license:
 *
 * uIP is an implementation of the TCP/IP protocol stack intended for
 * small 8-bit and 16-bit microcontrollers.
 *
 * uIP provides the necessary protocols for Internet communication,
 * with a very small code footprint and RAM requirements - the uIP
 * code size is on the order of a few kilobytes and RAM usage is on
 * the order of a few hundred bytes.
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
 * uIP is a small implementation of the IP, UDP and TCP protocols (as
 * well as some basic ICMP stuff). The implementation couples the IP,
 * UDP, TCP and the application layers very tightly. To keep the size
 * of the compiled code down, this code frequently uses the goto
 * statement. While it would be possible to break the ipv6_input()
 * function into many smaller functions, this would increase the code
 * size because of the overhead of parameter passing and the fact that
 * the optimizer would not be as efficient.
 *
 * The principle is that we have a small buffer, called the d_buf,
 * in which the device driver puts an incoming packet. The TCP/IP
 * stack parses the headers in the packet, and calls the
 * application. If the remote host has sent data to the application,
 * this data is present in the d_buf and the application read the
 * data from there. It is up to the application to put this data into
 * a byte stream if needed. The application will not be fed with data
 * that is out of sequence.
 *
 * If the application wishes to send data to the peer, it should put
 * its data into the d_buf. The d_appdata pointer points to the
 * first available byte. The TCP/IP stack will calculate the
 * checksums, and fill in the necessary header fields and finally send
 * the packet back to the peer.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#ifdef CONFIG_NET_IPv6

#include <sys/ioctl.h>
#include <stdint.h>
#include <debug.h>
#include <string.h>

#include <nuttx/net/netconfig.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/netstats.h>
#include <nuttx/net/ip.h>

#include "ipv6/ipv6.h"
#include "tcp/tcp.h"
#include "udp/udp.h"
#include "pkt/pkt.h"
#include "icmpv6/icmpv6.h"

#include "devif/devif.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Macros */

#define BUF  ((FAR struct net_ipv6hdr_s *)&dev->d_buf[NET_LL_HDRLEN(dev)])

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: ipv6_input
 *
 * Description:
 *
 * Returned Value:
 *   OK    The packet was processed (or dropped) and can be discarded.
 *   ERROR There is a matching connection, but could not dispatch the packet
 *         yet.  Currently useful for UDP when a packet arrives before a recv
 *         call is in place.
 *
 * Assumptions:
 *
 ****************************************************************************/

int ipv6_input(FAR struct net_driver_s *dev)
{
  FAR struct net_ipv6hdr_s *pbuf = BUF;
  uint16_t iplen;

  /* This is where the input processing starts. */

#ifdef CONFIG_NET_STATISTICS
  g_netstats.ip.recv++;
#endif

  /* Start of IP input header processing code. */
  /* Check validity of the IP header. */

  if ((pbuf->vtc & 0xf0) != 0x60)
    {
      /* IP version and header length. */

#ifdef CONFIG_NET_STATISTICS
      g_netstats.ip.drop++;
      g_netstats.ip.vhlerr++;
#endif
      nlldbg("Invalid IPv6 version: %d\n", pbuf->vtc >> 4);
      goto drop;
    }

  /* Check the size of the packet. If the size reported to us in d_len is
   * smaller the size reported in the IP header, we assume that the packet
   * has been corrupted in transit. If the size of d_len is larger than the
   * size reported in the IP packet header, the packet has been padded and
   * we set d_len to the correct value.
   *
   * The length reported in the IPv6 header is the length of the payload
   * that follows the header. However, uIP uses the d_len variable for
   * holding the size of the entire packet, including the IP header. For
   * IPv4 this is not a problem as the length field in the IPv4 header
   * contains the length of the entire packet. But for IPv6 we need to add
   * the size of the IPv6 header (40 bytes).
   */

  iplen = (pbuf->len[0] << 8) + pbuf->len[1] + IPv6_HDRLEN;
  if (iplen <= dev->d_len)
    {
      dev->d_len = iplen;
    }
  else
    {
      nlldbg("IP packet shorter than length in IP header\n");
      goto drop;
    }

   /* If IP broadcast support is configured, we check for a broadcast
    * UDP packet, which may be destined to us (even if there is no IP
    * address yet assigned to the device as is the case when we are
    * negotiating over DHCP for an address).
    */

#if defined(CONFIG_NET_BROADCAST) && defined(CONFIG_NET_UDP)
  if (pbuf->proto == IP_PROTO_UDP &&
      net_ipv6addr_cmp(pbuf->destipaddr, g_alloneaddr))
    {
      return udp_input(dev);
    }

  /* In most other cases, the device must be assigned a non-zero IP
   * address.  Another exception is when CONFIG_NET_PINGADDRCONF is
   * enabled...
   */

  else
#endif
#ifdef CONFIG_NET_ICMPv6
  if (net_ipv6addr_cmp(dev->d_ipaddr, g_allzeroaddr))
    {
      /* If we are configured to use ping IP address configuration and
       * hasn't been assigned an IP address yet, we accept all ICMP
       * packets.
       */

      nlldbg("No IP address assigned\n");
      goto drop;
    }

  /* Check if the packet is destined for out IP address */
  else
#endif
    {
      /* Check if the packet is destined for our IP address.
       *
       * For IPv6, packet reception is a little trickier as we need to
       * make sure that we listen to certain multicast addresses (all
       * hosts multicast address, and the solicited-node multicast
       * address) as well. However, we will cheat here and accept all
       * multicast packets that are sent to the ff02::/16 addresses.
       */

      if (!net_ipv6addr_cmp(pbuf->destipaddr, dev->d_ipaddr) &&
          pbuf->destipaddr[0] != 0xff02)
        {
#ifdef CONFIG_NET_STATISTICS
          g_netstats.ip.drop++;
#endif
          goto drop;
        }
    }

  /* Everything looks good so far.  Now process the incoming packet
   * according to the protocol.
   */

  switch (pbuf->proto)
    {
#ifdef CONFIG_NET_TCP
      case IP_PROTO_TCP:   /* TCP input */
        tcp_ipv6_input(dev);
        break;
#endif

#ifdef CONFIG_NET_UDP
      case IP_PROTO_UDP:   /* UDP input */
        udp_ipv6_input(dev);
        break;
#endif

  /* Check for ICMP input */

#ifdef CONFIG_NET_ICMPv6
      case IP_PROTO_ICMP6: /* ICMP6 input */
        icmpv6_input(dev);
        break;
#endif

      default:              /* Unrecognized/unsupported protocol */
#ifdef CONFIG_NET_STATISTICS
        g_netstats.ip.drop++;
        g_netstats.ip.protoerr++;
#endif

        nlldbg("Unrecognized IP protocol\n");
        goto drop;
    }

  /* Return and let the caller do any pending transmission. */

  return OK;

  /* Drop the packet.  NOTE that OK is returned meaning that the
   * packet has been processed (although processed unsuccessfully).
   */

drop:
  dev->d_len = 0;
  return OK;
}
#endif /* CONFIG_NET_IPv6 */
