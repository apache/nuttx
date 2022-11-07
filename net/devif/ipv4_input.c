/****************************************************************************
 * net/devif/ipv4_input.c
 * Device driver IPv4 packet receipt interface
 *
 *   Copyright (C) 2007-2009, 2013-2015, 2018-2019 Gregory Nutt. All rights
 *     reserved.
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
 * statement. While it would be possible to break the ipv4_input()
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
#ifdef CONFIG_NET_IPv4

#include <sys/ioctl.h>
#include <stdint.h>
#include <debug.h>
#include <string.h>

#include <netinet/in.h>
#include <net/if.h>

#include <nuttx/net/netconfig.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/netstats.h>
#include <nuttx/net/ip.h>

#include "inet/inet.h"
#include "tcp/tcp.h"
#include "udp/udp.h"
#include "pkt/pkt.h"
#include "icmp/icmp.h"
#include "igmp/igmp.h"

#include "ipforward/ipforward.h"
#include "devif/devif.h"

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
 * Name: ipv4_input
 *
 * Description:
 *
 * Returned Value:
 *   OK    - The packet was processed (or dropped) and can be discarded.
 *   ERROR - Hold the packet and try again later.  There is a listening
 *           socket but no receive in place to catch the packet yet.  The
 *           device's d_len will be set to zero in this case as there is
 *           no outgoing data.
 *
 ****************************************************************************/

int ipv4_input(FAR struct net_driver_s *dev)
{
  FAR struct ipv4_hdr_s *ipv4 = IPv4BUF;
  in_addr_t destipaddr;
  uint16_t llhdrlen;
  uint16_t totlen;

  /* This is where the input processing starts. */

#ifdef CONFIG_NET_STATISTICS
  g_netstats.ipv4.recv++;
#endif

  /* Start of IP input header processing code.
   *
   * Check validity of the IP header.
   * REVISIT:  Does not account for varying IP header length due to the
   * presences of IPv4 options.  The header length is encoded as a number
   * 32-bit words in the HL nibble of the VHL.
   */

  if ((ipv4->vhl & IP_VERSION_MASK) != 0x40 ||
      (ipv4->vhl & IPv4_HLMASK) < 5)
    {
      /* IP version and header length. */

#ifdef CONFIG_NET_STATISTICS
      g_netstats.ipv4.drop++;
      g_netstats.ipv4.vhlerr++;
#endif
      nwarn("WARNING: Invalid IP version or header length: %02x\n",
            ipv4->vhl);
      goto drop;
    }

  /* Get the size of the packet minus the size of link layer header */

  llhdrlen = NET_LL_HDRLEN(dev);
  if ((llhdrlen + IPv4_HDRLEN) > dev->d_len)
    {
      nwarn("WARNING: Packet shorter than IPv4 header\n");
      goto drop;
    }

  dev->d_len -= llhdrlen;

  /* Check the size of the packet.  If the size reported to us in d_len is
   * smaller the size reported in the IP header, we assume that the packet
   * has been corrupted in transit.  If the size of d_len is larger than the
   * size reported in the IP packet header, the packet has been padded and
   * we set d_len to the correct value.
   */

  totlen = (ipv4->len[0] << 8) + ipv4->len[1];
  if (totlen <= dev->d_len)
    {
      dev->d_len = totlen;
    }
  else
    {
      nwarn("WARNING: IP packet shorter than length in IP header\n");
      goto drop;
    }

  /* Check the fragment flag. */

  if ((ipv4->ipoffset[0] & 0x3f) != 0 || ipv4->ipoffset[1] != 0)
    {
#ifdef CONFIG_NET_STATISTICS
      g_netstats.ipv4.drop++;
      g_netstats.ipv4.fragerr++;
#endif
      nwarn("WARNING: IP fragment dropped\n");
      goto drop;
    }

  /* Get the destination IP address in a friendlier form */

  destipaddr = net_ip4addr_conv32(ipv4->destipaddr);

#ifdef CONFIG_NETUTILS_IPTLITE
  /* Check if packet needs to be dropped */

  bool is_valid_packet = nflite_verify_ipv4(dev);
  if (!is_valid_packet) goto drop;
#endif

#if defined(CONFIG_NET_BROADCAST) && defined(NET_UDP_HAVE_STACK)
  /* If IP broadcast support is configured, we check for a broadcast
   * UDP packet, which may be destined to us (even if there is no IP
   * address yet assigned to the device as is the case when we are
   * negotiating over DHCP for an address).
   */

  if (ipv4->proto == IP_PROTO_UDP &&
      net_ipv4addr_cmp(destipaddr, INADDR_BROADCAST))
    {
#ifdef CONFIG_NET_IPFORWARD_BROADCAST
      /* Forward broadcast packets */

      ipv4_forward_broadcast(dev, ipv4);
#endif
      return udp_ipv4_input(dev);
    }
  else
#endif
#if defined(CONFIG_NET_BROADCAST) && defined(NET_UDP_HAVE_STACK)
  /* The address is not the broadcast address and we have been assigned a
   * address.  So there is also the possibility that the destination address
   * is a sub-net broadcast address which we will need to handle just as for
   * the broadcast address above.
   */

  if (ipv4->proto == IP_PROTO_UDP &&
      net_ipv4addr_maskcmp(destipaddr, dev->d_ipaddr, dev->d_netmask) &&
      net_ipv4addr_broadcast(destipaddr, dev->d_netmask))
    {
#ifdef CONFIG_NET_IPFORWARD_BROADCAST
      /* Forward broadcast packets */

      ipv4_forward_broadcast(dev, ipv4);
#endif
      return udp_ipv4_input(dev);
    }
  else
#endif
  /* Check if the packet is destined for our IP address. */

  if (!net_ipv4addr_cmp(destipaddr, dev->d_ipaddr))
    {
      /* No.. This is not our IP address. Check for an IPv4 IGMP group
       * address
       */

#ifdef CONFIG_NET_IGMP
      in_addr_t destip = net_ip4addr_conv32(ipv4->destipaddr);
      if (igmp_grpfind(dev, &destip) != NULL)
        {
#ifdef CONFIG_NET_IPFORWARD_BROADCAST
          /* Forward multicast packets */

          ipv4_forward_broadcast(dev, ipv4);
#endif
        }
      else
#endif
        {
          /* No.. The packet is not destined for us. */

#ifdef CONFIG_NET_IPFORWARD
          /* Try to forward the packet */

          int ret = ipv4_forward(dev, ipv4);
          if (ret >= 0)
            {
              /* The packet was forwarded.  Return success; d_len will
               * be set appropriately by the forwarding logic:  Cleared
               * if the packet is forward via anoother device or non-
               * zero if it will be forwarded by the same device that
               * it was received on.
               */

              return OK;
            }
          else
#endif
#if defined(NET_UDP_HAVE_STACK) && defined(CONFIG_NET_BINDTODEVICE)
          /* If the protocol specific socket option NET_BINDTODEVICE
           * is selected, then we must forward all UDP packets to the bound
           * socket.
           */

          if (ipv4->proto != IP_PROTO_UDP)
#endif
            {
              /* Not destined for us and not forwardable... Drop the
               * packet.
               */

              ninfo("WARNING: Not destined for us; not forwardable... "
                    "Dropping!\n");

#ifdef CONFIG_NET_STATISTICS
              g_netstats.ipv4.drop++;
#endif
              goto drop;
            }
        }
    }
#ifdef CONFIG_NET_ICMP

  /* In other cases, the device must be assigned a non-zero IP address. */

  else if (net_ipv4addr_cmp(dev->d_ipaddr, INADDR_ANY))
    {
      nwarn("WARNING: No IP address assigned\n");
      goto drop;
    }
#endif

  if (ipv4_chksum(dev) != 0xffff)
    {
      /* Compute and check the IP header checksum. */

#ifdef CONFIG_NET_STATISTICS
      g_netstats.ipv4.drop++;
      g_netstats.ipv4.chkerr++;
#endif
      nwarn("WARNING: Bad IP checksum\n");
      goto drop;
    }

  /* Make sure that all packet processing logic knows that there is an IPv4
   * packet in the device buffer.
   */

  IFF_SET_IPv4(dev->d_flags);

  /* Now process the incoming packet according to the protocol. */

  switch (ipv4->proto)
    {
#ifdef NET_TCP_HAVE_STACK
      case IP_PROTO_TCP:   /* TCP input */
        tcp_ipv4_input(dev);
        break;
#endif

#ifdef NET_UDP_HAVE_STACK
      case IP_PROTO_UDP:   /* UDP input */
        udp_ipv4_input(dev);
        break;
#endif

#ifdef NET_ICMP_HAVE_STACK
  /* Check for ICMP input */

      case IP_PROTO_ICMP:  /* ICMP input */
        icmp_input(dev);
        break;
#endif

#ifdef CONFIG_NET_IGMP
  /* Check for IGMP input */

      case IP_PROTO_IGMP:  /* IGMP input */
        igmp_input(dev);
        break;
#endif

      default:              /* Unrecognized/unsupported protocol */
#ifdef CONFIG_NET_STATISTICS
        g_netstats.ipv4.drop++;
        g_netstats.ipv4.protoerr++;
#endif

        nwarn("WARNING: Unrecognized IP protocol\n");
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
#endif /* CONFIG_NET_IPv4 */
