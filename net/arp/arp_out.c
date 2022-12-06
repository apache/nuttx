/****************************************************************************
 * net/arp/arp_out.c
 *
 *   Copyright (C) 2007-2011, 2014-2015, 2017-2018 Gregory Nutt. All rights
 *     reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Based on uIP which also has a BSD style license:
 *
 *   Author: Adam Dunkels <adam@dunkels.com>
 *   Copyright (c) 2001-2003, Adam Dunkels.
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

#include <string.h>
#include <debug.h>

#include <nuttx/net/net.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/arp.h>

#include "route/route.h"
#include "arp/arp.h"

#ifdef CONFIG_NET_ARP

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Support for broadcast address */

static const struct ether_addr g_broadcast_ethaddr =
{
  {
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff
  }
};

static const uint16_t g_broadcast_ipaddr[2] =
{
  0xffff, 0xffff
};

/* Support for IGMP multicast addresses.
 *
 * Well-known ethernet multicast address:
 *
 * ADDRESS           TYPE   USAGE
 * 01-00-0c-cc-cc-cc 0x0802 CDP (Cisco Discovery Protocol),
 *                          VTP (Virtual Trunking Protocol)
 * 01-00-0c-cc-cc-cd 0x0802 Cisco Shared Spanning Tree Protocol Address
 * 01-80-c2-00-00-00 0x0802 Spanning Tree Protocol (for bridges) IEEE 802.1D
 * 01-80-c2-00-00-02 0x0809 Ethernet OAM Protocol IEEE 802.3ah
 * 01-00-5e-xx-xx-xx 0x0800 IPv4 IGMP Multicast Address
 * 33-33-00-00-00-00 0x86DD IPv6 Neighbor Discovery
 * 33-33-xx-xx-xx-xx 0x86DD IPv6 Multicast Address (RFC3307)
 *
 * The following is the first three octects of the IGMP address:
 */

#ifdef CONFIG_NET_IGMP
static const uint8_t g_multicast_ethaddr[3] =
{
  0x01, 0x00, 0x5e
};
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arp_out
 *
 * Description:
 *   This function should be called before sending out an IP packet. The
 *   function checks the destination IP address of the IP packet to see
 *   what Ethernet MAC address that should be used as a destination MAC
 *   address on the Ethernet.
 *
 *   If the destination IP address is in the local network (determined
 *   by logical ANDing of netmask and our IP address), the function
 *   checks the ARP cache to see if an entry for the destination IP
 *   address is found.  If so, an Ethernet header is pre-pended at the
 *   beginning of the packet and the function returns.
 *
 *   If no ARP cache entry is found for the destination IP address, the
 *   packet in the d_buf is replaced by an ARP request packet for the
 *   IP address. The IP packet is dropped and it is assumed that the
 *   higher level protocols (e.g., TCP) eventually will retransmit the
 *   dropped packet.
 *
 *   Upon return in either the case, a packet to be sent is present in the
 *   d_buf buffer and the d_len field holds the length of the Ethernet
 *   frame that should be transmitted.
 *
 ****************************************************************************/

void arp_out(FAR struct net_driver_s *dev)
{
  struct ether_addr ethaddr;
  FAR struct eth_hdr_s *peth = ETHBUF;
  FAR struct arp_iphdr_s *pip = ARPIPBUF;
  in_addr_t ipaddr;
  in_addr_t destipaddr;
  int ret;

  /* ARP support is only built if the Ethernet link layer is supported.
   * Continue and send the ARP request only if this device uses the
   * Ethernet link layer protocol.
   */

  if (dev->d_lltype != NET_LL_ETHERNET &&
      dev->d_lltype != NET_LL_IEEE80211)
    {
      return;
    }

#if defined(CONFIG_NET_PKT) || defined(CONFIG_NET_ARP_SEND)
  /* Skip sending ARP requests when the frame to be transmitted was
   * written into a packet socket.
   */

  if (IFF_IS_NOARP(dev->d_flags))
    {
      /* Clear the indication and let the packet continue on its way. */

      IFF_CLR_NOARP(dev->d_flags);
      return;
    }
#endif

  /* Find the destination IP address in the ARP table and construct
   * the Ethernet header. If the destination IP address isn't on the
   * local network, we use the default router's IP address instead.
   *
   * If not ARP table entry is found, we overwrite the original IP
   * packet with an ARP request for the IP address.
   */

  /* First check if destination is a local broadcast. */

  if (net_ipv4addr_hdrcmp(pip->eh_destipaddr, g_broadcast_ipaddr))
    {
      memcpy(peth->dest,
             g_broadcast_ethaddr.ether_addr_octet,
             ETHER_ADDR_LEN);
      goto finish_header;
    }

#ifdef CONFIG_NET_IGMP
  /* Check if the destination address is a multicast address
   *
   * - IPv4:
   *   multicast addresses lie in the class D group -- The address range
   *   224.0.0.0 to 239.255.255.255 (224.0.0.0/4)
   *
   * - IPv6 multicast addresses are have the high-order octet of the
   *   addresses=0xff (ff00::/8.)
   */

  if (NTOHS(pip->eh_destipaddr[0]) >= 0xe000 &&
      NTOHS(pip->eh_destipaddr[0]) <= 0xefff)
    {
      /* Build the well-known IPv4 IGMP Ethernet address.  The first
       * three bytes are fixed; the final three variable come from the
       * last three bytes of the IPv4 address (network order).
       *
       * Address range : 01:00:5e:00:00:00 to 01:00:5e:7f:ff:ff
       */

      FAR const uint8_t *ip = (FAR uint8_t *)pip->eh_destipaddr;

      peth->dest[0] = g_multicast_ethaddr[0];
      peth->dest[1] = g_multicast_ethaddr[1];
      peth->dest[2] = g_multicast_ethaddr[2];
      peth->dest[3] = ip[1] & 0x7f;
      peth->dest[4] = ip[2];
      peth->dest[5] = ip[3];

      goto finish_header;
    }
#endif

  /* Check if the destination address is on the local network. */

  destipaddr = net_ip4addr_conv32(pip->eh_destipaddr);
  if (!net_ipv4addr_maskcmp(destipaddr, dev->d_ipaddr, dev->d_netmask))
    {
      /* Destination address is not on the local network */

#ifdef CONFIG_NET_ROUTE
      /* We have a routing table.. find the correct router to use in
       * this case (or, as a fall-back, use the device's default router
       * address).  We will use the router IP address instead of the
       * destination address when determining the MAC address.
       */

      netdev_ipv4_router(dev, destipaddr, &ipaddr);
#else
      /* Use the device's default router IP address instead of the
       * destination address when determining the MAC address.
       */

      net_ipv4addr_copy(ipaddr, dev->d_draddr);
#endif
    }

  /* The destination address is on the local network.  Check if it is
   * the sub-net broadcast address.
   */

  else if (net_ipv4addr_broadcast(destipaddr, dev->d_netmask))
    {
      /* Yes.. then we won't need to know the destination MAC address */

      memcpy(peth->dest,
             g_broadcast_ethaddr.ether_addr_octet,
             ETHER_ADDR_LEN);
      goto finish_header;
    }
  else
    {
      /* Else, we use the destination IP address. */

      net_ipv4addr_copy(ipaddr, destipaddr);
    }

  /* Check if we already have this destination address in the ARP table */

  ret = arp_find(ipaddr, ethaddr.ether_addr_octet, dev);
  if (ret < 0)
    {
      ninfo("ARP request for IP %08lx\n", (unsigned long)ipaddr);

      /* The destination address was not in our ARP table, so we overwrite
       * the IP packet with an ARP request.
       */

      arp_format(dev, ipaddr);
      arp_dump(ARPBUF);
      return;
    }

  /* Build an Ethernet header. */

  memcpy(peth->dest, ethaddr.ether_addr_octet, ETHER_ADDR_LEN);

  /* Finish populating the Ethernet header */

finish_header:
  memcpy(peth->src, dev->d_mac.ether.ether_addr_octet, ETHER_ADDR_LEN);
  peth->type  = HTONS(ETHTYPE_IP);
  dev->d_len += ETH_HDRLEN;
}

#endif /* CONFIG_NET_ARP */
