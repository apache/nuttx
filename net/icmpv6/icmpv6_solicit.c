/****************************************************************************
 * net/icmpv6/icmpv6_solicit.c
 *
 *   Copyright (C) 2015, 2017 Gregory Nutt. All rights reserved.
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

#include <stdint.h>
#include <string.h>
#include <debug.h>

#include <nuttx/net/net.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/netstats.h>
#include <nuttx/net/ethernet.h>

#include "devif/devif.h"
#include "netdev/netdev.h"
#include "utils/utils.h"
#include "icmpv6/icmpv6.h"

#ifdef CONFIG_NET_ICMPv6

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ETHBUF   ((struct eth_hdr_s *)&dev->d_buf[0])
#define IPv6BUF  ((struct ipv6_hdr_s *)&dev->d_buf[NET_LL_HDRLEN(dev)])
#define ICMPv6SOLICIT \
  ((struct icmpv6_neighbor_solicit_s *)&dev->d_buf[NET_LL_HDRLEN(dev) + IPv6_HDRLEN])

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* First 6 hwords of the multi-cast address in network byte order */

static const uint16_t g_icmpv_mcastaddr[6] =
{
  HTONS(0xff02), HTONS(0x0000), HTONS(0x0000), HTONS(0x0000),
  HTONS(0x0000), HTONS(0x0001)
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: icmpv6_solicit
 *
 * Description:
 *   Set up to send an ICMPv6 Neighbor Solicitation message.  This version
 *   is for a standalone solicitation.  If formats:
 *
 *   - The Ethernet header
 *   - The IPv6 header
 *   - The ICMPv6 Neighbor Solicitation Message
 *
 * Input Parameters:
 *   dev - Reference to an Ethernet device driver structure
 *   ipaddr - IP address of Neighbor to be solicited
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void icmpv6_solicit(FAR struct net_driver_s *dev,
                    FAR const net_ipv6addr_t ipaddr)
{
  FAR struct ipv6_hdr_s *ipv6;
  FAR struct icmpv6_neighbor_solicit_s *sol;
  uint16_t lladdrsize;
  uint16_t l3size;

  /* Set up the IPv6 header (most is probably already in place) */

  ipv6          = IPv6BUF;
  ipv6->vtc     = 0x60;                    /* Version/traffic class (MS) */
  ipv6->tcf     = 0;                       /* Traffic class (LS)/Flow label (MS) */
  ipv6->flow    = 0;                       /* Flow label (LS) */

  /* Length excludes the IPv6 header */

  lladdrsize    = netdev_dev_lladdrsize(dev);
  l3size        = SIZEOF_ICMPV6_NEIGHBOR_SOLICIT_S(lladdrsize);
  ipv6->len[0]  = (l3size >> 8);
  ipv6->len[1]  = (l3size & 0xff);

  ipv6->proto   = IP_PROTO_ICMP6;          /* Next header */
  ipv6->ttl     = 255;                     /* Hop limit */

  /* Set the multicast destination IP address */

  memcpy(ipv6->destipaddr, g_icmpv_mcastaddr, 6*sizeof(uint16_t));
  ipv6->destipaddr[6] = ipaddr[6] | HTONS(0xff00);
  ipv6->destipaddr[7] = ipaddr[7];

  /* Add out IPv6 address as the source address */

  net_ipv6addr_copy(ipv6->srcipaddr, dev->d_ipv6addr);

  /* Set up the ICMPv6 Neighbor Solicitation message */

  sol           = ICMPv6SOLICIT;
  sol->type     = ICMPv6_NEIGHBOR_SOLICIT; /* Message type */
  sol->code     = 0;                       /* Message qualifier */
  sol->flags[0] = 0;                       /* flags */
  sol->flags[1] = 0;
  sol->flags[2] = 0;
  sol->flags[3] = 0;

  /* Copy the target address into the Neighbor Solicitation message */

  net_ipv6addr_copy(sol->tgtaddr, ipaddr);

  /* Set up the options */

  sol->opttype  = ICMPv6_OPT_SRCLLADDR;           /* Option type */
  sol->optlen   = ICMPv6_OPT_OCTECTS(lladdrsize); /* Option length in octets */

  /* Copy our link layer address into the message
   * REVISIT:  What if the link layer is not Ethernet?
   */

  memcpy(sol->srclladdr, &dev->d_mac, lladdrsize);

  /* Calculate the checksum over both the ICMP header and payload */

  sol->chksum   = 0;
  sol->chksum   = ~icmpv6_chksum(dev);

  /* Set the size to the size of the IPv6 header and the payload size */

  dev->d_len    = IPv6_HDRLEN + l3size;

#ifdef CONFIG_NET_ETHERNET
  if (dev->d_lltype == NET_LL_ETHERNET)
    {
      FAR struct eth_hdr_s *eth;

      /* Set the destination IPv6 multicast Ethernet address:
       *
       * For IPv6 multicast addresses, the Ethernet MAC is derived by
       * the four low-order octets OR'ed with the MAC 33:33:00:00:00:00,
       * so for example the IPv6 address FF02:DEAD:BEEF::1:3 would map
       * to the Ethernet MAC address 33:33:00:01:00:03.
       *
       * NOTES:  This appears correct for the ICMPv6 Router Solicitation
       * Message, but the ICMPv6 Neighbor Solicitation message seems to
       * use 33:33:ff:01:00:03.
       */

      eth          = ETHBUF;
      eth->dest[0] = 0x33;
      eth->dest[1] = 0x33;
      eth->dest[2] = 0xff;
      eth->dest[3] = ipaddr[6] >> 8;
      eth->dest[4] = ipaddr[7] & 0xff;
      eth->dest[5] = ipaddr[7] >> 8;

      /* Move our source Ethernet addresses into the Ethernet header */

      memcpy(eth->src, dev->d_mac.ether.ether_addr_octet, ETHER_ADDR_LEN);

      /* Set the IPv6 Ethernet type */

      eth->type  = HTONS(ETHTYPE_IP6);
#if 0
      /* No additional neighbor lookup is required on this packet.
       * REVISIT:  It is inappropriate to set this bit if we get here
       * via neighbor_out(); It is no necessary to set this bit if we
       * get here via icmpv6_input().  Is it ever necessary?
       */

      IFF_SET_NOARP(dev->d_flags);
#endif
    }
#endif

  /* Add the size of the layer layer header to the total size of the
   * outgoing packet.
   */

  dev->d_len += netdev_ipv6_hdrlen(dev);

  ninfo("Outgoing ICMPv6 Neighbor Solicitation length: %d (%d)\n",
          dev->d_len, (ipv6->len[0] << 8) | ipv6->len[1]);

#ifdef CONFIG_NET_STATISTICS
  g_netstats.icmpv6.sent++;
  g_netstats.ipv6.sent++;
#endif
}

#endif /* CONFIG_NET_ICMPv6 */
