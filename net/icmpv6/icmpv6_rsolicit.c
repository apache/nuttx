/****************************************************************************
 * net/icmpv6/icmpv6_rsolicit.c
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
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

#include <nuttx/net/netdev.h>
#include <nuttx/net/netstats.h>

#include "devif/devif.h"
#include "utils/utils.h"
#include "icmpv6/icmpv6.h"

#ifdef CONFIG_NET_ICMPv6_AUTOCONF

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ETHBUF    ((struct eth_hdr_s *)&dev->d_buf[0])
#define ICMPv6BUF ((struct icmpv6_iphdr_s *)&dev->d_buf[NET_LL_HDRLEN(dev)])
#define ICMPv6RSOLICIT \
  ((struct icmpv6_router_solicit_s *)&dev->d_buf[NET_LL_HDRLEN(dev) + IPv6_HDRLEN])

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: icmpv6_rsolicit
 *
 * Description:
 *   Set up to send an ICMPv6 Router Solicitation message.  This version
 *   is for a standalone solicitation.  If formats:
 *
 *   - The Ethernet header
 *   - The IPv6 header
 *   - The ICMPv6 Router Solicitation Message
 *
 *   The device IP address should have been set to the link local address
 *   prior to calling this function.
 *
 * Parameters:
 *   dev - Reference to an Ethernet device driver structure
 *
 * Return:
 *   None
 *
 ****************************************************************************/

void icmpv6_rsolicit(FAR struct net_driver_s *dev)
{
  FAR struct icmpv6_iphdr_s *icmp;
  FAR struct icmpv6_router_solicit_s *sol;
  FAR struct eth_hdr_s *eth;

  /* Set up the IPv6 header (most is probably already in place) */

  icmp          = ICMPv6BUF;
  icmp->vtc     = 0x60;                    /* Version/traffic class (MS) */
  icmp->tcf     = 0;                       /* Traffic class (LS)/Flow label (MS) */
  icmp->flow    = 0;                       /* Flow label (LS) */

  /* Length excludes the IPv6 header */

  icmp->len[0]  = (sizeof(struct icmpv6_router_solicit_s) >> 8);
  icmp->len[1]  = (sizeof(struct icmpv6_router_solicit_s) & 0xff);

  icmp->proto   = IP_PROTO_ICMP6;          /* Next header */
  icmp->ttl     = 255;                     /* Hop limit */

  /* Set the multicast destination IP address to the IPv6 all link-
   * loocal routers address: ff02::2
   */

  net_ipv6addr_copy(icmp->destipaddr, g_ipv6_allrouters);

  /* Add our link local IPv6 address as the source address */

  net_ipv6addr_copy(icmp->srcipaddr, dev->d_ipv6addr);

  /* Set up the ICMPv6 Router Solicitation message */

  sol           = ICMPv6RSOLICIT;
  sol->type     = ICMPV6_ROUTER_SOLICIT;   /* Message type */
  sol->code     = 0;                       /* Message qualifier */
  sol->flags[0] = 0;                       /* flags */
  sol->flags[1] = 0;
  sol->flags[2] = 0;
  sol->flags[3] = 0;

  /* Set up the options */

  sol->opttype  = ICMPv6_OPT_SRCLLADDR;    /* Option type */
  sol->optlen   = 1;                       /* Option length = 1 octet */

  /* Copy our link layer address into the message
   * REVISIT:  What if the link layer is not Ethernet?
   */

  memcpy(sol->srclladdr, dev->d_mac.ether_addr_octet, sizeof(net_ipv6addr_t));

  /* Calculate the checksum over both the ICMP header and payload */

  icmp->chksum  = 0;
  icmp->chksum  = ~icmpv6_chksum(dev);

  /* Set the size to the size of the IPv6 header and the payload size */

  dev->d_len    = IPv6_HDRLEN + sizeof(struct icmpv6_router_solicit_s);

#ifdef CONFIG_NET_ETHERNET
#ifdef CONFIG_NET_MULTILINK
  if (dev->d_lltype == NET_LL_ETHERNET)
#endif
    {
      /* Set the destination IPv6 all-routers multicast Ethernet
       * address
       */

      eth = ETHBUF;
      memcpy(eth->dest, g_ipv6_ethallrouters.ether_addr_octet, ETHER_ADDR_LEN);

      /* Move our source Ethernet addresses into the Ethernet header */

      memcpy(eth->src, dev->d_mac.ether_addr_octet, ETHER_ADDR_LEN);

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
  nllvdbg("Outgoing ICMPv6 Router Solicitation length: %d (%d)\n",
          dev->d_len, (icmp->len[0] << 8) | icmp->len[1]);

#ifdef CONFIG_NET_STATISTICS
  g_netstats.icmpv6.sent++;
  g_netstats.ipv6.sent++;
#endif
}

#endif /* CONFIG_NET_ICMPv6_AUTOCONF */
