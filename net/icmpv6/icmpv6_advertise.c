/****************************************************************************
 * net/icmpv6/icmpv6_advertise.c
 * Send an ICMPv6 Neighbor Advertisement
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
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

#include <stdint.h>
#include <string.h>
#include <debug.h>

#include <nuttx/net/netconfig.h>
#include <nuttx/net/netstats.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/icmpv6.h>

#include "utils/utils.h"
#include "icmpv6/icmpv6.h"

#ifdef CONFIG_NET_ICMPv6

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ETHBUF    ((struct eth_hdr_s *)&dev->d_buf[0])
#define ICMPv6BUF ((struct icmpv6_iphdr_s *)&dev->d_buf[NET_LL_HDRLEN(dev)])

#define ICMPv6ADVERTISE \
  ((struct icmpv6_neighbor_advertise_s *)&dev->d_buf[NET_LL_HDRLEN(dev) + IPv6_HDRLEN])

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: icmpv6_advertise
 *
 * Description:
 *   Send an ICMPv6 Neighbor Advertisement
 *
 * Parameters:
 *   dev - The device driver structure containing the outgoing ICMPv6 packet
 *         buffer
 *
 * Return:
 *   None
 *
 * Assumptions:
 *   The network is locked
 *
 ****************************************************************************/

void icmpv6_advertise(FAR struct net_driver_s *dev,
                      const net_ipv6addr_t destipaddr)
{
  FAR struct icmpv6_iphdr_s *icmp = ICMPv6BUF;
  FAR struct icmpv6_neighbor_advertise_s *adv;

  /* Set up the IPv6 header */

  icmp->vtc    = 0x60;                         /* Version/traffic class (MS) */
  icmp->tcf    = 0;                            /* Traffic class (LS)/Flow label (MS) */
  icmp->flow   = 0;                            /* Flow label (LS) */

  /* Length excludes the IPv6 header */

  icmp->len[0] = (sizeof(struct icmpv6_neighbor_advertise_s) >> 8);
  icmp->len[1] = (sizeof(struct icmpv6_neighbor_advertise_s) & 0xff);

  icmp->proto  = IP_PROTO_ICMP6;               /* Next header */
  icmp->ttl    = 255;                          /* Hop limit */

  /* Swap source for destination IP address, add our source IP address */

  net_ipv6addr_copy(icmp->destipaddr, destipaddr);
  net_ipv6addr_copy(icmp->srcipaddr, dev->d_ipv6addr);

  /* Set up the ICMPv6 Neighbor Advertise response */

  adv            = ICMPv6ADVERTISE;
  adv->type      = ICMPv6_NEIGHBOR_ADVERTISE;  /* Message type */
  adv->code      = 0;                          /* Message qualifier */
  adv->flags[0]  = ICMPv6_NADV_FLAG_S | ICMPv6_NADV_FLAG_O; /* Solicited+Override flags. */
  adv->flags[1]  = 0;
  adv->flags[2]  = 0;
  adv->flags[3]  = 0;

  /* Copy the target address into the Neighbor Advertisement message */

  net_ipv6addr_copy(adv->tgtaddr, dev->d_ipv6addr);

  /* Set up the options */

  adv->opttype   = ICMPv6_OPT_TGTLLADDR;       /* Option type */
  adv->optlen    = 1;                          /* Option length = 1 octet */

  /* Copy our link layer address into the message
   * REVISIT:  What if the link layer is not Ethernet?
   */

  memcpy(adv->tgtlladdr, &dev->d_mac, IFHWADDRLEN);

  /* Calculate the checksum over both the ICMP header and payload */

  icmp->chksum   = 0;
  icmp->chksum   = ~icmpv6_chksum(dev);

  /* Set the size to the size of the IPv6 header and the payload size */

  dev->d_len     = IPv6_HDRLEN + sizeof(struct icmpv6_neighbor_advertise_s);

#ifdef CONFIG_NET_ETHERNET
  /* Add the size of the Ethernet header */

  dev->d_len    += ETH_HDRLEN;

  /* Move the source and to the destination addresses in the Ethernet header
   * and use our MAC as the new source address
   */

#ifdef CONFIG_NET_MULTILINK
  if (dev->d_lltype == NET_LL_ETHERNET)
#endif
    {
      FAR struct eth_hdr_s *eth = ETHBUF;

      memcpy(eth->dest, eth->src, ETHER_ADDR_LEN);
      memcpy(eth->src, dev->d_mac.ether_addr_octet, ETHER_ADDR_LEN);

      /* Set the IPv6 Ethernet type */

      eth->type  = HTONS(ETHTYPE_IP6);
    }
#endif

  /* No additional neighbor lookup is required on this packet (We are using
   * a multicast address).
   */

  IFF_SET_NOARP(dev->d_flags);

  nllvdbg("Outgoing ICMPv6 Neighbor Advertise length: %d (%d)\n",
          dev->d_len, (icmp->len[0] << 8) | icmp->len[1]);

#ifdef CONFIG_NET_STATISTICS
  g_netstats.icmpv6.sent++;
  g_netstats.ipv6.sent++;
#endif
}

#endif /* CONFIG_NET_ICMPv6 */
