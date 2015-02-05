/****************************************************************************
 * net/icmpv6/icmpv6_radvertise.c
 * Send an ICMPv6 Router Advertisement
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

#ifdef CONFIG_NET_ICMPv6_ROUTER

/****************************************************************************
 * Private Types
 ****************************************************************************/
/* This is the same as struct icmpv6_router_advertise_s, but with the
 * source address and prefix options included for simplicity.
 */

struct icmpv6_radvertise_s
{
  uint8_t  type;             /* Message Type: ICMPV6_ROUTER_ADVERTISE */
  uint8_t  code;             /* Further qualifies the ICMP messages */
  uint16_t chksum;           /* Checksum of ICMP header and data */
  uint8_t  hoplimit;         /* Current hop limit */
  uint8_t  flags;            /* See ICMPv6_RADV_FLAG_* definitions */
  uint16_t lifetime;         /* Router lifetime */
  uint16_t reachable[2];     /* Reachable time */
  uint16_t retrans[2];       /* Retransmission timer */

  /* Options */

  struct icmpv6_srclladdr_s srcaddr;
  struct icmpv6_mtu_s mtu;
  struct icmpv6_prefixinfo_s prefix;
};

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ETHBUF    ((struct eth_hdr_s *)&dev->d_buf[0])
#define ICMPv6BUF ((struct icmpv6_iphdr_s *)&dev->d_buf[NET_LL_HDRLEN(dev)])

#define ICMPv6ADVERTISE \
  ((struct icmpv6_radvertise_s *)&dev->d_buf[NET_LL_HDRLEN(dev) + IPv6_HDRLEN])

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const net_ipv6addr_t g_ipv6_prefix =
{
  HTONS(CONFIG_NET_ICMPv6_PREFIX_1),
  HTONS(CONFIG_NET_ICMPv6_PREFIX_2),
  HTONS(CONFIG_NET_ICMPv6_PREFIX_3),
  HTONS(CONFIG_NET_ICMPv6_PREFIX_4),
  HTONS(CONFIG_NET_ICMPv6_PREFIX_5),
  HTONS(CONFIG_NET_ICMPv6_PREFIX_6),
  HTONS(CONFIG_NET_ICMPv6_PREFIX_7),
  HTONS(CONFIG_NET_ICMPv6_PREFIX_8)
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: icmpv6_radvertise
 *
 * Description:
 *   Send an ICMPv6 Router Advertisement
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

void icmpv6_radvertise(FAR struct net_driver_s *dev)
{
  FAR struct icmpv6_iphdr_s *icmp = ICMPv6BUF;
  FAR struct icmpv6_radvertise_s *adv;
  FAR struct icmpv6_srclladdr_s *srcaddr;
  FAR struct icmpv6_mtu_s *mtu;
  FAR struct icmpv6_prefixinfo_s *prefix;

  /* Set up the IPv6 header */

  icmp->vtc    = 0x60;                         /* Version/traffic class (MS) */
  icmp->tcf    = 0;                            /* Traffic class (LS)/Flow label (MS) */
  icmp->flow   = 0;                            /* Flow label (LS) */

  /* Length excludes the IPv6 header */

  icmp->len[0] = (sizeof(struct icmpv6_radvertise_s) >> 8);
  icmp->len[1] = (sizeof(struct icmpv6_radvertise_s) & 0xff);

  icmp->proto  = IP_PROTO_ICMP6;               /* Next header */
  icmp->ttl    = 255;                          /* Hop limit */

  /* Swap source for destination IP address, add our source IP address */

  net_ipv6addr_copy(icmp->destipaddr, g_ipv6_allnodes);
  net_ipv6addr_copy(icmp->srcipaddr, dev->d_ipv6addr);

  /* Set up the ICMPv6 Router Advertise response */

  adv               = ICMPv6ADVERTISE;
  adv->type         = ICMPV6_ROUTER_ADVERTISE; /* Message type */
  adv->code         = 0;                       /* Message qualifier */
  adv->hoplimit     = 64;                      /* Current hop limit */
  adv->flags        = ICMPv6_RADV_FLAG_M;      /* Managed address flag. */
  adv->lifetime     = HTONS(1800);             /* Router lifetime */
  adv->reachable[0] = 0;                       /* Reachable time */
  adv->reachable[1] = 0;
  adv->retrans[0]   = 0;                       /* Retransmission timer */
  adv->retrans[1]   = 0;

  /* Set up the source address option */

  srcaddr           = &adv->srcaddr;
  srcaddr->opttype  = ICMPv6_OPT_SRCLLADDR;
  srcaddr->optlen   = 1;
  memcpy(srcaddr->srclladdr, &dev->d_mac.ether_addr_octet, ETHER_ADDR_LEN);

  /* Set up the MTU option */

  mtu               = &adv->mtu;
  mtu->opttype      = ICMPv6_OPT_MTU;
  mtu->optlen       = 1;
  mtu->reserved     = 0;
  mtu->mtu[0]       = HTONS(CONFIG_NET_ETH_MTU >> 8);
  mtu->mtu[1]       = HTONS(CONFIG_NET_ETH_MTU & 0xff);

  /* Set up the prefix option */

  prefix            = &adv->prefix;
  prefix->opttype   = ICMPv6_OPT_MTU;
  prefix->optlen    = 4;
  prefix->preflen   = CONFIG_NET_ICMPv6_PREFLEN;
  prefix->flags     = ICMPv6_PRFX_FLAG_L | ICMPv6_PRFX_FLAG_A;

  prefix->vlifetime[0] = HTONS(2592000 >> 8);
  prefix->vlifetime[1] = HTONS(2592000 & 0xff);
  prefix->plifetime[0] = HTONS(604800 >> 8);
  prefix->plifetime[1] = HTONS(604800 & 0xff);
  prefix->reserved[0]  = 0;
  prefix->reserved[1]  = 0;
  net_ipv6addr_copy(prefix->prefix, g_ipv6_prefix);

  /* Calculate the checksum over both the ICMP header and payload */

  icmp->chksum = 0;
  icmp->chksum = ~icmpv6_chksum(dev);

  /* Set the size to the size of the IPv6 header and the payload size */

  dev->d_len   = IPv6_HDRLEN + sizeof(struct icmpv6_radvertise_s);

#ifdef CONFIG_NET_ETHERNET
  /* Add the size of the Ethernet header */

  dev->d_len  += ETH_HDRLEN;

  /* Move the source and to the destination addresses in the Ethernet header
   * and use our MAC as the new source address
   */

#ifdef CONFIG_NET_MULTILINK
  if (dev->d_lltype == NET_LL_ETHERNET)
#endif
    {
      FAR struct eth_hdr_s *eth = ETHBUF;

      memcpy(eth->dest, g_ipv6_ethallnodes.ether_addr_octet, ETHER_ADDR_LEN);
      memcpy(eth->src, dev->d_mac.ether_addr_octet, ETHER_ADDR_LEN);

      /* Set the IPv6 Ethernet type */

      eth->type  = HTONS(ETHTYPE_IP6);
    }
#endif

  /* No additional neighbor lookup is required on this packet (We are using
   * a multicast address).
   */

  IFF_SET_NOARP(dev->d_flags);

  nllvdbg("Outgoing ICMPv6 Router Advertise length: %d (%d)\n",
          dev->d_len, (icmp->len[0] << 8) | icmp->len[1]);

#ifdef CONFIG_NET_STATISTICS
  g_netstats.icmpv6.sent++;
  g_netstats.ipv6.sent++;
#endif
}

#endif /* CONFIG_NET_ICMPv6_ROUTER */
