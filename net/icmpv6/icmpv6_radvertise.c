/****************************************************************************
 * net/icmpv6/icmpv6_radvertise.c
 * Send an ICMPv6 Router Advertisement
 *
 *   Copyright (C) 2015, 2017, 2020 Gregory Nutt. All rights reserved.
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

#include "netdev/netdev.h"
#include "inet/inet.h"
#include "utils/utils.h"
#include "icmpv6/icmpv6.h"

#ifdef CONFIG_NET_ICMPv6_ROUTER

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define IPv6BUF  ((struct ipv6_hdr_s *)&dev->d_buf[NET_LL_HDRLEN(dev)])

#define ICMPv6ADVERTISE \
  ((struct icmpv6_router_advertise_s *)&dev->d_buf[NET_LL_HDRLEN(dev) + IPv6_HDRLEN])

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_NET_ICMPv6_ROUTER_MANUAL
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
#endif /* CONFIG_NET_ICMPv6_ROUTER_MANUAL */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ipv6addr_mask
 *
 * Description:
 *   Copy an IPv6 address under a mask
 *
 * Input Parameters:
 *   dest  - Location to return the masked address
 *   src   - The IPv6 address to mask
 *   maksk - The address mask
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifndef CONFIG_NET_ICMPv6_ROUTER_MANUAL
static inline void ipv6addr_mask(FAR uint16_t *dest, FAR const uint16_t *src,
                                 FAR const uint16_t *mask)
{
  int i;

  for (i = 0; i < 8; ++i)
    {
      dest[i] = src[i] & mask[i];
    }
}
#endif /* !CONFIG_NET_ICMPv6_ROUTER_MANUAL */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: icmpv6_radvertise
 *
 * Description:
 *   Send an ICMPv6 Router Advertisement
 *
 * Input Parameters:
 *   dev - The device driver structure containing the outgoing ICMPv6 packet
 *         buffer
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The network is locked
 *
 ****************************************************************************/

void icmpv6_radvertise(FAR struct net_driver_s *dev)
{
  FAR struct ipv6_hdr_s *ipv6 = IPv6BUF;
  FAR struct icmpv6_router_advertise_s *adv;
  FAR struct icmpv6_srclladdr_s *srcaddr;
  FAR struct icmpv6_mtu_s *mtu;
  FAR struct icmpv6_prefixinfo_s *prefix;
  uint16_t lladdrsize;
  uint16_t l3size;

  /* Set up the IPv6 header */

  ipv6->vtc    = 0x60;                         /* Version/traffic class (MS) */
  ipv6->tcf    = 0;                            /* Traffic class (LS)/Flow label (MS) */
  ipv6->flow   = 0;                            /* Flow label (LS) */

  /* Length excludes the IPv6 header */

  lladdrsize   = netdev_lladdrsize(dev);
  l3size       = sizeof(struct icmpv6_router_advertise_s) +
                 SIZEOF_ICMPV6_SRCLLADDR_S(lladdrsize) +
                 sizeof(struct icmpv6_mtu_s) +
                 sizeof(struct icmpv6_prefixinfo_s);

  ipv6->len[0] = (l3size >> 8);
  ipv6->len[1] = (l3size & 0xff);

  ipv6->proto  = IP_PROTO_ICMP6;               /* Next header */
  ipv6->ttl    = 255;                          /* Hop limit */

  /* Swap source for destination IP address, add our source IP address */

  net_ipv6addr_copy(ipv6->destipaddr, g_ipv6_allnodes);

  /* Source IP address must be set to link-local IP */

  icmpv6_linkipaddr(dev, ipv6->srcipaddr);

  /* Set up the ICMPv6 Router Advertise response */

  adv               = ICMPv6ADVERTISE;
  adv->type         = ICMPV6_ROUTER_ADVERTISE; /* Message type */
  adv->code         = 0;                       /* Message qualifier */
  adv->hoplimit     = 64;                      /* Current hop limit */
  adv->flags        = ICMPv6_RADV_FLAG_M;      /* Managed address flag. */
  adv->lifetime     = HTONS(1800);             /* Router lifetime */
  adv->reachable    = 0;                       /* Reachable time */
  adv->retrans      = 0;                       /* Retransmission timer */

  /* Set up the source address option */

  srcaddr           = (FAR struct icmpv6_srclladdr_s *)
                      ((FAR uint8_t *)adv +
                      sizeof(struct icmpv6_router_advertise_s));
  srcaddr->opttype  = ICMPv6_OPT_SRCLLADDR;
  srcaddr->optlen   = ICMPv6_OPT_OCTECTS(lladdrsize);
  memcpy(srcaddr->srclladdr, &dev->d_mac, lladdrsize);

  /* Set up the MTU option */

  mtu               = (FAR struct icmpv6_mtu_s *)
                      ((FAR uint8_t *)srcaddr +
                       SIZEOF_ICMPV6_SRCLLADDR_S(lladdrsize));
  mtu->opttype      = ICMPv6_OPT_MTU;
  mtu->optlen       = 1;
  mtu->reserved     = 0;
  mtu->mtu          = HTONL(dev->d_pktsize - dev->d_llhdrlen);

  /* Set up the prefix option */

  prefix              = (FAR struct icmpv6_prefixinfo_s *)
                        ((FAR uint8_t *)mtu + sizeof(struct icmpv6_mtu_s));
  prefix->opttype     = ICMPv6_OPT_PREFIX;
  prefix->optlen      = 4;
  prefix->flags       = ICMPv6_PRFX_FLAG_L | ICMPv6_PRFX_FLAG_A;
  prefix->vlifetime   = HTONL(2592000);
  prefix->plifetime   = HTONL(604800);
  prefix->reserved[0] = 0;
  prefix->reserved[1] = 0;

#ifdef CONFIG_NET_ICMPv6_ROUTER_MANUAL
  /* Copy the configured prefex */

  prefix->preflen     = CONFIG_NET_ICMPv6_PREFLEN;
  net_ipv6addr_copy(prefix->prefix, g_ipv6_prefix);
#else
  /* Set the prefix and prefix length based on net driver IP and netmask */

  prefix->preflen     = net_ipv6_mask2pref(dev->d_ipv6netmask);
  ipv6addr_mask(prefix->prefix, dev->d_ipv6addr, dev->d_ipv6netmask);
#endif /* CONFIG_NET_ICMPv6_ROUTER_MANUAL */

  /* Calculate the checksum over both the ICMP header and payload */

  adv->chksum  = 0;
  adv->chksum  = ~icmpv6_chksum(dev, IPv6_HDRLEN);

  /* Set the size to the size of the IPv6 header and the payload size */

  dev->d_len   = IPv6_HDRLEN + l3size;

  ninfo("Outgoing ICMPv6 Router Advertise length: %d (%d)\n",
          dev->d_len, (ipv6->len[0] << 8) | ipv6->len[1]);

#ifdef CONFIG_NET_STATISTICS
  g_netstats.icmpv6.sent++;
  g_netstats.ipv6.sent++;
#endif
}

#endif /* CONFIG_NET_ICMPv6_ROUTER */
