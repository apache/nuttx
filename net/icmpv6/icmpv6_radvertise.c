/****************************************************************************
 * net/icmpv6/icmpv6_radvertise.c
 * Send an ICMPv6 Router Advertisement
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <string.h>
#include <debug.h>
#include <netinet/in.h>

#include <nuttx/net/netconfig.h>
#include <nuttx/net/netstats.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/icmpv6.h>
#include <nuttx/net/dns.h>

#include "netdev/netdev.h"
#include "inet/inet.h"
#include "utils/utils.h"
#include "icmpv6/icmpv6.h"

#ifdef CONFIG_NET_ICMPv6_ROUTER

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct rdnss_add_dns_nameserver_s
{
  FAR struct icmpv6_rdnss_s *rdnss;
  int nservers;
};

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
 * Name: icmpv6_radvertise_fill_rndss
 *
 * Description:
 *   Copy an IPv6 DNS address into RDNSS field
 *
 * Input Parameters:
 *   arg     - RDNSS context infomation
 *   addr    - DNS server address
 *   addrlen - length of DNS server address
 *
 * Returned Value:
 *   0 is success
 *
 ****************************************************************************/

#ifdef CONFIG_NET_ICMPv6_ROUTER_RDNSS
static int icmpv6_radvertise_fill_rndss(FAR void *arg,
                                        FAR struct sockaddr *addr,
                                        FAR socklen_t addrlen)
{
  FAR struct rdnss_add_dns_nameserver_s *rdnss_context = arg;
  FAR struct sockaddr_in6 *addr6;

  if (addr->sa_family == AF_INET6)
    {
      FAR uint8_t *server = rdnss_context->rdnss->servers;

      addr6 = (FAR struct sockaddr_in6 *)addr;
      net_ipv6addr_copy(server + 16 * rdnss_context->nservers,
                        &addr6->sin6_addr);
      rdnss_context->nservers++;
    }

  return 0;
}
#endif /* CONFIG_NET_ICMPv6_ROUTER_RDNSS */

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
  FAR struct icmpv6_router_advertise_s *adv;
  FAR struct icmpv6_srclladdr_s *srcaddr;
  FAR struct icmpv6_mtu_s *mtu;
  FAR struct icmpv6_prefixinfo_s *prefix;
#ifdef CONFIG_NET_ICMPv6_ROUTER_RDNSS
  FAR struct icmpv6_rdnss_s *rdnss;
  struct rdnss_add_dns_nameserver_s rndss_context;
#endif
  net_ipv6addr_t srcv6addr;
  uint16_t lladdrsize;
  uint16_t l3size;

  /* Length excludes the IPv6 header */

  lladdrsize   = netdev_lladdrsize(dev);
  l3size       = sizeof(struct icmpv6_router_advertise_s) +
                 SIZEOF_ICMPV6_SRCLLADDR_S(lladdrsize) +
                 sizeof(struct icmpv6_mtu_s) +
                 sizeof(struct icmpv6_prefixinfo_s);

  /* Source IP address must be set to link-local IP */

  icmpv6_linkipaddr(dev, srcv6addr);

  /* Set up the ICMPv6 Router Advertise response */

  adv               = IPBUF(IPv6_HDRLEN);
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
  mtu->mtu[0]       = 0;
  mtu->mtu[1]       = HTONS(dev->d_pktsize - dev->d_llhdrlen);

  /* Set up the prefix option */

  prefix               = (FAR struct icmpv6_prefixinfo_s *)
                         ((FAR uint8_t *)mtu + sizeof(struct icmpv6_mtu_s));
  prefix->opttype      = ICMPv6_OPT_PREFIX;
  prefix->optlen       = 4;
  prefix->flags        = ICMPv6_PRFX_FLAG_L | ICMPv6_PRFX_FLAG_A;
  prefix->vlifetime[0] = HTONS(2592000 >> 16);
  prefix->vlifetime[1] = HTONS(2592000 & 0xffff);
  prefix->plifetime[0] = HTONS(604800 >> 16);
  prefix->plifetime[1] = HTONS(604800 & 0xffff);
  prefix->reserved[0]  = 0;
  prefix->reserved[1]  = 0;

#ifdef CONFIG_NET_ICMPv6_ROUTER_MANUAL
  /* Copy the configured prefex */

  prefix->preflen     = CONFIG_NET_ICMPv6_PREFLEN;
  net_ipv6addr_copy(prefix->prefix, g_ipv6_prefix);
#else
  /* Set the prefix and prefix length based on net driver IP and netmask */

  prefix->preflen     = net_ipv6_mask2pref(dev->d_ipv6netmask);
  ipv6addr_mask(prefix->prefix, dev->d_ipv6addr, dev->d_ipv6netmask);
#endif /* CONFIG_NET_ICMPv6_ROUTER_MANUAL */

#ifdef CONFIG_NET_ICMPv6_ROUTER_RDNSS
  rdnss                  = (FAR struct icmpv6_rdnss_s *)
                           ((FAR uint8_t *)prefix +
                           sizeof(struct icmpv6_prefixinfo_s));
  rndss_context.rdnss    = rdnss;
  rndss_context.nservers = 0;

  dns_foreach_nameserver(icmpv6_radvertise_fill_rndss, &rndss_context);

  if (rndss_context.nservers > 0)
    {
      rdnss->opttype     = ICMPv6_OPT_RDNSS;
      rdnss->optlen      = 1 + 2 * rndss_context.nservers;
      rdnss->reserved    = 0;

      /* RFC8106: The value of Lifetime SHOULD by default be at least
       * 3 * MaxRtrAdvInterval and the MaxRtrAdvInterval default value
       * is 600 seconds.
       */

      rdnss->lifetime[0] = 0;
      rdnss->lifetime[1] = HTONS(1800);

      l3size            += 8 * rdnss->optlen;
    }

#endif

  ipv6_build_header(IPv6BUF, l3size, IP_PROTO_ICMP6,
                    srcv6addr, g_ipv6_allnodes, 255, 0);

  /* Update device buffer length */

  iob_update_pktlen(dev->d_iob, IPv6_HDRLEN + l3size);

  /* Calculate the checksum over both the ICMP header and payload */

  adv->chksum  = 0;
  adv->chksum  = ~icmpv6_chksum(dev, IPv6_HDRLEN);

  /* Set the size to the size of the IPv6 header and the payload size */

  dev->d_len   = IPv6_HDRLEN + l3size;

  ninfo("Outgoing ICMPv6 Router Advertise length: %d\n", dev->d_len);

#ifdef CONFIG_NET_STATISTICS
  g_netstats.icmpv6.sent++;
  g_netstats.ipv6.sent++;
#endif
}

#endif /* CONFIG_NET_ICMPv6_ROUTER */
