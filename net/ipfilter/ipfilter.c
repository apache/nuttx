/****************************************************************************
 * net/ipfilter/ipfilter.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/net/icmpv6.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/udp.h>
#include <nuttx/queue.h>

#include "icmp/icmp.h"
#include "icmpv6/icmpv6.h"
#include "ipfilter/ipfilter.h"
#include "utils/utils.h"

#ifdef CONFIG_NET_IPFILTER

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SPORT_MATCH(entry, sport) \
  (((sport) >= (entry)->match.tcpudp.sports[0] && \
    (sport) <= (entry)->match.tcpudp.sports[1]) ^ (entry)->inv_sport)

#define DPORT_MATCH(entry, dport) \
  (((dport) >= (entry)->match.tcpudp.dports[0] && \
    (dport) <= (entry)->match.tcpudp.dports[1]) ^ (entry)->inv_dport)

#define ICMP_MATCH(entry, icmphdr) \
  (((entry)->match.icmp.type == 0xFF || \
    (entry)->match.icmp.type == (icmphdr)->type) ^ (entry)->inv_icmp)

/* Getting L4 header from IPv4/IPv6 header. */

#define IPv4_L4HDR(ipv4) \
  ((FAR void *)((FAR uint8_t *)(ipv4) + (((ipv4)->vhl & IPv4_HLMASK) << 2)))

#define IPv6_L4HDR(ipv6, proto) \
  ((FAR void *)(net_ipv6_payload((FAR struct ipv6_hdr_s *)(ipv6), &(proto))))

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_NET_IPv4
static sq_queue_t g_ipv4_filters[IPFILTER_CHAIN_MAX];
#endif
#ifdef CONFIG_NET_IPv6
static sq_queue_t g_ipv6_filters[IPFILTER_CHAIN_MAX];
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ipfilter_match_device
 *
 * Description:
 *   Match the packet with the filter entries on devices.
 *
 * Input Parameters:
 *   entry  - The filter entry to match
 *   indev  - The network device that the packet comes from
 *   outdev - The network device that the packet goes to
 *
 * Returned Value:
 *   true  - The input packet is matched
 *   false - The input packet is not matched
 *
 ****************************************************************************/

static bool ipfilter_match_device(FAR const struct ipfilter_entry_s *entry,
                                  FAR const struct net_driver_s *indev,
                                  FAR const struct net_driver_s *outdev)
{
  bool matched;

  if (indev != NULL && entry->indev != NULL)
    {
      matched = (indev == entry->indev) ^ entry->inv_indev;
      if (!matched)
        {
          return false;
        }
    }

  if (outdev != NULL && entry->outdev != NULL)
    {
      matched = (outdev == entry->outdev) ^ entry->inv_outdev;
      if (!matched)
        {
          return false;
        }
    }

  return true;
}

/****************************************************************************
 * Name: ipfilter_match_proto
 *
 * Description:
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

static bool ipfilter_match_proto(FAR const struct ipfilter_entry_s *entry,
                                 FAR const void *l4hdr, uint8_t proto)
{
  bool matched;

  if (entry->proto)
    {
      matched = (entry->proto == proto) ^ entry->inv_proto;
      if (!matched)
        {
          return false;
        }
    }

  if (entry->inv_proto)
    {
      /* Cannot match port/type for inversed proto, just success. */

      return true;
    }

  switch (proto)
    {
      case IP_PROTO_TCP:
      case IP_PROTO_UDP:
        if (entry->match_tcpudp)
          {
            /* Ports in TCP & UDP headers have same offset. */

            FAR const struct udp_hdr_s *udp = l4hdr;
            return SPORT_MATCH(entry, NTOHS(udp->srcport)) &&
                   DPORT_MATCH(entry, NTOHS(udp->destport));
          }

      case IP_PROTO_ICMP:
        if (entry->match_icmp)
          {
            FAR const struct icmp_hdr_s *icmp = l4hdr;
            return ICMP_MATCH(entry, icmp);
          }

      case IP_PROTO_ICMP6:
        if (entry->match_icmp)
          {
            FAR const struct icmpv6_hdr_s *icmpv6 = l4hdr;
            return ICMP_MATCH(entry, icmpv6);
          }

      default:
        return true;
    }
}

/****************************************************************************
 * Name: ipv4_filter_match / ipv6_filter_match
 *
 * Description:
 *   Match the input packet with the filter entries in the specified chain.
 *
 * Input Parameters:
 *   indev     - The network device that the packet comes from
 *   outdev    - The network device that the packet goes to
 *   ipv4/ipv6 - The IPv4/IPv6 header
 *   chain     - The chain to match the filter entries
 *
 * Returned Value:
 *   IPFILTER_TARGET_ACCEPT(0)  - The input packet is accepted
 *   IPFILTER_TARGET_DROP(-1)   - The input packet needs to be dropped
 *   IPFILTER_TARGET_REJECT(-2) - The input packet is rejected
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv4
static int ipv4_filter_match(FAR const struct net_driver_s *indev,
                             FAR const struct net_driver_s *outdev,
                             FAR const struct ipv4_hdr_s *ipv4,
                             enum ipfilter_chain_e chain)
{
  FAR const struct ipv4_filter_entry_s *filter;
  FAR const sq_queue_t *queue = &g_ipv4_filters[chain];
  FAR const sq_entry_t *entry;
  FAR const void *l4hdr;
  in_addr_t ipaddr;
  bool matched;

  /* Handle unexpected status, return ACCEPT to indicate doing nothing. */

  if ((indev == NULL && outdev == NULL) || ipv4 == NULL)
    {
      return IPFILTER_TARGET_ACCEPT;
    }

  l4hdr = IPv4_L4HDR(ipv4);

  sq_for_every(queue, entry)
    {
      filter = (FAR struct ipv4_filter_entry_s *)entry;

      /* Match device */

      if (!ipfilter_match_device(&filter->common, indev, outdev))
        {
          continue;
        }

      /* Match addresses */

      ipaddr  = net_ip4addr_conv32(ipv4->srcipaddr);
      matched = net_ipv4addr_maskcmp(filter->sip, ipaddr, filter->smsk)
                ^ filter->common.inv_srcip;
      if (!matched)
        {
          continue;
        }

      ipaddr  = net_ip4addr_conv32(ipv4->destipaddr);
      matched = net_ipv4addr_maskcmp(filter->dip, ipaddr, filter->dmsk)
                ^ filter->common.inv_dstip;
      if (!matched)
        {
          continue;
        }

      /* Match protocol */

      if (!ipfilter_match_proto(&filter->common, l4hdr, ipv4->proto))
        {
          continue;
        }

      /* Return the target action if matched. */

      return filter->common.target;
    }

  /* Normally there should be a default rule in chain, won't reach here. */

  ninfo("No filter matched, maybe uninitialized.\n");
  return IPFILTER_TARGET_ACCEPT;
}
#endif

#ifdef CONFIG_NET_IPv6
static int ipv6_filter_match(FAR const struct net_driver_s *indev,
                             FAR const struct net_driver_s *outdev,
                             FAR const struct ipv6_hdr_s *ipv6,
                             enum ipfilter_chain_e chain)
{
  FAR const struct ipv6_filter_entry_s *filter;
  FAR const sq_queue_t *queue = &g_ipv6_filters[chain];
  FAR const sq_entry_t *entry;
  FAR const void *l4hdr;
  uint8_t proto;
  bool matched;

  /* Handle unexpected status, return ACCEPT to indicate doing nothing. */

  if ((indev == NULL && outdev == NULL) || ipv6 == NULL)
    {
      return IPFILTER_TARGET_ACCEPT;
    }

  l4hdr = IPv6_L4HDR(ipv6, proto);

  sq_for_every(queue, entry)
    {
      filter = (FAR struct ipv6_filter_entry_s *)entry;

      /* Match device */

      if (!ipfilter_match_device(&filter->common, indev, outdev))
        {
          continue;
        }

      /* Match addresses */

      matched = net_ipv6addr_maskcmp(filter->sip, ipv6->srcipaddr,
                                     filter->smsk)
                ^ filter->common.inv_srcip;
      if (!matched)
        {
          continue;
        }

      matched = net_ipv6addr_maskcmp(filter->dip, ipv6->destipaddr,
                                     filter->dmsk)
                ^ filter->common.inv_dstip;
      if (!matched)
        {
          continue;
        }

      /* Match protocol */

      if (!ipfilter_match_proto(&filter->common, l4hdr, proto))
        {
          continue;
        }

      /* Return the target action if matched. */

      return filter->common.target;
    }

  /* Normally there should be a default rule in chain, won't reach here. */

  ninfo("No filter matched, maybe uninitialized.\n");
  return IPFILTER_TARGET_ACCEPT;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ipfilter_cfg_alloc
 *
 * Description:
 *   Allocate a new filter configuration entry for the given address family.
 *
 * Input Parameters:
 *   family - The address family of the filter entry
 *
 * Returned Value:
 *   A pointer to the newly allocated filter entry.  NULL is returned on
 *   failure.
 *
 ****************************************************************************/

FAR struct ipfilter_entry_s *ipfilter_cfg_alloc(sa_family_t family)
{
  /* We may optimize alloc / free later if we really have lots of configs. */

#ifdef CONFIG_NET_IPv4
  if (family == PF_INET)
    {
      return kmm_zalloc(sizeof(struct ipv4_filter_entry_s));
    }
#endif

#ifdef CONFIG_NET_IPv6
  if (family == PF_INET6)
    {
      return kmm_zalloc(sizeof(struct ipv6_filter_entry_s));
    }
#endif

  return NULL;
}

/****************************************************************************
 * Name: ipfilter_cfg_add
 *
 * Description:
 *   Add a new filter configuration entry for the given address family to the
 *   end of specified chain.
 *
 * Input Parameters:
 *   entry  - The filter entry to add
 *   family - The address family of the filter entry
 *   chain  - The chain to add the filter entry to
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void ipfilter_cfg_add(FAR struct ipfilter_entry_s *entry,
                      sa_family_t family, enum ipfilter_chain_e chain)
{
#ifdef CONFIG_NET_IPv4
  if (family == PF_INET)
    {
      sq_addlast((FAR sq_entry_t *)entry, &g_ipv4_filters[chain]);
    }
#endif

#ifdef CONFIG_NET_IPv6
  if (family == PF_INET6)
    {
      sq_addlast((FAR sq_entry_t *)entry, &g_ipv6_filters[chain]);
    }
#endif
}

/****************************************************************************
 * Name: ipfilter_cfg_clear
 *
 * Description:
 *   Clear all filter configuration entries for the given address family from
 *   the specified chain.
 *
 * Input Parameters:
 *   family - The address family of the filter entry to clear
 *   chain  - The chain to clear the filter entries from
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void ipfilter_cfg_clear(sa_family_t family, enum ipfilter_chain_e chain)
{
#ifdef CONFIG_NET_IPv4
  if (family == PF_INET)
    {
      FAR sq_queue_t *queue = &g_ipv4_filters[chain];
      while (!sq_empty(queue))
        {
          kmm_free(sq_remfirst(queue));
        }
    }
#endif

#ifdef CONFIG_NET_IPv6
  if (family == PF_INET6)
    {
      FAR sq_queue_t *queue = &g_ipv6_filters[chain];
      while (!sq_empty(queue))
        {
          kmm_free(sq_remfirst(queue));
        }
    }
#endif
}

/****************************************************************************
 * Name: ipv4_filter_in / ipv6_filter_in
 *
 * Description:
 *   Handling IPv4/IPv6 filter on local input.  Do nothing if the input
 *   packet is accepted, set d_len to 0 if the input packet needs to be
 *   dropped, and set d_iob to reject reply if the input packet is rejected.
 *
 * Input Parameters:
 *   dev - The network device that the packet comes from
 *
 * Returned Value:
 *   IPFILTER_TARGET_ACCEPT(0)  - The input packet is accepted
 *   IPFILTER_TARGET_DROP(-1)   - The input packet needs to be dropped
 *   IPFILTER_TARGET_REJECT(-2) - The input packet is rejected
 *
 * Assumptions:
 *   The network is locked.  The d_iob and d_len in the dev are set.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv4
int ipv4_filter_in(FAR struct net_driver_s *dev)
{
  FAR struct ipv4_hdr_s *ipv4 = IPv4BUF;
  int ret = ipv4_filter_match(dev, NULL, ipv4, IPFILTER_CHAIN_INPUT);

  if (ret == IPFILTER_TARGET_DROP)
    {
      dev->d_len = 0;
    }

  if (ret == IPFILTER_TARGET_REJECT)
    {
      /* TODO: Support more --reject-with types later. */
#if defined(CONFIG_NET_ICMP) && !defined(CONFIG_NET_ICMP_NO_STACK)
      icmp_reply(dev, ICMP_DEST_UNREACHABLE, ICMP_NET_UNREACH);
#endif
    }

  return ret;
}
#endif

#ifdef CONFIG_NET_IPv6
int ipv6_filter_in(FAR struct net_driver_s *dev)
{
  FAR struct ipv6_hdr_s *ipv6 = IPv6BUF;
  int ret = ipv6_filter_match(dev, NULL, ipv6, IPFILTER_CHAIN_INPUT);

  if (ret == IPFILTER_TARGET_DROP)
    {
      dev->d_len = 0;
    }

  if (ret == IPFILTER_TARGET_REJECT)
    {
      /* TODO: Support more --reject-with types later. */

      icmpv6_reply(dev, ICMPv6_DEST_UNREACHABLE, ICMPv6_ADDR_UNREACH, 0);
    }

  return ret;
}
#endif

/****************************************************************************
 * Name: ipfilter_out
 *
 * Description:
 *   Handling filter on local output.  Do nothing if the output packet
 *   is accepted, set d_len to 0 if the output packet needs to be dropped,
 *   and set d_iob to reject reply if the output packet is rejected.
 *
 * Input Parameters:
 *   dev - The network device that the packet goes to
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The network is locked.  Only IPv4 and IPv6 packets call this function.
 *
 ****************************************************************************/

void ipfilter_out(FAR struct net_driver_s *dev)
{
  if (dev->d_iob == NULL || dev->d_len == 0)
    {
      return;
    }

#ifdef CONFIG_NET_IPv4
  if (IFF_IS_IPv4(dev->d_flags))
    {
      FAR struct ipv4_hdr_s *ipv4 = IPv4BUF;
      int ret = ipv4_filter_match(NULL, dev, ipv4, IPFILTER_CHAIN_OUTPUT);

      if (ret == IPFILTER_TARGET_DROP)
        {
          dev->d_len = 0;
        }

      if (ret == IPFILTER_TARGET_REJECT)
        {
#if defined(CONFIG_NET_ICMP) && !defined(CONFIG_NET_ICMP_NO_STACK)
          icmp_reply(dev, ICMP_DEST_UNREACHABLE, ICMP_NET_UNREACH);
#endif
        }
    }
#endif

#ifdef CONFIG_NET_IPv6
  if (IFF_IS_IPv6(dev->d_flags))
    {
      FAR struct ipv6_hdr_s *ipv6 = IPv6BUF;
      int ret = ipv6_filter_match(NULL, dev, ipv6, IPFILTER_CHAIN_OUTPUT);

      if (ret == IPFILTER_TARGET_DROP)
        {
          dev->d_len = 0;
        }

      if (ret == IPFILTER_TARGET_REJECT)
        {
          icmpv6_reply(dev, ICMPv6_DEST_UNREACHABLE, ICMPv6_ADDR_UNREACH, 0);
        }
    }
#endif
}

/****************************************************************************
 * Name: ipv4_filter_fwd / ipv6_filter_fwd
 *
 * Description:
 *   Handling IPv4/IPv6 filter on forwarding.  Just return the target action,
 *   and relies on the caller to do the actual drop / reject.
 *
 * Input Parameters:
 *   indev     - The network device that the packet comes from
 *   outdev    - The network device that the packet goes to
 *   ipv4/ipv6 - The IPv4/IPv6 header
 *
 * Returned Value:
 *   IPFILTER_TARGET_ACCEPT(0)  - The input packet is accepted
 *   IPFILTER_TARGET_DROP(-1)   - The input packet needs to be dropped
 *   IPFILTER_TARGET_REJECT(-2) - The input packet needs to be rejected
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

#if defined(CONFIG_NET_IPFORWARD) && defined(CONFIG_NET_IPv4)
int ipv4_filter_fwd(FAR struct net_driver_s *indev,
                    FAR struct net_driver_s *outdev,
                    FAR struct ipv4_hdr_s *ipv4)
{
  return ipv4_filter_match(indev, outdev, ipv4, IPFILTER_CHAIN_FORWARD);
}
#endif

#if defined(CONFIG_NET_IPFORWARD) && defined(CONFIG_NET_IPv6)
int ipv6_filter_fwd(FAR struct net_driver_s *indev,
                    FAR struct net_driver_s *outdev,
                    FAR struct ipv6_hdr_s *ipv6)
{
  return ipv6_filter_match(indev, outdev, ipv6, IPFILTER_CHAIN_FORWARD);
}
#endif

#endif /* CONFIG_NET_IPFILTER */
