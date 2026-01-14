/****************************************************************************
 * net/ipfilter/ipfilter.h
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

#ifndef __NET_IPFILTER_IPFILTER_H
#define __NET_IPFILTER_IPFILTER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <nuttx/compiler.h>
#include <nuttx/net/ip.h>

#ifdef CONFIG_NET_IPFILTER

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define IPFILTER_TARGET_ACCEPT (0)
#define IPFILTER_TARGET_DROP   (-1)
#define IPFILTER_TARGET_REJECT (-2)

/****************************************************************************
 * Public Types
 ****************************************************************************/

enum ipfilter_chain_e
{
  IPFILTER_CHAIN_INPUT   = 0, /* = NF_IP_LOCAL_IN  - 1, Input chain */
  IPFILTER_CHAIN_FORWARD = 1, /* = NF_IP_FORWARD   - 1, Forward chain */
  IPFILTER_CHAIN_OUTPUT  = 2, /* = NF_IP_LOCAL_OUT - 1, Output chain */
  IPFILTER_CHAIN_MAX
};

/* The filter configuration entry */

struct ipfilter_entry_s
{
  FAR struct ipfilter_entry_s *flink;

  FAR struct net_driver_s *indev;
  FAR struct net_driver_s *outdev;

  union /* Matches in host byte order (because it's range) */
  {
    struct
    {
      uint16_t sports[2]; /* Source port range */
      uint16_t dports[2]; /* Destination port range */
    } tcpudp;

    struct
    {
      uint8_t type;       /* Type to match, 0xFF = ALL (Same as Linux) */
    } icmp;
  } match;

  uint8_t proto;          /* Protocol to match, 0 = ALL (Same as Linux) */
  int8_t  target;

  /* Match flags, whether we need to match protocol in detail */

  uint8_t match_tcpudp : 1; /* Match TCP/UDP */
  uint8_t match_icmp   : 1; /* Match ICMP */

  /* Inverse flags */

  uint8_t inv_indev  : 1; /* Inverse input device */
  uint8_t inv_outdev : 1; /* Inverse output device */
  uint8_t inv_proto  : 1; /* Inverse protocol */
  uint8_t inv_srcip  : 1; /* Inverse source IP */
  uint8_t inv_dstip  : 1; /* Inverse destination IP */
  uint8_t inv_sport  : 1; /* Inverse source port */
  uint8_t inv_dport  : 1; /* Inverse destination port */
  uint8_t inv_icmp   : 1; /* Inverse ICMP type */
};

struct ipv4_filter_entry_s
{
  struct ipfilter_entry_s common;

  /* Addresses in network byte order */

  in_addr_t sip;
  in_addr_t dip;
  in_addr_t smsk;
  in_addr_t dmsk;
};

struct ipv6_filter_entry_s
{
  struct ipfilter_entry_s common;

  /* Addresses in network byte order */

  net_ipv6addr_t sip;
  net_ipv6addr_t dip;
  net_ipv6addr_t smsk;
  net_ipv6addr_t dmsk;
};

/****************************************************************************
 * Public Function Prototypes
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

FAR struct ipfilter_entry_s *ipfilter_cfg_alloc(sa_family_t family);

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
                      sa_family_t family, enum ipfilter_chain_e chain);

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

void ipfilter_cfg_clear(sa_family_t family, enum ipfilter_chain_e chain);

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
int ipv4_filter_in(FAR struct net_driver_s *dev);
#endif
#ifdef CONFIG_NET_IPv6
int ipv6_filter_in(FAR struct net_driver_s *dev);
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

void ipfilter_out(FAR struct net_driver_s *dev);

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
                    FAR struct ipv4_hdr_s *ipv4);
#endif
#if defined(CONFIG_NET_IPFORWARD) && defined(CONFIG_NET_IPv6)
int ipv6_filter_fwd(FAR struct net_driver_s *indev,
                    FAR struct net_driver_s *outdev,
                    FAR struct ipv6_hdr_s *ipv6);
#endif

#endif /* CONFIG_NET_IPFILTER */
#endif /* __NET_IPFILTER_IPFILTER_H */
