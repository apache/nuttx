/****************************************************************************
 * net/inet/ipv6_build_header.c
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
#include <nuttx/net/ip.h>
#include <nuttx/net/netconfig.h>

#include "inet.h"

#ifdef CONFIG_NET_IPv6

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ipv6_build_header
 *
 * Description:
 *   build IPv6 header
 *
 * Input Parameters:
 *   ipv6         Pointer to IPv6 header's buffer
 *   payload_len  Length of the IPv6 payload(without IPv6 header length)
 *   prot         Type of header immediately following the IPv6 header
 *   src_ip       Source IPv6 address
 *   dst_ip       Destination IPv6 address
 *   ttl          Time to live(IPv4) hop limit(IPv6)
 *
 * Returned Value:
 *   length of IPv6 header
 *
 ****************************************************************************/

uint16_t ipv6_build_header(FAR struct ipv6_hdr_s *ipv6, uint16_t payload_len,
                           uint16_t prot, const net_ipv6addr_t src_ip,
                           const net_ipv6addr_t dst_ip, uint8_t ttl)
{
  /* Set up the IPv6 header */

  ipv6->vtc      = 0x60;                 /* Version/traffic class (MS) */
  ipv6->tcf      = 0;                    /* Traffic class(LS)/Flow label(MS) */
  ipv6->flow     = 0;                    /* Flow label (LS) */
  ipv6->len[0]   = (payload_len >> 8);   /* Length excludes the IPv6 header */
  ipv6->len[1]   = (payload_len & 0xff);
  ipv6->proto    = prot;                 /* Next header */
  ipv6->ttl      = ttl;

  /* It's possible to use srcip to initialize destip */

  net_ipv6addr_hdrcopy(ipv6->destipaddr, dst_ip);
  net_ipv6addr_hdrcopy(ipv6->srcipaddr, src_ip);

  ninfo("IPv6 Payload length: %d\n", payload_len);

  return IPv6_HDRLEN;
}

#endif /* CONFIG_NET_IPv6 */
