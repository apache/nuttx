/****************************************************************************
 * net/inet/ipv4_build_header.c
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
#include "utils/utils.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_NET_IPv4

/* Increasing number used for the IP ID field. */

static uint16_t g_ipid;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ipv4_build_header
 *
 * Description:
 *   build IPv4 header
 *
 * Input Parameters:
 *   ipv4       Pointer to IPv4 header's buffer
 *   total_len  total length of the IPv4 packet
 *   prot       the next level protocol used in IPv4 packet
 *   src_ip     Source IPv4 address
 *   dst_ip     Destination IPv4 address
 *   ttl        Time to live(IPv4)
 *   opt        IPv4 options
 *
 * Returned Value:
 *   length of IPv4 header
 *
 ****************************************************************************/

uint16_t ipv4_build_header(FAR struct ipv4_hdr_s *ipv4, uint16_t total_len,
                           uint16_t prot, FAR const in_addr_t *src_ip,
                           FAR const in_addr_t *dst_ip, uint8_t ttl,
                           FAR struct ipv4_opt_s *opt)
{
  /* Initialize the IP header. */

  ipv4->vhl         = 0x45;   /* orginal initial value like this */
  ipv4->tos         = 0;
  ipv4->len[0]      = (total_len >> 8);
  ipv4->len[1]      = (total_len & 0xff);
  ++g_ipid;
  ipv4->ipid[0]     = g_ipid >> 8;
  ipv4->ipid[1]     = g_ipid & 0xff;
  ipv4->ipoffset[0] = IP_FLAG_DONTFRAG >> 8;
  ipv4->ipoffset[1] = IP_FLAG_DONTFRAG & 0xff;
  ipv4->ttl         = ttl;
  ipv4->proto       = prot;

  /* It's possible to use its own src_ip to initialize its dest_ip */

  net_ipv4addr_hdrcopy(ipv4->destipaddr, dst_ip);
  net_ipv4addr_hdrcopy(ipv4->srcipaddr, src_ip);

  /* if ip has options, build it now */

  if (opt != NULL)
    {
      ipv4->vhl += opt->len >> 2;
      memcpy(ipv4 + 1, opt->data, opt->len);
    }

  /* Calculate IP checksum. */

  ipv4->ipchksum    = 0;
  ipv4->ipchksum    = ~ipv4_chksum(ipv4);

  ninfo("IPv4 Packet: ipid:%d, length: %d\n", g_ipid, total_len);

  return (ipv4->vhl & IPv4_HLMASK) << 2;
}

#endif /* CONFIG_NET_IPv4 */
