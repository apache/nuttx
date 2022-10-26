/****************************************************************************
 * net/icmpv6/icmpv6_reply.c
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

#include <sys/types.h>
#include <sys/socket.h>
#include <stdint.h>
#include <string.h>
#include <debug.h>

#include <netinet/in.h>
#include <net/if.h>
#include <arpa/inet.h>

#include <nuttx/net/net.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/ip.h>
#include <nuttx/net/icmpv6.h>

#include "utils/utils.h"
#include "netdev/netdev.h"
#include "devif/devif.h"
#include "inet/inet.h"
#include "icmpv6/icmpv6.h"

#ifdef CONFIG_NET_ICMPv6

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The latest drafts declared increase in minimal mtu up to 1280. */

#define ICMPv6_MINMTULEN  1280

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: icmp_reply
 *
 * Description:
 *   Send an ICMPv6 message in response to a situation
 *   RFC 1122: 3.2.2 MUST send at least the IP header and 8 bytes of header.
 *       MAY send more (we do).
 *       MUST NOT change this header information.
 *       MUST NOT reply to a multicast/broadcast IP address.
 *       MUST NOT reply to a multicast/broadcast MAC address.
 *       MUST reply to only the first fragment.
 *
 * Input Parameters:
 *   dev   - The device driver structure containing the received packet
 *   type  - ICMPv6 Message Type, eg. ICMPv6_DEST_UNREACHABLE
 *   code  - ICMPv6 Message Code, eg. ICMPv6_PORT_UNREACH
 *   data  - Additional 32-bit parameter in the ICMPv6 header
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void icmpv6_reply(FAR struct net_driver_s *dev, int type, int code, int data)
{
  int ipicmplen = IPv6_HDRLEN + sizeof(struct icmpv6_hdr_s);
  FAR struct ipv6_hdr_s *ipv6 = IPv6BUF;
  FAR struct icmpv6_hdr_s *icmpv6 = (FAR struct icmpv6_hdr_s *)(ipv6 + 1);
  uint16_t datalen;

  if (net_ipv6addr_cmp(ipv6->destipaddr, g_ipv6_unspecaddr)
#  ifdef CONFIG_NET_BROADCAST
      || net_is_addr_mcast(ipv6->destipaddr)
#  endif /* CONFIG_NET_BROADCAST */
     )
    {
      dev->d_len = 0;
      return;
    }

  /* Get the data size of the packet. */

  datalen = (ipv6->len[0] << 8) + ipv6->len[1];

  /* RFC says return as much as we can without exceeding 1280 bytes. */

  if (datalen > ICMPv6_MINMTULEN - ipicmplen)
    {
      datalen = ICMPv6_MINMTULEN - ipicmplen;
    }

  dev->d_len = ipicmplen + datalen;

  /* Copy fields from original packet */

  memmove(icmpv6 + 1, ipv6, datalen);

  /* Set up the IPv6 header (most is probably already in place) */

  ipv6->vtc      = 0x60;               /* Version/traffic class (MS) */
  ipv6->tcf      = 0;                  /* Traffic class(LS)/Flow label(MS) */
  ipv6->flow     = 0;                  /* Flow label (LS) */
  ipv6->len[0]   = (dev->d_len >> 8);  /* Length excludes the IPv6 header */
  ipv6->len[1]   = (dev->d_len & 0xff);
  ipv6->proto    = IP_PROTO_ICMP6;     /* Next header */
  ipv6->ttl      = 255;                /* Hop limit */

  net_ipv6addr_hdrcopy(ipv6->destipaddr, ipv6->srcipaddr);
  net_ipv6addr_hdrcopy(ipv6->srcipaddr, dev->d_ipv6addr);

  /* Initialize the ICMPv6 header */

  icmpv6->type    = type;
  icmpv6->code    = code;
  icmpv6->data[0] = data >> 16;
  icmpv6->data[1] = data & 0xffff;

  /* Calculate the ICMPv6 checksum over the ICMPv6 header and payload. */

  icmpv6->chksum = 0;
  icmpv6->chksum = ~icmpv6_chksum(dev, datalen + sizeof(*icmpv6));
  if (icmpv6->chksum == 0)
    {
      icmpv6->chksum = 0xffff;
    }

  ninfo("Outgoing ICMPv6 packet length: %d (%d)\n",
         dev->d_len, (ipv6->len[0] << 8) | ipv6->len[1]);
}

#endif /* CONFIG_NET_ICMPv6 */
