/****************************************************************************
 * net/icmpv6/icmpv6_solicit.c
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

#include <nuttx/net/net.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/netstats.h>

#include "devif/devif.h"
#include "netdev/netdev.h"
#include "utils/utils.h"
#include "icmpv6/icmpv6.h"

#ifdef CONFIG_NET_ICMPv6

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define IPv6BUF  ((struct ipv6_hdr_s *)&dev->d_buf[NET_LL_HDRLEN(dev)])
#define ICMPv6SOLICIT \
  ((struct icmpv6_neighbor_solicit_s *)&dev->d_buf[NET_LL_HDRLEN(dev) + IPv6_HDRLEN])

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* First 6 hwords of the multi-cast address in network byte order */

static const uint16_t g_icmpv_mcastaddr[6] =
{
  HTONS(0xff02), HTONS(0x0000), HTONS(0x0000), HTONS(0x0000),
  HTONS(0x0000), HTONS(0x0001)
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: icmpv6_solicit
 *
 * Description:
 *   Set up to send an ICMPv6 Neighbor Solicitation message.  This version
 *   is for a standalone solicitation.  If formats:
 *
 *   - The IPv6 header
 *   - The ICMPv6 Neighbor Solicitation Message
 *
 * Input Parameters:
 *   dev - Reference to a device driver structure
 *   ipaddr - IP address of Neighbor to be solicited
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void icmpv6_solicit(FAR struct net_driver_s *dev,
                    FAR const net_ipv6addr_t ipaddr)
{
  FAR struct ipv6_hdr_s *ipv6;
  FAR struct icmpv6_neighbor_solicit_s *sol;
  uint16_t lladdrsize;
  uint16_t l3size;

  /* Set up the IPv6 header (most is probably already in place) */

  ipv6          = IPv6BUF;
  ipv6->vtc     = 0x60;                    /* Version/traffic class (MS) */
  ipv6->tcf     = 0;                       /* Traffic class (LS)/Flow label (MS) */
  ipv6->flow    = 0;                       /* Flow label (LS) */

  /* Length excludes the IPv6 header */

  lladdrsize    = netdev_lladdrsize(dev);
  l3size        = SIZEOF_ICMPV6_NEIGHBOR_SOLICIT_S(lladdrsize);
  ipv6->len[0]  = (l3size >> 8);
  ipv6->len[1]  = (l3size & 0xff);

  ipv6->proto   = IP_PROTO_ICMP6;          /* Next header */
  ipv6->ttl     = 255;                     /* Hop limit */

  /* Set the multicast destination IP address */

  memcpy(ipv6->destipaddr, g_icmpv_mcastaddr, 6*sizeof(uint16_t));
  ipv6->destipaddr[6] = ipaddr[6] | HTONS(0xff00);
  ipv6->destipaddr[7] = ipaddr[7];

  /* Add out IPv6 address as the source address */

  net_ipv6addr_copy(ipv6->srcipaddr, dev->d_ipv6addr);

  /* Set up the ICMPv6 Neighbor Solicitation message */

  sol           = ICMPv6SOLICIT;
  sol->type     = ICMPv6_NEIGHBOR_SOLICIT; /* Message type */
  sol->code     = 0;                       /* Message qualifier */
  sol->flags[0] = 0;                       /* flags */
  sol->flags[1] = 0;
  sol->flags[2] = 0;
  sol->flags[3] = 0;

  /* Copy the target address into the Neighbor Solicitation message */

  net_ipv6addr_copy(sol->tgtaddr, ipaddr);

  /* Set up the options */

  sol->opttype  = ICMPv6_OPT_SRCLLADDR;           /* Option type */
  sol->optlen   = ICMPv6_OPT_OCTECTS(lladdrsize); /* Option length in octets */

  /* Copy our link layer address into the message */

  memcpy(sol->srclladdr, &dev->d_mac, lladdrsize);

  /* Calculate the checksum over both the ICMP header and payload */

  sol->chksum   = 0;
  sol->chksum   = ~icmpv6_chksum(dev, IPv6_HDRLEN);

  /* Set the size to the size of the IPv6 header and the payload size */

  dev->d_len    = IPv6_HDRLEN + l3size;

  ninfo("Outgoing ICMPv6 Neighbor Solicitation length: %d (%d)\n",
          dev->d_len, (ipv6->len[0] << 8) | ipv6->len[1]);

#ifdef CONFIG_NET_STATISTICS
  g_netstats.icmpv6.sent++;
  g_netstats.ipv6.sent++;
#endif
}

#endif /* CONFIG_NET_ICMPv6 */
