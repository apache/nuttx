/****************************************************************************
 * net/icmpv6/icmpv6_advertise.c
 * Send an ICMPv6 Neighbor Advertisement
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

#include <nuttx/net/netconfig.h>
#include <nuttx/net/netstats.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/net.h>
#include <nuttx/net/icmpv6.h>

#include "netdev/netdev.h"
#include "utils/utils.h"
#include "icmpv6/icmpv6.h"
#include "inet/inet.h"

#ifdef CONFIG_NET_ICMPv6

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: icmpv6_advertise
 *
 * Description:
 *   Send an ICMPv6 Neighbor Advertisement
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

void icmpv6_advertise(FAR struct net_driver_s *dev,
                      const net_ipv6addr_t destipaddr)
{
  FAR struct icmpv6_neighbor_advertise_s *adv;
  uint16_t lladdrsize;
  uint16_t l3size;

  /* Length excludes the IPv6 header */

  lladdrsize   = netdev_lladdrsize(dev);
  l3size       = SIZEOF_ICMPV6_NEIGHBOR_ADVERTISE_S(lladdrsize);

  ipv6_build_header(IPv6BUF, l3size, IP_PROTO_ICMP6,
                    dev->d_ipv6addr, destipaddr, 255);

  /* Set up the ICMPv6 Neighbor Advertise response */

  adv            = IPBUF(IPv6_HDRLEN);
  adv->type      = ICMPv6_NEIGHBOR_ADVERTISE;  /* Message type */
  adv->code      = 0;                          /* Message qualifier */
  adv->flags[0]  = ICMPv6_NADV_FLAG_S |
                   ICMPv6_NADV_FLAG_O; /* Solicited+Override flags. */
  adv->flags[1]  = 0;
  adv->flags[2]  = 0;
  adv->flags[3]  = 0;

  /* Copy the target address into the Neighbor Advertisement message */

  net_ipv6addr_copy(adv->tgtaddr, dev->d_ipv6addr);

  /* Set up the options */

  adv->opttype   = ICMPv6_OPT_TGTLLADDR;           /* Option type */
  adv->optlen    = ICMPv6_OPT_OCTECTS(lladdrsize); /* Option length in octets */

  /* Copy our link layer address into the message */

  memcpy(adv->tgtlladdr, &dev->d_mac, lladdrsize);

  /* Update device buffer length */

  iob_update_pktlen(dev->d_iob, IPv6_HDRLEN + l3size);

  /* Calculate the checksum over both the ICMP header and payload */

  adv->chksum    = 0;
  adv->chksum    = ~icmpv6_chksum(dev, IPv6_HDRLEN);

  /* Set the size to the size of the IPv6 header and the payload size */

  dev->d_len     = IPv6_HDRLEN + l3size;

  ninfo("Outgoing ICMPv6 Neighbor Advertise length: %d\n", dev->d_len);

#ifdef CONFIG_NET_STATISTICS
  g_netstats.icmpv6.sent++;
  g_netstats.ipv6.sent++;
#endif
}

#endif /* CONFIG_NET_ICMPv6 */
