/****************************************************************************
 * net/icmpv6/icmpv6_rsolicit.c
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

#include <nuttx/net/netdev.h>
#include <nuttx/net/netstats.h>

#include "devif/devif.h"
#include "netdev/netdev.h"
#include "inet/inet.h"
#include "utils/utils.h"
#include "icmpv6/icmpv6.h"

#ifdef CONFIG_NET_ICMPv6_AUTOCONF

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: icmpv6_rsolicit
 *
 * Description:
 *   Set up to send an ICMPv6 Router Solicitation message.  This version
 *   is for a standalone solicitation.  If formats:
 *
 *   - The IPv6 header
 *   - The ICMPv6 Router Solicitation Message
 *
 *   The device IP address should have been set to the link local address
 *   prior to calling this function.
 *
 * Input Parameters:
 *   dev - Reference to a device driver structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void icmpv6_rsolicit(FAR struct net_driver_s *dev)
{
  FAR struct icmpv6_router_solicit_s *sol;
  uint16_t lladdrsize;
  uint16_t l3size;

  /* Length excludes the IPv6 header */

  lladdrsize    = netdev_lladdrsize(dev);
  l3size        = SIZEOF_ICMPV6_ROUTER_SOLICIT_S(lladdrsize);

  ipv6_build_header(IPv6BUF, l3size, IP_PROTO_ICMP6,
                    dev->d_ipv6addr, g_ipv6_allrouters, 255, 0);

  /* Set up the ICMPv6 Router Solicitation message */

  sol           = IPBUF(IPv6_HDRLEN);
  sol->type     = ICMPV6_ROUTER_SOLICIT;   /* Message type */
  sol->code     = 0;                       /* Message qualifier */
  sol->flags[0] = 0;                       /* flags */
  sol->flags[1] = 0;
  sol->flags[2] = 0;
  sol->flags[3] = 0;

  /* Set up the options */

  sol->opttype  = ICMPv6_OPT_SRCLLADDR;           /* Option type */
  sol->optlen   = ICMPv6_OPT_OCTECTS(lladdrsize); /* Option length in octets */

  /* Copy our link layer address into the message */

  memcpy(sol->srclladdr, &dev->d_mac, lladdrsize);

  /* Update device buffer length */

  iob_update_pktlen(dev->d_iob, IPv6_HDRLEN + l3size);

  /* Calculate the checksum over both the ICMP header and payload */

  sol->chksum   = 0;
  sol->chksum   = ~icmpv6_chksum(dev, IPv6_HDRLEN);

  /* Set the size to the size of the IPv6 header and the payload size */

  dev->d_len    = IPv6_HDRLEN + l3size;

  ninfo("Outgoing ICMPv6 Router Solicitation length: %d\n", dev->d_len);

#ifdef CONFIG_NET_STATISTICS
  g_netstats.icmpv6.sent++;
  g_netstats.ipv6.sent++;
#endif
}

#endif /* CONFIG_NET_ICMPv6_AUTOCONF */
