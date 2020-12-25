/****************************************************************************
 * net/icmpv6/icmpv6_linkipaddr.c
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

#include "netdev/netdev.h"
#include "icmpv6/icmpv6.h"

#ifdef CONFIG_NET_ICMPv6

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline void
icmpv6_linkipaddr_0(FAR struct net_driver_s *dev, net_ipv6addr_t ipaddr)
{
  ipaddr[0]  = HTONS(0xfe80);
  ipaddr[1]  = 0;
  ipaddr[2]  = 0;
  ipaddr[3]  = 0;
  ipaddr[4]  = 0;
  ipaddr[5]  = 0;
  ipaddr[6]  = 0;
#ifdef CONFIG_NETDEV_IFINDEX
  ipaddr[7]  = HTONS(dev->d_ifindex);
#else
  ipaddr[7]  = 0;
#endif
}

static inline void
icmpv6_linkipaddr_1(FAR const void *mac_, net_ipv6addr_t ipaddr)
{
  FAR const uint8_t *mac = mac_;

  ipaddr[0]  = HTONS(0xfe80);
  ipaddr[1]  = 0;
  ipaddr[2]  = 0;
  ipaddr[3]  = 0;
  ipaddr[4]  = 0;
  ipaddr[5]  = HTONS(0x00ff);
  ipaddr[6]  = HTONS(0xfe00);
  ipaddr[7]  = HTONS(mac[0] << 8);
}

static inline void
icmpv6_linkipaddr_2(FAR const void *mac_, net_ipv6addr_t ipaddr)
{
  FAR const uint8_t *mac = mac_;

  ipaddr[0]  = HTONS(0xfe80);
  ipaddr[1]  = 0;
  ipaddr[2]  = 0;
  ipaddr[3]  = 0;
  ipaddr[4]  = 0;
  ipaddr[5]  = HTONS(0x00ff);
  ipaddr[6]  = HTONS(0xfe00);
  ipaddr[7]  = HTONS(mac[0] << 8 | mac[1]);
}

static inline void
icmpv6_linkipaddr_6(FAR const void *mac_, net_ipv6addr_t ipaddr)
{
  FAR const uint8_t *mac = mac_;

  ipaddr[0]  = HTONS(0xfe80);
  ipaddr[1]  = 0;
  ipaddr[2]  = 0;
  ipaddr[3]  = 0;
  ipaddr[4]  = HTONS(mac[0] << 8 | mac[1]);
  ipaddr[5]  = HTONS(mac[2] << 8 | 0x00ff);
  ipaddr[6]  = HTONS(0x00fe << 8 | mac[3]);
  ipaddr[7]  = HTONS(mac[4] << 8 | mac[5]);
  ipaddr[4] ^= HTONS(0x0200);
}

static inline void
icmpv6_linkipaddr_8(FAR const void *mac_, net_ipv6addr_t ipaddr)
{
  FAR const uint8_t *mac = mac_;

  ipaddr[0]  = HTONS(0xfe80);
  ipaddr[1]  = 0;
  ipaddr[2]  = 0;
  ipaddr[3]  = 0;
  ipaddr[4]  = HTONS(mac[0] << 8 | mac[1]);
  ipaddr[5]  = HTONS(mac[2] << 8 | mac[3]);
  ipaddr[6]  = HTONS(mac[4] << 8 | mac[5]);
  ipaddr[7]  = HTONS(mac[6] << 8 | mac[7]);
  ipaddr[4] ^= HTONS(0x0200);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: icmpv6_linkipaddr
 *
 * Description:
 *   Generate the device link scope ipv6 address as below:
 *    128  112   96   80   64   48   32   16
 *    ---- ---- ---- ---- ---- ---- ---- ----
 *    fe80 0000 0000 0000 0000 00ff fe00 xx00 1B short address IEEE 48b MAC
 *    fe80 0000 0000 0000 0000 00ff fe00 xxxx 2B short address IEEE 48b MAC
 *    fe80 0000 0000 0000 xxxx xxff fexx xxxx 6B normal address IEEE 48b MAC
 *    fe80 0000 0000 0000 xxxx xxxx xxxx xxxx 8B extended address IEEE EUI-64
 *
 * Input Parameters:
 *   dev    - The device driver structure containing the link layer address
 *   ipaddr - Receive the device link scope ipv6 address
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void icmpv6_linkipaddr(FAR struct net_driver_s *dev, net_ipv6addr_t ipaddr)
{
  switch (netdev_lladdrsize(dev))
    {
      case 1:
        icmpv6_linkipaddr_1(&dev->d_mac, ipaddr);
        break;

      case 2:
        icmpv6_linkipaddr_2(&dev->d_mac, ipaddr);
        break;

      case 6:
        icmpv6_linkipaddr_6(&dev->d_mac, ipaddr);
        break;

      case 8:
        icmpv6_linkipaddr_8(&dev->d_mac, ipaddr);
        break;

      default:
        icmpv6_linkipaddr_0(dev, ipaddr);
        break;
    }
}

#endif /* CONFIG_NET_ICMPv6 */
