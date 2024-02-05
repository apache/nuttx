/****************************************************************************
 * net/icmpv6/icmpv6_initialize.c
 * ICMPv6 initialization logic
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

#include <nuttx/net/ethernet.h>
#include <nuttx/net/netdev.h>

#include "icmpv6/icmpv6.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: icmpv6_devinit
 *
 * Description:
 *   Called when a new network device is registered to configure that device
 *   for ICMPv6 support.
 *
 * Input Parameters:
 *   dev   - The device driver structure to configure.
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

void icmpv6_devinit(FAR struct net_driver_s *dev)
{
  ninfo("ICMPv6 initializing dev %p\n", dev);

#ifdef CONFIG_NET_ETHERNET
#  ifdef CONFIG_NET_ICMPv6_AUTOCONF
  /* Add the IPv6 all link-local nodes Ethernet address.  This is the
   * address that we expect to receive ICMPv6 Router Advertisement
   * packets.
   */

  if (dev->d_addmac != NULL)
    {
      dev->d_addmac(dev, g_ipv6_ethallnodes.ether_addr_octet);
    }
#  endif /* CONFIG_NET_ICMPv6_AUTOCONF */

#  ifdef CONFIG_NET_ICMPv6_ROUTER
  /* Add the IPv6 all link-local routers Ethernet address.  This is the
   * address that we expect to receive ICMPv6 Router Solicitation
   * packets.
   */

  if (dev->d_addmac != NULL)
    {
      dev->d_addmac(dev, g_ipv6_ethallrouters.ether_addr_octet);
    }
#  endif /* CONFIG_NET_ICMPv6_ROUTER */
#endif   /* CONFIG_NET_ETHERNET */
}
