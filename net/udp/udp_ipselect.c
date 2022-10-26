/****************************************************************************
 * net/udp/udp_ipselect.c
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
#ifdef CONFIG_NET

#include <stdint.h>
#include <debug.h>

#include <net/if.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/ip.h>
#include <nuttx/net/udp.h>

#include "udp/udp.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: udp_ipv4_select
 *
 * Description:
 *   Configure to send or receive an UDP IPv4 packet
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv4
void udp_ipv4_select(FAR struct net_driver_s *dev)
{
  /* Clear a bit in the d_flags to distinguish this from an IPv6 packet */

  IFF_SET_IPv4(dev->d_flags);

  /* Set the offset to the beginning of the UDP data payload */

  dev->d_appdata = IPBUF(IPv4UDP_HDRLEN);
}
#endif /* CONFIG_NET_IPv4 */

/****************************************************************************
 * Name: udp_ipv6_select
 *
 * Description:
 *   Configure to send or receive an UDP IPv6 packet
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv6
void udp_ipv6_select(FAR struct net_driver_s *dev)
{
  /* Set a bit in the d_flags to distinguish this from an IPv6 packet */

  IFF_SET_IPv6(dev->d_flags);

  /* Set the offset to the beginning of the UDP data payload */

  dev->d_appdata = IPBUF(IPv6UDP_HDRLEN);
}
#endif /* CONFIG_NET_IPv6 */

#endif /* CONFIG_NET */
