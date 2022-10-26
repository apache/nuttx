/****************************************************************************
 * net/utils/net_icmpchksum.c
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

#include <nuttx/net/netdev.h>
#include <nuttx/net/icmp.h>

#include "icmp/icmp.h"
#include "utils/utils.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: icmp_chksum
 *
 * Description:
 *   Calculate the checksum of the ICMP message
 *
 ****************************************************************************/

#ifdef CONFIG_NET_ICMP
uint16_t icmp_chksum(FAR struct net_driver_s *dev, int len)
{
  FAR struct ipv4_hdr_s *ipv4 = IPv4BUF;
  FAR struct icmp_hdr_s *icmp;
  uint16_t iphdrlen;

  /* Get the IP header length (accounting for possible options). */

  iphdrlen = (ipv4->vhl & IPv4_HLMASK) << 2;

  /* The ICMP header immediately follows the IP header */

  icmp = IPBUF(iphdrlen);
  return net_chksum((FAR uint16_t *)&icmp->type, len);
}
#endif /* CONFIG_NET_ICMP */

/****************************************************************************
 * Name: icmpv6_chksum
 *
 * Description:
 *   Calculate the checksum of the ICMPv6 message
 *
 ****************************************************************************/

#ifdef CONFIG_NET_ICMPv6
uint16_t icmpv6_chksum(FAR struct net_driver_s *dev, unsigned int iplen)
{
  return ipv6_upperlayer_chksum(dev, IP_PROTO_ICMP6, iplen);
}
#endif

#endif /* CONFIG_NET */
