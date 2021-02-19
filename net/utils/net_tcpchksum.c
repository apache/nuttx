/****************************************************************************
 * net/utils/net_tcpchksum.c
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
#include <nuttx/net/netdev.h>

#include "utils/utils.h"

#ifdef CONFIG_NET_TCP

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tcp_chksum, tcp_ipv4_chksum, and tcp_ipv6_chksum
 *
 * Description:
 *   Calculate the TCP checksum of the packet in d_buf and d_appdata.
 *
 *   The TCP checksum is the Internet checksum of data contents of the
 *   TCP segment, and a pseudo-header as defined in RFC793.
 *
 *   Note: The d_appdata pointer that points to the packet data may
 *   point anywhere in memory, so it is not possible to simply calculate
 *   the Internet checksum of the contents of the d_buf buffer.
 *
 * Returned Value:
 *   The TCP checksum of the TCP segment in d_buf and pointed to by
 *   d_appdata.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv4
uint16_t tcp_ipv4_chksum(FAR struct net_driver_s *dev)
{
  return ipv4_upperlayer_chksum(dev, IP_PROTO_TCP);
}
#endif /* CONFIG_NET_IPv4 */

#ifdef CONFIG_NET_IPv6
uint16_t tcp_ipv6_chksum(FAR struct net_driver_s *dev)
{
  return ipv6_upperlayer_chksum(dev, IP_PROTO_TCP, IPv6_HDRLEN);
}
#endif /* CONFIG_NET_IPv6 */

#if defined(CONFIG_NET_IPv4) && defined(CONFIG_NET_IPv6)
uint16_t tcp_chksum(FAR struct net_driver_s *dev)
{
  if (IFF_IS_IPv6(dev->d_flags))
    {
      return tcp_ipv6_chksum(dev);
    }
  else
    {
      return tcp_ipv4_chksum(dev);
    }
}
#endif

#endif /* CONFIG_NET_TCP */
