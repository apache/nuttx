/****************************************************************************
 * net/netdev/netdev_checksum.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

#include <nuttx/net/netdev.h>
#include <nuttx/net/udp.h>
#include <nuttx/net/tcp.h>

#include "netdev/netdev.h"

#ifdef CONFIG_NETDEV_CHECKSUM

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: hardware_chksum_start
 *
 * Description:
 *   get checksum start offset position with iob buffer
 *
 * Input Parameters:
 *   dev       -  The driver structure
 *   iphdrlen  -  ipv4/ipv6 header length
 *
 * Returned Value:
 *   The checksum start offset position
 *
 ****************************************************************************/

static int32_t hardware_chksum_start(FAR struct iob_s *iob,
                                     uint16_t iphdrlen)
{
  int32_t start = 0;

  if (iphdrlen > iob->io_len)
    {
      return -EINVAL;
    }

  if (iob != NULL)
    {
      start = iob->io_offset + iphdrlen;
    }

  return start;
}

/****************************************************************************
 * Name: hardware_chksum_get_proto
 *
 * Description:
 *   get proto with dev.
 *
 * Input Parameters:
 *   dev  -  The driver structure
 *
 * Returned Value:
 *   The proto value
 *
 ****************************************************************************/

static uint8_t hardware_chksum_get_proto(FAR struct net_driver_s *dev)
{
  uint8_t proto;

  if (IFF_IS_IPv6(dev->d_flags))
    {
      FAR struct ipv6_hdr_s *ipv6 = IPv6BUF;
      proto = ipv6->proto;
    }
  else
    {
      FAR struct ipv4_hdr_s *ipv4 = IPv4BUF;
      proto = ipv4->proto;
    }

  return proto;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: netdev_checksum_start
 *
 * Description:
 *   get checksum start offset position with iob, then hardwear can
 *   use to calculate the package payload checksum value.
 *
 * Input Parameters:
 *   dev  -  The driver structure
 *
 * Returned Value:
 *   The checksum start offset position, -EINVAL is mean not need calculate
 *   with hardware
 *
 ****************************************************************************/

int netdev_checksum_start(FAR struct net_driver_s *dev)
{
  int start;

  if (IFF_IS_IPv6(dev->d_flags))
    {
      FAR struct ipv6_hdr_s *ipv6 =
        (FAR struct ipv6_hdr_s *)(IOB_DATA(dev->d_iob));

      if ((ipv6->proto == IP_PROTO_UDP) || (ipv6->proto == IP_PROTO_TCP))
        {
          start = hardware_chksum_start(dev->d_iob, IPv6_HDRLEN);
        }
      else
        {
          return -EINVAL;
        }
    }
  else
    {
      FAR struct ipv4_hdr_s *ipv4 =
        (FAR struct ipv4_hdr_s *)(IOB_DATA(dev->d_iob));

      if ((ipv4->proto == IP_PROTO_UDP) || (ipv4->proto == IP_PROTO_TCP))
        {
          start = hardware_chksum_start(dev->d_iob,
                                        ((ipv4->vhl & IPv4_HLMASK) << 2));
        }
      else
        {
          return -EINVAL;
        }
    }

  return start;
}

/****************************************************************************
 * Name: netdev_checksum_offset
 *
 * Description:
 *   get checksum field offset with tcp/udp header.
 *
 * Input Parameters:
 *   dev  -  The driver structure
 *
 * Returned Value:
 *   The checksum field offset with L4, -EINVAL is mean not need calculate
 *   with hardware
 *
 ****************************************************************************/

int netdev_checksum_offset(FAR struct net_driver_s *dev)
{
  int offset = 0;
  uint8_t proto = hardware_chksum_get_proto(dev);

  if (proto == IP_PROTO_UDP)
    {
      offset = offsetof(struct udp_hdr_s, udpchksum);
    }
  else if (proto == IP_PROTO_TCP)
    {
      offset = offsetof(struct tcp_hdr_s, tcpchksum);
    }
  else
    {
      return -EINVAL;
    }

  return offset;
}

/****************************************************************************
 * Name: netdev_upperlayer_header_checksum
 *
 * Description:
 *   get upperlayer header checksum with tcp/udp header.
 *
 * Input Parameters:
 *   dev  -  The driver structure
 *
 * Returned Value:
 *   The upperlayer header checksum
 *
 ****************************************************************************/

uint16_t netdev_upperlayer_header_checksum(FAR struct net_driver_s *dev)
{
  if (IFF_IS_IPv6(dev->d_flags))
    {
      FAR struct ipv6_hdr_s *ipv6 = IPv6BUF;

      return HTONS(ipv6_upperlayer_header_chksum(dev,
                                                 ipv6->proto,
                                                 IPv6_HDRLEN));
    }
  else
    {
      FAR struct ipv4_hdr_s *ipv4 = IPv4BUF;

      return HTONS(ipv4_upperlayer_header_chksum(dev,
                                                 ipv4->proto));
    }

  return 0;
}

#endif /* CONFIG_NETDEV_CHECKSUM */
