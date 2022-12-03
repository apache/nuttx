/****************************************************************************
 * net/utils/net_ipchksum.c
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

#include <assert.h>

#include <nuttx/net/netdev.h>
#include <nuttx/net/ip.h>

#include "utils/utils.h"

#ifdef CONFIG_NET

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ipv4_upperlayer_chksum
 *
 * Description:
 *   Perform the checksum calculation over the IPv4, protocol headers, and
 *   data payload as necessary.
 *
 * Input Parameters:
 *   dev   - The network driver instance.  The packet data is in the d_buf
 *           of the device.
 *   proto - The protocol being supported
 *
 * Returned Value:
 *   The calculated checksum
 *
 ****************************************************************************/

#if !defined(CONFIG_NET_ARCH_CHKSUM) && \
    defined(CONFIG_NET_IPv4) && defined(CONFIG_MM_IOB)
uint16_t ipv4_upperlayer_chksum(FAR struct net_driver_s *dev, uint8_t proto)
{
  FAR struct ipv4_hdr_s *ipv4 = IPv4BUF;
  uint16_t upperlen;
  uint16_t iphdrlen;
  uint16_t sum;

  /* Get the IP header length (accounting for possible options). */

  iphdrlen = (ipv4->vhl & IPv4_HLMASK) << 2;

  /* The length reported in the IPv4 header is the length of both the IPv4
   * header and the payload that follows the header.  We need to subtract
   * the size of the IPv4 header to get the size of the payload.
   */

  upperlen = (((uint16_t)(ipv4->len[0]) << 8) + ipv4->len[1]) - iphdrlen;

  /* Verify some minimal assumptions */

  if (upperlen > NETDEV_PKTSIZE(dev))
    {
      return 0;
    }

  /* First sum pseudo-header.
   *
   * IP protocol and length fields. This addition cannot carry.
   */

  sum = upperlen + proto;

  /* Sum IP source and destination addresses. */

  sum = chksum(sum, (FAR uint8_t *)&ipv4->srcipaddr, 2 * sizeof(in_addr_t));

  /* Sum IP payload data. */

  sum = chksum_iob(sum, dev->d_iob, iphdrlen);

  return (sum == 0) ? 0xffff : HTONS(sum);
}
#endif /* CONFIG_NET_ARCH_CHKSUM */

/****************************************************************************
 * Name: ipv6_upperlayer_chksum
 *
 * Description:
 *   Perform the checksum calculation over the IPv6, protocol headers, and
 *   data payload as necessary.
 *
 * Input Parameters:
 *   dev   - The network driver instance.  The packet data is in the d_buf
 *           of the device.
 *   proto - The protocol being supported
 *   iplen - The size of the IPv6 header.  This may be larger than
 *           IPv6_HDRLEN the IPv6 header if IPv6 extension headers are
 *           present.
 *
 * Returned Value:
 *   The calculated checksum
 *
 ****************************************************************************/

#if !defined(CONFIG_NET_ARCH_CHKSUM) && \
    defined(CONFIG_NET_IPv6) && defined(CONFIG_MM_IOB)
uint16_t ipv6_upperlayer_chksum(FAR struct net_driver_s *dev,
                                uint8_t proto, unsigned int iplen)
{
  FAR struct ipv6_hdr_s *ipv6 = IPv6BUF;
  uint16_t upperlen;
  uint16_t sum;

  DEBUGASSERT(dev != NULL && iplen >= IPv6_HDRLEN);

  /* The length reported in the IPv6 header is the length of the payload
   * that follows the header.  If extension heders are present, then this
   * size includes the size of the IPv6 extension headers.
   */

  upperlen = ((uint16_t)ipv6->len[0] << 8) + ipv6->len[1];

  /* Adjust for the presence of any extension headers */

  upperlen -= (iplen - IPv6_HDRLEN);

  /* Verify some minimal assumptions */

  if (upperlen > NETDEV_PKTSIZE(dev))
    {
      return 0;
    }

  /* The checksum is calculated starting with a pseudo-header of IPv6 header
   * fields according to the IPv6 standard, which consists of the source
   * and destination addresses, the packet length and the next header field.
   */

  sum = upperlen + proto;

  /* Sum IP source and destination addresses. */

  sum = chksum(sum, (FAR uint8_t *)&ipv6->srcipaddr,
               2 * sizeof(net_ipv6addr_t));

  /* Sum IP payload data. */

  sum = chksum_iob(sum, dev->d_iob, iplen);

  return (sum == 0) ? 0xffff : HTONS(sum);
}
#endif /* CONFIG_NET_ARCH_CHKSUM */

/****************************************************************************
 * Name: ipv4_chksum
 *
 * Description:
 *   Calculate the IPv4 header checksum of the packet header in d_buf.
 *
 *   The IPv4 header checksum is the Internet checksum of the 20 bytes of
 *   the IPv4 header.
 *
 *   If CONFIG_NET_ARCH_CHKSUM is defined, then this function must be
 *   provided by architecture-specific logic.
 *
 * Returned Value:
 *   The IPv4 header checksum of the IPv4 header in the d_buf buffer.
 *
 ****************************************************************************/

#if defined(CONFIG_NET_IPv4) && !defined(CONFIG_NET_ARCH_CHKSUM)
uint16_t ipv4_chksum(FAR struct ipv4_hdr_s *ipv4)
{
  uint16_t iphdrlen;
  uint16_t sum;

  /* Get the IP header length (accounting for possible options). */

  iphdrlen = (ipv4->vhl & IPv4_HLMASK) << 2;

  sum = chksum(0, (FAR const uint8_t *)ipv4, iphdrlen);
  return (sum == 0) ? 0xffff : HTONS(sum);
}
#endif /* CONFIG_NET_ARCH_CHKSUM */

#endif /* CONFIG_NET */
