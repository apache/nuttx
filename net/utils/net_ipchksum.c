/****************************************************************************
 * net/utils/net_ipchksum.c
 *
 *   Copyright (C) 2007-2010, 2012, 2014-2015, 2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <nuttx/net/netconfig.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/ip.h>

#include "utils/utils.h"

#ifdef CONFIG_NET

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define IPv4BUF   ((struct ipv4_hdr_s *)&dev->d_buf[NET_LL_HDRLEN(dev)])
#define IPv6BUF   ((struct ipv6_hdr_s *)&dev->d_buf[NET_LL_HDRLEN(dev)])

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ipv4_upperlayer_chksum
 *
 * Description:
 *   Perform the checksum calcaultion over the IPv4, protocol headers, and
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

#if !defined(CONFIG_NET_ARCH_CHKSUM) && defined(CONFIG_NET_IPv4)
uint16_t ipv4_upperlayer_chksum(FAR struct net_driver_s *dev, uint8_t proto)
{
  FAR struct ipv4_hdr_s *ipv4 = IPv4BUF;
  uint16_t upperlen;
  uint16_t sum;

  /* The length reported in the IPv4 header is the length of both the IPv4
   * header and the payload that follows the header.  We need to subtract
   * the size of the IPv4 header to get the size of the payload.
   */

  upperlen = (((uint16_t)(ipv4->len[0]) << 8) + ipv4->len[1]) - IPv4_HDRLEN;

  /* Verify some minimal assumptions */

  if (upperlen > NETDEV_PKTSIZE(dev))
    {
      return 0;
    }

  /* First sum pseudo-header. */
  /* IP protocol and length fields. This addition cannot carry. */

  sum = upperlen + proto;

  /* Sum IP source and destination addresses. */

  sum = chksum(sum, (FAR uint8_t *)&ipv4->srcipaddr, 2 * sizeof(in_addr_t));

  /* Sum IP payload data. */

  sum = chksum(sum, &dev->d_buf[IPv4_HDRLEN + NET_LL_HDRLEN(dev)], upperlen);
  return (sum == 0) ? 0xffff : htons(sum);
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
 *
 * Returned Value:
 *   The calculated checksum
 *
 ****************************************************************************/

#if !defined(CONFIG_NET_ARCH_CHKSUM) && defined(CONFIG_NET_IPv6)
uint16_t ipv6_upperlayer_chksum(FAR struct net_driver_s *dev, uint8_t proto)
{
  FAR struct ipv6_hdr_s *ipv6 = IPv6BUF;
  uint16_t upperlen;
  uint16_t sum;

  /* The length reported in the IPv6 header is the length of the payload
   * that follows the header.
   */

  upperlen = ((uint16_t)ipv6->len[0] << 8) + ipv6->len[1];

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

  sum = chksum(sum, (FAR uint8_t *)&ipv6->srcipaddr, 2 * sizeof(net_ipv6addr_t));

  /* Sum IP payload data. */

  sum = chksum(sum, &dev->d_buf[IPv6_HDRLEN + NET_LL_HDRLEN(dev)], upperlen);
  return (sum == 0) ? 0xffff : htons(sum);
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
uint16_t ipv4_chksum(FAR struct net_driver_s *dev)
{
  uint16_t sum;

  sum = chksum(0, &dev->d_buf[NET_LL_HDRLEN(dev)], IPv4_HDRLEN);
  return (sum == 0) ? 0xffff : htons(sum);
}
#endif /* CONFIG_NET_ARCH_CHKSUM */

#endif /* CONFIG_NET */
