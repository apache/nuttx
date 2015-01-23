/****************************************************************************
 * net/utils/net_chksum.c
 *
 *   Copyright (C) 2007-2010, 2012, 2014-2015 Gregory Nutt. All rights reserved.
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
#ifdef CONFIG_NET

#include <stdint.h>
#include <debug.h>

#include <nuttx/net/netconfig.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/ip.h>
#include <nuttx/net/icmp.h>

#include "utils/utils.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define IPv4BUF   ((struct ipv4_hdr_s *)&dev->d_buf[NET_LL_HDRLEN(dev)])
#define IPv6BUF   ((struct ipv6_hdr_s *)&dev->d_buf[NET_LL_HDRLEN(dev)])
#define ICMPBUF   ((struct icmp_iphdr_s *)&dev->d_buf[NET_LL_HDRLEN(dev)])
#define ICMPv6BUF ((struct icmp_ipv6hdr_s *)&dev->d_buf[NET_LL_HDRLEN(dev)])

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: chksum
 ****************************************************************************/

#if !CONFIG_NET_ARCH_CHKSUM
static uint16_t chksum(uint16_t sum, FAR const uint8_t *data, uint16_t len)
{
  FAR const uint8_t *dataptr;
  FAR const uint8_t *last_byte;
  uint16_t t;

  dataptr = data;
  last_byte = data + len - 1;

  while (dataptr < last_byte)
    {
      /* At least two more bytes */

      t = ((uint16_t)dataptr[0] << 8) + dataptr[1];
      sum += t;
      if (sum < t)
        {
          sum++; /* carry */
        }

      dataptr += 2;
    }

  if (dataptr == last_byte)
    {
      t = (dataptr[0] << 8) + 0;
      sum += t;
      if (sum < t)
        {
          sum++; /* carry */
        }
    }

  /* Return sum in host byte order. */

  return sum;
}
#endif /* CONFIG_NET_ARCH_CHKSUM */

/****************************************************************************
 * Name: ipv4_upperlayer_chksum
 ****************************************************************************/

#if !defined(CONFIG_NET_ARCH_CHKSUM) && defined(CONFIG_NET_IPv4)
static uint16_t ipv4_upperlayer_chksum(FAR struct net_driver_s *dev,
                                       uint8_t proto)
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

  if (upperlen > NET_DEV_MTU(dev))
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
 ****************************************************************************/

#if !defined(CONFIG_NET_ARCH_CHKSUM) && defined(CONFIG_NET_IPv6)
static uint16_t ipv6_upperlayer_chksum(FAR struct net_driver_s *dev,
                                       uint8_t proto)
{
  FAR struct ipv6_hdr_s *ipv6 = IPv6BUF;
  uint16_t upperlen;
  uint16_t sum;

  /* The length reported in the IPv6 header is the length of the payload
   * that follows the header.
   */

  upperlen = ((uint16_t)ipv6->len[0] << 8) + ipv6->len[1];

  /* Verify some minimal assumptions */

  if (upperlen > NET_DEV_MTU(dev))
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
 * Name: net_carry32
 *
 * Description:
 *   Calculate the Internet checksum over a buffer.
 *
 ****************************************************************************/

#if !CONFIG_NET_ARCH_INCR32
static inline void net_carry32(FAR uint8_t *sum, uint16_t op16)
{
  if (sum[2] < (op16 >> 8))
    {
      ++sum[1];
      if (sum[1] == 0)
        {
          ++sum[0];
        }
    }

  if (sum[3] < (op16 & 0xff))
    {
      ++sum[2];
      if (sum[2] == 0)
        {
          ++sum[1];
          if (sum[1] == 0)
            {
              ++sum[0];
            }
        }
    }
}
#endif /* CONFIG_NET_ARCH_INCR32 */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: net_incr32
 *
 * Description:
 *
 *   Carry out a 32-bit addition.
 *
 *   By defining CONFIG_NET_ARCH_INCR32, the architecture can replace
 *   net_incr32 with hardware assisted solutions.
 *
 * Input Parameters:
 *   op32 - A pointer to a 4-byte array representing a 32-bit integer in
 *          network byte order (big endian).  This value may not be word
 *          aligned. The value pointed to by op32 is modified in place
 *
 *   op16 - A 16-bit integer in host byte order.
 *
 ****************************************************************************/

#if !CONFIG_NET_ARCH_INCR32
void net_incr32(FAR uint8_t *op32, uint16_t op16)
{
  op32[3] += (op16 & 0xff);
  op32[2] += (op16 >> 8);
  net_carry32(op32, op16);
}
#endif /* CONFIG_NET_ARCH_INCR32 */

/****************************************************************************
 * Name: net_chksum
 *
 * Description:
 *   Calculate the Internet checksum over a buffer.
 *
 *   The Internet checksum is the one's complement of the one's complement
 *   sum of all 16-bit words in the buffer.
 *
 *   See RFC1071.
 *
 *   If CONFIG_NET_ARCH_CHKSUM is defined, then this function must be
 *   provided by architecture-specific logic.
 *
 * Input Parameters:
 *
 *   buf - A pointer to the buffer over which the checksum is to be computed.
 *
 *   len - The length of the buffer over which the checksum is to be computed.
 *
 * Returned Value:
 *   The Internet checksum of the buffer.
 *
 ****************************************************************************/

#if !CONFIG_NET_ARCH_CHKSUM
uint16_t net_chksum(FAR uint16_t *data, uint16_t len)
{
  return htons(chksum(0, (uint8_t *)data, len));
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

#if !CONFIG_NET_ARCH_CHKSUM
#ifdef CONFIG_NET_IPv4
uint16_t tcp_ipv4_chksum(FAR struct net_driver_s *dev)
{
  return ipv4_upperlayer_chksum(dev, IP_PROTO_TCP);
}
#endif /* CONFIG_NET_IPv4 */

#ifdef CONFIG_NET_IPv6
uint16_t tcp_ipv6_chksum(FAR struct net_driver_s *dev)
{
  return ipv6_upperlayer_chksum(dev, IP_PROTO_TCP);
}
#endif /* CONFIG_NET_IPv6 */
#endif /* !CONFIG_NET_ARCH_CHKSUM */

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

/****************************************************************************
 * Name: udp_ipv4_chksum
 *
 * Description:
 *   Calculate the UDP/IPv4 checksum of the packet in d_buf and d_appdata.
 *
 ****************************************************************************/

#if defined(CONFIG_NET_UDP_CHECKSUMS) && defined(CONFIG_NET_IPv4)
uint16_t udp_ipv4_chksum(FAR struct net_driver_s *dev)
{
  return ipv4_upperlayer_chksum(dev, IP_PROTO_UDP);
}
#endif

/****************************************************************************
 * Name: udp_ipv6_chksum
 *
 * Description:
 *   Calculate the UDP/IPv6 checksum of the packet in d_buf and d_appdata.
 *
 ****************************************************************************/

#if defined(CONFIG_NET_UDP_CHECKSUMS) && defined(CONFIG_NET_IPv6)
uint16_t udp_ipv6_chksum(FAR struct net_driver_s *dev)
{
  return ipv6_upperlayer_chksum(dev, IP_PROTO_UDP);
}
#endif

/****************************************************************************
 * Name: icmp_chksum
 *
 * Description:
 *   Calculate the checksum of the ICMP message
 *
 ****************************************************************************/

#if defined(CONFIG_NET_ICMP) && defined(CONFIG_NET_ICMP_PING)
uint16_t icmp_chksum(FAR struct net_driver_s *dev, int len)
{
  FAR struct icmp_iphdr_s *icmp = ICMPBUF;
  return net_chksum((uint16_t*)&icmp->type, len);
}
#endif /* CONFIG_NET_ICMP && CONFIG_NET_ICMP_PING */

/****************************************************************************
 * Name: icmpv6_chksum
 *
 * Description:
 *   Calculate the checksum of the ICMPv6 message
 *
 ****************************************************************************/

#ifdef CONFIG_NET_ICMPv6
uint16_t icmpv6_chksum(FAR struct net_driver_s *dev)
{
  return ipv6_upperlayer_chksum(dev, IP_PROTO_ICMP6);
}
#endif

#endif /* CONFIG_NET */
