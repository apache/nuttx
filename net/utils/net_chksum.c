/****************************************************************************
 * net/utils/net_chksum.c
 *
 *   Copyright (C) 2007-2010, 2012, 2014 Gregory Nutt. All rights reserved.
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

#define BUF ((struct net_iphdr_s *)&dev->d_buf[NET_LL_HDRLEN(dev)])
#define ICMPBUF ((struct icmp_iphdr_s *)&dev->d_buf[NET_LL_HDRLEN(dev)])

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
  uint16_t t;
  const uint8_t *dataptr;
  const uint8_t *last_byte;

  dataptr = data;
  last_byte = data + len - 1;

  while (dataptr < last_byte)
    {
      /* At least two more bytes */

      t = (dataptr[0] << 8) + dataptr[1];
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
 * Name: upper_layer_chksum
 ****************************************************************************/

#if !CONFIG_NET_ARCH_CHKSUM
static uint16_t upper_layer_chksum(FAR struct net_driver_s *dev, uint8_t proto)
{
  FAR struct net_iphdr_s *pbuf = BUF;
  uint16_t upper_layer_len;
  uint16_t sum;

#ifdef CONFIG_NET_IPv6
  upper_layer_len = (((uint16_t)(pbuf->len[0]) << 8) + pbuf->len[1]);
#else /* CONFIG_NET_IPv6 */
  upper_layer_len = (((uint16_t)(pbuf->len[0]) << 8) + pbuf->len[1]) - IP_HDRLEN;
#endif /* CONFIG_NET_IPv6 */

  /* Verify some minimal assumptions */

  if (upper_layer_len > NET_LL_MTU(dev))
    {
      return 0;
    }

  /* First sum pseudo-header. */

  /* IP protocol and length fields. This addition cannot carry. */

  sum = upper_layer_len + proto;

  /* Sum IP source and destination addresses. */

  sum = chksum(sum, (uint8_t *)&pbuf->srcipaddr, 2 * sizeof(net_ipaddr_t));

  /* Sum TCP header and data. */

  sum = chksum(sum, &dev->d_buf[IP_HDRLEN + NET_LL_HDRLEN(dev)], upper_layer_len);

  return (sum == 0) ? 0xffff : htons(sum);
}
#endif /* CONFIG_NET_ARCH_CHKSUM */

/****************************************************************************
 * Name: icmp_6chksum
 ****************************************************************************/

#if !CONFIG_NET_ARCH_CHKSUM
#ifdef CONFIG_NET_IPv6
static uint16_t icmp_6chksum(FAR struct net_driver_s *dev)
{
  return upper_layer_chksum(dev, IP_PROTO_ICMP6);
}
#endif /* CONFIG_NET_IPv6 */
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
 * Name: ip_chksum
 *
 * Description:
 *   Calculate the IP header checksum of the packet header in d_buf.
 *
 *   The IP header checksum is the Internet checksum of the 20 bytes of
 *   the IP header.
 *
 *   If CONFIG_NET_ARCH_CHKSUM is defined, then this function must be
 *   provided by architecture-specific logic.
 *
 * Returned Value:
 *   The IP header checksum of the IP header in the d_buf buffer.
 *
 ****************************************************************************/

#if !CONFIG_NET_ARCH_CHKSUM
uint16_t ip_chksum(FAR struct net_driver_s *dev)
{
  uint16_t sum;

  sum = chksum(0, &dev->d_buf[NET_LL_HDRLEN(dev)], IP_HDRLEN);
  return (sum == 0) ? 0xffff : htons(sum);
}
#endif /* CONFIG_NET_ARCH_CHKSUM */

/****************************************************************************
 * Name: tcp_chksum
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
uint16_t tcp_chksum(FAR struct net_driver_s *dev)
{
  return upper_layer_chksum(dev, IP_PROTO_TCP);
}
#endif /* CONFIG_NET_ARCH_CHKSUM */

/****************************************************************************
 * Name: udp_chksum
 *
 * Description:
 *   Calculate the UDP checksum of the packet in d_buf and d_appdata.
 *
 ****************************************************************************/

#if defined(CONFIG_NET_UDP_CHECKSUMS) && !defined(CONFIG_NET_ARCH_CHKSUM)
uint16_t udp_chksum(FAR struct net_driver_s *dev)
{
  return upper_layer_chksum(dev, IP_PROTO_UDP);
}
#endif /* CONFIG_NET_UDP_CHECKSUMS && !CONFIG_NET_ARCH_CHKSUM */

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
  FAR struct icmp_iphdr_s *picmp = ICMPBUF;
  return net_chksum((uint16_t*)&picmp->type, len);
}
#endif /* CONFIG_NET_ICMP && CONFIG_NET_ICMP_PING */

#endif /* CONFIG_NET */
