/****************************************************************************
 * net/icmpv6/icmpv6_linkipaddr.c
 * Generate the link local ipv6 address
 *
 *   Copyright (C) 2018 Pinecone Inc. All rights reserved.
 *   Author: Xiang Xiao <xiaoxiang@pinecone.net>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote
 *    products derived from this software without specific prior
 *    written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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
 *    128  112  96   80    64   48   32   16
 *    ---- ---- ---- ----  ---- ---- ---- ----
 *    fe80 0000 0000 0000  0000 00ff fe00 xx00 1-byte short address IEEE 48-bit MAC
 *    fe80 0000 0000 0000  0000 00ff fe00 xxxx 2-byte short address IEEE 48-bit MAC
 *    fe80 0000 0000 0000  xxxx xxff fexx xxxx 6-byte normal address IEEE 48-bit MAC
 *    fe80 0000 0000 0000  xxxx xxxx xxxx xxxx 8-byte extended address IEEE EUI-64
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
