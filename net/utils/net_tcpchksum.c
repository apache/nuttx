/****************************************************************************
 * net/utils/net_tcpchksum.c
 *
 *   Copyright (C) 2007-2010, 2012, 2014-2015, 2017 Gregory Nutt.  All
 *     rights reserved.
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
