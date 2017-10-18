/****************************************************************************
 * net/ipforward/ipfwd_dropstats.c
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
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

#include <errno.h>

#include <nuttx/mm/iob.h>
#include <nuttx/net/ip.h>
#include <nuttx/net/netstats.h>

#include "ipforward/ipforward.h"

#if defined(CONFIG_NET_IPFORWARD) && defined(CONFIG_NET_STATISTICS)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: proto_dropstats
 *
 * Description:
 *   Update statistics for a dropped L2 protocol packet.
 *
 * Input Parameters:
 *  proto - The protocol from the IPv4 or IPv6 header.
 *
 * Returned Value:
 *   On success, zero (OK) is returned.  Otherwise, a negated errno value
 *   is returned.  The only error is if th protocol is not recognized.
 *
 ****************************************************************************/

static int proto_dropstats(int proto)
{
  switch (proto)
    {
#ifdef CONFIG_NET_TCP
    case IP_PROTO_TCP:
      g_netstats.tcp.drop++;
      break;
#endif

#ifdef CONFIG_NET_UDP
    case IP_PROTO_UDP:
      g_netstats.udp.drop++;
      break;
#endif

#ifdef CONFIG_NET_ICMPv6
    case IP_PROTO_ICMP6:
      g_netstats.icmpv6.drop++;
      break;
#endif

    default:
      return -EPROTONOSUPPORT;
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ipv6_dropstats
 *
 * Description:
 *   Update statistics for a dropped Ipv6 packet.
 *
 * Input Parameters:
 *   ipv6  - A pointer to the IPv6 header in within the IPv6 packet to be
 *           dropped.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv6
void ipv6_dropstats(FAR struct ipv6_hdr_s *ipv6)
{
  int ret;

  ret = proto_dropstats(ipv6->proto);
  if (ret < 0)
    {
      g_netstats.ipv6.protoerr++;
    }

  g_netstats.ipv6.drop++;
}
#endif

/****************************************************************************
 * Name: ipv4_dropstats
 *
 * Description:
 *   Update statistics for a dropped Ipv4 packet.
 *
 * Input Parameters:
 *   ipv4  - A pointer to the IPv4 header in within the IPv4 packet to be
 *           dropped.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv4
void ipv4_dropstats(FAR struct ipv4_hdr_s *ipv4)
{
  int ret;

  ret = proto_dropstats(ipv4->proto);
  if (ret < 0)
    {
      g_netstats.ipv4.protoerr++;
    }

  g_netstats.ipv4.drop++;
}
#endif

/****************************************************************************
 * Name: ipfwd_dropstats
 *
 * Description:
 *   Update statistics for a dropped packet.
 *
 * Input Parameters:
 *   fwd - The forwarding state structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void ipfwd_dropstats(FAR struct forward_s *fwd)
{
#ifdef CONFIG_NET_IPv4
#ifdef CONFIG_NET_IPv6
  if (fwd->f_domain == PF_INET)
#endif
    {
      ipv4_dropstats((FAR struct ipv4_hdr_s *)fwd->f_iob->io_data);
    }
#endif
#ifdef CONFIG_NET_IPv6
#ifdef CONFIG_NET_IPv4
  else
#endif
    {
      ipv6_dropstats((FAR struct ipv6_hdr_s *)fwd->f_iob->io_data);
    }
#endif
}

#endif /* CONFIG_NET_IPFORWARD && CONFIG_NET_STATISTICS */
