/****************************************************************************
 * net/ipforward/ipfwd_dropstats.c
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
 *   is returned.  The only error is if the protocol is not recognized.
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
      ipv4_dropstats((FAR struct ipv4_hdr_s *)
                     &fwd->f_iob->io_data[CONFIG_NET_LL_GUARDSIZE]);
    }
#endif
#ifdef CONFIG_NET_IPv6
#ifdef CONFIG_NET_IPv4
  else
#endif
    {
      ipv6_dropstats((FAR struct ipv6_hdr_s *)
                     &fwd->f_iob->io_data[CONFIG_NET_LL_GUARDSIZE]);
    }
#endif
}

#endif /* CONFIG_NET_IPFORWARD && CONFIG_NET_STATISTICS */
