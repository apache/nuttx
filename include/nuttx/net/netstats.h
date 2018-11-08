/****************************************************************************
 * include/nuttx/net/netstats.h
 *
 *   Copyright (C) 2007-2012 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * This logic was leveraged from uIP which also has a BSD-style license:
 *
 *   Author Adam Dunkels <adam@dunkels.com>
 *   Copyright (c) 2001-2003, Adam Dunkels.
 *   All rights reserved.
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

#ifndef __INCLUDE_NUTTX_NET_NETSTATS_H
#define __INCLUDE_NUTTX_NET_NETSTATS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <nuttx/net/netconfig.h>

#include <nuttx/net/ip.h>
#ifdef CONFIG_NET_TCP
#  include <nuttx/net/tcp.h>
#endif
#ifdef CONFIG_NET_UDP
#  include <nuttx/net/udp.h>
#endif
#ifdef CONFIG_NET_ICMP
#  include <nuttx/net/icmp.h>
#endif
#ifdef CONFIG_NET_ICMPv6
#  include <nuttx/net/icmpv6.h>
#endif
#ifdef CONFIG_NET_IGMP
#  include <nuttx/net/igmp.h>
#endif
#ifdef CONFIG_NET_MLD
#  include <nuttx/net/mld.h>
#endif

#ifdef CONFIG_NET_STATISTICS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

/* The structure holding the networking statistics that are gathered if
 * CONFIG_NET_STATISTICS is defined.
 */

struct net_stats_s
{
#ifdef CONFIG_NET_IPv4
  struct ipv4_stats_s ipv4;     /* IPv4 statistics */
#endif

#ifdef CONFIG_NET_IPv6
  struct ipv6_stats_s ipv6;     /* IPv6 statistics */
#endif

#ifdef CONFIG_NET_ICMP
  struct icmp_stats_s icmp;     /* ICMP statistics */
#endif

#ifdef CONFIG_NET_ICMPv6
  struct icmpv6_stats_s icmpv6; /* ICMPv6 statistics */
#endif

#ifdef CONFIG_NET_IGMP
  struct igmp_stats_s igmp;     /* IGMP statistics */
#endif

#ifdef CONFIG_NET_MLD
  struct mld_stats_s  mld;      /* MLD statistics */
#endif

#ifdef CONFIG_NET_TCP
  struct tcp_stats_s  tcp;      /* TCP statistics */
#endif

#ifdef CONFIG_NET_UDP
  struct udp_stats_s  udp;      /* UDP statistics */
#endif
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* This is the structure in which the statistics are gathered. */

extern struct net_stats_s g_netstats;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* CONFIG_NET_STATISTICS */
#endif /* __INCLUDE_NUTTX_NET_NETSTATS_H */
