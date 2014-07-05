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

#ifdef CONFIG_NET_TCP
#  include <nuttx/net/tcp.h>
#endif
#ifdef CONFIG_NET_UDP
#  include <nuttx/net/udp.h>
#endif
#ifdef CONFIG_NET_ICMP
#  include <nuttx/net/icmp.h>
#endif
#ifdef CONFIG_NET_IGMP
#  include <nuttx/net/igmp.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

/* The structure holding the uIP statistics that are gathered if
 * CONFIG_NET_STATISTICS is defined.
 */

#ifdef CONFIG_NET_STATISTICS
struct ip_stats_s
{
  net_stats_t drop;       /* Number of dropped packets at the IP layer */
  net_stats_t recv;       /* Number of received packets at the IP layer */
  net_stats_t sent;       /* Number of sent packets at the IP layer */
  net_stats_t vhlerr;     /* Number of packets dropped due to wrong
                             IP version or header length */
  net_stats_t hblenerr;   /* Number of packets dropped due to wrong
                             IP length, high byte */
  net_stats_t lblenerr;   /* Number of packets dropped due to wrong
                             IP length, low byte */
  net_stats_t fragerr;    /* Number of packets dropped since they
                             were IP fragments */
  net_stats_t chkerr;     /* Number of packets dropped due to IP
                             checksum errors */
  net_stats_t protoerr;   /* Number of packets dropped since they
                             were neither ICMP, UDP nor TCP */
};

struct net_stats_s
{
  struct ip_stats_s   ip;   /* IP statistics */

#ifdef CONFIG_NET_ICMP
  struct icmp_stats_s icmp; /* ICMP statistics */
#endif

#ifdef CONFIG_NET_IGMP
  struct igmp_stats_s igmp; /* IGMP statistics */
#endif

#ifdef CONFIG_NET_TCP
  struct tcp_stats_s  tcp;  /* TCP statistics */
#endif

#ifdef CONFIG_NET_UDP
  struct udp_stats_s  udp;  /* UDP statistics */
#endif
};
#endif /* CONFIG_NET_STATISTICS */

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* This is the structure in which the statistics are gathered. */

#ifdef CONFIG_NET_STATISTICS
extern struct net_stats_s g_netstats;
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* __INCLUDE_NUTTX_NET_NETSTATS_H */
