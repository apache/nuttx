/****************************************************************************
 * net/netdev/netdev_stats.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

#include <syslog.h>

#include <nuttx/net/netdev.h>
#include <nuttx/net/netstats.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define stats_log(format, ...)   syslog(LOG_NOTICE, format, ##__VA_ARGS__)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: netdev_statistics_log
 *
 * Description:
 *   The actual implementation of the network statistics logging.  Log
 *   network statistics at regular intervals.
 *
 * Input Parameters:
 *   arg - The pointer to the network device
 *
 ****************************************************************************/

#if CONFIG_NETDEV_STATISTICS_LOG_PERIOD > 0
void netdev_statistics_log(FAR void *arg)
{
  FAR struct net_driver_s *dev = arg;
  FAR struct netdev_statistics_s *stats = &dev->d_statistics;

  stats_log("%s:T%" PRIu32 "/%" PRIu32 "(%" PRIu64 "B)" ",R"
#if defined(CONFIG_NET_IPv4) && defined(CONFIG_NET_IPv6)
            "(%" PRIu32 "+%" PRIu32 ")/"
#endif
            "%" PRIu32 "(%" PRIu64 "B)"
#ifdef CONFIG_NET_TCP
            " TCP:T%" PRIu16 ",R%" PRIu16 ",D%" PRIu16
#endif
#ifdef CONFIG_NET_UDP
            " UDP:T%" PRIu16 ",R%" PRIu16 ",D%" PRIu16
#endif
#ifdef CONFIG_NET_ICMP
            " ICMP:T%" PRIu16 ",R%" PRIu16 ",D%" PRIu16
#endif
#ifdef CONFIG_NET_ICMPv6
            " ICMP6:T%" PRIu16 ",R%" PRIu16 ",D%" PRIu16
#endif
            "\n",
            dev->d_ifname,
            stats->tx_done, stats->tx_packets, stats->tx_bytes
#if defined(CONFIG_NET_IPv4) && defined(CONFIG_NET_IPv6)
            , stats->rx_ipv4, stats->rx_ipv6
#endif
            , stats->rx_packets, stats->rx_bytes
#ifdef CONFIG_NET_TCP
            , g_netstats.tcp.sent, g_netstats.tcp.recv, g_netstats.tcp.drop
#endif
#ifdef CONFIG_NET_UDP
            , g_netstats.udp.sent, g_netstats.udp.recv, g_netstats.udp.drop
#endif
#ifdef CONFIG_NET_ICMP
            , g_netstats.icmp.sent, g_netstats.icmp.recv,
              g_netstats.icmp.drop
#endif
#ifdef CONFIG_NET_ICMPv6
            , g_netstats.icmpv6.sent, g_netstats.icmpv6.recv,
              g_netstats.icmpv6.drop
#endif
            );
}
#endif
