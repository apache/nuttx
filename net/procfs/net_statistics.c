/****************************************************************************
 * net/procfs/net_statistics.c
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
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

#include <sys/types.h>
#include <stdio.h>
#include <string.h>
#include <debug.h>

#include <nuttx/net/netstats.h>

#include "procfs/procfs.h"

#if !defined(CONFIG_DISABLE_MOUNTPOINT) && defined(CONFIG_FS_PROCFS) && \
    !defined(CONFIG_FS_PROCFS_EXCLUDE_NET) && defined(CONFIG_NET_STATISTICS)

#if defined(CONFIG_NET_IPv4) || defined(CONFIG_NET_IPv6) || \
    defined(CONFIG_NET_TCP) || defined(CONFIG_NET_UDP) || \
    defined(CONFIG_NET_ICMP) || defined(CONFIG_NET_ICMPv6)

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Line generating functions */

static int     netprocfs_header(FAR struct netprocfs_file_s *netfile);
static int     netprocfs_received(FAR struct netprocfs_file_s *netfile);
static int     netprocfs_dropped(FAR struct netprocfs_file_s *netfile);
#ifdef CONFIG_NET_IPv4
static int     netprocfs_ipv4_dropped(FAR struct netprocfs_file_s *netfile);
#endif /* CONFIG_NET_IPv4 */
#ifdef CONFIG_NET_IPv6
static int     netprocfs_ipv6_dropped(FAR struct netprocfs_file_s *netfile);
#endif /* CONFIG_NET_IPv4 */
static int     netprocfs_checksum(FAR struct netprocfs_file_s *netfile);
#ifdef CONFIG_NET_TCP
static int     netprocfs_tcp_dropped_1(FAR struct netprocfs_file_s *netfile);
static int     netprocfs_tcp_dropped_2(FAR struct netprocfs_file_s *netfile);
#endif /* CONFIG_NET_TCP */
static int     netprocfs_prototype(FAR struct netprocfs_file_s *netfile);
static int     netprocfs_sent(FAR struct netprocfs_file_s *netfile);
#ifdef CONFIG_NET_TCP
static int     netprocfs_retransmissions(FAR struct netprocfs_file_s *netfile);
#endif /* CONFIG_NET_TCP */

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Line generating functions */

static const linegen_t g_stat_linegen[] =
{
  netprocfs_header,
  netprocfs_received,
  netprocfs_dropped,

#ifdef CONFIG_NET_IPv4
  netprocfs_ipv4_dropped,
#endif /* CONFIG_NET_IPv4 */

#ifdef CONFIG_NET_IPv6
  netprocfs_ipv6_dropped,
#endif /* CONFIG_NET_IPv4 */

  netprocfs_checksum,

#ifdef CONFIG_NET_TCP
  netprocfs_tcp_dropped_1,
  netprocfs_tcp_dropped_2,
#endif /* CONFIG_NET_TCP */

  netprocfs_prototype,
  netprocfs_sent

#ifdef CONFIG_NET_TCP
  , netprocfs_retransmissions
#endif /* CONFIG_NET_TCP */
};

#define NSTAT_LINES (sizeof(g_stat_linegen) / sizeof(linegen_t))

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: netprocfs_header
 ****************************************************************************/

#ifdef CONFIG_NET_STATISTICS
static int netprocfs_header(FAR struct netprocfs_file_s *netfile)
{
  int len = 0;

  len += snprintf(&netfile->line[len], NET_LINELEN - len, "           ");
#ifdef CONFIG_NET_IPv4
  len += snprintf(&netfile->line[len], NET_LINELEN - len, "  IPv4");
#endif
#ifdef CONFIG_NET_IPv6
  len += snprintf(&netfile->line[len], NET_LINELEN - len, "  IPv6");
#endif
#ifdef CONFIG_NET_TCP
  len += snprintf(&netfile->line[len], NET_LINELEN - len, "   TCP");
#endif
#ifdef CONFIG_NET_UDP
  len += snprintf(&netfile->line[len], NET_LINELEN - len, "   UDP");
#endif
#ifdef CONFIG_NET_ICMP
  len += snprintf(&netfile->line[len], NET_LINELEN - len, "  ICMP");
#endif
#ifdef CONFIG_NET_ICMPv6
  len += snprintf(&netfile->line[len], NET_LINELEN - len, "  ICMPv6");
#endif

  len += snprintf(&netfile->line[len], NET_LINELEN - len, "\n");
  return len;
}
#endif /* CONFIG_NET_STATISTICS */

/****************************************************************************
 * Name: netprocfs_received
 ****************************************************************************/

#ifdef CONFIG_NET_STATISTICS
static int netprocfs_received(FAR struct netprocfs_file_s *netfile)
{
  int len = 0;

  len += snprintf(&netfile->line[len], NET_LINELEN - len, "Received   ");
#ifdef CONFIG_NET_IPv4
  len += snprintf(&netfile->line[len], NET_LINELEN - len, "  %04x",
                  g_netstats.ipv4.recv);
#endif
#ifdef CONFIG_NET_IPv6
  len += snprintf(&netfile->line[len], NET_LINELEN - len, "  %04x",
                  g_netstats.ipv6.recv);
#endif
#ifdef CONFIG_NET_TCP
  len += snprintf(&netfile->line[len], NET_LINELEN - len, "  %04x",
                  g_netstats.tcp.recv);
#endif
#ifdef CONFIG_NET_UDP
  len += snprintf(&netfile->line[len], NET_LINELEN - len, "  %04x",
                  g_netstats.udp.recv);
#endif
#ifdef CONFIG_NET_ICMP
  len += snprintf(&netfile->line[len], NET_LINELEN - len, "  %04x",
                  g_netstats.icmp.recv);
#endif
#ifdef CONFIG_NET_ICMPv6
  len += snprintf(&netfile->line[len], NET_LINELEN - len, "  %04x",
                  g_netstats.icmpv6.recv);
#endif

  len += snprintf(&netfile->line[len], NET_LINELEN - len, "\n");
  return len;
}
#endif /* CONFIG_NET_STATISTICS */

/****************************************************************************
 * Name: netprocfs_dropped
 ****************************************************************************/

#ifdef CONFIG_NET_STATISTICS
static int netprocfs_dropped(FAR struct netprocfs_file_s *netfile)
{
  int len = 0;

  len += snprintf(&netfile->line[len], NET_LINELEN - len, "Dropped    ");
#ifdef CONFIG_NET_IPv4
  len += snprintf(&netfile->line[len], NET_LINELEN - len, "  %04x",
                  g_netstats.ipv4.drop);
#endif
#ifdef CONFIG_NET_IPv6
  len += snprintf(&netfile->line[len], NET_LINELEN - len, "  %04x",
                  g_netstats.ipv6.drop);
#endif
#ifdef CONFIG_NET_TCP
  len += snprintf(&netfile->line[len], NET_LINELEN - len, "  %04x",
                  g_netstats.tcp.drop);
#endif
#ifdef CONFIG_NET_UDP
  len += snprintf(&netfile->line[len], NET_LINELEN - len, "  %04x",
                  g_netstats.udp.drop);
#endif
#ifdef CONFIG_NET_ICMP
  len += snprintf(&netfile->line[len], NET_LINELEN - len, "  %04x",
                  g_netstats.icmp.drop);
#endif
#ifdef CONFIG_NET_ICMPv6
  len += snprintf(&netfile->line[len], NET_LINELEN - len, "  %04x",
                  g_netstats.icmpv6.drop);
#endif

  len += snprintf(&netfile->line[len], NET_LINELEN - len, "\n");
  return len;
}
#endif /* CONFIG_NET_STATISTICS */

/****************************************************************************
 * Name: netprocfs_ipv4_dropped
 ****************************************************************************/

#if defined(CONFIG_NET_STATISTICS) && defined(CONFIG_NET_IPv4)
static int netprocfs_ipv4_dropped(FAR struct netprocfs_file_s *netfile)
{
  return snprintf(netfile->line, NET_LINELEN,
                  "  IPv4        VHL: %04x   Frg: %04x\n",
                  g_netstats.ipv4.vhlerr, g_netstats.ipv4.fragerr);
}
#endif /* CONFIG_NET_STATISTICS && CONFIG_NET_IPv4 */

/****************************************************************************
 * Name: netprocfs_ipv6_dropped
 ****************************************************************************/

#if defined(CONFIG_NET_STATISTICS) && defined(CONFIG_NET_IPv6)
static int netprocfs_ipv6_dropped(FAR struct netprocfs_file_s *netfile)
{
  return snprintf(netfile->line, NET_LINELEN,
                  "  IPv6        VHL: %04x\n",
                  g_netstats.ipv6.vhlerr);
}
#endif /* CONFIG_NET_STATISTICS && CONFIG_NET_IPv6 */

/****************************************************************************
 * Name: netprocfs_checksum
 ****************************************************************************/

#ifdef CONFIG_NET_STATISTICS
static int netprocfs_checksum(FAR struct netprocfs_file_s *netfile)
{
  int len = 0;

  len += snprintf(&netfile->line[len], NET_LINELEN - len, "  Checksum ");
#ifdef CONFIG_NET_IPv4
  len += snprintf(&netfile->line[len], NET_LINELEN - len, "  %04x",
                  g_netstats.ipv4.chkerr);
#endif
#ifdef CONFIG_NET_IPv6
  len += snprintf(&netfile->line[len], NET_LINELEN - len, "  ----");
#endif
#ifdef CONFIG_NET_TCP
  len += snprintf(&netfile->line[len], NET_LINELEN - len, "  %04x",
                  g_netstats.tcp.chkerr);
#endif
#ifdef CONFIG_NET_UDP
  len += snprintf(&netfile->line[len], NET_LINELEN - len, "  %04x",
                  g_netstats.udp.chkerr);
#endif
#ifdef CONFIG_NET_ICMP
  len += snprintf(&netfile->line[len], NET_LINELEN - len, "  ----");
#endif
#ifdef CONFIG_NET_ICMPv6
  len += snprintf(&netfile->line[len], NET_LINELEN - len, "  ----");
#endif

  len += snprintf(&netfile->line[len], NET_LINELEN - len, "\n");
  return len;
}
#endif /* CONFIG_NET_STATISTICS  */

/****************************************************************************
 * Name: netprocfs_tcp_dropped_1
 ****************************************************************************/

#if defined(CONFIG_NET_STATISTICS) && defined(CONFIG_NET_TCP)
static int netprocfs_tcp_dropped_1(FAR struct netprocfs_file_s *netfile)
{
  return snprintf(netfile->line, NET_LINELEN,
                  "  TCP         ACK: %04x   SYN: %04x\n",
                  g_netstats.tcp.ackerr, g_netstats.tcp.syndrop);
}
#endif /* CONFIG_NET_STATISTICS && CONFIG_NET_TCP */

/****************************************************************************
 * Name: netprocfs_tcp_dropped_2
 ****************************************************************************/

#if defined(CONFIG_NET_STATISTICS) && defined(CONFIG_NET_TCP)
static int netprocfs_tcp_dropped_2(FAR struct netprocfs_file_s *netfile)
{
  return snprintf(netfile->line, NET_LINELEN,
                  "              RST: %04x  %04x\n",
                  g_netstats.tcp.rst, g_netstats.tcp.synrst);
}
#endif /* CONFIG_NET_STATISTICS && CONFIG_NET_TCP */

/****************************************************************************
 * Name: netprocfs_prototype
 ****************************************************************************/

#ifdef CONFIG_NET_STATISTICS
static int netprocfs_prototype(FAR struct netprocfs_file_s *netfile)
{
  int len = 0;

  len += snprintf(&netfile->line[len], NET_LINELEN - len, "  Type     ");
#ifdef CONFIG_NET_IPv4
  len += snprintf(&netfile->line[len], NET_LINELEN - len, "  %04x",
                  g_netstats.ipv4.protoerr);
#endif
#ifdef CONFIG_NET_IPv6
  len += snprintf(&netfile->line[len], NET_LINELEN - len, "  %04x",
                  g_netstats.ipv6.protoerr);
#endif
#ifdef CONFIG_NET_TCP
  len += snprintf(&netfile->line[len], NET_LINELEN - len, "  ----");
#endif
#ifdef CONFIG_NET_UDP
  len += snprintf(&netfile->line[len], NET_LINELEN - len, "  ----");
#endif
#ifdef CONFIG_NET_ICMP
  len += snprintf(&netfile->line[len], NET_LINELEN - len, "  %04x",
                  g_netstats.icmp.typeerr);
#endif
#ifdef CONFIG_NET_ICMPv6
  len += snprintf(&netfile->line[len], NET_LINELEN - len, "  %04x",
                  g_netstats.icmpv6.typeerr);
#endif

  len += snprintf(&netfile->line[len], NET_LINELEN - len, "\n");
  return len;
}
#endif /* CONFIG_NET_STATISTICS */

/****************************************************************************
 * Name: netprocfs_sent
 ****************************************************************************/

#ifdef CONFIG_NET_STATISTICS
static int netprocfs_sent(FAR struct netprocfs_file_s *netfile)
{
  int len = 0;

  len += snprintf(&netfile->line[len], NET_LINELEN - len, "Sent       ");
#ifdef CONFIG_NET_IPv4
  len += snprintf(&netfile->line[len], NET_LINELEN - len, "  %04x",
                  g_netstats.ipv4.sent);
#endif
#ifdef CONFIG_NET_IPv6
  len += snprintf(&netfile->line[len], NET_LINELEN - len, "  %04x",
                  g_netstats.ipv6.sent);
#endif
#ifdef CONFIG_NET_TCP
  len += snprintf(&netfile->line[len], NET_LINELEN - len, "  %04x",
                  g_netstats.tcp.sent);
#endif
#ifdef CONFIG_NET_UDP
  len += snprintf(&netfile->line[len], NET_LINELEN - len, "  %04x",
                  g_netstats.udp.sent);
#endif
#ifdef CONFIG_NET_ICMP
  len += snprintf(&netfile->line[len], NET_LINELEN - len, "  %04x",
                  g_netstats.icmp.sent);
#endif
#ifdef CONFIG_NET_ICMPv6
  len += snprintf(&netfile->line[len], NET_LINELEN - len, "  %04x",
                  g_netstats.icmpv6.sent);
#endif

  len += snprintf(&netfile->line[len], NET_LINELEN - len, "\n");
  return len;
}
#endif /* CONFIG_NET_STATISTICS */

/****************************************************************************
 * Name: netprocfs_retransmissions
 ****************************************************************************/

#if defined(CONFIG_NET_STATISTICS) && defined(CONFIG_NET_TCP)
static int netprocfs_retransmissions(FAR struct netprocfs_file_s *netfile)
{
  int len = 0;

  len += snprintf(&netfile->line[len], NET_LINELEN - len, "  Rexmit   ");
#ifdef CONFIG_NET_IPv4
  len += snprintf(&netfile->line[len], NET_LINELEN - len, "  ----");
#endif
#ifdef CONFIG_NET_IPv6
  len += snprintf(&netfile->line[len], NET_LINELEN - len, "  ----");
#endif
  len += snprintf(&netfile->line[len], NET_LINELEN - len, "  %04x",
                  g_netstats.tcp.rexmit);
#ifdef CONFIG_NET_UDP
  len += snprintf(&netfile->line[len], NET_LINELEN - len, "  ----");
#endif
#ifdef CONFIG_NET_ICMP
  len += snprintf(&netfile->line[len], NET_LINELEN - len, "  ----");
#endif
#ifdef CONFIG_NET_ICMPv6
  len += snprintf(&netfile->line[len], NET_LINELEN - len, "  ----");
#endif

  len += snprintf(&netfile->line[len], NET_LINELEN - len, "\n");
  return len;
}
#endif /* CONFIG_NET_STATISTICS && CONFIG_NET_TCP */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: netprocfs_read_netstats
 *
 * Description:
 *   Read and format network layer statistics.
 *
 * Input Parameters:
 *   priv - A reference to the network procfs file structure
 *   buffer - The user-provided buffer into which network status will be
 *            returned.
 *   bulen  - The size in bytes of the user provided buffer.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

ssize_t netprocfs_read_netstats(FAR struct netprocfs_file_s *priv,
                                FAR char *buffer, size_t buflen)
{
  return netprocfs_read_linegen(priv, buffer, buflen, g_stat_linegen, NSTAT_LINES);
}

#else

ssize_t netprocfs_read_netstats(FAR struct netprocfs_file_s *priv,
                                FAR char *buffer, size_t buflen)
{
  return OK;
}

#endif /* CONFIG_NET_IPv4 || CONFIG_NET_IPv6 || CONFIG_NET_TCP || \
        * CONFIG_NET_UDP  || CONFIG_NET_ICMP || CONFIG_NET_ICMPv6 */
#endif /* !CONFIG_DISABLE_MOUNTPOINT && CONFIG_FS_PROCFS &&
        * !CONFIG_FS_PROCFS_EXCLUDE_NET */
