/****************************************************************************
 * net/procfs/netdev_statistics.c
 *
 *   Copyright (C) 2015, 2017 Gregory Nutt. All rights reserved.
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

#include <netinet/ether.h>

#include <nuttx/net/netdev.h>
#include <nuttx/net/sixlowpan.h>

#include "netdev/netdev.h"
#include "utils/utils.h"
#include "procfs/procfs.h"

#if !defined(CONFIG_DISABLE_MOUNTPOINT) && defined(CONFIG_FS_PROCFS) && \
    !defined(CONFIG_FS_PROCFS_EXCLUDE_NET)

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int netprocfs_linklayer(FAR struct netprocfs_file_s *netfile);
#ifdef CONFIG_NET_IPv4
static int netprocfs_inet4addresses(FAR struct netprocfs_file_s *netfile);
#endif
#ifdef CONFIG_NET_IPv6
static int netprocfs_inet6address(FAR struct netprocfs_file_s *netfile);
static int netprocfs_inet6draddress(FAR struct netprocfs_file_s *netfile);
#endif
#if !defined(CONFIG_NET_IPv4) && !defined(CONFIG_NET_IPv6)
static int netprocfs_blank_line(FAR struct netprocfs_file_s *netfile);
#endif
#ifdef CONFIG_NETDEV_STATISTICS
static int netprocfs_rxstatistics_header(FAR struct netprocfs_file_s *netfile);
static int netprocfs_rxstatistics(FAR struct netprocfs_file_s *netfile);
static int netprocfs_rxpackets_header(FAR struct netprocfs_file_s *netfile);
static int netprocfs_rxpackets(FAR struct netprocfs_file_s *netfile);
static int netprocfs_txstatistics_header(FAR struct netprocfs_file_s *netfile);
static int netprocfs_txstatistics(FAR struct netprocfs_file_s *netfile);
static int netprocfs_errors(FAR struct netprocfs_file_s *netfile);
#endif /* CONFIG_NETDEV_STATISTICS */

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Line generating functions */

static const linegen_t g_netstat_linegen[] =
{
  netprocfs_linklayer
#ifdef CONFIG_NET_IPv4
  , netprocfs_inet4addresses
#endif
#ifdef CONFIG_NET_IPv6
  , netprocfs_inet6address
  , netprocfs_inet6draddress
#endif
#if !defined(CONFIG_NET_IPv4) && !defined(CONFIG_NET_IPv6)
  , netprocfs_blank_line
#endif

#ifdef CONFIG_NETDEV_STATISTICS
  , netprocfs_rxstatistics_header,
  netprocfs_rxstatistics,
  netprocfs_rxpackets_header,
  netprocfs_rxpackets,
  netprocfs_txstatistics_header,
  netprocfs_txstatistics,
  netprocfs_errors
#endif /* CONFIG_NETDEV_STATISTICS */
};

#define NSTAT_LINES (sizeof(g_netstat_linegen) / sizeof(linegen_t))

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#if defined(CONFIG_NET_6LOWPAN) || defined(CONFIG_NET_IEEE802154)
static int netprocfs_radio_linklayer(FAR struct netprocfs_file_s *netfile,
                                       int len)
{
  FAR struct netdev_varaddr_s *addr;
  FAR struct net_driver_s *dev;

  DEBUGASSERT(netfile != NULL && netfile->dev != NULL);
  dev  = netfile->dev;
  addr = &dev->d_mac.radio;

#ifdef CONFIG_NET_6LOWPAN
  len += snprintf(&netfile->line[len], NET_LINELEN - len,
                  "%s\tLink encap:6LoWPAN HWaddr ",
                  dev->d_ifname);
#else
  len += snprintf(&netfile->line[len], NET_LINELEN - len,
                  "%s\tLink encap:Raw HWaddr ",
                  dev->d_ifname);
#endif

  switch (addr->nv_addrlen)
    {
      default:
      case 0:
        nwarn("WARNING: Bad or undefined node address: %u\n", addr->nv_addrlen);
        len += snprintf(&netfile->line[len], NET_LINELEN - len,
                        "--");
        break;

#ifdef CONFIG_WIRELESS_PKTRADIO
      case 1:
        len += snprintf(&netfile->line[len], NET_LINELEN - len,
                        "%02x", addr->nv_addr[0]);
        break;
#endif

#if defined(CONFIG_WIRELESS_PKTRADIO) || defined(CONFIG_WIRELESS_IEEE802154)
       case 2:
        len += snprintf(&netfile->line[len], NET_LINELEN - len,
                        "%02x:%02x",
                        addr->nv_addr[0], addr->nv_addr[1]);
        break;

      case 8:
        len += snprintf(&netfile->line[len], NET_LINELEN - len,
                        "%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x",
                         addr->nv_addr[0], addr->nv_addr[1],
                         addr->nv_addr[2], addr->nv_addr[3],
                         addr->nv_addr[4], addr->nv_addr[5],
                         addr->nv_addr[6], addr->nv_addr[7]);
        break;
#endif
    }

  return len;
}
#endif

  /****************************************************************************
 * Name: netprocfs_linklayer
 ****************************************************************************/

static int netprocfs_linklayer(FAR struct netprocfs_file_s *netfile)
{
  FAR struct net_driver_s *dev;
  FAR const char *status;
  int len = 0;

  DEBUGASSERT(netfile != NULL && netfile->dev != NULL);
  dev = netfile->dev;

  /* Get the interface status:  RUNNING, UP, or DOWN */

  if ((dev->d_flags & IFF_RUNNING) != 0)
    {
      status = "RUNNING";
    }
  else if ((dev->d_flags & IFF_UP) != 0)
    {
      status = "UP";
    }
  else
    {
      status = "DOWN";
    }

  /* Select the output appropriate for the link type associated with
   * this device.
   */

  switch (dev->d_lltype)
    {
#if defined(CONFIG_NET_ETHERNET) || defined(CONFIG_DRIVERS_IEEE80211)
      case NET_LL_ETHERNET:
      case NET_LL_IEEE80211:
        len += snprintf(&netfile->line[len], NET_LINELEN - len,
                        "%s\tLink encap:Ethernet HWaddr %s",
                        dev->d_ifname, ether_ntoa(&dev->d_mac.ether));
        break;
#endif

#if defined(CONFIG_NET_6LOWPAN) || defined(CONFIG_NET_IEEE802154)
      case NET_LL_IEEE802154:
      case NET_LL_PKTRADIO:
        {
          len += netprocfs_radio_linklayer(netfile, len);
        }
        break;
#endif /* CONFIG_NET_6LOWPAN || CONFIG_NET_IEEE802154 */

#ifdef CONFIG_NET_LOOPBACK
      case NET_LL_LOOPBACK:
        len += snprintf(&netfile->line[len], NET_LINELEN - len,
                        "%s\tLink encap:Local Loopback",
                        dev->d_ifname);
        break;
#endif

#ifdef CONFIG_NET_SLIP
      case NET_LL_SLIP:
        len += snprintf(&netfile->line[len], NET_LINELEN - len,
                        "%s\tLink encap:SLIP", dev->d_ifname);
        break;
#endif

#ifdef CONFIG_NET_PPP
      case NET_LL_PPP:
        len += snprintf(&netfile->line[len], NET_LINELEN - len,
                        "%s\tLink encap:P-t-P", dev->d_ifname);
        break;
#endif

#ifdef CONFIG_NET_TUN
      case NET_LL_TUN:
        len += snprintf(&netfile->line[len], NET_LINELEN - len,
                        "%s\tLink encap:TUN", dev->d_ifname);
        break;
#endif

      default:
        len += snprintf(&netfile->line[len], NET_LINELEN - len,
                        "%s\tLink encap:UNSPEC", dev->d_ifname);
    }

  len += snprintf(&netfile->line[len], NET_LINELEN - len,
                  " at %s\n", status);
  return len;
}

/****************************************************************************
 * Name: netprocfs_inet4addresses
 ****************************************************************************/

#ifdef CONFIG_NET_IPv4
static int netprocfs_inet4addresses(FAR struct netprocfs_file_s *netfile)
{
  FAR struct net_driver_s *dev;
  struct in_addr addr;
  int len = 0;

  DEBUGASSERT(netfile != NULL && netfile->dev != NULL);
  dev = netfile->dev;

  /* Show the IPv4 address */

  addr.s_addr = dev->d_ipaddr;
  len += snprintf(&netfile->line[len], NET_LINELEN - len,
                  "\tinet addr:%s ", inet_ntoa(addr));

  /* Show the IPv4 default router address */

  addr.s_addr = dev->d_draddr;
  len += snprintf(&netfile->line[len], NET_LINELEN - len,
                  "DRaddr:%s ", inet_ntoa(addr));

  /* Show the IPv4 network mask */

  addr.s_addr = dev->d_netmask;
  len += snprintf(&netfile->line[len], NET_LINELEN - len,
#ifdef CONFIG_NET_IPv6
                  "Mask:%s\n",    /* IPv6 addresses will follow */
#else
                  "Mask:%s\n\n",  /* Double space at end of device description */
#endif
                  inet_ntoa(addr));

  return len;
}
#endif

/****************************************************************************
 * Name: netprocfs_inet6address
 ****************************************************************************/

#ifdef CONFIG_NET_IPv6
static int netprocfs_inet6address(FAR struct netprocfs_file_s *netfile)
{
  FAR struct net_driver_s *dev;
  char addrstr[INET6_ADDRSTRLEN];
  uint8_t preflen;
  int len = 0;

  DEBUGASSERT(netfile != NULL && netfile->dev != NULL);
  dev = netfile->dev;

  /* Convert the 128 network mask to a human friendly prefix length */

  preflen = net_ipv6_mask2pref(dev->d_ipv6netmask);

  /* Show the assigned IPv6 address */

  if (inet_ntop(AF_INET6, dev->d_ipv6addr, addrstr, INET6_ADDRSTRLEN))
    {
      len += snprintf(&netfile->line[len], NET_LINELEN - len,
                      "\tinet6 addr: %s/%d\n", addrstr, preflen);
    }

  return len;
}
#endif

/****************************************************************************
 * Name: netprocfs_inet6draddress
 ****************************************************************************/

#ifdef CONFIG_NET_IPv6
static int netprocfs_inet6draddress(FAR struct netprocfs_file_s *netfile)
{
  FAR struct net_driver_s *dev;
  char addrstr[INET6_ADDRSTRLEN];
  uint8_t preflen;
  int len = 0;

  DEBUGASSERT(netfile != NULL && netfile->dev != NULL);
  dev = netfile->dev;

  /* Convert the 128 network mask to a human friendly prefix length */

  preflen = net_ipv6_mask2pref(dev->d_ipv6netmask);

  /* Show the IPv6 default router address */

  if (inet_ntop(AF_INET6, dev->d_ipv6draddr, addrstr, INET6_ADDRSTRLEN))
    {
      len += snprintf(&netfile->line[len], NET_LINELEN - len,
                      "\tinet6 DRaddr: %s/%d\n\n", addrstr, preflen);
    }

  return len;
}
#endif

/****************************************************************************
 * Name: netprocfs_blank_line
 ****************************************************************************/

#if !defined(CONFIG_NET_IPv4) && !defined(CONFIG_NET_IPv6)
static int netprocfs_blank_line(FAR struct netprocfs_file_s *netfile)
{
  netfile->line[0] = '\n';
  netfile->line[1] = '\0';
  return 1;
}
#endif

/****************************************************************************
 * Name: netprocfs_rxstatistics_header
 ****************************************************************************/

#ifdef CONFIG_NETDEV_STATISTICS
static int netprocfs_rxstatistics_header(FAR struct netprocfs_file_s *netfile)
{
  DEBUGASSERT(netfile != NULL);
  return snprintf(netfile->line, NET_LINELEN , "\tRX: %-8s %-8s %-8s\n",
                  "Received", "Fragment", "Errors");
}
#endif /* CONFIG_NETDEV_STATISTICS */

/****************************************************************************
 * Name: netprocfs_rxstatistics
 ****************************************************************************/

#ifdef CONFIG_NETDEV_STATISTICS
static int netprocfs_rxstatistics(FAR struct netprocfs_file_s *netfile)
{
  FAR struct netdev_statistics_s *stats;
  FAR struct net_driver_s *dev;

  DEBUGASSERT(netfile != NULL && netfile->dev != NULL);
  dev = netfile->dev;
  stats = &dev->d_statistics;

  return snprintf(netfile->line, NET_LINELEN, "\t    %08lx %08lx %08lx\n",
                  (unsigned long)stats->rx_packets,
                  (unsigned long)stats->rx_fragments,
                  (unsigned long)stats->rx_errors);
}
#endif /* CONFIG_NETDEV_STATISTICS */

/****************************************************************************
 * Name: netprocfs_rxpackets_header
 ****************************************************************************/

#ifdef CONFIG_NETDEV_STATISTICS
static int netprocfs_rxpackets_header(FAR struct netprocfs_file_s *netfile)
{
  FAR char *fmt;

  DEBUGASSERT(netfile != NULL);

  fmt = "\t    "
#ifdef CONFIG_NET_IPv4
        "%-8s "
#endif
#ifdef CONFIG_NET_IPv6
        "%-8s "
#endif
#ifdef CONFIG_NET_ARP
        "%-8s "
#endif
        "%-8s\n";

  return snprintf(netfile->line, NET_LINELEN, fmt
#ifdef CONFIG_NET_IPv4
        , "IPv4"
#endif
#ifdef CONFIG_NET_IPv6
        , "IPv6"
#endif
#ifdef CONFIG_NET_ARP
        , "ARP"
#endif
        , "Dropped");
}
#endif /* CONFIG_NETDEV_STATISTICS */

/****************************************************************************
 * Name: netprocfs_rxpackets
 ****************************************************************************/

#ifdef CONFIG_NETDEV_STATISTICS
static int netprocfs_rxpackets(FAR struct netprocfs_file_s *netfile)
{
  FAR struct netdev_statistics_s *stats;
  FAR struct net_driver_s *dev;
  FAR char *fmt;

  DEBUGASSERT(netfile != NULL && netfile->dev != NULL);
  dev = netfile->dev;
  stats = &dev->d_statistics;

  fmt = "\t    "
#ifdef CONFIG_NET_IPv4
        "%08lx "
#endif
#ifdef CONFIG_NET_IPv6
        "%08lx "
#endif
#ifdef CONFIG_NET_ARP
        "%08lx "
#endif
        "%08lx\n";

  return snprintf(netfile->line, NET_LINELEN, fmt
#ifdef CONFIG_NET_IPv4
        , (unsigned long)stats->rx_ipv4
#endif
#ifdef CONFIG_NET_IPv6
        , (unsigned long)stats->rx_ipv6
#endif
#ifdef CONFIG_NET_ARP
        , (unsigned long)stats->rx_arp
#endif
        , (unsigned long)stats->rx_dropped);
}
#endif /* CONFIG_NETDEV_STATISTICS */

/****************************************************************************
 * Name: netprocfs_txstatistics_header
 ****************************************************************************/

#ifdef CONFIG_NETDEV_STATISTICS
static int netprocfs_txstatistics_header(FAR struct netprocfs_file_s *netfile)
{
  DEBUGASSERT(netfile != NULL);

  return snprintf(netfile->line, NET_LINELEN, "\tTX: %-8s %-8s %-8s %-8s\n",
                 "Queued", "Sent", "Errors", "Timeouts");
}
#endif /* CONFIG_NETDEV_STATISTICS */

/****************************************************************************
 * Name: netprocfs_txstatistics
 ****************************************************************************/

#ifdef CONFIG_NETDEV_STATISTICS
static int netprocfs_txstatistics(FAR struct netprocfs_file_s *netfile)
{
  FAR struct netdev_statistics_s *stats;
  FAR struct net_driver_s *dev;

  DEBUGASSERT(netfile != NULL && netfile->dev != NULL);
  dev = netfile->dev;
  stats = &dev->d_statistics;

  return snprintf(netfile->line, NET_LINELEN, "\t    %08lx %08lx %08lx %08lx\n",
                  (unsigned long)stats->tx_packets,
                  (unsigned long)stats->tx_done,
                  (unsigned long)stats->tx_errors,
                  (unsigned long)stats->tx_timeouts);
}
#endif /* CONFIG_NETDEV_STATISTICS */

/****************************************************************************
 * Name: netprocfs_errors
 ****************************************************************************/

#ifdef CONFIG_NETDEV_STATISTICS
static int netprocfs_errors(FAR struct netprocfs_file_s *netfile)
{
  FAR struct netdev_statistics_s *stats;
  FAR struct net_driver_s *dev;

  DEBUGASSERT(netfile != NULL && netfile->dev != NULL);
  dev = netfile->dev;
  stats = &dev->d_statistics;

  return snprintf(netfile->line, NET_LINELEN , "\tTotal Errors: %08x\n\n",
                  (unsigned long)stats->errors);
}
#endif /* CONFIG_NETDEV_STATISTICS */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: netprocfs_read_devstats
 *
 * Description:
 *   Read and format network device statistics.
 *
 * Input Parameters:
 *   priv - A reference to the network procfs file structure
 *   buffer - The user-provided buffer into which device status will be
 *            returned.
 *   bulen  - The size in bytes of the user provided buffer.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

ssize_t netprocfs_read_devstats(FAR struct netprocfs_file_s *priv,
                                FAR char *buffer, size_t buflen)
{
  return netprocfs_read_linegen(priv, buffer, buflen, g_netstat_linegen,
                                NSTAT_LINES);
}

#endif /* !CONFIG_DISABLE_MOUNTPOINT && CONFIG_FS_PROCFS &&
        * !CONFIG_FS_PROCFS_EXCLUDE_NET */
