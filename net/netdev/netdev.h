/****************************************************************************
 * net/netdev/netdev.h
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
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

#ifndef __NET_NETDEV_NETDEV_H
#define __NET_NETDEV_NETDEV_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>

#include <nuttx/net/ip.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/* List of registered Ethernet device drivers */

#if CONFIG_NSOCKET_DESCRIPTORS > 0
EXTERN struct net_driver_s *g_netdevices;
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* netdev_register.c *********************************************************/

#if CONFIG_NSOCKET_DESCRIPTORS > 0
void netdev_seminit(void);
void netdev_semtake(void);
void netdev_semgive(void);
#endif

/* netdev_ioctl.c ************************************************************/

void netdev_ifup(FAR struct net_driver_s *dev);
void netdev_ifdown(FAR struct net_driver_s *dev);

/* netdev_findbyname.c *******************************************************/

#if CONFIG_NSOCKET_DESCRIPTORS > 0
FAR struct net_driver_s *netdev_findbyname(FAR const char *ifname);
#endif

/* netdev_findbyaddr.c *******************************************************/

#if CONFIG_NSOCKET_DESCRIPTORS > 0
#ifdef CONFIG_NET_IPv4
#ifdef CONFIG_NETDEV_MULTINIC
FAR struct net_driver_s *netdev_findby_ipv4addr(in_addr_t lipaddr,
                                                in_addr_t ripaddr);
#else
FAR struct net_driver_s *netdev_findby_ipv4addr(in_addr_t ripaddr);
#endif
#endif

#ifdef CONFIG_NET_IPv6
#ifdef CONFIG_NETDEV_MULTINIC
FAR struct net_driver_s *netdev_findby_ipv6addr(const net_ipv6addr_t lipaddr,
                                                const net_ipv6addr_t ripaddr);
#else
FAR struct net_driver_s *netdev_findby_ipv6addr(const net_ipv6addr_t ripaddr);
#endif
#endif
#endif

/* netdev_default.c ***********************************************************/

#if CONFIG_NSOCKET_DESCRIPTORS > 0
FAR struct net_driver_s *netdev_default(void);
#endif

/* netdev_txnotify.c *********************************************************/

#if CONFIG_NSOCKET_DESCRIPTORS > 0
#ifdef CONFIG_NET_IPv4
#  ifdef CONFIG_NETDEV_MULTINIC
void netdev_ipv4_txnotify(in_addr_t lipaddr, in_addr_t ripaddr);
#  else
void netdev_ipv4_txnotify(in_addr_t ripaddr);
#  endif
#endif /* CONFIG_NET_IPv4 */

#ifdef CONFIG_NET_IPv6
#  ifdef CONFIG_NETDEV_MULTINIC
void netdev_ipv6_txnotify(FAR const net_ipv6addr_t lipaddr,
                          FAR const net_ipv6addr_t ripaddr);
#  else
void netdev_ipv6_txnotify(FAR const net_ipv6addr_t ripaddr);
#  endif
#endif /* CONFIG_NET_IPv6 */
#endif /* CONFIG_NSOCKET_DESCRIPTORS > 0 */

/* netdev_rxnotify.c *********************************************************/

#if CONFIG_NSOCKET_DESCRIPTORS > 0 && defined(CONFIG_NET_RXAVAIL)

#ifdef CONFIG_NET_IPv4
#  ifdef CONFIG_NETDEV_MULTINIC
void netdev_ipv4_rxnotify(in_addr_t lipaddr, in_addr_t ripaddr);
#  else
void netdev_ipv4_rxnotify(in_addr_t ripaddr);
#  endif
#endif /* CONFIG_NET_IPv4 */

#ifdef CONFIG_NET_IPv6
#  ifdef CONFIG_NETDEV_MULTINIC
void netdev_ipv6_rxnotify(FAR const net_ipv6addr_t lipaddr,
                          FAR const net_ipv6addr_t ripaddr);
#  else
void netdev_ipv6_rxnotify(FAR const net_ipv6addr_t ripaddr);
#  endif
#endif /* CONFIG_NET_IPv6 */

#else
#ifdef CONFIG_NET_IPv4
#  ifdef CONFIG_NETDEV_MULTINIC
#    define netdev_ipv4_rxnotify(lipaddr,ripaddr)
#  else
#    define netdev_ipv4_rxnotify(ripaddr)
#  endif
#endif /* CONFIG_NET_IPv4 */

#ifdef CONFIG_NET_IPv6
#  ifdef CONFIG_NETDEV_MULTINIC
#    define netdev_ipv6_rxnotify(lipaddr,ripaddr)
#  else
#    define netdev_ipv6_rxnotify(ripaddr)
#  endif
#endif /* CONFIG_NET_IPv6 */
#endif

/* netdev_count.c ************************************************************/

#if CONFIG_NSOCKET_DESCRIPTORS > 0
int netdev_count(void);
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __NET_NETDEV_NETDEV_H */
