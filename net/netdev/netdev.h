/****************************************************************************
 * net/netdev/netdev.h
 *
 *   Copyright (C) 2014-2015 Gregory Nutt. All rights reserved.
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
#include <stdbool.h>

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

#if CONFIG_NSOCKET_DESCRIPTORS > 0
/* List of registered Ethernet device drivers.  You must have the network
 * locked in order to access this list.
 *
 * NOTE that this duplicates a declaration in net/tcp/tcp.h
 */

EXTERN struct net_driver_s *g_netdevices;
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: netdev_ifup / netdev_ifdown
 *
 * Description:
 *   Bring the interface up/down
 *
 ****************************************************************************/

void netdev_ifup(FAR struct net_driver_s *dev);
void netdev_ifdown(FAR struct net_driver_s *dev);

/****************************************************************************
 * Function: netdev_verify
 *
 * Description:
 *   Verify that the specified device still exists
 *
 * Assumptions:
 *   The caller has locked the network.
 *
 ****************************************************************************/

bool netdev_verify(FAR struct net_driver_s *dev);

/****************************************************************************
 * Function: netdev_findbyname
 *
 * Description:
 *   Find a previously registered network device using its assigned
 *   network interface name
 *
 * Parameters:
 *   ifname The interface name of the device of interest
 *
 * Returned Value:
 *  Pointer to driver on success; null on failure
 *
 * Assumptions:
 *  Called from normal user mode
 *
 ****************************************************************************/

#if CONFIG_NSOCKET_DESCRIPTORS > 0
FAR struct net_driver_s *netdev_findbyname(FAR const char *ifname);
#endif

/****************************************************************************
 * Function: netdev_findby_ipv4addr
 *
 * Description:
 *   Find a previously registered network device by matching an arbitrary
 *   IPv4 address.
 *
 * Parameters:
 *   lipaddr - Local, bound address of a connection.  Used only if ripaddr
 *     is the broadcast address.  Used only if CONFIG_NETDEV_MULTINIC.
 *   ripaddr - Remote address of a connection to use in the lookup
 *
 * Returned Value:
 *  Pointer to driver on success; null on failure
 *
 * Assumptions:
 *  Called from normal user mode
 *
 ****************************************************************************/

#if CONFIG_NSOCKET_DESCRIPTORS > 0
#ifdef CONFIG_NET_IPv4
#ifdef CONFIG_NETDEV_MULTINIC
FAR struct net_driver_s *netdev_findby_ipv4addr(in_addr_t lipaddr,
                                                in_addr_t ripaddr);
#else
FAR struct net_driver_s *netdev_findby_ipv4addr(in_addr_t ripaddr);
#endif
#endif

/****************************************************************************
 * Function: netdev_findby_ipv6addr
 *
 * Description:
 *   Find a previously registered network device by matching an arbitrary
 *   IPv6 address.
 *
 * Parameters:
 *   lipaddr - Local, bound address of a connection.  Used only if ripaddr
 *     is the broadcast address.  Used only if CONFIG_NETDEV_MULTINIC.
 *   ripaddr - Remote address of a connection to use in the lookup
 *
 * Returned Value:
 *  Pointer to driver on success; null on failure
 *
 * Assumptions:
 *  Called from normal user mode
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv6
#ifdef CONFIG_NETDEV_MULTINIC
FAR struct net_driver_s *netdev_findby_ipv6addr(const net_ipv6addr_t lipaddr,
                                                const net_ipv6addr_t ripaddr);
#else
FAR struct net_driver_s *netdev_findby_ipv6addr(const net_ipv6addr_t ripaddr);
#endif
#endif
#endif

/****************************************************************************
 * Function: netdev_default
 *
 * Description:
 *   Return the default network device.  REVISIT:  At present this function
 *   arbitrarily returns the first UP device at the head of the device
 *   list.  Perhaps the default device should be a device name
 *   configuration option?
 *
 *   So why is this here:  It represents my current though for what to do
 *   if a socket is connected with INADDY_ANY.  In this case, I suppose we
 *   should use the IP address associated with some default device???
 *
 * Parameters:
 *   NULL
 *
 * Returned Value:
 *  Pointer to default network driver on success; null on failure
 *
 * Assumptions:
 *  Called from normal user mode
 *
 ****************************************************************************/

#if CONFIG_NSOCKET_DESCRIPTORS > 0
FAR struct net_driver_s *netdev_default(void);
#endif

/****************************************************************************
 * Function: netdev_ipv4_txnotify
 *
 * Description:
 *   Notify the device driver that forwards the IPv4 address that new TX
 *   data is available.
 *
 * Parameters:
 *   lipaddr - The local address bound to the socket
 *   ripaddr - The remote address to send the data
 *
 * Returned Value:
 *  None
 *
 * Assumptions:
 *  Called from normal user mode
 *
 ****************************************************************************/

#if CONFIG_NSOCKET_DESCRIPTORS > 0
#ifdef CONFIG_NET_IPv4
#  ifdef CONFIG_NETDEV_MULTINIC
void netdev_ipv4_txnotify(in_addr_t lipaddr, in_addr_t ripaddr);
#  else
void netdev_ipv4_txnotify(in_addr_t ripaddr);
#  endif
#endif /* CONFIG_NET_IPv4 */

/****************************************************************************
 * Function: netdev_ipv6_txnotify
 *
 * Description:
 *   Notify the device driver that forwards the IPv4 address that new TX
 *   data is available.
 *
 * Parameters:
 *   lipaddr - The local address bound to the socket
 *   ripaddr - The remote address to send the data
 *
 * Returned Value:
 *  None
 *
 * Assumptions:
 *  Called from normal user mode
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv6
#  ifdef CONFIG_NETDEV_MULTINIC
void netdev_ipv6_txnotify(FAR const net_ipv6addr_t lipaddr,
                          FAR const net_ipv6addr_t ripaddr);
#  else
void netdev_ipv6_txnotify(FAR const net_ipv6addr_t ripaddr);
#  endif
#endif /* CONFIG_NET_IPv6 */
#endif /* CONFIG_NSOCKET_DESCRIPTORS > 0 */

/****************************************************************************
 * Function: netdev_txnotify_dev
 *
 * Description:
 *   Notify the device driver that new TX data is available.  This variant
 *   would be called when the upper level logic already understands how the
 *   packet will be routed.
 *
 * Parameters:
 *   dev - The network device driver state structure.
 *
 * Returned Value:
 *  None
 *
 * Assumptions:
 *  Called from normal user mode
 *
 ****************************************************************************/

void netdev_txnotify_dev(FAR struct net_driver_s *dev);

/****************************************************************************
 * Function: netdev_ipv4_rxnotify
 *
 * Description:
 *   Notify the device driver that forwards the IPv4 address that the
 *  application waits for RX data.
 *
 * Parameters:
 *   lipaddr - The local board IPv6 address of the socket
 *   ripaddr - The remote IPv4 address to send the data
 *
 * Returned Value:
 *  None
 *
 * Assumptions:
 *  Called from normal user mode
 *
 ****************************************************************************/

#if CONFIG_NSOCKET_DESCRIPTORS > 0 && defined(CONFIG_NET_RXAVAIL)

#ifdef CONFIG_NET_IPv4
#  ifdef CONFIG_NETDEV_MULTINIC
void netdev_ipv4_rxnotify(in_addr_t lipaddr, in_addr_t ripaddr);
#  else
void netdev_ipv4_rxnotify(in_addr_t ripaddr);
#  endif
#endif /* CONFIG_NET_IPv4 */

/****************************************************************************
 * Function: netdev_ipv6_rxnotify
 *
 * Description:
 *   Notify the device driver that forwards the IPv6 address that the
 *  application waits for RX data.
 *
 * Parameters:
 *   lipaddr - The local board IPv6 address of the socket
 *   ripaddr - The remote IPv6 address to send the data
 *
 * Returned Value:
 *  None
 *
 * Assumptions:
 *  Called from normal user mode
 *
 ****************************************************************************/

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

/****************************************************************************
 * Function: netdev_count
 *
 * Description:
 *   Return the number of network devices
 *
 * Parameters:
 *   None
 *
 * Returned Value:
 *   The number of network devices
 *
 * Assumptions:
 *  Called from normal user mode
 *
 ****************************************************************************/

#if CONFIG_NSOCKET_DESCRIPTORS > 0
int netdev_count(void);
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __NET_NETDEV_NETDEV_H */
