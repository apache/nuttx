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

/* If CONFIG_NETDEV_IFINDEX is enabled then there is limit to the number of
 * devices that can be registered due to the nature of some static data.
 */

#define MAX_IFINDEX  32

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

#ifdef CONFIG_NETDEV_IFINDEX
/* The set of network devices that have been registered.  This is used to
 * assign a unique device index to the newly registered device.
 *
 * REVISIT:  The width of g_nassigned limits the number of registered
 * devices to 32 (MAX_IFINDEX).
 */

EXTERN uint32_t g_devset;

/* The set of network devices that have been freed.  The purpose of this
 * set is to postpone reuse of a interface index for as long as possible,
 * i.e., don't reuse an interface index until all of the possible indices
 * have been used.
 */

EXTERN uint32_t g_devfreed;
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Callback from netdev_foreach() */

struct net_driver_s; /* Forward reference */
typedef int (*netdev_callback_t)(FAR struct net_driver_s *dev, FAR void *arg);

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
 * Name: netdev_verify
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
 * Name: netdev_findbyname
 *
 * Description:
 *   Find a previously registered network device using its assigned
 *   network interface name
 *
 * Input Parameters:
 *   ifname The interface name of the device of interest
 *
 * Returned Value:
 *  Pointer to driver on success; null on failure
 *
 ****************************************************************************/

#if CONFIG_NSOCKET_DESCRIPTORS > 0
FAR struct net_driver_s *netdev_findbyname(FAR const char *ifname);
#endif

/****************************************************************************
 * Name: netdev_foreach
 *
 * Description:
 *   Enumerate each registered network device.
 *
 *   NOTE: netdev semaphore held throughout enumeration.
 *
 * Input Parameters:
 *   callback - Will be called for each registered device
 *   arg      - User argument passed to callback()
 *
 * Returned Value:
 *  0:Enumeration completed 1:Enumeration terminated early by callback
 *
 ****************************************************************************/

int netdev_foreach(netdev_callback_t callback, FAR void *arg);

/****************************************************************************
 * Name: netdev_findby_ipv4addr
 *
 * Description:
 *   Find a previously registered network device by matching an arbitrary
 *   IPv4 address.
 *
 * Input Parameters:
 *   lipaddr - Local, bound address of a connection.
 *   ripaddr - Remote address of a connection to use in the lookup
 *
 * Returned Value:
 *  Pointer to driver on success; null on failure
 *
 ****************************************************************************/

#if CONFIG_NSOCKET_DESCRIPTORS > 0
#ifdef CONFIG_NET_IPv4
FAR struct net_driver_s *netdev_findby_ipv4addr(in_addr_t lipaddr,
                                                in_addr_t ripaddr);
#endif

/****************************************************************************
 * Name: netdev_findby_ipv6addr
 *
 * Description:
 *   Find a previously registered network device by matching an arbitrary
 *   IPv6 address.
 *
 * Input Parameters:
 *   lipaddr - Local, bound address of a connection.
 *   ripaddr - Remote address of a connection to use in the lookup
 *
 * Returned Value:
 *  Pointer to driver on success; null on failure
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv6
FAR struct net_driver_s *netdev_findby_ipv6addr(const net_ipv6addr_t lipaddr,
                                                const net_ipv6addr_t ripaddr);
#endif
#endif

/****************************************************************************
 * Name: netdev_findbyindex
 *
 * Description:
 *   Find a previously registered network device by assigned interface index.
 *
 * Input Parameters:
 *   ifindex - The interface index.  This is a one-based index and must be
 *             greater than zero.
 *
 * Returned Value:
 *  Pointer to driver on success; NULL on failure.  This function will return
 *  NULL only if there is no device corresponding to the provided index.
 *
 ****************************************************************************/

FAR struct net_driver_s *netdev_findbyindex(int ifindex);

/****************************************************************************
 * Name: netdev_nextindex
 *
 * Description:
 *   Return the interface index to the next valid device.
 *
 * Input Parameters:
 *   ifindex - The first interface index to check.  Usually in a traversal
 *             this would be the previous interface index plus 1.
 *
 * Returned Value:
 *   The interface index for the next network driver.  -ENODEV is returned if
 *   there are no further devices with assigned interface indices.
 *
 ****************************************************************************/

#ifdef CONFIG_NETDEV_IFINDEX
int netdev_nextindex(int ifindex);
#endif

/****************************************************************************
 * Name: netdev_indextoname
 *
 * Description:
 *   The if_indextoname() function maps an interface index to its
 *   corresponding name.
 *
 * Input Parameters:
 *   ifname  - Points to a buffer of at least IF_NAMESIZE bytes.
 *             if_indextoname() will place in this buffer the name of the
 *             interface with index ifindex.
 *
 * Returned Value:
 *   If ifindex is an interface index, then the function will return zero
 *   (OK). Otherwise, the function returns a negated errno value;
 *
 ****************************************************************************/

#ifdef CONFIG_NETDEV_IFINDEX
int netdev_indextoname(unsigned int ifindex, FAR char *ifname);
#endif

/****************************************************************************
 * Name: netdev_nametoindex
 *
 * Description:
 *   The if_nametoindex() function returns the interface index corresponding
 *   to name ifname.
 *
 * Input Parameters:
 *   ifname - The interface name
 *
 * Returned Value:
 *   The corresponding index if ifname is the name of an interface;
 *   otherwise, a negated errno value is returned.
 *
 ****************************************************************************/

#ifdef CONFIG_NETDEV_IFINDEX
unsigned int netdev_nametoindex(FAR const char *ifname);
#endif

/****************************************************************************
 * Name: netdev_default
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
 * Input Parameters:
 *   NULL
 *
 * Returned Value:
 *  Pointer to default network driver on success; null on failure
 *
 ****************************************************************************/

#if CONFIG_NSOCKET_DESCRIPTORS > 0
FAR struct net_driver_s *netdev_default(void);
#endif

/****************************************************************************
 * Name: netdev_ipv4_txnotify
 *
 * Description:
 *   Notify the device driver that forwards the IPv4 address that new TX
 *   data is available.
 *
 * Input Parameters:
 *   lipaddr - The local address bound to the socket
 *   ripaddr - The remote address to send the data
 *
 * Returned Value:
 *  None
 *
 ****************************************************************************/

#if CONFIG_NSOCKET_DESCRIPTORS > 0
#ifdef CONFIG_NET_IPv4
void netdev_ipv4_txnotify(in_addr_t lipaddr, in_addr_t ripaddr);
#endif /* CONFIG_NET_IPv4 */

/****************************************************************************
 * Name: netdev_ipv6_txnotify
 *
 * Description:
 *   Notify the device driver that forwards the IPv4 address that new TX
 *   data is available.
 *
 * Input Parameters:
 *   lipaddr - The local address bound to the socket
 *   ripaddr - The remote address to send the data
 *
 * Returned Value:
 *  None
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv6
void netdev_ipv6_txnotify(FAR const net_ipv6addr_t lipaddr,
                          FAR const net_ipv6addr_t ripaddr);
#endif /* CONFIG_NET_IPv6 */
#endif /* CONFIG_NSOCKET_DESCRIPTORS > 0 */

/****************************************************************************
 * Name: netdev_txnotify_dev
 *
 * Description:
 *   Notify the device driver that new TX data is available.  This variant
 *   would be called when the upper level logic already understands how the
 *   packet will be routed.
 *
 * Input Parameters:
 *   dev - The network device driver state structure.
 *
 * Returned Value:
 *  None
 *
 ****************************************************************************/

void netdev_txnotify_dev(FAR struct net_driver_s *dev);

/****************************************************************************
 * Name: netdev_count
 *
 * Description:
 *   Return the number of network devices
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   The number of network devices
 *
 ****************************************************************************/

#if CONFIG_NSOCKET_DESCRIPTORS > 0
int netdev_count(void);
#endif

/****************************************************************************
 * Name: netdev_ipv4_ifconf
 *
 * Description:
 *   Return the IPv4 configuration of each network adaptor
 *
 * Input Parameters:
 *   ifc - A reference to the instance of struct ifconf in which to return
 *         the information.
 *
 * Returned Value:
 *   Zero is returned on success; a negated errno value is returned on any
 *   failure.
 *
 * Assumptions:
 *  The nework is locked
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv4
struct ifconf;  /* Forward reference */
int netdev_ipv4_ifconf(FAR struct ifconf *ifc);
#endif

/****************************************************************************
 * Name: netdev_ipv6_ifconf
 *
 * Description:
 *   Return the IPv6 configuration of each network adaptor
 *
 * Input Parameters:
 *   lifc - A reference to the instance of struct lifconf in which to return
 *          the information.
 *
 * Returned Value:
 *   Zero is returned on success; a negated errno value is returned on any
 *   failure.
 *
 * Assumptions:
 *  The nework is locked
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv6
struct lifconf;  /* Forward reference */
int netdev_ipv6_ifconf(FAR struct lifconf *lifc);
#endif

/****************************************************************************
 * Name: netdev_dev_lladdrsize
 *
 * Description:
 *   Returns the size of the MAC address associated with a network device.
 *
 * Input Parameters:
 *   dev - A reference to the device of interest
 *
 * Returned Value:
 *   The size of the MAC address associated with this device
 *
 ****************************************************************************/

int netdev_dev_lladdrsize(FAR struct net_driver_s *dev);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __NET_NETDEV_NETDEV_H */
