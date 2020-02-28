/****************************************************************************
 * net/netdev/netdev.h
 *
 *   Copyright (C) 2014-2015, 2018 Gregory Nutt. All rights reserved.
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

#ifdef CONFIG_NETDOWN_NOTIFIER
#  include <nuttx/wqueue.h>
#endif

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

/* List of registered Ethernet device drivers.  You must have the network
 * locked in order to access this list.
 *
 * NOTE that this duplicates a declaration in net/tcp/tcp.h
 */

EXTERN struct net_driver_s *g_netdevices;

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

FAR struct net_driver_s *netdev_findbyname(FAR const char *ifname);

/****************************************************************************
 * Name: netdev_foreach
 *
 * Description:
 *   Enumerate each registered network device.  This function will terminate
 *   when either (1) all devices have been enumerated or (2) when a callback
 *   returns any non-zero value.
 *
 *   NOTE 1:  The network must be locked throughout the enumeration.
 *   NOTE 2:  No checks are made on devices.  For examples, callbacks will
 *            will be made on network devices that are in the 'down' state.
 *            The callback implementations must take into account all
 *            network device state.  Typically, a network in the down state
 *            would not terminate the traversal.
 *
 * Input Parameters:
 *   callback - Will be called for each registered device
 *   arg      - Opaque user argument passed to callback()
 *
 * Returned Value:
 *  0: Enumeration completed
 *  1: Enumeration terminated early by callback
 *
 * Assumptions:
 *  The network is locked.
 *
 ****************************************************************************/

int netdev_foreach(netdev_callback_t callback, FAR void *arg);

/****************************************************************************
 * Name: netdev_findby_lipv4addr
 *
 * Description:
 *   Find a previously registered network device by matching a local address
 *   with the subnet served by the device.  Only "up" devices are considered
 *   (since a "down" device has no meaningful address).
 *
 * Input Parameters:
 *   lipaddr - Local, IPv4 address assigned to the network device.  Or any
 *             IPv4 address on the sub-net served by the network device.
 *
 * Returned Value:
 *   Pointer to driver on success; null on failure
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv4
FAR struct net_driver_s *netdev_findby_lipv4addr(in_addr_t lipaddr);
#endif

/****************************************************************************
 * Name: netdev_findby_lipv6addr
 *
 * Description:
 *   Find a previously registered network device by matching a local address
 *   with the subnet served by the device.  Only "up" devices are considered
 *   (since a "down" device has no meaningful address).
 *
 * Input Parameters:
 *   lipaddr - Local, IPv6 address assigned to the network device.  Or any
 *             IPv6 address on the sub-net served by the network device.
 *
 * Returned Value:
 *   Pointer to driver on success; null on failure
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv6
FAR struct net_driver_s *netdev_findby_lipv6addr(const net_ipv6addr_t lipaddr);
#endif

/****************************************************************************
 * Name: netdev_findby_ripv4addr
 *
 * Description:
 *   Find a previously registered network device by matching the remote
 *   IPv4 address that can be reached by the device.
 *
 * Input Parameters:
 *   lipaddr - Local, bound address of a connection (used only if ripaddr is
 *             the broadcast address).
 *   ripaddr - Remote address of a connection to use in the lookup
 *
 * Returned Value:
 *   Pointer to driver on success; null on failure
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv4
FAR struct net_driver_s *netdev_findby_ripv4addr(in_addr_t lipaddr,
                                                 in_addr_t ripaddr);
#endif

/****************************************************************************
 * Name: netdev_findby_ripv6addr
 *
 * Description:
 *   Find a previously registered network device by matching the remote
 *   IPv6 address that can be reached by the device.
 *
 * Input Parameters:
 *   lipaddr - Local, bound address of a connection (used only if ripaddr is
 *             a multicast address).
 *   ripaddr - Remote address of a connection to use in the lookup
 *
 * Returned Value:
 *   Pointer to driver on success; null on failure
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv6
FAR struct net_driver_s *netdev_findby_ripv6addr(const net_ipv6addr_t lipaddr,
                                                 const net_ipv6addr_t ripaddr);
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

FAR struct net_driver_s *netdev_default(void);

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

int netdev_count(void);

/****************************************************************************
 * Name: netdev_ipv4_ifconf
 *
 * Description:
 *   Return the IPv4 configuration of each network adapter
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
 *  The network is locked
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
 *   Return the IPv6 configuration of each network adapter
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
 *  The network is locked
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv6
struct lifconf;  /* Forward reference */
int netdev_ipv6_ifconf(FAR struct lifconf *lifc);
#endif

/****************************************************************************
 * Name: netdown_notifier_setup
 *
 * Description:
 *   Set up to perform a callback to the worker function when the network
 *   goes down.  The worker function will execute on the high priority
 *   worker thread.
 *
 * Input Parameters:
 *   worker - The worker function to execute on the high priority work
 *            queue when data is available in the UDP readahead buffer.
 *   dev   - The network driver to be monitored
 *   arg    - A user-defined argument that will be available to the worker
 *            function when it runs.
 * Returned Value:
 *   > 0   - The signal notification is in place.  The returned value is a
 *           key that may be used later in a call to
 *           netdown_notifier_teardown().
 *   == 0  - The the device is already down.  No signal notification will
 *           be provided.
 *   < 0   - An unexpected error occurred and no signal will be sent.  The
 *           returned value is a negated errno value that indicates the
 *           nature of the failure.
 *
 ****************************************************************************/

#ifdef CONFIG_NETDOWN_NOTIFIER
int netdown_notifier_setup(worker_t worker, FAR struct net_driver_s *dev,
                           FAR void *arg);
#endif

/****************************************************************************
 * Name: netdown_notifier_teardown
 *
 * Description:
 *   Eliminate a network down notification previously setup by
 *   netdown_notifier_setup().  This function should only be called if the
 *   notification should be aborted prior to the notification.  The
 *   notification will automatically be torn down after the signal is sent.
 *
 * Input Parameters:
 *   key - The key value returned from a previous call to
 *         netdown_notifier_setup().
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_NETDOWN_NOTIFIER
int netdown_notifier_teardown(int key);
#endif

/****************************************************************************
 * Name: netdown_notifier_signal
 *
 * Description:
 *   A network has gone down has been buffered.  Execute worker thread
 *   functions for all threads monitoring the state of the device.
 *
 * Input Parameters:
 *   dev  - The TCP connection where read-ahead data was just buffered.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#ifdef CONFIG_NETDOWN_NOTIFIER
void netdown_notifier_signal(FAR struct net_driver_s *dev);
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __NET_NETDEV_NETDEV_H */
