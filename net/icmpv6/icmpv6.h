/****************************************************************************
 * net/icmpv6/icmpv6.h
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

#ifndef __NET_ICMPv6_ICMPv6_H
#define __NET_ICMPv6_ICMPv6_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <semaphore.h>

#include <nuttx/net/ip.h>

#ifdef CONFIG_NET_ICMPv6

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Allocate a new ICMPv6 data callback */

#define icmpv6_callback_alloc(dev)   devif_callback_alloc(dev, &(dev)->d_conncb)
#define icmpv6_callback_free(dev,cb) devif_dev_callback_free(dev, cb)

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

#if defined(CONFIG_NET_ICMPv6_PING) || defined(CONFIG_NET_ICMPv6_NEIGHBOR) || \
    defined(CONFIG_NET_ICMPv6_AUTOCONF)
/* For symmetry with other protocols, a "connection" structure is
 * provided.  But, in this case, it is a singleton.
 */

struct icmpv6_conn_s
{
  FAR struct devif_callback_s *list;    /* Neighbor discovery callbacks */
};
#endif

#ifdef CONFIG_NET_ICMPv6_NEIGHBOR
/* Used to notify a thread waiting for a particular Neighbor Advertisement */

struct icmpv6_notify_s
{
  FAR struct icmpv6_notify_s *nt_flink; /* Supports singly linked list */
  net_ipv6addr_t nt_ipaddr;             /* Waited for IP address in the mapping */
  sem_t nt_sem;                         /* Will wake up the waiter */
  int nt_result;                        /* The result of the wait */
};
#endif

#ifdef CONFIG_NET_ICMPv6_AUTOCONF
/* Used to notify a thread waiting for a particular Router Advertisement */

struct icmpv6_rnotify_s
{
#ifdef CONFIG_NET_MULTILINK
  FAR struct icmpv6_notify_s *rn_flink; /* Supports singly linked list */
  char rn_ifname[IFNAMSIZ];             /* Device name */
#endif
  sem_t rn_sem;                         /* Will wake up the waiter */
  int rn_result;                        /* The result of the wait */
};
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#  define EXTERN extern "C"
extern "C"
{
#else
#  define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

struct timespec;     /* Forward reference */
struct net_driver_s; /* Forward reference */

/****************************************************************************
 * Name: icmpv6_input
 *
 * Description:
 *   Handle incoming ICMPv6 input
 *
 * Parameters:
 *   dev - The device driver structure containing the received ICMPv6
 *         packet
 *
 * Return:
 *   None
 *
 * Assumptions:
 *   Called from the interrupt level or with interrupts disabled.
 *
 ****************************************************************************/

void icmpv6_input(FAR struct net_driver_s *dev);

/****************************************************************************
 * Name: icmpv6_neighbor
 *
 * Description:
 *   The icmpv6_solicit() call may be to send an ICMPv6 Neighbor
 *   Solicitation to resolve an IPv6 address.  This function first checks if
 *   the IPv6 address is already in the Neighbor Table.  If so, then it
 *   returns success immediately.
 *
 *   If the requested IPv6 address in not in the Neighbor Table, then this
 *   function will send the Neighbor Solicitation, delay, then check if the
 *   IP address is now in the Neighbor able.  It will repeat this sequence
 *   until either (1) the IPv6 address mapping is now in the Neighbor table,
 *   or (2) a configurable number of timeouts occur without receiving the
 *   ICMPv6 Neighbor Advertisement.
 *
 * Parameters:
 *   ipaddr   The IPv6 address to be queried.
 *
 * Returned Value:
 *   Zero (OK) is returned on success and the IP address mapping can now be
 *   found in the Neighbor Table.  On error a negated errno value is returned:
 *
 *     -ETIMEDOUT:    The number or retry counts has been exceed.
 *     -EHOSTUNREACH: Could not find a route to the host
 *
 * Assumptions:
 *   This function is called from the normal tasking context.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_ICMPv6_NEIGHBOR
int icmpv6_neighbor(const net_ipv6addr_t ipaddr);
#else
#  define icmpv6_neighbor(i) (0)
#endif

/****************************************************************************
 * Name: icmpv6_poll
 *
 * Description:
 *   Poll a UDP "connection" structure for availability of TX data
 *
 * Parameters:
 *   dev - The device driver structure to use in the send operation
 *
 * Return:
 *   None
 *
 * Assumptions:
 *   Called from the interrupt level or with interrupts disabled.
 *
 ****************************************************************************/

#if defined(CONFIG_NET_ICMPv6_PING) || defined(CONFIG_NET_ICMPv6_NEIGHBOR)
void icmpv6_poll(FAR struct net_driver_s *dev);
#endif

/****************************************************************************
 * Name: icmpv6_solicit
 *
 * Description:
 *   Set up to send an ICMPv6 Neighbor Solicitation message
 *
 * Parameters:
 *   dev - Reference to an Ethernet device driver structure
 *   ipaddr - IP address of Neighbor to be solicited
 *
 * Return:
 *   None
 *
 ****************************************************************************/

void icmpv6_solicit(FAR struct net_driver_s *dev,
                    FAR const net_ipv6addr_t ipaddr);

#undef EXTERN
#ifdef __cplusplus
}
#endif

/****************************************************************************
 * Name: icmpv6_rsolicit
 *
 * Description:
 *   Set up to send an ICMPv6 Router Solicitation message.  This version
 *   is for a standalone solicitation.  If formats:
 *
 *   - The Ethernet header
 *   - The IPv6 header
 *   - The ICMPv6 Neighbor Router Message
 *
 *   The device IP address should have been set to the link local address
 *   prior to calling this function.
 *
 * Parameters:
 *   dev - Reference to an Ethernet device driver structure
 *
 * Return:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_NET_ICMPv6_AUTOCONF
void icmpv6_rsolicit(FAR struct net_driver_s *dev);
#endif

/****************************************************************************
 * Name: icmpv6_advertise
 *
 * Description:
 *   Send an ICMPv6 Neighbor Advertisement
 *
 * Parameters:
 *   dev - The device driver structure containing the outgoing ICMPv6 packet
 *         buffer
 *
 * Return:
 *   None.
 *
 * Assumptions:
 *   The network is locked
 *
 ****************************************************************************/

void icmpv6_advertise(FAR struct net_driver_s *dev,
                      const net_ipv6addr_t destipaddr);

/****************************************************************************
 * Name: icmpv6_radvertise
 *
 * Description:
 *   Send an ICMPv6 Router Advertisement
 *
 * Parameters:
 *   dev - The device driver structure containing the outgoing ICMPv6 packet
 *         buffer
 *
 * Return:
 *   None
 *
 * Assumptions:
 *   The network is locked
 *
 ****************************************************************************/

#ifdef CONFIG_NET_ICMPv6_ROUTER
void icmpv6_radvertise(FAR struct net_driver_s *dev);
#endif

/****************************************************************************
 * Function: icmpv6_wait_setup
 *
 * Description:
 *   Called BEFORE an Neighbor Solicitation is sent.  This function sets up
 *   the Neighbor Advertisement timeout before the Neighbor Solicitation
 *   is sent so that there is no race condition when icmpv6_wait() is called.
 *
 * Assumptions:
 *   This function is called from icmpv6_neighbor() and executes in the normal
 *   tasking environment.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_ICMPv6_NEIGHBOR
void icmpv6_wait_setup(const net_ipv6addr_t ipaddr,
                       FAR struct icmpv6_notify_s *notify);
#else
#  define icmpv6_wait_setup(i,n)
#endif

/****************************************************************************
 * Function: icmpv6_wait_cancel
 *
 * Description:
 *   Cancel any wait set after icmpv6_wait_setup is called but before
 *   icmpv6_wait()is called (icmpv6_wait() will automatically cancel the
 *   wait).
 *
 * Assumptions:
 *   This function may execute in the interrupt context when called from
 *   icmpv6_wait().
 *
 ****************************************************************************/

#ifdef CONFIG_NET_ICMPv6_NEIGHBOR
int icmpv6_wait_cancel(FAR struct icmpv6_notify_s *notify);
#else
#  define icmpv6_wait_cancel(n) (0)
#endif

/****************************************************************************
 * Function: icmpv6_wait
 *
 * Description:
 *   Called each time that a Neighbor Solicitation is sent.  This function
 *   will sleep until either: (1) the matching Neighbor Advertisement is
 *   received, or (2) a timeout occurs.
 *
 * Assumptions:
 *   This function is called from icmpv6_neighbor() and must execute with
 *   the network un-locked (interrupts may be disabled to keep the things
 *   stable).
 *
 ****************************************************************************/

#ifdef CONFIG_NET_ICMPv6_NEIGHBOR
int icmpv6_wait(FAR struct icmpv6_notify_s *notify,
                FAR struct timespec *timeout);
#else
#  define icmpv6_wait(n,t) (0)
#endif

/****************************************************************************
 * Function: icmpv6_notify
 *
 * Description:
 *   Called each time that a Neighbor Advertisement is received in order to
 *   wake-up any threads that may be waiting for this particular Neighbor
 *   Advertisement.
 *
 * Assumptions:
 *   This function is called from the MAC device driver indirectly through
 *   icmpv6_icmpv6in() will execute with the network locked.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_ICMPv6_NEIGHBOR
void icmpv6_notify(net_ipv6addr_t ipaddr);
#else
#  define icmpv6_notify(i)
#endif

/****************************************************************************
 * Name: icmpv6_autoconfig
 *
 * Description:
 *   Perform IPv6 auto-configuration to assign an IPv6 address to this
 *   device.
 *
 * Parameters:
 *   dev - The device driver structure to assign the address to
 *
 * Return:
 *   Zero (OK) is returned on success; A negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_ICMPv6_AUTOCONF
int icmpv6_autoconfig(FAR struct net_driver_s *dev);
#endif

/****************************************************************************
 * Function: icmpv6_rwait_setup
 *
 * Description:
 *   Called BEFORE an Router Solicitation is sent.  This function sets up
 *   the Router Advertisement timeout before the Router Solicitation
 *   is sent so that there is no race condition when icmpv6_rwait() is
 *   called.
 *
 * Assumptions:
 *   This function is called from icmpv6_autoconfig() and executes in the
 *   normal tasking environment.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_ICMPv6_AUTOCONF
void icmpv6_rwait_setup(FAR struct net_driver_s *dev,
                        FAR struct icmpv6_rnotify_s *notify);
#else
#  define icmpv6_rwait_setup(d,n)
#endif

/****************************************************************************
 * Function: icmpv6_rwait_cancel
 *
 * Description:
 *   Cancel any wait set after icmpv6_rwait_setup() is called but before
 *   icmpv6_rwait()is called (icmpv6_rwait() will automatically cancel the
 *   wait).
 *
 * Assumptions:
 *   This function may execute in the interrupt context when called from
 *   icmpv6_rwait().
 *
 ****************************************************************************/

#ifdef CONFIG_NET_ICMPv6_AUTOCONF
int icmpv6_rwait_cancel(FAR struct icmpv6_rnotify_s *notify);
#else
#  define icmpv6_rwait_cancel(n) (0)
#endif

/****************************************************************************
 * Function: icmpv6_rwait
 *
 * Description:
 *   Called each time that a Router Solicitation is sent.  This function
 *   will sleep until either: (1) the matching Router Advertisement is
 *   received, or (2) a timeout occurs.
 *
 * Assumptions:
 *   This function is called from icmpv6_autoconfig() and must execute with
 *   the network un-locked (interrupts may be disabled to keep the things
 *   stable).
 *
 ****************************************************************************/

#ifdef CONFIG_NET_ICMPv6_AUTOCONF
int icmpv6_rwait(FAR struct icmpv6_rnotify_s *notify,
                 FAR struct timespec *timeout);
#else
#  define icmpv6_rwait(n,t) (0)
#endif

/****************************************************************************
 * Function: icmpv6_rnotify
 *
 * Description:
 *   Called each time that a Router Advertisement is received in order to
 *   wake-up any threads that may be waiting for this particular Router
 *   Advertisement.
 *
 *   NOTE:  On success the network has the new address applied and is in
 *   the down state.
 *
 * Assumptions:
 *   This function is called from the MAC device driver indirectly through
 *   icmpv6_icmpv6in() will execute with the network locked.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_ICMPv6_AUTOCONF
void icmpv6_rnotify(FAR struct net_driver_s *dev, const net_ipv6addr_t draddr,
                    const net_ipv6addr_t prefix, unsigned int preflen);
#else
#  define icmpv6_rnotify(d,p,l)
#endif

#endif /* CONFIG_NET_ICMPv6 */
#endif /* __NET_ICMPv6_ICMPv6_H */
