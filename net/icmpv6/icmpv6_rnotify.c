/****************************************************************************
 * net/icmpv6/icmpv6_rnotify.c
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

#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <netinet/in.h>

#include <nuttx/irq.h>
#include <nuttx/net/net.h>
#include <nuttx/net/netdev.h>

#include "netdev/netdev.h"
#include "utils/utils.h"
#include "icmpv6/icmpv6.h"

#ifdef CONFIG_NET_ICMPv6_AUTOCONF

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* List of tasks waiting for Neighbor Discover events */

static struct icmpv6_rnotify_s *g_icmpv6_rwaiters;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: icmpv6_setaddresses
 *
 * Description:
 *   We successfully obtained the Router Advertisement.  Set the new IPv6
 *   addresses in the driver structure.
 *
 ****************************************************************************/

void icmpv6_setaddresses(FAR struct net_driver_s *dev,
                         const net_ipv6addr_t draddr,
                         const net_ipv6addr_t prefix,
                         unsigned int preflen)
{
  unsigned int i;

  /* Lock the network.
   *
   * NOTE:  Normally it is required that the network be in the "down" state
   * when re-configuring the network interface.  This is thought not to be
   * a problem here because.
   *
   *   1. The ICMPv6 logic here runs with the network locked so there can be
   *      no outgoing packets with bad source IP addresses from any
   *      asynchronous network activity using the device being reconfigured.
   *   2. Incoming packets depend only upon the MAC filtering.  Network
   *      drivers do not use the IP address; they filter incoming packets
   *      using only the MAC address which is not being changed here.
   */

  net_lock();

  /* Create an address mask from the prefix */

  if (preflen > 128)
    {
      preflen = 128;
    }

  net_ipv6_pref2mask(preflen, dev->d_ipv6netmask);

  ninfo("preflen=%d netmask=%04x:%04x:%04x:%04x:%04x:%04x:%04x:%04x\n",
        preflen, NTOHS(dev->d_ipv6netmask[0]), NTOHS(dev->d_ipv6netmask[1]),
        NTOHS(dev->d_ipv6netmask[2]), NTOHS(dev->d_ipv6netmask[3]),
        NTOHS(dev->d_ipv6netmask[4]), NTOHS(dev->d_ipv6netmask[5]),
        NTOHS(dev->d_ipv6netmask[6]), NTOHS(dev->d_ipv6netmask[7]));

  /* Copy prefix to the current IPv6 address, applying the mask */

  for (i = 0; i < 7; i++)
    {
      dev->d_ipv6addr[i] = (dev->d_ipv6addr[i] & ~dev->d_ipv6netmask[i]) |
                           (prefix[i] & dev->d_ipv6netmask[i]);
    }

  ninfo("prefix=%04x:%04x:%04x:%04x:%04x:%04x:%04x:%04x\n",
        NTOHS(prefix[0]), NTOHS(prefix[1]), NTOHS(prefix[2]),
        NTOHS(prefix[3]), NTOHS(prefix[4]), NTOHS(prefix[5]),
        NTOHS(prefix[6]), NTOHS(prefix[7]));
  ninfo("IP address=%04x:%04x:%04x:%04x:%04x:%04x:%04x:%04x\n",
        NTOHS(dev->d_ipv6addr[0]), NTOHS(dev->d_ipv6addr[1]),
        NTOHS(dev->d_ipv6addr[2]), NTOHS(dev->d_ipv6addr[3]),
        NTOHS(dev->d_ipv6addr[4]), NTOHS(dev->d_ipv6addr[5]),
        NTOHS(dev->d_ipv6addr[6]), NTOHS(dev->d_ipv6addr[7]));

  /* Finally, copy the router address */

  net_ipv6addr_copy(dev->d_ipv6draddr, draddr);

  ninfo("DR address=%04x:%04x:%04x:%04x:%04x:%04x:%04x:%04x\n",
        NTOHS(dev->d_ipv6draddr[0]), NTOHS(dev->d_ipv6draddr[1]),
        NTOHS(dev->d_ipv6draddr[2]), NTOHS(dev->d_ipv6draddr[3]),
        NTOHS(dev->d_ipv6draddr[4]), NTOHS(dev->d_ipv6draddr[5]),
        NTOHS(dev->d_ipv6draddr[6]), NTOHS(dev->d_ipv6draddr[7]));

  net_unlock();
}

/****************************************************************************
 * Name: icmpv6_rwait_setup
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

void icmpv6_rwait_setup(FAR struct net_driver_s *dev,
                        FAR struct icmpv6_rnotify_s *notify)
{
  irqstate_t flags;

  /* Initialize the wait structure */

  memcpy(notify->rn_ifname, dev->d_ifname, IFNAMSIZ);
  notify->rn_result = -ETIMEDOUT;

  nxsem_init(&notify->rn_sem, 0, 0);

  /* Add the wait structure to the list with interrupts disabled */

  flags             = enter_critical_section();
  notify->rn_flink  = g_icmpv6_rwaiters;
  g_icmpv6_rwaiters  = notify;
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: icmpv6_rwait_cancel
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

int icmpv6_rwait_cancel(FAR struct icmpv6_rnotify_s *notify)
{
  FAR struct icmpv6_rnotify_s *curr;
  FAR struct icmpv6_rnotify_s *prev;
  irqstate_t flags;
  int ret = -ENOENT;

  ninfo("Canceling...\n");

  /* Remove our wait structure from the list (we may no longer be at the
   * head of the list).
   */

  flags = enter_critical_section();
  for (prev = NULL, curr = g_icmpv6_rwaiters;
       curr && curr != notify;
       prev = curr, curr = curr->rn_flink)
    {
    }

  DEBUGASSERT(curr && curr == notify);
  if (curr)
    {
      if (prev)
        {
          prev->rn_flink = notify->rn_flink;
        }
      else
        {
          g_icmpv6_rwaiters = notify->rn_flink;
        }

      ret = OK;
    }

  leave_critical_section(flags);
  nxsem_destroy(&notify->rn_sem);
  return ret;
}

/****************************************************************************
 * Name: icmpv6_rwait
 *
 * Description:
 *   Called each time that a Router Solicitation is sent.  This function
 *   will sleep until either: (1) the matching Router Advertisement is
 *   received, or (2) a timeout occurs.
 *
 * Assumptions:
 *   This function is called from icmpv6_autoconfig() and must execute with
 *   the network locked.
 *
 ****************************************************************************/

int icmpv6_rwait(FAR struct icmpv6_rnotify_s *notify, unsigned int timeout)
{
  int ret;

  ninfo("Waiting...\n");

  /* And wait for the Neighbor Advertisement (or a timeout). */

  ret = net_sem_timedwait(&notify->rn_sem, timeout);
  if (ret >= 0)
    {
      ret = notify->rn_result;
    }

  /* Remove our wait structure from the list (we may no longer be at the
   * head of the list).
   */

  icmpv6_rwait_cancel(notify);
  return ret;
}

/****************************************************************************
 * Name: icmpv6_rnotify
 *
 * Description:
 *   Called each time that a Router Advertisement is received in order to
 *   wake-up any threads that may be waiting for this particular Router
 *   Advertisement.
 *
 * Assumptions:
 *   This function is called from the MAC device driver indirectly through
 *   icmpv6_icmpv6in() will execute with the network locked.
 *
 ****************************************************************************/

void icmpv6_rnotify(FAR struct net_driver_s *dev)
{
  FAR struct icmpv6_rnotify_s *curr;

  ninfo("Notified\n");

  /* Find an entry with the matching device name in the list of waiters */

  for (curr = g_icmpv6_rwaiters; curr; curr = curr->rn_flink)
    {
      /* Does this entry match?  If the result is okay, then we have
       * already notified this waiter and it has not yet taken the
       * entry from the list.
       */

      if (curr->rn_result != OK &&
          strncmp(curr->rn_ifname, dev->d_ifname, IFNAMSIZ) == 0)
        {
          /* And signal the waiting, returning success */

          curr->rn_result = OK;
          nxsem_post(&curr->rn_sem);
          break;
        }
    }
}

#endif /* CONFIG_NET_ICMPv6_AUTOCONF */
