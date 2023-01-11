/****************************************************************************
 * net/icmpv6/icmpv6_neighbor.c
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

#include <unistd.h>
#include <string.h>
#include <debug.h>

#include <netinet/in.h>
#include <net/if.h>

#include <nuttx/semaphore.h>
#include <nuttx/net/net.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/ip.h>
#include <nuttx/net/icmpv6.h>

#include "netdev/netdev.h"
#include "devif/devif.h"
#include "inet/inet.h"
#include "neighbor/neighbor.h"
#include "route/route.h"
#include "icmpv6/icmpv6.h"

#ifdef CONFIG_NET_ICMPv6_NEIGHBOR

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure holds the state of the send operation until it can be
 * operated upon from the event handler.
 */

struct icmpv6_neighbor_s
{
  FAR struct devif_callback_s *snd_cb; /* Reference to callback instance */
  sem_t snd_sem;                       /* Used to wake up the waiting thread */
  uint8_t snd_retries;                 /* Retry count */
  volatile bool snd_sent;              /* True: if request sent */
  uint8_t snd_ifname[IFNAMSIZ];        /* Interface name */
  net_ipv6addr_t snd_ipaddr;           /* The IPv6 address to be queried */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: icmpv6_neighbor_eventhandler
 ****************************************************************************/

static uint16_t icmpv6_neighbor_eventhandler(FAR struct net_driver_s *dev,
                                             FAR void *priv, uint16_t flags)
{
  FAR struct icmpv6_neighbor_s *state = (FAR struct icmpv6_neighbor_s *)priv;

  ninfo("flags: %04x sent: %d\n", flags, state->snd_sent);

  if (state)
    {
      /* Is this the device that we need to route this request? */

      if (strncmp((FAR const char *)dev->d_ifname,
                  (FAR const char *)state->snd_ifname, IFNAMSIZ) != 0)
        {
          /* No... pass on this one and wait for the device that we want */

          return flags;
        }

      /* Check if the outgoing packet is available. It may have been claimed
       * by a send event handler serving a different thread -OR- if the
       * output buffer currently contains unprocessed incoming data.  In
       * these cases we will just have to wait for the next polling cycle.
       */

      if (dev->d_sndlen > 0 || (flags & ICMPv6_NEWDATA) != 0)
        {
          /* Another thread has beat us sending data or the buffer is busy,
           * Check for a timeout. If not timed out, wait for the next
           * polling cycle and check again.
           */

          /* REVISIT: No timeout. Just wait for the next polling cycle */

          return flags;
        }

      /* Prepare device buffer */

      if (netdev_iob_prepare(dev, false, 0) != OK)
        {
          return flags;
        }

      /* It looks like we are good to send the data.
       *
       * Copy the packet data into the device packet buffer and send it.
       */

      icmpv6_solicit(dev, state->snd_ipaddr);

      IFF_SET_IPv6(dev->d_flags);

      /* Don't allow any further call backs. */

      state->snd_sent         = true;
      state->snd_cb->flags    = 0;
      state->snd_cb->priv     = NULL;
      state->snd_cb->event    = NULL;

      /* Wake up the waiting thread */

      nxsem_post(&state->snd_sem);
    }

  return flags;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

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
 * Input Parameters:
 *   ipaddr   The IPv6 address to be queried.
 *
 * Returned Value:
 *   Zero (OK) is returned on success and the IP address mapping can now be
 *   found in the Neighbor Table.  On error a negated errno value is
 *   returned:
 *
 *     -ETIMEDOUT:    The number or retry counts has been exceed.
 *     -EHOSTUNREACH: Could not find a route to the host
 *
 * Assumptions:
 *   This function is called from the normal tasking context.
 *
 ****************************************************************************/

int icmpv6_neighbor(const net_ipv6addr_t ipaddr)
{
  FAR struct net_driver_s *dev;
  struct icmpv6_notify_s notify;
  struct icmpv6_neighbor_s state;
  net_ipv6addr_t lookup;
  int ret;

  /* First check if destination is a local broadcast or a multicast address.
   *
   * - IPv6 multicast addresses are have the high-order octet of the
   *   addresses=0xff (ff00::/8.)
   */

  if (net_ipv6addr_cmp(ipaddr, g_ipv6_unspecaddr) ||
      net_is_addr_mcast(ipaddr))
    {
      /* We don't need to send the Neighbor Solicitation.  But for the case
       * of the Multicast address, a routing able entry will be required.
       */

      return OK;
    }

  /* Get the device that can route this request */

  dev = netdev_findby_ripv6addr(g_ipv6_unspecaddr, ipaddr);
  if (!dev)
    {
      nerr("ERROR: Unreachable: %08lx\n", (unsigned long)ipaddr);
      ret = -EHOSTUNREACH;
      goto errout;
    }

  /* Check if the destination address is on the local network. */

  if (net_ipv6addr_maskcmp(ipaddr, dev->d_ipv6addr, dev->d_ipv6netmask))
    {
      /* Yes.. use the input address for the lookup */

      net_ipv6addr_copy(lookup, ipaddr);
    }
  else
    {
      /* Destination address is not on the local network */

#ifdef CONFIG_NET_ROUTE

      /* We have a routing table.. find the correct router to use in
       * this case (or, as a fall-back, use the device's default router
       * address).  We will use the router IP address instead of the
       * destination address when determining the MAC address.
       */

      netdev_ipv6_router(dev, ipaddr, lookup);
#else
      /* Use the device's default router IP address instead of the
       * destination address when determining the MAC address.
       */

      net_ipv6addr_copy(lookup, dev->d_ipv6draddr);
#endif
    }

  /* Allocate resources to receive a callback.  This and the following
   * initialization is performed with the network lock because we don't
   * want anything to happen until we are ready.
   */

  net_lock();
  state.snd_cb = devif_callback_alloc((dev),
                                      &(dev)->d_conncb,
                                      &(dev)->d_conncb_tail);
  if (!state.snd_cb)
    {
      nerr("ERROR: Failed to allocate a callback\n");
      ret = -ENOMEM;
      goto errout_with_lock;
    }

  nxsem_init(&state.snd_sem, 0, 0);        /* Doesn't really fail */

  state.snd_retries = 0;                       /* No retries yet */
  net_ipv6addr_copy(state.snd_ipaddr, lookup); /* IP address to query */

  /* Remember the routing device name */

  strlcpy((FAR char *)state.snd_ifname, (FAR const char *)dev->d_ifname,
          IFNAMSIZ);

  /* Now loop, testing if the address mapping is in the Neighbor Table and
   * re-sending the Neighbor Solicitation if it is not.
   */

  ret = -ETIMEDOUT; /* Assume a timeout failure */

  while (state.snd_retries < CONFIG_ICMPv6_NEIGHBOR_MAXTRIES)
    {
      /* Check if the address mapping is present in the Neighbor Table.  This
       * is only really meaningful on the first time through the loop.
       *
       * NOTE: If the Neighbor Table is large than this could be a
       * performance issue.
       */

      if (neighbor_lookup(lookup, NULL) >= 0)
        {
          /* We have it!  Break out with success */

          ret = OK;
          break;
        }

      /* Set up the Neighbor Advertisement wait BEFORE we send the Neighbor
       * Solicitation.
       */

      icmpv6_wait_setup(lookup, &notify);

      /* Arm/re-arm the callback */

      state.snd_sent      = false;
      state.snd_cb->flags = ICMPv6_POLL;
      state.snd_cb->priv  = (FAR void *)&state;
      state.snd_cb->event = icmpv6_neighbor_eventhandler;

      /* Notify the device driver that new TX data is available. */

      netdev_txnotify_dev(dev);

      /* Wait for the send to complete or an error to occur.
       * net_lockedwait will also terminate if a signal is received.
       */

      do
        {
          net_lockedwait(&state.snd_sem);
        }
      while (!state.snd_sent);

      /* Now wait for response to the Neighbor Advertisement to be
       * received.
       */

      ret = icmpv6_wait(&notify, CONFIG_ICMPv6_NEIGHBOR_DELAYMSEC);

      /* icmpv6_wait will return OK if and only if the matching Neighbor
       * Advertisement is received.  Otherwise, it will return -ETIMEDOUT.
       */

      if (ret == OK)
        {
          break;
        }

      /* Increment the retry count */

      state.snd_retries++;
    }

  nxsem_destroy(&state.snd_sem);
  devif_dev_callback_free(dev, state.snd_cb);

errout_with_lock:
  net_unlock();

errout:
  return ret;
}

#endif /* CONFIG_NET_ICMPv6_NEIGHBOR */
