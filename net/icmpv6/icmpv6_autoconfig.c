/****************************************************************************
 * net/icmpv6/icmpv6_autoconfig.c
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

#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <arpa/inet.h>

#include <nuttx/semaphore.h>
#include <nuttx/net/net.h>
#include <nuttx/net/netdev.h>

#include "devif/devif.h"
#include "netdev/netdev.h"
#include "inet/inet.h"
#include "icmpv6/icmpv6.h"

#ifdef CONFIG_NET_ICMPv6_AUTOCONF

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure holds the state of the send operation until it can be
 * operated upon by the event handler.
 */

struct icmpv6_router_s
{
  FAR struct devif_callback_s *snd_cb; /* Reference to callback instance */
  sem_t snd_sem;                       /* Used to wake up the waiting thread */
  volatile bool snd_sent;              /* True: if request sent */
  bool snd_advertise;                  /* True: Send Neighbor Advertisement */
  uint8_t snd_ifname[IFNAMSIZ];        /* Interface name */
  int16_t snd_result;                  /* Result of the send */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: icmpv6_router_terminate
 ****************************************************************************/

static void icmpv6_router_terminate(FAR struct icmpv6_router_s *state,
                                    int result)
{
  /* Don't allow any further call backs. */

  state->snd_sent         = true;
  state->snd_result       = (int16_t)result;
  state->snd_cb->flags    = 0;
  state->snd_cb->priv     = NULL;
  state->snd_cb->event    = NULL;

  /* Wake up the waiting thread */

  nxsem_post(&state->snd_sem);
}

/****************************************************************************
 * Name: icmpv6_router_eventhandler
 ****************************************************************************/

static uint16_t icmpv6_router_eventhandler(FAR struct net_driver_s *dev,
                                           FAR void *priv, uint16_t flags)
{
  FAR struct icmpv6_router_s *state = (FAR struct icmpv6_router_s *)priv;

  ninfo("flags: %04x sent: %d\n", flags, state->snd_sent);

  if (state)
    {
      /* Check if the network is still up */

      if ((flags & NETDEV_DOWN) != 0)
        {
          nerr("ERROR: Interface is down\n");
          icmpv6_router_terminate(state, -ENETUNREACH);
          return flags;
        }

      /* Check if the outgoing packet is available. It may have been claimed
       * by a send event handler serving a different thread -OR- if the
       * output buffer currently contains unprocessed incoming data. In
       * these cases we will just have to wait for the next polling cycle.
       */

      else if (dev->d_sndlen > 0 || (flags & ICMPv6_NEWDATA) != 0)
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

      if (state->snd_advertise)
        {
          /* Send the ICMPv6 Neighbor Advertisement message */

          icmpv6_advertise(dev, g_ipv6_allnodes);
        }
      else
        {
          /* Send the ICMPv6 Router Solicitation message */

          icmpv6_rsolicit(dev);
        }

      IFF_SET_IPv6(dev->d_flags);

      /* Don't allow any further call backs. */

      icmpv6_router_terminate(state, OK);
    }

  return flags;
}

/****************************************************************************
 * Name: icmpv6_send_message
 *
 * Description:
 *   Send an ICMPv6 Router Solicitation to resolve an IPv6 address.
 *
 * Input Parameters:
 *   dev       - The device to use to send the solicitation
 *   advertise - True: Send the Neighbor Advertisement message
 *
 * Returned Value:
 *   Zero (OK) is returned on success; On error a negated errno value is
 *   returned.
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

static int icmpv6_send_message(FAR struct net_driver_s *dev, bool advertise)
{
  struct icmpv6_router_s state;
  int ret;

  /* Initialize the state structure with the network locked. */

  nxsem_init(&state.snd_sem, 0, 0); /* Doesn't really fail */

  /* Remember the routing device name */

  strlcpy((FAR char *)state.snd_ifname, (FAR const char *)dev->d_ifname,
          IFNAMSIZ);

  /* Allocate resources to receive a callback.  This and the following
   * initialization is performed with the network lock because we don't
   * want anything to happen until we are ready.
   */

  state.snd_cb = devif_callback_alloc(dev,
                                      &dev->d_conncb,
                                      &dev->d_conncb_tail);
  if (!state.snd_cb)
    {
      nerr("ERROR: Failed to allocate a cllback\n");
      ret = -ENOMEM;
      goto errout_with_semaphore;
    }

  /* Arm the callback */

  state.snd_sent      = false;
  state.snd_result    = -EBUSY;
  state.snd_advertise = advertise;
  state.snd_cb->flags = (ICMPv6_POLL | NETDEV_DOWN);
  state.snd_cb->priv  = (FAR void *)&state;
  state.snd_cb->event = icmpv6_router_eventhandler;

  /* Notify the device driver that new TX data is available. */

  netdev_txnotify_dev(dev);

  /* Wait for the send to complete or an error to occur
   * net_lockedwait will also terminate if a signal is received.
   */

  do
    {
      net_lockedwait(&state.snd_sem);
    }
  while (!state.snd_sent);

  ret = state.snd_result;
  devif_dev_callback_free(dev, state.snd_cb);

errout_with_semaphore:
  nxsem_destroy(&state.snd_sem);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: icmpv6_autoconfig
 *
 * Description:
 *   Perform IPv6 auto-configuration to assign an IPv6 address to this
 *   device.
 *
 *   Stateless auto-configuration exploits several other features in IPv6,
 *   including link-local addresses, multi-casting, the Neighbor Discovery
 *   protocol, and the ability to generate the interface identifier of an
 *   address from the underlying link layer address. The general idea is
 *   to have a device generate a temporary address until it can determine
 *   the characteristics of the network it is on, and then create a permanent
 *   address it can use based on that information.
 *
 * Input Parameters:
 *   dev   - The device driver structure to assign the address to
 *   psock - A pointer to a NuttX-specific, internal socket structure
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

int icmpv6_autoconfig(FAR struct net_driver_s *dev)
{
  struct icmpv6_rnotify_s notify;
  net_ipv6addr_t lladdr;
  int retries;
  int ret;

  /* Sanity checks */

  DEBUGASSERT(dev);
  ninfo("Auto-configuring %s\n", dev->d_ifname);

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

  /* IPv6 Stateless Autoconfiguration
   * Reference:
   * http://www.tcpipguide.com/free/t_IPv6AutoconfigurationandRenumbering.htm
   *
   * The following is a summary of the steps a device takes when using
   * stateless auto-configuration:
   *
   * 1. Link-Local Address Generation: The device generates a link-local
   *    address. Recall that this is one of the two types of local-use IPv6
   *    addresses. Link-local addresses have "1111 1110 10" for the first
   *    ten bits. The generated address uses those ten bits followed by 54
   *    zeroes and then the 64 bit interface identifier. Typically this
   *    will be derived from the link layer (MAC) address.
   */

  icmpv6_linkipaddr(dev, lladdr);

  ninfo("lladdr=%04x:%04x:%04x:%04x:%04x:%04x:%04x:%04x\n",
        NTOHS(lladdr[0]), NTOHS(lladdr[1]),
        NTOHS(lladdr[2]), NTOHS(lladdr[3]),
        NTOHS(lladdr[4]), NTOHS(lladdr[5]),
        NTOHS(lladdr[6]), NTOHS(lladdr[7]));

#ifdef CONFIG_NET_ICMPv6_NEIGHBOR
  /* 2. Link-Local Address Uniqueness Test:  The node tests to ensure that
   *    the address it generated isn't for some reason already in use on the
   *    local network. (This is very unlikely to be an issue if the link-
   *    local address came from a MAC address but more likely if it was
   *    based on a generated token.) It sends a Neighbor Solicitation
   *    message using the Neighbor Discovery (ND) protocol. It then listens
   *    for a Neighbor Advertisement in response that indicates that another
   *    device is already using its link-local address; if so, either a new
   *    address  must be generated, or auto-configuration fails and another
   *    method must be employed.
   */

  ret = icmpv6_neighbor(lladdr);
  if (ret >= 0)
    {
      /* Hmmm... someone else responded to our Neighbor Solicitation.  We
       * have no back-up plan in place.  Just bail.
       */

      nerr("ERROR: IP conflict\n");

      net_unlock();
      return -EEXIST;
    }
#endif

  /* 3. Link-Local Address Assignment: Assuming the uniqueness test passes,
   *    the device assigns the link-local address to its IP interface. This
   *    address can be used for communication on the local network, but not
   *    on the wider Internet (since link-local addresses are not routed).
   */

  net_ipv6addr_copy(dev->d_ipv6addr, lladdr);

  /* 4. Router Contact: The node next attempts to contact a local router for
   *    more information on continuing the configuration. This is done either
   *    by listening for Router Advertisement messages sent periodically by
   *    routers, or by sending a specific Router Solicitation to ask a router
   *    for information on what to do next.
   */

  for (retries = 0; retries < CONFIG_ICMPv6_AUTOCONF_MAXTRIES; retries++)
    {
      /* Set up the Router Advertisement BEFORE we send the Router
       * Solicitation.
       */

      icmpv6_rwait_setup(dev, &notify);

      /* Send the ICMPv6 Router solicitation message */

      ret = icmpv6_send_message(dev, false);
      if (ret < 0)
        {
          nerr("ERROR: Failed send router solicitation: %d\n", ret);
          break;
        }

      /* Wait to receive the Router Advertisement message */

      ret = icmpv6_rwait(&notify, CONFIG_ICMPv6_AUTOCONF_DELAYMSEC);
      if (ret != -ETIMEDOUT)
        {
          /* ETIMEDOUT is the only expected failure.  We will retry on that
           * case only.
           */

          break;
        }

      ninfo("Timed out... retrying %d\n", retries + 1);
    }

  /* Check for failures. */

  if (ret < 0)
    {
      int senderr;

      nerr("ERROR: Failed to get the router advertisement: "
           "%d (retries=%d)\n",
           ret, retries);

      /* Claim the link local address as ours by sending the ICMPv6 Neighbor
       * Advertisement message.
       */

      senderr = icmpv6_send_message(dev, true);
      if (senderr < 0)
        {
          nerr("ERROR: Failed send neighbor advertisement: %d\n", senderr);
        }

      /* No off-link communications; No router address. */

      net_ipv6addr_copy(dev->d_ipv6draddr, g_ipv6_unspecaddr);

      /* Set a netmask for the local link address */

      net_ipv6addr_copy(dev->d_ipv6netmask, g_ipv6_llnetmask);
    }

  /* 5. Router Direction: The router provides direction to the node on how
   *    to proceed with the auto-configuration. It may tell the node that on
   *    this network "stateful" auto-configuration is in use, and tell it
   *    the address of a DHCP server to use. Alternately, it will tell the
   *    host how to determine its global Internet address.
   *
   * 6. Global Address Configuration: Assuming that stateless auto-
   *    configuration is in use on the network, the host will configure
   *    itself with its globally-unique Internet address. This address is
   *    generally formed from a network prefix provided to the host by the
   *    router, combined with the device's identifier as generated in the
   *    first step.
   */

  /* On success, the new address was already set (in icmpv_rnotify()). */

  net_unlock();
  return ret;
}

#endif /* CONFIG_NET_ICMPv6_AUTOCONF */
