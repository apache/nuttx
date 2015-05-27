/****************************************************************************
 * net/icmpv6/icmpv6_autoconfig.c
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

#include <stdint.h>
#include <string.h>
#include <time.h>
#include <errno.h>
#include <debug.h>

#include <net/ethernet.h>

#include <nuttx/net/net.h>
#include <nuttx/net/netdev.h>

#include "devif/devif.h"
#include "netdev/netdev.h"
#include "icmpv6/icmpv6.h"

#ifdef CONFIG_NET_ICMPv6_AUTOCONF

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CONFIG_ICMPv6_AUTOCONF_DELAYSEC  \
  (CONFIG_ICMPv6_AUTOCONF_DELAYMSEC / 1000)
#define CONFIG_ICMPv6_AUTOCONF_DELAYNSEC \
  ((CONFIG_ICMPv6_AUTOCONF_DELAYMSEC - 1000*CONFIG_ICMPv6_AUTOCONF_DELAYSEC) * 1000000)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure holds the state of the send operation until it can be
 * operated upon from the interrupt level.
 */

struct icmpv6_router_s
{
  FAR struct devif_callback_s *snd_cb; /* Reference to callback instance */
  sem_t snd_sem;                       /* Used to wake up the waiting thread */
  volatile bool snd_sent;              /* True: if request sent */
  bool snd_advertise;                  /* True: Send Neighbor Advertisement */
#ifdef CONFIG_NETDEV_MULTINIC
  uint8_t snd_ifname[IFNAMSIZ];        /* Interface name */
#endif
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

  sem_post(&state->snd_sem);
}

/****************************************************************************
 * Name: icmpv6_router_interrupt
 ****************************************************************************/

static uint16_t icmpv6_router_interrupt(FAR struct net_driver_s *dev,
                                        FAR void *pvconn,
                                        FAR void *priv, uint16_t flags)
{
  FAR struct icmpv6_router_s *state = (FAR struct icmpv6_router_s *)priv;

  nllvdbg("flags: %04x sent: %d\n", flags, state->snd_sent);

  if (state)
    {
      /* Check if the network is still up */

      if ((flags & NETDEV_DOWN) != 0)
        {
          nlldbg("ERROR: Interface is down\n");
          icmpv6_router_terminate(state, -ENETUNREACH);
          return flags;
        }

      /* Check if the outgoing packet is available. It may have been claimed
       * by a send interrupt serving a different thread -OR- if the output
       * buffer currently contains unprocessed incoming data. In these cases
       * we will just have to wait for the next polling cycle.
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

      /* It looks like we are good to send the data */
      /* Copy the packet data into the device packet buffer and send it */

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

      /* Make sure no additional Router Solicitation overwrites this one.
       * This flag will be cleared in icmpv6_out().
       */

      IFF_SET_NOARP(dev->d_flags);

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
 * Parameters:
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

  /* Initialize the state structure. This is done with interrupts
   * disabled
   */

  (void)sem_init(&state.snd_sem, 0, 0); /* Doesn't really fail */

#ifdef CONFIG_NETDEV_MULTINIC
  /* Remember the routing device name */

  strncpy((FAR char *)state.snd_ifname, (FAR const char *)dev->d_ifname,
          IFNAMSIZ);
#endif

  /* Allocate resources to receive a callback.  This and the following
   * initialization is performed with the network lock because we don't
   * want anything to happen until we are ready.
   */

  state.snd_cb = icmpv6_callback_alloc(dev);
  if (!state.snd_cb)
    {
      ndbg("ERROR: Failed to allocate a cllback\n");
      ret = -ENOMEM;
      goto errout_with_semaphore;
    }

  /* Arm the callback */

  state.snd_sent      = false;
  state.snd_result    = -EBUSY;
  state.snd_advertise = advertise;
  state.snd_cb->flags = (ICMPv6_POLL | NETDEV_DOWN);
  state.snd_cb->priv  = (FAR void *)&state;
  state.snd_cb->event = icmpv6_router_interrupt;

  /* Notify the device driver that new TX data is available. */

  dev->d_txavail(dev);

  /* Wait for the send to complete or an error to occur: NOTES: (1)
   * net_lockedwait will also terminate if a signal is received, (2)
   * interrupts may be disabled! They will be re-enabled while the
   * task sleeps and automatically re-enabled when the task restarts.
   */

  do
    {
      (void)net_lockedwait(&state.snd_sem);
    }
  while (!state.snd_sent);

  ret = state.snd_result;
  icmpv6_callback_free(dev, state.snd_cb);

errout_with_semaphore:
  sem_destroy(&state.snd_sem);
  return ret;
}

/****************************************************************************
 * Name: icmpv6_wait_radvertise
 *
 * Description:
 *   Wait for the receipt of the Router Advertisement matching the Router
 *   Solicitation that we just sent.
 *
 * Parameters:
 *   dev    - The device to use to send the solicitation
 *   notify - The pre-initialized notification structure
 *   save   - We will need this to temporarily release the net lock
 *
 * Returned Value:
 *   Zero (OK) is returned on success; On error a negated errno value is
 *   returned.
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

static int icmpv6_wait_radvertise(FAR struct net_driver_s *dev,
                                  FAR struct icmpv6_rnotify_s *notify,
                                  net_lock_t *save)
{
  struct timespec delay;
  int ret;

  /* Wait for response to the Router Advertisement to be received.  The
   * optimal delay would be the work case round trip time.
   * NOTE: The network is locked.
   */

  delay.tv_sec  = CONFIG_ICMPv6_AUTOCONF_DELAYSEC;
  delay.tv_nsec = CONFIG_ICMPv6_AUTOCONF_DELAYNSEC;

  ret = icmpv6_rwait(notify, &delay);

 /* icmpv6_wait will return OK if and only if the matching Router
  * Advertisement is received.  Otherwise, it will return -ETIMEDOUT.
  */

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
 *   address from the underlying data link layer address. The general idea
 *   is to have a device generate a temporary address until it can determine
 *   the characteristics of the network it is on, and then create a permanent
 *   address it can use based on that information.
 *
 * Parameters:
 *   dev - The device driver structure to assign the address to
 *
 * Return:
 *   Zero (OK) is returned on success; A negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

int icmpv6_autoconfig(FAR struct net_driver_s *dev)
{
#ifndef CONFIG_NET_ETHERNET
  /* Only Ethernet supported for now */

  ndbg("ERROR: Only Ethernet is supported\n");
  return -ENOSYS;

#else /* CONFIG_NET_ETHERNET */
  struct icmpv6_rnotify_s notify;
  net_ipv6addr_t lladdr;
  net_lock_t save;
  int retries;
  int ret;

  /* Sanity checks */

  DEBUGASSERT(dev);
  nvdbg("Auto-configuring %s\n", dev->d_ifname);

#ifdef CONFIG_NET_MULTILINK
  /* Only Ethernet devices are supported for now */

  if (dev->d_lltype != NET_LL_ETHERNET)
    {
      ndbg("ERROR: Only Ethernet is supported\n");
      return -ENOSYS;
    }
#endif

  /* The interface should be in the down state */

 save = net_lock();
 netdev_ifdown(dev);
 net_unlock(save);

  /* IPv6 Stateless Autoconfiguration
   * Reference: http://www.tcpipguide.com/free/t_IPv6AutoconfigurationandRenumbering.htm
   *
   * The following is a summary of the steps a device takes when using
   * stateless auto-configuration:
   *
   * 1. Link-Local Address Generation: The device generates a link-local
   *    address. Recall that this is one of the two types of local-use IPv6
   *    addresses. Link-local addresses have "1111 1110 10" for the first
   *    ten bits. The generated address uses those ten bits followed by 54
   *    zeroes and then the 64 bit interface identifier. Typically this
   *    will be derived from the data link layer (MAC) address.
   *
   *    IEEE 802 MAC addresses, used by Ethernet and other IEEE 802 Project
   *    networking technologies, have 48 bits.  The IEEE has also defined a
   *    format called the 64-bit extended unique identifier, abbreviated
   *    EUI-64.  To get the modified EUI-64 interface ID for a device, you
   *    simply take the EUI-64 address and change the 7th bit from the left
   *    (the"universal/local" or "U/L" bit) from a zero to a one.
   *
   *    128  112  96   80    64   48   32   16
   *    ---- ---- ---- ----  ---- ---- ---- ----
   *    fe80 0000 0000 0000  0000 xxxx xxxx xxxx
   */

  lladdr[0] = HTONS(0xfe80);                        /* 10-bit address + 6 zeroes */
  memset(&lladdr[1], 0, 4* sizeof(uint16_t));       /* 64 more zeroes */
  memcpy(&lladdr[5], dev->d_mac.ether_addr_octet,
        sizeof(struct ether_addr));                 /* 48-bit Ethernet address */

  nvdbg("lladdr=%04x:%04x:%04x:%04x:%04x:%04x:%04x:%04x\n",
        lladdr[0], lladdr[1], lladdr[2], lladdr[3],
        lladdr[4], lladdr[6], lladdr[6], lladdr[7]);

#ifdef CONFIG_NET_ICMPv6_NEIGHBOR
  /* Bring the interface up with no IP address */

  save = net_lock();
  netdev_ifup(dev);
  net_unlock(save);

  /* 2. Link-Local Address Uniqueness Test: The node tests to ensure that
   *    the address it generated isn't for some reason already in use on the
   *    local network. (This is very unlikely to be an issue if the link-local
   *    address came from a MAC address but more likely if it was based on a
   *    generated token.) It sends a Neighbor Solicitation message using the
   *    Neighbor Discovery (ND) protocol. It then listens for a Neighbor
   *    Advertisement in response that indicates that another device is
   *    already using its link-local address; if so, either a new address
   *    must be generated, or auto-configuration fails and another method
   *    must be employed.
   */

  ret = icmpv6_neighbor(lladdr);

  /* Take the interface back down */

  save = net_lock();
  netdev_ifdown(dev);
  net_unlock(save);

  if (ret == OK)
    {
      /* Hmmm... someone else responded to our Neighbor Solicitation.  We
       * have not back-up plan in place.  Just bail.
       */

      ndbg("ERROR: IP conflict\n");
      return -EEXIST;
    }
#endif

  /* 3. Link-Local Address Assignment: Assuming the uniqueness test passes,
   *    the device assigns the link-local address to its IP interface. This
   *    address can be used for communication on the local network, but not
   *    on the wider Internet (since link-local addresses are not routed).
   */

  save = net_lock();
  net_ipv6addr_copy(dev->d_ipv6addr, lladdr);

  /* Bring the interface up with the new, temporary IP address */

  netdev_ifup(dev);

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
          ndbg("ERROR: Failed send router solicitation: %d\n", ret);
          break;
        }

      /* Wait to receive the Router Advertisement message */

      ret = icmpv6_wait_radvertise(dev, &notify, &save);
      if (ret != -ETIMEDOUT)
        {
          /* ETIMEDOUT is the only expected failure.  We will retry on that
           * case only.
           */

          break;
        }

      nvdbg("Timed out... retrying %d\n", retries + 1);
    }

  /* Check for failures.  Note:  On successful return, the network will be 
   * in the down state, but not in the event of failures.
   */

  if (ret < 0)
    {
      ndbg("ERROR: Failed to get the router advertisement: %d (retries=%d)\n",
           ret, retries);

      /* Claim the link local address as ours by sending the ICMPv6 Neighbor
       * Advertisement message.
       */

      ret = icmpv6_send_message(dev, true);
      if (ret < 0)
        {
          ndbg("ERROR: Failed send neighbor advertisement: %d\n", ret);
          netdev_ifdown(dev);
        }

      /* No off-link communications; No router address. */

      net_ipv6addr_copy(dev->d_ipv6draddr, g_ipv6_allzeroaddr);

      /* Set a netmask for the local link address */

      net_ipv6addr_copy(dev->d_ipv6netmask, g_ipv6_llnetmask);

      /* Leave the network up and return success (even though things did not
       * work out quite the way we wanted).
       */

      net_unlock(save);
      return ret;
    }

  /* 5. Router Direction: The router provides direction to the node on how to
   *    proceed with the auto-configuration. It may tell the node that on this
   *    network "stateful" auto-configuration is in use, and tell it the
   *    address of a DHCP server to use. Alternately, it will tell the host
   *    how to determine its global Internet address.
   *
   * 6. Global Address Configuration: Assuming that stateless auto-
   *    configuration is in use on the network, the host will configure
   *    itself with its globally-unique Internet address. This address is
   *    generally formed from a network prefix provided to the host by the
   *    router, combined with the device's identifier as generated in the
   *    first step.
   */

   /* On success, the new address was already set (in icmpv_rnotify()).  We
    * need only to bring the network back to the up state and return success.
    */

  netdev_ifup(dev);
  net_unlock(save);
  return OK;
#endif /* CONFIG_NET_ETHERNET */
}

#endif /* CONFIG_NET_ICMPv6_AUTOCONF */
