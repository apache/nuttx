/****************************************************************************
 * net/icmpv6/icmpv6_neighbor.c
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

#include <unistd.h>
#include <string.h>
#include <semaphore.h>
#include <time.h>
#include <debug.h>

#include <netinet/in.h>
#include <net/if.h>

#include <nuttx/net/net.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/ip.h>
#include <nuttx/net/icmpv6.h>

#include "netdev/netdev.h"
#include "devif/devif.h"
#include "neighbor/neighbor.h"
#include "icmpv6/icmpv6.h"

#ifdef CONFIG_NET_ICMPv6_NEIGHBOR

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CONFIG_ICMPv6_NEIGHBOR_DELAYSEC  \
  (CONFIG_ICMPv6_NEIGHBOR_DELAYMSEC / 1000)
#define CONFIG_ICMPv6_NEIGHBOR_DELAYNSEC \
  ((CONFIG_ICMPv6_NEIGHBOR_DELAYMSEC - 1000*CONFIG_ICMPv6_NEIGHBOR_DELAYSEC) * 1000000)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure holds the state of the send operation until it can be
 * operated upon from the interrupt level.
 */

struct icmpv6_neighbor_s
{
  FAR struct devif_callback_s *snd_cb; /* Reference to callback instance */
  sem_t snd_sem;                       /* Used to wake up the waiting thread */
  uint8_t snd_retries;                 /* Retry count */
  volatile bool snd_sent;              /* True: if request sent */
#ifdef CONFIG_NETDEV_MULTINIC
  uint8_t snd_ifname[IFNAMSIZ];        /* Interface name */
#endif
  net_ipv6addr_t snd_ipaddr;           /* The IPv6 address to be queried */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: icmpv6_neighbor_interrupt
 ****************************************************************************/

static uint16_t icmpv6_neighbor_interrupt(FAR struct net_driver_s *dev,
                                          FAR void *pvconn,
                                          FAR void *priv, uint16_t flags)
{
  FAR struct icmpv6_neighbor_s *state = (FAR struct icmpv6_neighbor_s *)priv;

  nllvdbg("flags: %04x sent: %d\n", flags, state->snd_sent);

  if (state)
    {
#ifdef CONFIG_NETDEV_MULTINIC
      /* Is this the device that we need to route this request? */

      if (strncmp((FAR const char *)dev->d_ifname,
                  (FAR const char *)state->snd_ifname, IFNAMSIZ) != 0)
        {
          /* No... pass on this one and wait for the device that we want */

          return flags;
        }

#endif

      /* Check if the outgoing packet is available. It may have been claimed
       * by a send interrupt serving a different thread -OR- if the output
       * buffer currently contains unprocessed incoming data. In these cases
       * we will just have to wait for the next polling cycle.
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

      /* It looks like we are good to send the data */
      /* Copy the packet data into the device packet buffer and send it */

      icmpv6_solicit(dev, state->snd_ipaddr);

      /* Make sure no additional Neighbor Solicitation overwrites this one.
       * This flag will be cleared in icmpv6_out().
       */

      IFF_SET_NOARP(dev->d_flags);

      /* Don't allow any further call backs. */

      state->snd_sent         = true;
      state->snd_cb->flags    = 0;
      state->snd_cb->priv     = NULL;
      state->snd_cb->event    = NULL;

      /* Wake up the waiting thread */

      sem_post(&state->snd_sem);
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

int icmpv6_neighbor(const net_ipv6addr_t ipaddr)
{
  FAR struct net_driver_s *dev;
  struct icmpv6_notify_s notify;
  struct timespec delay;
  struct icmpv6_neighbor_s state;
  FAR const uint16_t *lookup;
  net_lock_t save;
  int ret;

  /* First check if destination is a local broadcast or a multicast address.
   *
   * - IPv6 multicast addresses are have the high-order octet of the
   *   addresses=0xff (ff00::/8.)
   */

  if (net_ipv6addr_cmp(ipaddr, g_ipv6_allzeroaddr) ||
      (ipaddr[0] & NTOHS(0xff00)) == NTOHS(0xff00))
    {
      /* We don't need to send the Neighbor Solicitation */

      return OK;
    }

  /* Get the device that can route this request */

#ifdef CONFIG_NETDEV_MULTINIC
  dev = netdev_findby_ipv6addr(g_ipv6_allzeroaddr, ipaddr);
#else
  dev = netdev_findby_ipv6addr(ipaddr);
#endif
  if (!dev)
    {
      ndbg("ERROR: Unreachable: %08lx\n", (unsigned long)ipaddr);
      ret = -EHOSTUNREACH;
      goto errout;
    }

#ifdef CONFIG_NET_MULTILINK
  /* If we are supporting multiple network devices and using different
   * link level protocols then we can get here for other link protocols
   * as well.  Continue and send the Neighbor Solicitation request only
   * if this device uses the Ethernet data link protocol.
   *
   * REVISIT:  Other link layer protocols may require Neighbor Discovery
   * as well (but not SLIP which is the only other option at the moment).
   */

  if (dev->d_lltype != NET_LL_ETHERNET)
    {
      return OK;
    }
#endif

  /* Check if the destination address is on the local network. */

  if (net_ipv6addr_maskcmp(ipaddr, dev->d_ipv6addr, dev->d_ipv6netmask))
    {
      /* Yes.. use the input address for the lookup */

      lookup = ipaddr;
    }
  else
    {
      net_ipv6addr_t dripaddr;

      /* Destination address is not on the local network */

#ifdef CONFIG_NET_ROUTE

      /* We have a routing table.. find the correct router to use in
       * this case (or, as a fall-back, use the device's default router
       * address).  We will use the router IP address instead of the
       * destination address when determining the MAC address.
       */

      netdev_ipv6_router(dev, ipaddr, dripaddr);
#else
      /* Use the device's default router IP address instead of the
       * destination address when determining the MAC address.
       */

      net_ipv6addr_copy(dripaddr, dev->d_ipv6draddr);
#endif

      /* Use the router address for the lookup */

      lookup = dripaddr;
    }

  /* Allocate resources to receive a callback.  This and the following
   * initialization is performed with the network lock because we don't
   * want anything to happen until we are ready.
   */

  save = net_lock();
  state.snd_cb = icmpv6_callback_alloc(dev);
  if (!state.snd_cb)
    {
      ndbg("ERROR: Failed to allocate a cllback\n");
      ret = -ENOMEM;
      goto errout_with_lock;
    }

  /* Initialize the state structure. This is done with interrupts
   * disabled
   */

  (void)sem_init(&state.snd_sem, 0, 0);        /* Doesn't really fail */
  state.snd_retries = 0;                       /* No retries yet */
  net_ipv6addr_copy(state.snd_ipaddr, lookup); /* IP address to query */

#ifdef CONFIG_NETDEV_MULTINIC
  /* Remember the routing device name */

  strncpy((FAR char *)state.snd_ifname, (FAR const char *)dev->d_ifname,
          IFNAMSIZ);
#endif

  /* Now loop, testing if the address mapping is in the Neighbor Table and
   * re-sending the Neighbor Solicitation if it is not.
   */

  ret = -ETIMEDOUT; /* Assume a timeout failure */

  while (state.snd_retries < CONFIG_ICMPv6_NEIGHBOR_MAXTRIES)
    {
      /* Check if the address mapping is present in the Neighbor Table.  This
       * is only really meaningful on the first time through the loop.
       *
       * NOTE: If the Neighbor Table is large than this could be a performance
       * issue.
       */

      if (neighbor_findentry(lookup) != NULL)
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
      state.snd_cb->event = icmpv6_neighbor_interrupt;

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

      /* Now wait for response to the Neighbor Advertisement to be received.
       * The optimal delay would be the work case round trip time.
       * NOTE: The network is locked.
       */

      delay.tv_sec  = CONFIG_ICMPv6_NEIGHBOR_DELAYSEC;
      delay.tv_nsec = CONFIG_ICMPv6_NEIGHBOR_DELAYNSEC;

      ret = icmpv6_wait(&notify, &delay);

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

  sem_destroy(&state.snd_sem);
  icmpv6_callback_free(dev, state.snd_cb);
errout_with_lock:
  net_unlock(save);
errout:
  return ret;
}

#endif /* CONFIG_NET_ICMPv6_NEIGHBOR */
