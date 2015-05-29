/****************************************************************************
 * net/arp/arp_send.c
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
#include <nuttx/net/arp.h>

#include "netdev/netdev.h"
#include "devif/devif.h"
#include "route/route.h"
#include "arp/arp.h"

#ifdef CONFIG_NET_ARP_SEND

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CONFIG_ARP_SEND_DELAYSEC  \
  (CONFIG_ARP_SEND_DELAYMSEC / 1000)
#define CONFIG_ARP_SEND_DELAYNSEC \
  ((CONFIG_ARP_SEND_DELAYMSEC - 1000*CONFIG_ARP_SEND_DELAYSEC) * 1000000)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Function: arp_send_terminate
 ****************************************************************************/

static void arp_send_terminate(FAR struct arp_send_s *state, int result)
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
 * Function: arp_send_interrupt
 ****************************************************************************/

static uint16_t arp_send_interrupt(FAR struct net_driver_s *dev,
                                   FAR void *pvconn,
                                   FAR void *priv, uint16_t flags)
{
  FAR struct arp_send_s *state = (FAR struct arp_send_s *)priv;

  nllvdbg("flags: %04x sent: %d\n", flags, state->snd_sent);

  if (state)
    {
      /* Check if the network is still up */

      if ((flags & NETDEV_DOWN) != 0)
        {
          nlldbg("ERROR: Interface is down\n");
          arp_send_terminate(state, -ENETUNREACH);
          return flags;
        }

      /* Check if the outgoing packet is available. It may have been claimed
       * by a send interrupt serving a different thread -OR- if the output
       * buffer currently contains unprocessed incoming data. In these cases
       * we will just have to wait for the next polling cycle.
       */

      if (dev->d_sndlen > 0 || (flags & PKT_NEWDATA) != 0)
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

      arp_format(dev, state->snd_ipaddr);

      /* Make sure no ARP request overwrites this ARP request.  This
       * flag will be cleared in arp_out().
       */

      IFF_SET_NOARP(dev->d_flags);

      /* Don't allow any further call backs. */

      arp_send_terminate(state, OK);
    }

  return flags;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: arp_send
 *
 * Description:
 *   The arp_send() call may be to send an ARP request to resolve an IP
 *   address.  This function first checks if the IP address is already in
 *   ARP table.  If so, then it returns success immediately.
 *
 *   If the requested IP address in not in the ARP table, then this function
 *   will send an ARP request, delay, then check if the IP address is now in
 *   the ARP table.  It will repeat this sequence until either (1) the IP
 *   address mapping is now in the ARP table, or (2) a configurable number
 *   of timeouts occur without receiving the ARP replay.
 *
 * Parameters:
 *   ipaddr   The IP address to be queried (in network order).
 *
 * Returned Value:
 *   Zero (OK) is returned on success and the IP address mapping can now be
 *   found in the ARP table.  On error a negated errno value is returned:
 *
 *     -ETIMEDOUT:    The number or retry counts has been exceed.
 *     -EHOSTUNREACH: Could not find a route to the host
 *
 * Assumptions:
 *   This function is called from the normal tasking context.
 *
 ****************************************************************************/

int arp_send(in_addr_t ipaddr)
{
  FAR struct net_driver_s *dev;
  struct arp_notify_s notify;
  struct timespec delay;
  struct arp_send_s state;
  net_lock_t save;
  int ret;

  /* First check if destination is a local broadcast. */

  if (ipaddr == INADDR_BROADCAST)
    {
      /* We don't need to send the ARP request */

      return OK;
    }

#ifdef CONFIG_NET_IGMP
  /* Check if the destination address is a multicast address
   *
   * - IPv4: multicast addresses lie in the class D group -- The address range
   *   224.0.0.0 to 239.255.255.255 (224.0.0.0/4)
   *
   * - IPv6 multicast addresses are have the high-order octet of the
   *   addresses=0xff (ff00::/8.)
   */

 if (NTOHL(ipaddr) >= 0xe0000000 && NTOHL(ipaddr) <= 0xefffffff)
    {
      /* We don't need to send the ARP request */

      return OK;
    }
#endif

  /* Get the device that can route this request */

#ifdef CONFIG_NETDEV_MULTINIC
  dev = netdev_findby_ipv4addr(INADDR_ANY, ipaddr);
#else
  dev = netdev_findby_ipv4addr(ipaddr);
#endif
  if (!dev)
    {
      ndbg("ERROR: Unreachable: %08lx\n", (unsigned long)ipaddr);
      ret = -EHOSTUNREACH;
      goto errout;
    }

#ifdef CONFIG_NET_MULTILINK
  /* ARP support is only built if the Ethernet data link is supported.
   * However, if we are supporting multiple network devices and using
   * different link level protocols then we can get here for other
   * link protocols as well.  Continue and send the ARP request only
   * if this device uses the Ethernet data link protocol.
   */

  if (dev->d_lltype != NET_LL_ETHERNET)
    {
      return OK;
    }
#endif

  /* Check if the destination address is on the local network. */

  if (!net_ipv4addr_maskcmp(ipaddr, dev->d_ipaddr, dev->d_netmask))
    {
      in_addr_t dripaddr;

      /* Destination address is not on the local network */

#ifdef CONFIG_NET_ROUTE

      /* We have a routing table.. find the correct router to use in
       * this case (or, as a fall-back, use the device's default router
       * address).  We will use the router IP address instead of the
       * destination address when determining the MAC address.
       */

      netdev_ipv4_router(dev, ipaddr, &dripaddr);
#else
      /* Use the device's default router IP address instead of the
       * destination address when determining the MAC address.
       */

      net_ipv4addr_copy(dripaddr, dev->d_draddr);
#endif
      ipaddr = dripaddr;
    }

  /* Allocate resources to receive a callback.  This and the following
   * initialization is performed with the network lock because we don't
   * want anything to happen until we are ready.
   */

  save = net_lock();
  state.snd_cb = arp_callback_alloc(dev);
  if (!state.snd_cb)
    {
      ndbg("ERROR: Failed to allocate a callback\n");
      ret = -ENOMEM;
      goto errout_with_lock;
    }

  /* Initialize the state structure. This is done with interrupts
   * disabled
   */

  (void)sem_init(&state.snd_sem, 0, 0); /* Doesn't really fail */
  state.snd_retries   = 0;              /* No retries yet */
  state.snd_ipaddr    = ipaddr;         /* IP address to query */

#ifdef CONFIG_NETDEV_MULTINIC
  /* Remember the routing device name */

  strncpy((FAR char *)state.snd_ifname, (FAR const char *)dev->d_ifname, IFNAMSIZ);
#endif

  /* Now loop, testing if the address mapping is in the ARP table and re-sending the ARP request if it is not.
   */

  ret = -ETIMEDOUT; /* Assume a timeout failure */

  while (state.snd_retries < CONFIG_ARP_SEND_MAXTRIES)
    {
      /* Check if the address mapping is present in the ARP table.  This
       * is only really meaningful on the first time through the loop.
       *
       * NOTE: If the ARP table is large than this could be a performance
       * issue.
       */

      if (arp_find(ipaddr))
        {
          /* We have it!  Break out with success */

          ret = OK;
          break;
        }

      /* Set up the ARP response wait BEFORE we send the ARP request */

      arp_wait_setup(ipaddr, &notify);

      /* Arm/re-arm the callback */

      state.snd_sent      = false;
      state.snd_result    = -EBUSY;
      state.snd_cb->flags = (ARP_POLL | NETDEV_DOWN);
      state.snd_cb->priv  = (FAR void *)&state;
      state.snd_cb->event = arp_send_interrupt;

      /* Notify the device driver that new TX data is available.
       * NOTES: This is in essence what netdev_ipv4_txnotify() does, which
       * is not possible to call since it expects a in_addr_t as
       * its single argument to lookup the network interface.
       */

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

      /* Check the result of the send operation */

      ret = state.snd_result;
      if (ret < 0)
        {
          /* Break out on a send failure */

          ndbg("ERROR: Send failed: %d\n", ret);
          break;
        }

      /* Now wait for response to the ARP response to be received.  The
       * optimal delay would be the work case round trip time.
       * NOTE: The network is locked.
       */

      delay.tv_sec  = CONFIG_ARP_SEND_DELAYSEC;
      delay.tv_nsec = CONFIG_ARP_SEND_DELAYNSEC;

      ret = arp_wait(&notify, &delay);

      /* arp_wait will return OK if and only if the matching ARP response
       * is received.  Otherwise, it will return -ETIMEDOUT.
       */

      if (ret >= OK)
        {
          /* Break out if arp_wait() fails */

          break;
        }

      /* Increment the retry count */

      state.snd_retries++;
      ndbg("ERROR: arp_wait failed: %d\n", ret);
    }

  sem_destroy(&state.snd_sem);
  arp_callback_free(dev, state.snd_cb);
errout_with_lock:
  net_unlock(save);
errout:
  return ret;
}

#endif /* CONFIG_NET_ARP_SEND */
