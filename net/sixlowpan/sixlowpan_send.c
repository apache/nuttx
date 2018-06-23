/****************************************************************************
 * net/sixlowpan/sixlowpan_send.c
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
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

#include <semaphore.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/clock.h>
#include <nuttx/semaphore.h>
#include <nuttx/net/net.h>
#include <nuttx/net/radiodev.h>

#include "netdev/netdev.h"
#include "devif/devif.h"

#include "sixlowpan/sixlowpan_internal.h"

#ifdef CONFIG_NET_6LOWPAN

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* These are temporary stubs.  Something like this would be needed to
 * monitor the health of a IPv6 neighbor.
 */

#define neighbor_reachable(dev)
#define neighbor_notreachable(dev)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This is the state data provided to the send event handler.  No actions
 * can be taken until the until we receive the TX poll, then we can call
 * sixlowpan_queue_frames() with this data strurcture.
 */

struct sixlowpan_send_s
{
  FAR struct devif_callback_s *s_cb;      /* Reference to callback instance */
  sem_t                        s_waitsem; /* Supports waiting for driver events */
  int                          s_result;  /* The result of the transfer */
  uint16_t                     s_timeout; /* Send timeout in deciseconds */
  clock_t                      s_time;    /* Last send time for determining timeout */
  FAR const struct ipv6_hdr_s *s_ipv6hdr; /* IPv6 header, followed by UDP or ICMP header. */
  FAR const struct netdev_varaddr_s *s_destmac; /* Destination MAC address */
  FAR const void              *s_buf;     /* Data to send */
  size_t                       s_len;     /* Length of data in buf */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: send_timeout
 *
 * Description:
 *   Check for send timeout.
 *
 * Input Parameters:
 *   sinfo - Send state structure reference
 *
 * Returned Value:
 *   TRUE:timeout FALSE:no timeout
 *
 * Assumptions:
 *   The network is locked
 *
 ****************************************************************************/

static inline bool send_timeout(FAR struct sixlowpan_send_s *sinfo)
{
  /* Check for a timeout.  Zero means none and, in that case, we will let
   * the send wait forever.
   */

  if (sinfo->s_timeout != 0)
    {
      /* Check if the configured timeout has elapsed */

      clock_t timeo_ticks =  DSEC2TICK(sinfo->s_timeout);
      clock_t elapsed     =  clock_systimer() - sinfo->s_time;

      if (elapsed >= timeo_ticks)
        {
          return true;
        }
    }

  /* No timeout */

  return false;
}

/****************************************************************************
 * Name: send_eventhandler
 *
 * Description:
 *   This function is called with the network locked to perform the actual
 *   send operation when polled by the lower, device interfacing layer.
 *
 * Input Parameters:
 *   dev   - The structure of the network driver that generated the event.
 *   conn  - The connection structure associated with the socket
 *   flags - Set of events describing why the callback was invoked
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

static uint16_t send_eventhandler(FAR struct net_driver_s *dev,
                                  FAR void *pvconn,
                                  FAR void *pvpriv, uint16_t flags)
{
  FAR struct sixlowpan_send_s *sinfo = (FAR struct sixlowpan_send_s *)pvpriv;

  ninfo("flags: %04x: %d\n", flags);

  /* Verify that this is a compatible network driver. */

  if (dev->d_lltype != NET_LL_IEEE802154 &&
      dev->d_lltype != NET_LL_PKTRADIO)
    {
      ninfo("Not a compatible network device\n");
      return flags;
    }

  /* REVISIT: Verify that this is the correct IEEE802.15.4 network driver to
   * route the outgoing frame(s).  Chances are that there is only one
   * IEEE802.15.4 network driver
   */

  /* Check if the IEEE802.15.4 network driver went down */

  if ((flags & NETDEV_DOWN) != 0)
    {
      nwarn("WARNING: Device is down\n");
      sinfo->s_result = -ENOTCONN;
      goto end_wait;
    }

  /* Check for a poll for TX data. */

  if ((flags & WPAN_NEWDATA) == 0)
    {
      DEBUGASSERT((flags & WPAN_POLL) != 0);

      /* Transfer the frame list to the IEEE802.15.4 MAC device */

      sinfo->s_result =
        sixlowpan_queue_frames((FAR struct radio_driver_s *)dev,
                               sinfo->s_ipv6hdr, sinfo->s_buf, sinfo->s_len,
                               sinfo->s_destmac);

      flags &= ~WPAN_POLL;
      neighbor_reachable(dev);
      goto end_wait;
    }

  /* Check for a timeout. */

  if (send_timeout(sinfo))
    {
      /* Yes.. report the timeout */

      nwarn("WARNING: SEND timeout\n");
      sinfo->s_result = -ETIMEDOUT;
      neighbor_notreachable(dev);
      goto end_wait;
    }

  /* Continue waiting */

  return flags;

end_wait:
  /* Do not allow any further callbacks */

  sinfo->s_cb->flags   = 0;
  sinfo->s_cb->priv    = NULL;
  sinfo->s_cb->event   = NULL;

  /* Wake up the waiting thread */

  nxsem_post(&sinfo->s_waitsem);
  return flags;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sixlowpan_send
 *
 * Description:
 *   Process an outgoing UDP or ICMPv6 packet.  Takes an IP packet and formats
 *   it to be sent on an 802.15.4 network using 6lowpan.  Called from common
 *   UDP/ICMPv6 send logic.
 *
 *   The payload data is in the caller 'buf' and is of length 'buflen'.
 *   Compressed headers will be added and if necessary the packet is
 *   fragmented. The resulting packet/fragments are submitted to the MAC
 *   via the network driver r_req_data method.
 *
 * Input Parameters:
 *   dev     - The IEEE802.15.4 MAC network driver interface.
 *   list    - Head of callback list for send events
 *   ipv6hdr - IPv6 header followed by UDP or ICMPv6 header.
 *   buf     - Data to send
 *   len     - Length of data to send
 *   destmac - The IEEE802.15.4 MAC address of the destination
 *   timeout - Send timeout in deciseconds
 *
 * Returned Value:
 *   Ok is returned on success; Othewise a negated errno value is returned.
 *   This function is expected to fail if the driver is not an IEEE802.15.4
 *   MAC network driver.  In that case, the logic will fall back to normal
 *   IPv4/IPv6 formatting.
 *
 * Assumptions:
 *   Called with the network locked.
 *
 ****************************************************************************/

int sixlowpan_send(FAR struct net_driver_s *dev,
                   FAR struct devif_callback_s **list,
                   FAR const struct ipv6_hdr_s *ipv6hdr, FAR const void *buf,
                   size_t len, FAR const struct netdev_varaddr_s *destmac,
                   uint16_t timeout)
{
  struct sixlowpan_send_s sinfo;

  ninfo("len=%lu timeout=%u\n", (unsigned long)len, timeout);

  /* Initialize the send state structure */

  nxsem_init(&sinfo.s_waitsem, 0, 0);
  (void)nxsem_setprotocol(&sinfo.s_waitsem, SEM_PRIO_NONE);

  sinfo.s_result  = -EBUSY;
  sinfo.s_timeout = timeout;
  sinfo.s_time    = clock_systimer();
  sinfo.s_ipv6hdr = ipv6hdr;
  sinfo.s_destmac = destmac;
  sinfo.s_buf     = buf;
  sinfo.s_len     = len;

  net_lock();
  if (len > 0)
    {
      /* Allocate resources to receive a callback.
       *
       * The second parameter is NULL meaning that we can get only
       * device related events, no connect-related events.
       */

      sinfo.s_cb = devif_callback_alloc(dev, list);
      if (sinfo.s_cb != NULL)
        {
          int ret;

          /* Set up the callback in the connection */

          sinfo.s_cb->flags = (NETDEV_DOWN | WPAN_POLL);
          sinfo.s_cb->priv  = (FAR void *)&sinfo;
          sinfo.s_cb->event = send_eventhandler;

          /* Notify the IEEE802.15.4 MAC that we have data to send. */

          netdev_txnotify_dev(dev);

          /* Wait for the send to complete or an error to occur.
           * net_lockedwait will also terminate if a signal is received.
           */

          ninfo("Wait for send complete\n");

          ret = net_lockedwait(&sinfo.s_waitsem);
          if (ret < 0)
            {
              sinfo.s_result = ret;
            }

          /* Make sure that no further events are processed */

          devif_conn_callback_free(dev, sinfo.s_cb, list);
        }
    }

  nxsem_destroy(&sinfo.s_waitsem);
  net_unlock();

  return (sinfo.s_result < 0 ? sinfo.s_result : len);
}

#endif /* CONFIG_NET_6LOWPAN */
