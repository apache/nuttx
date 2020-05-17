/****************************************************************************
 * net/sixlowpan/sixlowpan_send.c
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

#include <assert.h>
#include <errno.h>
#include <debug.h>

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
  FAR struct devif_callback_s *s_cb;            /* Reference to callback
                                                 * instance */
  sem_t                        s_waitsem;       /* Supports waiting for
                                                 * driver events */
  int                          s_result;        /* The result of the transfer */
  FAR const struct ipv6_hdr_s *s_ipv6hdr;       /* IPv6 header, followed by
                                                 * UDP or ICMP header. */
  FAR const struct netdev_varaddr_s *s_destmac; /* Destination MAC address */
  FAR const void              *s_buf;           /* Data to send */
  size_t                       s_len;           /* Length of data in buf */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

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
 *   Process an outgoing UDP or ICMPv6 packet.  Takes an IP packet and
 *   formats it to be sent on an 802.15.4 network using 6lowpan.  Called
 *   from common UDP/ICMPv6 send logic.
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
 *   timeout - Send timeout in milliseconds
 *
 * Returned Value:
 *   Ok is returned on success; Otherwise a negated errno value is returned.
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
                   unsigned int timeout)
{
  struct sixlowpan_send_s sinfo;

  ninfo("len=%lu timeout=%u\n", (unsigned long)len, timeout);

  /* Initialize the send state structure */

  nxsem_init(&sinfo.s_waitsem, 0, 0);
  nxsem_set_protocol(&sinfo.s_waitsem, SEM_PRIO_NONE);

  sinfo.s_result  = -EBUSY;
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
           * net_timedwait will also terminate if a signal is received.
           */

          ninfo("Wait for send complete\n");

          ret = net_timedwait(&sinfo.s_waitsem, timeout);
          if (ret < 0)
            {
              if (ret == -ETIMEDOUT)
                {
                  neighbor_notreachable(dev);
                }

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
