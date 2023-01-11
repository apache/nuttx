/****************************************************************************
 * net/ipforward/ipfwd_forward.c
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

#include <net/if.h>

#include <nuttx/mm/iob.h>
#include <nuttx/net/net.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/ip.h>
#include <nuttx/net/netstats.h>

#include "devif/devif.h"
#include "netdev/netdev.h"
#include "arp/arp.h"
#include "neighbor/neighbor.h"
#include "ipforward/ipforward.h"

#ifdef CONFIG_NET_IPFORWARD
/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: forward_ipselect
 *
 * Description:
 *   If both IPv4 and IPv6 support are enabled, then we will need to select
 *   which one to use when generating the outgoing packet.  If only one
 *   domain is selected, then the setup is already in place and we need do
 *   nothing.
 *
 * Input Parameters:
 *   fwd - The forwarding state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

#if defined(CONFIG_NET_IPv4) && defined(CONFIG_NET_IPv6)
static inline void forward_ipselect(FAR struct forward_s *fwd)
{
  FAR struct net_driver_s *dev = fwd->f_dev;

  /* Select IPv4 or IPv6 */

  if (fwd->f_domain == PF_INET)
    {
      /* Clear a bit in the d_flags to distinguish this from an IPv6 packet */

      IFF_SET_IPv4(dev->d_flags);

      /* Set the offset to the beginning of the UDP data payload */

      dev->d_appdata = IPBUF(IPv4UDP_HDRLEN);
    }
  else
    {
      /* Set a bit in the d_flags to distinguish this from an IPv6 packet */

      IFF_SET_IPv6(dev->d_flags);

      /* Set the offset to the beginning of the UDP data payload */

      dev->d_appdata = IPBUF(IPv6UDP_HDRLEN);
    }
}
#endif

/****************************************************************************
 * Name: ipfwd_eventhandler
 *
 * Description:
 *   This function is called with the network locked to perform the actual
 *   send operation when polled by the lower, device interfacing layer.
 *
 * Input Parameters:
 *   dev        The structure of the network driver that generated the
 *              event
 *   pvpriv     An instance of struct forward_s cast to (void *)
 *   flags      Set of events describing why the callback was invoked
 *
 * Returned Value:
 *   Modified value of the input flags
 *
 * Assumptions:
 *   The network is locked
 *
 ****************************************************************************/

static uint16_t ipfwd_eventhandler(FAR struct net_driver_s *dev,
                                   FAR void *pvpriv, uint16_t flags)
{
  FAR struct forward_s *fwd = pvpriv;

  ninfo("flags: %04x\n", flags);
  DEBUGASSERT(fwd != NULL && fwd->f_iob != NULL && fwd->f_dev != NULL);

  /* Make sure that this is from the forwarding device */

  if (dev == fwd->f_dev)
    {
      /* If the network device has gone down, then we will have terminate
       * the wait now with an error.
       */

      if ((flags & NETDEV_DOWN) != 0)
        {
          /* Terminate the transfer with an error. */

          nwarn("WARNING: Network is down... Dropping\n");
          ipfwd_dropstats(fwd);
        }

      /* Check if the outgoing packet is available.  It may have been claimed
       * by a sendto event handler serving a different thread -OR- if the
       * output buffer currently contains unprocessed incoming data.  In
       * these cases we will just have to wait for the next polling cycle.
       */

      else if (dev->d_sndlen > 0 || (flags & IPFWD_NEWDATA) != 0)
        {
          /* Another thread has beat us sending data or the buffer is busy,
           * Wait for the next polling cycle and check again.
           */

          return flags;
        }

      /* It looks like we are good to forward the data */

      else
        {
          /* Copy the user data into d_appdata and send it. */

          devif_forward(fwd);
          flags &= ~DEVPOLL_MASK;

#if defined(CONFIG_NET_IPv4) && defined(CONFIG_NET_IPv6)
          /* If both IPv4 and IPv6 support are enabled, then we will need to
           * select which one to use when generating the outgoing packet.
           * If only one domain is selected, then the setup is already in
           * place and we need do nothing.
           */

          forward_ipselect(fwd);
#endif
        }

      /* Free the allocated callback structure */

      fwd->f_cb->flags = 0;
      fwd->f_cb->priv  = NULL;
      fwd->f_cb->event = NULL;

      ipfwd_callback_free(dev, fwd->f_cb);

      /* Free any IOBs */

      if (fwd->f_iob != NULL)
        {
          iob_free_chain(fwd->f_iob);
        }

      /* And release the forwarding state structure */

      ipfwd_free(fwd);
    }

  return flags;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ipfwd_forward
 *
 * Description:
 *   Called by the IP forwarding logic when a packet is received on one
 *   network device, but must be forwarded on another network device.
 *
 *   Set up to forward the packet on the specified device.  This function
 *   will set up a send  event handler that will perform the actual send
 *   asynchronously and must return without waiting for the send to
 *   complete.
 *
 * Input Parameters:
 *   fwd - An initialized instance of the common forwarding structure that
 *         includes everything needed to perform the forwarding operation.
 *
 * Returned Value:
 *   Zero is returned if the packet was successfully forwarded;  A negated
 *   errno value is returned if the packet is not forwardable.  In that
 *   latter case, the caller should free the IOB list and drop the packet.
 *
 ****************************************************************************/

int ipfwd_forward(FAR struct forward_s *fwd)
{
  DEBUGASSERT(fwd != NULL && fwd->f_iob != NULL && fwd->f_dev != NULL);

  /* Set up the callback in the connection */

  fwd->f_cb = ipfwd_callback_alloc(fwd->f_dev);
  if (fwd->f_cb != NULL)
    {
      fwd->f_cb->flags   = (IPFWD_POLL | NETDEV_DOWN);
      fwd->f_cb->priv    = (FAR void *)fwd;
      fwd->f_cb->event   = ipfwd_eventhandler;

      /* Notify the device driver of the availability of TX data */

      netdev_txnotify_dev(fwd->f_dev);
      return OK;
    }

  return -EBUSY;
}

#endif /* CONFIG_NET_IPFORWARD */
