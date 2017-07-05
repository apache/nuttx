/****************************************************************************
 * net/icmpv6/icmpv6_forward.c
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
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote
 *    products derived from this software without specific prior
 *    written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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

#include "devif/ip_forward.h"
#include "devif/devif.h"
#include "netdev/netdev.h"
#include "arp/arp.h"
#include "neighbor/neighbor.h"
#include "icmpv6/icmpv6.h"

#if defined(CONFIG_NET_IPFORWARD) && defined(CONFIG_NET_ICMPv6) && \
    defined(CONFIG_NETDEV_MULTINIC)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: icmpv6_forward_addrchck
 *
 * Description:
 *   Check if the destination IP address is in the IPv4 ARP or IPv6 Neighbor
 *   tables.  If not, then the send won't actually make it out... it will be
 *   replaced with an ARP request (IPv4) or a Neighbor Solicitation (IPv6).
 *
 *   NOTE 1: This could be an expensive check if there are a lot of
 *   entries in the ARP or Neighbor tables.
 *
 *   NOTE 2: If we are actually harvesting IP addresses on incoming IP
 *   packets, then this check should not be necessary; the MAC mapping
 *   should already be in the ARP table in many cases (IPv4 only).
 *
 *   NOTE 3: If CONFIG_NET_ARP_SEND then we can be assured that the IP
 *   address mapping is already in the ARP table.
 *
 * Parameters:
 *   fwd - The forwarding state structure
 *
 * Returned Value:
 *   true - The Ethernet MAC address is in the ARP or Neighbor table (OR
 *          the network device is not Ethernet).
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_ETHERNET
static inline bool icmpv6_forward_addrchck(FAR struct forward_s *fwd)
{
  /* REVISIT: Could the MAC address not also be in a routing table? */

#ifdef CONFIG_NET_MULTILINK
  if (fwd->f_dev->d_lltype != NET_LL_ETHERNET)
    {
      return true;
    }
#endif

#ifdef CONFIG_NET_IPv4
#ifdef CONFIG_NET_IPv6
  if ((fwd->f_hdr.ipv4.l2.vhl & IP_VERSION_MASK) == IPv4_VERSION)
#endif
    {
#if !defined(CONFIG_NET_ARP_IPIN) && !defined(CONFIG_NET_ARP_SEND)
      return (arp_find(fwd->f_hdr.ipv4.l2.destipaddr) != NULL);
#else
      return true;
#endif
    }
#endif /* CONFIG_NET_IPv4 */

#ifdef CONFIG_NET_IPv6
#ifdef CONFIG_NET_IPv4
  else
#endif
    {
#if !defined(CONFIG_NET_ICMPv6_NEIGHBOR)
      return (neighbor_findentry(fwd->f_hdr.ipv6.l2.destipaddr) != NULL);
#else
      return true;
#endif
    }
#endif /* CONFIG_NET_IPv6 */
}

#else /* CONFIG_NET_ETHERNET */
#  define icmpv6_forward_addrchck(r) (true)
#endif /* CONFIG_NET_ETHERNET */

/****************************************************************************
 * Name: icmpv6_dropstats
 *
 * Description:
 *   Update statistics for a dropped packet.
 *
 * Input Parameters:
 *   fwd - The forwarding state structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_NET_STATISTICS
static void icmpv6_dropstats(FAR struct forward_s *fwd)
{
  /* Increment the count of dropped ICMPV6 packets */

  g_netstats.icmpv6.drop++;

  /* Increment the count of dropped IPv4 or IPv6 packets */

#ifdef CONFIG_NET_IPv4
#ifdef CONFIG_NET_IPv6
  if ((fwd->f_hdr.ipv4.l2.vhl & IP_VERSION_MASK) == IPv4_VERSION)
#endif
    {
      g_netstats.ipv4.drop++;
    }
#endif
#ifdef CONFIG_NET_IPv6
#ifdef CONFIG_NET_IPv4
  else
#endif
    {
      g_netstats.ipv6.drop++;
    }
#endif
}
#else
#  define icmpv6_dropstats(ipv6)
#endif

/****************************************************************************
 * Name: icmpv6_forward_interrupt
 *
 * Description:
 *   This function is called from the interrupt level to perform the actual
 *   send operation when polled by the lower, device interfacing layer.
 *
 * Parameters:
 *   dev        The structure of the network driver that caused the interrupt
 *   conn       An instance of the ICMPV6 connection structure cast to void *
 *   pvpriv     An instance of struct forward_s cast to void*
 *   flags      Set of events describing why the callback was invoked
 *
 * Returned Value:
 *   Modified value of the input flags
 *
 * Assumptions:
 *   The network is locked
 *
 ****************************************************************************/

static uint16_t icmpv6_forward_interrupt(FAR struct net_driver_s *dev,
                                         FAR void *conn, FAR void *pvpriv,
                                         uint16_t flags)
{
  FAR struct forward_s *fwd = (FAR struct forward_s *)pvpriv;

  ninfo("flags: %04x\n", flags);
  DEBUGASSERT(fwd != NULL);

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
          icmpv6_dropstats(fwd);
        }

      /* Check if the outgoing packet is available.  It may have been claimed
       * by another send interrupt serving a different thread -OR- if the output
       * buffer currently contains unprocessed incoming data.  In these cases
       * we will just have to wait for the next polling cycle.
       */

      else if (dev->d_sndlen > 0 || (flags & ICMPv6_NEWDATA) != 0)
        {
          /* Another thread has beat us sending data or the buffer is busy,
           * Wait for the next polling cycle and check again.
           */

          return flags;
        }

      /* It looks like we are good to forward the data */

      else
        {
          /* Copy the ICMPv6 data into driver's packet buffer and send it. */

          devif_forward(fwd);

          /* Check if the destination IP address is in the ARP or Neighbor
           * table.  If not, then the send won't actually make it out... it
           * will be replaced with an ARP request or Neighbor Solicitation.
           */

          if (!icmpv6_forward_addrchck(fwd))
            {
              return flags;
            }
        }

      /* Free the allocated callback structure */

      fwd->f_cb->flags = 0;
      fwd->f_cb->priv  = NULL;
      fwd->f_cb->event = NULL;

      icmpv6_callback_free(dev, fwd->f_cb);

      /* Free any IOBs */

      if (fwd->f_iob != NULL)
        {
          iob_free_chain(fwd->f_iob);
        }

      /* And release the forwarding state structure */

      ip_forward_free(fwd);
    }

  return flags;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: icmpv6_forward
 *
 * Description:
 *   Called by the IP forwarding logic when an ICMPV6 packet is received on
 *   one network device, but must be forwarded on another network device.
 *
 *   Set up to forward the ICMPV6 packet on the specified device.  This
 *   function will set up a send "interrupt" handler that will perform the
 *   actual send asynchronously and must return without waiting for the
 *   send to complete.
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

int icmpv6_forward(FAR struct forward_s *fwd)
{
  DEBUGASSERT(fwd != NULL && fwd->f_dev != NULL);

  /* Set up the callback in the connection */

  fwd->f_cb = icmpv6_callback_alloc(fwd->f_dev);
  if (fwd->f_cb != NULL)
    {
      fwd->f_cb->flags   = (ICMPv6_POLL | NETDEV_DOWN);
      fwd->f_cb->priv    = (FAR void *)fwd;
      fwd->f_cb->event   = icmpv6_forward_interrupt;

      /* Notify the device driver of the availability of TX data */

      netdev_txnotify_dev(fwd->f_dev);
      return OK;
    }

  return -EBUSY;
}

#endif /* CONFIG_NET_IPFORWARD && CONFIG_NET_ICMPv6 && CONFIG_NETDEV_MULTINIC */
