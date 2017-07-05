/****************************************************************************
 * net/tcp/tcp_forward.c
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

#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <net/if.h>

#include <nuttx/mm/iob.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/ip.h>
#include <nuttx/net/netstats.h>

#include "devif/ip_forward.h"
#include "devif/devif.h"
#include "netdev/netdev.h"
#include "tcp/tcp.h"

#if defined(CONFIG_NET_IPFORWARD) && defined(CONFIG_NET_TCP) && \
    defined(CONFIG_NETDEV_MULTINIC)

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
 * Parameters:
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
  /* Which domain the connection support */

  if (fwd->f_conn.tcp.domain == PF_INET)
    {
      /* Select the IPv4 domain */

      tcp_ipv4_select(dev);
    }
  else /* if (conn->domain == PF_INET6) */
    {
      /* Select the IPv6 domain */

      DEBUGASSERT(conn->domain == PF_INET6);
      tcp_ipv6_select(dev);
    }
}
#endif

/****************************************************************************
 * Name: tcp_forward_addrchck
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
 *   None
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_ETHERNET
static inline bool tcp_forward_addrchck(FAR struct forward_s *fwd)
{
  FAR struct tcp_conn_s *conn = &fwd->f_conn.tcp;

#ifdef CONFIG_NET_IPv4
#ifdef CONFIG_NET_IPv6
  if (conn->domain == PF_INET)
#endif
    {
#if !defined(CONFIG_NET_ARP_IPIN) && !defined(CONFIG_NET_ARP_SEND)
      return (arp_find(conn->u.ipv4.raddr) != NULL);
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
      return (neighbor_findentry(conn->u.ipv6.raddr) != NULL);
#else
      return true;
#endif
    }
#endif /* CONFIG_NET_IPv6 */

  UNUSED(conn);
}

#else /* CONFIG_NET_ETHERNET */
#  define tcp_forward_addrchck(r) (true)
#endif /* CONFIG_NET_ETHERNET */

/****************************************************************************
 * Name: tcp_dropstats
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
static void tcp_dropstats(FAR struct forward_s *fwd)
{
  /* Increment the count of dropped TCP packets */

  g_netstats.tcp.drop++;

  /* Increment the count of dropped IPv4 or IPv6 packets */

#ifdef CONFIG_NET_IPv4
#ifdef CONFIG_NET_IPv6
  if (fwd->f_conn.tcp.domain == PF_INET)
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
#  define tcp_dropstats(ipv6)
#endif

/****************************************************************************
 * Name: tcp_forward_interrupt
 *
 * Description:
 *   This function is called from the interrupt level to perform the actual
 *   send operation when polled by the lower, device interfacing layer.
 *
 *   NOTE: Our role here is just data passthrough.  We don't really care
 *   about ACKing, dynamic windows or any of the other TCP complexities.
 *   That is really something between the two endpoints and does not matter
 *   the forwarding hub.
 *
 * Parameters:
 *   dev        The structure of the network driver that caused the interrupt
 *   conn       An instance of the TCP connection structure cast to void *
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

static uint16_t tcp_forward_interrupt(FAR struct net_driver_s *dev,
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
       *
       * REVISIT: TCP disconnection events should should not be recieved here.
       * Rather the disconnection events will be handled by the TCP endpoints.
       */

      if ((flags & NETDEV_DOWN) != 0)
        {
          /* Terminate the transfer with an error. */

          nwarn("WARNING: Network is down... Dropping\n");
          tcp_dropstats(fwd);
        }

      /* Check if the outgoing packet is available.  It may have been claimed
       * by a sendto interrupt serving a different thread -OR- if the output
       * buffer currently contains unprocessed incoming data.  In these cases
       * we will just have to wait for the next polling cycle.
       */

      else if (dev->d_sndlen > 0 || (flags & TCP_NEWDATA) != 0)
        {
          /* Another thread has beat us sending data or the buffer is busy,
           * Wait for the next polling cycle and check again.
           */

          return flags;
        }

      /* It looks like we are good to forward the data */

      else
        {
#if defined(CONFIG_NET_IPv4) && defined(CONFIG_NET_IPv6)
          /* If both IPv4 and IPv6 support are enabled, then we will need to
           * select which one to use when generating the outgoing packet.
           * If only one domain is selected, then the setup is already in
           * place and we need do nothing.
           */

          forward_ipselect(dev, fwd);
#endif
          /* Copy the user data into d_appdata and send it. */

          devif_forward(fwd);

          /* Check if the destination IP address is in the ARP or Neighbor
           * table.  If not, then the send won't actually make it out... it
           * will be replaced with an ARP request or Neighbor Solicitation.
           */

          if (!tcp_forward_addrchck(fwd))
            {
              return flags;
            }
        }

      /* Free the allocated callback structure */

      fwd->f_cb->flags = 0;
      fwd->f_cb->priv  = NULL;
      fwd->f_cb->event = NULL;

      tcp_callback_free(&fwd->f_conn.tcp, fwd->f_cb);

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
 * Name: tcp_forward
 *
 * Description:
 *   Called by the IP forwarding logic when an TCP packet is received on
 *   one network device, but must be forwarded on another network device.
 *
 *   Set up to forward the TCP packet on the specified device.  This
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

int tcp_forward(FAR struct forward_s *fwd)
{
  DEBUGASSERT(fwd != NULL && fwd->f_dev != NULL);
  FAR struct tcp_conn_s *conn = &fwd->f_conn.tcp;

  /* Set up some minimal connection structure so that we appear to be a
   * real TCP connection.
   */

  conn->dev = fwd->f_dev;

#ifdef CONFIG_NET_IPv4
#ifdef CONFIG_NET_IPv6
  if ((fwd->f_hdr.ipv4.l2.vhl & IP_VERSION_MASK) == IPv4_VERSION)
#endif
    {
      FAR struct ipv4_hdr_s *ipv4 = &fwd->f_hdr.ipv4.l2;
      FAR struct tcp_hdr_s  *tcp  = &fwd->f_hdr.ipv4.l3.tcp;

#if defined(CONFIG_NET_IPv4) && defined(CONFIG_NET_IPv6)
      conn->domain = PF_INET;
#endif
      conn->lport  = tcp->srcport;
      conn->rport  = tcp->destport;
      net_ipv4addr_copy(conn->u.ipv4.laddr, ipv4->srcipaddr);
      net_ipv4addr_copy(conn->u.ipv4.raddr, ipv4->destipaddr);
    }
#endif
#ifdef CONFIG_NET_IPv6
#ifdef CONFIG_NET_IPv4
  else
#endif
    {
      FAR struct ipv6_hdr_s *ipv6 = &fwd->f_hdr.ipv6.l2;
      FAR struct tcp_hdr_s  *tcp  = &fwd->f_hdr.ipv6.l3.tcp;

#if defined(CONFIG_NET_IPv4) && defined(CONFIG_NET_IPv6)
      conn->domain = PF_INET6;
#endif
      conn->lport  = tcp->srcport;
      conn->rport  = tcp->destport;
      net_ipv6addr_copy(conn->u.ipv6.laddr, ipv6->srcipaddr);
      net_ipv6addr_copy(conn->u.ipv6.raddr, ipv6->destipaddr);
    }
#endif

  /* Set up the callback in the connection */

  fwd->f_cb = tcp_callback_alloc(conn);
  if (fwd->f_cb != NULL)
    {
      fwd->f_cb->flags = (TCP_POLL | NETDEV_DOWN);
      fwd->f_cb->priv  = (FAR void *)fwd;
      fwd->f_cb->event = tcp_forward_interrupt;

      /* Notify the device driver of the availability of TX data */

      netdev_txnotify_dev(fwd->f_dev);
      return OK;
    }

  return -EBUSY;
}

#endif /* CONFIG_NET_IPFORWARD && CONFIG_NET_TCP && CONFIG_NETDEV_MULTINIC */
