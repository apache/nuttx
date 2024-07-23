/****************************************************************************
 * net/udp/udp_input.c
 * Handling incoming UDP input
 *
 *   Copyright (C) 2007-2009, 2011, 2018-2019 Gregory Nutt. All rights
 *     reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Adapted for NuttX from logic in uIP which also has a BSD-like license:
 *
 *   Original author Adam Dunkels <adam@dunkels.com>
 *   Copyright () 2001-2003, Adam Dunkels.
 *   All rights reserved.
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
#if defined(CONFIG_NET) && defined(CONFIG_NET_UDP)

#include <debug.h>

#include <nuttx/net/netconfig.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/udp.h>
#include <nuttx/net/netstats.h>

#include "devif/devif.h"
#include "utils/utils.h"
#include "udp/udp.h"
#include "icmp/icmp.h"
#include "icmpv6/icmpv6.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: udp_is_broadcast
 *
 * Description:
 *   Check if the destination address is a broadcast/multicast address.
 *
 * Input Parameters:
 *   dev - The device driver structure containing the received UDP packet
 *
 * Returned Value:
 *   True if the destination address is a broadcast/multicast address
 *
 ****************************************************************************/

#if defined(CONFIG_NET_SOCKOPTS) && defined(CONFIG_NET_BROADCAST)
static bool udp_is_broadcast(FAR struct net_driver_s *dev)
{
  /* Check if the destination address is a broadcast/multicast address */

#ifdef CONFIG_NET_IPv4
#  ifdef CONFIG_NET_IPv6
  if (IFF_IS_IPv4(dev->d_flags))
#  endif
    {
      FAR struct ipv4_hdr_s *ipv4 = IPv4BUF;
      in_addr_t destipaddr = net_ip4addr_conv32(ipv4->destipaddr);

      return net_ipv4addr_cmp(destipaddr, INADDR_BROADCAST) ||
             IN_MULTICAST(NTOHL(destipaddr)) ||
             (net_ipv4addr_maskcmp(destipaddr, dev->d_ipaddr, dev->d_netmask)
              && net_ipv4addr_broadcast(destipaddr, dev->d_netmask));
    }
#endif
#ifdef CONFIG_NET_IPv6
#  ifdef CONFIG_NET_IPv4
  else
#  endif
    {
      FAR struct ipv6_hdr_s *ipv6 = IPv6BUF;
      return net_is_addr_mcast(ipv6->destipaddr);
    }
#endif

  return false;
}
#endif

/****************************************************************************
 * Name: udp_input_conn
 *
 * Description:
 *   Handle incoming UDP input for the case where there is an active
 *   connection.
 *
 * Input Parameters:
 *   dev      - The device driver structure containing the received UDP pkt
 *   conn     - The UDP connection structure associated with the packet
 *   udpiplen - Length of the IP and UDP headers
 *
 * Returned Value:
 *   OK     - The packet has been processed
 *  -EAGAIN - Hold the packet and try again later.  There is a listening
 *            socket but no receive in place to catch the packet yet.  The
 *            device's d_len will be set to zero in this case as there is
 *            no outgoing data.
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

static int udp_input_conn(FAR struct net_driver_s *dev,
                          FAR struct udp_conn_s *conn, unsigned int udpiplen)
{
  uint16_t flags;

  /* Set-up for the application callback */

  dev->d_appdata = IPBUF(udpiplen);
  dev->d_sndlen  = 0;

  /* Perform the application callback */

  flags = udp_callback(dev, conn, UDP_NEWDATA);

  /* If the operation was successful and the UDP data was "consumed,"
   * then the UDP_NEWDATA flag will be cleared by logic in
   * udp_callback().  The packet memory can then be freed by the
   * network driver.  OK will be returned to the network driver to
   * indicate this case.
   *
   * "Consumed" here means that either the received data was (1)
   * accepted by a socket waiting for data on the port or was (2)
   * buffered in the UDP socket's read-ahead buffer.
   */

  if ((flags & UDP_NEWDATA) != 0)
    {
      /* No.. the packet was not processed now.  Return -EAGAIN so
       * that the driver may retry again later.  We still need to
       * set d_len to zero so that the driver is aware that there
       * is nothing to be sent.
       */

      nwarn("WARNING: Packet not processed\n");
      dev->d_len = 0;
      return -EAGAIN;
    }

  /* If the application has data to send, setup the UDP/IP header */

  if (dev->d_sndlen > 0)
    {
      udp_send(dev, conn);
    }

  return OK;
}

/****************************************************************************
 * Name: udp_input
 *
 * Description:
 *   Handle incoming UDP input
 *
 * Input Parameters:
 *   dev   - The device driver structure containing the received UDP packet
 *   udp   - A pointer to the UDP header in the packet
 *   iplen - Length of the IP and UDP headers
 *
 * Returned Value:
 *   OK     - The packet has been processed  and can be deleted
 *  -EAGAIN - Hold the packet and try again later.  There is a listening
 *            socket but no receive in place to catch the packet yet.  The
 *            device's d_len will be set to zero in this case as there is
 *            no outgoing data.
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

static int udp_input(FAR struct net_driver_s *dev, unsigned int iplen)
{
  FAR struct udp_hdr_s *udp;
  FAR struct udp_conn_s *conn;
#if defined(CONFIG_NET_SOCKOPTS) && defined(CONFIG_NET_BROADCAST)
  FAR struct udp_conn_s *nextconn;
  FAR struct iob_s *iob;
#endif
  unsigned int udpiplen;
#ifdef CONFIG_NET_UDP_CHECKSUMS
  uint16_t chksum;
#endif
  int ret = OK;

  /* Update the count of UDP packets received */

#ifdef CONFIG_NET_STATISTICS
  g_netstats.udp.recv++;
#endif

  /* Get a pointer to the UDP header.  The UDP header lies just after the
   * the link layer header and the IP header.
   */

  udp = IPBUF(iplen);

  /* Get the size of the IP header and the UDP header */

  udpiplen = iplen + UDP_HDRLEN;

  /* UDP processing is really just a hack. We don't do anything to the UDP/IP
   * headers, but let the UDP application do all the hard work. If the
   * application sets d_sndlen, it has a packet to send.
   */

  dev->d_len    -= udpiplen;
  dev->d_appdata = IPBUF(udpiplen);

#ifdef CONFIG_NET_UDP_CHECKSUMS
  chksum = udp->udpchksum;
  if (chksum != 0)
    {
#ifdef CONFIG_NET_IPv6
#ifdef CONFIG_NET_IPv4
      if (IFF_IS_IPv6(dev->d_flags))
#endif
        {
          chksum = ~udp_ipv6_chksum(dev);
        }
#endif /* CONFIG_NET_IPv6 */

#ifdef CONFIG_NET_IPv4
#ifdef CONFIG_NET_IPv6
      else
#endif
        {
          chksum = ~udp_ipv4_chksum(dev);
        }
#endif /* CONFIG_NET_IPv6 */
    }

  if (chksum != 0)
    {
#ifdef CONFIG_NET_STATISTICS
      g_netstats.udp.drop++;
      g_netstats.udp.chkerr++;
#endif
      nwarn("WARNING: Bad UDP checksum\n");
      dev->d_len = 0;
    }
  else
#endif
    {
      /* Demultiplex this UDP packet between the UDP "connections".
       *
       * REVISIT:  If the callback logic that receives a packet responds with
       * an outgoing packet, then it may be ignored.  recvfrom() will not do
       * that, however.
       */

      conn = udp_active(dev, NULL, udp);
      if (conn)
        {
          /* We'll only get multiple conn when we support SO_REUSEADDR */

#if defined(CONFIG_NET_SOCKOPTS) && defined(CONFIG_NET_BROADCAST)
          /* Check if the destination is a broadcast/multicast address */

          if (udp_is_broadcast(dev))
            {
              /* Do we have second connection that can hold this packet? */

              while ((nextconn = udp_active(dev, conn, udp)) != NULL)
                {
                  /* Yes... There are multiple listeners on the same port.
                   * We need to clone the packet and deliver it to each
                   * listener.
                   */

                  iob = netdev_iob_clone(dev, true);
                  if (iob == NULL)
                    {
                      nerr("ERROR: IOB clone failed.\n");
                      break; /* We can still process once without clone. */
                    }

                  ret = udp_input_conn(dev, conn, udpiplen);
                  if (ret < 0)
                    {
                      nwarn("WARNING: A conn failed to process the pkt %d\n",
                            ret); /* We can still continue for next conn. */
                    }

                  netdev_iob_replace(dev, iob);
                  udp  = IPBUF(iplen);
                  conn = nextconn;
                }
            }
#endif

          /* We can deliver the packet directly to the last listener. */

          ret = udp_input_conn(dev, conn, udpiplen);
        }
      else
        {
          nwarn("WARNING: No listener on UDP port\n");

          /* No match was found, send ICMP destination port unreachable
           * unless destination address was broadcast/multicast.
           */

#if defined(CONFIG_NET_ICMP) || defined(CONFIG_NET_ICMPv6)
#  ifdef CONFIG_NET_ICMPv6
#    ifdef CONFIG_NET_ICMP
          if (IFF_IS_IPv6(dev->d_flags))
#    endif
            {
              icmpv6_reply(dev, ICMPv6_DEST_UNREACHABLE,
                           ICMPv6_PORT_UNREACH, 0);
            }
#  endif /* CONFIG_NET_ICMPv6 */

#  ifdef CONFIG_NET_ICMP
#    ifdef CONFIG_NET_ICMPv6
          else
#    endif
            {
              icmp_reply(dev, ICMP_DEST_UNREACHABLE, ICMP_PORT_UNREACH);
            }
#  endif /* CONFIG_NET_ICMP */
#else
          dev->d_len = 0;
#endif /* CONFIG_NET_ICMP || CONFIG_NET_ICMPv6 */
        }
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: udp_ipv4_input
 *
 * Description:
 *   Handle incoming UDP input in an IPv4 packet
 *
 * Input Parameters:
 *   dev - The device driver structure containing the received UDP packet
 *
 * Returned Value:
 *   OK  The packet has been processed  and can be deleted
 *   ERROR Hold the packet and try again later. There is a listening socket
 *         but no receive in place to catch the packet yet.
 *
 * Assumptions:
 *   Called from network stack logic with the network stack locked
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv4
int udp_ipv4_input(FAR struct net_driver_s *dev)
{
  FAR struct ipv4_hdr_s *ipv4 = IPv4BUF;
  uint16_t iphdrlen;

  /* Configure to receive an UDP IPv4 packet */

  udp_ipv4_select(dev);

  /* Get the IP header length (accounting for possible options). */

  iphdrlen = (ipv4->vhl & IPv4_HLMASK) << 2;

  /* Then process in the UDP IPv4 input */

  return udp_input(dev, iphdrlen);
}
#endif

/****************************************************************************
 * Name: udp_ipv6_input
 *
 * Description:
 *   Handle incoming UDP input in an IPv6 packet
 *
 * Input Parameters:
 *   dev   - The device driver structure containing the received UDP packet
 *   iplen - The size of the IPv6 header.  This may be larger than
 *           IPv6_HDRLEN the IPv6 header if IPv6 extension headers are
 *           present.
 *
 * Returned Value:
 *   OK  The packet has been processed  and can be deleted
 *   ERROR Hold the packet and try again later. There is a listening socket
 *         but no receive in place to catch the packet yet.
 *
 * Assumptions:
 *   Called from network stack logic with the network stack locked
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv6
int udp_ipv6_input(FAR struct net_driver_s *dev, unsigned int iplen)
{
  /* Configure to receive an UDP IPv6 packet */

  udp_ipv6_select(dev);

  /* Then process in the UDP IPv6 input */

  return udp_input(dev, iplen);
}
#endif

#endif /* CONFIG_NET && CONFIG_NET_UDP */
