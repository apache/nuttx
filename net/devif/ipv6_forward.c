/****************************************************************************
 * net/devif/ipv6_forward.c
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

#include <debug.h>
#include <errno.h>

#include <nuttx/mm/iob.h>
#include <nuttx/net/net.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/netstats.h>

#include "netdev/netdev.h"
#include "sixlowpan/sixlowpan.h"
#include "tcp/tcp.h"
#include "devif/devif.h"

#if defined(CONFIG_NET_IPFORWARD) && defined(CONFIG_NET_IPv6)

/****************************************************************************
 * Private Types
 ****************************************************************************/

#if defined(CONFIG_NETDEV_MULTINIC) && \
   (defined(CONFIG_NET_UDP) || defined(CONFIG_NET_ICMPv6))

/* IPv6 + UDP or ICMPv6 header */

struct ipv6l3_hdr_s
{
  struct ipv6_hdr_s       ipv6;
  union
  {
#ifdef CONFIG_NET_UDP
    struct udp_hdr_s      udp;
#endif
#ifdef CONFIG_NET_ICMPv6
    struct icmpv6_iphdr_s icmp;
#endif
  } u;
};

/* This is the send state structure */

struct forward_s
{
  FAR struct net_driver_s *dev;  /* Forwarding device */
  struct ipv6l3_hdr_s      hdr;  /* Copy of origin L2+L3 headers */
  FAR struct iob_queue_s   iobq; /* IOBs contained the data payload */
};

#endif /* CONFIG_NETDEV_MULTINIC && (CONFIG_NET_UDP || CONFIG_NET_ICMPv6) */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ipv6_packet_conversion
 *
 * Description:
 *   Generic output conversion hook.  Only needed for IEEE802.15.4 for now
 *   but this is a point where support for other conversions may be
 *   provided.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_6LOWPAN
static int ipv6_packet_conversion(FAR struct net_driver_s *dev,
                                  FAR struct net_driver_s *fwddev,
                                  FAR struct ipv6_hdr_s *ipv6)
{
#ifdef CONFIG_NET_MULTILINK
  /* Handle the case where multiple link layer protocols are supported */

  if (dev->d_len > 0 && fwddev->d_lltype == NET_LL_IEEE802154)
#else
  if (dev->d_len > 0)
#endif
    {
#ifdef CONFIG_NET_TCP
      if (ipv6->proto == IP_PROTO_TCP)
        {
          /* Let 6LoWPAN convert IPv6 TCP output into IEEE802.15.4 frames. */

          sixlowpan_tcp_send(dev, fwddev, ipv6);
        }
      else
#endif
#ifdef CONFIG_NET_UDP
      if (ipv6->proto == IP_PROTO_UDP)
        {
          /* Let 6LoWPAN convert IPv6 UDP output into IEEE802.15.4 frames. */

          sixlowpan_udp_send(dev, fwddev, ipv6);
        }
      else
#endif
        {
          /* Otherwise, we will have to drop the packet */

          nwarn("WARNING: Dropping.  Unsupported 6LoWPAN protocol: %d\n",
                ipv6->proto);

#ifdef CONFIG_NET_STATISTICS
          g_netstats.ipv6.drop++;
#endif
        }

      dev->d_len = 0;
      return OK;
    }

  return -EPFNOSUPPORT;
}
#else
# define ipv6_packet_conversion(dev, ipv6)
#endif /* CONFIG_NET_6LOWPAN */

/****************************************************************************
 * Name: ipv6_hdrsize
 *
 * Description:
 *   Return the size of the IPv6 header and the following.
 *
 * Input Parameters:
 *   ipv6  - A pointer to the IPv6 header in within the IPv6 packet.  This
 *           is immeidately followed by the L3 header which may be TCP, UDP,
 *           or ICMPv6.
 *
 * Returned Value:
 *   The size of the combined L2 + L3 headers is returned on success.  An
 *   error is returned only if the prototype is not supported.
 *
 ****************************************************************************/

#ifdef CONFIG_NETDEV_MULTINIC
static int ipv6_hdrsize(FAR struct ipv6_hdr_s *ipv6)
{
  /* Size is determined by the following protocol header, */

  switch (ipv6->proto)
    {
#ifdef CONFIG_NET_TCP
    case IP_PROTO_TCP:
      {
        FAR struct tcp_hdr_s *tcp =
          (FAR struct tcp_hdr_s *)((FAR uintptr_t *)ipv6 + IPv6_HDRLEN);
        unsigned int tcpsize;

        /* The TCP header length is encoded in the top 4 bits of the
         * tcpoffset field (in units of 32-bit words).
         */

        tcpsize = ((uint16_t)tcp->tcpoffset >> 4) << 2;
        return IPv6_HDRLEN + tcpsize;
      }
      break;
#endif

#ifdef CONFIG_NET_UDP
    case IP_PROTO_UDP:
      return IPv6_HDRLEN + UDP_HDRLEN;
      break;
#endif

#ifdef CONFIG_NET_ICMPv6
    case IP_PROTO_ICMP6:
      return IPv6_HDRLEN + ICMPv6_HDRLEN;
      break;
#endif

    default:
      nwarn("WARNING: Unrecognized proto: %u\n", ipv6->proto);
      return -EPROTONOSUPPORT;
    }
}
#endif

/****************************************************************************
 * Name: ipv6_dev_forward
 *
 * Description:
 *   Set up to forward the UDP or ICMPv6 packet on the specified device.
 *   This function will set up a send "interrupt" handler that will perform
 *   the actual send asynchronously and must return without waiting for the
 *   send to complete.
 *
 * Input Parameters:
 *   dev   - The device on which the packet should be forwarded.
 *   ipv6  - A pointer to the IPv6 header in within the IPv6 packet.  This
 *           is immeidately followed by the L3 header which may be UDP or
 *           ICMPv6.
 *   iob   - A list of IOBs containing the data payload to be sent.
 *
 * Returned Value:
 *   Zero is returned if the packet was successfully forwarded;  A negated
 *   errno value is returned if the packet is not forwardable.  In that
 *   latter case, the caller should free the IOB list and drop the packet.
 *
 ****************************************************************************/

#if defined(CONFIG_NETDEV_MULTINIC) && \
   (defined(CONFIG_NET_UDP) || defined(CONFIG_NET_ICMPv6))
static int ipv6_dev_forward(FAR struct net_driver_s *dev,
                            FAR struct ipv6_hdr_s *ipv6,
                            FAR struct iob_s *iob)
{
  /* Notify the forwarding device that TX data is available */

  /* Set up to send the packet when the selected device polls for TX data. */

#warning Missing logic

  /* REVISIT:  For Ethernet we may have to fix up the Ethernet header:
   * - source MAC, the MAC of the current device.
   * - dest MAC, the MAC associated with the destination IPv6 adress.
   *   This will involve ICMPv6 and Neighbor Discovery.
   */

  nwarn("WARNING: UPD/ICMPv6 packet forwarding not yet supported\n");
  return -ENOSYS;
}
#else
#  define ipv6_dev_forward(dev,ipv6,iob) -EPROTONOSUPPORT
#endif

/****************************************************************************
 * Name: ipv6_decr_ttl
 *
 * Description:
 *   Decrement the IPv6 TTL (time to live value).  TTL field is set by the
 *   sender of the packet and reduced by every router on the route to its
 *   destination. If the TTL field reaches zero before the datagram arrives
 *   at its destination, then the datagram is discarded and an ICMP error
 *   packet (11 - Time Exceeded) is sent back to the sender.
 *
 *   The purpose of the TTL field is to avoid a situation in which an
 *   undeliverable datagram keeps circulating on an Internet system, and
 *   such a system eventually becoming swamped by such "immortals".
 *
 * Input Parameters:
 *   ipv6  - A pointer to the IPv6 header in within the IPv6 packet to be
 *           forwarded.
 *
 * Returned Value:
 *   The new TTL value is returned.  A value <= 0 means the hop limit has
 *   expired.
 *
 ****************************************************************************/

static int ipv6_decr_ttl(FAR struct ipv6_hdr_s *ipv6)
{
  int ttl = (int)ipv6->ttl - 1;

  if (ttl <= 0)
    {
#ifdef CONFIG_NET_ICMPv6
      /* Return an ICMPv6 error packet back to the sender. */
#warning Missing logic
#endif

      /* Return zero which must cause the packet to be dropped */

      return 0;
    }

  /* Save the updated TTL value */

  ipv6->ttl = ttl;

  /* NOTE: We do not have to recalculate the IPv6 checksum because (1) the
   * IPv6 header does not include a checksum itself and (2) the TTL is not
   * included in the sum for the TCP and UDP headers.
   */

  return ttl;
}

/****************************************************************************
 * Name: ipv6_dropstats
 *
 * Description:
 *   Update statistics for a droped packet.
 *
 * Input Parameters:
 *   ipv6  - A convenience pointer to the IPv6 header in within the IPv6
 *           packet to be dropped.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_NET_STATISTICS
static void ipv6_dropstats(FAR struct ipv6_hdr_s *ipv6)
{
  switch (ipv6->proto)
    {
#ifdef CONFIG_NET_TCP
    case IP_PROTO_TCP:
      g_netstats.tcp.drop++;
      break;
#endif

#ifdef CONFIG_NET_UDP
    case IP_PROTO_UDP:
      g_netstats.udp.drop++;
      break;
#endif

#ifdef CONFIG_NET_ICMPv6
    case IP_PROTO_ICMP6:
      g_netstats.icmpv6.drop++;
      break;
#endif

    default:
      break;
    }

  g_netstats.ipv6.drop++;
}
#else
#  define ipv6_dropstats(ipv6)
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ipv6_forward
 *
 * Description:
 *   This function is called from ipv6_input when a packet is received that
 *   is not destined for us.  In this case, the packet may need to be
 *   forwarded to another device (or sent back out the same device)
 *   depending configuration, routing table information, and the IPv6
 *   networks served by various network devices.
 *
 * Input Parameters:
 *   dev   - The device on which the packet was received and which contains
 *           the IPv6 packet.
 *   ipv6  - A convenience pointer to the IPv6 header in within the IPv6
 *           packet
 *
 *   On input:
 *   - dev->d_buf holds the received packet.
 *   - dev->d_len holds the length of the received packet MINUS the
 *     size of the L1 header.  That was subtracted out by ipv6_input.
 *   - ipv6 points to the IPv6 header with dev->d_buf.
 *
 * Returned Value:
 *   Zero is returned if the packet was successfully forward;  A negated
 *   errno value is returned if the packet is not forwardable.  In that
 *   latter case, the caller (ipv6_input()) should drop the packet.
 *
 ****************************************************************************/

int ipv6_forward(FAR struct net_driver_s *dev, FAR struct ipv6_hdr_s *ipv6)
{
  /* Multiple network devices */

  FAR struct net_driver_s *fwddev;
  int ret;

  /* Decrement the TTL.  If it decrements to zero, then drop the packet */

  ret = ipv6_decr_ttl(ipv6);
  if (ret < 1)
    {
      ret = -EMULTIHOP;
      goto drop;
    }

  /* Search for a device that can forward this packet.  This is a trivial
   * serch if there is only a single network device (CONFIG_NETDEV_MULTINIC
   * not defined).  But netdev_findby_ipv6addr() will still assure
   * routability in that case.
   */

#ifdef CONFIG_NETDEV_MULTINIC
  fwddev = netdev_findby_ipv6addr(ipv6->srcipaddr, ipv6->destipaddr);
#else
  fwddev = netdev_findby_ipv6addr(ipv6->destipaddr);
#endif
  if (fwddev == NULL)
    {
      nwarn("WARNING: Not routable\n");
      return (ssize_t)-ENETUNREACH;
    }

#if defined(CONFIG_NETDEV_MULTINIC)
  /* Check if we are forwarding on the same device that we received the
   * packet from.
   */

  if (fwddev != dev)
    {
      /* Perform any necessary packet conversions. */

      ret = ipv6_packet_conversion(dev, fwddev, ipv6);
      if (ret < 0)
        {
          FAR struct iob_s *iob = NULL;
          FAR uint8_t *payload;
          unsigned int paysize;
          int hdrsize;

          /* Get the size of the IPv6 + L3 header.  Use this to determine
           * start of the data payload.
           *
           * Remember that the size of the L1 header has already been
           * subtracted from dev->d_len.
           */

          hdrsize = ipv6_hdrsize(ipv6);
          if (hdrsize < 0)
            {
              ret = -EPROTONOSUPPORT;
              goto drop;
            }

          payload = (FAR uint8_t *)ipv6 + hdrsize;
          paysize = dev->d_len - hdrsize;

          if (paysize > 0)
            {
              /* Try to allocate the head of an IOB chain.  If this fails,
               * the the packet will be dropped; we are not operating in a
               * context where waiting for an IOB is a good idea
               */

              iob = iob_tryalloc(false);
              if (iob == NULL)
                {
                  ret = -ENOMEM;
                  goto drop;
                }

              /* Copy the packet data payload into an IOB chain.
               * iob_trycopin() will not wait, but will fail there are no
               * available IOBs.
               */

              ret = iob_trycopyin(iob, payload, paysize, 0, false);
              if (ret < 0)
                {
                  iob_free_chain(iob);
                  goto drop;
                }
            }

          /* Then set up to forward the packet */

#ifdef CONFIG_NET_TCP
          if (ipv6->proto == IP_PROTO_TCP)
            {
              /* Forward a TCP packet, handling ACKs, windowing, etc. */

              ret = tcp_ipv6_forward(fwddev, ipv6, iob);
            }
          else
#endif
            {
              /* Forward a UDP or ICMPv6 packet.  Because ipv6_hdrsize() succeeded,
               * we know that it is a forward-able type.
               */

              ret = ipv6_dev_forward(fwddev, ipv6, iob);
            }

          if (ret < 0)
            {
              goto drop;
            }

          dev->d_len = 0;
          return OK;
        }
    }
  else
#endif /* CONFIG_NETDEV_MULTINIC */

#if defined(CONFIG_NET_6LOWPAN) /* REVISIT:  Currently only suport for 6LoWPAN */
    {
      /* Single network device */

      /* Perform any necessary packet conversions.  If the packet was handled
       * via a backdoor path (or dropped), then dev->d_len will be zero.  If
       * the packet needs to be forwarded in the normal manner then
       * dev->d_len will be unchanged.
       */

      ret = ipv6_packet_conversion(dev, dev, ipv6);
      if (ret < 0)
        {
#ifdef CONFIG_NET_ETHERNET
          /* REVISIT:  For Ethernet we may have to fix up the Ethernet header:
           * - source MAC, the MAC of the current device.
           * - dest MAC, the MAC associated with the destination IPv6 adress.
           *   This  will involve ICMPv6 and Neighbor Discovery.
           */

          /* Correct dev->d_buf by adding back the L1 header length */
#endif

          /* Nothing other 6LoWPAN forwarding is currently handled and that
           * case was dealt with in ipv6_packet_conversion().
           */

#  warning Missing logic
          nwarn("WARNING: Packet forwarding supported only for 6LoWPAN\n");
          return -ENOSYS;
        }
    }

#else /* CONFIG_NET_6LOWPAN */
    {
      nwarn("WARNING: Packet forwarding not supported in this configuration\n");
      return -ENOSYS;
    }
#endif /* CONFIG_NET_6LOWPAN */

  /* Return success.  ipv6_input will return to the network driver with
   * dev->d_len set to the packet size and the network driver will perform
   * the transfer.
   */

  return OK;

drop:
  ipv6_dropstats(ipv6);
  dev->d_len = 0;
  return ret;
}

#endif /* CONFIG_NET_IPFORWARD && CONFIG_NET_IPv6 */
