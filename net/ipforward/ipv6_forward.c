/****************************************************************************
 * net/ipforward/ipv6_forward.c
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

#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/mm/iob.h>
#include <nuttx/net/net.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/netstats.h>

#include "netdev/netdev.h"
#include "sixlowpan/sixlowpan.h"
#include "devif/devif.h"
#include "icmpv6/icmpv6.h"
#include "ipforward/ipforward.h"

#if defined(CONFIG_NET_IPFORWARD) && defined(CONFIG_NET_IPv6)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define PACKET_FORWARDED     0
#define PACKET_NOT_FORWARDED 1

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ipv6_hdrsize
 *
 * Description:
 *   Return the size of the IPv6 header and the following.
 *
 * Input Parameters:
 *   ipv6  - A pointer to the IPv6 header in within the IPv6 packet.  This
 *           is immediately followed by the L3 header which may be TCP, UDP,
 *           or ICMPv6.
 *
 * Returned Value:
 *   The size of the combined L2 + L3 headers is returned on success.  An
 *   error is returned only if the prototype is not supported.
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_NET_WARN
static int ipv6_hdrsize(FAR struct ipv6_hdr_s *ipv6)
{
  /* Size is determined by the following protocol header, */

  switch (ipv6->proto)
    {
#ifdef CONFIG_NET_TCP
    case IP_PROTO_TCP:
      {
        FAR struct tcp_hdr_s *tcp =
          (FAR struct tcp_hdr_s *)((FAR uint8_t *)ipv6 + IPv6_HDRLEN);
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
 * Name: ipv6_packet_conversion
 *
 * Description:
 *   Generic output conversion hook.  Only needed for IEEE802.15.4 for now
 *   but this is a point where support for other conversions may be
 *   provided.
 *
 * Returned Value:
 *   PACKET_FORWARDED     - Packet was forwarded
 *   PACKET_NOT_FORWARDED - Packet was not forwarded
 *   < 0                  - And error occurred (and packet not forwarded).
 *
 ****************************************************************************/

#ifdef CONFIG_NET_6LOWPAN
static int ipv6_packet_conversion(FAR struct net_driver_s *dev,
                                  FAR struct net_driver_s *fwddev,
                                  FAR struct ipv6_hdr_s *ipv6)
{
  int ret = PACKET_NOT_FORWARDED;

  if (dev->d_len > 0)
    {
      /* Check if this is a device served by 6LoWPAN */

      if (fwddev->d_lltype != NET_LL_IEEE802154 &&
          fwddev->d_lltype != NET_LL_PKTRADIO)
        {
          nwarn("WARNING:  Unsupported link layer... Not forwarded\n");
        }
      else
#ifdef CONFIG_NET_TCP
      if (ipv6->proto == IP_PROTO_TCP)
        {
          /* Decrement the TTL in the IPv6 header.  If it decrements to
           * zero, then drop the packet.
           */

          ret = ipv6_decr_ttl(ipv6);
          if (ret < 1)
            {
              nwarn("WARNING: Hop limit exceeded... Dropping!\n");
              ret = -EMULTIHOP;
            }
          else
            {
              /* Let 6LoWPAN convert IPv6 TCP output into IEEE802.15.4
               * frames.
               */

              sixlowpan_tcp_send(dev, fwddev, ipv6);

              /* The packet was forwarded */

              dev->d_len = 0;
              return PACKET_FORWARDED;
            }
        }
      else
#endif
#ifdef CONFIG_NET_UDP
      if (ipv6->proto == IP_PROTO_UDP)
        {
          /* Decrement the TTL in the IPv6 header.  If it decrements to
           * zero, then drop the packet.
           */

          ret = ipv6_decr_ttl(ipv6);
          if (ret < 1)
            {
              nwarn("WARNING: Hop limit exceeded... Dropping!\n");
              ret = -EMULTIHOP;
            }
          else
            {
              /* Let 6LoWPAN convert IPv6 UDP output into IEEE802.15.4
               * frames.
               */

              sixlowpan_udp_send(dev, fwddev, ipv6);

              /* The packet was forwarded */

              dev->d_len = 0;
              return PACKET_FORWARDED;
            }
        }
      else
#endif

#ifdef CONFIG_NET_ICMPv6
      if (ipv6->proto == IP_PROTO_ICMP6)
        {
          /* Decrement the TTL in the IPv6 header.  If it decrements to
           * zero, then drop the packet.
           */

          ret = ipv6_decr_ttl(ipv6);
          if (ret < 1)
            {
              nwarn("WARNING: Hop limit exceeded... Dropping!\n");
              ret = -EMULTIHOP;
            }
          else
            {
              /* Let 6LoWPAN convert IPv6 ICMPv6 output into IEEE802.15.4
               * frames.
               */

              sixlowpan_icmpv6_send(dev, fwddev, ipv6);

              /* The packet was forwarded */

              dev->d_len = 0;
              return PACKET_FORWARDED;
            }
        }
      else
#endif
        {
          /* Otherwise, we cannot forward the packet */

          nwarn("WARNING: Dropping. Unsupported 6LoWPAN protocol: %d\n",
                ipv6->proto);
        }
    }

  /* The packet was not forwarded (or the HOP limit was exceeded) */

  ipv6_dropstats(ipv6);
  return ret;
}
#else
#  define ipv6_packet_conversion(dev, fwddev, ipv6) (PACKET_NOT_FORWARDED)
#endif /* CONFIG_NET_6LOWPAN */

/****************************************************************************
 * Name: ipv6_dev_forward
 *
 * Description:
 *   This function is called from ipv6_forward when it is necessary to
 *   forward a packet from the current device to different device.  In this
 *   case, the forwarding operation must be performed asynchronously when
 *   the TX poll is received from the forwarding device.
 *
 * Input Parameters:
 *   dev      - The device on which the packet was received and which
 *              contains the IPv6 packet.
 *   fwdddev  - The device on which the packet must be forwarded.
 *   ipv6     - A pointer to the IPv6 header in within the IPv6 packet
 *
 * Returned Value:
 *   Zero is returned if the packet was successfully forwarded;  A negated
 *   errno value is returned if the packet is not forwardable.  In that
 *   latter case, the caller (ipv6_input()) should drop the packet.
 *
 ****************************************************************************/

static int ipv6_dev_forward(FAR struct net_driver_s *dev,
                            FAR struct net_driver_s *fwddev,
                            FAR struct ipv6_hdr_s *ipv6)
{
  FAR struct forward_s *fwd = NULL;
#ifdef CONFIG_DEBUG_NET_WARN
  int hdrsize;
#endif
  int ret;

  /* If the interface isn't "up", we can't forward. */

  if ((fwddev->d_flags & IFF_UP) == 0)
    {
      nwarn("WARNING: device is DOWN\n");
      ret = -EHOSTUNREACH;
      goto errout;
    }

  /* Perform any necessary packet conversions. */

  ret = ipv6_packet_conversion(dev, fwddev, ipv6);
  if (ret < 0)
    {
      nwarn("WARNING: ipv6_packet_conversion failed: %d\n", ret);
      goto errout;
    }
  else if (ret == PACKET_NOT_FORWARDED)
    {
      /* Verify that the full packet will fit within the forwarding devices
       * MTU.  We provide no support for fragmenting forwarded packets.
       */

      if (NET_LL_HDRLEN(fwddev) + dev->d_len > NETDEV_PKTSIZE(fwddev))
        {
          nwarn("WARNING: Packet > MTU... Dropping\n");
          ret = -EFBIG;
          goto errout;
        }

      /* Get a pre-allocated forwarding structure,  This structure will be
       * completely zeroed when we receive it.
       */

      fwd = ipfwd_alloc();
      if (fwd == NULL)
        {
          nwarn("WARNING: Failed to allocate forwarding structure\n");
          ret = -ENOMEM;
          goto errout;
        }

      /* Initialize the easy stuff in the forwarding structure */

      fwd->f_dev    = fwddev;   /* Forwarding device */
#ifdef CONFIG_NET_IPv4
      fwd->f_domain = PF_INET6; /* IPv6 address domain */
#endif

#ifdef CONFIG_DEBUG_NET_WARN
      /* Get the size of the IPv6 + L3 header. */

      hdrsize = ipv6_hdrsize(ipv6);
      if (hdrsize < IPv6_HDRLEN)
        {
          nwarn("WARNING: Could not determine L2+L3 header size\n");
          ret = -EPROTONOSUPPORT;
          goto errout_with_fwd;
        }

      /* The L2/L3 headers must fit within one, contiguous IOB. */

      if (hdrsize > CONFIG_IOB_BUFSIZE)
        {
          nwarn("WARNING: Header is too big for pre-allocated structure\n");
          ret = -E2BIG;
          goto errout_with_fwd;
        }
#endif

      /* Relay the device buffer */

      fwd->f_iob = dev->d_iob;

      /* Decrement the TTL in the copy of the IPv6 header (retaining the
       * original TTL in the sourcee to handle the broadcast case).  If the
       * TTL decrements to zero, then do not forward the packet.
       */

      ret = ipv6_decr_ttl(ipv6);
      if (ret < 1)
        {
          nwarn("WARNING: Hop limit exceeded... Dropping!\n");
          ret = -EMULTIHOP;
          goto errout_with_fwd;
        }

      /* Then set up to forward the packet according to the protocol. */

      ret = ipfwd_forward(fwd);
      if (ret >= 0)
        {
          netdev_iob_clear(dev);
          return OK;
        }
    }

errout_with_fwd:
  if (fwd != NULL)
    {
      ipfwd_free(fwd);
    }

errout:
  return ret;
}

/****************************************************************************
 * Name: ipv6_forward_callback
 *
 * Description:
 *   This function is a callback from netdev_foreach.  It implements the
 *   the broadcast forwarding action for each network device (other than, of
 *   course, the device that received the packet).
 *
 * Input Parameters:
 *   dev   - The device on which the packet was received and which contains
 *           the IPv6 packet.
 *   ipv6  - A convenience pointer to the IPv6 header in within the IPv6
 *           packet
 *
 * Returned Value:
 *   Typically returns zero (meaning to continue the enumeration), but will
 *   return a non-zero to stop the enumeration if an error occurs.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPFORWARD_BROADCAST
int ipv6_forward_callback(FAR struct net_driver_s *fwddev, FAR void *arg)
{
  FAR struct net_driver_s *dev = (FAR struct net_driver_s *)arg;
  FAR struct ipv6_hdr_s *ipv6;
  int ret;

  DEBUGASSERT(fwddev != NULL && dev != NULL && dev->d_buf != NULL);

  /* Check if we are forwarding on the same device that we received the
   * packet from.
   */

  if (fwddev != dev)
    {
      /* Recover the pointer to the IPv6 header in the receiving device's
       * d_buf.
       */

      ipv6 = IPv6BUF;

      /* Send the packet asynchrously on the forwarding device. */

      ret = ipv6_dev_forward(dev, fwddev, ipv6);
      if (ret < 0)
        {
          nwarn("WARNING: ipv6_dev_forward failed: %d\n", ret);
          return ret;
        }
    }

  return OK;
}
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
  FAR struct net_driver_s *fwddev;
  int ret;

  /* Search for a device that can forward this packet. */

  fwddev = netdev_findby_ripv6addr(ipv6->srcipaddr, ipv6->destipaddr);
  if (fwddev == NULL)
    {
      nwarn("WARNING: Not routable\n");
      ret = -ENETUNREACH;
      goto drop;
    }

  /* Check if we are forwarding on the same device that we received the
   * packet from.
   */

  if (fwddev != dev)
    {
      /* Send the packet asynchrously on the forwarding device. */

      ret = ipv6_dev_forward(dev, fwddev, ipv6);
      if (ret < 0)
        {
          nwarn("WARNING: ipv6_dev_forward failed: %d\n", ret);
          goto drop;
        }
    }
  else
#if defined(CONFIG_NET_6LOWPAN) /* REVISIT:  Currently only support for 6LoWPAN */
    {
      /* Single network device.  The use case here is where an endpoint acts
       * as a hub in a star configuration.  This is typical for a wireless
       * star configuration where not all endpoints are accessible from all
       * other endpoints, but seems less useful for a wired network.
       */

      /* Perform any necessary packet conversions.  If the packet was handled
       * via a backdoor path (or dropped), then dev->d_len will be zero.  If
       * the packet needs to be forwarded in the normal manner then
       * dev->d_len will be unchanged.
       */

      ret = ipv6_packet_conversion(dev, dev, ipv6);
      if (ret < 0)
        {
          nwarn("WARNING: ipv6_packet_conversion failed: %d\n", ret);
          goto drop;
        }
      else if (ret == PACKET_NOT_FORWARDED)
        {
#ifdef CONFIG_NET_ETHERNET
          /* REVISIT:
           *  For Ethernet we may have to fix up the Ethernet header:
           * - source MAC, the MAC of the current device.
           * - dest MAC, the MAC associated with the destination IPv6
           *   address.
           *   This  will involve ICMPv6 and Neighbor Discovery.
           */

          /* Correct dev->d_buf by adding back the L1 header length */

#endif

          /* Nothing other 6LoWPAN forwarding is currently handled and that
           * case was dealt with in ipv6_packet_conversion().
           *
           * REVISIT: Is this an issue?  Do other use cases make sense?
           */

          nwarn("WARNING: Packet forwarding supported only for 6LoWPAN\n");
          ret = -ENOSYS;
          goto drop;
        }
    }

#else /* CONFIG_NET_6LOWPAN */
    {
      nwarn(
         "WARNING: Packet forwarding not supported in this configuration\n");
      ret = -ENOSYS;
      goto drop;
    }
#endif /* CONFIG_NET_6LOWPAN */

  /* Return success.  ipv6_input will return to the network driver with
   * dev->d_len set to the packet size and the network driver will perform
   * the transfer.
   */

  return OK;

drop:
  ipv6_dropstats(ipv6);

#ifdef CONFIG_NET_ICMPv6
  /* Try reply ICMPv6 to the sender. */

  switch (ret)
    {
      case -ENETUNREACH:
        icmpv6_reply(dev, ICMPv6_DEST_UNREACHABLE, ICMPv6_ADDR_UNREACH, 0);
        return OK;

      case -EFBIG:
        icmpv6_reply(dev, ICMPv6_PACKET_TOO_BIG, 0,
                     NETDEV_PKTSIZE(fwddev) - NET_LL_HDRLEN(fwddev));
        return OK;

      case -EMULTIHOP:
        icmpv6_reply(dev, ICMPv6_PACKET_TIME_EXCEEDED, 0, 0);
        return OK;

      default:
        break; /* We don't know how to reply, just go on (to drop). */
    }
#endif

  dev->d_len = 0;
  return ret;
}

/****************************************************************************
 * Name: ipv6_forward_broadcast
 *
 * Description:
 *   This function is called from ipv6_input when a broadcast or multicast
 *   packet is received.  If CONFIG_NET_IPFORWARD_BROADCAST is enabled, this
 *   function will forward the broadcast packet to other networks through
 *   other network devices.
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
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPFORWARD_BROADCAST
void ipv6_forward_broadcast(FAR struct net_driver_s *dev,
                            FAR struct ipv6_hdr_s *ipv6)
{
  /* Don't bother if the TTL would expire */

  if (ipv6->ttl > 1)
    {
      /* Forward the the broadcast/multicast packet to all devices except,
       * of course, the device that received the packet.
       */

      netdev_foreach(ipv6_forward_callback, dev);
    }
}
#endif

#endif /* CONFIG_NET_IPFORWARD && CONFIG_NET_IPv6 */
