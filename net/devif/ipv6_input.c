/****************************************************************************
 * net/devif/ipv6_input.c
 * Device driver IPv6 packet receipt interface
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
#ifdef CONFIG_NET_IPv6

#include <sys/ioctl.h>
#include <stdint.h>
#include <stdbool.h>
#include <debug.h>
#include <string.h>

#include <net/if.h>

#include <nuttx/net/netconfig.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/netstats.h>
#include <nuttx/net/ip.h>
#include <nuttx/net/ipv6ext.h>

#include "neighbor/neighbor.h"
#include "tcp/tcp.h"
#include "udp/udp.h"
#include "sixlowpan/sixlowpan.h"
#include "pkt/pkt.h"
#include "icmpv6/icmpv6.h"

#include "netdev/netdev.h"
#include "ipforward/ipforward.h"
#include "inet/inet.h"
#include "devif/devif.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Macros */

#define IPv6BUF ((FAR struct ipv6_hdr_s *)&dev->d_buf[NET_LL_HDRLEN(dev)])
#define PAYLOAD ((FAR uint8_t *)&dev->d_buf[NET_LL_HDRLEN(dev)] + IPv6_HDRLEN)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ipv6_exthdr
 *
 * Description:
 *   Return true if the next header value is an IPv6 extension header.
 *
 ****************************************************************************/

static bool ipv6_exthdr(uint8_t nxthdr)
{
  switch (nxthdr)
    {
      case NEXT_HOPBYBOT_EH:    /* Hop-by-Hop Options Header */
      case NEXT_ENCAP_EH:       /* Encapsulated IPv6 Header */
      case NEXT_ROUTING_EH:     /* Routing Header */
      case NEXT_FRAGMENT_EH:    /* Fragment Header */
      case NEXT_RRSVP_EH:       /* Resource ReSerVation Protocol */
      case NEXT_ENCAPSEC_EH:    /* Encapsulating Security Payload */
      case NEXT_AUTH_EH:        /* Authentication Header */
      case NEXT_DESTOPT_EH:     /* Destination Options Header */
      case NEXT_MOBILITY_EH:    /* Mobility */
      case NEXT_HOSTID_EH:      /* Host Identity Protocol */
      case NEXT_SHIM6_EH:       /* Shim6 Protocol */
        return true;

      case NEXT_NOHEADER:       /* No next header */
      default:
        return false;
    }
}

/****************************************************************************
 * Name: check_dev_destipaddr
 *
 * Description:
 *   Check if the destination address in the IPv6 is destined for the
 *   provided network device.
 *
 * Returned Value:
 *   1 - This packet is destined for this network device
 *   0 - This packet is NOT destined for this network device
 *
 ****************************************************************************/

static int check_dev_destipaddr(FAR struct net_driver_s *dev, FAR void *arg)
{
  FAR struct ipv6_hdr_s *ipv6 = (FAR struct ipv6_hdr_s *)arg;

  /* Check if the IPv6 destination address matches the IPv6 address assigned
   * to this device.
   */

  if (net_ipv6addr_cmp(ipv6->destipaddr, dev->d_ipv6addr))
    {
      return 1;
    }

  /* No match, return 0 to keep searching */

  return 0;
}

/****************************************************************************
 * Name: check_destipaddr
 *
 * Description:
 *   Check if the destination address in the IPv6 is destined for us.  This
 *   is typically just a comparison the of the IPv6 destination address in
 *   the IPv6 packet with the IPv6 address assigned to the receiving device.
 *
 * Input Parameters:
 *   dev   - The device on which the packet was received and which contains
 *           the IPv6 packet.
 *   ipv6  - A convenience pointer to the IPv6 header in within the IPv6
 *           packet
 *
 * Returned Value:
 *   true  - This packet is destined for us
 *   false - This packet is NOT destined for us and may need to be forwarded.
 *
 ****************************************************************************/

static bool check_destipaddr(FAR struct net_driver_s *dev,
                             FAR struct ipv6_hdr_s *ipv6)
{
  int ret;

  /* For IPv6, packet reception is a little trickier as we need to make sure
   * that we listen to certain multicast addresses (all hosts multicast
   * address, and the solicited-node multicast address) as well.  However,
   * we will cheat here and accept all multicast packets that are sent to
   * the ff02::/16 addresses.
   */

  if (ipv6->destipaddr[0] == HTONS(0xff02))
    {
#ifdef CONFIG_NET_IPFORWARD_BROADCAST
      /* Forward multicast packets */

      ipv6_forward_broadcast(dev, ipv6);
#endif
      return true;
    }

  /* We will also allow for a perverse case where we receive a packet
   * addressed to us, but on a different device.  Can that really happen?
   */

  ret = netdev_foreach(check_dev_destipaddr, ipv6);
  if (ret == 1)
    {
      /* The traversal of the network devices will return 0 if there is
       * no network device with that address or 1 if there is a network
       * device with such an address.
       */

      return true;
    }

  return false;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ipv6_input
 *
 * Description:
 *   Receive an IPv6 packet from the network device.  Verify and forward to
 *   L3 packet handling logic if the packet is destined for us.
 *
 * Input Parameters:
 *   dev   - The device on which the packet was received and which contains
 *           the IPv6 packet.
 * Returned Value:
 *   OK    - The packet was processed (or dropped) and can be discarded.
 *   ERROR - Hold the packet and try again later.  There is a listening
 *           socket but no receive in place to catch the packet yet.  The
 *           device's d_len will be set to zero in this case as there is
 *           no outgoing data.
 *
 *   If this function returns to the network driver with dev->d_len > 0,
 *   that is an indication to the driver that there is an outgoing response
 *   to this input.
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

int ipv6_input(FAR struct net_driver_s *dev)
{
  FAR struct ipv6_hdr_s *ipv6 = IPv6BUF;
  FAR uint8_t *payload;
  uint16_t llhdrlen;
  uint16_t iphdrlen;
  uint16_t paylen;
  uint8_t nxthdr;
#ifdef CONFIG_NET_IPFORWARD
  int ret;
#endif

  /* This is where the input processing starts. */

#ifdef CONFIG_NET_STATISTICS
  g_netstats.ipv6.recv++;
#endif

  /* Start of IP input header processing code.
   *
   * Check validity of the IP header.
   */

  if ((ipv6->vtc & 0xf0) != 0x60)
    {
      /* IP version and header length. */

      nwarn("WARNING: Invalid IPv6 version: %d\n", ipv6->vtc >> 4);

#ifdef CONFIG_NET_STATISTICS
      g_netstats.ipv6.vhlerr++;
#endif
      goto drop;
    }

  /* Get the size of the packet minus the size of link layer header */

  llhdrlen = NET_LL_HDRLEN(dev);
  if ((llhdrlen + IPv6_HDRLEN) > dev->d_len)
    {
      nwarn("WARNING: Packet shorter than IPv6 header\n");
      goto drop;
    }

  dev->d_len -= llhdrlen;

  /* Make sure that all packet processing logic knows that there is an IPv6
   * packet in the device buffer.
   */

  IFF_SET_IPv6(dev->d_flags);

  /* Check the size of the packet. If the size reported to us in d_len is
   * smaller the size reported in the IP header, we assume that the packet
   * has been corrupted in transit. If the size of d_len is larger than the
   * size reported in the IP packet header, the packet has been padded and
   * we set d_len to the correct value.
   *
   * The length reported in the IPv6 header is the length of the payload
   * that follows the header.  The device interface uses the d_len variable
   * for holding the size of the entire packet, including the IP header but
   * without the link layer header (subtracted out above).
   *
   * NOTE: The payload length in the includes the size of the Ipv6 extension
   * options, but not the size of the IPv6 header.
   *
   * REVISIT:  Length will be set to zero if the extension header carries
   * a Jumbo payload option.
   */

  paylen = ((uint16_t)ipv6->len[0] << 8) + (uint16_t)ipv6->len[1] +
           IPv6_HDRLEN;

  if (paylen <= dev->d_len)
    {
      dev->d_len = paylen;
    }
  else
    {
      nwarn("WARNING: IP packet shorter than length in IP header\n");
      goto drop;
    }

  /* Parse IPv6 extension headers (parsed but ignored) */

  payload  = PAYLOAD;     /* Assume payload starts right after IPv6 header */
  iphdrlen = IPv6_HDRLEN; /* Total length of the IPv6 header */
  nxthdr   = ipv6->proto; /* Next header determined by IPv6 header prototype */

  while (ipv6_exthdr(nxthdr))
    {
      FAR struct ipv6_extension_s *exthdr;
      uint16_t extlen;

      /* Just skip over the extension header */

      exthdr    = (FAR struct ipv6_extension_s *)payload;
      extlen    = EXTHDR_LEN((unsigned int)exthdr->len);
      payload  += extlen;
      iphdrlen += extlen;
      nxthdr    = exthdr->nxthdr;
    }

#ifdef CONFIG_NET_BROADCAST
  /* Check for a multicast packet, which may be destined to us (even if
   * there is no IP address yet assigned to the device).  We only expect
   * multicast packets destined for sockets that have joined a multicast
   * group or for ICMPv6 Autoconfiguration and Neighbor discovery or ICMPv6
   * MLD packets.
   *
   * We should actually pick off certain multicast address (all hosts
   * multicast address, and the solicited-node multicast address).  We
   * will cheat here and accept all multicast packets that are sent to the
   * ff00::/8 addresses (see net_is_addr_mcast).
   */

  if (net_is_addr_mcast(ipv6->destipaddr))
    {
#ifdef CONFIG_NET_IPFORWARD_BROADCAST

      /* Packets sent to ffx0 are reserved, ffx1 are interface-local, and
       * ffx2 are interface-local, and therefore, should not be forwarded
       */

      if (((ipv6->destipaddr[0] & HTONS(0xff0f)) != HTONS(0xff00)) &&
          ((ipv6->destipaddr[0] & HTONS(0xff0f)) != HTONS(0xff01)) &&
          ((ipv6->destipaddr[0] & HTONS(0xff0f)) != HTONS(0xff02)))
        {
          /* Forward broadcast packets */

          ipv6_forward_broadcast(dev, ipv6);
        }
#endif

      /* Fall through with no further address checks and handle the multicast
       * address by its IPv6 nexthdr field.
       */
    }
  else
#endif
    {
      /* Check if the packet is destined for us. */

      if (!check_destipaddr(dev, ipv6))
        {
#ifdef CONFIG_NET_IPFORWARD
          /* Not destined for us, try to forward the packet */

          ret = ipv6_forward(dev, ipv6);
          if (ret >= 0)
            {
              /* The packet was forwarded.  Return success; d_len will
               * be set appropriately by the forwarding logic:  Cleared
               * if the packet is forward via another device or non-
               * zero if it will be forwarded by the same device that
               * it was received on.
               */

              return OK;
            }
          else
#endif
#if defined(NET_UDP_HAVE_STACK) && defined(CONFIG_NET_UDP_BINDTODEVICE)
          /* If the UDP protocol specific socket option UDP_BINDTODEVICE
           * is selected, then we must forward all UDP packets to the bound
           * socket.
           */

          if (nxthdr != IP_PROTO_UDP || !IFF_IS_BOUND(dev->d_flags))
#endif
            {
              /* Not destined for us and not forwardable...
               * drop the packet.
               */

              nwarn("WARNING: Not destined for us... Dropping!\n");
              goto drop;
            }
        }
    }
#ifdef CONFIG_NET_ICMPv6

  /* In other cases, the device must be assigned a non-zero IP address
   * (the all zero address is the "unspecified" address.
   */

  if (net_ipv6addr_cmp(dev->d_ipv6addr, g_ipv6_unspecaddr))
    {
      nwarn("WARNING: No IP address assigned\n");
      goto drop;
    }
#endif

  /* Now process the incoming packet according to the protocol specified in
   * the next header IPv6 field.
   */

  switch (nxthdr)
    {
#ifdef NET_TCP_HAVE_STACK
      case IP_PROTO_TCP:   /* TCP input */

        /* Forward the IPv6 TCP packet */

        tcp_ipv6_input(dev, iphdrlen);

#ifdef CONFIG_NET_6LOWPAN
        /* TCP output comes through three different mechanisms.  Either from:
         *
         *   1. TCP socket output.  For the case of TCP output to an
         *      IEEE802.15.4, the TCP output is caught in the socket
         *      send()/sendto() logic and and redirected to 6LoWPAN logic.
         *   2. TCP output from the TCP state machine.  That will occur
         *      during TCP packet processing by the TCP state machine.
         *   3. TCP output resulting from TX or timer polling
         *
         * Case 3 is handled here.  Logic here detects if (1) an attempt
         * to return with d_len > 0 and (2) that the device is an
         * IEEE802.15.4 MAC network driver. Under those conditions, 6LoWPAN
         * logic will be called to create the IEEE80215.4 frames.
         */

        if (dev->d_len > 0 && dev->d_lltype == CONFIG_NET_6LOWPAN)
          {
            /* Let 6LoWPAN handle the TCP output */

            sixlowpan_tcp_send(dev, dev, ipv6);

            /* Drop the packet in the d_buf */

            goto drop;
          }
#endif /* CONFIG_NET_6LOWPAN */
        break;
#endif /* NET_TCP_HAVE_STACK */

#ifdef NET_UDP_HAVE_STACK
      case IP_PROTO_UDP:   /* UDP input */

        /* Forward the IPv6 UDP packet */

        udp_ipv6_input(dev, iphdrlen);
        break;
#endif

      /* Check for ICMP input */

#ifdef NET_ICMPv6_HAVE_STACK
      case IP_PROTO_ICMP6: /* ICMP6 input */

        /* Forward the ICMPv6 packet */

        icmpv6_input(dev, iphdrlen);

#ifdef CONFIG_NET_6LOWPAN
        /* All outgoing ICMPv6 messages come through one of two mechanisms:
         *
         *   1. The output from internal ICMPv6 message passing.  These
         *      outgoing  messages will use device polling and will be
         *      handled elsewhere.
         *   2. ICMPv6 output resulting from TX or timer polling.
         *
         * Case 2 is handled here.  Logic here detects if (1) an attempt
         * to return with d_len > 0 and (2) that the device is an
         * IEEE802.15.4 MAC network driver. Under those conditions, 6LoWPAN
         * logic will be called to create the IEEE80215.4 frames.
         */

        if (dev->d_len > 0 && dev->d_lltype == CONFIG_NET_6LOWPAN)
          {
            /* Let 6LoWPAN handle the ICMPv6 output */

            sixlowpan_icmpv6_send(dev, dev, ipv6);

            /* Drop the packet in the d_buf */

            goto drop;
          }
#endif /* CONFIG_NET_6LOWPAN */
        break;
#endif /* NET_ICMPv6_HAVE_STACK */

      default:              /* Unrecognized/unsupported protocol */
        nwarn("WARNING: Unrecognized IP protocol: %04x\n", ipv6->proto);

#ifdef CONFIG_NET_STATISTICS
        g_netstats.ipv6.protoerr++;
#endif
        goto drop;
    }

  /* Return and let the caller do any pending transmission. */

  return OK;

  /* Drop the packet.  NOTE that OK is returned meaning that the
   * packet has been processed (although processed unsuccessfully).
   */

drop:
#ifdef CONFIG_NET_STATISTICS
  g_netstats.ipv6.drop++;
#endif
  dev->d_len = 0;
  return OK;
}
#endif /* CONFIG_NET_IPv6 */
