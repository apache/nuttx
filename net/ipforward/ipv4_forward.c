/****************************************************************************
 * net/ipforward/ipv4_forward.c
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
#include <debug.h>
#include <errno.h>

#include <nuttx/mm/iob.h>
#include <nuttx/net/net.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/netstats.h>

#include "netdev/netdev.h"
#include "utils/utils.h"
#include "sixlowpan/sixlowpan.h"
#include "icmp/icmp.h"
#include "ipforward/ipforward.h"
#include "nat/nat.h"
#include "devif/devif.h"

#if defined(CONFIG_NET_IPFORWARD) && defined(CONFIG_NET_IPv4)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ipv4_hdrsize
 *
 * Description:
 *   Return the size of the IPv4 header and the following.
 *
 * Input Parameters:
 *   ipv4  - A pointer to the IPv4 header in within the IPv4 packet.  This
 *           is immediately followed by the L3 header which may be TCP, UDP
 *           or ICMP.
 *
 * Returned Value:
 *   The size of the combined L2 + L3 headers is returned on success.  An
 *   error is returned only if the prototype is not supported.
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_NET_WARN
static int ipv4_hdrsize(FAR struct ipv4_hdr_s *ipv4)
{
  uint16_t iphdrlen;

  /* Get the IP header length (accounting for possible options). */

  iphdrlen = (ipv4->vhl & IPv4_HLMASK) << 2;

  /* Size is also determined by the following protocol header, */

  switch (ipv4->proto)
    {
#ifdef CONFIG_NET_TCP
    case IP_PROTO_TCP:
      {
        FAR struct tcp_hdr_s *tcp =
          (FAR struct tcp_hdr_s *)((FAR uint8_t *)ipv4 + iphdrlen);
        unsigned int tcpsize;

        /* The TCP header length is encoded in the top 4 bits of the
         * tcpoffset field (in units of 32-bit words).
         */

        tcpsize = ((uint16_t)tcp->tcpoffset >> 4) << 2;
        return iphdrlen + tcpsize;
      }
      break;
#endif

#ifdef CONFIG_NET_UDP
    case IP_PROTO_UDP:
      return iphdrlen + UDP_HDRLEN;
      break;
#endif

#ifdef CONFIG_NET_ICMP
    case IP_PROTO_ICMP:
      return iphdrlen + ICMP_HDRLEN;
      break;
#endif

    default:
      nwarn("WARNING: Unrecognized proto: %u\n", ipv4->proto);
      return -EPROTONOSUPPORT;
    }
}
#endif

/****************************************************************************
 * Name: ipv4_decr_ttl
 *
 * Description:
 *   Decrement the IPv4 TTL (time to live value).  TTL field is set by the
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
 *   ipv4  - A pointer to the IPv4 header in within the IPv4 packet to be
 *           forwarded.
 *
 * Returned Value:
 *   The new TTL value is returned.  A value <= 0 means the hop limit has
 *   expired.
 *
 ****************************************************************************/

static int ipv4_decr_ttl(FAR struct ipv4_hdr_s *ipv4)
{
  uint16_t iphdrlen;
  uint16_t sum;
  int ttl;

  /* Check time-to-live (TTL) */

  ttl = (int)ipv4->ttl - 1;
  if (ttl <= 0)
    {
      /* Return zero which must cause the packet to be dropped */

      return 0;
    }

  /* Save the updated TTL value */

  ipv4->ttl = ttl;

  /* Get the IP header length (accounting for possible options). */

  iphdrlen = (ipv4->vhl & IPv4_HLMASK) << 2;

  /* Re-calculate the IPv4 checksum.  This checksum is the Internet checksum
   * of the 20 bytes of the IPv4 header.  This checksum will be different
   * because we just modify the IPv4 TTL.
   */

  ipv4->ipchksum = 0;
  sum            = chksum(0, (FAR const uint8_t *)ipv4, iphdrlen);
  if (sum == 0)
    {
      sum = 0xffff;
    }
  else
    {
      sum = HTONS(sum);
    }

  ipv4->ipchksum = ~sum;
  return ttl;
}

/****************************************************************************
 * Name: ipv4_dev_forward
 *
 * Description:
 *   This function is called from ipv4_forward when it is necessary to
 *   forward a packet from the current device to different device.  In this
 *   case, the forwarding operation must be performed asynchronously when
 *   the TX poll is received from the forwarding device.
 *
 * Input Parameters:
 *   dev      - The device on which the packet was received and which
 *              contains the IPv4 packet.
 *   fwdddev  - The device on which the packet must be forwarded.
 *   ipv4     - A pointer to the IPv4 header in within the IPv4 packet
 *
 * Returned Value:
 *   Zero is returned if the packet was successfully forward;  A negated
 *   errno value is returned if the packet is not forwardable.  In that
 *   latter case, the caller (ipv4_input()) should drop the packet.
 *
 ****************************************************************************/

static int ipv4_dev_forward(FAR struct net_driver_s *dev,
                            FAR struct net_driver_s *fwddev,
                            FAR struct ipv4_hdr_s *ipv4)
{
  FAR struct forward_s *fwd = NULL;
#ifdef CONFIG_DEBUG_NET_WARN
  int hdrsize;
#endif
  int ret;

  /* Verify that the full packet will fit within the forwarding device's MTU
   * if DF is set.
   */

  if (NET_LL_HDRLEN(fwddev) + dev->d_len > NETDEV_PKTSIZE(fwddev)
#ifdef CONFIG_NET_IPFRAG
      && (ipv4->ipoffset[0] & (IP_FLAG_DONTFRAG >> 8))
#endif
     )
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

  fwd->f_dev    = fwddev;  /* Forwarding device */
#ifdef CONFIG_NET_IPv6
  fwd->f_domain = PF_INET; /* IPv4 address domain */
#endif

#ifdef CONFIG_DEBUG_NET_WARN
  /* Get the size of the IPv4 + L3 header. */

  hdrsize = ipv4_hdrsize(ipv4);
  if (hdrsize < IPv4_HDRLEN)
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

  /* Decrement the TTL in the copy of the IPv4 header (retaining the
   * original TTL in the source to handle the broadcast case).  If the
   * TLL decrements to zero, then do not forward the packet.
   */

  ret = ipv4_decr_ttl(ipv4);
  if (ret < 1)
    {
      nwarn("WARNING: Hop limit exceeded... Dropping!\n");
      ret = -EMULTIHOP;
      goto errout_with_fwd;
    }

#ifdef CONFIG_NET_NAT
  /* Try NAT outbound, rule matching will be performed in NAT module. */

  ret = ipv4_nat_outbound(fwd->f_dev, ipv4, NAT_MANIP_SRC);
  if (ret < 0)
    {
      nwarn("WARNING: Performing NAT outbound failed, dropping!\n");
      goto errout_with_fwd;
    }
#endif

  /* Then set up to forward the packet according to the protocol. */

  ret = ipfwd_forward(fwd);
  if (ret >= 0)
    {
      netdev_iob_clear(dev);
      return OK;
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
 * Name: ipv4_forward_callback
 *
 * Description:
 *   This function is a callback from netdev_foreach.  It implements the
 *   the broadcast forwarding action for each network device (other than, of
 *   course, the device that received the packet).
 *
 * Input Parameters:
 *   dev   - The device on which the packet was received and which contains
 *           the IPv4 packet.
 *   ipv4  - A convenience pointer to the IPv4 header in within the IPv4
 *           packet
 *
 * Returned Value:
 *   Typically returns zero (meaning to continue the enumeration), but will
 *   return a non-zero to stop the enumeration if an error occurs.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPFORWARD_BROADCAST
static int ipv4_forward_callback(FAR struct net_driver_s *fwddev,
                                 FAR void *arg)
{
  FAR struct net_driver_s *dev = (FAR struct net_driver_s *)arg;
  FAR struct ipv4_hdr_s *ipv4;
  FAR struct iob_s *iob;
  int ret;

  DEBUGASSERT(fwddev != NULL);

  /* Only IFF_UP device and non-loopback device need forward packet */

  if (!IFF_IS_UP(fwddev->d_flags) || fwddev->d_lltype == NET_LL_LOOPBACK)
    {
      return OK;
    }

  DEBUGASSERT(dev != NULL && dev->d_buf != NULL);

  /* Check if we are forwarding on the same device that we received the
   * packet from.
   */

  if (fwddev != dev)
    {
      /* Backup the forward IP packet */

      iob = iob_tryalloc(true);
      if (iob == NULL)
        {
          nerr("ERROR: iob alloc failed when forward broadcast\n");
          return -ENOMEM;
        }

      iob_reserve(iob, CONFIG_NET_LL_GUARDSIZE);
      ret = iob_clone_partial(dev->d_iob, dev->d_iob->io_pktlen, 0,
                              iob, 0, true, false);
      if (ret < 0)
        {
          iob_free_chain(iob);
          return ret;
        }

      /* Recover the pointer to the IPv4 header in the receiving device's
       * d_buf.
       */

      ipv4 = IPv4BUF;

      /* Send the packet asynchrously on the forwarding device. */

      ret = ipv4_dev_forward(dev, fwddev, ipv4);
      if (ret < 0)
        {
          iob_free_chain(iob);
          nwarn("WARNING: ipv4_dev_forward failed: %d\n", ret);
          return ret;
        }

      /* Restore device iob with backup iob */

      netdev_iob_replace(dev, iob);
    }

  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ipv4_forward
 *
 * Description:
 *   This function is called from ipv4_input when a packet is received that
 *   is not destined for us.  In this case, the packet may need to be
 *   forwarded to another device (or sent back out the same device)
 *   depending configuration, routing table information, and the IPv4
 *   networks served by various network devices.
 *
 * Input Parameters:
 *   dev   - The device on which the packet was received and which contains
 *           the IPv4 packet.
 *   ipv4  - A convenience pointer to the IPv4 header in within the IPv4
 *           packet
 *
 *   On input:
 *   - dev->d_buf holds the received packet.
 *   - dev->d_len holds the length of the received packet MINUS the
 *     size of the L1 header.  That was subtracted out by ipv4_input.
 *   - ipv4 points to the IPv4 header with dev->d_buf.
 *
 * Returned Value:
 *   Zero is returned if the packet was successfully forward;  A negated
 *   errno value is returned if the packet is not forwardable.  In that
 *   latter case, the caller (ipv4_input()) should drop the packet.
 *
 ****************************************************************************/

int ipv4_forward(FAR struct net_driver_s *dev, FAR struct ipv4_hdr_s *ipv4)
{
  in_addr_t destipaddr;
  in_addr_t srcipaddr;
  FAR struct net_driver_s *fwddev;
  int ret;
#ifdef CONFIG_NET_ICMP
  int icmp_reply_type;
  int icmp_reply_code;
#endif /* CONFIG_NET_ICMP */

  /* Search for a device that can forward this packet. */

  destipaddr = net_ip4addr_conv32(ipv4->destipaddr);
  srcipaddr  = net_ip4addr_conv32(ipv4->srcipaddr);

  fwddev     = netdev_findby_ripv4addr(srcipaddr, destipaddr);
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

      ret = ipv4_dev_forward(dev, fwddev, ipv4);
      if (ret < 0)
        {
          nwarn("WARNING: ipv4_dev_forward failed: %d\n", ret);
          goto drop;
        }
    }
  else
    {
      /* Single network device.  The use case here is where an endpoint acts
       * as a hub in a star configuration.  This is typical for a wireless
       * star configuration where not all endpoints are accessible from all
       * other endpoints, but seems less useful for a wired network.
       */

#ifdef CONFIG_NET_ETHERNET
      /* REVISIT:  For Ethernet we may have to fix up the Ethernet header:
       * - source MAC, the MAC of the current device.
       * - dest MAC, the MAC associated with the destination IPv4 address.
       *   This  will involve ICMP.
       */

      /* Correct dev->d_buf by adding back the L1 header length */

#endif

      nwarn("WARNING: Packet forwarding to same device not supportedN\n");
      ret = -ENOSYS;
      goto drop;
    }

  /* Return success.  ipv4_input will return to the network driver with
   * dev->d_len set to the packet size and the network driver will perform
   * the transfer.
   */

  return OK;

drop:
  ipv4_dropstats(ipv4);

#ifdef CONFIG_NET_ICMP
  /* Reply ICMP to the sender for particular errors. */

  switch (ret)
    {
      case -ENETUNREACH:
        icmp_reply_type = ICMP_DEST_UNREACHABLE;
        icmp_reply_code = ICMP_NET_UNREACH;
        goto reply;

      case -EFBIG:
        icmp_reply_type = ICMP_DEST_UNREACHABLE;
        icmp_reply_code = ICMP_FRAG_NEEDED;
        goto reply;

      case -EMULTIHOP:
        icmp_reply_type = ICMP_TIME_EXCEEDED;
        icmp_reply_code = ICMP_EXC_TTL;
        goto reply;

      default:
        break; /* We don't know how to reply, just go on (to drop). */
    }
#endif /* CONFIG_NET_ICMP */

  dev->d_len = 0;
  return ret;

#ifdef CONFIG_NET_ICMP
reply:
#  ifdef CONFIG_NET_NAT
  /* Before we reply ICMP, call NAT outbound to try to translate destination
   * address & port back to original status.
   */

  ipv4_nat_outbound(dev, ipv4, NAT_MANIP_DST);
#  endif /* CONFIG_NET_NAT */

  icmp_reply(dev, icmp_reply_type, icmp_reply_code);
  return OK;
#endif /* CONFIG_NET_ICMP */
}

/****************************************************************************
 * Name: ipv4_forward_broadcast
 *
 * Description:
 *   This function is called from ipv4_input when a broadcast or multicast
 *   packet is received.  If CONFIG_NET_IPFORWARD_BROADCAST is enabled, this
 *   function will forward the broadcast packet to other networks through
 *   other network devices.
 *
 * Input Parameters:
 *   dev   - The device on which the packet was received and which contains
 *           the IPv4 packet.
 *   ipv4  - A convenience pointer to the IPv4 header in within the IPv4
 *           packet
 *
 *   On input:
 *   - dev->d_buf holds the received packet.
 *   - dev->d_len holds the length of the received packet MINUS the
 *     size of the L1 header.  That was subtracted out by ipv4_input.
 *   - ipv4 points to the IPv4 header with dev->d_buf.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPFORWARD_BROADCAST
void ipv4_forward_broadcast(FAR struct net_driver_s *dev,
                            FAR struct ipv4_hdr_s *ipv4)
{
  /* Don't bother if the TTL would expire */

  if (ipv4->ttl > 1)
    {
      /* Forward the the broadcast/multicast packet to all devices except,
       * of course, the device that received the packet.
       */

      netdev_foreach(ipv4_forward_callback, dev);
    }
}
#endif

#endif /* CONFIG_NET_IPFORWARD && CONFIG_NET_IPv4 */
