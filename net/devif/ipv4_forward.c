/****************************************************************************
 * net/devif/ipv4_forward.c
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

#include <string.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/mm/iob.h>
#include <nuttx/net/net.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/netstats.h>

#include "netdev/netdev.h"
#include "sixlowpan/sixlowpan.h"
#include "udp/udp.h"
#include "tcp/tcp.h"
#include "icmp/icmp.h"
#include "devif/ip_forward.h"
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
 *           is immeidately followed by the L3 header which may be TCP, UDP
 *           or ICMP.
 *
 * Returned Value:
 *   The size of the combined L2 + L3 headers is returned on success.  An
 *   error is returned only if the prototype is not supported.
 *
 ****************************************************************************/

#ifdef CONFIG_NETDEV_MULTINIC
static int ipv4_hdrsize(FAR struct ipv4_hdr_s *ipv4)
{
  /* Size is determined by the following protocol header, */

  switch (ipv4->proto)
    {
#ifdef CONFIG_NET_TCP
    case IP_PROTO_TCP:
      {
        FAR struct tcp_hdr_s *tcp =
          (FAR struct tcp_hdr_s *)((FAR uintptr_t *)ipv4 + IPv4_HDRLEN);
        unsigned int tcpsize;

        /* The TCP header length is encoded in the top 4 bits of the
         * tcpoffset field (in units of 32-bit words).
         */

        tcpsize = ((uint16_t)tcp->tcpoffset >> 4) << 2;
        return IPv4_HDRLEN + tcpsize;
      }
      break;
#endif

#ifdef CONFIG_NET_UDP
    case IP_PROTO_UDP:
      return IPv4_HDRLEN + UDP_HDRLEN;
      break;
#endif

#ifdef CONFIG_NET_ICMP
    case IP_PROTO_ICMP:
      return IPv4_HDRLEN + ICMP_HDRLEN;
      break;
#endif

    default:
      nwarn("WARNING: Unrecognized proto: %u\n", ipv4->proto);
      return -EPROTONOSUPPORT;
    }
}
#endif

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

#ifdef CONFIG_NETDEV_MULTINIC
static int ipv4_dev_forward(FAR struct net_driver_s *dev,
                            FAR struct net_driver_s *fwddev,
                            FAR struct ipv4_hdr_s *ipv4)
{
  FAR struct forward_s *fwd = NULL;
  FAR uint8_t *payload;
  unsigned int paysize;
  int hdrsize;
  int ret;

  /* Verify that the full packet will fit within the forwarding devices MTU.
   * We provide no support for fragmenting forwarded packets.
   */

  if (NET_LL_HDRLEN(fwddev) + dev->d_len > NET_DEV_MTU(fwddev))
    {
      nwarn("WARNING: Packet > MTU... Dropping\n");
      ret = -EFBIG;
      goto errout;
    }

  /* Get a pre-allocated forwarding structure,  This structure will be
   * completely zeroed when we receive it.
   */

  fwd = ip_forward_alloc();
  if (fwd == NULL)
    {
      nwarn("WARNING: Failed to allocate forwarding structure\n");
      ret = -ENOMEM;
      goto errout;
    }

  /* Initialize the easy stuff in the forwarding structure */

  fwd->f_dev = fwddev;   /* Forwarding device */

  /* Get the size of the IPv4 + L3 header.  Use this to determine start of
   * the data payload.
   *
   * Remember that the size of the L1 header has already been subtracted
   * from dev->d_len.
   *
   * REVISIT: Consider an alternative design that does not require data
   * copying.  This would require a pool of d_buf's that are managed by
   * the network rather than the network device.
   */

  hdrsize = ipv4_hdrsize(ipv4);
  if (hdrsize < IPv4_HDRLEN)
    {
      nwarn("WARNING: Could not determine L2+L3 header size\n");
      ret = -EPROTONOSUPPORT;
      goto errout_with_fwd;
    }

  /* Save the entire L2 and L3 headers in the state structure */

  if (hdrsize >  sizeof(union fwd_iphdr_u))
    {
      nwarn("WARNING: Header is too big for pre-allocated structure\n");
      ret = -E2BIG;
      goto errout_with_fwd;
    }

  memcpy(&fwd->f_hdr, ipv4, hdrsize);
  fwd->f_hdrsize = hdrsize;

  /* Use the L2 + L3 header size to determine start and size of the data
   * payload.
   *
   * Remember that the size of the L1 header has already been subtracted
   * from dev->d_len.
   */

  payload = (FAR uint8_t *)ipv4 + hdrsize;
  paysize = dev->d_len - hdrsize;

  /* If there is a payload, then copy it into an IOB chain.
   *
   * REVISIT: Consider an alternative design that does not require data
   * copying.  This would require a pool of d_buf's that are managed by
   * the network rather than the network device.
   */

  if (paysize > 0)
    {
      /* Try to allocate the head of an IOB chain.  If this fails, the
       * packet will be dropped; we are not operating in a context
       * where waiting for an IOB is a good idea
       */

      fwd->f_iob = iob_tryalloc(false);
      if (fwd->f_iob == NULL)
        {
          nwarn("WARNING: iob_tryalloc() failed\n");
          ret = -ENOMEM;
          goto errout_with_fwd;
        }

      /* Copy the packet data payload into an IOB chain. iob_trycopin() will
       * not wait, but will fail there are no available IOBs.
       */

      ret = iob_trycopyin(fwd->f_iob, payload, paysize, 0, false);
      if (ret < 0)
        {
          nwarn("WARNING: iob_trycopyin() failed: %d\n", ret);
          goto errout_with_iobchain;
        }
    }

  /* Then set up to forward the packet according to the protocol.
   *
   * REVISIT: Are these protocol specific forwarders necessary?  I think
   * that this could be done with a single forwarding function for all
   * protocols.
   */

  switch (ipv4->proto)
    {
#ifdef CONFIG_NET_TCP
    case IP_PROTO_TCP:
      {
        /* Forward a TCP packet. */

        ret = tcp_forward(fwd);
      }
      break;
#endif

#ifdef CONFIG_NET_UDP
    case IP_PROTO_UDP:
      {
        /* Forward a UDP packet */

        ret = udp_forward(fwd);
      }
      break;
#endif

    case IP_PROTO_ICMP: /* Not yet supported */
    default:
      nwarn("WARNING: Unrecognized proto: %u\n", ipv4->proto);
      ret = -EPROTONOSUPPORT;
      break;
    }

  if (ret >= 0)
    {
      dev->d_len = 0;
      return OK;
    }

errout_with_iobchain:
  if (fwd != NULL && fwd->f_iob != NULL)
    {
      iob_free_chain(fwd->f_iob);
    }

errout_with_fwd:
  if (fwd != NULL)
    {
      ip_forward_free(fwd);
    }

errout:
  return ret;
}
#endif /* CONFIG_NETDEV_MULTINIC */

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
  int ttl = (int)ipv4->ttl - 1;

  if (ttl <= 0)
    {
#ifdef CONFIG_NET_ICMP
      /* Return an ICMP error packet back to the sender. */
#warning Missing logic
#endif

      /* Return zero which must cause the packet to be dropped */

      return 0;
    }

  /* Save the updated TTL value */

  ipv4->ttl = ttl;

  /* NOTE: We do not have to recalculate the IPv4 checksum because (1) the
   * IPv4 header does not include a checksum itself and (2) the TTL is not
   * included in the sum for the TCP and UDP headers.
   */

  return ttl;
}

/****************************************************************************
 * Name: ipv4_dropstats
 *
 * Description:
 *   Update statistics for a dropped packet.
 *
 * Input Parameters:
 *   ipv4  - A convenience pointer to the IPv4 header in within the IPv4
 *           packet to be dropped.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_NET_STATISTICS
static void ipv4_dropstats(FAR struct ipv4_hdr_s *ipv4)
{
  switch (ipv4->proto)
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

#ifdef CONFIG_NET_ICMP
    case IP_PROTO_ICMP:
      g_netstats.icmp.drop++;
      break;
#endif

    default:
      break;
    }

  g_netstats.ipv4.drop++;
}
#else
#  define ipv4_dropstats(ipv4)
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
#ifdef CONFIG_NETDEV_MULTINIC
  in_addr_t srcipaddr;
#endif
  FAR struct net_driver_s *fwddev;
  int ret;

  /* Decrement the TTL.  If it decrements to zero, then drop the packet */

  ret = ipv4_decr_ttl(ipv4);
  if (ret < 1)
    {
      nwarn("WARNING: Hop limit exceeded... Dropping!\n");
      ret = -EMULTIHOP;
      goto drop;
    }

  /* Re-calculate the IPv4 checksum.  This checksum is the Internet checksum
   * of the 20 bytes of the IPv4 header.  This checksum will be different
   * because we just modify the IPv4 TTL.
   */

  ipv4->ipchksum = 0;
  ipv4->ipchksum = ~ipv4_chksum(dev);

  /* Search for a device that can forward this packet.  This is a trivial
   * search if there is only a single network device (CONFIG_NETDEV_MULTINIC
   * not defined).  But netdev_findby_ipv4addr() will still assure
   * routability in that case.
   */

  destipaddr = net_ip4addr_conv32(ipv4->destipaddr);
#ifdef CONFIG_NETDEV_MULTINIC
  srcipaddr  = net_ip4addr_conv32(ipv4->srcipaddr);

  fwddev     = netdev_findby_ipv4addr(srcipaddr, destipaddr);
#else
  fwddev     = netdev_findby_ipv4addr(destipaddr);
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
      /* Send the packet asynchrously on the forwarding device. */

      ret = ipv4_dev_forward(dev, fwddev, ipv4);
      if (ret < 0)
        {
          nwarn("WARNING: ipv4_dev_forward faield: %d\n", ret);
          goto drop;
        }
    }
  else
#endif /* CONFIG_NETDEV_MULTINIC */

    {
      /* Single network device.  The use case here is where an endpoint acts
       * as a hub in a star configuration.  This is typical for a wireless star
       * configuration where not all endpoints are accessible from all other
       * endpoints, but seems less useful for a wired network.
       */

#ifdef CONFIG_NET_ETHERNET
      /* REVISIT:  For Ethernet we may have to fix up the Ethernet header:
       * - source MAC, the MAC of the current device.
       * - dest MAC, the MAC associated with the destination IPv4 adress.
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
  dev->d_len = 0;
  return ret;
}

#endif /* CONFIG_NET_IPFORWARD && CONFIG_NET_IPv4 */
