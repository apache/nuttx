/****************************************************************************
 * net/devif/ipv4_input.c
 * Device driver IPv4 packet receipt interface
 *
 *   Copyright (C) 2007-2009, 2013-2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Adapted for NuttX from logic in uIP which also has a BSD-like license:
 *
 * uIP is an implementation of the TCP/IP protocol stack intended for
 * small 8-bit and 16-bit microcontrollers.
 *
 * uIP provides the necessary protocols for Internet communication,
 * with a very small code footprint and RAM requirements - the uIP
 * code size is on the order of a few kilobytes and RAM usage is on
 * the order of a few hundred bytes.
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
 * uIP is a small implementation of the IP, UDP and TCP protocols (as
 * well as some basic ICMP stuff). The implementation couples the IP,
 * UDP, TCP and the application layers very tightly. To keep the size
 * of the compiled code down, this code frequently uses the goto
 * statement. While it would be possible to break the ipv4_input()
 * function into many smaller functions, this would increase the code
 * size because of the overhead of parameter passing and the fact that
 * the optimizer would not be as efficient.
 *
 * The principle is that we have a small buffer, called the d_buf,
 * in which the device driver puts an incoming packet. The TCP/IP
 * stack parses the headers in the packet, and calls the
 * application. If the remote host has sent data to the application,
 * this data is present in the d_buf and the application read the
 * data from there. It is up to the application to put this data into
 * a byte stream if needed. The application will not be fed with data
 * that is out of sequence.
 *
 * If the application wishes to send data to the peer, it should put
 * its data into the d_buf. The d_appdata pointer points to the
 * first available byte. The TCP/IP stack will calculate the
 * checksums, and fill in the necessary header fields and finally send
 * the packet back to the peer.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#ifdef CONFIG_NET_IPv4

#include <sys/ioctl.h>
#include <stdint.h>
#include <debug.h>
#include <string.h>

#include <netinet/in.h>
#include <net/if.h>

#include <nuttx/net/netconfig.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/netstats.h>
#include <nuttx/net/ip.h>

#include "tcp/tcp.h"
#include "udp/udp.h"
#include "pkt/pkt.h"
#include "icmp/icmp.h"
#include "igmp/igmp.h"

#include "devif/devif.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Macros */

#define BUF                  ((FAR struct ipv4_hdr_s *)&dev->d_buf[NET_LL_HDRLEN(dev)])
#define FBUF                 ((FAR struct ipv4_hdr_s *)&g_reassembly_buffer[0])

/* IP fragment re-assembly */

#define IP_MF                0x20  /* See IP_FLAG_MOREFRAGS */
#define TCP_REASS_BUFSIZE    (NET_DEV_MTU(dev) - NET_LL_HDRLEN(dev))
#define TCP_REASS_LASTFRAG   0x01

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

#if defined(CONFIG_NET_TCP_REASSEMBLY) && !defined(CONFIG_NET_IPv6)

static uint8_t g_reassembly_buffer[TCP_REASS_BUFSIZE];
static uint8_t g_reassembly_bitmap[TCP_REASS_BUFSIZE / (8 * 8)];

static const uint8_t g_bitmap_bits[8] =
  {0xff, 0x7f, 0x3f, 0x1f, 0x0f, 0x07, 0x03, 0x01};

static uint16_t g_reassembly_len;
static uint8_t g_reassembly_flags;

#endif /* CONFIG_NET_TCP_REASSEMBLY */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Function: devif_reassembly
 *
 * Description:
 *   IP fragment reassembly: not well-tested.
 *
 * Assumptions:
 *
 ****************************************************************************/

#if defined(CONFIG_NET_TCP_REASSEMBLY) && !defined(CONFIG_NET_IPv6)
static uint8_t devif_reassembly(void)
{
  FAR struct ipv4_hdr_s *pbuf  = BUF;
  FAR struct ipv4_hdr_s *pfbuf = FBUF;
  uint16_t offset;
  uint16_t len;
  uint16_t i;

  /* If g_reassembly_timer is zero, no packet is present in the buffer, so
   * we write the IP header of the fragment into the reassembly buffer.  The
   * timer is updated with the maximum age.
   */

  if (!g_reassembly_timer)
    {
      memcpy(g_reassembly_buffer, &pbuf->vhl, IPv4_HDRLEN);
      g_reassembly_timer = CONFIG_NET_TCP_REASS_MAXAGE;
      g_reassembly_flags = 0;

      /* Clear the bitmap. */

      memset(g_reassembly_bitmap, 0, sizeof(g_reassembly_bitmap));
    }

  /* Check if the incoming fragment matches the one currently present
   * in the reassembly buffer. If so, we proceed with copying the
   * fragment into the buffer.
   */

  if (net_ipv4addr_hdrcmp(pbuf->srcipaddr, pfbuf->srcipaddr) &&
      net_ipv4addr_hdrcmp(pbuf->destipaddr, pfbuf->destipaddr) &&
      pbuf->g_ipid[0] == pfbuf->g_ipid[0] && pbuf->g_ipid[1] == pfbuf->g_ipid[1])
    {
      len = (pbuf->len[0] << 8) + pbuf->len[1] - (pbuf->vhl & 0x0f) * 4;
      offset = (((pbuf->ipoffset[0] & 0x3f) << 8) + pbuf->ipoffset[1]) * 8;

      /* If the offset or the offset + fragment length overflows the
       * reassembly buffer, we discard the entire packet.
       */

      if (offset > TCP_REASS_BUFSIZE || offset + len > TCP_REASS_BUFSIZE)
        {
          g_reassembly_timer = 0;
          goto nullreturn;
        }

      /* Copy the fragment into the reassembly buffer, at the right offset. */

      memcpy(&g_reassembly_buffer[IPv4_HDRLEN + offset], (char *)pbuf + (int)((pbuf->vhl & 0x0f) * 4), len);

    /* Update the bitmap. */

    if (offset / (8 * 8) == (offset + len) / (8 * 8))
      {
        /* If the two endpoints are in the same byte, we only update that byte. */

        g_reassembly_bitmap[offset / (8 * 8)] |=
          g_bitmap_bits[(offset / 8 ) & 7] & ~g_bitmap_bits[((offset + len) / 8 ) & 7];

      }
    else
      {
        /* If the two endpoints are in different bytes, we update the bytes
         * in the endpoints and fill the stuff inbetween with 0xff.
         */

        g_reassembly_bitmap[offset / (8 * 8)] |= g_bitmap_bits[(offset / 8 ) & 7];
        for (i = 1 + offset / (8 * 8); i < (offset + len) / (8 * 8); ++i)
          {
            g_reassembly_bitmap[i] = 0xff;
          }

        g_reassembly_bitmap[(offset + len) / (8 * 8)] |=
          ~g_bitmap_bits[((offset + len) / 8 ) & 7];
      }

    /* If this fragment has the More Fragments flag set to zero, we know that
     * this is the last fragment, so we can calculate the size of the entire
     * packet. We also set the IP_REASS_FLAG_LASTFRAG flag to indicate that
     * we have received the final fragment.
     */

    if ((pbuf->ipoffset[0] & IP_MF) == 0)
      {
        g_reassembly_flags |= TCP_REASS_LASTFRAG;
        g_reassembly_len = offset + len;
      }

    /* Finally, we check if we have a full packet in the buffer. We do this
     * by checking if we have the last fragment and if all bits in the bitmap
     * are set.
     */

    if (g_reassembly_flags & TCP_REASS_LASTFRAG)
      {
        /* Check all bytes up to and including all but the last byte in
         * the bitmap.
         */

        for (i = 0; i < g_reassembly_len / (8 * 8) - 1; ++i)
          {
            if (g_reassembly_bitmap[i] != 0xff)
              {
                goto nullreturn;
              }
          }

        /* Check the last byte in the bitmap. It should contain just the
         * right amount of bits.
         */

        if (g_reassembly_bitmap[g_reassembly_len / (8 * 8)] != (uint8_t)~g_bitmap_bits[g_reassembly_len / 8 & 7])
          {
            goto nullreturn;
          }

        /* If we have come this far, we have a full packet in the buffer,
         * so we allocate a pbuf and copy the packet into it. We also reset
         * the timer.
         */

        g_reassembly_timer = 0;
        memcpy(pbuf, pfbuf, g_reassembly_len);

        /* Pretend to be a "normal" (i.e., not fragmented) IP packet from
         * now on.
         */

        pbuf->ipoffset[0] = pbuf->ipoffset[1] = 0;
        pbuf->len[0] = g_reassembly_len >> 8;
        pbuf->len[1] = g_reassembly_len & 0xff;
        pbuf->ipchksum = 0;
        pbuf->ipchksum = ~(ipv4_chksum(dev));

        return g_reassembly_len;
      }
  }

nullreturn:
  return 0;
}
#endif /* CONFIG_NET_TCP_REASSEMBLY */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: ipv4_input
 *
 * Description:
 *
 * Returned Value:
 *   OK    The packet was processed (or dropped) and can be discarded.
 *   ERROR There is a matching connection, but could not dispatch the packet
 *         yet.  Currently useful for UDP when a packet arrives before a recv
 *         call is in place.
 *
 * Assumptions:
 *
 ****************************************************************************/

int ipv4_input(FAR struct net_driver_s *dev)
{
  FAR struct ipv4_hdr_s *pbuf = BUF;
  uint16_t iplen;

  /* This is where the input processing starts. */

#ifdef CONFIG_NET_STATISTICS
  g_netstats.ipv4.recv++;
#endif

  /* Start of IP input header processing code. */
  /* Check validity of the IP header. */

  if (pbuf->vhl != 0x45)
    {
      /* IP version and header length. */

#ifdef CONFIG_NET_STATISTICS
      g_netstats.ipv4.drop++;
      g_netstats.ipv4.vhlerr++;
#endif
      nlldbg("Invalid IP version or header length: %02x\n", pbuf->vhl);
      goto drop;
    }

  /* Check the size of the packet. If the size reported to us in d_len is
   * smaller the size reported in the IP header, we assume that the packet
   * has been corrupted in transit. If the size of d_len is larger than the
   * size reported in the IP packet header, the packet has been padded and
   * we set d_len to the correct value.
   */

  iplen = (pbuf->len[0] << 8) + pbuf->len[1];
  if (iplen <= dev->d_len)
    {
      dev->d_len = iplen;
    }
  else
    {
      nlldbg("IP packet shorter than length in IP header\n");
      goto drop;
    }

  /* Check the fragment flag. */

  if ((pbuf->ipoffset[0] & 0x3f) != 0 || pbuf->ipoffset[1] != 0)
    {
#if defined(CONFIG_NET_TCP_REASSEMBLY)
      dev->d_len = devif_reassembly();
      if (dev->d_len == 0)
        {
          goto drop;
        }
#else /* CONFIG_NET_TCP_REASSEMBLY */
#ifdef CONFIG_NET_STATISTICS
      g_netstats.ipv4.drop++;
      g_netstats.ipv4.fragerr++;
#endif
      nlldbg("IP fragment dropped\n");
      goto drop;
#endif /* CONFIG_NET_TCP_REASSEMBLY */
    }

#if defined(CONFIG_NET_BROADCAST) && defined(CONFIG_NET_UDP)
   /* If IP broadcast support is configured, we check for a broadcast
    * UDP packet, which may be destined to us (even if there is no IP
    * address yet assigned to the device as is the case when we are
    * negotiating over DHCP for an address).
    */

  if (pbuf->proto == IP_PROTO_UDP &&
      net_ipv4addr_cmp(net_ip4addr_conv32(pbuf->destipaddr),
                       INADDR_BROADCAST))
    {
      return udp_ipv4_input(dev);
    }

  /* In most other cases, the device must be assigned a non-zero IP
   * address.  Another exception is when CONFIG_NET_PINGADDRCONF is
   * enabled...
   */

  else
#endif
#ifdef CONFIG_NET_ICMP
  if (net_ipv4addr_cmp(dev->d_ipaddr, INADDR_ANY))
    {
#ifdef CONFIG_NET_PINGADDRCONF
      /* If we are configured to use ping IP address configuration and
       * hasn't been assigned an IP address yet, we accept all ICMP
       * packets.
       */

      if (pbuf->proto == IP_PROTO_ICMP)
        {
          nlldbg("Possible ping config packet received\n");
          icmp_input(dev);
          goto drop;
        }
      else
#endif
        {
          nlldbg("No IP address assigned\n");
          goto drop;
        }
    }

  /* Check if the packet is destined for out IP address */
  else
#endif
    {
      /* Check if the packet is destined for our IP address. */

      if (!net_ipv4addr_cmp(net_ip4addr_conv32(pbuf->destipaddr), dev->d_ipaddr))
        {
#ifdef CONFIG_NET_IGMP
          in_addr_t destip = net_ip4addr_conv32(pbuf->destipaddr);
          if (igmp_grpfind(dev, &destip) == NULL)
#endif
            {
#ifdef CONFIG_NET_STATISTICS
              g_netstats.ipv4.drop++;
#endif
              goto drop;
            }
        }
    }

  if (ipv4_chksum(dev) != 0xffff)
    {
      /* Compute and check the IP header checksum. */

#ifdef CONFIG_NET_STATISTICS
      g_netstats.ipv4.drop++;
      g_netstats.ipv4.chkerr++;
#endif
      nlldbg("Bad IP checksum\n");
      goto drop;
    }

  /* Make sure that all packet processing logic knows that there is an IPv4
   * packet in the device buffer.
   */

  IFF_SET_IPv4(dev->d_flags);

  /* Now process the incoming packet according to the protocol. */

  switch (pbuf->proto)
    {
#ifdef CONFIG_NET_TCP
      case IP_PROTO_TCP:   /* TCP input */
        tcp_ipv4_input(dev);
        break;
#endif

#ifdef CONFIG_NET_UDP
      case IP_PROTO_UDP:   /* UDP input */
        udp_ipv4_input(dev);
        break;
#endif

#ifdef CONFIG_NET_ICMP
  /* Check for ICMP input */

      case IP_PROTO_ICMP:  /* ICMP input */
        icmp_input(dev);
        break;
#endif

#ifdef CONFIG_NET_IGMP
  /* Check for IGMP input */

      case IP_PROTO_IGMP:  /* IGMP input */
        igmp_input(dev);
        break;
#endif

      default:              /* Unrecognized/unsupported protocol */
#ifdef CONFIG_NET_STATISTICS
        g_netstats.ipv4.drop++;
        g_netstats.ipv4.protoerr++;
#endif

        nlldbg("Unrecognized IP protocol\n");
        goto drop;
    }

  /* Return and let the caller do any pending transmission. */

  return OK;

  /* Drop the packet.  NOTE that OK is returned meaning that the
   * packet has been processed (although processed unsuccessfully).
   */

drop:
  dev->d_len = 0;
  return OK;
}
#endif /* CONFIG_NET_IPv4 */
