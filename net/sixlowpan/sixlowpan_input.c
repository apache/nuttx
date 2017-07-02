/****************************************************************************
 * net/sixlowpan/sixlowpan_input.c
 * 6LoWPAN implementation (RFC4944 and draft-ietf-6LoWPAN-hc-06)
 *
 *   Copyright (C) 2017, Gregory Nutt, all rights reserved
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Derives in large part from Contiki:
 *
 *   Copyright (c) 2008, Swedish Institute of Computer Science.
 *   All rights reserved.
 *   Authors: Adam Dunkels <adam@sics.se>
 *            Nicolas Tsiftes <nvt@sics.se>
 *            Niclas Finne <nfi@sics.se>
 *            Mathilde Durvy <mdurvy@cisco.com>
 *            Julien Abeille <jabeille@cisco.com>
 *            Joakim Eriksson <joakime@sics.se>
 *            Joel Hoglund <joel@sics.se>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
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

#ifdef CONFIG_NET_6LOWPAN_FRAG
#  include "nuttx/clock.h"
#endif

#include "nuttx/net/netdev.h"
#include "nuttx/net/ip.h"
#include "nuttx/net/icmpv6.h"
#include "nuttx/net/sixlowpan.h"
#include "nuttx/wireless/ieee802154/ieee802154_mac.h"

#ifdef CONFIG_NET_PKT
#  include "pkt/pkt.h"
#endif

#include "sixlowpan/sixlowpan_internal.h"

#ifdef CONFIG_NET_6LOWPAN

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Success return values from sixlowpan_frame_process */

#define INPUT_PARTIAL  0 /* Frame processed successful, packet incomplete */
#define INPUT_COMPLETE 1 /* Frame processed successful, packet complete */

/* Re-assembly timeout in clock ticks */

#define NET_6LOWPAN_TIMEOUT SEC2TICK(CONFIG_NET_6LOWPAN_MAXAGE)

/* This is the size of a buffer large enough to hold the largest uncompressed
 * HC06 or HC1 headers.
 */

#ifdef CONFIG_NET_TCP
/* The basic TCP header length is TCP_HDRLEN but could include up to 16
 * additional 32-bit words of optional data.
 */

#  define UNCOMP_MAXHDR  (IPv6_HDRLEN + TCP_HDRLEN + 16*sizeof(uint32_t))

#elif defined(CONFIG_NET_UDP)
/* The UDP header length is always 8 bytes */

#  define UNCOMP_MAXHDR  (IPv6_HDRLEN + UDP_HDRLEN)

#elif defined(CONFIG_NET_ICMPv6)
/* The ICMPv6 header length is a mere 4 bytes */

#  define UNCOMP_MAXHDR  (IPv6_HDRLEN + ICMPv6_HDRLEN)

#else
/* No other header type is handled. */

#  define UNCOMP_MAXHDR  IPv6_HDRLEN
#endif

/* Buffer access helpers */

#define IPv6BUF(dev)  ((FAR struct ipv6_hdr_s *)((dev)->d_buf))
#define TCPBUF(dev)   ((FAR struct tcp_hdr_s *)&(dev)->d_buf[IPv6_HDRLEN])

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_NET_6LOWPAN_FRAG
/* This big buffer could be avoided with a little more effort */

static uint8_t g_bitbucket[UNCOMP_MAXHDR];
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sixlowpan_compare_fragsrc
 *
 * Description:
 *   Check if the fragment that we just received is from the same source as
 *   the previosly received fragements.
 *
 * Input Parameters:
 *   ieee - IEEE 802.15.4 MAC driver state reference
 *   ind  - Characteristics of the newly received frame
 *
 * Returned Value:
 *   true if the sources are the same.
 *
 ****************************************************************************/

static bool sixlowpan_compare_fragsrc(FAR struct ieee802154_driver_s *ieee,
                                      FAR const struct ieee802154_data_ind_s *ind)
{
  /* Check for an extended source address */

  if (ind->src.mode == IEEE802154_ADDRMODE_EXTENDED)
    {
      /* Was the first source address also extended? */

      if (ieee->i_fragsrc.extended)
        {
          /* Yes.. perform the extended address comparison */

          return sixlowpan_eaddrcmp(ieee->i_fragsrc.u.eaddr.u8, ind->src.eaddr);
        }
    }
  else
    {
      /* Short source address.  Was the first source address also short? */

      if (!ieee->i_fragsrc.extended)
        {
          /* Yes.. perform the extended short comparison */

          return sixlowpan_saddrcmp(ieee->i_fragsrc.u.saddr.u8, &ind->src.saddr);
        }
    }

  /* Address are different size and, hence, cannot match */

  return false;
}

/****************************************************************************
 * Name: sixlowpan_compress_ipv6hdr
 *
 * Description:
 *   IPv6 dispatch "compression" function.  Packets "Compression" when only
 *   IPv6 dispatch is used
 *
 *   There is no compression in this case, all fields are sent
 *   inline. We just add the IPv6 dispatch byte before the packet.
 *
 *   0               1                   2                   3
 *   0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
 *   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *   | IPv6 Dsp      | IPv6 header and payload ...
 *   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *
 * Input Parameters:
 *   fptr - Pointer to the beginning of the frame under construction
 *   bptr - Output goes here.  Normally this is a known offset into d_buf,
 *          may be redirected to g_bitbucket on the case of FRAGN frames.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void sixlowpan_uncompress_ipv6hdr(FAR uint8_t *fptr, FAR uint8_t *bptr)
{
  FAR struct ipv6_hdr_s *ipv6 = (FAR struct ipv6_hdr_s *)bptr;
  uint16_t protosize;

  /* Put uncompressed IPv6 header in d_buf. */

  g_frame_hdrlen  += SIXLOWPAN_IPV6_HDR_LEN;
  memcpy(ipv6, fptr + g_frame_hdrlen, IPv6_HDRLEN);

  /* Update g_uncomp_hdrlen and g_frame_hdrlen. */

  g_frame_hdrlen  += IPv6_HDRLEN;
  g_uncomp_hdrlen += IPv6_HDRLEN;

  /* Copy the following protocol header, */

   switch (ipv6->proto)
     {
#ifdef CONFIG_NET_TCP
     case IP_PROTO_TCP:
       {
         FAR struct tcp_hdr_s *tcp =
           (FAR struct tcp_hdr_s *)(fptr + g_frame_hdrlen);

         /* The TCP header length is encoded in the top 4 bits of the
          * tcpoffset field (in units of 32-bit words).
          */

         protosize = ((uint16_t)tcp->tcpoffset >> 4) << 2;
       }
       break;
#endif

#ifdef CONFIG_NET_UDP
     case IP_PROTO_UDP:
       protosize = sizeof(struct udp_hdr_s);
       break;
#endif

#ifdef CONFIG_NET_ICMPv6
     case IP_PROTO_ICMP6:
       protosize = sizeof(struct icmpv6_hdr_s);
       break;
#endif

     default:
       nwarn("WARNING: Unrecognized proto: %u\n", ipv6->proto);
       return;
     }

  /* Copy the protocol header. */

  memcpy((FAR uint8_t *)ipv6 + g_uncomp_hdrlen, fptr + g_frame_hdrlen,
         protosize);

  g_frame_hdrlen  += protosize;
  g_uncomp_hdrlen += protosize;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sixlowpan_frame_process
 *
 * Description:
 *   Process an incoming 6LoWPAN frame in 'iob'.
 *
 *   If its a FRAG1 or a non-fragmented frame we first uncompress the IP
 *   header. The 6LoWPAN payload and possibly the uncompressed IP header
 *   are then copied into d_buf.  An indication is returned if the packet
 *   in d_buf is complete (i.e., non-fragmented frame or and the last
 *   FRAGN frame).
 *
 *   NOTE: We do not check for overlapping sixlowpan fragments (that is a
 *   SHALL in the RFC 4944 and should never happen)
 *
 * Input Parameters:
 *   ieee - The IEEE802.15.4 MAC network driver interface.
 *   ind  - Meta data characterizing the received frame.
 *   iob  - The IOB containing the frame.
 *
 * Returned Value:
 *   On success, a value greater than equal to zero is returned, either:
 *
 *     INPUT_PARTIAL  Frame processed successful, packet incomplete
 *     INPUT_COMPLETE Frame processed successful, packet complete
 *
 *   Othewise a negated errno value is returned to indicate the nature of the
 *   failure.
 *
 * Assumptions:
 *   Network is locked
 *
 ****************************************************************************/

static int sixlowpan_frame_process(FAR struct ieee802154_driver_s *ieee,
                                   FAR const struct ieee802154_data_ind_s *ind,
                                   FAR struct iob_s *iob)
{
  FAR uint8_t *fptr;          /* Convenience pointer to beginning of the frame */
  FAR uint8_t *bptr;          /* Used to redirect uncompressed header to the bitbucket */
  FAR uint8_t *hc1;           /* Convenience pointer to HC1 data */
  uint16_t fragsize  = 0;     /* Size of the IP packet (read from fragment) */
  uint16_t paysize;           /* Size of the data payload */
  uint8_t fragoffset = 0;     /* Offset of the fragment in the IP packet */
  int reqsize;                /* Required buffer size */
  int hdrsize;                /* Size of the IEEE802.15.4 header */

#ifdef CONFIG_NET_6LOWPAN_FRAG
  FAR uint8_t *fragptr;       /* Pointer to the fragmentation header */
  bool isfrag        = false;
  bool isfirstfrag   = false;
  uint16_t fragtag   = 0;     /* Tag of the fragment */
  systime_t elapsed;          /* Elapsed time */
#endif /* CONFIG_NET_6LOWPAN_FRAG */

  /* Get a pointer to the payload following the IEEE802.15.4 frame header(s).
   * This size includes both fragmentation and FCF headers.
   */

  fptr    = iob->io_data;    /* Frame data is in I/O buffer */
  hdrsize = iob->io_offset;  /* Offset past the MAC header */

  DEBUGASSERT((unsigned)hdrsize < iob->io_len);

  /* Initialize global data.  Locking the network guarantees that we have
   * exclusive use of the global values for intermediate calculations.
   */

  g_uncomp_hdrlen = 0;
  g_frame_hdrlen  = hdrsize;

#ifdef CONFIG_NET_6LOWPAN_FRAG
  /* Since we don't support the mesh and broadcast header, the first header
   * we look for is the fragmentation header.  NOTE that g_frame_hdrlen
   * already includes the fragementation header, if presetn.
   */

  fragptr = fptr + hdrsize;
  switch ((GETUINT16(fragptr, SIXLOWPAN_FRAG_DISPATCH_SIZE) & 0xf800) >> 8)
    {
    /* First fragment of new reassembly */

    case SIXLOWPAN_DISPATCH_FRAG1:
      {
        /* Set up for the reassembly */

        fragsize        = GETUINT16(fragptr, SIXLOWPAN_FRAG_DISPATCH_SIZE) & 0x07ff;
        fragtag         = GETUINT16(fragptr, SIXLOWPAN_FRAG_TAG);
        g_frame_hdrlen += SIXLOWPAN_FRAG1_HDR_LEN;

        ninfo("FRAG1: fragsize=%d fragtag=%d fragoffset=%d\n",
              fragsize, fragtag, fragoffset);

        /* Indicate the first fragment of the reassembly */

        isfirstfrag     = true;
        isfrag          = true;
      }
      break;

    case SIXLOWPAN_DISPATCH_FRAGN:
      {
        /* Set offset, tag, size.  Offset is in units of 8 bytes. */

        fragoffset      = fragptr[SIXLOWPAN_FRAG_OFFSET];
        fragtag         = GETUINT16(fragptr, SIXLOWPAN_FRAG_TAG);
        fragsize        = GETUINT16(fragptr, SIXLOWPAN_FRAG_DISPATCH_SIZE) & 0x07ff;
        g_frame_hdrlen += SIXLOWPAN_FRAGN_HDR_LEN;

        ninfo("FRAGN: fragsize=%d fragtag=%d fragoffset=%d\n",
              fragsize, fragtag, fragoffset);
        ninfo("FRAGN: i_accumlen=%d paysize=%u fragsize=%u\n",
              ieee->i_accumlen, iob->io_len - g_frame_hdrlen, fragsize);

        /* Indicate that this frame is a another fragment for reassembly */

        isfrag          = true;
      }
      break;

    /* Not a fragment */

    default:
      break;
    }

  /* Check if we are currently reassembling a packet */

  bptr = ieee->i_dev.d_buf;
  if (ieee->i_accumlen > 0)
    {
      /* If reassembly timed out, cancel it */

      elapsed = clock_systimer() - ieee->i_time;
      if (elapsed > NET_6LOWPAN_TIMEOUT)
        {
          nwarn("WARNING: Reassembly timed out\n");
          ieee->i_pktlen   = 0;
          ieee->i_accumlen = 0;
        }

      /* In this case what we expect is that the next frame will hold the
       * next FRAGN of the sequence.  We have to handle a few exeptional
       * cases that we need to handle:
       *
       * 1. If we are currently reassembling a packet, but have just received
       *    the first fragment of another packet. We can either ignore it and
       *    hope to receive the rest of the under-reassembly packet fragments,
       *    or we can discard the previous packet altogether, and start
       *    reassembling the new packet.  Here we discard the previous packet,
       *    and start reassembling the new packet.
       * 2. The new frame is not a fragment.  We should be able to handle this
       *    case, but we cannot because that would require two packet buffers.
       *    It could be handled with a more extensive design.
       * 3. The fragment came from a different sender.  What would this mean?
       *
       */

      else if (!isfrag)
        {
          /* Discard the partially assembled packet */

          nwarn("WARNING: Non-fragment frame received during reassembly\n");
          ieee->i_pktlen   = 0;
          ieee->i_accumlen = 0;
        }

      /* It is a fragment of some kind.  Drop any zero length fragments */

      else if (fragsize == 0)
        {
          nwarn("WARNING: Dropping zero-length 6LoWPAN fragment\n");
          return INPUT_PARTIAL;
        }

      /* A non-zero, first fragement received while we are in the middle of
       * rassembly.  Discard the partially assembled packet and start over.
       */

      else if (isfirstfrag)
        {
          nwarn("WARNING: First fragment frame received during reassembly\n");
          ieee->i_pktlen   = 0;
          ieee->i_accumlen = 0;
        }

      /* Verify that this fragment is part of that reassembly sequence */

      else if (fragsize != ieee->i_pktlen || ieee->i_reasstag != fragtag  ||
               !sixlowpan_compare_fragsrc(ieee, ind))
        {
          /* The packet is a fragment that does not belong to the packet
           * being reassembled or the packet is not a fragment.
           */

          nwarn("WARNING: Dropping 6LoWPAN packet that is not a fragment of "
                "the packet currently being reassembled\n");
          return -EPERM;
        }
      else
        {
          /* Looks good.  We are currently processing a reassembling sequence
           * and we recieved a valid FRAGN fragment.  Redirect the header
           * uncompression to our bitbucket.
           */

          bptr = g_bitbucket;
        }
    }

  /* There is no reassembly in progress. Check if we received a fragment */

  else if (isfrag)
    {
      /* Another case that we have to handle is if a FRAGN fragment of a
       * reassembly is received, but we are not currently reassembling a
       * packet. I think we have no choice but to drop the packet in this
       * case.
       */

      if (!isfirstfrag)
        {
          nwarn("WARNING: FRAGN 6LoWPAN fragment while not reassembling\n");
          return -EPERM;
        }

      /* Drop the packet if it cannot fit into the d_buf */

      if (fragsize > CONFIG_NET_6LOWPAN_MTU)
        {
          nwarn("WARNING: Reassembled packet size exeeds CONFIG_NET_6LOWPAN_MTU\n");
          return -ENOSPC;
        }

      ieee->i_pktlen   = fragsize;
      ieee->i_reasstag = fragtag;
      ieee->i_time     = clock_systimer();

      ninfo("Starting reassembly: i_pktlen %u, i_reasstag %d\n",
            ieee->i_pktlen, ieee->i_reasstag);

      /* Extract the source address from the 'ind' meta data.  NOTE that the
       * size of the source address may be different that our local, destination
       * address.
       */

      if (ind->src.mode == IEEE802154_ADDRMODE_EXTENDED)
        {
          ieee->i_fragsrc.extended = true;
          sixlowpan_eaddrcopy(ieee->i_fragsrc.u.eaddr.u8, ind->src.eaddr);
        }
      else
        {
          memset(&ieee->i_fragsrc, 0, sizeof(struct sixlowpan_tagaddr_s));
          sixlowpan_saddrcopy(ieee->i_fragsrc.u.saddr.u8, &ind->src.saddr);
        }
    }
#endif /* CONFIG_NET_6LOWPAN_FRAG */

  /* Process next dispatch and headers */

  hc1 = fptr + g_frame_hdrlen;

#ifdef CONFIG_NET_6LOWPAN_COMPRESSION_HC06
  if ((hc1[SIXLOWPAN_HC1_DISPATCH] & SIXLOWPAN_DISPATCH_IPHC_MASK) == SIXLOWPAN_DISPATCH_IPHC)
    {
      ninfo("IPHC Dispatch\n");
      sixlowpan_uncompresshdr_hc06(ind, fragsize, iob, fptr, bptr);
    }
  else
#endif /* CONFIG_NET_6LOWPAN_COMPRESSION_HC06 */

#ifdef CONFIG_NET_6LOWPAN_COMPRESSION_HC1
  if (hc1[SIXLOWPAN_HC1_DISPATCH] == SIXLOWPAN_DISPATCH_HC1)
    {
      ninfo("HC1 Dispatch\n");
      sixlowpan_uncompresshdr_hc1(ind, fragsize, iob, fptr, bptr);
    }
  else
#endif /* CONFIG_NET_6LOWPAN_COMPRESSION_HC1 */

  if (hc1[SIXLOWPAN_HC1_DISPATCH] == SIXLOWPAN_DISPATCH_IPV6)
    {
      ninfo("IPv6 Dispatch\n");
      sixlowpan_uncompress_ipv6hdr(fptr, bptr);
    }
  else
    {
      /* Unknown or unsupported header */

      nwarn("WARNING: Unknown dispatch: %u\n",  hc1[SIXLOWPAN_HC1_DISPATCH]);
      return -ENOSYS;
    }

#ifdef CONFIG_NET_6LOWPAN_FRAG
  /* Is this the first fragment is a sequence? */

  if (isfirstfrag)
    {
      /* Yes.. Remember the offset from the beginning of d_buf where we
       * begin placing the data payload.
       */

      ieee->i_boffset = g_uncomp_hdrlen;
    }

  /* No.. is this a subsequent fragment in the same sequence? */

  else if (isfrag)
    {
      /* Yes, recover the offset from the beginning of the d_buf where
       * we began placing payload data.
       */

      g_uncomp_hdrlen = ieee->i_boffset;
    }
#endif /* CONFIG_NET_6LOWPAN_FRAG */

  /* Copy "payload" from the frame buffer to the IEEE802.15.4 MAC driver's
   * packet buffer, d_buf.  If this frame is a first fragment or not part of
   * a fragmented packet, we have already copied the compressed headers,
   * g_uncomp_hdrlen and g_frame_hdrlen are non-zerio, fragoffset is.
   */

  paysize = iob->io_len - g_frame_hdrlen;
  if (paysize > CONFIG_NET_6LOWPAN_MTU)
    {
      nwarn("WARNING: Packet dropped due to payload (%u) > packet buffer (%u)\n",
            paysize, CONFIG_NET_6LOWPAN_MTU);
      return -ENOSPC;
    }

  /* Sanity-check size of incoming packet to avoid buffer overflow */

  reqsize = g_uncomp_hdrlen + (fragoffset << 3) + paysize;
  if (reqsize > CONFIG_NET_6LOWPAN_MTU)
    {
      nwarn("WARNING: Required buffer size: %u+%u+%u=%u Available=%u\n",
            g_uncomp_hdrlen, (fragoffset << 3), paysize,
            reqsize, CONFIG_NET_6LOWPAN_MTU);
      return -ENOMEM;
    }

  memcpy(ieee->i_dev.d_buf + g_uncomp_hdrlen + (fragoffset << 3),
         fptr + g_frame_hdrlen, paysize);

#ifdef CONFIG_NET_6LOWPAN_FRAG
  /* Update ieee->i_accumlen if the frame is a fragment, ieee->i_pktlen
   * otherwise.
   */

  if (isfrag)
    {
      /* Check if it is the last fragment to be processed.
       *
       * If this is the last fragment, we may shave off any extrenous
       * bytes at the end. We must be liberal in what we accept.
       */

      ieee->i_accumlen = g_uncomp_hdrlen + (fragoffset << 3) + paysize;
    }
  else
    {
      ieee->i_pktlen = paysize + g_uncomp_hdrlen;
    }

  /* If we have a full IP packet in sixlowpan_buf, deliver it to
   * the IP stack
   */

  ninfo("i_accumlen=%d i_pktlen=%d paysize=%d\n",
         ieee->i_accumlen, ieee->i_pktlen, paysize);

  if (ieee->i_accumlen == 0 || ieee->i_accumlen >= ieee->i_pktlen)
    {
      ninfo("IP packet ready (length %d)\n", ieee->i_pktlen);

      ieee->i_dev.d_len = ieee->i_pktlen;
      ieee->i_pktlen    = 0;
      ieee->i_accumlen  = 0;
      return INPUT_COMPLETE;
    }

  return INPUT_PARTIAL;
#else
  /* Deliver the packet to the IP stack */

  ieee->i_dev.d_len = paysize + g_uncomp_hdrlen;
  return INPUT_COMPLETE;
#endif /* CONFIG_NET_6LOWPAN_FRAG */
}

/****************************************************************************
 * Name: sixlowpan_dispatch
 *
 * Description:
 *   Inject the packet in d_buf into the network for normal packet processing.
 *
 * Parameters:
 *   ieee - The IEEE802.15.4 MAC network driver interface.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static int sixlowpan_dispatch(FAR struct ieee802154_driver_s *ieee)
{
  sixlowpan_dumpbuffer("Incoming packet",
                       (FAR const uint8_t *)IPv6BUF(&ieee->i_dev),
                       ieee->i_dev.d_len);

#ifdef CONFIG_NET_PKT
  /* When packet sockets are enabled, feed the frame into the packet tap */

  ninfo("Packet tap\n");
  pkt_input(&ieee->i_dev);
#endif

  /* We only accept IPv6 packets. */

  ninfo("IPv6 packet dispatch\n");
  NETDEV_RXIPV6(&ieee->i_dev);

  /* Give the IPv6 packet to the network layer.  NOTE:  If there is a
   * problem with IPv6 header, it will be silently dropped and d_len will
   * be set to zero.  Oddly, ipv6_input() will return OK in this case.
   */

  return ipv6_input(&ieee->i_dev);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sixlowpan_input
 *
 * Description:
 *   Process an incoming 6LoWPAN frame.
 *
 *   This function is called when the device driver has received an
 *   IEEE802.15.4 frame from the network.  The frame from the device
 *   driver must be provided in by the IOB frame argument of the
 *   function call:
 *
 *   - The frame data is in the IOB io_data[] buffer,
 *   - The length of the frame is in the IOB io_len field, and
 *   - The offset past the IEEE802.15.4 MAC header is provided in the
 *     io_offset field.
 *
 *   The frame argument may refer to a single frame (a list of length one)
 *   or may it be the head of a list of multiple frames.
 *
 *   - The io_flink field points to the next frame in the list (if enable)
 *   - The last frame in the list will have io_flink == NULL.
 *
 *   An non-NULL d_buf of size CONFIG_NET_6LOWPAN_MTU + CONFIG_NET_GUARDSIZE
 *   must also be provided.  The frame will be decompressed and placed in
 *   the d_buf. Fragmented packets will also be reassembled in the d_buf as
 *   they are received (meaning for the driver, that two packet buffers are
 *   required:  One for reassembly of RX packets and one used for TX polling).
 *
 *   After each frame is processed into d_buf, the IOB is deallocated.  If
 *   reassembly is incomplete, the partially reassembled packet must be
 *   preserved by the IEEE802.15.4 MAC network drvier sand provided again
 *   when the next frame is received.
 *
 *   When the packet in the d_buf is fully reassembled, it will be provided
 *   to the network as with any other received packet.  d_len will be set
 *   the length of the uncompressed, reassembled packet.
 *
 *   After the network processes the packet, d_len will be set to zero.
 *   Network logic may also decide to send a response to the packet.  In
 *   that case, the outgoing network packet will be placed in d_buf and
 *   d_len will be set to a non-zero value.  That case is handled by this
 *   function.
 *
 *   If that case occurs, the packet will be converted to a list of
 *   compressed and possibly fragmented frames and provided to the MAC
 *   network driver via the req_data() method as with other TX operations.
 *
 * Input Parameters:
 *   ieee      - The IEEE802.15.4 MAC network driver interface.
 *   framelist - The head of an incoming list of frames.  Normally this
 *               would be a single frame.  A list may be provided if
 *               appropriate, however.
 *   ind       - Meta data characterizing the received frame.  If there are
 *               multilple frames in the list, this meta data must apply to
 *               all of the frames!
 *
 * Returned Value:
 *   Ok is returned on success; Othewise a negated errno value is returned.
 *
 ****************************************************************************/

int sixlowpan_input(FAR struct ieee802154_driver_s *ieee,
                    FAR struct iob_s *framelist,
                    FAR const struct ieee802154_data_ind_s *ind)
{
  int ret = -EINVAL;

  DEBUGASSERT(ieee != NULL && framelist != NULL);

  /* Verify that an frame has been provided. */

  while (framelist != NULL)
    {
      FAR struct iob_s *iob;

      /* Remove the IOB containing the frame from the device structure */

      iob       = framelist;
      framelist = iob->io_flink;

      sixlowpan_dumpbuffer("Incoming frame", iob->io_data, iob->io_len);

      /* Process the frame, decompressing it into the packet buffer */

      ret = sixlowpan_frame_process(ieee, ind, iob);

      /* Free the IOB the held the consumed frame */

      iob_free(iob);

      /* Was the frame successfully processed? Is the packet in d_buf fully
       * reassembled?
       */

      if (ret == INPUT_COMPLETE)
        {
          /* Inject the uncompressed, reassembled packet into the network */

          ret = sixlowpan_dispatch(ieee);
          if (ret >= 0)
            {
              /* Check if this resulted in a request to send an outgoing
               * packet.
               */

              if (ieee->i_dev.d_len > 0)
                {
                  FAR struct ipv6_hdr_s *ipv6hdr;
                  FAR uint8_t *buffer;
                  struct sixlowpan_tagaddr_s destmac;
                  size_t hdrlen;
                  size_t buflen;

                  /* The IPv6 header followed by TCP or UDP headers should
                   * lie at the beginning of d_buf since there is no link
                   * layer protocol header.
                   */

                  ipv6hdr = IPv6BUF(&ieee->i_dev);

                  /* Get the IEEE 802.15.4 MAC address of the destination.
                   * This assumes an encoding of the MAC address in the IPv6
                   * address.
                   */

                  ret = sixlowpan_destaddrfromip(ieee, ipv6hdr->destipaddr,
                                                 &destmac);
                  if (ret < 0)
                    {
                      nerr("ERROR: Failed to dest MAC address: %d\n", ret);
                      goto drop;
                    }

                  /* The data payload should follow the IPv6 header plus
                   * the protocol header.
                   */

                  switch (ipv6hdr->proto)
                    {
#ifdef CONFIG_NET_TCP
                      case IP_PROTO_TCP:
                        {
                          FAR struct tcp_hdr_s *tcp = TCPBUF(&ieee->i_dev);
                          uint16_t tcplen;

                          /* The TCP header length is encoded in the top 4 bits
                           * of the tcpoffset field (in units of 32-bit words).
                           */

                          tcplen = ((uint16_t)tcp->tcpoffset >> 4) << 2;
                          hdrlen = IPv6_HDRLEN + tcplen;
                        }
                        break;
#endif
#ifdef CONFIG_NET_UDP
                      case IP_PROTO_UDP:
                        {
                          hdrlen = IPv6_HDRLEN + UDP_HDRLEN;
                        }
                        break;
#endif
#ifdef CONFIG_NET_ICMPv6
                      case IP_PROTO_ICMP6:
                        {
                          hdrlen = IPv6_HDRLEN + ICMPv6_HDRLEN;
                        }
                        break;
#endif
                      default:
                        {
                          nwarn("WARNING: Unsupported protoype: %u\n",
                                ipv6hdr->proto);
                          ret = -EPROTO;
                          goto drop;
                        }
                    }

                  if (hdrlen < ieee->i_dev.d_len)
                    {
                      nwarn("WARNING: Packet to small: Have %u need >%u\n",
                            ieee->i_dev.d_len, hdrlen);
                      ret = -ENOBUFS;
                      goto drop;
                    }

                  /* Convert the outgoing packet into a frame list. */

                  buffer = ieee->i_dev.d_buf + hdrlen;
                  buflen = ieee->i_dev.d_len - hdrlen;

                  ret = sixlowpan_queue_frames(ieee, ipv6hdr, buffer, buflen,
                                               &destmac);
drop:
                  ieee->i_dev.d_len = 0;
                }
            }
        }
    }

  return ret;
}

#endif /* CONFIG_NET_6LOWPAN */
