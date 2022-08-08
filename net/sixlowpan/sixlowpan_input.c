/****************************************************************************
 * net/sixlowpan/sixlowpan_input.c
 * 6LoWPAN implementation (RFC 4944 and RFC 6282)
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

#include "nuttx/mm/iob.h"
#include "nuttx/net/netdev.h"
#include "nuttx/net/radiodev.h"
#include "nuttx/net/ip.h"
#include "nuttx/net/icmpv6.h"
#include "nuttx/net/sixlowpan.h"
#include "nuttx/wireless/ieee802154/ieee802154_mac.h"

#ifdef CONFIG_NET_PKT
#  include <nuttx/net/pkt.h>
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

/* This big buffer could be avoided with a little more effort */

static uint8_t g_bitbucket[UNCOMP_MAXHDR];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

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
 *   fptr  - Pointer to the beginning of the frame under construction
 *   bptr  - Output goes here.  Normally this is a known offset into d_buf,
 *           may be redirected to g_bitbucket on the case of FRAGN frames.
 *   proto - True: Copy the protocol header following the IPv6 header too.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void sixlowpan_uncompress_ipv6hdr(FAR uint8_t *fptr,
                                         FAR uint8_t *bptr)
{
  FAR struct ipv6_hdr_s *ipv6 = (FAR struct ipv6_hdr_s *)bptr;

  /* Put uncompressed IPv6 header in d_buf. */

  g_frame_hdrlen  += SIXLOWPAN_IPV6_HDR_LEN;
  memcpy(ipv6, fptr + g_frame_hdrlen, IPv6_HDRLEN);

  /* Update g_uncomp_hdrlen and g_frame_hdrlen. */

  g_frame_hdrlen  += IPv6_HDRLEN;
  g_uncomp_hdrlen += IPv6_HDRLEN;
}

/****************************************************************************
 * Name: sixlowpan_uncompress_ipv6proto
 *
 * Description:
 *   Copy the protocol header following the IPv4 header
 *
 * Input Parameters:
 *   fptr  - Pointer to the beginning of the frame under construction
 *   bptr  - Output goes here.  Normally this is a known offset into d_buf,
 *           may be redirected to g_bitbucket on the case of FRAGN frames.
 *   proto - True: Copy the protocol header following the IPv6 header too.
 *
 * Returned Value:
 *   The size of the protocol header that was copied.
 *
 ****************************************************************************/

static uint16_t sixlowpan_uncompress_ipv6proto(FAR uint8_t *fptr,
                                               FAR uint8_t *bptr)
{
  FAR struct ipv6_hdr_s *ipv6 = (FAR struct ipv6_hdr_s *)bptr;
  uint16_t protosize = 0;

  /* Copy the following protocol header. */

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
      return 0;
    }

  /* Copy the protocol header. */

  memcpy((FAR uint8_t *)ipv6 + g_uncomp_hdrlen, fptr + g_frame_hdrlen,
         protosize);

  g_frame_hdrlen   += protosize;
  g_uncomp_hdrlen  += protosize;
  return protosize;
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
 *   radio    - The radio network device driver interface.
 *   metadata - Metadata characterizing the received frame.
 *   iob      - The IOB containing the frame.
 *
 * Returned Value:
 *   On success, a value greater than equal to zero is returned, either:
 *
 *     INPUT_PARTIAL  Frame processed successful, packet incomplete
 *     INPUT_COMPLETE Frame processed successful, packet complete
 *
 *   Otherwise a negated errno value is returned to indicate the nature of
 *   the failure.
 *
 * Assumptions:
 *   Network is locked
 *
 ****************************************************************************/

static int sixlowpan_frame_process(FAR struct radio_driver_s *radio,
                                   FAR const void *metadata,
                                   FAR struct iob_s *iob)
{
  FAR struct sixlowpan_reassbuf_s *reass;
  struct netdev_varaddr_s fragsrc;
  FAR uint8_t *fptr;          /* Convenience pointer to beginning of the frame */
  FAR uint8_t *bptr;          /* Used to redirect uncompressed header to the bitbucket */
  FAR uint8_t *hc1;           /* Convenience pointer to HC1 data */
  FAR uint8_t *fragptr;       /* Pointer to the fragmentation header */
  uint16_t fragsize  = 0;     /* Size of the IP packet (read from fragment) */
  uint16_t paysize;           /* Size of the data payload */
  uint16_t fragtag   = 0;     /* Tag of the fragment */
  uint8_t fragoffset = 0;     /* Offset of the fragment in the IP packet */
  uint8_t protosize  = 0;     /* Length of the protocol header (treated like payload) */
  bool isfrag        = false; /* true: Frame is a fragment */
  bool isfrag1       = false; /* true: Frame is the first fragment of the series */
  int reqsize;                /* Required buffer size */
  int hdrsize;                /* Size of the IEEE802.15.4 header */
  int ret;

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

  /* Since we don't support the mesh and broadcast header, the first header
   * we look for is the fragmentation header.  NOTE that g_frame_hdrlen
   * already includes the fragmentation header, if presetn.
   */

  fragptr = fptr + hdrsize;
  switch ((GETUINT16(fragptr, SIXLOWPAN_FRAG_DISPATCH_SIZE) & 0xf800) >> 8)
    {
    /* First fragment of new reassembly */

    case SIXLOWPAN_DISPATCH_FRAG1:
      {
        /* Set up for the reassembly */

        fragsize = GETUINT16(fragptr, SIXLOWPAN_FRAG_DISPATCH_SIZE) & 0x07ff;
        fragtag  = GETUINT16(fragptr, SIXLOWPAN_FRAG_TAG);
        g_frame_hdrlen += SIXLOWPAN_FRAG1_HDR_LEN;

        ninfo("FRAG1: fragsize=%d fragtag=%d fragoffset=%d\n",
              fragsize, fragtag, fragoffset);

        /* Drop any zero length fragments */

        if (fragsize == 0)
          {
            nwarn("WARNING: Dropping zero-length 6LoWPAN fragment\n");
            return INPUT_PARTIAL;
          }

        /* Drop the packet if it cannot fit into the d_buf */

        if (fragsize > CONFIG_NET_6LOWPAN_PKTSIZE)
          {
            nwarn("WARNING:  Reassembled packet size exceeds "
                  "CONFIG_NET_6LOWPAN_PKTSIZE\n");
            return -ENOSPC;
          }

        /* Extract the source address from the 'metadata'. */

        ret = sixlowpan_extract_srcaddr(radio, metadata, &fragsrc);
        if (ret < 0)
          {
            nerr("ERROR: sixlowpan_extract_srcaddr failed: %d\n", ret);
            return ret;
          }

        /* Allocate a new reassembly buffer */

        reass = sixlowpan_reass_allocate(fragtag, &fragsrc);
        if (reass == NULL)
          {
            nerr("ERROR: Failed to allocate a reassembly buffer\n");
            return -ENOMEM;
          }

        radio->r_dev.d_buf = reass->rb_buf;
        radio->r_dev.d_len = 0;
        reass->rb_pktlen   = fragsize;

        /* Indicate the first fragment of the reassembly */

        bptr               = reass->rb_buf;
        isfrag1            = true;
        isfrag             = true;
      }
      break;

    case SIXLOWPAN_DISPATCH_FRAGN:
      {
        /* Get offset, tag, size.  Offset is in units of 8 bytes. */

        fragoffset = fragptr[SIXLOWPAN_FRAG_OFFSET];
        fragtag  = GETUINT16(fragptr, SIXLOWPAN_FRAG_TAG);
        fragsize = GETUINT16(fragptr, SIXLOWPAN_FRAG_DISPATCH_SIZE) & 0x07ff;
        g_frame_hdrlen += SIXLOWPAN_FRAGN_HDR_LEN;

        /* Extract the source address from the 'metadata'. */

        ret = sixlowpan_extract_srcaddr(radio, metadata, &fragsrc);
        if (ret < 0)
          {
            nerr("ERROR: sixlowpan_extract_srcaddr failed: %d\n", ret);
            return ret;
          }

        /* Find the existing reassembly buffer
         * with the same tag and source address
         */

        reass = sixlowpan_reass_find(fragtag, &fragsrc);
        if (reass == NULL)
          {
            nerr("ERROR: Failed to find a reassembly buffer for tag=%04x\n",
                 fragtag);
            return -ENOENT;
          }

       if (fragsize != reass->rb_pktlen)
        {
          /* The packet is a fragment but its size does not match. */

          nwarn("WARNING: Dropping 6LoWPAN packet. Bad fragsize: %u vs %u\n",
                fragsize, reass->rb_pktlen);
          ret = -EPERM;
          goto errout_with_reass;
        }

        radio->r_dev.d_buf  = reass->rb_buf;
        radio->r_dev.d_len  = 0;

        ninfo("FRAGN: fragsize=%d fragtag=%d fragoffset=%d\n",
              fragsize, fragtag, fragoffset);
        ninfo("FRAGN: rb_accumlen=%d paysize=%u fragsize=%u\n",
              reass->rb_accumlen, iob->io_len - g_frame_hdrlen, fragsize);

        /* Indicate that this frame is a another fragment for reassembly */

        bptr   = g_bitbucket;
        isfrag = true;
      }
      break;

    /* Not a fragment */

    default:
      /* We still need a packet buffer.  But in this case, the driver should
       * have provided one.
       */

      DEBUGASSERT(radio->r_dev.d_buf != NULL);
      reass = (FAR struct sixlowpan_reassbuf_s *)radio->r_dev.d_buf;
      reass->rb_pool = REASS_POOL_RADIO;
      bptr  = reass->rb_buf;
      break;
    }

  /* Process next dispatch and headers */

  hc1 = fptr + g_frame_hdrlen;

#ifdef CONFIG_NET_6LOWPAN_COMPRESSION_HC06
  if ((hc1[SIXLOWPAN_HC1_DISPATCH] & SIXLOWPAN_DISPATCH_IPHC_MASK) ==
      SIXLOWPAN_DISPATCH_IPHC)
    {
      ninfo("IPHC Dispatch\n");
      sixlowpan_uncompresshdr_hc06(radio, metadata,
                                   fragsize, iob, fptr, bptr);
    }
  else
#endif /* CONFIG_NET_6LOWPAN_COMPRESSION_HC06 */

#ifdef CONFIG_NET_6LOWPAN_COMPRESSION_HC1
  if (hc1[SIXLOWPAN_HC1_DISPATCH] == SIXLOWPAN_DISPATCH_HC1)
    {
      ninfo("HC1 Dispatch\n");
      sixlowpan_uncompresshdr_hc1(radio, metadata,
                                  fragsize, iob, fptr, bptr);
    }
  else
#endif /* CONFIG_NET_6LOWPAN_COMPRESSION_HC1 */

  if (hc1[SIXLOWPAN_HC1_DISPATCH] == SIXLOWPAN_DISPATCH_IPV6)
    {
      ninfo("IPv6 Dispatch\n");

      /* Uncompress the IPv6 header */

      sixlowpan_uncompress_ipv6hdr(fptr, bptr);

      /* A protocol header will follow the IPv6 header only on a non-
       * fragmented packet or on the first fragment of a fragmented
       * packet.
       */

      if (!isfrag || isfrag1)
        {
          protosize = sixlowpan_uncompress_ipv6proto(fptr, bptr);
        }
    }
  else
    {
      /* Unknown or unsupported header */

      nwarn("WARNING: Unknown dispatch: %u\n",  hc1[SIXLOWPAN_HC1_DISPATCH]);
      ret = -ENOSYS;
      goto errout_with_reass;
    }

  /* Is this the first fragment is a sequence? */

  if (isfrag1)
    {
      /* Yes.. Remember the offset from the beginning of d_buf where we
       * begin placing the data payload.
       */

      reass->rb_boffset = g_uncomp_hdrlen - protosize;
    }

  /* No.. is this a subsequent fragment in the same sequence? */

  else if (isfrag)
    {
      /* Yes, recover the offset from the beginning of the d_buf where
       * we began placing payload data.
       */

      g_uncomp_hdrlen = reass->rb_boffset;
    }

  /* Copy "payload" from the frame buffer to the IEEE802.15.4 MAC driver's
   * packet buffer, d_buf.  If this frame is a first fragment or not part of
   * a fragmented packet, we have already copied the compressed headers,
   * g_uncomp_hdrlen and g_frame_hdrlen are non-zerio, fragoffset is.
   */

  paysize = iob->io_len - g_frame_hdrlen;
  if (paysize > CONFIG_NET_6LOWPAN_PKTSIZE)
    {
      nwarn("Packet dropped due to payload (%u) > packet buffer (%u)\n",
            paysize, CONFIG_NET_6LOWPAN_PKTSIZE);
      ret = -ENOSPC;
      goto errout_with_reass;
    }

  /* Sanity-check size of incoming packet to avoid buffer overflow */

  reqsize = g_uncomp_hdrlen + (fragoffset << 3) + paysize;
  if (reqsize > CONFIG_NET_6LOWPAN_PKTSIZE)
    {
      nwarn("WARNING: Required buffer size: %u+%u+%u=%u Available=%u\n",
            g_uncomp_hdrlen, (fragoffset << 3), paysize,
            reqsize, CONFIG_NET_6LOWPAN_PKTSIZE);
      ret = -ENOMEM;
      goto errout_with_reass;
    }

  memcpy(radio->r_dev.d_buf + g_uncomp_hdrlen + (fragoffset << 3),
         fptr + g_frame_hdrlen, paysize);

  /* Update reass->rb_accumlen if the frame is a fragment, reass->rb_pktlen
   * otherwise.
   */

  if (isfrag)
    {
      /* Check if it is the last fragment to be processed.
       *
       * If this is the last fragment, we may shave off any extrenous
       * bytes at the end. We must be liberal in what we accept.
       */

      reass->rb_accumlen = g_uncomp_hdrlen + (fragoffset << 3) + paysize;
    }
  else
    {
      reass->rb_pktlen = paysize + g_uncomp_hdrlen;
    }

  /* If we have a full IP packet in sixlowpan_buf, deliver it to
   * the IP stack
   */

  ninfo("rb_accumlen=%d rb_pktlen=%d paysize=%d\n",
         reass->rb_accumlen, reass->rb_pktlen, paysize);

  if (reass->rb_accumlen == 0 || reass->rb_accumlen >= reass->rb_pktlen)
    {
      ninfo("IP packet ready (length %d)\n", reass->rb_pktlen);

      radio->r_dev.d_buf  = reass->rb_buf;
      radio->r_dev.d_len  = reass->rb_pktlen;
      reass->rb_active    = false;
      reass->rb_pktlen    = 0;
      reass->rb_accumlen  = 0;
      return INPUT_COMPLETE;
    }

  radio->r_dev.d_buf  = NULL;
  radio->r_dev.d_len  = 0;
  return INPUT_PARTIAL;

errout_with_reass:
  sixlowpan_reass_free(reass);
  return ret;
}

/****************************************************************************
 * Name: sixlowpan_dispatch
 *
 * Description:
 *   Inject the packet in d_buf into the network for normal packet
 *   processing.
 *
 * Input Parameters:
 *   radio - The IEEE802.15.4 MAC network driver interface.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static int sixlowpan_dispatch(FAR struct radio_driver_s *radio)
{
  FAR struct sixlowpan_reassbuf_s *reass;
  int ret;

  sixlowpan_dumpbuffer("Incoming packet",
                       (FAR const uint8_t *)IPv6BUF(&radio->r_dev),
                       radio->r_dev.d_len);

#ifdef CONFIG_NET_PKT
  /* When packet sockets are enabled, feed the frame into the tap */

  ninfo("Packet tap\n");
  pkt_input(&radio->r_dev);
#endif

  /* We only accept IPv6 packets. */

  ninfo("IPv6 packet dispatch\n");
  NETDEV_RXIPV6(&radio->r_dev);

  /* Give the IPv6 packet to the network layer.  NOTE:  If there is a
   * problem with IPv6 header, it will be silently dropped and d_len will
   * be set to zero.  Oddly, ipv6_input() will return OK in this case.
   */

  ret = ipv6_input(&radio->r_dev);

  /* Free the reassemby buffer */

  reass = (FAR struct sixlowpan_reassbuf_s *)radio->r_dev.d_buf;
  DEBUGASSERT(reass != NULL);
  sixlowpan_reass_free(reass);

  return ret;
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
 *   This function is called when the radio device driver has received an
 *   frame from the network.  The frame from the device driver must be
 *   provided in by the IOB frame argument of the  function call:
 *
 *   - The frame data is in the IOB io_data[] buffer,
 *   - The length of the frame is in the IOB io_len field, and
 *   - The offset past and radio MAC header is provided in the io_offset
 *     field.
 *
 *   The frame argument may refer to a single frame (a list of length one)
 *   or may it be the head of a list of multiple frames.
 *
 *   - The io_flink field points to the next frame in the list (if enable)
 *   - The last frame in the list will have io_flink == NULL.
 *
 *   An non-NULL d_buf of size CONFIG_NET_6LOWPAN_PKTSIZE +
 *   CONFIG_NET_GUARDSIZE must also be provided. The frame will be
 *   decompressed and placed in the d_buf. Fragmented packets will also be
 *   reassembled in the d_buf as they are received (meaning for the driver,
 *   that two packet buffers are required: One for reassembly of RX packets
 *   and one used for TX polling).
 *
 *   After each frame is processed into d_buf, the IOB is deallocated.  If
 *   reassembly is incomplete, the partially reassembled packet must be
 *   preserved by the radio network driver and provided again when the next
 *   frame is received.
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
 *   radio       The radio network driver interface.
 *   framelist - The head of an incoming list of frames.  Normally this
 *               would be a single frame.  A list may be provided if
 *               appropriate, however.
 *   metadata  - Meta data characterizing the received packet.  The specific
 *               type of this metadata is obfuscated and depends on the
 *               type of the radio driver.  This could be be either
 *               (1) struct ieee802154_data_ind_s for an IEEE 802.15.4
 *               radio, or (2) struct pktradio_metadata_s for a non-standard
 *               packet radio.
 *
 *               If there are multilple frames in the list, this metadata
 *               must apply to all of the frames in the list.
 *
 * Returned Value:
 *   Zero (OK) is returned if the the frame was consumed; Otherwise a negated
 *   errno value is returned.
 *
 ****************************************************************************/

int sixlowpan_input(FAR struct radio_driver_s *radio,
                    FAR struct iob_s *framelist,  FAR const void *metadata)
{
  int ret = -EINVAL;
  uint8_t *d_buf_backup;

  DEBUGASSERT(radio != NULL && framelist != NULL);

  /* Sixlowpan modifies the d_buf to process fragments using reassembly
   * buffers. Save the value of d_buf on entry and set it back before
   * returning
   */

  d_buf_backup = radio->r_dev.d_buf;

  /* Verify that an frame has been provided. */

  while (framelist != NULL)
    {
      FAR struct iob_s *iob;

      /* Remove the IOB containing the frame from the device structure */

      iob       = framelist;
      framelist = iob->io_flink;

      sixlowpan_dumpbuffer("Incoming frame", iob->io_data, iob->io_len);

      /* Process the frame, decompressing it into the packet buffer */

      ret = sixlowpan_frame_process(radio, metadata, iob);

      /* If the frame was a valid 6LoWPAN frame, free the IOB the held the
       * consumed frame. Otherwise, the frame must stay allocated since the
       * MAC layer will try and pass it to another receiver to see if that
       * receiver wants it.
       */

      if (ret >= 0)
        {
          iob_free(iob);
        }

      /* Was the frame successfully processed? Is the packet in d_buf fully
       * reassembled?
       */

      if (ret == INPUT_COMPLETE)
        {
          /* Inject the uncompressed, reassembled packet into the network */

          ret = sixlowpan_dispatch(radio);
          if (ret >= 0)
            {
              /* Check if this resulted in a request to send an outgoing
               * packet.
               */

              if (radio->r_dev.d_len > 0)
                {
                  FAR struct ipv6_hdr_s *ipv6hdr;
                  FAR uint8_t *buffer;
                  struct netdev_varaddr_s destmac;
                  size_t hdrlen;
                  size_t buflen;

                  /* The IPv6 header followed by TCP or UDP headers should
                   * lie at the beginning of d_buf since there is no link
                   * layer protocol header.
                   */

                  ipv6hdr = IPv6BUF(&radio->r_dev);

                  /* Get the IEEE 802.15.4 MAC address of the destination.
                   * This assumes an encoding of the MAC address in the IPv6
                   * address.
                   */

                  ret = sixlowpan_nexthopaddr(radio, ipv6hdr->destipaddr,
                                              &destmac);
                  if (ret < 0)
                    {
                      nerr("ERROR: Failed to get dest MAC address: %d\n",
                           ret);
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
                          FAR struct tcp_hdr_s *tcp = TCPBUF(&radio->r_dev);
                          uint16_t tcplen;

                          /* The TCP header length is encoded in the top 4
                           * bits of the tcpoffset field (in units of 32-bit
                           * words).
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
                          nwarn("WARNING: Unsupported prototype: %u\n",
                                ipv6hdr->proto);
                          goto drop;
                        }
                    }

                  if (hdrlen > radio->r_dev.d_len)
                    {
                      nwarn("WARNING: Packet too small: Have %u need >%zu\n",
                            radio->r_dev.d_len, hdrlen);
                      goto drop;
                    }

                  /* Convert the outgoing packet into a frame list. */

                  buffer = radio->r_dev.d_buf + hdrlen;
                  buflen = radio->r_dev.d_len - hdrlen;

                  ret = sixlowpan_queue_frames(radio, ipv6hdr, buffer,
                                               buflen, &destmac);
drop:
                  radio->r_dev.d_len = 0;

                  /* We consumed the frame, so we must return 0. */

                  ret = 0;
                }
            }
        }
    }

  /* Restore the d_buf back to it's original state */

  radio->r_dev.d_buf = d_buf_backup;

  return ret;
}

#endif /* CONFIG_NET_6LOWPAN */
