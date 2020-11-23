/****************************************************************************
 * net/sixlowpan/sixlowpan_framelist.c
 *
 *   Copyright (C) 2017-2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Parts of this file derive from Contiki:
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
 *
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

#include <nuttx/mm/iob.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/radiodev.h>
#include <nuttx/wireless/ieee802154/ieee802154_mac.h>

#include "utils/utils.h"
#include "sixlowpan/sixlowpan_internal.h"

#ifdef CONFIG_NET_6LOWPAN

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* A single IOB must be big enough to hold a full frame.  This we have to
 * check at run time.  A IOB must also be big enough to hold the maximum MAC
 * header (25 bytes?) plus the FCS and have some amount of space left for
 * the payload.
 */

#define MAX_MACHDR 25 /* REVISIT: This is IEEE 802.15.4 specific */
#if CONFIG_IOB_BUFSIZE < (SIXLOWPAN_MAC_FCSSIZE + MAX_MACHDR)
#  error CONFIG_IOB_BUFSIZE too small to hold a IEEE802.14.5 frame
#endif

/* There must be at least enough IOBs to hold the full MTU.  Probably still
 * won't work unless there are a few more.
 */

#if CONFIG_NET_6LOWPAN_PKTSIZE > (CONFIG_IOB_BUFSIZE * CONFIG_IOB_NBUFFERS)
#  error Not enough IOBs to hold one full 6LoWPAN packet
#endif

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
 *   ipv6hdr - Pointer to the IPv6 header to "compress"
 *   fptr    - Pointer to the beginning of the frame under construction
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static int sixlowpan_compress_ipv6hdr(FAR const struct ipv6_hdr_s *ipv6hdr,
                                      FAR uint8_t *fptr)
{
  /* Indicate the IPv6 dispatch and length */

  fptr[g_frame_hdrlen] = SIXLOWPAN_DISPATCH_IPV6;
  g_frame_hdrlen      += SIXLOWPAN_IPV6_HDR_LEN;

  /* Copy the IPv6 header and adjust pointers */

  memcpy(&fptr[g_frame_hdrlen], ipv6hdr, IPv6_HDRLEN);
  g_frame_hdrlen      += IPv6_HDRLEN;
  g_uncomp_hdrlen     += IPv6_HDRLEN;

  return COMPRESS_HDR_INLINE;
}

/****************************************************************************
 * Name: sixlowpan_protosize
 *
 * Description:
 *   Get the size of any uncompressed protocol header that follows the
 *   IPv6 header.
 *
 ****************************************************************************/

static uint16_t sixlowpan_protosize(FAR const struct ipv6_hdr_s *ipv6hdr,
                                    FAR uint8_t *fptr)
{
  uint16_t protosize;

  /* Copy the following protocol header, */

  switch (ipv6hdr->proto)
    {
#ifdef CONFIG_NET_TCP
      case IP_PROTO_TCP:
        {
          FAR struct tcp_hdr_s *tcp =
            &((FAR struct ipv6tcp_hdr_s *)ipv6hdr)->tcp;

          /* The TCP header length is encoded in the top 4 bits of the
           * tcpoffset field (in units of 32-bit words).
           */

          protosize = ((uint16_t)tcp->tcpoffset >> 4) << 2;
        }
        break;
#endif

#ifdef CONFIG_NET_UDP
      case IP_PROTO_UDP:
        protosize = UDP_HDRLEN;
        break;
#endif

#ifdef CONFIG_NET_ICMPv6
      case IP_PROTO_ICMP6:
        protosize = ICMPv6_HDRLEN;
        break;
#endif

      default:
        nwarn("WARNING: Unrecognized proto: %u\n", ipv6hdr->proto);
        protosize = 0;
        break;
    }

  return protosize;
}

/****************************************************************************
 * Name: sixlowpan_ieee802154_metadata
 *
 * Description:
 *   Create the meta data that describes the IEEE 802.15.4 MAC header.
 *
 * Input Parameters:
 *   radio   - The radio network driver instance
 *   destmac - The IEEE802.15.4 MAC address of the destination
 *   meta    - Location to return the final metadata.
 *
 * Returned Value:
 *   OK is returned on success; Otherwise a negated errno value is returned.
 *
 ****************************************************************************/

#ifdef CONFIG_WIRELESS_IEEE802154
static int sixlowpan_ieee802154_metadata(FAR struct radio_driver_s *radio,
                                 FAR const struct netdev_varaddr_s *destmac,
                                 FAR union sixlowpan_metadata_u *meta)
{
  struct ieee802_txmetadata_s pktmeta;
  int ret;

  /* Reset frame meta data */

  memset(&pktmeta, 0, sizeof(struct ieee802_txmetadata_s));
  pktmeta.xmits = CONFIG_NET_6LOWPAN_MAX_MACTRANSMITS;

  /* Set stream mode for all TCP packets, except FIN packets. */

#if 0 /* Currently the frame type is always data */
  if (ipv6->proto == IP_PROTO_TCP)
    {
      FAR const struct tcp_hdr_s *tcp =
        &((FAR const struct ipv6tcp_hdr_s *)ipv6)->tcp;

      if ((tcp->flags & TCP_FIN) == 0 &&
          (tcp->flags & TCP_CTL) != TCP_ACK)
        {
          pktmeta.type = FRAME_ATTR_TYPE_STREAM;
        }
      else if ((tcp->flags & TCP_FIN) == TCP_FIN)
        {
          pktmeta.type = FRAME_ATTR_TYPE_STREAM_END;
        }
    }
#endif

  /* Set the source and destination address.  The source MAC address
   * is a fixed size, determined by a configuration setting.  The
   * destination MAC address many be either short or extended.
   */

#ifdef CONFIG_NET_6LOWPAN_EXTENDEDADDR
  pktmeta.sextended = TRUE;
  sixlowpan_eaddrcopy(pktmeta.source.nm_addr,
                      &radio->r_dev.d_mac.radio.nv_addr);
#else
  sixlowpan_saddrcopy(pktmeta.source.nm_addr,
                      &radio->r_dev.d_mac.radio.nv_addr);
#endif

  /* Copy the destination node address into the meta data */

  if (destmac->nv_addrlen == NET_6LOWPAN_EADDRSIZE)
    {
      pktmeta.dextended = TRUE;
      sixlowpan_eaddrcopy(pktmeta.dest.nm_addr, destmac->nv_addr);
    }
  else
    {
      DEBUGASSERT(destmac->nv_addrlen == NET_6LOWPAN_SADDRSIZE);
      sixlowpan_saddrcopy(pktmeta.dest.nm_addr, destmac->nv_addr);
    }

  /* Get the destination PAN ID.
   *
   * REVISIT: For now I am assuming that the source and destination
   * PAN IDs are the same.
   */

  sixlowpan_src_panid(radio, pktmeta.dpanid);

  /* Based on the collected attributes and addresses, construct the MAC meta
   * data structure that we need to interface with the IEEE802.15.4 MAC (we
   * will update the MSDU payload size when the IOB has been setup).
   */

  ret = sixlowpan_meta_data(radio, &pktmeta, &meta->ieee802154);
  if (ret < 0)
    {
      nerr("ERROR: sixlowpan_meta_data() failed: %d\n", ret);
    }

  return ret;
}
#endif

/****************************************************************************
 * Name: sixlowpan_pktradio_metadata
 *
 * Description:
 *   Create the meta data that describes the MAC header for a generic radio.
 *
 * Input Parameters:
 *   radio   - The radio network driver instance
 *   destmac - The radio-specific MAC address of the destination
 *   meta    - Location to return the final metadata.
 *
 * Returned Value:
 *   OK is returned on success; Otherwise a negated errno value is returned.
 *
 ****************************************************************************/

#ifdef CONFIG_WIRELESS_PKTRADIO
static int sixlowpan_pktradio_metadata(FAR struct radio_driver_s *radio,
                               FAR const struct netdev_varaddr_s *destmac,
                               FAR union sixlowpan_metadata_u *meta)
{
  FAR struct pktradio_metadata_s *pktmeta = &meta->pktradio;

  /* Reset the meta data */

  memset(pktmeta, 0, sizeof(struct pktradio_metadata_s));

  /* Set the source address */

  pktmeta->pm_src.pa_addrlen = radio->r_dev.d_mac.radio.nv_addrlen;
  memcpy(pktmeta->pm_src.pa_addr,
         radio->r_dev.d_mac.radio.nv_addr,
         radio->r_dev.d_mac.radio.nv_addrlen);

  /* Set the destination address.
   * REVISIT: Do we need to check for multicast or broadcast addresses
   * here?
   */

  pktmeta->pm_dest.pa_addrlen = destmac->nv_addrlen;
  memcpy(pktmeta->pm_dest.pa_addr, destmac->nv_addr, destmac->nv_addrlen);
  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sixlowpan_queue_frames
 *
 * Description:
 *   Process an outgoing UDP, TCP, or ICMPv6 packet.  This function is
 *   called from send event handler when a TX poll is received.  It
 *   formats the list of frames to be sent by the IEEE802.15.4 MAC driver.
 *
 *   The payload data is in the caller 'buf' and is of length 'buflen'.
 *   Compressed headers will be added and if necessary the packet is
 *   fragmented. The resulting packet/fragments are submitted to the MAC
 *   where they are queue for transfer.
 *
 * Input Parameters:
 *   radio   - The radio network driver instance
 *   ipv6    - IPv6 header followed by TCP, UDP, or ICMPv6 header.
 *   buf     - Beginning of the packet packet to send (with IPv6 + protocol
 *             headers)
 *   buflen  - Length of data to send (includes IPv6 and protocol headers)
 *   destmac - The IEEE802.15.4 MAC address of the destination
 *
 * Returned Value:
 *   Ok is returned on success; Otherwise a negated errno value is returned.
 *   This function is expected to fail if the driver is not an IEEE802.15.4
 *   MAC network driver.  In that case, the UDP/TCP will fall back to normal
 *   IPv4/IPv6 formatting.
 *
 * Assumptions:
 *   Called with the network locked.
 *
 ****************************************************************************/

int sixlowpan_queue_frames(FAR struct radio_driver_s *radio,
                           FAR const struct ipv6_hdr_s *ipv6,
                           FAR const void *buf, size_t buflen,
                           FAR const struct netdev_varaddr_s *destmac)
{
  union sixlowpan_metadata_u meta;
  FAR struct iob_s *iob;
  FAR uint8_t *fptr;
  int framer_hdrlen;
  struct netdev_varaddr_s bcastmac;
  uint16_t framelen;
  uint16_t pktlen;
  uint16_t paysize;
  uint16_t outlen = 0;
  uint8_t protosize;
  int ret;

  ninfo("buflen=%lu\n", (unsigned long)buflen);

  /* Initialize global data.  Locking the network guarantees that we have
   * exclusive use of the global values for intermediate calculations.
   */

  g_uncomp_hdrlen = 0;
  g_frame_hdrlen  = 0;
  protosize       = 0;

  /* The destination address will be tagged to each outbound packet. If the
   * argument destmac is NULL, we are sending a broadcast packet.
   */

  if (destmac == NULL)
    {
      memset(&bcastmac, 0, sizeof(struct netdev_varaddr_s));
      destmac = &bcastmac;
    }

  /* Pre-allocate the IOB to hold frame or the first fragment, waiting if
   * necessary.
   */

  iob = net_ioballoc(false, IOBUSER_NET_6LOWPAN);
  DEBUGASSERT(iob != NULL);

  /* Initialize the IOB */

  iob->io_flink  = NULL;
  iob->io_len    = 0;
  iob->io_offset = 0;
  iob->io_pktlen = 0;
  fptr           = iob->io_data;

  ninfo("Sending packet length %zd\n", buflen);

  /* Get the metadata that describes the MAC header on the packet */

#ifdef CONFIG_WIRELESS_IEEE802154
#ifdef CONFIG_WIRELESS_PKTRADIO
  if (radio->r_dev.d_lltype == NET_LL_IEEE802154)
#endif
    {
      ret = sixlowpan_ieee802154_metadata(radio, destmac, &meta);
    }
#endif
#ifdef CONFIG_WIRELESS_PKTRADIO
#ifdef CONFIG_WIRELESS_IEEE802154
  else
#endif
    {
      ret = sixlowpan_pktradio_metadata(radio, destmac, &meta);
    }
#endif

  /* Pre-calculate frame header length. */

  framer_hdrlen = sixlowpan_frame_hdrlen(radio, &meta);
  if (framer_hdrlen < 0)
    {
      /* Failed to determine the size of the header failed. */

      nerr("ERROR: sixlowpan_frame_hdrlen() failed: %d\n", framer_hdrlen);
      return framer_hdrlen;
    }

  /* This sill be the initial offset into io_data.  Valid data begins at
   * this offset and must be reflected in io_offset.
   */

  g_frame_hdrlen = framer_hdrlen;
  iob->io_offset = framer_hdrlen;

#ifndef CONFIG_NET_6LOWPAN_COMPRESSION_IPv6
  if (buflen >= CONFIG_NET_6LOWPAN_COMPRESSION_THRESHOLD)
    {
      /* Try to compress the headers */

#if defined(CONFIG_NET_6LOWPAN_COMPRESSION_HC1)
      ret = sixlowpan_compresshdr_hc1(radio, ipv6, destmac, fptr);
#elif defined(CONFIG_NET_6LOWPAN_COMPRESSION_HC06)
      ret = sixlowpan_compresshdr_hc06(radio, ipv6, destmac, fptr);
#else
#  error No compression specified
#endif
    }
  else
#endif /* !CONFIG_NET_6LOWPAN_COMPRESSION_IPv6 */
    {
      /* Small.. use IPv6 dispatch (no compression) */

      ret = sixlowpan_compress_ipv6hdr(ipv6, fptr);
    }

  /* Get the size of any uncompressed protocol headers */

  if (ret == COMPRESS_HDR_INLINE)
    {
      protosize = sixlowpan_protosize(ipv6, fptr);
    }

  ninfo("Header of length=%u protosize=%u\n", g_frame_hdrlen, protosize);

  /* Get the maximum packet size supported by this radio. */

  ret = sixlowpan_radio_framelen(radio);
  if (ret < 0)
    {
      nerr("ERROR: sixlowpan_radio_framelen() failed: %d\n", ret);
      return ret;
    }

  /* Limit to the maximum size supported by the IOBs */

  if (ret > CONFIG_IOB_BUFSIZE)
    {
      ret = CONFIG_IOB_BUFSIZE;
    }

  /* Reserve space at the end for any FCS that the hardware may include
   * in the payload.
   */

  ret -= SIXLOWPAN_MAC_FCSSIZE;
  if (ret < MAX_MACHDR || ret > UINT16_MAX)
    {
      nerr("ERROR: Invalid frame size: %d\n", ret);
      return ret;
    }

  framelen = (uint16_t)ret;

  /* Check if we need to fragment the packet into several frames.
   * We may need to reserve space at the end of the frame for a 2-byte FCS
   */

  if (buflen > (framelen - g_frame_hdrlen - protosize))
    {
      /* qhead will hold the generated frame list; frames will be
       * added at qtail.
       */

      FAR struct sixlowpan_reassbuf_s *reass;
      FAR struct iob_s *qhead;
      FAR struct iob_s *qtail;
      FAR uint8_t *frame1;
      FAR uint8_t *fragptr;
      uint16_t frag1_hdrlen;

      /* Recover the reassembly buffer from the driver d_buf. */

      reass = (FAR struct sixlowpan_reassbuf_s *)radio->r_dev.d_buf;
      DEBUGASSERT(reass != NULL);

      /* The outbound IPv6 packet is too large to fit into a single 15.4
       * packet, so we fragment it into multiple packets and send them.
       * The first fragment contains frag1 dispatch, then
       * IPv6/HC1/HC06/HC_UDP dispatchs/headers.
       * The following fragments contain only the fragn dispatch.
       */

      ninfo("Sending fragmented packet length %zd\n", buflen);

      /* Create 1st Fragment */

      /* Move HC1/HC06/IPv6 header to make space for the FRAG1 header at the
       * beginning of the frame.
       */

      fragptr = fptr + framer_hdrlen;
      memmove(fragptr + SIXLOWPAN_FRAG1_HDR_LEN, fragptr,
              g_frame_hdrlen - framer_hdrlen);

      /* Setup up the fragment header.
       *
       * The fragment header contains three fields:  Datagram size, datagram
       * tag and datagram offset:
       *
       *   1. Datagram size describes the total (un-fragmented) payload.
       *   2. Datagram tag identifies the set of fragments and is used to
       *      match fragments of the same payload.
       *   3. Datagram offset identifies the fragment’s offset within the
       *      unfragmented payload.
       *
       * The fragment header length is 4 bytes for the first header and 5
       * bytes for all subsequent headers.
       */

      pktlen = buflen + g_uncomp_hdrlen + protosize;
      PUTHOST16(fragptr, SIXLOWPAN_FRAG_DISPATCH_SIZE,
                ((SIXLOWPAN_DISPATCH_FRAG1 << 8) | pktlen));
      PUTHOST16(fragptr, SIXLOWPAN_FRAG_TAG, reass->rb_dgramtag);

      g_frame_hdrlen += SIXLOWPAN_FRAG1_HDR_LEN;

      /* Copy any uncompressed protocol headers that must appear only in th
       * first fragment.
       */

      if (protosize > 0)
        {
          FAR uint8_t *src = (FAR uint8_t *)ipv6 + IPv6_HDRLEN;
          memcpy(fptr + g_frame_hdrlen, src, protosize);
        }

      /* Copy payload and enqueue.  NOTE that the size is a multiple of eight
       * bytes.
       */

      paysize = (framelen - g_frame_hdrlen) & ~7;
      memcpy(fptr + g_frame_hdrlen + protosize, buf, paysize - protosize);

      /* Set outlen to what we already sent from the IP payload */

      iob->io_len    = paysize + g_frame_hdrlen;
      outlen         = paysize;

      ninfo("First fragment: length %d, tag %d\n",
            paysize, reass->rb_dgramtag);
      sixlowpan_dumpbuffer("Outgoing frame",
                           (FAR const uint8_t *)iob->io_data, iob->io_len);

      /* Add the first frame to the IOB queue */

      ninfo("Queuing frame io_len=%u io_offset=%u\n",
            iob->io_len, iob->io_offset);

      qhead          = iob;
      qtail          = iob;

      /* Keep track of the total amount of data queue */

      iob->io_pktlen = iob->io_len;

      /* Create following fragments */

      frame1         = iob->io_data;
      frag1_hdrlen   = g_frame_hdrlen;

      while (outlen < (buflen + protosize))
        {
          uint16_t fragn_hdrlen;

          /* Allocate an IOB to hold the next fragment, waiting if
           * necessary.
           */

          iob = net_ioballoc(false, IOBUSER_NET_6LOWPAN);
          DEBUGASSERT(iob != NULL);

          /* Initialize the IOB */

          iob->io_flink  = NULL;
          iob->io_len    = 0;
          iob->io_offset = framer_hdrlen;
          iob->io_pktlen = 0;
          fptr           = iob->io_data;

          /* Copy the HC1/HC06/IPv6 header the frame header from first
           * frame, into the correct location after the FRAGN header
           * of subsequent frames.
           */

          fragptr = fptr + framer_hdrlen;
          memcpy(fragptr + SIXLOWPAN_FRAGN_HDR_LEN,
                 frame1 + framer_hdrlen + SIXLOWPAN_FRAG1_HDR_LEN,
                 frag1_hdrlen - framer_hdrlen);
          fragn_hdrlen = frag1_hdrlen - SIXLOWPAN_FRAG1_HDR_LEN;

          /* Setup up the FRAGN header after the frame header. */

          PUTHOST16(fragptr, SIXLOWPAN_FRAG_DISPATCH_SIZE,
                    ((SIXLOWPAN_DISPATCH_FRAGN << 8) | pktlen));
          PUTHOST16(fragptr, SIXLOWPAN_FRAG_TAG, reass->rb_dgramtag);
          fragptr[SIXLOWPAN_FRAG_OFFSET] = outlen >> 3;

          fragn_hdrlen += SIXLOWPAN_FRAGN_HDR_LEN;

          /* Copy payload and enqueue.
           *
           * Check for the last fragment.
           */

          paysize = (framelen - fragn_hdrlen) & SIXLOWPAN_DISPATCH_FRAG_MASK;
          if (paysize > buflen - outlen + protosize)
            {
              /* Last fragment, truncate to the correct length */

              paysize = buflen - outlen + protosize;
            }

          memcpy(fptr + fragn_hdrlen, buf + outlen - protosize, paysize);

          /* Set outlen to what we already sent from the IP payload */

          iob->io_len = paysize + fragn_hdrlen;
          outlen     += paysize;

          ninfo("Fragment offset=%d, paysize=%d, rb_dgramtag=%d\n",
                outlen >> 3, paysize, reass->rb_dgramtag);

          sixlowpan_dumpbuffer("Outgoing frame",
                               (FAR const uint8_t *)iob->io_data,
                               iob->io_len);

          /* Add the next frame to the tail of the IOB queue */

          ninfo("Queuing frame io_len=%u io_offset=%u\n",
                iob->io_len, iob->io_offset);

          qtail->io_flink = iob;
          qtail           = iob;

          /* Keep track of the total amount of data queue */

          qhead->io_pktlen += iob->io_len;
        }

      /* Submit all of the fragments to the MAC.  We send all frames back-
       * to-back like this to minimize any possible condition where some
       * frame which is not a fragment from this sequence from intervening.
       */

      for (iob = qhead; iob != NULL; iob = qhead)
        {
          /* Remove the IOB containing the frame from the list */

          qhead         = iob->io_flink;
          iob->io_flink = NULL;

          /* And submit the frame to the MAC */

          ninfo("Submitting frame\n");
          ret = sixlowpan_frame_submit(radio, &meta, iob);
          if (ret < 0)
            {
              nerr("ERROR: sixlowpan_frame_submit() failed: %d\n", ret);
            }
        }

      /* Update the datagram TAG value */

      reass->rb_dgramtag++;
    }
  else
    {
      /* The packet does not need to be fragmented just copy the "payload"
       * and send in one frame.
       */

      /* Copy any uncompressed protocol headers that must appear only in th
       * first fragment.
       */

      if (protosize > 0)
        {
          FAR uint8_t *src = (FAR uint8_t *)ipv6 + IPv6_HDRLEN;
          memcpy(fptr + g_frame_hdrlen, src, protosize);
        }

      /* Copy the payload into the frame. */

      memcpy(fptr + g_frame_hdrlen + protosize, buf, buflen);
      iob->io_len    = buflen + g_frame_hdrlen + protosize;
      iob->io_pktlen = iob->io_len;

      ninfo("Non-fragmented: length %d\n", iob->io_len);
      sixlowpan_dumpbuffer("Outgoing frame",
                       (FAR const uint8_t *)iob->io_data, iob->io_len);

      /* And submit the frame to the MAC */

      ninfo("Submitting frame length=%u io_offset=%u\n",
            iob->io_len, iob->io_offset);

      ret = sixlowpan_frame_submit(radio, &meta, iob);
      if (ret < 0)
        {
          nerr("ERROR: sixlowpan_frame_submit() failed: %d\n", ret);
        }
    }

  return OK;
}

#endif /* CONFIG_NET_6LOWPAN */
