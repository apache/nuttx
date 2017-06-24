/****************************************************************************
 * include/nuttx/net/sixlowpan.h
 * Header file for the 6lowpan implementation (RFC4944 and
 * draft-hui-6lowpan-hc-01)
 *
 *   Copyright (C) 2017, Gregory Nutt, all rights reserved
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Derives from Contiki:
 *
 *   Copyright (c) 2008, Swedish Institute of Computer Science.
 *   All rights reserved.
 *   Authors: Adam Dunkels <adam@sics.se>
 *            Nicolas Tsiftes <nvt@sics.se>
 *            Niclas Finne <nfi@sics.se>
 *            Mathilde Durvy <mdurvy@cisco.com>
 *            Julien Abeille <jabeille@cisco.com>
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

#ifndef __INCLUDE_NUTTX_NET_SIXLOWPAN_H
#define __INCLUDE_NUTTX_NET_SIXLOWPAN_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <nuttx/clock.h>
#include <nuttx/mm/iob.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/ieee802154.h>

#ifdef CONFIG_NET_6LOWPAN

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Frame format definitions *************************************************/
/* Fragment header.
 *
 * The fragment header is used when the payload is too large to fit in a
 * single IEEE 802.15.4 frame. The fragment header contains three fields:
 * Datagram size, datagram tag and datagram offset.
 *
 * 1. Datagram size describes the total (un-fragmented) payload.
 * 2. Datagram tag identifies the set of fragments and is used to match
 *    fragments of the same payload.
 * 3. Datagram offset identifies the fragment’s offset within the un-
 *    fragmented payload.
 *
 * The fragment header length is 4 bytes for the first header and 5
 * bytes for all subsequent headers.
 */

#define SIXLOWPAN_FRAG_DISPATCH_SIZE      0  /* 16 bit */
#define SIXLOWPAN_FRAG_TAG                2  /* 16 bit */
#define SIXLOWPAN_FRAG_OFFSET             4  /* 8 bit */

/* Define the frame buffer as a byte array */

#define SIXLOWPAN_HC1_DISPATCH            0  /* 8 bit */
#define SIXLOWPAN_HC1_ENCODING            1  /* 8 bit */
#define SIXLOWPAN_HC1_TTL                 2  /* 8 bit */

#define SIXLOWPAN_HC1_HC_UDP_DISPATCH     0  /* 8 bit */
#define SIXLOWPAN_HC1_HC_UDP_HC1_ENCODING 1  /* 8 bit */
#define SIXLOWPAN_HC1_HC_UDP_UDP_ENCODING 2  /* 8 bit */
#define SIXLOWPAN_HC1_HC_UDP_TTL          3  /* 8 bit */
#define SIXLOWPAN_HC1_HC_UDP_PORTS        4  /* 8 bit */
#define SIXLOWPAN_HC1_HC_UDP_CHKSUM       5  /* 16 bit */

/* Min and Max compressible UDP ports - HC06 */

#define SIXLOWPAN_UDP_4_BIT_PORT_MIN      0xf0b0
#define SIXLOWPAN_UDP_4_BIT_PORT_MAX      0xf0bf   /* f0b0 + 15 */
#define SIXLOWPAN_UDP_8_BIT_PORT_MIN      0xf000
#define SIXLOWPAN_UDP_8_BIT_PORT_MAX      0xf0ff   /* f000 + 255 */

/* 6lowpan dispatches */

#define SIXLOWPAN_DISPATCH_NALP           0x00 /* 00xxxxxx Not a LoWPAN packet */
#define SIXLOWPAN_DISPATCH_NALP_MASK      0xc0 /* 11000000 */

#define SIXLOWPAN_DISPATCH_IPV6           0x41 /* 01000001 Uncompressed IPv6 addresses */
#define SIXLOWPAN_DISPATCH_HC1            0x42 /* 01000010 HC1 Compressed IPv6 header */
#define SIXLOWPAN_DISPATCH_BC0            0x50 /* 01010000 BC0 Broadcast header */
#define SIXLOWPAN_DISPATCH_ESC            0x7f /* 01111111 Additional Dispatch octet follows  */

#define SIXLOWPAN_DISPATCH_IPHC           0x60 /* 011xxxxx IP Header Compression (IPHC)*/
#define SIXLOWPAN_DISPATCH_IPHC_MASK      0xe0 /* 11100000 */

#define SIXLOWPAN_DISPATCH_MESH           0x80 /* 10xxxxxx Mesh routing header */
#define SIXLOWPAN_DISPATCH_MESH_MASK      0xc0 /* 11000000 */

#define SIXLOWPAN_DISPATCH_FRAG1          0xc0 /* 11000xxx Fragmentation header (ﬁrst) */
#define SIXLOWPAN_DISPATCH_FRAGN          0xe0 /* 11100xxx Fragmentation header (subsequent) */
#define SIXLOWPAN_DISPATCH_FRAG_MASK      0xf8 /* 11111000 */

/* HC1 encoding (RFC4944)
 *
 *   PI:  Prefix carried in-line
 *   PC:  Prefix compressed (link-local prefix assumed)
 *   II:  Interface identifier carried in-line
 *   IC:  Interface identifier elided (derivable from the corresponding
 *        link-layer address).
 */

#define SIXLOWPAN_HC1_SRCADDR_MASK        0xc0 /* Bits 0-1: IPv6 source address */
#  define SIXLOWPAN_HC1_SRCADDR_PIII      0x00 /*   PI,II */
#  define SIXLOWPAN_HC1_SRCADDR_PIIC      0x40 /*   PI,IC */
#  define SIXLOWPAN_HC1_SRCADDR_PCII      0x80 /*   PC,II */
#  define SIXLOWPAN_HC1_SRCADDR_PCIC      0xc0 /*   PC,IC */
#define SIXLOWPAN_HC1_DESTADDR_MASK       0x30 /* Bits 2-3: IPv6 destination address */
#  define SIXLOWPAN_HC1_DESTADDR_PIII     0x00 /*   PI,II */
#  define SIXLOWPAN_HC1_DESTADDR_PIIC     0x10 /*   PI,IC */
#  define SIXLOWPAN_HC1_DESTADDR_PCII     0x20 /*   PC,II */
#  define SIXLOWPAN_HC1_DESTADDR_PCIC     0x30 /*   PC,IC */
#define SIXLOWPAN_HC1_TCFL_C              0x08 /* Bit 4: Traffic class and flow label are zero */
#define SIXLOWPAN_HC1_NH_MASK             0x06 /* Bits 5-6: Next HC1 header type */
#  define SIXLOWPAN_HC1_NH_NC             0x00 /*   Not compressed */
#  define SIXLOWPAN_HC1_NH_UDP            0x02 /*   UDP */
#  define SIXLOWPAN_HC1_NH_ICMPv6         0x04 /*   ICMPv6 */
#  define SIXLOWPAN_HC1_NH_TCP            0x06 /*   TCP */
#define SIXLOWPAN_HC1_H2ENCODE            0x01 /* Bit 0: HC2 encoding follows */

/* HC_UDP encoding (works together with HC1) */

#define SIXLOWPAN_HC_UDP_SRCPORT_C        0x80 /* Source port compressed to 4 bits */
#define SIXLOWPAN_HC_UDP_DESTPORT_C       0x40 /* Destination port compressed to 4 bits */
#define SIXLOWPAN_HC_UDP_LENGTH  _C       0x20 /* Elided, compute from IPv6 length */
#define SIXLOWPAN_HC_UDP_ALL_C            0xe0 /* All commpressed */

/* IPHC encoding
 *
 * Values of fields within the IPHC encoding first byte
 * (Using MS-to-LS bit numbering of the draft RFC)
 */
                                                /* Bits 0-2: 011 */
#define SIXLOWPAN_IPHC_TC_MASK            0x18  /* Bits 3-4: Traffic Class, Flow Label */
#  define SIXLOWPAN_IPHC_TC_00            0x00  /*   ECN+DSCP+4-bit Pad+Flow Label (4 bytes) */
#  define SIXLOWPAN_IPHC_TC_01            0x08  /*   ECN+2-bit Pad+ Flow Label (3 bytes), DSCP is elided. */
#  define SIXLOWPAN_IPHC_TC_10            0x10  /*   ECN+DSCP (1 byte), Flow Label is elided */
#  define SIXLOWPAN_IPHC_TC_11            0x18  /*   Traffic Class and Flow Label are elided */
#define SIXLOWPAN_IPHC_NH                 0x04  /* Bit 5: Next Header Compressed */
#define SIXLOWPAN_IPHC_HLIM_MASK          0x03  /* Bits 6-7: Hop Limit */
#  define SIXLOWPAN_IPHC_HLIM_INLINE      0x00  /*   Carried in-line */
#  define SIXLOWPAN_IPHC_HLIM_1           0x01  /*   Compressed hop limit of 1 */
#  define SIXLOWPAN_IPHC_HLIM_64          0x02  /*   Compressed hop limit of 64 */
#  define SIXLOWPAN_IPHC_HLIM_255         0x03  /*   Compressed hop limit of 255 */

/* Values of fields within the IPHC encoding second byte */

#define SIXLOWPAN_IPHC_CID                0x80  /* Bit 8: Context identifier extension */
#define SIXLOWPAN_IPHC_SAC                0x40  /* Bit 9: Source address compression */
#define SIXLOWPAN_IPHC_SAM_MASK           0x30  /* Bits 10-11: Source address mode */
#  define SIXLOWPAN_IPHC_SAM_128          0x00  /*   128-bits */
#  define SIXLOWPAN_IPHC_SAM_64           0x10  /*   64-bits */
#  define SIXLOWPAN_IPHC_SAM_16           0x20  /*   16-bits */
#  define SIXLOWPAN_IPHC_SAM_0            0x30  /*   0-bits */
#define SIXLOWPAN_IPHC_M                  0x08  /* Bit 12: Multicast compression */
#define SIXLOWPAN_IPHC_DAC                0x04  /* Bit 13: Destination address compression */
#define SIXLOWPAN_IPHC_DAM_MASK           0x03  /* Bits 14-15: Destination address mode */
#  define SIXLOWPAN_IPHC_DAM_128          0x00  /*   128-bits */
#  define SIXLOWPAN_IPHC_DAM_64           0x01  /*   64-bits */
#  define SIXLOWPAN_IPHC_DAM_16           0x02  /*   16-bits */
#  define SIXLOWPAN_IPHC_DAM_0            0x03  /*   0-bits */

#define SIXLOWPAN_IPHC_SAM_BIT            4
#define SIXLOWPAN_IPHC_DAM_BIT            0

/* Link local context number */

#define SIXLOWPAN_IPHC_ADDR_CONTEXT_LL    0

/* 16-bit multicast addresses compression */

#define SIXLOWPAN_IPHC_MCAST_RANGE        0xa0

/* NHC_EXT_HDR */

#define SIXLOWPAN_NHC_MASK                0xf0
#define SIXLOWPAN_NHC_EXT_HDR             0xe0

/* LOWPAN_UDP encoding (works together with IPHC) */

#define SIXLOWPAN_NHC_UDP_MASK            0xf8
#define SIXLOWPAN_NHC_UDP_ID              0xf0
#define SIXLOWPAN_NHC_UDP_CHECKSUMC       0x04
#define SIXLOWPAN_NHC_UDP_CHECKSUMI       0x00

/* Values for port compression, _with checksum_ ie bit 5 set to 0 */

#define SIXLOWPAN_NHC_UDP_CS_P_00         0xf0 /* All inline */
#define SIXLOWPAN_NHC_UDP_CS_P_01         0xf1 /* Source 16bit inline, dest = 0xf0 + 8 bit inline */
#define SIXLOWPAN_NHC_UDP_CS_P_10         0xf2 /* Source = 0xf0 + 8bit inline, dest = 16 bit inline */
#define SIXLOWPAN_NHC_UDP_CS_P_11         0xf3 /* Source & dest = 0xf0b + 4bit inline */

/* The 6lowpan "headers" length */

#define SIXLOWPAN_IPV6_HDR_LEN            1    /* One byte */
#define SIXLOWPAN_HC1_HDR_LEN             3
#define SIXLOWPAN_HC1_HC_UDP_HDR_LEN      7
#define SIXLOWPAN_FRAG1_HDR_LEN           4
#define SIXLOWPAN_FRAGN_HDR_LEN           5

/* Address compressibility test macros **************************************/

/* Check whether we can compress the IID in address 'a' to 16 bits.  This is
 * used for unicast addresses only, and is true if the address is on the
 * format <PREFIX>::0000:00ff:fe00:XXXX
 *
 * NOTE: we currently assume 64-bits prefixes
 */

/* Check whether we can compress the IID in address 'a' to 16 bits.  This is
 * used for unicast addresses only, and is true if the address is on the
 * format <PREFIX>::0000:00ff:fe00:XXXX.
 *
 * NOTE: we currently assume 64-bits prefixes.  Big-endian, network order is
 * assumed.
 */

#define SIXLOWPAN_IS_IID_16BIT_COMPRESSABLE(a) \
  ((((a)[4]) == 0x0000) && (((a)[5]) == HTONS(0x00ff)) && \
   (((a)[6]) == HTONS(0xfe00)))

/* Check whether the 9-bit group-id of the compressed multicast address is
 * known. It is true if the 9-bit group is the all nodes or all routers
 * group.  Parameter 'a' is typed uint8_t *
 */

#define SIXLOWPAN_IS_MCASTADDR_DECOMPRESSABLE(a) \
   (((*a & 0x01) == 0) && \
    ((*(a + 1) == 0x01) || (*(a + 1) == 0x02)))

/* Check whether the 112-bit group-id of the multicast address is mappable
 * to a 9-bit group-id. It is true if the group is the all nodes or all
 * routers group:
 *
 *    XXXX:0000:0000:0000:0000:0000:0000:0001  All nodes address
 *    XXXX:0000:0000:0000:0000:0000:0000:0002  All routers address
 */

#define SIXLOWPAN_IS_MCASTADDR_COMPRESSABLE(a) \
  ((a)[1] == 0 && (a)[2] == 0 && (a)[3] == 0 && \
   (a)[4] == 0 && (a)[5] == 0 && (a)[6] == 0 && \
   ((a)[7] == HTONS(0x0001) || (a)[7] == HTONS(0x0002)))

/* FFXX:0000:0000:0000:0000:00XX:XXXX:XXXX */

#define SIXLOWPAN_IS_MCASTADDR_COMPRESSABLE48(a) \
  ((a)[1] == 0 && (a)[2] == 0 && (a)[3] == 0 && \
   (a)[4] == 0 && (((a)[5] & HTONS(0xff00)) == 0))

/* FFXX:0000:0000:0000:0000:0000:00XX:XXXX */

#define SIXLOWPAN_IS_MCASTADDR_COMPRESSABLE32(a) \
  ((a)[1] == 0 && (a)[2] == 0 && (a)[3] == 0 && \
   (a)[4] == 0 && (a)[5] == 0 && ((a)[6] & HTONS(0xff00)) == 0)

/* FF02:0000:0000:0000:0000:0000:0000:00XX */

#define SIXLOWPAN_IS_MCASTADDR_COMPRESSABLE8(a) \
  ((((a)[0] & HTONS(0x00ff)) == HTONS(0x0002)) && \
   (a)[1] == 0 && (a)[2] == 0 && (a)[3] == 0 && \
   (a)[4] == 0 && (a)[5] == 0 && (a)[6] == 0 && \
   (((a)[7] & HTONS(0xff00)) == 0x0000))

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* The device structure for IEEE802.15.4 MAC network device differs from the
 * standard Ethernet MAC device structure.  The main reason for this
 * difference is that fragmentation must be supported.
 *
 * The IEEE802.15.4 MAC does not use the d_buf packet buffer directly.
 * Rather, it uses a list smaller frame buffers.
 *
 *   - The packet fragment data is provided in an IOB in the via the
 *     i_req_data() interface method each time that the IEEE802.15.4 MAC
 *     needs to send more data.  The length of the frame is provided in the
 *      io_len field of the IOB.
 *
 *     In this case, the d_buf is not used at all and, if fact, may be
 *     NULL.
 *
 *   - Received frames are provided by IEEE802.15.4 MAC to the network
 *     via an IOB parameter in the sixlowpan_submit() interface.  The
 *     length of the frawme is io_len and will be uncompressed and possibly
 *     reassembled in the d_buf;  d_len will hold the size of the
 *     reassembled packet.
 *
 *     In this case, a d_buf of size CONFIG_NET_6LOWPAN_MTU must be provided.
 *
 * This is accomplished by "inheriting" the standard 'struct net_driver_s'
 * and appending the frame buffer as well as other metadata needed to
 * manage the fragmentation.  'struct ieee802154_driver_s' is cast
 * compatible with 'struct net_driver_s' when CONFIG_NET_MULTINIC is not
 * defined or when dev->d_lltype == NET_LL_IEEE802154.
 *
 * The IEEE802.15.4 MAC network driver has reponsibility for initializing
 * this structure.  In general, all fields must be set to NULL.  In
 * addtion:
 *
 * 1. On a TX poll, the IEEE802.15.4 MAC driver should provide its driver
 *    structure.  During the course of the poll, the networking layer may
 *    generate outgoing frames.  These frames will by provided to the MAC
 *    driver via the req_data() method.
 *
 *    After sending each frame through the radio, the MAC driver must
 *    return the frame to the pool of free IOBs using the iob_free().
 *
 * 2. When receiving data both buffers must be provided:
 *
 *    The IEEE802.15.4 MAC driver should receive the frame data directly
 *    into the payload area of an IOB frame structure.  That IOB structure
 *    may be obtained using the iob_alloc() function.
 *
 *    The larger dev.d_buf must have a size of at least the advertised MTU
 *    of the protocol, CONFIG_NET_6LOWPAN_MTU, plus CONFIG_NET_GUARDSIZE.
 *    If fragmentation is enabled, then the logical packet size may be
 *    significantly larger than the size of the frame buffer.  The dev.d_buf
 *    is used for de-compressing each frame and reassembling any fragmented
 *    packets to create the full input packet that is provided to the
 *    application.
 *
 *    The MAC driver should then inform the network of the by calling
 *    sixlowpan_input().  That single frame (or, perhaps, list of frames)
 *    should be provided as second argument of that call.
 *
 *    The network will free the IOB by calling iob_free after it has
 *    processed the incoming frame.  As a complexity, the result of
 *    receiving a frame may be that the network may respond provide an
 *    outgoing frames in the via a nested calle to the req_data() method.
 */

struct ieee802154_frame_meta_s; /* Forward reference */
struct ieee802154_data_ind_s;   /* Forward reference */
struct iob_s;                   /* Forward reference */

struct ieee802154_driver_s
{
  /* This definitiona must appear first in the structure definition to
   * assure cast compatibility.
   */

  struct net_driver_s i_dev;

  /* IEEE802.15.4 MAC-specific definitions follow. */

  /* The msdu_handle is basically an id for the frame.  The standard just
   * says that the next highest layer should determine it.  It is used in
   * three places
   *
   * 1. When you do that data request
   * 2. When the transmission is complete, the conf_data is called with
   *    that handle so that the user can be notified of the frames success/
   *    failure
   * 3. For a req_purge, to basically "cancel" the transaction.  This is
   *    often particularly useful on a coordinator that has indirect data
   *    waiting to be requested from another device
   *
   * Here is a simple frame counter.
   */

  uint8_t i_msdu_handle;

#if CONFIG_NET_6LOWPAN_FRAG
  /* Fragmentation Support *************************************************/
  /* Fragmentation is handled frame by frame and requires that certain
   * state information be retained from frame to frame.
   */

  /* i_dgramtag.  Datagram tag to be put in the header of the set of
   * fragments.  It is used by the recipient to match fragments of the
   * same payload.
   *
   * This is the sender's copy of the tag.  It is incremented after each
   * fragmented packet is sent so that it will be unique to that
   * sequence fragmentation.  Its value is then persistent, the values of
   * other fragmentatin variables are valid on during a single
   * fragmentation sequence (while i_accumlen > 0)
   */

  uint16_t i_dgramtag;

  /* i_reasstag.  Each frame in the reassembly has a tag.  That tag must
   * match the reassembly tag in the fragments being merged.
   *
   * This is the same tag as i_dgramtag but is saved on the receiving
   * side to match all of the fragments of the packet.
   */

  uint16_t i_reasstag;

  /* i_pktlen. The total length of the IPv6 packet to be re-assembled in
   * d_buf.  Used to determine when the re-assembly is complete.
   */

  uint16_t i_pktlen;

  /* The current accumulated length of the packet being received in d_buf.
   * Included IPv6 and protocol headers.  Currently used only to determine
   * there is a fragmentation sequence in progress.
   */

  uint16_t i_accumlen;

  /* i_boffset.  Offset to the beginning of data in d_buf.  As each fragment
   * is received, data is placed at an appriate offset added to this.
   */

  uint16_t i_boffset;

  /* The source MAC address of the fragments being merged */

  struct sixlowpan_tagaddr_s i_fragsrc;

  /* That time at which reassembly was started.  If the elapsed time
   * exceeds CONFIG_NET_6LOWPAN_MAXAGE, then the reassembly will
   * be cancelled.
   */

  systime_t i_time;
#endif /* CONFIG_NET_6LOWPAN_FRAG */

  /* MAC network driver callback functions **********************************/
  /**************************************************************************
   * Name: mac802154_get_mhrlen
   *
   * Description:
   *   Calculate the MAC header length given the frame meta-data.
   *
   * Input parameters:
   *   netdev    - The networkd device that will mediate the MAC interface
   *   meta      - Meta data needed to recreate the MAC header
   *
   * Returned Value:
   *   A non-negative MAC headeer length is returned on success; a negated
   *   errno value is returned on any failure.
   *
   **************************************************************************/

  CODE int (*i_get_mhrlen)(FAR struct ieee802154_driver_s *netdev,
                           FAR const struct ieee802154_frame_meta_s *meta);

  /**************************************************************************
   * Name: mac802154_req_data
   *
   * Description:
   *   Requests the transfer of a list of frames to the MAC.
   *
   * Input parameters:
   *   netdev    - The networkd device that will mediate the MAC interface
   *   meta      - Meta data needed to recreate the MAC header
   *   framelist - Head of a list of frames to be transferred.
   *
   * Returned Value:
   *   Zero (OK) returned on success; a negated errno value is returned on
   *   any failure.
   *
   **************************************************************************/

  CODE int (*i_req_data)(FAR struct ieee802154_driver_s *netdev,
                         FAR const struct ieee802154_frame_meta_s *meta,
                         FAR struct iob_s *framelist);
};

/****************************************************************************
 * Public Function Prototypes
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
 *   ind       - Meta data characterizing the received packet.  If there are
 *               multilple frames in the list, this meta data must apply to
 *               all of the frames!
 *
 * Returned Value:
 *   Ok is returned on success; Othewise a negated errno value is returned.
 *
 ****************************************************************************/

int sixlowpan_input(FAR struct ieee802154_driver_s *ieee,
                    FAR struct iob_s *framelist,
                    FAR const struct ieee802154_data_ind_s *ind);

#endif /* CONFIG_NET_6LOWPAN */
#endif /* __INCLUDE_NUTTX_NET_SIXLOWPAN_H */
