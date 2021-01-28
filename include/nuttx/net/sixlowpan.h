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
#include <stdbool.h>

#include <nuttx/clock.h>
#include <nuttx/net/netdev.h>

#ifdef CONFIG_NET_6LOWPAN

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Frame format definitions *************************************************/

/* Fragment header.
 *
 * The fragment header is used when the payload is too large to fit in a
 * single radio frame. The fragment header contains three fields: Datagram
 * size, datagram tag and datagram offset.
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
#define SIXLOWPAN_HC_UDP_ALL_C            0xe0 /* All compressed */

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
#  define SIXLOWPAN_IPHC_SAM_128          0x00  /*   128-bits   */
#  define SIXLOWPAN_IPHC_SAM_64           0x10  /*   64-bits    */
#  define SIXLOWPAN_IPHC_SAM_16           0x20  /*   16-bits    */
#  define SIXLOWPAN_IPHC_SAM_0            0x30  /*   0-bits     */
#define SIXLOWPAN_IPHC_M                  0x08  /* Bit 12: Multicast compression */
#define SIXLOWPAN_IPHC_DAC                0x04  /* Bit 13: Destination address compression */
#define SIXLOWPAN_IPHC_DAM_MASK           0x03  /* Bits 14-15: Destination address mode */
                                                /* M=0 DAC=0/1: */
#  define SIXLOWPAN_IPHC_DAM_128          0x00  /*   128-bits   */
#  define SIXLOWPAN_IPHC_DAM_64           0x01  /*   64-bits    */
#  define SIXLOWPAN_IPHC_DAM_16           0x02  /*   16-bits    */
#  define SIXLOWPAN_IPHC_DAM_0            0x03  /*   0-bits     */
                                                /* M=1 DAC=0:   */
#  define SIXLOWPAN_IPHC_MDAM_128         0x00  /*   128-bits   */
#  define SIXLOWPAN_IPHC_MDAM_48          0x01  /*   48-bits: ffxx::00xx:xxxx:xxxx  */
#  define SIXLOWPAN_IPHC_MDAM_32          0x02  /*   16-bits: ffxx::00xx:xxxx  */
#  define SIXLOWPAN_IPHC_MDAM_8           0x03  /*   8-bits:  ff02::00xx */
                                                /* M=1 DAC=1:   */
#  define SIXLOWPAN_IPHC_MDDAM_48         0x00  /*   48-bits: ffxx:xxll:pppp:pppp:pppp:pppp:xxxx:xxxx */

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

/* IEEE 802.15.4 Definitions ************************************************/

/* By default, a 2-byte short address is used for the IEEE802.15.4 MAC
 * device's link layer address.  If CONFIG_NET_6LOWPAN_EXTENDEDADDR
 * is selected, then an 8-byte extended address will be used.
 */

#define NET_6LOWPAN_SADDRSIZE  2
#define NET_6LOWPAN_EADDRSIZE  8

#ifdef CONFIG_NET_6LOWPAN_EXTENDEDADDR
#  define NET_6LOWPAN_ADDRSIZE NET_6LOWPAN_EADDRSIZE
#else
#  define NET_6LOWPAN_ADDRSIZE NET_6LOWPAN_SADDRSIZE
#endif

/* This maximum size of an IEEE802.15.4 frame.  Certain, non-standard
 * devices may exceed this value, however.
 */

#define SIXLOWPAN_MAC_STDFRAME 127

/* Space for a two byte FCS must be reserved at the end of the frame.
 * REVISIT:  True for IEEE 802.15.4, but not for some other packet radios.
 */

#define SIXLOWPAN_MAC_FCSSIZE  2

/* Address compressibility test macros **************************************/

/* Check whether we can compress the IID in address 'a' to 16 bits.  This is
 * used for unicast addresses only, and is true if the address is on the
 * format <PREFIX>::0000:00ff:fe00:XXXX
 *
 * NOTE: we currently assume 64-bits prefixes
 */

/* Check whether we can compress the IID in address 'a' to 16 bits.  This is
 * used for unicast addresses only, and is true if the address is on the
 * format <PREFIX>::0000:00ff:fe00:00xx.
 *
 * NOTE: we currently assume 64-bits prefixes.  Big-endian, network order is
 * assumed.
 */

/* Check whether we can compress the IID in address 'a' to 8 bits.  This is
 * used for unicast addresses only, and is true if the address is on the
 * format <PREFIX>::0000:00ff:fe00:00XX.
 *
 * NOTE: we currently assume 64-bits prefixes.  Big-endian, network order is
 * assumed.
 */

#define SIXLOWPAN_IS_IID_8BIT_COMPRESSABLE(a) \
  ((((a)[4]) == 0x0000) && (((a)[5]) == HTONS(0x00ff)) && \
   (((a)[6]) == HTONS(0xfe00)) && ((((a)[7]) & HTONS(0xff00)) == 0))

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

/* Fragmentation Support
 *
 * This structure defines the reassembly buffer.  NOTE:  The packet buffer
 * is needed even in the case where reassembly is disabled.
 */

struct sixlowpan_reassbuf_s
{
  /* This is the externally visible packet buffer.  This is assigned
   * to the driver's d_buf field when the reassembly is complete and
   * provides the full reassembly packet to the network.
   */

  uint8_t rb_buf[CONFIG_NET_6LOWPAN_PKTSIZE + CONFIG_NET_GUARDSIZE];

  /* Memory pool used to allocate this reassembly buffer */

  uint8_t rb_pool;

  /* True if the reassemby buffer is active (set to false when reassembly is
   * complete).
   */

  bool rb_active;

  /* Supports a singly linked list */

  FAR struct sixlowpan_reassbuf_s *rb_flink;

  /* Fragmentation is handled frame by frame and requires that certain
   * state information be retained from frame to frame.  That additional
   * information follows the externally visible packet buffer.
   */

  /* rb_dgramtag.  Datagram tag to be put in the header of the set of
   * fragments.  It is used by the recipient to match fragments of the
   * same payload.
   *
   * This is the sender's copy of the tag.  It is incremented after each
   * fragmented packet is sent so that it will be unique to that
   * sequence fragmentation.  Its value is then persistent, the values of
   * other fragmentation variables are valid on during a single
   * fragmentation sequence (while rb_accumlen > 0)
   */

  uint16_t rb_dgramtag;

  /* rb_reasstag.  Each frame in the reassembly has a tag.  That tag must
   * match the reassembly tag in the fragments being merged.
   *
   * This is the same tag as rb_dgramtag but is saved on the receiving
   * side to match all of the fragments of the packet.
   */

  uint16_t rb_reasstag;

  /* rb_pktlen. The total length of the IPv6 packet to be re-assembled in
   * d_buf.  Used to determine when the re-assembly is complete.
   */

  uint16_t rb_pktlen;

  /* The current accumulated length of the packet being received in d_buf.
   * Included IPv6 and protocol headers.  Currently used only to determine
   * there is a fragmentation sequence in progress.
   */

  uint16_t rb_accumlen;

  /* rb_boffset.  Offset to the beginning of data in d_buf.  As each fragment
   * is received, data is placed at an appriate offset added to this.
   */

  uint16_t rb_boffset;

  /* The source MAC address of the fragments being merged */

  struct netdev_varaddr_s rb_fragsrc;

  /* That time at which reassembly was started.  If the elapsed time
   * exceeds CONFIG_NET_6LOWPAN_MAXAGE, then the reassembly will
   * be cancelled.
   */

  clock_t rb_time;
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
 *   CONFIG_NET_GUARDSIZE must also be provided.
 *   The frame will be decompressed and placed in the d_buf.
 *   Fragmented packets will also be reassembled in the d_buf as
 *   they are received (meaning for the driver, that two packet buffers are
 *   required: One for reassembly of RX packets and one used for TX polling).
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
 *   Ok is returned on success; Otherwise a negated errno value is returned.
 *
 ****************************************************************************/

struct radio_driver_s;   /* Forward reference.  See radiodev.h */
struct iob_s;            /* Forward reference See iob.h */

int sixlowpan_input(FAR struct radio_driver_s *radio,
                    FAR struct iob_s *framelist, FAR const void *metadata);

#endif /* CONFIG_NET_6LOWPAN */
#endif /* __INCLUDE_NUTTX_NET_SIXLOWPAN_H */
