/****************************************************************************
 * include/nuttx/net/sixlowpan.h
 * Header file for the 6lowpan implementation (RFC4944 and draft-hui-6lowpan-hc-01)
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

#ifndef __INCLUDE_NUTTX_NET_SIXLOWOAN_H
#define __INCLUDE_NUTTX_NET_SIXLOWOAN_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <nuttx/net/netdev.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Min and Max compressible UDP ports - HC06 */

#define SIXLOWPAN_UDP_4_BIT_PORT_MIN     0xf0b0
#define SIXLOWPAN_UDP_4_BIT_PORT_MAX     0xf0bf   /* F0B0 + 15 */
#define SIXLOWPAN_UDP_8_BIT_PORT_MIN     0xF000
#define SIXLOWPAN_UDP_8_BIT_PORT_MAX     0xf0ff   /* F000 + 255 */

/* 6lowpan dispatches */

#define SIXLOWPAN_DISPATCH_IPV6          0x41 /* 01000001 = 65 */
#define SIXLOWPAN_DISPATCH_HC1           0x42 /* 01000010 = 66 */
#define SIXLOWPAN_DISPATCH_IPHC          0x60 /* 011xxxxx = ... */
#define SIXLOWPAN_DISPATCH_FRAG1         0xc0 /* 11000xxx */
#define SIXLOWPAN_DISPATCH_FRAGN         0xe0 /* 11100xxx */

/* HC1 encoding */

#define SIXLOWPAN_HC1_NH_UDP             0x02
#define SIXLOWPAN_HC1_NH_TCP             0x06
#define SIXLOWPAN_HC1_NH_ICMP6           0x04

/* HC_UDP encoding (works together with HC1) */

#define SIXLOWPAN_HC_UDP_ALL_C           0xe0

/* IPHC encoding
 *
 * Values of fields within the IPHC encoding first byte (C stands for
 * compressed and I for inline)
 */

#define SIXLOWPAN_IPHC_FL_C              0x10
#define SIXLOWPAN_IPHC_TC_C              0x08
#define SIXLOWPAN_IPHC_NH_C              0x04
#define SIXLOWPAN_IPHC_TTL_1             0x01
#define SIXLOWPAN_IPHC_TTL_64            0x02
#define SIXLOWPAN_IPHC_TTL_255           0x03
#define SIXLOWPAN_IPHC_TTL_I             0x00


/* Values of fields within the IPHC encoding second byte */

#define SIXLOWPAN_IPHC_CID               0x80

#define SIXLOWPAN_IPHC_SAC               0x40
#define SIXLOWPAN_IPHC_SAM_00            0x00
#define SIXLOWPAN_IPHC_SAM_01            0x10
#define SIXLOWPAN_IPHC_SAM_10            0x20
#define SIXLOWPAN_IPHC_SAM_11            0x30

#define SIXLOWPAN_IPHC_SAM_BIT           4

#define SIXLOWPAN_IPHC_M                 0x08
#define SIXLOWPAN_IPHC_DAC               0x04
#define SIXLOWPAN_IPHC_DAM_00            0x00
#define SIXLOWPAN_IPHC_DAM_01            0x01
#define SIXLOWPAN_IPHC_DAM_10            0x02
#define SIXLOWPAN_IPHC_DAM_11            0x03

#define SIXLOWPAN_IPHC_DAM_BIT           0

/* Link local context number */

#define SIXLOWPAN_IPHC_ADDR_CONTEXT_LL   0

/* 16-bit multicast addresses compression */

#define SIXLOWPAN_IPHC_MCAST_RANGE       0xa0

/* NHC_EXT_HDR */

#define SIXLOWPAN_NHC_MASK               0xf0
#define SIXLOWPAN_NHC_EXT_HDR            0xe0

/* LOWPAN_UDP encoding (works together with IPHC) */

#define SIXLOWPAN_NHC_UDP_MASK           0xf8
#define SIXLOWPAN_NHC_UDP_ID             0xf0
#define SIXLOWPAN_NHC_UDP_CHECKSUMC      0x04
#define SIXLOWPAN_NHC_UDP_CHECKSUMI      0x00

/* Values for port compression, _with checksum_ ie bit 5 set to 0 */

#define SIXLOWPAN_NHC_UDP_CS_P_00        0xf0 /* All inline */
#define SIXLOWPAN_NHC_UDP_CS_P_01        0xf1 /* Source 16bit inline, dest = 0xf0 + 8 bit inline */
#define SIXLOWPAN_NHC_UDP_CS_P_10        0xf2 /* Source = 0xf0 + 8bit inline, dest = 16 bit inline */
#define SIXLOWPAN_NHC_UDP_CS_P_11        0xf3 /* Source & dest = 0xf0b + 4bit inline */

/* The 6lowpan "headers" length */

#define SIXLOWPAN_IPV6_HDR_LEN           1    /* One byte */
#define SIXLOWPAN_HC1_HDR_LEN            3
#define SIXLOWPAN_HC1_HC_UDP_HDR_LEN     7
#define SIXLOWPAN_FRAG1_HDR_LEN          4
#define SIXLOWPAN_FRAGN_HDR_LEN          5

/* Address compressibility test macros */

/* Check whether we can compress the IID in address 'a' to 16 bits.  This is
 * used for unicast addresses only, and is true if the address is on the
 * format <PREFIX>::0000:00ff:fe00:XXXX
 *
 * NOTE: we currently assume 64-bits prefixes
 */

#define SIXLOWPAN_IS_IID_16BIT_COMPRESSABLE(a) \
  ((((a)->u16[4]) == 0) && \
   // (((a)->u8[10]) == 0)&& \
   (((a)->u8[11]) == 0xff)&& \
   (((a)->u8[12]) == 0xfe)&& \
   (((a)->u8[13]) == 0))

/* Check whether the 9-bit group-id of the compressed multicast address is
 * known. It is true if the 9-bit group is the all nodes or all routers
 * group.  Parameter 'a' is typed uint8_t *
 */

#define SIXLOWPAN_IS_MCASTADDR_DECOMPRESSABLE(a) \
   (((*a & 0x01) == 0) && \
    ((*(a + 1) == 0x01) || (*(a + 1) == 0x02)))

/* Check whether the 112-bit group-id of the multicast address is mappable
 * to a 9-bit group-id. It is true if the group is the all nodes or all
 * routers group.
 */

#define SIXLOWPAN_IS_MCASTADDR_COMPRESSABLE(a) \
  ((((a)->u16[1]) == 0) && \
   (((a)->u16[2]) == 0) && \
   (((a)->u16[3]) == 0) && \
   (((a)->u16[4]) == 0) && \
   (((a)->u16[5]) == 0) && \
   (((a)->u16[6]) == 0) && \
   (((a)->u8[14]) == 0) && \
   ((((a)->u8[15]) == 1) || (((a)->u8[15]) == 2)))

/* FFXX::00XX:XXXX:XXXX */

#define SIXLOWPAN_IS_MCASTADDR_COMPRESSABLE48(a) \
  ((((a)->u16[1]) == 0) && \
   (((a)->u16[2]) == 0) && \
   (((a)->u16[3]) == 0) && \
   (((a)->u16[4]) == 0) && \
   (((a)->u8[10]) == 0))

/* FFXX::00XX:XXXX */

#define SIXLOWPAN_IS_MCASTADDR_COMPRESSABLE32(a) \
  ((((a)->u16[1]) == 0) && \
   (((a)->u16[2]) == 0) && \
   (((a)->u16[3]) == 0) && \
   (((a)->u16[4]) == 0) && \
   (((a)->u16[5]) == 0) && \
   (((a)->u8[12]) == 0))

/* FF02::00XX */

#define SIXLOWPAN_IS_MCASTADDR_COMPRESSABLE8(a) \
  ((((a)->u8[1]) == 2) && \
   (((a)->u16[1]) == 0) && \
   (((a)->u16[2]) == 0) && \
   (((a)->u16[3]) == 0) && \
   (((a)->u16[4]) == 0) && \
   (((a)->u16[5]) == 0) && \
   (((a)->u16[6]) == 0) && \
   (((a)->u8[14]) == 0))

/* This maximum size of an IEEE802.15.4 frame.  Certain, non-standard
 * devices may exceed this value, however.
 */

#define SIXLOWPAN_MAC_STDFRAME 127

/* Packet buffer Definitions */

#define PACKETBUF_HDR_SIZE                    48

#define PACKETBUF_ATTR_PACKET_TYPE_DATA       0
#define PACKETBUF_ATTR_PACKET_TYPE_ACK        1
#define PACKETBUF_ATTR_PACKET_TYPE_STREAM     2
#define PACKETBUF_ATTR_PACKET_TYPE_STREAM_END 3
#define PACKETBUF_ATTR_PACKET_TYPE_TIMESTAMP  4

/* Packet buffer attributes (indices into i_pktattr) */

#define PACKETBUF_ATTR_NONE                   0

/* Scope 0 attributes: used only on the local node. */

#define PACKETBUF_ATTR_CHANNEL                1
#define PACKETBUF_ATTR_NETWORK_ID             2
#define PACKETBUF_ATTR_LINK_QUALITY           3
#define PACKETBUF_ATTR_RSSI                   4
#define PACKETBUF_ATTR_TIMESTAMP              5
#define PACKETBUF_ATTR_RADIO_TXPOWER          6
#define PACKETBUF_ATTR_LISTEN_TIME            7
#define PACKETBUF_ATTR_TRANSMIT_TIME          8
#define PACKETBUF_ATTR_MAX_MAC_TRANSMISSIONS  9
#define PACKETBUF_ATTR_MAC_SEQNO              10
#define PACKETBUF_ATTR_MAC_ACK                11

/* Scope 1 attributes: used between two neighbors only. */

#define PACKETBUF_ATTR_RELIABLE               12
#define PACKETBUF_ATTR_PACKET_ID              13
#define PACKETBUF_ATTR_PACKET_TYPE            14
#define PACKETBUF_ATTR_REXMIT                 15
#define PACKETBUF_ATTR_MAX_REXMIT             16
#define PACKETBUF_ATTR_NUM_REXMIT             17
#define PACKETBUF_ATTR_PENDING                18

/* Scope 2 attributes: used between end-to-end nodes. */

#define PACKETBUF_ATTR_HOPS                   11
#define PACKETBUF_ATTR_TTL                    20
#define PACKETBUF_ATTR_EPACKET_ID             21
#define PACKETBUF_ATTR_EPACKET_TYPE           22
#define PACKETBUF_ATTR_ERELIABLE              23

#define PACKETBUF_NUM_ATTRS                   24

  /* Addresses (indices into i_pktaddr) */

#define PACKETBUF_ADDR_SENDER                 0
#define PACKETBUF_ADDR_RECEIVER               1
#define PACKETBUF_ADDR_ESENDER                2
#define PACKETBUF_ADDR_ERECEIVER              3

#define PACKETBUF_NUM_ADDRS                   4

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Rime address representation */

struct rimeaddr_s
{
  uint8_t u8[CONFIG_NET_6LOWPAN_RIMEADDR_SIZE];
};

/* The device structure for IEEE802.15.4 MAC network device differs from the
 * standard Ethernet MAC device structure.  The main reason for this
 * difference is that fragmentation must be supported.
 *
 * The IEEE802.15.4 MAC does not use the d_buf packet buffer directly.
 * Rather, it uses a smaller frame buffer, i_frame.
 *
 *   - The packet fragment data is provided to the i_frame buffer each time
 *     that the IEEE802.15.4 MAC needs to send more data.  The length of
 *     the frame is provided in i_frame.
 *
 *     In this case, the d_buf holds the packet data yet to be sent; d_len
 *     holds the size of entire packet.
 *
 *   - Received frames are provided by IEEE802.15.4 MAC to the network
 *     via i_frame with length i_framelen for reassembly in d_buf;  d_len
 *     will hold the size of the reassembled packet.
 *
 * This is accomplished by "inheriting" the standard 'struct net_driver_s'
 * and appending the frame buffer as well as other metadata needed to
 * manage the fragmentation.  'struct ieee802154_driver_s' is cast
 * compatible with 'struct net_driver_s' when CONFIG_NET_MULTINIC is not
 * defined or when dev->d_lltype == NET_LL_IEEE802154.
 */

struct ieee802154_driver_s
{
  /* This definitiona must appear first in the structure definition to
   * assure cast compatibility.
   */

  struct net_driver_s i_dev;

  /* IEEE802.15.4 MAC-specific definitions follow. */

  /* The i_frame array is used to hold outgoing frame.   When the
   * IEEE802.15.4 device polls for new data, the outgoing frame containing
   * the next fragment is placed in i_frame.
   *
   * The network will handle only a single outgong frame at a time.  The
   * IEEE802.15.4 MAC driver design may be concurrently sending and
   * requesting new framesusing break-off fram buffers.  That frame buffer
   * management must be controlled by the IEEE802.15.4 MAC driver.
   *
   * Driver provied frame buffers should be 16-bit aligned.
   */

  FAR uint8_t *i_frame;

  /* The length of valid data in the i_frame buffer.
   *
   * When the network device driver calls the network input function,
   * i_framelen should be set to zero.  If there is frame to be sent
   * by the network, i_framelen will be set to indicate the size of
   * frame to be sent.  The value zero means that there is no frame
   * to be sent.
   */

  uint16_t i_framelen;

  /* The following fields are device-specific metadata used by the 6loWPAN
   * stack and should not be modified by the IEEE802.15.4 MAC network drvier.
   */

  /* A pointer to the rime buffer.
   *
   * We initialize it to the beginning of the rime buffer, then access
   * different fields by updating the offset ieee->i_rime_hdrlen.
   */

  FAR uint8_t *i_rimeptr;

  /* i_uncomp_hdrlen is the length of the headers before compression (if HC2
   * is used this includes the UDP header in addition to the IP header).
   */

  uint8_t i_uncomp_hdrlen;

  /* i_rime_hdrlen is the total length of (the processed) 6lowpan headers
   * (fragment headers, IPV6 or HC1, HC2, and HC1 and HC2 non compressed
   * fields).
   */

  uint8_t i_rime_hdrlen;

  /* Next available pointer into header */

  uint8_t i_hdrptr;

  /* Packet buffer metadata: Attributes and addresses */

  uint16_t i_pktattrs[PACKETBUF_NUM_ATTRS];
  struct rimeaddr_s i_pktaddrs[PACKETBUF_NUM_ADDRS];
};

/* The structure of a next header compressor.  This compressor is provided
 * by architecture-specific logic outside of the network stack.
 *
 * TODO: needs more parameters when compressing extension headers, etc.
 */

struct sixlowpan_nhcompressor_s
{
  CODE int (*is_compressable)(uint8_t next_header);

  /* Compress next header (TCP/UDP, etc) - ptr points to next header to
   * compress.
   */

  CODE int (*compress)(FAR uint8_t *compressed, FAR uint8_t *uncompressed_len);

  /* Uncompress next header (TCP/UDP, etc) - ptr points to next header to
   * uncompress.
   */

  CODE int (*uncompress)(FAR uint8_t *compressed, FAR uint8_t *lowpanbuf,
                         FAR uint8_t *uncompressed_len);
};

/* RIME sniffer callbacks */

struct sixlowpan_rime_sniffer_s
{
  CODE void (*input)(void);
  CODE void (*output)(int mac_status);
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Function: sixlowpan_set_compressor
 *
 * Description:
 *   Configure to use the architecture-specific compressor.
 *
 * Input parameters:
 *   compressor - A reference to the new compressor to be used.  This may
 *                be a NULL value to disable the compressor.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void sixlowpan_set_compressor(FAR struct sixlowpan_nhcompressor_s *compressor);

/****************************************************************************
 * Function: sixlowpan_set_sniffer
 *
 * Description:
 *   Configure to use an architecture-specific sniffer to enable tracing of
 *   IP.
 *
 * Input parameters:
 *   sniffer - A reference to the new sniffer to be used.  This may
 *             be a NULL value to disable the sniffer.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_NET_6LOWPAN_SNIFFER
void sixlowpan_set_sniffer(FAR struct sixlowpan_rime_sniffer_s *sniffer);
#endif

#endif /* __INCLUDE_NUTTX_NET_SIXLOWOAN_H */
