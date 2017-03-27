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

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Min and Max compressible UDP ports - HC06 */

#define SIXLOWPAN_UDP_4_BIT_PORT_MIN     0xf0b0
#define SIXLOWPAN_UDP_4_BIT_PORT_MAX     0xf0bf   /* F0B0 + 15 */
#define SIXLOWPAN_UDP_8_BIT_PORT_MIN     0xF000
#define SIXLOWPAN_UDP_8_BIT_PORT_MAX     0xf0ff   /* F000 + 255 */

/* 6lowpan compressions */

#define SIXLOWPAN_COMPRESSION_IPV6       0
#define SIXLOWPAN_COMPRESSION_HC1        1
#define SIXLOWPAN_COMPRESSION_HC06       2

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

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* The header for fragments
 *
 * NOTE: We do not define different structures for FRAG1 and FRAGN headers,
 * which are different. For FRAG1, the offset field is just not used
 */

struct sixlowpan_frag_hdr
{
  uint16_t dispatch_size;
  uint16_t tag;
  uint8_t offset;
};

/* The HC1 header when HC_UDP is not used
 *
 * When all fields are compressed and HC_UDP is not used, we use this
 * structure. If HC_UDP is used, the ttl is in another spot, and we use the
 * sixlowpan_hc1_hc_udp structure
 */

struct sixlowpan_hc1_hdr
{
  uint8_t dispatch;
  uint8_t encoding;
  uint8_t ttl;
};

/* HC1 followed by HC_UDP */

struct sixlowpan_hc1_hc_udp_hdr
{
  uint8_t dispatch;
  uint8_t hc1_encoding;
  uint8_t hc_udp_encoding;
  uint8_t ttl;
  uint8_t ports;
  uint16_t udpchksum;
};

/* An address context for IPHC address compression each context can have up
 * to 8 bytes
 */

struct sixlowpan_addr_context
{
  uint8_t used;       /* Possibly use as prefix-length */
  uint8_t number;
  uint8_t prefix[8];
};


/* The structure of a next header compressor.
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

#endif /* __INCLUDE_NUTTX_NET_SIXLOWOAN_H */
