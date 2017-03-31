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

#include <nuttx/net/iob.h>
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

/* Frame buffer helper macros.
 *
 * The IEEE802.15.4 MAC driver structures includes a list of IOB
 * structures, i_framelist, containing frames to be sent by the driver or
 * that were received by the driver.  The IOB structure is defined in
 * include/nuttx/net/iob.h.  The length of data in the IOB is provided by
 * the io_len field of the IOB structure.
 *
 * NOTE that IOBs must be configured such that CONFIG_IOB_BUFSIZE >=
 * CONFIG_NET_6LOWPAN_FRAMELEN
 *
 * 1. On a TX poll, the IEEE802.15.4 MAC driver should provide its driver
 *    structure with i_framelist set to NULL.  At the conclusion of the
 *    poll, if there are frames to be sent, they will have been added to
 *    the i_framelist.  The non-empty frame list is the indication that
 *    there is data to be sent.
 *
 *    The IEEE802.15.4 may use the FRAME_IOB_EMPTY() macro to determine
 *    if there there frames to be sent.  If so, it should remove each
 *    frame from the frame list using the FRAME_IOB_REMOVE() macro and send
 *    it.  That macro will return NULL when all of the frames have been
 *    sent.
 *
 *    After sending each frame, the driver must return the IOB to the pool
 *    of free IOBs using the FROM_IOB_FREE() macro.
 */


#define FRAME_IOB_EMPTY(ieee)  ((ieee)->framelist == NULL)
#define FRAME_IOB_REMOVE(ieee, iob) \
  do \
    { \
      (iob)               = (ieee)->i_framelist; \
      (ieee)->i_framelist = (iob)->io_flink; \
      (iob)->io_flink     = NULL; \
    } \
  while (0)
#define FRAME_IOB_FREE(iob)    iob_free(iob)

/* 2. When receiving data, the IEEE802.15.4 MAC driver should receive the
 *    frame data directly into the payload area of an IOB structure.  That
 *    IOB structure may be obtained using the FRAME_IOB_ALLOC() macro.  The
 *    single frame should be added to the frame list using FRAME_IOB_ADD()
 *    (it will be a list of length one) .  The MAC driver should then inform
 *    the network of the by calling sixlowpan_input().
 */

#define FRAME_IOB_ALLOC()      iob_alloc(false)
#define FRAME_IOB_ADD(ieee, iob) \
  do \
    { \
      (iob)->io_flink     = (ieee)->i_framelist; \
      (ieee)->i_framelist = (iob); \
    } \
  while (0)

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
 * Rather, it uses a list smaller frame buffers, i_framelist.
 *
 *   - The packet fragment data is provided to an IOB in the i_framelist
 *     buffer each time that the IEEE802.15.4 MAC needs to send more data.
 *     The length of the frame is provided in the io_len field of the IOB.
 *
 *     In this case, the d_buf is not used at all and, if fact, may be
 *     NULL.
 *
 *   - Received frames are provided by IEEE802.15.4 MAC to the network
 *     via and IOB in i_framelist with length io_len for reassembly in
 *     d_buf;  d_len will hold the size of the reassembled packet.
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
 * 1. i_panid must be set to identify the network.  It may be set to 0xfff
 *    if the device is not associated.
 *
 * 2. i_dsn must be set to a random value.  After that, it will be managed
 *    by the network.
 *
 * 3. i_nodeaddr must be set after the MAC is assigned an address.
 *
 * 4. On a TX poll, the IEEE802.15.4 MAC driver should provide its driver
 *    structure with i_framelist set to NULL.  At the conclusion of the
 *    poll, if there are frames to be sent, they will have been added to
 *    the i_framelist.  The non-empty frame list at the conclusion of the
 *    TX poll is the indication that is data to be sent.
 *
 *    The IEEE802.15.4 may use the FRAME_IOB_EMPTY() macro to determine
 *    if there there frames to be sent.  If so, it should remove each
 *    frame from the frame list using the FRAME_IOB_REMOVE() macro and send
 *    it.  That macro will return NULL when all of the frames have been
 *    sent.
 *
 *    After sending each frame, the driver must return the IOB to the pool
 *    of free IOBs using the FROM_IOB_FREE() macro.
 *
 * 5. When receiving data both buffers must be provided:
 *
 *    The IEEE802.15.4 MAC driver should receive the frame data directly
 *    into the payload area of an IOB structure.  That IOB structure may be
 *    obtained using the FRAME_IOB_ALLOC() macro.  The single frame should
 *    be added to the frame list using FRAME_IOB_ADD() (it will be a list of
 *    length one).
 *
 *    The larger dev.d_buf must have a size of at least the advertised MTU
 *    of the protocol, CONFIG_NET_6LOWPAN_MTU.  If fragmentation is enabled,
 *    then the logical packet size may be significantly larger than the
 *    size of the frame buffer.  The dev.d_buf is used for de-compressing
 *    each frame and reassembling any fragmented packets to create the full
 *    input packet that is provided to the application.
 *
 *    The MAC driver should then inform the network of the by calling
 *    sixlowpan_input().
 */

struct ieee802154_driver_s
{
  /* This definitiona must appear first in the structure definition to
   * assure cast compatibility.
   */

  struct net_driver_s i_dev;

  /* IEEE802.15.4 MAC-specific definitions follow. */

  /* The i_framelist is used to hold a outgoing frames contained in IOB
   * structures.   When the IEEE802.15.4 device polls for new TX data, the
   * outgoing frame(s) containing the packet fragments are placed in IOBs
   * and queued in i_framelist.
   *
   * The i_framelist is similary used to hold incoming frames in IOB
   * structures.  The IEEE802.15.4 MAC driver must receive frames in an IOB,
   * place the IOB in the i_framelist, and call sixlowpan_input().
   *
   * The IEEE802.15.4 MAC driver design may be concurrently sending and
   * requesting new frames using lists of IOBs.  That IOB frame buffer
   * management must be managed by the IEEE802.15.4 MAC driver.
   */

  FAR struct iob_s *i_framelist;

  /* i_panid.  The PAN ID is 16-bit number that identifies the network. It
   * must be unique to differentiate a network. All the nodes in the same
   * network should have the same PAN ID.  This value must be provided to
   * the network from the IEEE802.15.4 MAC driver.
   *
   * If this value is 0xffff, the device is not associated.
   */

  uint16_t i_panid;

  /* i_node_addr.  The address assigned to this node. */

  struct rimeaddr_s i_nodeaddr;

  /* i_dsn.  The sequence number in the range 0x00-0xff added to the
   * transmitted data or MAC command frame. The default is a random value
   * within that range.
   *
   * This field must be initialized to a random number by the IEEE802.15.4
   * MAC driver.  It sill be subsequently incremented on each frame by the
   * network logic.
   */

  uint8_t i_dsn;

  /* i_dgramtag.  Datagram tag to be put in the header of the set of
   * fragments.  It is used by the recipient to match fragments of the
   * same payload.
   */

  uint16_t i_dgramtag;
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
