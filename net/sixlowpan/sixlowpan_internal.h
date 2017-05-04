/****************************************************************************
 * net/sixlowpan/sixlowpan_internal.h
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Parts of this file derive from Contiki:
 *
 *   Copyright (c) 2008, Swedish Institute of Computer Science
 *   All rights reserved.
 *
 *   Additional fixes for AVR contributed by:
 *         Colin O'Flynn coflynn@newae.com
 *         Eric Gnoske egnoske@gmail.com
 *         Blake Leverett bleverett@gmail.com
 *         Mike Vidales mavida404@gmail.com
 *         Kevin Brown kbrown3@uccs.edu
 *         Nate Bohlmann nate@elfwerks.com
 *
 *   Additional fixes for MSP430 contributed by:
 *         Joakim Eriksson
 *         Niclas Finne
 *         Nicolas Tsiftes
 *
 *    All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name of the copyright holders nor the names of
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 ****************************************************************************/

#ifndef _NET_SIXLOWPAN_SIXLOWPAN_INTERNAL_H
#define _NET_SIXLOWPAN_SIXLOWPAN_INTERNAL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>

#include <nuttx/net/tcp.h>
#include <nuttx/net/udp.h>
#include <nuttx/net/icmpv6.h>
#include <nuttx/net/sixlowpan.h>

#ifdef CONFIG_NET_6LOWPAN

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* IEEE 802.15.4  addres macros */
/* Copy a an IEEE 802.15.4 address */

#define sixlowpan_addrcopy(dest,src)  \
  memcpy(dest, src, NET_6LOWPAN_ADDRSIZE)

/* Compare two IEEE 802.15.4 addresses */

#define sixlowpan_addrcmp(addr1,addr2) \
  (memcmp(addr1, addr2, NET_6LOWPAN_ADDRSIZE) == 0)

/* Packet buffer Definitions */

#define PACKETBUF_ATTR_PACKET_TYPE_DATA       0
#define PACKETBUF_ATTR_PACKET_TYPE_ACK        1
#define PACKETBUF_ATTR_PACKET_TYPE_STREAM     2
#define PACKETBUF_ATTR_PACKET_TYPE_STREAM_END 3
#define PACKETBUF_ATTR_PACKET_TYPE_TIMESTAMP  4

/* Packet buffer attributes (indices into g_pktattrs) */

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
#define PACKETBUF_ATTR_MAC_ACK                10

/* Scope 1 attributes: used between two neighbors only. */

#define PACKETBUF_ATTR_RELIABLE               11
#define PACKETBUF_ATTR_PACKET_ID              12
#define PACKETBUF_ATTR_PACKET_TYPE            13
#define PACKETBUF_ATTR_REXMIT                 14
#define PACKETBUF_ATTR_MAX_REXMIT             15
#define PACKETBUF_ATTR_NUM_REXMIT             16
#define PACKETBUF_ATTR_PENDING                17

/* Scope 2 attributes: used between end-to-end nodes. */

#define PACKETBUF_ATTR_HOPS                   18
#define PACKETBUF_ATTR_TTL                    19
#define PACKETBUF_ATTR_EPACKET_ID             20
#define PACKETBUF_ATTR_EPACKET_TYPE           21
#define PACKETBUF_ATTR_ERELIABLE              22

#define PACKETBUF_NUM_ATTRS                   23

/* Addresses (indices into g_pktaddrs) */

#define PACKETBUF_ADDR_SENDER                 0
#define PACKETBUF_ADDR_RECEIVER               1
#define PACKETBUF_ADDR_ESENDER                2
#define PACKETBUF_ADDR_ERECEIVER              3

#define PACKETBUF_NUM_ADDRS                   4

/* General helper macros ****************************************************/

#define GETINT16(ptr,index) \
  ((((uint16_t)((ptr)[index])) << 8) | ((uint16_t)(((ptr)[(index) + 1]))))
#define PUTINT16(ptr,index,value) \
  do \
    { \
      (ptr)[index]     = ((uint16_t)(value) >> 8) & 0xff; \
      (ptr)[index + 1] = (uint16_t)(value) & 0xff; \
    } \
  while(0)

/* Debug ********************************************************************/

#ifdef CONFIG_NET_6LOWPAN_DUMPBUFFER
#  define sixlowpan_dumpbuffer(m,b,s) ninfodumpbuffer(m,b,s)
#else
#  define sixlowpan_dumpbuffer(m,b,s)
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* IPv^ TCP/UDP Definitions *************************************************/
/* IPv6 + TCP header.  Cast compatible based on IPv6 protocol field. */

struct ipv6tcp_hdr_s
{
  struct ipv6_hdr_s     ipv6;
  struct tcp_hdr_s      tcp;
};

/* IPv6 + UDP header */

struct ipv6udp_hdr_s
{
  struct ipv6_hdr_s     ipv6;
  struct udp_hdr_s      udp;
};

/* IPv6 + ICMPv6 header */

struct ipv6icmp_hdr_s
{
  struct ipv6_hdr_s     ipv6;
  struct icmpv6_iphdr_s icmp;
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* The following data values are used to hold intermediate settings while
 * processing IEEE802.15.4 frames.  These globals are shared with incoming
 * and outgoing frame processing and possibly with mutliple IEEE802.15.4 MAC
 * devices.  The network lock provides exclusive use of these globals
 * during that processing
 */

/* g_uncomp_hdrlen is the length of the headers before compression (if HC2
 * is used this includes the UDP header in addition to the IP header).
 */

extern uint8_t g_uncomp_hdrlen;

/* g_frame_hdrlen is the total length of (the processed) 6lowpan headers
 * (fragment headers, IPV6 or HC1, HC2, and HC1 and HC2 non compressed
 * fields).
 */

extern uint8_t g_frame_hdrlen;

/* Packet buffer metadata: Attributes and addresses */

extern uint16_t g_pktattrs[PACKETBUF_NUM_ATTRS];
extern struct sixlowpan_addr_s g_pktaddrs[PACKETBUF_NUM_ADDRS];

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

struct net_driver_s;         /* Forward reference */
struct ieee802154_driver_s;  /* Forward reference */
struct devif_callback_s;     /* Forward reference */
struct ipv6_hdr_s;           /* Forward reference */
struct sixlowpan_addr_s;           /* Forward reference */
struct iob_s;                /* Forward reference */

/****************************************************************************
 * Name: sixlowpan_send
 *
 * Description:
 *   Process an outgoing UDP or TCP packet.  Takes an IP packet and formats
 *   it to be sent on an 802.15.4 network using 6lowpan.  Called from common
 *   UDP/TCP send logic.
 *
 *   The payload data is in the caller 'buf' and is of length 'buflen'.
 *   Compressed headers will be added and if necessary the packet is
 *   fragmented. The resulting packet/fragments are submitted to the MAC
 *   via the network driver i_req_data method.
 *
 * Input Parameters:
 *   dev     - The IEEE802.15.4 MAC network driver interface.
 *   list    - Head of callback list for send interrupt
 *   ipv6hdr - IPv6 plus TCP or UDP headers.
 *   buf     - Data to send
 *   buflen  - Length of data to send
 *   raddr   - The MAC address of the destination
 *   timeout - Send timeout in deciseconds
 *
 * Returned Value:
 *   Ok is returned on success; Othewise a negated errno value is returned.
 *   This function is expected to fail if the driver is not an IEEE802.15.4
 *   MAC network driver.  In that case, the UDP/TCP will fall back to normal
 *   IPv4/IPv6 formatting.
 *
 * Assumptions:
 *   Called with the network locked.
 *
 ****************************************************************************/

int sixlowpan_send(FAR struct net_driver_s *dev,
                   FAR struct devif_callback_s **list,
                   FAR const struct ipv6_hdr_s *ipv6hdr, FAR const void *buf,
                   size_t buflen, FAR const struct sixlowpan_addr_s *raddr,
                   uint16_t timeout);

/****************************************************************************
 * Name: sixlowpan_meta_data
 *
 * Description:
 *   Based on the collected attributes and addresses, construct the MAC meta
 *   data structure that we need to interface with the IEEE802.15.4 MAC.
 *
 * Input Parameters:
 *   dest_panid - PAN ID of the destination.  May be 0xffff if the destination
 *                is not associated.
 *   meta       - Location to return the corresponding meta data.
 *
 * Returned Value:
 *   Ok is returned on success; Othewise a negated errno value is returned.
 *
 * Assumptions:
 *   Called with the network locked.
 *
 ****************************************************************************/

int sixlowpan_meta_data(uint16_t dest_panid,
                        FAR struct ieee802154_frame_meta_s *meta);

/****************************************************************************
 * Name: sixlowpan_frame_hdrlen
 *
 * Description:
 *   This function is before the first frame has been sent in order to
 *   determine what the size of the IEEE802.15.4 header will be.  No frame
 *   buffer is required to make this determination.
 *
 * Input parameters:
 *   ieee - A reference IEEE802.15.4 MAC network device structure.
 *   meta - Meta data that describes the MAC header
 *
 * Returned Value:
 *   The frame header length is returnd on success; otherwise, a negated
 *   errno value is return on failure.
 *
 ****************************************************************************/

int sixlowpan_frame_hdrlen(FAR struct ieee802154_driver_s *ieee,
                           FAR const struct ieee802154_frame_meta_s *meta);

/****************************************************************************
 * Name: sixlowpan_frame_submit
 *
 * Description:
 *   This function is called after eiether (1) the IEEE802.15.4 MAC driver
 *   polls for TX data or (2) after the IEEE802.15.4 MAC driver provides a
 *   new incoming frame and the network responds with an outgoing packet.  It
 *   submits any new outgoing frame to the MAC.
 *
 * Input parameters:
 *   ieee  - A reference IEEE802.15.4 MAC network device structure.
 *   meta  - Meta data that describes the MAC header
 *   frame - The IOB containing the frame to be submitted.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; otherwise, a negated errno value is
 *   return on any failure.
 *
 ****************************************************************************/

int sixlowpan_frame_submit(FAR struct ieee802154_driver_s *ieee,
                           FAR const struct ieee802154_frame_meta_s *meta,
                           FAR struct iob_s *frame);

/****************************************************************************
 * Name: sixlowpan_queue_frames
 *
 * Description:
 *   Process an outgoing UDP or TCP packet.  This function is called from
 *   send interrupt logic when a TX poll is received.  It formates the
 *   list of frames to be sent by the IEEE802.15.4 MAC driver.
 *
 *   The payload data is in the caller 'buf' and is of length 'buflen'.
 *   Compressed headers will be added and if necessary the packet is
 *   fragmented. The resulting packet/fragments are submitted to the MAC
 *   via the network driver i_req_data method.
 *
 * Input Parameters:
 *   ieee    - The IEEE802.15.4 MAC driver instance
 *   ipv6hdr - IPv6 header followed by TCP or UDP header.
 *   buf     - Beginning of the packet packet to send (with IPv6 + protocol
 *             headers)
 *   buflen  - Length of data to send (include IPv6 and protocol headers)
 *   destmac - The IEEE802.15.4 MAC address of the destination
 *
 * Returned Value:
 *   Ok is returned on success; Othewise a negated errno value is returned.
 *   This function is expected to fail if the driver is not an IEEE802.15.4
 *   MAC network driver.  In that case, the UDP/TCP will fall back to normal
 *   IPv4/IPv6 formatting.
 *
 * Assumptions:
 *   Called with the network locked.
 *
 ****************************************************************************/

int sixlowpan_queue_frames(FAR struct ieee802154_driver_s *ieee,
                           FAR const struct ipv6_hdr_s *ipv6hdr,
                           FAR const void *buf,  size_t buflen,
                           FAR const struct sixlowpan_addr_s *destmac);

/****************************************************************************
 * Name: sixlowpan_hc06_initialize
 *
 * Description:
 *   sixlowpan_hc06_initialize() is called during OS initialization at power-up
 *   reset.  It is called from the common sixlowpan_initialize() function.
 *   sixlowpan_hc06_initialize() configures HC06 networking data structures.
 *   It is called prior to platform-specific driver initialization so that
 *   the 6loWPAN networking subsystem is prepared to deal with network
 *   driver initialization actions.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_NET_6LOWPAN_COMPRESSION_HC06
void sixlowpan_hc06_initialize(void);
#endif

/****************************************************************************
 * Name: sixlowpan_compresshdr_hc06
 *
 * Description:
 *   Compress IP/UDP header
 *
 *   This function is called by the 6lowpan code to create a compressed
 *   6lowpan packet in the packetbuf buffer from a full IPv6 packet in the
 *   uip_buf buffer.
 *
 *     HC-06 (draft-ietf-6lowpan-hc, version 6)
 *     http://tools.ietf.org/html/draft-ietf-6lowpan-hc-06
 *
 *   NOTE: sixlowpan_compresshdr_hc06() does not support ISA100_UDP header
 *   compression
 *
 * Input Parameters:
 *   ieee     - A reference to the IEE802.15.4 network device state
 *   ipv6     - The IPv6 header to be compressed
 *   destmac  - L2 destination address, needed to compress the IP
 *              destination field
 *   fptr     - Pointer to frame to be compressed.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_NET_6LOWPAN_COMPRESSION_HC06
void sixlowpan_compresshdr_hc06(FAR struct ieee802154_driver_s *ieee,
                                FAR const struct ipv6_hdr_s *ipv6,
                                FAR const struct sixlowpan_addr_s *destmac,
                                FAR uint8_t *fptr);
#endif

/****************************************************************************
 * Name: sixlowpan_uncompresshdr_hc06
 *
 * Description:
 *   Uncompress HC06 (i.e., IPHC and LOWPAN_UDP) headers and put them in
 *   sixlowpan_buf
 *
 *   This function is called by the input function when the dispatch is HC06.
 *   We process the frame in the IOB buffer, uncompress the header fields,
 *   and copy the result into the driver packet buffer.  At the end of the
 *   decompression, g_frame_hdrlen and g_uncompressed_hdrlen are set to the
 *   appropriate values
 *
 * Input Parmeters:
 *   iplen  - Equal to 0 if the packet is not a fragment (IP length is then
 *            inferred from the L2 length), non 0 if the packet is a first
 *            fragment.
 *   iob    - Pointer to the IOB containing the received frame.
 *   fptr   - Pointer to frame to be compressed.
 *   bptr   - Output goes here.  Normally this is a known offset into d_buf,
 *            may be redirected to a "bitbucket" on the case of FRAGN frames.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_NET_6LOWPAN_COMPRESSION_HC06
void sixlowpan_uncompresshdr_hc06(uint16_t iplen, FAR struct iob_s *iob,
                                  FAR uint8_t *fptr, FAR uint8_t *bptr);
#endif

/****************************************************************************
 * Name: sixlowpan_compresshdr_hc1
 *
 * Description:
 *   Compress IP/UDP header using HC1 and HC_UDP
 *
 *   This function is called by the 6lowpan code to create a compressed
 *   6lowpan packet in the packetbuf buffer from a full IPv6 packet in the
 *   uip_buf buffer.
 *
 * Input Parmeters:
 *   ieee    - A reference to the IEE802.15.4 network device state
 *   ipv6    - The IPv6 header to be compressed
 *   destmac - L2 destination address, needed to compress the IP
 *             destination field
 *   fptr    - Pointer to frame to be compressed.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_NET_6LOWPAN_COMPRESSION_HC1
void sixlowpan_compresshdr_hc1(FAR struct ieee802154_driver_s *ieee,
                               FAR const struct ipv6_hdr_s *ipv6,
                               FAR const struct sixlowpan_addr_s *destmac,
                               FAR uint8_t *fptr);
#endif

/****************************************************************************
 * Name: sixlowpan_uncompresshdr_hc1
 *
 * Description:
 *   Uncompress HC1 (and HC_UDP) headers and put them in sixlowpan_buf
 *
 *   This function is called by the input function when the dispatch is
 *   HC1.  It processes the frame in the IOB buffer, uncompresses the
 *   header fields, and copies the result in the packet buffer.  At the
 *   end of the decompression, g_frame_hdrlen and uncompressed_hdr_len
 *   are set to the appropriate values
 *
 * Input Parameters:
 *   iplen - Equal to 0 if the packet is not a fragment (IP length is then
 *           inferred from the L2 length), non 0 if the packet is a 1st
 *           fragment.
 *   iob   - Pointer to the IOB containing the received frame.
 *   fptr  - Pointer to frame to be uncompressed.
 *   bptr  - Output goes here.  Normally this is a known offset into d_buf,
 *           may be redirected to a "bitbucket" on the case of FRAGN frames.
 *
 * Returned Value:
 *   Zero (OK) is returned on success, on failure a negater errno value is
 *   returned.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_6LOWPAN_COMPRESSION_HC1
int sixlowpan_uncompresshdr_hc1(uint16_t iplen, FAR struct iob_s *iob,
                                FAR uint8_t *fptr, FAR uint8_t *bptr);
#endif

/****************************************************************************
 * Name: sixlowpan_islinklocal, sixlowpan_ipfromaddr, sixlowpan_addrfromip,
 *       and sixlowpan_ismacbased
 *
 * Description:
 *   sixlowpan_ipfromaddr: Create a link local IPv6 address from an IEEE
 *   802.15.4 address.
 *
 *   sixlowpan_addrfromip: Extract the IEEE 802.15.14 address from a link
 *   local IPv6 address.
 *
 *   sixlowpan_islinklocal and sixlowpan_ismacbased will return true for
 *   address created in this fashion.
 *
 *    128  112  96   80    64   48   32   16
 *    ---- ---- ---- ----  ---- ---- ---- ----
 *    fe80 0000 0000 0000  xxxx 0000 0000 0000 2-byte short address (VALID?)
 *    fe80 0000 0000 0000  xxxx xxxx xxxx xxxx 8-byte extended address
 *
 ****************************************************************************/

#define sixlowpan_islinklocal(ipaddr) ((ipaddr)[0] == NTOHS(0xfe80))

void sixlowpan_ipfromaddr(FAR const struct sixlowpan_addr_s *addr,
                          net_ipv6addr_t ipaddr);
void sixlowpan_addrfromip(const net_ipv6addr_t ipaddr,
                          FAR struct sixlowpan_addr_s *addr);
bool sixlowpan_ismacbased(const net_ipv6addr_t ipaddr,
                          FAR const struct sixlowpan_addr_s *addr);

/****************************************************************************
 * Name: sixlowpan_src_panid
 *
 * Description:
 *   Get the source PAN ID from the IEEE802.15.4 radio.
 *
 * Input parameters:
 *   ieee  - A reference IEEE802.15.4 MAC network device structure.
 *   panid - The location in which to return the PAN ID.  0xfff may be
 *           returned if the device is not associated.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int sixlowpan_src_panid(FAR struct ieee802154_driver_s *ieee,
                        FAR uint16_t *panid);
#endif /* CONFIG_NET_6LOWPAN */
#endif /* _NET_SIXLOWPAN_SIXLOWPAN_INTERNAL_H */
