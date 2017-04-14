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

/* Rime addres macros */
/* Copy a Rime address */

#define rimeaddr_copy(dest,src)  \
  memcpy(dest, src, NET_6LOWPAN_RIMEADDR_SIZE)

/* Compare two Rime addresses */

#define rimeaddr_cmp(addr1,addr2) \
  (memcmp(addr1, addr2, NET_6LOWPAN_RIMEADDR_SIZE) == 0)

/* Pointers in the Rime buffer */

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

/* IEEE802.15.4 Frame Definitions *******************************************/
/* The IEEE 802.15.4 frame has a number of constant/fixed fields that can be
 * counted to make frame construction and max payload calculations easier.
 * These include:
 *
 *   1. FCF                  - 2 bytes       - Fixed
 *   2. Sequence number      - 1 byte        - Fixed
 *   3. Addressing fields    - 4 - 20 bytes  - Variable
 *   4. Aux security header  - 0 - 14 bytes  - Variable
 *   5. CRC                  - 2 bytes       - Fixed
*/

/* Defines the bitfields of the frame control field (FCF). */

struct frame802154_fcf_s
{
  uint8_t frame_type;        /* 3 bit. Frame type field, see 802.15.4 */
  uint8_t security_enabled;  /* 1 bit. True if security is used in this frame */
  uint8_t frame_pending;     /* 1 bit. True if sender has more data to send */
  uint8_t ack_required;      /* 1 bit. Is an ack frame required? */
  uint8_t panid_compression; /* 1 bit. Is this a compressed header? */
                             /* 3 bit. Unused bits */
  uint8_t dest_addr_mode;    /* 2 bit. Destination address mode, see 802.15.4 */
  uint8_t frame_version;     /* 2 bit. 802.15.4 frame version */
  uint8_t src_addr_mode;     /* 2 bit. Source address mode, see 802.15.4 */
};

/* 802.15.4 security control bitfield.  See section 7.6.2.2.1 in 802.15.4
 * specification.
 */

struct frame802154_scf_s
{
  uint8_t  security_level;   /* 3 bit. security level      */
  uint8_t  key_id_mode;      /* 2 bit. Key identifier mode */
  uint8_t  reserved;         /* 3 bit. Reserved bits       */
};

/* 802.15.4 Aux security header */

struct frame802154_aux_hdr_s
{
  struct frame802154_scf_s security_control;  /* Security control bitfield */
  uint32_t frame_counter;    /* Frame counter, used for security */
  uint8_t  key[9];           /* The key itself, or an index to the key */
};

/* Parameters used by the frame802154_create() function.  These  parameters
 * are used in the 802.15.4 frame header.  See the 802.15.4 specification
 * for details.
 */

struct frame802154_s
{
  struct frame802154_fcf_s fcf;          /* Frame control field  */
  uint8_t seq;               /* Sequence number */
  uint16_t dest_pid;         /* Destination PAN ID */
  uint8_t dest_addr[8];      /* Destination address */
  uint16_t src_pid;          /* Source PAN ID */
  uint8_t src_addr[8];       /* Source address */
  struct frame802154_aux_hdr_s aux_hdr;  /* Aux security header */
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

/* A pointer to the rime buffer.
 *
 * We initialize it to the beginning of the rime buffer, then access
 * different fields by updating the offset ieee->g_frame_hdrlen.
 */

extern FAR uint8_t *g_rimeptr;

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
extern struct rimeaddr_s g_pktaddrs[PACKETBUF_NUM_ADDRS];

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
struct rimeaddr_s;           /* Forward reference */
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
 *   fragmented. The resulting packet/fragments are put in ieee->i_framelist
 *   and the entire list of frames will be delivered to the 802.15.4 MAC via
 *   ieee->i_framelist.
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
                   size_t buflen, FAR const struct rimeaddr_s *raddr,
                   uint16_t timeout);

/****************************************************************************
 * Function: sixlowpan_send_hdrlen
 *
 * Description:
 *   This function is before the first frame has been sent in order to
 *   determine what the size of the IEEE802.15.4 header will be.  No frame
 *   buffer is required to make this determination.
 *
 * Input parameters:
 *   ieee       - A reference IEEE802.15.4 MAC network device structure.
 *   dest_panid - PAN ID of the destination.  May be 0xffff if the destination
 *                is not associated.
 *
 * Returned Value:
 *   The frame header length is returnd on success; otherwise, a negated
 *   errno value is return on failure.
 *
 ****************************************************************************/

int sixlowpan_send_hdrlen(FAR struct ieee802154_driver_s *ieee,
                     uint16_t dest_panid);

/****************************************************************************
 * Function: sixlowpan_framecreate
 *
 * Description:
 *   This function is called after the IEEE802.15.4 MAC driver polls for
 *   TX data.  It creates the IEEE802.15.4 header in the frame buffer.
 *
 * Input parameters:
 *   ieee       - A reference IEEE802.15.4 MAC network device structure.
 *   iob        - The IOB in which to create the frame.
 *   dest_panid - PAN ID of the destination.  May be 0xffff if the destination
 *                is not associated.
 *
 * Returned Value:
 *   The frame header length is returnd on success; otherwise, a negated
 *   errno value is return on failure.
 *
 ****************************************************************************/

int sixlowpan_framecreate(FAR struct ieee802154_driver_s *ieee,
                          FAR struct iob_s *iob, uint16_t dest_panid);

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
 *   fragmented. The resulting packet/fragments are put in ieee->i_framelist
 *   and the entire list of frames will be delivered to the 802.15.4 MAC via
 *   ieee->i_framelist.
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
                           FAR const struct rimeaddr_s *destmac);

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
                                FAR const struct rimeaddr_s *destmac,
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
 *   We process the packet in the rime buffer, uncompress the header fields,
 *   and copy the result in the sixlowpan buffer.  At the end of the
 *   decompression, g_frame_hdrlen and g_uncompressed_hdrlen are set to the
 *   appropriate values
 *
 * Input Parmeters:
 *   ieee   - A reference to the IEE802.15.4 network device state
 *   iplen  - Equal to 0 if the packet is not a fragment (IP length is then
 *            inferred from the L2 length), non 0 if the packet is a first
 *            fragment.
 *   iob    - Pointer to the IOB containing the received frame.
 *   fptr   - Pointer to frame to be uncompressed.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_NET_6LOWPAN_COMPRESSION_HC06
void sixlowpan_uncompresshdr_hc06(FAR struct ieee802154_driver_s *ieee,
                                  uint16_t iplen, FAR struct iob_s *iob,
                                  FAR uint8_t *fptr);
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
 *   fptr     - Pointer to frame to be compressed.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_NET_6LOWPAN_COMPRESSION_HC1
void sixlowpan_compresshdr_hc1(FAR struct ieee802154_driver_s *ieee,
                               FAR const struct ipv6_hdr_s *ipv6,
                               FAR const struct rimeaddr_s *destmac,
                               FAR uint8_t *fptr);
#endif

/****************************************************************************
 * Name: sixlowpan_uncompresshdr_hc1
 *
 * Description:
 *   Uncompress HC1 (and HC_UDP) headers and put them in sixlowpan_buf
 *
 *   This function is called by the input function when the dispatch is
 *   HC1.  It processes the packet in the rime buffer, uncompresses the
 *   header fields, and copies the result in the sixlowpan buffer.  At the
 *   end of the decompression, g_frame_hdrlen and uncompressed_hdr_len
 *   are set to the appropriate values
 *
 * Input Parameters:
 *   ieee  - A reference to the IEE802.15.4 network device state
 *   iplen - Equal to 0 if the packet is not a fragment (IP length is then
 *           inferred from the L2 length), non 0 if the packet is a first
 *           fragment.
 *   iob    - Pointer to the IOB containing the received frame.
 *   fptr   - Pointer to frame to be uncompressed.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_NET_6LOWPAN_COMPRESSION_HC1
int sixlowpan_uncompresshdr_hc1(FAR struct ieee802154_driver_s *ieee,
                                uint16_t ip_len, FAR struct iob_s *iob,
                                FAR uint8_t *fptr);
#endif

/****************************************************************************
 * Name: sixlowpan_islinklocal, sixlowpan_ipfromrime, sixlowpan_rimefromip,
 *       and sixlowpan_ismacbased
 *
 * Description:
 *   sixlowpan_ipfromrime: Create a link local IPv6 address from a rime
 *   address.
 *
 *   sixlowpan_rimefromip: Extract the rime address from a link local IPv6
 *   address.
 *
 *   sixlowpan_islinklocal and sixlowpan_ismacbased will return true for
 *   address created in this fashion.
 *
 *    128  112  96   80    64   48   32   16
 *    ---- ---- ---- ----  ---- ---- ---- ----
 *    fe80 0000 0000 0000  xxxx 0000 0000 0000 2-byte Rime address (VALID?)
 *    fe80 0000 0000 0000  xxxx xxxx xxxx xxxx 8-byte Rime address
 *
 ****************************************************************************/

#define sixlowpan_islinklocal(ipaddr) ((ipaddr)[0] == NTOHS(0xfe80))

void sixlowpan_ipfromrime(FAR const struct rimeaddr_s *rime,
                          net_ipv6addr_t ipaddr);
void sixlowpan_rimefromip(const net_ipv6addr_t ipaddr,
                          FAR struct rimeaddr_s *rime);
bool sixlowpan_ismacbased(const net_ipv6addr_t ipaddr,
                          FAR const struct rimeaddr_s *rime);

#endif /* CONFIG_NET_6LOWPAN */
#endif /* _NET_SIXLOWPAN_SIXLOWPAN_INTERNAL_H */
