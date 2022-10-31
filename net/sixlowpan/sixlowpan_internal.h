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
 * modification, are permitted provided that the following conditions are
 * met:
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
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
 * IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __NET_SIXLOWPAN_SIXLOWPAN_INTERNAL_H
#define __NET_SIXLOWPAN_SIXLOWPAN_INTERNAL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdbool.h>

#include <nuttx/net/tcp.h>
#include <nuttx/net/udp.h>
#include <nuttx/net/icmpv6.h>
#include <nuttx/net/sixlowpan.h>
#include <nuttx/wireless/pktradio.h>

#ifdef CONFIG_NET_6LOWPAN

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Copy a generic address */

#define sixlowpan_anyaddrcopy(dest,src,len) \
  memcpy(dest, src, len)

#ifdef CONFIG_WIRELESS_IEEE802154
/* IEEE 802.15.4  address macros
 *
 * Copy a an IEEE 802.15.4 address.
 */

#define sixlowpan_saddrcopy(dest,src) \
  sixlowpan_anyaddrcopy(dest,src,NET_6LOWPAN_SADDRSIZE)

#define sixlowpan_eaddrcopy(dest,src)  \
  sixlowpan_anyaddrcopy(dest,src,NET_6LOWPAN_EADDRSIZE)

#define sixlowpan_addrcopy(dest,src)  \
  sixlowpan_anyaddrcopy(dest,src,NET_6LOWPAN_ADDRSIZE)

#endif

/* General helper macros ****************************************************/

/* GET 16-bit data:  source in network order */

#define GETUINT16(ptr,index) \
  ((uint16_t)((((uint16_t)((ptr)[index])) << 8) | ((uint16_t)(((ptr)[(index) + 1])))))

/* PUT 16-bit data:  source in host order, result in network order */

#define PUTHOST16(ptr,index,value) \
  do \
    { \
      (ptr)[index]     = ((uint16_t)(value) >> 8) & 0xff; \
      (ptr)[index + 1] = (uint16_t)(value) & 0xff; \
    } \
  while (0)

/* Return values ************************************************************/

/* Successful return values from header compression logic */

#define COMPRESS_HDR_INLINE     0 /* L2 header not compressed */
#define COMPRESS_HDR_ELIDED     1 /* L2 header compressed */

/* Memory Pools *************************************************************/

#define REASS_POOL_PREALLOCATED 0
#define REASS_POOL_DYNAMIC      1
#define REASS_POOL_RADIO        2

/* Debug ********************************************************************/

#ifdef CONFIG_NET_6LOWPAN_DUMPBUFFER
#  define sixlowpan_dumpbuffer(m,b,s) ninfodumpbuffer(m,b,s)
#else
#  define sixlowpan_dumpbuffer(m,b,s)
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* IPv6 TCP/UDP/ICMPv6 Definitions ******************************************/

#ifdef CONFIG_NET_TCP
/* IPv6 + TCP header.  Cast compatible based on IPv6 protocol field. */

struct ipv6tcp_hdr_s
{
  struct ipv6_hdr_s     ipv6;
  struct tcp_hdr_s      tcp;
};
#endif

#ifdef CONFIG_NET_UDP
/* IPv6 + UDP header */

struct ipv6udp_hdr_s
{
  struct ipv6_hdr_s     ipv6;
  struct udp_hdr_s      udp;
};
#endif

#ifdef CONFIG_NET_ICMPv6
/* IPv6 + ICMPv6 header */

struct ipv6icmp_hdr_s
{
  struct ipv6_hdr_s     ipv6;
  struct icmpv6_iphdr_s icmp;
};
#endif

#ifdef CONFIG_WIRELESS_IEEE802154
/* In order to provide a customizable IEEE 802.15.4 MAC header, a structure
 * of meta data is passed to the MAC network driver, struct
 * ieee802154_frame_meta_s.  Many of the settings in this meta data are
 * fixed, determined by the 6LoWPAN configuration.  Other settings depend
 * on the protocol used in the current packet or on chacteristics of the
 * destination node.
 *
 * The following structure is used to summarize those per-packet
 * customizations and, along, with the fixed configuration settings,
 * determines the full form of that meta data.
 */

struct ieee802_txmetadata_s
{
  uint8_t sextended : 1;                 /* Extended source address */
  uint8_t dextended : 1;                 /* Extended destination address */
  uint8_t xmits;                         /* Max MAC transmisstion */
  uint8_t dpanid[IEEE802154_PANIDSIZE];  /* Destination PAN ID */
  struct netdev_maxaddr_s source;        /* Source IEEE 802.15.4 address */
  struct netdev_maxaddr_s dest;          /* Destination IEEE 802.15.4 address */
};
#endif

/* This structure holds the packet metadata as a union when multiple
 * different radio types are supported.
 */

union sixlowpan_metadata_u
{
#ifdef CONFIG_WIRELESS_IEEE802154
  struct ieee802154_frame_meta_s ieee802154;
#endif
#ifdef CONFIG_WIRELESS_PKTRADIO
  struct pktradio_metadata_s pktradio;
#endif
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* The following data values are used to hold intermediate settings while
 * processing IEEE802.15.4 frames.  These globals are shared with incoming
 * and outgoing frame processing and possibly with multiple IEEE802.15.4 MAC
 * devices.  The network lock provides exclusive use of these globals
 * during that processing.
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

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

struct net_driver_s;        /* Forward reference */
struct radio_driver_s;      /* Forward reference */
struct devif_callback_s;    /* Forward reference */
struct ipv6_hdr_s;          /* Forward reference */
struct netdev_varaddr_s;    /* Forward reference */
struct iob_s;               /* Forward reference */

/****************************************************************************
 * Name: sixlowpan_send
 *
 * Description:
 *   Process an outgoing UDP or ICMPv6 packet.  Takes an IP packet and
 *   formats it to be sent on an 802.15.4 network using 6lowpan.  Called
 *   from common UDP/ICMPv6 send logic.
 *
 *   The payload data is in the caller 'buf' and is of length 'buflen'.
 *   Compressed headers will be added and if necessary the packet is
 *   fragmented. The resulting packet/fragments are submitted to the MAC
 *   via the network driver r_req_data method.
 *
 * Input Parameters:
 *   dev     - The IEEE802.15.4 MAC network driver interface.
 *   list    - Head of callback list for send event handler
 *   ipv6hdr - IPv6 header followed by UDP or ICMPv6 header.
 *   buf     - Data to send
 *   len     - Length of data to send
 *   destmac - The IEEE802.15.4 MAC address of the destination
 *   timeout - Send timeout in milliseconds
 *
 * Returned Value:
 *   Ok is returned on success; Otherwise a negated errno value is returned.
 *   This function is expected to fail if the driver is not an IEEE802.15.4
 *   MAC network driver.  In that case, the logic will fall back to normal
 *   IPv4/IPv6 formatting.
 *
 * Assumptions:
 *   Called with the network locked.
 *
 ****************************************************************************/

int sixlowpan_send(FAR struct net_driver_s *dev,
                   FAR struct devif_callback_s **list,
                   FAR struct devif_callback_s **list_tail,
                   FAR const struct ipv6_hdr_s *ipv6hdr, FAR const void *buf,
                   size_t len, FAR const struct netdev_varaddr_s *destmac,
                   unsigned int timeout);

/****************************************************************************
 * Name: sixlowpan_meta_data
 *
 * Description:
 *   Based on the collected attributes and addresses, construct the MAC meta
 *   data structure that we need to interface with the IEEE802.15.4 MAC.
 *
 * Input Parameters:
 *   radio   - Reference to a radio network driver state instance.
 *   pktmeta - Meta-data specific to the current outgoing frame
 *   meta    - Location to return the corresponding meta data reference
 *             (obfuscated).
 *
 * Returned Value:
 *   Ok is returned on success; Otherwise a negated errno value is returned.
 *
 * Assumptions:
 *   Called with the network locked.
 *
 ****************************************************************************/

#ifdef CONFIG_WIRELESS_IEEE802154
int sixlowpan_meta_data(FAR struct radio_driver_s *radio,
                        FAR const struct ieee802_txmetadata_s *pktmeta,
                        FAR struct ieee802154_frame_meta_s *meta);
#endif

/****************************************************************************
 * Name: sixlowpan_frame_hdrlen
 *
 * Description:
 *   This function is before the first frame has been sent in order to
 *   determine what the size of the IEEE802.15.4 header will be.  No frame
 *   buffer is required to make this determination.
 *
 * Input Parameters:
 *   radio - Reference to a radio network driver state instance.
 *   meta  - obfuscated meta data that describes the MAC header
 *
 * Returned Value:
 *   The frame header length is returned on success; otherwise, a negated
 *   errno value is return on failure.
 *
 ****************************************************************************/

int sixlowpan_frame_hdrlen(FAR struct radio_driver_s *radio,
                           FAR const void *meta);

/****************************************************************************
 * Name: sixlowpan_frame_submit
 *
 * Description:
 *   This function is called after eiether (1) the IEEE802.15.4 MAC driver
 *   polls for TX data or (2) after the IEEE802.15.4 MAC driver provides a
 *   new incoming frame and the network responds with an outgoing packet.  It
 *   submits any new outgoing frame to the MAC.
 *
 * Input Parameters:
 *   radio - Reference to a radio network driver state instance.
 *   meta  - Obfuscated metadata that describes the MAC header
 *   frame - The IOB containing the frame to be submitted.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; otherwise, a negated errno value is
 *   return on any failure.
 *
 ****************************************************************************/

int sixlowpan_frame_submit(FAR struct radio_driver_s *radio,
                           FAR const void *meta, FAR struct iob_s *frame);

/****************************************************************************
 * Name: sixlowpan_queue_frames
 *
 * Description:
 *   Process an outgoing UDP or TCP packet.  This function is called from
 *   the send event handler when a TX poll is received.  It formats the
 *   list of frames to be sent by the IEEE802.15.4 MAC driver.
 *
 *   The payload data is in the caller 'buf' and is of length 'buflen'.
 *   Compressed headers will be added and if necessary the packet is
 *   fragmented. The resulting packet/fragments are submitted to the MAC
 *   via the network driver r_req_data method.
 *
 * Input Parameters:
 *   radio   - Reference to a radio network driver state instance.
 *   ipv6    - IPv6 header followed by TCP or UDP header.
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
                           FAR const void *buf,  size_t buflen,
                           FAR const struct netdev_varaddr_s *destmac);

/****************************************************************************
 * Name: sixlowpan_hc06_initialize
 *
 * Description:
 *   sixlowpan_hc06_initialize() is called during OS initialization at
 *   power-up reset.  It is called from the common sixlowpan_initialize()
 *   function.  sixlowpan_hc06_initialize() configures HC06 networking data
 *   structures.  It is called prior to platform-specific driver
 *   initialization so that the 6LoWPAN networking subsystem is prepared to
 *   deal with network driver initialization actions.
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
 *     HC-06:
 *
 *     Originally draft-ietf-6lowpan-hc, version 6:
 *     http://tools.ietf.org/html/draft-ietf-6lowpan-hc-06,
 *
 *   Updated to:
 *
 *     RFC 6282:
 *     https://tools.ietf.org/html/rfc6282
 *
 *   NOTE: sixlowpan_compresshdr_hc06() does not support ISA100_UDP header
 *   compression
 *
 * Input Parameters:
 *   radio   - Reference to a radio network driver state instance.
 *   ipv6    - The IPv6 header to be compressed
 *   destmac - L2 destination address, needed to compress the IP
 *             destination field
 *   fptr    - Pointer to frame to be compressed.
 *
 * Returned Value:
 *   On success the indications of the defines COMPRESS_HDR_* are returned.
 *   A negated errno value is returned on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_6LOWPAN_COMPRESSION_HC06
int sixlowpan_compresshdr_hc06(FAR struct radio_driver_s *radio,
                               FAR const struct ipv6_hdr_s *ipv6,
                               FAR const struct netdev_varaddr_s *destmac,
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
 * Input Parameters:
 *   radio    - Reference to a radio network driver state instance.
 *   metadata - Obfuscated MAC metadata including node addressing
 *              information.
 *   iplen    - Equal to 0 if the packet is not a fragment (IP length is
 *              then inferred from the L2 length), non 0 if the packet is
 *              a first fragment.
 *   iob      - Pointer to the IOB containing the received frame.
 *   fptr     - Pointer to frame to be compressed.
 *   bptr     - Output goes here.  Normally this is a known offset into
 *              d_buf, may be redirected to a "bitbucket" on the case of
 *              FRAGN frames.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_NET_6LOWPAN_COMPRESSION_HC06
void sixlowpan_uncompresshdr_hc06(FAR struct radio_driver_s *radio,
                                  FAR const void *metadata,
                                  uint16_t iplen, FAR struct iob_s *iob,
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
 * Input Parameters:
 *   radio   - Reference to a radio network driver state instance.
 *   ipv6    - The IPv6 header to be compressed
 *   destmac - L2 destination address, needed to compress the IP
 *             destination field
 *   fptr    - Pointer to frame to be compressed.
 *
 * Returned Value:
 *   On success the indications of the defines COMPRESS_HDR_* are returned.
 *   A negated errno value is returned on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_6LOWPAN_COMPRESSION_HC1
int sixlowpan_compresshdr_hc1(FAR struct radio_driver_s *radio,
                              FAR const struct ipv6_hdr_s *ipv6,
                              FAR const struct netdev_varaddr_s *destmac,
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
 *   radio    - Reference to a radio network driver state instance.
 *   metadata - Obfuscated MAC metadata including node addressing
 *              information.
 *   iplen    - Equal to 0 if the packet is not a fragment (IP length is
 *              then inferred from the L2 length), non 0 if the packet is
 *              a 1st fragment.
 *   iob      - Pointer to the IOB containing the received frame.
 *   fptr     - Pointer to frame to be uncompressed.
 *   bptr     - Output goes here.  Normally this is a known offset into
 *              d_buf, may be redirected to a "bitbucket" on the case of
 *              FRAGN frames.
 *
 * Returned Value:
 *   Zero (OK) is returned on success, on failure a negated errno value is
 *   returned.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_6LOWPAN_COMPRESSION_HC1
int sixlowpan_uncompresshdr_hc1(FAR struct radio_driver_s *radio,
                                FAR const void *metadata, uint16_t iplen,
                                FAR struct iob_s *iob, FAR uint8_t *fptr,
                                FAR uint8_t *bptr);
#endif

/****************************************************************************
 * Name: sixlowpan_nexthopaddr
 *
 * Description:
 *   sixlowpan_nexthopaddr(): If the destination is on-link, extract the
 *   IEEE 802.15.14 destination address from the destination IP address.  If
 *   the destination is not reachable directly, use the routing table (if
 *   available) or fall back to the default router IP address and use the
 *   router IP address to derive the IEEE 802.15.4 MAC address.
 *
 ****************************************************************************/

int sixlowpan_nexthopaddr(FAR struct radio_driver_s *radio,
                          FAR const net_ipv6addr_t ipaddr,
                          FAR struct netdev_varaddr_s *destaddr);

/****************************************************************************
 * Name: sixlowpan_islinklocal, sixlowpan_destaddrfromip, and
 *       sixlowpan_ismacbased
 *
 * Description:
 *   sixlowpan_destaddrfromip(): Extract the IEEE 802.15.14 destination
 *   address from a MAC-based destination IPv6 address.  This function
 *   handles a tagged address union which may either a short or and
 *   extended destination address.
 *
 *   In the case there the IEEE 802.15.4 node functions as an endpoint in a
 *   start topology, the destination address will, instead, be the address
 *   of the star hub (which is assumed to be the address of the coordinator).
 *
 *   sixlowpan_ipfrom[s|e]addr():  Create a link-local, MAC-based IPv6
 *   address from an IEEE802.15.4 short address (saddr) or extended address
 *   (eaddr).
 *
 *   sixlowpan_islinklocal() and sixlowpan_ismacbased() will return true for
 *   address created in this fashion.  sixlowpan_destaddrfromip() is
 *   intended to handle a tagged address or any size.  Local addresses are
 *   of a fixed but configurable size and sixlowpan_isaddrbased() is for use
 *   with such local addresses.
 *
 *    128  112  96   80    64   48   32   16
 *    ---- ---- ---- ----  ---- ---- ---- ----
 *    fe80 0000 0000 0000  0000 00ff fe00 xxxx 2-byte short address IEEE
 *                                             48-bit MAC
 *    fe80 0000 0000 0000  xxxx xxxx xxxx xxxx 8-byte extended address IEEE
 *                                             EUI-64
 *
 ****************************************************************************/

#define sixlowpan_islinklocal(ipaddr) ((ipaddr)[0] == NTOHS(0xfe80))

int sixlowpan_destaddrfromip(FAR struct radio_driver_s *radio,
                             const net_ipv6addr_t ipaddr,
                             FAR struct netdev_varaddr_s *addr);

void sixlowpan_ipfromaddr(FAR const struct netdev_varaddr_s *addr,
                          FAR net_ipv6addr_t ipaddr);

bool sixlowpan_ismacbased(const net_ipv6addr_t ipaddr,
                          FAR const struct netdev_varaddr_s *addr);

/****************************************************************************
 * Name: sixlowpan_radio_framelen
 *
 * Description:
 *   Get the maximum frame length supported by radio network driver.
 *
 * Input Parameters:
 *   radio - Reference to a radio network driver state instance.
 *
 * Returned Value:
 *   A non-negative, maximum frame lengthis returned on success;  A negated
 *   errno valueis returned on any failure.
 *
 ****************************************************************************/

int sixlowpan_radio_framelen(FAR struct radio_driver_s *radio);

/****************************************************************************
 * Name: sixlowpan_src_panid
 *
 * Description:
 *   Get the source PAN ID from the IEEE802.15.4 radio.
 *
 * Input Parameters:
 *   radio - Reference to a radio network driver state instance.
 *   panid - The location in which to return the PAN ID.  0xfff may be
 *           returned if the device is not associated.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_WIRELESS_IEEE802154
int sixlowpan_src_panid(FAR struct radio_driver_s *radio,
                        FAR uint8_t *panid);
#endif

/****************************************************************************
 * Name: sixlowpan_extract_srcaddr
 *
 * Description:
 *   Extract the source MAC address from the radio-specific RX metadata, and
 *   return the source address in a radio-agnostic form.
 *
 * Input Parameters:
 *   radio    - Reference to a radio network driver state instance.
 *   metadata - Opaque reference to the radio-specific RX metadata.
 *   srcaddr  - The location in which to return the source MAC address.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int sixlowpan_extract_srcaddr(FAR struct radio_driver_s *radio,
                              FAR const void *metadata,
                              FAR struct netdev_varaddr_s *srcaddr);

/****************************************************************************
 * Name: sixlowpan_extract_destaddr
 *
 * Description:
 *   Extract the destination MAC address from the radio-specific RX metadata,
 *   and return the destination address in a radio-agnostic form.
 *
 * Input Parameters:
 *   radio    - Reference to a radio network driver state instance.
 *   metadata - Opaque reference to the radio-specific RX metadata.
 *   destaddr - The location in which to return the destination MAC address.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int sixlowpan_extract_destaddr(FAR struct radio_driver_s *radio,
                               FAR const void *metadata,
                               FAR struct netdev_varaddr_s *destaddr);

/****************************************************************************
 * Name: sixlowpan_reass_initialize
 *
 * Description:
 *   This function initializes the reassembly buffer allocator.  This
 *   function must be called early in the initialization sequence before
 *   any radios begin operation.
 *
 *   Called only once during network initialization.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void sixlowpan_reass_initialize(void);

/****************************************************************************
 * Name: sixlowpan_reass_allocate
 *
 * Description:
 *   The sixlowpan_reass_allocate function will get a free reassembly buffer
 *   structure for use by 6LoWPAN.
 *
 *   This function will first attempt to allocate from the g_free_reass
 *   list.  If that the list is empty, then the reassembly buffer structure
 *   will be allocated from the dynamic memory pool.
 *
 * Input Parameters:
 *   reasstag - The reassembly tag for subsequent lookup.
 *   fragsrc  - The source address of the fragment.
 *
 * Returned Value:
 *   A reference to the allocated reass structure.  All fields used by the
 *   reasembly logic have been zeroed.  On a failure to allocate, NULL is
 *   returned.
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

FAR struct sixlowpan_reassbuf_s *
  sixlowpan_reass_allocate(uint16_t reasstag,
                           FAR const struct netdev_varaddr_s *fragsrc);

/****************************************************************************
 * Name: sixlowpan_reass_find
 *
 * Description:
 *   Find a previously allocated, active reassembly buffer with the specified
 *   reassembly tag.
 *
 * Input Parameters:
 *   reasstag - The reassembly tag to match.
 *   fragsrc  - The source address of the fragment.
 *
 * Returned Value:
 *   A reference to the matching reass structure.
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

FAR struct sixlowpan_reassbuf_s *
  sixlowpan_reass_find(uint16_t reasstag,
                       FAR const struct netdev_varaddr_s *fragsrc);

/****************************************************************************
 * Name: sixlowpan_reass_free
 *
 * Description:
 *   The sixlowpan_reass_free function will return a reass structure
 *   to the free list of  messages if it was a pre-allocated reass
 *   structure. If the reass structure was allocated dynamically it will
 *   be deallocated.
 *
 * Input Parameters:
 *   reass - reass structure to free
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

void sixlowpan_reass_free(FAR struct sixlowpan_reassbuf_s *reass);

#endif /* CONFIG_NET_6LOWPAN */
#endif /* __NET_SIXLOWPAN_SIXLOWPAN_INTERNAL_H */
