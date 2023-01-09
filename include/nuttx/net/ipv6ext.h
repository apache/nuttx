/****************************************************************************
 * include/nuttx/net/ipv6ext.h
 * IPv6 Extension Header Definitions
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/* Extension headers carry optional Internet Layer information, and are
 * placed between the fixed IPv6 header and the upper-layer protocol header.
 * The headers form a chain, using the Next Header fields. The Next Header
 * field in the fixed header indicates the type of the first extension
 * header; the Next Header field of the last extension header indicates the
 * type of the upper-layer protocol header in the payload of the packet.
 *
 * All extension headers are a multiple of 8 octets in size; some extension
 * headers require internal padding to meet this requirement.
 *
 * Extension headers are to be examined and processed at the packet's
 * destination only, except for Hop-by-Hop Options, which need to be
 * processed at every intermediate node on the packet's path, including
 * sending and receiving node.
 *
 * If a node does not recognize a specific extension header, it should
 * discard the packet and send a Parameter Problem message (ICMPv6 type 4,
 * code 1).
 * When a Next Header value 0 appears in a header other than the fixed header
 * a node should do the same.
 */

#ifndef __INCLUDE_NUTTX_NET_IPV6EXT_H
#define __INCLUDE_NUTTX_NET_IPV6EXT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Commonly Used Extension Headers
 *
 * - Hop-by-Hop EH is used for the support of Jumbo-grams or, with the
 *   Router Alert option, it is an integral part in the operation of the
 *   IPv6 Multicast through Multicast Listener Discovery (MLD).
 * - Destination EH is used in IPv6 Mobility as well as support of certain
 *   applications.
 * - Routing EH is used in IPv6 Mobility and in Source Routing.
 * - Fragmentation EH is critical in support of communication using
 *   fragmented packets (in IPv6, the traffic source must do fragmentation-
 *   routers do not perform fragmentation of the packets they forward)
 * - Mobility EH is used in support of Mobile IPv6 service
 * - Authentication EH is similar in format and use to the IPv4
 *   authentication header defined in RFC2402.
 * - Encapsulating Security Payload EH is similar in format and use to the
 *   IPv4 ESP header defined in RFC2406 [5]. All information following the
 *   Encapsulating Security Header (ESH) is encrypted and for that reason, it
 *   is inaccessible to intermediary network devices. The ESH can be followed
 *   by an additional Destination Options EH and the upper layer datagram.
 */

/* Values of the Next Header Field.  See also IP_PROTO_* definitions in
 * include/nuttx/net/ip.h.
 *
 * These options should be applied a specific order:
 *
 *  1. Basic IPv6 Header
 *  2. Hop-by-Hop Options (0)
 *  3. Destination Options (with Routing Options) (60)
 *  4. Routing Header (43)
 *  5. Fragment Header (44)
 *  6. Authentication Header (51)
 *  7. Encapsulation Security Payload Header (50)
 *  8. Destination Options (60)
 *  9. Mobility Header (135)
 *     No next header (59)
 * 10. Upper Layer TCP (6), UDP (17), ICMPv6 (58)
 */

#define NEXT_HOPBYBOT_EH       0     /*   0  Hop-by-Hop Options Header */
                                     /*   1  See IP_PROTO_ICMP in
                                      *   include/nuttx/net/ip.h. */
                                     /*   2  See IP_PROTO_IGMP in
                                      *   include/nuttx/net/ip.h. */
                                     /*   6  See IP_PROTO_TCP in
                                      *   include/nuttx/net/ip.h. */
                                     /*  17  See IP_PROTO_UDP in
                                      *      include/nuttx/net/ip.h. */
#define NEXT_ENCAP_EH          41    /*  41  Encapsulated IPv6 Header */
#define NEXT_ROUTING_EH        43    /*  43  Routing Header */
#define NEXT_FRAGMENT_EH       44    /*  44  Fragment Header */
#define NEXT_RRSVP_EH          46    /*  46  Resource ReSerVation Protocol */
#define NEXT_ENCAPSEC_EH       50    /*  50  Encapsulating Security Payload */
#define NEXT_AUTH_EH           51    /*  51  Authentication Header */
                                     /*  58  See IP_PROTO_ICMP6 in
                                      *      include/nuttx/net/ip.h. */
#define NEXT_NOHEADER          59    /*  59  No next header */
#define NEXT_DESTOPT_EH        60    /*  60  Destination Options Header */
#define NEXT_MOBILITY_EH       135   /* 135  Mobility */
#define NEXT_HOSTID_EH         139   /* 139  Host Identity Protocol */
#define NEXT_SHIM6_EH          140   /* 140  Shim6 Protocol */
                                     /* 253, 254 Reserved for experimentation
                                      * and testing */

/* Size of the extension header */

#define EXTHDR_LEN(hdrlen)     ((hdrlen + 1) << 3)

/* Fragment header has no length field and has a fixed size */

#define EXTHDR_FRAG_LEN        8

/* More frags flag bits in 16-bit flags in fragment header */

#define FRAGHDR_FRAG_MOREFRAGS 0x0001

/* Values of the Two High-Order Bits in the Hop-to-hop Option Type Field */

#define HOPBYHOP_TYPE_MASK     0xc0
#define HOPBYHOP_TYPE_SKIP     0x00 /* Skip the option */
#define HOPBYHOP_TYPE_DISCARD  0x40 /* Silently discard the packet */
#define HOPBYHOP_TYPE_DISCARD1 0x80 /* Discard and send problem message if
                                     * ucast or mcast */
#define HOPBYHOP_TYPE_DISCARD2 0xc0 /* Discard and send problem message if
                                     * not mcast */

/* Hop-to-hop Option Types */

#define HOPBYHOP_ROUTER_ALERT  5

/* Hop-to-hop Router Alert Values */

#define OPT_RA_MLD             0    /* Datagram contains a Multicast Listener
                                     * Discovery message (RFC 2710) */
#define OPT_RA_RSVP            1    /* Datagram contains RSVP message */
#define OPT_RA_ACTIVE          2    /*  Datagram contains an Active Networks
                                     * message */
                                    /* 3-65535  Reserved to IANA for future use */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* "Generic" extension option header.  Actual size is a multiple of 8 bytes */

struct ipv6_extension_s
{
  uint8_t  nxthdr;      /* Next header */
  uint8_t  len;         /* Extension header length */
  uint8_t  pad[6];      /* Pad to a multiple of 8 bytes */
};

/* Hop-by-hop Option Header */

struct ipv6_hopbyhop_extension_s
{
  uint8_t  nxthdr;      /* Next header */
  uint8_t  len;         /* Extension header length */
  uint8_t  options[6];  /* Options */
};

struct ipv6_routing_extension_s
{
  uint8_t  nxthdr;      /* Next header */
  uint8_t  len;         /* Extension header length */
  uint8_t  type;        /* Routing type */
  uint8_t  segments;    /* Segments left */
  uint8_t  reserved[4]; /* Reserved, must be zero */
                        /* IPv6 addresses follow */
};

struct ipv6_fragment_extension_s
{
  uint8_t nxthdr;      /* Next header */
  uint8_t reserved;    /* Reserved, must be zero */
  uint8_t msoffset;    /* MS offset 5:12 */
  uint8_t lsoffset;    /* LS offset 0:4, M flag */
  uint8_t id[4];       /* Identification */
};

struct ipv6_authenitcation_extension_s
{
  uint8_t nxthdr;      /* Next header */
  uint8_t len;         /* Payload length */
  uint8_t reserved[2]; /* Reserved, must be zero */
  uint8_t security[4]; /* Security parameters */
  uint8_t sequence[4]; /* Sequence number */
  uint8_t authdata[4]; /* Authentication data */
};

struct ipv6_destoptions_extension_s
{
  uint8_t  nxthdr;     /* Next header */
  uint8_t  len;        /* Extension header length */
  uint8_t  options[6]; /* Options */
};

/* Router Alert Hop-to-Hop option */

struct ipv6_router_alert_s
{
  struct ipv6_hopbyhop_extension_s hbyh;

  uint8_t type;       /* Hop-by-hop option number (5) */
  uint8_t len;        /* Length = 2 */
  uint16_t value;     /* Value.  See OPT_RA_* Definitions */
  uint8_t pad[4];     /* Pad to a multiple of 8 bytes */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: ipv6_exthdr
 *
 * Description:
 *   Check whether it is an IPv6 extension header.
 *
 * Input Parameters:
 *   The next header value extracted from an IPv6 frame.
 *
 * Returned Value:
 *   Return true if the next header value is an IPv6 extension header.
 *
 ****************************************************************************/

bool ipv6_exthdr(uint8_t nxthdr);

#endif /* __INCLUDE_NUTTX_NET_IPV6EXT_H */
