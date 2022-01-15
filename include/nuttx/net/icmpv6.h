/****************************************************************************
 * include/nuttx/net/icmpv6.h
 * Header file for the NuttX ICMPv6 stack.
 *
 *   Copyright (C) 2007-2009, 2012, 2014, 2017 Gregory Nutt.
 *   All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * This logic was leveraged from uIP which also has a BSD-style license:
 *
 *   Author Adam Dunkels <adam@dunkels.com>
 *   Copyright (c) 2001-2003, Adam Dunkels.
 *   All rights reserved.
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
 * 3. The name of the author may not be used to endorse or promote
 *    products derived from this software without specific prior
 *    written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_NET_ICMPV6_H
#define __INCLUDE_NUTTX_NET_ICMPV6_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <nuttx/net/netconfig.h>
#include <nuttx/net/ip.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* ICMPv6 definitions */

/* ICMPv6 Message Types */

#define ICMPv6_RESERVED                0    /* RFC 4443 */
#define ICMPv6_DEST_UNREACHABLE        1
#define ICMPv6_PACKET_TOO_BIG          2
#define ICMPv6_PACKET_TIME_EXCEEDED    3
#define ICMPv6_PACKET_PARAM_PROBLEM    4
#define ICMPv6_PRIVATE_ERR_MSG_1       100
#define ICMPv6_PRIVATE_ERR_MSG_2       101
#define ICMPv6_RESERVED_ERROR_MSG      127
#define ICMPv6_ECHO_REQUEST            128
#define ICMPv6_ECHO_REPLY              129
#define ICMPV6_MCAST_LISTEN_QUERY      130   /* RFC 2710 and 3810 */
#define ICMPV6_MCAST_LISTEN_REPORT_V1  131   /* RFC 2710 */
#define ICMPV6_MCAST_LISTEN_DONE       132   /* RFC 2710 */
#define ICMPV6_ROUTER_SOLICIT          133   /* RFC 4861 */
#define ICMPV6_ROUTER_ADVERTISE        134
#define ICMPv6_NEIGHBOR_SOLICIT        135
#define ICMPv6_NEIGHBOR_ADVERTISE      136
#define ICMPv6_REDIRECT                137
#define ICMPV6_ROUTER_RENUMBERING      138   /* Matt Crawford */
#define ICMPV6_NODE_INFO_QUERY         139   /* RFC 4620 */
#define ICMPV6_NODE_INFO_REPLY         140
#define ICMPV6_INV_NEIGHBOR_DISCOVERY  141   /* RFC 3122 */
#define ICMPV6_INV_NEIGHBOR_ADVERTISE  142
#define ICMPV6_MCAST_LISTEN_REPORT_V2  143   /* RFC 3810 */
#define ICMPV6_HOME_AGENT_DISCOVERY    144   /* RFC 3775 */
#define ICMPV6_HOME_AGENT_REPLY        145
#define ICMPV6_MOBILE_PREFIX_SOLICIT   146
#define ICMPV6_MOBILE_PREFIX_ADVERTISE 147
#define ICMPv6_PRIVATE_INFO_MSG_1      200   /* RFC 4443 */
#define ICMPv6_PRIVATE_INFO_MSG_2      201
#define ICMPv6_RESERVED_INFO_MSG       255

#define ICMPv6_FLAG_S (1 << 6)

/* Header sizes */

#define ICMPv6_HDRLEN    4                             /* Size of ICMPv6 header */
#define IPICMPv6_HDRLEN  (ICMPv6_HDRLEN + IPv6_HDRLEN) /* Size of IPv6 + ICMPv6 header */

/* Option types */

#define ICMPv6_OPT_SRCLLADDR  1 /* Source Link-Layer Address */
#define ICMPv6_OPT_TGTLLADDR  2 /* Target Link-Layer Address */
#define ICMPv6_OPT_PREFIX     3 /* Prefix Information */
#define ICMPv6_OPT_REDIRECT   4 /* Redirected Header */
#define ICMPv6_OPT_MTU        5 /* MTU */

/* ICMPv6 Neighbor Advertisement message flags */

#define ICMPv6_NADV_FLAG_R    (1 << 7) /* Router flag */
#define ICMPv6_NADV_FLAG_S    (1 << 6) /* Solicited flag */
#define ICMPv6_NADV_FLAG_O    (1 << 5) /* Override flag */

/* ICMPv6 Router Advertisement message flags */

#define ICMPv6_RADV_FLAG_M    (1 << 7) /* Managed address configuration flag */
#define ICMPv6_RADV_FLAG_O    (1 << 6) /* Other configuration flag */

/* Prefix option flags */

#define ICMPv6_PRFX_FLAG_L    (1 << 7) /* On-link flag */
#define ICMPv6_PRFX_FLAG_A    (1 << 6) /* Autonomous address-configuration flag */

/* Return with size of an option (in full octects) using the size of a link
 * layer address taking into account a header of the two-bytes.
 */

#define ICMPv6_OPT_SIZE(a)    ((a) > 0 ? ((a) + 2 + 7) & ~7 : 0)
#define ICMPv6_OPT_OCTECTS(a) ((a) > 0 ? ((a) + 2 + 7) >> 3 : 0)

/* Codes for Destination Unreachable */

#define ICMPv6_NOROUTE        0
#define ICMPv6_ADM_PROHIBITED 1
#define ICMPv6_NOT_NEIGHBOUR  2
#define ICMPv6_ADDR_UNREACH   3
#define ICMPv6_PORT_UNREACH   4
#define ICMPv6_POLICY_FAIL    5
#define ICMPv6_REJECT_ROUTE   6

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

/* The ICMPv6 header */

struct icmpv6_hdr_s
{
  uint8_t  type;             /* Defines the format of the ICMP message */
  uint8_t  code;             /* Further qualifies the ICMP messages */
  uint16_t chksum;           /* Checksum of ICMP header and data */

  /* Data following the ICMP header contains the data specific to the
   * message type indicated by the Type and Code fields.
   */

  uint16_t data[2];
};

/* The ICMPv6 and IPv6 headers */

struct icmpv6_iphdr_s
{
  /* IPv6 Ip header */

  uint8_t  vtc;              /* Bits 0-3: version, bits 4-7: traffic class (MS) */
  uint8_t  tcf;              /* Bits 0-3: traffic class (LS), 4-bits: flow label (MS) */
  uint16_t flow;             /* 16-bit flow label (LS) */
  uint8_t  len[2];           /* 16-bit Payload length */
  uint8_t  proto;            /*  8-bit Next header (same as IPv4 protocol field) */
  uint8_t  ttl;              /*  8-bit Hop limit (like IPv4 TTL field) */
  net_ipv6addr_t srcipaddr;  /* 128-bit Source address */
  net_ipv6addr_t destipaddr; /* 128-bit Destination address */

  /* ICMPv6 header */

  uint8_t  type;             /* Defines the format of the ICMP message */
  uint8_t  code;             /* Further qualifies the ICMP messages */
  uint16_t chksum;           /* Checksum of ICMP header and data */

  /* Data following the ICMP header contains the data specific to the
   * message type indicated by the Type and Code fields.
   */
};

/* This the message format for the ICMPv6 Neighbor Solicitation message */

struct icmpv6_neighbor_solicit_s
{
  uint8_t  type;             /* Message Type: ICMPv6_NEIGHBOR_SOLICIT */
  uint8_t  code;             /* Further qualifies the ICMP messages */
  uint16_t chksum;           /* Checksum of ICMP header and data */
  uint8_t  flags[4];         /* See ICMPv6_FLAG_ definitions */
  net_ipv6addr_t tgtaddr;    /* 128-bit Target IPv6 address */

  uint8_t  opttype;          /* Option Type: ICMPv6_OPT_SRCLLADDR */
  uint8_t  optlen;           /* Option length: 1 octet */
  uint8_t  srclladdr[6];     /* Options: Source link layer address */
};

#define SIZEOF_ICMPV6_NEIGHBOR_SOLICIT_S(n) \
  (sizeof(struct icmpv6_neighbor_solicit_s) + ICMPv6_OPT_SIZE(n) - 8)

/* This the message format for the ICMPv6 Neighbor Advertisement message */

struct icmpv6_neighbor_advertise_s
{
  uint8_t  type;             /* Message Type: ICMPv6_NEIGHBOR_ADVERTISE */
  uint8_t  code;             /* Further qualifies the ICMP messages */
  uint16_t chksum;           /* Checksum of ICMP header and data */
  uint8_t  flags[4];         /* See ICMPv6_NADV_FLAG_ definitions */
  net_ipv6addr_t tgtaddr;    /* Target IPv6 address */

  uint8_t  opttype;          /* Option Type: ICMPv6_OPT_TGTLLADDR */
  uint8_t  optlen;           /* Option length in octets */
  uint8_t  tgtlladdr[6];     /* Options: Target link layer address */
                             /* Actual size determined by optlen */
};

#define SIZEOF_ICMPV6_NEIGHBOR_ADVERTISE_S(n) \
  (sizeof(struct icmpv6_neighbor_advertise_s) + ICMPv6_OPT_SIZE(n) - 8)

/* This the message format for the ICMPv6 Router Solicitation message */

struct icmpv6_router_solicit_s
{
  uint8_t  type;             /* Message Type: ICMPV6_ROUTER_SOLICIT */
  uint8_t  code;             /* Further qualifies the ICMP messages */
  uint16_t chksum;           /* Checksum of ICMP header and data */
  uint8_t  flags[4];         /* See ICMPv6_RADV_FLAG_ definitions (must be zero) */

  uint8_t  opttype;          /* Option Type: ICMPv6_OPT_SRCLLADDR */
  uint8_t  optlen;           /* Option length in octets */
  uint8_t  srclladdr[6];     /* Options: Source link layer address */
};

#define SIZEOF_ICMPV6_ROUTER_SOLICIT_S(n) \
  (sizeof(struct icmpv6_router_solicit_s) + ICMPv6_OPT_SIZE(n) - 8)

/* This the message format for the ICMPv6 Router Advertisement message:
 * Options may include: ICMPv6_OPT_SRCLLADDR, ICMPv6_OPT_MTU, and/or
 *                      ICMPv6_OPT_PREFIX
 */

struct icmpv6_router_advertise_s
{
  uint8_t  type;             /* Message Type: ICMPV6_ROUTER_ADVERTISE */
  uint8_t  code;             /* Further qualifies the ICMP messages */
  uint16_t chksum;           /* Checksum of ICMP header and data */
  uint8_t  hoplimit;         /* Current hop limit */
  uint8_t  flags;            /* See ICMPv6_RADV_FLAG_* definitions */
  uint16_t lifetime;         /* Router lifetime */
  uint32_t reachable;        /* Reachable time */
  uint32_t retrans;          /* Retransmission timer */
                             /* Options begin here */
};

#define ICMPv6_RADV_MINLEN    (16)
#define ICMPv6_RADV_OPTLEN(n) ((n) - ICMPv6_RADV_MINLEN)

/* This the message format for the ICMPv6 Echo Request message */

struct icmpv6_echo_request_s
{
  uint8_t  type;             /* Message Type: ICMPv6_ECHO_REQUEST */
  uint8_t  code;             /* Further qualifies the ICMP messages */
  uint16_t chksum;           /* Checksum of ICMP header and data */
  uint16_t id;               /* Identifier */
  uint16_t seqno;            /* Sequence Number */
  uint8_t  data[1];          /* Data follows */
};

#define SIZEOF_ICMPV6_ECHO_REQUEST_S(n) \
  (sizeof(struct icmpv6_echo_request_s) - 1 + (n))

/* This the message format for the ICMPv6 Echo Reply message */

struct icmpv6_echo_reply_s
{
  uint8_t  type;             /* Message Type: ICMPv6_ECHO_REQUEST */
  uint8_t  code;             /* Further qualifies the ICMP messages */
  uint16_t chksum;           /* Checksum of ICMP header and data */
  uint16_t id;               /* Identifier */
  uint16_t seqno;            /* Sequence Number */
  uint8_t  data[1];          /* Data follows */
};

#define SIZEOF_ICMPV6_ECHO_REPLY_S(n) \
  (sizeof(struct icmpv6_echo_reply_s) - 1 + (n))

/* Option types */

struct icmpv6_generic_s
{
  uint8_t  opttype;          /* Octet 1: Option Type */
  uint8_t  optlen;           /* "   " ": Option length (in octets) */
  uint16_t pad;              /* "   " ": The rest depends on the option */
};

struct icmpv6_srclladdr_s
{
  uint8_t  opttype;          /* Octet 1: Option Type: ICMPv6_OPT_SRCLLADDR */
  uint8_t  optlen;           /* "   " ": Option length: 1 octet */
  uint8_t  srclladdr[6];     /* "   " ": Options: Source link layer address */
};

#define SIZEOF_ICMPV6_SRCLLADDR_S(n) ICMPv6_OPT_SIZE(n)

struct icmpv6_tgrlladdr_s
{
  uint8_t  opttype;          /* Octet 1: Option Type: ICMPv6_OPT_TGTLLADDR */
  uint8_t  optlen;           /* "   " ": Option length in octets */
  uint8_t  tgtlladdr[6];     /* "   " ": Options: Target link layer address */
};

#define SIZEOF_ICMPV6_TGRLLADDR_S(n) ICMPv6_OPT_SIZE(n)

struct icmpv6_prefixinfo_s
{
  uint8_t  opttype;          /* Octet 1: Option Type: ICMPv6_OPT_PREFIX */
  uint8_t  optlen;           /* "   " ": Option length: 4 octets */
  uint8_t  preflen;          /* "   " ": Prefix length */
  uint8_t  flags;            /* "   " ": Flags */
  uint32_t vlifetime;        /* "   " ": Valid lifetime */
  uint32_t plifetime;        /* Octet 2: Preferred lifetime */
  uint16_t reserved[2];      /* "   " ": Reserved */
  uint16_t prefix[8];        /* Octets 3-4: Prefix */
};

struct icmpv6_redirect_s
{
  uint8_t  opttype;          /* Octet 1: Option Type: ICMPv6_OPT_REDIRECT */
  uint8_t  optlen;           /* "   " ": Option length: 1 octet */
  uint16_t reserved[3];      /* "   " ": Reserved */
  uint8_t  header[1];        /* Octets 2-: Beginning of the IP header */
};

struct icmpv6_mtu_s
{
  uint8_t  opttype;          /* Octet 1: Option Type: ICMPv6_OPT_MTU */
  uint8_t  optlen;           /* "   " ": Option length: 1 octet */
  uint16_t reserved;         /* "   " ": Reserved */
  uint32_t mtu;              /* "   " ": MTU */
};

/* The structure holding the ICMP statistics that are gathered if
 * CONFIG_NET_STATISTICS is defined.
 */

#ifdef CONFIG_NET_STATISTICS
struct icmpv6_stats_s
{
  net_stats_t drop;       /* Number of dropped ICMP packets */
  net_stats_t recv;       /* Number of received ICMP packets */
  net_stats_t sent;       /* Number of sent ICMP packets */
  net_stats_t typeerr;    /* Number of ICMP packets with a wrong type */
};
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif /* __INCLUDE_NUTTX_NET_ICMPV6_H */
