/****************************************************************************
 * include/netinet/in.h
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

#ifndef __INCLUDE_NETINET_IN_H
#define __INCLUDE_NETINET_IN_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <stdint.h>
#include <endian.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Values for protocol argument to socket() */

#define IPPROTO_IP            0    /* Default protocol */
#define IPPROTO_HOPOPTS       0    /* IPv6 Hop-by-Hop options.  */
#define IPPROTO_ICMP          1    /* Internet Control Message Protocol */
#define IPPROTO_IGMP          2    /* Internet Group Management Protocol */
#define IPPROTO_IPIP          4    /* IPIP tunnels (older KA9Q tunnels use 94) */
#define IPPROTO_TCP           6    /* Transmission Control Protocol */
#define IPPROTO_EGP           8    /* Exterior Gateway Protocol */
#define IPPROTO_PUP           12   /* PUP protocol */
#define IPPROTO_UDP           17   /* User Datagram Protocol */
#define IPPROTO_IDP           22   /* XNS IDP protocol */
#define IPPROTO_TP            29   /* SO Transport Protocol Class 4.  */
#define IPPROTO_DCCP          33   /* Datagram Congestion Control Protocol */
#define IPPROTO_IPV6          41   /* IPv6-in-IPv4 tunnelling */
#define IPPROTO_ROUTING       43   /* IPv6 routing header. */
#define IPPROTO_FRAGMENT      44   /* IPv6 fragmentation header.  */
#define IPPROTO_RSVP          46   /* Reservation Protocol. */
#define IPPROTO_GRE           47   /* General Routing Encapsulation. */
#define IPPROTO_ESP           50   /* Encapsulation Security Payload protocol */
#define IPPROTO_AH            51   /* Authentication Header protocol */
#define IPPROTO_ICMP6         58   /* Internal Control Message Protocol v6 */
#define IPPROTO_NONE          59   /* IPv6 no next header. */
#define IPPROTO_DSTOPTS       60   /* IPv6 destination options. */
#define IPPROTO_MTP           92   /* Multicast Transport Protocol.  */
#define IPPROTO_ENCAP         98   /* Encapsulation Header. */
#define IPPROTO_BEETPH        94   /* IP option pseudo header for BEET */
#define IPPROTO_PIM           103  /* Protocol Independent Multicast */
#define IPPROTO_COMP          108  /* Compression Header protocol */
#define IPPROTO_SCTP          132  /* Stream Control Transport Protocol */
#define IPPROTO_UDPLITE       136  /* UDP-Lite (RFC 3828) */
#define IPPROTO_MPLS          137  /* MPLS in IP (RFC 4023) */
#define IPPROTO_RAW           255  /* Raw IP packets */

/* SOL_IP protocol-level socket options.  */

#define IP_MULTICAST_IF       (__SO_PROTOCOL + 1)  /* Set local device for a
                                                    * multicast socket */
#define IP_MULTICAST_TTL      (__SO_PROTOCOL + 2)  /* Set/read the time-to-
                                                    * live value of outgoing
                                                    * multicast packets */
#define IP_MULTICAST_LOOP     (__SO_PROTOCOL + 3)  /* Set/read boolean that
                                                    * determines whether sent
                                                    * multicast packets should
                                                    * be looped back to local
                                                    * sockets. */
#define IP_ADD_MEMBERSHIP     (__SO_PROTOCOL + 4)  /* Join a multicast group */
#define IP_DROP_MEMBERSHIP    (__SO_PROTOCOL + 5)  /* Leave a multicast group */
#define IP_UNBLOCK_SOURCE     (__SO_PROTOCOL + 6)  /* Unblock previously
                                                    * blocked multicast source */
#define IP_BLOCK_SOURCE       (__SO_PROTOCOL + 7)  /* Stop receiving multicast
                                                    * data from source */

#define IP_ADD_SOURCE_MEMBERSHIP (__SO_PROTOCOL + 8) /* Join a multicast group;
                                                      * allow receive only from
                                                      * source */

#define IP_DROP_SOURCE_MEMBERSHIP (__SO_PROTOCOL + 9) /* Leave a source-specific
                                                       * group.  Stop receiving
                                                       * data from a given
                                                       * multicast group that come
                                                       * from a given source */

#define IP_MSFILTER           (__SO_PROTOCOL + 10) /* Access advanced, full-
                                                    * state filtering API */
#define IP_MULTICAST_ALL      (__SO_PROTOCOL + 11) /* Modify the delivery policy
                                                    * of multicast messages bound
                                                    * to INADDR_ANY */
#define IP_PKTINFO            (__SO_PROTOCOL + 12) /* Get some information about
                                                    * the incoming packet */
#define IP_TOS                (__SO_PROTOCOL + 13) /* Access the Type-Of-Service
                                                    * (TOS) field  */
#define IP_TTL                (__SO_PROTOCOL + 14) /* The IP TTL (time to live)
                                                    * of IP packets sent by the
                                                    * network stack */

/* SOL_IPV6 protocol-level socket options. */

#define IPV6_JOIN_GROUP       (__SO_PROTOCOL + 1)  /* Join a multicast group */
#define IPV6_LEAVE_GROUP      (__SO_PROTOCOL + 2)  /* Quit a multicast group */
#define IPV6_MULTICAST_HOPS   (__SO_PROTOCOL + 3)  /* Multicast hop limit */
#define IPV6_MULTICAST_IF     (__SO_PROTOCOL + 4)  /* Interface to use for
                                                    * outgoing multicast packets */
#define IPV6_MULTICAST_LOOP   (__SO_PROTOCOL + 5)  /* Multicast packets are
                                                    * delivered back to the local
                                                    * application */
#define IPV6_UNICAST_HOPS     (__SO_PROTOCOL + 6)  /* Unicast hop limit */
#define IPV6_V6ONLY           (__SO_PROTOCOL + 7)  /* Restrict AF_INET6 socket
                                                    * to IPv6 communications only */
#define IPV6_PKTINFO          (__SO_PROTOCOL + 8)  /* Get some information about
                                                    * the incoming packet */
#define IPV6_RECVPKTINFO      (__SO_PROTOCOL + 9)  /* It functions just same as
                                                    * IPV6_PKTINFO for now */

/* Values used with SIOCSIFMCFILTER and SIOCGIFMCFILTER ioctl's */

#define MCAST_EXCLUDE         0
#define MCAST_INCLUDE         1

/* Definitions of the bits in an Internet address integer.
 * On subnets, host and network parts are found according to
 * the subnet mask, not these masks.
 */

#define IN_CLASSA(a)          ((((in_addr_t)(a)) & 0x80000000) == 0)
#define IN_CLASSA_NET         0xff000000
#define IN_CLASSA_NSHIFT      24
#define IN_CLASSA_HOST        (0xffffffff & ~IN_CLASSA_NET)
#define IN_CLASSA_MAX         128

#define IN_CLASSB(a)          ((((in_addr_t)(a)) & 0xc0000000) == 0x80000000)
#define IN_CLASSB_NET         0xffff0000
#define IN_CLASSB_NSHIFT      16
#define IN_CLASSB_HOST        (0xffffffff & ~IN_CLASSB_NET)
#define IN_CLASSB_MAX         65536

#define IN_CLASSC(a)          ((((in_addr_t)(a)) & 0xe0000000) == 0xc0000000)
#define IN_CLASSC_NET         0xffffff00
#define IN_CLASSC_NSHIFT      8
#define IN_CLASSC_HOST        (0xffffffff & ~IN_CLASSC_NET)

/* Test if an IPv4 address is a multicast address */

#define IN_CLASSD(i)          (((in_addr_t)(i) & 0xf0000000) == 0xe0000000)
#define IN_MULTICAST(i)       IN_CLASSD(i)

/* Special values of in_addr_t */

#define INADDR_ANY            ((in_addr_t)0x00000000) /* Address to accept any
                                                       * incoming messages */
#define INADDR_BROADCAST      ((in_addr_t)0xffffffff) /* Address to send to
                                                       * all hosts */
#define INADDR_NONE           ((in_addr_t)0xffffffff) /* Address indicating an
                                                       * error return */
#define INADDR_LOOPBACK       ((in_addr_t)0x7f000001) /* Inet 127.0.0.1.  */

/* Special initializer for in6_addr_t */

#define IN6ADDR_ANY_INIT      {{{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}}}
#define IN6ADDR_LOOPBACK_INIT {{{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1}}}

/* Sizes of addresses (per OpenGroup.org) */

#define INET_ADDRSTRLEN       16   /* nnn.nnn.nnn.nnn */
#define INET6_ADDRSTRLEN      46   /* xxxx:xxxx:xxxx:xxxx:xxxx:xxxx:xxxx:xxxx */

/* struct in6_addr union selectors */

#define s6_addr               in6_u.u6_addr8
#define s6_addr16             in6_u.u6_addr16
#define s6_addr32             in6_u.u6_addr32

/* Checks for special IPv6 addresses */

#define IN6_IS_ADDR_MULTICAST(a) \
  ((a)->s6_addr[0] == 0xff)

#define IN6_IS_ADDR_LINKLOCAL(a) \
  ((a)->s6_addr16[0] & HTONS(0xffc0) == HTONS(0xfe80))

#define IN6_IS_ADDR_LOOPBACK(a) \
  ((a)->s6_addr32[0] == 0 && \
   (a)->s6_addr32[1] == 0 && \
   (a)->s6_addr32[2] == 0 && \
   (a)->s6_addr32[3] == HTONL(1))

#define IN6_IS_ADDR_UNSPECIFIED(a) \
  ((a)->s6_addr32[0] == 0 && \
   (a)->s6_addr32[1] == 0 && \
   (a)->s6_addr32[2] == 0 && \
   (a)->s6_addr32[3] == 0)

#define IN6_IS_ADDR_V4COMPAT(a) \
  ((a)->s6_addr32[0] == 0 && \
   (a)->s6_addr32[1] == 0 && \
   (a)->s6_addr32[2] == 0 && \
   (a)->s6_addr32[3] != 0 && \
   (a)->s6_addr32[3] != NTOHL(1))

#define IN6_IS_ADDR_V4MAPPED(a) \
  ((a)->s6_addr32[0] == 0 && \
   (a)->s6_addr32[1] == 0 && \
   (a)->s6_addr32[2] == HTONL(0xffff))

/* This macro to convert a 16/32-bit constant values quantity from host byte
 * order to network byte order.  The 16-bit version of this macro is required
 * for uIP:
 */

#ifdef CONFIG_ENDIAN_BIG
#  define HTONS(ns) (ns)
#  define HTONL(nl) (nl)
#else
#  define HTONS __swap_uint16
#  define HTONL __swap_uint32
#endif

#define NTOHS(hs) HTONS(hs)
#define NTOHL(hl) HTONL(hl)

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

/* IPv4/IPv6 port number */

typedef uint16_t in_port_t;

/* IPv4 Internet address */

typedef uint32_t in_addr_t;

struct in_addr
{
  in_addr_t       s_addr;           /* Address (network byte order) */
};

struct sockaddr_in
{
  sa_family_t     sin_family;       /* Address family: AF_INET */
  in_port_t       sin_port;         /* Port in network byte order */
  struct in_addr  sin_addr;         /* Internet address */
  uint8_t         sin_zero[8];
};

/* Used with certain IPv4 socket options */

struct ip_mreq
{
  struct in_addr  imr_multiaddr;    /* IPv4 multicast address of group */
  struct in_addr  imr_interface;    /* Local IPv4 address of interface */
};

struct ip_mreqn
{
  struct in_addr  imr_multiaddr;    /* IPv4 multicast address of group */
  struct in_addr  imr_interface;    /* Local IPv4 address of interface */
  unsigned int    imr_ifindex;      /* Local interface index */
};

struct ip_mreq_source
{
  struct in_addr  imr_multiaddr;    /* IPv4 multicast group address */
  struct in_addr  imr_interface;    /* IPv4 address of local interface */
  struct in_addr  imr_sourceaddr;   /* IPv4 address of multicast source */
};

struct ip_msfilter
{
  struct in_addr imsf_multiaddr;    /* IPv4 multicast group address */
  struct in_addr imsf_interface;    /* IPv4 address of local interface */
  uint32_t       imsf_fmode;        /* Filter mode */
  uint32_t       imsf_numsrc;       /* Number of sources in the following array */
  struct in_addr imsf_slist[1];     /* Array of source addresses */
};

struct in_pktinfo
{
  int            ipi_ifindex;       /* Interface index */
  struct in_addr ipi_spec_dst;      /* Local address */
  struct in_addr ipi_addr;          /* Header Destination address */
};

/* IPv6 Internet address */

struct in6_addr
{
  union
  {
    uint8_t       u6_addr8[16];
    uint16_t      u6_addr16[8];
    uint32_t      u6_addr32[4];
  } in6_u;
};

struct sockaddr_in6
{
  sa_family_t     sin6_family;      /* Address family: AF_INET6 */
  in_port_t       sin6_port;        /* Port in network byte order */
  uint32_t        sin6_flowinfo;    /* IPv6 traffic class and flow information */
  struct in6_addr sin6_addr;        /* IPv6 Internet address */
  uint32_t        sin6_scope_id;    /* Set of interfaces for a scope */
};

/* Used with certain IPv6 socket options */

struct ipv6_mreq
{
  struct in6_addr ipv6mr_multiaddr; /* IPv6 multicast address of group */
  unsigned int    ipv6mr_interface; /* Local interface index */
};

struct in6_pktinfo
{
  struct in6_addr ipi6_addr;        /* src/dst IPv6 address */
  int             ipi6_ifindex;     /* send/recv interface index */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/* Global IPv6 in6addr_any */

EXTERN const struct in6_addr in6addr_any;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Functions to convert between host and network byte ordering.
 *
 * REVISIT:  Since network order is defined as big-endian, the following
 * functions are equivalent to functions declared in endian.h:
 *
 *   htonl   htobe32
 *   htons   htobe16
 *   ntohl   be32toh
 *   ntohs   be16toh
 */

uint32_t    ntohl(uint32_t nl);
uint16_t    ntohs(uint16_t ns);
uint32_t    htonl(uint32_t hl);
uint16_t    htons(uint16_t hs);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_NETINET_IN_H */
