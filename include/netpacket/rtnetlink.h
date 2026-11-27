/****************************************************************************
 * include/netpacket/rtnetlink.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifndef __INCLUDE_NETPACKET_RTNETLINK_H
#define __INCLUDE_NETPACKET_RTNETLINK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <netpacket/netlink.h>
#include <netpacket/if_addr.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Attribute definitions for struct rtattr **********************************/

/* Macros to handle attribute lists */

#define RTA_MASK                         (sizeof(uint32_t) - 1)
#define RTA_ALIGN(n)                     (((n) + RTA_MASK) & ~RTA_MASK)
#define RTA_OK(rta, n)                   ((n) >= (int)sizeof(struct rtattr) && \
                                          (rta)->rta_len >= sizeof(struct rtattr) && \
                                          (rta)->rta_len <= (n))
#define RTA_NEXT(rta, attrlen)           ((attrlen) -= RTA_ALIGN((rta)->rta_len), \
                                          (FAR struct rtattr *)(((FAR char *)(rta)) + \
                                          RTA_ALIGN((rta)->rta_len)))
#define RTA_LENGTH(n)                    (sizeof(struct rtattr) + (n))
#define RTA_SPACE(n)                     RTA_ALIGN(RTA_LENGTH(n))
#define RTA_DATA(rta)                    ((FAR void *)(((FAR char *)(rta)) + RTA_LENGTH(0)))
#define RTA_PAYLOAD(rta)                 ((int)((rta)->rta_len) - RTA_LENGTH(0))

/* NETLINK_ROUTE: Routing table attributes */

#define RTA_UNSPEC                       0    /* Inored */
#define RTA_DST                          1    /* Argument:  Route destination address */
#define RTA_SRC                          2    /* Argument:  Route source address */
#define RTA_IIF                          3    /* Argument:  Input interface index */
#define RTA_OIF                          4    /* Argument:  Output interface index */
#define RTA_GATEWAY                      5    /* Argument:  Gateway address of the route */
#define RTA_GENMASK                      6    /* Argument:  Network address mask of sub-net */
#define RTA_PRIORITY                     7    /* Argument:  Route priority */
#define RTA_TABLE                        8    /* Argument:  Route table */
#define RTA_PREFSRC                      9    /* Argument:  Preferred source address */
#define RTA_METRICS                      10   /* Argument:  Route metric */
#define RTA_MAX                          10   /* MAX type, same as last argument */

/* NETLINK_ROUTE protocol message types *************************************/

/* Link layer:
 *
 * RTM_NEWLINK, RTM_DELLINK, RTM_GETLINK
 *   Create, remove or get information about a specific network interface.
 *   These messages contain an ifinfomsg structure followed by a series
 *   of rtattr structures.
 */

#define RTM_NEWLINK                      16
#define RTM_DELLINK                      17
#define RTM_GETLINK                      18
#define RTM_SETLINK                      19

/* Address settings:
 *
 * RTM_NEWADDR, RTM_DELADDR, RTM_GETADDR
 *   Add, remove or receive information about an IP address associated with
 *   an interface.  These messages contain an ifaddrmsg structure, optionally
 *   followed by rtattr routing attributes.
 */

#define RTM_NEWADDR                      20
#define RTM_DELADDR                      21
#define RTM_GETADDR                      22

/* Routing tables:
 *
 * RTM_NEWROUTE, RTM_DELROUTE, RTM_GETROUTE
 *   Create, remove or receive information about a network route.  These
 *   messages contain an rtmsg structure with an optional sequence of
 *   rtattr structures following.
 *
 *   For RTM_GETROUTE, setting rtm_dst_len and rtm_src_len to 0 means you
 *   get all entries for the specified routing table.  For the other fields,
 *   except rtm_table and rtm_protocol, 0 is the wildcard.
 */

#define RTM_NEWROUTE                     24
#define RTM_DELROUTE                     25
#define RTM_GETROUTE                     26

/* Neighbor cache:
 *
 * RTM_NEWNEIGH, RTM_DELNEIGH, RTM_GETNEIGH
 *   Add, remove or receive information about a neighbor table entry (e.g.,
 *   an ARP entry).  The message contains an ndmsg structure.
 */

#define RTM_NEWNEIGH                     28
#define RTM_DELNEIGH                     29
#define RTM_GETNEIGH                     30

/* Routing rules:
 *
 * RTM_NEWRULE, RTM_DELRULE, RTM_GETRULE
 *   Add, delete or retrieve a routing rule.  Carries a struct rtmsg
 */

#define RTM_NEWRULE                      32
#define RTM_DELRULE                      33
#define RTM_GETRULE                      34

/* Queuing discipline settings:
 *
 * RTM_NEWQDISC, RTM_DELQDISC, RTM_GETQDISC
 *   Add, remove or get a queuing discipline.  The message contains a
 *   struct tcmsg and may be followed by a series of attributes.
 */

#define RTM_NEWQDISC                     36
#define RTM_DELQDISC                     37
#define RTM_GETQDISC                     38

/* Traffic classes used with queues:
 *
 * RTM_NEWTCLASS, RTM_DELTCLASS, RTM_GETTCLASS
 *   Add, remove or get a traffic class.  These messages contain a struct
 *   tcmsg as described above.
 */

#define RTM_NEWTCLASS                    40
#define RTM_DELTCLASS                    41
#define RTM_GETTCLASS                    42

/* Traffic filters:
 *
 * RTM_NEWTFILTER, RTM_DELTFILTER, RTM_GETTFILTER
 *   Add, remove or receive information about a traffic filter.  These
 *   messages contain a struct tcmsg as described above.
 */

#define RTM_NEWTFILTER                   44
#define RTM_DELTFILTER                   45
#define RTM_GETTFILTER                   46

/* Others: */

#define RTM_NEWACTION                    48
#define RTM_DELACTION                    49
#define RTM_GETACTION                    50
#define RTM_NEWPREFIX                    52
#define RTM_GETMULTICAST                 58
#define RTM_GETANYCAST                   62
#define RTM_NEWNEIGHTBL                  64
#define RTM_GETNEIGHTBL                  66
#define RTM_SETNEIGHTBL                  67
#define RTM_NEWNDUSEROPT                 68

#define RTM_BASE                         16
#define RTM_MAX                          68

/* Definitions for struct ifinfomsg *****************************************/

#define IFLA_RTA(r)                      ((FAR struct rtattr *) \
                                          (((FAR char *)(r)) + \
                                           NLMSG_ALIGN(sizeof(struct ifinfomsg))))
#define IFLA_PAYLOAD(n)                  NLMSG_PAYLOAD(n, sizeof(struct ifinfomsg))

/* Values for rta_type */

#define IFLA_UNSPEC                      0
#define IFLA_ADDRESS                     1
#define IFLA_BROADCAST                   2
#define IFLA_IFNAME                      3
#define IFLA_MTU                         4
#define IFLA_LINK                        5
#define IFLA_QDISC                       6
#define IFLA_STATS                       7
#define IFLA_COST                        8
#define IFLA_PRIORITY                    9
#define IFLA_MASTER                      10
#define IFLA_WIRELESS                    11
#define IFLA_PROTINFO                    12
#define IFLA_TXQLEN                      13
#define IFLA_MAP                         14
#define IFLA_WEIGHT                      15
#define IFLA_OPERSTATE                   16
#define IFLA_LINKMODE                    17

/* Definitions for struct rtmsg *********************************************/

#define RTM_RTA(r)                       ((FAR struct rtattr *)\
                                          (((FAR char *)(r)) + \
                                          NLMSG_ALIGN(sizeof(struct rtmsg))))
#define RTM_PAYLOAD(n)                   NLMSG_PAYLOAD(n, sizeof(struct rtmsg))

/* rtm_table.  Routing table identifiers */

#define RT_TABLE_UNSPEC                  0
                                         /* 1-251: User defined values */
#define RT_TABLE_MAIN                    254
#define RT_TABLE_LOCAL                   255
#define RT_TABLE_MAX                     0xffffffff

/* rtm_type */

#define RTN_UNSPEC                       0
#define RTN_UNICAST                      1    /* Gateway or direct route  */
#define RTN_LOCAL                        2    /* Accept locally */
#define RTN_BROADCAST                    3    /* Accept locally as broadcast;
                                               * send as broadcast */
#define RTN_ANYCAST                      4    /* Accept locally as broadcast
                                               * but send as unicast */
#define RTN_MULTICAST                    5    /* Multicast route */

/* rtm_protocol */

#define RTPROT_UNSPEC                    0
#define RTPROT_REDIRECT                  1    /* Route installed by ICMP redirects */
#define RTPROT_KERNEL                    2    /* Route installed by kernel */
#define RTPROT_BOOT                      3    /* Route installed during boot */
#define RTPROT_STATIC                    4    /* Route installed by administrator */
#define RTPROT_RA                        9    /* RDISC/ND router advertisements */
#define RTPROT_DHCP                      16   /* DHCP client */

/* rtm_scope */

#define  RT_SCOPE_UNIVERSE               0    /* Global route */
                                              /* 1-199: User defined values */
#define  RT_SCOPE_SITE                   200  /* Interior route in local system */
#define  RT_SCOPE_LINK                   253  /* Route on this link */
#define  RT_SCOPE_HOST                   254  /* Route on local host */
#define  RT_SCOPE_NOWHERE                255  /* Destination does not exist */

/* RTnetlink multicast groups (userspace) */

#define RTMGRP_LINK                      1
#define RTMGRP_NOTIFY                    2
#define RTMGRP_NEIGH                     4
#define RTMGRP_TC                        8

#define RTMGRP_IPV4_IFADDR               0x10
#define RTMGRP_IPV4_MROUTE               0x20
#define RTMGRP_IPV4_ROUTE                0x40
#define RTMGRP_IPV4_RULE                 0x80

#define RTMGRP_IPV6_IFADDR               0x100
#define RTMGRP_IPV6_MROUTE               0x200
#define RTMGRP_IPV6_ROUTE                0x400
#define RTMGRP_IPV6_IFINFO               0x800

#define RTMGRP_DECnet_IFADDR             0x1000
#define RTMGRP_DECnet_ROUTE              0x4000

#define RTMGRP_IPV6_PREFIX               0x20000

/* RTnetlink multicast groups */

#define RTNLGRP_NONE                     0
#define RTNLGRP_LINK                     1
#define RTNLGRP_NOTIFY                   2
#define RTNLGRP_NEIGH                    3
#define RTNLGRP_TC                       4
#define RTNLGRP_IPV4_IFADDR              5
#define RTNLGRP_IPV4_MROUTE              6
#define RTNLGRP_IPV4_ROUTE               7
#define RTNLGRP_IPV4_RULE                8
#define RTNLGRP_IPV6_IFADDR              9
#define RTNLGRP_IPV6_MROUTE              10
#define RTNLGRP_IPV6_ROUTE               11
#define RTNLGRP_IPV6_IFINFO              12
#define RTNLGRP_DECnet_IFADDR            13
#define RTNLGRP_NOP2                     14
#define RTNLGRP_DECnet_ROUTE             15
#define RTNLGRP_DECnet_RULE              16
#define RTNLGRP_NOP4                     17
#define RTNLGRP_IPV6_PREFIX              18
#define RTNLGRP_IPV6_RULE                19
#define RTNLGRP_ND_USEROPT               20
#define RTNLGRP_PHONET_IFADDR            21
#define RTNLGRP_PHONET_ROUTE             22
#define RTNLGRP_DCB                      23
#define RTNLGRP_IPV4_NETCONF             24
#define RTNLGRP_IPV6_NETCONF             25
#define RTNLGRP_MDB                      26
#define RTNLGRP_MPLS_ROUTE               27
#define RTNLGRP_NSID                     28
#define RTNLGRP_MAX                      29

#define FRA_UNSPEC                       0
#define FRA_FWMARK                       1  /* Mark */
#define FRA_TABLE                        2  /* Extended table id */
#define FRA_FWMASK                       3  /* Mask for netfilter mark */

/* rtm_flags */

#define RTM_F_CLONED                     0x200  /* This route is cloned */

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

/* NETLINK_ROUTE Message Structures *****************************************/

/* RTM_NEWLINK, RTM_DELLINK, RTM_GETLINK
 *
 * Create, remove or get information about a specific network interface.
 * These messages contain an ifinfomsg structure followed by a series
 * of rtattr structures.
 *
 * These attributes should be manipulated using only the RTA_*
 */

struct rtattr
{
  uint16_t rta_len;       /* Length of option */
  uint16_t rta_type;      /* Type of option */
                          /* Data follows */
};

struct ifinfomsg
{
  uint8_t  ifi_family;    /* AF_UNSPEC */
  uint8_t  ifi_pad;
  uint16_t ifi_type;      /* Device type (ARPHRD) */
  int32_t  ifi_index;     /* Unique interface index */
  uint32_t ifi_flags;     /* Device IFF_* flags  */
  uint32_t ifi_change;    /* Change mask, must always be 0xffffffff */
};

/* General form of address family dependent message. */

struct rtgenmsg
{
  uint8_t  rtgen_family;
};

/* RTM_NEWNEIGH, RTM_DELNEIGH, RTM_GETNEIGH
 *   Add, remove or receive information about a neighbor table entry (e.g.,
 *   an ARP entry).  The message contains an ndmsg structure.
 */

struct ndmsg
{
  uint8_t  ndm_family;
  uint8_t  ndm_pad1;
  uint16_t ndm_pad2;
  int32_t  ndm_ifindex;
  uint16_t ndm_state;
  uint8_t  ndm_flags;
  uint8_t  ndm_type;
};

/* Structures used in routing table administration. */

struct rtmsg
{
  uint8_t  rtm_family;
  uint8_t  rtm_dst_len;
  uint8_t  rtm_src_len;
  uint8_t  rtm_tos;
  uint8_t  rtm_table;     /* Routing table id */
  uint8_t  rtm_protocol;  /* Routing protocol; See RTPROT_ definitions. */
  uint8_t  rtm_scope;     /* See RT_SCOPE_* definitions */
  uint8_t  rtm_type;      /* See RTN_* definitions */
  uint32_t rtm_flags;
};

/* Structures used in prefix information. */

struct prefixmsg
{
  uint8_t  prefix_family;
  uint8_t  prefix_pad1;
  uint16_t prefix_pad2;
  int32_t  prefix_ifindex;
  uint8_t  prefix_type;
  uint8_t  prefix_len;
  uint8_t  prefix_flags;
  uint8_t  prefix_pad3;
};

enum
{
  PREFIX_UNSPEC,
  PREFIX_ADDRESS,
  PREFIX_CACHEINFO,
  PREFIX_MAX = PREFIX_CACHEINFO
};

struct prefix_cacheinfo
{
  uint32_t preferred_time;
  uint32_t valid_time;
};

/* Neighbor Discovery userland options **************************************/

struct nduseroptmsg
{
  uint8_t  nduseropt_family;
  uint8_t  nduseropt_pad1;
  uint16_t nduseropt_opts_len;     /* Total length of options */
  int32_t  nduseropt_ifindex;
  uint8_t  nduseropt_icmp_type;
  uint8_t  nduseropt_icmp_code;
  uint16_t nduseropt_pad2;
  uint32_t nduseropt_pad3;

  /* Followed by one or more ND options */
};

enum
{
  NDUSEROPT_UNSPEC,
  NDUSEROPT_SRCADDR,
  NDUSEROPT_MAX = NDUSEROPT_SRCADDR
};

/* This struct should be in sync with struct rtnl_link_stats64 */

struct rtnl_link_stats
{
  uint32_t rx_packets;       /* Total packets received  */
  uint32_t tx_packets;       /* Total packets transmitted  */
  uint32_t rx_bytes;         /* Total bytes received   */
  uint32_t tx_bytes;         /* Total bytes transmitted  */
  uint32_t rx_errors;        /* Bad packets received    */
  uint32_t tx_errors;        /* Packet transmit problems  */
  uint32_t rx_dropped;       /* No space in linux buffers  */
  uint32_t tx_dropped;       /* No space available in linux  */
  uint32_t multicast;        /* Multicast packets received  */
  uint32_t collisions;

  /* Detailed rx_errors: */

  uint32_t rx_length_errors;
  uint32_t rx_over_errors;   /* Receiver ring buff overflow  */
  uint32_t rx_crc_errors;    /* Recved pkt with crc error  */
  uint32_t rx_frame_errors;  /* Recv'd frame alignment error */
  uint32_t rx_fifo_errors;   /* Recv'r fifo overrun    */
  uint32_t rx_missed_errors; /* Receiver missed packet  */

  /* Detailed tx_errors */

  uint32_t tx_aborted_errors;
  uint32_t tx_carrier_errors;
  uint32_t tx_fifo_errors;
  uint32_t tx_heartbeat_errors;
  uint32_t tx_window_errors;

  /* For cslip etc */

  uint32_t rx_compressed;
  uint32_t tx_compressed;

  uint32_t rx_nohandler;     /* Dropped, no handler found  */
};

/* The main device statistics structure */

struct rtnl_link_stats64
{
  uint64_t rx_packets;       /* Total packets received       */
  uint64_t tx_packets;       /* Total packets transmitted    */
  uint64_t rx_bytes;         /* Total bytes received         */
  uint64_t tx_bytes;         /* Total bytes transmitted      */
  uint64_t rx_errors;        /* Bad packets received         */
  uint64_t tx_errors;        /* Packet transmit problems     */
  uint64_t rx_dropped;       /* No space in linux buffers    */
  uint64_t tx_dropped;       /* No space available in linux  */
  uint64_t multicast;        /* Multicast packets received   */
  uint64_t collisions;

  /* Detailed rx_errors: */

  uint64_t rx_length_errors;
  uint64_t rx_over_errors;   /* Receiver ring buff overflow  */
  uint64_t rx_crc_errors;    /* Recved pkt with crc error    */
  uint64_t rx_frame_errors;  /* Recv'd frame alignment error */
  uint64_t rx_fifo_errors;   /* Recv'r fifo overrun          */
  uint64_t rx_missed_errors; /* Receiver missed packet       */

  /* Detailed tx_errors */

  uint64_t tx_aborted_errors;
  uint64_t tx_carrier_errors;
  uint64_t tx_fifo_errors;
  uint64_t tx_heartbeat_errors;
  uint64_t tx_window_errors;

  /* For cslip etc */

  uint64_t rx_compressed;
  uint64_t tx_compressed;
  uint64_t rx_nohandler;     /* Dropped, no handler found    */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* __INCLUDE_NETPACKET_RTNETLINK_H */
