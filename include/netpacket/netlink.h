/****************************************************************************
 * include/netpacket/netlink.h
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

#ifndef __INCLUDE_NETPACKET_NETLINK_H
#define __INCLUDE_NETPACKET_NETLINK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <sys/socket.h>
#include <stdint.h>

#include <netpacket/if_addr.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Netlink socket protocols *************************************************/

/* The AF_NETLINK family offers multiple protocol subsets. Each interfaces
 * to a different kernel component and has a different messaging subset. The
 * subset is referenced by the protocol field in the socket call:
 *
 *   int socket(AF_NETLINK, SOCK_DGRAM or SOCK_RAW, protocol)
 *
 * Ref. Wikipedia.org
 *
 * Namespace is Linux compatible.
 */

#define NETLINK_ROUTE                    0       /* Routing/device hook for user-space
                                                  * routing daemons (default) */
#define NETLINK_UNUSED                   1       /* Unused number */
#define NETLINK_USERSOCK                 2       /* Reserved for user mode socket protocols */
#define NETLINK_FIREWALL                 3       /* Interface to receive packets from
                                                  * the firewall */
#define NETLINK_SOCK_DIAG                4       /* Socket monitoring */
#define NETLINK_NFLOG                    5       /* netfilter/iptables ULOG */
#define NETLINK_XFRM                     6       /* Interface to IPsec security databases
                                                  * for key-manager daemons using the
                                                  * Internet Key Exchange protocol. */
#define NETLINK_SELINUX                  7       /* SELinux event notifications */
#define NETLINK_ISCSI                    8       /* Open-iSCSI */
#define NETLINK_AUDIT                    9       /* Interface to auditing sub-system */
#define NETLINK_FIB_LOOKUP               10
#define NETLINK_CONNECTOR                11
#define NETLINK_NETFILTER                12      /* netfilter subsystem */
#define NETLINK_IP6_FW                   13      /* Interface to transport packets from
                                                  * netfilter to user-space. */
#define NETLINK_DNRTMSG                  14      /* DECnet routing messages */
#define NETLINK_KOBJECT_UEVENT           15      /* Kernel messages to userspace */
#define NETLINK_GENERIC                  16      /* NETLINK_DM (DM Events) */
#define NETLINK_SCSITRANSPORT            18      /* SCSI Transports */
#define NETLINK_ECRYPTFS                 19
#define NETLINK_RDMA                     20
#define NETLINK_CRYPTO                   21      /* Crypto layer */
#define NETLINK_SMC                      22      /* SMC monitoring */

/* Definitions associated with struct sockaddr_nl ***************************/

/* Flags values */

#define NLM_F_REQUEST                    0x0001  /* It is request message.   */
#define NLM_F_MULTI                      0x0002  /* Multipart message, terminated by NLMSG_DONE */
#define NLM_F_ACK                        0x0004  /* Reply with ack, with zero or error code */
#define NLM_F_ECHO                       0x0008  /* Echo this request     */
#define NLM_F_DUMP_INTR                  0x0010  /* Dump was inconsistent due to sequence change */
#define NLM_F_DUMP_FILTERED              0x0020  /* Dump was filtered as requested */

/* Modifiers to GET request */

#define NLM_F_ROOT                       0x0100  /* Specify tree root  */
#define NLM_F_MATCH                      0x0200  /* Return all matching  */
#define NLM_F_ATOMIC                     0x0400  /* Atomic GET    */
#define NLM_F_DUMP                       (NLM_F_ROOT | NLM_F_MATCH)

/* Modifiers to NEW request */

#define NLM_F_REPLACE                    0x0100  /* Override existing    */
#define NLM_F_EXCL                       0x0200  /* Do not touch, if it exists  */
#define NLM_F_CREATE                     0x0400  /* Create, if it does not exist  */
#define NLM_F_APPEND                     0x0800  /* Add to end of list    */

/* Modifiers to DELETE request */

#define NLM_F_NONREC                     0x0100  /* Do not delete recursively  */

/* Flags for ACK message */

#define NLM_F_CAPPED                     0x0100  /* Request was capped */
#define NLM_F_ACK_TLVS                   0x0200  /* Extended ACK TVLs were included */

/* Definitions for struct nlmsghdr ******************************************/

#define NLMSG_MASK                       (sizeof(uint32_t) - 1)
#define NLMSG_ALIGN(n)                   (((n) + NLMSG_MASK) & ~NLMSG_MASK)
#define NLMSG_HDRLEN                     sizeof(struct nlmsghdr)
#define NLMSG_LENGTH(n)                  (NLMSG_HDRLEN + (n))
#define NLMSG_SPACE(len)                 NLMSG_ALIGN(NLMSG_LENGTH(len))
#define NLMSG_DATA(hdr)                  ((FAR void *)(((FAR char *)hdr) + NLMSG_HDRLEN))
#define NLMSG_NEXT(hdr, n)               ((n) -= NLMSG_ALIGN((hdr)->nlmsg_len), \
                                          (FAR struct nlmsghdr *) \
                                          (((FAR char *)(hdr)) + NLMSG_ALIGN((hdr)->nlmsg_len)))
#define NLMSG_OK(nlh, len)               ((len) >= (int)sizeof(struct nlmsghdr) && \
                                          (nlh)->nlmsg_len >= sizeof(struct nlmsghdr) && \
                                          (nlh)->nlmsg_len <= (len))
#define NLMSG_PAYLOAD(hdr, len)          ((hdr)->nlmsg_len - NLMSG_SPACE(len))

#define NLMSG_NOOP                       1    /* Nothing */
#define NLMSG_ERROR                      2    /* Error */
#define NLMSG_DONE                       3    /* End of a dump */
#define NLMSG_OVERRUN                    4    /* Data lost */
#define NLMSG_MIN_TYPE                   16   /* < 16:  Reserved control messages */

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

#define IFLA_IFNAME                      1
#define IFLA_ADDRESS                     2
#define IFLA_MTU                         3
#define IFLA_WIRELESS                    4
#define IFLA_STATS                       5
#define IFLA_OPERSTATE                   6
#define IFLA_LINKMODE                    7
#define IFLA_BROADCAST                   8
#define IFLA_LINK                        9
#define IFLA_QDISC                       10
#define IFLA_COST                        11
#define IFLA_PRIORITY                    12
#define IFLA_MASTER                      13
#define IFLA_PROTINFO                    14
#define IFLA_TXQLEN                      15
#define IFLA_MAP                         16
#define IFLA_WEIGHT                      17

/* Definitions for struct rtmsg *********************************************/

#define RTM_RTA(r)                       ((FAR struct rtattr *)\
                                          (((FAR char *)(r)) + \
                                          NLMSG_ALIGN(sizeof(struct rtmsg))))
#define RTM_PAYLOAD(n)                   NLMSG_PAYLOAD(n, sizeof(struct rtmsg))

/* rtm_table.  Routing table identifiers */

#define RT_TABLE_UNSPEC                  0
                                            /* 1-251: User defined values */
#define RT_TABLE_MAIN                    254
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

/* nla_type (16 bits)
 * +---+---+-------------------------------+
 * | N | O | Attribute Type                |
 * +---+---+-------------------------------+
 * N := Carries nested attributes
 * O := Payload stored in network byte order
 *
 * Note: The N and O flag are mutually exclusive.
 */

#define NLA_F_NET_BYTEORDER              (1 << 14)
#define NLA_F_NESTED                     (1 << 15)
#define NLA_TYPE_MASK                    ~(NLA_F_NESTED | NLA_F_NET_BYTEORDER)

#define NLA_ALIGNTO                      4
#define NLA_ALIGN(len)                   (((len) + NLA_ALIGNTO - 1) & ~(NLA_ALIGNTO - 1))
#define NLA_HDRLEN                       (NLA_ALIGN(sizeof(struct nlattr)))

/* rtm_flags */

#define RTM_F_CLONED                     0x200  /* This route is cloned */

/* Attribute definitions for struct nfattr **********************************/

/* Macros to handle attribute lists */

#define NFNL_NFA_NEST                    0x8000
#define NFA_TYPE(attr)                   ((attr)->nfa_type & 0x7fff)

#define NFA_MASK                         (sizeof(uint32_t) - 1)
#define NFA_ALIGN(n)                     (((n) + NFA_MASK) & ~NFA_MASK)
#define NFA_OK(nfa, n)                   ((n) >= (int)sizeof(struct nfattr) && \
                                          (nfa)->nfa_len >= sizeof(struct nfattr) && \
                                          (nfa)->nfa_len <= (n))
#define NFA_NEXT(nfa, attrlen)           ((attrlen) -= NFA_ALIGN((nfa)->nfa_len), \
                                          (FAR struct nfattr *)(((FAR char *)(nfa)) + \
                                          NFA_ALIGN((nfa)->nfa_len)))
#define NFA_LENGTH(len)                  ((sizeof(struct nfattr)) + (len))
#define NFA_SPACE(len)                    NFA_ALIGN(NFA_LENGTH(len))
#define NFA_DATA(nfa)                    ((FAR void *)(((FAR char *)(nfa)) + NFA_LENGTH(0)))
#define NFA_PAYLOAD(nfa)                 ((int)((nfa)->nfa_len) - NFA_LENGTH(0))

/* Definitions for struct nfgenmsg ******************************************/

#define NFM_NFA(n)                       ((FAR struct nfattr *) \
                                          (((FAR char *)(n)) + \
                                          NLMSG_ALIGN(sizeof(struct nfgenmsg))))
#define NFM_PAYLOAD(n)                   NLMSG_PAYLOAD(n, sizeof(struct nfgenmsg))

/* Definitions for NETLINK_NETFILTER ****************************************/

#define NFNETLINK_V0                     0

/* netfilter netlink message types are split in two pieces:
 * 8 bit subsystem, 8bit operation.
 */

#define NFNL_SUBSYS_ID(x)                (((x) & 0xff00) >> 8)
#define NFNL_MSG_TYPE(x)                 ((x) & 0x00ff)

/* Subsystems */

#define NFNL_SUBSYS_NONE                 0
#define NFNL_SUBSYS_CTNETLINK            1
#define NFNL_SUBSYS_CTNETLINK_EXP        2
#define NFNL_SUBSYS_QUEUE                3
#define NFNL_SUBSYS_ULOG                 4
#define NFNL_SUBSYS_OSF                  5
#define NFNL_SUBSYS_IPSET                6
#define NFNL_SUBSYS_ACCT                 7
#define NFNL_SUBSYS_CTNETLINK_TIMEOUT    8
#define NFNL_SUBSYS_CTHELPER             9
#define NFNL_SUBSYS_NFTABLES             10
#define NFNL_SUBSYS_NFT_COMPAT           11
#define NFNL_SUBSYS_HOOK                 12
#define NFNL_SUBSYS_COUNT                13

/* NETLINK_NETFILTER: subsystem CTNL (ip conntrack netlink) message types */

#define IPCTNL_MSG_CT_NEW                0
#define IPCTNL_MSG_CT_GET                1
#define IPCTNL_MSG_CT_DELETE             2
#define IPCTNL_MSG_CT_GET_CTRZERO        3
#define IPCTNL_MSG_CT_GET_STATS_CPU      4
#define IPCTNL_MSG_CT_GET_STATS          5
#define IPCTNL_MSG_CT_GET_DYING          6
#define IPCTNL_MSG_CT_GET_UNCONFIRMED    7
#define IPCTNL_MSG_MAX                   8

/* NETLINK_NETFILTER: Conntrack attributes */

#define CTA_UNSPEC                       0
#define CTA_TUPLE_ORIG                   1
#define CTA_TUPLE_REPLY                  2
#define CTA_STATUS                       3
#define CTA_PROTOINFO                    4
#define CTA_HELP                         5
#define CTA_NAT_SRC                      6
#define CTA_TIMEOUT                      7
#define CTA_MARK                         8
#define CTA_COUNTERS_ORIG                9
#define CTA_COUNTERS_REPLY               10
#define CTA_USE                          11
#define CTA_ID                           12
#define CTA_NAT_DST                      13
#define CTA_TUPLE_MASTER                 14
#define CTA_SEQ_ADJ_ORIG                 15
#define CTA_NAT_SEQ_ADJ_ORIG             CTA_SEQ_ADJ_ORIG
#define CTA_SEQ_ADJ_REPLY                16
#define CTA_NAT_SEQ_ADJ_REPLY            CTA_SEQ_ADJ_REPLY
#define CTA_ZONE                         18
#define CTA_SECCTX                       19
#define CTA_TIMESTAMP                    20
#define CTA_MARK_MASK                    21
#define CTA_LABELS                       22
#define CTA_LABELS_MASK                  23
#define CTA_SYNPROXY                     24
#define CTA_FILTER                       25
#define CTA_STATUS_MASK                  26
#define CTA_MAX                          26

/* NETLINK_NETFILTER: Conntrack tuple attributes */

#define CTA_TUPLE_UNSPEC                 0
#define CTA_TUPLE_IP                     1
#define CTA_TUPLE_PROTO                  2
#define CTA_TUPLE_ZONE                   3
#define CTA_TUPLE_MAX                    3

/* NETLINK_NETFILTER: Conntrack IP attributes */

#define CTA_IP_UNSPEC                    0
#define CTA_IP_V4_SRC                    1
#define CTA_IP_V4_DST                    2
#define CTA_IP_V6_SRC                    3
#define CTA_IP_V6_DST                    4
#define CTA_IP_MAX                       4

/* NETLINK_NETFILTER: Conntrack protocol attributes */

#define CTA_PROTO_UNSPEC                 0
#define CTA_PROTO_NUM                    1
#define CTA_PROTO_SRC_PORT               2
#define CTA_PROTO_DST_PORT               3
#define CTA_PROTO_ICMP_ID                4
#define CTA_PROTO_ICMP_TYPE              5
#define CTA_PROTO_ICMP_CODE              6
#define CTA_PROTO_ICMPV6_ID              7
#define CTA_PROTO_ICMPV6_TYPE            8
#define CTA_PROTO_ICMPV6_CODE            9
#define CTA_PROTO_MAX                    9

/* NFnetlink multicast groups (userspace) */

#define NF_NETLINK_CONNTRACK_NEW         0x00000001
#define NF_NETLINK_CONNTRACK_UPDATE      0x00000002
#define NF_NETLINK_CONNTRACK_DESTROY     0x00000004
#define NF_NETLINK_CONNTRACK_EXP_NEW     0x00000008
#define NF_NETLINK_CONNTRACK_EXP_UPDATE  0x00000010
#define NF_NETLINK_CONNTRACK_EXP_DESTROY 0x00000020

/* NFnetlink multicast groups */

#define NFNLGRP_NONE                     0
#define NFNLGRP_CONNTRACK_NEW            1
#define NFNLGRP_CONNTRACK_UPDATE         2
#define NFNLGRP_CONNTRACK_DESTROY        3
#define NFNLGRP_CONNTRACK_EXP_NEW        4
#define NFNLGRP_CONNTRACK_EXP_UPDATE     5
#define NFNLGRP_CONNTRACK_EXP_DESTROY    6
#define NFNLGRP_NFTABLES                 7
#define NFNLGRP_ACCT_QUOTA               8
#define NFNLGRP_NFTRACE                  9
#define NFNLGRP_MAX                      9

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

/* Netlink socket address type. */

struct sockaddr_nl
{
  sa_family_t nl_family;  /* AF_NETLINK */
  uint16_t nl_pad;        /* Zero */
  uint32_t nl_pid;        /* Port ID  */
  uint32_t nl_groups;     /* Multicast groups mask */
};

/* Packet structure.  The Netlink message header, struct nlmsghdr, must be
 * prepared by the caller. The Netlink socket generally works in a SOCK_RAW-
 * like mode (even if SOCK_DGRAM was used to create it).
 *
 * The data portion then contains a subsystem-specific message that may be
 * further nested.
 */

struct nlmsghdr
{
  uint32_t nlmsg_len;     /* Length of message including header */
  uint16_t nlmsg_type;    /* Message content */
  uint16_t nlmsg_flags;   /* Additional flags */
  uint32_t nlmsg_seq;     /* Sequence number */
  uint32_t nlmsg_pid;     /* Sending process port ID */
                          /* Data follows */
};

struct nlmsgerr
{
  int32_t error;
  struct nlmsghdr msg;

  /* Followed by the message contents unless NETLINK_CAP_ACK was set
   * or the ACK indicates success (error == 0)
   * message length is aligned with NLMSG_ALIGN()
   */

  /* Followed by TLVs defined in enum nlmsgerr_attrs
   * if NETLINK_EXT_ACK was set
   */
};

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

/* <------- NLA_HDRLEN ------> <-- NLA_ALIGN(payload)-->
 * +---------------------+- - -+- - - - - - - - - -+- - -+
 * |        Header       | Pad |     Payload       | Pad |
 * |   (struct nlattr)   | ing |                   | ing |
 * +---------------------+- - -+- - - - - - - - - -+- - -+
 *  <-------------- nlattr->nla_len -------------->
 */

struct nlattr
{
  uint16_t nla_len;
  uint16_t nla_type;
};

/* Generic 32 bitflags attribute content sent to the kernel.
 *
 * The value is a bitmap that defines the values being set
 * The selector is a bitmask that defines which value is legit
 *
 * Examples:
 *  value = 0x0, and selector = 0x1
 *  implies we are selecting bit 1 and we want to set its value to 0.
 *
 *  value = 0x2, and selector = 0x2
 *  implies we are selecting bit 2 and we want to set its value to 1.
 *
 */

struct nla_bitfield32
{
  uint32_t value;
  uint32_t selector;
};

/* NETLINK_NETFILTER Message Structures *************************************/

/* These attributes should be manipulated using only the NFA_* */

struct nfattr
{
  uint16_t nfa_len;
  uint16_t nfa_type;
};

/* General form of address family dependent message. */

struct nfgenmsg
{
  uint8_t  nfgen_family; /* AF_xxx */
  uint8_t  version;      /* nfnetlink version */
  uint16_t res_id;       /* Resource id */
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

#endif /* __INCLUDE_NETPACKET_NETLINK_H */
