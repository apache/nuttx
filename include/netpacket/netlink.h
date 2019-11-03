/****************************************************************************
 * include/netpacket/netlink.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef  __INCLUDE_NETPACKET_NETLINK_H
#define  __INCLUDE_NETPACKET_NETLINK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/socket.h>
#include <stdint.h>

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

#define NETLINK_ROUTE          0       /* Routing/device hook for user-space
                                        * routing daemons (default) */
#define NETLINK_USERSOCK       1       /* Reserved for user mode socket protocols */
#define NETLINK_FIREWALL       2       /* Interface to receive packets from
                                        * the firewall */
#define NETLINK_SOCK_DIAG      3       /* Socket monitoring */
#define NETLINK_NFLOG          4       /* netfilter/iptables ULOG */
#define NETLINK_XFRM           5       /* Interface to IPsec security databases
                                        * for key-manager daemons using the Internet
                                        * Key Exchange protocol. */
#define NETLINK_ISCSI          6       /* Open-iSCSI */
#define NETLINK_AUDIT          7       /* Interface to auditing sub-system */
#define NETLINK_FIB_LOOKUP     8
#define NETLINK_CONNECTOR      9
#define NETLINK_NETFILTER      10      /* netfilter subsystem */
#define NETLINK_IP6_FW         11      /* Interface to transport packets from
                                        * netfilter to user-space. */
#define NETLINK_DNRTMSG        12      /* DECnet routing messages */
#define NETLINK_KOBJECT_UEVENT 13      /* Kernel messages to userspace */
#define NETLINK_GENERIC        14
                                       /* NETLINK_DM (DM Events) */
#define NETLINK_SCSITRANSPORT  16      /* SCSI Transports */
#define NETLINK_ECRYPTFS       17
#define NETLINK_RDMA           18
#define NETLINK_CRYPTO         19      /* Crypto layer */
#define NETLINK_SMC            20      /* SMC monitoring */

/* NETLINK_ROUTE protocol message types *************************************/
/* Link layer:
 *
 * RTM_NEWLINK, RTM_DELLINK, RTM_GETLINK
 *   Create, remove or get information about a specific network interface.
 *   These messages contain an ifinfomsg structure followed by a series
 *   of rtattr structures.
 */

#define RTM_NEWLINK          0
#define RTM_DELLINK          1
#define RTM_GETLINK          2
#define RTM_SETLINK          3

/* Address settings:
 *
 * RTM_NEWADDR, RTM_DELADDR, RTM_GETADDR
 *   Add, remove or receive information about an IP address associated with
 *   an interface.  These messages contain an ifaddrmsg structure, optionally
 *   followed by rtattr routing attributes.
 */

#define RTM_NEWADDR          4
#define RTM_DELADDR          5
#define RTM_GETADDR          6

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

#define RTM_NEWROUTE         7
#define RTM_DELROUTE         8
#define RTM_GETROUTE         9

/* Neighbor cache:
 *
 * RTM_NEWNEIGH, RTM_DELNEIGH, RTM_GETNEIGH
 *   Add, remove or receive information about a neighbor table entry (e.g.,
 *   an ARP entry).  The message contains an ndmsg structure.
 */

#define RTM_NEWNEIGH         10
#define RTM_DELNEIGH         11
#define RTM_GETNEIGH         12

/* Routing rules:
 *
 * RTM_NEWRULE, RTM_DELRULE, RTM_GETRULE
 *   Add, delete or retrieve a routing rule.  Carries a struct rtmsg
 */

#define RTM_NEWRULE          13
#define RTM_DELRULE          14
#define RTM_GETRULE          15

/* Queuing discipline settings:
 *
 * RTM_NEWQDISC, RTM_DELQDISC, RTM_GETQDISC
 *   Add, remove or get a queuing discipline.  The message contains a
 *   struct tcmsg and may be followed by a series of attributes.
 */

#define RTM_NEWQDISC         16
#define RTM_DELQDISC         17
#define RTM_GETQDISC         18

/* Traffic classes used with queues:
 *
 * RTM_NEWTCLASS, RTM_DELTCLASS, RTM_GETTCLASS
 *   Add, remove or get a traffic class.  These messages contain a struct
 *   tcmsg as described above.
 */

#define RTM_NEWTCLASS        19
#define RTM_DELTCLASS        20
#define RTM_GETTCLASS        21

/* Traffic filters:
 *
 * RTM_NEWTFILTER, RTM_DELTFILTER, RTM_GETTFILTER
 *   Add, remove or receive information about a traffic filter.  These
 *   messages contain a struct tcmsg as described above.
 */

#define RTM_NEWTFILTER       22
#define RTM_DELTFILTER       23
#define RTM_GETTFILTER       24

/* Others: */

#define RTM_NEWACTION        25
#define RTM_DELACTION        26
#define RTM_GETACTION        27
#define RTM_NEWPREFIX        28
#define RTM_GETPREFIX        29
#define RTM_GETMULTICAST     30
#define RTM_GETANYCAST       31
#define RTM_NEWNEIGHTBL      32
#define RTM_GETNEIGHTBL      33
#define RTM_SETNEIGHTBL      34

/* Definitions associated with struct sockaddr_nl ***************************/
/* Flags values */

#define NLM_F_REQUEST        0x0001  /* It is request message.   */
#define NLM_F_MULTI          0x0002  /* Multipart message, terminated by NLMSG_DONE */
#define NLM_F_ACK            0x0004  /* Reply with ack, with zero or error code */
#define NLM_F_ECHO           0x0008  /* Echo this request     */
#define NLM_F_DUMP_INTR      0x0010  /* Dump was inconsistent due to sequence change */
#define NLM_F_DUMP_FILTERED  0x0020  /* Dump was filtered as requested */

/* Modifiers to GET request */

#define NLM_F_ROOT           0x0100  /* specify tree  root  */
#define NLM_F_MATCH          0x0200  /* return all matching  */
#define NLM_F_ATOMIC         0x0400  /* atomic GET    */
#define NLM_F_DUMP           (NLM_F_ROOT|NLM_F_MATCH)

/* Modifiers to NEW request */

#define NLM_F_REPLACE        0x0100  /* Override existing    */
#define NLM_F_EXCL           0x0200  /* Do not touch, if it exists  */
#define NLM_F_CREATE         0x0400  /* Create, if it does not exist  */
#define NLM_F_APPEND         0x0800  /* Add to end of list    */

/* Modifiers to DELETE request */

#define NLM_F_NONREC         0x0100  /* Do not delete recursively  */

/* Flags for ACK message */

#define NLM_F_CAPPED         0x0100  /* request was capped */
#define NLM_F_ACK_TLVS       0x0200  /* extended ACK TVLs were included */

/* Definitions for struct nlmsghdr ******************************************/

#define NLMSG_MASK        (sizeof(uint32_t) - 1)
#define NLMSG_ALIGN(n)    (((n) + NLMSG_MASK) & ~NLMSG_MASK)
#define NLMSG_HDRLEN      sizeof(struct nlmsghdr)
#define NLMSG_LENGTH(n)   (NLMSG_HDRLEN + (n))
#define NLMSG_DATA(hdr)   ((FAR void*)(((FAR char*)hdr) + NLMSG_HDRLEN)
#define NLMSG_NEXT(hdr,n) \
  ((n) -= NLMSG_ALIGN((hdr)->nlmsg_len), \
   (FAR struct nlmsghdr*)(((FAR cha r*)(hdr)) + NLMSG_ALIGN((hdr)->nlmsg_len)))

/* Definitions for struct rtattr ********************************************/

/* Macros to handle rtattributes */

#define RTA_MASK          (sizeof(uint32_t) - 1)
#define RTA_ALIGN(n)      (((n) + RTA_MASK) & ~RTA_MASK)
#define RTA_OK(rta,n) \
  ((n) >= (int)sizeof(struct rtattr) && \
   (rta)->rta_len >= sizeof(struct rtattr) && \
   (rta)->rta_len <= (n))
#define RTA_NEXT(rta,attrlen) \
  ((attrlen) -= RTA_ALIGN((rta)->rta_len), \
   (struct rtattr*)(((char*)(rta)) + RTA_ALIGN((rta)->rta_len)))
#define RTA_LENGTH(n)     (RTA_ALIGN(sizeof(struct rtattr)) + (n))
#define RTA_SPACE(n)      RTA_ALIGN(RTA_LENGTH(n))
#define RTA_DATA(rta)     ((FAR void *)(((FAR char *)(rta)) + RTA_LENGTH(0)))
#define RTA_PAYLOAD(rta)  ((int)((rta)->rta_len) - RTA_LENGTH(0))

/* Definitions for struct ifaddrmsg  ****************************************/

/* ifa_flags definitions:  ifa_flags is a flag word of IFA_F_SECONDARY for
 * secondary address (old alias interface), IFA_F_PERMANENT for a permanent
 * address set by the user and other undocumented flags.
 */

#define IFA_F_SECONDARY      0x01
#define IFA_F_PERMANENT      0x02

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
  uint32_t nlmsg_len;    /* Length of message including header */
  uint16_t nlmsg_type;   /* Message content */
  uint16_t nlmsg_flags;  /* Additional flags */
  uint32_t nlmsg_seq;    /* Sequence number */
  uint32_t nlmsg_pid;    /* Sending process port ID */
                         /* Data follows */
};

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
  uint16_t rta_len;      /* Length of option */
  uint16_t rta_type;     /* Type of option */
                         /* Data follows */
};

struct ifinfomsg
{
  uint8_t ifi_family;    /* AF_UNSPEC */
  uint16_t ifi_type;     /* Device type */
  int16_t ifi_index;     /* Unique interface index */
  uint32_t ifi_flags;    /* Device flags  */
  uint32_t ifi_change;   /* Change mask, must always be 0xffffffff */
};

/* RTM_NEWADDR, RTM_DELADDR, RTM_GETADDR
 *
 * Add, remove or receive information about an IP address associated with
 * an interface.  These messages contain an ifaddrmsg structure, optionally
 * followed by rtattr routing attributes.
 */

struct ifaddrmsg
{
  uint8_t ifa_family;    /* Address type:  AF_INET or AF_INET6 */
  uint8_t ifa_prefixlen; /* Prefix length of address */
  uint8_t ifa_flags;     /* Address flags.  See IFA_F_* definitions */
  uint8_t ifa_scope;     /* Address scope */
  int16_t ifa_index;     /* Unique interface index */
};

/* RTM_NEWNEIGH, RTM_DELNEIGH, RTM_GETNEIGH
 *   Add, remove or receive information about a neighbor table entry (e.g.,
 *   an ARP entry).  The message contains an ndmsg structure.
 */

struct ndmsg
{
  uint8_t   ndm_family;
  uint8_t   ndm_pad1;
  uint16_t  ndm_pad2;
  int32_t   ndm_ifindex;
  uint16_t  ndm_state;
  uint8_t   ndm_flags;
  uint8_t   ndm_type;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /*  __INCLUDE_NETPACKET_NETLINK_H */
