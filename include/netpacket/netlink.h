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

#define NLMSG_ALIGNTO                    4
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

#define NETLINK_ADD_MEMBERSHIP           1
#define NETLINK_DROP_MEMBERSHIP          2

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

/* Standard attribute types to specify validation policy */

enum
{
  NLA_UNSPEC,
  NLA_U8,
  NLA_U16,
  NLA_U32,
  NLA_U64,
  NLA_STRING,
  NLA_FLAG,
  NLA_MSECS,
  NLA_NESTED,
  NLA_NESTED_COMPAT,
  NLA_NUL_STRING,
  NLA_BINARY,
  NLA_S8,
  NLA_S16,
  NLA_S32,
  NLA_S64,
  NLA_BITFIELD32,
  NLA_TYPE_MAX = NLA_BITFIELD32,
};

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

/****************************************************************************
 * Name: nla_data
 *
 * Description:
 *   Return pointer to the payload of a netlink attribute.
 *
 * Input Parameters:
 *   nla - The netlink attribute
 *
 * Returned Value:
 *   Pointer to attribute payload
 *
 ****************************************************************************/

static inline FAR void *nla_data(FAR const struct nlattr *nla)
{
  return (FAR void *)(((FAR char *)nla) + NLA_HDRLEN);
}

/****************************************************************************
 * Name: nla_len
 *
 * Description:
 *   Return length of the payload of a netlink attribute.
 *
 * Input Parameters:
 *   nla - The netlink attribute
 *
 * Returned Value:
 *   Length of attribute payload
 *
 ****************************************************************************/

static inline int nla_len(FAR const struct nlattr *nla)
{
  return nla->nla_len - NLA_HDRLEN;
}

/****************************************************************************
 * Name: nla_type
 *
 * Description:
 *   Return attribute type (without flags).
 *
 * Input Parameters:
 *   nla - The netlink attribute
 *
 * Returned Value:
 *   Attribute type
 *
 ****************************************************************************/

static inline int nla_type(FAR const struct nlattr *nla)
{
  return nla->nla_type & NLA_TYPE_MASK;
}

/****************************************************************************
 * Name: nla_get_u8
 *
 * Description:
 *   Return the value of a u8 attribute.
 *
 * Input Parameters:
 *   nla - The netlink attribute
 *
 * Returned Value:
 *   The u8 value
 *
 ****************************************************************************/

static inline uint8_t nla_get_u8(FAR const struct nlattr *nla)
{
  return *(FAR uint8_t *)nla_data(nla);
}

/****************************************************************************
 * Name: nla_get_u16
 *
 * Description:
 *   Return the value of a u16 attribute.
 *
 * Input Parameters:
 *   nla - The netlink attribute
 *
 * Returned Value:
 *   The u16 value
 *
 ****************************************************************************/

static inline uint16_t nla_get_u16(FAR const struct nlattr *nla)
{
  return *(FAR uint16_t *)nla_data(nla);
}

/****************************************************************************
 * Name: nla_get_u32
 *
 * Description:
 *   Return the value of a u32 attribute.
 *
 * Input Parameters:
 *   nla - The netlink attribute
 *
 * Returned Value:
 *   The u32 value
 *
 ****************************************************************************/

static inline uint32_t nla_get_u32(FAR const struct nlattr *nla)
{
  return *(FAR uint32_t *)nla_data(nla);
}

/****************************************************************************
 * Name: nla_get_u64
 *
 * Description:
 *   Return the value of a u64 attribute.
 *
 * Input Parameters:
 *   nla - The netlink attribute
 *
 * Returned Value:
 *   The u64 value
 *
 ****************************************************************************/

static inline uint64_t nla_get_u64(FAR const struct nlattr *nla)
{
  return *(FAR uint64_t *)nla_data(nla);
}

/****************************************************************************
 * Name: nla_get_s8
 *
 * Description:
 *   Return the value of a s8 attribute.
 *
 * Input Parameters:
 *   nla - The netlink attribute
 *
 * Returned Value:
 *   The s8 value
 *
 ****************************************************************************/

static inline int8_t nla_get_s8(FAR const struct nlattr *nla)
{
  return *(FAR int8_t *)nla_data(nla);
}

/****************************************************************************
 * Name: nla_get_s16
 *
 * Description:
 *   Return the value of a s16 attribute.
 *
 * Input Parameters:
 *   nla - The netlink attribute
 *
 * Returned Value:
 *   The s16 value
 *
 ****************************************************************************/

static inline int16_t nla_get_s16(FAR const struct nlattr *nla)
{
  return *(FAR int16_t *)nla_data(nla);
}

/****************************************************************************
 * Name: nla_get_s32
 *
 * Description:
 *   Return the value of a s32 attribute.
 *
 * Input Parameters:
 *   nla - The netlink attribute
 *
 * Returned Value:
 *   The s32 value
 *
 ****************************************************************************/

static inline int32_t nla_get_s32(FAR const struct nlattr *nla)
{
  return *(FAR int32_t *)nla_data(nla);
}

/****************************************************************************
 * Name: nla_get_s64
 *
 * Description:
 *   Return the value of a s64 attribute.
 *
 * Input Parameters:
 *   nla - The netlink attribute
 *
 * Returned Value:
 *   The s64 value
 *
 ****************************************************************************/

static inline int64_t nla_get_s64(FAR const struct nlattr *nla)
{
  return *(FAR int64_t *)nla_data(nla);
}

/****************************************************************************
 * Name: nla_get_flag
 *
 * Description:
 *   Return the value of a flag attribute.
 *
 * Input Parameters:
 *   nla - The netlink attribute
 *
 * Returned Value:
 *   1 if flag is present, 0 otherwise
 *
 ****************************************************************************/

static inline int nla_get_flag(FAR const struct nlattr *nla)
{
  return !!nla;
}

/****************************************************************************
 * Name: nla_for_each_attr
 *
 * Description:
 *   Iterate over a stream of attributes.
 *
 * Input Parameters:
 *   pos  - Loop counter, set to current attribute
 *   head - Head of attribute stream
 *   len  - Length of attribute stream
 *   rem  - Initialized to len, holds bytes currently remaining in stream
 *
 ****************************************************************************/

#define nla_for_each_attr(pos, head, len, rem) \
  for (pos = (FAR struct nlattr *)(head), rem = (len); \
       nla_len(pos) >= 0 && rem >= (int)sizeof(struct nlattr) && \
       pos->nla_len >= sizeof(struct nlattr) && \
       pos->nla_len <= rem; \
       rem -= NLA_ALIGN(pos->nla_len), \
       pos = (FAR struct nlattr *)(((FAR char *)pos) + NLA_ALIGN(pos->nla_len)))

/****************************************************************************
 * Name: nla_for_each_nested
 *
 * Description:
 *   Iterate over nested attributes.
 *
 * Input Parameters:
 *   pos - Loop counter, set to current attribute
 *   nla - Attribute containing the nested attributes
 *   rem - Initialized to nla_len(nla), holds bytes currently remaining
 *
 ****************************************************************************/

#define nla_for_each_nested(pos, nla, rem) \
  nla_for_each_attr(pos, nla_data(nla), nla_len(nla), rem)

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

/* The definitions of the individual protocols used to live in this header.
 * They now have one header each, which are included here so that a user
 * that includes only <netpacket/netlink.h> still sees everything, as it
 * did before.  Include the protocol header directly in new code.
 *
 * These come last: each of them includes this header for the common
 * definitions, and the include guard above breaks the cycle.
 */

#include <netpacket/rtnetlink.h>
#include <netpacket/nfnetlink.h>
#include <netpacket/genetlink.h>

#endif /* __INCLUDE_NETPACKET_NETLINK_H */
